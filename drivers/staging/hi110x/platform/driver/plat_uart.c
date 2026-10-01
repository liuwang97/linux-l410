// SPDX-License-Identifier: GPL-2.0
/*
 * BFGX (BT/GNSS/FM subsystem) UART transport.
 *
 * The vendor driver opened /dev/ttyAMA4 from the kernel and attached its own line
 * discipline. Here the UART is a serdev controller: the DT fixup adds a child node
 * ("hisilicon,hi110x-bfgx-uart") under the BUART port, and this file keeps the
 * vendor entry points (open_tty_drv/release_tty_drv/ps_change_uart_baud_rate) on
 * top of the serdev API. ps_core_s::tty is the serdev device while the port is open.
 */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/serdev.h>

#include "plat_uart.h"
#include "chr_user.h"
#include "oal_ext_if.h"
#include "plat_exception_rst.h"
#include "board.h"
#include "plat_debug.h"
#include "bfgx_dev.h"
#include "bfgx_data_parse.h"

uint32_t g_uart_default_baud_rate = DEFAULT_BAUD_RATE;

/* the BUART serdev device, set when the serdev driver binds */
static struct serdev_device *g_bfgx_serdev;
static DEFINE_MUTEX(g_bfgx_serdev_lock);

/* no lock while getting the state, just statistic */
void ps_tty_tx_cnt_add(struct ps_core_s *ps_core_d, uint32_t cnt)
{
    oal_atomic_add(&(ps_core_d->tty_tx_cnt), cnt);
}

STATIC void ps_tty_rx_cnt_add(struct ps_core_s *ps_core_d, uint32_t cnt)
{
    oal_atomic_add(&(ps_core_d->tty_rx_cnt), cnt);
}

void ps_uart_state_dump(struct ps_core_s *ps_core_d)
{
    if (ps_core_d == NULL) {
        return;
    }
    ps_print_info("===uart state=== open:%d tx:%x rx:%x tiocm:0x%x\n", ps_core_d->tty != NULL,
                  oal_atomic_read(&(ps_core_d->tty_tx_cnt)), oal_atomic_read(&(ps_core_d->tty_rx_cnt)),
                  ps_core_d->tty ? serdev_device_get_tiocm(ps_core_d->tty) : 0);
}

/* modem lines of the open BUART (TIOCM_CTS: the chip accepts data), -1 while closed */
int ps_uart_get_tiocm(struct ps_core_s *ps_core_d)
{
    struct serdev_device *serdev = ps_core_d->tty;

    return serdev ? serdev_device_get_tiocm(serdev) : -1;
}

static size_t ps_serdev_receive(struct serdev_device *serdev, const u8 *data, size_t count)
{
    struct ps_core_s *ps_core_d = serdev_device_get_drvdata(serdev);
#ifdef PLATFORM_DEBUG_ENABLE
    struct st_exception_info *pst_exception_data = NULL;
#endif

    if (unlikely((ps_core_d == NULL) || (ps_core_d->tty == NULL) || (st_tty_recv == NULL))) {
        return count; /* port not in use by the driver: drop */
    }

#ifdef PLATFORM_DEBUG_ENABLE
    // 心跳超时DFR打桩验证
    pst_exception_data = get_exception_info_reference();
    if ((pst_exception_data != NULL) && (pst_exception_data->debug_beat_flag == 0)) {
        return count;
    }
#endif

    oal_spin_lock(&ps_core_d->rx_lock);
    ps_tty_rx_cnt_add(ps_core_d, count);
    st_tty_recv(ps_core_d, data, count);
    oal_spin_unlock(&ps_core_d->rx_lock);

    return count;
}

static void ps_serdev_write_wakeup(struct serdev_device *serdev)
{
    struct ps_core_s *ps_core_d = serdev_device_get_drvdata(serdev);

    serdev_device_write_wakeup(serdev);
    if ((ps_core_d != NULL) && (ps_core_d->tty != NULL)) {
        queue_work(ps_core_d->ps_tx_workqueue, &ps_core_d->tx_skb_work);
    }
}

static const struct serdev_device_ops g_ps_serdev_ops = {
    .receive_buf = ps_serdev_receive,
    .write_wakeup = ps_serdev_write_wakeup,
};

STATIC void ps_clean_tx_skb_buf(struct ps_core_s *ps_core_d)
{
#define WAIT_TX_WORK_DELAY 20
    uint8_t delay_times = RELEASE_DELAT_TIMES;

    ps_kfree_skb(ps_core_d, TX_HIGH_QUEUE);
    ps_kfree_skb(ps_core_d, TX_LOW_QUEUE);

    ps_print_info("free tx sbk buf done!\n");

    /* clean all tx sk_buff */
    while (((ps_core_d->tx_high_seq.qlen) || (ps_core_d->tx_low_seq.qlen)) && (delay_times)) {
        msleep(10); // sleep 10ms
        delay_times--;
    }

    if (oal_work_is_busy(&ps_core_d->tx_skb_work)) {
        ps_print_info("hisi bfgx notify tx work exit\n");
        atomic_set(&ps_core_d->force_tx_exit, 1);
        // wait for tx work exit
        msleep(WAIT_TX_WORK_DELAY);
    }

    if (ps_core_d->tty != NULL) {
        /* drop what is still queued, or closing the port may block */
        serdev_device_write_flush(ps_core_d->tty);
    }
}

/*
 * The HiSilicon UARTs sit behind a PERI CRG reset bit that the vendor pl011 driver
 * pulsed on every port startup ("reset-enable-flag", "reset-reg-base",
 * "reset-controller-reg" = <assert deassert status bit> in the UART node). Upstream
 * pl011 knows nothing about it and the BUART may be left in reset by the firmware, so
 * do the same pulse before opening the port (it is closed at this point).
 */
static void ps_serdev_uart_reset(struct serdev_device *serdev)
{
    struct device_node *np = of_get_parent(serdev->dev.of_node);
    u32 flag = 0, base[4], regs[4];
    void __iomem *crg = NULL;
    u32 before, after;
    int i;

    if (np == NULL) {
        return;
    }
    if (of_property_read_u32(np, "reset-enable-flag", &flag) || !flag ||
        of_property_read_u32_array(np, "reset-reg-base", base, 4) ||
        of_property_read_u32_array(np, "reset-controller-reg", regs, 4) || regs[3] > 31) {
        of_node_put(np);
        return;
    }
    of_node_put(np);

    crg = ioremap(((u64)base[0] << 32) | base[1], base[3]);
    if (crg == NULL) {
        return;
    }
    before = readl(crg + regs[2]);
    writel(BIT(regs[3]), crg + regs[0]); /* assert */
    for (i = 0; i < 100 && !(readl(crg + regs[2]) & BIT(regs[3])); i++) {
        udelay(1);
    }
    writel(BIT(regs[3]), crg + regs[1]); /* deassert */
    for (i = 0; i < 100 && (readl(crg + regs[2]) & BIT(regs[3])); i++) {
        udelay(1);
    }
    after = readl(crg + regs[2]);
    iounmap(crg);
    ps_print_info("BUART reset pulse: bit %u was %s, now %s\n", regs[3],
                  (before & BIT(regs[3])) ? "asserted" : "released",
                  (after & BIT(regs[3])) ? "asserted" : "released");
}

/* bring-up knob: -1 = what the protocol code asks for, 0/1 = force RTS/CTS off/on */
static int buart_flowctl = -1;
module_param(buart_flowctl, int, 0644);
MODULE_PARM_DESC(buart_flowctl, "BUART hardware flow control: -1 auto, 0 off, 1 on");

static void ps_serdev_set_termios(struct serdev_device *serdev, long baud_rate, uint8_t enable_flowctl)
{
    unsigned int real;

    if (buart_flowctl >= 0) {
        enable_flowctl = buart_flowctl ? FLOW_CTRL_ENABLE : FLOW_CTRL_DISABLE;
    }

    serdev_device_set_flow_control(serdev, enable_flowctl == FLOW_CTRL_ENABLE);
    (void)serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
    real = serdev_device_set_baudrate(serdev, (unsigned int)baud_rate);
    ps_print_info("set baud_rate=%u, except=%d, flowctl=%d\n", real, (int)baud_rate, enable_flowctl);
}

/*
 * Prototype    : ps_change_uart_baud_rate
 * Description  : change the BUART baud rate (and flow control)
 */
int32_t ps_change_uart_baud_rate(struct ps_core_s *ps_core_d, long baud_rate, uint8_t enable_flowctl)
{
    struct ps_plat_s *ps_plat_d = (struct ps_plat_s *)(ps_core_d->ps_plat);

    ps_print_info("%s: %lu\n", __func__, baud_rate);

    /* for debug only */
    dump_uart_rx_buf(ps_core_d);

    if (wait_bfgx_memdump_complete() != EXCEPTION_SUCCESS) {
        ps_print_err("wait memdump complete failed\n");
    }

    ps_plat_d->flow_cntrl = enable_flowctl;
    ps_plat_d->baud_rate = baud_rate;

    mutex_lock(&ps_core_d->tty_mutex);
    if (ps_core_d->tty == NULL) {
        mutex_unlock(&ps_core_d->tty_mutex);
        ps_print_err("tty is closed\n");
        return -ENODEV;
    }
    serdev_device_write_flush(ps_core_d->tty);
    ps_serdev_set_termios(ps_core_d->tty, ps_plat_d->baud_rate, ps_plat_d->flow_cntrl);
    mutex_unlock(&ps_core_d->tty_mutex);

    return 0;
}

/*
 * Prototype    : open_tty_drv
 * Description  : open the BUART for the BFGX subsystem
 */
int32_t open_tty_drv(struct ps_core_s *ps_core_d)
{
    int ret;
    struct serdev_device *serdev = NULL;
    struct ps_plat_s *ps_plat_d = (struct ps_plat_s *)(ps_core_d->ps_plat);

    ps_print_info("%s\n", __func__);
    mutex_lock(&ps_core_d->tty_mutex);

    if (ps_core_d->tty_have_open == true) {
        ps_print_warning("hisi bfgx uart already open\n");
        mutex_unlock(&ps_core_d->tty_mutex);
        return 0;
    }

    mutex_lock(&g_bfgx_serdev_lock);
    serdev = g_bfgx_serdev;
    mutex_unlock(&g_bfgx_serdev_lock);
    if ((serdev == NULL) || (ps_core_d != ps_get_core_reference(BUART))) {
        ps_print_err("no BUART serdev device (DT node hisilicon,hi110x-bfgx-uart)\n");
        mutex_unlock(&ps_core_d->tty_mutex);
        return -ENODEV;
    }

    reset_uart_rx_buf(ps_core_d);

    if (oal_atomic_read(&g_ir_only_mode) != 0) {
        /* ir only mode use baudrate 921600 */
        ps_plat_d->baud_rate = IR_ONLY_BAUD_RATE;
    }

    serdev_device_set_drvdata(serdev, ps_core_d);
    ps_serdev_uart_reset(serdev);
    ret = serdev_device_open(serdev);
    if (ret) {
        ps_print_err("failed to open BUART: %d\n", ret);
        mutex_unlock(&ps_core_d->tty_mutex);
        return ret;
    }

    ps_serdev_set_termios(serdev, ps_plat_d->baud_rate, ps_plat_d->flow_cntrl);
    ps_print_info("BUART open, modem lines 0x%x\n", serdev_device_get_tiocm(serdev));

    ps_core_d->tty = serdev;
    ps_core_d->tty_have_open = true;

    mutex_unlock(&ps_core_d->tty_mutex);
    ps_print_info("open tty success\n");

    return 0;
}

/*
 * Prototype    : release_tty_drv
 * Description  : close the BUART
 */
int32_t release_tty_drv(struct ps_core_s *ps_core_d)
{
    struct ps_plat_s *ps_plat_d = (struct ps_plat_s *)(ps_core_d->ps_plat);
    struct serdev_device *serdev = NULL;

    ps_print_info("%s\n", __func__);

    // 置位tty发送中断标志位
    atomic_set(&ps_core_d->force_tx_exit, 1);

    mutex_lock(&ps_core_d->tty_mutex);
    if (ps_core_d->tty_have_open == false) {
        atomic_set(&ps_core_d->force_tx_exit, 0);
        ps_print_info("hisi bfgx uart already closed, ignored\n");
        mutex_unlock(&ps_core_d->tty_mutex);
        return 0;
    }

    ps_clean_tx_skb_buf(ps_core_d);

    serdev = ps_core_d->tty;
    ps_core_d->tty = NULL;
    /* bytes stuck behind an inactive CTS would hold the close for closing_wait (30 s) */
    serdev_device_set_flow_control(serdev, false);
    serdev_device_close(serdev);

    atomic_set(&ps_core_d->force_tx_exit, 0);
    ps_core_d->tty_have_open = false;

    ps_plat_d->flow_cntrl = FLOW_CTRL_ENABLE;
    ps_plat_d->baud_rate = g_uart_default_baud_rate;

    ps_print_info("close tty success\n");
    mutex_unlock(&ps_core_d->tty_mutex);
    return 0;
}

static int ps_serdev_probe(struct serdev_device *serdev)
{
    mutex_lock(&g_bfgx_serdev_lock);
    if (g_bfgx_serdev != NULL) {
        mutex_unlock(&g_bfgx_serdev_lock);
        return -EBUSY;
    }
    serdev_device_set_client_ops(serdev, &g_ps_serdev_ops);
    g_bfgx_serdev = serdev;
    mutex_unlock(&g_bfgx_serdev_lock);
    dev_info(&serdev->dev, "hi110x BUART\n");
    return 0;
}

static void ps_serdev_remove(struct serdev_device *serdev)
{
    struct ps_core_s *ps_core_d = ps_get_core_reference(BUART);

    if (ps_core_d != NULL) {
        release_tty_drv(ps_core_d);
    }
    mutex_lock(&g_bfgx_serdev_lock);
    g_bfgx_serdev = NULL;
    mutex_unlock(&g_bfgx_serdev_lock);
}

static const struct of_device_id g_ps_serdev_of_match[] = {
    { .compatible = "hisilicon,hi110x-bfgx-uart" },
    { },
};
MODULE_DEVICE_TABLE(of, g_ps_serdev_of_match);

static struct serdev_device_driver g_ps_serdev_driver = {
    .probe = ps_serdev_probe,
    .remove = ps_serdev_remove,
    .driver = {
        .name = "hi110x-bfgx-uart",
        .of_match_table = g_ps_serdev_of_match,
    },
};

int32_t plat_uart_init(int index)
{
    if (index == BUART) {
        return serdev_device_driver_register(&g_ps_serdev_driver);
    }
    /* GUART (GNSS "ME" UART) is not used on this board */
    return OAL_SUCC;
}

int32_t plat_uart_exit(int index)
{
    if (index == BUART) {
        serdev_device_driver_unregister(&g_ps_serdev_driver);
    }
    return OAL_SUCC;
}
