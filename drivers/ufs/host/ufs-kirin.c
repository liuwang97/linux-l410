// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon Kirin 990 UFS host controller glue.
 *
 * Binds the vendor firmware node "hisilicon,kirin-ufs" (Huawei L410 laptop):
 * a Synopsys UFSHCI 2.x/3.x controller with a Synopsys M-PHY whose SRAM
 * firmware has to be loaded by the host before every link startup.
 *
 * Ported from the Huawei 4.19 vendor kernel (drivers/scsi/ufs/ufs-kirin.c,
 * ufs-taurus.c, ufs_mphy_firmware.c; Copyright (c) Huawei Technologies Co.,
 * Ltd. 2019). Inline encryption/FBE, RPMB, HPB, the mas_blk I/O scheduler,
 * the DFX/debug counters and the on-FPGA "HISI MPHY TC" paths are dropped.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <ufs/ufshcd.h>
#include <ufs/ufshci.h>
#include <ufs/unipro.h>

#include "ufshcd-pltfrm.h"
#include "ufs-kirin-mphy-fw.h"

/* UFS subsystem control block (second "reg" entry, 0xf81ff000) */
#define UFS_SYS_MEMORY_CTRL		0x000
#define UFS_PSW_POWER_CTRL		0x004
#define UFS_PHY_ISO_EN			0x008
#define UFS_HC_LP_CTRL			0x00c
#define UFS_PHY_CLK_CTRL		0x010
#define UFS_PSW_CLK_CTRL		0x014
#define UFS_CLOCK_GATE_BYPASS		0x018
#define UFS_RESET_CTRL_EN		0x01c
#define UFS_PHY_RESET_STATUS		0x028
#define UFS_SYS_MK2_CTRL		0x050
#define UFS_SYSCTRL			0x05c
#define UFS_DEVICE_RESET_CTRL		0x060
#define UFS_UMECTRL			0x064
#define UFS_CRG_UFS_CFG			0x07c
#define UFS_DEBUG_CTRL			0x0ac
#define UFS_DEBUG_STAT			0x0b0
#define UFS_PHY_FSM_STATE		0x0d8
#define UFS_PHY_SRAM_MEM_CTRL_S		0x0ec
#define UFS_SYS_POWER_GATING		0x0f4
#define UFS_SYS_MEMORY_BP_CTRL		0x0f8

/* UFS_PSW_POWER_CTRL */
#define BIT_UFS_PSW_MTCMOS_EN		BIT(0)
#define BIT_UFS_PSW_ISO_CTRL		BIT(16)
/* UFS_PHY_ISO_EN */
#define BIT_UFS_PHY_ISO_CTRL		BIT(0)
#define BIT_WDP_BYPASS_EC		BIT(16)
/* UFS_HC_LP_CTRL */
#define BIT_SYSCTRL_PWR_READY		BIT(8)
#define BIT_SYSCTRL_LP_ISOL_EN		BIT(16)
/* UFS_PHY_CLK_CTRL */
#define MASK_SYSCTRL_CFG_CLOCK_FREQ	GENMASK(7, 0)
#define MASK_SYSCTRL_REF_CLOCK_SEL	GENMASK(9, 8)
#define BIT_SYSCTRL_REF_CLOCK_EN	BIT(24)
#define UFS_CFG_CLOCK_FREQ_38M4		0x26	/* cfg clock is clkin_sys, 38.4 MHz */
#define UFS_REF_CLOCK_SEL_38M4		2
/* UFS_PSW_CLK_CTRL */
#define BIT_SYSCTRL_PSW_CLK_EN		BIT(4)
/* UFS_CLOCK_GATE_BYPASS / UFS_SYSCTRL */
#define MASK_UFS_CLK_GATE_BYPASS	GENMASK(5, 0)
#define MASK_UFS_SYSCTRL_BYPASS		GENMASK(21, 16)
/* UFS_RESET_CTRL_EN */
#define BIT_SYSCTRL_LP_RESET_N		BIT(0)
/* UFS_SYS_MK2_CTRL */
#define BIT_OVERALL_BYPASS_EC		BIT(1)
/* UFS_DEVICE_RESET_CTRL: bit 0 is the (active-low) RST_n pad, bit 16 its write mask */
#define MASK_UFS_DEVICE_RESET		BIT(16)
#define BIT_UFS_DEVICE_RESET		BIT(0)
/* UFS_UMECTRL */
#define BIT_UFS_IES_EN_MASK		BIT(0)
/* UFS_CRG_UFS_CFG: hiword-masked, active-low resets of the host controller */
#define BIT_IP_RST_UFS			BIT(0)
#define BIT_IP_ARST_UFS			BIT(1)
#define CRG_UFS_CFG_MASK_SHIFT		16
/* UFS_PHY_SRAM_MEM_CTRL_S */
#define BIT_PHY_SRAM_INIT_DONE		BIT(26)
#define BIT_PHY_SRAM_EXT_LD_DONE	BIT(27)
#define BIT_PHY_SRAM_BYPASS		BIT(28)
/* UFS_SYS_MEMORY_CTRL / UFS_SYS_MEMORY_BP_CTRL: SRAM timing, leave low power */
#define UFS_MEM_CTRL_MASK		GENMASK(15, 0)
#define UFS_MEM_CTRL_VAL		0x0850
#define UFS_BP_MEM_CTRL_VAL		0x4858
/* UFS_SYS_POWER_GATING: pcs_pwr_stable_sc, pma_pwr_en_sc on */
#define UFS_SYS_POWER_GATING_VAL	0x4300

/* Vendor-specific UFSHCI registers */
#define UFS_REG_OCPTHRTL		0xc0
#define LP_PGE				BIT(16)
#define LP_AH8_PGE			BIT(17)

/* SCTRL (0xfa89b000): efuse-controlled RX Rhold option */
#define SCTRL_SCDEEPSLEEPED		0x008
#define EFUSE_RHOLD_BIT			BIT(22)

/* Synopsys M-PHY / UniPro vendor-specific attributes */
#define RXSQCONTROL			0x8009
#define RXRHOLDCTRLOPT			0x8013
#define CBCREGADDRLSB			0x8116
#define CBCREGADDRMSB			0x8117
#define CBCREGWRLSB			0x8118
#define CBCREGWRMSB			0x8119
#define CBCREGRDLSB			0x811a
#define CBCREGRDMSB			0x811b
#define CBCREGRDWRSEL			0x811c
#define CBCRCTRL			0x811f
#define CBENBLCPBATTRWR			0x8113
#define CBRATESEL			0x8114
#define CBREFCLKCTRL2			0x8132
#define VS_ADJUSTTRAILINGCLOCKS		0xd086
#define VS_DEBUGCOUNTER0MASK		0xd09a
#define VS_DEBUGCOUNTERCONTROL		0xd09c
#define VS_DEBUGSAVECONFIGTIME		0xd0a0
#define VS_MK2EXTNSUPPORT		0xd0ab
#define VS_MPHYDISABLE			0xd0c1
#define PA_TXHSG1SYNCLENGTH		0x1552
#define PA_TXHSG2SYNCLENGTH		0x1554
#define PA_TXHSG3SYNCLENGTH		0x1556
#define PA_TXSKIP			0x155c
#define PA_ADAPTAFTERLRSTINPA_INIT	0x15d5
#define MPHY_TX_FSM_STATE		0x41
#define TX_FSM_HIBERN8			0x1

#define MPHY_SRAM_BASE			0xc000
#define HIBERN8_POLL_TIMEOUT_MS		1000

/* host->caps, from the vendor "ufs-kirin-*" DT flags */
#define KIRIN_CAP_RATE_B		BIT(0)
#define KIRIN_CAP_BROKEN_FASTAUTO	BIT(1)
#define KIRIN_CAP_ONE_LANE		BIT(2)
#define KIRIN_CAP_BROKEN_CLK_GATE_BYPASS BIT(3)
#define KIRIN_CAP_RX_VCO_VREF		BIT(4)

enum {
	TX_EQUALIZER_0DB = 0,
	TX_EQUALIZER_35DB = 35,
	TX_EQUALIZER_60DB = 60,
};

/*
 * Bring-up switch: with full_init=0 the host controller is only re-enabled
 * (HCE) and the PHY is left as the UEFI firmware configured it.
 */
static bool full_init = true;
module_param(full_init, bool, 0444);
MODULE_PARM_DESC(full_init, "Reset and re-initialise the UFS subsystem and M-PHY on every host enable (default: true)");

struct ufs_kirin_host {
	struct ufs_hba *hba;
	void __iomem *ufs_sys_ctrl;

	struct clk *ref_clk;		/* clk_ufsio_ref (PHY reference) */
	struct clk *subsys_clk;		/* optional UFS subsystem bus clock */
	struct clk *dev_ref_clk;	/* optional device reference clock (PMIC) */

	u32 caps;
	u32 max_hs_gear;
	u32 tx_equalizer;
	bool efuse_rhold;
	bool in_suspend;
	bool wp_armed_once;
};

static inline u32 ufs_sys_ctrl_readl(struct ufs_kirin_host *host, u32 reg)
{
	return readl(host->ufs_sys_ctrl + reg);
}

static inline void ufs_sys_ctrl_writel(struct ufs_kirin_host *host, u32 val,
				       u32 reg)
{
	writel(val, host->ufs_sys_ctrl + reg);
}

static inline void ufs_sys_ctrl_set_bits(struct ufs_kirin_host *host, u32 mask,
					 u32 reg)
{
	ufs_sys_ctrl_writel(host, ufs_sys_ctrl_readl(host, reg) | mask, reg);
}

static inline void ufs_sys_ctrl_clr_bits(struct ufs_kirin_host *host, u32 mask,
					 u32 reg)
{
	ufs_sys_ctrl_writel(host, ufs_sys_ctrl_readl(host, reg) & ~mask, reg);
}

/* hiword-masked write of the host controller reset bits */
static inline void ufs_kirin_crg_cfg(struct ufs_kirin_host *host, u32 bit,
				     bool released)
{
	ufs_sys_ctrl_writel(host, (bit << CRG_UFS_CFG_MASK_SHIFT) |
			    (released ? bit : 0), UFS_CRG_UFS_CFG);
}

/* Synopsys M-PHY control register access through the CR port (UIC) */
static u16 ufs_kirin_mphy_read(struct ufs_hba *hba, u16 addr)
{
	u32 msb = 0, lsb = 0;

	ufshcd_dme_set(hba, UIC_ARG_MIB(CBCREGADDRMSB), addr >> 8);
	ufshcd_dme_set(hba, UIC_ARG_MIB(CBCREGADDRLSB), addr & 0xff);
	ufshcd_dme_set(hba, UIC_ARG_MIB(CBCREGRDWRSEL), 0);
	ufshcd_dme_get(hba, UIC_ARG_MIB(CBCREGRDMSB), &msb);
	ufshcd_dme_get(hba, UIC_ARG_MIB(CBCREGRDLSB), &lsb);

	return ((msb & 0xff) << 8) | (lsb & 0xff);
}

static void ufs_kirin_mphy_write(struct ufs_hba *hba, u16 addr, u16 val)
{
	ufshcd_dme_set(hba, UIC_ARG_MIB(CBCREGADDRMSB), addr >> 8);
	ufshcd_dme_set(hba, UIC_ARG_MIB(CBCREGADDRLSB), addr & 0xff);
	ufshcd_dme_set(hba, UIC_ARG_MIB(CBCREGWRMSB), val >> 8);
	ufshcd_dme_set(hba, UIC_ARG_MIB(CBCREGWRLSB), val & 0xff);
	ufshcd_dme_set(hba, UIC_ARG_MIB(CBCREGRDWRSEL), 1);
}

/*
 * DME_SET by polling, with the UIC completion interrupt masked. Used for the
 * ~30000 UIC commands of the M-PHY SRAM download, where the interrupt driven
 * path would cost a context switch per command. Caller holds uic_cmd_mutex.
 */
static int ufs_kirin_dme_set_poll(struct ufs_hba *hba, u32 attr_sel, u32 val)
{
	u32 reg;
	int ret;

	ret = readl_poll_timeout(hba->mmio_base + REG_CONTROLLER_STATUS, reg,
				 reg & UIC_COMMAND_READY, 0, 100 * USEC_PER_MSEC);
	if (ret)
		return ret;

	ufshcd_writel(hba, UIC_COMMAND_COMPL, REG_INTERRUPT_STATUS);
	ufshcd_writel(hba, attr_sel, REG_UIC_COMMAND_ARG_1);
	ufshcd_writel(hba, 0, REG_UIC_COMMAND_ARG_2);
	ufshcd_writel(hba, val, REG_UIC_COMMAND_ARG_3);
	ufshcd_writel(hba, UIC_CMD_DME_SET & COMMAND_OPCODE_MASK,
		      REG_UIC_COMMAND);

	ret = readl_poll_timeout(hba->mmio_base + REG_INTERRUPT_STATUS, reg,
				 reg & UIC_COMMAND_COMPL, 0,
				 500 * USEC_PER_MSEC);
	if (ret) {
		/*
		 * The interrupt handler acks every pending status bit, so an
		 * unrelated interrupt may have eaten UCCS: accept the command
		 * as done if the controller is ready for the next one.
		 */
		if (!(ufshcd_readl(hba, REG_CONTROLLER_STATUS) & UIC_COMMAND_READY))
			return ret;
	}
	ufshcd_writel(hba, UIC_COMMAND_COMPL, REG_INTERRUPT_STATUS);

	/* config result code; the vendor driver never checked it */
	return ufshcd_readl(hba, REG_UIC_COMMAND_ARG_2) & MASK_UIC_COMMAND_RESULT;
}

static int ufs_kirin_load_mphy_fw(struct ufs_hba *hba)
{
	struct ufs_kirin_host *host = ufshcd_get_variant(hba);
	ktime_t start = ktime_get();
	int i, j, ret, nres = 0, res = 0;
	u32 ie, reg;

	ret = readl_poll_timeout(host->ufs_sys_ctrl + UFS_PHY_SRAM_MEM_CTRL_S,
				 reg, reg & BIT_PHY_SRAM_INIT_DONE,
				 1000, 100 * USEC_PER_MSEC);
	if (ret)
		dev_warn(hba->dev, "M-PHY SRAM init not done (0x%08x)\n", reg);

	mutex_lock(&hba->uic_cmd_mutex);
	ie = ufshcd_readl(hba, REG_INTERRUPT_ENABLE);
	ufshcd_writel(hba, ie & ~UIC_COMMAND_COMPL, REG_INTERRUPT_ENABLE);

	ret = 0;
	for (i = 0; i < ARRAY_SIZE(ufs_kirin_mphy_fw) && ret >= 0; i++) {
		u16 addr = MPHY_SRAM_BASE + i;
		u16 val = ufs_kirin_mphy_fw[i];
		const u32 cmd[][2] = {
			{ CBCREGADDRMSB, addr >> 8 },
			{ CBCREGADDRLSB, addr & 0xff },
			{ CBCREGWRMSB, val >> 8 },
			{ CBCREGWRLSB, val & 0xff },
			{ CBCREGRDWRSEL, 1 },
		};

		for (j = 0; j < ARRAY_SIZE(cmd); j++) {
			ret = ufs_kirin_dme_set_poll(hba, UIC_ARG_MIB(cmd[j][0]),
						     cmd[j][1]);
			if (ret < 0) {
				dev_err(hba->dev, "M-PHY firmware write timed out at word %d\n", i);
				break;
			}
			if (ret && !nres++)
				res = ret;
		}
	}

	ufshcd_writel(hba, ie, REG_INTERRUPT_ENABLE);
	mutex_unlock(&hba->uic_cmd_mutex);

	/* tell the PHY its SRAM is loaded */
	ufs_sys_ctrl_set_bits(host, BIT_PHY_SRAM_EXT_LD_DONE,
			      UFS_PHY_SRAM_MEM_CTRL_S);

	if (nres)
		dev_warn(hba->dev, "M-PHY firmware: %d UIC config errors (first %d)\n",
			 nres, res);
	dev_info(hba->dev, "M-PHY firmware loaded in %lld us\n",
		 ktime_us_delta(ktime_get(), start));
	return ret < 0 ? ret : 0;
}

static int ufs_kirin_check_hibern8(struct ufs_hba *hba)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(HIBERN8_POLL_TIMEOUT_MS);
	u32 fsm0 = 0, fsm1 = 0;
	int err;

	do {
		err = ufshcd_dme_get(hba, UIC_ARG_MIB_SEL(MPHY_TX_FSM_STATE, 0), &fsm0);
		err |= ufshcd_dme_get(hba, UIC_ARG_MIB_SEL(MPHY_TX_FSM_STATE, 1), &fsm1);
		if (err || (fsm0 == TX_FSM_HIBERN8 && fsm1 == TX_FSM_HIBERN8))
			break;
		usleep_range(100, 200);
	} while (time_before(jiffies, timeout));

	if (time_after(jiffies, timeout)) {
		err = ufshcd_dme_get(hba, UIC_ARG_MIB_SEL(MPHY_TX_FSM_STATE, 0), &fsm0);
		err |= ufshcd_dme_get(hba, UIC_ARG_MIB_SEL(MPHY_TX_FSM_STATE, 1), &fsm1);
	}

	if (err) {
		dev_err(hba->dev, "unable to get TX_FSM_STATE: %d\n", err);
		return err;
	}
	if (fsm0 != TX_FSM_HIBERN8 || fsm1 != TX_FSM_HIBERN8) {
		dev_err(hba->dev, "invalid TX_FSM_STATE, lane0 = %u, lane1 = %u\n",
			fsm0, fsm1);
		return -EIO;
	}
	return 0;
}

/* TX analog settings per RAWCMN_DIG_TX_CAL_CODE: {leg_pull_en, post} */
static const u16 tx_eq_0db[][2] = {
	{ 252, 0 }, { 252, 0 }, { 252, 0 }, { 255, 0 }, { 255, 0 }, { 1020, 0 },
	{ 1020, 0 }, { 1023, 0 }, { 1023, 0 }, { 4092, 0 }, { 4092, 0 },
};

static const u16 tx_eq_35db[][2] = {
	{ 252, 3 }, { 252, 3 }, { 252, 3 }, { 255, 3 }, { 255, 6 }, { 1020, 6 },
	{ 1020, 6 }, { 1023, 6 }, { 1023, 6 }, { 4092, 6 }, { 4092, 7 },
};

static const u16 tx_eq_60db[][2] = {
	{ 252, 6 }, { 252, 7 }, { 252, 7 }, { 255, 7 }, { 255, 14 }, { 1020, 14 },
	{ 1020, 14 }, { 1023, 14 }, { 1023, 15 }, { 4092, 15 }, { 4092, 15 },
};

/*
 * Override the TX driver strength/de-emphasis of both lanes (vendor
 * workaround for reduced PWM amplitude and PMC/H8 glitches).
 */
static void ufs_kirin_tx_equalizer(struct ufs_hba *hba)
{
	struct ufs_kirin_host *host = ufshcd_get_variant(hba);
	const u16 (*table)[2] = tx_eq_35db;
	u16 leg, post;
	u32 code;

	if (host->tx_equalizer == TX_EQUALIZER_0DB)
		table = tx_eq_0db;
	else if (host->tx_equalizer == TX_EQUALIZER_60DB)
		table = tx_eq_60db;

	code = ufs_kirin_mphy_read(hba, 0x200c) & 0xf;	/* RAWCMN_DIG_TX_CAL_CODE */
	if (code >= ARRAY_SIZE(tx_eq_35db))
		code = 0;
	leg = table[code][0] << 1;
	post = table[code][1];

	/* LANEN_DIG_ANA_TX_EQ_OVRD_OUT_0..5, lane 0 (0x10xx) and lane 1 (0x11xx) */
	ufs_kirin_mphy_write(hba, 0x10a3, leg);
	ufs_kirin_mphy_write(hba, 0x11a3, leg);
	ufs_kirin_mphy_write(hba, 0x10a6, post);
	ufs_kirin_mphy_write(hba, 0x11a6, post);
	ufs_kirin_mphy_write(hba, 0x10a4, 0);
	ufs_kirin_mphy_write(hba, 0x11a4, 0);
	ufs_kirin_mphy_write(hba, 0x10a5, 0);
	ufs_kirin_mphy_write(hba, 0x11a5, 0);
	ufs_kirin_mphy_write(hba, 0x10a7, 0);
	ufs_kirin_mphy_write(hba, 0x11a7, 0);
	ufs_kirin_mphy_write(hba, 0x10a8, 0);
	ufs_kirin_mphy_write(hba, 0x11a8, 0);

	/* enable the override, then pulse TX_ANA_LOAD_CLK */
	leg |= BIT(15);
	ufs_kirin_mphy_write(hba, 0x10a3, leg);
	ufs_kirin_mphy_write(hba, 0x11a3, leg);
	ufs_kirin_mphy_write(hba, 0x10a3, leg | 1);
	ufs_kirin_mphy_write(hba, 0x11a3, leg | 1);
	ufs_kirin_mphy_write(hba, 0x10a3, leg);
	ufs_kirin_mphy_write(hba, 0x11a3, leg);
}

/* Synopsys M-PHY attributes applied before link startup: {attr, selector, value} */
static const u32 snps_mphy_attrs[][3] = {
	{ CBENBLCPBATTRWR, 0x0, 0x1 },
	{ VS_MPHYCFGUPDT, 0x0, 0x1 },
	{ 0x008c, 0x4, 0xf },		/* RX_HS_G1_PREPARE_LENGTH_CAPABILITY */
	{ 0x008c, 0x5, 0xf },
	{ VS_MPHYCFGUPDT, 0x0, 0x1 },
	{ PA_TXHSADAPTTYPE, 0x0, PA_NO_ADAPT },
	{ PA_ADAPTAFTERLRSTINPA_INIT, 0x0, PA_NO_ADAPT },
	{ 0x0005, 0x0, 0x2 },		/* TX: only LA */
	{ 0x0005, 0x1, 0x2 },
	{ VS_MPHYCFGUPDT, 0x0, 0x1 },
	{ 0x0092, 0x4, 0xa },		/* RX_Hibern8Time_Capability */
	{ 0x0092, 0x5, 0xa },
	{ 0x008f, 0x4, 0xa },		/* RX_Min_ActivateTime */
	{ 0x008f, 0x5, 0xa },
	{ 0x0095, 0x4, 0x4f },		/* Gear3 sync length */
	{ 0x0095, 0x5, 0x4f },
	{ 0x0094, 0x4, 0x4f },		/* Gear2 sync length */
	{ 0x0094, 0x5, 0x4f },
	{ 0x008b, 0x4, 0x4f },		/* Gear1 sync length */
	{ 0x008b, 0x5, 0x4f },
	{ 0x000f, 0x0, 0x5 },		/* TX Thibernate */
	{ 0x000f, 0x1, 0x5 },
	{ VS_MPHYCFGUPDT, 0x0, 0x1 },
	{ CBENBLCPBATTRWR, 0x0, 0x0 },
	{ VS_MPHYCFGUPDT, 0x0, 0x1 },
};

static int ufs_kirin_setup_mphy(struct ufs_hba *hba)
{
	struct ufs_kirin_host *host = ufshcd_get_variant(hba);
	u32 rhold = host->efuse_rhold ? 0x2 : 0x0;
	u32 value = 0;
	int i, err;

	ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MPHYDISABLE, 0x0), 0x1);

	ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RXRHOLDCTRLOPT, 0x4), rhold);
	ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RXRHOLDCTRLOPT, 0x5), rhold);
	ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MPHYCFGUPDT, 0x0), 0x1);

	/* reference clock is running during calibration */
	ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(CBREFCLKCTRL2, 0x0), 0x80);
	ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MPHYCFGUPDT, 0x0), 0x1);

	/* enable the CR port */
	ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(CBCRCTRL, 0x0), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MPHYCFGUPDT, 0x0), 0x1);

	/* RAWCMN_DIG_AON_CMN_SUP_OVRD_IN[5:4]: keep PHY clock on in H8 */
	ufs_kirin_mphy_write(hba, 0x203b, 0x30);
	/* LANEN_ANA_TX/RX_OVRD_MEAS: clear P-N abnormal common voltage */
	ufs_kirin_mphy_write(hba, 0x10e0, 0x10);
	ufs_kirin_mphy_write(hba, 0x11e0, 0x10);
	/* close AFE calibration */
	ufs_kirin_mphy_write(hba, 0x401c, 0x0004);
	ufs_kirin_mphy_write(hba, 0x411c, 0x0004);
	/* "slow process" corner settings */
	ufs_kirin_mphy_write(hba, 0x401e, ufs_kirin_mphy_read(hba, 0x401e) | 0x1);
	ufs_kirin_mphy_write(hba, 0x411e, ufs_kirin_mphy_read(hba, 0x411e) | 0x1);
	ufs_kirin_mphy_write(hba, 0x401f, ufs_kirin_mphy_read(hba, 0x401f) | 0x1);
	ufs_kirin_mphy_write(hba, 0x411f, ufs_kirin_mphy_read(hba, 0x411f) | 0x1);

	err = ufs_kirin_load_mphy_fw(hba);
	if (err)
		return err;

	ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(PA_HSSERIES, 0x0),
		       (host->caps & KIRIN_CAP_RATE_B) ? PA_HS_MODE_B : PA_HS_MODE_A);
	ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(CBRATESEL, 0x0),
		       (host->caps & KIRIN_CAP_RATE_B) ? 0x1 : 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MPHYCFGUPDT, 0x0), 0x1);

	ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RXSQCONTROL, 0x4), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RXSQCONTROL, 0x5), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MPHYCFGUPDT, 0x0), 0x1);

	if (host->caps & KIRIN_CAP_RX_VCO_VREF)
		ufs_kirin_mphy_write(hba, 0x0042, 0x28);	/* rx_vco_vref = 501 mV */

	for (i = 0; i < ARRAY_SIZE(snps_mphy_attrs); i++)
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(snps_mphy_attrs[i][0],
						    snps_mphy_attrs[i][1]),
			       snps_mphy_attrs[i][2]);

	ufshcd_dme_get(hba, UIC_ARG_MIB_SEL(VS_MPHYDISABLE, 0x0), &value);
	if (value != 0x1)
		dev_warn(hba->dev, "VS_mphy_disable is 0x%x\n", value);
	ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MPHYDISABLE, 0x0), 0x0);

	if (ufs_kirin_check_hibern8(hba))
		dev_err(hba->dev, "M-PHY did not reach HIBERN8\n");

	ufs_kirin_tx_equalizer(hba);

	/* drop the ref_clk_en override again */
	ufs_kirin_mphy_write(hba, 0x203b, 0x0);

	return 0;
}

/*
 * Power up, un-isolate and reset the whole UFS subsystem: host controller,
 * M-PHY (SRAM firmware is reloaded at link startup) and the UFS device.
 * Vendor ufs_soc_init() for Kirin 990 ("taurus"), minus the SCTRL clock setup
 * that belongs to the clock driver.
 */
static void ufs_kirin_soc_init(struct ufs_hba *hba)
{
	struct ufs_kirin_host *host = ufshcd_get_variant(hba);
	u32 reg;

	/* PHY runs from SRAM, loaded by us */
	ufs_sys_ctrl_clr_bits(host, BIT_PHY_SRAM_BYPASS, UFS_PHY_SRAM_MEM_CTRL_S);
	ufs_sys_ctrl_clr_bits(host, BIT_PHY_SRAM_EXT_LD_DONE, UFS_PHY_SRAM_MEM_CTRL_S);

	ufs_sys_ctrl_writel(host, BIT_UFS_IES_EN_MASK, UFS_UMECTRL);

	/* hold the host controller in reset */
	ufs_kirin_crg_cfg(host, BIT_IP_RST_UFS, false);

	/* HC power switch on, then tell the controller */
	ufs_sys_ctrl_set_bits(host, BIT_UFS_PSW_MTCMOS_EN, UFS_PSW_POWER_CTRL);
	udelay(10);
	ufs_sys_ctrl_set_bits(host, BIT_SYSCTRL_PWR_READY, UFS_HC_LP_CTRL);

	/* SRAMs out of shutdown */
	reg = ufs_sys_ctrl_readl(host, UFS_SYS_MEMORY_CTRL);
	reg = (reg & ~UFS_MEM_CTRL_MASK) | UFS_MEM_CTRL_VAL;
	ufs_sys_ctrl_writel(host, reg, UFS_SYS_MEMORY_CTRL);
	reg = ufs_sys_ctrl_readl(host, UFS_SYS_MEMORY_BP_CTRL);
	reg = (reg & ~UFS_MEM_CTRL_MASK) | UFS_BP_MEM_CTRL_VAL;
	ufs_sys_ctrl_writel(host, reg, UFS_SYS_MEMORY_BP_CTRL);

	/* PHY reference clock off while selecting 38.4 MHz ref/cfg clocks */
	ufs_sys_ctrl_clr_bits(host, BIT_SYSCTRL_REF_CLOCK_EN, UFS_PHY_CLK_CTRL);
	reg = ufs_sys_ctrl_readl(host, UFS_PHY_CLK_CTRL);
	reg &= ~(MASK_SYSCTRL_REF_CLOCK_SEL | MASK_SYSCTRL_CFG_CLOCK_FREQ);
	reg |= UFS_CFG_CLOCK_FREQ_38M4 |
	       FIELD_PREP(MASK_SYSCTRL_REF_CLOCK_SEL, UFS_REF_CLOCK_SEL_38M4);
	ufs_sys_ctrl_writel(host, reg, UFS_PHY_CLK_CTRL);

	ufs_sys_ctrl_writel(host, UFS_SYS_POWER_GATING_VAL, UFS_SYS_POWER_GATING);

	/* bypass the controller clock gates while initialising */
	ufs_sys_ctrl_set_bits(host, MASK_UFS_CLK_GATE_BYPASS, UFS_CLOCK_GATE_BYPASS);
	ufs_sys_ctrl_set_bits(host, MASK_UFS_SYSCTRL_BYPASS, UFS_SYSCTRL);

	ufs_sys_ctrl_set_bits(host, BIT_SYSCTRL_PSW_CLK_EN, UFS_PSW_CLK_CTRL);
	/* remove HC and PHY isolation */
	ufs_sys_ctrl_clr_bits(host, BIT_UFS_PSW_ISO_CTRL, UFS_PSW_POWER_CTRL);
	ufs_sys_ctrl_clr_bits(host, BIT_UFS_PHY_ISO_CTRL, UFS_PHY_ISO_EN);
	ufs_sys_ctrl_clr_bits(host, BIT_SYSCTRL_LP_ISOL_EN, UFS_HC_LP_CTRL);

	ufs_kirin_crg_cfg(host, BIT_IP_ARST_UFS, true);

	ufs_sys_ctrl_set_bits(host, BIT_SYSCTRL_LP_RESET_N, UFS_RESET_CTRL_EN);
	usleep_range(1000, 1100);

	/* PHY reference clock on, then pulse the device reset */
	ufs_sys_ctrl_set_bits(host, BIT_SYSCTRL_REF_CLOCK_EN, UFS_PHY_CLK_CTRL);
	ufs_sys_ctrl_writel(host, MASK_UFS_DEVICE_RESET, UFS_DEVICE_RESET_CTRL);
	usleep_range(1000, 1100);
	ufs_sys_ctrl_writel(host, MASK_UFS_DEVICE_RESET | BIT_UFS_DEVICE_RESET,
			    UFS_DEVICE_RESET_CTRL);
	usleep_range(10000, 11000);

	/* error correction on */
	ufs_sys_ctrl_clr_bits(host, BIT_WDP_BYPASS_EC, UFS_PHY_ISO_EN);
	ufs_sys_ctrl_set_bits(host, BIT_OVERALL_BYPASS_EC, UFS_SYS_MK2_CTRL);

	/* release the host controller */
	ufs_kirin_crg_cfg(host, BIT_IP_RST_UFS, true);
	if (ufs_sys_ctrl_readl(host, UFS_CRG_UFS_CFG) & BIT_IP_RST_UFS)
		usleep_range(1000, 1100);
}

static int ufs_kirin_hce_enable_notify(struct ufs_hba *hba,
				       enum ufs_notify_change_status status)
{
	if (status == PRE_CHANGE && full_init)
		ufs_kirin_soc_init(hba);

	return 0;
}

static int ufs_kirin_link_startup_pre_change(struct ufs_hba *hba)
{
	u32 value = 0;
	int err;

	if (full_init) {
		err = ufs_kirin_setup_mphy(hba);
		if (err)
			return err;
	}

	/* auto-hibern8 stays off until the link is up */
	ufshcd_rmwl(hba, UFSHCI_AHIBERN8_TIMER_MASK, 0,
		    REG_AUTO_HIBERNATE_IDLE_TIMER);

	ufshcd_disable_host_tx_lcc(hba);
	ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_ADJUSTTRAILINGCLOCKS, 0x0), 0xf0);
	ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_DEBUGSAVECONFIGTIME, 0x0), 0x3);
	ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(PA_ADAPTAFTERLRSTINPA_INIT, 0x0),
		       PA_NO_ADAPT);

	ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MK2EXTNSUPPORT, 0x0), 0x0);
	ufshcd_dme_get(hba, UIC_ARG_MIB_SEL(VS_MK2EXTNSUPPORT, 0x0), &value);
	if (value)
		dev_warn(hba->dev, "failed to close VS_Mk2ExtnSupport\n");

	return 0;
}

static int ufs_kirin_link_startup_post_change(struct ufs_hba *hba)
{
	struct ufs_kirin_host *host = ufshcd_get_variant(hba);

	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_AFC0CREDITTHRESHOLD), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_TC0OUTACKTHRESHOLD), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_TC0TXFCTHRESHOLD), 0x9);

	if (host->caps & KIRIN_CAP_BROKEN_CLK_GATE_BYPASS) {
		/* let the controller gate its own clocks again */
		ufs_sys_ctrl_clr_bits(host, MASK_UFS_CLK_GATE_BYPASS,
				      UFS_CLOCK_GATE_BYPASS);
		ufs_sys_ctrl_clr_bits(host, MASK_UFS_SYSCTRL_BYPASS, UFS_SYSCTRL);
	}

	/* no power gating in (auto-)hibern8 */
	if (ufshcd_is_auto_hibern8_supported(hba))
		ufshcd_rmwl(hba, LP_AH8_PGE | LP_PGE, 0, UFS_REG_OCPTHRTL);

	/* debug counter 0 counts received symbols */
	ufshcd_dme_set(hba, UIC_ARG_MIB(VS_DEBUGCOUNTER0MASK), 0x80000000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(VS_DEBUGCOUNTERCONTROL), 0x5);

	return 0;
}

static int ufs_kirin_link_startup_notify(struct ufs_hba *hba,
					 enum ufs_notify_change_status status)
{
	if (status == PRE_CHANGE)
		return ufs_kirin_link_startup_pre_change(hba);
	return ufs_kirin_link_startup_post_change(hba);
}

static void ufs_kirin_pwr_change_pre_change(struct ufs_hba *hba,
					    struct ufs_pa_layer_attr *params)
{
	struct ufs_kirin_host *host = ufshcd_get_variant(hba);
	u32 equalizer, value = 0;

	/* HS-G4 wants no TX de-emphasis, lower gears 3.5 dB */
	equalizer = params->gear_tx == UFS_HS_G4 ? TX_EQUALIZER_0DB :
						   TX_EQUALIZER_35DB;
	if (full_init && host->tx_equalizer != equalizer) {
		host->tx_equalizer = equalizer;
		ufs_kirin_tx_equalizer(hba);
	}

	ufshcd_dme_set(hba, UIC_ARG_MIB(VS_DEBUGSAVECONFIGTIME), 0x13);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_TXHSG1SYNCLENGTH), 0x4f);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_TXHSG2SYNCLENGTH), 0x4f);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_TXHSG3SYNCLENGTH), 0x4f);

	ufshcd_dme_get(hba, UIC_ARG_MIB(PA_HIBERN8TIME), &value);
	if (value < 0xa)
		ufshcd_dme_set(hba, UIC_ARG_MIB(PA_HIBERN8TIME), 0xa);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_TACTIVATE), 0xa);
	ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MPHYCFGUPDT, 0x0), 0x1);

	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_TXSKIP), 0x0);

	/* TX adapt only for UniPro >= 1.8 peers; the vendor keeps it off for WDC */
	value = 0;
	ufshcd_dme_get(hba, UIC_ARG_MIB(PA_REMOTEVERINFO), &value);
	if ((value & 0xf) >= UFS_UNIPRO_VER_1_8)
		ufshcd_dme_set(hba, UIC_ARG_MIB(PA_TXHSADAPTTYPE), PA_NO_ADAPT);
}

static int ufs_kirin_pwr_change_notify(struct ufs_hba *hba,
				       enum ufs_notify_change_status status,
				       const struct ufs_pa_layer_attr *dev_max_params,
				       struct ufs_pa_layer_attr *dev_req_params)
{
	struct ufs_kirin_host *host = ufshcd_get_variant(hba);
	struct ufs_host_params host_params;
	int ret;

	if (status == POST_CHANGE) {
		dev_info(hba->dev, "power mode: %s-G%u x%u, rate %c, TX equalizer %u.%u dB\n",
			 dev_req_params->pwr_rx == FAST_MODE ? "FAST" :
			 dev_req_params->pwr_rx == FASTAUTO_MODE ? "FASTAUTO" :
			 dev_req_params->pwr_rx == SLOW_MODE ? "SLOW" : "SLOWAUTO",
			 dev_req_params->gear_rx, dev_req_params->lane_rx,
			 dev_req_params->hs_rate == PA_HS_MODE_B ? 'B' : 'A',
			 host->tx_equalizer / 10, host->tx_equalizer % 10);
		return 0;
	}

	ufshcd_init_host_params(&host_params);
	host_params.hs_rx_gear = host->max_hs_gear;
	host_params.hs_tx_gear = host->max_hs_gear;
	host_params.hs_rate = (host->caps & KIRIN_CAP_RATE_B) ? PA_HS_MODE_B :
								PA_HS_MODE_A;
	if (host->caps & KIRIN_CAP_ONE_LANE)
		host_params.rx_lanes = host_params.tx_lanes = UFS_LANE_1;
	if (!(host->caps & KIRIN_CAP_BROKEN_FASTAUTO))
		host_params.rx_pwr_hs = host_params.tx_pwr_hs = FASTAUTO_MODE;
	ufshcd_parse_gear_limits(hba, &host_params);

	ret = ufshcd_negotiate_pwr_params(&host_params, dev_max_params,
					  dev_req_params);
	if (ret) {
		dev_err(hba->dev, "failed to determine power mode: %d\n", ret);
		return ret;
	}

	ufs_kirin_pwr_change_pre_change(hba, dev_req_params);
	return 0;
}

/*
 * The firmware LUs (0-2) are configured with bLUWriteProtect = power-on write
 * protect; the vendor kernel arms it on every device init by setting the
 * volatile fPowerOnWPEn flag, which only a power cycle or device reset
 * clears. Do the same, so the device itself rejects writes to them. This is a
 * flag set, no descriptor or other persistent configuration is touched.
 * apply_dev_quirks runs after fDeviceInit on every (re)initialisation of the
 * device, i.e. also after resets that cleared the flag again.
 */
static bool protect_fw_luns = true;
module_param(protect_fw_luns, bool, 0444);
MODULE_PARM_DESC(protect_fw_luns, "Arm the power-on write protection of the firmware LUs (default: true)");

static int ufs_kirin_apply_dev_quirks(struct ufs_hba *hba)
{
	struct ufs_kirin_host *host = ufshcd_get_variant(hba);
	bool armed = false;
	int err;

	if (!protect_fw_luns)
		return 0;

	err = ufshcd_query_flag(hba, UPIU_QUERY_OPCODE_SET_FLAG,
				QUERY_FLAG_IDN_PWR_ON_WPE, 0, NULL);
	if (!err) {
		/* the device needs ~1 ms before the protection takes effect */
		usleep_range(1000, 1100);
		err = ufshcd_query_flag(hba, UPIU_QUERY_OPCODE_READ_FLAG,
					QUERY_FLAG_IDN_PWR_ON_WPE, 0, &armed);
	}
	if (err || !armed) {
		dev_err(hba->dev, "failed to arm LU power-on write protection: %d\n", err);
		return 0;
	}

	hba->dev_info.f_power_on_wp_en = true;
	if (!host->wp_armed_once) {
		host->wp_armed_once = true;
		dev_info(hba->dev, "LU power-on write protection armed (fPowerOnWPEn)\n");
	}
	return 0;
}

static int ufs_kirin_device_reset(struct ufs_hba *hba)
{
	struct ufs_kirin_host *host = ufshcd_get_variant(hba);

	ufs_sys_ctrl_writel(host, MASK_UFS_DEVICE_RESET, UFS_DEVICE_RESET_CTRL);
	usleep_range(1000, 1100);
	ufs_sys_ctrl_writel(host, MASK_UFS_DEVICE_RESET | BIT_UFS_DEVICE_RESET,
			    UFS_DEVICE_RESET_CTRL);
	usleep_range(10000, 11000);

	return 0;
}

static int ufs_kirin_suspend(struct ufs_hba *hba, enum ufs_pm_op pm_op,
			     enum ufs_notify_change_status status)
{
	struct ufs_kirin_host *host = ufshcd_get_variant(hba);

	if (status == PRE_CHANGE || pm_op == UFS_RUNTIME_PM)
		return 0;
	if (host->in_suspend)
		return 0;

	/* link is in hibern8 (or off): stop the PHY and device reference clocks */
	ufs_sys_ctrl_clr_bits(host, BIT_SYSCTRL_REF_CLOCK_EN, UFS_PHY_CLK_CTRL);
	udelay(10);
	clk_disable_unprepare(host->dev_ref_clk);

	host->in_suspend = true;
	return 0;
}

static int ufs_kirin_resume(struct ufs_hba *hba, enum ufs_pm_op pm_op)
{
	struct ufs_kirin_host *host = ufshcd_get_variant(hba);
	int ret;

	if (!host->in_suspend)
		return 0;

	ret = clk_prepare_enable(host->dev_ref_clk);
	if (ret)
		return ret;
	if (host->dev_ref_clk)
		udelay(250);
	ufs_sys_ctrl_set_bits(host, BIT_SYSCTRL_REF_CLOCK_EN, UFS_PHY_CLK_CTRL);

	host->in_suspend = false;
	return 0;
}

static void ufs_kirin_dbg_register_dump(struct ufs_hba *hba)
{
	struct ufs_kirin_host *host = ufshcd_get_variant(hba);
	static const struct {
		u32 off;
		const char *name;
	} regs[] = {
		{ UFS_SYS_MEMORY_CTRL, "MEMORY_CTRL" },
		{ UFS_PSW_POWER_CTRL, "PSW_POWER_CTRL" },
		{ UFS_PHY_ISO_EN, "PHY_ISO_EN" },
		{ UFS_HC_LP_CTRL, "HC_LP_CTRL" },
		{ UFS_PHY_CLK_CTRL, "PHY_CLK_CTRL" },
		{ UFS_PSW_CLK_CTRL, "PSW_CLK_CTRL" },
		{ UFS_CLOCK_GATE_BYPASS, "CLOCK_GATE_BYPASS" },
		{ UFS_RESET_CTRL_EN, "RESET_CTRL_EN" },
		{ UFS_PHY_RESET_STATUS, "PHY_RESET_STATUS" },
		{ UFS_SYS_MK2_CTRL, "MK2_CTRL" },
		{ UFS_SYSCTRL, "UFS_SYSCTRL" },
		{ UFS_DEVICE_RESET_CTRL, "DEVICE_RESET_CTRL" },
		{ UFS_CRG_UFS_CFG, "CRG_UFS_CFG" },
		{ UFS_DEBUG_STAT, "DEBUG_STAT" },
		{ UFS_PHY_FSM_STATE, "PHY_FSM_STATE" },
		{ UFS_PHY_SRAM_MEM_CTRL_S, "PHY_SRAM_MEM_CTRL_S" },
		{ UFS_SYS_POWER_GATING, "POWER_GATING" },
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(regs); i++)
		dev_err(hba->dev, "sysctrl %-20s 0x%03x: 0x%08x\n", regs[i].name,
			regs[i].off, ufs_sys_ctrl_readl(host, regs[i].off));
	dev_err(hba->dev, "OCPTHRTL 0x%08x\n", ufshcd_readl(hba, UFS_REG_OCPTHRTL));
}

static void ufs_kirin_read_efuse(struct ufs_kirin_host *host)
{
	struct device_node *np;
	void __iomem *sctrl;

	np = of_find_compatible_node(NULL, NULL, "hisilicon,sysctrl");
	if (!np)
		return;
	sctrl = of_iomap(np, 0);
	of_node_put(np);
	if (!sctrl)
		return;
	host->efuse_rhold = !!(readl(sctrl + SCTRL_SCDEEPSLEEPED) & EFUSE_RHOLD_BIT);
	iounmap(sctrl);
}

static void ufs_kirin_parse_dt(struct ufs_kirin_host *host)
{
	struct device_node *np = host->hba->dev->of_node;

	if (of_property_read_bool(np, "ufs-kirin-use-rate-B"))
		host->caps |= KIRIN_CAP_RATE_B;
	if (of_property_read_bool(np, "ufs-kirin-broken-fastauto"))
		host->caps |= KIRIN_CAP_BROKEN_FASTAUTO;
	if (of_property_read_bool(np, "ufs-kirin-use-one-line"))
		host->caps |= KIRIN_CAP_ONE_LANE;
	if (of_property_read_bool(np, "ufs-kirin-broken-clk-gate-bypass"))
		host->caps |= KIRIN_CAP_BROKEN_CLK_GATE_BYPASS;
	if (of_property_read_bool(np, "ufs-kirin-rx-vco-vref"))
		host->caps |= KIRIN_CAP_RX_VCO_VREF;

	if (of_property_read_bool(np, "ufs-kirin-use-HS-GEAR4"))
		host->max_hs_gear = UFS_HS_G4;
	else if (of_property_read_bool(np, "ufs-kirin-use-HS-GEAR3"))
		host->max_hs_gear = UFS_HS_G3;
	else if (of_property_read_bool(np, "ufs-kirin-use-HS-GEAR2"))
		host->max_hs_gear = UFS_HS_G2;
	else
		host->max_hs_gear = UFS_HS_G1;
}

static int ufs_kirin_init(struct ufs_hba *hba)
{
	struct device *dev = hba->dev;
	struct ufs_kirin_host *host;

	host = devm_kzalloc(dev, sizeof(*host), GFP_KERNEL);
	if (!host)
		return -ENOMEM;
	host->hba = hba;

	host->ufs_sys_ctrl = devm_platform_ioremap_resource(to_platform_device(dev), 1);
	if (IS_ERR(host->ufs_sys_ctrl))
		return PTR_ERR(host->ufs_sys_ctrl);

	host->ref_clk = devm_clk_get_optional_enabled(dev, "clk_ufsio_ref");
	if (IS_ERR(host->ref_clk))
		return dev_err_probe(dev, PTR_ERR(host->ref_clk),
				     "failed to get clk_ufsio_ref\n");
	host->subsys_clk = devm_clk_get_optional_enabled(dev, "ufs_subsys");
	if (IS_ERR(host->subsys_clk))
		return dev_err_probe(dev, PTR_ERR(host->subsys_clk),
				     "failed to get ufs_subsys clock\n");
	host->dev_ref_clk = devm_clk_get_optional_enabled(dev, "dev_ref");
	if (IS_ERR(host->dev_ref_clk))
		return dev_err_probe(dev, PTR_ERR(host->dev_ref_clk),
				     "failed to get device reference clock\n");

	ufs_kirin_parse_dt(host);
	ufs_kirin_read_efuse(host);
	/* the link comes up with the vendor default of 3.5 dB */
	host->tx_equalizer = TX_EQUALIZER_35DB;

	/* runtime: device active + link hibern8; system: device sleep + link hibern8 */
	hba->rpm_lvl = UFS_PM_LVL_1;
	hba->spm_lvl = UFS_PM_LVL_3;

	ufshcd_set_variant(hba, host);

	dev_info(dev, "Kirin UFS: caps 0x%x, max HS-G%u, rate %c, rhold %d, %s init\n",
		 host->caps, host->max_hs_gear,
		 (host->caps & KIRIN_CAP_RATE_B) ? 'B' : 'A', host->efuse_rhold,
		 full_init ? "full" : "firmware-state");
	return 0;
}

/* the vendor driver always used 64-bit DMA, whatever CAP.64AS says */
static int ufs_kirin_set_dma_mask(struct ufs_hba *hba)
{
	return dma_set_mask_and_coherent(hba->dev, DMA_BIT_MASK(64));
}

static const struct ufs_hba_variant_ops ufs_hba_kirin_vops = {
	.name = "kirin",
	.init = ufs_kirin_init,
	.set_dma_mask = ufs_kirin_set_dma_mask,
	.hce_enable_notify = ufs_kirin_hce_enable_notify,
	.link_startup_notify = ufs_kirin_link_startup_notify,
	.pwr_change_notify = ufs_kirin_pwr_change_notify,
	.suspend = ufs_kirin_suspend,
	.resume = ufs_kirin_resume,
	.apply_dev_quirks = ufs_kirin_apply_dev_quirks,
	.dbg_register_dump = ufs_kirin_dbg_register_dump,
	.device_reset = ufs_kirin_device_reset,
};

static const struct of_device_id ufs_kirin_of_match[] = {
	{ .compatible = "hisilicon,kirin-ufs" },
	{ },
};
MODULE_DEVICE_TABLE(of, ufs_kirin_of_match);

static int ufs_kirin_probe(struct platform_device *pdev)
{
	return ufshcd_pltfrm_init(pdev, &ufs_hba_kirin_vops);
}

static void ufs_kirin_remove(struct platform_device *pdev)
{
	ufshcd_pltfrm_remove(pdev);
}

static int ufs_kirin_suspend_prepare(struct device *dev)
{
	/* runtime and system PM levels differ, see ufs_kirin_init() */
	return __ufshcd_suspend_prepare(dev, false);
}

static const struct dev_pm_ops ufs_kirin_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ufshcd_system_suspend, ufshcd_system_resume)
	SET_RUNTIME_PM_OPS(ufshcd_runtime_suspend, ufshcd_runtime_resume, NULL)
	.prepare = ufs_kirin_suspend_prepare,
	.complete = ufshcd_resume_complete,
};

static struct platform_driver ufs_kirin_driver = {
	.probe = ufs_kirin_probe,
	.remove = ufs_kirin_remove,
	.driver = {
		.name = "ufshcd-kirin",
		.pm = &ufs_kirin_pm_ops,
		.of_match_table = ufs_kirin_of_match,
	},
};
module_platform_driver(ufs_kirin_driver);

MODULE_DESCRIPTION("HiSilicon Kirin 990 UFS host controller glue");
MODULE_LICENSE("GPL");
