// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Kirin IPC mailbox controller (HiIPCV230), as found on Kirin 990.
 *
 * Every IPC block holds up to 32 mailboxes of 64 bytes each. A mailbox is
 * claimed by writing the sender's bit to its SOURCE register, the receiver is
 * selected through DSET, and writing the sender bit to SEND raises the
 * interrupt on the receiver. The receiver answers by writing the data
 * registers and "acking" (the mailbox goes to ACK state and interrupts the
 * sender); the sender reads the acknowledge payload and releases the mailbox
 * by writing its SOURCE bit again.
 *
 * The firmware device tree describes each mailbox as a child node with the
 * vendor properties src_bit/des_bit/index/func/rproc. Mailboxes where the
 * ACPU is the source are TX channels; their ACK interrupt is the combined
 * per-source interrupt of the IPC block (interrupts 0/1 of the IPC node).
 * Mailboxes where the ACPU is the destination are RX channels with their own
 * interrupt.
 *
 * Based on the vendor hisi_mailbox driver (Huawei, GPL-2.0).
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include <linux/mailbox/kirin-ipc.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define IPC_MBX_SHIFT		6	/* 64 bytes per mailbox */
#define IPC_SRC			0x00
#define IPC_DSET		0x04
#define IPC_DCLR		0x08
#define IPC_DSTAT		0x0c
#define IPC_MODE		0x10
#define IPC_IMASK		0x14
#define IPC_ICLR		0x18
#define IPC_SEND		0x1c
#define IPC_DATA(i)		(0x20 + ((i) << 2))
#define IPC_CPU_IMST(cpu)	(0x800 + ((cpu) << 3))
#define IPC_LOCK		0xa00

#define MODE_AUTO_ACK		BIT(0)
#define MODE_IDLE		BIT(4)
#define MODE_SOURCE		BIT(5)
#define MODE_DEST		BIT(6)
#define MODE_ACK		BIT(7)

#define IPC_MAX_MBOX		32
#define IPC_INDEX_RESIDUE	100	/* DT "index" is <ipc_type> + mailbox */
#define IPC_DEFAULT_TIMEOUT	300	/* ms */

struct kirin_ipc;

struct kirin_ipc_chan {
	struct kirin_ipc *ipc;
	struct mbox_chan *chan;
	const char *node_name;
	const char *rproc;		/* vendor channel name, may be NULL */
	unsigned int idx;		/* hardware mailbox number */
	unsigned int src, dst;		/* processor bits */
	bool tx;			/* ACPU is the source */
	int irq;			/* RX: own interrupt */
	unsigned int timeout;		/* ms */
	u32 buf[KIRIN_IPC_MAX_WORDS];

	/* helper client (kirin_ipc_send / kirin_ipc_register_rx) */
	struct mbox_client cl;
	bool bound;
	struct mutex xfer_lock;		/* one helper transfer at a time */
	spinlock_t ack_lock;
	u32 *ack;
	unsigned int ack_len;
	struct blocking_notifier_head rx_chain;

	/* statistics (debugfs) */
	unsigned long n_sent, n_acked, n_timeout, n_rx;
};

struct kirin_ipc {
	struct device *dev;
	void __iomem *base;
	u32 unlock_key;
	unsigned int cap;		/* data words per mailbox */
	u32 src_id[2];
	int ack_irq[2];
	struct kirin_ipc_chan *kchans;
	struct mbox_chan *chans;
	unsigned int nchans;
	struct mbox_controller mbox;
	spinlock_t lock;
	struct list_head node;
};

static LIST_HEAD(kirin_ipc_list);
static DEFINE_MUTEX(kirin_ipc_list_lock);
static struct workqueue_struct *kirin_ipc_wq;

static inline void __iomem *kc_reg(struct kirin_ipc_chan *kc, unsigned int off)
{
	return kc->ipc->base + (kc->idx << IPC_MBX_SHIFT) + off;
}

static void kirin_ipc_unlock(struct kirin_ipc *ipc)
{
	writel(ipc->unlock_key, ipc->base + IPC_LOCK);
}

/* give the mailbox back (IDLE) */
static void kirin_ipc_release(struct kirin_ipc_chan *kc)
{
	writel(~0U, kc_reg(kc, IPC_IMASK));
	writel(BIT(kc->src), kc_reg(kc, IPC_SRC));
}

static u32 kirin_ipc_src_bits(struct kirin_ipc *ipc)
{
	return BIT(ipc->src_id[0]) | BIT(ipc->src_id[1]);
}

/* read the payload, clear our interrupt bits; ack an incoming message */
static unsigned int kirin_ipc_read_and_clear(struct kirin_ipc_chan *kc,
					     bool rx)
{
	struct kirin_ipc *ipc = kc->ipc;
	unsigned int i;
	u32 imask, todo, mode;

	for (i = 0; i < ipc->cap; i++)
		kc->buf[i] = readl(kc_reg(kc, IPC_DATA(i)));
	if (rx)
		for (i = 0; i < ipc->cap; i++)
			writel(0, kc_reg(kc, IPC_DATA(i)));

	imask = readl(kc_reg(kc, IPC_IMASK));
	todo = kirin_ipc_src_bits(ipc) & ~imask;
	writel(todo, kc_reg(kc, IPC_ICLR));

	mode = readl(kc_reg(kc, IPC_MODE));
	if (rx && (mode & MODE_DEST) && !(mode & MODE_AUTO_ACK))
		writel(todo, kc_reg(kc, IPC_SEND));

	return ipc->cap;
}

static int kirin_ipc_send_data(struct mbox_chan *chan, void *data)
{
	struct kirin_ipc_chan *kc = chan->con_priv;
	struct kirin_ipc *ipc = kc->ipc;
	struct kirin_ipc_msg *msg = data;
	unsigned int i, len;
	unsigned long flags;
	u32 mode;
	int ret = 0;

	if (!kc->tx || !msg || !msg->data)
		return -EINVAL;
	len = min(msg->len, ipc->cap);

	spin_lock_irqsave(&ipc->lock, flags);
	kirin_ipc_unlock(ipc);

	mode = readl(kc_reg(kc, IPC_MODE));
	if (!(mode & MODE_IDLE)) {
		/* a previous transfer timed out, or the remote is still busy */
		if (!(mode & MODE_ACK) &&
		    readl_poll_timeout_atomic(kc_reg(kc, IPC_MODE), mode,
					      mode & (MODE_IDLE | MODE_ACK),
					      2, 2000))
			dev_warn_ratelimited(ipc->dev,
					     "%s: stuck in mode 0x%x, releasing\n",
					     kc->node_name, mode);
		if (!(mode & MODE_IDLE))
			kirin_ipc_release(kc);
	}

	/* claim */
	writel(BIT(kc->src), kc_reg(kc, IPC_SRC));
	if (!(readl(kc_reg(kc, IPC_SRC)) & BIT(kc->src))) {
		dev_err_ratelimited(ipc->dev, "%s: cannot claim mailbox\n",
				    kc->node_name);
		ret = -EBUSY;
		goto out;
	}

	/* ack interrupt to us, message interrupt to the remote */
	writel(~(BIT(kc->src) | BIT(kc->dst)), kc_reg(kc, IPC_IMASK));
	writel(BIT(kc->dst), kc_reg(kc, IPC_DSET));
	writel(0, kc_reg(kc, IPC_MODE));	/* manual acknowledge */
	for (i = 0; i < len; i++)
		writel(msg->data[i], kc_reg(kc, IPC_DATA(i)));
	writel(BIT(kc->src), kc_reg(kc, IPC_SEND));
out:
	spin_unlock_irqrestore(&ipc->lock, flags);
	return ret;
}

static irqreturn_t kirin_ipc_ack_irq(int irq, void *p)
{
	struct kirin_ipc *ipc = p;
	struct kirin_ipc_msg msg;
	irqreturn_t ret = IRQ_NONE;
	unsigned int s, i;

	for (s = 0; s < 2; s++) {
		u32 imst;

		if (ipc->ack_irq[s] != irq ||
		    (s == 1 && ipc->src_id[1] == ipc->src_id[0]))
			continue;
		imst = readl(ipc->base + IPC_CPU_IMST(ipc->src_id[s]));
		if (!imst)
			continue;

		for (i = 0; i < ipc->nchans; i++) {
			struct kirin_ipc_chan *kc = &ipc->kchans[i];
			u32 mode;

			if (!kc->tx || kc->src != ipc->src_id[s] ||
			    !(imst & BIT(kc->idx)))
				continue;

			spin_lock(&ipc->lock);
			mode = readl(kc_reg(kc, IPC_MODE));
			if (!(mode & MODE_ACK)) {
				/* spurious: just clear it */
				writel(BIT(kc->src), kc_reg(kc, IPC_ICLR));
				spin_unlock(&ipc->lock);
				ret = IRQ_HANDLED;
				continue;
			}
			msg.len = kirin_ipc_read_and_clear(kc, false);
			msg.data = kc->buf;
			kirin_ipc_release(kc);
			spin_unlock(&ipc->lock);

			kc->n_acked++;
			mbox_chan_received_data(kc->chan, &msg);
			mbox_chan_txdone(kc->chan, 0);
			ret = IRQ_HANDLED;
		}
	}
	return ret;
}

static irqreturn_t kirin_ipc_rx_irq(int irq, void *p)
{
	struct kirin_ipc_chan *kc = p;
	struct kirin_ipc_msg msg;
	u32 mode;

	mode = readl(kc_reg(kc, IPC_MODE));
	if (!(mode & MODE_DEST))
		return IRQ_NONE;

	msg.len = kirin_ipc_read_and_clear(kc, true);
	msg.data = kc->buf;
	kc->n_rx++;
	mbox_chan_received_data(kc->chan, &msg);
	return IRQ_HANDLED;
}

static const struct mbox_chan_ops kirin_ipc_ops = {
	.send_data = kirin_ipc_send_data,
};

/* #mbox-cells = <1>: the hardware mailbox number */
static struct mbox_chan *kirin_ipc_of_xlate(struct mbox_controller *mbox,
					    const struct of_phandle_args *sp)
{
	struct kirin_ipc *ipc = container_of(mbox, struct kirin_ipc, mbox);
	unsigned int i;

	if (sp->args_count != 1)
		return ERR_PTR(-EINVAL);
	for (i = 0; i < ipc->nchans; i++)
		if (ipc->kchans[i].idx == sp->args[0])
			return &ipc->chans[i];
	return ERR_PTR(-ENOENT);
}

/* ---- helper client ---------------------------------------------------- */

static void kirin_ipc_cl_rx(struct mbox_client *cl, void *data)
{
	struct kirin_ipc_chan *kc = container_of(cl, struct kirin_ipc_chan, cl);
	struct kirin_ipc_msg *msg = data;
	unsigned long flags;

	if (kc->tx) {
		spin_lock_irqsave(&kc->ack_lock, flags);
		if (kc->ack)
			memcpy(kc->ack, msg->data,
			       min(kc->ack_len, msg->len) * sizeof(u32));
		spin_unlock_irqrestore(&kc->ack_lock, flags);
		return;
	}
	blocking_notifier_call_chain(&kc->rx_chain, msg->len,
				     (void *)msg->data);
}

static int kirin_ipc_bind_helper(struct kirin_ipc_chan *kc)
{
	int ret;

	if (kc->bound)
		return 0;
	kc->cl.dev = kc->ipc->dev;
	kc->cl.tx_block = true;
	kc->cl.tx_tout = kc->timeout;
	kc->cl.knows_txdone = false;
	kc->cl.rx_callback = kirin_ipc_cl_rx;
	ret = mbox_bind_client(kc->chan, &kc->cl);
	if (!ret)
		kc->bound = true;
	return ret;
}

static struct kirin_ipc_chan *kirin_ipc_find(const char *name)
{
	struct kirin_ipc *ipc;
	unsigned int i;

	if (!name)
		return ERR_PTR(-EINVAL);

	guard(mutex)(&kirin_ipc_list_lock);
	if (list_empty(&kirin_ipc_list))
		return ERR_PTR(-EPROBE_DEFER);
	list_for_each_entry(ipc, &kirin_ipc_list, node)
		for (i = 0; i < ipc->nchans; i++)
			if (ipc->kchans[i].rproc &&
			    !strcmp(ipc->kchans[i].rproc, name))
				return &ipc->kchans[i];
	return ERR_PTR(-ENODEV);
}

int kirin_ipc_send(const char *mbox, const u32 *msg, unsigned int len,
		   u32 *ack, unsigned int ack_len)
{
	struct kirin_ipc_chan *kc = kirin_ipc_find(mbox);
	struct kirin_ipc_msg m = { .data = msg, .len = len };
	unsigned long flags;
	int ret;

	if (IS_ERR(kc))
		return PTR_ERR(kc);
	if (!kc->tx || !len || len > KIRIN_IPC_MAX_WORDS)
		return -EINVAL;

	mutex_lock(&kc->xfer_lock);
	ret = kirin_ipc_bind_helper(kc);
	if (ret)
		goto out;

	spin_lock_irqsave(&kc->ack_lock, flags);
	kc->ack = ack;
	kc->ack_len = ack ? ack_len : 0;
	spin_unlock_irqrestore(&kc->ack_lock, flags);

	kc->n_sent++;
	ret = mbox_send_message(kc->chan, &m);
	if (ret == -ETIME)
		kc->n_timeout++;
	if (ret == -ETIME)
		dev_err_ratelimited(kc->ipc->dev,
				    "%s (%s): no ack, msg 0x%08x 0x%08x\n",
				    kc->node_name, mbox, msg[0],
				    len > 1 ? msg[1] : 0);

	spin_lock_irqsave(&kc->ack_lock, flags);
	kc->ack = NULL;
	spin_unlock_irqrestore(&kc->ack_lock, flags);
out:
	mutex_unlock(&kc->xfer_lock);
	return ret < 0 ? ret : 0;
}
EXPORT_SYMBOL_GPL(kirin_ipc_send);

struct kirin_ipc_async {
	struct work_struct work;
	const char *mbox;
	unsigned int len;
	u32 data[KIRIN_IPC_MAX_WORDS];
};

static void kirin_ipc_async_fn(struct work_struct *work)
{
	struct kirin_ipc_async *a = container_of(work, struct kirin_ipc_async,
						 work);

	kirin_ipc_send(a->mbox, a->data, a->len, NULL, 0);
	kfree(a);
}

int kirin_ipc_send_async(const char *mbox, const u32 *msg, unsigned int len)
{
	struct kirin_ipc_async *a;

	/* @mbox must stay valid (a string literal); it is looked up later */
	if (!mbox || !len || len > KIRIN_IPC_MAX_WORDS || !kirin_ipc_wq)
		return -EINVAL;

	a = kzalloc(sizeof(*a), GFP_ATOMIC);
	if (!a)
		return -ENOMEM;
	INIT_WORK(&a->work, kirin_ipc_async_fn);
	a->mbox = mbox;
	a->len = len;
	memcpy(a->data, msg, len * sizeof(u32));
	queue_work(kirin_ipc_wq, &a->work);
	return 0;
}
EXPORT_SYMBOL_GPL(kirin_ipc_send_async);

int kirin_ipc_register_rx(const char *mbox, struct notifier_block *nb)
{
	struct kirin_ipc_chan *kc = kirin_ipc_find(mbox);

	if (IS_ERR(kc))
		return PTR_ERR(kc);
	if (kc->tx)
		return -EINVAL;
	return blocking_notifier_chain_register(&kc->rx_chain, nb);
}
EXPORT_SYMBOL_GPL(kirin_ipc_register_rx);

int kirin_ipc_unregister_rx(const char *mbox, struct notifier_block *nb)
{
	struct kirin_ipc_chan *kc = kirin_ipc_find(mbox);

	if (IS_ERR(kc))
		return PTR_ERR(kc);
	return blocking_notifier_chain_unregister(&kc->rx_chain, nb);
}
EXPORT_SYMBOL_GPL(kirin_ipc_unregister_rx);

/* ---- debugfs: channel list and a test transfer ------------------------ */

static struct dentry *kirin_ipc_debugfs;
static DEFINE_MUTEX(kirin_ipc_dbg_lock);
static char kirin_ipc_dbg_result[160];

static int kirin_ipc_channels_show(struct seq_file *s, void *unused)
{
	struct kirin_ipc *ipc;
	unsigned int i;

	seq_puts(s, "ipc      node         mbx dir src dst  sent acked tmout    rx  rproc\n");
	guard(mutex)(&kirin_ipc_list_lock);
	list_for_each_entry(ipc, &kirin_ipc_list, node)
		for (i = 0; i < ipc->nchans; i++) {
			struct kirin_ipc_chan *kc = &ipc->kchans[i];

			seq_printf(s, "%-8s %-12s %3u %s %3u %3u %5lu %5lu %5lu %5lu  %s\n",
				   dev_name(ipc->dev), kc->node_name, kc->idx,
				   kc->tx ? " tx" : " rx", kc->src, kc->dst,
				   kc->n_sent, kc->n_acked, kc->n_timeout,
				   kc->n_rx, kc->rproc ?: "-");
		}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(kirin_ipc_channels);

/* write "<rproc> <word> [word...]", read back "ret ack0 ack1 ..." */
static ssize_t kirin_ipc_xfer_write(struct file *f, const char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	u32 msg[KIRIN_IPC_MAX_WORDS], ack[KIRIN_IPC_MAX_WORDS] = { };
	char buf[160], *p = buf, *tok, *name;
	unsigned int n = 0, i;
	int ret, len;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = 0;

	name = strsep(&p, " \t\n");
	if (!name || !*name)
		return -EINVAL;
	while ((tok = strsep(&p, " \t\n"))) {
		if (!*tok)
			continue;
		if (n == KIRIN_IPC_MAX_WORDS || kstrtou32(tok, 0, &msg[n]))
			return -EINVAL;
		n++;
	}
	if (!n)
		return -EINVAL;

	ret = kirin_ipc_send(name, msg, n, ack, KIRIN_IPC_MAX_WORDS);
	mutex_lock(&kirin_ipc_dbg_lock);
	len = scnprintf(kirin_ipc_dbg_result, sizeof(kirin_ipc_dbg_result),
			"%d", ret);
	for (i = 0; i < KIRIN_IPC_MAX_WORDS; i++)
		len += scnprintf(kirin_ipc_dbg_result + len,
				 sizeof(kirin_ipc_dbg_result) - len,
				 " 0x%08x", ack[i]);
	mutex_unlock(&kirin_ipc_dbg_lock);
	return count;
}

static ssize_t kirin_ipc_xfer_read(struct file *f, char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	char buf[170];
	int len;

	mutex_lock(&kirin_ipc_dbg_lock);
	len = scnprintf(buf, sizeof(buf), "%s\n", kirin_ipc_dbg_result);
	mutex_unlock(&kirin_ipc_dbg_lock);
	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
}

static const struct file_operations kirin_ipc_xfer_fops = {
	.write = kirin_ipc_xfer_write,
	.read = kirin_ipc_xfer_read,
	.llseek = default_llseek,
};

/* ---- probe ------------------------------------------------------------- */

static int kirin_ipc_parse_chan(struct kirin_ipc *ipc, struct device_node *np,
				struct kirin_ipc_chan *kc)
{
	u32 func[3], idx;

	if (of_property_read_u32(np, "src_bit", &kc->src) ||
	    of_property_read_u32(np, "des_bit", &kc->dst) ||
	    of_property_read_u32(np, "index", &idx) ||
	    of_property_read_u32_array(np, "func", func, 3))
		return -EINVAL;

	kc->idx = idx % IPC_INDEX_RESIDUE;
	if (kc->idx >= IPC_MAX_MBOX || kc->src > 31 || kc->dst > 31)
		return -EINVAL;
	kc->tx = func[1] != 0;	/* <fast is_src is_dst> */
	kc->node_name = np->name;
	if (of_property_read_string(np, "rproc", &kc->rproc))
		kc->rproc = NULL;
	if (of_property_read_u32(np, "timeout", &kc->timeout) || !kc->timeout)
		kc->timeout = IPC_DEFAULT_TIMEOUT;
	kc->irq = kc->tx ? 0 : of_irq_get(np, 0);
	return 0;
}

static int kirin_ipc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node, *child;
	struct kirin_ipc *ipc;
	struct clk *clk;
	unsigned int n = 0, i;
	int ret;

	ipc = devm_kzalloc(dev, sizeof(*ipc), GFP_KERNEL);
	if (!ipc)
		return -ENOMEM;
	ipc->dev = dev;
	spin_lock_init(&ipc->lock);

	ipc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ipc->base))
		return PTR_ERR(ipc->base);

	clk = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "no clock\n");

	if (of_property_read_u32(np, "unlock_key", &ipc->unlock_key) ||
	    of_property_read_u32(np, "capability", &ipc->cap) ||
	    of_property_read_u32_array(np, "rproc_src_id", ipc->src_id, 2))
		return dev_err_probe(dev, -EINVAL, "missing IPC properties\n");
	ipc->cap = clamp_t(unsigned int, ipc->cap, 1, KIRIN_IPC_MAX_WORDS);
	if (ipc->src_id[0] > 31 || ipc->src_id[1] > 31)
		return -EINVAL;

	for_each_available_child_of_node(np, child) {
		u32 used = 0;

		of_property_read_u32(child, "used", &used);
		if (used)
			n++;
	}
	if (!n)
		return dev_err_probe(dev, -ENODEV, "no mailboxes\n");

	ipc->kchans = devm_kcalloc(dev, n, sizeof(*ipc->kchans), GFP_KERNEL);
	ipc->chans = devm_kcalloc(dev, n, sizeof(*ipc->chans), GFP_KERNEL);
	if (!ipc->kchans || !ipc->chans)
		return -ENOMEM;

	for_each_available_child_of_node(np, child) {
		struct kirin_ipc_chan *kc = &ipc->kchans[ipc->nchans];
		u32 used = 0;

		of_property_read_u32(child, "used", &used);
		if (!used)
			continue;
		if (kirin_ipc_parse_chan(ipc, child, kc)) {
			dev_warn(dev, "%pOFn: bad mailbox description, skipped\n",
				 child);
			continue;
		}
		kc->ipc = ipc;
		kc->chan = &ipc->chans[ipc->nchans];
		kc->chan->con_priv = kc;
		mutex_init(&kc->xfer_lock);
		spin_lock_init(&kc->ack_lock);
		BLOCKING_INIT_NOTIFIER_HEAD(&kc->rx_chain);
		ipc->nchans++;
	}

	/*
	 * No register access here: some IPC blocks (e.g. the NPU one at
	 * 0xe5e01000) sit in power domains that are off at boot, and touching
	 * them raises an SError. The block is unlocked before each send.
	 */
	ipc->mbox.dev = dev;
	ipc->mbox.ops = &kirin_ipc_ops;
	ipc->mbox.chans = ipc->chans;
	ipc->mbox.num_chans = ipc->nchans;
	ipc->mbox.txdone_irq = true;
	ipc->mbox.of_xlate = kirin_ipc_of_xlate;
	ret = devm_mbox_controller_register(dev, &ipc->mbox);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register controller\n");

	/* combined ACK interrupts, one per ACPU source id */
	for (i = 0; i < 2; i++) {
		ipc->ack_irq[i] = platform_get_irq_optional(pdev, i);
		if (ipc->ack_irq[i] <= 0 ||
		    (i == 1 && ipc->ack_irq[1] == ipc->ack_irq[0]))
			continue;
		ret = devm_request_irq(dev, ipc->ack_irq[i], kirin_ipc_ack_irq,
				       IRQF_NO_SUSPEND, dev_name(dev), ipc);
		if (ret)
			return dev_err_probe(dev, ret, "ack irq %d\n",
					     ipc->ack_irq[i]);
	}

	/*
	 * RX mailboxes are always serviced (and acknowledged), even without a
	 * listener, so that the remote processors never stall on us.
	 */
	for (i = 0; i < ipc->nchans; i++) {
		struct kirin_ipc_chan *kc = &ipc->kchans[i];

		if (kc->tx)
			continue;
		ret = kirin_ipc_bind_helper(kc);
		if (ret)
			return ret;
		if (kc->irq <= 0) {
			dev_warn(dev, "%s: no interrupt\n", kc->node_name);
			continue;
		}
		ret = devm_request_threaded_irq(dev, kc->irq, NULL,
						kirin_ipc_rx_irq,
						IRQF_ONESHOT | IRQF_NO_SUSPEND,
						kc->node_name, kc);
		if (ret)
			return dev_err_probe(dev, ret, "%s: irq %d\n",
					     kc->node_name, kc->irq);
	}

	mutex_lock(&kirin_ipc_list_lock);
	list_add_tail(&ipc->node, &kirin_ipc_list);
	mutex_unlock(&kirin_ipc_list_lock);

	platform_set_drvdata(pdev, ipc);
	dev_info(dev, "%u mailboxes, %u words\n", ipc->nchans, ipc->cap);
	return 0;
}

static const struct of_device_id kirin_ipc_of_match[] = {
	{ .compatible = "hisilicon,HiIPCV230" },
	{ }
};
MODULE_DEVICE_TABLE(of, kirin_ipc_of_match);

static struct platform_driver kirin_ipc_driver = {
	.probe = kirin_ipc_probe,
	.driver = {
		.name = "kirin-ipc-mailbox",
		.of_match_table = kirin_ipc_of_match,
		.suppress_bind_attrs = true,
	},
};

static int __init kirin_ipc_init(void)
{
	kirin_ipc_wq = alloc_ordered_workqueue("kirin-ipc", WQ_HIGHPRI);
	if (!kirin_ipc_wq)
		return -ENOMEM;
	kirin_ipc_debugfs = debugfs_create_dir("kirin-ipc", NULL);
	debugfs_create_file("channels", 0444, kirin_ipc_debugfs, NULL,
			    &kirin_ipc_channels_fops);
	debugfs_create_file("xfer", 0600, kirin_ipc_debugfs, NULL,
			    &kirin_ipc_xfer_fops);
	return platform_driver_register(&kirin_ipc_driver);
}
core_initcall(kirin_ipc_init);

MODULE_DESCRIPTION("HiSilicon Kirin IPC mailbox driver");
MODULE_LICENSE("GPL");
