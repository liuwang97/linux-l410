// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Kirin peripheral DMA controller with 64-bit addressing
 * ("hisilicon,hisi-dma64-1.0", Kirin 990 at 0xfa000000).
 *
 * A k3dma relative: physical channels at base + 0x40 * n with 64-bit
 * source/destination/link registers, one request line per virtual channel.
 * Channel 0 (below "dma-min-chan") belongs to the secure world.
 *
 * Ported from the Huawei vendor driver (hisi_dma_64.c, GPL-2.0).
 */

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/dmaengine.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "virt-dma.h"

#define INT_STAT		0x00
#define INT_TC1			0x04
#define INT_ERR1		0x0c
#define INT_ERR2		0x10
#define INT_TC1_MASK		0x18
#define INT_ERR1_MASK		0x20
#define INT_ERR2_MASK		0x24
#define INT_TC1_RAW		0x600
#define INT_ERR1_RAW		0x610
#define INT_ERR2_RAW		0x618
#define CH_PRI			0x688
#define CH_STAT			0x690
#define DMA_CTRL		0x698
#define DMA_CTRL_PIPE_LINE	BIT(8)
#define CX_CUR_CNT(n)		(0x404 + (n) * 0x20)

#define CHAN_OFFSET		0x40
#define CX_LLI_L		0x800
#define CX_LLI_H		0x804
#define CX_CNT			0x81c
#define CX_SRC_L		0x820
#define CX_SRC_H		0x824
#define CX_DST_L		0x828
#define CX_DST_H		0x82c
#define CX_CONFIG		0x830

#define LLI_CHAIN_EN		BIT(1)
#define CCFG_EN			BIT(0)
#define CCFG_MEM2PER		(1 << 2)
#define CCFG_PERI_SHIFT		4
#define CCFG_DW_SHIFT		12
#define CCFG_SW_SHIFT		16
#define CCFG_DL_SHIFT		20
#define CCFG_SL_SHIFT		24
#define CCFG_DSTINCR		BIT(30)
#define CCFG_SRCINCR		BIT(31)

#define DMA_MAX_SIZE		0x1ffc
#define DMA_ALIGN		3
#define LLI_BLOCK_SIZE		SZ_4K
#define DMA_PAUSE_TIMEOUT_US	2000

/* ATF service: tell the secure world which channels the kernel owns */
#define DMAC_REGISTER_FN_ID	0xc501de00

struct hisi_desc_hw {
	u64 lli;
	u32 reserved1[5];
	u32 count;
	u64 saddr;
	u64 daddr;
	u32 config;
	u32 reserved2[3];
} __aligned(64);

#define LLI_PER_BLOCK		(LLI_BLOCK_SIZE / sizeof(struct hisi_desc_hw))

struct hisi_dma_desc_sw {
	struct virt_dma_desc vd;
	dma_addr_t desc_hw_lli;
	size_t desc_num;
	size_t size;
	struct hisi_desc_hw *desc_hw;
};

struct hisi_dma_phy;

struct hisi_dma_chan {
	u32 ccfg;
	struct virt_dma_chan vc;
	struct hisi_dma_phy *phy;
	struct list_head node;
	enum dma_transfer_direction dir;
	dma_addr_t dev_addr;
	enum dma_status status;
};

struct hisi_dma_phy {
	u32 idx;
	void __iomem *base;
	struct hisi_dma_chan *vchan;
	struct hisi_dma_desc_sw *ds_run;
	struct hisi_dma_desc_sw *ds_done;
};

struct hisi_dma_dev {
	struct dma_device slave;
	void __iomem *base;
	struct tasklet_struct task;
	spinlock_t lock;
	struct list_head chan_pending;
	struct hisi_dma_phy *phy;
	struct hisi_dma_chan *chans;
	struct clk *clk;
	struct dma_pool *pool;
	u32 dma_channels;
	u32 dma_requests;
	u32 dma_min_chan;
	u32 dma_used_chans;
};

#define to_hisi_dma(dmadev) container_of(dmadev, struct hisi_dma_dev, slave)

static struct hisi_dma_chan *to_hisi_chan(struct dma_chan *chan)
{
	return container_of(chan, struct hisi_dma_chan, vc.chan);
}

static void hisi_dma_pause_dma(struct hisi_dma_phy *phy,
			       struct hisi_dma_dev *d, bool on)
{
	u32 val = readl(phy->base + CX_CONFIG);
	int timeout;

	if (on) {
		writel(val | CCFG_EN, phy->base + CX_CONFIG);
		return;
	}
	val &= ~CCFG_EN;
	writel(val, phy->base + CX_CONFIG);
	for (timeout = DMA_PAUSE_TIMEOUT_US; timeout > 0; timeout--) {
		if (!(readl(d->base + CH_STAT) & BIT(phy->idx)))
			return;
		writel(val, phy->base + CX_CONFIG);
		udelay(1);
	}
	dev_err(d->slave.dev, "channel %u: pause timeout\n", phy->idx);
}

static void hisi_dma_terminate_chan(struct hisi_dma_phy *phy,
				    struct hisi_dma_dev *d)
{
	hisi_dma_pause_dma(phy, d, false);
	writel(BIT(phy->idx), d->base + INT_TC1_RAW);
	writel(BIT(phy->idx), d->base + INT_ERR1_RAW);
	writel(BIT(phy->idx), d->base + INT_ERR2_RAW);
}

static void hisi_dma_set_desc(struct hisi_dma_phy *phy,
			      struct hisi_desc_hw *hw)
{
	writel(upper_32_bits(hw->lli), phy->base + CX_LLI_H);
	writel(lower_32_bits(hw->lli), phy->base + CX_LLI_L);
	writel(hw->count, phy->base + CX_CNT);
	writel(upper_32_bits(hw->saddr), phy->base + CX_SRC_H);
	writel(lower_32_bits(hw->saddr), phy->base + CX_SRC_L);
	writel(upper_32_bits(hw->daddr), phy->base + CX_DST_H);
	writel(lower_32_bits(hw->daddr), phy->base + CX_DST_L);
	writel(hw->config, phy->base + CX_CONFIG);
}

static void hisi_dma_enable_dma(struct hisi_dma_dev *d, bool on)
{
	u32 mask = on ? d->dma_used_chans : 0;

	if (on)
		writel(0, d->base + CH_PRI);
	writel(mask, d->base + INT_TC1_MASK);
	writel(mask, d->base + INT_ERR1_MASK);
	writel(mask, d->base + INT_ERR2_MASK);
}

static irqreturn_t hisi_dma_int_handler(int irq, void *dev_id)
{
	struct hisi_dma_dev *d = dev_id;
	u32 stat = readl(d->base + INT_STAT);
	u32 tc1 = readl(d->base + INT_TC1);
	u32 err1 = readl(d->base + INT_ERR1);
	u32 err2 = readl(d->base + INT_ERR2);
	u32 tc1_irq = 0, err1_irq = 0, err2_irq = 0;
	unsigned long flags;
	u32 i;

	stat &= d->dma_used_chans;
	while (stat) {
		struct hisi_dma_phy *p;
		struct hisi_dma_chan *c;

		i = __ffs(stat);
		stat &= stat - 1;
		if (i >= d->dma_channels)
			continue;
		p = &d->phy[i];
		c = p->vchan;

		if (tc1 & BIT(i)) {
			if (c) {
				spin_lock_irqsave(&c->vc.lock, flags);
				if (p->ds_run)
					vchan_cookie_complete(&p->ds_run->vd);
				p->ds_done = p->ds_run;
				spin_unlock_irqrestore(&c->vc.lock, flags);
			}
			tc1_irq |= BIT(i);
		}
		if ((err1 | err2) & BIT(i)) {
			if (c)
				c->status = DMA_ERROR;
			err1_irq |= err1 & BIT(i);
			err2_irq |= err2 & BIT(i);
			dev_warn(d->slave.dev, "channel %u error 0x%x/0x%x\n",
				 i, err1, err2);
		}
	}

	writel(tc1_irq, d->base + INT_TC1_RAW);
	writel(err1_irq, d->base + INT_ERR1_RAW);
	writel(err2_irq, d->base + INT_ERR2_RAW);

	if (tc1_irq || err1_irq || err2_irq) {
		tasklet_schedule(&d->task);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

static int hisi_dma_start_txd(struct hisi_dma_chan *c)
{
	struct hisi_dma_dev *d = to_hisi_dma(c->vc.chan.device);
	struct virt_dma_desc *vd = vchan_next_desc(&c->vc);
	struct hisi_dma_desc_sw *ds;

	if (!c->phy)
		return -ENODEV;
	if (readl(d->base + CH_STAT) & BIT(c->phy->idx))
		return -EBUSY;

	if (!vd) {
		c->phy->ds_done = NULL;
		c->phy->ds_run = NULL;
		return -EAGAIN;
	}

	ds = container_of(vd, struct hisi_dma_desc_sw, vd);
	list_del(&ds->vd.node);
	c->phy->ds_run = ds;
	c->phy->ds_done = NULL;
	hisi_dma_set_desc(c->phy, &ds->desc_hw[0]);
	return 0;
}

static void hisi_dma_tasklet(struct tasklet_struct *t)
{
	struct hisi_dma_dev *d = from_tasklet(d, t, task);
	struct hisi_dma_chan *c, *cn;
	struct hisi_dma_phy *p;
	unsigned int pch, pch_alloc = 0;
	unsigned long flags;

	/* next descriptor of running channels */
	list_for_each_entry_safe(c, cn, &d->slave.channels, vc.chan.device_node) {
		spin_lock_irqsave(&c->vc.lock, flags);
		p = c->phy;
		if (p && p->ds_done) {
			if (hisi_dma_start_txd(c) == -EAGAIN) {
				c->phy = NULL;
				p->vchan = NULL;
			}
		} else if (p && c->status == DMA_ERROR) {
			hisi_dma_terminate_chan(p, d);
			c->phy = NULL;
			p->vchan = NULL;
			p->ds_run = p->ds_done = NULL;
		}
		spin_unlock_irqrestore(&c->vc.lock, flags);
	}

	/* give free physical channels to pending virtual channels */
	spin_lock_irqsave(&d->lock, flags);
	for (pch = d->dma_min_chan; pch < d->dma_channels; pch++) {
		p = &d->phy[pch];
		if (p->vchan || list_empty(&d->chan_pending) ||
		    !(d->dma_used_chans & BIT(pch)))
			continue;
		c = list_first_entry(&d->chan_pending, struct hisi_dma_chan,
				     node);
		list_del_init(&c->node);
		pch_alloc |= BIT(pch);
		p->vchan = c;
		c->phy = p;
	}
	spin_unlock_irqrestore(&d->lock, flags);

	for (pch = d->dma_min_chan; pch < d->dma_channels; pch++) {
		if (!(pch_alloc & BIT(pch)))
			continue;
		c = d->phy[pch].vchan;
		if (c) {
			spin_lock_irqsave(&c->vc.lock, flags);
			hisi_dma_start_txd(c);
			spin_unlock_irqrestore(&c->vc.lock, flags);
		}
	}
}

static void hisi_dma_free_chan_resources(struct dma_chan *chan)
{
	struct hisi_dma_chan *c = to_hisi_chan(chan);
	struct hisi_dma_dev *d = to_hisi_dma(chan->device);
	unsigned long flags;

	spin_lock_irqsave(&d->lock, flags);
	list_del_init(&c->node);
	spin_unlock_irqrestore(&d->lock, flags);

	vchan_free_chan_resources(&c->vc);
	c->ccfg = 0;
}

static enum dma_status hisi_dma_tx_status(struct dma_chan *chan,
					  dma_cookie_t cookie,
					  struct dma_tx_state *state)
{
	struct hisi_dma_chan *c = to_hisi_chan(chan);
	struct hisi_dma_dev *d = to_hisi_dma(chan->device);
	struct virt_dma_desc *vd;
	struct hisi_dma_phy *p;
	enum dma_status ret;
	unsigned long flags;
	size_t bytes = 0;

	ret = dma_cookie_status(&c->vc.chan, cookie, state);
	if (ret == DMA_COMPLETE)
		return ret;

	spin_lock_irqsave(&c->vc.lock, flags);
	p = c->phy;
	ret = c->status;
	vd = vchan_find_desc(&c->vc, cookie);
	if (vd) {
		bytes = container_of(vd, struct hisi_dma_desc_sw, vd)->size;
	} else if (p && p->ds_run) {
		struct hisi_dma_desc_sw *ds = p->ds_run;
		u64 clli = ((u64)readl(p->base + CX_LLI_H) << 32) |
			   readl(p->base + CX_LLI_L);
		size_t index;

		bytes = readl(d->base + CX_CUR_CNT(p->idx)) & 0xffff;
		clli &= ~(u64)(sizeof(struct hisi_desc_hw) - 1);
		index = clli ? (clli - ds->desc_hw_lli) /
			       sizeof(struct hisi_desc_hw) : ds->desc_num;
		for (; index < ds->desc_num; index++) {
			bytes += ds->desc_hw[index].count;
			if (!ds->desc_hw[index].lli)
				break;
		}
	}
	spin_unlock_irqrestore(&c->vc.lock, flags);
	dma_set_residue(state, bytes);
	return ret;
}

static void hisi_dma_issue_pending(struct dma_chan *chan)
{
	struct hisi_dma_chan *c = to_hisi_chan(chan);
	struct hisi_dma_dev *d = to_hisi_dma(chan->device);
	unsigned long flags;

	spin_lock_irqsave(&c->vc.lock, flags);
	if (vchan_issue_pending(&c->vc)) {
		spin_lock(&d->lock);
		if (!c->phy && list_empty(&c->node)) {
			list_add_tail(&c->node, &d->chan_pending);
			tasklet_schedule(&d->task);
		}
		spin_unlock(&d->lock);
	}
	spin_unlock_irqrestore(&c->vc.lock, flags);
}

static struct hisi_dma_desc_sw *hisi_dma_alloc_desc(struct hisi_dma_dev *d,
						    size_t num)
{
	struct hisi_dma_desc_sw *ds;

	if (num > LLI_PER_BLOCK) {
		dev_err(d->slave.dev, "transfer needs %zu > %zu descriptors\n",
			num, LLI_PER_BLOCK);
		return NULL;
	}
	ds = kzalloc(sizeof(*ds), GFP_NOWAIT);
	if (!ds)
		return NULL;
	ds->desc_hw = dma_pool_zalloc(d->pool, GFP_NOWAIT, &ds->desc_hw_lli);
	if (!ds->desc_hw) {
		kfree(ds);
		return NULL;
	}
	ds->desc_num = num;
	return ds;
}

static void hisi_dma_fill_desc(struct hisi_dma_desc_sw *ds, dma_addr_t dst,
			       dma_addr_t src, size_t len, u32 num, u32 ccfg)
{
	if (num + 1 < ds->desc_num)
		ds->desc_hw[num].lli = ds->desc_hw_lli +
			(num + 1) * sizeof(struct hisi_desc_hw);
	ds->desc_hw[num].lli |= LLI_CHAIN_EN;
	ds->desc_hw[num].count = len;
	ds->desc_hw[num].saddr = src;
	ds->desc_hw[num].daddr = dst;
	ds->desc_hw[num].config = ccfg;
}

static struct dma_async_tx_descriptor *
hisi_dma_prep_memcpy(struct dma_chan *chan, dma_addr_t dst, dma_addr_t src,
		     size_t len, unsigned long flags)
{
	struct hisi_dma_chan *c = to_hisi_chan(chan);
	struct hisi_dma_dev *d = to_hisi_dma(chan->device);
	struct hisi_dma_desc_sw *ds;
	u32 num = 0;
	size_t copy;

	if (!len)
		return NULL;
	ds = hisi_dma_alloc_desc(d, DIV_ROUND_UP(len, DMA_MAX_SIZE));
	if (!ds)
		return NULL;
	ds->size = len;

	if (!c->ccfg) {
		/* mem to mem: incrementing, burst 16, 64-bit */
		c->ccfg = CCFG_SRCINCR | CCFG_DSTINCR | CCFG_EN |
			  (0xf << CCFG_DL_SHIFT) | (0xf << CCFG_SL_SHIFT) |
			  (0x3 << CCFG_DW_SHIFT) | (0x3 << CCFG_SW_SHIFT);
	}

	do {
		copy = min_t(size_t, len, DMA_MAX_SIZE);
		hisi_dma_fill_desc(ds, dst, src, copy, num++, c->ccfg);
		src += copy;
		dst += copy;
		len -= copy;
	} while (len);

	ds->desc_hw[num - 1].lli = 0;
	return vchan_tx_prep(&c->vc, &ds->vd, flags);
}

static struct dma_async_tx_descriptor *
hisi_dma_prep_slave_sg(struct dma_chan *chan, struct scatterlist *sgl,
		       unsigned int sglen, enum dma_transfer_direction dir,
		       unsigned long flags, void *context)
{
	struct hisi_dma_chan *c = to_hisi_chan(chan);
	struct hisi_dma_dev *d = to_hisi_dma(chan->device);
	struct hisi_dma_desc_sw *ds;
	struct scatterlist *sg;
	size_t num = 0, total = 0;
	u32 i, n = 0;

	if (!sgl || (dir != DMA_MEM_TO_DEV && dir != DMA_DEV_TO_MEM))
		return NULL;

	for_each_sg(sgl, sg, sglen, i)
		num += DIV_ROUND_UP(sg_dma_len(sg), DMA_MAX_SIZE);
	ds = hisi_dma_alloc_desc(d, num);
	if (!ds)
		return NULL;

	for_each_sg(sgl, sg, sglen, i) {
		dma_addr_t addr = sg_dma_address(sg);
		size_t avail = sg_dma_len(sg);

		total += avail;
		while (avail) {
			size_t len = min_t(size_t, avail, DMA_MAX_SIZE);

			if (dir == DMA_MEM_TO_DEV)
				hisi_dma_fill_desc(ds, c->dev_addr, addr, len,
						   n++, c->ccfg);
			else
				hisi_dma_fill_desc(ds, addr, c->dev_addr, len,
						   n++, c->ccfg);
			addr += len;
			avail -= len;
		}
	}

	ds->desc_hw[n - 1].lli = 0;
	ds->size = total;
	return vchan_tx_prep(&c->vc, &ds->vd, flags);
}

static u32 hisi_dma_width(enum dma_slave_buswidth width)
{
	switch (width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		return 0;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		return 1;
	case DMA_SLAVE_BUSWIDTH_8_BYTES:
		return 3;
	default:
		return 2;
	}
}

static int hisi_dma_config(struct dma_chan *chan,
			   struct dma_slave_config *cfg)
{
	struct hisi_dma_chan *c = to_hisi_chan(chan);
	enum dma_slave_buswidth width;
	u32 maxburst, val;

	c->dir = cfg->direction;
	if (c->dir == DMA_DEV_TO_MEM) {
		c->ccfg = CCFG_DSTINCR;
		c->dev_addr = cfg->src_addr;
		maxburst = cfg->src_maxburst;
		width = cfg->src_addr_width;
	} else if (c->dir == DMA_MEM_TO_DEV) {
		c->ccfg = CCFG_SRCINCR;
		c->dev_addr = cfg->dst_addr;
		maxburst = cfg->dst_maxburst;
		width = cfg->dst_addr_width;
	} else {
		return -EINVAL;
	}

	val = hisi_dma_width(width);
	c->ccfg |= (val << CCFG_DW_SHIFT) | (val << CCFG_SW_SHIFT);
	val = (!maxburst || maxburst > 16) ? 15 : maxburst - 1;
	c->ccfg |= (val << CCFG_DL_SHIFT) | (val << CCFG_SL_SHIFT);
	c->ccfg |= CCFG_MEM2PER | CCFG_EN;
	/* request line = virtual channel number */
	c->ccfg |= c->vc.chan.chan_id << CCFG_PERI_SHIFT;
	return 0;
}

static int hisi_dma_pause(struct dma_chan *chan)
{
	struct hisi_dma_chan *c = to_hisi_chan(chan);
	struct hisi_dma_dev *d = to_hisi_dma(chan->device);
	unsigned long flags;
	struct hisi_dma_phy *p;

	if (c->status != DMA_IN_PROGRESS)
		return 0;
	c->status = DMA_PAUSED;
	spin_lock_irqsave(&d->lock, flags);
	p = c->phy;
	if (!p)
		list_del_init(&c->node);
	spin_unlock_irqrestore(&d->lock, flags);
	if (p)
		hisi_dma_pause_dma(p, d, false);
	return 0;
}

static int hisi_dma_resume(struct dma_chan *chan)
{
	struct hisi_dma_chan *c = to_hisi_chan(chan);
	struct hisi_dma_dev *d = to_hisi_dma(chan->device);
	unsigned long flags;

	spin_lock_irqsave(&c->vc.lock, flags);
	if (c->status == DMA_PAUSED) {
		c->status = DMA_IN_PROGRESS;
		if (c->phy) {
			hisi_dma_pause_dma(c->phy, d, true);
		} else if (!list_empty(&c->vc.desc_issued)) {
			spin_lock(&d->lock);
			list_add_tail(&c->node, &d->chan_pending);
			spin_unlock(&d->lock);
		}
	}
	spin_unlock_irqrestore(&c->vc.lock, flags);
	return 0;
}

static int hisi_dma_terminate_all(struct dma_chan *chan)
{
	struct hisi_dma_chan *c = to_hisi_chan(chan);
	struct hisi_dma_dev *d = to_hisi_dma(chan->device);
	struct hisi_dma_phy *p;
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&d->lock, flags);
	list_del_init(&c->node);
	spin_unlock_irqrestore(&d->lock, flags);

	spin_lock_irqsave(&c->vc.lock, flags);
	p = c->phy;
	vchan_get_all_descriptors(&c->vc, &head);
	if (p) {
		hisi_dma_terminate_chan(p, d);
		if (p->ds_run && !p->ds_done)
			vchan_terminate_vdesc(&p->ds_run->vd);
		c->phy = NULL;
		p->vchan = NULL;
		p->ds_run = p->ds_done = NULL;
	}
	c->status = DMA_IN_PROGRESS;
	spin_unlock_irqrestore(&c->vc.lock, flags);
	vchan_dma_desc_free_list(&c->vc, &head);
	return 0;
}

static void hisi_dma_synchronize(struct dma_chan *chan)
{
	vchan_synchronize(&to_hisi_chan(chan)->vc);
}

static void hisi_dma_free_desc(struct virt_dma_desc *vd)
{
	struct hisi_dma_desc_sw *ds =
		container_of(vd, struct hisi_dma_desc_sw, vd);
	struct hisi_dma_dev *d = to_hisi_dma(vd->tx.chan->device);

	dma_pool_free(d->pool, ds->desc_hw, ds->desc_hw_lli);
	kfree(ds);
}

static struct dma_chan *hisi_dma_of_xlate(struct of_phandle_args *spec,
					  struct of_dma *ofdma)
{
	struct hisi_dma_dev *d = ofdma->of_dma_data;
	unsigned int request = spec->args[0];

	if (request >= d->dma_requests)
		return NULL;
	return dma_get_slave_channel(&d->chans[request].vc.chan);
}

static int hisi_dma_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct arm_smccc_res res;
	struct hisi_dma_dev *d;
	u32 pipeline, i;
	int irq, ret;

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	d->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(d->base))
		return PTR_ERR(d->base);

	if (of_property_read_u32(np, "dma-channels", &d->dma_channels) ||
	    of_property_read_u32(np, "dma-requests", &d->dma_requests) ||
	    of_property_read_u32(np, "dma-min-chan", &d->dma_min_chan) ||
	    of_property_read_u32(np, "dma-used-chans", &d->dma_used_chans) ||
	    d->dma_channels > 32 || d->dma_min_chan >= d->dma_channels)
		return dev_err_probe(dev, -EINVAL, "bad DMA description\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret)
		return ret;

	d->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(d->clk))
		return dev_err_probe(dev, PTR_ERR(d->clk), "no clock\n");

	/* the secure world owns the channels below dma-min-chan */
	arm_smccc_smc(DMAC_REGISTER_FN_ID, d->dma_min_chan, 0, 0, 0, 0, 0, 0,
		      &res);

	if (!of_property_read_u32(np, "dma-pipeline-en", &pipeline) && !pipeline)
		writel(readl(d->base + DMA_CTRL) & ~DMA_CTRL_PIPE_LINE,
		       d->base + DMA_CTRL);

	d->pool = dmam_pool_create(dev_name(dev), dev, LLI_BLOCK_SIZE,
				   sizeof(struct hisi_desc_hw), 0);
	if (!d->pool)
		return -ENOMEM;

	d->phy = devm_kcalloc(dev, d->dma_channels, sizeof(*d->phy), GFP_KERNEL);
	d->chans = devm_kcalloc(dev, d->dma_requests, sizeof(*d->chans),
				GFP_KERNEL);
	if (!d->phy || !d->chans)
		return -ENOMEM;
	for (i = 0; i < d->dma_channels; i++) {
		d->phy[i].idx = i;
		d->phy[i].base = d->base + i * CHAN_OFFSET;
	}

	spin_lock_init(&d->lock);
	INIT_LIST_HEAD(&d->chan_pending);
	tasklet_setup(&d->task, hisi_dma_tasklet);

	INIT_LIST_HEAD(&d->slave.channels);
	dma_cap_set(DMA_SLAVE, d->slave.cap_mask);
	dma_cap_set(DMA_MEMCPY, d->slave.cap_mask);
	dma_cap_set(DMA_PRIVATE, d->slave.cap_mask);
	d->slave.dev = dev;
	d->slave.device_free_chan_resources = hisi_dma_free_chan_resources;
	d->slave.device_tx_status = hisi_dma_tx_status;
	d->slave.device_prep_dma_memcpy = hisi_dma_prep_memcpy;
	d->slave.device_prep_slave_sg = hisi_dma_prep_slave_sg;
	d->slave.device_issue_pending = hisi_dma_issue_pending;
	d->slave.device_config = hisi_dma_config;
	d->slave.device_pause = hisi_dma_pause;
	d->slave.device_resume = hisi_dma_resume;
	d->slave.device_terminate_all = hisi_dma_terminate_all;
	d->slave.device_synchronize = hisi_dma_synchronize;
	d->slave.copy_align = DMA_ALIGN;
	d->slave.src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |
		BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) | BIT(DMA_SLAVE_BUSWIDTH_4_BYTES) |
		BIT(DMA_SLAVE_BUSWIDTH_8_BYTES);
	d->slave.dst_addr_widths = d->slave.src_addr_widths;
	d->slave.directions = BIT(DMA_MEM_TO_DEV) | BIT(DMA_DEV_TO_MEM) |
			      BIT(DMA_MEM_TO_MEM);
	d->slave.residue_granularity = DMA_RESIDUE_GRANULARITY_SEGMENT;

	for (i = 0; i < d->dma_requests; i++) {
		struct hisi_dma_chan *c = &d->chans[i];

		c->status = DMA_IN_PROGRESS;
		INIT_LIST_HEAD(&c->node);
		c->vc.desc_free = hisi_dma_free_desc;
		vchan_init(&c->vc, &d->slave);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = devm_request_irq(dev, irq, hisi_dma_int_handler, IRQF_SHARED,
			       dev_name(dev), d);
	if (ret)
		return ret;

	hisi_dma_enable_dma(d, true);

	ret = dmaenginem_async_device_register(&d->slave);
	if (ret)
		return ret;
	ret = of_dma_controller_register(np, hisi_dma_of_xlate, d);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, d);
	dev_info(dev, "%u channels (from %u), %u requests\n", d->dma_channels,
		 d->dma_min_chan, d->dma_requests);
	return 0;
}

static void hisi_dma_remove(struct platform_device *pdev)
{
	struct hisi_dma_dev *d = platform_get_drvdata(pdev);

	of_dma_controller_free(pdev->dev.of_node);
	hisi_dma_enable_dma(d, false);
	tasklet_kill(&d->task);
}

static const struct of_device_id hisi_dma64_of_match[] = {
	{ .compatible = "hisilicon,hisi-dma64-1.0" },
	{ }
};
MODULE_DEVICE_TABLE(of, hisi_dma64_of_match);

static struct platform_driver hisi_dma64_driver = {
	.probe = hisi_dma_probe,
	.remove = hisi_dma_remove,
	.driver = {
		.name = "hisi-dma64",
		.of_match_table = hisi_dma64_of_match,
	},
};
module_platform_driver(hisi_dma64_driver);

MODULE_DESCRIPTION("HiSilicon Kirin 64-bit peripheral DMA driver");
MODULE_LICENSE("GPL");
