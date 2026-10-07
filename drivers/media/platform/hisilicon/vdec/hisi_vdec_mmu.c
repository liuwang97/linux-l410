// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Kirin 990 video decoder: private IOMMU
 *
 * The vdec has its own SMMU (a common block at +0x20000 and a master block at
 * +0xf000). It walks an ARMv7 LPAE stage-1 table with a 32-bit input address,
 * the same format io-pgtable produces for ARM_32_LPAE_S1 with the NS quirk.
 * There is no known TLB invalidation register, so address ranges are only
 * handed out again after the block has been powered down.
 */

#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include "hisi_vdec.h"

#define HIVDEC_IOVA_START	0x00100000UL
#define HIVDEC_IOVA_END		0xe0000000UL

struct hivdec_deferred_range {
	struct list_head node;
	unsigned long iova;
	size_t size;
};

static void hivdec_tlb_flush_all(void *cookie)
{
}

static void hivdec_tlb_flush_walk(unsigned long iova, size_t size,
				  size_t granule, void *cookie)
{
}

static const struct iommu_flush_ops hivdec_flush_ops = {
	.tlb_flush_all = hivdec_tlb_flush_all,
	.tlb_flush_walk = hivdec_tlb_flush_walk,
};

int hivdec_mmu_init(struct hivdec_dev *vdec)
{
	struct hivdec_mmu *mmu = &vdec->mmu;
	int ret;

	mutex_init(&mmu->lock);
	INIT_LIST_HEAD(&mmu->deferred);

	mmu->cfg = (struct io_pgtable_cfg) {
		.pgsize_bitmap	= SZ_4K | SZ_2M | SZ_1G,
		.ias		= 32,
		.oas		= 40,
		.coherent_walk	= false,
		.tlb		= &hivdec_flush_ops,
		.iommu_dev	= vdec->dev,
		.quirks		= IO_PGTABLE_QUIRK_ARM_NS,
	};
	mmu->ops = alloc_io_pgtable_ops(ARM_32_LPAE_S1, &mmu->cfg, vdec);
	if (!mmu->ops)
		return -ENOMEM;

	mmu->iova_pool = gen_pool_create(PAGE_SHIFT, -1);
	if (!mmu->iova_pool) {
		ret = -ENOMEM;
		goto err_pgtable;
	}
	ret = gen_pool_add(mmu->iova_pool, HIVDEC_IOVA_START,
			   HIVDEC_IOVA_END - HIVDEC_IOVA_START, -1);
	if (ret)
		goto err_pool;

	/* scratch pages the SMMU writes to on a translation fault */
	ret = hivdec_aux_alloc(vdec, &mmu->err_rd, PAGE_SIZE);
	if (ret)
		goto err_pool;
	ret = hivdec_aux_alloc(vdec, &mmu->err_wr, PAGE_SIZE);
	if (ret)
		goto err_rd;

	dev_dbg(vdec->dev, "page table at %pad\n",
		&mmu->cfg.arm_lpae_s1_cfg.ttbr);
	return 0;

err_rd:
	hivdec_aux_free(vdec, &mmu->err_rd);
err_pool:
	gen_pool_destroy(mmu->iova_pool);
err_pgtable:
	free_io_pgtable_ops(mmu->ops);
	return ret;
}

void hivdec_mmu_fini(struct hivdec_dev *vdec)
{
	struct hivdec_mmu *mmu = &vdec->mmu;

	hivdec_aux_free(vdec, &mmu->err_wr);
	hivdec_aux_free(vdec, &mmu->err_rd);
	hivdec_mmu_powered_off(vdec);
	gen_pool_destroy(mmu->iova_pool);
	free_io_pgtable_ops(mmu->ops);
}

/* program the SMMU after the block was powered up (vendor smmu_init_global_reg) */
void hivdec_mmu_hw_setup(struct hivdec_dev *vdec)
{
	struct hivdec_mmu *mmu = &vdec->mmu;
	u64 ttbr = mmu->cfg.arm_lpae_s1_cfg.ttbr;
	u32 scr;
	int i;

	scr = vdh_read(vdec, SMMU_SCR);
	scr &= ~BIT(0);			/* glb_bypass off */
	scr |= BIT(1) | BIT(2);		/* rqos_en, wqos_en */
	vdh_write(vdec, SMMU_SCR, scr);

	/* stream matching: even SIDs 0x1c, odd SIDs 0x1d (non-secure, context 0) */
	for (i = 0; i < 32; i++)
		vdh_write(vdec, SMMU_SMRX_NS(i), (i & 1) ? 0x1d : 0x1c);

	vdh_write(vdec, SMMU_CB_TTBR0, lower_32_bits(ttbr));
	vdh_write(vdec, SMMU_CB_TTBR_MSB, upper_32_bits(ttbr) & 0xffff);
	vdh_write(vdec, SMMU_CB_TTBCR, vdh_read(vdec, SMMU_CB_TTBCR) | BIT(0));

	vdh_write(vdec, SMMU_ERR_RDADDR_NS, lower_32_bits(mmu->err_rd.dma));
	vdh_write(vdec, SMMU_ERR_WRADDR_NS, lower_32_bits(mmu->err_wr.dma));
	vdh_write(vdec, SMMU_ERR_ADDR_MSB_NS,
		  (upper_32_bits(mmu->err_rd.dma) & 0xffff) |
		  (upper_32_bits(mmu->err_wr.dma) & 0xffff) << 16);

	vdh_write(vdec, SMMU_MSTR_GLB_BYPASS, 0);
}

/* the SMMU lost its TLB: deferred ranges can be reused */
void hivdec_mmu_powered_off(struct hivdec_dev *vdec)
{
	struct hivdec_mmu *mmu = &vdec->mmu;
	struct hivdec_deferred_range *r, *tmp;

	mutex_lock(&mmu->lock);
	list_for_each_entry_safe(r, tmp, &mmu->deferred, node) {
		gen_pool_free(mmu->iova_pool, r->iova, r->size);
		list_del(&r->node);
		kfree(r);
	}
	mutex_unlock(&mmu->lock);
}

/* unmap_pages() also stops at a table boundary */
static void hivdec_unmap_range(struct io_pgtable_ops *ops, unsigned long iova,
			       size_t size)
{
	while (size) {
		size_t n = ops->unmap_pages(ops, iova, SZ_4K, size / SZ_4K, NULL);

		if (WARN_ON(!n))
			break;
		iova += n;
		size -= n;
	}
}

int hivdec_mmu_map_sgt(struct hivdec_dev *vdec, struct sg_table *sgt,
		       size_t size, struct hivdec_mapping *map)
{
	struct hivdec_mmu *mmu = &vdec->mmu;
	struct io_pgtable_ops *ops = mmu->ops;
	struct scatterlist *sg;
	unsigned long iova, cur;
	size_t mapped_total = 0;
	int i, ret = 0;

	size = PAGE_ALIGN(size);
	iova = gen_pool_alloc(mmu->iova_pool, size);
	if (!iova)
		return -ENOSPC;

	mutex_lock(&mmu->lock);
	cur = iova;
	for_each_sgtable_sg(sgt, sg, i) {
		phys_addr_t pa = sg_phys(sg);
		size_t len = sg->length, mapped = 0;

		if (WARN_ON(!PAGE_ALIGNED(pa) || !PAGE_ALIGNED(len))) {
			ret = -EINVAL;
			break;
		}
		len = min_t(size_t, len, size - mapped_total);
		if (!len)
			break;
		/* map_pages() stops at the end of a leaf table: loop */
		while (len) {
			mapped = 0;
			ret = ops->map_pages(ops, cur, pa, SZ_4K, len / SZ_4K,
					     IOMMU_READ | IOMMU_WRITE, GFP_KERNEL,
					     &mapped);
			cur += mapped;
			pa += mapped;
			len -= mapped;
			mapped_total += mapped;
			if (ret || !mapped)
				break;
		}
		if (ret || len)
			break;
	}
	mutex_unlock(&mmu->lock);

	if (!ret && mapped_total < size)
		ret = -EINVAL;
	if (ret) {
		/* never used by the hardware, so the range can be reused at once */
		if (mapped_total)
			hivdec_unmap_range(ops, iova, mapped_total);
		gen_pool_free(mmu->iova_pool, iova, size);
		return ret;
	}

	map->iova = iova;
	map->size = size;
	return 0;
}

void hivdec_mmu_unmap(struct hivdec_dev *vdec, struct hivdec_mapping *map)
{
	struct hivdec_mmu *mmu = &vdec->mmu;
	struct hivdec_deferred_range *r;

	if (!map->size)
		return;

	mutex_lock(&mmu->lock);
	hivdec_unmap_range(mmu->ops, map->iova, map->size);
	r = kmalloc(sizeof(*r), GFP_KERNEL);
	if (r && vdec->powered) {
		r->iova = map->iova;
		r->size = map->size;
		list_add_tail(&r->node, &mmu->deferred);
	} else {
		kfree(r);
		gen_pool_free(mmu->iova_pool, map->iova, map->size);
	}
	mutex_unlock(&mmu->lock);
	map->size = 0;
}

/*
 * Driver-internal buffers: page based (no CMA), kernel-mapped, cache
 * maintenance through the streaming DMA API.
 */
struct hivdec_aux_priv {
	struct sg_table sgt;
	struct page **pages;
	unsigned int npages;
};

int hivdec_aux_alloc(struct hivdec_dev *vdec, struct hivdec_aux_buf *buf,
		     size_t size)
{
	struct hivdec_aux_priv *p;
	unsigned int i;
	int ret = -ENOMEM;

	size = PAGE_ALIGN(size);
	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->npages = size >> PAGE_SHIFT;
	p->pages = kvcalloc(p->npages, sizeof(*p->pages), GFP_KERNEL);
	if (!p->pages)
		goto err_free;
	for (i = 0; i < p->npages; i++) {
		p->pages[i] = alloc_page(GFP_KERNEL | __GFP_ZERO);
		if (!p->pages[i])
			goto err_pages;
	}
	ret = sg_alloc_table_from_pages(&p->sgt, p->pages, p->npages, 0, size,
					GFP_KERNEL);
	if (ret)
		goto err_pages;
	ret = dma_map_sgtable(vdec->dev, &p->sgt, DMA_BIDIRECTIONAL, 0);
	if (ret)
		goto err_sgt;
	buf->cpu = vmap(p->pages, p->npages, VM_MAP, PAGE_KERNEL);
	if (!buf->cpu) {
		ret = -ENOMEM;
		goto err_unmap;
	}
	ret = hivdec_mmu_map_sgt(vdec, &p->sgt, size, &buf->map);
	if (ret)
		goto err_vunmap;

	buf->dma = page_to_phys(p->pages[0]);
	buf->size = size;
	buf->priv = p;
	return 0;

err_vunmap:
	vunmap(buf->cpu);
	buf->cpu = NULL;
err_unmap:
	dma_unmap_sgtable(vdec->dev, &p->sgt, DMA_BIDIRECTIONAL, 0);
err_sgt:
	sg_free_table(&p->sgt);
err_pages:
	while (i--)
		__free_page(p->pages[i]);
	kvfree(p->pages);
err_free:
	kfree(p);
	return ret;
}

void hivdec_aux_free(struct hivdec_dev *vdec, struct hivdec_aux_buf *buf)
{
	struct hivdec_aux_priv *p = buf->priv;
	unsigned int i;

	if (!p)
		return;
	hivdec_mmu_unmap(vdec, &buf->map);
	vunmap(buf->cpu);
	dma_unmap_sgtable(vdec->dev, &p->sgt, DMA_BIDIRECTIONAL, 0);
	sg_free_table(&p->sgt);
	for (i = 0; i < p->npages; i++)
		__free_page(p->pages[i]);
	kvfree(p->pages);
	kfree(p);
	buf->priv = NULL;
	buf->cpu = NULL;
}

void hivdec_aux_sync_for_device(struct hivdec_dev *vdec, struct hivdec_aux_buf *buf)
{
	struct hivdec_aux_priv *p = buf->priv;

	dma_sync_sgtable_for_device(vdec->dev, &p->sgt, DMA_BIDIRECTIONAL);
}

void hivdec_aux_sync_for_cpu(struct hivdec_dev *vdec, struct hivdec_aux_buf *buf)
{
	struct hivdec_aux_priv *p = buf->priv;

	dma_sync_sgtable_for_cpu(vdec->dev, &p->sgt, DMA_BIDIRECTIONAL);
}
