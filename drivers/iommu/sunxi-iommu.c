// SPDX-License-Identifier: GPL-2.0
/*
 * Allwinner IOMMU driver (quirks-based, multi-SoC)
 *
 * Derived from the Allwinner BSP IOMMU driver.
 * Original author: zhuxianbin <zhuxianbin@allwinnertech.com>
 * Copyright (C) 2016-2018, Allwinner Technology CO., LTD.
 *
 * Refactored to use per-SoC quirks instead of compile-time #ifdef.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/of_iommu.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/iommu.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/sizes.h>
#include <linux/device.h>
#include <asm/cacheflush.h>
#include <linux/pm_runtime.h>

#include "sunxi-iommu.h"

/* ================================================================== */
/* Per-SoC quirks definitions                                          */
/* ================================================================== */

/*
 * V1 — 6 micro-TLBs, addr+mask TLB/PTW invalidation.
 * BSP platforms: sun50iw3 (A63 / H6-class), sun50iw6
 */
static const char *const sunxi_masters_v1[] = {
	"DE", "DI", "VE", "CSI", "G2D", "RESERVED",
};

static const struct sunxi_iommu_quirks sun50i_a63_iommu_quirks = {
	.num_micro_tlbs		= 6,
	.tlb_prefetch_val	= 0x7f,
	.master_names		= sunxi_masters_v1,
};

/*
 * V2 — 7 micro-TLBs, addr+mask TLB/PTW invalidation.
 * BSP platforms: sun8iw15 (A50 / V831-class)
 */
static const char *const sunxi_masters_v2[] = {
	"DE", "E_EDMA", "E_FE", "VE", "CSI", "G2D", "E_BE",
};

static const struct sunxi_iommu_quirks sun8i_a50_iommu_quirks = {
	.num_micro_tlbs		= 7,
	.tlb_prefetch_val	= 0x5f,
	.master_names		= sunxi_masters_v2,
};

/*
 * V3 — 7 micro-TLBs, range-based TLB invalidation, single-addr PTW.
 * BSP platforms: sun50iw9 (H616), sun8iw19, sun50iw10
 *
 * sun50iw10 has two IC revisions; the newer one has a prefetch-valid
 * fix that can be indicated via the DT boolean property
 * "allwinner,sun50iw10-iommu-fixed".  When that property is present
 * the driver enables the fix and uses prefetch = 0x5f instead of 0x00.
 */
static const char *const sunxi_masters_v3[] = {
	"DE", "DI", "VE_R", "VE", "CSI0", "CSI1", "G2D",
};

static const struct sunxi_iommu_quirks sun50i_h616_iommu_quirks = {
	.num_micro_tlbs		= 7,
	.tlb_ivld_range		= true,
	.tlb_prefetch_val	= 0x00,
	.master_names		= sunxi_masters_v3,
};

/*
 * V4 — 7 micro-TLBs, range-based TLB + PTW, prefetch-valid fix.
 * BSP platforms: sun8iw20 (T113), sun20iw1 (D1), sun50iw12, sun55iw3
 */
static const char *const sunxi_masters_v4[] = {
	"DE", "DI", "VE", "CSI", "G2D", "RESERVED", "RESERVED",
};

static const struct sunxi_iommu_quirks sun20i_d1_iommu_quirks = {
	.num_micro_tlbs		= 7,
	.tlb_ivld_range		= true,
	.ptw_ivld_range		= true,
	.has_prefetch_fix	= true,
	.tlb_prefetch_val	= 0x16,
	.master_names		= sunxi_masters_v4,
};

/* ================================================================== */
/* Static globals (kept for BSP compatibility)                         */
/* ================================================================== */
static struct kmem_cache *iopte_cache;
static struct sunxi_iommu_dev *global_iommu_dev;
static struct sunxi_iommu_domain *global_sunxi_iommu_domain;
struct iommu_domain *global_iommu_domain;
struct sunxi_iommu_owner *global_iommu_owner;
static bool tlb_init_flag;
static struct device *dma_dev;
EXPORT_SYMBOL_GPL(global_iommu_domain);

/* ================================================================== */
/* Low-level register helpers                                          */
/* ================================================================== */
static inline u32 sunxi_iommu_read(struct sunxi_iommu_dev *iommu, u32 offset)
{
	return readl(iommu->base + offset);
}

static inline void sunxi_iommu_write(struct sunxi_iommu_dev *iommu,
				      u32 offset, u32 value)
{
	writel(value, iommu->base + offset);
}

/* ================================================================== */
/* Page-table address helpers                                          */
/* ================================================================== */
static inline u32 *iopde_offset(u32 *iopd, unsigned int iova)
{
	return iopd + IOPDE_INDEX(iova);
}

static inline u32 *iopte_offset(u32 *ent, unsigned int iova)
{
	unsigned long iopte_base = IOPTE_BASE(*ent);

	if (iopte_base < SUNXI_PHYS_OFFSET)
		iopte_base += SUNXI_4G_PHYS_OFFSET;

	return (u32 *)__va(iopte_base) + IOPTE_INDEX(iova);
}

static inline u32 *iopte_page(u32 *ent)
{
	unsigned long iopte_base = IOPTE_BASE(*ent);

	if (iopte_base < SUNXI_PHYS_OFFSET)
		iopte_base += SUNXI_4G_PHYS_OFFSET;

	return (u32 *)__va(iopte_base);
}

static inline dma_addr_t iommu_phy_to_cpu_phy(dma_addr_t iommu_phy)
{
	return iommu_phy < SUNXI_PHYS_OFFSET ?
		iommu_phy + SUNXI_4G_PHYS_OFFSET : iommu_phy;
}

/* ================================================================== */
/* TLB flush (full)                                                    */
/* ================================================================== */
static int sunxi_tlb_flush(struct sunxi_iommu_dev *iommu)
{
	int ret;

	sunxi_iommu_write(iommu, IOMMU_TLB_FLUSH_ENABLE_REG,
			  MICRO_TLB_FLUSH_MASK(iommu->quirks->num_micro_tlbs) |
			  PTW_CACHE_FLUSH_BIT |
			  MACRO_TLB_FLUSH_BIT);
	ret = sunxi_wait_when(
		sunxi_iommu_read(iommu, IOMMU_TLB_FLUSH_ENABLE_REG), 2);
	if (ret)
		dev_err(iommu->dev, "Enable flush all request timed out\n");

	return ret;
}

/* ================================================================== */
/* TLB invalidation primitives                                         */
/* ================================================================== */

/* Range-based TLB invalidation (V3 / V4) */
static int sunxi_tlb_invalid_range(struct sunxi_iommu_dev *iommu,
				   dma_addr_t start, dma_addr_t end)
{
	int ret;
	unsigned long mflag;

	spin_lock_irqsave(&iommu->iommu_lock, mflag);
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_START_ADDR_REG, start);
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_END_ADDR_REG, end);
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_ENABLE_REG, 0x1);
	ret = sunxi_wait_when(
		sunxi_iommu_read(iommu, IOMMU_TLB_IVLD_ENABLE_REG) & 0x1, 2);
	if (ret)
		dev_err(iommu->dev, "TLB range-invalid timed out\n");
	spin_unlock_irqrestore(&iommu->iommu_lock, mflag);

	return ret;
}

/* Address + mask TLB invalidation (V1 / V2) */
static int sunxi_tlb_invalid_addr(struct sunxi_iommu_dev *iommu,
				  dma_addr_t addr, u32 mask)
{
	int ret;
	unsigned long mflag;

	spin_lock_irqsave(&iommu->iommu_lock, mflag);
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_ADDR_REG, addr);
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_ADDR_MASK_REG, mask);
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_ENABLE_REG, 0x1);
	ret = sunxi_wait_when(
		sunxi_iommu_read(iommu, IOMMU_TLB_IVLD_ENABLE_REG) & 0x1, 2);
	if (ret)
		dev_err(iommu->dev, "TLB addr-invalid timed out\n");
	spin_unlock_irqrestore(&iommu->iommu_lock, mflag);

	return ret;
}

/* ================================================================== */
/* PTW cache invalidation primitives                                   */
/* ================================================================== */

/* Range-based PTW cache invalidation (V4) */
static int sunxi_ptw_cache_invalid_range(struct sunxi_iommu_dev *iommu,
					 dma_addr_t start, dma_addr_t end)
{
	int ret;
	unsigned long mflag;

	spin_lock_irqsave(&iommu->iommu_lock, mflag);
	sunxi_iommu_write(iommu, IOMMU_PC_IVLD_START_ADDR_REG, start);
	sunxi_iommu_write(iommu, IOMMU_PC_IVLD_END_ADDR_REG, end);
	sunxi_iommu_write(iommu, IOMMU_PC_IVLD_ENABLE_REG, 0x1);
	ret = sunxi_wait_when(
		sunxi_iommu_read(iommu, IOMMU_PC_IVLD_ENABLE_REG) & 0x1, 2);
	if (ret)
		dev_err(iommu->dev, "PTW range-invalid timed out\n");
	spin_unlock_irqrestore(&iommu->iommu_lock, mflag);

	return ret;
}

/* Single-address PTW cache invalidation (V1 / V2 / V3) */
static int sunxi_ptw_cache_invalid_addr(struct sunxi_iommu_dev *iommu,
					dma_addr_t addr)
{
	int ret;
	unsigned long mflag;

	spin_lock_irqsave(&iommu->iommu_lock, mflag);
	sunxi_iommu_write(iommu, IOMMU_PC_IVLD_ADDR_REG, addr);
	sunxi_iommu_write(iommu, IOMMU_PC_IVLD_ENABLE_REG, 0x1);
	ret = sunxi_wait_when(
		sunxi_iommu_read(iommu, IOMMU_PC_IVLD_ENABLE_REG) & 0x1, 2);
	if (ret)
		dev_err(iommu->dev, "PTW addr-invalid timed out\n");
	spin_unlock_irqrestore(&iommu->iommu_lock, mflag);

	return ret;
}

/* ================================================================== */
/* Unified zap-TLB (called after map / before unmap completes)         */
/* ================================================================== */
static void sunxi_zap_tlb(struct sunxi_iommu_dev *iommu,
			  unsigned long iova, size_t size)
{
	const struct sunxi_iommu_quirks *q = iommu->quirks;
	bool effective_fix = q->has_prefetch_fix || iommu->prefetch_fixed;

	/* --- TLB invalidation --- */
	if (q->tlb_ivld_range) {
		sunxi_tlb_invalid_range(iommu, iova,
					iova + 2 * SPAGE_SIZE);
		sunxi_tlb_invalid_range(iommu,
					iova + size - SPAGE_SIZE,
					iova + size + 8 * SPAGE_SIZE);
	} else {
		sunxi_tlb_invalid_addr(iommu, iova, (u32)IOMMU_PT_MASK);
		sunxi_tlb_invalid_addr(iommu, iova + SPAGE_SIZE,
				       (u32)IOMMU_PT_MASK);
		sunxi_tlb_invalid_addr(iommu, iova + size,
				       (u32)IOMMU_PT_MASK);
		sunxi_tlb_invalid_addr(iommu, iova + size + SPAGE_SIZE,
				       (u32)IOMMU_PT_MASK);
	}

	/* --- PTW cache invalidation --- */
	if (q->ptw_ivld_range) {
		/* V4: range-based PTW */
		sunxi_ptw_cache_invalid_range(iommu, iova,
					      iova + SPD_SIZE);
		sunxi_ptw_cache_invalid_range(iommu,
					      iova + size - SPD_SIZE,
					      iova + size);
	} else if (!q->tlb_ivld_range) {
		/* V1 / V2: always 4 PTW invalidations */
		sunxi_ptw_cache_invalid_addr(iommu, iova);
		sunxi_ptw_cache_invalid_addr(iommu, iova + SPD_SIZE);
		sunxi_ptw_cache_invalid_addr(iommu, iova + size);
		sunxi_ptw_cache_invalid_addr(iommu, iova + size + SPD_SIZE);
	} else {
		/* V3: 2 basic PTW invalidations + workaround for old HW */
		sunxi_ptw_cache_invalid_addr(iommu, iova);
		sunxi_ptw_cache_invalid_addr(iommu, iova + size);

		if (!effective_fix) {
			sunxi_ptw_cache_invalid_addr(iommu,
						     iova + SPD_SIZE);
			sunxi_ptw_cache_invalid_addr(iommu,
						     iova + size + SPD_SIZE);
			sunxi_ptw_cache_invalid_addr(iommu,
						     iova + size + 2 * SPD_SIZE);
		}
	}
}

/* ================================================================== */
/* TLB init (called on first attach and on resume)                     */
/* ================================================================== */
static int sunxi_tlb_init(struct sunxi_iommu_owner *owner,
			  struct iommu_domain *input_domain)
{
	struct sunxi_iommu_dev *iommu = owner->data;
	const struct sunxi_iommu_quirks *q = iommu->quirks;
	bool effective_fix = q->has_prefetch_fix || iommu->prefetch_fixed;
	u32 prefetch_val;
	int ret = 0;
	int iommu_enable = 0;
	phys_addr_t dte_addr;
	unsigned long mflag;
	struct sunxi_iommu_domain *sunxi_domain =
		container_of(input_domain, struct sunxi_iommu_domain, domain);

	/* If DT override is active, use 0x5f instead of the quirk default */
	prefetch_val = iommu->prefetch_fixed ? 0x5f : q->tlb_prefetch_val;

	spin_lock_irqsave(&iommu->iommu_lock, mflag);

	dte_addr = __pa(sunxi_domain->pgtable);
	sunxi_iommu_write(iommu, IOMMU_TTB_REG, dte_addr);

	/* TLB prefetch configuration */
	sunxi_iommu_write(iommu, IOMMU_TLB_PREFETCH_REG, prefetch_val);

	/* Interrupt enable (micro-TLBs 0..5 + L1/L2) */
	sunxi_iommu_write(iommu, IOMMU_INT_ENABLE_REG, 0x3003f);
	sunxi_iommu_write(iommu, IOMMU_BYPASS_REG, iommu->bypass);

	/* TLB / PTW invalidation mode selection */
	if (q->ptw_ivld_range) {
		/* V4: both range-based */
		sunxi_iommu_write(iommu, IOMMU_PC_IVLD_MODE_SEL_REG, 0x1);
		sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_MODE_SEL_REG, 0x1);
	} else if (q->tlb_ivld_range) {
		/* V3: range-based TLB, single-addr PTW */
		sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_MODE_SEL_REG, 0x1);
	} else {
		/* V1 / V2: addr+mask for both */
		sunxi_iommu_write(iommu, IOMMU_PC_IVLD_MODE_SEL_REG, 0x0);
		sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_MODE_SEL_REG, 0x0);
	}

	/* Enable prefetch-valid fix if applicable */
	if (effective_fix) {
		u32 pf = sunxi_iommu_read(iommu, IOMMU_TLB_PREFETCH_REG);

		sunxi_iommu_write(iommu, IOMMU_TLB_PREFETCH_REG,
				  pf | 0x30000);
	}

	ret = sunxi_tlb_flush(iommu);
	if (ret) {
		dev_err(iommu->dev, "Enable flush all request timed out\n");
		goto out;
	}

	sunxi_iommu_write(iommu, IOMMU_AUTO_GATING_REG, 0x1);
	sunxi_iommu_write(iommu, IOMMU_ENABLE_REG, IOMMU_ENABLE);

	iommu_enable = sunxi_iommu_read(iommu, IOMMU_ENABLE_REG);
	if (iommu_enable != 0x1) {
		iommu_enable = sunxi_iommu_read(iommu, IOMMU_ENABLE_REG);
		if (iommu_enable != 0x1) {
			dev_err(iommu->dev,
				"iommu enable failed! No iommu in bitfile!\n");
			ret = -ENODEV;
			goto out;
		}
	}
	tlb_init_flag = true;

out:
	spin_unlock_irqrestore(&iommu->iommu_lock, mflag);
	return ret;
}

/* ================================================================== */
/* Page-table entry helpers                                            */
/* ================================================================== */
static int sunxi_alloc_iopte(u32 *sent, int prot)
{
	u32 *pent;

	pent = kmem_cache_zalloc(iopte_cache, GFP_ATOMIC);
	WARN_ON((unsigned long)pent & (PT_SIZE - 1));
	if (!pent)
		return 0;

	dma_sync_single_for_cpu(dma_dev, virt_to_phys(sent),
				sizeof(*sent), DMA_TO_DEVICE);
	*sent = __pa(pent) | DENT_VALID;
	dma_sync_single_for_device(dma_dev, virt_to_phys(sent),
				   sizeof(*sent), DMA_TO_DEVICE);

	return 1;
}

static void sunxi_free_iopte(u32 *pent)
{
	kmem_cache_free(iopte_cache, pent);
}

static inline u32 sunxi_mk_pte(u32 page, int prot)
{
	u32 flags = 0;

	flags |= (prot & IOMMU_READ) ? SUNXI_PTE_PAGE_READABLE : 0;
	flags |= (prot & IOMMU_WRITE) ? SUNXI_PTE_PAGE_WRITABLE : 0;
	page &= IOMMU_PT_MASK;

	return page | flags | SUNXI_PTE_PAGE_VALID;
}

/* ================================================================== */
/* Exported device helpers                                             */
/* ================================================================== */
void sunxi_reset_device_iommu(unsigned int master_id)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	unsigned int regval;

	regval = sunxi_iommu_read(iommu, IOMMU_RESET_REG);
	sunxi_iommu_write(iommu, IOMMU_RESET_REG,
			  regval & ~BIT(master_id));
	regval = sunxi_iommu_read(iommu, IOMMU_RESET_REG);
	if (!(regval & BIT(master_id)))
		sunxi_iommu_write(iommu, IOMMU_RESET_REG,
				  regval | BIT(master_id));
}
EXPORT_SYMBOL(sunxi_reset_device_iommu);

void sunxi_enable_device_iommu(unsigned int master_id, bool flag)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	unsigned long mflag;

	spin_lock_irqsave(&iommu->iommu_lock, mflag);
	if (flag)
		iommu->bypass &= ~BIT(master_id);
	else
		iommu->bypass |= BIT(master_id);
	sunxi_iommu_write(iommu, IOMMU_BYPASS_REG, iommu->bypass);
	spin_unlock_irqrestore(&iommu->iommu_lock, mflag);
}
EXPORT_SYMBOL(sunxi_enable_device_iommu);

/* ================================================================== */
/* IOMMU map / unmap                                                   */
/* ================================================================== */
static int sunxi_iommu_map(struct iommu_domain *domain, unsigned long iova,
			   phys_addr_t paddr, size_t pgsize, size_t pgcount,
			   int prot, gfp_t gfp, size_t *mapped)
{
	struct sunxi_iommu_domain *sunxi_domain =
		container_of(domain, struct sunxi_iommu_domain, domain);
	size_t iova_start, iova_end, paddr_start, s_iova_start;
	size_t size = pgsize * pgcount;
	u32 *dent, *pent;
	int i, j;
	int flush_count = 0;

	(void)gfp;
	WARN_ON(!sunxi_domain->pgtable);
	iova_start = iova & IOMMU_PT_MASK;
	paddr_start = paddr & IOMMU_PT_MASK;
	iova_end = SPAGE_ALIGN(iova + size) - SPAGE_SIZE;
	s_iova_start = iova_start;

	mutex_lock(&sunxi_domain->dt_lock);

	if (IOPDE_INDEX(iova_start) == IOPDE_INDEX(iova_end)) {
		dent = iopde_offset(sunxi_domain->pgtable, iova_start);
		if (!IS_VALID(*dent))
			sunxi_alloc_iopte(dent, prot);
		for (flush_count = 0; iova_start <= iova_end;
		     iova_start += SPAGE_SIZE, paddr_start += SPAGE_SIZE) {
			pent = iopte_offset(dent, iova_start);
			WARN_ON(*pent);
			*pent = sunxi_mk_pte(paddr_start, prot);
			++flush_count;
		}
		dma_sync_single_for_device(dma_dev,
			virt_to_phys(iopte_offset(dent, s_iova_start)),
			flush_count << 2, DMA_TO_DEVICE);
		goto out;
	}

	/* First partial PDE */
	dent = iopde_offset(sunxi_domain->pgtable, iova_start);
	if (!IS_VALID(*dent))
		sunxi_alloc_iopte(dent, prot);
	for (flush_count = 0;
	     iova_start < SPDE_ALIGN(s_iova_start + 1);
	     iova_start += SPAGE_SIZE, paddr_start += SPAGE_SIZE) {
		pent = iopte_offset(dent, iova_start);
		WARN_ON(*pent);
		*pent = sunxi_mk_pte(paddr_start, prot);
		++flush_count;
	}
	dma_sync_single_for_device(dma_dev,
		virt_to_phys(iopte_offset(dent, s_iova_start)),
		flush_count << 2, DMA_TO_DEVICE);

	/* Full PDEs in the middle */
	if (IOPDE_INDEX(iova_start) < IOPDE_INDEX(iova_end)) {
		for (i = IOPDE_INDEX(iova_start); i < IOPDE_INDEX(iova_end);
		     i++, iova_start += SPD_SIZE, paddr_start += SPD_SIZE) {
			dent = iopde_offset(sunxi_domain->pgtable, iova_start);
			if (!IS_VALID(*dent))
				sunxi_alloc_iopte(dent, prot);
			pent = iopte_offset(dent, iova_start);
			dma_sync_single_for_cpu(dma_dev,
				virt_to_phys(pent), PT_SIZE, DMA_TO_DEVICE);
			for (j = 0; j < NUM_ENTRIES_PTE; j++)
				*(pent + j) = sunxi_mk_pte(
					paddr_start + (j * SPAGE_SIZE), prot);
			dma_sync_single_for_device(dma_dev,
				virt_to_phys(pent), PT_SIZE, DMA_TO_DEVICE);
		}
	}

	/* Last partial PDE */
	s_iova_start = iova_start;
	dent = iopde_offset(sunxi_domain->pgtable, iova_start);
	if (!IS_VALID(*dent))
		sunxi_alloc_iopte(dent, prot);
	for (flush_count = 0; iova_start <= iova_end;
	     iova_start += SPAGE_SIZE, paddr_start += SPAGE_SIZE) {
		pent = iopte_offset(dent, iova_start);
		WARN_ON(*pent);
		*pent = sunxi_mk_pte(paddr_start, prot);
		++flush_count;
	}
	dma_sync_single_for_device(dma_dev,
		virt_to_phys(iopte_offset(dent, s_iova_start)),
		flush_count << 2, DMA_TO_DEVICE);

out:
	if (!(prot & BIT(16)))
		sunxi_zap_tlb(global_iommu_dev, iova, size);
	mutex_unlock(&sunxi_domain->dt_lock);

	if (mapped)
		*mapped = size;

	return 0;
}

static size_t sunxi_iommu_unmap(struct iommu_domain *domain, unsigned long iova,
				size_t pgsize, size_t pgcount,
				struct iommu_iotlb_gather *gather)
{
	struct sunxi_iommu_domain *sunxi_domain =
		container_of(domain, struct sunxi_iommu_domain, domain);
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	const struct sunxi_iommu_quirks *q = iommu->quirks;
	bool effective_fix = q->has_prefetch_fix || iommu->prefetch_fixed;
	size_t iova_start, iova_end, s_iova_start;
	size_t size = pgsize * pgcount;
	u32 *dent, *pent;
	int i;
	int flush_count = 0;

	WARN_ON(!sunxi_domain->pgtable);
	iova_start = iova & IOMMU_PT_MASK;
	iova_end = SPAGE_ALIGN(iova + size) - SPAGE_SIZE;
	s_iova_start = iova_start;

	gather->start = iova_start;
	gather->end = iova_end;

	mutex_lock(&sunxi_domain->dt_lock);

	/* V3/V4: bulk TLB invalidation first */
	if (q->tlb_ivld_range)
		sunxi_tlb_invalid_range(iommu, iova_start, iova_end);

	/* V4: also bulk PTW cache invalidation */
	if (q->ptw_ivld_range)
		sunxi_ptw_cache_invalid_range(iommu, iova_start, iova_end);

	/* Case 1: iova_start and iova_end within same PDE */
	if (IOPDE_INDEX(iova_start) == IOPDE_INDEX(iova_end)) {
		dent = iopde_offset(sunxi_domain->pgtable, iova_start);
		if (IS_VALID(*dent)) {
			u32 *pent_page = iopte_page(dent);

			for (flush_count = 0; iova_start <= iova_end;
			     iova_start += SPAGE_SIZE) {
				pent = iopte_offset(dent, iova_start);
				*pent = 0;
				if (!q->tlb_ivld_range)
					sunxi_tlb_invalid_addr(iommu,
						iova_start,
						(u32)IOMMU_PT_MASK);
				++flush_count;
			}
			dma_sync_single_for_device(dma_dev,
				virt_to_phys(iopte_offset(dent, s_iova_start)),
				flush_count << 2, DMA_TO_DEVICE);

			if (!q->ptw_ivld_range)
				sunxi_ptw_cache_invalid_addr(iommu,
							     s_iova_start);

			/* Free the PTE page */
			dma_sync_single_for_cpu(dma_dev,
				virt_to_phys(dent), sizeof(*dent),
				DMA_TO_DEVICE);
			*dent = 0;
			dma_sync_single_for_device(dma_dev,
				virt_to_phys(dent), sizeof(*dent),
				DMA_TO_DEVICE);
			sunxi_free_iopte(pent_page);
		}
		goto done;
	}

	/* Case 2.1: first partial PDE */
	dent = iopde_offset(sunxi_domain->pgtable, iova_start);
	if (IS_VALID(*dent)) {
		for (flush_count = 0;
		     iova_start < SPDE_ALIGN(s_iova_start + 1);
		     iova_start += SPAGE_SIZE) {
			pent = iopte_offset(dent, iova_start);
			*pent = 0;
			if (!q->tlb_ivld_range)
				sunxi_tlb_invalid_addr(iommu, iova_start,
						       (u32)IOMMU_PT_MASK);
			++flush_count;
		}
		dma_sync_single_for_device(dma_dev,
			virt_to_phys(iopte_offset(dent, s_iova_start)),
			flush_count << 2, DMA_TO_DEVICE);

		if (!q->ptw_ivld_range)
			sunxi_ptw_cache_invalid_addr(iommu, s_iova_start);
	}

	/* Case 2.2: full PDEs in the middle */
	if (IOPDE_INDEX(iova_start) < IOPDE_INDEX(iova_end)) {
		for (i = IOPDE_INDEX(iova_start); i < IOPDE_INDEX(iova_end);
		     i++, iova_start += SPD_SIZE) {
			dent = iopde_offset(sunxi_domain->pgtable, iova_start);
			if (!IS_VALID(*dent))
				continue;

			{
				u32 *pent_page = iopte_page(dent);

				pent = iopte_offset(dent, iova_start);
				dma_sync_single_for_cpu(dma_dev,
					virt_to_phys(pent), PT_SIZE,
					DMA_TO_DEVICE);
				memset(pent, 0, PT_SIZE);
				dma_sync_single_for_device(dma_dev,
					virt_to_phys(pent), PT_SIZE,
					DMA_TO_DEVICE);

				if (!q->tlb_ivld_range)
					sunxi_tlb_invalid_addr(iommu,
						iova_start,
						(u32)IOMMU_PD_MASK);

				dma_sync_single_for_cpu(dma_dev,
					virt_to_phys(dent), sizeof(*dent),
					DMA_TO_DEVICE);
				*dent = 0;
				dma_sync_single_for_device(dma_dev,
					virt_to_phys(dent), sizeof(*dent),
					DMA_TO_DEVICE);

				if (!q->ptw_ivld_range)
					sunxi_ptw_cache_invalid_addr(iommu,
								     iova_start);
				sunxi_free_iopte(pent_page);
			}
		}
	}

	/* Case 2.3: last partial PDE */
	s_iova_start = iova_start;
	dent = iopde_offset(sunxi_domain->pgtable, iova_start);
	if (IS_VALID(*dent)) {
		for (flush_count = 0; iova_start <= iova_end;
		     iova_start += SPAGE_SIZE) {
			pent = iopte_offset(dent, iova_start);
			*pent = 0;
			if (!q->tlb_ivld_range)
				sunxi_tlb_invalid_addr(iommu, iova_start,
						       (u32)IOMMU_PT_MASK);
			++flush_count;
		}
		dma_sync_single_for_device(dma_dev,
			virt_to_phys(iopte_offset(dent, s_iova_start)),
			flush_count << 2, DMA_TO_DEVICE);

		if (!q->ptw_ivld_range)
			sunxi_ptw_cache_invalid_addr(iommu, s_iova_start);
	}

done:
	/*
	 * V3 without prefetch-valid fix: need extra zap_tlb because the
	 * hardware may have prefetched stale entries during bulk-invalidate.
	 * V1/V2 already invalidate per-page inline.
	 * V4 does range PTW invalidation upfront so this is not needed.
	 */
	if (q->tlb_ivld_range && !q->ptw_ivld_range && !effective_fix)
		sunxi_zap_tlb(iommu, iova, size);

	mutex_unlock(&sunxi_domain->dt_lock);

	return size;
}

/* ================================================================== */
/* IOTLB sync                                                          */
/* ================================================================== */
static void sunxi_iommu_iotlb_sync(struct iommu_domain *domain,
				    struct iommu_iotlb_gather *gather)
{
	struct sunxi_iommu_domain *sunxi_domain =
		container_of(domain, struct sunxi_iommu_domain, domain);
	unsigned long iova_start = gather->start & IOMMU_PT_MASK;
	unsigned long iova_end = SPAGE_ALIGN(gather->end);
	size_t size = iova_end - iova_start;

	mutex_lock(&sunxi_domain->dt_lock);
	sunxi_zap_tlb(global_iommu_dev, iova_start, size);
	mutex_unlock(&sunxi_domain->dt_lock);
}

/* ================================================================== */
/* IOVA-to-phys                                                        */
/* ================================================================== */
static phys_addr_t
sunxi_iommu_iova_to_phys(struct iommu_domain *domain, dma_addr_t iova)
{
	struct sunxi_iommu_domain *sunxi_domain =
		container_of(domain, struct sunxi_iommu_domain, domain);
	u32 *dent, *pent;
	phys_addr_t ret = 0;

	WARN_ON(!sunxi_domain->pgtable);
	mutex_lock(&sunxi_domain->dt_lock);
	dent = iopde_offset(sunxi_domain->pgtable, iova);
	if (IS_VALID(*dent)) {
		pent = iopte_offset(dent, iova);
		if (*pent) {
			ret = IOPTE_TO_PFN(pent) + IOVA_PAGE_OFT(iova);
			if (ret < SUNXI_PHYS_OFFSET)
				ret += SUNXI_4G_PHYS_OFFSET;
		}
	}
	mutex_unlock(&sunxi_domain->dt_lock);

	return ret;
}

/* ================================================================== */
/* Domain alloc / free                                                 */
/* ================================================================== */
static struct iommu_domain *sunxi_iommu_domain_alloc_paging(struct device *dev)
{
	struct sunxi_iommu_domain *sunxi_domain;

	/* Only a single domain is used */
	if (global_sunxi_iommu_domain)
		return &global_sunxi_iommu_domain->domain;

	sunxi_domain = kzalloc(sizeof(*sunxi_domain), GFP_KERNEL);
	if (!sunxi_domain)
		return NULL;

	sunxi_domain->pgtable = (unsigned int *)__get_free_pages(
				GFP_KERNEL, get_order(PD_SIZE));
	if (!sunxi_domain->pgtable)
		goto err_page;

	sunxi_domain->sg_buffer = (unsigned int *)__get_free_pages(
				GFP_KERNEL, get_order(MAX_SG_TABLE_SIZE));
	if (!sunxi_domain->sg_buffer)
		goto err_sg_buffer;

	memset(sunxi_domain->pgtable, 0, PD_SIZE);
	sunxi_domain->domain.pgsize_bitmap = SZ_4K | SZ_16K | SZ_64K |
			SZ_256K | SZ_1M | SZ_4M | SZ_16M;
	sunxi_domain->domain.geometry.aperture_start = 0;
	sunxi_domain->domain.geometry.aperture_end = (1ULL << 32) - 1;
	sunxi_domain->domain.geometry.force_aperture = true;
	mutex_init(&sunxi_domain->dt_lock);
	global_sunxi_iommu_domain = sunxi_domain;
	global_iommu_domain = &sunxi_domain->domain;

	return &sunxi_domain->domain;

err_sg_buffer:
	free_pages((unsigned long)sunxi_domain->pgtable, get_order(PD_SIZE));
	sunxi_domain->pgtable = NULL;
err_page:
	kfree(sunxi_domain);
	return NULL;
}

static void sunxi_iommu_domain_free(struct iommu_domain *domain)
{
	struct sunxi_iommu_domain *sunxi_domain =
		container_of(domain, struct sunxi_iommu_domain, domain);
	int i;
	size_t dom_iova;
	u32 *dent, *pent;

	mutex_lock(&sunxi_domain->dt_lock);
	for (i = 0; i < NUM_ENTRIES_PDE; ++i) {
		dent = sunxi_domain->pgtable + i;
		dom_iova = (size_t)i << IOMMU_PD_SHIFT;
		if (IS_VALID(*dent)) {
			u32 *pent_page = iopte_page(dent);

			pent = iopte_offset(dent, dom_iova);
			dma_sync_single_for_cpu(dma_dev, virt_to_phys(pent),
						PT_SIZE, DMA_TO_DEVICE);
			memset(pent, 0, PT_SIZE);
			dma_sync_single_for_device(dma_dev, virt_to_phys(pent),
						   PT_SIZE, DMA_TO_DEVICE);
			dma_sync_single_for_cpu(dma_dev, virt_to_phys(dent),
						sizeof(*dent), DMA_TO_DEVICE);
			*dent = 0;
			dma_sync_single_for_device(dma_dev, virt_to_phys(dent),
						   sizeof(*dent), DMA_TO_DEVICE);
			sunxi_free_iopte(pent_page);
		}
	}
	sunxi_tlb_flush(global_iommu_dev);
	mutex_unlock(&sunxi_domain->dt_lock);

	free_pages((unsigned long)sunxi_domain->pgtable, get_order(PD_SIZE));
	sunxi_domain->pgtable = NULL;
	free_pages((unsigned long)sunxi_domain->sg_buffer,
		   get_order(MAX_SG_TABLE_SIZE));
	sunxi_domain->sg_buffer = NULL;
	kfree(sunxi_domain);
}

/* ================================================================== */
/* Attach / probe / release                                            */
/* ================================================================== */
static int sunxi_iommu_attach_dev(struct iommu_domain *domain,
				  struct device *dev)
{
	struct sunxi_iommu_owner *owner = dev_iommu_priv_get(dev);
	int ret = 0;

	if (!owner)
		return -ENODEV;

	if (!tlb_init_flag) {
		global_iommu_owner = owner;
		ret = sunxi_tlb_init(owner, domain);
		if (ret)
			return ret;
		tlb_init_flag = true;
	}

	sunxi_enable_device_iommu(owner->tlbid, owner->flag);

	return ret;
}

static struct iommu_device *sunxi_iommu_probe_device(struct device *dev)
{
	struct sunxi_iommu_owner *owner = dev_iommu_priv_get(dev);

	if (!owner || !owner->data)
		return ERR_PTR(-ENODEV);

	return &owner->data->iommu;
}

static void sunxi_iommu_release_device(struct device *dev)
{
	struct sunxi_iommu_owner *owner = dev_iommu_priv_get(dev);

	if (!owner || !owner->data)
		return;

	sunxi_enable_device_iommu(owner->tlbid, false);
	kfree(owner);
	dev_iommu_priv_set(dev, NULL);
}

static int sunxi_iommu_of_xlate(struct device *dev,
				const struct of_phandle_args *args)
{
	struct platform_device *sysmmu = of_find_device_by_node(args->np);
	struct sunxi_iommu_owner *owner;
	struct sunxi_iommu_dev *data;

	if (!sysmmu)
		return -ENODEV;

	data = platform_get_drvdata(sysmmu);
	if (!data)
		return -ENODEV;

	owner = kzalloc(sizeof(*owner), GFP_KERNEL);
	if (!owner)
		return -ENOMEM;

	owner->tlbid = args->args_count > 0 ? args->args[0] : 0;
	owner->flag = args->args_count > 1 ? args->args[1] : 0;
	owner->data = data;
	owner->dev = dev;

	dev_iommu_priv_set(dev, owner);

	return iommu_fwspec_add_ids(dev, &owner->tlbid, 1);
}

/* ================================================================== */
/* Debug mode (V2+ only)                                               */
/* ================================================================== */
void sunxi_set_debug_mode(void)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;

	if (iommu && iommu->quirks->num_micro_tlbs == 7)
		sunxi_iommu_write(iommu, IOMMU_VA_CONFIG_REG, 0x80000000);
}

void sunxi_set_prefetch_mode(void)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;

	if (iommu && iommu->quirks->num_micro_tlbs == 7)
		sunxi_iommu_write(iommu, IOMMU_VA_CONFIG_REG, 0x00000000);
}

/* ================================================================== */
/* IOVA debug read / write                                             */
/* ================================================================== */
int sunxi_iova_test_write(dma_addr_t iova, u32 val)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	int retval;

	sunxi_iommu_write(iommu, IOMMU_VA_REG, iova);
	sunxi_iommu_write(iommu, IOMMU_VA_DATA_REG, val);
	sunxi_iommu_write(iommu, IOMMU_VA_CONFIG_REG, 0x80000100);
	sunxi_iommu_write(iommu, IOMMU_VA_CONFIG_REG, 0x80000101);
	retval = sunxi_wait_when(
		sunxi_iommu_read(iommu, IOMMU_VA_CONFIG_REG) & 0x1, 1);
	if (retval)
		dev_err(iommu->dev, "write VA address request timed out\n");

	return retval;
}

unsigned long sunxi_iova_test_read(dma_addr_t iova)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	unsigned long retval;

	sunxi_iommu_write(iommu, IOMMU_VA_REG, iova);
	sunxi_iommu_write(iommu, IOMMU_VA_CONFIG_REG, 0x80000000);
	sunxi_iommu_write(iommu, IOMMU_VA_CONFIG_REG, 0x80000001);
	retval = sunxi_wait_when(
		sunxi_iommu_read(iommu, IOMMU_VA_CONFIG_REG) & 0x1, 1);
	if (retval) {
		dev_err(iommu->dev, "read VA address request timed out\n");
		return 0;
	}

	return sunxi_iommu_read(iommu, IOMMU_VA_DATA_REG);
}

/* ================================================================== */
/* IRQ handler                                                         */
/* ================================================================== */
static int sunxi_iova_invalid_helper(unsigned long iova)
{
	struct sunxi_iommu_domain *sunxi_domain = global_sunxi_iommu_domain;
	u32 *pte_addr, *dte_addr;

	dte_addr = iopde_offset(sunxi_domain->pgtable, iova);
	if ((*dte_addr & 0x3) != 0x1) {
		pr_err("0x%lx is not mapped!\n", iova);
		return 1;
	}
	pte_addr = iopte_offset(dte_addr, iova);
	if ((*pte_addr & 0x2) == 0) {
		pr_err("0x%lx is not mapped!\n", iova);
		return 1;
	}
	pr_err("0x%lx is mapped!\n", iova);
	return 0;
}

/*
 * Recover TLB/PTW state after an IOMMU fault.
 * Called from IRQ context with iommu_lock held.
 */
static void sunxi_iommu_irq_recover(struct sunxi_iommu_dev *iommu,
				     u32 fault_addr)
{
	const struct sunxi_iommu_quirks *q = iommu->quirks;

	if (q->tlb_ivld_range) {
		/* Range-based TLB invalidation */
		sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_START_ADDR_REG,
				  fault_addr);
		sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_END_ADDR_REG,
				  fault_addr + 4 * SPAGE_SIZE);
		sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_ENABLE_REG, 0x1);
		while (sunxi_iommu_read(iommu,
					IOMMU_TLB_IVLD_ENABLE_REG) & 0x1)
			;

		if (q->ptw_ivld_range) {
			/* V4: range-based PTW */
			sunxi_iommu_write(iommu,
					  IOMMU_PC_IVLD_START_ADDR_REG,
					  fault_addr);
			sunxi_iommu_write(iommu,
					  IOMMU_PC_IVLD_END_ADDR_REG,
					  fault_addr + 2 * SPD_SIZE);
			sunxi_iommu_write(iommu,
					  IOMMU_PC_IVLD_ENABLE_REG, 0x1);
			while (sunxi_iommu_read(iommu,
						IOMMU_PC_IVLD_ENABLE_REG) & 0x1)
				;
		} else {
			/* V3: single-addr PTW */
			sunxi_iommu_write(iommu, IOMMU_PC_IVLD_ADDR_REG,
					  fault_addr);
			sunxi_iommu_write(iommu, IOMMU_PC_IVLD_ENABLE_REG,
					  0x1);
			while (sunxi_iommu_read(iommu,
						IOMMU_PC_IVLD_ENABLE_REG) & 0x1)
				;
			sunxi_iommu_write(iommu, IOMMU_PC_IVLD_ADDR_REG,
					  fault_addr + 0x200000);
			sunxi_iommu_write(iommu, IOMMU_PC_IVLD_ENABLE_REG,
					  0x1);
			while (sunxi_iommu_read(iommu,
						IOMMU_PC_IVLD_ENABLE_REG) & 0x1)
				;
		}
	} else {
		/* V1/V2: addr+mask TLB */
		sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_ADDR_REG,
				  fault_addr);
		sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_ADDR_MASK_REG,
				  (u32)IOMMU_PT_MASK);
		sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_ENABLE_REG, 0x1);
		while (sunxi_iommu_read(iommu,
					IOMMU_TLB_IVLD_ENABLE_REG) & 0x1)
			;
		sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_ADDR_REG,
				  fault_addr + 0x2000);
		sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_ADDR_MASK_REG,
				  (u32)IOMMU_PT_MASK);
		sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_ENABLE_REG, 0x1);
		while (sunxi_iommu_read(iommu,
					IOMMU_TLB_IVLD_ENABLE_REG) & 0x1)
			;
		sunxi_iommu_write(iommu, IOMMU_PC_IVLD_ADDR_REG,
				  fault_addr);
		sunxi_iommu_write(iommu, IOMMU_PC_IVLD_ENABLE_REG, 0x1);
		while (sunxi_iommu_read(iommu,
					IOMMU_PC_IVLD_ENABLE_REG) & 0x1)
			;
		sunxi_iommu_write(iommu, IOMMU_PC_IVLD_ADDR_REG,
				  fault_addr + 0x200000);
		sunxi_iommu_write(iommu, IOMMU_PC_IVLD_ENABLE_REG, 0x1);
		while (sunxi_iommu_read(iommu,
					IOMMU_PC_IVLD_ENABLE_REG) & 0x1)
			;
	}
}

static irqreturn_t sunxi_iommu_irq(int irq, void *dev_id)
{
	struct sunxi_iommu_dev *iommu = dev_id;
	const struct sunxi_iommu_quirks *q = iommu->quirks;
	u32 inter_status_reg;
	u32 addr_reg = 0;
	u32 int_masterid_bitmap;
	u32 data_reg = 0;
	u32 l1_pgint_reg;
	u32 l2_pgint_reg;
	u32 master_id = 0;
	unsigned long mflag;
	int i;
	bool handled = false;

	spin_lock_irqsave(&iommu->iommu_lock, mflag);

	inter_status_reg = sunxi_iommu_read(iommu, IOMMU_INT_STA_REG) & 0x3ffff;
	l1_pgint_reg = sunxi_iommu_read(iommu, IOMMU_L1PG_INT_REG);
	l2_pgint_reg = sunxi_iommu_read(iommu, IOMMU_L2PG_INT_REG);
	int_masterid_bitmap = inter_status_reg | l1_pgint_reg | l2_pgint_reg;

	/* Check micro-TLB 0..5 */
	for (i = 0; i < 6; i++) {
		if (inter_status_reg & MICRO_TLB_INVALID_INTER_MASK(i)) {
			pr_err("%s Invalid Authority\n",
			       q->master_names[i]);
			addr_reg = sunxi_iommu_read(iommu,
						    IOMMU_INT_ERR_ADDR_REG(i));
			data_reg = sunxi_iommu_read(iommu,
						    IOMMU_INT_ERR_DATA_REG(i));
			handled = true;
			break;
		}
	}

	/* Check micro-TLB 6 (7-TLB variants only) */
	if (!handled && q->num_micro_tlbs == 7 &&
	    (inter_status_reg & MICRO_TLB_INVALID_INTER_MASK(6))) {
		pr_err("%s Invalid Authority\n", q->master_names[6]);
		addr_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_ADDR_UTLB6);
		data_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_DATA_UTLB6);
		handled = true;
	}

	/* L1 page-table error */
	if (!handled &&
	    (inter_status_reg & L1_PAGETABLE_INVALID_INTER_MASK)) {
		if (q->num_micro_tlbs < 7 || !(int_masterid_bitmap & BIT(31)))
			pr_err("L1 PageTable Invalid\n");
		addr_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_ADDR_L1);
		data_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_DATA_L1);
		handled = true;
	}

	/* L2 page-table error */
	if (!handled &&
	    (inter_status_reg & L2_PAGETABLE_INVALID_INTER_MASK)) {
		if (q->num_micro_tlbs < 7 || !(int_masterid_bitmap & BIT(31)))
			pr_err("L2 PageTable Invalid\n");
		addr_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_ADDR_L2);
		data_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_DATA_L2);
		handled = true;
	}

	if (!handled)
		pr_err("sunxi iommu int error!!!\n");

	if (handled && !(int_masterid_bitmap & BIT(31))) {
		if (sunxi_iova_invalid_helper(addr_reg)) {
			int_masterid_bitmap &= 0xffff;
			master_id = __ffs(int_masterid_bitmap);
		}
		if (master_id < q->num_micro_tlbs)
			pr_err("%s invalid address: 0x%x, data:0x%x, id:0x%x\n",
			       q->master_names[master_id],
			       addr_reg, data_reg, int_masterid_bitmap);
		else
			pr_err("unknown master %u invalid address: 0x%x, data:0x%x, id:0x%x\n",
			       master_id, addr_reg, data_reg,
			       int_masterid_bitmap);
	}

	/* Recover TLB / PTW state */
	sunxi_iommu_irq_recover(iommu, addr_reg);

	sunxi_iommu_write(iommu, IOMMU_INT_CLR_REG, inter_status_reg);
	inter_status_reg |= (l1_pgint_reg | l2_pgint_reg);
	inter_status_reg &= 0xffff;
	sunxi_iommu_write(iommu, IOMMU_RESET_REG, ~inter_status_reg);
	sunxi_iommu_write(iommu, IOMMU_RESET_REG, 0xffffffff);

	spin_unlock_irqrestore(&iommu->iommu_lock, mflag);

	return IRQ_HANDLED;
}

/* ================================================================== */
/* Sysfs: PMU enable                                                   */
/* ================================================================== */
static ssize_t sunxi_iommu_enable_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	u32 data;

	spin_lock(&iommu->iommu_lock);
	data = sunxi_iommu_read(iommu, IOMMU_PMU_ENABLE_REG);
	spin_unlock(&iommu->iommu_lock);

	return scnprintf(buf, PAGE_SIZE, "enable = %d\n", data & 0x1 ? 1 : 0);
}

static ssize_t sunxi_iommu_enable_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	unsigned long val;
	u32 data;
	int retval;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	spin_lock(&iommu->iommu_lock);
	if (val) {
		data = sunxi_iommu_read(iommu, IOMMU_PMU_ENABLE_REG);
		sunxi_iommu_write(iommu, IOMMU_PMU_ENABLE_REG, data | 0x1);
		data = sunxi_iommu_read(iommu, IOMMU_PMU_CLR_REG);
		sunxi_iommu_write(iommu, IOMMU_PMU_CLR_REG, data | 0x1);
		retval = sunxi_wait_when(
			sunxi_iommu_read(iommu, IOMMU_PMU_CLR_REG) & 0x1, 1);
		if (retval)
			dev_err(iommu->dev, "Clear PMU Count timed out\n");
	} else {
		data = sunxi_iommu_read(iommu, IOMMU_PMU_CLR_REG);
		sunxi_iommu_write(iommu, IOMMU_PMU_CLR_REG, data | 0x1);
		retval = sunxi_wait_when(
			sunxi_iommu_read(iommu, IOMMU_PMU_CLR_REG) & 0x1, 1);
		if (retval)
			dev_err(iommu->dev, "Clear PMU Count timed out\n");
		data = sunxi_iommu_read(iommu, IOMMU_PMU_ENABLE_REG);
		sunxi_iommu_write(iommu, IOMMU_PMU_ENABLE_REG, data & ~0x1);
	}
	spin_unlock(&iommu->iommu_lock);

	return count;
}

/* ================================================================== */
/* Sysfs: PMU profiling (unified loop over micro-TLBs)                 */
/* ================================================================== */
static ssize_t sunxi_iommu_profilling_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	const struct sunxi_iommu_quirks *q = iommu->quirks;
	ssize_t len = 0;
	u64 access_cnt, hit_cnt, latency;
	int i;

	spin_lock(&iommu->iommu_lock);

	/* Per micro-TLB access / hit counts */
	for (i = 0; i < q->num_micro_tlbs; i++) {
		access_cnt =
			((u64)(sunxi_iommu_read(iommu,
				IOMMU_PMU_ACCESS_HIGH_REG(i)) & 0x7ff) << 32) |
			sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_LOW_REG(i));
		hit_cnt =
			((u64)(sunxi_iommu_read(iommu,
				IOMMU_PMU_HIT_HIGH_REG(i)) & 0x7ff) << 32) |
			sunxi_iommu_read(iommu, IOMMU_PMU_HIT_LOW_REG(i));

		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "%s_access_count = 0x%llx\n"
				 "%s_hit_count = 0x%llx\n",
				 q->master_names[i], access_cnt,
				 q->master_names[i], hit_cnt);
	}

	/* Macro-TLB */
	access_cnt =
		((u64)(sunxi_iommu_read(iommu,
			IOMMU_PMU_ACCESS_HIGH_MACROTLB) & 0x7ff) << 32) |
		sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_LOW_MACROTLB);
	hit_cnt =
		((u64)(sunxi_iommu_read(iommu,
			IOMMU_PMU_HIT_HIGH_MACROTLB) & 0x7ff) << 32) |
		sunxi_iommu_read(iommu, IOMMU_PMU_HIT_LOW_MACROTLB);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "macrotlb_access_count = 0x%llx\n"
			 "macrotlb_hit_count = 0x%llx\n",
			 access_cnt, hit_cnt);

	/* PTW cache */
	access_cnt =
		((u64)(sunxi_iommu_read(iommu,
			IOMMU_PMU_ACCESS_HIGH_PTWCACHE) & 0x7ff) << 32) |
		sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_LOW_PTWCACHE);
	hit_cnt =
		((u64)(sunxi_iommu_read(iommu,
			IOMMU_PMU_HIT_HIGH_PTWCACHE) & 0x7ff) << 32) |
		sunxi_iommu_read(iommu, IOMMU_PMU_HIT_LOW_PTWCACHE);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "ptwcache_access_count = 0x%llx\n"
			 "ptwcache_hit_count = 0x%llx\n",
			 access_cnt, hit_cnt);

	/* Per micro-TLB total latency */
	for (i = 0; i < q->num_micro_tlbs; i++) {
		latency =
			((u64)(sunxi_iommu_read(iommu,
				IOMMU_PMU_TL_HIGH_REG(i)) & 0x3ffff) << 32) |
			sunxi_iommu_read(iommu, IOMMU_PMU_TL_LOW_REG(i));
		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "%s_total_latency = 0x%llx\n",
				 q->master_names[i], latency);
	}

	/* Per micro-TLB max latency (7-TLB variants only) */
	if (q->num_micro_tlbs == 7) {
		for (i = 0; i < q->num_micro_tlbs; i++) {
			u32 ml = sunxi_iommu_read(iommu,
						  IOMMU_PMU_ML_REG(i));

			len += scnprintf(buf + len, PAGE_SIZE - len,
					 "%s_max_latency = 0x%x\n",
					 q->master_names[i], ml);
		}
	}

	spin_unlock(&iommu->iommu_lock);

	return len;
}

/* ================================================================== */
/* Sysfs: page-table dump                                              */
/* ================================================================== */
static inline bool __region_ended(u32 pent)
{
	return !(pent & SUNXI_PTE_PAGE_VALID);
}

static inline bool __access_mask_changed(u32 pent, u32 old_mask)
{
	return old_mask != (pent & (SUNXI_PTE_PAGE_READABLE |
				    SUNXI_PTE_PAGE_WRITABLE));
}

#define DUMP_REGION_MAP		0
#define DUMP_REGION_RESERVE	1

struct dump_region {
	u32 access_mask;
	size_t size;
	u32 type;
	dma_addr_t phys, iova;
};

static inline u32 __print_region(char *buf, size_t buf_len, ssize_t len,
				 struct dump_region *r)
{
	if (r->type == DUMP_REGION_RESERVE)
		len += scnprintf(buf + len, buf_len - len,
			"iova:%pad                            size:0x%zx\n",
			&r->iova, r->size);
	else
		len += scnprintf(buf + len, buf_len - len,
			"iova:%pad phys:%pad %s%s size:0x%zx\n",
			&r->iova, &r->phys,
			r->access_mask & SUNXI_PTE_PAGE_READABLE ? "R" : " ",
			r->access_mask & SUNXI_PTE_PAGE_WRITABLE ? "W" : " ",
			r->size);
	return len;
}

ssize_t sunxi_iommu_dump_pgtable(char *buf, size_t buf_len)
{
	struct sunxi_iommu_domain *sunxi_domain = container_of(
		global_iommu_domain, struct sunxi_iommu_domain, domain);
	ssize_t len = 0;
	int i, j;
	u32 *dent, *pent;
	struct dump_region active_region;
	struct iommu_resv_region *resv;

	len += scnprintf(buf + len, buf_len - len, "reserved\n");
	list_for_each_entry(resv, &global_iommu_dev->rsv_list, list) {
		active_region.access_mask = 0;
		active_region.iova = resv->start;
		active_region.type = DUMP_REGION_RESERVE;
		active_region.size = resv->length;
		len = __print_region(buf, buf_len, len, &active_region);
	}
	len += scnprintf(buf + len, buf_len - len, "mapped\n");

	dent = sunxi_domain->pgtable;
	active_region.type = DUMP_REGION_MAP;
	active_region.size = 0;
	active_region.access_mask = 0;
	for (i = 0; i < NUM_ENTRIES_PDE; i++) {
		if (!IS_VALID(dent[i]))
			continue;
		pent = iopte_offset(dent + i, 0);
		for (j = 0; j < NUM_ENTRIES_PTE; j++) {
			if (active_region.size) {
				if (__region_ended(pent[j]) ||
				    (active_region.access_mask &&
				     __access_mask_changed(pent[j],
					active_region.access_mask))) {
					len = __print_region(buf, buf_len, len,
							     &active_region);
					active_region.size = 0;
					active_region.access_mask = 0;
				}
			}
			if (pent[j] & SUNXI_PTE_PAGE_VALID) {
				if (active_region.size == 0) {
					active_region.iova =
						((dma_addr_t)i << IOMMU_PD_SHIFT) +
						((dma_addr_t)j << IOMMU_PT_SHIFT);
					active_region.phys =
						iommu_phy_to_cpu_phy(
							IOPTE_TO_PFN(&pent[j]));
					active_region.access_mask =
						pent[j] & (SUNXI_PTE_PAGE_READABLE |
							   SUNXI_PTE_PAGE_WRITABLE);
				}
				active_region.size += 1 << IOMMU_PT_SHIFT;
			}
		}
	}
	if (active_region.size)
		len = __print_region(buf, buf_len, len, &active_region);

	return len;
}
EXPORT_SYMBOL_GPL(sunxi_iommu_dump_pgtable);

static ssize_t sunxi_iommu_map_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return sunxi_iommu_dump_pgtable(buf, PAGE_SIZE);
}

static struct device_attribute sunxi_iommu_enable_attr =
	__ATTR(enable, 0644, sunxi_iommu_enable_show, sunxi_iommu_enable_store);
static struct device_attribute sunxi_iommu_profilling_attr =
	__ATTR(profilling, 0444, sunxi_iommu_profilling_show, NULL);
static struct device_attribute sunxi_iommu_map_attr =
	__ATTR(page_debug, 0444, sunxi_iommu_map_show, NULL);

static void sunxi_iommu_sysfs_create(struct platform_device *pdev)
{
	device_create_file(&pdev->dev, &sunxi_iommu_enable_attr);
	device_create_file(&pdev->dev, &sunxi_iommu_profilling_attr);
	device_create_file(&pdev->dev, &sunxi_iommu_map_attr);
}

static void sunxi_iommu_sysfs_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &sunxi_iommu_enable_attr);
	device_remove_file(&pdev->dev, &sunxi_iommu_profilling_attr);
	device_remove_file(&pdev->dev, &sunxi_iommu_map_attr);
}

/* ================================================================== */
/* Reserved-memory regions                                             */
/* ================================================================== */
static int sunxi_iommu_check_cmd(struct device *dev, void *data)
{
	struct iommu_resv_region *region;
	int prot = IOMMU_WRITE | IOMMU_READ;
	struct list_head *rsv_list = data;
	struct {
		const char *name;
		u32 region_type;
	} supported_region[] = {
		{ "sunxi-iova-reserve", IOMMU_RESV_RESERVED },
		{ "sunxi-iova-premap",  IOMMU_RESV_DIRECT },
	};
	int i, j;
#define REGION_CNT_MAX 8
	struct {
		u64 array[REGION_CNT_MAX * 2];
		int count;
	} *tmp_data;

	tmp_data = kzalloc(sizeof(*tmp_data), GFP_KERNEL);
	if (!tmp_data)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(supported_region); i++) {
		if (!of_find_property(dev->of_node, supported_region[i].name,
				      NULL))
			continue;

		tmp_data->count = of_property_read_variable_u64_array(
			dev->of_node, supported_region[i].name, tmp_data->array,
			0, REGION_CNT_MAX);
		if (tmp_data->count <= 0 || (tmp_data->count & 1) != 0) {
			if (tmp_data->count > 0)
				dev_err(dev, "array %s size %d should be even\n",
					supported_region[i].name,
					tmp_data->count);
			continue;
		}
		tmp_data->count /= 2;

		for (j = 0; j < tmp_data->count; j++) {
			region = iommu_alloc_resv_region(
				tmp_data->array[j * 2],
				tmp_data->array[j * 2 + 1], prot,
				supported_region[i].region_type, GFP_KERNEL);
			if (!region) {
				dev_err(dev, "no memory for iova rsv region\n");
				continue;
			}
			list_add_tail(&region->list, rsv_list);
		}
	}
	kfree(tmp_data);
#undef REGION_CNT_MAX

	return 0;
}

static int __init_reserve_mem(struct sunxi_iommu_dev *dev)
{
	return bus_for_each_dev(&platform_bus_type, NULL, &dev->rsv_list,
				sunxi_iommu_check_cmd);
}

static void sunxi_iommu_get_resv_regions(struct device *dev,
					  struct list_head *head)
{
	struct iommu_resv_region *entry;
	struct iommu_resv_region *region;

	if (!head || !global_iommu_dev)
		return;

	if (list_empty(&global_iommu_dev->rsv_list))
		__init_reserve_mem(global_iommu_dev);

	if (list_empty(&global_iommu_dev->rsv_list))
		return;

	list_for_each_entry(entry, &global_iommu_dev->rsv_list, list) {
		region = iommu_alloc_resv_region(entry->start, entry->length,
						 entry->prot, entry->type,
						 GFP_KERNEL);
		if (region)
			list_add_tail(&region->list, head);
	}
}

/* ================================================================== */
/* IOMMU ops                                                           */
/* ================================================================== */
static const struct iommu_ops sunxi_iommu_ops = {
	.domain_alloc_paging = sunxi_iommu_domain_alloc_paging,
	.probe_device	= sunxi_iommu_probe_device,
	.release_device = sunxi_iommu_release_device,
	.get_resv_regions = sunxi_iommu_get_resv_regions,
	.of_xlate	= sunxi_iommu_of_xlate,
	.default_domain_ops = &(const struct iommu_domain_ops) {
		.attach_dev	= sunxi_iommu_attach_dev,
		.map_pages	= sunxi_iommu_map,
		.unmap_pages	= sunxi_iommu_unmap,
		.iotlb_sync	= sunxi_iommu_iotlb_sync,
		.iova_to_phys	= sunxi_iommu_iova_to_phys,
		.free		= sunxi_iommu_domain_free,
	},
	.device_group	= generic_single_device_group,
	.owner		= THIS_MODULE,
};

/* ================================================================== */
/* Platform driver probe / remove / suspend / resume                   */
/* ================================================================== */
static int sunxi_iommu_probe(struct platform_device *pdev)
{
	int ret, irq;
	struct device *dev = &pdev->dev;
	struct sunxi_iommu_dev *sunxi_iommu;
	struct resource *res;
	const struct sunxi_iommu_quirks *quirks;

	quirks = of_device_get_match_data(dev);
	if (!quirks) {
		dev_err(dev, "No matching quirks found\n");
		return -ENODEV;
	}

	iopte_cache = kmem_cache_create("sunxi-iopte-cache", PT_SIZE,
					PT_SIZE, SLAB_HWCACHE_ALIGN, NULL);
	if (!iopte_cache) {
		dev_err(dev, "Failed to create sunxi-iopte-cache.\n");
		return -ENOMEM;
	}

	sunxi_iommu = devm_kzalloc(dev, sizeof(*sunxi_iommu), GFP_KERNEL);
	if (!sunxi_iommu)
		return -ENOMEM;

	sunxi_iommu->quirks = quirks;

	/* Check for the sun50iw10 "fixed" IC revision DT property */
	sunxi_iommu->prefetch_fixed =
		of_property_read_bool(dev->of_node,
				      "allwinner,sun50iw10-iommu-fixed");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_dbg(dev, "Unable to find resource region\n");
		ret = -ENOENT;
		goto err_res;
	}

	sunxi_iommu->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(sunxi_iommu->base)) {
		ret = PTR_ERR(sunxi_iommu->base);
		goto err_res;
	}

	sunxi_iommu->bypass = GENMASK(quirks->num_micro_tlbs - 1, 0);

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_dbg(dev, "Unable to find IRQ resource\n");
		ret = -ENOENT;
		goto err_irq;
	}

	dev_info(dev, "sunxi iommu: irq = %d, %u micro-TLBs\n",
		 irq, quirks->num_micro_tlbs);

	ret = devm_request_irq(dev, irq, sunxi_iommu_irq, 0,
			       dev_name(dev), sunxi_iommu);
	if (ret < 0) {
		dev_dbg(dev, "Unable to register interrupt handler\n");
		goto err_irq;
	}

	sunxi_iommu->irq = irq;

	sunxi_iommu->clk = of_clk_get_by_name(dev->of_node, "iommu");
	if (IS_ERR(sunxi_iommu->clk)) {
		sunxi_iommu->clk = NULL;
		dev_dbg(dev, "Unable to find clock\n");
		ret = -ENOENT;
		goto err_clk;
	}
	clk_prepare_enable(sunxi_iommu->clk);

	platform_set_drvdata(pdev, sunxi_iommu);
	sunxi_iommu->dev = dev;
	spin_lock_init(&sunxi_iommu->iommu_lock);
	INIT_LIST_HEAD(&sunxi_iommu->rsv_list);
	global_iommu_dev = sunxi_iommu;

	if (dev->parent)
		pm_runtime_enable(dev);

	sunxi_iommu_sysfs_create(pdev);

	ret = iommu_device_sysfs_add(&sunxi_iommu->iommu, dev, NULL,
				     dev_name(dev));
	if (ret) {
		dev_err(dev, "Failed to register iommu in sysfs\n");
		return ret;
	}

	ret = iommu_device_register(&sunxi_iommu->iommu, &sunxi_iommu_ops,
				    dev);
	if (ret) {
		dev_err(dev, "Failed to register iommu\n");
		goto err_remove_sysfs;
	}

	if (!dma_dev)
		dma_dev = dev;

	return 0;

err_remove_sysfs:
	iommu_device_sysfs_remove(&sunxi_iommu->iommu);
err_clk:
	free_irq(irq, sunxi_iommu);
err_irq:
	devm_iounmap(dev, sunxi_iommu->base);
err_res:
	kmem_cache_destroy(iopte_cache);
	dev_err(dev, "Failed to initialize\n");

	return ret;
}

static void sunxi_iommu_remove(struct platform_device *pdev)
{
	struct sunxi_iommu_dev *sunxi_iommu = platform_get_drvdata(pdev);
	struct iommu_resv_region *entry, *next;

	kmem_cache_destroy(iopte_cache);
	if (!list_empty(&sunxi_iommu->rsv_list)) {
		list_for_each_entry_safe(entry, next,
					 &sunxi_iommu->rsv_list, list)
			kfree(entry);
	}
	free_irq(sunxi_iommu->irq, sunxi_iommu);
	devm_iounmap(sunxi_iommu->dev, sunxi_iommu->base);
	sunxi_iommu_sysfs_remove(pdev);
	iommu_device_sysfs_remove(&sunxi_iommu->iommu);
	iommu_device_unregister(&sunxi_iommu->iommu);
	global_iommu_dev = NULL;
}

static int sunxi_iommu_suspend(struct device *dev)
{
	clk_disable_unprepare(global_iommu_dev->clk);
	return 0;
}

static int sunxi_iommu_resume(struct device *dev)
{
	int err;

	clk_prepare_enable(global_iommu_dev->clk);

	if (unlikely(!global_sunxi_iommu_domain))
		return 0;

	err = sunxi_tlb_init(global_iommu_owner,
			     &global_sunxi_iommu_domain->domain);
	return err;
}

static const struct dev_pm_ops sunxi_iommu_pm_ops = {
	.suspend	= sunxi_iommu_suspend,
	.resume		= sunxi_iommu_resume,
};

/* ================================================================== */
/* OF match table                                                      */
/* ================================================================== */
static const struct of_device_id sunxi_iommu_dt[] = {
	/* V1: 6 micro-TLBs, addr+mask invalidation (sun50iw3, sun50iw6) */
	{ .compatible = "allwinner,sun50i-a63-iommu",
	  .data = &sun50i_a63_iommu_quirks },

	/* V2: 7 micro-TLBs, addr+mask invalidation (sun8iw15) */
	{ .compatible = "allwinner,sun8i-a50-iommu",
	  .data = &sun8i_a50_iommu_quirks },

	/* V3: 7 micro-TLBs, range TLB + single PTW (sun50iw9, sun8iw19, sun50iw10) */
	{ .compatible = "allwinner,sun50i-h616-iommu",
	  .data = &sun50i_h616_iommu_quirks },

	/* V4: 7 micro-TLBs, range TLB + range PTW (sun8iw20, sun20iw1, sun50iw12, sun55iw3) */
	{ .compatible = "allwinner,sun20i-d1-iommu",
	  .data = &sun20i_d1_iommu_quirks },

	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, sunxi_iommu_dt);

static struct platform_driver sunxi_iommu_driver = {
	.probe		= sunxi_iommu_probe,
	.remove		= sunxi_iommu_remove,
	.driver		= {
		.owner		= THIS_MODULE,
		.name		= "sunxi-iommu",
		.pm		= &sunxi_iommu_pm_ops,
		.of_match_table	= sunxi_iommu_dt,
	},
};

static int __init sunxi_iommu_init(void)
{
	return platform_driver_register(&sunxi_iommu_driver);
}

static void __exit sunxi_iommu_exit(void)
{
	platform_driver_unregister(&sunxi_iommu_driver);
}

subsys_initcall(sunxi_iommu_init);
module_exit(sunxi_iommu_exit);

MODULE_LICENSE("GPL v2");
