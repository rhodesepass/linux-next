/*******************************************************************************
 * Copyright (C) 2016-2018, Allwinner Technology CO., LTD.
 * Author: zhuxianbin <zhuxianbin@allwinnertech.com>
 *
 * This file is provided under a dual BSD/GPL license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 ******************************************************************************/
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
#include <linux/of_fdt.h>
#include <linux/libfdt.h>

#include "sunxi-iommu.h"

#define _max(x, y) (((u64)(x) > (u64)(y)) ? (x) : (y))

static const char *const master[] = {"VE", "CSI", "DE", "G2D", "DI",
				     "DEBUG_MODE", "RESERVED"};


static struct kmem_cache *iopte_cache;
static struct sunxi_iommu_dev *global_iommu_dev;
static struct sunxi_iommu_domain *global_sunxi_iommu_domain;
struct iommu_domain *global_iommu_domain;
struct sunxi_iommu_owner *global_iommu_owner;
static bool tlb_init_flag;
static struct device *dma_dev;
unsigned int sunxi_iommu_version;
unsigned int sun50iw10_ic_version;
EXPORT_SYMBOL_GPL(global_iommu_domain);

#define IOMMU_VERSION_V14 0x14

static inline u32 *iopde_offset(u32 *iopd, unsigned int iova)
{
	return iopd + IOPDE_INDEX(iova);
}

static inline u32 *iopte_offset(u32 *ent, unsigned int iova)
{
	unsigned long iopte_base = 0;

	if (IOPTE_BASE(*ent) < SUNXI_PHYS_OFFSET)
		iopte_base = IOPTE_BASE(*ent) + SUNXI_4G_PHYS_OFFSET;
	else
		iopte_base = IOPTE_BASE(*ent);

	return (u32 *)__va(iopte_base) + IOPTE_INDEX(iova);
}

static inline u32 *iopte_page(u32 *ent)
{
	unsigned long iopte_base = IOPTE_BASE(*ent);

	if (iopte_base < SUNXI_PHYS_OFFSET)
		iopte_base += SUNXI_4G_PHYS_OFFSET;

	return (u32 *)__va(iopte_base);
}

/*
 * cpu phy 0x0000 0000 ~ 0x4000 0000 is reserved for IO access,
 * iommu phy in between 0x0000 0000 ~ 0x4000 0000 should not used
 * as cpu phy directly, move this address space beyond iommu
 * phy max, so iommu phys 0x0000 0000 ~ 0x4000 0000 shoule be
 * iommu_phy_max + 0x0000 0000 ~ iommu_phy_max + 0x4000 0000(as
 * spec said)
 */

static inline dma_addr_t iommu_phy_to_cpu_phy(dma_addr_t iommu_phy)
{
	return iommu_phy < SUNXI_PHYS_OFFSET ?
		       iommu_phy + SUNXI_4G_PHYS_OFFSET :
		       iommu_phy;
}

static inline dma_addr_t cpu_phy_to_iommu_phy(dma_addr_t cpu_phy)
{
	return cpu_phy > SUNXI_4G_PHYS_OFFSET ?
		       cpu_phy - SUNXI_4G_PHYS_OFFSET :
		       cpu_phy;
}


static inline u32 sunxi_iommu_read(struct sunxi_iommu_dev *iommu, u32 offset)
{
	return readl(iommu->base + offset);
}

static inline void sunxi_iommu_write(struct sunxi_iommu_dev *iommu,
						u32 offset, u32 value)
{
	writel(value, iommu->base + offset);
}

void sunxi_reset_device_iommu(unsigned int master_id)
{
	unsigned int regval;
	struct sunxi_iommu_dev *iommu = global_iommu_dev;

	regval = sunxi_iommu_read(iommu, IOMMU_RESET_REG);
	sunxi_iommu_write(iommu, IOMMU_RESET_REG, regval & (~(1 << master_id)));
	regval = sunxi_iommu_read(iommu, IOMMU_RESET_REG);
	if (!(regval & ((1 << master_id)))) {
		sunxi_iommu_write(iommu, IOMMU_RESET_REG, regval | ((1 << master_id)));
	}
}
EXPORT_SYMBOL(sunxi_reset_device_iommu);

void sunxi_enable_device_iommu(unsigned int master_id, bool flag)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	unsigned long mflag;

	spin_lock_irqsave(&iommu->iommu_lock, mflag);
	if (flag)
		iommu->bypass &= ~(master_id_bitmap[master_id]);
	else
		iommu->bypass |= master_id_bitmap[master_id];
	sunxi_iommu_write(iommu, IOMMU_BYPASS_REG, iommu->bypass);
	spin_unlock_irqrestore(&iommu->iommu_lock, mflag);
}
EXPORT_SYMBOL(sunxi_enable_device_iommu);

static int sunxi_tlb_flush(struct sunxi_iommu_dev *iommu)
{
	int ret;

#if IS_ENABLED(CONFIG_ARCH_SUN8IW20) || IS_ENABLED(CONFIG_ARCH_SUN20IW1)
	sunxi_iommu_write(iommu, IOMMU_TLB_FLUSH_ENABLE_REG, 0x0003001f);
#elif IS_ENABLED(CONFIG_ARCH_SUN50IW3) || IS_ENABLED(CONFIG_ARCH_SUN50IW6)
	sunxi_iommu_write(iommu, IOMMU_TLB_FLUSH_ENABLE_REG, 0x0003003f);
#else
	sunxi_iommu_write(iommu, IOMMU_TLB_FLUSH_ENABLE_REG, 0x0003007f);
#endif
	ret = sunxi_wait_when(
		(sunxi_iommu_read(iommu, IOMMU_TLB_FLUSH_ENABLE_REG)), 2);
	if (ret)
		dev_err(iommu->dev, "Enable flush all request timed out\n");
	return ret;
}

static bool __maybe_unused is_sun50iw10_ic_new(void)
{
	if (sun50iw10_ic_version == 0 || sun50iw10_ic_version == 3 ||
		sun50iw10_ic_version == 4)
		return false;
	else
		return true;

}

static int
sunxi_tlb_init(struct sunxi_iommu_owner *owner,
				struct iommu_domain *input_domain)
{
	int ret = 0;
	int iommu_enable = 0;
	phys_addr_t dte_addr;
	unsigned long mflag;
	struct sunxi_iommu_dev *iommu = owner->data;
	struct sunxi_iommu_domain *sunxi_domain =
		container_of(input_domain, struct sunxi_iommu_domain, domain);

	/* iommu init */
	spin_lock_irqsave(&iommu->iommu_lock, mflag);
	dte_addr = __pa(sunxi_domain->pgtable);
	sunxi_iommu_write(iommu, IOMMU_TTB_REG, dte_addr);
	sunxi_iommu_version = sunxi_iommu_read(iommu, IOMMU_VERSION_REG);
	/* sun8iw15p1: disable preftech on G2D for better performance */
#if IS_ENABLED(CONFIG_ARCH_SUN8IW15)
	sunxi_iommu_write(iommu, IOMMU_TLB_PREFETCH_REG, 0x5f);
#elif IS_ENABLED(CONFIG_ARCH_SUN8IW19) || IS_ENABLED(CONFIG_ARCH_SUN50IW9)
	sunxi_iommu_write(iommu, IOMMU_TLB_PREFETCH_REG, 0x0);
#elif IS_ENABLED(CONFIG_ARCH_SUN50IW10)
	sun50iw10_ic_version = sunxi_get_soc_ver();
	if (is_sun50iw10_ic_new()) {
		sunxi_iommu_write(iommu, IOMMU_TLB_PREFETCH_REG, 0x5f);
	} else {
		/* disable preftech for tlb invalid mode for unsolved issue for sun50iw10(old)*/
		sunxi_iommu_write(iommu, IOMMU_TLB_PREFETCH_REG, 0x0);
	}
#elif IS_ENABLED(CONFIG_ARCH_SUN8IW20) || IS_ENABLED(CONFIG_ARCH_SUN20IW1)
	/* disable preftech on G2D/VE for better performance */
	sunxi_iommu_write(iommu, IOMMU_TLB_PREFETCH_REG, 0x16);
#else
	sunxi_iommu_write(iommu, IOMMU_TLB_PREFETCH_REG, 0x7f);
#endif
	/* disable interrupt of prefetch */
	sunxi_iommu_write(iommu, IOMMU_INT_ENABLE_REG, 0x3003f);
	sunxi_iommu_write(iommu, IOMMU_BYPASS_REG, iommu->bypass);
#if IS_ENABLED(CONFIG_ARCH_SUN50IW9) || IS_ENABLED(CONFIG_ARCH_SUN8IW19)
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_MODE_SEL_REG, 0x1);
#elif IS_ENABLED(CONFIG_ARCH_SUN50IW10)
	/* use range(iova_start, iova_end) to invalid TLB */
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_MODE_SEL_REG, 0x1);
	if (is_sun50iw10_ic_new()) {
		/* for sun50iw10(new) */
		/* enable prefetch valid fix mode to avoid prefetching extra invalid TLB or PTW */
		ret = sunxi_iommu_read(iommu, IOMMU_TLB_PREFETCH_REG);
		sunxi_iommu_write(iommu, IOMMU_TLB_PREFETCH_REG, ret | 0x30000);
	} else {
		/* for sun50iw10(old) */
		/* disable prefetch valid fix mode*/
		ret = sunxi_iommu_read(iommu, IOMMU_TLB_PREFETCH_REG);
		sunxi_iommu_write(iommu, IOMMU_TLB_PREFETCH_REG, ret & (0xfffcffff));
	}
#elif IS_ENABLED(CONFIG_ARCH_SUN8IW20) || IS_ENABLED(CONFIG_ARCH_SUN50IW12) \
	|| IS_ENABLED(CONFIG_ARCH_SUN20IW1) || IS_ENABLED(CONFIG_ARCH_SUN55IW3)
	/* use range(iova_start, iova_end) to invalid PTW and TLB */
	sunxi_iommu_write(iommu, IOMMU_PC_IVLD_MODE_SEL_REG, 0x1);
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_MODE_SEL_REG, 0x1);
	/* enable prefetch valid fix mode to avoid prefetching extra invalid TLB or PTW */
	ret = sunxi_iommu_read(iommu, IOMMU_TLB_PREFETCH_REG);
	sunxi_iommu_write(iommu, IOMMU_TLB_PREFETCH_REG, ret | 0x30000);
#else
	sunxi_iommu_write(iommu, IOMMU_PC_IVLD_MODE_SEL_REG, 0x0);
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_MODE_SEL_REG, 0x0);
#endif

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
			dev_err(iommu->dev, "iommu enable failed! No iommu in bitfile!\n");
			ret = -ENODEV;
			goto out;
		}
	}
	tlb_init_flag = true;

out:
	spin_unlock_irqrestore(&iommu->iommu_lock, mflag);
	return ret;

}

#if IS_ENABLED(CONFIG_ARCH_SUN50IW9) || IS_ENABLED(CONFIG_ARCH_SUN8IW19) \
	|| IS_ENABLED(CONFIG_ARCH_SUN50IW10) || IS_ENABLED(CONFIG_ARCH_SUN50IW12) \
	|| IS_ENABLED(CONFIG_ARCH_SUN8IW20) || IS_ENABLED(CONFIG_ARCH_SUN20IW1) \
	|| IS_ENABLED(CONFIG_ARCH_SUN55IW3)
static int sunxi_tlb_invalid(dma_addr_t iova_start, dma_addr_t iova_end)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	int ret = 0;
	unsigned long mflag;

	pr_debug("iommu: tlb invalid:0x%x-0x%x\n", (unsigned int)iova_start,
			 (unsigned int)iova_end);
	spin_lock_irqsave(&iommu->iommu_lock, mflag);
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_START_ADDR_REG, iova_start);
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_END_ADDR_REG, iova_end);
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_ENABLE_REG, 0x1);
	ret = sunxi_wait_when(
		(sunxi_iommu_read(iommu, IOMMU_TLB_IVLD_ENABLE_REG)&0x1), 2);
	if (ret) {
		dev_err(iommu->dev, "TLB Invalid timed out\n");
		goto out;
	}

out:
	spin_unlock_irqrestore(&iommu->iommu_lock, mflag);
	return ret;
}
#else
static int sunxi_tlb_invalid(dma_addr_t iova, u32 mask)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	int ret = 0;
	unsigned long mflag;

	spin_lock_irqsave(&iommu->iommu_lock, mflag);
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_ADDR_REG, iova);
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_ADDR_MASK_REG, mask);
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_ENABLE_REG, 0x1);
	ret = sunxi_wait_when(
		(sunxi_iommu_read(iommu, IOMMU_TLB_IVLD_ENABLE_REG)&0x1), 2);
	if (ret) {
		dev_err(iommu->dev, "TLB Invalid timed out\n");
		goto out;
	}

out:
	spin_unlock_irqrestore(&iommu->iommu_lock, mflag);
	return ret;
}
#endif

/* sun8iw20/sun20iw1/sun50iw12/sun50iw10(new) iommu version >= 0x14: use start/end to invalid ptw*/
static int __maybe_unused sunxi_ptw_cache_invalid_new(dma_addr_t iova_start, dma_addr_t iova_end)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	int ret = 0;
	unsigned long mflag;

	pr_debug("iommu: ptw invalid:0x%x-0x%x\n", (unsigned int)iova_start,
			 (unsigned int)iova_end);
	spin_lock_irqsave(&iommu->iommu_lock, mflag);
	sunxi_iommu_write(iommu, IOMMU_PC_IVLD_START_ADDR_REG, iova_start);
	sunxi_iommu_write(iommu, IOMMU_PC_IVLD_END_ADDR_REG, iova_end);
	sunxi_iommu_write(iommu, IOMMU_PC_IVLD_ENABLE_REG, 0x1);
	ret = sunxi_wait_when(
		(sunxi_iommu_read(iommu, IOMMU_PC_IVLD_ENABLE_REG)&0x1), 2);
	if (ret) {
		dev_err(iommu->dev, "PTW cache invalid timed out\n");
		goto out;
	}

out:
	spin_unlock_irqrestore(&iommu->iommu_lock, mflag);
	return ret;
}

/* old platform(including sun50iw10(old)) iommu version < 0x14*/
static int __maybe_unused sunxi_ptw_cache_invalid(dma_addr_t iova)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	int ret = 0;
	unsigned long mflag;

	pr_debug("iommu: ptw invalid:0x%x\n", (unsigned int)iova);
	spin_lock_irqsave(&iommu->iommu_lock, mflag);
	sunxi_iommu_write(iommu, IOMMU_PC_IVLD_ADDR_REG, iova);
	sunxi_iommu_write(iommu, IOMMU_PC_IVLD_ENABLE_REG, 0x1);
	ret = sunxi_wait_when(
		(sunxi_iommu_read(iommu, IOMMU_PC_IVLD_ENABLE_REG)&0x1), 2);
	if (ret) {
		dev_err(iommu->dev, "PTW cache invalid timed out\n");
		goto out;
	}

out:
	spin_unlock_irqrestore(&iommu->iommu_lock, mflag);
	return ret;
}

static int sunxi_alloc_iopte(u32 *sent, int prot)
{
	u32 *pent;
	u32 flags = 0;

	flags |= (prot & IOMMU_READ) ? DENT_READABLE : 0;
	flags |= (prot & IOMMU_WRITE) ? DENT_WRITABLE : 0;

	pent = kmem_cache_zalloc(iopte_cache, GFP_ATOMIC);
	WARN_ON((unsigned long)pent & (PT_SIZE - 1));
	if (!pent) {
		pr_err("%s, %d, malloc failed!\n", __func__, __LINE__);
		return 0;
	}
	dma_sync_single_for_cpu(dma_dev, virt_to_phys(sent), sizeof(*sent), DMA_TO_DEVICE);
	*sent = __pa(pent) | DENT_VALID;
	dma_sync_single_for_device(dma_dev, virt_to_phys(sent), sizeof(*sent), DMA_TO_DEVICE);

	return 1;
}

static void sunxi_free_iopte(u32 *pent)
{
	kmem_cache_free(iopte_cache, pent);
}

#if IS_ENABLED(CONFIG_ARCH_SUN50IW6) || IS_ENABLED(CONFIG_ARCH_SUN50IW3) || IS_ENABLED(CONFIG_ARCH_SUN8IW15)
void sunxi_zap_tlb(unsigned long iova, size_t size)
{
	sunxi_tlb_invalid(iova, (u32)IOMMU_PT_MASK);
	sunxi_tlb_invalid(iova + SPAGE_SIZE, (u32)IOMMU_PT_MASK);
	sunxi_tlb_invalid(iova + size, (u32)IOMMU_PT_MASK);
	sunxi_tlb_invalid(iova + size + SPAGE_SIZE, (u32)IOMMU_PT_MASK);
	sunxi_ptw_cache_invalid(iova);
	sunxi_ptw_cache_invalid(iova + SPD_SIZE);
	sunxi_ptw_cache_invalid(iova + size);
	sunxi_ptw_cache_invalid(iova + size + SPD_SIZE);
}
#elif IS_ENABLED(CONFIG_ARCH_SUN8IW20) || IS_ENABLED(CONFIG_ARCH_SUN50IW12) \
	|| IS_ENABLED(CONFIG_ARCH_SUN20IW1) || IS_ENABLED(CONFIG_ARCH_SUN55IW3)
static inline void sunxi_zap_tlb(unsigned long iova, size_t size)
{
	sunxi_tlb_invalid(iova, iova + 2 * SPAGE_SIZE);
	sunxi_tlb_invalid(iova + size - SPAGE_SIZE, iova + size + 8 * SPAGE_SIZE);
	sunxi_ptw_cache_invalid_new(iova, iova + SPD_SIZE);
	sunxi_ptw_cache_invalid_new(iova + size - SPD_SIZE, iova + size);
}
#else
static inline void sunxi_zap_tlb(unsigned long iova, size_t size)
{
	sunxi_tlb_invalid(iova, iova + 2 * SPAGE_SIZE);
	sunxi_tlb_invalid(iova + size - SPAGE_SIZE, iova + size + 8 * SPAGE_SIZE);
	sunxi_ptw_cache_invalid(iova);
	sunxi_ptw_cache_invalid(iova + size);
	/* new version of iommu (>= 0x14) and sun50iw10(new) will not pretetch any extra invalid TLB or PTW ,
	   so no need to walk around(sunxi_zap_tlb) when unmap, still need it in map */
	if (sunxi_iommu_version >= IOMMU_VERSION_V14 || is_sun50iw10_ic_new())
		return ;

	sunxi_ptw_cache_invalid(iova + SPD_SIZE);
	sunxi_ptw_cache_invalid(iova + size + SPD_SIZE);
	sunxi_ptw_cache_invalid(iova + size + 2 * SPD_SIZE);
}
#endif

static inline u32 sunxi_mk_pte(u32 page, int prot)
{
	u32 flags = 0;

	flags |= (prot & IOMMU_READ) ? SUNXI_PTE_PAGE_READABLE : 0;
	flags |= (prot & IOMMU_WRITE) ? SUNXI_PTE_PAGE_WRITABLE : 0;
	page &= IOMMU_PT_MASK;
	return page | flags | SUNXI_PTE_PAGE_VALID;
}


static int sunxi_iommu_map(struct iommu_domain *domain, unsigned long iova,
	   phys_addr_t paddr, size_t pgsize, size_t pgcount, int prot,
	   gfp_t gfp, size_t *mapped)
{
	struct sunxi_iommu_domain *sunxi_domain =
		container_of(domain, struct sunxi_iommu_domain, domain);
	size_t iova_start, iova_end, paddr_start, s_iova_start;
	size_t size = pgsize * pgcount;
	u32 *dent, *pent;
	/* silence unused warning, the allocator is fixed to DMA32 pages */
	(void)gfp;
	int i, j;
	int flush_count = 0;

	WARN_ON(sunxi_domain->pgtable == NULL);
	iova_start = iova & IOMMU_PT_MASK;
	paddr_start = paddr & IOMMU_PT_MASK;
	iova_end = SPAGE_ALIGN(iova+size) - SPAGE_SIZE;
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
		dma_sync_single_for_device(dma_dev, virt_to_phys(iopte_offset(dent, s_iova_start)),
			flush_count << 2, DMA_TO_DEVICE);
		goto out;
	} else {
		dent = iopde_offset(sunxi_domain->pgtable, iova_start);
		if (!IS_VALID(*dent))
			sunxi_alloc_iopte(dent, prot);
		for (flush_count = 0; iova_start < SPDE_ALIGN(s_iova_start + 1);
			iova_start += SPAGE_SIZE, paddr_start += SPAGE_SIZE) {
			pent = iopte_offset(dent, iova_start);
			WARN_ON(*pent);
			*pent = sunxi_mk_pte(paddr_start, prot);
			++flush_count;
		}
		dma_sync_single_for_device(dma_dev, virt_to_phys(iopte_offset(dent, s_iova_start)),
			flush_count << 2, DMA_TO_DEVICE);
	}
	if (IOPDE_INDEX(iova_start) < IOPDE_INDEX(iova_end)) {
		for (i = IOPDE_INDEX(iova_start); i < IOPDE_INDEX(iova_end);
			i++, iova_start += SPD_SIZE, paddr_start += SPD_SIZE) {
			dent = iopde_offset(sunxi_domain->pgtable, iova_start);
			if (!IS_VALID(*dent))
				sunxi_alloc_iopte(dent, prot);
			pent = iopte_offset(dent, iova_start);
			dma_sync_single_for_cpu(dma_dev, virt_to_phys(pent), PT_SIZE, DMA_TO_DEVICE);
			for (j = 0; j < NUM_ENTRIES_PTE; j++)
				*(pent + j) =
			sunxi_mk_pte(paddr_start + (j * SPAGE_SIZE), prot);
			dma_sync_single_for_device(dma_dev, virt_to_phys(pent), PT_SIZE, DMA_TO_DEVICE);
		}
	}

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
	dma_sync_single_for_device(dma_dev, virt_to_phys(iopte_offset(dent, s_iova_start)),
			flush_count << 2, DMA_TO_DEVICE);
out:
	if (!(prot & (1 << 16)))
		sunxi_zap_tlb(iova, size);
	mutex_unlock(&sunxi_domain->dt_lock);

	if (mapped)
		*mapped = size;

	return 0;
}

#if IS_ENABLED(CONFIG_ARCH_SUN50IW9) || IS_ENABLED(CONFIG_ARCH_SUN8IW19) \
	|| IS_ENABLED(CONFIG_ARCH_SUN50IW10)
static size_t sunxi_iommu_unmap(struct iommu_domain *domain, unsigned long iova,
		 size_t pgsize, size_t pgcount,
		 struct iommu_iotlb_gather *gather)
{
	struct sunxi_iommu_domain *sunxi_domain =
			container_of(domain, struct sunxi_iommu_domain, domain);
	size_t iova_start, iova_end;
	size_t size = pgsize * pgcount;
	size_t s_iova_start;
	u32 *dent, *pent;
	int i;
	int flush_count = 0;

	WARN_ON(sunxi_domain->pgtable == NULL);
	iova_start = iova & IOMMU_PT_MASK;
	iova_end = SPAGE_ALIGN(iova + size) - SPAGE_SIZE;

	gather->start = iova_start;
	gather->end = iova_end;

	mutex_lock(&sunxi_domain->dt_lock);

	/* Invalid TLB */
	sunxi_tlb_invalid(iova_start, iova_end);

	/* Invalid PTW and clear iommu pagetable pde and pte */
	/* 1
	 * if the iova_start and iova_end are between PD_SIZE(1M)
	 * for example the iova is between: 0x100000--0x120000 */
	s_iova_start = iova_start;
	if (IOPDE_INDEX(iova_start) == IOPDE_INDEX(iova_end)) {
		dent = iopde_offset(sunxi_domain->pgtable, iova_start);
		if (IS_VALID(*dent)) {
			u32 *pent_page = iopte_page(dent);
			pent = iopte_offset(dent, s_iova_start);
			for (flush_count = 0; iova_start <= iova_end;
						iova_start += SPAGE_SIZE) {
				pent = iopte_offset(dent, iova_start);
				*pent = 0;
				++flush_count;
			}
			dma_sync_single_for_device(dma_dev, virt_to_phys(iopte_offset(dent, s_iova_start)),
					flush_count << 2, DMA_TO_DEVICE);
			/*invalid ptwcache*/
			sunxi_ptw_cache_invalid(s_iova_start);
			/* Free the PTE page to fix memory leak */
			dma_sync_single_for_cpu(dma_dev, virt_to_phys(dent), sizeof(*dent), DMA_TO_DEVICE);
			*dent = 0;
			dma_sync_single_for_device(dma_dev, virt_to_phys(dent), sizeof(*dent), DMA_TO_DEVICE);
			sunxi_free_iopte(pent_page);
		}
		goto done;
	}
	/* 2
	 * if the iova_start and iova_end are not between PD_SIZE(1M)
	 * for example the iova is between: 0x120000--0x540000
	 */

	/* 2.1. Handle the iova between:0x120000--0x1ff000 */
	dent = iopde_offset(sunxi_domain->pgtable, iova_start);
	if (IS_VALID(*dent)) {
		for (flush_count = 0;
			iova_start < SPDE_ALIGN(s_iova_start + 1);
					iova_start += SPAGE_SIZE) {
			pent = iopte_offset(dent, iova_start);
			*pent = 0;
			++flush_count;
		}
		dma_sync_single_for_device(dma_dev, virt_to_phys(iopte_offset(dent, s_iova_start)),
				flush_count << 2, DMA_TO_DEVICE);
		/*invalid ptwcache*/
		sunxi_ptw_cache_invalid(s_iova_start);
	}

	/* 2.2. Handle the iova between:0x200000--0x500000 */
	if (IOPDE_INDEX(iova_start) < IOPDE_INDEX(iova_end)) {
		for (i = IOPDE_INDEX(iova_start); i < IOPDE_INDEX(iova_end);
			i++, iova_start += SPD_SIZE) {
			dent = iopde_offset(sunxi_domain->pgtable, iova_start);
			if (IS_VALID(*dent)) {
				u32 *pent_page = iopte_page(dent);
				pent = iopte_offset(dent, iova_start);
				dma_sync_single_for_cpu(dma_dev, virt_to_phys(pent), PT_SIZE, DMA_TO_DEVICE);
				memset(pent, 0, PT_SIZE);
				dma_sync_single_for_device(dma_dev, virt_to_phys(pent), PT_SIZE, DMA_TO_DEVICE);
				dma_sync_single_for_cpu(dma_dev, virt_to_phys(dent), sizeof(*dent), DMA_TO_DEVICE);
				*dent = 0;
				dma_sync_single_for_device(dma_dev, virt_to_phys(dent), sizeof(*dent), DMA_TO_DEVICE);
				sunxi_ptw_cache_invalid(iova_start);
				sunxi_free_iopte(pent_page);
			}
		}
	}
	/* 2.3. Handle the iova between:0x500000--0x520000 */
	s_iova_start = iova_start;
	dent = iopde_offset(sunxi_domain->pgtable, iova_start);
	if (IS_VALID(*dent)) {
		for (flush_count = 0; iova_start <= iova_end;
						iova_start += SPAGE_SIZE) {
			pent = iopte_offset(dent, iova_start);
			*pent = 0;
			++flush_count;
		}
		dma_sync_single_for_cpu(dma_dev, virt_to_phys(iopte_offset(dent, s_iova_start)),
				flush_count << 2, DMA_TO_DEVICE);
		sunxi_ptw_cache_invalid(s_iova_start);
	}

done:
	if (sunxi_iommu_version < IOMMU_VERSION_V14 || !is_sun50iw10_ic_new())
		sunxi_zap_tlb(iova, size);
	mutex_unlock(&sunxi_domain->dt_lock);

	return size;
}
#elif IS_ENABLED(CONFIG_ARCH_SUN8IW20) || IS_ENABLED(CONFIG_ARCH_SUN50IW12) \
	|| IS_ENABLED(CONFIG_ARCH_SUN20IW1) || IS_ENABLED(CONFIG_ARCH_SUN55IW3)
static size_t sunxi_iommu_unmap(struct iommu_domain *domain, unsigned long iova,
		 size_t pgsize, size_t pgcount,
		 struct iommu_iotlb_gather *gather)
{
	struct sunxi_iommu_domain *sunxi_domain =
			container_of(domain, struct sunxi_iommu_domain, domain);
	size_t iova_start, iova_end;
	size_t size = pgsize * pgcount;
	size_t s_iova_start;
	u32 *dent, *pent;
	int i;
	int flush_count = 0;

	WARN_ON(sunxi_domain->pgtable == NULL);
	iova_start = iova & IOMMU_PT_MASK;
	iova_end = SPAGE_ALIGN(iova + size) - SPAGE_SIZE;

	gather->start = iova_start;
	gather->end = iova_end;

	mutex_lock(&sunxi_domain->dt_lock);

	/* Invalid TLB and PTW :*/
	sunxi_tlb_invalid(iova_start, iova_end);
	sunxi_ptw_cache_invalid_new(iova_start, iova_end);

	/* Invalid PTW and clear iommu pagetable pde and pte */
	/* 1
	 * if the iova_start and iova_end are between PD_SIZE(1M)
	 * for example the iova is between: 0x100000--0x200000 */
	s_iova_start = iova_start;
	if (IOPDE_INDEX(iova_start) == IOPDE_INDEX(iova_end)) {
		dent = iopde_offset(sunxi_domain->pgtable, iova_start);
		if (IS_VALID(*dent)) {
			u32 *pent_page = iopte_page(dent);
			pent = iopte_offset(dent, s_iova_start);
			for (flush_count = 0; iova_start <= iova_end;
						iova_start += SPAGE_SIZE) {
				pent = iopte_offset(dent, iova_start);
				*pent = 0;
				++flush_count;
			}
			dma_sync_single_for_device(dma_dev, virt_to_phys(iopte_offset(dent, s_iova_start)),
					flush_count << 2, DMA_TO_DEVICE);
			/* Free the PTE page to fix memory leak */
			dma_sync_single_for_cpu(dma_dev, virt_to_phys(dent), sizeof(*dent), DMA_TO_DEVICE);
			*dent = 0;
			dma_sync_single_for_device(dma_dev, virt_to_phys(dent), sizeof(*dent), DMA_TO_DEVICE);
			sunxi_free_iopte(pent_page);
		}
		goto done;
	}
	/* 2
	 * if the iova_start and iova_end are not between PD_SIZE(1M)
	 * for example the iova is between: 0x120000--0x540000
	 */
	/* 2.1. Handle the iova between:0x120000--0x1ff000 */
	dent = iopde_offset(sunxi_domain->pgtable, iova_start);
	if (IS_VALID(*dent)) {
		for (flush_count = 0;
			iova_start < SPDE_ALIGN(s_iova_start + 1);
					iova_start += SPAGE_SIZE) {
			pent = iopte_offset(dent, iova_start);
			*pent = 0;
			++flush_count;
		}
		dma_sync_single_for_device(dma_dev, virt_to_phys(iopte_offset(dent, s_iova_start)),
				flush_count << 2, DMA_TO_DEVICE);
	}
	/* 2.2. Handle the iova between:0x200000--0x500000 */
	if (IOPDE_INDEX(iova_start) < IOPDE_INDEX(iova_end)) {
		for (i = IOPDE_INDEX(iova_start); i < IOPDE_INDEX(iova_end);
			i++, iova_start += SPD_SIZE) {
			dent = iopde_offset(sunxi_domain->pgtable, iova_start);
			if (IS_VALID(*dent)) {
				u32 *pent_page = iopte_page(dent);
				pent = iopte_offset(dent, iova_start);
				dma_sync_single_for_cpu(dma_dev, virt_to_phys(pent), PT_SIZE, DMA_TO_DEVICE);
				memset(pent, 0, PT_SIZE);
				dma_sync_single_for_device(dma_dev, virt_to_phys(pent), PT_SIZE, DMA_TO_DEVICE);
				dma_sync_single_for_cpu(dma_dev, virt_to_phys(dent), sizeof(*dent), DMA_TO_DEVICE);
				*dent = 0;
				dma_sync_single_for_device(dma_dev, virt_to_phys(dent), sizeof(*dent), DMA_TO_DEVICE);
				sunxi_free_iopte(pent_page);
			}
		}
	}
	/* 2.3. Handle the iova between:0x500000--0x520000 */
	s_iova_start = iova_start;
	dent = iopde_offset(sunxi_domain->pgtable, iova_start);
	if (IS_VALID(*dent)) {
		for (flush_count = 0; iova_start <= iova_end;
						iova_start += SPAGE_SIZE) {
			pent = iopte_offset(dent, iova_start);
			*pent = 0;
			++flush_count;
		}
		dma_sync_single_for_cpu(dma_dev, virt_to_phys(iopte_offset(dent, s_iova_start)),
				flush_count << 2, DMA_TO_DEVICE);
	}

done:
	mutex_unlock(&sunxi_domain->dt_lock);

	return size;
}
#else
static size_t sunxi_iommu_unmap(struct iommu_domain *domain, unsigned long iova,
		 size_t pgsize, size_t pgcount,
		 struct iommu_iotlb_gather *gather)
{
	struct sunxi_iommu_domain *sunxi_domain =
			container_of(domain, struct sunxi_iommu_domain, domain);
	size_t iova_start, iova_end, s_iova_start;
	size_t size = pgsize * pgcount;
	u32 *dent, *pent;
	int i;
	int flush_count = 0;

	WARN_ON(sunxi_domain->pgtable == NULL);
	iova_start = iova & IOMMU_PT_MASK;
	iova_end = SPAGE_ALIGN(iova+size) - SPAGE_SIZE;
	s_iova_start = iova_start;

	gather->start = iova_start;
	gather->end = iova_end;

	mutex_lock(&sunxi_domain->dt_lock);

	if (IOPDE_INDEX(iova_start) == IOPDE_INDEX(iova_end)) {
		dent = iopde_offset(sunxi_domain->pgtable, iova_start);
		if (IS_VALID(*dent)) {
			u32 *pent_page = iopte_page(dent);
			pent = iopte_offset(dent, s_iova_start);
			for (flush_count = 0; iova_start <= iova_end;
						iova_start += SPAGE_SIZE) {
				pent = iopte_offset(dent, iova_start);
				*pent = 0;
				sunxi_tlb_invalid(iova_start,
							(u32)IOMMU_PT_MASK);
				++flush_count;
			}
			dma_sync_single_for_device(dma_dev, virt_to_phys(iopte_offset(dent, s_iova_start)),
					flush_count << 2, DMA_TO_DEVICE);
			/*invalid ptwcache*/
			sunxi_ptw_cache_invalid(s_iova_start);
			/* Free PTE page to fix memory leak */
			dma_sync_single_for_cpu(dma_dev, virt_to_phys(dent), sizeof(*dent), DMA_TO_DEVICE);
			*dent = 0;
			dma_sync_single_for_device(dma_dev, virt_to_phys(dent), sizeof(*dent), DMA_TO_DEVICE);
			sunxi_free_iopte(pent_page);
		}
		goto out;
	} else {
		dent = iopde_offset(sunxi_domain->pgtable, iova_start);
		if (IS_VALID(*dent)) {
			for (flush_count = 0;
				iova_start < SPDE_ALIGN(s_iova_start + 1);
						iova_start += SPAGE_SIZE) {
				pent = iopte_offset(dent, iova_start);
				*pent = 0;
				sunxi_tlb_invalid(iova_start,
							(u32)IOMMU_PT_MASK);
				++flush_count;
			}
			dma_sync_single_for_device(dma_dev, virt_to_phys(iopte_offset(dent, s_iova_start)),
					flush_count << 2, DMA_TO_DEVICE);
			/*invalid ptwcache*/
			sunxi_ptw_cache_invalid(s_iova_start);
		}
	}
	if (IOPDE_INDEX(iova_start) < IOPDE_INDEX(iova_end)) {
		for (i = IOPDE_INDEX(iova_start); i < IOPDE_INDEX(iova_end);
			i++, iova_start += SPD_SIZE) {
			dent = iopde_offset(sunxi_domain->pgtable, iova_start);
			if (IS_VALID(*dent)) {
				u32 *pent_page = iopte_page(dent);
				pent = iopte_offset(dent, iova_start);
				dma_sync_single_for_cpu(dma_dev, virt_to_phys(pent), PT_SIZE, DMA_TO_DEVICE);
				memset(pent, 0, PT_SIZE);
				dma_sync_single_for_device(dma_dev, virt_to_phys(pent), PT_SIZE, DMA_TO_DEVICE);
				sunxi_tlb_invalid(iova_start,
						(u32)IOMMU_PD_MASK);
				dma_sync_single_for_cpu(dma_dev, virt_to_phys(dent), sizeof(*dent), DMA_TO_DEVICE);
				*dent = 0;
				dma_sync_single_for_device(dma_dev, virt_to_phys(dent), sizeof(*dent), DMA_TO_DEVICE);
				sunxi_ptw_cache_invalid(iova_start);
				sunxi_free_iopte(pent_page);
			}
		}
	}
	s_iova_start = iova_start;
	dent = iopde_offset(sunxi_domain->pgtable, iova_start);
	if (IS_VALID(*dent)) {
		for (flush_count = 0; iova_start <= iova_end;
						iova_start += SPAGE_SIZE) {
			pent = iopte_offset(dent, iova_start);
			*pent = 0;
			sunxi_tlb_invalid(iova_start, (u32)IOMMU_PT_MASK);
			++flush_count;
		}
		dma_sync_single_for_device(dma_dev, virt_to_phys(iopte_offset(dent, s_iova_start)),
				flush_count << 2, DMA_TO_DEVICE);
		sunxi_ptw_cache_invalid(s_iova_start);
	}
out:
	mutex_unlock(&sunxi_domain->dt_lock);

	return size;
}
#endif

static void sunxi_iommu_iotlb_sync(struct iommu_domain *domain,
			struct iommu_iotlb_gather *gather)
{
	struct sunxi_iommu_domain *sunxi_domain =
		container_of(domain, struct sunxi_iommu_domain, domain);
	unsigned long iova_start = gather->start & IOMMU_PT_MASK;
	unsigned long iova_end = SPAGE_ALIGN(gather->end);

	size_t size = iova_end - iova_start;

	mutex_lock(&sunxi_domain->dt_lock);
	sunxi_zap_tlb(iova_start, size);
	mutex_unlock(&sunxi_domain->dt_lock);

	return;
}

static phys_addr_t
sunxi_iommu_iova_to_phys(struct iommu_domain *domain, dma_addr_t iova)
{
	struct sunxi_iommu_domain *sunxi_domain =
		container_of(domain, struct sunxi_iommu_domain, domain);
	u32 *dent, *pent;
	phys_addr_t ret = 0;


	WARN_ON(sunxi_domain->pgtable == NULL);
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

static struct iommu_domain *sunxi_iommu_domain_alloc_paging(struct device *dev)
{
	struct sunxi_iommu_domain *sunxi_domain;

	(void)dev;

	/* we just use one domain */
	if (global_sunxi_iommu_domain)
		return &global_sunxi_iommu_domain->domain;

	sunxi_domain = kzalloc(sizeof(*sunxi_domain), GFP_KERNEL);

	if (!sunxi_domain)
		return NULL;

	sunxi_domain->pgtable = (unsigned int *)__get_free_pages(
				GFP_KERNEL, get_order(PD_SIZE));
	if (!sunxi_domain->pgtable) {
		pr_err("sunxi domain get pgtable failed\n");
		goto err_page;
	}

	sunxi_domain->sg_buffer = (unsigned int *)__get_free_pages(
				GFP_KERNEL, get_order(MAX_SG_TABLE_SIZE));
	if (!sunxi_domain->sg_buffer) {
		pr_err("sunxi domain get sg_buffer failed\n");
		goto err_sg_buffer;
	}

	memset(sunxi_domain->pgtable, 0, PD_SIZE);
	sunxi_domain->domain.pgsize_bitmap = SZ_4K | SZ_16K | SZ_64K |
			SZ_256K | SZ_1M | SZ_4M | SZ_16M;
	sunxi_domain->domain.geometry.aperture_start = 0;
	sunxi_domain->domain.geometry.aperture_end	 = (1ULL << 32)-1;
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
	int i = 0;
	size_t iova;
	u32 *dent, *pent;

	mutex_lock(&sunxi_domain->dt_lock);
	for (i = 0; i < NUM_ENTRIES_PDE; ++i) {
		dent = sunxi_domain->pgtable + i;
		iova = i << IOMMU_PD_SHIFT;
		if (IS_VALID(*dent)) {
			u32 *pent_page = iopte_page(dent);
			pent = iopte_offset(dent, iova);
			dma_sync_single_for_cpu(dma_dev, virt_to_phys(pent), PT_SIZE, DMA_TO_DEVICE);
			memset(pent, 0, PT_SIZE);
			dma_sync_single_for_device(dma_dev, virt_to_phys(pent), PT_SIZE, DMA_TO_DEVICE);
			dma_sync_single_for_cpu(dma_dev, virt_to_phys(dent), PT_SIZE, DMA_TO_DEVICE);
			*dent = 0;
			dma_sync_single_for_device(dma_dev, virt_to_phys(dent), sizeof(*dent), DMA_TO_DEVICE);
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

static int
sunxi_iommu_attach_dev(struct iommu_domain *domain, struct device *dev)
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

static int
sunxi_iommu_of_xlate(struct device *dev, const struct of_phandle_args *args)
{
	struct platform_device *sysmmu = of_find_device_by_node(args->np);
	struct sunxi_iommu_owner *owner;
	struct sunxi_iommu_dev *data;

	if (!sysmmu)
		return -ENODEV;

	data = platform_get_drvdata(sysmmu);
	if (!data)
		return -ENODEV;

	/* Do not use devm here: of_xlate runs before driver probe, so devres
	 * would remain on the device and trigger "Resources present before probing".
	 */
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

#if IS_ENABLED(CONFIG_ARCH_SUN8IW15) || IS_ENABLED(CONFIG_ARCH_SUN50IW9) \
	|| IS_ENABLED(CONFIG_ARCH_SUN8IW19) || IS_ENABLED(CONFIG_ARCH_SUN50IW10) \
	|| IS_ENABLED(CONFIG_ARCH_SUN8IW20) || IS_ENABLED(CONFIG_ARCH_SUN50IW12) \
	|| IS_ENABLED(CONFIG_ARCH_SUN20IW1) || IS_ENABLED(CONFIG_ARCH_SUN55IW3)
void sunxi_set_debug_mode(void)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;

	sunxi_iommu_write(iommu,
			IOMMU_VA_CONFIG_REG, 0x80000000);
}

void sunxi_set_prefetch_mode(void)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;

	sunxi_iommu_write(iommu,
			IOMMU_VA_CONFIG_REG, 0x00000000);
}

#else
void sunxi_set_debug_mode(void){ return; }
void sunxi_set_prefetch_mode(void){ return; }
#endif

int sunxi_iova_test_write(dma_addr_t iova, u32 val)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	int retval;

	sunxi_iommu_write(iommu, IOMMU_VA_REG, iova);
	sunxi_iommu_write(iommu, IOMMU_VA_DATA_REG, val);
	sunxi_iommu_write(iommu,
			IOMMU_VA_CONFIG_REG, 0x80000100);
	sunxi_iommu_write(iommu,
			IOMMU_VA_CONFIG_REG, 0x80000101);
	retval = sunxi_wait_when((sunxi_iommu_read(iommu,
				IOMMU_VA_CONFIG_REG) & 0x1), 1);
	if (retval)
		dev_err(iommu->dev,
			"write VA address request timed out\n");
	return retval;
}

unsigned long sunxi_iova_test_read(dma_addr_t iova)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	unsigned long retval;

	sunxi_iommu_write(iommu, IOMMU_VA_REG, iova);
	sunxi_iommu_write(iommu,
			IOMMU_VA_CONFIG_REG, 0x80000000);
	sunxi_iommu_write(iommu,
			IOMMU_VA_CONFIG_REG, 0x80000001);
	retval = sunxi_wait_when((sunxi_iommu_read(iommu,
				IOMMU_VA_CONFIG_REG) & 0x1), 1);
	if (retval) {
		dev_err(iommu->dev,
			"read VA address request timed out\n");
		retval = 0;
		goto out;
	}
	retval = sunxi_iommu_read(iommu,
			IOMMU_VA_DATA_REG);
out:
	return retval;
}

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

static irqreturn_t sunxi_iommu_irq(int irq, void *dev_id)
{

	u32 inter_status_reg = 0;
	u32 addr_reg = 0;
	u32	int_masterid_bitmap = 0;
	u32	data_reg = 0;
	u32	l1_pgint_reg = 0;
	u32	l2_pgint_reg = 0;
	u32	master_id = 0;
	unsigned long mflag;
	struct sunxi_iommu_dev *iommu = dev_id;

	spin_lock_irqsave(&iommu->iommu_lock, mflag);
	inter_status_reg = sunxi_iommu_read(iommu, IOMMU_INT_STA_REG) & 0x3ffff;
	l1_pgint_reg = sunxi_iommu_read(iommu, IOMMU_L1PG_INT_REG);
	l2_pgint_reg = sunxi_iommu_read(iommu, IOMMU_L2PG_INT_REG);
	int_masterid_bitmap = inter_status_reg | l1_pgint_reg | l2_pgint_reg;

	if (inter_status_reg & MICRO_TLB0_INVALID_INTER_MASK) {
		pr_err("%s Invalid Authority\n", master[0]);
		addr_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_ADDR_REG0);
		data_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_DATA_REG0);
	} else if (inter_status_reg & MICRO_TLB1_INVALID_INTER_MASK) {
		pr_err("%s Invalid Authority\n", master[1]);
		addr_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_ADDR_REG1);
		data_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_DATA_REG1);
	} else if (inter_status_reg & MICRO_TLB2_INVALID_INTER_MASK) {
		pr_err("%s Invalid Authority\n", master[2]);
		addr_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_ADDR_REG2);
		data_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_DATA_REG2);
	} else if (inter_status_reg & MICRO_TLB3_INVALID_INTER_MASK) {
		pr_err("%s Invalid Authority\n", master[3]);
		addr_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_ADDR_REG3);
		data_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_DATA_REG3);
	} else if (inter_status_reg & MICRO_TLB4_INVALID_INTER_MASK) {
		pr_err("%s Invalid Authority\n", master[4]);
		addr_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_ADDR_REG4);
		data_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_DATA_REG4);
	} else if (inter_status_reg & MICRO_TLB5_INVALID_INTER_MASK) {
		pr_err("%s Invalid Authority\n", master[5]);
		addr_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_ADDR_REG5);
		data_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_DATA_REG5);
	}
#if IS_ENABLED(CONFIG_ARCH_SUN8IW15) || IS_ENABLED(CONFIG_ARCH_SUN50IW9) \
	|| IS_ENABLED(CONFIG_ARCH_SUN8IW19) || IS_ENABLED(CONFIG_ARCH_SUN50IW10) \
	|| IS_ENABLED(CONFIG_ARCH_SUN8IW20) || IS_ENABLED(CONFIG_ARCH_SUN50IW12) \
	|| IS_ENABLED(CONFIG_ARCH_SUN20IW1) || IS_ENABLED(CONFIG_ARCH_SUN55IW3)
	else if (inter_status_reg & MICRO_TLB6_INVALID_INTER_MASK) {
		pr_err("%s Invalid Authority\n", master[6]);
		addr_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_ADDR_REG6);
		data_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_DATA_REG6);
	} else if (inter_status_reg & L1_PAGETABLE_INVALID_INTER_MASK) {
		/*It's OK to prefetch an invalid pagetable,no need to print msg for debug.*/
		if (!(int_masterid_bitmap & (1U << 31)))
			pr_err("L1 PageTable Invalid\n");
		addr_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_ADDR_REG7);
		data_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_DATA_REG7);
	} else if (inter_status_reg & L2_PAGETABLE_INVALID_INTER_MASK) {
		if (!(int_masterid_bitmap & (1U << 31)))
			pr_err("L2 PageTable Invalid\n");
		addr_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_ADDR_REG8);
		data_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_DATA_REG8);
	}
#else
	else if (inter_status_reg & L1_PAGETABLE_INVALID_INTER_MASK) {
		pr_err("L1 PageTable Invalid\n");
		addr_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_ADDR_REG6);
		data_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_DATA_REG6);
	} else if (inter_status_reg & L2_PAGETABLE_INVALID_INTER_MASK) {
		pr_err("L2 PageTable Invalid\n");
		addr_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_ADDR_REG7);
		data_reg = sunxi_iommu_read(iommu, IOMMU_INT_ERR_DATA_REG7);
	}
#endif
	else
		pr_err("sunxi iommu int error!!!\n");

	if (!(int_masterid_bitmap & (1U << 31))) {
		if (sunxi_iova_invalid_helper(addr_reg)) {
			int_masterid_bitmap &= 0xffff;
			master_id = __ffs(int_masterid_bitmap);
		}
		pr_err("%s invalid address: 0x%x, data:0x%x, id:0x%x\n",
			master[master_id], addr_reg, data_reg,
				int_masterid_bitmap);
	}

#if IS_ENABLED(CONFIG_ARCH_SUN8IW19) || IS_ENABLED(CONFIG_ARCH_SUN50IW9) \
	|| IS_ENABLED(CONFIG_ARCH_SUN50IW10)
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_START_ADDR_REG, addr_reg);
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_END_ADDR_REG, addr_reg + 4 * SPAGE_SIZE);
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_ENABLE_REG, 0x1);
	while (sunxi_iommu_read(iommu, IOMMU_TLB_IVLD_ENABLE_REG) & 0x1)
		;
	sunxi_iommu_write(iommu, IOMMU_PC_IVLD_ADDR_REG, addr_reg);
	sunxi_iommu_write(iommu, IOMMU_PC_IVLD_ENABLE_REG, 0x1);
	while (sunxi_iommu_read(iommu, IOMMU_PC_IVLD_ENABLE_REG) & 0x1)
		;
	sunxi_iommu_write(iommu, IOMMU_PC_IVLD_ADDR_REG,
		addr_reg + 0x200000);
	sunxi_iommu_write(iommu, IOMMU_PC_IVLD_ENABLE_REG, 0x1);
	while (sunxi_iommu_read(iommu, IOMMU_PC_IVLD_ENABLE_REG) & 0x1)
		;
#elif IS_ENABLED(CONFIG_ARCH_SUN8IW20) || IS_ENABLED(CONFIG_ARCH_SUN50IW12) \
	|| IS_ENABLED(CONFIG_ARCH_SUN20IW1) || IS_ENABLED(CONFIG_ARCH_SUN55IW3)
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_START_ADDR_REG, addr_reg);
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_END_ADDR_REG, addr_reg + 4 * SPAGE_SIZE);
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_ENABLE_REG, 0x1);
	while (sunxi_iommu_read(iommu, IOMMU_TLB_IVLD_ENABLE_REG) & 0x1)
		;
	sunxi_iommu_write(iommu, IOMMU_PC_IVLD_START_ADDR_REG, addr_reg);
	sunxi_iommu_write(iommu, IOMMU_PC_IVLD_END_ADDR_REG, addr_reg + 2 * SPD_SIZE);
	sunxi_iommu_write(iommu, IOMMU_PC_IVLD_ENABLE_REG, 0x1);
	while (sunxi_iommu_read(iommu, IOMMU_PC_IVLD_ENABLE_REG) & 0x1)
		;
#else
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_ADDR_REG, addr_reg);
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_ADDR_MASK_REG,
		(u32)IOMMU_PT_MASK);
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_ENABLE_REG, 0x1);
	while (sunxi_iommu_read(iommu, IOMMU_TLB_IVLD_ENABLE_REG) & 0x1)
		;
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_ADDR_REG,
		addr_reg + 0x2000);
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_ADDR_MASK_REG,
		(u32)IOMMU_PT_MASK);
	sunxi_iommu_write(iommu, IOMMU_TLB_IVLD_ENABLE_REG, 0x1);
	while (sunxi_iommu_read(iommu, IOMMU_TLB_IVLD_ENABLE_REG) & 0x1)
		;
	sunxi_iommu_write(iommu, IOMMU_PC_IVLD_ADDR_REG, addr_reg);
	sunxi_iommu_write(iommu, IOMMU_PC_IVLD_ENABLE_REG, 0x1);
	while (sunxi_iommu_read(iommu, IOMMU_PC_IVLD_ENABLE_REG) & 0x1)
		;
	sunxi_iommu_write(iommu, IOMMU_PC_IVLD_ADDR_REG,
		addr_reg + 0x200000);
	sunxi_iommu_write(iommu, IOMMU_PC_IVLD_ENABLE_REG, 0x1);
	while (sunxi_iommu_read(iommu, IOMMU_PC_IVLD_ENABLE_REG) & 0x1)
		;
#endif

	sunxi_iommu_write(iommu, IOMMU_INT_CLR_REG, inter_status_reg);
	inter_status_reg |= (l1_pgint_reg | l2_pgint_reg);
	inter_status_reg &= 0xffff;
	sunxi_iommu_write(iommu, IOMMU_RESET_REG, ~inter_status_reg);
	sunxi_iommu_write(iommu, IOMMU_RESET_REG, 0xffffffff);
	spin_unlock_irqrestore(&iommu->iommu_lock, mflag);

	return IRQ_HANDLED;
}

static ssize_t sunxi_iommu_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	u32 data;

	spin_lock(&iommu->iommu_lock);
	data = sunxi_iommu_read(iommu, IOMMU_PMU_ENABLE_REG);
	spin_unlock(&iommu->iommu_lock);
	return snprintf(buf, PAGE_SIZE,
		"enable = %d\n", data & 0x1 ? 1 : 0);
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

	if (val) {
		spin_lock(&iommu->iommu_lock);
		data = sunxi_iommu_read(iommu, IOMMU_PMU_ENABLE_REG);
		sunxi_iommu_write(iommu, IOMMU_PMU_ENABLE_REG, data | 0x1);
		data = sunxi_iommu_read(iommu, IOMMU_PMU_CLR_REG);
		sunxi_iommu_write(iommu, IOMMU_PMU_CLR_REG, data | 0x1);
		retval = sunxi_wait_when((sunxi_iommu_read(iommu,
				IOMMU_PMU_CLR_REG) & 0x1), 1);
		if (retval)
			dev_err(iommu->dev, "Clear PMU Count timed out\n");
		spin_unlock(&iommu->iommu_lock);
	} else {
		spin_lock(&iommu->iommu_lock);
		data = sunxi_iommu_read(iommu, IOMMU_PMU_CLR_REG);
		sunxi_iommu_write(iommu, IOMMU_PMU_CLR_REG, data | 0x1);
		retval = sunxi_wait_when((sunxi_iommu_read(iommu,
				IOMMU_PMU_CLR_REG) & 0x1), 1);
		if (retval)
			dev_err(iommu->dev, "Clear PMU Count timed out\n");
		data = sunxi_iommu_read(iommu, IOMMU_PMU_ENABLE_REG);
		sunxi_iommu_write(iommu, IOMMU_PMU_ENABLE_REG, data & ~0x1);
		spin_unlock(&iommu->iommu_lock);
	}

	return count;
}

static ssize_t sunxi_iommu_profilling_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sunxi_iommu_dev *iommu = global_iommu_dev;
	u64 micro_tlb0_access_count;
	u64 micro_tlb0_hit_count;
	u64 micro_tlb1_access_count;
	u64 micro_tlb1_hit_count;
	u64 micro_tlb2_access_count;
	u64 micro_tlb2_hit_count;
	u64 micro_tlb3_access_count;
	u64 micro_tlb3_hit_count;
	u64 micro_tlb4_access_count;
	u64 micro_tlb4_hit_count;
	u64 micro_tlb5_access_count;
	u64 micro_tlb5_hit_count;
#if IS_ENABLED(CONFIG_ARCH_SUN8IW15) | IS_ENABLED(CONFIG_ARCH_SUN50IW9) \
	|| IS_ENABLED(CONFIG_ARCH_SUN8IW19) || IS_ENABLED(CONFIG_ARCH_SUN50IW10) \
	|| IS_ENABLED(CONFIG_ARCH_SUN8IW20) || IS_ENABLED(CONFIG_ARCH_SUN50IW12) \
	|| IS_ENABLED(CONFIG_ARCH_SUN55IW3)
	u64 micro_tlb6_access_count;
	u64 micro_tlb6_hit_count;
#endif
	u64 macrotlb_access_count;
	u64 macrotlb_hit_count;
	u64 ptwcache_access_count;
	u64 ptwcache_hit_count;
	u64 micro_tlb0_latency;
	u64 micro_tlb1_latency;
	u64 micro_tlb2_latency;
	u64 micro_tlb3_latency;
	u64 micro_tlb4_latency;
	u64 micro_tlb5_latency;
#if IS_ENABLED(CONFIG_ARCH_SUN8IW15) | IS_ENABLED(CONFIG_ARCH_SUN50IW9) \
	|| IS_ENABLED(CONFIG_ARCH_SUN8IW19) || IS_ENABLED(CONFIG_ARCH_SUN50IW10) \
	|| IS_ENABLED(CONFIG_ARCH_SUN8IW20) || IS_ENABLED(CONFIG_ARCH_SUN50IW12) \
	|| IS_ENABLED(CONFIG_ARCH_SUN55IW3)
	u64 micro_tlb6_latency;
	u32 micro_tlb0_max_latency;
	u32 micro_tlb1_max_latency;
	u32 micro_tlb2_max_latency;
	u32 micro_tlb3_max_latency;
	u32 micro_tlb4_max_latency;
	u32 micro_tlb5_max_latency;
	u32 micro_tlb6_max_latency;
#endif

	spin_lock(&iommu->iommu_lock);

#if IS_ENABLED(CONFIG_ARCH_SUN8IW15) | IS_ENABLED(CONFIG_ARCH_SUN50IW9) \
	|| IS_ENABLED(CONFIG_ARCH_SUN8IW19) || IS_ENABLED(CONFIG_ARCH_SUN50IW10) \
	|| IS_ENABLED(CONFIG_ARCH_SUN8IW20) || IS_ENABLED(CONFIG_ARCH_SUN50IW12) \
	|| IS_ENABLED(CONFIG_ARCH_SUN55IW3)
	macrotlb_access_count =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_HIGH_REG6) &
		0x7ff) << 32) |
		sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_LOW_REG6);
	macrotlb_hit_count =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_HIT_HIGH_REG6) &
		0x7ff) << 32) | sunxi_iommu_read(iommu, IOMMU_PMU_HIT_LOW_REG6);
#else
	macrotlb_access_count =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_HIGH_REG7) &
		0x7ff) << 32) |
		sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_LOW_REG7);
	macrotlb_hit_count =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_HIT_HIGH_REG7) &
		0x7ff) << 32) | sunxi_iommu_read(iommu, IOMMU_PMU_HIT_LOW_REG7);
#endif

	micro_tlb0_access_count =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_HIGH_REG0) &
		0x7ff) << 32) |
		sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_LOW_REG0);
	micro_tlb0_hit_count =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_HIT_HIGH_REG0) &
		0x7ff) << 32) | sunxi_iommu_read(iommu, IOMMU_PMU_HIT_LOW_REG0);

	micro_tlb1_access_count =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_HIGH_REG1) &
		0x7ff) << 32) |
		sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_LOW_REG1);
	micro_tlb1_hit_count =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_HIT_HIGH_REG1) &
		0x7ff) << 32) | sunxi_iommu_read(iommu, IOMMU_PMU_HIT_LOW_REG1);

	micro_tlb2_access_count =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_HIGH_REG2) &
		0x7ff) << 32) |
		sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_LOW_REG2);
	micro_tlb2_hit_count =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_HIT_HIGH_REG2) &
		0x7ff) << 32) | sunxi_iommu_read(iommu, IOMMU_PMU_HIT_LOW_REG2);

	micro_tlb3_access_count =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_HIGH_REG3) &
		0x7ff) << 32) |
		sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_LOW_REG3);
	micro_tlb3_hit_count =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_HIT_HIGH_REG3) &
		0x7ff) << 32) | sunxi_iommu_read(iommu, IOMMU_PMU_HIT_LOW_REG3);

	micro_tlb4_access_count =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_HIGH_REG4) &
		0x7ff) << 32) |
		sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_LOW_REG4);
	micro_tlb4_hit_count =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_HIT_HIGH_REG4) &
		0x7ff) << 32) | sunxi_iommu_read(iommu, IOMMU_PMU_HIT_LOW_REG4);

	micro_tlb5_access_count =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_HIGH_REG5) &
		0x7ff) << 32) |
		sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_LOW_REG5);
	micro_tlb5_hit_count =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_HIT_HIGH_REG5) &
		0x7ff) << 32) | sunxi_iommu_read(iommu, IOMMU_PMU_HIT_LOW_REG5);

#if IS_ENABLED(CONFIG_ARCH_SUN8IW15) | IS_ENABLED(CONFIG_ARCH_SUN50IW9) \
	|| IS_ENABLED(CONFIG_ARCH_SUN8IW19) || IS_ENABLED(CONFIG_ARCH_SUN50IW10) \
	|| IS_ENABLED(CONFIG_ARCH_SUN8IW20) || IS_ENABLED(CONFIG_ARCH_SUN50IW12) \
	|| IS_ENABLED(CONFIG_ARCH_SUN55IW3)
	micro_tlb6_access_count =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_HIGH_REG6) &
		0x7ff) << 32) |
		sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_LOW_REG6);
	micro_tlb6_hit_count =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_HIT_HIGH_REG6) &
		0x7ff) << 32) | sunxi_iommu_read(iommu, IOMMU_PMU_HIT_LOW_REG6);

	ptwcache_access_count =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_HIGH_REG8) &
		0x7ff) << 32) |
		sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_LOW_REG8);
	ptwcache_hit_count =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_HIT_HIGH_REG8) &
		0x7ff) << 32) | sunxi_iommu_read(iommu, IOMMU_PMU_HIT_LOW_REG8);
#else
	ptwcache_access_count =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_HIGH_REG7) &
		0x7ff) << 32) |
		sunxi_iommu_read(iommu, IOMMU_PMU_ACCESS_LOW_REG7);
	ptwcache_hit_count =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_HIT_HIGH_REG7) &
		0x7ff) << 32) | sunxi_iommu_read(iommu, IOMMU_PMU_HIT_LOW_REG7);
#endif

	micro_tlb0_latency =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_TL_HIGH_REG0) &
		0x3ffff) << 32) |
		sunxi_iommu_read(iommu, IOMMU_PMU_TL_LOW_REG0);
	micro_tlb1_latency =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_TL_HIGH_REG1) &
		0x3ffff) << 32) |
		sunxi_iommu_read(iommu, IOMMU_PMU_TL_LOW_REG1);
	micro_tlb2_latency =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_TL_HIGH_REG2) &
		0x3ffff) << 32) |
		sunxi_iommu_read(iommu, IOMMU_PMU_TL_LOW_REG2);
	micro_tlb3_latency =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_TL_HIGH_REG3) &
		0x3ffff) << 32) |
		sunxi_iommu_read(iommu, IOMMU_PMU_TL_LOW_REG3);
	micro_tlb4_latency =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_TL_HIGH_REG4) &
		0x3ffff) << 32) |
		sunxi_iommu_read(iommu, IOMMU_PMU_TL_LOW_REG4);
	micro_tlb5_latency =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_TL_HIGH_REG5) &
		0x3ffff) << 32) |
		sunxi_iommu_read(iommu, IOMMU_PMU_TL_LOW_REG5);

#if IS_ENABLED(CONFIG_ARCH_SUN8IW15) | IS_ENABLED(CONFIG_ARCH_SUN50IW9) \
	|| IS_ENABLED(CONFIG_ARCH_SUN8IW19) || IS_ENABLED(CONFIG_ARCH_SUN50IW10) \
	|| IS_ENABLED(CONFIG_ARCH_SUN8IW20) || IS_ENABLED(CONFIG_ARCH_SUN50IW12) \
	|| IS_ENABLED(CONFIG_ARCH_SUN55IW3)
	micro_tlb6_latency =
		((u64)(sunxi_iommu_read(iommu, IOMMU_PMU_TL_HIGH_REG6) &
		0x3ffff) << 32) |
		sunxi_iommu_read(iommu, IOMMU_PMU_TL_LOW_REG6);

	micro_tlb0_max_latency = sunxi_iommu_read(iommu, IOMMU_PMU_ML_REG0);
	micro_tlb1_max_latency = sunxi_iommu_read(iommu, IOMMU_PMU_ML_REG1);
	micro_tlb2_max_latency = sunxi_iommu_read(iommu, IOMMU_PMU_ML_REG2);
	micro_tlb3_max_latency = sunxi_iommu_read(iommu, IOMMU_PMU_ML_REG3);
	micro_tlb4_max_latency = sunxi_iommu_read(iommu, IOMMU_PMU_ML_REG4);
	micro_tlb5_max_latency = sunxi_iommu_read(iommu, IOMMU_PMU_ML_REG5);
	micro_tlb6_max_latency = sunxi_iommu_read(iommu, IOMMU_PMU_ML_REG6);

#endif
	spin_unlock(&iommu->iommu_lock);

	return snprintf(buf, PAGE_SIZE,
		"%s_access_count = 0x%llx\n"
		"%s_hit_count = 0x%llx\n"
		"%s_access_count = 0x%llx\n"
		"%s_hit_count = 0x%llx\n"
		"%s_access_count = 0x%llx\n"
		"%s_hit_count = 0x%llx\n"
		"%s_access_count = 0x%llx\n"
		"%s_hit_count = 0x%llx\n"
		"%s_access_count = 0x%llx\n"
		"%s_hit_count = 0x%llx\n"
		"%s_access_count = 0x%llx\n"
		"%s_hit_count = 0x%llx\n"
#if IS_ENABLED(CONFIG_ARCH_SUN8IW15) | IS_ENABLED(CONFIG_ARCH_SUN50IW9) \
	|| IS_ENABLED(CONFIG_ARCH_SUN8IW19) || IS_ENABLED(CONFIG_ARCH_SUN50IW10) \
	|| IS_ENABLED(CONFIG_ARCH_SUN8IW20) || IS_ENABLED(CONFIG_ARCH_SUN50IW12) \
	|| IS_ENABLED(CONFIG_ARCH_SUN55IW3)
		"%s_access_count = 0x%llx\n"
		"%s_hit_count = 0x%llx\n"
#endif
		"macrotlb_access_count = 0x%llx\n"
		"macrotlb_hit_count = 0x%llx\n"
		"ptwcache_access_count = 0x%llx\n"
		"ptwcache_hit_count = 0x%llx\n"
		"%s_total_latency = 0x%llx\n"
		"%s_total_latency = 0x%llx\n"
		"%s_total_latency = 0x%llx\n"
		"%s_total_latency = 0x%llx\n"
		"%s_total_latency = 0x%llx\n"
		"%s_total_latency = 0x%llx\n"
#if IS_ENABLED(CONFIG_ARCH_SUN8IW15) | IS_ENABLED(CONFIG_ARCH_SUN50IW9) \
	|| IS_ENABLED(CONFIG_ARCH_SUN8IW19) || IS_ENABLED(CONFIG_ARCH_SUN50IW10) \
	|| IS_ENABLED(CONFIG_ARCH_SUN8IW20) || IS_ENABLED(CONFIG_ARCH_SUN50IW12) \
	|| IS_ENABLED(CONFIG_ARCH_SUN55IW3)
		"%s_total_latency = 0x%llx\n"
		"%s_max_latency = 0x%x\n"
		"%s_max_latency = 0x%x\n"
		"%s_max_latency = 0x%x\n"
		"%s_max_latency = 0x%x\n"
		"%s_max_latency = 0x%x\n"
		"%s_max_latency = 0x%x\n"
		"%s_max_latency = 0x%x\n"
#endif
		,
		master[0], micro_tlb0_access_count,
		master[0], micro_tlb0_hit_count,
		master[1], micro_tlb1_access_count,
		master[1], micro_tlb1_hit_count,
		master[2], micro_tlb2_access_count,
		master[2], micro_tlb2_hit_count,
		master[3], micro_tlb3_access_count,
		master[3], micro_tlb3_hit_count,
		master[4], micro_tlb4_access_count,
		master[4], micro_tlb4_hit_count,
		master[5], micro_tlb5_access_count,
		master[5], micro_tlb5_hit_count,
#if IS_ENABLED(CONFIG_ARCH_SUN8IW15) | IS_ENABLED(CONFIG_ARCH_SUN50IW9) \
	|| IS_ENABLED(CONFIG_ARCH_SUN8IW19) || IS_ENABLED(CONFIG_ARCH_SUN50IW10) \
	|| IS_ENABLED(CONFIG_ARCH_SUN8IW20) || IS_ENABLED(CONFIG_ARCH_SUN50IW12) \
	|| IS_ENABLED(CONFIG_ARCH_SUN55IW3)
		master[6], micro_tlb6_access_count,
		master[6], micro_tlb6_hit_count,
#endif
		macrotlb_access_count,
		macrotlb_hit_count,
		ptwcache_access_count,
		ptwcache_hit_count,
		master[0], micro_tlb0_latency,
		master[1], micro_tlb1_latency,
		master[2], micro_tlb2_latency,
		master[3], micro_tlb3_latency,
		master[4], micro_tlb4_latency,
		master[5], micro_tlb5_latency
#if IS_ENABLED(CONFIG_ARCH_SUN8IW15) | IS_ENABLED(CONFIG_ARCH_SUN50IW9) \
	|| IS_ENABLED(CONFIG_ARCH_SUN8IW19) || IS_ENABLED(CONFIG_ARCH_SUN50IW10) \
	|| IS_ENABLED(CONFIG_ARCH_SUN8IW20) || IS_ENABLED(CONFIG_ARCH_SUN50IW12) \
	|| IS_ENABLED(CONFIG_ARCH_SUN55IW3)
		,
		master[6], micro_tlb6_latency,
		master[0], micro_tlb0_max_latency,
		master[1], micro_tlb1_max_latency,
		master[2], micro_tlb2_max_latency,
		master[3], micro_tlb3_max_latency,
		master[4], micro_tlb4_max_latency,
		master[5], micro_tlb5_max_latency,
		master[6], micro_tlb6_max_latency
#endif
			);
}

#define DUMP_REGION_MAP 0
#define DUMP_REGION_RESERVE 1
struct dump_region {
	u32 access_mask;
	size_t size;
	u32 type;
	dma_addr_t phys, iova;
};

static inline bool __region_ended(u32 pent)
{
	return !(pent & SUNXI_PTE_PAGE_VALID);
}

static inline bool __access_mask_changed(u32 pent, u32 old_mask)
{
	return old_mask !=
	       (pent & (SUNXI_PTE_PAGE_READABLE | SUNXI_PTE_PAGE_WRITABLE));
}

static inline u32 __print_region(char *buf, size_t buf_len, ssize_t len,
				 struct dump_region *active_region)
{
	if (active_region->type == DUMP_REGION_RESERVE) {
		len += scnprintf(
			buf + len, buf_len - len,
			"iova:%pad                            size:0x%zx\n",
			&active_region->iova, active_region->size);
	} else {
		len += scnprintf(
			buf + len, buf_len - len,
			"iova:%pad phys:%pad %s%s size:0x%zx\n",
			&active_region->iova, &active_region->phys,
			active_region->access_mask & SUNXI_PTE_PAGE_READABLE ?
				"R" :
				" ",
			active_region->access_mask & SUNXI_PTE_PAGE_WRITABLE ?
				"W" :
				" ",
			active_region->size);
	}
	return len;
}

ssize_t sunxi_iommu_dump_pgtable(char *buf, size_t buf_len)
{
	/* walk and dump */
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
		len = __print_region(buf, buf_len, len, &active_region
				      );
	}
	len += scnprintf(buf + len, buf_len - len, "mapped\n");

	dent = sunxi_domain->pgtable;
	active_region.type = DUMP_REGION_MAP;
	active_region.size = 0;
	active_region.access_mask = 0;
	for (i = 0; i < NUM_ENTRIES_PDE; i++) {
		j = 0;
		if (!IS_VALID(dent[i]))
			continue;
		/* iova here use for l1 idx, safe to pass 0 to get entry for 1st page(idx 0)*/
		pent = iopte_offset(dent + i, 0);
		for (; j < NUM_ENTRIES_PTE; j++) {
			if (active_region.size) {
				//looks like we are counting something, check if it need printing
				if (__region_ended(pent[j]) /* not contiguous */
				    ||
				    (active_region.access_mask &&
				     __access_mask_changed(
					     pent[j],
					     active_region
						     .access_mask)) /* different access */
				) {
					len = __print_region(buf, buf_len, len,
							      &active_region
							      );

					//prepare next region
					active_region.size = 0;
					active_region.access_mask = 0;
				}
			}

			if (pent[j] & SUNXI_PTE_PAGE_VALID) {
				//no on count region, mark start address
				if (active_region.size == 0) {
					active_region.iova =
						((dma_addr_t)i
						 << IOMMU_PD_SHIFT) +
						((dma_addr_t)j
						 << IOMMU_PT_SHIFT);
					active_region.phys =
						iommu_phy_to_cpu_phy(
							IOPTE_TO_PFN(&pent[j]));
					active_region.access_mask =
						(pent[j] &
						 (SUNXI_PTE_PAGE_READABLE |
						  SUNXI_PTE_PAGE_WRITABLE));
				}
				active_region.size += 1 << IOMMU_PT_SHIFT;
			}
		}
	}
	//dump last region (if any)
	if (active_region.size) {
		len = __print_region(buf, buf_len, len, &active_region
				      );
	}
	return len;
}
EXPORT_SYMBOL_GPL(sunxi_iommu_dump_pgtable);

static ssize_t sunxi_iommu_map_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return sunxi_iommu_dump_pgtable(buf, PAGE_SIZE);
}


static struct device_attribute sunxi_iommu_enable_attr =
	__ATTR(enable, 0644, sunxi_iommu_enable_show,
	sunxi_iommu_enable_store);
static struct device_attribute sunxi_iommu_profilling_attr =
	__ATTR(profilling, 0444, sunxi_iommu_profilling_show, NULL);
static struct device_attribute sunxi_iommu_map_attr =
	__ATTR(page_debug, 0444, sunxi_iommu_map_show, NULL);

static void sunxi_iommu_sysfs_create(struct platform_device *_pdev)
{
	device_create_file(&_pdev->dev, &sunxi_iommu_enable_attr);
	device_create_file(&_pdev->dev, &sunxi_iommu_profilling_attr);
	device_create_file(&_pdev->dev, &sunxi_iommu_map_attr);
}

static void sunxi_iommu_sysfs_remove(struct platform_device *_pdev)
{
	device_remove_file(&_pdev->dev, &sunxi_iommu_enable_attr);
	device_remove_file(&_pdev->dev, &sunxi_iommu_profilling_attr);
	device_remove_file(&_pdev->dev, &sunxi_iommu_map_attr);
}

static int sunxi_iommu_check_cmd(struct device *dev, void *data)
{
	struct iommu_resv_region *region;
	int prot = IOMMU_WRITE | IOMMU_READ;
	struct list_head *rsv_list = data;
	struct {
		const char *name;
		u32 region_type;
	} supported_region[2] = { { "sunxi-iova-reserve", IOMMU_RESV_RESERVED },
				  { "sunxi-iova-premap", IOMMU_RESV_DIRECT } };
	int i, j;
#define REGION_CNT_MAX (8)
	struct {
		u64 array[REGION_CNT_MAX * 2];
		int count;
	} *tmp_data;

	tmp_data = kzalloc(sizeof(*tmp_data), GFP_KERNEL);
	if (!tmp_data)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(supported_region); i++) {
		/* search all supported argument */
		if (!of_find_property(dev->of_node, supported_region[i].name,
				      NULL))
			continue;

		tmp_data->count = of_property_read_variable_u64_array(
			dev->of_node, supported_region[i].name, tmp_data->array,
			0, REGION_CNT_MAX);
		if (tmp_data->count <= 0)
			continue;
		if ((tmp_data->count & 1) != 0) {
			dev_err(dev,
				"size %d of array %s should be even\n",
				tmp_data->count, supported_region[i].name);
			continue;
		}

		/* two u64 describe one region */
		tmp_data->count /= 2;

		/* prepared reserve region data */
		for (j = 0; j < tmp_data->count; j++) {
			region = iommu_alloc_resv_region(
				tmp_data->array[j * 2],
				tmp_data->array[j * 2 + 1], prot,
				supported_region[i].region_type,
				GFP_KERNEL);
			if (!region) {
				dev_err(dev, "no memory for iova rsv region");
			} else {
				struct iommu_resv_region *walk;
				/* warn on region overlaps */
				list_for_each_entry(walk, rsv_list, list) {
					phys_addr_t walk_end =
						walk->start + walk->length;
					phys_addr_t region_end =
						region->start + region->length;
					if (!(walk->start >
						      region->start +
							      region->length ||
					      walk->start + walk->length <
						      region->start)) {
						dev_warn(
							dev,
							"overlap on iova-reserve %pap~%pap with %pap~%pap",
							&walk->start, &walk_end,
							&region->start,
							&region_end);
					}
				}
				list_add_tail(&region->list, rsv_list);
			}
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

	if (!head) {
		pr_warn("sunxi-iommu: get_resv_regions called with NULL list head\n");
		return;
	}

	if (!global_iommu_dev) {
		pr_warn("sunxi-iommu: get_resv_regions with NULL global_iommu_dev\n");
		return;
	}

	if (list_empty(&global_iommu_dev->rsv_list))
		__init_reserve_mem(global_iommu_dev);

	if (list_empty(&global_iommu_dev->rsv_list))
		return;
	list_for_each_entry (entry, &global_iommu_dev->rsv_list, list) {
		region = iommu_alloc_resv_region(entry->start, entry->length,
						 entry->prot, entry->type,
						 GFP_KERNEL);
		list_add_tail(&region->list, head);
	}
}


static const struct iommu_ops sunxi_iommu_ops = {
	.domain_alloc_paging = sunxi_iommu_domain_alloc_paging,
	.probe_device	= sunxi_iommu_probe_device,
	.release_device = sunxi_iommu_release_device,
	.get_resv_regions = sunxi_iommu_get_resv_regions,
	.of_xlate = sunxi_iommu_of_xlate,
	.default_domain_ops = &(const struct iommu_domain_ops) {
		.attach_dev	= sunxi_iommu_attach_dev,
		.map_pages	= sunxi_iommu_map,
		.unmap_pages	= sunxi_iommu_unmap,
		.iotlb_sync	= sunxi_iommu_iotlb_sync,
		.iova_to_phys	= sunxi_iommu_iova_to_phys,
		.free		= sunxi_iommu_domain_free,
	},
	.device_group = generic_single_device_group,
	.owner = THIS_MODULE,
};

static int sunxi_iommu_probe(struct platform_device *pdev)
{
	int ret, irq;
	struct device *dev = &pdev->dev;
	struct sunxi_iommu_dev *sunxi_iommu;
	struct resource *res;

	iopte_cache = kmem_cache_create("sunxi-iopte-cache", PT_SIZE,
				PT_SIZE, SLAB_HWCACHE_ALIGN, NULL);
	if (!iopte_cache) {
		pr_err("%s: Failed to create sunx-iopte-cache.\n", __func__);
		return -ENOMEM;
	}

	sunxi_iommu = devm_kzalloc(dev, sizeof(*sunxi_iommu), GFP_KERNEL);
	if (!sunxi_iommu)
		return	-ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_dbg(dev, "Unable to find resource region\n");
		ret = -ENOENT;
		goto err_res;
	}

	sunxi_iommu->base = devm_ioremap_resource(&pdev->dev, res);
	if (!sunxi_iommu->base) {
		dev_dbg(dev, "Unable to map IOMEM @ PA:%#x\n",
				(unsigned int)res->start);
		ret = -ENOENT;
		goto err_res;
	}

	sunxi_iommu->bypass = DEFAULT_BYPASS_VALUE;

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_dbg(dev, "Unable to find IRQ resource\n");
		ret = -ENOENT;
		goto err_irq;
	}

	pr_info("sunxi iommu: irq = %d\n", irq);

	ret = devm_request_irq(dev, irq, sunxi_iommu_irq, 0,
			dev_name(dev), (void *)sunxi_iommu);
	if (ret < 0) {
		dev_dbg(dev, "Unabled to register interrupt handler\n");
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

	ret = iommu_device_register(&sunxi_iommu->iommu, &sunxi_iommu_ops, dev);
	if (ret) {
		dev_err(dev, "Failed to register iommu\n");
		goto err_remove_sysfs;
	}

	if (!dma_dev)
		dma_dev = &pdev->dev;

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
		list_for_each_entry_safe (entry, next, &sunxi_iommu->rsv_list,
					  list)
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

const struct dev_pm_ops sunxi_iommu_pm_ops = {
	.suspend	= sunxi_iommu_suspend,
	.resume		= sunxi_iommu_resume,
};

static const struct of_device_id sunxi_iommu_dt[] = {
	{ .compatible = "allwinner,sunxi-iommu", },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, sunxi_iommu_dt);

static struct platform_driver sunxi_iommu_driver = {
	.probe		= sunxi_iommu_probe,
	.remove		= sunxi_iommu_remove,
	.driver		= {
		.owner		= THIS_MODULE,
		.name		= "sunxi-iommu",
		.pm 		= &sunxi_iommu_pm_ops,
		.of_match_table = sunxi_iommu_dt,
	}
};

static int __init sunxi_iommu_init(void)
{
	return platform_driver_register(&sunxi_iommu_driver);
}

static void __exit sunxi_iommu_exit(void)
{
	return platform_driver_unregister(&sunxi_iommu_driver);
}

subsys_initcall(sunxi_iommu_init);
module_exit(sunxi_iommu_exit);

MODULE_LICENSE("GPL v2");
