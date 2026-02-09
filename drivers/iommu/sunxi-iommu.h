/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Allwinner IOMMU driver - internal structures and register definitions
 *
 * Copyright (C) 2016-2018 Allwinner Technology CO., LTD.
 * Copyright (C) 2024-2025 various contributors
 */

#ifndef _SUNXI_IOMMU_H_
#define _SUNXI_IOMMU_H_

#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/types.h>

/* ------------------------------------------------------------------ */
/* Register offsets                                                     */
/* ------------------------------------------------------------------ */
#define IOMMU_VERSION_REG		0x0000
#define IOMMU_RESET_REG			0x0010
#define IOMMU_ENABLE_REG		0x0020
#define IOMMU_BYPASS_REG		0x0030
#define IOMMU_AUTO_GATING_REG		0x0040
#define IOMMU_WBUF_CTRL_REG		0x0044
#define IOMMU_OOO_CTRL_REG		0x0048
#define IOMMU_4KB_BDY_PRT_CTRL_REG	0x004C
#define IOMMU_TTB_REG			0x0050
#define IOMMU_TLB_ENABLE_REG		0x0060
#define IOMMU_TLB_PREFETCH_REG		0x0070
#define IOMMU_TLB_FLUSH_ENABLE_REG	0x0080
#define IOMMU_TLB_IVLD_MODE_SEL_REG	0x0084
#define IOMMU_TLB_IVLD_START_ADDR_REG	0x0088
#define IOMMU_TLB_IVLD_END_ADDR_REG	0x008C
#define IOMMU_TLB_IVLD_ADDR_REG	0x0090
#define IOMMU_TLB_IVLD_ADDR_MASK_REG	0x0094
#define IOMMU_TLB_IVLD_ENABLE_REG	0x0098
#define IOMMU_PC_IVLD_MODE_SEL_REG	0x009C
#define IOMMU_PC_IVLD_ADDR_REG		0x00A0
#define IOMMU_PC_IVLD_START_ADDR_REG	0x00A4
#define IOMMU_PC_IVLD_ENABLE_REG	0x00A8
#define IOMMU_PC_IVLD_END_ADDR_REG	0x00AC

#define IOMMU_DM_AUT_CTRL_REG(n)	(0x00B0 + (n) * 4)	/* n = 0..7 */
#define IOMMU_DM_AUT_OVWT_REG		0x00D0

/* Interrupt registers */
#define IOMMU_INT_ENABLE_REG		0x0100
#define IOMMU_INT_CLR_REG		0x0104
#define IOMMU_INT_STA_REG		0x0108

/*
 * Error address / data registers for micro-TLB 0..5 share a common
 * stride and are identical across all known SoC variants.
 */
#define IOMMU_INT_ERR_ADDR_REG(n)	(0x0110 + (n) * 4)	/* n = 0..5 */
#define IOMMU_INT_ERR_DATA_REG(n)	(0x0150 + (n) * 4)	/* n = 0..5 */

/*
 * Variants with 7 micro-TLBs insert an extra register pair for micro-TLB 6
 * at 0x128 / 0x168.  The L1 / L2 page-table error registers always live at
 * 0x130-0x134 / 0x170-0x174 regardless of the micro-TLB count; the BSP
 * header used different REG6/REG7/REG8 numbering depending on the SoC which
 * was confusing — the macros below use semantic names instead.
 */
#define IOMMU_INT_ERR_ADDR_UTLB6	0x0128	/* 7-TLB only */
#define IOMMU_INT_ERR_ADDR_L1		0x0130
#define IOMMU_INT_ERR_ADDR_L2		0x0134

#define IOMMU_INT_ERR_DATA_UTLB6	0x0168	/* 7-TLB only */
#define IOMMU_INT_ERR_DATA_L1		0x0170
#define IOMMU_INT_ERR_DATA_L2		0x0174

#define IOMMU_L1PG_INT_REG		0x0180
#define IOMMU_L2PG_INT_REG		0x0184

/* Virtual-address debug registers */
#define IOMMU_VA_REG			0x0190
#define IOMMU_VA_DATA_REG		0x0194
#define IOMMU_VA_CONFIG_REG		0x0198

/* ---- PMU registers ------------------------------------------------ */
#define IOMMU_PMU_ENABLE_REG		0x0200
#define IOMMU_PMU_CLR_REG		0x0210

/* Per-micro-TLB access / hit counters (n = 0..5 always, 6 on 7-TLB) */
#define IOMMU_PMU_ACCESS_LOW_REG(n)	(0x0230 + (n) * 0x10)
#define IOMMU_PMU_ACCESS_HIGH_REG(n)	(0x0234 + (n) * 0x10)
#define IOMMU_PMU_HIT_LOW_REG(n)	(0x0238 + (n) * 0x10)
#define IOMMU_PMU_HIT_HIGH_REG(n)	(0x023C + (n) * 0x10)

/* Macro-TLB PMU (same address on 6-TLB and 7-TLB variants) */
#define IOMMU_PMU_ACCESS_LOW_MACROTLB	0x02D0
#define IOMMU_PMU_ACCESS_HIGH_MACROTLB	0x02D4
#define IOMMU_PMU_HIT_LOW_MACROTLB	0x02D8
#define IOMMU_PMU_HIT_HIGH_MACROTLB	0x02DC

/* PTW-cache PMU (same address on 6-TLB and 7-TLB variants) */
#define IOMMU_PMU_ACCESS_LOW_PTWCACHE	0x02E0
#define IOMMU_PMU_ACCESS_HIGH_PTWCACHE	0x02E4
#define IOMMU_PMU_HIT_LOW_PTWCACHE	0x02E8
#define IOMMU_PMU_HIT_HIGH_PTWCACHE	0x02EC

/* Per-micro-TLB total / max latency (n = 0..5 always, 6 on 7-TLB) */
#define IOMMU_PMU_TL_LOW_REG(n)	(0x0300 + (n) * 0x10)
#define IOMMU_PMU_TL_HIGH_REG(n)	(0x0304 + (n) * 0x10)
#define IOMMU_PMU_ML_REG(n)		(0x0308 + (n) * 0x10)	/* 7-TLB only */

/* ------------------------------------------------------------------ */
/* Bit-field helpers                                                    */
/* ------------------------------------------------------------------ */
#define IOMMU_RESET_SHIFT	31
#define IOMMU_RESET_MASK	BIT(IOMMU_RESET_SHIFT)
#define IOMMU_RESET_SET		(0U << 31)
#define IOMMU_RESET_RELEASE	(1U << 31)
#define IOMMU_ENABLE		0x1

/* Micro-TLB interrupt masks */
#define MICRO_TLB_INVALID_INTER_MASK(n)	BIT(n)		/* n = 0..6 */
#define L1_PAGETABLE_INVALID_INTER_MASK	BIT(16)
#define L2_PAGETABLE_INVALID_INTER_MASK	BIT(17)

/* TLB flush masks */
#define MICRO_TLB_FLUSH_MASK(n)		(((1U << (n)) - 1))
#define PTW_CACHE_FLUSH_BIT	BIT(16)
#define MACRO_TLB_FLUSH_BIT	BIT(17)


/* ------------------------------------------------------------------ */
/* Page-table geometry                                                  */
/* ------------------------------------------------------------------ */

/*
 * Two-level page table: 4096 first-level (PDE) entries, each pointing
 * to 256 second-level (PTE) entries.  Only 4 KB pages are supported.
 *
 * Virtual Address:
 *  31              20 | 19        12 | 11     0
 *  +------------------+-------------+---------+
 *  |    PDE Index     |  PTE Index  | offset  |
 *  +------------------+-------------+---------+
 */
#define NUM_ENTRIES_PDE		4096
#define NUM_ENTRIES_PTE		256
#define PD_SIZE			(NUM_ENTRIES_PDE * sizeof(u32))
#define PT_SIZE			(NUM_ENTRIES_PTE * sizeof(u32))

#define IOMMU_PD_SHIFT		20
#define IOMMU_PD_MASK		(~((1UL << IOMMU_PD_SHIFT) - 1))
#define IOMMU_PT_SHIFT		12
#define IOMMU_PT_MASK		(~((1UL << IOMMU_PT_SHIFT) - 1))
#define PAGE_OFFSET_MASK	((1UL << IOMMU_PT_SHIFT) - 1)
#define IOPTE_BASE_MASK		(~(PT_SIZE - 1))

/* Page Directory Entry control bits */
#define DENT_VALID		0x01
#define DENT_PTE_SHIFT		10
#define DENT_WRITABLE		BIT(3)
#define DENT_READABLE		BIT(2)

/* Page Table Entry control bits */
#define SUNXI_PTE_PAGE_WRITABLE	BIT(3)
#define SUNXI_PTE_PAGE_READABLE	BIT(2)
#define SUNXI_PTE_PAGE_VALID	BIT(1)

#define IS_VALID(x)		(((x) & 0x03) == DENT_VALID)

#define IOPDE_INDEX(va)		(((va) >> IOMMU_PD_SHIFT) & (NUM_ENTRIES_PDE - 1))
#define IOPTE_INDEX(va)		(((va) >> IOMMU_PT_SHIFT) & (NUM_ENTRIES_PTE - 1))
#define IOPTE_BASE(ent)		((ent) & IOPTE_BASE_MASK)
#define IOPTE_TO_PFN(ent)	((*ent) & IOMMU_PT_MASK)
#define IOVA_PAGE_OFT(va)	((va) & PAGE_OFFSET_MASK)

#define SPAGE_SIZE		(1 << IOMMU_PT_SHIFT)
#define SPD_SIZE		(1 << IOMMU_PD_SHIFT)
#define SPAGE_ALIGN(addr)	ALIGN(addr, SPAGE_SIZE)
#define SPDE_ALIGN(addr)	ALIGN(addr, SPD_SIZE)
#define MAX_SG_SIZE		(128 << 20)
#define MAX_SG_TABLE_SIZE	((MAX_SG_SIZE / SPAGE_SIZE) * sizeof(u32))

/* IO virtual address helpers */
#define IOVA_START_PFN		1
#define IOVA_PFN(addr)		((addr) >> PAGE_SHIFT)
#define DMA_32BIT_PFN		IOVA_PFN(DMA_BIT_MASK(32))
#define IOVA_4M_ALIGN(iova)	((iova) & (~0x3fffff))

/* Physical address fixup (common to all known Allwinner SoCs) */
#define SUNXI_PHYS_OFFSET	0x40000000UL
#define SUNXI_4G_PHYS_OFFSET	0x100000000UL

/* ------------------------------------------------------------------ */
/* Busy-wait helper                                                     */
/* ------------------------------------------------------------------ */
#define sunxi_wait_when(COND, MS) ({				\
	unsigned long timeout__ = jiffies + msecs_to_jiffies(MS) + 1; \
	int ret__ = 0;						\
	while ((COND)) {					\
		if (time_after(jiffies, timeout__)) {		\
			ret__ = (!COND) ? 0 : -ETIMEDOUT;	\
			break;					\
		}						\
		udelay(1);					\
	}							\
	ret__;							\
})

/* ------------------------------------------------------------------ */
/* Per-SoC quirks                                                       */
/* ------------------------------------------------------------------ */
#define SUNXI_IOMMU_MAX_MASTERS	7

struct sunxi_iommu_quirks {
	/* Number of micro-TLBs present in hardware (6 or 7) */
	u8 num_micro_tlbs;

	/* TLB invalidation uses start/end range (else: addr + mask) */
	bool tlb_ivld_range;

	/* PTW cache invalidation uses start/end range (else: single addr) */
	bool ptw_ivld_range;

	/* Hardware has the prefetch-valid fix (bits 16-17 of TLB_PREFETCH) */
	bool has_prefetch_fix;

	/* Initial value for TLB_PREFETCH_REG */
	u32 tlb_prefetch_val;

	/* Master device names (one per micro-TLB, for error reporting) */
	const char *const *master_names;
};

/* ------------------------------------------------------------------ */
/* Driver structures                                                    */
/* ------------------------------------------------------------------ */
struct sunxi_iommu_dev {
	struct iommu_device iommu;
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	int irq;
	u32 bypass;
	spinlock_t iommu_lock;
	struct list_head rsv_list;

	const struct sunxi_iommu_quirks *quirks;

	/*
	 * Runtime override: set when DT contains the boolean property
	 * "allwinner,sun50iw10-iommu-fixed", which indicates the newer
	 * sun50iw10 IC revision with the prefetch-valid fix.
	 */
	bool prefetch_fixed;
};

struct sunxi_iommu_domain {
	unsigned int *pgtable;		/* first-level page directory (16 KB) */
	u32 *sg_buffer;
	struct mutex dt_lock;		/* protects page-table modifications */
	struct dma_iommu_mapping *mapping;
	struct iommu_domain domain;
	struct list_head mdevs;		/* list of master devices */
	spinlock_t lock;
};

struct sunxi_mdev {
	struct list_head node;
	struct device *dev;
	unsigned int tlbid;
	bool flag;
};

struct sunxi_iommu_owner {
	unsigned int tlbid;
	bool flag;
	struct sunxi_iommu_dev *data;
	struct device *dev;
	struct dma_iommu_mapping *mapping;
};

/* ------------------------------------------------------------------ */
/* Exported helpers (used by other BSP drivers)                         */
/* ------------------------------------------------------------------ */
void sunxi_reset_device_iommu(unsigned int master_id);
void sunxi_enable_device_iommu(unsigned int master_id, bool enable);
int sunxi_iova_test_write(dma_addr_t iova, u32 val);
unsigned long sunxi_iova_test_read(dma_addr_t iova);
void sunxi_set_debug_mode(void);
void sunxi_set_prefetch_mode(void);
ssize_t sunxi_iommu_dump_pgtable(char *buf, size_t buf_len);
extern struct iommu_domain *global_iommu_domain;

#endif /* _SUNXI_IOMMU_H_ */
