// SPDX-License-Identifier: GPL-2.0
/*
 * Remoteproc driver for the Allwinner T113 HiFi4 DSP core.
 *
 * This is a minimal loader: it can load an ELF image into the DSP's
 * address space and release reset. RPMsg/kicks are intentionally
 * omitted for now.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/vmalloc.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/remoteproc.h>
#include <linux/slab.h>
#include <linux/sizes.h>

#include "remoteproc_internal.h"
#include "remoteproc_elf_helpers.h"

#define SUNXI_SYSCRL_BASE	0x03000000
#define SUNXI_SYSCRL_SIZE	SZ_4K
#define SUNXI_SRAM_REMAP_REG	0x0008
#define SUNXI_SRAM_REMAP_EN	BIT(0)

/* DSP CFG block offsets */
#define SUNXI_DSP_ALT_RESET_VEC	0x0000
#define SUNXI_DSP_CTRL0		0x0004

#define SUNXI_DSP_RUNSTALL	BIT(0)
#define SUNXI_DSP_VEC_SEL	BIT(1)
#define SUNXI_DSP_CLKEN		BIT(2)

#define SUNXI_DSP_RST_VEC_DEFAULT	0x00100000

struct sunxi_hifi4_map {
	u64 da;
	u64 pa;
	u64 len;
};

struct sunxi_hifi4 {
	struct device *dev;
	void __iomem *cfg_base;
	void __iomem *syscon_base;
	struct rproc *rproc;

	struct clk_bulk_data *clks;
	int num_clks;
	struct reset_control *rst_core;
	struct reset_control *rst_cfg;
	struct reset_control *rst_dbg;

	struct sunxi_hifi4_map *maps;
	int maps_cnt;
};

static void sunxi_hifi4_flush_carveouts(struct sunxi_hifi4 *priv)
{
    struct rproc_mem_entry *carveout;

    list_for_each_entry(carveout, &priv->rproc->carveouts, node) {
        /* SRAM / MMIO 通过 ioremap 映成 device memory，本身就不走 CPU D-cache */
        if (carveout->is_iomem) {
            dev_dbg(priv->dev,
                    "skip cache flush for IOMEM carveout %s pa=%pa len=%zu\n",
                    carveout->name, &carveout->dma, carveout->len);
            continue;
        }

        dma_sync_single_for_device(priv->dev, carveout->dma,
                                   carveout->len, DMA_TO_DEVICE);
    }
}

static int sunxi_hifi4_translate_da(struct sunxi_hifi4 *priv, u64 da,
				    phys_addr_t *pa)
{
	int i;

	for (i = 0; i < priv->maps_cnt; i++) {
		struct sunxi_hifi4_map *m = &priv->maps[i];

		if (da >= m->da && da < m->da + m->len) {
			*pa = m->pa + (da - m->da);
			return 0;
		}
	}

	*pa = da;
	return 0;
}

static int sunxi_hifi4_translate_pa(struct sunxi_hifi4 *priv, phys_addr_t pa,
				    u64 *da)
{
	int i;

	for (i = 0; i < priv->maps_cnt; i++) {
		struct sunxi_hifi4_map *m = &priv->maps[i];

		if (pa >= m->pa && pa < m->pa + m->len) {
			*da = m->da + (pa - m->pa);
			return 0;
		}
	}

	*da = pa;
	return 0;
}

static void sunxi_hifi4_sram_remap(struct sunxi_hifi4 *priv, bool enable_dsp)
{
	u32 v;

	if (!priv->syscon_base)
		return;

	v = readl(priv->syscon_base + SUNXI_SRAM_REMAP_REG);
	if (enable_dsp)
		v &= ~SUNXI_SRAM_REMAP_EN;
	else
		v |= SUNXI_SRAM_REMAP_EN;
	writel(v, priv->syscon_base + SUNXI_SRAM_REMAP_REG);
}

static void sunxi_hifi4_set_runstall(struct sunxi_hifi4 *priv, bool stall)
{
	u32 v;

	v = readl(priv->cfg_base + SUNXI_DSP_CTRL0);
	if (stall)
		v |= SUNXI_DSP_RUNSTALL;
	else
		v &= ~SUNXI_DSP_RUNSTALL;
	writel(v, priv->cfg_base + SUNXI_DSP_CTRL0);
}

static void sunxi_hifi4_set_bootvec(struct sunxi_hifi4 *priv, u32 boot)
{
	u32 v;

	writel(boot, priv->cfg_base + SUNXI_DSP_ALT_RESET_VEC);

	v = readl(priv->cfg_base + SUNXI_DSP_CTRL0);
	if (boot != SUNXI_DSP_RST_VEC_DEFAULT)
		v |= SUNXI_DSP_VEC_SEL;
	else
		v &= ~SUNXI_DSP_VEC_SEL;
	writel(v, priv->cfg_base + SUNXI_DSP_CTRL0);
}

static void sunxi_hifi4_enable_clken(struct sunxi_hifi4 *priv)
{
	u32 v;

	v = readl(priv->cfg_base + SUNXI_DSP_CTRL0);
	v |= SUNXI_DSP_CLKEN;
	writel(v, priv->cfg_base + SUNXI_DSP_CTRL0);
}

static void *sunxi_hifi4_da_to_va(struct rproc *rproc, u64 da, size_t len,
				  bool *is_iomem)
{
	struct sunxi_hifi4 *priv = rproc->priv;
	struct rproc_mem_entry *carveout;
	phys_addr_t pa;

	if (sunxi_hifi4_translate_da(priv, da, &pa))
		return NULL;

	list_for_each_entry(carveout, &rproc->carveouts, node) {
		if (pa >= carveout->dma && pa < carveout->dma + carveout->len) {
			if (is_iomem)
				*is_iomem = carveout->is_iomem;
			dev_dbg(priv->dev,
				 "da_to_va: da=0x%llx pa=%pa len=%zu -> va=%p is_iomem=%d (%s)\n",
				 da, &pa, len, carveout->va + (pa - carveout->dma),
				 carveout->is_iomem, carveout->name);
			return carveout->va + (pa - carveout->dma);
		}
	}

	return NULL;
}

static int sunxi_hifi4_mem_alloc(struct rproc *rproc,
			 struct rproc_mem_entry *mem)
{
	struct sunxi_hifi4 *priv = rproc->priv;
	bool is_ram = pfn_valid(PFN_DOWN(mem->dma));

	/* DSP DDR: map via vmap of struct pages to avoid ioremap() warning */
	if (!strcmp(mem->name, "dsp-ddr"))
		is_ram = true;

	/* Map carveouts: DDR (vmap), normal RAM (phys_to_virt), SRAM/IOMEM (ioremap) */
	if (!strcmp(mem->name, "dsp-ddr")) {
		unsigned int pages = DIV_ROUND_UP(mem->len, PAGE_SIZE);
		struct page **pagelist;
		int i;

		pagelist = kcalloc(pages, sizeof(*pagelist), GFP_KERNEL);
		if (!pagelist)
			return -ENOMEM;
		for (i = 0; i < pages; i++)
			pagelist[i] = pfn_to_page(PFN_DOWN(mem->dma) + i);

		mem->va = vmap(pagelist, pages, VM_MAP, PAGE_KERNEL);
		if (!mem->va) {
			kfree(pagelist);
			return -ENOMEM;
		}
		mem->priv = pagelist;
		mem->is_iomem = false;
	} else if (is_ram) {
		mem->va = phys_to_virt(mem->dma);
		mem->is_iomem = false;
	} else {
		mem->va = ioremap(mem->dma, mem->len);
		if (!mem->va)
			return -ENOMEM;
		mem->is_iomem = true;
	}

	dev_dbg(priv->dev,
		 "alloc carveout %s: pa=%pad len=%zu is_ram=%d va=%px is_iomem=%d\n",
		 mem->name, &mem->dma, mem->len, is_ram, mem->va, mem->is_iomem);
	return 0;
}

static int sunxi_hifi4_mem_release(struct rproc *rproc,
			   struct rproc_mem_entry *mem)
{
	if (mem->is_iomem) {
		iounmap(mem->va);
	} else if (!strcmp(mem->name, "dsp-ddr")) {
		vunmap(mem->va);
		kfree(mem->priv);
		mem->priv = NULL;
	}

	return 0;
}

static int sunxi_hifi4_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	struct sunxi_hifi4 *priv = rproc->priv;
	struct device *dev = priv->dev;
	struct device_node *np = dev->of_node;
	struct of_phandle_iterator it;
	struct reserved_mem *rmem;
	struct rproc_mem_entry *mem;
	u64 da;
	int ret;

	ret = of_phandle_iterator_init(&it, np, "memory-region", NULL, 0);
	if (ret)
		return ret;

	while (of_phandle_iterator_next(&it) == 0) {
		rmem = of_reserved_mem_lookup(it.node);
		if (!rmem) {
			dev_err(dev, "unable to acquire memory-region %s\n",
				it.node->name);
			return -EINVAL;
		}

		sunxi_hifi4_translate_pa(priv, rmem->base, &da);

		mem = rproc_mem_entry_init(dev, NULL, (dma_addr_t)rmem->base,
					   rmem->size, da,
					   sunxi_hifi4_mem_alloc,
					   sunxi_hifi4_mem_release,
					   it.node->name);
		if (!mem)
			return -ENOMEM;

		rproc_coredump_add_segment(rproc, da, rmem->size);
		rproc_add_carveout(rproc, mem);
	}

	ret = rproc_elf_load_rsc_table(rproc, fw);
	if (ret)
		dev_warn(dev, "no resource table found for this firmware\n");

	return 0;
}

static int sunxi_hifi4_prepare(struct rproc *rproc)
{
	struct sunxi_hifi4 *priv = rproc->priv;
	int ret;

	ret = clk_bulk_prepare_enable(priv->num_clks, priv->clks);
	if (ret)
		return ret;

	sunxi_hifi4_sram_remap(priv, false);

	if (priv->rst_core)
		reset_control_assert(priv->rst_core);
	if (priv->rst_dbg)
		reset_control_assert(priv->rst_dbg);
	if (priv->rst_cfg)
		reset_control_deassert(priv->rst_cfg);

	sunxi_hifi4_set_runstall(priv, true);

	return 0;
}

static int sunxi_hifi4_unprepare(struct rproc *rproc)
{
	struct sunxi_hifi4 *priv = rproc->priv;

	sunxi_hifi4_sram_remap(priv, false);

	if (priv->rst_core)
		reset_control_assert(priv->rst_core);
	if (priv->rst_dbg)
		reset_control_assert(priv->rst_dbg);
	if (priv->rst_cfg)
		reset_control_assert(priv->rst_cfg);

	clk_bulk_disable_unprepare(priv->num_clks, priv->clks);
	return 0;
}

static int sunxi_hifi4_start(struct rproc *rproc)
{
	struct sunxi_hifi4 *priv = rproc->priv;
	u32 boot = (u32)rproc->bootaddr;
	int ret;

	sunxi_hifi4_set_runstall(priv, true);
	sunxi_hifi4_set_bootvec(priv, boot);
	sunxi_hifi4_enable_clken(priv);

	if (priv->rst_dbg) {
		ret = reset_control_deassert(priv->rst_dbg);
		if (ret)
			goto err;
	}

	if (priv->rst_core) {
		ret = reset_control_deassert(priv->rst_core);
		if (ret)
			goto err;
	}

	/* Make sure all cached writes to shared DDR are visible to the DSP */
	sunxi_hifi4_flush_carveouts(priv);

	sunxi_hifi4_sram_remap(priv, true);
	sunxi_hifi4_set_runstall(priv, false);
	dev_info(priv->dev, "HiFi4 started at 0x%x\n", boot);
	return 0;
err:
	dev_err(priv->dev, "failed to deassert reset: %d\n", ret);
	return ret;
}

static int sunxi_hifi4_stop(struct rproc *rproc)
{
	struct sunxi_hifi4 *priv = rproc->priv;

	sunxi_hifi4_set_runstall(priv, true);
	if (priv->rst_core)
		reset_control_assert(priv->rst_core);
	if (priv->rst_dbg)
		reset_control_assert(priv->rst_dbg);

	return 0;
}

static u64 sunxi_hifi4_get_boot_addr(struct rproc *rproc,
				     const struct firmware *fw)
{
	return rproc_elf_get_boot_addr(rproc, fw);
}

static int sunxi_hifi4_parse_mappings(struct device *dev,
				      struct sunxi_hifi4 *priv)
{
	int count, i, ret;
	u32 *arr;

	count = of_property_count_elems_of_size(dev->of_node,
						"allwinner,memory-mappings",
						sizeof(u32) * 3);
	if (count <= 0)
		return 0;

	priv->maps_cnt = count;
	priv->maps = devm_kcalloc(dev, count, sizeof(*priv->maps), GFP_KERNEL);
	if (!priv->maps)
		return -ENOMEM;

	arr = devm_kcalloc(dev, count * 3, sizeof(u32), GFP_KERNEL);
	if (!arr)
		return -ENOMEM;

	ret = of_property_read_u32_array(dev->of_node,
					 "allwinner,memory-mappings",
					 arr, count * 3);
	if (ret)
		return ret;

	for (i = 0; i < count; i++) {
		priv->maps[i].da = arr[i * 3];
		priv->maps[i].len = arr[i * 3 + 1];
		priv->maps[i].pa = arr[i * 3 + 2];
	}

	return 0;
}

static const struct rproc_ops sunxi_hifi4_ops = {
	.prepare	= sunxi_hifi4_prepare,
	.unprepare	= sunxi_hifi4_unprepare,
	.start		= sunxi_hifi4_start,
	.stop		= sunxi_hifi4_stop,
	.da_to_va	= sunxi_hifi4_da_to_va,
	.parse_fw	= sunxi_hifi4_parse_fw,
	.load		= rproc_elf_load_segments,
	.sanity_check	= rproc_elf_sanity_check,
	.find_loaded_rsc_table = rproc_elf_find_loaded_rsc_table,
	.get_boot_addr	= sunxi_hifi4_get_boot_addr,
};

static int sunxi_hifi4_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sunxi_hifi4 *priv;
	struct rproc *rproc;
	struct resource *res;
	const char *fw_name = NULL;
	int ret, i;
	static const char * const clk_ids[] = { "core", "bus" };

	of_property_read_string(dev->of_node, "firmware-name", &fw_name);

	rproc = rproc_alloc(dev, dev_name(dev), &sunxi_hifi4_ops, fw_name,
			    sizeof(*priv));
	if (!rproc)
		return -ENOMEM;

	priv = rproc->priv;
	priv->dev = dev;
	priv->rproc = rproc;

	priv->syscon_base = devm_ioremap(dev, SUNXI_SYSCRL_BASE,
					 SUNXI_SYSCRL_SIZE);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->cfg_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->cfg_base)) {
		ret = PTR_ERR(priv->cfg_base);
		goto free_rproc;
	}

	priv->num_clks = ARRAY_SIZE(clk_ids);
	priv->clks = devm_kcalloc(dev, priv->num_clks, sizeof(*priv->clks),
				  GFP_KERNEL);
	if (!priv->clks) {
		ret = -ENOMEM;
		goto free_rproc;
	}

	for (i = 0; i < priv->num_clks; i++)
		priv->clks[i].id = clk_ids[i];

	ret = devm_clk_bulk_get_optional(dev, priv->num_clks, priv->clks);
	if (ret)
		goto free_rproc;

	priv->rst_core = devm_reset_control_get_optional_exclusive(dev, "core");
	if (IS_ERR(priv->rst_core)) {
		ret = PTR_ERR(priv->rst_core);
		goto free_rproc;
	}

	priv->rst_cfg = devm_reset_control_get_optional_exclusive(dev, "cfg");
	if (IS_ERR(priv->rst_cfg)) {
		ret = PTR_ERR(priv->rst_cfg);
		goto free_rproc;
	}

	priv->rst_dbg = devm_reset_control_get_optional_exclusive(dev, "dbg");
	if (IS_ERR(priv->rst_dbg)) {
		ret = PTR_ERR(priv->rst_dbg);
		goto free_rproc;
	}

	ret = sunxi_hifi4_parse_mappings(dev, priv);
	if (ret)
		goto free_rproc;

	platform_set_drvdata(pdev, rproc);

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "failed to add rproc: %d\n", ret);
		goto free_rproc;
	}

	dev_info(dev, "Sunxi HiFi4 remoteproc registered\n");
	return 0;

free_rproc:
	rproc_free(rproc);
	return ret;
}

static void sunxi_hifi4_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	rproc_del(rproc);
	rproc_free(rproc);
}

static const struct of_device_id sunxi_hifi4_of_match[] = {
	{ .compatible = "allwinner,hifi4-rproc" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sunxi_hifi4_of_match);

static struct platform_driver sunxi_hifi4_driver = {
	.probe  = sunxi_hifi4_probe,
	.remove = sunxi_hifi4_remove,
	.driver = {
		.name           = "sunxi-hifi4-rproc",
		.of_match_table = sunxi_hifi4_of_match,
	},
};
module_platform_driver(sunxi_hifi4_driver);

MODULE_DESCRIPTION("Allwinner T113 HiFi4 remote processor driver");
MODULE_AUTHOR("TaterLi");
MODULE_LICENSE("GPL v2");
