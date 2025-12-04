// SPDX-License-Identifier: GPL-2.0
/*
 * remoteproc driver for the Allwinner T113 C906 coprocessor.
 *
 * Based on the vendor implementation but reduced to upstream interfaces.
 */

#include <asm/delay.h>
#include <linux/clk.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/mailbox_client.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/remoteproc.h>
#include <linux/mm.h>
#include <linux/dma-direct.h>
#include <linux/sizes.h>
#include <linux/virtio_ring.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "remoteproc_internal.h"
#include "remoteproc_elf_helpers.h"

#define SUNXI_C906_STA_ADDL	0x0004
#define SUNXI_C906_STA_ADDH	0x0008

/* Shared RAM layout (must match the remote firmware) */
#define SUNXI_C906_SHMEM_DA		0x41000000
#define SUNXI_C906_RPMSG_BUF_OFFSET	0x00020000
#define SUNXI_C906_RPMSG_BUF_LEN	0x00040000

/* CCU Base address for T113 */
#define SUNXI_CCU_BASE		0x02001000
#define SUNXI_CCU_SIZE		SZ_4K

#define SUNXI_RISCV_CLK_REG	0x0d00
#define SUNXI_RISCV_CFG_BGR_REG	0x0d0c
#define SUNXI_RISCV_RST_REG	0x0f20

#define SUNXI_RISCV_KEY		0x16aa
#define SUNXI_RISCV_RST_KEY	(SUNXI_RISCV_KEY << 16)

#define SUNXI_RISCV_SOFT_RST	BIT(0)
#define SUNXI_RISCV_CFG_GATE	BIT(0)
#define SUNXI_RISCV_CFG_RST	BIT(16)

#define SUNXI_RISCV_CLK_SRC_MASK	GENMASK(26, 24)
#define SUNXI_RISCV_CLK_SRC_PERI800M	(0x3 << 24)

struct sunxi_c906_map {
	u64 da;
	u64 pa;
	u64 len;
};

struct sunxi_c906_mbox {
	struct mbox_client client;
	struct mbox_chan *chan;
	struct work_struct vq_work;
	u32 vq_id;
};

struct sunxi_c906 {
	struct device *dev;
	void __iomem *cfg_base;
	void __iomem *ccu_base;
	struct rproc *rproc;
	struct sunxi_c906_mbox mbox;
	struct workqueue_struct *wq;

	struct sunxi_c906_map *maps;
	int maps_cnt;

	struct clk_bulk_data *clks;
	int num_clks;
	struct reset_control *rst;
};

static void *sunxi_c906_da_to_va(struct rproc *rproc, u64 da, size_t len,
				 bool *is_iomem);

static int sunxi_c906_translate_da(struct sunxi_c906 *priv, u64 da, phys_addr_t *pa)
{
	int i;

	for (i = 0; i < priv->maps_cnt; i++) {
		struct sunxi_c906_map *m = &priv->maps[i];
		if (da >= m->da && da < m->da + m->len) {
			*pa = m->pa + (da - m->da);
			return 0;
		}
	}

	/* identity mapping fallback */
	*pa = da;
	return 0;
}

static int sunxi_c906_translate_pa(struct sunxi_c906 *priv, phys_addr_t pa, u64 *da)
{
	int i;

	for (i = 0; i < priv->maps_cnt; i++) {
		struct sunxi_c906_map *m = &priv->maps[i];
		if (pa >= m->pa && pa < m->pa + m->len) {
			*da = m->da + (pa - m->pa);
			return 0;
		}
	}

	*da = pa;
	return 0;
}

static int sunxi_c906_mem_alloc(struct rproc *rproc, struct rproc_mem_entry *mem)
{
	mem->va = ioremap_wc(mem->dma, mem->len);
	return mem->va ? 0 : -ENOMEM;
}

static int sunxi_c906_mem_release(struct rproc *rproc, struct rproc_mem_entry *mem)
{
	iounmap(mem->va);
	return 0;
}

/*
 * VRing structures live inside the shared RAM window described in the firmware
 * resource table.  If we let the core allocate them with dma_alloc_coherent(),
 * they might end up at a physical address the C906 cannot access (its view is
 * offset).  Pre-register vring carveouts that simply reuse the existing
 * mapping.
 */
static int sunxi_c906_shared_alloc(struct rproc *rproc,
				   struct rproc_mem_entry *mem)
{
	struct sunxi_c906 *priv = rproc->priv;
	phys_addr_t pa;
	bool is_iomem;

	if (sunxi_c906_translate_da(priv, mem->da, &pa))
		return -EINVAL;

	mem->dma = pa;
	mem->va = sunxi_c906_da_to_va(rproc, mem->da, mem->len, &is_iomem);
	mem->is_iomem = is_iomem;

	return mem->va ? 0 : -ENOMEM;
}

static int sunxi_c906_shared_release(struct rproc *rproc,
				     struct rproc_mem_entry *mem)
{
	/* Mapping belongs to the parent carveout; nothing to undo here. */
	return 0;
}

static void *sunxi_c906_da_to_va(struct rproc *rproc, u64 da, size_t len,
				 bool *is_iomem)
{
	struct sunxi_c906 *priv = rproc->priv;
	struct rproc_mem_entry *carveout;
	phys_addr_t pa;

	if (sunxi_c906_translate_da(priv, da, &pa))
		return NULL;

	list_for_each_entry(carveout, &rproc->carveouts, node) {
		if (pa >= carveout->dma && pa < carveout->dma + carveout->len) {
			if (is_iomem)
				*is_iomem = true;
			return carveout->va + (pa - carveout->dma);
		}
	}

	return NULL;
}

static int sunxi_c906_register_vdev_buffer(struct rproc *rproc, int vdev_idx)
{
	struct sunxi_c906 *priv = rproc->priv;
	struct device *dev = priv->dev;
	struct rproc_mem_entry *mem;
	phys_addr_t pa;
	u64 buf_da = SUNXI_C906_SHMEM_DA + SUNXI_C906_RPMSG_BUF_OFFSET;

	if (sunxi_c906_translate_da(priv, buf_da, &pa))
		return -EINVAL;

	mem = rproc_mem_entry_init(dev, NULL, 0,
				   SUNXI_C906_RPMSG_BUF_LEN, buf_da,
				   sunxi_c906_shared_alloc,
				   sunxi_c906_shared_release,
				   "vdev%dbuffer", vdev_idx);
	if (!mem)
		return -ENOMEM;

	rproc_add_carveout(rproc, mem);
	return 0;
}

static int sunxi_c906_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	struct sunxi_c906 *priv = rproc->priv;
	struct device *dev = priv->dev;
	struct device_node *np = dev->of_node;
	struct of_phandle_iterator it;
	struct reserved_mem *rmem;
	struct rproc_mem_entry *mem;
	u64 da;
	int idx = 0, ret;
	int vdev_idx = 0;

	ret = of_phandle_iterator_init(&it, np, "memory-region", NULL, 0);
	if (ret)
		return ret;

	while (of_phandle_iterator_next(&it) == 0) {
		rmem = of_reserved_mem_lookup(it.node);
		if (!rmem) {
			dev_err(dev, "unable to acquire memory-region %s\n", it.node->name);
			return -EINVAL;
		}

		/* translate pa->da if mapping is provided */
		sunxi_c906_translate_pa(priv, rmem->base, &da);

		mem = rproc_mem_entry_init(dev, NULL,
					 (dma_addr_t)rmem->base,
					 rmem->size, da,
					 sunxi_c906_mem_alloc,
					 sunxi_c906_mem_release,
					 it.node->name);
		if (!mem)
			return -ENOMEM;

		rproc_coredump_add_segment(rproc, da, rmem->size);
		rproc_add_carveout(rproc, mem);
		idx++;
	}

	/* copy resource table (if present) into cached_table/table_ptr */
	ret = rproc_elf_load_rsc_table(rproc, fw);
	if (ret) {
		dev_warn(dev, "no resource table found for this firmware\n");
		return 0;
	}

	/* Pre-register vring carveouts so they stay inside the shared RAM window */
	if (rproc->table_ptr) {
		struct resource_table *table = rproc->table_ptr;
		int i;

		for (i = 0; i < table->num; i++) {
			int offset = table->offset[i];
			struct fw_rsc_hdr *hdr = (void *)table + offset;
			struct fw_rsc_vdev *vrsc;
			int j;

			if (hdr->type != RSC_VDEV)
				continue;

			vrsc = (void *)hdr + sizeof(*hdr);
			for (j = 0; j < vrsc->num_of_vrings; j++) {
				struct fw_rsc_vdev_vring *vr = &vrsc->vring[j];
				size_t size = PAGE_ALIGN(vring_size(vr->num, vr->align));
				phys_addr_t pa;

				if (sunxi_c906_translate_da(priv, vr->da, &pa)) {
					dev_err(dev, "failed to translate vring da 0x%x\n",
						vr->da);
					return -EINVAL;
				}

				mem = rproc_mem_entry_init(dev, NULL,
							   (dma_addr_t)pa, size, vr->da,
							   sunxi_c906_shared_alloc,
							   sunxi_c906_shared_release,
							   "vdev%dvring%d",
							   vdev_idx, j);
				if (!mem)
					return -ENOMEM;

				rproc_add_carveout(rproc, mem);
			}

			vdev_idx++;
		}
	}

	if (vdev_idx > 1)
		dev_warn(dev, "multiple vdevs not fully supported\n");

	if (vdev_idx >= 1) {
		ret = sunxi_c906_register_vdev_buffer(rproc, 0);
		if (ret)
			return ret;
	}

	return 0;
}

static void sunxi_c906_mbox_vq_work(struct work_struct *work)
{
	struct sunxi_c906_mbox *mb = container_of(work, struct sunxi_c906_mbox, vq_work);
	struct sunxi_c906 *priv = container_of(mb, struct sunxi_c906, mbox);
	struct rproc *rproc = priv->rproc;

	if (rproc_vq_interrupt(rproc, mb->vq_id) == IRQ_NONE)
		dev_dbg(priv->dev, "no message found in vq%d\n", mb->vq_id);
}

static void sunxi_c906_mbox_rx(struct mbox_client *cl, void *data)
{
	struct sunxi_c906_mbox *mb = container_of(cl, struct sunxi_c906_mbox, client);
	struct sunxi_c906 *priv = container_of(mb, struct sunxi_c906, mbox);

	if (!data)
		return;

	mb->vq_id = *(u32 *)data;
	queue_work(priv->wq ? priv->wq : system_wq, &mb->vq_work);
}

static void sunxi_c906_mbox_tx_done(struct mbox_client *cl, void *msg, int r)
{
	kfree(msg);
}

static int sunxi_c906_start(struct rproc *rproc)
{
	struct sunxi_c906 *priv = rproc->priv;
	int ret;
	u64 bootaddr = rproc->bootaddr;
	phys_addr_t boot_pa;
	u32 val;

	sunxi_c906_translate_da(priv, bootaddr, &boot_pa);
	dev_info(priv->dev, "Starting C906, bootaddr = 0x%llx (pa 0x%pa)\n",
		 bootaddr, &boot_pa);

	ret = clk_bulk_prepare_enable(priv->num_clks, priv->clks);
	if (ret) {
		dev_err(priv->dev, "failed to enable clocks: %d\n", ret);
		return ret;
	}

	/* allow access to the cfg block */
	if (priv->rst) {
		ret = reset_control_deassert(priv->rst);
		if (ret) {
			dev_err(priv->dev, "failed to deassert cfg reset: %d\n", ret);
			goto err_clk;
		}
		udelay(5);
	}

	/* release CFG bus reset and clock gate, keep other bits */
	val = readl(priv->ccu_base + SUNXI_RISCV_CFG_BGR_REG);
	val |= SUNXI_RISCV_CFG_GATE | SUNXI_RISCV_CFG_RST;
	writel(val, priv->ccu_base + SUNXI_RISCV_CFG_BGR_REG);

	/* program reset vector (SoC expects physical address) */
	writel(lower_32_bits(boot_pa), priv->cfg_base + SUNXI_C906_STA_ADDL);
	writel(upper_32_bits(boot_pa), priv->cfg_base + SUNXI_C906_STA_ADDH);

	dev_info(priv->dev, "STA_ADDL = 0x%08x, STA_ADDH = 0x%08x\n",
		 readl(priv->cfg_base + SUNXI_C906_STA_ADDL),
		 readl(priv->cfg_base + SUNXI_C906_STA_ADDH));

	/* make sure core clock mux points to the fast peri PLL (optional but safe) */
	val = readl(priv->ccu_base + SUNXI_RISCV_CLK_REG);
	val &= ~SUNXI_RISCV_CLK_SRC_MASK;
	val |= SUNXI_RISCV_CLK_SRC_PERI800M;
	writel(val, priv->ccu_base + SUNXI_RISCV_CLK_REG);

	/* finally deassert core soft reset (requires key field) */
	writel(SUNXI_RISCV_RST_KEY | SUNXI_RISCV_SOFT_RST,
	       priv->ccu_base + SUNXI_RISCV_RST_REG);

	dev_info(priv->dev, "C906 Core Soft Reset Deasserted\n");

	return 0;

err_clk:
	clk_bulk_disable_unprepare(priv->num_clks, priv->clks);
	return ret;
}

static int sunxi_c906_stop(struct rproc *rproc)
{	
	int val;

	struct sunxi_c906 *priv = rproc->priv;

	/* assert core soft reset */
	writel(SUNXI_RISCV_RST_KEY, priv->ccu_base + SUNXI_RISCV_RST_REG);

	/* assert cfg reset / gate bus */
	val = readl(priv->ccu_base + SUNXI_RISCV_CFG_BGR_REG);
	val &= ~(SUNXI_RISCV_CFG_GATE | SUNXI_RISCV_CFG_RST);
	writel(val, priv->ccu_base + SUNXI_RISCV_CFG_BGR_REG);

	if (priv->rst)
		reset_control_assert(priv->rst);

	clk_bulk_disable_unprepare(priv->num_clks, priv->clks);
	return 0;
}

static void sunxi_c906_kick(struct rproc *rproc, int vqid)
{
	struct sunxi_c906 *priv = rproc->priv;
	u32 *msg;
	int ret;
	
	if (!priv->mbox.chan) {
		dev_warn(priv->dev, "kick with no mailbox channel\n");
		return;
	}

	msg = kmalloc(sizeof(*msg), GFP_ATOMIC);
	if (!msg)
		return;

	*msg = vqid;
	ret = mbox_send_message(priv->mbox.chan, msg);
	if (ret < 0) {
		dev_err(priv->dev, "mbox kick failed: %d\n", ret);
		kfree(msg);
	}
}

static u64 sunxi_c906_get_boot_addr(struct rproc *rproc, const struct firmware *fw)
{
	return rproc_elf_get_boot_addr(rproc, fw);
}

static const struct rproc_ops sunxi_c906_ops = {
	.start		= sunxi_c906_start,
	.stop		= sunxi_c906_stop,
	.kick		= sunxi_c906_kick,
	.da_to_va	= sunxi_c906_da_to_va,
	.load		= rproc_elf_load_segments,
	.parse_fw	= sunxi_c906_parse_fw,
	.find_loaded_rsc_table = rproc_elf_find_loaded_rsc_table,
	.get_boot_addr	= sunxi_c906_get_boot_addr,
};

static int sunxi_c906_parse_mappings(struct device *dev, struct sunxi_c906 *priv)
{
	int count, i, ret;
	u32 *arr;

	count = of_property_count_elems_of_size(dev->of_node,
			"allwinner,memory-mappings", sizeof(u32) * 3);
	if (count <= 0)
		return 0; /* optional */

	priv->maps_cnt = count;
	priv->maps = devm_kcalloc(dev, count, sizeof(*priv->maps), GFP_KERNEL);
	if (!priv->maps)
		return -ENOMEM;

	arr = devm_kcalloc(dev, count * 3, sizeof(u32), GFP_KERNEL);
	if (!arr)
		return -ENOMEM;

	ret = of_property_read_u32_array(dev->of_node,
			"allwinner,memory-mappings", arr, count * 3);
	if (ret)
		return ret;

	for (i = 0; i < count; i++) {
		priv->maps[i].da = arr[i * 3];
		priv->maps[i].len = arr[i * 3 + 1];
		priv->maps[i].pa = arr[i * 3 + 2];
	}

	return 0;
}

/*
 * Expose the DA<->PA mapping to DMA so that buffers handed to the remote core
 * use device addresses (DA) while the backing memory stays at the ARM PA.
 * This makes vring buffer descriptors usable by the C906, which sees the
 * shared SRAM at 0x4100_0000 instead of the ARM's 0x4500_0000 window.
 */
static int sunxi_c906_setup_dma_ranges(struct device *dev, struct sunxi_c906 *priv)
{
	struct bus_dma_region *map;
	int i;

	if (!priv->maps_cnt)
		return 0;

	map = devm_kcalloc(dev, priv->maps_cnt + 1, sizeof(*map), GFP_KERNEL);
	if (!map)
		return -ENOMEM;

	for (i = 0; i < priv->maps_cnt; i++) {
		map[i].cpu_start = priv->maps[i].pa;
		map[i].dma_start = priv->maps[i].da;
		map[i].size = priv->maps[i].len;
	}
	/* last entry already zeroed acts as sentinel */

	dev->dma_range_map = map;

	/*
	 * Make sure the mask covers the remotecore view; the mapping keeps
	 * phys<->dma translations consistent for the rpmsg virtio device.
	 */
	dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));

	return 0;
}

static int sunxi_c906_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sunxi_c906 *priv;
	struct rproc *rproc;
	struct resource *res;
	const char *fw_name = NULL;
	int ret, i;
	static const char * const clk_ids[] = {
		"core", "bus", "cfg", "axi", "pll",
	};

	of_property_read_string(dev->of_node, "firmware-name", &fw_name);

	rproc = rproc_alloc(dev, dev_name(dev), &sunxi_c906_ops, fw_name,
			sizeof(*priv));
	if (!rproc)
		return -ENOMEM;

	rproc->has_iommu = false;
	rproc->auto_boot = of_property_read_bool(dev->of_node,
					   "allwinner,auto-boot");
	priv = rproc->priv;
	priv->dev = dev;
	priv->rproc = rproc;

	priv->wq = alloc_ordered_workqueue(dev_name(dev), WQ_MEM_RECLAIM);
	if (!priv->wq) {
		ret = -ENOMEM;
		goto free_rproc;
	}

	priv->ccu_base = devm_ioremap(dev, SUNXI_CCU_BASE, SUNXI_CCU_SIZE);
	if (!priv->ccu_base) {
		ret = -ENOMEM;
		goto destroy_wq;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->cfg_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->cfg_base)) {
		ret = PTR_ERR(priv->cfg_base);
		goto free_rproc;
	}

	priv->rst = devm_reset_control_get_optional_exclusive(dev, "cfg");
	if (IS_ERR(priv->rst)) {
		ret = PTR_ERR(priv->rst);
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
	if (ret) {
		dev_err(dev, "failed to get clocks: %d\n", ret);
		goto destroy_wq;
	}

	ret = sunxi_c906_parse_mappings(dev, priv);
	if (ret) {
		dev_err(dev, "failed to parse memory-mappings: %d\n", ret);
		goto destroy_wq;
	}

	ret = sunxi_c906_setup_dma_ranges(dev, priv);
	if (ret) {
		dev_err(dev, "failed to set dma ranges: %d\n", ret);
		goto destroy_wq;
	}

	priv->mbox.client.dev = dev;
	priv->mbox.client.rx_callback = sunxi_c906_mbox_rx;
	priv->mbox.client.tx_done = sunxi_c906_mbox_tx_done;
	priv->mbox.client.tx_block = false;
	priv->mbox.client.knows_txdone = true;

	INIT_WORK(&priv->mbox.vq_work, sunxi_c906_mbox_vq_work);

	priv->mbox.chan = mbox_request_channel_byname(&priv->mbox.client, "arm-kick");
	if (IS_ERR(priv->mbox.chan)) {
		ret = PTR_ERR(priv->mbox.chan);
		dev_err(dev, "failed to request mbox channel: %d\n", ret);
		priv->mbox.chan = NULL;
		goto destroy_wq;
	}

	platform_set_drvdata(pdev, rproc);

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "failed to add rproc: %d\n", ret);
		goto free_rproc;
	}

	dev_info(dev, "Sunxi C906 remoteproc registered\n");
	return 0;

free_rproc:
	mbox_free_channel(priv->mbox.chan);
	if (priv->wq)
		destroy_workqueue(priv->wq);
	rproc_free(rproc);
	return ret;

destroy_wq:
	if (priv->wq)
		destroy_workqueue(priv->wq);
	return ret;
}

static void sunxi_c906_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct sunxi_c906 *priv = rproc->priv;

	if (priv->mbox.chan)
		mbox_free_channel(priv->mbox.chan);

	if (priv->wq)
		destroy_workqueue(priv->wq);

	rproc_del(rproc);
	rproc_free(rproc);
}

static const struct of_device_id sunxi_c906_of_match[] = {
	{ .compatible = "allwinner,c906-rproc" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sunxi_c906_of_match);

static struct platform_driver sunxi_c906_driver = {
	.probe	= sunxi_c906_probe,
	.remove = sunxi_c906_remove,
	.driver	= {
		.name		= "sunxi-c906-rproc",
		.of_match_table = sunxi_c906_of_match,
	},
};
module_platform_driver(sunxi_c906_driver);

MODULE_DESCRIPTION("Allwinner T113/D1 C906 remote processor driver");
MODULE_AUTHOR("TaterLi");
MODULE_LICENSE("GPL v2");
