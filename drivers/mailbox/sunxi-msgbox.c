// SPDX-License-Identifier: GPL-2.0
/*
 * Allwinner sun8iw20/T113 message box controller
 *
 * This is a trimmed-down variant of the vendor driver that only keeps
 * the pieces needed for AMP use (remoteproc/rpmsg).  It uses the mailbox
 * framework with txdone polling and exposes up to 8 channels between the
 * application ARM cores (local_id = 0) and the C906 co-processor.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>

#define SUNXI_MSGBOX_OFFSET(n)             (0x100 * (n))
#define SUNXI_MSGBOX_READ_IRQ_ENABLE(n)    (0x20 + SUNXI_MSGBOX_OFFSET(n))
#define SUNXI_MSGBOX_READ_IRQ_STATUS(n)    (0x24 + SUNXI_MSGBOX_OFFSET(n))
#define SUNXI_MSGBOX_WRITE_IRQ_ENABLE(n)   (0x30 + SUNXI_MSGBOX_OFFSET(n))
#define SUNXI_MSGBOX_WRITE_IRQ_STATUS(n)   (0x34 + SUNXI_MSGBOX_OFFSET(n))
#define SUNXI_MSGBOX_MSG_STATUS(n, p)      (0x60 + SUNXI_MSGBOX_OFFSET(n) + 0x4 * (p))
#define SUNXI_MSGBOX_MSG_FIFO(n, p)        (0x70 + SUNXI_MSGBOX_OFFSET(n) + 0x4 * (p))
#define SUNXI_MSGBOX_WRITE_IRQ_THRESHOLD(n, p) (0x80 + SUNXI_MSGBOX_OFFSET(n) + 0x4 * (p))

#define RD_IRQ_EN_MASK             0x1
#define RD_IRQ_EN_SHIFT(p)         ((p) * 2)

#define RD_IRQ_PEND_MASK           0x1
#define RD_IRQ_PEND_SHIFT(p)       ((p) * 2)

#define WR_IRQ_EN_MASK             0x1
#define WR_IRQ_EN_SHIFT(p)         ((p) * 2 + 1)

#define WR_IRQ_PEND_MASK           0x1
#define WR_IRQ_PEND_SHIFT(p)       ((p) * 2 + 1)

#define MSG_NUM_MASK               0xF
#define MSG_NUM_SHIFT              0

#define WR_IRQ_THR_MASK            0x3
#define WR_IRQ_THR_SHIFT           0

/*
 * Hardware description for a given SoC:
 *  processors_max   : number of CPU instances that can talk to the msgbox
 *  channels_max     : number of FIFO channels per CPU pair
 *  fifo_msg_max     : depth of each FIFO
 *  mbox_num_chans   : total exposed mailbox channels to Linux
 *  to_coef_n        : map [local_id][remote_id] -> coefficient N
 *  to_remote_id     : map [local_id][coef_n]    -> remote_id
 */
struct sunxi_msgbox_hwdata {
       int processors_max;
       int channels_max;
       int fifo_msg_max;
       int mbox_num_chans;
       int to_coef_n[4][4];
       int to_remote_id[4][4];
};

	struct sunxi_msgbox {
		struct mbox_controller controller;
		const struct sunxi_msgbox_hwdata *hwdata;
		struct platform_device *pdev;
		struct device *dev;

		struct resource *res;
		struct clk_bulk_data *clks;
		int num_clks;
		struct reset_control *reset;
		int *irq;
		int irq_cnt;
		int local_id;
		void __iomem *base_addr[0];
	};

static inline void reg_bits_set(void __iomem *reg, u32 mask, u32 shift)
{
       u32 val = readl(reg);

       val |= (mask << shift);
       writel(val, reg);
}

static inline void reg_bits_clear(void __iomem *reg, u32 mask, u32 shift)
{
       u32 val = readl(reg);

       val &= ~(mask << shift);
       writel(val, reg);
}

static inline u32 reg_bits_get(void __iomem *reg, u32 mask, u32 shift)
{
       return (readl(reg) & (mask << shift)) >> shift;
}

static inline void reg_val_update(void __iomem *reg, u32 mask, u32 shift, u32 val)
{
       u32 reg_val = readl(reg);

       reg_val &= ~(mask << shift);
       reg_val |= ((val & mask) << shift);
       writel(reg_val, reg);
}

static inline void mbox_chan_id_to_coef_n_p(const struct sunxi_msgbox *chip,
                                           int chan_id, int *coef_n, int *coef_p)
{
       *coef_n = chan_id / chip->hwdata->channels_max;
       *coef_p = chan_id % chip->hwdata->channels_max;
}

static inline void mbox_chan_to_coef_n_p(const struct sunxi_msgbox *chip,
                                        struct mbox_chan *chan,
                                        int *coef_n, int *coef_p)
{
       mbox_chan_id_to_coef_n_p(chip, chan - chan->mbox->chans, coef_n, coef_p);
}

static inline int sunxi_msgbox_coef_n(const struct sunxi_msgbox *chip,
                                     int sender_id, int receiver_id)
{
       return chip->hwdata->to_coef_n[sender_id][receiver_id];
}

static inline int sunxi_msgbox_remote_id(const struct sunxi_msgbox *chip,
                                        int local_id, int coef_n)
{
       return chip->hwdata->to_remote_id[local_id][coef_n];
}

static inline void *sunxi_msgbox_reg_base(const struct sunxi_msgbox *chip, int index)
{
       return chip->base_addr[index];
}

static inline struct sunxi_msgbox *to_sunxi_msgbox(struct mbox_chan *chan)
{
       return chan->con_priv;
}

static inline void sunxi_msgbox_set_write_irq_threshold(struct sunxi_msgbox *chip,
                                                       void __iomem *base, int n,
                                                       int p, int threshold)
{
       u32 thr_val;

       switch (threshold) {
       case 8:
               thr_val = 3;
               break;
       case 4:
               thr_val = 2;
               break;
       case 2:
               thr_val = 1;
               break;
       case 1:
       default:
               thr_val = 0;
               break;
       }

       reg_val_update(base + SUNXI_MSGBOX_WRITE_IRQ_THRESHOLD(n, p),
                      WR_IRQ_THR_MASK, WR_IRQ_THR_SHIFT, thr_val);
}

static void sunxi_msgbox_read_handler(struct sunxi_msgbox *chip, struct mbox_chan *chan,
                                     void __iomem *base, int local_n, int p)
{
       unsigned long timeout = jiffies + msecs_to_jiffies(10);
       u32 msg;
       int ret = -1;

       while (reg_bits_get(base + SUNXI_MSGBOX_MSG_STATUS(local_n, p),
                           MSG_NUM_MASK, MSG_NUM_SHIFT) &&
              time_before(jiffies, timeout)) {
               msg = readl(base + SUNXI_MSGBOX_MSG_FIFO(local_n, p));
               mbox_chan_received_data(chan, &msg);
               ret = 0;
       }

       if (ret)
               dev_err(chip->dev, "msgbox: read timeout on chan %d\n", p);

       reg_bits_set(base + SUNXI_MSGBOX_READ_IRQ_STATUS(local_n),
                    RD_IRQ_PEND_MASK, RD_IRQ_PEND_SHIFT(p));
}

static irqreturn_t sunxi_msgbox_handler(int irq, void *dev_id)
{
       struct sunxi_msgbox *chip = dev_id;
       int i;

       for (i = 0; i < chip->hwdata->mbox_num_chans; i++) {
               struct mbox_chan *chan = &chip->controller.chans[i];
               int local_n, p;
               int local_id = chip->local_id;
               int remote_id, remote_n;
               void __iomem *read_reg_base;
               void __iomem *write_reg_base;
               u32 read_irq_en, read_irq_pending;
               u32 write_irq_en, write_irq_pending;

               mbox_chan_id_to_coef_n_p(chip, i, &local_n, &p);

               remote_id = sunxi_msgbox_remote_id(chip, local_id, local_n);
               remote_n = sunxi_msgbox_coef_n(chip, remote_id, local_id);

               read_reg_base = sunxi_msgbox_reg_base(chip, local_id);
               write_reg_base = sunxi_msgbox_reg_base(chip, remote_id);

               read_irq_en = reg_bits_get(read_reg_base +
                               SUNXI_MSGBOX_READ_IRQ_ENABLE(local_n),
                               RD_IRQ_EN_MASK, RD_IRQ_EN_SHIFT(p));
               read_irq_pending = reg_bits_get(read_reg_base +
                                    SUNXI_MSGBOX_READ_IRQ_STATUS(local_n),
                                    RD_IRQ_PEND_MASK, RD_IRQ_PEND_SHIFT(p));
               write_irq_en = reg_bits_get(write_reg_base +
                                SUNXI_MSGBOX_WRITE_IRQ_ENABLE(remote_n),
                                WR_IRQ_EN_MASK, WR_IRQ_EN_SHIFT(p));
               write_irq_pending = reg_bits_get(write_reg_base +
                                     SUNXI_MSGBOX_WRITE_IRQ_STATUS(remote_n),
                                     WR_IRQ_PEND_MASK, WR_IRQ_PEND_SHIFT(p));

               if (read_irq_en && read_irq_pending)
                       sunxi_msgbox_read_handler(chip, chan, read_reg_base, local_n, p);

               /* Clear stray write IRQs if firmware left them enabled */
               if (write_irq_en && write_irq_pending)
                       reg_bits_set(write_reg_base +
                                    SUNXI_MSGBOX_WRITE_IRQ_STATUS(remote_n),
                                    WR_IRQ_PEND_MASK, WR_IRQ_PEND_SHIFT(p));
       }

       return IRQ_HANDLED;
}

static int sunxi_msgbox_startup(struct mbox_chan *chan)
{
       struct sunxi_msgbox *chip = to_sunxi_msgbox(chan);
       int local_id, remote_id, local_n, remote_n, p;
       void __iomem *read_reg_base;
       void __iomem *write_reg_base;
       unsigned long timeout = jiffies + msecs_to_jiffies(10);

       mbox_chan_to_coef_n_p(chip, chan, &local_n, &p);
       local_id = chip->local_id;
       remote_id = sunxi_msgbox_remote_id(chip, local_id, local_n);
       remote_n = sunxi_msgbox_coef_n(chip, remote_id, local_id);

       read_reg_base = sunxi_msgbox_reg_base(chip, local_id);
       write_reg_base = sunxi_msgbox_reg_base(chip, remote_id);

       while (reg_bits_get(read_reg_base + SUNXI_MSGBOX_MSG_STATUS(local_n, p),
                           MSG_NUM_MASK, MSG_NUM_SHIFT) &&
              time_before(jiffies, timeout))
               readl(read_reg_base + SUNXI_MSGBOX_MSG_FIFO(local_n, p));

       reg_bits_set(read_reg_base + SUNXI_MSGBOX_READ_IRQ_STATUS(local_n),
                    RD_IRQ_PEND_MASK, RD_IRQ_PEND_SHIFT(p));

       reg_bits_set(read_reg_base + SUNXI_MSGBOX_READ_IRQ_ENABLE(local_n),
                    RD_IRQ_EN_MASK, RD_IRQ_EN_SHIFT(p));

       reg_bits_set(write_reg_base + SUNXI_MSGBOX_WRITE_IRQ_STATUS(remote_n),
                    WR_IRQ_PEND_MASK, WR_IRQ_PEND_SHIFT(p));

       sunxi_msgbox_set_write_irq_threshold(chip, write_reg_base, remote_n, p, 1);

       dev_dbg(chip->dev, "msgbox: channel %d ready (local %d -> remote %d)\n",
               p, local_id, remote_id);
       return 0;
}

static int sunxi_msgbox_send_data(struct mbox_chan *chan, void *data)
{
       struct sunxi_msgbox *chip = to_sunxi_msgbox(chan);
       int local_id = chip->local_id;
       int local_n, remote_id, remote_n, p;
       u32 msg_num, msg;
       void __iomem *write_reg_base;

       msg = *(u32 *)data;

       mbox_chan_to_coef_n_p(chip, chan, &local_n, &p);
       remote_id = sunxi_msgbox_remote_id(chip, local_id, local_n);
       remote_n = sunxi_msgbox_coef_n(chip, remote_id, local_id);
       write_reg_base = sunxi_msgbox_reg_base(chip, remote_id);

       msg_num = reg_bits_get(write_reg_base + SUNXI_MSGBOX_MSG_STATUS(remote_n, p),
                              MSG_NUM_MASK, MSG_NUM_SHIFT);
       if (msg_num >= chip->hwdata->fifo_msg_max)
               return -EBUSY;

       writel(msg, write_reg_base + SUNXI_MSGBOX_MSG_FIFO(remote_n, p));

       return 0;
}

static void sunxi_msgbox_shutdown(struct mbox_chan *chan)
{
       struct sunxi_msgbox *chip = to_sunxi_msgbox(chan);
       int local_id, remote_id;
       int local_n, remote_n, p;
       void __iomem *read_reg_base;
       void __iomem *write_reg_base;
       unsigned long timeout = jiffies + msecs_to_jiffies(10);

       mbox_chan_to_coef_n_p(chip, chan, &local_n, &p);
       local_id = chip->local_id;
       remote_id = sunxi_msgbox_remote_id(chip, local_id, local_n);
       remote_n = sunxi_msgbox_coef_n(chip, remote_id, local_id);

       read_reg_base = sunxi_msgbox_reg_base(chip, local_id);
       write_reg_base = sunxi_msgbox_reg_base(chip, remote_id);

       reg_bits_clear(write_reg_base + SUNXI_MSGBOX_WRITE_IRQ_ENABLE(remote_n),
                      WR_IRQ_EN_MASK, WR_IRQ_EN_SHIFT(p));
       reg_bits_set(write_reg_base + SUNXI_MSGBOX_WRITE_IRQ_STATUS(remote_n),
                    WR_IRQ_PEND_MASK, WR_IRQ_PEND_SHIFT(p));

       do {
               while (reg_bits_get(read_reg_base + SUNXI_MSGBOX_MSG_STATUS(local_n, p),
                                   MSG_NUM_MASK, MSG_NUM_SHIFT) &&
                      time_before(jiffies, timeout))
                       readl(read_reg_base + SUNXI_MSGBOX_MSG_FIFO(local_n, p));

               reg_bits_clear(read_reg_base + SUNXI_MSGBOX_READ_IRQ_ENABLE(local_n),
                              RD_IRQ_EN_MASK, RD_IRQ_EN_SHIFT(p));
               reg_bits_set(read_reg_base + SUNXI_MSGBOX_READ_IRQ_STATUS(local_n),
                            RD_IRQ_PEND_MASK, RD_IRQ_PEND_SHIFT(p));
       } while (reg_bits_get(read_reg_base + SUNXI_MSGBOX_READ_IRQ_STATUS(local_n),
                             RD_IRQ_PEND_MASK, RD_IRQ_PEND_SHIFT(p)) &&
                time_before(jiffies, timeout));

       dev_dbg(chip->dev, "msgbox: channel %d shutdown\n", p);
}

static bool sunxi_msgbox_last_tx_done(struct mbox_chan *chan)
{
       struct sunxi_msgbox *chip = to_sunxi_msgbox(chan);
       int local_id = chip->local_id;
       int local_n, remote_id, remote_n, p;
       void __iomem *status_reg;
       u32 msg_num;

       mbox_chan_to_coef_n_p(chip, chan, &local_n, &p);
       remote_id = sunxi_msgbox_remote_id(chip, local_id, local_n);
       remote_n = sunxi_msgbox_coef_n(chip, remote_id, local_id);

       status_reg = sunxi_msgbox_reg_base(chip, remote_id) +
                    SUNXI_MSGBOX_MSG_STATUS(remote_n, p);
       msg_num = reg_bits_get(status_reg, MSG_NUM_MASK, MSG_NUM_SHIFT);

       return msg_num < chip->hwdata->fifo_msg_max;
}

static bool sunxi_msgbox_peek_data(struct mbox_chan *chan)
{
       struct sunxi_msgbox *chip = to_sunxi_msgbox(chan);
       int local_n, p;
       u32 msg_num;

       mbox_chan_to_coef_n_p(chip, chan, &local_n, &p);
       msg_num = reg_bits_get(sunxi_msgbox_reg_base(chip, chip->local_id) +
                              SUNXI_MSGBOX_MSG_STATUS(local_n, p),
                              MSG_NUM_MASK, MSG_NUM_SHIFT);

       return !!msg_num;
}

static struct mbox_chan *sunxi_mbox_xlate(struct mbox_controller *mbox,
                                         const struct of_phandle_args *sp)
{
       int ind = sp->args[0];

       if (ind >= mbox->num_chans)
               return ERR_PTR(-EINVAL);

       return &mbox->chans[ind];
}

static const struct mbox_chan_ops sunxi_msgbox_chan_ops = {
       .startup      = sunxi_msgbox_startup,
       .send_data    = sunxi_msgbox_send_data,
       .shutdown     = sunxi_msgbox_shutdown,
       .last_tx_done = sunxi_msgbox_last_tx_done,
       .peek_data    = sunxi_msgbox_peek_data,
};

/* sun8iw20 / T113 mapping: ARM(0) <-> DSP(1) <-> C906(2) */
static const struct sunxi_msgbox_hwdata sun8iw20_hwdata = {
       .processors_max = 3,
       .channels_max = 4,
       .fifo_msg_max = 8,
       .mbox_num_chans = 8,
       .to_coef_n = {
               {-1, 0, 1, -1},
               {0, -1, 1, -1},
               {0, 1, -1, -1},
               {-1, -1, -1, -1}
       },
       .to_remote_id = {
               {1, 2, -1, -1},
               {0, 2, -1, -1},
               {0, 1, -1, -1},
               {-1, -1, -1, -1}
       }
};

static const struct of_device_id sunxi_msgbox_of_match[] = {
       { .compatible = "allwinner,sun8iw20-msgbox", .data = &sun8iw20_hwdata },
       { .compatible = "allwinner,sun20i-d1-msgbox", .data = &sun8iw20_hwdata },
       { .compatible = "allwinner,sun8i-t113-msgbox", .data = &sun8iw20_hwdata },
       { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sunxi_msgbox_of_match);

static void sunxi_msgbox_disable_irq(struct sunxi_msgbox *chip)
{
       int i, j;

       for (i = 0; i < chip->hwdata->processors_max - 1; i++) {
               int local_n = i;
               int local_id = chip->local_id;
               int remote_id = sunxi_msgbox_remote_id(chip, local_id, local_n);
               int remote_n = sunxi_msgbox_coef_n(chip, remote_id, local_id);
               void __iomem *read_reg_base = sunxi_msgbox_reg_base(chip, local_id);
               void __iomem *write_reg_base = sunxi_msgbox_reg_base(chip, remote_id);
               void __iomem *read_irq_en_reg = read_reg_base +
                       SUNXI_MSGBOX_READ_IRQ_ENABLE(local_n);
               void __iomem *write_irq_en_reg = write_reg_base +
                       SUNXI_MSGBOX_WRITE_IRQ_ENABLE(remote_n);
               u32 read_irq_en_value = readl(read_irq_en_reg);
               u32 write_irq_en_value = readl(write_irq_en_reg);

               for (j = 0; j < chip->hwdata->channels_max; ++j) {
                       read_irq_en_value &= ~(RD_IRQ_EN_MASK << RD_IRQ_EN_SHIFT(j));
                       write_irq_en_value &= ~(WR_IRQ_EN_MASK << WR_IRQ_EN_SHIFT(j));
               }

               writel(read_irq_en_value, read_irq_en_reg);
               writel(write_irq_en_value, write_irq_en_reg);
       }
}

static int sunxi_msgbox_resource_get(struct sunxi_msgbox *chip)
{
	int ret, i;

	chip->num_clks = devm_clk_bulk_get_all(chip->dev, &chip->clks);
	if (chip->num_clks < 0)
		return chip->num_clks;
	if (!chip->num_clks)
		return dev_err_probe(chip->dev, -ENOENT, "no msgbox clocks defined\n");

	chip->reset = devm_reset_control_array_get_optional_exclusive(chip->dev);
	if (IS_ERR(chip->reset))
		return PTR_ERR(chip->reset);

	for (i = 0; i < chip->hwdata->processors_max; i++) {
		chip->res = platform_get_resource(chip->pdev, IORESOURCE_MEM, i);
		if (!chip->res)
			return -ENODEV;

               chip->base_addr[i] = devm_ioremap_resource(chip->dev, chip->res);
               if (IS_ERR(chip->base_addr[i]))
                       return PTR_ERR(chip->base_addr[i]);
       }

       ret = of_property_read_u32(chip->dev->of_node, "local_id", &chip->local_id);
       if (ret)
               return ret;

       chip->irq_cnt = of_irq_count(chip->dev->of_node);
       chip->irq = devm_kcalloc(chip->dev, chip->irq_cnt, sizeof(int), GFP_KERNEL);
       if (!chip->irq)
               return -ENOMEM;

       for (i = 0; i < chip->irq_cnt; i++) {
               ret = of_irq_get(chip->dev->of_node, i);
               if (ret <= 0)
                       return ret ? ret : -EINVAL;
               chip->irq[i] = ret;
       }

       for (i = 0; i < chip->irq_cnt; i++) {
               ret = devm_request_irq(&chip->pdev->dev, chip->irq[i],
                                     sunxi_msgbox_handler, 0,
                                     dev_name(chip->dev), chip);
               if (ret)
                       return ret;
       }

       return 0;
}

static int sunxi_msgbox_hw_init(struct sunxi_msgbox *chip)
{
	int ret;

	if (chip->reset) {
		ret = reset_control_deassert(chip->reset);
		if (ret)
			return ret;
	}

	ret = clk_bulk_prepare_enable(chip->num_clks, chip->clks);
	if (ret)
		goto err_reset;

	sunxi_msgbox_disable_irq(chip);

	return 0;

err_reset:
	if (chip->reset)
		reset_control_assert(chip->reset);
	return ret;
}

static void sunxi_msgbox_hw_deinit(struct sunxi_msgbox *chip)
{
	clk_bulk_disable_unprepare(chip->num_clks, chip->clks);
}

static int sunxi_msgbox_probe(struct platform_device *pdev)
{
       const struct sunxi_msgbox_hwdata *priv_data;
       struct mbox_chan *chans;
       struct sunxi_msgbox *chip;
       int ret, i, processors_max_tmp;

       priv_data = of_device_get_match_data(&pdev->dev);
       if (!priv_data)
               return -ENODEV;

       processors_max_tmp = priv_data->processors_max;
       chip = devm_kzalloc(&pdev->dev,
                           sizeof(*chip) + sizeof(void __iomem *) * processors_max_tmp,
                           GFP_KERNEL);
       if (!chip)
               return -ENOMEM;

       chip->hwdata = priv_data;
       chip->pdev = pdev;
       chip->dev = &pdev->dev;

       chans = devm_kcalloc(chip->dev, chip->hwdata->mbox_num_chans,
                            sizeof(*chans), GFP_KERNEL);
       if (!chans)
               return -ENOMEM;

       for (i = 0; i < chip->hwdata->mbox_num_chans; i++)
               chans[i].con_priv = chip;

       ret = sunxi_msgbox_resource_get(chip);
       if (ret)
               return ret;

       ret = sunxi_msgbox_hw_init(chip);
       if (ret)
               goto err_hw;

       chip->controller.dev            = chip->dev;
       chip->controller.ops            = &sunxi_msgbox_chan_ops;
       chip->controller.chans          = chans;
       chip->controller.num_chans      = chip->hwdata->mbox_num_chans;
       chip->controller.of_xlate       = sunxi_mbox_xlate;
       chip->controller.txdone_irq     = false;
       chip->controller.txdone_poll    = true;
       chip->controller.txpoll_period  = 5; /* ms */

       platform_set_drvdata(pdev, chip);

       ret = devm_mbox_controller_register(chip->dev, &chip->controller);
       if (ret)
               goto err_clk;

       dev_info(chip->dev, "sunxi msgbox registered (local_id=%d)\n", chip->local_id);
       return 0;

err_clk:
       sunxi_msgbox_hw_deinit(chip);
err_hw:
       return ret;
}

static void sunxi_msgbox_remove(struct platform_device *pdev)
{
	struct sunxi_msgbox *chip = platform_get_drvdata(pdev);

	sunxi_msgbox_hw_deinit(chip);
}

static struct platform_driver sunxi_msgbox_driver = {
       .probe  = sunxi_msgbox_probe,
       .remove = sunxi_msgbox_remove,
       .driver = {
               .name           = "sunxi-msgbox",
               .of_match_table = sunxi_msgbox_of_match,
       },
};
module_platform_driver(sunxi_msgbox_driver);

MODULE_DESCRIPTION("Allwinner sun8iw20/T113 Message Box Controller");
MODULE_AUTHOR("Codex AI (ported from Allwinner vendor driver)");
MODULE_LICENSE("GPL");
