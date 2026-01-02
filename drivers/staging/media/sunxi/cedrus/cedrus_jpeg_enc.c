// SPDX-License-Identifier: GPL-2.0
/*
 * Cedrus VPU driver
 *
 * Copyright (C) 2016 Florent Revest <florent.revest@free-electrons.com>
 * Copyright (C) 2018 Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 * Copyright (C) 2018 Bootlin
 *
 * Based on https://gitorious.org/recedro/jepoc
 * Copyright (c) 2014 Manuel Braga <mul.braga@gmail.com>
 *
 * Copyright (C) 2025 starterkit.ru <info@starterkit.ru>
 */

#include <media/videobuf2-dma-contig.h>

#include "cedrus.h"
#include "cedrus_hw.h"
#include "cedrus_regs.h"

#define VE_ISP_COLOR_FORMAT_NV12	0
#define VE_AVC_TRIGGER_JPEG		BIT(16)
#define VE_AVC_STATUS_PUT_BITS_READY	BIT(9)

static void cedrus_put_bits(struct cedrus_dev *dev, u32 data, u32 nbits)
{
	u32 trigger = VE_AVC_TRIGGER_JPEG | ((nbits & 0x1f) << 8) | 0x1;

//	cedrus_wait_for(dev, VE_AVC_STATUS, VE_AVC_STATUS_PUT_BITS_READY);
	cedrus_write(dev, VE_AVC_BASIC_BITS, data);
	cedrus_write(dev, VE_AVC_TRIGGER, trigger);
}

static enum cedrus_irq_status cedrus_jpeg_irq_status(struct cedrus_ctx *ctx)
{
	struct cedrus_dev *dev = ctx->dev;
	u32 status = cedrus_read(dev, VE_AVC_STATUS);

	if (!(status & 0xf))
		return CEDRUS_IRQ_NONE;

	if (status & 0x1) {
		u32 length = cedrus_read(dev, VE_AVC_VLE_LENGTH) / 8;
		void *vaddr = vb2_plane_vaddr(ctx->dst_vb, 0);
		u32 buf_sz = vb2_plane_size(ctx->dst_vb, 0);

		/*
		 * Hardware JPEG stream lacks the EOI marker. If the buffer is
		 * CPU-accessible and large enough, append 0xffd9 manually so
		 * userspace gets a valid JPEG bitstream.
		 */
		if (vaddr && length + 2 <= buf_sz) {
			u8 *buf = vaddr;

			if (length < 2 || buf[length - 2] != 0xff ||
			    buf[length - 1] != 0xd9) {
				buf[length++] = 0xff;
				buf[length++] = 0xd9;
			}
		}
		vb2_set_plane_payload(ctx->dst_vb, 0, length);
		return CEDRUS_IRQ_OK;
	}

	return CEDRUS_IRQ_ERROR;
}

static void cedrus_jpeg_irq_clear(struct cedrus_ctx *ctx)
{
	struct cedrus_dev *dev = ctx->dev;

	u32 status = cedrus_read(dev, VE_AVC_STATUS);
	cedrus_write(dev, VE_AVC_STATUS, status);
}

static void cedrus_jpeg_irq_disable(struct cedrus_ctx *ctx)
{
	struct cedrus_dev *dev = ctx->dev;

	u32 ctrl = cedrus_read(dev, VE_AVC_CTRL);
	cedrus_write(dev, VE_AVC_CTRL, ctrl & ~0xf);
}

static void cedrus_set_jpeg_quantization(struct cedrus_dev *dev,
		struct jpegenc *c)
{
	u32 val, i;

	cedrus_write(dev, VE_AVC_SRAM_INDEX, 0);

	for(i = 0; i < JPEG_QUANT_SIZE; i++) {
		val = 0x0000ffff & (0xffff / c->hw_luma_qtable[i]);
		val |= 0x00ff0000 & (((c->hw_luma_qtable[i] + 1) / 2) << 16);
		cedrus_write(dev, VE_AVC_SRAM_DATA, val);
	}

	for(i = 0; i < JPEG_QUANT_SIZE; i++) {
		val = 0x0000ffff & (0xffff / c->hw_chroma_qtable[i]);
		val |= 0x00ff0000 & (((c->hw_chroma_qtable[i] + 1) / 2) << 16);
		cedrus_write(dev, VE_AVC_SRAM_DATA, val);
	}
}

static int cedrus_jpeg_setup(struct cedrus_ctx *ctx, struct cedrus_run *run)
{
	struct cedrus_dev *dev = ctx->dev;
	struct jpegenc *c = &ctx->codec.jpegenc;
	dma_addr_t pa_bytestream, pa_luma, pa_chroma;
	u32 i, val, sz_bytestream;

	/* Activate JPEG engine. */
	cedrus_engine_enable(ctx);

	cedrus_write(dev, VE_RESET, 0x1);
	cedrus_write(dev, VE_RESET, 0x0);

	cedrus_write(dev, VE_AVC_TRIGGER, VE_AVC_TRIGGER_JPEG);

	/* set output buffer */
	ctx->dst_vb = &run->dst->vb2_buf;
	pa_bytestream = vb2_dma_contig_plane_dma_addr(ctx->dst_vb, 0);
	sz_bytestream = vb2_plane_size(ctx->dst_vb, 0);

	cedrus_write(dev, VE_AVC_VLE_ADDR, pa_bytestream);
	cedrus_write(dev, VE_AVC_VLE_END, pa_bytestream + sz_bytestream - 1);
	cedrus_write(dev, VE_AVC_VLE_MAX, sz_bytestream * 8);
	cedrus_write(dev, VE_AVC_VLE_OFFSET, 0);

	/* write header */
	cedrus_set_jpeg_quantization(dev, c);

	cedrus_write(dev, VE_AVC_PARAM, 1 << 31);
	for (i = 0; i < JPEG_HEADER_SIZE; i++)
		cedrus_put_bits(dev, c->header[i], 8);

	val = (1 << 31) | (1 << 30) |
		(((0x400 / c->hw_chroma_qtable[0]) & 0x7ff) << 16) |
		(((0x400 / c->hw_luma_qtable[0]) & 0x7ff) << 0);
	cedrus_write(dev, VE_AVC_PARAM, val);

	/* set input buffer */
	cedrus_write(dev, VE_ISP_INPUT_STRIDE, c->mb_stride << 16);
	cedrus_write(dev, VE_ISP_INPUT_SIZE, (c->mb_width << 16) | c->mb_height);
	cedrus_write(dev, VE_ISP_CTRL, VE_ISP_COLOR_FORMAT_NV12 << 29);

	pa_luma = vb2_dma_contig_plane_dma_addr(&run->src->vb2_buf, 0);
	cedrus_write(dev, VE_ISP_INPUT_LUMA, pa_luma);

	pa_chroma = pa_luma + ctx->src_fmt.bytesperline * ctx->src_fmt.height;
	cedrus_write(dev, VE_ISP_INPUT_CHROMA, pa_chroma);

	/* clear status flags and enable interrupts */
	val = cedrus_read(dev, VE_AVC_STATUS);
	cedrus_write(dev, VE_AVC_STATUS, val);

	val = cedrus_read(dev, VE_AVC_CTRL);
	cedrus_write(dev, VE_AVC_CTRL, 0xf);

	return 0;
}

static int cedrus_jpeg_start(struct cedrus_ctx *ctx)
{
	struct jpegenc *c = &ctx->codec.jpegenc;

	c->width = ctx->src_fmt.width;
	c->height = ctx->src_fmt.height;
	c->mb_width = DIV_ROUND_UP(ctx->src_fmt.width, 16);
	c->mb_height = DIV_ROUND_UP(ctx->src_fmt.height, 16);
	c->mb_stride = ctx->src_fmt.bytesperline / 16;

	cedrus_jpeg_header_assemble(c);

	return 0;
}

static void cedrus_jpeg_trigger(struct cedrus_ctx *ctx)
{
	struct cedrus_dev *dev = ctx->dev;

	cedrus_write(dev, VE_AVC_TRIGGER, VE_AVC_TRIGGER_JPEG | 0x8);
}

struct cedrus_codec_ops cedrus_enc_ops_jpeg = {
	.irq_clear	= cedrus_jpeg_irq_clear,
	.irq_disable	= cedrus_jpeg_irq_disable,
	.irq_status	= cedrus_jpeg_irq_status,
	.start		= cedrus_jpeg_start,
	.setup		= cedrus_jpeg_setup,
	.trigger	= cedrus_jpeg_trigger,
};