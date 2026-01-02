// SPDX-License-Identifier: GPL-2.0
/*
 * Cedrus VPU driver
 *
 * Copyright (C) 2016 Florent Revest <florent.revest@free-electrons.com>
 * Copyright (C) 2018 Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 * Copyright (C) 2018 Bootlin
 *
 * Based on https://github.com/jemk/cedrus
 * Copyright (c) 2014-2015 Jens Kuske <jenskuske@gmail.com>
 *
 * Copyright (C) 2025 starterkit.ru <info@starterkit.ru>
 */

#include <media/videobuf2-dma-contig.h>

#include "cedrus.h"
#include "cedrus_hw.h"
#include "cedrus_regs.h"

#define VE_ISP_COLOR_FORMAT_NV12    0
#define VE_AVC_STATUS_PUT_BITS_READY	BIT(9)

static void put_bits(struct cedrus_dev *dev, u32 data, u32 nbits)
{
	cedrus_wait_for(dev, VE_AVC_STATUS, VE_AVC_STATUS_PUT_BITS_READY);

	cedrus_write(dev, VE_AVC_BASIC_BITS, data);
	cedrus_write(dev, VE_AVC_TRIGGER, ((nbits & 0x1f) << 8) | 0x1);
}

static void put_ue(struct cedrus_dev *dev, u32 x)
{
	x++;
	put_bits(dev, x, (32 - __builtin_clz(x)) * 2 - 1);
}

static void put_se(struct cedrus_dev *dev, int x)
{
	x = 2 * x - 1;
	x ^= (x >> 31);
	put_ue(dev, x);
}

static void put_start_code(struct cedrus_dev *dev, u32 nal_ref_idc, u32 nal_unit_type)
{
	uint32_t tmp = cedrus_read(dev, VE_AVC_PARAM);

	/* disable emulation_prevention_three_byte */
	cedrus_write(dev, VE_AVC_PARAM, tmp | (0x1 << 31));

	put_bits(dev, 0, 24);
	put_bits(dev, 0x100 | (nal_ref_idc << 5) | (nal_unit_type << 0), 16);

	cedrus_write(dev, VE_AVC_PARAM, tmp);
}

static void put_rbsp_trailing_bits(struct cedrus_dev *dev)
{
	unsigned int cur_bs_len = cedrus_read(dev, VE_AVC_VLE_LENGTH);

	int num_zero_bits = 8 - ((cur_bs_len + 1) & 0x7);
	put_bits(dev, 1 << num_zero_bits, num_zero_bits + 1);
}

static void put_seq_parameter_set(struct cedrus_dev *dev, struct h264enc *c)
{
	put_start_code(dev, 3, 7);

	put_bits(dev, /* profile_idc = */ 77, 8);
	put_bits(dev, /* constraints = */  BIT(6), 8);
	put_bits(dev, /* level_idc =   */ 41, 8);

	put_ue(dev, /* seq_parameter_set_id = */ 0);

	put_ue(dev, /* log2_max_frame_num_minus4 = */ 0);
	put_ue(dev, /* pic_order_cnt_type = */ 2);

	put_ue(dev, /* max_num_ref_frames = */ 1);
	put_bits(dev, /* gaps_in_frame_num_value_allowed_flag = */ 0, 1);

	put_ue(dev, c->mb_width - 1);
	put_ue(dev, c->mb_height - 1);

	put_bits(dev, /* frame_mbs_only_flag = */ 1, 1);

	put_bits(dev, /* direct_8x8_inference_flag = */ 0, 1);

	unsigned int frame_cropping_flag = c->crop_right || c->crop_bottom;
	put_bits(dev, frame_cropping_flag, 1);
	if (frame_cropping_flag) {
		put_ue(dev, 0);
		put_ue(dev, c->crop_right);
		put_ue(dev, 0);
		put_ue(dev, c->crop_bottom);
	}

	put_bits(dev, /* vui_parameters_present_flag = */ 0, 1);

	put_rbsp_trailing_bits(dev);
}

static void put_pic_parameter_set(struct cedrus_dev *dev, struct h264enc *c)
{
	put_start_code(dev, 3, 8);

	put_ue(dev, /* pic_parameter_set_id = */ 0);
	put_ue(dev, /* seq_parameter_set_id = */ 0);

	put_bits(dev, /* entropy_coding_mode_flag  CABAC = */ 1, 1);

	put_bits(dev, /* bottom_field_pic_order_in_frame_present_flag = */ 0, 1);
	put_ue(dev, /* num_slice_groups_minus1 = */ 0);

	put_ue(dev, /* num_ref_idx_l0_default_active_minus1 = */ 0);
	put_ue(dev, /* num_ref_idx_l1_default_active_minus1 = */ 0);

	put_bits(dev, /* weighted_pred_flag = */ 0, 1);
	put_bits(dev, /* weighted_bipred_idc = */ 0, 2);

	put_se(dev, (int)c->pic_init_qp - 26);
	put_se(dev, (int)c->pic_init_qp - 26);
	put_se(dev, /* chroma_qp_index_offset = */ 4);

	put_bits(dev, /* deblocking_filter_control_present_flag = */ 1, 1);
	put_bits(dev, /* constrained_intra_pred_flag = */ 0, 1);
	put_bits(dev, /* redundant_pic_cnt_present_flag = */ 0, 1);

	put_rbsp_trailing_bits(dev);
}

static void put_slice_header(struct cedrus_dev *dev, struct h264enc *c)
{
	if (c->current_slice_type == SLICE_I)
		put_start_code(dev, 3, 5);
	else
		put_start_code(dev, 2, 1);

	put_ue(dev, /* first_mb_in_slice = */ 0);
	put_ue(dev, c->current_slice_type);
	put_ue(dev, /* pic_parameter_set_id = */ 0);

	put_bits(dev, c->current_frame_num & 0xf, 4);

	if (c->current_slice_type == SLICE_I)
		put_ue(dev, /* idr_pic_id = */ 0);

	if (c->current_slice_type == SLICE_P) {
		put_bits(dev, /* num_ref_idx_active_override_flag = */ 0, 1);
		put_bits(dev, /* ref_pic_list_modification_flag_l0 = */ 0, 1);
		put_bits(dev, /* adaptive_ref_pic_marking_mode_flag = */ 0, 1);
		if (/* CABAC */ 1)
			put_ue(dev, /* cabac_init_idc = */ 0);
	}

	if (c->current_slice_type == SLICE_I) {
		put_bits(dev, /* no_output_of_prior_pics_flag = */ 0, 1);
		put_bits(dev, /* long_term_reference_flag = */ 0, 1);
	}

	put_se(dev, /* slice_qp_delta = */ 0);

	put_ue(dev, /* disable_deblocking_filter_idc = */ 0);
	put_se(dev, /* slice_alpha_c0_offset_div2 = */ 0);
	put_se(dev, /* slice_beta_offset_div2 = */ 0);
}

static enum cedrus_irq_status cedrus_avc_irq_status(struct cedrus_ctx *ctx)
{
	struct cedrus_dev *dev = ctx->dev;
	u32 status = cedrus_read(dev, VE_AVC_STATUS);

	if (!(status & 0xf))
		return CEDRUS_IRQ_NONE;

	if (status & 0x1) {
		u32 length = cedrus_read(dev, VE_AVC_VLE_LENGTH) / 8;

		vb2_set_plane_payload(ctx->dst_vb, 0, length);
		return CEDRUS_IRQ_OK;
	}

	return CEDRUS_IRQ_ERROR;
}

static void cedrus_avc_irq_clear(struct cedrus_ctx *ctx)
{
	struct cedrus_dev *dev = ctx->dev;

	u32 status = cedrus_read(dev, VE_AVC_STATUS);
	cedrus_write(dev, VE_AVC_STATUS, status);
}

static void cedrus_avc_irq_disable(struct cedrus_ctx *ctx)
{
	struct cedrus_dev *dev = ctx->dev;

	u32 ctrl = cedrus_read(dev, VE_AVC_CTRL);
	cedrus_write(dev, VE_AVC_CTRL, ctrl & ~0xf);
}

static void cedrus_avc_trigger(struct cedrus_ctx *ctx)
{
	struct cedrus_dev *dev = ctx->dev;

	cedrus_write(dev, VE_AVC_TRIGGER, 0x8);
}

static int cedrus_avc_setup(struct cedrus_ctx *ctx, struct cedrus_run *run)
{
	struct cedrus_dev *dev = ctx->dev;
	struct h264enc *c = &ctx->codec.h264enc;
	struct h264enc_ref_pic *ref_pic;
	dma_addr_t pa_bytestream, pa_luma, pa_chroma;
	u32 sz_bytestream, val;

	c->current_slice_type = c->current_frame_num ? SLICE_P : SLICE_I;

	/* Activate AVC engine. */
	cedrus_engine_enable(ctx);

	cedrus_write(dev, VE_RESET, 0x1);
	cedrus_write(dev, VE_RESET, 0x0);

	/* set output buffer */
	ctx->dst_vb = &run->dst->vb2_buf;
	pa_bytestream = vb2_dma_contig_plane_dma_addr(ctx->dst_vb, 0);
	sz_bytestream = vb2_plane_size(ctx->dst_vb, 0);

	cedrus_write(dev, VE_AVC_VLE_ADDR, pa_bytestream);
	cedrus_write(dev, VE_AVC_VLE_END, pa_bytestream + sz_bytestream - 1);
	cedrus_write(dev, VE_AVC_VLE_MAX, sz_bytestream * 8);
	cedrus_write(dev, VE_AVC_VLE_OFFSET, 0);

	/* write headers */
	if (c->write_sps_pps) {
		put_seq_parameter_set(dev, c);
		put_pic_parameter_set(dev, c);
		c->write_sps_pps = 0;
	}
	put_slice_header(dev, c);

	/* set input buffer */
	cedrus_write(dev, VE_ISP_INPUT_STRIDE, c->mb_stride << 16);
	cedrus_write(dev, VE_ISP_INPUT_SIZE, (c->mb_width << 16) | c->mb_height);
	cedrus_write(dev, VE_ISP_CTRL, VE_ISP_COLOR_FORMAT_NV12 << 29);

	pa_luma = vb2_dma_contig_plane_dma_addr(&run->src->vb2_buf, 0);
	cedrus_write(dev, VE_ISP_INPUT_LUMA, pa_luma);

	pa_chroma = pa_luma + ctx->src_fmt.bytesperline * ctx->src_fmt.height;
	cedrus_write(dev, VE_ISP_INPUT_CHROMA, pa_chroma);

	/* set reconstruction buffers */
	ref_pic = &c->ref_picture[c->current_frame_num % 2];
	cedrus_write(dev, VE_AVC_REC_LUMA, ref_pic->pa_luma_buffer);
	cedrus_write(dev, VE_AVC_REC_CHROMA, ref_pic->pa_chroma_buffer);
	cedrus_write(dev, VE_AVC_REC_SLUMA, ref_pic->pa_extra_buffer);

	/* set reference buffers */
	if (c->current_slice_type != SLICE_I) {
		ref_pic = &c->ref_picture[(c->current_frame_num + 1) % 2];
		cedrus_write(dev, VE_AVC_REF_LUMA, ref_pic->pa_luma_buffer);
		cedrus_write(dev, VE_AVC_REF_CHROMA, ref_pic->pa_chroma_buffer);
		cedrus_write(dev, VE_AVC_REF_SLUMA, ref_pic->pa_extra_buffer);
	}

	/* set unknown purpose buffers */
	cedrus_write(dev, VE_AVC_MB_INFO, c->pa_extra_buffer_line);
	cedrus_write(dev, VE_AVC_UNK_BUF, c->pa_extra_buffer_frame);

	/* set encoding parameters */
	val = 0x0;
	if (/* CABAC = */ 1)
		val |= 0x100;
	if (c->current_slice_type == SLICE_P)
		val |= 0x10;
	cedrus_write(dev, VE_AVC_PARAM, val);

	val = (4 << 16) | (c->pic_init_qp << 8) | c->pic_init_qp;
	cedrus_write(dev, VE_AVC_QP, val);

	cedrus_write(dev, VE_AVC_MOTION_EST, 0x00010004 /* 0x00000104 */);

	/* next frame */
	c->current_frame_num++;
	if (c->current_frame_num >= c->keyframe_interval)
		c->current_frame_num = 0;

	/* clear status flags and enable interrupts */
	val = cedrus_read(dev, VE_AVC_STATUS);
	cedrus_write(dev, VE_AVC_STATUS, val);

	val = cedrus_read(dev, VE_AVC_CTRL);
	cedrus_write(dev, VE_AVC_CTRL, val | 0xf);

	return 0;
}

static void cedrus_avc_stop(struct cedrus_ctx *ctx)
{
	struct cedrus_dev *dev = ctx->dev;
	struct h264enc *c = &ctx->codec.h264enc;
	u32 i;

	for (i = 0; i < 2; i++) {
		if (c->ref_picture[i].va_luma_buffer != NULL)
			dma_free_coherent(dev->dev, c->sz_luma_buffer,
				c->ref_picture[i].va_luma_buffer,
				c->ref_picture[i].pa_luma_buffer);

		if (c->ref_picture[i].va_extra_buffer != NULL)
			dma_free_coherent(dev->dev, c->sz_extra_buffer,
				c->ref_picture[i].va_extra_buffer,
				c->ref_picture[i].pa_extra_buffer);

		c->ref_picture[i].va_luma_buffer = NULL;
		c->ref_picture[i].va_extra_buffer = NULL;
	}

	if (c->va_extra_buffer_frame != NULL)
		dma_free_coherent(dev->dev, c->sz_extra_buffer_frame,
				c->va_extra_buffer_frame,
				c->pa_extra_buffer_frame);

	if (c->va_extra_buffer_line != NULL)
		dma_free_coherent(dev->dev, c->sz_extra_buffer_line,
				c->va_extra_buffer_line,
				c->pa_extra_buffer_line);

	c->va_extra_buffer_frame = NULL;
	c->va_extra_buffer_line = NULL;
}

static int cedrus_avc_start(struct cedrus_ctx *ctx)
{
	struct cedrus_dev *dev = ctx->dev;
	struct h264enc *c = &ctx->codec.h264enc;
	u32 width = ctx->src_fmt.width;
	u32 height = ctx->src_fmt.height;
	u32 stride = ctx->src_fmt.bytesperline;
	u32 i, luma_size, chroma_size;

	c->mb_width = DIV_ROUND_UP(width, 16);
	c->mb_height = DIV_ROUND_UP(height, 16);
	/* XXX: cedar rounds down, not up here. */
	c->mb_stride = stride / 16;

	c->crop_right = (c->mb_width * 16 - width) / 2;
	c->crop_bottom = (c->mb_height * 16 - height) / 2;

	c->write_sps_pps = 1;
	c->current_frame_num = 0;

	luma_size = ALIGN(c->mb_width * 16, 32) * ALIGN(c->mb_height * 16, 32);
	chroma_size = ALIGN(c->mb_width * 16, 32) * ALIGN(c->mb_height * 8, 32);

	c->sz_luma_buffer = luma_size + chroma_size;
	c->sz_extra_buffer = luma_size / 4;
	c->sz_extra_buffer_frame = ALIGN(c->mb_width, 4) * c->mb_height * 8;
	c->sz_extra_buffer_line = c->mb_width * 32;

	c->ref_picture[0].va_luma_buffer = NULL;
	c->ref_picture[0].va_extra_buffer = NULL;
	c->ref_picture[1].va_luma_buffer = NULL;
	c->ref_picture[1].va_extra_buffer = NULL;
	c->va_extra_buffer_frame = NULL;
	c->va_extra_buffer_line = NULL;

	for (i = 0; i < 2; i++) {
		c->ref_picture[i].va_luma_buffer = dma_alloc_coherent(dev->dev,
				c->sz_luma_buffer,
				&c->ref_picture[i].pa_luma_buffer,
				GFP_KERNEL);
		if (c->ref_picture[i].va_luma_buffer == NULL)
			goto mem_err;

		c->ref_picture[i].va_extra_buffer = dma_alloc_coherent(dev->dev,
				c->sz_extra_buffer,
				&c->ref_picture[i].pa_extra_buffer,
				GFP_KERNEL);
		if (c->ref_picture[i].va_extra_buffer == NULL)
			goto mem_err;

		c->ref_picture[i].va_chroma_buffer =
				c->ref_picture[i].va_luma_buffer + luma_size;

		c->ref_picture[i].pa_chroma_buffer =
				c->ref_picture[i].pa_luma_buffer + luma_size;
	}

	c->va_extra_buffer_frame = dma_alloc_coherent(dev->dev,
				c->sz_extra_buffer_frame,
				&c->pa_extra_buffer_frame,
				GFP_KERNEL);
	if (c->va_extra_buffer_frame == NULL)
		goto mem_err;

	c->va_extra_buffer_line = dma_alloc_coherent(dev->dev,
				c->sz_extra_buffer_line,
				&c->pa_extra_buffer_line,
				GFP_KERNEL);
	if (c->va_extra_buffer_line == NULL)
		goto mem_err;

	return 0;

mem_err:
	/* Clean up any partially allocated buffers */
	for (i = 0; i < 2; i++) {
		if (c->ref_picture[i].va_luma_buffer != NULL)
			dma_free_coherent(dev->dev, c->sz_luma_buffer,
					c->ref_picture[i].va_luma_buffer,
					c->ref_picture[i].pa_luma_buffer);

		if (c->ref_picture[i].va_extra_buffer != NULL)
			dma_free_coherent(dev->dev, c->sz_extra_buffer,
					c->ref_picture[i].va_extra_buffer,
					c->ref_picture[i].pa_extra_buffer);
	}

	if (c->va_extra_buffer_frame != NULL)
		dma_free_coherent(dev->dev, c->sz_extra_buffer_frame,
				c->va_extra_buffer_frame,
				c->pa_extra_buffer_frame);

	if (c->va_extra_buffer_line != NULL)
		dma_free_coherent(dev->dev, c->sz_extra_buffer_line,
				c->va_extra_buffer_line,
				c->pa_extra_buffer_line);

	return -ENOMEM;
}

struct cedrus_codec_ops cedrus_enc_ops_h264 = {
	.irq_clear	= cedrus_avc_irq_clear,
	.irq_disable	= cedrus_avc_irq_disable,
	.irq_status	= cedrus_avc_irq_status,
	.setup		= cedrus_avc_setup,
	.start		= cedrus_avc_start,
	.stop		= cedrus_avc_stop,
	.trigger	= cedrus_avc_trigger,
};