// SPDX-License-Identifier: GPL-2.0
/*
 * Cedrus VPU driver
 *
 * Copyright (C) 2016 Florent Revest <florent.revest@free-electrons.com>
 * Copyright (C) 2018 Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 * Copyright (C) 2018 Bootlin
 *
 * Based on the vim2m driver, that is:
 *
 * Copyright (c) 2009-2010 Samsung Electronics Co., Ltd.
 * Pawel Osciak, <pawel@osciak.com>
 * Marek Szyprowski, <m.szyprowski@samsung.com>
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/iommu.h>
#include <linux/reset.h>
#include <linux/dma-mapping.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-mem2mem.h>

#include "cedrus.h"
#include "cedrus_video.h"
#include "cedrus_codec.h"
#include "cedrus_hw.h"

static int cedrus_try_ctrl(struct v4l2_ctrl *ctrl)
{
	if (ctrl->id == V4L2_CID_STATELESS_H264_SPS) {
		const struct v4l2_ctrl_h264_sps *sps = ctrl->p_new.p_h264_sps;

		if (sps->chroma_format_idc != 1)
			/* Only 4:2:0 is supported */
			return -EINVAL;
		if (sps->bit_depth_luma_minus8 != sps->bit_depth_chroma_minus8)
			/* Luma and chroma bit depth mismatch */
			return -EINVAL;
		if (sps->bit_depth_luma_minus8 != 0)
			/* Only 8-bit is supported */
			return -EINVAL;
	} else if (ctrl->id == V4L2_CID_STATELESS_HEVC_SPS) {
		const struct v4l2_ctrl_hevc_sps *sps = ctrl->p_new.p_hevc_sps;
		struct cedrus_ctx *ctx = container_of(ctrl->handler, struct cedrus_ctx, hdl);
		unsigned int bit_depth, max_depth;
		struct vb2_queue *vq;

		if (sps->chroma_format_idc != 1)
			/* Only 4:2:0 is supported */
			return -EINVAL;

		bit_depth = max(sps->bit_depth_luma_minus8,
				sps->bit_depth_chroma_minus8) + 8;

		if (cedrus_is_capable(ctx, CEDRUS_CAPABILITY_H265_10_DEC))
			max_depth = 10;
		else
			max_depth = 8;

		if (bit_depth > max_depth)
			return -EINVAL;

		vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
				     V4L2_BUF_TYPE_VIDEO_CAPTURE);

		/*
		 * Bit depth can't be higher than currently set once
		 * buffers are allocated.
		 */
		if (vb2_is_busy(vq)) {
			if (ctx->bit_depth < bit_depth)
				return -EINVAL;
		} else {
			ctx->bit_depth = bit_depth;
			cedrus_reset_cap_format(ctx);
		}
	}

	return 0;
}

static int cedrus_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct cedrus_ctx *ctx = container_of(ctrl->handler, struct cedrus_ctx, hdl);

	switch (ctrl->id) {
	case V4L2_CID_JPEG_COMPRESSION_QUALITY:
		ctx->codec.jpegenc.jpeg_quality = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_I_PERIOD:
		ctx->codec.h264enc.keyframe_interval = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MAX_QP:
		ctx->codec.h264enc.pic_init_qp = ctrl->val;
		break;
	}

	return 0;
}

static const struct v4l2_ctrl_ops cedrus_ctrl_ops = {
	.try_ctrl = cedrus_try_ctrl,
	.s_ctrl = cedrus_s_ctrl,
};

static const struct cedrus_control cedrus_controls[] = {
	{
		.cfg = {
			.id	= V4L2_CID_STATELESS_MPEG2_SEQUENCE,
		},
		.capabilities	= CEDRUS_CAPABILITY_MPEG2_DEC,
	},
	{
		.cfg = {
			.id	= V4L2_CID_STATELESS_MPEG2_PICTURE,
		},
		.capabilities	= CEDRUS_CAPABILITY_MPEG2_DEC,
	},
	{
		.cfg = {
			.id	= V4L2_CID_STATELESS_MPEG2_QUANTISATION,
		},
		.capabilities	= CEDRUS_CAPABILITY_MPEG2_DEC,
	},
	{
		.cfg = {
			.id	= V4L2_CID_STATELESS_H264_DECODE_PARAMS,
		},
		.capabilities	= CEDRUS_CAPABILITY_H264_DEC,
	},
	{
		.cfg = {
			.id	= V4L2_CID_STATELESS_H264_SLICE_PARAMS,
		},
		.capabilities	= CEDRUS_CAPABILITY_H264_DEC,
	},
	{
		.cfg = {
			.id	= V4L2_CID_STATELESS_H264_SPS,
			.ops	= &cedrus_ctrl_ops,
		},
		.capabilities	= CEDRUS_CAPABILITY_H264_DEC,
	},
	{
		.cfg = {
			.id	= V4L2_CID_STATELESS_H264_PPS,
		},
		.capabilities	= CEDRUS_CAPABILITY_H264_DEC,
	},
	{
		.cfg = {
			.id	= V4L2_CID_STATELESS_H264_SCALING_MATRIX,
		},
		.capabilities	= CEDRUS_CAPABILITY_H264_DEC,
	},
	{
		.cfg = {
			.id	= V4L2_CID_STATELESS_H264_PRED_WEIGHTS,
		},
		.capabilities	= CEDRUS_CAPABILITY_H264_DEC,
	},
	{
		.cfg = {
			.id	= V4L2_CID_STATELESS_H264_DECODE_MODE,
			.max	= V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED,
			.def	= V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED,
		},
		.capabilities	= CEDRUS_CAPABILITY_H264_DEC,
	},
	{
		.cfg = {
			.id	= V4L2_CID_STATELESS_H264_START_CODE,
			.max	= V4L2_STATELESS_H264_START_CODE_NONE,
			.def	= V4L2_STATELESS_H264_START_CODE_NONE,
		},
		.capabilities	= CEDRUS_CAPABILITY_H264_DEC,
	},
	/*
	 * We only expose supported profiles information,
	 * and not levels as it's not clear what is supported
	 * for each hardware/core version.
	 * In any case, TRY/S_FMT will clamp the format resolution
	 * to the maximum supported.
	 */
	{
		.cfg = {
			.id	= V4L2_CID_MPEG_VIDEO_H264_PROFILE,
			.min	= V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE,
			.def	= V4L2_MPEG_VIDEO_H264_PROFILE_MAIN,
			.max	= V4L2_MPEG_VIDEO_H264_PROFILE_HIGH,
			.menu_skip_mask =
				BIT(V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED),
		},
		.capabilities	= CEDRUS_CAPABILITY_H264_DEC,
	},
	{
		.cfg = {
			.id	= V4L2_CID_STATELESS_HEVC_SPS,
			.ops	= &cedrus_ctrl_ops,
		},
		.capabilities	= CEDRUS_CAPABILITY_H265_DEC,
	},
	{
		.cfg = {
			.id	= V4L2_CID_STATELESS_HEVC_PPS,
		},
		.capabilities	= CEDRUS_CAPABILITY_H265_DEC,
	},
	{
		.cfg = {
			.id	= V4L2_CID_STATELESS_HEVC_SLICE_PARAMS,
			/* The driver can only handle 1 entry per slice for now */
			.dims   = { 1 },
		},
		.capabilities	= CEDRUS_CAPABILITY_H265_DEC,
	},
	{
		.cfg = {
			.id	= V4L2_CID_STATELESS_HEVC_SCALING_MATRIX,
		},
		.capabilities	= CEDRUS_CAPABILITY_H265_DEC,
	},
	{
		.cfg = {
			.id	= V4L2_CID_STATELESS_HEVC_ENTRY_POINT_OFFSETS,
			/* maximum 256 entry point offsets per slice */
			.dims	= { 256 },
			.max = 0xffffffff,
			.step = 1,
		},
		.capabilities	= CEDRUS_CAPABILITY_H265_DEC,
	},
	{
		.cfg = {
			.id	= V4L2_CID_STATELESS_HEVC_DECODE_MODE,
			.max	= V4L2_STATELESS_HEVC_DECODE_MODE_SLICE_BASED,
			.def	= V4L2_STATELESS_HEVC_DECODE_MODE_SLICE_BASED,
		},
		.capabilities	= CEDRUS_CAPABILITY_H265_DEC,
	},
	{
		.cfg = {
			.id	= V4L2_CID_STATELESS_HEVC_START_CODE,
			.max	= V4L2_STATELESS_HEVC_START_CODE_NONE,
			.def	= V4L2_STATELESS_HEVC_START_CODE_NONE,
		},
		.capabilities	= CEDRUS_CAPABILITY_H265_DEC,
	},
	{
		.cfg = {
			.id	= V4L2_CID_STATELESS_VP8_FRAME,
		},
		.capabilities	= CEDRUS_CAPABILITY_VP8_DEC,
	},
	{
		.cfg = {
			.id = V4L2_CID_STATELESS_HEVC_DECODE_PARAMS,
		},
		.capabilities	= CEDRUS_CAPABILITY_H265_DEC,
	},
	{
		.cfg = {
			.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL,
			.def = V4L2_MPEG_VIDEO_H264_LEVEL_4_1,
			.min = V4L2_MPEG_VIDEO_H264_LEVEL_4_1,
			.max = V4L2_MPEG_VIDEO_H264_LEVEL_4_1,
		},
		.capabilities   = CEDRUS_CAPABILITY_H264_ENC,
	},
	{
		.cfg = {
			.id = V4L2_CID_JPEG_COMPRESSION_QUALITY,
			.min = 5,
			.max = 100,
			.step = 1,
			.def = 50,
			.ops	= &cedrus_ctrl_ops,
		},
		.capabilities	= CEDRUS_CAPABILITY_JPEG_ENC,
	},
	{
		.cfg = {
			.id = V4L2_CID_MPEG_VIDEO_H264_I_PERIOD,
			.min = 1,
			.max = 50,
			.step = 1,
			.def = 12,
			.ops	= &cedrus_ctrl_ops,
		},
		.capabilities	= CEDRUS_CAPABILITY_H264_ENC,
	},
	{
		.cfg = {
			.id = V4L2_CID_MPEG_VIDEO_H264_MAX_QP,
			.min = 1,
			.max = 50,
			.step = 1,
			.def = 24,
			.ops	= &cedrus_ctrl_ops,
		},
		.capabilities	= CEDRUS_CAPABILITY_H264_ENC,
	},
};

#define CEDRUS_CONTROLS_COUNT	ARRAY_SIZE(cedrus_controls)

void *cedrus_find_control_data(struct cedrus_ctx *ctx, u32 id)
{
	unsigned int i;

	for (i = 0; ctx->ctrls[i]; i++)
		if (ctx->ctrls[i]->id == id)
			return ctx->ctrls[i]->p_cur.p;

	return NULL;
}

u32 cedrus_get_num_of_controls(struct cedrus_ctx *ctx, u32 id)
{
	unsigned int i;

	for (i = 0; ctx->ctrls[i]; i++)
		if (ctx->ctrls[i]->id == id)
			return ctx->ctrls[i]->elems;

	return 0;
}

static int cedrus_init_ctrls(struct cedrus_dev *dev, struct cedrus_ctx *ctx)
{
	struct v4l2_ctrl_handler *hdl = &ctx->hdl;
	struct v4l2_ctrl *ctrl;
	unsigned int ctrl_size;
	unsigned int i, j;

	v4l2_ctrl_handler_init(hdl, CEDRUS_CONTROLS_COUNT);
	if (hdl->error) {
		v4l2_err(&dev->v4l2_dev,
			 "Failed to initialize control handler: %d\n",
			 hdl->error);
		return hdl->error;
	}

	ctrl_size = sizeof(ctrl) * CEDRUS_CONTROLS_COUNT + 1;

	ctx->ctrls = kzalloc(ctrl_size, GFP_KERNEL);
	if (!ctx->ctrls)
		return -ENOMEM;

	j = 0;
	for (i = 0; i < CEDRUS_CONTROLS_COUNT; i++) {
		if (!cedrus_is_capable(ctx, cedrus_controls[i].capabilities))
			continue;

		ctrl = v4l2_ctrl_new_custom(hdl, &cedrus_controls[i].cfg,
					    NULL);
		if (hdl->error) {
			v4l2_err(&dev->v4l2_dev,
				 "Failed to create %s control: %d\n",
				 v4l2_ctrl_get_name(cedrus_controls[i].cfg.id),
				 hdl->error);

			v4l2_ctrl_handler_free(hdl);
			kfree(ctx->ctrls);
			ctx->ctrls = NULL;
			return hdl->error;
		}

		ctx->ctrls[j++] = ctrl;
	}

	ctx->fh.ctrl_handler = hdl;
	v4l2_ctrl_handler_setup(hdl);

	return 0;
}

static int cedrus_request_validate(struct media_request *req)
{
	struct media_request_object *obj;
	struct cedrus_ctx *ctx = NULL;
	unsigned int count;

	list_for_each_entry(obj, &req->objects, list) {
		struct vb2_buffer *vb;

		if (vb2_request_object_is_buffer(obj)) {
			vb = container_of(obj, struct vb2_buffer, req_obj);
			ctx = vb2_get_drv_priv(vb->vb2_queue);

			break;
		}
	}

	if (!ctx)
		return -ENOENT;

	count = vb2_request_buffer_cnt(req);
	if (!count) {
		v4l2_info(&ctx->dev->v4l2_dev,
			  "No buffer was provided with the request\n");
		return -ENOENT;
	} else if (count > 1) {
		v4l2_info(&ctx->dev->v4l2_dev,
			  "More than one buffer was provided with the request\n");
		return -EINVAL;
	}

	return vb2_request_validate(req);
}

static int cedrus_open_dec(struct file *file)
{
	struct cedrus_dev *dev = video_drvdata(file);
	struct cedrus_ctx *ctx = NULL;
	int ret;

	if (mutex_lock_interruptible(&dev->dev_mutex))
		return -ERESTARTSYS;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		mutex_unlock(&dev->dev_mutex);
		return -ENOMEM;
	}

	ctx->is_enc = false;

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	ctx->dev = dev;
	ctx->bit_depth = 8;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev_dec, ctx,
		      &cedrus_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_free;
	}

	cedrus_reset_out_format(ctx);

	ret = cedrus_init_ctrls(dev, ctx);
	if (ret)
		goto err_m2m_release;

	v4l2_fh_add(&ctx->fh, file);

	mutex_unlock(&dev->dev_mutex);

	return 0;

err_m2m_release:
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
err_free:
	kfree(ctx);
	mutex_unlock(&dev->dev_mutex);

	return ret;
}

static int cedrus_open_enc(struct file *file)
{
	struct cedrus_dev *dev = video_drvdata(file);
	struct cedrus_ctx *ctx = NULL;
	int ret;

	if (mutex_lock_interruptible(&dev->dev_mutex))
		return -ERESTARTSYS;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		mutex_unlock(&dev->dev_mutex);
		return -ENOMEM;
	}

	ctx->is_enc = true;

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	ctx->dev = dev;
	ctx->bit_depth = 8;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev_enc, ctx,
					    &cedrus_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_free;
	}

	cedrus_reset_out_format(ctx);

	ret = cedrus_init_ctrls(dev, ctx);
	if (ret)
		goto err_m2m_release;

	v4l2_fh_add(&ctx->fh, file);

	mutex_unlock(&dev->dev_mutex);

	return 0;

err_m2m_release:
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
err_free:
	kfree(ctx);
	mutex_unlock(&dev->dev_mutex);

	return ret;
}

static int cedrus_release(struct file *file)
{
	struct cedrus_dev *dev = video_drvdata(file);
	struct cedrus_ctx *ctx = cedrus_file2ctx(file);

	mutex_lock(&dev->dev_mutex);

	v4l2_fh_del(&ctx->fh, file);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);

	v4l2_ctrl_handler_free(&ctx->hdl);
	kfree(ctx->ctrls);

	v4l2_fh_exit(&ctx->fh);

	kfree(ctx);

	mutex_unlock(&dev->dev_mutex);

	return 0;
}

static void cedrus_job_abort(void *priv)
{
	struct cedrus_ctx *ctx = priv;
	struct cedrus_dev *dev = ctx->dev;
	struct v4l2_m2m_dev *m2m_dev = ctx->is_enc ?
					 dev->m2m_dev_enc : dev->m2m_dev_dec;
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *src, *dst;

	/* Stop the watchdog to avoid double completion. */
	cancel_delayed_work_sync(&dev->watchdog_work);

	/* Best effort to quiesce the hardware. */
	if (ctx->current_codec && ctx->current_codec->irq_disable)
		ctx->current_codec->irq_disable(ctx);

	reset_control_reset(dev->rstc);

	/* Return the in-flight buffers to userspace with an error state. */
		if (v4l2_m2m_get_curr_priv(m2m_dev) == ctx) {
			src = v4l2_m2m_src_buf_remove(m2m_ctx);
			dst = v4l2_m2m_dst_buf_remove(m2m_ctx);
			if (src)
			v4l2_m2m_buf_done(src, VB2_BUF_STATE_ERROR);
		if (dst)
			v4l2_m2m_buf_done(dst, VB2_BUF_STATE_ERROR);
		v4l2_m2m_job_finish(m2m_dev, m2m_ctx);
	}
}

static const struct v4l2_file_operations cedrus_fops_dec = {
	.owner		= THIS_MODULE,
	.open		= cedrus_open_dec,
	.release	= cedrus_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

static const struct video_device cedrus_video_device_dec = {
	.name		= "cedrus-dec",
	.vfl_dir	= VFL_DIR_M2M,
	.fops		= &cedrus_fops_dec,
	.ioctl_ops	= &cedrus_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release_empty,
	.device_caps	= V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING,
};

static const struct v4l2_m2m_ops cedrus_m2m_ops_dec = {
	.device_run	= cedrus_dec_run,
	.job_abort	= cedrus_job_abort,
};

static const struct v4l2_file_operations cedrus_fops_enc = {
	.owner		= THIS_MODULE,
	.open		= cedrus_open_enc,
	.release	= cedrus_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

static const struct video_device cedrus_video_device_enc = {
	.name		= "cedrus-enc",
	.vfl_dir	= VFL_DIR_M2M,
	.fops		= &cedrus_fops_enc,
	.ioctl_ops	= &cedrus_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release_empty,
	.device_caps	= V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING,
};

static const struct v4l2_m2m_ops cedrus_m2m_ops_enc = {
	.device_run	= cedrus_enc_run,
	.job_abort	= cedrus_job_abort,
};

static const struct media_device_ops cedrus_m2m_media_ops = {
	.req_validate	= cedrus_request_validate,
	.req_queue	= v4l2_m2m_request_queue,
};

static int cedrus_m2m_register_dec(struct cedrus_dev *dev)
{
	struct v4l2_device *v4l2_dev = &dev->v4l2_dev;
	struct video_device *vfd = &dev->vfd_dec;
	struct v4l2_m2m_dev	*m2m_dev;
	int ret;

	vfd->lock = &dev->dev_mutex;
	vfd->v4l2_dev = v4l2_dev;
	strscpy(vfd->name, "cedrus-dec", sizeof(vfd->name));

	v4l2_disable_ioctl(vfd, VIDIOC_ENCODER_CMD);
	v4l2_disable_ioctl(vfd, VIDIOC_TRY_ENCODER_CMD);

	video_set_drvdata(vfd, dev);

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, 0);
	if (ret) {
		v4l2_err(v4l2_dev, "Failed to register vfd dec\n");
		return ret;
	}

	v4l2_info(v4l2_dev, "dec registered as /dev/video%d\n", vfd->num);

	m2m_dev = v4l2_m2m_init(&cedrus_m2m_ops_dec);
	if (IS_ERR(m2m_dev)) {
		v4l2_err(v4l2_dev, "Failed to initialize m2m_dev dec\n");
		video_unregister_device(vfd);
		return PTR_ERR(m2m_dev);
	}

	ret = v4l2_m2m_register_media_controller(m2m_dev, vfd,
		      MEDIA_ENT_F_PROC_VIDEO_DECODER);
	if (ret) {
		v4l2_err(v4l2_dev, "Failed to register m2m_dev dec\n");
		v4l2_m2m_release(m2m_dev);
		video_unregister_device(vfd);
		return ret;
	}

	dev->m2m_dev_dec = m2m_dev;

	return 0;
}

static int cedrus_m2m_register_enc(struct cedrus_dev *dev)
{
	struct v4l2_device *v4l2_dev = &dev->v4l2_dev;
	struct video_device *vfd = &dev->vfd_enc;
	struct v4l2_m2m_dev	*m2m_dev;
	int ret;

	vfd->lock = &dev->dev_mutex;
	vfd->v4l2_dev = v4l2_dev;
	strscpy(vfd->name, "cedrus-enc", sizeof(vfd->name));

	v4l2_disable_ioctl(vfd, VIDIOC_DECODER_CMD);
	v4l2_disable_ioctl(vfd, VIDIOC_TRY_DECODER_CMD);

	video_set_drvdata(vfd, dev);

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, 0);
	if (ret) {
		v4l2_err(v4l2_dev, "Failed to register vfd enc\n");
		return ret;
	}

	v4l2_info(v4l2_dev, "enc registered as /dev/video%d\n", vfd->num);

	m2m_dev = v4l2_m2m_init(&cedrus_m2m_ops_enc);
	if (IS_ERR(m2m_dev)) {
		v4l2_err(v4l2_dev, "Failed to initialize m2m_dev enc\n");
		video_unregister_device(vfd);
		return PTR_ERR(m2m_dev);
	}

	ret = v4l2_m2m_register_media_controller(m2m_dev, vfd,
		      MEDIA_ENT_F_PROC_VIDEO_ENCODER);
	if (ret) {
		v4l2_err(v4l2_dev, "Failed to register m2m_dev enc\n");
		v4l2_m2m_release(m2m_dev);
		video_unregister_device(vfd);
		return ret;
	}

	dev->m2m_dev_enc = m2m_dev;

	return 0;
}

static int cedrus_probe(struct platform_device *pdev)
{
	struct cedrus_dev *dev;
	int ret;

	dev_info(&pdev->dev, "cedrus probe start\n");

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, dev);

	dev->vfd_dec = cedrus_video_device_dec;
	dev->vfd_enc = cedrus_video_device_enc;
	dev->dev = &pdev->dev;
	dev->pdev = pdev;

	ret = cedrus_hw_probe(dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to probe hardware\n");
		return ret;
	}

	/* When an IOMMU is present, allow vb2 to use non-contiguous backing and
	 * ask the DMA layer to merge large segments into a single IOVA. This
	 * relaxes the contiguous-physical memory requirement that pressured CMA.
	 */
	dev->use_iommu = device_iommu_mapped(dev->dev);

	ret = dma_set_mask_and_coherent(dev->dev, DMA_BIT_MASK(32));
	if (ret)
		dev_warn(dev->dev, "Failed to set 32-bit DMA mask: %d\n", ret);

	if (dev->use_iommu)
		dev_info(dev->dev, "VE buffers will use IOMMU-backed non-contiguous allocations\n");
	else
		dev_info(dev->dev, "VE buffers will use physically contiguous (CMA) allocations\n");

	if (dev->use_iommu) {
		ret = vb2_dma_contig_set_max_seg_size(dev->dev,
						      DMA_BIT_MASK(32));
		if (ret)
			dev_warn(dev->dev,
				 "Failed to raise max segment size for IOMMU: %d\n",
				 ret);
	}

	mutex_init(&dev->dev_mutex);
	INIT_DELAYED_WORK(&dev->watchdog_work, cedrus_watchdog);

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register V4L2 device\n");
		return ret;
	}

	dev->mdev.dev = &pdev->dev;
	strscpy(dev->mdev.model, CEDRUS_NAME, sizeof(dev->mdev.model));
	strscpy(dev->mdev.bus_info, "platform:" CEDRUS_NAME,
		sizeof(dev->mdev.bus_info));

	media_device_init(&dev->mdev);
	dev->mdev.ops = &cedrus_m2m_media_ops;
	dev->v4l2_dev.mdev = &dev->mdev;

	ret = cedrus_m2m_register_dec(dev);
	if (ret)
		goto err_mdev_clean;

	ret = cedrus_m2m_register_enc(dev);
	if (ret)
		goto err_m2m_dec;

	ret = media_device_register(&dev->mdev);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "Failed to register media device\n");
		goto err_m2m_enc;
	}

	dev_info(&pdev->dev, "cedrus probe done (IOMMU %s)\n",
		 dev->use_iommu ? "enabled" : "disabled");

	return 0;

err_m2m_enc:
	v4l2_m2m_unregister_media_controller(dev->m2m_dev_enc);
	v4l2_m2m_release(dev->m2m_dev_enc);
	video_unregister_device(&dev->vfd_enc);
err_m2m_dec:
	v4l2_m2m_unregister_media_controller(dev->m2m_dev_dec);
	v4l2_m2m_release(dev->m2m_dev_dec);
	video_unregister_device(&dev->vfd_dec);
err_mdev_clean:
	media_device_cleanup(&dev->mdev);
	v4l2_device_unregister(&dev->v4l2_dev);

	return ret;
}

static void cedrus_remove(struct platform_device *pdev)
{
	struct cedrus_dev *dev = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&dev->watchdog_work);
	if (media_devnode_is_registered(dev->mdev.devnode)) {
		media_device_unregister(&dev->mdev);
		v4l2_m2m_unregister_media_controller(dev->m2m_dev_dec);
		v4l2_m2m_unregister_media_controller(dev->m2m_dev_enc);
		media_device_cleanup(&dev->mdev);
	}

	v4l2_m2m_release(dev->m2m_dev_dec);
	v4l2_m2m_release(dev->m2m_dev_enc);
	video_unregister_device(&dev->vfd_dec);
	video_unregister_device(&dev->vfd_enc);
	v4l2_device_unregister(&dev->v4l2_dev);

	cedrus_hw_remove(dev);
}

static const struct cedrus_variant sun4i_a10_cedrus_variant = {
	.capabilities	= CEDRUS_CAPABILITY_MPEG2_DEC |
			  CEDRUS_CAPABILITY_H264_DEC |
			  CEDRUS_CAPABILITY_VP8_DEC,
	.mod_rate	= 320000000,
};

static const struct cedrus_variant sun5i_a13_cedrus_variant = {
	.capabilities	= CEDRUS_CAPABILITY_MPEG2_DEC |
			  CEDRUS_CAPABILITY_H264_DEC |
			  CEDRUS_CAPABILITY_VP8_DEC,
	.mod_rate	= 320000000,
};

static const struct cedrus_variant sun7i_a20_cedrus_variant = {
	.capabilities	= CEDRUS_CAPABILITY_MPEG2_DEC |
			  CEDRUS_CAPABILITY_H264_DEC |
			  CEDRUS_CAPABILITY_VP8_DEC,
	.mod_rate	= 320000000,
};

static const struct cedrus_variant sun8i_a33_cedrus_variant = {
	.capabilities	= CEDRUS_CAPABILITY_UNTILED |
			  CEDRUS_CAPABILITY_MPEG2_DEC |
			  CEDRUS_CAPABILITY_H264_DEC |
			  CEDRUS_CAPABILITY_VP8_DEC,
	.mod_rate	= 320000000,
};

static const struct cedrus_variant sun8i_h3_cedrus_variant = {
	.capabilities	= CEDRUS_CAPABILITY_UNTILED |
			  CEDRUS_CAPABILITY_MPEG2_DEC |
			  CEDRUS_CAPABILITY_H264_DEC |
			  CEDRUS_CAPABILITY_H265_DEC |
			  CEDRUS_CAPABILITY_VP8_DEC,
	.mod_rate	= 402000000,
};

static const struct cedrus_variant sun8i_v3s_cedrus_variant = {
	.capabilities	= CEDRUS_CAPABILITY_UNTILED |
			  CEDRUS_CAPABILITY_H264_DEC,
	.mod_rate	= 297000000,
};

static const struct cedrus_variant sun8i_r40_cedrus_variant = {
	.capabilities	= CEDRUS_CAPABILITY_UNTILED |
			  CEDRUS_CAPABILITY_MPEG2_DEC |
			  CEDRUS_CAPABILITY_H264_DEC |
			  CEDRUS_CAPABILITY_VP8_DEC |
			  CEDRUS_CAPABILITY_JPEG_ENC |
			  CEDRUS_CAPABILITY_H264_ENC,
	.mod_rate	= 297000000,
};

static const struct cedrus_variant sun20i_d1_cedrus_variant = {
	.capabilities	= CEDRUS_CAPABILITY_UNTILED |
			  CEDRUS_CAPABILITY_MPEG2_DEC |
			  CEDRUS_CAPABILITY_H264_DEC |
			  CEDRUS_CAPABILITY_H265_DEC |
			  CEDRUS_CAPABILITY_JPEG_ENC,
	.mod_rate	= 432000000,
};

static const struct cedrus_variant sun50i_a64_cedrus_variant = {
	.capabilities	= CEDRUS_CAPABILITY_UNTILED |
			  CEDRUS_CAPABILITY_MPEG2_DEC |
			  CEDRUS_CAPABILITY_H264_DEC |
			  CEDRUS_CAPABILITY_H265_DEC |
			  CEDRUS_CAPABILITY_VP8_DEC,
	.mod_rate	= 402000000,
};

static const struct cedrus_variant sun50i_h5_cedrus_variant = {
	.capabilities	= CEDRUS_CAPABILITY_UNTILED |
			  CEDRUS_CAPABILITY_MPEG2_DEC |
			  CEDRUS_CAPABILITY_H264_DEC |
			  CEDRUS_CAPABILITY_H265_DEC |
			  CEDRUS_CAPABILITY_VP8_DEC,
	.mod_rate	= 402000000,
};

static const struct cedrus_variant sun50i_h6_cedrus_variant = {
	.capabilities	= CEDRUS_CAPABILITY_UNTILED |
			  CEDRUS_CAPABILITY_MPEG2_DEC |
			  CEDRUS_CAPABILITY_H264_DEC |
			  CEDRUS_CAPABILITY_H265_DEC |
			  CEDRUS_CAPABILITY_H265_10_DEC |
			  CEDRUS_CAPABILITY_VP8_DEC,
	.mod_rate	= 600000000,
};

static const struct of_device_id cedrus_dt_match[] = {
	{
		.compatible = "allwinner,sun4i-a10-video-engine",
		.data = &sun4i_a10_cedrus_variant,
	},
	{
		.compatible = "allwinner,sun5i-a13-video-engine",
		.data = &sun5i_a13_cedrus_variant,
	},
	{
		.compatible = "allwinner,sun7i-a20-video-engine",
		.data = &sun7i_a20_cedrus_variant,
	},
	{
		.compatible = "allwinner,sun8i-a33-video-engine",
		.data = &sun8i_a33_cedrus_variant,
	},
	{
		.compatible = "allwinner,sun8i-h3-video-engine",
		.data = &sun8i_h3_cedrus_variant,
	},
	{
		.compatible = "allwinner,sun8i-v3s-video-engine",
		.data = &sun8i_v3s_cedrus_variant,
	},
	{
		.compatible = "allwinner,sun8i-r40-video-engine",
		.data = &sun8i_r40_cedrus_variant,
	},
	{
		.compatible = "allwinner,sun20i-d1-video-engine",
		.data = &sun20i_d1_cedrus_variant,
	},
	{
		.compatible = "allwinner,sun50i-a64-video-engine",
		.data = &sun50i_a64_cedrus_variant,
	},
	{
		.compatible = "allwinner,sun50i-h5-video-engine",
		.data = &sun50i_h5_cedrus_variant,
	},
	{
		.compatible = "allwinner,sun50i-h6-video-engine",
		.data = &sun50i_h6_cedrus_variant,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, cedrus_dt_match);

static const struct dev_pm_ops cedrus_dev_pm_ops = {
	SET_RUNTIME_PM_OPS(cedrus_hw_suspend,
			   cedrus_hw_resume, NULL)
};

static struct platform_driver cedrus_driver = {
	.probe		= cedrus_probe,
	.remove		= cedrus_remove,
	.driver		= {
		.name		= CEDRUS_NAME,
		.of_match_table	= cedrus_dt_match,
		.pm		= &cedrus_dev_pm_ops,
	},
};
module_platform_driver(cedrus_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Florent Revest <florent.revest@free-electrons.com>");
MODULE_AUTHOR("Paul Kocialkowski <paul.kocialkowski@bootlin.com>");
MODULE_AUTHOR("Maxime Ripard <maxime.ripard@bootlin.com>");
MODULE_DESCRIPTION("Cedrus VPU driver");
