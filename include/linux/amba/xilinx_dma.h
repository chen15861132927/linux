/*
 * Xilinx Central DMA Engine support
 *
 * Copyright (C) 2010 Xilinx, Inc. All rights reserved.
 *
 * Based on the Freescale DMA driver.
 *
 * Description:
 * This driver supports three Xilinx DMA engines:
 *  . Axi CDMA engine, it does transfers between memory and memory, it
 *    only has one channel.
 *  . Axi DMA engine, it does transfers between memory and device. It can be
 *    configured to have one channel or two channels. If configured as two
 *    channels, one is to transmit to device and another is to receive from
 *    device.
 *  . Axi VDMA engine, it does transfers between memory and video devices.
 *    It can be configured to have one channel or two channels. If configured
 *    as two channels, one is to transmit to the video device and another is
 *    to receive from the video device.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __DMA_XILINX_DMA_H
#define __DMA_XILINX_DMA_H

#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>

/* Specific hardware configuration-related constants
 */
#define XILINX_DMA_NO_CHANGE             0xFFFF;

/* DMA IP masks 
 */
#define XILINX_DMA_IP_DMA              0x00100000 /* A DMA IP */
#define XILINX_DMA_IP_CDMA             0x00200000 /* A Central DMA IP */
#define XILINX_DMA_IP_VDMA             0x00400000 /* A Video DMA IP */
#define XILINX_DMA_IP_MASK             0x00700000 /* DMA IP MASK */

/* shared by all Xilinx DMA engines
 */
/* Device configuration structure
 *
 * Xilinx CDMA and Xilinx DMA only use interrupt coalescing and delay counter
 * settings.
 *
 * If used to start/stop parking mode for Xilinx VDMA, vsize must be -1
 * If used to set interrupt coalescing and delay counter only for
 * Xilinx VDMA, hsize must be -1 */
struct xilinx_dma_config {
	enum dma_data_direction direction; /* Channel direction */
	int vsize;                         /* Vertical size */
	int hsize;                         /* Horizontal size */
	int stride;                        /* Stride */
	int frm_dly;                       /* Frame delay */
	int gen_lock;                      /* Whether in gen-lock mode */
	int master;                        /* Master that it syncs to */
	int frm_cnt_en;                    /* Enable frame count enable */
	int park;                          /* Whether wants to park */
	int park_frm;                      /* Frame to park on */
	int coalesc;                       /* Interrupt coalescing threshold */
	int delay;                         /* Delay counter */
	int disable_intr;                  /* Whether use interrupts */
};

/* Platform data definition until ARM supports device tree */

struct dma_channel_config {
	char *type;	
	unsigned int lite_mode;       /* cdma only */
	unsigned int include_dre;
	unsigned int genlock_mode;    /* vdma only */
	unsigned int datawidth;
	unsigned int max_burst_len;
};

struct dma_device_config {
	char *type;	
	unsigned int include_sg;
	unsigned int num_fstores;    /* vdma only */
	unsigned int sg_include_stscntrl_strm;  /* dma only */
	unsigned int channel_count;
	struct dma_channel_config *channel_config;
};

#endif
