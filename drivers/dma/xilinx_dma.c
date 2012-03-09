/*
 * Xilinx DMA Engine support
 *
 * Copyright (C) 2010 Xilinx, Inc. All rights reserved.
 *
 * Based on the Freescale DMA driver.
 *
 * Description:
 * This driver supports three Xilinx DMA engines:
 *  . Axi DMA engine, it does transfers between memory and device. It can be
 *    configured to have one channel or two channels. If configured as two
 *    channels, one is to transmit to a device and another is to receive from
 *    a device.
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


#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/dmapool.h>
#include <asm/io.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/amba/xilinx_dma.h>

/* Hw specific definitions
 */
#define XILINX_DMA_MAX_CHANS_PER_DEVICE  0x2
#define XILINX_DMA_MAX_TRANS_LEN         0x7FFFFF

/* General register bits definitions
 */
#define XILINX_DMA_CR_RESET_MASK    0x00000004  /* Reset DMA engine */
#define XILINX_DMA_CR_RUNSTOP_MASK  0x00000001  /* Start/stop DMA engine */

#define XILINX_DMA_SR_HALTED_MASK   0x00000001  /* DMA channel halted */
#define XILINX_DMA_SR_IDLE_MASK     0x00000002  /* DMA channel idle */

#define XILINX_DMA_SR_ERR_INTERNAL_MASK 0x00000010 /* Datamover internal err */
#define XILINX_DMA_SR_ERR_SLAVE_MASK    0x00000020 /* Datamover slave err */
#define XILINX_DMA_SR_ERR_DECODE_MASK   0x00000040 /* Datamover decode err */
#define XILINX_DMA_SR_ERR_SG_INT_MASK   0x00000100 /* SG internal err */
#define XILINX_DMA_SR_ERR_SG_SLV_MASK   0x00000200 /* SG slave err */
#define XILINX_DMA_SR_ERR_SG_DEC_MASK   0x00000400 /* SG decode err */
#define XILINX_DMA_SR_ERR_ALL_MASK      0x00000770 /* All errors */

#define XILINX_DMA_XR_IRQ_IOC_MASK	0x00001000 /* Completion interrupt */
#define XILINX_DMA_XR_IRQ_DELAY_MASK	0x00002000 /* Delay interrupt */
#define XILINX_DMA_XR_IRQ_ERROR_MASK	0x00004000 /* Error interrupt */
#define XILINX_DMA_XR_IRQ_ALL_MASK	    0x00007000 /* All interrupts */

#define XILINX_DMA_XR_DELAY_MASK    0xFF000000 /* Delay timeout counter */
#define XILINX_DMA_XR_COALESCE_MASK 0x00FF0000 /* Coalesce counter */

#define XILINX_DMA_DELAY_SHIFT    24
#define XILINX_DMA_COALESCE_SHIFT 16

#define XILINX_DMA_DELAY_MAX     0xFF /**< Maximum delay counter value */
#define XILINX_DMA_COALESCE_MAX  0xFF /**< Maximum coalescing counter value */

#define XILINX_DMA_RX_CHANNEL_OFFSET      0x30

/* Axi VDMA special register bits
 */
#define XILINX_VDMA_CIRC_EN         0x00000002  /* Circular mode */
#define XILINX_VDMA_SYNC_EN         0x00000008  /* Sync enable mode */
#define XILINX_VDMA_FRMCNT_EN       0x00000010  /* Frm Cnt enable mode */
#define XILINX_VDMA_MSTR_MASK       0x00000F00  /* Master in control */

#define XILINX_VDMA_MSTR_SHIFT      8
#define XILINX_VDMA_WR_REF_SHIFT    8

#define XILINX_VDMA_FRMDLY_SHIFT  24

#define XILINX_VDMA_DIRECT_REG_OFFSET     0x50
#define XILINX_VDMA_CHAN_DIRECT_REG_SIZE  0x50

/* BD definitions for Axi DMA
 */
#define XILINX_DMA_BD_STS_COMPL_MASK 0x80000000
#define XILINX_DMA_BD_STS_ERR_MASK   0x70000000
#define XILINX_DMA_BD_STS_ALL_MASK   0xF0000000

/* Axi DMA BD special bits definitions
 */
#define XILINX_DMA_BD_SOP       0x08000000    /* Start of packet bit */
#define XILINX_DMA_BD_EOP       0x04000000    /* End of packet bit */

/* Feature encodings
 */
#define XILINX_DMA_FTR_DATA_WIDTH_MASK 0x000000FF /* Data width mask, 1024 */
#define XILINX_DMA_FTR_HAS_SG          0x00000100 /* Has SG */
#define XILINX_DMA_FTR_HAS_SG_SHIFT    8          /* Has SG shift */
#define XILINX_DMA_FTR_STSCNTRL_STRM   0x00010000 /* Optional feature for dma */

/* Delay loop counter to prevent hardware failure
 */
#define XILINX_DMA_RESET_LOOP            1000000
#define XILINX_DMA_HALT_LOOP             1000000

/* IO accessors
 */
#define DMA_OUT(addr, val)  (iowrite32(val, addr))
#define DMA_IN(addr)  (ioread32(addr))

/* Hardware descriptor
 *
 * shared by all Xilinx DMA engines
 */
struct xilinx_dma_desc_hw {
	u32 next_desc;	/* 0x00 */
	u32 pad1;       /* 0x04 */
	u32 buf_addr;   /* 0x08 */
	u32 pad2;       /* 0x0C */
	u32 addr_vsize; /* 0x10 */
	u32 hsize;      /* 0x14 */
	u32 control;    /* 0x18 */
	u32 status;     /* 0x1C */
	u32 app_0;      /* 0x20 */
	u32 app_1;      /* 0x24 */
	u32 app_2;      /* 0x28 */
	u32 app_3;      /* 0x2C */
	u32 app_4;      /* 0x30 */
} __attribute__((aligned(64)));

struct xilinx_dma_desc_sw {
	struct xilinx_dma_desc_hw *hw;
	dma_addr_t phys;
};

struct xilinx_dma_transfer {
	struct dma_async_tx_descriptor async_tx;
	struct list_head head;

	unsigned int current_desc;
	unsigned int num_descs;
	struct xilinx_dma_desc_sw descs[];
};

struct xdma_regs {
	u32 cr;     /* 0x00 Control Register */
	u32 sr;     /* 0x04 Status Register */
	u32 cdr;    /* 0x08 Current Descriptor Register */
	u32 pad1;
	u32 tdr;    /* 0x10 Tail Descriptor Register */
	u32 pad2;
	u32 src;    /* 0x18 Source Address Register (cdma) */
	u32 pad3;
	u32 dst;    /* 0x20 Destination Address Register (cdma) */
	u32 pad4;
	u32 btt_ref;/* 0x28 Bytes To Transfer (cdma) or park_ref (vdma) */
	u32 version;         /* 0x2c version (vdma) */
};

struct vdma_addr_regs {
	u32 vsize;          /* 0x0 Vertical size */
	u32 hsize;          /* 0x4 Horizontal size */
	u32 frmdly_stride;  /* 0x8 Frame delay and stride */
	u32 buf_addr[16];   /* 0xC - 0x48 Src addresses */
};

/* Per DMA specific operations should be embedded in the channel structure
 */
struct xilinx_dma_chan {
	struct xdma_regs __iomem *regs;   /* Control status registers */
	struct vdma_addr_regs *addr_regs; /* Direct address registers */
	dma_cookie_t completed_cookie;	  /* The maximum cookie completed */
	dma_cookie_t cookie;	          /* The current cookie */
	spinlock_t lock;                  /* Descriptor operation lock */
	bool   sg_waiting;                /* Scatter gather transfer waiting */
	struct list_head active_list;	  /* Active descriptors */
	struct list_head pending_list;	  /* Descriptors waiting */
	struct dma_chan common;           /* DMA common channel */
	struct dma_pool *desc_pool;       /* Descriptors pool */
	struct device *dev;               /* The dma device */
	int    irq;                       /* Channel IRQ */
	int    id;                        /* Channel ID */
	enum dma_transfer_direction direction;/* Transfer direction */
	int    max_len;                   /* Maximum data len per transfer */
	int    is_lite;                   /* Whether is light build */
	int    num_frms;                  /* Number of frames */
	int    has_SG;                    /* Support scatter transfers */
	int    has_DRE;                   /* Support unaligned transfers */
	int    genlock;                   /* Support genlock mode */
	int    err;                       /* Channel has errors */
	struct tasklet_struct tasklet;    /* Cleanup work after irq */
	u32    feature;                   /* IP feature */
	u32    private;                   /* Match info for channel request */
	void   (*start_transfer)(struct xilinx_dma_chan *chan);
	struct xilinx_dma_config config;  /* Device configuration info */
};

struct xilinx_dma_device {
	void __iomem *regs;
	struct device *dev;
	struct dma_device common;
	struct xilinx_dma_chan *chan[XILINX_DMA_MAX_CHANS_PER_DEVICE];
	u32 feature;
	int irq;
};

static dma_cookie_t xilinx_dma_tx_submit(struct dma_async_tx_descriptor *tx);

#define to_xilinx_chan(chan) container_of(chan, struct xilinx_dma_chan, common)

/* Required functions
 */
static int xilinx_dma_alloc_chan_resources(struct dma_chan *dchan)
{
	struct xilinx_dma_chan *chan = to_xilinx_chan(dchan);

	/* Has this channel already been allocated? */
	if (chan->desc_pool)
		return 1;

	/*
	 * We need the descriptor to be aligned to 64bytes
	 * for meeting Xilinx DMA specification requirement.
	 */
	chan->desc_pool = dma_pool_create("xilinx_dma_desc_pool",
				  chan->dev,
				  sizeof(struct xilinx_dma_desc_hw),
				  __alignof__(struct xilinx_dma_desc_hw), 0);
	if (!chan->desc_pool) {
		dev_err(chan->dev, "unable to allocate channel %d "
				   "descriptor pool\n", chan->id);
		return -ENOMEM;
	}

	chan->completed_cookie = 1;
	chan->cookie = 1;

	/* there is at least one descriptor free to be allocated */
	return 1;
}

static struct xilinx_dma_transfer *xilinx_dma_alloc_transfer(
	struct xilinx_dma_chan *chan, unsigned int num_descs)
{
	struct xilinx_dma_transfer *t;

	t = kzalloc(sizeof(*t) + num_descs * sizeof(*t->descs), GFP_KERNEL);
	if (!t)
		return NULL;

	dma_async_tx_descriptor_init(&t->async_tx, &chan->common);
	t->async_tx.tx_submit = xilinx_dma_tx_submit;

	return t;
}

static void xilinx_dma_free_transfer(struct xilinx_dma_chan *chan,
	struct xilinx_dma_transfer *t)
{
	unsigned int i;
	for (i = 0; i < t->num_descs; ++i)
		dma_pool_free(chan->desc_pool, t->descs[i].hw, t->descs[i].phys);
	kfree(t);
}

static void xilinx_dma_free_transfer_list(struct xilinx_dma_chan *chan,
	struct list_head *list)
{
	struct xilinx_dma_transfer *t;
	list_for_each_entry(t, list, head)
		xilinx_dma_free_transfer(chan, t);
	INIT_LIST_HEAD(list);
}

static void xilinx_dma_free_transfers(struct xilinx_dma_chan *chan)
{
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);
	xilinx_dma_free_transfer_list(chan, &chan->active_list);
	xilinx_dma_free_transfer_list(chan, &chan->pending_list);
	spin_unlock_irqrestore(&chan->lock, flags);
}

static void xilinx_dma_free_chan_resources(struct dma_chan *dchan)
{
	struct xilinx_dma_chan *chan = to_xilinx_chan(dchan);

	dev_dbg(chan->dev, "Free all channel resources.\n");
	xilinx_dma_free_transfers(chan);
	dma_pool_destroy(chan->desc_pool);
	chan->desc_pool = NULL;
}

static enum dma_status xilinx_dma_desc_status(struct xilinx_dma_chan *chan,
					  struct xilinx_dma_transfer *t)
{
	return dma_async_is_complete(t->async_tx.cookie,
				     chan->completed_cookie,
				     chan->cookie);
}

static void xilinx_chan_desc_cleanup(struct xilinx_dma_chan *chan)
{
	struct xilinx_dma_transfer *t, *_t;
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&chan->lock, flags);

	list_for_each_entry_safe(t, _t, &chan->active_list, head) {
		dma_async_tx_callback callback;
		void *callback_param;

		if (xilinx_dma_desc_status(chan, t) == DMA_IN_PROGRESS)
			break;

		/* Remove from the list of running transactions */
		list_del(&t->head);

		/* Run the link descriptor callback function */
		callback = t->async_tx.callback;
		callback_param = t->async_tx.callback_param;
		if (callback) {
			spin_unlock_irqrestore(&chan->lock, flags);
			callback(callback_param);
			spin_lock_irqsave(&chan->lock, flags);
		}

		/* Run any dependencies, then free the descriptor */
		dma_run_dependencies(&t->async_tx);
		for (i = 0; i < t->num_descs; ++i)
			dma_pool_free(chan->desc_pool, t->descs[i].hw, t->descs[i].phys);
		kfree(t);
	}

	spin_unlock_irqrestore(&chan->lock, flags);
}

static enum dma_status xilinx_tx_status(struct dma_chan *dchan,
					dma_cookie_t cookie,
					struct dma_tx_state *txstate)
{
	struct xilinx_dma_chan *chan = to_xilinx_chan(dchan);
	dma_cookie_t last_used;
	dma_cookie_t last_complete;

	xilinx_chan_desc_cleanup(chan);

	last_used = dchan->cookie;
	last_complete = chan->completed_cookie;

	dma_set_tx_state(txstate, last_complete, last_used, 0);

	return dma_async_is_complete(cookie, last_complete, last_used);
}

static int dma_is_running(struct xilinx_dma_chan *chan)
{
	return !(DMA_IN(&chan->regs->sr) & XILINX_DMA_SR_HALTED_MASK) &&
	   (DMA_IN(&chan->regs->cr) & XILINX_DMA_CR_RUNSTOP_MASK);
}

static int dma_is_idle(struct xilinx_dma_chan *chan)
{
	return DMA_IN(&chan->regs->sr) & XILINX_DMA_SR_IDLE_MASK;
}

#if 0
/* Only needed for Axi CDMA v2_00_a or earlier core
 */
static void dma_sg_toggle(struct xilinx_dma_chan *chan)
{

	DMA_OUT(&chan->regs->cr,
	    DMA_IN(&chan->regs->cr) & ~XILINX_CDMA_CR_SGMODE_MASK);

	DMA_OUT(&chan->regs->cr,
	    DMA_IN(&chan->regs->cr) | XILINX_CDMA_CR_SGMODE_MASK);
}
#endif
#define XILINX_DMA_DRIVER_DEBUG 0

#if (XILINX_DMA_DRIVER_DEBUG == 1)
static void desc_dump(struct xilinx_dma_desc_hw *hw)
{
	printk(KERN_INFO "hw desc %x:\n", (unsigned int)hw);
	printk(KERN_INFO "\tnext_desc %x\n", hw->next_desc);
	printk(KERN_INFO "\tbuf_addr %x\n", hw->buf_addr);
	printk(KERN_INFO "\taddr_vsize %x\n", hw->addr_vsize);
	printk(KERN_INFO "\thsize %x\n", hw->hsize);
	printk(KERN_INFO "\tcontrol %x\n", hw->control);
	printk(KERN_INFO "\tstatus %x\n", hw->status);

}
#endif

/* Stop the hardware, the ongoing transfer will be finished */
static void dma_halt(struct xilinx_dma_chan *chan)
{
	int loop = XILINX_DMA_HALT_LOOP;

	DMA_OUT(&chan->regs->cr,
	    DMA_IN(&chan->regs->cr) & ~XILINX_DMA_CR_RUNSTOP_MASK);

	/* Wait for the hardware to halt
	 */
	while (loop) {
		if (!(DMA_IN(&chan->regs->cr) & XILINX_DMA_CR_RUNSTOP_MASK))
			break;

		loop -= 1;
	}

	if (!loop) {
		pr_debug("Cannot stop channel %x: %x\n",
			(unsigned int)chan,
		    (unsigned int)DMA_IN(&chan->regs->cr));
		chan->err = 1;
	}

	return;
}

/* Start the hardware. Transfers are not started yet */
static void dma_start(struct xilinx_dma_chan *chan)
{
	int loop = XILINX_DMA_HALT_LOOP;

	DMA_OUT(&chan->regs->cr,
	    DMA_IN(&chan->regs->cr) | XILINX_DMA_CR_RUNSTOP_MASK);

	/* Wait for the hardware to start
	 */
	while (loop) {
		if (DMA_IN(&chan->regs->cr) & XILINX_DMA_CR_RUNSTOP_MASK)
			break;

		loop -= 1;
	}

	if (!loop) {
		pr_debug("Cannot start channel %x: %x\n",
			(unsigned int)chan,
		    (unsigned int)DMA_IN(&chan->regs->cr));

		chan->err = 1;
	}

	return;
}

static void xilinx_dma_start_transfer(struct xilinx_dma_chan *chan)
{
	struct xilinx_dma_transfer *last_transfer, *first_transfer;
	dma_addr_t first_addr, last_addr;
	struct xilinx_dma_desc_hw *hw;
	unsigned long flags;

	if (chan->err)
		return;

	spin_lock_irqsave(&chan->lock, flags);

	if (list_empty(&chan->pending_list))
		goto out_unlock;

	/* If hardware is busy, cannot submit
	 */
	if (dma_is_running(chan) && !dma_is_idle(chan)) {
		dev_dbg(chan->dev, "DMA controller still busy\n");
		goto out_unlock;
	}

	/* If hardware is idle, then all descriptors on active list are
	 * done, start new transfers
	 */
	dma_halt(chan);

	if (chan->err)
		goto out_unlock;

	first_transfer = list_first_entry(&chan->pending_list,
			struct xilinx_dma_transfer, head);

	if (chan->has_SG) {
		last_transfer = container_of(chan->pending_list.prev,
				struct xilinx_dma_transfer, head);

		first_addr = first_transfer->descs[0].phys;
		last_addr = last_transfer->descs[last_transfer->num_descs-1].phys;

		DMA_OUT(&chan->regs->cdr, first_addr);

		dma_start(chan);

		if (chan->err)
			goto out_unlock;
		list_splice_tail_init(&chan->pending_list, &chan->active_list);

		/* Enable interrupts
		*/
		DMA_OUT(&chan->regs->cr,
			DMA_IN(&chan->regs->cr) | XILINX_DMA_XR_IRQ_ALL_MASK);

		/* Update tail ptr register and start the transfer
		*/
		DMA_OUT(&chan->regs->tdr, last_addr);
	} else {
		/* In simple mode */

		list_move_tail(&first_transfer->head, &chan->active_list);

		dma_start(chan);

		if (chan->err)
			goto out_unlock;

		hw = first_transfer->descs[0].hw;

		/* Enable interrupts
		*/
		DMA_OUT(&chan->regs->cr,
			DMA_IN(&chan->regs->cr) | XILINX_DMA_XR_IRQ_ALL_MASK);

		DMA_OUT(&chan->regs->src, hw->buf_addr);

		/* Start the transfer
		*/
		DMA_OUT(&chan->regs->btt_ref,
			hw->control & XILINX_DMA_MAX_TRANS_LEN);
	}

out_unlock:
	spin_unlock_irqrestore(&chan->lock, flags);
}

static void xilinx_dma_issue_pending(struct dma_chan *dchan)
{
	struct xilinx_dma_chan *chan = to_xilinx_chan(dchan);
	xilinx_dma_start_transfer(chan);
}

/**
 * xilinx_dma_update_completed_cookie - Update the completed cookie.
 * @chan : xilinx DMA channel
 *
 * CONTEXT: hardirq
 */
static void xilinx_dma_update_completed_cookie(struct xilinx_dma_chan *chan)
{
	struct xilinx_dma_transfer *t;
	struct xilinx_dma_desc_hw *hw = NULL;
	unsigned long flags;
	dma_cookie_t cookie = -EBUSY;
	bool done = 0;

	spin_lock_irqsave(&chan->lock, flags);

	if (list_empty(&chan->active_list)) {
		dev_dbg(chan->dev, "no running descriptors\n");
		goto out_unlock;
	}

	if ((!(chan->feature & XILINX_DMA_IP_VDMA)) && chan->has_SG) {
		/* Get the last completed descriptor, update the cookie to that */
		list_for_each_entry(t, &chan->active_list, head) {
			for (; t->current_desc < t->num_descs; t->current_desc++) {
				hw = t->descs[t->current_desc].hw;
				if (!(hw->status & XILINX_DMA_BD_STS_ALL_MASK))
					break;
			}
			if (t->current_desc != t->num_descs)
				break;

			done = true;
			cookie = t->async_tx.cookie;
		}
	} else {
		/* In non-SG mode, there is only one transfer active at a time */
		t = list_first_entry(&chan->active_list,
				struct xilinx_dma_transfer, head);
		t->current_desc++;
		if (t->current_desc == t->num_descs) {
			done = true;
			cookie = t->async_tx.cookie;
		}
	}

	if (done)
		chan->completed_cookie = cookie;

out_unlock:
	spin_unlock_irqrestore(&chan->lock, flags);
}

/* Reset hardware
 */
static int dma_init(struct xilinx_dma_chan *chan)
{
	int loop = XILINX_DMA_RESET_LOOP;
	u32 tmp;

	DMA_OUT(&chan->regs->cr,
	       DMA_IN(&chan->regs->cr) | XILINX_DMA_CR_RESET_MASK);

	tmp = DMA_IN(&chan->regs->cr) & XILINX_DMA_CR_RESET_MASK;

	/* Wait for the hardware to finish reset
	 */
	while (loop && tmp) {
		tmp = DMA_IN(&chan->regs->cr) & XILINX_DMA_CR_RESET_MASK;
		loop -= 1;
	}

	if (!loop) {
		dev_err(chan->dev, "reset timeout, cr %x, sr %x\n",
		    DMA_IN(&chan->regs->cr), DMA_IN(&chan->regs->sr));
		return 1;
	}

	return 0;
}

static irqreturn_t dma_intr_handler(int irq, void *data)
{
	struct xilinx_dma_chan *chan = data;
	int update_cookie = 0;
	int to_transfer = 0;
	u32 stat;

	/* Disable intr
	 */
	DMA_OUT(&chan->regs->cr,
	   DMA_IN(&chan->regs->cr) & ~XILINX_DMA_XR_IRQ_ALL_MASK);

	stat = DMA_IN(&chan->regs->sr);
	if (!(stat & XILINX_DMA_XR_IRQ_ALL_MASK))
		return IRQ_NONE;

	/* Ack the interrupts
	 */
	DMA_OUT(&chan->regs->sr, XILINX_DMA_XR_IRQ_ALL_MASK);

	if (stat & XILINX_DMA_XR_IRQ_ERROR_MASK) {
		dev_err(chan->dev, "Channel %x has errors %x, cdr %x tdr %x\n",
		    (unsigned int)chan, (unsigned int)stat,
		    (unsigned int)DMA_IN(&chan->regs->cdr),
		    (unsigned int)DMA_IN(&chan->regs->tdr));
		chan->err = 1;
	}

	/* Device takes too long to do the transfer when user requires
	 * responsiveness
	 */
	if (stat & XILINX_DMA_XR_IRQ_DELAY_MASK)
		dev_dbg(chan->dev, "Inter-packet latency too long\n");

	if (stat & XILINX_DMA_XR_IRQ_IOC_MASK) {
		update_cookie = 1;
		to_transfer = 1;
	}

	if (update_cookie)
		xilinx_dma_update_completed_cookie(chan);

	if (to_transfer)
		chan->start_transfer(chan);

	tasklet_schedule(&chan->tasklet);
	return IRQ_HANDLED;
}

static void dma_do_tasklet(unsigned long data)
{
	struct xilinx_dma_chan *chan = (struct xilinx_dma_chan *)data;

	xilinx_chan_desc_cleanup(chan);
}

/* Append the descriptor list to the pending list */
static void append_desc_queue(struct xilinx_dma_chan *chan,
			struct xilinx_dma_transfer *t)
{
	struct xilinx_dma_transfer *tail = container_of(chan->pending_list.prev,
					struct xilinx_dma_transfer, head);
	struct xilinx_dma_desc_hw *hw;

	if (!list_empty(&chan->pending_list)) {
		/* Add the hardware descriptor to the chain of hardware descriptors
		 * that already exists in memory.
		 */
		hw = tail->descs[tail->num_descs-1].hw;
		hw->next_desc = t->descs[0].phys;
	}

	/* Add the software descriptor and all children to the list
	 * of pending transactions
	 */
	list_add_tail(&t->head, &chan->pending_list);
}

/* Assign cookie to each descriptor, and append the descriptors to the pending
 * list
 */
static dma_cookie_t xilinx_dma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct xilinx_dma_chan *chan = to_xilinx_chan(tx->chan);
	struct xilinx_dma_transfer *t = container_of(tx,
				struct xilinx_dma_transfer, async_tx);
	dma_cookie_t cookie = -EBUSY;
	unsigned long flags;

	if (chan->err) {
		/* If reset fails, need to hard reset the system.
		 * Channel is no longer functional
		 */
		if (!dma_init(chan))
			chan->err = 0;
		else
			return cookie;
	}

	spin_lock_irqsave(&chan->lock, flags);

	/*
	 * assign cookies to all of the software descriptors
	 * that make up this transaction
	 */
	chan->cookie++;
	if (chan->cookie <= 0)
		chan->cookie = DMA_MIN_COOKIE;

	t->async_tx.cookie = chan->cookie;
	cookie = chan->cookie;

	/* put this transaction onto the tail of the pending queue */
	append_desc_queue(chan, t);

	spin_unlock_irqrestore(&chan->lock, flags);

	return cookie;
}

static struct xilinx_dma_desc_hw *xilinx_dma_alloc_descriptor(
	struct xilinx_dma_chan *chan, dma_addr_t *phys)
{
	struct xilinx_dma_desc_hw *desc;

	desc = dma_pool_alloc(chan->desc_pool, GFP_ATOMIC, phys);
	if (!desc) {
		dev_dbg(chan->dev, "out of memory for desc\n");
		return NULL;
	}

	memset(desc, 0, sizeof(*desc));

	return desc;
}

/**
 * xilinx_dma_prep_slave_sg - prepare descriptors for a DMA_SLAVE transaction
 * @chan: DMA channel
 * @sgl: scatterlist to transfer to/from
 * @sg_len: number of entries in @scatterlist
 * @direction: DMA direction
 * @flags: transfer ack flags
 */
static struct dma_async_tx_descriptor *xilinx_dma_prep_slave_sg(
	struct dma_chan *dchan, struct scatterlist *sgl, unsigned int sg_len,
	enum dma_transfer_direction direction, unsigned long flags)
{
	struct xilinx_dma_desc_hw *new = NULL, *prev = NULL;
	struct xilinx_dma_transfer *t;
	struct xilinx_dma_chan *chan;
	unsigned int total_len = 0;
	unsigned int num_descs = 0;
	struct scatterlist *sg;
	dma_addr_t dma_src;
	dma_addr_t phys;
	size_t num_bytes;
	size_t sg_used;
	unsigned int i;

	if (!dchan)
		return NULL;

	chan = to_xilinx_chan(dchan);

	if (chan->direction != direction)
		return NULL;

	for_each_sg(sgl, sg, sg_len, i) {
		total_len += sg_dma_len(sg);
		num_descs += DIV_ROUND_UP(sg_dma_len(sg), chan->max_len);
	}

	if (!num_descs)
		return NULL;

	t = xilinx_dma_alloc_transfer(chan, num_descs);
	if (!t)
		return NULL;

	/*
	 * Build transactions using information in the scatter gather list
	 */
	for_each_sg(sgl, sg, sg_len, i) {
		sg_used = 0;

		/* Loop until the entire scatterlist entry is used */
		while (sg_used < sg_dma_len(sg)) {

			/* Allocate the link descriptor from DMA pool */
			new = xilinx_dma_alloc_descriptor(chan, &phys);
			if (!new) {
				dev_err(chan->dev, "No free memory for "
				     "link descriptor\n");
				goto err_free;
			}

			/*
			 * Calculate the maximum number of bytes to transfer,
			 * making sure it is less than the hw limit
			 */
			num_bytes = min_t(size_t, sg_dma_len(sg) - sg_used,
					chan->max_len);

			dma_src = sg_dma_address(sg) + sg_used;

			new->buf_addr = dma_src;
			new->control = num_bytes;

			if (prev)
				prev->next_desc = phys;

			prev = new;
			t->descs[t->num_descs].hw = new;
			t->descs[t->num_descs].phys = phys;
			++t->num_descs;

			sg_used += num_bytes;
		}
	}

	if (new) {
		/* Link the last BD with the first BD */
		new->next_desc = t->descs[0].phys;

		/* Set EOP to the last link descriptor of new list and
		   SOP to the first link descriptor. */
		t->descs[0].hw->control |= XILINX_DMA_BD_SOP;
		new->control |= XILINX_DMA_BD_EOP;
	}

	t->async_tx.flags = flags;
	t->async_tx.cookie = -EBUSY;

	return &t->async_tx;
err_free:
	xilinx_dma_free_transfer(chan, t);

	return NULL;
}

/**
 * xilinx_vdma_prep_slave_sg - prepare descriptors for a DMA_SLAVE transaction
 * @chan: VDMA channel
 * @sgl: scatterlist to transfer to/from
 * @sg_len: number of entries in @scatterlist
 * @direction: DMA direction
 * @flags: transfer ack flags
 */
static struct dma_async_tx_descriptor *xilinx_vdma_prep_slave_sg(
	struct dma_chan *dchan, struct scatterlist *sgl, unsigned int sg_len,
	enum dma_transfer_direction direction, unsigned long flags)
{
	struct xilinx_dma_transfer *t;
	struct xilinx_dma_chan *chan;
	struct xilinx_dma_desc_hw *new = NULL, *prev = NULL;
	dma_addr_t phys;
	struct scatterlist *sg;
	unsigned int i, j;
	dma_addr_t dma_src;

	if (!dchan)
		return NULL;

	chan = to_xilinx_chan(dchan);

	if (chan->direction != direction)
		return NULL;

	/* Enforce one sg entry for one frame */
	if (chan->num_frms % sg_len != 0) {
		dev_err(chan->dev, "number of entries %d not the "
		    "same as num stores %d\n", sg_len, chan->num_frms);

		return NULL;
	}

	t = xilinx_dma_alloc_transfer(chan, sg_len);
	if (!t)
		return NULL;

	if (!chan->has_SG) {
		DMA_OUT(&chan->addr_regs->hsize, chan->config.hsize);
		DMA_OUT(&chan->addr_regs->frmdly_stride,
		     chan->config.frm_dly << XILINX_VDMA_FRMDLY_SHIFT |
		     chan->config.stride);
	}

	for (j = 0; j < chan->num_frms / sg_len; ++j) {
		/* Build transactions using information in the scatter gather list
		 */
		for_each_sg(sgl, sg, sg_len, i) {

			/* Allocate the link descriptor from DMA pool */
			new = xilinx_dma_alloc_descriptor(chan, &phys);
			if (!new) {
				dev_err(chan->dev, "No free memory for "
					"link descriptor\n");
				goto err_free;
			}

			dma_src = sg_dma_address(sg);
			if (chan->has_SG) {
				new->buf_addr = dma_src;

				/* Fill in the descriptor */
				new->addr_vsize = chan->config.vsize;
				new->hsize = chan->config.hsize;
				new->control = (chan->config.frm_dly <<
						XILINX_VDMA_FRMDLY_SHIFT) |
						chan->config.stride;
			} else {
				/* Update the registers */
				DMA_OUT(&(chan->addr_regs->buf_addr[j * sg_len + i]), dma_src);
			}

			/* If this is not the first descriptor, chain the
			 * current descriptor after the previous descriptor
			 */
			if (prev)
				prev->next_desc = phys;

			prev = new;
			t->descs[t->num_descs].hw = new;
			t->descs[t->num_descs].phys = phys;
			++t->num_descs;
		}
	}

	if (!new)
		goto err_free;

	new->next_desc = t->descs[0].phys;


	t->async_tx.flags = flags;
	t->async_tx.cookie = -EBUSY;

	return &t->async_tx;

err_free:
	xilinx_dma_free_transfer(chan, t);
	return NULL;
}

static void xilinx_vdma_start_transfer(struct xilinx_dma_chan *chan)
{
	unsigned long flags;
	struct xilinx_dma_transfer *t;
	struct xilinx_dma_config *config;
	u32 reg;

	if (chan->err)
		return;

	spin_lock_irqsave(&chan->lock, flags);

	if (list_empty(&chan->pending_list))
		goto out_unlock;

	/* If it is SG mode and hardware is busy, cannot submit
	 */
	if (chan->has_SG && dma_is_running(chan) && !dma_is_idle(chan)) {
		dev_dbg(chan->dev, "DMA controller still busy\n");
		goto out_unlock;
	}

	/* If hardware is idle, then all descriptors on the running lists are
	 * done, start new transfers
	 */
	dma_halt(chan);

	if (chan->err)
		goto out_unlock;

	t = list_first_entry(&chan->pending_list, struct xilinx_dma_transfer, head);

	if (chan->has_SG)
		DMA_OUT(&chan->regs->cdr, t->descs[0].phys);

	/* Configure the hardware using info in the config structure */
	config = &(chan->config);
	reg = DMA_IN(&chan->regs->cr);

	if (config->frm_cnt_en)
		reg |= XILINX_VDMA_FRMCNT_EN;
	else
		reg &= ~XILINX_VDMA_FRMCNT_EN;

	/* With SG, start with circular mode, so that BDs can be fetched.
	 * In direct register mode, if not parking, enable circular mode */
	if ((chan->has_SG) || (!config->park))
		reg |= XILINX_VDMA_CIRC_EN;

	DMA_OUT(&chan->regs->cr, reg);

	if ((config->park_frm >= 0) && (config->park_frm < chan->num_frms)) {
		if (config->direction == DMA_TO_DEVICE) {
			DMA_OUT(&chan->regs->btt_ref,
			    config->park_frm << XILINX_VDMA_WR_REF_SHIFT);
		} else {
			DMA_OUT(&chan->regs->btt_ref, config->park_frm);
		}
	}

	/* Start the hardware
	 */
	dma_start(chan);

	if (chan->err)
		goto out_unlock;
	list_splice_tail_init(&chan->pending_list, &chan->active_list);

	/* Enable interrupts
	 *
	 * park/genlock testing does not use interrupts */
	if (!chan->config.disable_intr) {
		DMA_OUT(&chan->regs->cr,
		   DMA_IN(&chan->regs->cr) | XILINX_DMA_XR_IRQ_ALL_MASK);
	}

	/* Start the transfer
	 */
	if (chan->has_SG)
		DMA_OUT(&chan->regs->tdr, t->descs[t->num_descs-1].phys);
	else
		DMA_OUT(&chan->addr_regs->vsize, config->vsize);

out_unlock:
	spin_unlock_irqrestore(&chan->lock, flags);
}

static void xilinx_vdma_issue_pending(struct dma_chan *dchan)
{
	struct xilinx_dma_chan *chan = to_xilinx_chan(dchan);
	xilinx_vdma_start_transfer(chan);
}

/* Run-time configuration for Axi VDMA, supports:
 * . halt the channel
 * . configure interrupt coalescing and inter-packet delay threshold
 * . start/stop parking
 * . enable genlock
 * . set transfer information using config struct
 */
static int xilinx_vdma_device_control(struct dma_chan *dchan,
				  enum dma_ctrl_cmd cmd, unsigned long arg)
{
	struct xilinx_dma_chan *chan;

	if (!dchan)
		return -EINVAL;

	chan = to_xilinx_chan(dchan);

	if (cmd == DMA_TERMINATE_ALL) {
		/* Halt the DMA engine */
		dma_halt(chan);
		xilinx_dma_free_transfers(chan);

		return 0;
	} else if (cmd == DMA_SLAVE_CONFIG) {
		struct xilinx_dma_config *cfg = (struct xilinx_dma_config *)arg;
		u32 reg = DMA_IN(&chan->regs->cr);

		/* If vsize is -1, it is park-related operations */
		if (cfg->vsize == -1) {
			if (cfg->park)
				reg &= ~XILINX_VDMA_CIRC_EN;
			else
				reg |= XILINX_VDMA_CIRC_EN;

			DMA_OUT(&chan->regs->cr, reg);
			return 0;
		}

		/* If hsize is -1, it is interrupt threshold settings */
		if (cfg->hsize == -1) {
			if (cfg->coalesc <= XILINX_DMA_COALESCE_MAX) {
				reg &= ~XILINX_DMA_XR_COALESCE_MASK;
				reg |= cfg->coalesc <<
					XILINX_DMA_COALESCE_SHIFT;
				chan->config.coalesc = cfg->coalesc;
			}

			if (cfg->delay <= XILINX_DMA_DELAY_MAX) {
				reg &= ~XILINX_DMA_XR_DELAY_MASK;
				reg |= cfg->delay << XILINX_DMA_DELAY_SHIFT;
				chan->config.delay = cfg->delay;
			}

			DMA_OUT(&chan->regs->cr, reg);
			return 0;
		}

		/* Transfer information */
		chan->config.vsize = cfg->vsize;
		chan->config.hsize = cfg->hsize;
		chan->config.stride = cfg->stride;
		chan->config.frm_dly = cfg->frm_dly;
		chan->config.park = cfg->park;

		/* genlock settings */
		chan->config.gen_lock = cfg->gen_lock;
		chan->config.master = cfg->master;

		if (cfg->gen_lock) {
			if (chan->genlock) {
				reg |= XILINX_VDMA_SYNC_EN;
				reg |= cfg->master << XILINX_VDMA_MSTR_SHIFT;
			}
		}

		chan->config.frm_cnt_en = cfg->frm_cnt_en;
		if (cfg->park)
			chan->config.park_frm = cfg->park_frm;

		chan->config.coalesc = cfg->coalesc;
		chan->config.delay = cfg->delay;
		if (cfg->coalesc <= XILINX_DMA_COALESCE_MAX) {
			reg |= cfg->coalesc << XILINX_DMA_COALESCE_SHIFT;
			chan->config.coalesc = cfg->coalesc;
		}

		if (cfg->delay <= XILINX_DMA_DELAY_MAX) {
			reg |= cfg->delay << XILINX_DMA_DELAY_SHIFT;
			chan->config.delay = cfg->delay;
		}

		chan->config.disable_intr = cfg->disable_intr;

		DMA_OUT(&chan->regs->cr, reg);
		return 0;
	} else
		return -ENXIO;
}


/* Run-time device configuration for Axi DMA and Axi CDMA */
static int xilinx_dma_device_control(struct dma_chan *dchan,
				  enum dma_ctrl_cmd cmd, unsigned long arg)
{
	struct xilinx_dma_chan *chan;

	if (!dchan)
		return -EINVAL;

	chan = to_xilinx_chan(dchan);

	if (cmd == DMA_TERMINATE_ALL) {
		/* Halt the DMA engine */
		dma_halt(chan);
		xilinx_dma_free_transfers(chan);

		return 0;
	} else if (cmd == DMA_SLAVE_CONFIG) {
		/* Configure interrupt coalescing and delay counter
		 * Use value XILINX_DMA_NO_CHANGE to signal no change
		 */
		struct xilinx_dma_config *cfg = (struct xilinx_dma_config *)arg;
		u32 reg = DMA_IN(&chan->regs->cr);

		if (cfg->coalesc <= XILINX_DMA_COALESCE_MAX) {
			reg &= ~XILINX_DMA_XR_COALESCE_MASK;
			reg |= cfg->coalesc << XILINX_DMA_COALESCE_SHIFT;

			chan->config.coalesc = cfg->coalesc;
		}

		if (cfg->delay <= XILINX_DMA_DELAY_MAX) {
			reg &= ~XILINX_DMA_XR_DELAY_MASK;
			reg |= cfg->delay << XILINX_DMA_DELAY_SHIFT;
			chan->config.delay = cfg->delay;
		}

		DMA_OUT(&chan->regs->cr, reg);

		return 0;
	} else
		return -ENXIO;
}


/* Logarithm function to compute alignment shift
 *
 * Only deals with value less than 4096.
 */
static int my_log(int value)
{
	int i = 0;
	while ((1 << i) < value) {
		i++;

		if (i >= 12)
			return 0;
	}

	return i;
}

static void xilinx_dma_chan_remove(struct xilinx_dma_chan *chan)
{
	irq_dispose_mapping(chan->irq);
	list_del(&chan->common.device_node);
	kfree(chan);
}

/*
 * Probing channels
 *
 * . Get channel features from the device tree entry
 * . Initialize special channel handling routines
 */
static int __devinit xilinx_dma_chan_probe(struct xilinx_dma_device *xdev,
	struct device_node *node, u32 feature)
{
	struct xilinx_dma_chan *chan;
	int err;
	int *value;
	u32 width = 0;

	/* alloc channel */
	chan = kzalloc(sizeof(*chan), GFP_KERNEL);
	if (!chan) {
		dev_err(xdev->dev, "no free memory for DMA channels!\n");
		err = -ENOMEM;
		goto out_return;
	}

	chan->feature = feature;
	chan->is_lite = 0;
	chan->has_DRE = 0;
	chan->has_SG = 0;
	chan->max_len = XILINX_DMA_MAX_TRANS_LEN;

	value = (int *)of_get_property(node, "xlnx,include-dre",
			NULL);
	if (value) {
		if (be32_to_cpup(value) == 1)
			chan->has_DRE = 1;
	}

	value = (int *)of_get_property(node, "xlnx,genlock-mode",
			NULL);
	if (value) {
		if (be32_to_cpup(value) == 1)
			chan->genlock = 1;
	}

	value = (int *)of_get_property(node,
			"xlnx,datawidth",
			NULL);
	if (value) {
		width = be32_to_cpup(value) >> 3; /* convert bits to bytes */

		/* If data width is greater than 8 bytes, DRE is not in hw */
		if (width > 8)
			chan->has_DRE = 0;

		chan->feature |= width - 1;
	}

	if (feature & XILINX_DMA_IP_DMA) {
		chan->has_SG = (xdev->feature & XILINX_DMA_FTR_HAS_SG) >>
					XILINX_DMA_FTR_HAS_SG_SHIFT;

		chan->start_transfer = xilinx_dma_start_transfer;

		if (of_device_is_compatible(node,
			 "xlnx,axi-dma-mm2s-channel"))
			chan->direction = DMA_MEM_TO_DEV;

		if (of_device_is_compatible(node,
				"xlnx,axi-dma-s2mm-channel"))
			chan->direction = DMA_DEV_TO_MEM;

	}

	if (feature & XILINX_DMA_IP_VDMA) {
		chan->start_transfer = xilinx_vdma_start_transfer;

		chan->has_SG = (xdev->feature & XILINX_DMA_FTR_HAS_SG) >>
			XILINX_DMA_FTR_HAS_SG_SHIFT;

		if (of_device_is_compatible(node,
				"xlnx,axi-vdma-mm2s-channel")) {
			chan->direction = DMA_TO_DEVICE;
			if (!chan->has_SG) {
				chan->addr_regs = (struct vdma_addr_regs *)
				    ((u32)xdev->regs +
					 XILINX_VDMA_DIRECT_REG_OFFSET);
			}
		}

		if (of_device_is_compatible(node,
				"xlnx,axi-vdma-s2mm-channel")) {
			chan->direction = DMA_FROM_DEVICE;
			if (!chan->has_SG) {
				chan->addr_regs = (struct vdma_addr_regs *)
				    ((u32)xdev->regs +
					XILINX_VDMA_DIRECT_REG_OFFSET +
					XILINX_VDMA_CHAN_DIRECT_REG_SIZE);
			}
		}
	}

	chan->regs = (struct xdma_regs *)xdev->regs;
	chan->id = 0;

	if (chan->direction == DMA_FROM_DEVICE) {
		chan->regs = (struct xdma_regs *)((u32)xdev->regs +
					XILINX_DMA_RX_CHANNEL_OFFSET);
		chan->id = 1;
	}

	/* Used by dmatest channel matching in slave transfers
	 * Can change it to be a structure to have more matching information
	 */
	chan->private = (chan->direction & 0xFF) |
		(chan->feature & XILINX_DMA_IP_MASK);
	chan->common.private = (void *)&(chan->private);

	if (!chan->has_DRE)
		xdev->common.copy_align = my_log(width);

	chan->dev = xdev->dev;
	xdev->chan[chan->id] = chan;

	tasklet_init(&chan->tasklet, dma_do_tasklet, (unsigned long)chan);

	/* Initialize the channel */
	if (dma_init(chan)) {
		dev_err(xdev->dev, "Reset channel failed\n");
		goto out_free_chan;
	}


	spin_lock_init(&chan->lock);
	INIT_LIST_HEAD(&chan->pending_list);
	INIT_LIST_HEAD(&chan->active_list);

	chan->common.device = &xdev->common;

	/* find the IRQ line, if it exists in the device tree */
	chan->irq = irq_of_parse_and_map(node, 0);
	err = request_irq(chan->irq, dma_intr_handler, IRQF_SHARED,
				"xilinx-dma-controller", chan);
	if (err) {
		dev_err(xdev->dev, "unable to request IRQ\n");
		goto out_free_irq;
	}

	/* Add the channel to DMA device channel list */
	list_add_tail(&chan->common.device_node, &xdev->common.channels);
	xdev->common.chancnt++;

	return 0;

out_free_irq:
	irq_dispose_mapping(chan->irq);
out_free_chan:
	kfree(chan);
out_return:
	return err;
}

static int __devinit xilinx_dma_of_probe(struct platform_device *op)
{
	struct xilinx_dma_device *xdev;
	struct device_node *child, *node;
	int err;
	int *value;
	int num_frames = 0;

	dev_info(&op->dev, "Probing xilinx axi dma engines\n");

	xdev = kzalloc(sizeof(struct xilinx_dma_device), GFP_KERNEL);
	if (!xdev) {
		dev_err(&op->dev, "Not enough memory for device\n");
		err = -ENOMEM;
		goto out_return;
	}

	xdev->dev = &(op->dev);
	INIT_LIST_HEAD(&xdev->common.channels);

	node = op->dev.of_node;
	xdev->feature = 0;

	/* iomap registers */
	xdev->regs = of_iomap(node, 0);
	if (!xdev->regs) {
		dev_err(&op->dev, "unable to iomap registers\n");
		err = -ENOMEM;
		goto out_free_xdev;
	}

	/* Axi DMA and VDMA only do slave transfers
	 */
	if (of_device_is_compatible(node, "xlnx,axi-dma")) {

		xdev->feature |= XILINX_DMA_IP_DMA;
		value = (int *)of_get_property(node,
				"xlnx,sg-include-stscntrl-strm",
				NULL);
		if (value) {
			if (be32_to_cpup(value) == 1) {
				xdev->feature |= (XILINX_DMA_FTR_STSCNTRL_STRM |
							XILINX_DMA_FTR_HAS_SG);
			}
		}

		dma_cap_set(DMA_SLAVE, xdev->common.cap_mask);
		dma_cap_set(DMA_PRIVATE, xdev->common.cap_mask);
		xdev->common.device_prep_slave_sg = xilinx_dma_prep_slave_sg;
		xdev->common.device_control = xilinx_dma_device_control;
		xdev->common.device_issue_pending = xilinx_dma_issue_pending;
	}

	if (of_device_is_compatible(node, "xlnx,axi-vdma")) {
		xdev->feature |= XILINX_DMA_IP_VDMA;

		value = (int *)of_get_property(node, "xlnx,include-sg",
				NULL);
		if (value) {
			if (be32_to_cpup(value) == 1)
				xdev->feature |= XILINX_DMA_FTR_HAS_SG;
		}

		value = (int *)of_get_property(node, "xlnx,num-fstores",
			NULL);
		if (value)
			num_frames	= be32_to_cpup(value);

		dma_cap_set(DMA_SLAVE, xdev->common.cap_mask);
		dma_cap_set(DMA_PRIVATE, xdev->common.cap_mask);
		xdev->common.device_prep_slave_sg = xilinx_vdma_prep_slave_sg;
		xdev->common.device_control = xilinx_vdma_device_control;
		xdev->common.device_issue_pending = xilinx_vdma_issue_pending;
	}

	xdev->common.device_alloc_chan_resources =
				xilinx_dma_alloc_chan_resources;
	xdev->common.device_free_chan_resources =
				xilinx_dma_free_chan_resources;
	xdev->common.device_tx_status = xilinx_tx_status;
	xdev->common.dev = &op->dev;

	dev_set_drvdata(&op->dev, xdev);

	for_each_child_of_node(node, child) {
		xilinx_dma_chan_probe(xdev, child, xdev->feature);
	}

	if (xdev->feature & XILINX_DMA_IP_VDMA) {
		int i;

		for (i = 0; i < XILINX_DMA_MAX_CHANS_PER_DEVICE; i++) {
			if (xdev->chan[i])
				xdev->chan[i]->num_frms = num_frames;
		}
	}

	dma_async_device_register(&xdev->common);

	return 0;

out_free_xdev:
	kfree(xdev);

out_return:
	return err;
}

static int xilinx_dma_of_remove(struct platform_device *op)
{
	struct xilinx_dma_device *xdev;
	int i;

	xdev = dev_get_drvdata(&op->dev);
	dma_async_device_unregister(&xdev->common);

	for (i = 0; i < XILINX_DMA_MAX_CHANS_PER_DEVICE; i++) {
		if (xdev->chan[i])
			xilinx_dma_chan_remove(xdev->chan[i]);
	}

	iounmap(xdev->regs);
	dev_set_drvdata(&op->dev, NULL);
	kfree(xdev);

	return 0;
}

static const struct of_device_id xilinx_dma_of_ids[] = {
	{ .compatible = "xlnx,axi-dma",},
	{ .compatible = "xlnx,axi-vdma",},
	{}
};

static struct platform_driver xilinx_dma_of_driver = {
	.driver = {
		.name = "xilinx-dma",
		.owner = THIS_MODULE,
		.of_match_table = xilinx_dma_of_ids,
	},
	.probe = xilinx_dma_of_probe,
	.remove = xilinx_dma_of_remove,
};

/*----------------------------------------------------------------------------*/
/* Module Init / Exit                                                         */
/*----------------------------------------------------------------------------*/

static __init int xilinx_dma_init(void)
{
	int ret;

	pr_info("Xilinx DMA driver\n");

	ret = platform_driver_register(&xilinx_dma_of_driver);
	if (ret)
		pr_err("xilinx_dma: failed to register platform driver\n");

	return ret;
}

static void __exit xilinx_dma_exit(void)
{
	platform_driver_unregister(&xilinx_dma_of_driver);
}

subsys_initcall(xilinx_dma_init);
module_exit(xilinx_dma_exit);

MODULE_DESCRIPTION("Xilinx DMA/VDMA driver");
MODULE_LICENSE("GPL");
