/*
 * TI edma driver
 *
 * Copyright (C) 2014, Texas Instruments, Incorporated
 *
 * SPDX-License-Identifier:	GPL-2.0+
 *
 * Support for the EDMA (sometimes referred to as EDMA3) controller found
 * in a number of TI parts.  This implementation does not support QDMA nor
 * channels above 32.  We also only support region 0.  We only support manual
 * triggering of DMA events as well.
 */

#include <common.h>
#include <stdbool.h>
#include <asm/types.h>
#include <asm/io.h>
#include <asm/ti-common/edma.h>

#define EDMA_BASE_ADDR     0x43300000

static u32 base_add = EDMA_BASE_ADDR;

/* This buffer is cleared during the call to edma_init(). */
static u32 edma_zero_buff[512] __aligned(256) __attribute__ ((section(".data")));

/**
 * @brief EDMA Initialization
 *
 * This function initializes the EDMA Driver Clears the error specific
 * registers (EMCR/EMCRh, CCERRCLR) and initialize the Queue Number Registers
 *
 * @param  que_num	Event Queue Number to which the channel will be mapped
 *			(valid only for the Master Channel (DMA/QDMA) request).
 *
 * @return None
 *
 * @note We only support region 0 (of the two that exist) and the Event Queue
 * used is either (0 or 1). There are only four shadow regions and only two
 * event Queues.
 */
void edma_init(u32 que_num)
{
	int i;
	u32 addr;

	/* Clear the Event miss Registers */
	writel(EDMA_SET_ALL_BITS,(base_add + EDMA_TPCC_EMCR));
	writel(EDMA_SET_ALL_BITS,(base_add + EDMA_TPCC_EMCRH));

	/* Clear CCERR register */
	writel(EDMA_SET_ALL_BITS,(base_add + EDMA_TPCC_CCERRCLR));

	/* FOR TYPE EDMA*/
	/* Enable the DMA (0 - 32) channels in the DRAE and DRAEH register */
	writel(EDMA_SET_ALL_BITS,(base_add + EDMA_TPCC_DRAEM(0)));

	for (i = 0; i < 32; i++)
		/* All events are one to one mapped with the channels */
		writel(i << EDMA_TPCC_DCHMAPN_SHIFT,
				base_add + EDMA_TPCC_DCHMAPN(i));

	/* Initialize the DMA Queue Number Registers */
	for (i = 0; i < (SOC_EDMA_NUM_DMACH / 2); i++) {
		addr = base_add;
		addr += EDMA_TPCC_DMAQNUMN(i >> EDMA_TPCC_DMAQNUMN_SHIFT);
		writel(readl(addr) & EDMACC_DMAQNUM_CLR(i), addr);
		writel(readl(addr) | EDMACC_DMAQNUM_SET(i, que_num), addr);
	}
	memset(edma_zero_buff, 0x0, sizeof(edma_zero_buff));
}

/**
 * @brief  Map channel to Event Queue
 *
 * This API maps DMA/QDMA channels to the Event Queue.  QDMA channels are
 * not supported.
 *
 * @param  base_add    Memory address of the EDMA instance used.
 *
 * @param  ch_num      Allocated channel number.
 *
 * @param  evt_qnum    Event Queue Number to which the channel
 *                    will be mapped (valid only for the
 *                    Master Channel (DMA/QDMA) request).
 *
 * @return  None
 */
static void edma_mapch_to_evtq(u32 ch_num, u32 evt_qnum)
{
	u32 addr = base_add;

	addr += EDMA_TPCC_DMAQNUMN((ch_num) >> EDMA_TPCC_DMAQNUMN_SHIFT);

	/* Associate DMA Channel to Event Queue */
	writel(readl(addr) & EDMACC_DMAQNUM_CLR(ch_num), addr);
	writel(readl(addr) | EDMACC_DMAQNUM_SET((ch_num), evt_qnum), addr);
}

/**
 * @brief Request a DMA/QDMA/Link channel.
 *
 * Each channel (DMA/QDMA/Link) must be requested  before initiating a DMA
 * transfer on that channel.
 *
 * This API is used to allocate a logical channel (DMA/QDMA/Link) along with
 * the associated resources. For DMA and QDMA channels, TCC and PaRAM Set are
 * also allocated along with the requested channel.
 *
 * User can request a specific logical channel by passing the channel number
 * in 'ch_num'.
 *
 * For DMA/QDMA channels, after allocating all the EDMA resources, this API
 * sets the TCC field of the OPT PaRAM Word with the allocated TCC. It also
 * sets the event queue for the channel allocated. The event queue needs to
 * be specified by the user.
 *
 * For DMA channel, it also sets the DCHMAP register.
 *
 * @param  ch_num                    This is the channel number requested for a
 *                                  particular event.
 *
 * @param  tcc_num                   The channel number on which the
 *                                  completion/error interrupt is generated.
 *                                  Not used if user requested for a Link
 *                                  channel.
 *
 * @param  evt_qnum                  Event Queue Number to which the channel
 *                                  will be mapped (valid only for the
 *                                  Master Channel (DMA/QDMA) request).
 *
 * @return  true if parameters are valid, else false
 */
bool edma_request_channel(u32 ch_num, u32 tcc_num, u32 evt_qnum)
{
	bool ret_val = false;

	if (ch_num < (SOC_EDMA_NUM_DMACH / 2)) {
		/*
		 * Enable the DMA channel in the enabled in the shadow region
		 * specific register
		 */
		writel(readl(base_add + EDMA_TPCC_DRAEM(0)) | (0x01 << ch_num),
				base_add + EDMA_TPCC_DRAEM(0));

		edma_mapch_to_evtq(ch_num, evt_qnum);
		/* Interrupt channel nums are < 32 */
		if (tcc_num < (SOC_EDMA_NUM_DMACH / 2)) {
			/* Enable the Event Interrupt */
			writel(readl(base_add + EDMA_TPCC_IESR_RN(0)) |
					(0x01 << ch_num),
					base_add + EDMA_TPCC_IESR_RN(0));
			ret_val = true;
		}

		writel(readl(base_add + EDMA_TPCC_OPT(ch_num))&
				EDMACC_OPT_TCC_CLR, base_add +
				EDMA_TPCC_OPT(ch_num));
		writel(readl(base_add + EDMA_TPCC_OPT(ch_num)) |
				EDMACC_OPT_TCC_SET(ch_num), base_add +
				EDMA_TPCC_OPT(ch_num));
	}

	return ret_val;
}

/**
 * @brief Copy the user specified PaRAM Set onto the PaRAM Set associated with
 * the logical channel (DMA/Link).
 *
 * This API takes a PaRAM Set as input and copies it onto the actual PaRAM Set
 * associated with the logical channel. OPT field of the PaRAM Set is written
 * first and the CCNT field is written last.
 *
 * @param param_id	PaRAMset ID whose parameter set has to be updated
 *
 * @param new_param	Parameter RAM set to be copied onto existing PaRAM.
 *
 */
void edma_set_param(u32 param_id, struct edma_param_entry *new_param)
{
	struct edma_param_entry *dest;

	dest = (struct edma_param_entry *)(base_add + EDMA_TPCC_OPT(param_id));

	memcpy(dest, new_param, sizeof(struct edma_param_entry));
}

/**
 * @brief Start EDMA transfer on the specified channel.
 *
 * In manual triggered mode, CPU manually triggers a transfer by writing a 1
 * in the Event Set Register ESR. This API writes to the ESR to start the
 * transfer.
 *
 * @param  ch_num	Channel being used to enable transfer.
 *
 * @return true or false depending on a valid channel number being passed
 *
 */
bool edma_enable_transfer(u32 ch_num)
{
	/* Only the first half of the channels are supported. */
	if (ch_num < (SOC_EDMA_NUM_DMACH / 2)) {
		/* (ESR) - set corresponding bit to set a event */
		writel(readl(base_add + EDMA_TPCC_ESR_RN(0)) |
				(0x01 << ch_num),
				base_add + EDMA_TPCC_ESR_RN(0));
		return true;
	}
	return false;
}

/**
 * @brief This function returns interrupts status of those events which is less
 * than 32.
 *
 * @return Status of the Interrupt Pending Register
 */
u32 edma_get_intr_status(void)
{
	return readl(base_add + EDMA_TPCC_IPR_RN(0));
}

void edma_clr_intr(u32 value)
{
	writel((1 << value), base_add + EDMA_TPCC_ICR_RN(0));
}

/**
 * @brief This function clears a region of memory using EDMA. This function
 * has significantly lower execution time compared to CPU when the data cache
 * is disabled.
 *
 * @param data pointer to the buffer to be cleared.
 *
 * @param len length of the buffer
 *
 * @param edma_ch_num EDMA channel to be used for the transfer. The function
 * uses one additional PARAM set for linking. The linking PARAM set is always
 * selected at an offset of 64 channels to the edma channel performing the
 * transfer. If edma_ch_num is specified as 5, this function uses channel 5 and
 * PARAM sets 5 and 69.
 *
 * @param wait_till_complete flag indicating whether the function should return
 * only after the transfer is complete. If this flag is set to 0, the
 * programmer should call edma_wait_till_complete() before using the data
 * buffer.
 */
u32 edma_zero_memory(void *data, size_t len, unsigned int edma_ch_num,
		     u32 wait_till_complete)
{
	struct edma_param_entry edma_param;
	int bcnt = 1;
	int acnt = len;
	int ccnt = 1;
	unsigned int addr = (unsigned int) (data);
	unsigned int max_bcnt = 0xFFFFU;
	size_t rlen = 0;
	int ccnt_found = 0;

	acnt = sizeof(edma_zero_buff);

	/* Amount of data in multiples of edma_zero_buff length */
	rlen = len - (len % acnt);

	if (rlen == 0) {
		/* Less than 1 buffer of data.
		 * The first paramset is a dummy transfer.
		 * The linked paramset does the actual transfer.
		 */
		acnt = 0;
		bcnt = 1;
		ccnt = 1;
		ccnt_found = 1;
	} else {
		/* split large transfers between 2nd and 3rd dimensions */
		for (ccnt = 1; (ccnt <= 6) && (ccnt_found == 0); ccnt++) {
			bcnt = (rlen/(acnt*ccnt));
			if ((bcnt < max_bcnt) && (rlen % bcnt == 0)) {
				ccnt_found = 1;
				break;
			}
		}
	}
	if (ccnt_found == 0) {
		printf("Unable to configure EDMA exiting\n");
		return 1;
	}

	/* base param set does tranfers in multiples of edma_zero_buff length*/
	edma_param.opt       = 0;
	edma_param.src_addr  = ((unsigned int) edma_zero_buff);
	edma_param.dest_addr = addr;
	edma_param.a_cnt     = acnt;
	edma_param.b_cnt     = bcnt;
	edma_param.c_cnt     = ccnt;
	edma_param.src_bidx  = 0;
	edma_param.dest_bidx = acnt;
	edma_param.b_cnt_reload = bcnt;
	edma_param.src_cidx  = 0;
	edma_param.dest_cidx = 0;
	edma_param.link_addr = (edma_ch_num+64)*32;
	edma_param.opt     |=
		(EDMA_TPCC_OPT_TCCHEN_MASK |
		 ((edma_ch_num <<
		   EDMA_TPCC_OPT_TCC_SHIFT) &
		  EDMA_TPCC_OPT_TCC_MASK) |
		 EDMA_TPCC_OPT_ITCCHEN_MASK);

	edma_set_param(edma_ch_num, &edma_param);

	/* linked param set transfers remaining data if any */
	edma_param.opt       = 0;
	edma_param.src_addr  = ((unsigned int) edma_zero_buff);
	edma_param.dest_addr = addr + rlen;
	edma_param.a_cnt     = len-rlen;
	edma_param.b_cnt     = 1;
	edma_param.c_cnt     = 1;
	edma_param.src_bidx  = 0;
	edma_param.dest_bidx = 1;
	edma_param.b_cnt_reload = bcnt;
	edma_param.src_cidx  = 0;
	edma_param.dest_cidx = 0;
	edma_param.link_addr = 0xFFFF;
	edma_param.opt     |=
		(EDMA_TPCC_OPT_TCINTEN_MASK |
		 ((edma_ch_num <<
		   EDMA_TPCC_OPT_TCC_SHIFT) &
		  EDMA_TPCC_OPT_TCC_MASK));

	edma_set_param((edma_ch_num+64), &edma_param);

	edma_enable_transfer(edma_ch_num);
	if (wait_till_complete)
		edma_wait_till_complete(edma_ch_num, 0);
	return 0;
}

/**
 * @brief Wait until a transfer on a given channel is complete.
 *
 * @param edma_ch_num Number of the edma channel whose completion this function
 * checks for. It is is assumed that there is a 1-to-1 mapping between edma
 * channels and TCC's. If not, supply the TCC code as argument instead of the
 * edma channel number.
 *
 * @param wait_cnt Number of times the function should check for completion.
 * Set to 0 if this function should only return on completion of transfer.
 */
u32 edma_wait_till_complete(unsigned int edma_ch_num, u32 wait_cnt)
{
	if (wait_cnt == 0) {
		/* Large enough to be wait forever */
		wait_cnt = 0x0FFFFFFF;
	}
	while (!(edma_get_intr_status() & (1 << edma_ch_num))) {
		wait_cnt--;
		if (wait_cnt == 0)
			return 1;
	}
	edma_clr_intr(edma_ch_num);
	return 0;
}
