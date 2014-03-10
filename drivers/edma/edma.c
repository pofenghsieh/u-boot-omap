/*
 * TI edma driver
 *
 * Copyright (C) 2014, Texas Instruments, Incorporated
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR /PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <asm/types.h>
#include <asm/io.h>
#include "edma.h"
#include "hw_edma_tc.h"
#include "hw_edma_tpcc.h"

/****************************************************************************/
/*                     API FUNCTION DEFINITIONS                             */
/****************************************************************************/
#define EDMA3_BASE_ADDR     0x43300000

u32 region_id;
u32 base_add = EDMA3_BASE_ADDR;

/***************Internal function declaration ***************/

void edma3_enablech_in_shadowreg(u32 ch_type,u32 ch_num);
void edma3_mapch_to_evtq(u32 ch_type,u32 ch_num,u32 evt_qnum);
void edma3_enable_evt_intr(u32 ch_num);
void edma3_set_evt(u32 ch_num);
void edma3_clr_miss_evt(u32 ch_num);
void edma3_enable_dma_evt(u32 ch_num);

/**
 *  \brief   EDMA3 Initialization
 *
 *  This function initializes the EDMA3 Driver
 *  Clears the error specific registers (EMCR/EMCRh, QEMCR, CCERRCLR) &
 *  initialize the Queue Number Registers
 *
 *  \param  base_add                  Memory address of the EDMA instance used.\n
 *
 *  \param  que_num                   Event Queue Number to which the channel
 *                                   will be mapped (valid only for the
 *                                   Master Channel (DMA/QDMA) request).\n
 *
 *  \return None
 *
 *  \note   The region_id is the shadow region(0 or 1) used and the,
 *          Event Queue used is either (0 or 1). There are only four shadow
 *          regions and only two event Queues
 */
void edma3_init(u32 que_num)
{
    u32 count = 0;
    u32 i     = 0;

    region_id = 0;

    /* Clear the Event miss Registers                                       */
    writel(EDMA3_SET_ALL_BITS,(base_add + EDMA_TPCC_EMCR));
    writel(EDMA3_SET_ALL_BITS,(base_add + EDMA_TPCC_EMCRH));

    writel(EDMA3_SET_ALL_BITS,(base_add + EDMA_TPCC_QEMCR));

    /* Clear CCERR register                                                 */
    writel(EDMA3_SET_ALL_BITS,(base_add + EDMA_TPCC_CCERRCLR));

    /* FOR TYPE EDMA*/
    /* Enable the DMA (0 - 64) channels in the DRAE and DRAEH register */

    writel(EDMA3_SET_ALL_BITS,(base_add + EDMA_TPCC_DRAEM(region_id)));
    writel(EDMA3_SET_ALL_BITS,(base_add + EDMA_TPCC_DRAEHM(region_id)));

    for (i = 0; i < 64; i++)
    {
        /* All events are one to one mapped with the channels */
        writel(i << 5,base_add + EDMA_TPCC_DCHMAPN(i));
    }

    /* Initialize the DMA Queue Number Registers                            */
    for (count = 0; count < SOC_EDMA3_NUM_DMACH; count++)
    {
        writel(readl(base_add + EDMA_TPCC_DMAQNUMN(count >> 3U))&EDMA3CC_DMAQNUM_CLR(count),
              base_add + EDMA_TPCC_DMAQNUMN(count >> 3U));
        writel(readl(base_add + EDMA_TPCC_DMAQNUMN(count >> 3U))|EDMA3CC_DMAQNUM_SET(count, que_num),
              base_add + EDMA_TPCC_DMAQNUMN(count >> 3U));
    }

    /* FOR TYPE QDMA */
    /* Enable the DMA (0 - 64) channels in the DRAE register                */
    writel(EDMA3_SET_ALL_BITS,(base_add + EDMA_TPCC_QRAEN(region_id)));

    /* Initialize the QDMA Queue Number Registers                           */
    for (count = 0; count < SOC_EDMA3_NUM_QDMACH; count++)
    {
        writel(readl(base_add + EDMA_TPCC_QDMAQNUM)& EDMA3CC_QDMAQNUM_CLR(count),
              base_add + EDMA_TPCC_QDMAQNUM);
        writel(readl(base_add + EDMA_TPCC_QDMAQNUM)|EDMA3CC_QDMAQNUM_SET(count, que_num),
              base_add + EDMA_TPCC_QDMAQNUM);
    }
}
/**
 *  \brief Request a DMA/QDMA/Link channel.
 *
 *  Each channel (DMA/QDMA/Link) must be requested  before initiating a DMA
 *  transfer on that channel.
 *
 *  This API is used to allocate a logical channel (DMA/QDMA/Link) along with
 *  the associated resources. For DMA and QDMA channels, TCC and PaRAM Set are
 *  also allocated along with the requested channel.
 *
 *  User can request a specific logical channel by passing the channel number
 *  in 'ch_num'.
 *
 *  For DMA/QDMA channels, after allocating all the EDMA3 resources, this API
 *  sets the TCC field of the OPT PaRAM Word with the allocated TCC. It also
 *  sets the event queue for the channel allocated. The event queue needs to
 *  be specified by the user.
 *
 *  For DMA channel, it also sets the DCHMAP register.
 *
 *  For QDMA channel, it sets the QCHMAP register and CCNT as trigger word and
 *  enables the QDMA channel by writing to the QEESR register.
 *
 *  \param  base_add                  Memory address of the EDMA instance used.\n
 *
 *  \param  ch_type                   (DMA/QDMA) Channel
 *                                    For Example: For DMA it is
 *                                    EDMA3_CHANNEL_TYPE_DMA.\n
 *
 *  \param  ch_num                    This is the channel number requested for a
 *                                   particular event.\n
 *
 *  \param  tcc_num                   The channel number on which the
 *                                   completion/error interrupt is generated.
 *                                   Not used if user requested for a Link
 *                                   channel.\n
 *
 *  \param  evt_qnum                  Event Queue Number to which the channel
 *                                   will be mapped (valid only for the
 *                                   Master Channel (DMA/QDMA) request).\n
 *
 *  \return  TRUE if parameters are valid, else FALSE
 */
u32 edma3_request_channel( u32 ch_type,
                          u32 ch_num,
                          u32 tcc_num,
                          u32 evt_qnum)
{
    u32 ret_val = FALSE;
    if (ch_num < SOC_EDMA3_NUM_DMACH)
    {
        /* Enable the DMA channel in the enabled in the shadow region
         * specific register
         */
        edma3_enablech_in_shadowreg(ch_type, ch_num);

        edma3_mapch_to_evtq(ch_type, ch_num, evt_qnum);
        if (EDMA3_CHANNEL_TYPE_DMA == ch_type)
        {
            /* Interrupt channel nums are < 32 */
            if (tcc_num < SOC_EDMA3_NUM_DMACH)
            {
                /* Enable the Event Interrupt                             */
                edma3_enable_evt_intr(ch_num);
                ret_val = TRUE;
            }

            writel(readl(base_add + EDMA_TPCC_OPT(ch_num))& EDMA3CC_OPT_TCC_CLR,
                   base_add + EDMA_TPCC_OPT( ch_num));
            writel( readl(base_add + EDMA_TPCC_OPT(ch_num))|EDMA3CC_OPT_TCC_SET(ch_num),
                     base_add + EDMA_TPCC_OPT(ch_num));
        }
        else if (EDMA3_CHANNEL_TYPE_QDMA == ch_type)
        {
            /* Interrupt channel nums are < 8 */
            if (tcc_num < SOC_EDMA3_NUM_QDMACH)
            {
                /* Enable the Event Interrupt                             */
                edma3_enable_evt_intr(ch_num);
                ret_val = TRUE;
            }

            writel(readl(base_add + EDMA_TPCC_OPT(ch_num))& EDMA3CC_OPT_TCC_CLR,
                   base_add + EDMA_TPCC_OPT( ch_num));
            writel( readl(base_add + EDMA_TPCC_OPT(ch_num))|EDMA3CC_OPT_TCC_SET(ch_num),
                     base_add + EDMA_TPCC_OPT(ch_num));
        }
        else
        {
            /*An error will be generated automatically.*/
        }
    }
    return ret_val;
}


/**
 * \brief  Enable channel to Shadow region mapping
 *
 * This API allocates DMA/QDMA channels or TCCs, and the same resources are
 * enabled in the shadow region specific register (DRAE/DRAEH/QRAE).
 * Here only one shadow region is used since, there is only one Master.
 *
 *  \param   base_add     Memory address of the EDMA instance used.\n
 *
 *  \param   ch_type      (DMA/QDMA) Channel
 *                        For Example: For DMA it is,
 *                        EDMA3_CHANNEL_TYPE_DMA.\n
 *
 *  \param   ch_num       Allocated channel number.\n
 *
 *  ch_type can have values
 *        EDMA3_CHANNEL_TYPE_DMA\n
 *        EDMA3_CHANNEL_TYPE_QDMA
 *
 *  \return  None
 */
void edma3_enablech_in_shadowreg(u32 ch_type,
                                 u32 ch_num)
{
    /* Allocate the DMA/QDMA channel */
    if (EDMA3_CHANNEL_TYPE_DMA == ch_type)
    {
        /* FOR TYPE EDMA*/
        if (ch_num < 32)
        {
            /* Enable the DMA channel in the DRAE registers */
            writel(readl(base_add + EDMA_TPCC_DRAEM(region_id))|(0x01U << ch_num),
                    base_add + EDMA_TPCC_DRAEM( region_id));
        }
        else
        {
            /* Enable the DMA channel in the DRAEH registers */
            writel(readl(base_add + EDMA_TPCC_DRAEHM(region_id)) | (0x01U << (ch_num - 32)),
                   base_add + EDMA_TPCC_DRAEHM(region_id));
        }
    }
    else if (EDMA3_CHANNEL_TYPE_QDMA == ch_type)
    {
        /* FOR TYPE QDMA */
        /* Enable the QDMA channel in the DRAE/DRAEH registers */
        writel(readl(base_add + EDMA_TPCC_QRAEN(region_id))|0x01U << ch_num,
                base_add + EDMA_TPCC_QRAEN(region_id));
    }
    else
    {
        /*An error will be generated automatically.*/
    }
}

/**
 *  \brief  Map channel to Event Queue
 *
 *  This API maps DMA/QDMA channels to the Event Queue
 *
 *  \param  base_add    Memory address of the EDMA instance used.\n
 *
 *  \param  ch_type     (DMA/QDMA) Channel
 *                     For Example: For QDMA it is
 *                     EDMA3_CHANNEL_TYPE_QDMA.\n
 *
 *  \param  ch_num      Allocated channel number.\n
 *
 *  \param  evt_qnum    Event Queue Number to which the channel
 *                     will be mapped (valid only for the
 *                     Master Channel (DMA/QDMA) request).\n
 *
 *  ch_type can have values
 *        EDMA3_CHANNEL_TYPE_DMA\n
 *        EDMA3_CHANNEL_TYPE_QDMA
 *
 *  \return  None
 */
void edma3_mapch_to_evtq(u32 ch_type,
                          u32 ch_num,
                          u32 evt_qnum)
{
    if (EDMA3_CHANNEL_TYPE_DMA == ch_type)
    {
        /* Associate DMA Channel to Event Queue */
        writel( readl(base_add + EDMA_TPCC_DMAQNUMN((ch_num) >> 3U)) &
                EDMA3CC_DMAQNUM_CLR(ch_num),
                base_add + EDMA_TPCC_DMAQNUMN((ch_num) >> 3U));

        writel(readl(base_add + EDMA_TPCC_DMAQNUMN((ch_num) >> 3U)) |
               EDMA3CC_DMAQNUM_SET((ch_num), evt_qnum),
               base_add + EDMA_TPCC_DMAQNUMN((ch_num) >> 3U));
    }
    else if (EDMA3_CHANNEL_TYPE_QDMA == ch_type)
    {
        /* Associate QDMA Channel to Event Queue */
        writel(readl(base_add + EDMA_TPCC_QDMAQNUM)|
            EDMA3CC_QDMAQNUM_SET(ch_num, evt_qnum),
            base_add + EDMA_TPCC_QDMAQNUM);

    }
    else
    {
        /*An error will be generated automatically.*/
    }
}

/**
 *  \brief   Enables the user to enable the transfer completion interrupt
 *           generation by the EDMA3CC for all DMA/QDMA channels.
 *
 *  \param   base_add                Memory address of the EDMA instance used.\n
 *
 *  \param   ch_num                  Allocated channel number.\n
 *
 *  \return  None
 *
 *  Note :   To set any interrupt bit in IER, a 1 must be written to the
 *           corresponding interrupt bit in the interrupt enable set register.
 */
void edma3_enable_evt_intr(u32 ch_num)
{
    if (ch_num < 32)
    {
        /*  Interrupt Enable Set Register (IESR)
         *                               */
        writel(readl(base_add + EDMA_TPCC_IESR_RN(region_id))|(0x01U << ch_num),
               base_add + EDMA_TPCC_IESR_RN(region_id));
    }
    else
    {
        /*  Interrupt Enable Set Register (IESRH)
         *                               */
        writel(readl(base_add + EDMA_TPCC_IESRH_RN(region_id))|(0x01U << (ch_num - 32)),
               base_add + EDMA_TPCC_IESRH_RN(region_id));
    }
}

/**
 * \brief   Copy the user specified PaRAM Set onto the PaRAM Set associated
 *          with the logical channel (DMA/Link).
 *
 * This API takes a PaRAM Set as input and copies it onto the actual PaRAM Set
 * associated with the logical channel. OPT field of the PaRAM Set is written
 * first and the CCNT field is written last.
 *
 *
 * \param   base_add                Memory address of the EDMA instance used.\n
 *
 * \param   param_id                paRAMset ID whose parameter set has to be
 *                                 updated
 *
 * \param   new_param               Parameter RAM set to be copied onto existing
 *                                 PaRAM.\n
 *
 * \return  None
 */
void edma3_set_param(u32           param_id,
                     EDMA3CCPaRAMEntry *new_param)
{
    u32           i  = 0;
    u32          *sr = (u32 *) new_param;
    volatile u32 *ds;

    ds = (u32 *) (base_add + EDMA_TPCC_OPT(param_id));

    for (i = 0; i < EDMA3CC_PARAM_ENTRY_FIELDS; i++)
    {
        *ds = *sr;
        ds++;
        sr++;
    }
}


/**
 *  \brief    Start EDMA transfer on the specified channel.
 *
 *  There are multiple ways to trigger an EDMA3 transfer. The triggering mode
 *  option allows choosing from the available triggering modes: Event,
 *  Manual or QDMA.
 *
 *  In event triggered, a peripheral or an externally generated event triggers
 *  the transfer. This API clears the Event and Event Miss Register and then
 *  enables the DMA channel by writing to the EESR.
 *
 *  In manual triggered mode, CPU manually triggers a transfer by writing a 1
 *  in the Event Set Register ESR. This API writes to the ESR to start the
 *  transfer.
 *
 *  In QDMA triggered mode, a QDMA transfer is triggered when a CPU (or other
 *  EDMA3 programmer) writes to the trigger word of the QDMA channel PaRAM set
 *  (auto-triggered) or when the EDMA3CC performs a link update on a PaRAM set
 *  that has been mapped to a QDMA channel (link triggered). This API enables
 *  the QDMA channel by writing to the QEESR register.
 *
 *  \param  base_add         Memory address of the EDMA instance used.\n
 *
 *  \param  ch_num           Channel being used to enable transfer.\n
 *
 *  \param  trig_mode        Mode of triggering start of transfer (Manual,
 *                          QDMA or Event).\n
 *
 *  trig_mode can have values:
 *        EDMA3_TRIG_MODE_MANUAL\n
 *        EDMA3_TRIG_MODE_QDMA\n
 *        EDMA3_TRIG_MODE_EVENT\n
 *
 *  \return  ret_val         TRUE or FALSE depending on the param passed.\n
 *
 */
u32 edma3_enable_transfer(u32 ch_num,
                          u32 trig_mode)
{
    u32 ret_val = FALSE;
    switch (trig_mode)
    {
        case EDMA3_TRIG_MODE_MANUAL:
            if (ch_num < SOC_EDMA3_NUM_DMACH)
            {
                edma3_set_evt(ch_num);
                ret_val = TRUE;
            }
            break;

        case EDMA3_TRIG_MODE_EVENT:
            if (ch_num < SOC_EDMA3_NUM_DMACH)
            {
                /*clear SECR & EMCR to clean any previous NULL request    */
                edma3_clr_miss_evt(ch_num);

                /* Set EESR to enable event                               */
                edma3_enable_dma_evt(ch_num);
                ret_val = TRUE;
            }
            break;

        default:
            ret_val = FALSE;
            break;
    }
    return ret_val;
}

void edma3_set_evt(u32 ch_num)
{
    if (ch_num < 32)
    {
        /* (ESR) - set corresponding bit to set a event
         *                        */
        writel(readl(base_add + EDMA_TPCC_ESR_RN(region_id)) | (0x01U << ch_num),
               base_add + EDMA_TPCC_ESR_RN(region_id));

    }
    else
    {
        /* (ESRH) - set corresponding bit to set a event
         *                        */
        writel(readl(base_add + EDMA_TPCC_ESRH_RN(region_id)) | (0x01U << (ch_num - 32)),
                base_add + EDMA_TPCC_ESRH_RN(region_id));

    }
}

void edma3_clr_miss_evt(u32 ch_num)
{
    if (ch_num < 32)
    {
        /*clear SECR to clean any previous NULL request
         *                        */
        writel((0x01U << ch_num),base_add + EDMA_TPCC_SECR_RN(region_id));
        /*clear EMCR to clean any previous NULL request
         *                        */
        writel(readl(base_add + EDMA_TPCC_EMCR) | (0x01U << ch_num),
              base_add + EDMA_TPCC_EMCR);
    }
    else
    {
        writel(0x01U << (ch_num - 32),base_add + EDMA_TPCC_SECRH_RN(region_id));
        /*clear EMCRH to clean any previous NULL request
         *                        */
        writel(readl(base_add + EDMA_TPCC_EMCRH) | (0x01U << (ch_num - 32)),
               base_add + EDMA_TPCC_EMCRH);
    }
}

void edma3_enable_dma_evt(u32 ch_num)
{
    if (ch_num < 32)
    {
        /* (EESR) - set corresponding bit to enable DMA event
         *                  */
        writel(readl(base_add + EDMA_TPCC_EESR_RN(region_id))|(0x01U << ch_num),
               base_add + EDMA_TPCC_EESR_RN(region_id));
    }
    else
    {
        /* (EESRH) - set corresponding bit to enable DMA event
         *                  */
        writel( readl(base_add + EDMA_TPCC_EESRH_RN(region_id)) | (0x01U << (ch_num - 32)),
                base_add + EDMA_TPCC_EESRH_RN(region_id));
    }
}

/**
 *  \brief   This function returns interrupts status of those events
 *           which is less than 32.
 *
 *  \param   base_add                Memory address of the EDMA instance used.\n
 *
 *  \return  value                  Status of the Interrupt Pending Register
 *
 */
u32 edma3_get_intr_status(void)
{
    u32 intr_status_val = 0;

    intr_status_val = (u32) readl(base_add + EDMA_TPCC_IPR_RN(region_id));

    return intr_status_val;
}

void edma3_clr_intr(u32 value)
{
    if (value < 32)
    {
        writel((1U << value),base_add + EDMA_TPCC_ICR_RN(region_id));
    }
    else
    {
        writel((1U << (value - 32)),base_add + EDMA_TPCC_ICRH_RN(region_id));
    }
}
