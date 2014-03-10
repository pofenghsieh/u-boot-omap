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

/***********************************************************************/
/* Values that can be used by the application for configuration via APIs*/
/***********************************************************************/
/** Maximum number of EDMA Shadow regions available */
#define EDMA3_MAX_REGIONS                     (8U)

/** Number of PaRAM Sets available */
#define EDMA3_NUM_PARAMSET                    (512U)

/** Number of Event Queues available */
#define EDMA3_0_NUM_EVTQUE                    (4U)

/** Number of Transfer Controllers available */
#define EDMA3_0_NUM_TC                        (4U)

/** Interrupt no. for Transfer Completion */
#define EDMA3_0_CC_XFER_COMPLETION_INT        (11U)

/** Interrupt no. for CC Error */
#define EDMA3_0_CC0_ERRINT                    (12U)

/** Interrupt no. for TCs Error */
#define EDMA3_0_TC0_ERRINT                    (13U)
#define EDMA3_0_TC1_ERRINT                    (32U)

#define EDMA3CC_DMA_CHANNELS                  (64U)
#define EDMA3CC_QDMA_CHANNELS                 (8U)

/** DMAQNUM bits Clear */
#define EDMA3CC_DMAQNUM_CLR(ch_num)            ( ~ (0x7U << (((ch_num) % 8U) \
                                                                      * 4U)))
/** DMAQNUM bits Set */
#define EDMA3CC_DMAQNUM_SET(ch_num,que_num)     ((0x7U & (que_num)) << \
                                                       (((ch_num) % 8U) * 4U))
/** QDMAQNUM bits Clear */
#define EDMA3CC_QDMAQNUM_CLR(ch_num)           ( ~ (0x7U << ((ch_num) * 4U)))
/** QDMAQNUM bits Set */
#define EDMA3CC_QDMAQNUM_SET(ch_num,que_num)    ((0x7U & (que_num)) << \
                                                           ((ch_num) * 4U))

#define EDMA3CC_QCHMAP_PAENTRY_CLR            (~EDMA_TPCC_QCHMAPN_PAENTRY_MASK)
/** QCHMAP-PaRAMEntry bitfield Set */
#define EDMA3CC_QCHMAP_PAENTRY_SET(param_id)   (((EDMA_TPCC_QCHMAPN_PAENTRY_MASK >> \
                                              EDMA_TPCC_QCHMAPN_PAENTRY_SHIFT) & \
                                              (param_id)) << \
                                              EDMA_TPCC_QCHMAPN_PAENTRY_SHIFT)
/** QCHMAP-TrigWord bitfield Clear */
#define EDMA3CC_QCHMAP_TRWORD_CLR             (~EDMA_TPCC_QCHMAPN_TRWORD_MASK)
/** QCHMAP-TrigWord bitfield Set */
#define EDMA3CC_QCHMAP_TRWORD_SET(param_id)    (((EDMA_TPCC_QCHMAPN_TRWORD_MASK >> \
                                              EDMA_TPCC_QCHMAPN_TRWORD_SHIFT) & \
                                              (param_id)) << \
                                              EDMA_TPCC_QCHMAPN_TRWORD_SHIFT)

/** OPT-TCC bitfield Clear */
#define EDMA3CC_OPT_TCC_CLR                   (~EDMA_TPCC_OPT_TCC_MASK)

/** OPT-TCC bitfield Set */
#define EDMA3CC_OPT_TCC_SET(tcc)              (((EDMA_TPCC_OPT_TCC_MASK >> \
                                              EDMA_TPCC_OPT_TCC_SHIFT) & \
                                              (tcc)) << EDMA_TPCC_OPT_TCC_SHIFT)

#define EDMA3_SET_ALL_BITS                    (0xFFFFFFFFU)
#define EDMA3_CLR_ALL_BITS                    (0x00000000U)

#define EDMA3CC_COMPL_HANDLER_RETRY_COUNT     (10U)
#define EDMA3CC_ERR_HANDLER_RETRY_COUNT       (10U)

#define EDMA3_TRIG_MODE_MANUAL                (0U)
#define EDMA3_TRIG_MODE_QDMA                  (1U)
#define EDMA3_TRIG_MODE_EVENT                 (2U)

#define EDMA3_CHANNEL_TYPE_DMA                (0U)
#define EDMA3_CHANNEL_TYPE_QDMA               (1U)


#define EDMA3_XFER_COMPLETE                   (0U)
#define EDMA3_CC_DMA_EVT_MISS                 (1U)
#define EDMA3_CC_QDMA_EVT_MISS                (2U)

#define EDMA3_SYNC_A                          (0U)
#define EDMA3_SYNC_AB                         (1U)

#define EDMA3CC_CLR_TCCERR                     (EDMA_TPCC_CCERRCLR_TCERR_MASK)
#define EDMA3CC_CLR_QTHRQ0                     (EDMA_TPCC_CCERRCLR_QTHRXCD0_MASK)
#define EDMA3CC_CLR_QTHRQ1                     (EDMA_TPCC_CCERRCLR_QTHRXCD1_MASK)

#define EDMA3_OPT_TCCHEN_MASK                  EDMA_TPCC_OPT_TCCHEN_MASK
#define EDMA3_OPT_ITCCHEN_MASK                 EDMA_TPCC_OPT_ITCCHEN_MASK
#define EDMA3_OPT_TCINTEN_MASK                 EDMA_TPCC_OPT_TCINTEN_MASK
#define EDMA3_OPT_ITCINTEN_MASK                EDMA_TPCC_OPT_ITCINTEN_MASK

#define EDMA3_OPT_TCC_MASK                     EDMA_TPCC_OPT_TCC_MASK
#define EDMA3_OPT_TCC_SHIFT                    EDMA_TPCC_OPT_TCC_SHIFT


/* paRAMEntry Fields*/
    /**
     * The OPT field (Offset Address 0x0 Bytes)
     */
#define    EDMA3CC_PARAM_ENTRY_OPT            (0x0U)

    /**
     * The SRC field (Offset Address 0x4 Bytes)
     */
#define    EDMA3CC_PARAM_ENTRY_SRC            (0x1U)

    /**
     * The (ACNT+BCNT) field (Offset Address 0x8 Bytes)
     */
#define    EDMA3CC_PARAM_ENTRY_ACNT_BCNT      (0x2U)

    /**
     * The DST field (Offset Address 0xC Bytes)
     */
#define    EDMA3CC_PARAM_ENTRY_DST            (0x3U)

    /**
     * The (SRCBIDX+DSTBIDX) field (Offset Address 0x10 Bytes)
     */
#define    EDMA3CC_PARAM_ENTRY_SRC_DST_BIDX   (0x4U)

    /**
     * The (LINK+BCNTRLD) field (Offset Address 0x14 Bytes)
     */
#define    EDMA3CC_PARAM_ENTRY_LINK_BCNTRLD   (0x5U)

    /**
     * The (SRCCIDX+DSTCIDX) field (Offset Address 0x18 Bytes)
     */
#define    EDMA3CC_PARAM_ENTRY_SRC_DST_CIDX   (0x6U)

    /**
     * The (CCNT+RSVD) field (Offset Address 0x1C Bytes)
     */
#define    EDMA3CC_PARAM_ENTRY_CCNT           (0x7U)


/** The offset for each PaRAM Entry field  */
#define    EDMA3CC_PARAM_FIELD_OFFSET         (0x4U)

/** Number of PaRAM Entry fields
  * OPT, SRC, A_B_CNT, DST, SRC_DST_BIDX, LINK_BCNTRLD, SRC_DST_CIDX
  * and CCNT
  */
#define    EDMA3CC_PARAM_ENTRY_FIELDS         (0x8U)



#define SOC_EDMA3_NUM_DMACH                 64
#define SOC_EDMA3_NUM_QDMACH                8
#define SOC_EDMA3_NUM_PARAMSETS             512
#define SOC_EDMA3_NUM_EVQUE                 4
#define SOC_EDMA3_CHMAPEXIST                0
#define SOC_EDMA3_NUM_REGIONS               8
#define SOC_EDMA3_MEMPROTECT                0


/** Number of TCCS available */
#define EDMA3_NUM_TCC                        (SOC_EDMA3_NUM_DMACH)

#define SOC_EDMA_TPCC_BASE      0x43300000

#define HWREG(X) (*((volatile u32*)(X)))

#define TRUE 1
#define FALSE 0

/**
 * \brief EDMA3 Parameter RAM Set in User Configurable format
 *
 * This is a mapping of the EDMA3 PaRAM set provided to the user
 * for ease of modification of the individual fields
 */
typedef struct EDMA3CCPaRAMEntry {
        /** OPT field of PaRAM Set */
        u32 opt;

        /**
         * \brief Starting byte address of Source
         * For FIFO mode, src_addr must be a 256-bit aligned address.
         */
        u32 src_addr;

        /**
         * \brief Number of bytes in each Array (ACNT)
         */
        u16 a_cnt;

        /**
         * \brief Number of Arrays in each Frame (BCNT)
         */
        u16 b_cnt;

        /**
         * \brief Starting byte address of destination
         * For FIFO mode, dest_addr must be a 256-bit aligned address.
         * i.e. 5 LSBs should be 0.
         */
        u32 dest_addr;

        /**
         * \brief Index between consec. arrays of a Source Frame (SRCBIDX)
         */
        s16  src_bidx;

        /**
         * \brief Index between consec. arrays of a Destination Frame (DSTBIDX)
         */
        s16  dest_bidx;

        /**
         * \brief Address for linking (AutoReloading of a PaRAM Set)
         * This must point to a valid aligned 32-byte PaRAM set
         * A value of 0xFFFF means no linking
         */
        u16 link_addr;

        /**
         * \brief Reload value of the numArrInFrame (BCNT)
         * Relevant only for A-sync transfers
         */
        u16 b_cnt_reload;

        /**
         * \brief Index between consecutive frames of a Source Block (SRCCIDX)
         */
        s16  src_cidx;

        /**
         * \brief Index between consecutive frames of a Dest Block (DSTCIDX)
         */
        s16  dest_cidx;

        /**
         * \brief Number of Frames in a block (CCNT)
         */
        u16 c_cnt;
}EDMA3CCPaRAMEntry;

void edma3_init(u32 que_num);

void edma3_set_region(u32 i);

u32 edma3_request_channel(u32 ch_type,
                          u32 ch_num, u32 tcc_num,
                          u32 evt_qnum);

void edma3_set_param(u32 param_id,
                     EDMA3CCPaRAMEntry* new_param);

u32 edma3_enable_transfer(u32 ch_num,
                          u32 trig_mode);

u32 edma3_get_intr_status(void);

void edma3_clr_intr(u32 value);

void edma3_deinit(u32 que_num);
