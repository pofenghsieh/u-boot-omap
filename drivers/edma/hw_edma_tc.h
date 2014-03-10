/*
 * hw_edma_tc.h - register-level header file for EDMA_TC
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

#ifndef HW_EDMA_TC_H_
#define HW_EDMA_TC_H_

/****************************************************************************************************
* Register Definitions
****************************************************************************************************/
#define EDMA_TC_PID		(0x0U)
#define EDMA_TC_TCCFG	(0x4U)
#define EDMA_TC_TCSTAT	(0x100U)
#define EDMA_TC_INTSTAT	(0x104U)
#define EDMA_TC_INTEN	(0x108U)
#define EDMA_TC_INTCLR	(0x10cU)
#define EDMA_TC_INTCMD	(0x110U)
#define EDMA_TC_ERRSTAT	(0x120U)
#define EDMA_TC_ERREN	(0x124U)
#define EDMA_TC_ERRCLR	(0x128U)
#define EDMA_TC_ERRDET	(0x12cU)
#define EDMA_TC_ERRCMD	(0x130U)
#define EDMA_TC_RDRATE	(0x140U)
#define EDMA_TC_POPT	(0x0U)
#define EDMA_TC_PSRC	(0x4U)
#define EDMA_TC_PCNT	(0x8U)
#define EDMA_TC_PDST	(0xcU)
#define EDMA_TC_PBIDX	(0x10U)
#define EDMA_TC_PMPPRXY	(0x14U)
#define EDMA_TC_SAOPT	(0x240U)
#define EDMA_TC_SASRC	(0x244U)
#define EDMA_TC_SACNT	(0x248U)
#define EDMA_TC_SADST	(0x24cU)
#define EDMA_TC_SABIDX	(0x250U)
#define EDMA_TC_SAMPPRXY	(0x254U)
#define EDMA_TC_SACNTRLD	(0x258U)
#define EDMA_TC_SASRCBREF	(0x25cU)
#define EDMA_TC_SADSTBREF	(0x260U)
#define EDMA_TC_DFCNTRLD	(0x280U)
#define EDMA_TC_DFSRCBREF	(0x284U)
#define EDMA_TC_DFDSTBREF	(0x288U)
#define EDMA_TC_DFOPT(n)	(0x300U + (n * 64))
#define EDMA_TC_DFSRC(n)	(0x304U + (n * 64))
#define EDMA_TC_DFCNT(n)	(0x308U + (n * 64))
#define EDMA_TC_DFDST(n)	(0x30cU + (n * 64))
#define EDMA_TC_DFBIDX(n)	(0x310U + (n * 64))
#define EDMA_TC_DFMPPRXY(n)	(0x314U + (n * 64))

#endif  /* _HW_EDMA_TC_H_ */
