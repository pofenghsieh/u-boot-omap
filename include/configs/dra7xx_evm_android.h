/*
 * (C) Copyright 2013
 * Texas Instruments Incorporated.
 * Lokesh Vutla	  <lokeshvutla@ti.com>
 *
 * Configuration settings for the TI DRA7XX board.
 * See omap5_common.h for omap5 common settings.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_DRA7XX_EVM_ANDROID_H
#define __CONFIG_DRA7XX_EVM_ANDROID_H

#define CONFIG_ENV_IS_NOWHERE		/* For now. */

#include <configs/omap5_common.h>

#define CONFIG_DRA7XX		/* in a TI DRA7XX core */
#define CONFIG_SYS_PROMPT		"DRA752 EVM # "

#define CONFIG_CONS_INDEX		1
#define CONFIG_SYS_NS16550_COM1		UART1_BASE
#define CONFIG_BAUDRATE			115200

/* Clock Defines */
#define V_OSCK			20000000	/* Clock output from T2 */

#define NON_SECURE_SRAM_START	0x40300000
#define NON_SECURE_SRAM_END	0x40380000	/* Not inclusive */

#define CONFIG_SYS_OMAP_ABE_SYSCK
#define CONFIG_SYS_DCACHE_OFF
#define CONFIG_SYS_ICACHE_OFF

#define EMIF1_EMIF2

/* SPI */
#define CONFIG_TI_QSPI
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_SPANSION
#define CONFIG_CMD_SF
#define CONFIG_CMD_SPI
#define CONFIG_SF_QUAD_RD
#define CONFIG_MMAP
#define CONFIG_SF_DEFAULT_SPEED                48000000
#define CONFIG_DEFAULT_SPI_MODE                SPI_MODE_3
#define CONFIG_SPI_FLASH_BAR
#define CONFIG_CH_QSPI

/* Defines for SPL */
#ifdef CONFIG_SPL
#define CONFIG_SPL_USB_BOOT_SUPPORT
#define CONFIG_SPL_BOARD_INIT
#define CONFIG_SPL_EARLY_BOOT

/* SPI SPL */
#define CONFIG_SPL_SPI_SUPPORT
#define CONFIG_SPL_SPI_LOAD
#define CONFIG_SPL_SPI_FLASH_SUPPORT
#define CONFIG_SPL_SPI_BUS		0
#define CONFIG_SPL_SPI_CS		0
#define CONFIG_SYS_SPI_U_BOOT_OFFS	0x80000
#endif

/* CPSW Ethernet */
#define CONFIG_CMD_NET
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_PING
#define CONFIG_CMD_MII
#define CONFIG_DRIVER_TI_CPSW
#define CONFIG_MII
#define CONFIG_BOOTP_DEFAULT
#define CONFIG_BOOTP_DNS
#define CONFIG_BOOTP_DNS2
#define CONFIG_BOOTP_SEND_HOSTNAME
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_SUBNETMASK
#define CONFIG_NET_RETRY_COUNT         10
#define CONFIG_NET_MULTI
#define CONFIG_PHY_GIGE
#define CONFIG_PHYLIB
#define CONFIG_PHY_ADDR			2

#define CONFIG_USB_DWC3		1
#define CONFIG_USB_DWC3_UDC_REGS		((void *)0x48890000)
#define CONFIG_USB_DWC3_UDC_REGS_END		((void *)0x4889ffff)
#define CONFIG_USB_DWC3_WRAP_REGS		((void *)0x48880000)
#define CONFIG_USB_DWC3_WRAP_REGS_END		((void *)0x4888ffff)
#ifdef CONFIG_USB_DWC3
#define CONFIG_SYS_L2CACHE_OFF	1
#endif

#define DRA7XX_PUBLIC_SAR_RAM_1_FREE           (0x4AE26000 + 0xFE0)
#define DRA7XX_PRM_RSTCTRL_RESET_WARM_BIT      (1<<0)
#define DRA7XX_PRM_RSTST_RESET_WARM_BIT        (1<<1)
#define DRA7XX_PRM_RSTST                       0x4AE07D04
#define DRA7XX_PRM_RSTCTRL                     0x4AE07D00
#define DRA7XX_PRM_RSTST_CLR                   0xfff
#define DRA7XX_REBOOT_REASON_SIZE              0xf

#define CONFIG_BOARD_MACH_TYPE			4070
#define MEMORY_BASE 				0x80000000
#define CONFIG_ADDR_ATAGS			(MEMORY_BASE + 0x100)
#define CONFIG_ADDR_DOWNLOAD			(MEMORY_BASE + 0x02000000)
#define DEVICE_TREE 				0x81f80000


/* Enable this flag if you want IPU based early camera */
/* Disabled IPU1 booting option for the demo */
/* #define CONFIG_BOOTIPU1 */

#ifdef CONFIG_BOOTIPU1
#undef CONFIG_BOOTDELAY
#define CONFIG_BOOTDELAY		3
#define CONFIG_CMD_ELF
#define IPU_LOAD_ADDR		0xa0fff000
#endif

#endif /* __CONFIG_DRA7XX_EVM_ANDROID_H   */
