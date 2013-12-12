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

#ifndef __CONFIG_DRA7XX_EVM_H
#define __CONFIG_DRA7XX_EVM_H

#define CONFIG_ENV_IS_NOWHERE		/* For now. */

#include <configs/omap5_common.h>

#define CONFIG_DRA7XX		/* in a TI DRA7XX core */
#define CONFIG_SYS_PROMPT		"DRA752 EVM # "

#define CONFIG_SYS_NS16550_COM1		UART1_BASE	/* Base EVM has UART0 */
#define CONFIG_SYS_NS16550_COM2		UART2_BASE	/* UART1 */
#define CONFIG_SYS_NS16550_COM3		UART3_BASE	/* UART2 */
#define CONFIG_BAUDRATE			115200

/* Clock Defines */
#define V_OSCK			20000000	/* Clock output from T2 */

#define NON_SECURE_SRAM_START	0x40300000
#define NON_SECURE_SRAM_END	0x40380000	/* Not inclusive */

#define CONFIG_SYS_OMAP_ABE_SYSCK
#define CONFIG_SYS_DCACHE_OFF
#define CONFIG_SYS_ICACHE_OFF

#define EMIF1_EMIF2

#ifdef CONFIG_XIP_NOR
#undef CONFIG_SYS_NO_FLASH
#undef CONFIG_SYS_CONSOLE_IS_IN_ENV
#undef CONFIG_ENV_IS_NOWHERE
#define CONFIG_SYS_FLASH_CFI
#define CONFIG_FLASH_CFI_DRIVER
#define CONFIG_FLASH_CFI_MTD
#define CONFIG_MTD_DEVICE
#define CONFIG_SYS_MAX_FLASH_SECT	512
#define CONFIG_SYS_MAX_FLASH_BANKS	1
#define CONFIG_SYS_FLASH_USE_BUFFER_WRITE 1
#define CONFIG_CMD_FLASH
#define CONFIG_SYS_FLASH_BASE		(0x08000000)
#define CONFIG_SYS_MONITOR_BASE	CONFIG_SYS_FLASH_BASE
#define NOR_SECT_SIZE		(128 * 1024)
#define CONFIG_SYS_TEXT_BASE		0x08000000
#define CONFIG_ENV_IS_IN_FLASH
#define CONFIG_SYS_ENV_SECT_SIZE	(NOR_SECT_SIZE)
#define CONFIG_ENV_SECT_SIZE		(NOR_SECT_SIZE)
#define CONFIG_ENV_OFFSET		(3 * NOR_SECT_SIZE)
#define CONFIG_ENV_ADDR			(CONFIG_SYS_FLASH_BASE + \
						CONFIG_ENV_OFFSET)
#endif

#ifndef CONFIG_XIP_NOR
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

/* SPI SPL */
#define CONFIG_SPL_SPI_SUPPORT
#define CONFIG_SPL_SPI_LOAD
#define CONFIG_SPL_SPI_FLASH_SUPPORT
#define CONFIG_SPL_SPI_BUS		0
#define CONFIG_SPL_SPI_CS		0
#define CONFIG_SYS_SPI_U_BOOT_OFFS	0x20000
#define CONFIG_CH_QSPI
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

/* Enable this flag if you want IPU based early camera */
#undef CONFIG_BOOTIPU1

#ifdef CONFIG_BOOTIPU1
#undef CONFIG_BOOTDELAY
#define CONFIG_BOOTDELAY		3
#define CONFIG_CMD_ELF
#define IPU_LOAD_ADDR		0xa0fff000
#endif

#ifdef CONFIG_HS_AUTH
#define CONFIG_HS_ENFORCE_AUTH
#endif

#endif /* __CONFIG_DRA7XX_EVM_H */
