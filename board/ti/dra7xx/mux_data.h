/*
 * (C) Copyright 2013
 * Texas Instruments Incorporated, <www.ti.com>
 *
 * Sricharan R	<r.sricharan@ti.com>
 * Nishant Kamat <nskamat@ti.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#ifndef _MUX_DATA_DRA7XX_H_
#define _MUX_DATA_DRA7XX_H_

#include <asm/arch/mux_dra7xx.h>

const struct pad_conf_entry core_padconf_array_essential[] = {
	{MMC1_CLK, (IEN | PTU | PDIS | M0)},
	{MMC1_CMD, (IEN | PTU | PDIS | M0)},
	{MMC1_DAT0, (IEN | PTU | PDIS | M0)},
	{MMC1_DAT1, (IEN | PTU | PDIS | M0)},
	{MMC1_DAT2, (IEN | PTU | PDIS | M0)},
	{MMC1_DAT3, (IEN | PTU | PDIS | M0)},
	{MMC1_SDCD, (FSC | IEN | PTU | PDIS | M0)},
	{MMC1_SDWP, (FSC | IEN | PTD | PEN | M14)},
#ifdef CONFIG_XIP_NOR
	{GPMC_A19, (IEN | M0)}, /* GPMC_A19 */
	{GPMC_A20, (IEN | M0)}, /* GPMC_A20 */
	{GPMC_A21, (IEN | M0)}, /* GPMC_A21 */
	{GPMC_A22, (IEN | M0)}, /* GPMC_A22 */
	{GPMC_A23, (IEN | M0)}, /* GPMC_A23 */
	{GPMC_A24, (IEN | M0)}, /* GPMC_A24 */
	{GPMC_A25, (IEN | M0)}, /* GPMC_A25 */
	{GPMC_A26, (IEN | M0)}, /* GPMC_A26 */
	{GPMC_A27, (IEN | M0)}, /* GPMC_A27 */
#endif
#ifndef CONFIG_XIP_NOR
	{GPMC_A19, (IEN | PTU | M1)}, /* mmc2_dat4 */
	{GPMC_A20, (IEN | PTU | M1)}, /* mmc2_dat5 */
	{GPMC_A21, (IEN | PTU | M1)}, /* mmc2_dat6 */
	{GPMC_A22, (IEN | PTU | M1)}, /* mmc2_dat7 */
	{GPMC_A23, (IEN | PTU | M1)}, /* mmc2_clk */
	{GPMC_A24, (IEN | PTU | M1)}, /* mmc2_dat0 */
	{GPMC_A25, (IEN | PTU | M1)}, /* mmc2_dat1 */
	{GPMC_A26, (IEN | PTU | M1)}, /* mmc2_dat2 */
	{GPMC_A27, (IEN | PTU | M1)}, /* mmc2_dat3 */
	{GPMC_CS1, (IEN | PTU | M1)}, /* mmm2_cmd */
#endif
	{UART1_RXD, (FSC | IEN | PTU | PDIS | M0)},
	{UART1_TXD, (FSC | IEN | PTU | PDIS | M0)},
	{UART1_CTSN, (IEN | PTU | PDIS | M3)},
	{UART1_RTSN, (IEN | PTU | PDIS | M3)},
	{I2C1_SDA, (IEN | PTU | PDIS | M0)},
	{I2C1_SCL, (IEN | PTU | PDIS | M0)},
#ifndef CONFIG_XIP_NOR
	{GPMC_A13, (IEN | PDIS | M1)},  /* QSPI1_RTCLK */
	{GPMC_A14, (IEN | PDIS | M1)},  /* QSPI1_D[3] */
	{GPMC_A15, (IEN | PDIS | M1)},  /* QSPI1_D[2] */
	{GPMC_A16, (IEN | PDIS | M1)},  /* QSPI1_D[1] */
	{GPMC_A17, (IEN | PDIS | M1)},  /* QSPI1_D[0] */
	{GPMC_A18, (IEN | PDIS | M1)},  /* QSPI1_SCLK */
	{GPMC_A3, (IEN | PDIS | M1)},   /* QSPI1_CS2 */
	{GPMC_A4, (IEN | PDIS | M1)},   /* QSPI1_CS3 */
	{GPMC_CS0, (IEN | PTU | PDIS | M15)},
	{GPMC_CS2, (IEN | PTU | PDIS | M1)},    /* QSPI1_CS0 */
	{GPMC_CS3, (IEN | PTU | PDIS | M1)},    /* QSPI1_CS1*/
#endif
	{MDIO_MCLK, (PTU | PEN | M0)},		/* MDIO_MCLK  */
	{MDIO_D, (IEN | PTU | PEN | M0)},	/* MDIO_D  */
	{RGMII0_TXC, (M0) },
	{RGMII0_TXCTL, (M0) },
	{RGMII0_TXD3, (M0) },
	{RGMII0_TXD2, (M0) },
	{RGMII0_TXD1, (M0) },
	{RGMII0_TXD0, (M0) },
	{RGMII0_RXC, (IEN | M0) },
	{RGMII0_RXCTL, (IEN | M0) },
	{RGMII0_RXD3, (IEN | M0) },
	{RGMII0_RXD2, (IEN | M0) },
	{RGMII0_RXD1, (IEN | M0) },
	{RGMII0_RXD0, (IEN | M0) },
};

const struct pad_conf_entry core_padconf_array_non_essential[] = {
	{GPMC_AD0, (IEN | PTD | PEN | M3)},
	{GPMC_AD1, (IEN | PTD | PEN | M3)},
	{GPMC_AD2, (IEN | PTD | PEN | M3)},
	{GPMC_AD3, (IEN | PTD | PEN | M3)},
	{GPMC_AD4, (IEN | PTD | PEN | M3)},
	{GPMC_AD5, (IEN | PTD | PEN | M3)},
	{GPMC_AD6, (IEN | PTD | PEN | M3)},
	{GPMC_AD7, (IEN | PTD | PEN | M3)},
	{GPMC_AD8, (IEN | PTD | PEN | M3)},
	{GPMC_AD9, (IEN | PTD | PEN | M3)},
	{GPMC_AD10, (IEN | PTD | PEN | M3)},
	{GPMC_AD11, (IEN | PTD | PEN | M3)},
	{GPMC_AD12, (IEN | PTD | PEN | M3)},
	{GPMC_AD13, (IEN | PTD | PEN | M3)},
	{GPMC_AD14, (IEN | PTD | PEN | M3)},
	{GPMC_AD15, (IEN | PTD | PEN | M3)},
	{GPMC_A0, (IEN | PDIS | M3)},
	{GPMC_A1, (IEN | PDIS | M3)},
	{GPMC_A2, (IEN | PDIS | M3)},
	{GPMC_A3, (IEN | PDIS | M3)},
	{GPMC_A4, (IEN | PDIS | M3)},
	{GPMC_A5, (IEN | PDIS | M3)},
	{GPMC_A6, (IEN | PDIS | M3)},
	{GPMC_A7, (IEN | PDIS | M3)},
	{GPMC_A8, (IEN | PDIS | M3)},
	{GPMC_A9, (IEN | PDIS | M3)},
	{GPMC_A10, (IEN | PDIS | M3)},
	{GPMC_A11, (IEN | PDIS | M3)},
	{GPMC_A12, (IEN | PDIS | M15)},
	{GPMC_A13, (IEN | PDIS | M1)},
	{GPMC_A14, (IEN | PDIS | M1)},
	{GPMC_A15, (IEN | PDIS | M1)},
	{GPMC_A16, (IEN | PDIS | M1)},
	{GPMC_A17, (IEN | PDIS | M1)},
	{GPMC_A18, (IEN | PDIS | M1)},
	{GPMC_CS0, (IEN | PTU | PDIS | M15)},
	{GPMC_CS2, (IEN | PTU | PDIS | M1)},
	{GPMC_CS3, (IEN | PTU | PDIS | M3)},
	{GPMC_CLK, (IEN | PTU | PDIS | M15)},
	{GPMC_ADVN_ALE, (IEN | PTU | PDIS | M15)},
	{GPMC_OEN_REN, (IEN | PTU | PDIS | M15)},
	{GPMC_WEN, (IEN | PTU | PDIS | M15)},
	{GPMC_BEN0, (IEN | PTU | PDIS | M15)},
	{GPMC_BEN1, (IEN | PTU | PDIS | M15)},
	{GPMC_WAIT0, (FSC | IEN | PTU | PDIS | M15)},
	{VIN1A_CLK0, (IEN | PDIS | M0)},
	{VIN1B_CLK1, (FSC | IEN | PDIS | M0)},
	{VIN1A_DE0, (IEN | PDIS | M0)},
	{VIN1A_FLD0, (IEN | PDIS | M15)},
	{VIN1A_HSYNC0, (IEN | PDIS | M0)},
	{VIN1A_VSYNC0, (IEN | PDIS | M0)},
	{VIN1A_D0, (IEN | PDIS | M0)},
	{VIN1A_D1, (IEN | PDIS | M0)},
	{VIN1A_D2, (IEN | PDIS | M0)},
	{VIN1A_D3, (IEN | PDIS | M0)},
	{VIN1A_D4, (IEN | PDIS | M0)},
	{VIN1A_D5, (IEN | PDIS | M0)},
	{VIN1A_D6, (IEN | PDIS | M0)},
	{VIN1A_D7, (IEN | PDIS | M0)},
	{VIN1A_D8, (IEN | PDIS | M0)},
	{VIN1A_D9, (IEN | PDIS | M0)},
	{VIN1A_D10, (IEN | PDIS | M0)},
	{VIN1A_D11, (IEN | PDIS | M0)},
	{VIN1A_D12, (IEN | PDIS | M0)},
	{VIN1A_D13, (IEN | PDIS | M0)},
	{VIN1A_D14, (IEN | PDIS | M0)},
	{VIN1A_D15, (IEN | PDIS | M0)},
	{VIN1A_D16, (IEN | PDIS | M0)},
	{VIN1A_D17, (IEN | PDIS | M0)},
	{VIN1A_D18, (IEN | PDIS | M0)},
	{VIN1A_D19, (IEN | PDIS | M0)},
	{VIN1A_D20, (IEN | PDIS | M0)},
	{VIN1A_D21, (IEN | PDIS | M0)},
	{VIN1A_D22, (IEN | PDIS | M0)},
	{VIN1A_D23, (IEN | PDIS | M0)},
	{VIN2A_CLK0, (IEN | PDIS | M15)},
	{VIN2A_DE0, (IEN | PDIS | M15)},
	{VIN2A_FLD0, (IEN | PDIS | M15)},
	{VIN2A_HSYNC0, (IEN | PDIS | M15)},
	{VIN2A_VSYNC0, (IEN | PDIS | M15)},
	{VIN2A_D0, (IEN | PDIS | M15)},
	{VIN2A_D1, (IEN | PDIS | M15)},
	{VIN2A_D2, (IEN | PDIS | M15)},
	{VIN2A_D3, (IEN | PDIS | M15)},
	{VIN2A_D4, (IEN | PDIS | M15)},
	{VIN2A_D5, (IEN | PDIS | M15)},
	{VIN2A_D6, (IEN | PDIS | M15)},
	{VIN2A_D7, (IEN | PDIS | M15)},
	{VIN2A_D8, (IEN | PDIS | M15)},
	{VIN2A_D9, (IEN | PDIS | M15)},
	{VIN2A_D10, (IEN | PDIS | M15)},
	{VIN2A_D11, (IEN | PDIS | M15)},
	{VIN2A_D12, (IEN | PDIS | M3)},
	{VIN2A_D13, (IEN | PDIS | M3)},
	{VIN2A_D14, (IEN | PDIS | M3)},
	{VIN2A_D15, (IEN | PDIS | M3)},
	{VIN2A_D16, (IEN | PDIS | M3)},
	{VIN2A_D17, (IEN | PDIS | M3)},
	{VIN2A_D18, (IEN | PDIS | M3)},
	{VIN2A_D19, (IEN | PDIS | M3)},
	{VIN2A_D20, (IEN | PDIS | M3)},
	{VIN2A_D21, (IEN | PDIS | M3)},
	{VIN2A_D22, (IEN | PDIS | M3)},
	{VIN2A_D23, (IEN | PDIS | M3)},
	{VOUT1_CLK, (IEN | PDIS | M0)},
	{VOUT1_DE, (IEN | PDIS | M0)},
	{VOUT1_FLD, (IEN | PDIS | M15)},
	{VOUT1_HSYNC, (IEN | PDIS | M0)},
	{VOUT1_VSYNC, (IEN | PDIS | M0)},
	{VOUT1_D0, (IEN | PDIS | M0)},
	{VOUT1_D1, (IEN | PDIS | M0)},
	{VOUT1_D2, (IEN | PDIS | M0)},
	{VOUT1_D3, (IEN | PDIS | M0)},
	{VOUT1_D4, (IEN | PDIS | M0)},
	{VOUT1_D5, (IEN | PDIS | M0)},
	{VOUT1_D6, (IEN | PDIS | M0)},
	{VOUT1_D7, (IEN | PDIS | M0)},
	{VOUT1_D8, (IEN | PDIS | M0)},
	{VOUT1_D9, (IEN | PDIS | M0)},
	{VOUT1_D10, (IEN | PDIS | M0)},
	{VOUT1_D11, (IEN | PDIS | M0)},
	{VOUT1_D12, (IEN | PDIS | M0)},
	{VOUT1_D13, (IEN | PDIS | M0)},
	{VOUT1_D14, (IEN | PDIS | M0)},
	{VOUT1_D15, (IEN | PDIS | M0)},
	{VOUT1_D16, (IEN | PDIS | M0)},
	{VOUT1_D17, (IEN | PDIS | M0)},
	{VOUT1_D18, (IEN | PDIS | M0)},
	{VOUT1_D19, (IEN | PDIS | M0)},
	{VOUT1_D20, (IEN | PDIS | M0)},
	{VOUT1_D21, (IEN | PDIS | M0)},
	{VOUT1_D22, (IEN | PDIS | M0)},
	{VOUT1_D23, (IEN | PDIS | M0)},
	{RMII_MHZ_50_CLK, (IEN | PDIS | M15)},
	{UART3_RXD, (FSC | IEN | PDIS | M15)},
	{UART3_TXD, (FSC | IEN | PDIS | M15)},
	{USB1_DRVVBUS, (FSC | IEN | PDIS | M0)},
	{USB2_DRVVBUS, (FSC | IEN | PDIS | M0)},
	{GPIO6_14, (IEN | PTU | PEN | M9)},
	{GPIO6_15, (IEN | PTU | PEN | M9)},
	{GPIO6_16, (IEN | PTU | PDIS | M14)},
	{XREF_CLK0, (IEN | PDIS | M4)},
	{XREF_CLK1, (IEN | PDIS | M4)},
	{XREF_CLK2, (IEN | PDIS | M3)},
	{XREF_CLK3, (IEN | PTD | PEN | M14)},
	{MCASP1_ACLKX, (IEN | PDIS | M0)},
	{MCASP1_FSX, (FSC | IEN | PDIS | M0)},
	{MCASP1_ACLKR, (IEN | PTD | PEN | M14)},
	{MCASP1_FSR, (IEN | PDIS | M15)},
	{MCASP1_AXR0, (FSC | IEN | PDIS | M0)},
	{MCASP1_AXR1, (FSC | IEN | PDIS | M0)},
	{MCASP1_AXR2, (IEN | PTD | PEN | M14)},
	{MCASP1_AXR3, (IEN | PTD | PEN | M14)},
	{MCASP1_AXR4, (IEN | PTD | PEN | M14)},
	{MCASP1_AXR5, (IEN | PTD | PEN | M14)},
	{MCASP1_AXR6, (IEN | PTD | PEN | M14)},
	{MCASP1_AXR7, (IEN | PTD | PEN | M14)},
	{MCASP1_AXR8, (FSC | IEN | PDIS | M1)},
	{MCASP1_AXR9, (FSC | IEN | PDIS | M1)},
	{MCASP1_AXR10, (FSC | IEN | PDIS | M1)},
	{MCASP1_AXR11, (FSC | IEN | PDIS | M1)},
	{MCASP1_AXR12, (FSC | IEN | PDIS | M1)},
	{MCASP1_AXR13, (FSC | IEN | PDIS | M1)},
	{MCASP1_AXR14, (FSC | IEN | PDIS | M1)},
	{MCASP1_AXR15, (FSC | IEN | PDIS | M1)},
	{MCASP2_ACLKX, (IEN | PDIS | M0)},
	{MCASP2_FSX, (FSC | IEN | PDIS | M0)},
	{MCASP2_ACLKR, (IEN | PDIS | M15)},
	{MCASP2_FSR, (IEN | PDIS | M15)},
	{MCASP2_AXR0, (IEN | PDIS | M0)},
	{MCASP2_AXR1, (IEN | PDIS | M0)},
	{MCASP2_AXR2, (FSC | IEN | PDIS | M0)},
	{MCASP2_AXR3, (FSC | IEN | PDIS | M0)},
	{MCASP2_AXR4, (IEN | PDIS | M0)},
	{MCASP2_AXR5, (IEN | PDIS | M0)},
	{MCASP2_AXR6, (IEN | PDIS | M0)},
	{MCASP2_AXR7, (IEN | PDIS | M0)},
	{MCASP3_ACLKX, (IEN | PDIS | M0)},
	{MCASP3_FSX, (FSC | IEN | PDIS | M0)},
	{MCASP3_AXR0, (FSC | IEN | PDIS | M0)},
	{MCASP3_AXR1, (FSC | IEN | PDIS | M0)},
	{MCASP4_ACLKX, (IEN | PTU | PEN | M4)},
	{MCASP4_FSX, (IEN | PTU | PEN | M4)},
	{MCASP4_AXR0, (IEN | PDIS | M15)},
	{MCASP4_AXR1, (IEN | PDIS | M15)},
	{MCASP5_ACLKX, (IEN | PDIS | M0)},
	{MCASP5_FSX, (IEN | PDIS | M0)},
	{MCASP5_AXR0, (IEN | PDIS | M0)},
	{MCASP5_AXR1, (IEN | PDIS | M15)},
	{GPIO6_10, (IEN | PTU | PDIS | M15)},
	{GPIO6_11, (IEN | PTU | PDIS | M0)},
	{MMC3_CLK, (IEN | PTU | PDIS | M0)},
	{MMC3_CMD, (IEN | PTU | PDIS | M0)},
	{MMC3_DAT0, (IEN | PTU | PDIS | M0)},
	{MMC3_DAT1, (IEN | PTU | PDIS | M0)},
	{MMC3_DAT2, (IEN | PTU | PDIS | M0)},
	{MMC3_DAT3, (IEN | PTU | PDIS | M0)},
	{MMC3_DAT4, (IEN | PTU | PDIS | M15)},
	{MMC3_DAT5, (IEN | PTU | PDIS | M15)},
	{MMC3_DAT6, (IEN | PTU | PDIS | M15)},
	{MMC3_DAT7, (IEN | PTU | PDIS | M15)},
	{SPI1_SCLK, (IEN | PDIS | M0)},
	{SPI1_D1, (IEN | PDIS | M0)},
	{SPI1_D0, (IEN | PDIS | M0)},
	{SPI1_CS0, (IEN | PTU | PDIS | M0)},
	{SPI1_CS1, (IEN | PTU | PDIS | M0)},
	{SPI1_CS2, (FSC | IEN | PTU | PDIS | M6)},
	{SPI1_CS3, (FSC | IEN | PTU | PEN | M6)},
	{SPI2_SCLK, (IEN | PDIS | M1)},
	{SPI2_D1, (FSC | IEN | PDIS | M1)},
	{SPI2_D0, (FSC | IEN | PDIS | M1)},
	{SPI2_CS0, (FSC | IEN | PTU | PDIS | M1)},
	{DCAN1_TX, (FSC | IEN | PTU | PDIS | M0)},
	{DCAN1_RX, (FSC | IEN | PTU | PDIS | M0)},
	{UART2_RXD, (IEN | PTU | PDIS | M3)},
	{UART2_TXD, (IEN | PTU | PDIS | M3)},
	{UART2_CTSN, (IEN | PTU | PDIS | M3)},
	{UART2_RTSN, (IEN | PTU | PDIS | M3)},
	{I2C2_SDA, (IEN | PTU | PDIS | M0)},
	{I2C2_SCL, (IEN | PTU | PDIS | M0)},
	{WAKEUP0, (PEN | M0)},
	{WAKEUP1, (PEN | M0)},
	{WAKEUP2, (PTU | PEN | M14)},
	{WAKEUP3, (PEN | M15)},
	{ON_OFF, (PTU | PDIS | M0)},
	{RTC_PORZ, (PEN | M0)},
	{TMS, (IEN | PTU | PDIS | M0)},
	{TDI, (FSC | IEN | PTU | PDIS | M0)},
	{TDO, (IEN | PTU | PDIS | M0)},
	{TCLK, (IEN | PTU | PDIS | M0)},
	{TRSTN, (IEN | PDIS | M0)},
	{RTCK, (IEN | PTD | PEN | M0)},
	{EMU0, (IEN | PTU | PDIS | M0)},
	{EMU1, (IEN | PTU | PDIS | M0)},
	{RESETN, (PTU | PDIS | M0)},
	{NMIN, (PDIS | M0)},
	{RSTOUTN, (PDIS | M0)},
};
#endif /* _MUX_DATA_DRA7XX_H_ */
