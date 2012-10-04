/*
 * (C) Copyright 2011
 * Texas Instruments, <www.ti.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/* BQ2415x I2C bus address */
#define BQ2415x_CHIP_CHARGER	0x6a

/* BQ2415x control/status register addresses */
#define BQ2415x_STATUS_CTRL_REG		0
#define BQ2415x_CONTROL_REG		1
#define BQ2415x_VOLTAGE_REG		2
#define BQ2415x_CURRENT_REG		4
#define BQ2415x_SPECIAL_REG		5
#define BQ2415x_SAFETY_LIMIT_REG	6

/* bit definitions for BQ2415x_STATUS_CTRL_REG */
#define TMR_RST				(1 << 7)
#define EN_STAT				(1 << 6)

/* values put to the registers */
#define MAX_CHARGE_CURRENT		0xa0
#define NO_INPUT_CURRENT_LIMIT		0xc0
#define CHARGE_CURRENT			0x50
#define NORMAL_CHARGE_CURRENT		0x02
#define RGULATION_VOLTAGE		0x8c
