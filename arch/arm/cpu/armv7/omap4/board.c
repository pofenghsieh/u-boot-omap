/*
 *
 * Common functions for OMAP4 based boards
 *
 * (C) Copyright 2010
 * Texas Instruments, <www.ti.com>
 *
 * Author :
 *	Aneesh V	<aneesh@ti.com>
 *	Steve Sakoman	<steve@sakoman.com>
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
#include <common.h>
#include <asm/armv7.h>
#include <asm/arch/cpu.h>
#include <asm/arch/sys_proto.h>
#include <asm/sizes.h>
#include <asm/arch/emif.h>
#include "omap4_mux_data.h"

DECLARE_GLOBAL_DATA_PTR;

void do_set_mux(u32 base, struct pad_conf_entry const *array, int size)
{
	int i;
	struct pad_conf_entry *pad = (struct pad_conf_entry *) array;

	for (i = 0; i < size; i++, pad++)
		writew(pad->val, base + pad->offset);
}

static void set_muxconf_regs_essential(void)
{
	do_set_mux(CONTROL_PADCONF_CORE, core_padconf_array_essential,
		   sizeof(core_padconf_array_essential) /
		   sizeof(struct pad_conf_entry));

	do_set_mux(CONTROL_PADCONF_WKUP, wkup_padconf_array_essential,
		   sizeof(wkup_padconf_array_essential) /
		   sizeof(struct pad_conf_entry));
}

#ifdef CONFIG_PRELOADER
u32 omap4_boot_device = BOOT_DEVICE_MMC1;
u32 omap4_boot_mode = MMCSD_MODE_FAT;
u32 omap_boot_device(void)
{
	return omap4_boot_device;
}

u32 omap_boot_mode(void)
{
	return omap4_boot_mode;
}
#endif

static void set_mux_conf_regs(void)
{
	switch (omap4_hw_init_context()) {
	case OMAP_INIT_CONTEXT_SPL:
		set_muxconf_regs_essential();
		break;
	case OMAP_INIT_CONTEXT_UBOOT_LOADED_BY_SPL:
		set_muxconf_regs_non_essential();
		break;
	case OMAP_INIT_CONTEXT_XIP_UBOOT:
	case OMAP_INIT_CONTEXT_UBOOT_LOADED_BY_CH:
		set_muxconf_regs_essential();
		set_muxconf_regs_non_essential();
		break;
	}
}

/*
 * Routine: s_init
 * Description: Does early system init of watchdog, muxing, clocks, and
 * sdram. Watchdog disable is done always. For the rest what gets done
 * depends on the boot mode in which this function is executed
 *   1. s_init of SPL running from SRAM
 *   2. s_init of U-Boot running from FLASH
 *   3. s_init of U-Boot loaded to SDRAM by SPL
 *   4. s_init of U-Boot loaded to SDRAM by ROM code using the Configuration
 *	Header feature
 * Please have a look at the respective functions to see what gets done in
 * each of these cases
 * This function is called with SRAM stack.
 */
void s_init(void)
{
	watchdog_init();
	set_mux_conf_regs();
	prcm_init();
#ifdef CONFIG_PRELOADER
	preloader_console_init();
#endif
	sdram_init();
}

/*
 * Routine: wait_for_command_complete
 * Description: Wait for posting to finish on watchdog
 */
void wait_for_command_complete(struct watchdog *wd_base)
{
	int pending = 1;
	do {
		pending = readl(&wd_base->wwps);
	} while (pending);
}

/*
 * Routine: watchdog_init
 * Description: Shut down watch dogs
 */
void watchdog_init(void)
{
	struct watchdog *wd2_base = (struct watchdog *)WDT2_BASE;

	writel(WD_UNLOCK1, &wd2_base->wspr);
	wait_for_command_complete(wd2_base);
	writel(WD_UNLOCK2, &wd2_base->wspr);
}


/*
 * This function finds the SDRAM size available in the system
 * based on DMM section configurations
 * This is needed because the size of memory installed may be
 * different on different versions of the board
 */
u32 sdram_size(void)
{
	u32 section, i, total_size = 0, size, addr;
	for (i = 0; i < 4; i++) {
		section	= __raw_readl(OMAP44XX_DMM_LISA_MAP_BASE + i*4);
		addr = section & OMAP44XX_SYS_ADDR_MASK;
		/* See if the address is valid */
		if ((addr >= OMAP44XX_DRAM_ADDR_SPACE_START) &&
		    (addr < OMAP44XX_DRAM_ADDR_SPACE_END)) {
			size	= ((section & OMAP44XX_SYS_SIZE_MASK) >>
				   OMAP44XX_SYS_SIZE_SHIFT);
			size	= 1 << size;
			size	*= SZ_16M;
			total_size += size;
		}
	}
	return total_size;
}


/*
 * Routine: dram_init
 * Description: sets uboots idea of sdram size
 */
int dram_init(void)
{
	gd->ram_size = sdram_size();

	return 0;
}

/*
 * Print board information
 */
int checkboard(void)
{
	puts(sysinfo.board_string);
	return 0;
}

/*
* This function is called by start_armboot. You can reliably use static
* data. Any boot-time function that require static data should be
* called from here
*/
int arch_cpu_init(void)
{
	return 0;
}

static u32 cortex_a9_rev(void)
{

	unsigned int rev;

	/* Read Main ID Register (MIDR) */
	asm ("mrc p15, 0, %0, c0, c0, 0" : "=r" (rev));

	return rev;
}

u32 omap4_revision(void)
{
	if (readl(CONTROL_ID_CODE) == OMAP4_CONTROL_ID_CODE_ES2_1)
		return OMAP4430_ES2_1;
	else if (readl(CONTROL_ID_CODE) == OMAP4_CONTROL_ID_CODE_ES2_2)
		return OMAP4430_ES2_2;
	/*
	 * For some of the ES2/ES1 boards ID_CODE is not reliable:
	 * Also, ES1 and ES2 have different ARM revisions
	 * So use ARM revision for identification
	 */
	unsigned int rev = cortex_a9_rev();

	switch (rev) {
	case MIDR_CORTEX_A9_R0P1:
		return OMAP4430_ES1_0;
	case MIDR_CORTEX_A9_R1P2:
		return OMAP4430_ES2_0;
	default:
		return OMAP4430_SILICON_ID_INVALID;
	}
}
