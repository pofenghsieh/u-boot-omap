/*
 *
 * Clock initialization for OMAP4
 *
 * (C) Copyright 2010
 * Texas Instruments, <www.ti.com>
 *
 * Aneesh V <aneesh@ti.com>
 *
 * Based on previous work by:
 *	Santosh Shilimkar <santosh.shilimkar@ti.com>
 *	Rajendra Nayak <rnayak@ti.com>
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
#include <asm/omap_common.h>
#include <asm/arch/clocks.h>
#include <asm/arch/sys_proto.h>
#include <asm/utils.h>
#include <asm/omap_gpio.h>

#define abs(x) (((x) < 0) ? ((x)*-1) : (x))

static const u32 sys_clk_array[8] = {
	12000000,	       /* 12 MHz */
	13000000,	       /* 13 MHz */
	16800000,	       /* 16.8 MHz */
	19200000,	       /* 19.2 MHz */
	26000000,	       /* 26 MHz */
	27000000,	       /* 27 MHz */
	38400000,	       /* 38.4 MHz */
};

static const struct dpll_params mpu_dpll_params_1ghz[NUM_SYS_CLKS] = {
	{250, 2, 1, -1, -1, -1, -1, -1},	/* 12 MHz   */
	{1000, 12, 1, -1, -1, -1, -1, -1},	/* 13 MHz   */
	{119, 1, 1, -1, -1, -1, -1, -1},	/* 16.8 MHz */
	{625, 11, 1, -1, -1, -1, -1, -1},	/* 19.2 MHz */
	{500, 12, 1, -1, -1, -1, -1, -1},	/* 26 MHz   */
	{1000, 26, 1, -1, -1, -1, -1, -1},	/* 27 MHz   */
	{625, 23, 1, -1, -1, -1, -1, -1}	/* 38.4 MHz */
};

static const struct dpll_params mpu_dpll_params_es1_600mhz[NUM_SYS_CLKS] = {
	{50, 0, 1, -1, -1, -1, -1, -1},		/* 12 MHz   */
	{600, 12, 1, -1, -1, -1, -1, -1},	/* 13 MHz   */
	{250, 6, 1, -1, -1, -1, -1, -1},	/* 16.8 MHz */
	{125, 3, 1, -1, -1, -1, -1, -1},	/* 19.2 MHz */
	{300, 12, 1, -1, -1, -1, -1, -1},	/* 26 MHz   */
	{200, 8, 1, -1, -1, -1, -1, -1},	/* 27 MHz   */
	{125, 7, 1, -1, -1, -1, -1, -1}		/* 38.4 MHz */
};

static const struct dpll_params core_dpll_params_1600mhz[NUM_SYS_CLKS] = {
	{200, 2, 1, 5, 8, 4, 6, 5},	/* 12 MHz   */
	{800, 12, 1, 5, 8, 4, 6, 5},	/* 13 MHz   */
	{619, 12, 1, 5, 8, 4, 6, 5},	/* 16.8 MHz */
	{125, 2, 1, 5, 8, 4, 6, 5},	/* 19.2 MHz */
	{400, 12, 1, 5, 8, 4, 6, 5},	/* 26 MHz   */
	{800, 26, 1, 5, 8, 4, 6, 5},	/* 27 MHz   */
	{125, 5, 1, 5, 8, 4, 6, 5}	/* 38.4 MHz */
};

static const struct dpll_params core_dpll_params_es1_1524mhz[NUM_SYS_CLKS] = {
	{127, 1, 1, 5, 8, 4, 6, 5},	/* 12 MHz   */
	{762, 12, 1, 5, 8, 4, 6, 5},	/* 13 MHz   */
	{635, 13, 1, 5, 8, 4, 6, 5},	/* 16.8 MHz */
	{635, 15, 1, 5, 8, 4, 6, 5},	/* 19.2 MHz */
	{381, 12, 1, 5, 8, 4, 6, 5},	/* 26 MHz   */
	{254, 8, 1, 5, 8, 4, 6, 5},	/* 27 MHz   */
	{496, 24, 1, 5, 8, 4, 6, 5}	/* 38.4 MHz */
};

static const struct dpll_params
		core_dpll_params_es2_1600mhz_ddr200mhz[NUM_SYS_CLKS] = {
	{200, 2, 2, 5, 8, 4, 6, 5},	/* 12 MHz   */
	{800, 12, 2, 5, 8, 4, 6, 5},	/* 13 MHz   */
	{619, 12, 2, 5, 8, 4, 6, 5},	/* 16.8 MHz */
	{125, 2, 2, 5, 8, 4, 6, 5},	/* 19.2 MHz */
	{400, 12, 2, 5, 8, 4, 6, 5},	/* 26 MHz   */
	{800, 26, 2, 5, 8, 4, 6, 5},	/* 27 MHz   */
	{125, 5, 2, 5, 8, 4, 6, 5}	/* 38.4 MHz */
};

static const struct dpll_params per_dpll_params_1536mhz[NUM_SYS_CLKS] = {
	{64, 0, 8, 6, 12, 9, 4, 5},	/* 12 MHz   */
	{768, 12, 8, 6, 12, 9, 4, 5},	/* 13 MHz   */
	{320, 6, 8, 6, 12, 9, 4, 5},	/* 16.8 MHz */
	{40, 0, 8, 6, 12, 9, 4, 5},	/* 19.2 MHz */
	{384, 12, 8, 6, 12, 9, 4, 5},	/* 26 MHz   */
	{256, 8, 8, 6, 12, 9, 4, 5},	/* 27 MHz   */
	{20, 0, 8, 6, 12, 9, 4, 5}	/* 38.4 MHz */
};

static const struct dpll_params iva_dpll_params_1862mhz[NUM_SYS_CLKS] = {
	{931, 11, -1, -1, 4, 7, -1, -1},	/* 12 MHz   */
	{931, 12, -1, -1, 4, 7, -1, -1},	/* 13 MHz   */
	{665, 11, -1, -1, 4, 7, -1, -1},	/* 16.8 MHz */
	{727, 14, -1, -1, 4, 7, -1, -1},	/* 19.2 MHz */
	{931, 25, -1, -1, 4, 7, -1, -1},	/* 26 MHz   */
	{931, 26, -1, -1, 4, 7, -1, -1},	/* 27 MHz   */
	{412, 16, -1, -1, 4, 7, -1, -1}		/* 38.4 MHz */
};

/* ABE M & N values with sys_clk as source */
static const struct dpll_params
		abe_dpll_params_sysclk_196608khz[NUM_SYS_CLKS] = {
	{49, 5, 1, 1, -1, -1, -1, -1},	/* 12 MHz   */
	{68, 8, 1, 1, -1, -1, -1, -1},	/* 13 MHz   */
	{35, 5, 1, 1, -1, -1, -1, -1},	/* 16.8 MHz */
	{46, 8, 1, 1, -1, -1, -1, -1},	/* 19.2 MHz */
	{34, 8, 1, 1, -1, -1, -1, -1},	/* 26 MHz   */
	{29, 7, 1, 1, -1, -1, -1, -1},	/* 27 MHz   */
	{64, 24, 1, 1, -1, -1, -1, -1}	/* 38.4 MHz */
};

/* ABE M & N values with 32K clock as source */
static const struct dpll_params abe_dpll_params_32k_196608khz = {
	750, 0, 1, 1, -1, -1, -1, -1
};


static const struct dpll_params usb_dpll_params_1920mhz[NUM_SYS_CLKS] = {
	{80, 0, 2, -1, -1, -1, -1, -1},		/* 12 MHz   */
	{960, 12, 2, -1, -1, -1, -1, -1},	/* 13 MHz   */
	{400, 6, 2, -1, -1, -1, -1, -1},	/* 16.8 MHz */
	{50, 0, 2, -1, -1, -1, -1, -1},		/* 19.2 MHz */
	{480, 12, 2, -1, -1, -1, -1, -1},	/* 26 MHz   */
	{320, 8, 2, -1, -1, -1, -1, -1},	/* 27 MHz   */
	{25, 0, 2, -1, -1, -1, -1, -1}		/* 38.4 MHz */
};

static const u32 clk_domains_essential[] = {
	CM_L4PER_CLKSTCTRL,
	CM_L3INIT_CLKSTCTRL,
	CM_MEMIF_CLKSTCTRL,
	CM_L4CFG_CLKSTCTRL,
	0
};

static const u32 clk_domains_non_essential[] = {
	CM_MPU_M3_CLKSTCTRL,
	CM_IVAHD_CLKSTCTRL,
	CM_DSP_CLKSTCTRL,
	CM_DSS_CLKSTCTRL,
	CM_SGX_CLKSTCTRL,
	CM1_ABE_CLKSTCTRL,
	CM_C2C_CLKSTCTRL,
	CM_CAM_CLKSTCTRL,
	CM_DSS_CLKSTCTRL,
	CM_SDMA_CLKSTCTRL,
	0
};

static const u32 clk_modules_hw_auto_essential[] = {
	CM_WKUP_GPIO1_CLKCTRL,
	CM_L4PER_GPIO2_CLKCTRL,
	CM_L4PER_GPIO3_CLKCTRL,
	CM_L4PER_GPIO4_CLKCTRL,
	CM_L4PER_GPIO5_CLKCTRL,
	CM_L4PER_GPIO6_CLKCTRL,
	CM_MEMIF_EMIF_1_CLKCTRL,
	CM_MEMIF_EMIF_2_CLKCTRL,
	CM_L3INIT_HSUSBOTG_CLKCTRL,
	CM_L3INIT_USBPHY_CLKCTRL,
	CM_L4CFG_L4_CFG_CLKCTRL,
	0
};

static const u32 clk_modules_hw_auto_non_essential[] = {
	CM_MPU_M3_MPU_M3_CLKCTRL,
	CM_IVAHD_IVAHD_CLKCTRL,
	CM_IVAHD_SL2_CLKCTRL,
	CM_DSP_DSP_CLKCTRL,
	CM_L3_2_GPMC_CLKCTRL,
	CM_L3INSTR_L3_3_CLKCTRL,
	CM_L3INSTR_L3_INSTR_CLKCTRL,
	CM_L3INSTR_INTRCONN_WP1_CLKCTRL,
	CM_L3INIT_HSI_CLKCTRL,
	CM_L3INIT_HSUSBTLL_CLKCTRL,
	0
};

static const u32 clk_modules_explicit_en_essential[] = {
	CM_L4PER_GPTIMER2_CLKCTRL,
	CM_L3INIT_HSMMC1_CLKCTRL,
	CM_L3INIT_HSMMC2_CLKCTRL,
	CM_L4PER_MCSPI1_CLKCTRL,
	CM_WKUP_GPTIMER1_CLKCTRL,
	CM_L4PER_I2C1_CLKCTRL,
	CM_L4PER_I2C2_CLKCTRL,
	CM_L4PER_I2C3_CLKCTRL,
	CM_L4PER_I2C4_CLKCTRL,
	CM_WKUP_WDTIMER2_CLKCTRL,
	CM_L4PER_UART3_CLKCTRL,
	0
};

static const u32 clk_modules_explicit_en_non_essential[] = {
	CM1_ABE_AESS_CLKCTRL,
	CM1_ABE_PDM_CLKCTRL,
	CM1_ABE_DMIC_CLKCTRL,
	CM1_ABE_MCASP_CLKCTRL,
	CM1_ABE_MCBSP1_CLKCTRL,
	CM1_ABE_MCBSP2_CLKCTRL,
	CM1_ABE_MCBSP3_CLKCTRL,
	CM1_ABE_SLIMBUS_CLKCTRL,
	CM1_ABE_TIMER5_CLKCTRL,
	CM1_ABE_TIMER6_CLKCTRL,
	CM1_ABE_TIMER7_CLKCTRL,
	CM1_ABE_TIMER8_CLKCTRL,
	CM1_ABE_WDT3_CLKCTRL,
	CM_L4PER_GPTIMER9_CLKCTRL,
	CM_L4PER_GPTIMER10_CLKCTRL,
	CM_L4PER_GPTIMER11_CLKCTRL,
	CM_L4PER_GPTIMER3_CLKCTRL,
	CM_L4PER_GPTIMER4_CLKCTRL,
	CM_L4PER_HDQ1W_CLKCTRL,
	CM_L4PER_MCBSP4_CLKCTRL,
	CM_L4PER_MCSPI2_CLKCTRL,
	CM_L4PER_MCSPI3_CLKCTRL,
	CM_L4PER_MCSPI4_CLKCTRL,
	CM_L4PER_MMCSD3_CLKCTRL,
	CM_L4PER_MMCSD4_CLKCTRL,
	CM_L4PER_MMCSD5_CLKCTRL,
	CM_L4PER_UART1_CLKCTRL,
	CM_L4PER_UART2_CLKCTRL,
	CM_L4PER_UART4_CLKCTRL,
	CM_WKUP_KEYBOARD_CLKCTRL,
	CM_WKUP_WDTIMER2_CLKCTRL,
	CM_CAM_ISS_CLKCTRL,
	CM_CAM_FDIF_CLKCTRL,
	CM_DSS_DSS_CLKCTRL,
	CM_SGX_SGX_CLKCTRL,
	CM_L3INIT_HSUSBHOST_CLKCTRL,
	CM_L3INIT_FSUSB_CLKCTRL,
	0
};

static inline u32 __get_sys_clk_index(void)
{
	u32 ind;
	/*
	 * For ES1 the ROM code calibration of sys clock is not reliable
	 * due to hw issue. So, use hard-coded value. If this value is not
	 * correct for any board over-ride this function in board file
	 * From ES2.0 onwards you will get this information from
	 * CM_SYS_CLKSEL
	 */
	if (omap4_revision() == OMAP4430_ES1_0)
		ind = OMAP_SYS_CLK_IND_38_4_MHZ;
	else {
		/* SYS_CLKSEL - 1 to match the dpll param array indices */
		ind = (readl(CM_SYS_CLKSEL) & CM_SYS_CLKSEL_SYS_CLKSEL_MASK)
			- 1;
	}
	return ind;
}

u32 get_sys_clk_index(void)
	__attribute__ ((weak, alias("__get_sys_clk_index")));

u32 get_sys_clk_freq(void)
{
	u8 index = get_sys_clk_index();
	return sys_clk_array[index];
}

static inline void do_bypass_dpll(u32 base)
{
	struct dpll_regs *dpll_regs = (struct dpll_regs *)base;

	modify_reg_32(&dpll_regs->cm_clkmode_dpll,
		      CM_CLKMODE_DPLL_DPLL_EN_SHIFT,
		      CM_CLKMODE_DPLL_DPLL_EN_MASK, DPLL_EN_FAST_RELOCK_BYPASS);
}

static inline void wait_for_bypass(u32 base)
{
	struct dpll_regs *dpll_regs = (struct dpll_regs *)base;

	while (readl(&dpll_regs->cm_idlest_dpll) & ST_DPLL_CLK_MASK)
		;
}

static inline void do_lock_dpll(u32 base)
{
	struct dpll_regs *dpll_regs = (struct dpll_regs *)base;

	modify_reg_32(&dpll_regs->cm_clkmode_dpll,
		      CM_CLKMODE_DPLL_DPLL_EN_SHIFT,
		      CM_CLKMODE_DPLL_DPLL_EN_MASK, DPLL_EN_LOCK);
}

static inline void wait_for_lock(u32 base)
{
	struct dpll_regs *dpll_regs = (struct dpll_regs *)base;

	while (!(readl(&dpll_regs->cm_idlest_dpll) & ST_DPLL_CLK_MASK))
		;
}

static void do_setup_dpll(u32 base, const struct dpll_params *params, u8 lock)
{
	u32 temp;
	struct dpll_regs *dpll_regs = (struct dpll_regs *)base;

	bypass_dpll(base);

	/* Set M & N */
	temp = readl(&dpll_regs->cm_clksel_dpll);
	set_bit_field(temp, CM_CLKSEL_DPLL_M_SHIFT, CM_CLKSEL_DPLL_M_MASK,
			params->m);
	set_bit_field(temp, CM_CLKSEL_DPLL_N_SHIFT, CM_CLKSEL_DPLL_N_MASK,
			params->n);
	writel(temp, &dpll_regs->cm_clksel_dpll);

	/* Lock */
	if (lock)
		do_lock_dpll(base);

	/* Setup post-dividers */
	if (params->m2 >= 0)
		writel(params->m2, &dpll_regs->cm_div_m2_dpll);
	if (params->m3 >= 0)
		writel(params->m3, &dpll_regs->cm_div_m3_dpll);
	if (params->m4 >= 0)
		writel(params->m4, &dpll_regs->cm_div_m4_dpll);
	if (params->m5 >= 0)
		writel(params->m5, &dpll_regs->cm_div_m5_dpll);
	if (params->m6 >= 0)
		writel(params->m6, &dpll_regs->cm_div_m6_dpll);
	if (params->m7 >= 0)
		writel(params->m7, &dpll_regs->cm_div_m7_dpll);

	/* Wait till the DPLL locks */
	if (lock)
		wait_for_lock(base);
}

const struct dpll_params *get_core_dpll_params(void)
{
	u32 sysclk_ind = get_sys_clk_index();

	switch (omap4_revision()) {
	case OMAP4430_ES1_0:
		return &core_dpll_params_es1_1524mhz[sysclk_ind];
	case OMAP4430_ES2_0:
	case OMAP4430_SILICON_ID_INVALID:
		 /* safest */
		return &core_dpll_params_es2_1600mhz_ddr200mhz[sysclk_ind];
	default:
		return &core_dpll_params_1600mhz[sysclk_ind];
	}
}

u32 omap4_ddr_clk(void)
{
	u32 ddr_clk, sys_clk_khz;
	const struct dpll_params *core_dpll_params;

	sys_clk_khz = get_sys_clk_freq() / 1000;

	core_dpll_params = get_core_dpll_params();

	spl_debug("sys_clk %d\n ", sys_clk_khz * 1000);

	/* Find Core DPLL locked frequency first */
	ddr_clk = sys_clk_khz * 2 * core_dpll_params->m /
			(core_dpll_params->n + 1); 
	/*
	 * DDR frequency is PHY_ROOT_CLK/2
	 * PHY_ROOT_CLK = Fdpll/2/M2
	 */
	ddr_clk = ddr_clk / 4 / core_dpll_params->m2;

	ddr_clk *= 1000;	/* convert to Hz */
	spl_debug("ddr_clk %d\n ", ddr_clk);

	return ddr_clk;
}

static void setup_dplls(void)
{
	u32 sysclk_ind, temp;
	const struct dpll_params *params;
	spl_debug("setup_dplls\n");

	sysclk_ind = get_sys_clk_index();

	/* CORE dpll */
	params = get_core_dpll_params();	/* default - safest */
	/*
	 * Do not lock the core DPLL now. Just set it up.
	 * Core DPLL will be locked after setting up EMIF
	 * using the FREQ_UPDATE method(freq_update_core())
	 */
	do_setup_dpll(CM_CLKMODE_DPLL_CORE, params, DPLL_NO_LOCK);
	/* Set the ratios for CORE_CLK, L3_CLK, L4_CLK */
	temp = (CLKSEL_CORE_X2_DIV_1 << CLKSEL_CORE_SHIFT) |
	    (CLKSEL_L3_CORE_DIV_2 << CLKSEL_L3_SHIFT) |
	    (CLKSEL_L4_L3_DIV_2 << CLKSEL_L4_SHIFT);
	writel(temp, CM_CLKSEL_CORE);
	spl_debug("Core DPLL configured\n");

	/* lock PER dpll */
	do_setup_dpll(CM_CLKMODE_DPLL_PER,
			&per_dpll_params_1536mhz[sysclk_ind], DPLL_LOCK);
	spl_debug("PER DPLL locked\n");

	/* MPU dpll */
	if (omap4_revision() == OMAP4430_ES1_0)
		params = &mpu_dpll_params_es1_600mhz[sysclk_ind];
	else
		params = &mpu_dpll_params_1ghz[sysclk_ind];
	do_setup_dpll(CM_CLKMODE_DPLL_MPU, params, DPLL_LOCK);
	spl_debug("MPU DPLL locked\n");
}

static void setup_non_essential_dplls(void)
{
	u32 sys_clk_khz, temp, abe_ref_clk;
	u32 sysclk_ind, sd_div, num, den;
	const struct dpll_params *params;

	sysclk_ind = get_sys_clk_index();
	sys_clk_khz = get_sys_clk_freq() / 1000;

	/* IVA */
	do_setup_dpll(CM_CLKMODE_DPLL_IVA, &iva_dpll_params_1862mhz[sysclk_ind],
			DPLL_LOCK);

	/*
	 * USB:
	 * USB dpll is J-type. Need to set DPLL_SD_DIV for jitter correction
	 * DPLL_SD_DIV = CEILING ([DPLL_MULT/(DPLL_DIV+1)]* CLKINP / 250)
	 *      - where CLKINP is sys_clk in MHz
	 * Use CLKINP in KHz and adjust the denominator accordingly so
	 * that we have enough accuracy and at the same time no overflow
	 */
	params = &usb_dpll_params_1920mhz[sysclk_ind];
	num = params->m * sys_clk_khz;
	den = (params->n + 1) * 250 * 1000;
	num += den - 1;
	sd_div = num / den;
	temp = readl(CM_CLKSEL_DPLL_USB);
	set_bit_field(temp, CM_CLKSEL_DPLL_DPLL_SD_DIV_SHIFT,
		      CM_CLKSEL_DPLL_DPLL_SD_DIV_MASK, sd_div);
	writel(temp, CM_CLKSEL_DPLL_USB);

	/* Now setup the dpll with the regular function */
	do_setup_dpll(CM_CLKMODE_DPLL_USB, params, DPLL_LOCK);

#ifdef CONFIG_SYS_OMAP4_ABE_SYSCK
	params = &abe_dpll_params_sysclk_196608khz[sysclk_ind];
	abe_ref_clk = CM_ABE_PLL_REF_CLKSEL_CLKSEL_SYSCLK;
#else
	params = &abe_dpll_params_32k_196608khz;
	abe_ref_clk = CM_ABE_PLL_REF_CLKSEL_CLKSEL_32KCLK;

	/*
	 * Enable REGM4XEN to achieve 196.608MHz from 32768 Hz
	 * We need an additional multiplier of 4 on the input frequency
	 * since the input frequency is very low
	 */
	writel(readl(CM_CLKMODE_DPLL_ABE) | CM_CLKMODE_DPLL_DPLL_REGM4XEN_MASK,
		CM_CLKMODE_DPLL_ABE);
#endif

	/* Select the right reference clk */
	modify_reg_32(CM_ABE_PLL_REF_CLKSEL,
			CM_ABE_PLL_REF_CLKSEL_CLKSEL_SHIFT,
			CM_ABE_PLL_REF_CLKSEL_CLKSEL_MASK,
			abe_ref_clk);
	/* Lock the dpll */
	do_setup_dpll(CM_CLKMODE_DPLL_ABE, params, DPLL_LOCK);
}

static void do_scale_tps62361(u32 reg, u32 val)
{
	u32 temp;

	/*
	 * Select SET1 in TPS62361:
	 * VSEL1 is grounded on board. So the following selects
	 * VSEL1 = 0 and VSEL0 = 1
	 */
	omap_set_gpio_direction(TPS62361_VSEL0_GPIO, 0);
	omap_set_gpio_dataout(TPS62361_VSEL0_GPIO, 1);

	temp = TPS62361_I2C_SLAVE_ADDR |
	    (reg << PRM_VC_VAL_BYPASS_REGADDR_SHIFT) |
	    (val << PRM_VC_VAL_BYPASS_DATA_SHIFT) |
	    PRM_VC_VAL_BYPASS_VALID_BIT;

	writel(temp, PRM_VC_VAL_BYPASS);

	while (readl(PRM_VC_VAL_BYPASS) & PRM_VC_VAL_BYPASS_VALID_BIT)
		;
}

static void do_scale_vcore(u32 vcore_reg, u32 volt)
{
	u32 temp;

	temp = SMPS_I2C_SLAVE_ADDR |
	    (vcore_reg << PRM_VC_VAL_BYPASS_REGADDR_SHIFT) |
	    (volt << PRM_VC_VAL_BYPASS_DATA_SHIFT) |
	    PRM_VC_VAL_BYPASS_VALID_BIT;
	writel(temp, PRM_VC_VAL_BYPASS);
	while (readl(PRM_VC_VAL_BYPASS) & PRM_VC_VAL_BYPASS_VALID_BIT)
		;
}

static void scale_vcores(void)
{
	u32 volt, sys_clk_khz, cycles_hi, cycles_low, temp;
	u32 omap4_rev = omap4_revision();

	sys_clk_khz = get_sys_clk_freq() / 1000;

	/*
	 * Setup the dedicated I2C controller for Voltage Control
	 * I2C clk - high period 40% low period 60%
	 */
	cycles_hi = sys_clk_khz * 4 / PRM_VC_I2C_CHANNEL_FREQ_KHZ / 10;
	cycles_low = sys_clk_khz * 6 / PRM_VC_I2C_CHANNEL_FREQ_KHZ / 10;
	/* values to be set in register - less by 5 & 7 respectively */
	cycles_hi -= 5;
	cycles_low -= 7;
	temp = (cycles_hi << PRM_VC_CFG_I2C_CLK_SCLH_SHIFT) |
	       (cycles_low << PRM_VC_CFG_I2C_CLK_SCLL_SHIFT);
	writel(temp, PRM_VC_CFG_I2C_CLK);

	/* Disable high speed mode and all advanced features */
	writel(0x0, PRM_VC_CFG_I2C_MODE);

	/* VCORE 1 */
	if (omap4_rev >= OMAP4460_ES1_0) {
		volt = 1400;
		volt -= TPS62361_BASE_VOLT_MV;
		volt /= 10;
		do_scale_tps62361(TPS62361_REG_ADDR_SET1, volt);
	} else if (omap4_rev == OMAP4430_ES1_0) {
		do_scale_vcore(SMPS_REG_ADDR_VCORE1, SMPS_VOLT_1_5000_V);
	} else {
		do_scale_vcore(SMPS_REG_ADDR_VCORE1, SMPS_VOLT_1_3500_V);
	}

	/* VCORE 2 */
	if ((omap4_rev == OMAP4430_ES2_0) || (omap4_rev == OMAP4430_ES2_1))
		volt = SMPS_VOLT_1_1000_V;
	else
		volt = SMPS_VOLT_1_2000_V;
	do_scale_vcore(SMPS_REG_ADDR_VCORE2, volt);

	/* VCORE 3 */
	if (omap4_rev == OMAP4430_ES2_0)
		volt = SMPS_VOLT_1_1000_V;
	else if (omap4_rev == OMAP4430_ES2_1)
		volt = SMPS_VOLT_1_1125_V;
	else
		volt = SMPS_VOLT_1_2000_V;

	do_scale_vcore(SMPS_REG_ADDR_VCORE3, volt);
}

static void enable_clock_domain(u32 clkctrl_reg, u32 enable_mode)
{
	spl_debug("Enable clock domain - 0x%08x\n", clkctrl_reg);
	modify_reg_32(clkctrl_reg, CD_CLKCTRL_CLKTRCTRL_SHIFT,
		      CD_CLKCTRL_CLKTRCTRL_MASK, enable_mode);
}

static inline void wait_for_clk_enable(u32 clkctrl_addr)
{
	u32 idlest = MODULE_CLKCTRL_IDLEST_DISABLED;

	while ((idlest == MODULE_CLKCTRL_IDLEST_DISABLED) ||
	       (idlest == MODULE_CLKCTRL_IDLEST_TRANSITIONING)) {
		idlest = readl(clkctrl_addr);
		idlest = get_bit_field(idlest, MODULE_CLKCTRL_IDLEST_SHIFT,
				       MODULE_CLKCTRL_IDLEST_MASK);
	}
}

static void enable_clock_module(u32 clkctrl_addr, u32 enable_mode,
				u32 wait_for_enable)
{
	spl_debug("Enable clock module - 0x%08x\n", clkctrl_addr);
	modify_reg_32(clkctrl_addr, MODULE_CLKCTRL_MODULEMODE_SHIFT,
			MODULE_CLKCTRL_MODULEMODE_MASK, enable_mode);
	if (wait_for_enable)
		wait_for_clk_enable(clkctrl_addr);
}

/*
 * Enable a set of clock domains and clock modules associated with them
 */
static void enable_clocks(const u32 *clock_domains,
			  const u32 *clock_modules_hw_auto,
			  const u32 *clock_modules_explicit_en,
			  u32 wait_for_enable)
{
	int i = 0, max = 100;
	/* Put the clock domains in SW_WKUP mode */
	for (i = 0; (i < max) && clock_domains[i]; i++) {
		enable_clock_domain(clock_domains[i],
				    CD_CLKCTRL_CLKTRCTRL_SW_WKUP);
	}

	/* Clock modules that need to be put in HW_AUTO */
	for (i = 0; (i < max) && clock_modules_hw_auto[i]; i++) {
		enable_clock_module(clock_modules_hw_auto[i],
				    MODULE_CLKCTRL_MODULEMODE_HW_AUTO,
				    wait_for_enable);
	};

	/* Clock modules that need to be put in SW_EXPLICIT_EN mode */
	for (i = 0; (i < max) && clock_modules_explicit_en[i]; i++) {
		enable_clock_module(clock_modules_explicit_en[i],
				    MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN,
				    wait_for_enable);
	};

	/* Put the clock domains in HW_AUTO mode now */
	for (i = 0; (i < max) && clock_domains[i]; i++) {
		enable_clock_domain(clock_domains[i],
				    CD_CLKCTRL_CLKTRCTRL_HW_AUTO);
	}
}

/*
 * Enable essential clock domains, modules and
 * do some additional special settings needed
 */
static void enable_basic_clocks(void)
{
	/* Enable optional additional functional clock for GPIO4 */
	writel(readl(CM_L4PER_GPIO4_CLKCTRL) | GPIO4_CLKCTRL_OPTFCLKEN_MASK,
	       CM_L4PER_GPIO4_CLKCTRL);

	/* Enable 96 MHz clock for MMC1 & MMC2 */
	writel(readl(CM_L3INIT_HSMMC1_CLKCTRL) | HSMMC_CLKCTRL_CLKSEL_MASK,
	       CM_L3INIT_HSMMC1_CLKCTRL);
	writel(readl(CM_L3INIT_HSMMC2_CLKCTRL) | HSMMC_CLKCTRL_CLKSEL_MASK,
	       CM_L3INIT_HSMMC2_CLKCTRL);

	/* Select 32KHz clock as the source of GPTIMER1 */
	writel(readl(CM_WKUP_GPTIMER1_CLKCTRL) | GPTIMER1_CLKCTRL_CLKSEL_MASK,
	       CM_WKUP_GPTIMER1_CLKCTRL);

	/* Enable optional 48M functional clock for USB  PHY */
	writel(readl(CM_L3INIT_USBPHY_CLKCTRL) |
	       USBPHY_CLKCTRL_OPTFCLKEN_PHY_48M_MASK, CM_L3INIT_USBPHY_CLKCTRL);

	/* Enable all essential clock domains and modules */
	enable_clocks(clk_domains_essential,
		      clk_modules_hw_auto_essential,
		      clk_modules_explicit_en_essential, 1);
}

/*
 * Enable non-essential clock domains, modules and
 * do some additional special settings needed
 */
static void enable_non_essential_clocks(void)
{
	u32 tmp;
	/* Enable optional functional clock for ISS */
	writel(readl(CM_CAM_ISS_CLKCTRL) | ISS_CLKCTRL_OPTFCLKEN_MASK,
	       CM_CAM_ISS_CLKCTRL);

	/* Enable all optional functional clocks of DSS */
	writel(readl(CM_DSS_DSS_CLKCTRL) | DSS_CLKCTRL_OPTFCLKEN_MASK,
	       CM_DSS_DSS_CLKCTRL);

	/* Enable all non-essential clock domains and modules */
	enable_clocks(clk_domains_non_essential,
		      clk_modules_hw_auto_non_essential,
		      clk_modules_explicit_en_non_essential, 0);

	/* Put camera module in no sleep mode */
	tmp = readl(CM_CAM_CLKSTCTRL);
	set_bit_field(tmp, MODULE_CLKCTRL_MODULEMODE_SHIFT,
		      MODULE_CLKCTRL_MODULEMODE_MASK,
		      CD_CLKCTRL_CLKTRCTRL_NO_SLEEP);
	writel(tmp, CM_CAM_CLKSTCTRL);
}

void freq_update_core(void)
{
	u32 freq_config1 = 0;
	const struct dpll_params *core_dpll_params;

	core_dpll_params = get_core_dpll_params();
	/* Put EMIF clock domain in sw wakeup mode */
	enable_clock_domain(CM_MEMIF_CLKSTCTRL, CD_CLKCTRL_CLKTRCTRL_SW_WKUP);
	wait_for_clk_enable(CM_MEMIF_EMIF_1_CLKCTRL);
	wait_for_clk_enable(CM_MEMIF_EMIF_2_CLKCTRL);

	freq_config1 = SHADOW_FREQ_CONFIG1_FREQ_UPDATE_MASK |
	    SHADOW_FREQ_CONFIG1_DLL_RESET_MASK;

	set_bit_field(freq_config1, SHADOW_FREQ_CONFIG1_DPLL_EN_SHIFT,
		      SHADOW_FREQ_CONFIG1_DPLL_EN_MASK, DPLL_EN_LOCK);

	set_bit_field(freq_config1, SHADOW_FREQ_CONFIG1_M2_DIV_SHIFT,
		      SHADOW_FREQ_CONFIG1_M2_DIV_MASK, core_dpll_params->m2);

	writel(freq_config1, CM_SHADOW_FREQ_CONFIG1);
	while (readl(CM_SHADOW_FREQ_CONFIG1) &
		SHADOW_FREQ_CONFIG1_FREQ_UPDATE_MASK)
		;

	/* Put EMIF clock domain back in hw auto mode */
	enable_clock_domain(CM_MEMIF_CLKSTCTRL, CD_CLKCTRL_CLKTRCTRL_HW_AUTO);
	wait_for_clk_enable(CM_MEMIF_EMIF_1_CLKCTRL);
	wait_for_clk_enable(CM_MEMIF_EMIF_2_CLKCTRL);
}

void bypass_dpll(u32 base)
{
	do_bypass_dpll(base);
	wait_for_bypass(base);
}

void lock_dpll(u32 base)
{
	do_lock_dpll(base);
	wait_for_lock(base);
}

void setup_clocks_for_console(void)
{
	/* Do not add any spl_debug prints in this function */
	modify_reg_32(CM_L4PER_CLKSTCTRL, CD_CLKCTRL_CLKTRCTRL_SHIFT,
		      CD_CLKCTRL_CLKTRCTRL_MASK, CD_CLKCTRL_CLKTRCTRL_SW_WKUP);

	/* Enable all UARTs - console will be on one of them */
	modify_reg_32(CM_L4PER_UART1_CLKCTRL, MODULE_CLKCTRL_MODULEMODE_SHIFT,
			MODULE_CLKCTRL_MODULEMODE_MASK,
			MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN);

	modify_reg_32(CM_L4PER_UART2_CLKCTRL, MODULE_CLKCTRL_MODULEMODE_SHIFT,
			MODULE_CLKCTRL_MODULEMODE_MASK,
			MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN);

	modify_reg_32(CM_L4PER_UART3_CLKCTRL, MODULE_CLKCTRL_MODULEMODE_SHIFT,
			MODULE_CLKCTRL_MODULEMODE_MASK,
			MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN);

	modify_reg_32(CM_L4PER_UART3_CLKCTRL, MODULE_CLKCTRL_MODULEMODE_SHIFT,
			MODULE_CLKCTRL_MODULEMODE_MASK,
			MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN);

	modify_reg_32(CM_L4PER_CLKSTCTRL, CD_CLKCTRL_CLKTRCTRL_SHIFT,
		      CD_CLKCTRL_CLKTRCTRL_MASK, CD_CLKCTRL_CLKTRCTRL_HW_AUTO);
}

void prcm_init(void)
{
	switch (omap4_hw_init_context()) {
	case OMAP_INIT_CONTEXT_SPL:
	case OMAP_INIT_CONTEXT_XIP_UBOOT:
	case OMAP_INIT_CONTEXT_UBOOT_LOADED_BY_CH:
		enable_basic_clocks();
		scale_vcores();
		setup_dplls();
		setup_non_essential_dplls();
		enable_non_essential_clocks();
		break;
	default:
		break;
	}
}
