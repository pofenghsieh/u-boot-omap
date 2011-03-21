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

#define abs(x) (((x) < 0) ? ((x)*-1) : (x))

static const u32 sys_clk_array[8] = {
	0,		      /* Uninitialized */
	12000000,	       /* 12 MHz */
	13000000,	       /* 13 MHz */
	16800000,	       /* 16.8 MHz */
	19200000,	       /* 19.2 MHz */
	26000000,	       /* 26 MHz */
	27000000,	       /* 27 MHz */
	38400000,	       /* 38.4 MHz */
};

static const struct dpll_params core_dpll_params_opp100 = {
	0, 0, 1600000, 1, 5, 8, 4, 6, 5
};

static const struct dpll_params core_dpll_params_opp100_ddr200 = {
	0, 0, 1600000, 2, 5, 8, 4, 6, 5
};

static const struct dpll_params core_dpll_params_es1_l3_190 = {
	0, 0, 1523712, 1, 5, 8, 4, 6, 5
};

static const struct dpll_params per_dpll_params_opp100 = {
	0, 0, 1536000, 8, 6, 12, 9, 4, 5
};

/* TODO - fix MPU mult */
static const struct dpll_params mpu_dpll_params_600mhz = {
	0, 0, 1200000, 1, -1, -1, -1, -1, -1
};

static const struct dpll_params mpu_dpll_params_1000mhz = {
	0, 0, 2000000, 1, -1, -1, -1, -1, -1
};

static const struct dpll_params usb_dpll_params = {
	0, 0, 1920000, 2, -1, -1, -1, -1, -1
};

static const struct dpll_params iva_dpll_params = {
	0, 0, 1862000, -1, -1, 4, 7, -1, -1
};

static const struct dpll_params abe_dpll_params = {
	0, 0, 196608, 1, 1, -1, -1, -1, -1
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

static u32 get_m_n(u32 target_freq_khz, u32 ref_freq_khz, u32 *m, u32 *n,
		   u32 tolerance_khz)
{
	u32 min_freq = target_freq_khz - tolerance_khz;
	u32 max_freq = target_freq_khz;
	u32 freq, freq_old;
	*n = 1;
	while (1) {
		*m = min_freq / ref_freq_khz / 2 * (*n);
		freq_old = 0;
		while (1) {
			freq = ref_freq_khz * 2 * (*m) / (*n);
			if (abs(target_freq_khz - freq_old) <=
			    abs(target_freq_khz - freq)) {
				freq = freq_old;
				(*m)--;
				break;
			}
			(*m)++;
			freq_old = freq;
		}
		if (freq >= min_freq && freq <= max_freq)
			break;
		(*n)++;
		if ((*n) > OMAP_DPLL_MAX_N + 1)
			return 1;
	}
	(*n)--;
	return 0;
}

static u32 __get_syc_clk_freq(void)
{
	/*
	 * For ES1 the ROM code calibration of sys clock is not reliable
	 * due to hw issue. So, use hard-coded value. If this value is not
	 * correct for any board over-ride this function in board file
	 * From ES2.0 onwards you will get this information from
	 * CM_SYS_CLKSEL
	 */
	if (omap4_revision() == OMAP4430_ES1_0)
		return OMAP_SYS_CLK_FREQ_38_4_MHZ;
	else {
		u32 sys_clk_ind = readl(CM_SYS_CLKSEL) &
				  CM_SYS_CLKSEL_SYS_CLKSEL_MASK;
		return sys_clk_array[sys_clk_ind];
	}
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

static void do_setup_dpll(u32 base, const struct dpll_params *params,
			  u32 ref_clk_khz, u32 tolerance, u8 lock)
{
	u32 temp, m, n;
	struct dpll_regs *dpll_regs = (struct dpll_regs *)base;

	do_bypass_dpll(base);

	/* Get the M & N values */
	m = params->m;
	n = params->n;
	/* if m & n are not specified calculate them */
	if (!(m && n)) {
		if (get_m_n(params->locked_freq_khz, ref_clk_khz,
			    &m, &n, tolerance)) {
			/* DPLL locking is critical if it fails just hang */
			for (;;)
				;
			return;
		}
	}

	/* Wait till the DPLL is in BYPASS */
	wait_for_bypass(base);

	/* Set M & N */
	temp = readl(&dpll_regs->cm_clksel_dpll);
	set_bit_field(temp, CM_CLKSEL_DPLL_M_SHIFT, CM_CLKSEL_DPLL_M_MASK, m);
	set_bit_field(temp, CM_CLKSEL_DPLL_N_SHIFT, CM_CLKSEL_DPLL_N_MASK, n);
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
	switch (omap4_revision()) {
	case OMAP4430_ES1_0:
		return &core_dpll_params_es1_l3_190;
	case OMAP4430_ES2_0:
	case OMAP4430_SILICON_ID_INVALID:
		return &core_dpll_params_opp100_ddr200; /* safest */
	case OMAP4430_ES2_1:
	case OMAP4430_ES2_2:
		return &core_dpll_params_opp100;
	default:
		return NULL;
	}
}

u32 omap4_ddr_clk(void)
{
	u32 ddr_clk;
	const struct dpll_params *core_dpll_params;

	core_dpll_params = get_core_dpll_params();
	/*
	 * DDR frequency is PHY_ROOT_CLK/2
	 * PHY_ROOT_CLK = Fdpll/2/M2
	 */
	ddr_clk = core_dpll_params->locked_freq_khz / 4 / core_dpll_params->m2;
	ddr_clk *= 1000;	/* convert to Hz */
	return ddr_clk;
}

static void setup_dplls(void)
{
	u32 sys_clk_khz, temp;
	const struct dpll_params *params;

	sys_clk_khz = get_syc_clk_freq() / 1000;

	/* CORE dpll */
	params = get_core_dpll_params();	/* default - safest */
	/*
	 * Do not lock the core DPLL now. Just set it up.
	 * Core DPLL will be locked after setting up EMIF
	 * using the FREQ_UPDATE method(freq_update_core())
	 */
	do_setup_dpll(CM_CLKMODE_DPLL_CORE, params, sys_clk_khz,
		      DPLL_LOCKED_FREQ_TOLERANCE_0, CONFIGURE_NO_LOCK);
	/* Set the ratios for CORE_CLK, L3_CLK, L4_CLK */
	temp = (CLKSEL_CORE_X2_DIV_1 << CLKSEL_CORE_SHIFT) |
	    (CLKSEL_L3_CORE_DIV_2 << CLKSEL_L3_SHIFT) |
	    (CLKSEL_L4_L3_DIV_2 << CLKSEL_L4_SHIFT);
	writel(temp, CM_CLKSEL_CORE);

	/* lock PER dpll */
	do_setup_dpll(CM_CLKMODE_DPLL_PER, &per_dpll_params_opp100, sys_clk_khz,
		      DPLL_LOCKED_FREQ_TOLERANCE_0, CONFIGURE_AND_LOCK);

	/* MPU dpll */
	if (omap4_revision() == OMAP4430_ES1_0)
		params = &mpu_dpll_params_600mhz;
	else
		params = &mpu_dpll_params_1000mhz;
	do_setup_dpll(CM_CLKMODE_DPLL_MPU, params, sys_clk_khz,
		      DPLL_LOCKED_FREQ_TOLERANCE_0, CONFIGURE_AND_LOCK);
}

static void setup_non_essential_dplls(void)
{
	u32 sys_clk_khz, temp, abe_ref_clk;
	u32 m, n, sd_div, num, den;
	struct dpll_params tmp_params;

	sys_clk_khz = get_syc_clk_freq() / 1000;

	/* IVA */
	do_setup_dpll(CM_CLKMODE_DPLL_IVA, &iva_dpll_params, sys_clk_khz,
		      DPLL_LOCKED_FREQ_TOLERANCE_500_KHZ, CONFIGURE_AND_LOCK);

	/* USB */
	tmp_params = usb_dpll_params;
	get_m_n(tmp_params.locked_freq_khz, sys_clk_khz, &m, &n,
		DPLL_LOCKED_FREQ_TOLERANCE_0);
	/*
	 * USB dpll is J-type. Need to set DPLL_SD_DIV for jitter correction
	 * DPLL_SD_DIV = CEILING ([DPLL_MULT/(DPLL_DIV+1)]* CLKINP / 250)
	 *      - where CLKINP is sys_clk in MHz
	 * Use CLKINP in KHz and adjust the denominator accordingly so
	 * that we have enough accuracy and at the same time no overflow
	 */
	num = m * sys_clk_khz;
	den = (n + 1) * 250 * 1000;
	num += den - 1;
	sd_div = num / den;
	temp = readl(CM_CLKSEL_DPLL_USB);
	set_bit_field(temp, CM_CLKSEL_DPLL_DPLL_SD_DIV_SHIFT,
		      CM_CLKSEL_DPLL_DPLL_SD_DIV_MASK, sd_div);
	writel(temp, CM_CLKSEL_DPLL_USB);
	/* Now setup the dpll with the regular function */
	tmp_params.m = m;
	tmp_params.n = n;
	do_setup_dpll(CM_CLKMODE_DPLL_USB, &tmp_params, sys_clk_khz,
		      DPLL_LOCKED_FREQ_TOLERANCE_0, CONFIGURE_AND_LOCK);

	/* ABE dpll */
	tmp_params = abe_dpll_params;
#ifdef CONFIG_SYS_OMAP4_ABE_SYSCK
	abe_ref_clk = sys_clk_khz;
#else
	/* Enable REGM4XEN to achieve 196.608MHz from 32768 Hz */
	writel(readl(CM_CLKMODE_DPLL_ABE) | CM_CLKMODE_DPLL_DPLL_REGM4XEN_MASK,
		CM_CLKMODE_DPLL_ABE);

	/*
	 * Converting the input clock 32768 Hz to KHz would result in accuracy
	 * loss. Instead make both target frequency and reference clock to be
	 * in terms of Hz.
	 *
	 * Also, REGM4XEN is enabled so there is an additional
	 * multiplier of 4 for M. So adjust for this while passing target
	 * frequency to get_m_n()
	 */
	tmp_params.locked_freq_khz = tmp_params.locked_freq_khz * 1000 / 4;
	abe_ref_clk = OMAP_32K_CLK_FREQ;
#endif
	temp = (abe_ref_clk == OMAP_32K_CLK_FREQ) ?
			CM_ABE_PLL_REF_CLKSEL_CLKSEL_32KCLK :
			CM_ABE_PLL_REF_CLKSEL_CLKSEL_SYSCLK;
	/* Select the right reference clk */
	modify_reg_32(CM_ABE_PLL_REF_CLKSEL,
			CM_ABE_PLL_REF_CLKSEL_CLKSEL_SHIFT,
			CM_ABE_PLL_REF_CLKSEL_CLKSEL_MASK,
			temp);
	/* Lock the dpll */
	do_setup_dpll(CM_CLKMODE_DPLL_ABE, &tmp_params, abe_ref_clk,
		      DPLL_LOCKED_FREQ_TOLERANCE_0, CONFIGURE_AND_LOCK);
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

	sys_clk_khz = get_syc_clk_freq() / 1000;

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
	if ((omap4_rev == OMAP4430_ES2_0) || (omap4_rev == OMAP4430_ES2_1))
		volt = SMPS_VOLT_1_3500_V;
	else
		volt = SMPS_VOLT_1_5000_V;
	do_scale_vcore(SMPS_REG_ADDR_VCORE1, volt);

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
	writel(readl(CM_L4PER_GPIO4_CLKCTRL) | GPIO4_CLKCTRL_OPTFCLKEN_SHIFT,
	       CM_L4PER_GPIO4_CLKCTRL);

	/* Enable 96 MHz clock for MMC1 & MMC2 */
	writel(readl(CM_L3INIT_HSMMC1_CLKCTRL) | HSMMC_CLKCTRL_CLKSEL_SHIFT,
	       CM_L3INIT_HSMMC1_CLKCTRL);
	writel(readl(CM_L3INIT_HSMMC2_CLKCTRL) | HSMMC_CLKCTRL_CLKSEL_SHIFT,
	       CM_L3INIT_HSMMC2_CLKCTRL);

	/* Select 32KHz clock as the source of GPTIMER1 */
	writel(readl(CM_WKUP_GPTIMER1_CLKCTRL) | GPTIMER1_CLKCTRL_CLKSEL_SHIFT,
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
	writel(readl(CM_CAM_ISS_CLKCTRL) | ISS_CLKCTRL_OPTFCLKEN_SHIFT,
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

u32 get_syc_clk_freq(void)
	__attribute__ ((weak, alias("__get_syc_clk_freq")));

void setup_clocks_for_console(void)
{
	enable_clock_domain(CM_L4PER_CLKSTCTRL, CD_CLKCTRL_CLKTRCTRL_SW_WKUP);

	/* Enable all UARTs - console will be on one of them */
	enable_clock_module(CM_L4PER_UART1_CLKCTRL,
			    MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN, 1);
	enable_clock_module(CM_L4PER_UART2_CLKCTRL,
			    MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN, 1);
	enable_clock_module(CM_L4PER_UART3_CLKCTRL,
			    MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN, 1);
	enable_clock_module(CM_L4PER_UART4_CLKCTRL,
			    MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN, 1);

	enable_clock_domain(CM_L4PER_CLKSTCTRL, CD_CLKCTRL_CLKTRCTRL_HW_AUTO);
}

void prcm_init(void)
{
	switch (omap4_hw_init_context()) {
	case OMAP_INIT_CONTEXT_SPL:
		scale_vcores();
		setup_dplls();
		enable_basic_clocks();
		break;
	case OMAP_INIT_CONTEXT_UBOOT_LOADED_BY_SPL:
		setup_non_essential_dplls();
		enable_non_essential_clocks();
		break;
	case OMAP_INIT_CONTEXT_XIP_UBOOT:
	case OMAP_INIT_CONTEXT_UBOOT_LOADED_BY_CH:
		scale_vcores();
		setup_dplls();
		enable_basic_clocks();
		setup_non_essential_dplls();
		enable_non_essential_clocks();
		break;
	}
}
