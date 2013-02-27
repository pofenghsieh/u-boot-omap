/*
 * (C) Copyright 2009
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
#include <config.h>
#ifdef CONFIG_TWL6030

#include <twl6030.h>
#include <bq2415x.h>

#define L_BATT_POLL_PERIOD		20000000	/* 20 seconds */
#define S_BATT_POLL_PERIOD		2000000		/* 2 seconds */
#define POLL_INTERVAL			100000		/* 100 ms */
#define L_BATT_POLL_TIMEOUT		(L_BATT_POLL_PERIOD/POLL_INTERVAL)
#define S_BATT_POLL_TIMEOUT		(S_BATT_POLL_PERIOD/POLL_INTERVAL)
#define CHARGE_DELAY			5
#define SHUTDOWN_COUNT			10
#define BOOT_VOLTAGE			3400		/* 3.4V */

/* Charging Active */
enum {
	NOT_CHARGING = 0,
	CHARGING_AC,
	CHARGING_USB
};

/* Charger Presence */
enum {
	NO_CHARGER = 0,
	VAC_CHARGER,
	VBUS_CHARGER
};

/* Battery Presence */
enum {
	NO_BATTERY = 0,
	BATTERY
};

static int charging = NOT_CHARGING;

static t_channel_calibration_info channel_calib_data[2] = {
	{0, 0xCD, 0xCE, 116, 745, 0, 0, 0}, /* BATT_PRESENCE */
	{7, 0xD3, 0xD4, 614, 941, 0, 0, 0}  /* BATT_VOLTAGE  */
};

/* Functions to read and write from TWL6030 */
static inline int twl6030_i2c_write_u8(u8 chip_no, u8 val, u8 reg)
{
	return i2c_write(chip_no, reg, 1, &val, 1);
}

static inline int twl6030_i2c_read_u8(u8 chip_no, u8 *val, u8 reg)
{
	return i2c_read(chip_no, reg, 1, val, 1);
}

static int twl6030_gpadc_sw2_trigger(t_twl6030_gpadc_data * gpadc)
{
	u8 val;
	int ret = 0;

	ret = twl6030_i2c_write_u8(TWL6030_CHIP_ADC, gpadc->enable, gpadc->ctrl);
	if (ret)
		return -1;

	/* Waiting until the SW1 conversion ends*/
	val =  TWL6030_GPADC_BUSY;

	while (!((val & TWL6030_GPADC_EOC_SW) && (!(val & TWL6030_GPADC_BUSY)))) {
		ret = twl6030_i2c_read_u8(TWL6030_CHIP_ADC, &val, gpadc->ctrl);
		if (ret)
			return -1;
		udelay(1000);
	}

	return 0;
}

static int twl6030_gpadc_read_channel(t_twl6030_gpadc_data * gpadc, u8 channel_no)
{
	u8 lsb = 0;
	u8 msb = 0;
	int ret;
	u8 channel = channel_no;

	if (gpadc->twl_chip_type == chip_TWL6032) {
		ret = twl6030_i2c_write_u8(TWL6030_CHIP_ADC, channel_no,
				TWL6032_GPSELECT_ISB);
		if (ret)
			return -1;
	}

	ret = twl6030_gpadc_sw2_trigger(gpadc);
	if (ret)
		return ret;

	if (gpadc->twl_chip_type == chip_TWL6032)
		channel = 0;

	ret = twl6030_i2c_read_u8(TWL6030_CHIP_ADC, &lsb,
			gpadc->rbase + channel * 2);
	if (ret)
		return -1;

	ret = twl6030_i2c_read_u8(TWL6030_CHIP_ADC, &msb,
			gpadc->rbase + 1 + channel * 2);
	if (ret)
		return -1;

	return (msb << 8) | lsb;
}
/*
 * Function to read in calibration errors and offset data
 * for later ADC conversion use
 */
static int twl6030_calibration(void)
{
	int i;
	int ret = 1;
	int gain_error_1;
	int offset_error;
	s16 ideal_code1, ideal_code2;
	u8 tmp1, tmp2;
	s8 delta_error1, delta_error2;

	for (i = ADC_CH0; i < ADC_CHANNEL_MAX; i++) {

		ret = twl6030_i2c_read_u8(TWL6030_CHIP_ID2, &tmp1,
			channel_calib_data[i].delta_err_reg1);
		if (ret)
			return -1;

		ret = twl6030_i2c_read_u8(TWL6030_CHIP_ID2, &tmp2,
			channel_calib_data[i].delta_err_reg2);
		if (ret)
			return -1;

		delta_error1 = ((s8)(tmp1 << 1) >> 1);
		delta_error2 = ((s8)(tmp2 << 1) >> 1);
		ideal_code1 = channel_calib_data[i].ideal_code1;
		ideal_code2 = channel_calib_data[i].ideal_code2;

		gain_error_1 = (delta_error2 - delta_error1) * SCALE
					/ (ideal_code2 - ideal_code1);
		offset_error = delta_error1 * SCALE - gain_error_1
					*  ideal_code1;
		channel_calib_data[i].gain_err = gain_error_1 + SCALE;
		channel_calib_data[i].offset_err = offset_error;

		channel_calib_data[i].calibrated = 1;

	}

	return 0;
}

/*
 * Function to determine if either a PC or Wall USB is attached
 */
static int is_charger_present(void)
{
	u8 val;
	int ret;

	ret = twl6030_i2c_read_u8(TWL6030_CHIP_CHARGER, &val,
				  CONTROLLER_STAT1);
	if (ret)
		return NO_CHARGER;
	if (val & VAC_DET)
		return VAC_CHARGER;
	else if (val & VBUS_DET)
		return VBUS_CHARGER;

	return NO_CHARGER;
}

/*
 * Function to power down TWL and device
 * It should not return as device will be powered off
 */
void twl6030_shutdown(void)
{
	u8 val;

	twl6030_i2c_read_u8(TWL6030_CHIP_PM, &val,
		TWL6030_PHOENIX_DEV_ON);

	/* Write out all 3 bits to shtudown PMIC power */
	val |= APP_DEVOFF | CON_DEVOFF | MOD_DEVOFF;

	twl6030_i2c_write_u8(TWL6030_CHIP_PM, val,
		TWL6030_PHOENIX_DEV_ON);

	/* TWL should be powered off here */
	while(1) {}
}

/*
 * Function to stop USB charging if charging
 * was enabled. Returns 1 if charging is disabled;
 * and 0 if it is not disabled.
 */
int twl6030_stop_usb_charging(void)
{
	if(charging == NOT_CHARGING)
		return 0;

	/* Disable USB charging */
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, 0, CONTROLLER_CTRL1);

	charging = NOT_CHARGING;

	return 1;
}
/*
 * Function to start USB charging if charging
 * was disabled and charger is now present.
 * Returns 1 if charging is enabled;
 * and 0 if it is not enabled.
 */
int twl6030_start_usb_charging(void)
{
	/*
	 * Only start charging if currently
	 * not charging and there is a charger
	 */
	if(charging != NOT_CHARGING)
		return 0;

	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, CHARGERUSB_VICHRG_1500,
							CHARGERUSB_VICHRG);
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, CHARGERUSB_CIN_LIMIT_NONE,
							CHARGERUSB_CINLIMIT);
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, MBAT_TEMP,
							CONTROLLER_INT_MASK);
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, MASK_MCHARGERUSB_THMREG,
							CHARGERUSB_INT_MASK);
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, CHARGERUSB_VOREG_4P0,
							CHARGERUSB_VOREG);
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, CHARGERUSB_CTRL2_VITERM_100,
							CHARGERUSB_CTRL2);
	/* Enable USB charging */
	twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, CONTROLLER_CTRL1_EN_CHARGER,
							CONTROLLER_CTRL1);
	charging = CHARGING_USB;

	return 1;
}

int twl6030_start_ac_charging(void)
{
	/*
	 * Only start charging if currently
	 * not charging and there is a charger
	 */
	if(charging != NOT_CHARGING)
		return 0;

        twl6030_i2c_write_u8(BQ2415x_CHIP_CHARGER, MAX_CHARGE_CURRENT,
                                                BQ2415x_SAFETY_LIMIT_REG);
        twl6030_i2c_write_u8(BQ2415x_CHIP_CHARGER, NO_INPUT_CURRENT_LIMIT,
                                                BQ2415x_CONTROL_REG);
        twl6030_i2c_write_u8(BQ2415x_CHIP_CHARGER, RGULATION_VOLTAGE,
                                                BQ2415x_VOLTAGE_REG);
        twl6030_i2c_write_u8(BQ2415x_CHIP_CHARGER, CHARGE_CURRENT,
                                                BQ2415x_CURRENT_REG);
        twl6030_i2c_write_u8(BQ2415x_CHIP_CHARGER, NORMAL_CHARGE_CURRENT,
                                                BQ2415x_SPECIAL_REG);
        /* enable AC charging */
        twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, CONTROLLER_CTRL1_EN_CHARGER
                                                | CONTROLLER_CTRL1_SEL_CHARGER,
                                                CONTROLLER_CTRL1);
	charging = CHARGING_AC;

	return 1;
}

static int is_battery_present(t_twl6030_gpadc_data * gpadc)
{
	int bat_id_val;
	unsigned int current_src_val;
	u8 reg;
	int ret;

	bat_id_val = twl6030_gpadc_read_channel(gpadc, 0);
	if (bat_id_val < 0) {
		printf("Failed to read GPADC\n");
		return bat_id_val;
	}

	if (channel_calib_data[ADC_CH0].calibrated) {
		bat_id_val = (bat_id_val * SCALE -
			channel_calib_data[ADC_CH0].offset_err)/
			channel_calib_data[ADC_CH0].gain_err;
	}

	if (gpadc->twl_chip_type == chip_TWL6030)
		bat_id_val = (bat_id_val* 5 * 1000) >> (10 + 2);
	else
		bat_id_val = (bat_id_val * 5 * 1000) >> (12 + 2);

	ret = twl6030_i2c_read_u8(TWL6030_CHIP_ADC, &reg, GPADC_CTRL);
	if (ret) {
		printf("Failed to read GPADC\n");
		return -1;
	}

	current_src_val = (reg & GPADC_CTRL_ISOURCE_EN) ?
				GPADC_ISOURCE_22uA :
				GPADC_ISOURCE_7uA;

	bat_id_val = (bat_id_val * 1000) / current_src_val;

	if (bat_id_val < BATTERY_DETECT_THRESHOLD)
		return NO_BATTERY;

	return BATTERY;
}

int twl6030_get_battery_voltage(t_twl6030_gpadc_data * gpadc)
{
	int battery_volt = 0;
	int stopped_charging, saved_charging;
	u8 vbatch = TWL6030_GPADC_VBAT_CHNL;

	if (gpadc->twl_chip_type == chip_TWL6032)
		vbatch = TWL6032_GPADC_VBAT_CHNL;
	saved_charging = charging;
	/* Stop charging to achieve better accuracy */
	stopped_charging = twl6030_stop_usb_charging();

	/* measure Vbat voltage */
	battery_volt = twl6030_gpadc_read_channel(gpadc, vbatch);
	if (battery_volt < 0) {
		printf("Failed to read battery voltage\n");
		return battery_volt;
	}

	if(stopped_charging) {
		if (saved_charging == CHARGING_USB)
			twl6030_start_usb_charging();
		else
			twl6030_start_ac_charging();
	}
	/* Offset calibration data */
	if(channel_calib_data[ADC_CH7].calibrated) {
		battery_volt = (battery_volt * SCALE -
		channel_calib_data[ADC_CH7].offset_err)/
		channel_calib_data[ADC_CH7].gain_err;
	}

	if (gpadc->twl_chip_type == chip_TWL6030) {
		/*
		 * multiply by 1000 to convert the unit to milli
		 * division by 1024 (>> 10) for 10 bit ADC
		 * division by 8 (>> 3) for actual scaler gain
		 */
		battery_volt = (battery_volt * 40 * 1000) >> (10 + 3);
	}
	else {
		battery_volt = (battery_volt * 25 * 1000) >> (12 + 2);
	}
	return battery_volt;
}

void twl6030_init_battery_charging(void)
{
	u8 val;
	int ret = 0;
	int abort = 0;
	int battery_volt = 0;
	int chargedelay = CHARGE_DELAY;
	int timeout, charger_state;
	int shutdown_counter = SHUTDOWN_COUNT;
	t_twl6030_gpadc_data gpadc;

	gpadc.twl_chip_type = chip_TWL6030;
	gpadc.rbase = GPCH0_LSB;
	gpadc.ctrl = CTRL_P2;
	gpadc.enable = CTRL_P2_SP2;

	ret = twl6030_i2c_read_u8(TWL6030_CHIP_USB, &val, USB_PRODUCT_ID_LSB);

	if (ret == 0) {
		if(val == 0x32)
		{
			gpadc.twl_chip_type = chip_TWL6032;
			gpadc.rbase = TWL6032_GPCH0_LSB;
			gpadc.ctrl = TWL6032_CTRL_P1;
			gpadc.enable = CTRL_P1_SP1;
		}
	} else {
		printf("twl6030_init_battery_charging(): "
		       "could not determine chip!\n");
		return;
	}

	/* Forced stop charging */
	charging = CHARGING_USB;
	ret = twl6030_stop_usb_charging();
	if (ret == 0 ) {
	        printf("Failed to stop charging\n");
	}

	/* Calibration */
	ret = twl6030_calibration();
	if (ret) {
	        printf("Failed to calibrate\n");
	}

	/* Enable VBAT measurement */
	if (gpadc.twl_chip_type == chip_TWL6030) {
		twl6030_i2c_write_u8(TWL6030_CHIP_PM, VBAT_MEAS, MISC1);
		twl6030_i2c_write_u8(TWL6030_CHIP_ADC, GPADC_CTRL_SCALER_DIV4,
			TWL6030_GPADC_CTRL);
	}
	else
		twl6030_i2c_write_u8(TWL6030_CHIP_ADC,
			GPADC_CTRL2_CH18_SCALER_EN, TWL6032_GPADC_CTRL2);

	/* Enable GPADC module */
	ret = twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, FGS | GPADCS, TOGGLE1);
	if (ret) {
		printf("Failed to enable GPADC\n");
		return;
	}

	/*
	 * Make dummy conversion for the TWL6032
	 * (first conversion may be failed)
	 */
	if (gpadc.twl_chip_type == chip_TWL6032)
		twl6030_gpadc_sw2_trigger(&gpadc);

	/*
	 * In case if battery is absent or error occurred while the battery
	 * detection we will not turn on the battery charging
	 */
	if (is_battery_present(&gpadc) <= 0) {
		printf("Battery not detected\n");
		return;
	}

	battery_volt = twl6030_get_battery_voltage(&gpadc);

	if (battery_volt < 0 || battery_volt >= BOOT_VOLTAGE)
		return;

#ifdef CONFIG_SILENT_CONSOLE
	if (gd->flags & GD_FLG_SILENT) {
		/* Restore serial console */
		console_assign (stdout, "serial");
		console_assign (stderr, "serial");
	}
#endif

	printf("Main battery voltage too low!\n");
	printf("Hit any key to stop charging: %2d ", chargedelay);

	if (tstc()) {	/* we got a key press	*/
		(void) getc();  /* consume input	*/
	}

	while ((chargedelay > 0) && (!abort)) {
		int i;

		--chargedelay;
		/* delay 100 * 10ms */
		for (i=0; !abort && i<100; ++i) {
			if (tstc()) {	/* we got a key press	*/
				abort  = 1;	/* don't auto boot	*/
				chargedelay = 0;	/* no more delay	*/
				(void) getc();  /* consume input	*/
				break;
			}
			udelay (10000);
		}
		printf ("\b\b\b%2d ", chargedelay);
	}
	putc ('\n');

#ifdef CONFIG_SILENT_CONSOLE
	if (gd->flags & GD_FLG_SILENT) {
		/* Restore silent console */
		console_assign (stdout, "nulldev");
		console_assign (stderr, "nulldev");
	}
#endif

	if (abort)
		return;

	charger_state = is_charger_present();
	if (charger_state == VAC_CHARGER)
		ret = twl6030_start_ac_charging();
	else if (charger_state == VBUS_CHARGER)
		ret = twl6030_start_usb_charging();
	if (charger_state == NO_CHARGER || ret==0){
		printf("Charger not detected.");
		goto shutdown;
	}

	printf("Charging...\n");
	/*
	 * Wait for battery to charge to the level when kernel can boot
	 * During this time, battery voltage is polled periodically and
	 * charger presence is monitored. If charger is detected to be
	 * unplugged for a period of time, the device will proceed
	 * to shutdown to avoid battery drain.
	 */
	do {
		/* Read battery voltage */
		battery_volt = twl6030_get_battery_voltage(&gpadc);

		printf("\rBattery Voltage: %d mV", battery_volt);

		/* reset safety timer */
		twl6030_i2c_write_u8(TWL6030_CHIP_CHARGER, 32,
					CONTROLLER_WDG);

		twl6030_i2c_write_u8(BQ2415x_CHIP_CHARGER,
                                                        TMR_RST | EN_STAT,
                                                        BQ2415x_STATUS_CTRL_REG);
		timeout = (charger_state == NO_CHARGER)?
			S_BATT_POLL_TIMEOUT : L_BATT_POLL_TIMEOUT;

		while (timeout--)
			udelay(POLL_INTERVAL);

		/* Charger plug or unplug action detected*/
		if (charger_state != is_charger_present()) {

			charger_state = is_charger_present();

			if (charging != NOT_CHARGING){
				ret = twl6030_stop_usb_charging();
			}

			if (charging == NOT_CHARGING) {
				if (charger_state == VAC_CHARGER) {
					/* VAC Charger plugged */
					twl6030_start_ac_charging();
				} else if (charger_state == VBUS_CHARGER) {
					/* VBUS Charger plugged */
					twl6030_start_usb_charging();
				}
			} else if (charger_state == NO_CHARGER) {

					printf("\rCharger Unplugged!       ");
					/* Charger unplugged */
					twl6030_stop_usb_charging();

					/* Will count down before shutdown */
					shutdown_counter = SHUTDOWN_COUNT;
				}

		} else if (charging == NOT_CHARGING &&
				charger_state == NO_CHARGER) {

			/*
			 * Charger continues to be unplugged.
			 * Countdown until 0 and shut off device
			 */

			printf("\rCharger Unplugged! (%d)   ",
				--shutdown_counter);

			if (shutdown_counter == 0)
				goto shutdown;
		}


	} while (battery_volt < BOOT_VOLTAGE);

	printf("\n");

	return;

shutdown:

	printf("\nShutdown!\n");
	twl6030_shutdown();
	return;	/*Should never get here */

}

void twl6030_usb_device_settings()
{
	u8 data = 0;

	/* Select APP Group and set state to ON */
	twl6030_i2c_write_u8(TWL6030_CHIP_PM, 0x21, VUSB_CFG_STATE);

	twl6030_i2c_read_u8(TWL6030_CHIP_PM, &data, MISC2);
	data |= 0x10;

	/* Select the input supply for VBUS regulator */
	twl6030_i2c_write_u8(TWL6030_CHIP_PM, data, MISC2);
}
#endif
