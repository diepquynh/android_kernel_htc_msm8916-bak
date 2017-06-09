/* Copyright (c) 2013-2014 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) "[BATT] SMB:%s: " fmt, __func__

#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/math64.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/wakelock.h>


#ifdef CONFIG_HTC_BATT_8960
#include <linux/power/smb1360-charger-fg.h>
#include <linux/usb/cable_detect.h>
#include <htc/devices_dtb.h>
#include <linux/power/htc_gauge.h>
#include <linux/power/htc_charger.h>
#include <linux/power/htc_battery_cell.h>
#include <linux/power/htc_battery_common.h>
#include <linux/of_batterydata.h>
#endif

#ifdef pr_debug
#undef pr_debug
#endif
#define pr_debug(fmt, args...) do { \
		if (flag_enable_bms_charger_log) \
			printk(KERN_INFO pr_fmt(fmt), ## args); \
	} while (0)


#define _SMB1360_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define SMB1360_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_SMB1360_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
				(RIGHT_BIT_POS))

#define CFG_BATT_CHG_REG		0x00
#define CHG_ITERM_MASK			SMB1360_MASK(2, 0)
#define CHG_ITERM_25MA			0x0
#define CHG_ITERM_200MA			0x7
#define RECHG_MV_MASK			SMB1360_MASK(6, 5)
#define RECHG_MV_SHIFT			5
#define OTG_CURRENT_MASK		SMB1360_MASK(4, 3)
#define OTG_CURRENT_SHIFT		3

#define CFG_BATT_CHG_ICL_REG		0x05
#define AC_INPUT_ICL_PIN_BIT		BIT(7)
#define AC_INPUT_PIN_HIGH_BIT		BIT(6)
#define RESET_STATE_USB_500		BIT(5)
#define INPUT_CURR_LIM_MASK		SMB1360_MASK(3, 0)
#define INPUT_CURR_LIM_300MA		0x0

#define CFG_GLITCH_FLT_REG		0x06
#define AICL_ENABLED_BIT		BIT(0)
#define INPUT_UV_GLITCH_FLT_20MS_BIT	BIT(7)

#define CFG_CHG_MISC_REG		0x7
#define CHG_EN_BY_PIN_BIT		BIT(7)
#define CHG_EN_ACTIVE_LOW_BIT		BIT(6)
#define PRE_TO_FAST_REQ_CMD_BIT		BIT(5)
#define CHG_CURR_TERM_DIS_BIT		BIT(3)
#define CFG_AUTO_RECHG_DIS_BIT		BIT(2)
#define CFG_CHG_INHIBIT_EN_BIT		BIT(0)

#define CFG_CHG_FUNC_CTRL_REG		0x08
#define CHG_RECHG_THRESH_FG_SRC_BIT	BIT(1)

#define CFG_STAT_CTRL_REG		0x09
#define CHG_STAT_IRQ_ONLY_BIT		BIT(4)
#define CHG_TEMP_CHG_ERR_BLINK_BIT	BIT(3)
#define CHG_STAT_ACTIVE_HIGH_BIT	BIT(1)
#define CHG_STAT_DISABLE_BIT		BIT(0)

#define CFG_SFY_TIMER_CTRL_REG		0x0A
#define SAFETY_TIME_EN_BIT		BIT(4)
#define SAFETY_TIME_DISABLE_BIT		BIT(5)
#define SAFETY_TIME_MINUTES_SHIFT	2
#define SAFETY_TIME_MINUTES_MASK	SMB1360_MASK(3, 2)

#define CFG_WD_CTRL			0x0C
#define CFG_WD_CTRL_BIT			BIT(0)

#define CFG_BATT_MISSING_REG		0x0D
#define BATT_MISSING_SRC_THERM_BIT	BIT(1)

#define CFG_FG_BATT_CTRL_REG		0x0E
#define CFG_FG_OTP_BACK_UP_ENABLE	BIT(7)
#define CHG_JEITA_DISABLED_BIT		BIT(6)
#define BATT_ID_ENABLED_BIT		BIT(5)
#define CHG_BATT_ID_FAIL		BIT(4)
#define BATT_ID_FAIL_SELECT_PROFILE	BIT(3)
#define BATT_PROFILE_SELECT_MASK	SMB1360_MASK(3, 0)
#define BATT_PROFILEA_MASK		0x0
#define BATT_PROFILEB_MASK		0xF

#define IRQ_CFG_REG			0x0F
#define IRQ_BAT_HOT_COLD_HARD_BIT	BIT(7)
#define IRQ_BAT_HOT_COLD_SOFT_BIT	BIT(6)
#define IRQ_DCIN_UV_BIT			BIT(2)
#define IRQ_INTERNAL_TEMPERATURE_BIT	BIT(0)

#define IRQ2_CFG_REG			0x10
#define IRQ2_SAFETY_TIMER_BIT		BIT(7)
#define IRQ2_CHG_ERR_BIT		BIT(6)
#define IRQ2_CHG_PHASE_CHANGE_BIT	BIT(4)
#define IRQ2_POWER_OK_BIT		BIT(2)
#define IRQ2_BATT_MISSING_BIT		BIT(1)
#define IRQ2_VBAT_LOW_BIT		BIT(0)

#define IRQ3_CFG_REG			0x11
#define IRQ3_SOC_CHANGE_BIT		BIT(4)
#define IRQ3_SOC_MIN_BIT		BIT(3)
#define IRQ3_SOC_MAX_BIT		BIT(2)
#define IRQ3_SOC_EMPTY_BIT		BIT(1)
#define IRQ3_SOC_FULL_BIT		BIT(0)

#define CHG_CURRENT_REG			0x13
#define FASTCHG_CURR_MASK		SMB1360_MASK(4, 2)
#define FASTCHG_CURR_SHIFT		2

#define TEMP_COMPEN_REG				0x14
#define TEMP_HOT_VOL_COMPEN_BIT		BIT(7)
#define TEMP_COLD_VOL_COMPEN_BIT	BIT(6)
#define TEMP_HOT_CURR_COMPEN_BIT	BIT(5)
#define TEMP_COLD_CURR_COMPEN_BIT	BIT(4)

#define BATT_CHG_FLT_VTG_REG		0x15
#define VFLOAT_MASK			SMB1360_MASK(6, 0)

#define SHDN_CTRL_REG			0x1A
#define SHDN_CMD_USE_BIT		BIT(1)
#define SHDN_CMD_POLARITY_BIT		BIT(2)

#define CURRENT_GAIN_LSB_REG		0x1D
#define CURRENT_GAIN_MSB_REG		0x1E

#define CMD_I2C_REG			0x40
#define ALLOW_VOLATILE_BIT		BIT(6)
#define FG_ACCESS_ENABLED_BIT		BIT(5)
#define FG_RESET_BIT			BIT(4)
#define CYCLE_STRETCH_CLEAR_BIT		BIT(3)

#define CMD_IL_REG			0x41
#define USB_CTRL_MASK			SMB1360_MASK(1 , 0)
#define USB_100_BIT			0x01
#define USB_500_BIT			0x00
#define USB_AC_BIT			0x02
#define SHDN_CMD_BIT			BIT(7)

#define CMD_CHG_REG			0x42
#define CMD_CHG_EN			BIT(1)
#define CMD_OTG_EN_BIT			BIT(0)

#define STATUS_3_REG			0x4B
#define CHG_HOLD_OFF_BIT		BIT(3)
#define CHG_TYPE_MASK			SMB1360_MASK(2, 1)
#define CHG_TYPE_SHIFT			1
#define BATT_NOT_CHG_VAL		0x0
#define BATT_PRE_CHG_VAL		0x1
#define BATT_FAST_CHG_VAL		0x2
#define BATT_TAPER_CHG_VAL		0x3
#define CHG_EN_BIT			BIT(0)
#define CHG_FASTCHG_BIT		BIT(2)

#define STATUS_4_REG			0x4C
#define CYCLE_STRETCH_ACTIVE_BIT	BIT(5)

#define REVISION_CTRL_REG		0x4F
#define DEVICE_REV_MASK			SMB1360_MASK(3, 0)

#define IRQ_A_REG			0x50
#define IRQ_A_HOT_HARD_BIT		BIT(6)
#define IRQ_A_COLD_HARD_BIT		BIT(4)
#define IRQ_A_HOT_SOFT_BIT		BIT(2)
#define IRQ_A_COLD_SOFT_BIT		BIT(0)

#define IRQ_B_REG			0x51
#define IRQ_B_BATT_TERMINAL_BIT		BIT(6)
#define IRQ_B_BATT_MISSING_BIT		BIT(4)

#define IRQ_C_REG			0x52
#define IRQ_C_CHG_TERM			BIT(0)
#define IRQ_C_TAPER_CHG_BIT		BIT(2)
#define IRQ_C_FAST_CHG_BIT		BIT(6)

#define IRQ_D_REG			0x53
#define IRQ_D_CHARGE_TIMEOUT_BIT	BIT(2)

#define IRQ_E_REG			0x54
#define IRQ_E_USBIN_OV_BIT		BIT(2)
#define IRQ_E_USBIN_UV_BIT		BIT(0)

#define IRQ_F_REG			0x55
#define IRQ_F_POWER_OK_BIT		BIT(0)

#define IRQ_G_REG			0x56

#define IRQ_H_REG			0x57
#define IRQ_H_FULL_SOC_BIT		BIT(6)

#define IRQ_I_REG			0x58
#define FG_ACCESS_ALLOWED_BIT		BIT(0)
#define BATT_ID_RESULT_BIT		SMB1360_MASK(6, 4)
#define BATT_ID_SHIFT			4

#define SOC_MAX_REG			0x24
#define SOC_MIN_REG			0x25
#define VTG_EMPTY_REG			0x26
#define SOC_DELTA_REG			0x28
#define JEITA_SOFT_COLD_REG		0x29
#define JEITA_SOFT_HOT_REG		0x2A
#define VTG_MIN_REG			0x2B
#define ZERO_DEGREE_BASE_REG		0X1E

#define SHDW_FG_ESR_ACTUAL		0x20
#define SHDW_FG_BATT_STATUS		0x60
#define BATTERY_PROFILE_BIT		BIT(0)

#define SHDW_FG_MSYS_SOC		0x61
#define SHDW_FG_CAPACITY		0x62
#define SHDW_FG_VTG_NOW			0x69
#define SHDW_FG_CURR_NOW		0x6B
#define SHDW_FG_BATT_TEMP		0x6D

#define CC_TO_SOC_COEFF			0xBA
#define NOMINAL_CAPACITY_REG		0xBC
#define ACTUAL_CAPACITY_REG		0xBE
#define FG_AUTO_RECHARGE_SOC		0xD2
#define FG_SYS_CUTOFF_V_REG		0xD3
#define FG_CC_TO_CV_V_REG		0xD5
#define FG_ITERM_REG			0xD9
#define FG_THERM_C1_COEFF_REG		0xDB
#define FG_IBATT_STANDBY_REG		0xCF

#define FG_I2C_CFG_MASK			SMB1360_MASK(2, 1)
#define FG_CFG_I2C_ADDR			0x2
#define FG_PROFILE_A_ADDR		0x4
#define FG_PROFILE_B_ADDR		0x6

#define CURRENT_100_MA			100
#define CURRENT_500_MA			500
#define MAX_8_BITS			255

#define SMB1360_REV_1			0x01

#define A51_BATT_ID_1			0x06 
#define A51_BATT_ID_2			0x05 
#define A51_BATT_ID_RESULT_ERR		0x04 

#define SMB1360_EOC_CHECK_PERIOD_MS	10000

#ifdef CONFIG_HTC_BATT_8960
#define USB_MA_0       (0)
#define USB_MA_2       (2)
#define USB_MA_100     (100)
#define USB_MA_200     (200)
#define USB_MA_300     (300)
#define USB_MA_400     (400)
#define USB_MA_450     (450)
#define USB_MA_500     (500)
#define USB_MA_900     (900)
#define USB_MA_1000    (1000)
#define USB_MA_1100    (1100)
#define USB_MA_1200	(1200)
#define USB_MA_1500	(1500)
#define USB_MA_1600	(1600)

#define SMB1360_INPUT_LIMIT_MA	800
#define SMB1360_CHG_I_MIN_MA	450

#define COMBINED_CHGR_MAX_AICL_INPUT_CURR_LIMIT	1700
#define PM8916_MAX_INPUT_CURR_LIMIT	500

#define SMB1360_AICL_START_PM8916_DELAY_MS	1000
#define SMB1360_AICL_CHECK_PERIOD_MS	1000

#define VBUS_DROP_SCALE_UPPER	150
#define VBUS_DROP_SCALE_LOWER	50
#endif 

enum {
	WRKRND_FG_CONFIG_FAIL = BIT(0),
	WRKRND_BATT_DET_FAIL = BIT(1),
	WRKRND_USB100_FAIL = BIT(2),
};

enum {
	USER	= BIT(0),
	THERMAL = BIT(1),
	CURRENT = BIT(2),
};

enum fg_i2c_access_type {
	FG_ACCESS_CFG = 0x1,
	FG_ACCESS_PROFILE_A = 0x2,
	FG_ACCESS_PROFILE_B = 0x3
};

enum {
	BATTERY_PROFILE_A,
	BATTERY_PROFILE_B,
	BATTERY_PROFILE_MAX,
};

enum {
	IRQ_A_REG_SEQUENCE,
	IRQ_B_REG_SEQUENCE,
	IRQ_C_REG_SEQUENCE,
	IRQ_D_REG_SEQUENCE,
	IRQ_E_REG_SEQUENCE,
	IRQ_F_REG_SEQUENCE,
	IRQ_G_REG_SEQUENCE,
	IRQ_H_REG_SEQUENCE,
	IRQ_I_REG_SEQUENCE,

	TOTAL_IRQ_REG_SEQUENCE
};

static int otg_curr_ma[] = {350, 550, 950, 1500};

struct smb1360_otg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct smb1360_chip {
	struct i2c_client		*client;
	struct device			*dev;
	u8				revision;
	unsigned short			default_i2c_addr;
	unsigned short			fg_i2c_addr;
	bool				pulsed_irq;

	
	int				fake_battery_soc;
	bool				batt_id_disabled;
	bool				charging_disabled;
	bool				recharge_disabled;
	bool				chg_inhibit_disabled;
	bool				iterm_disabled;
	bool				shdn_after_pwroff;
	int				iterm_ma;
	int				vfloat_mv;
	int				safety_time;
	int				resume_delta_mv;
	u32				default_batt_profile;
	unsigned int			thermal_levels;
	unsigned int			therm_lvl_sel;
	unsigned int			*thermal_mitigation;
	int				otg_batt_curr_limit;
	bool				min_icl_usb100;
	unsigned int			warm_bat_mv;
	unsigned int			clamp_soc_voltage_mv;
	int				batt_full_criteria;
	int				batt_eoc_criteria;
	int				cool_bat_decidegc;
	int				warm_bat_decidegc;

	
	int				soc_max;
	int				soc_min;
	int				delta_soc;
	int				voltage_min_mv;
	int				voltage_empty_mv;
	int				voltage_empty_re_check_mv;
	int				batt_capacity_mah;
	int				cc_soc_coeff;
	int				v_cutoff_mv;
	int				fg_iterm_ma;
	int				fg_ibatt_standby_ma;
	int				fg_thermistor_c1_coeff;
	int				fg_cc_to_cv_mv;
	int				fg_auto_recharge_soc;
	bool				empty_soc_disabled;
	bool				rsense_10mohm;
	bool				otg_fet_present;
	bool				fet_gain_enabled;
	int				otg_fet_enable_gpio;

	
	int				voltage_now;
	int				current_now;
	int				resistance_now;
	int				temp_now;
	int				soc_now;
	int				fcc_mah;
	bool				usb_present;
	bool				batt_present;
	bool				batt_hot;
	bool				batt_cold;
	bool				batt_warm;
	bool				batt_cool;
	bool				batt_full;
	bool				resume_completed;
	bool				irq_waiting;
	bool				empty_soc;
	int				workaround_flags;
	u8				irq_cfg_mask[3];
	int				usb_psy_ma;
	unsigned int			max_fastchg_ma;
	int				charging_disabled_status;
	u32				connected_rid;
	u32				profile_rid[BATTERY_PROFILE_MAX];

	u32				peek_poke_address;
	u32				fg_access_type;
	u32				fg_peek_poke_address;
	int				skip_writes;
	int				skip_reads;
	struct dentry			*debug_root;

	struct qpnp_vadc_chip		*vadc_dev;
	struct power_supply		*usb_psy;
	struct power_supply		batt_psy;
	struct smb1360_otg_regulator	otg_vreg;
	struct mutex			irq_complete;
	struct mutex			charging_disable_lock;
	struct mutex			current_change_lock;
	struct mutex			read_write_lock;

	int				batt_id;
	struct delayed_work		update_ovp_uvp_work;
	struct delayed_work		kick_wd_work;
	struct delayed_work		eoc_work;

	struct wake_lock		eoc_worker_lock;

	struct single_row_lut		*vth_sf_lut;
	struct single_row_lut		*rbatt_temp_lut;

	struct delayed_work		aicl_work;
	int				pm8916_psy_ma;
	bool				pm8916_enabled;
	int				combined_chgr_max_aicl_input_curr_limit;
	int				pm8916_max_input_curr_limit;
	bool				wa_to_re_calc_soc_in_low_temp;

	struct delayed_work		vin_collapse_check_work;
	struct wake_lock		vin_collapse_check_wake_lock;

	struct delayed_work		cable_impedance_work;
};

static int chg_time[] = {
	192,
	384,
	768,
	1536,
};

static int input_current_limit[] = {
	300, 400, 450, 500, 600, 700, 800, 850, 900,
	950, 1000, 1100, 1200, 1300, 1400, 1500,
};

static int fastchg_current[] = {
	450, 600, 750, 900, 1050, 1200, 1350, 1500,
};

#ifdef CONFIG_HTC_BATT_8960
static struct smb1360_chip *the_chip;
enum htc_power_source_type pwr_src;

static bool is_ac_safety_timeout = false;

static int hsml_target_ma;
static int usb_target_ma;
static bool is_batt_full = false;
static bool is_batt_full_eoc_stop = false;
static unsigned long wa_batt_full_eoc_stop_last_do_jiffies;
static unsigned long wa_batt_full_eoc_stop_time_accumulated_ms;
static int g_chg_error = 0;
static int eoc_count; 
static int eoc_count_by_curr; 
static int dischg_count; 
static int iusb_limit_reason;
static bool iusb_limit_enable = false;
static unsigned int chg_limit_current; 

static bool flag_keep_charge_on;
static bool flag_pa_recharge;
static bool flag_enable_fast_charge;
static bool flag_enable_bms_charger_log;

static int ovp = false;
static int uvp = false;

static bool gs_measure_cable_impedance = true;
static bool gs_stop_eoc = false;
static int gs_R_cable_impedance = 0;
static int gs_cable_impedance = 0;     
static int gs_aicl_result = 0;
static int gs_R_HW_MB_Impedance = 0;

static int smb1360_input_current_limit_set(struct smb1360_chip *chip, int mA);
int smb1360_chg_aicl_enable(struct smb1360_chip *chip, bool enable);

static int smb1360_read_bytes(struct smb1360_chip *chip, int reg, u8 *val, u8 bytes);
static int smb1360_masked_write(struct smb1360_chip *chip, int reg, u8 mask, u8 val);
static int smb1360_charging_disable(struct smb1360_chip *chip, int reason, int disable);
static int smb1360_get_prop_batt_capacity(struct smb1360_chip *chip);
static int smb1360_get_prop_batt_temp(struct smb1360_chip *chip);
static int smb1360_get_prop_voltage_now(struct smb1360_chip *chip);
static int smb1360_get_prop_current_now(struct smb1360_chip *chip);
static int smb1360_get_prop_charge_type(struct smb1360_chip *chip);
static int smb1360_get_prop_batt_health(struct smb1360_chip *chip);
static int smb1360_get_input_current_limit(struct smb1360_chip *chip);
static int smb1360_get_input_current_limit_status(struct smb1360_chip *chip);
static int smb1360_set_appropriate_usb_current(struct smb1360_chip *chip);
static int smb1360_chg_is_usb_chg_plugged_in(struct smb1360_chip *chip);
static int get_prop_usb_valid_status(struct smb1360_chip *chip,
					int *ov, int *v, int *uv);
static int smb1360_read(struct smb1360_chip *chip, int reg, u8 *val);
static int smb1360_get_usbin(struct smb1360_chip *chip);
static void dump_all(void);

static int smb1360_ext_charging_disable(struct smb1360_chip *chip, int disable);
#ifdef CONFIG_QPNP_LINEAR_CHARGER_SMB1360
extern int pm8916_charger_enable(bool enable, int chg_current);
#else
static int pm8916_charger_enable(bool enable, int chg_current)
{
	return -ENXIO;
}
#endif 


int htc_battery_is_support_qc20(void)
{
	
	return 0;
}
EXPORT_SYMBOL(htc_battery_is_support_qc20);

int htc_battery_check_cable_type_from_usb(void)
{
	
	return 0;
}
EXPORT_SYMBOL(htc_battery_check_cable_type_from_usb);

static u32 htc_fake_charger_for_testing(enum htc_power_source_type src)
{
	
	enum htc_power_source_type new_src = HTC_PWR_SOURCE_TYPE_AC;

	if((src > HTC_PWR_SOURCE_TYPE_9VAC) || (src == HTC_PWR_SOURCE_TYPE_BATT))
		return src;

	pr_info("(%d -> %d)\n", src, new_src);
	return new_src;
}

static void htc_start_eoc_work(struct smb1360_chip *chip)
{
	if (delayed_work_pending(&chip->eoc_work))
		cancel_delayed_work_sync(&chip->eoc_work);
	schedule_delayed_work(&chip->eoc_work,
			msecs_to_jiffies(SMB1360_EOC_CHECK_PERIOD_MS));
	wake_lock(&chip->eoc_worker_lock);
}

static void htc_reset_safety_timer(struct smb1360_chip *chip)
{
	int rc = 0;

	if (flag_keep_charge_on || flag_pa_recharge)
		return;

	
	rc = smb1360_masked_write(chip, CFG_SFY_TIMER_CTRL_REG,
				SAFETY_TIME_DISABLE_BIT, SAFETY_TIME_DISABLE_BIT);
	if (rc < 0) {
		pr_err("Couldn't disable safety timer rc = %d\n", rc);
		return;
	}

	
	rc = smb1360_masked_write(chip, CFG_SFY_TIMER_CTRL_REG,
				SAFETY_TIME_DISABLE_BIT, 0);
	if (rc < 0) {
		pr_err("Couldn't enable safety timer rc = %d\n", rc);
		return;
	}
}

static void handle_usb_present_change(struct smb1360_chip *chip,
				int usb_present)
{
	int rc;

	if (chip->usb_present ^ usb_present) {
		chip->usb_present = usb_present;
		is_ac_safety_timeout = false;
		if (!usb_present) {
			hsml_target_ma = 0;
			usb_target_ma = 0;
			
			eoc_count_by_curr = eoc_count = dischg_count = 0;
			is_batt_full = false;
			is_batt_full_eoc_stop = false;
			wa_batt_full_eoc_stop_last_do_jiffies = 0;
			wa_batt_full_eoc_stop_time_accumulated_ms = 0;
			chip->pm8916_psy_ma = 0;

			if (chip->pm8916_enabled) {
				
				rc = smb1360_chg_aicl_enable(the_chip, 1);
				if (rc < 0)
					pr_err("enable/disable AICL failed rc=%d\n", rc);
			}

			
			mutex_lock(&chip->charging_disable_lock);
			chip->charging_disabled_status &= ~USER;
			mutex_unlock(&chip->charging_disable_lock);
			if (delayed_work_pending(&the_chip->kick_wd_work))
				cancel_delayed_work_sync(&the_chip->kick_wd_work);

		} else {
			htc_start_eoc_work(chip);
			
			if (iusb_limit_enable)
				smb1360_limit_input_current(
					iusb_limit_enable, iusb_limit_reason);
		}
	}
}

static void update_ovp_uvp_state(int ov, int v, int uv)
{
	if ( ov && !v && !uv) {
		if (!ovp) {
			ovp = 1;
			pr_info("OVP: 0 -> 1, USB_Valid: %d\n", v);
			htc_charger_event_notify(HTC_CHARGER_EVENT_OVP);
		}
		if (uvp) {
			uvp = 0;
			pr_info("UVP: 1 -> 0, USB_Valid: %d\n", v);
		}
	} else if ( !ov && !v && uv) {
		if (ovp) {
			ovp = 0;
			pr_info("OVP: 1 -> 0, USB_Valid: %d\n", v);
			htc_charger_event_notify(HTC_CHARGER_EVENT_OVP_RESOLVE);
		}
		if (!uvp) {
			uvp = 1;
			pr_info("UVP: 0 -> 1, USB_Valid: %d\n", v);
		}
	} else {
		if (ovp) {
			ovp = 0;
			pr_info("OVP: 1 -> 0, USB_Valid: %d\n", v);
			htc_charger_event_notify(HTC_CHARGER_EVENT_OVP_RESOLVE);
		}
		if (uvp) {
			uvp = 0;
			pr_info("UVP: 1 -> 0, USB_Valid: %d\n", v);
		}
	}
	pr_debug("ovp=%d, uvp=%d [%d,%d,%d]\n", ovp, uvp, ov, v, uv);
}

int smb1360_is_pwr_src_plugged_in(void)
{
	int ov = false, uv = false, v = false;
	int rc = 0;
	static bool first = true;
	u8 reg = 0;

	
	if (!the_chip) {
		pr_warn("called before init\n");
		return -EINVAL;
	}

	if (first) {
		rc = smb1360_read(the_chip, IRQ_F_REG, &reg);
		if (rc < 0) {
			pr_err("Couldn't read irq F rc = %d\n", rc);
			return rc;
		}
		v = (reg & IRQ_F_POWER_OK_BIT) ? true : false;
	} else {
		get_prop_usb_valid_status(the_chip, &ov, &v, &uv);
	}
	first = false;

	return v;
}

int smb1360_get_batt_temperature(int *result)
{
	if (!the_chip) {
		pr_warn("called before init\n");
		return -EINVAL;
	}

	*result = smb1360_get_prop_batt_temp(the_chip);

	if ((*result >= 650) &&
			(flag_keep_charge_on || flag_pa_recharge))
			*result = 650;

	return 0;
}
int smb1360_get_batt_voltage(int *result)
{
	if (!the_chip) {
		pr_warn("called before init\n");
		return -EINVAL;
	}

	*result = smb1360_get_prop_voltage_now(the_chip);
	return 0;
}
int smb1360_get_batt_current(int *result)
{
	if (!the_chip) {
		pr_warn("called before init\n");
		return -EINVAL;
	}

	*result = smb1360_get_prop_current_now(the_chip);
	return 0;
}

static int smb1360_re_calculate_soc_in_low_temperature(struct smb1360_chip *chip, int soc)
{
	int batt_temp, vbat_mv, ibat_ma, rbatt_mohm;
	int vth, soc_new;

	batt_temp = smb1360_get_prop_batt_temp(chip);

	if (!chip->vth_sf_lut || !chip->rbatt_temp_lut) {
		pr_info("Vth sf lut or rbatt_temp_lut is not ready\n");
		return soc;
	}

	if (batt_temp >= 0) {
		pr_debug("battery temperature (%d)>0, do nothing\n", batt_temp / 10);
		return soc;
	}

	if (soc == 0) {
		pr_debug("soc=0, do nothing\n");
		return soc;
	}

	vbat_mv = smb1360_get_prop_voltage_now(chip);
	ibat_ma = smb1360_get_prop_current_now(chip) / 1000;
	rbatt_mohm = interpolate_rbatt_temp(chip->rbatt_temp_lut, batt_temp / 10);

	
	vth = vbat_mv + ((ibat_ma - 520) * rbatt_mohm) / 1000; 
	soc_new = interpolate_vth(chip->vth_sf_lut, vth);

	pr_info("vbat=%dmV, ibat=%dmA, batt_temp=%ddegC, rbatt=%dmohm, vth=%dmV, soc_org=%d, soc_new=%d\n",
			vbat_mv, ibat_ma, batt_temp / 10, rbatt_mohm, vth, soc, soc_new);

	return soc_new;
}

static int smb1360_clamp_soc_based_on_voltage(struct smb1360_chip *chip, int soc)
{
	int vbat_mv, batt_temp;

	vbat_mv = smb1360_get_prop_voltage_now(chip);
	batt_temp = smb1360_get_prop_batt_temp(chip);

	if (soc == 0)
		pr_info("batt_vol = %d, batt_temp = %d\n", vbat_mv, batt_temp);

	if ((chip->clamp_soc_voltage_mv != -EINVAL) && soc == 0 &&
			vbat_mv > chip->clamp_soc_voltage_mv && batt_temp > 0) {
		pr_debug("clamping soc to 1, temp = %d, vbat (%d) > cutoff (%d)\n",
						batt_temp, vbat_mv, chip->clamp_soc_voltage_mv);
		return 1;
	} else {
		pr_debug("not clamping, using soc = %d, vbat = %d and cutoff = %d\n",
				soc, vbat_mv, chip->clamp_soc_voltage_mv);
		return soc;
	}
}

int smb1360_get_batt_soc(int *result)
{
	if (!the_chip) {
		pr_warn("called before init\n");
		return -EINVAL;
	}

	*result = smb1360_get_prop_batt_capacity(the_chip);
	return 0;
}
int smb1360_get_batt_cc(int *result)
{
	
	return 0;
}
int smb1360_get_batt_present(void)
{
	return -ENXIO;
}
int smb1360_get_batt_status(void)
{
	return -ENXIO;
}
int smb1360_is_batt_temp_fault_disable_chg(int *result)
{
	int vbat_mv, is_vbatt_over_vddmax;
	int is_cold = 0, is_hot = 0;
	int is_warm = 0, is_cool = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	vbat_mv = smb1360_get_prop_voltage_now(the_chip);

	is_hot = the_chip->batt_hot;
	is_cold = the_chip->batt_cold;

	is_warm = the_chip->batt_warm;
	is_cool = the_chip->batt_cool;

	if(vbat_mv >= the_chip->warm_bat_mv)
		is_vbatt_over_vddmax = true;
	else
		is_vbatt_over_vddmax = false;

	pr_debug("is_cold=%d, is_hot=%d, is_warm=%d, is_vbatt_over_vddmax=%d, warm_bat_mv:%d\n",
			is_cold, is_hot, is_warm, is_vbatt_over_vddmax, the_chip->warm_bat_mv);
	if ((is_cold || is_cool || is_hot || (is_warm && is_vbatt_over_vddmax)) &&
			!flag_keep_charge_on && !flag_pa_recharge)
		*result = 1;
	else
		*result = 0;

	return 0;
}
int smb1360_vm_bms_report_eoc(void)
{
	return -ENXIO;
}
int smb1360_is_batt_temperature_fault(int *result)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	pr_debug("is_cool=%d,is_warm=%d\n", the_chip->batt_cool, the_chip->batt_warm);
	if (the_chip->batt_cool || the_chip->batt_warm)
		*result = 1;
	else
		*result = 0;
	return 0;
}
int smb1360_set_pwrsrc_and_charger_enable(enum htc_power_source_type src,
			bool chg_enable, bool pwrsrc_enable)
{
	int rc;
	int current_ma = 0;
	static int pre_pwr_src;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	pr_info("src=%d, pre_pwr_src=%d, chg_enable=%d, pwrsrc_enable=%d\n",
				src, pre_pwr_src, chg_enable, pwrsrc_enable);

	if (src == 2 && gs_R_HW_MB_Impedance != -EINVAL) {
		if (gs_measure_cable_impedance == true) {
			gs_measure_cable_impedance = false;
			gs_stop_eoc = true;
			schedule_delayed_work(&the_chip->cable_impedance_work,
				msecs_to_jiffies(10000));
		}
	} else {
		gs_measure_cable_impedance = true;
		gs_cable_impedance = 0;
		gs_R_cable_impedance = 0;
		gs_aicl_result = 0;
		gs_stop_eoc = false;
		if (delayed_work_pending(&the_chip->cable_impedance_work))
			cancel_delayed_work_sync(&the_chip->cable_impedance_work);
	}

	if (flag_enable_fast_charge)
		src = htc_fake_charger_for_testing(src);

	pwr_src = src;

	if(src > HTC_PWR_SOURCE_TYPE_BATT)
		the_chip->charging_disabled = false;
	else
		the_chip->charging_disabled = true;

	switch (src) {
	case HTC_PWR_SOURCE_TYPE_BATT:
		current_ma = USB_MA_2;
		break;
	case HTC_PWR_SOURCE_TYPE_WIRELESS:
	case HTC_PWR_SOURCE_TYPE_DETECTING:
	case HTC_PWR_SOURCE_TYPE_UNKNOWN_USB:
	case HTC_PWR_SOURCE_TYPE_USB:
		current_ma = USB_MA_500;
		break;
	case HTC_PWR_SOURCE_TYPE_AC:
	case HTC_PWR_SOURCE_TYPE_9VAC:
	case HTC_PWR_SOURCE_TYPE_MHL_AC:
		current_ma = USB_MA_1000;
		break;
	default:
		current_ma = USB_MA_2;
		break;
	}

	the_chip->usb_psy_ma = current_ma;


	mutex_lock(&the_chip->current_change_lock);
	rc = smb1360_set_appropriate_usb_current(the_chip);
	if (rc)
		pr_err("Couldn't set usb current rc = %d\n", rc);
	mutex_unlock(&the_chip->current_change_lock);

	if (HTC_PWR_SOURCE_TYPE_BATT == src)
		handle_usb_present_change(the_chip, 0);
	else
		handle_usb_present_change(the_chip, 1);

	
	if (src != HTC_PWR_SOURCE_TYPE_BATT && !chg_enable)
		smb1360_charger_enable(0);

	return 0;
}
int smb1360_charger_enable(bool enable)
{
	int rc;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	
	
	
	rc = smb1360_masked_write(the_chip, CFG_WD_CTRL,
					CFG_WD_CTRL_BIT,
					enable ? 0 : CFG_WD_CTRL_BIT);
	if (rc < 0)
		dev_err(the_chip->dev, "Couldn't set CHGR_WD_CTRL = %d\n", rc);

	
	if (!enable)
		schedule_delayed_work(&the_chip->kick_wd_work, 0);
	else {
		if (delayed_work_pending(&the_chip->kick_wd_work))
			cancel_delayed_work_sync(&the_chip->kick_wd_work);
	}
	

	g_chg_error = 1;
	rc = smb1360_charging_disable(the_chip, USER, !enable);
	if (rc < 0)
		pr_err("Unable to disable charging rc=%d\n", rc);
	g_chg_error = 0;

	if (enable) {
		mutex_lock(&the_chip->current_change_lock);
		
		rc = smb1360_set_appropriate_usb_current(the_chip);
		if (rc)
			pr_err("Couldn't set USB current rc = %d\n", rc);
		mutex_unlock(&the_chip->current_change_lock);

		htc_start_eoc_work(the_chip);
	}


	return rc;
}
int smb1360_is_charger_ovp(int* result)
{
	int ov = false, uv = false, v = false;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	get_prop_usb_valid_status(the_chip, &ov, &v, &uv);
	update_ovp_uvp_state(ov, v, uv);

	*result = ovp;

	return 0;
}
int smb1360_pwrsrc_enable(bool enable)
{
	return -ENXIO;
}
int smb1360_get_battery_status(void)
{
	return -ENXIO;
}
int smb1360_get_charging_source(int *result)
{
	return -ENXIO;
}
int smb1360_get_charging_enabled(int *result)
{
	return -ENXIO;
}
int smb1360_get_charge_type(void)
{
	if (!the_chip) {
		pr_err("%s: called before init\n", __func__);
		return -EINVAL;
	}
	return  smb1360_get_prop_charge_type(the_chip);
}
int smb1360_get_chg_usb_iusbmax(void)
{
	if (!the_chip) {
		pr_err("%s: called before init\n", __func__);
		return -EINVAL;
	}
	return smb1360_get_input_current_limit(the_chip);
}
int smb1360_get_chg_vinmin(void)
{
	
	return 0;
}
int smb1360_get_input_voltage_regulation(void)
{
	
	return 0;
}
int smb1360_set_chg_vin_min(int val)
{
	return -ENXIO;
}
int smb1360_set_chg_iusbmax(int val)
{
	return -ENXIO;
}
int smb1360_set_hsml_target_ma(int target_ma)
{
	return -ENXIO;
}
int smb1360_dump_all(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	dump_all();

	return 0;
}
int smb1360_is_batt_full(int *result)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	*result = is_batt_full;
	return 0;
}
int smb1360_is_batt_full_eoc_stop(int *result)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	*result = is_batt_full_eoc_stop;
	return 0;
}
int smb1360_charger_get_attr_text(char *buf, int size)
{
	int len = 0;
	int aicl_result_now;

	aicl_result_now = smb1360_get_input_current_limit_status(the_chip);

	
	len += scnprintf(buf + len, size -len,
			"gs_R_HW_MB_Impedance: %d\n"
			"gs_cable_impedance: %d\n"
			"gs_R_cable_impedance: %d\n"
			"gs_aicl_result: %d\n"
			"gs_aicl_result_now: %d\n",
			gs_R_HW_MB_Impedance,
			gs_cable_impedance,
			gs_R_cable_impedance,
			gs_aicl_result,
			aicl_result_now);

	return len;
}
int smb1360_fake_usbin_valid_irq_handler(void)
{
	return -ENXIO;
}
int smb1360_fake_coarse_det_usb_irq_handler(void)
{
	return -ENXIO;
}
int smb1360_is_chg_safety_timer_timeout(int *result)
{
	*result = is_ac_safety_timeout;
	return 0;
}
int smb1360_get_usb_temperature(int *result)
{
	return -ENXIO;
}

static int is_ac_online(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return smb1360_chg_is_usb_chg_plugged_in(the_chip) &&
				(pwr_src == HTC_PWR_SOURCE_TYPE_AC ||
							pwr_src == HTC_PWR_SOURCE_TYPE_9VAC ||
							pwr_src == HTC_PWR_SOURCE_TYPE_MHL_AC);
}

#define KICK_CHG_WD_PERIOD_MS	10000
static void smb1360_kick_wd_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct smb1360_chip *chip = container_of(dwork,
				struct smb1360_chip, kick_wd_work);
	int batt_level;

	pm_stay_awake(chip->dev);
	if(smb1360_chg_is_usb_chg_plugged_in(chip)) {
		smb1360_get_batt_soc(&batt_level);
		schedule_delayed_work(&chip->kick_wd_work,
			msecs_to_jiffies(KICK_CHG_WD_PERIOD_MS));
	} else {
		pr_info("not charging, stop to kick watchdog\n");
		goto stop_kick_wd;
	}

	return;

stop_kick_wd:
	pm_relax(chip->dev);
}

static void smb1360_update_ovp_uvp_work(struct work_struct *work)
{
	int ov = false, uv = false, v = false;

	if (!the_chip) {
		pr_err("called before init\n");
		return;
	}

	get_prop_usb_valid_status(the_chip, &ov, &v, &uv);
	update_ovp_uvp_state(ov, v, uv);
}

int smb1360_get_batt_id(int *result)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	*result = the_chip->batt_id;
	return 0;
}

static int smb1360_is_fastchg_on(struct smb1360_chip *chip)
{
	int rc;
	u8 reg = 0;

	rc = smb1360_read(chip, STATUS_3_REG, &reg);
	if (rc) {
		pr_err("Couldn't read STATUS_3_REG rc=%d\n", rc);
		return 0;
	}

	return (reg & CHG_FASTCHG_BIT) ? 1 : 0;
}

static int smb1360_chg_is_taper(struct smb1360_chip *chip)
{
	int rc;
	u8 reg = 0, chg_type;

	rc = smb1360_read(chip, STATUS_3_REG, &reg);
	if (rc) {
		pr_err("Couldn't read STATUS_3_REG rc=%d\n", rc);
		return 0;
	}

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;

	if (chg_type == BATT_TAPER_CHG_VAL)
		return true;
	else
		return false;
}

#define CONSECUTIVE_COUNT	3
#define CLEAR_FULL_STATE_BY_LEVEL_THR	90
#define WA_BATT_FULL_EOC_STOP_ACCUM_TIME_MS	(1000 * 60 * 60)
#define SMB1360_LOW_CURRENT_TOLERANCE	10
static void
smb1360_eoc_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct smb1360_chip *chip = container_of(dwork,
				struct smb1360_chip, eoc_work);
	int ibat_ma, vbat_mv, soc = 0;
	u8 chg_state = 0;
	int rc = 0;

	if (gs_stop_eoc == true)
		goto end;

	wake_lock(&chip->eoc_worker_lock);
	if (!smb1360_chg_is_usb_chg_plugged_in(chip)){
		pr_info("no chg connected, stopping\n");
		is_batt_full = false;
		is_batt_full_eoc_stop = false;
		goto stop_eoc;
	}

	if (smb1360_is_fastchg_on(chip)) {
		smb1360_read(chip, STATUS_3_REG, &chg_state);
		ibat_ma = smb1360_get_prop_current_now(chip)/1000;
		vbat_mv = smb1360_get_prop_voltage_now(chip);

		pr_info("ibat_ma=%d, vbat_mv=%d, batt_full_cri=%d, eoc_ma=%d, "
				"eoc_count=%d, eoc_count_by_curr=%d, dischg_count=%d, "
				"chg_state=0x%X, fastchg=%d, chgtaper=%d\n",
				ibat_ma, vbat_mv, chip->batt_full_criteria, chip->iterm_ma,
				eoc_count, eoc_count_by_curr, dischg_count,
				chg_state, smb1360_is_fastchg_on(chip), smb1360_chg_is_taper(chip));

		if (!smb1360_chg_is_taper(chip)) {
			
			eoc_count_by_curr = eoc_count = dischg_count = 0;
		} else if (ibat_ma > (0 + SMB1360_LOW_CURRENT_TOLERANCE)) {
			
			dischg_count++;
			if (dischg_count >= CONSECUTIVE_COUNT)
				eoc_count_by_curr = eoc_count = 0;
		} else if ((ibat_ma * -1) > chip->batt_full_criteria) {
			
			eoc_count_by_curr = eoc_count = dischg_count = 0;
		} else if ((ibat_ma * -1) > chip->batt_eoc_criteria) {
			
			eoc_count_by_curr = dischg_count = 0;
			eoc_count++;

			if (eoc_count == CONSECUTIVE_COUNT) {
				is_batt_full = true;
				htc_gauge_event_notify(HTC_GAUGE_EVENT_EOC);
			}
		} else {
			dischg_count = 0;
			eoc_count++;
			
			if (eoc_count_by_curr == CONSECUTIVE_COUNT) {
				pr_info("End of Charging\n");
				is_batt_full_eoc_stop = true;
				wa_batt_full_eoc_stop_last_do_jiffies = jiffies;
				eoc_count_by_curr += 1;
				htc_reset_safety_timer(chip);
			} else {
				if (eoc_count == CONSECUTIVE_COUNT && !is_batt_full) {
					is_batt_full = true;
					htc_gauge_event_notify(HTC_GAUGE_EVENT_EOC);
				}
				eoc_count_by_curr += 1;
			}

			if (is_batt_full_eoc_stop == true
					&& eoc_count_by_curr >= CONSECUTIVE_COUNT
					&& wa_batt_full_eoc_stop_last_do_jiffies != 0) {
				unsigned long cur_jiffies;

				cur_jiffies = jiffies;
				wa_batt_full_eoc_stop_time_accumulated_ms += (cur_jiffies - wa_batt_full_eoc_stop_last_do_jiffies) * MSEC_PER_SEC / HZ;
				wa_batt_full_eoc_stop_last_do_jiffies = cur_jiffies;
				pr_info("wa_batt_full_eoc_stop_time_accumulated_ms=%lu\n", wa_batt_full_eoc_stop_time_accumulated_ms);

				if (wa_batt_full_eoc_stop_time_accumulated_ms >= WA_BATT_FULL_EOC_STOP_ACCUM_TIME_MS) {
					eoc_count_by_curr = eoc_count = 0;
					
					

					
					g_chg_error = 1;
					rc = smb1360_charging_disable(chip, USER, true);
					if (rc < 0)
						pr_err("Unable to disable charging rc=%d\n", rc);
					g_chg_error = 0;

					rc = smb1360_charging_disable(chip, USER, false);
					if (rc < 0)
						pr_err("Unable to enable charging rc=%d\n", rc);
				}
			}
		}

		if (is_batt_full) {
			smb1360_get_batt_soc(&soc);
			if (soc < CLEAR_FULL_STATE_BY_LEVEL_THR) {
				
				if (vbat_mv > (chip->vfloat_mv - 100)) {
					pr_info("Not satisfy overloading battery voltage"
						" critiria (%dmV < %dmV).\n", vbat_mv,
						(chip->vfloat_mv - 100));
				} else {
					is_batt_full = false;
					eoc_count = eoc_count_by_curr = 0;
					pr_info("%s: Clear is_batt_full & eoc_count due to"
						" Overloading happened, soc=%d\n",
						__func__, soc);
					htc_gauge_event_notify(HTC_GAUGE_EVENT_EOC);
				}
			}
		}

		if (eoc_count_by_curr == 0) {
			is_batt_full_eoc_stop = false;
			wa_batt_full_eoc_stop_last_do_jiffies = 0;
			wa_batt_full_eoc_stop_time_accumulated_ms = 0;
		}
	} else {
		pr_info("not charging\n");
			goto stop_eoc;
	}

end:
	schedule_delayed_work(&chip->eoc_work,
		msecs_to_jiffies(SMB1360_EOC_CHECK_PERIOD_MS));
	return;

stop_eoc:
	eoc_count_by_curr = eoc_count = dischg_count = 0;
	wa_batt_full_eoc_stop_last_do_jiffies = 0;
	wa_batt_full_eoc_stop_time_accumulated_ms = 0;
	wake_unlock(&chip->eoc_worker_lock);

}

#define VUSBIN_SAMPLE_COUNTS	5
static int usbin[VUSBIN_SAMPLE_COUNTS];
static int usbin_sample_counts;
static int vbus_uv[8];
static int vbus_diff[7];
static bool vbus_drop_sharply;
static bool is_vin_collapse;
static bool is_aicl_ongoing;

static void smb1360_clean_aicl_flag(struct smb1360_chip *chip)
{
	if (chip->pm8916_enabled) {
		is_vin_collapse = FALSE;
		vbus_drop_sharply = FALSE;
		is_aicl_ongoing = FALSE;
		usbin_sample_counts = 0;
		memset(usbin, 0, sizeof(usbin));
	}
}

static void
smb1360_aicl_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct smb1360_chip *chip = container_of(dwork,
				struct smb1360_chip, aicl_work);
	static u8 aiclval[2];
	int ibat_ma, vbat_mv, usbin_uv;
	bool disable_pm8916_chg = FALSE;
	bool is_charging_resumed = FALSE;
	int cnt = 0, vbus_drop_scale;
	int max_aicl_input_curr_limit;
	int is_temp_fault_dis_chg;
	int rc = 0;
	int i = 0, j = 0;

	if (gs_stop_eoc == true) {
		schedule_delayed_work(&chip->aicl_work, msecs_to_jiffies(SMB1360_AICL_CHECK_PERIOD_MS));
		pr_info("waiting due to cable impedance worker is running\n");
		return;
	}

	max_aicl_input_curr_limit = (chip->combined_chgr_max_aicl_input_curr_limit == -EINVAL) ?
			COMBINED_CHGR_MAX_AICL_INPUT_CURR_LIMIT : chip->combined_chgr_max_aicl_input_curr_limit;

	vbat_mv = smb1360_get_prop_voltage_now(chip);

	
	if (is_aicl_ongoing == FALSE
			&& chip->pm8916_psy_ma == chip->pm8916_max_input_curr_limit) {
		is_charging_resumed = TRUE;
		goto pm8916_aicl_finished;
	}

	if (is_batt_full_eoc_stop) {
		pr_info("charging is in EOC mode\n");
		disable_pm8916_chg = TRUE;
		goto disable_pm8916_chg;
	}

	
	if (vbat_mv >= 4200 && smb1360_chg_is_taper(chip)) {
		pr_info("charging is in CC mode, vbat=%dmV\n", vbat_mv);
		disable_pm8916_chg = TRUE;
		goto disable_pm8916_chg;
	}

	
	if (is_vin_collapse) {
		pr_info("is_vin_collapse was triggered");
		is_vin_collapse = FALSE;
		disable_pm8916_chg = TRUE;
		goto disable_pm8916_chg;
	}

	
	smb1360_is_batt_temp_fault_disable_chg(&is_temp_fault_dis_chg);
	if (is_temp_fault_dis_chg) {
		pr_info("temperature is fault and disable to charge\n");
		disable_pm8916_chg = TRUE;
		goto disable_pm8916_chg;
	}

	memset(aiclval, 0, sizeof(aiclval));
	smb1360_read_bytes(chip, 0x48, aiclval, sizeof(aiclval));


	ibat_ma = smb1360_get_prop_current_now(chip)/1000;
	usbin_uv = smb1360_get_usbin(chip);
	usbin_sample_counts++;
	pr_info("usbin_mv=%d, ibat_ma=%d, usbin_sample_counts=%d\n", usbin_uv / 1000, ibat_ma, usbin_sample_counts);

	pr_info("usbin[%d, %d, %d, %d, %d]\n",
					usbin[0], usbin[1], usbin[2], usbin[3], usbin[4]);
	for (i = 0; i < VUSBIN_SAMPLE_COUNTS; i++) {
		if (usbin[i] == 0) {
			usbin[i] = usbin_uv;
			break;
		} else {
			if (usbin[i] >= usbin_uv)
				continue;
			else {
				for (j = (usbin_sample_counts - 1); j > i; j--)
					usbin[j] = usbin[j - 1];
				usbin[i] = usbin_uv;
				break;
			}
		}
	}

	pr_info("usbin[%d, %d, %d, %d, %d]\n",
					usbin[0], usbin[1], usbin[2], usbin[3], usbin[4]);

	if (usbin_sample_counts < VUSBIN_SAMPLE_COUNTS) {
		schedule_delayed_work(&chip->aicl_work, msecs_to_jiffies(SMB1360_AICL_CHECK_PERIOD_MS));
		return;
	} else {
		usbin_sample_counts = 0;
		usbin_uv = (usbin[1] + usbin[2] + usbin[3]) / 3;
		memset(usbin, 0, sizeof(usbin));
	}

	pr_info("aicl_ma=%d, vbat_mv=%d, ibat_ma=%d, usbin_mv=%d, [48h, 49h]=[0x%02X, 0x%02X], pm8916_ma=%d,"
		" max_aicl_input_curr_limit=%d, pm8916_max_input_curr_limit=%d\n",
			chip->usb_psy_ma, vbat_mv, ibat_ma, usbin_uv / 1000, aiclval[0], aiclval[1], chip->pm8916_psy_ma,
			max_aicl_input_curr_limit, chip->pm8916_max_input_curr_limit);

	
	

	switch (pwr_src) {
	case HTC_PWR_SOURCE_TYPE_AC:
	case HTC_PWR_SOURCE_TYPE_9VAC:
	case HTC_PWR_SOURCE_TYPE_MHL_AC:
		switch (chip->pm8916_psy_ma) {
		case 0:
			is_aicl_ongoing = TRUE;
		case 100:
		case 200:
		case 300:
		case 400:
		case 500:
		case 600:
		case 700:
			cnt = chip->pm8916_psy_ma / 100;
			vbus_uv[cnt] = usbin_uv;

			if (cnt != 0)
				vbus_diff[cnt - 1] = vbus_uv[cnt - 1] - vbus_uv[cnt];

			pr_info("vbus_uv[%d, %d, %d, %d, %d, %d, %d, %d]\n",
				vbus_uv[0], vbus_uv[1], vbus_uv[2], vbus_uv[3], vbus_uv[4], vbus_uv[5], vbus_uv[6], vbus_uv[7]);
			pr_info("vbus_diff[%d, %d, %d, %d, %d, %d, %d]\n",
				vbus_diff[0], vbus_diff[1], vbus_diff[2], vbus_diff[3], vbus_diff[4], vbus_diff[5], vbus_diff[6]);

			if ((cnt - 2) > 0) {
				vbus_drop_scale = vbus_diff[cnt - 1] * 100 / vbus_diff[1];
				pr_info("vbus_drop_scale = %d\n", vbus_drop_scale);

				if ((vbus_drop_sharply == TRUE && vbus_drop_scale < VBUS_DROP_SCALE_LOWER)
						|| vbus_drop_scale > VBUS_DROP_SCALE_UPPER) {
					disable_pm8916_chg = TRUE;
					goto disable_pm8916_chg;
				}

				if (vbus_drop_sharply == FALSE && vbus_drop_scale < VBUS_DROP_SCALE_LOWER)
					vbus_drop_sharply = TRUE;
			}
			break;
		default:
			pr_err("pm8916_psy_ma is out of range\n");
			break;
		}

		
		rc = smb1360_chg_aicl_enable(chip, 0);
		if (rc < 0)
			pr_err("enable/disable AICL failed rc=%d\n", rc);

		if (chip->usb_psy_ma + chip->pm8916_psy_ma != max_aicl_input_curr_limit) {
			chip->pm8916_psy_ma += 100;
			rc = pm8916_charger_enable(1, chip->pm8916_psy_ma);
			if (rc)
				pr_err("Couldn't set pm8916 current rc = %d\n", rc);
			schedule_delayed_work(&chip->aicl_work, msecs_to_jiffies(SMB1360_AICL_CHECK_PERIOD_MS));

			return;
		}
		break;
	default:
		break;
	}

disable_pm8916_chg:
	if (disable_pm8916_chg) {
		pr_info("disable_pm8916_chg\n");
		chip->pm8916_psy_ma = 0;
		pm8916_charger_enable(0, chip->pm8916_psy_ma);

		
		rc = smb1360_chg_aicl_enable(chip, 1);
		if (rc < 0)
			pr_err("enable/disable AICL failed rc=%d\n", rc);
	}

pm8916_aicl_finished:
	smb1360_clean_aicl_flag(chip);
	memset(vbus_uv, 0, sizeof(vbus_uv));
	memset(vbus_diff, 0, sizeof(vbus_diff));
	pr_info("finish\n");

	
	if (chip->usb_psy_ma + chip->pm8916_psy_ma == max_aicl_input_curr_limit) {
		chip->pm8916_psy_ma = chip->pm8916_max_input_curr_limit;
		rc = pm8916_charger_enable(1, chip->pm8916_psy_ma);
		if (rc)
			pr_err("Couldn't set pm8916 current rc = %d\n", rc);
	}

	if (is_charging_resumed) {
		pr_info("charging is resumed!\n");
		
		rc = smb1360_chg_aicl_enable(chip, 0);
		if (rc < 0)
			pr_err("enable/disable AICL failed rc=%d\n", rc);

		rc = pm8916_charger_enable(1, chip->pm8916_psy_ma);
		if (rc)
			pr_err("Couldn't set pm8916 current rc = %d\n", rc);
	}
}

static int
smb1360_fastchg_current_set(struct smb1360_chip *chip, int mA)
{
	int rc = 0, i;

	for (i = ARRAY_SIZE(fastchg_current) - 1; i >= 0; i--) {
		if (fastchg_current[i] <= mA)
			break;
	}
	if (i < 0) {
		pr_warn("Couldn't find fastchg mA=%d, i=%d\n", mA, i);
		i = 0;
	}
	
	rc = smb1360_masked_write(chip, CHG_CURRENT_REG,
		FASTCHG_CURR_MASK, i << FASTCHG_CURR_SHIFT);
	if (rc) {
		pr_err("Couldn't set fastchg mA rc=%d\n", rc);
		return rc;
	}
	rc = smb1360_masked_write(chip, CMD_IL_REG,
			USB_CTRL_MASK, USB_500_BIT);
	if (rc)
		pr_err("Couldn't configure for USB500 rc=%d\n", rc);

	rc = smb1360_masked_write(chip, CMD_IL_REG,
			USB_CTRL_MASK, USB_AC_BIT);
	if (rc) {
		pr_err("Couldn't configure for USB AC rc=%d\n", rc);
		return rc;
	}
	pr_info("fast-chg current set to = %d\n", fastchg_current[i]);
	return 0;
}

static void
qpnp_chg_set_appropriate_battery_current(struct smb1360_chip *chip)
{
	unsigned int chg_current = chip->max_fastchg_ma;


	if (chg_limit_current != 0)
		chg_current = min(chg_current, chg_limit_current);

	pr_info("setting: %dmA\n", chg_current);
	smb1360_fastchg_current_set(chip, chg_current);
}

int smb1360_limit_charge_enable(bool enable)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	pr_debug("limit_charge=%d\n", enable);

	if (enable)
		chg_limit_current = SMB1360_CHG_I_MIN_MA;
	else
		chg_limit_current = 0;

	qpnp_chg_set_appropriate_battery_current(the_chip);

	smb1360_ext_charging_disable(the_chip, enable);

	return 0;
}

static int
smb1360_input_current_limit_set(struct smb1360_chip *chip, int mA)
{
	int rc = 0, i;

	for (i = ARRAY_SIZE(input_current_limit) - 1; i >= 0; i--) {
		if (input_current_limit[i] <= mA)
			break;
	}
	if (i < 0) {
		pr_warn("Couldn't find ICL mA rc=%d\n", rc);
		i = 0;
	}
	
	rc = smb1360_masked_write(chip, CFG_BATT_CHG_ICL_REG,
					INPUT_CURR_LIM_MASK, i);
	if (rc < 0) {
		pr_err("Couldn't set ICL mA rc=%d\n", rc);
		return rc;
	}

	pr_info("Input current limit set to = %dmA\n", input_current_limit[i]);
	return 0;
}

int smb1360_chg_aicl_enable(struct smb1360_chip *chip, bool enable)
{
	int rc;
	u8 reg;

	pr_debug("charger AICL enable: %d\n", enable);
	if (enable)
		reg = AICL_ENABLED_BIT;
	else
		reg = 0x00;
	
	rc = smb1360_masked_write(chip, CFG_GLITCH_FLT_REG, AICL_ENABLED_BIT, reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set AICL_ENABLED_BIT rc=%d\n",
				rc);
		return rc;
	}
	return 0;
}

int smb1360_limit_input_current(bool enable, int reason)
{
	int rc = 0;
	int limit_intput_current = 0;
	iusb_limit_enable = enable;
	iusb_limit_reason = reason;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	pr_debug("enable=%d, usb_psy_ma=%dmA\n", enable, the_chip->usb_psy_ma);

	if(the_chip->usb_psy_ma && is_ac_online()) {
		limit_intput_current = the_chip->usb_psy_ma;

		
		smb1360_ext_charging_disable(the_chip, true);

		
		rc = smb1360_chg_aicl_enable(the_chip, 0);
		if (rc < 0) {
			pr_err("enable/disable AICL failed rc=%d\n", rc);
			return rc;
		}

		if (enable) {
			
			if (iusb_limit_reason & HTC_BATT_CHG_LIMIT_BIT_THRML)
				limit_intput_current =
					min(limit_intput_current, SMB1360_INPUT_LIMIT_MA);
		}

		pr_info("Set input current limit to %dmA due to reason=0x%X, "
				"usb_psy_ma=%dmA\n",
				limit_intput_current, iusb_limit_reason,
				the_chip->usb_psy_ma);

		rc = smb1360_input_current_limit_set(the_chip, limit_intput_current);
		if (rc < 0) {
			pr_err("set ICL fail rc=%d\n", rc);
			return rc;
		}
		
		rc = smb1360_chg_aicl_enable(the_chip, 1);
		if (rc < 0) {
			pr_err("enable/disable AICL failed rc=%d\n", rc);
			return rc;
		}

		
		smb1360_ext_charging_disable(the_chip, false);
	}
	return 0;
}

#define AICL_CHECK_RAMP_MS 	(1000)
static void vin_collapse_check_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct smb1360_chip *chip = container_of(dwork,
			struct smb1360_chip, vin_collapse_check_work);
	int rc = 0;

	if (is_ac_online()) {
		is_vin_collapse = TRUE;
		mutex_lock(&chip->current_change_lock);
		
		rc = smb1360_set_appropriate_usb_current(chip);
		if (rc)
			pr_err("Couldn't set USB current rc = %d\n", rc);
		mutex_unlock(&chip->current_change_lock);
	} else {
		
		cable_detection_vbus_irq_handler();
	}
}

int smb1360_is_bad_cable_used(int *result)
{
	*result = 0;

	if(!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (gs_cable_impedance == 3)
		*result = 1;

	return 0;
}

#define SOC_LEVEL_AT_4200MV	85
static void
cable_impedance_worker(struct work_struct *work)
{
	static int vusbin_mv_1, vusbin_mv_2, vusbin_mv_3, aicl_result;
	int vusbin1, vusbin2, vusbin3;
	int iusb1, iusb2, iusb3;
	int rRes1, rRes2;
	int icl_cfg_ma;
	int rc = 0;

	if (!the_chip) {
		pr_err("[Cable impedance]called before the_chip initialized\n");
		gs_stop_eoc = false;
		return;
	}

	if (gs_R_HW_MB_Impedance == -EINVAL) {
		pr_info("[Cable impedance]gs_R_HW_MB_Impedance missing, ignore detection.");
		goto endWorker;
	}

	if (the_chip->soc_now >= SOC_LEVEL_AT_4200MV) {
		pr_info("[Cable impedance] battery capacity(%d%%) is higher, ignore detection.",
			the_chip->soc_now);
		goto endWorker;
	}

	pr_info("[Cable impedance]Start to calculate cable impedance!\n");
	gs_cable_impedance = 4;

	icl_cfg_ma = smb1360_get_input_current_limit(the_chip);

	aicl_result = smb1360_get_input_current_limit_status(the_chip);
	gs_aicl_result = aicl_result;

	if (((!smb1360_is_fastchg_on(the_chip)) || (pwr_src != HTC_PWR_SOURCE_TYPE_AC)) ||
			((smb1360_get_prop_batt_health(the_chip) != POWER_SUPPLY_HEALTH_GOOD) || (aicl_result > 700))) {
		pr_info("[Cable impedance]Ignore detection. aicl=%dmA, chg_type=%d, batt_temp=%d\n",
			aicl_result, smb1360_get_prop_charge_type(the_chip), smb1360_get_prop_batt_temp(the_chip));
		gs_cable_impedance = 1;
		goto endWorker;
	}

	smb1360_chg_aicl_enable(the_chip, false);

	

	
	{
		rc = smb1360_input_current_limit_set(the_chip, 500);
		if (rc < 0) {
			pr_err("Couldn't set cable impedance 500mA ICL vote rc=%d\n", rc);
			goto turn_on_hw_aicl_rerun;
		}
	}
	msleep(3000);
	if (pwr_src != HTC_PWR_SOURCE_TYPE_AC) {
		pr_info("[Cable impedance]Not correct charger source, ignore detection.!\n");
		gs_cable_impedance = 1;
		goto turn_on_hw_aicl_rerun;
	}
	vusbin1 = smb1360_get_usbin(the_chip)/1000;
	vusbin2 = smb1360_get_usbin(the_chip)/1000;
	vusbin3 = smb1360_get_usbin(the_chip)/1000;
	vusbin_mv_1 = (vusbin1 + vusbin2 + vusbin3)/3;
	iusb1 = 500;
	pr_info("[Cable impedance] Iusb=500mA(%d), Vusb1-1=%d,Vusb1-2=%d,Vusb1-3=%d,Vusb1=%d\n",
			iusb1, vusbin1, vusbin2, vusbin3, vusbin_mv_1);

	
	{
		rc = smb1360_input_current_limit_set(the_chip, 400);
		if (rc < 0) {
			pr_err("Couldn't set cable impedance 400mA ICL vote rc=%d\n", rc);
			goto turn_on_hw_aicl_rerun;
		}
	}
	msleep(3000);
	if (pwr_src != HTC_PWR_SOURCE_TYPE_AC) {
		pr_info("[Cable impedance]Not correct charger source, ignore detection.!\n");
		gs_cable_impedance = 1;
		goto turn_on_hw_aicl_rerun;
	}
	vusbin1 = smb1360_get_usbin(the_chip)/1000;
	vusbin2 = smb1360_get_usbin(the_chip)/1000;
	vusbin3 = smb1360_get_usbin(the_chip)/1000;
	vusbin_mv_2 = (vusbin1 + vusbin2 + vusbin3)/3;
	iusb2 = 400;
	pr_info("[Cable impedance] Iusb=400mA(%d), Vusb2-1=%d,Vusb2-2=%d,Vusb2-3=%d,Vusb2=%d\n",
			iusb2, vusbin1, vusbin2, vusbin3, vusbin_mv_2);

	
	{
		rc = smb1360_input_current_limit_set(the_chip, 300);
		if (rc < 0) {
			pr_err("Couldn't set cable impedance 300mA ICL vote rc=%d\n", rc);
			goto turn_on_hw_aicl_rerun;
		}
	}
	msleep(3000);
	if (pwr_src != HTC_PWR_SOURCE_TYPE_AC) {
		pr_info("[Cable impedance]Not correct charger source, ignore detection.!\n");
		gs_cable_impedance = 1;
		goto turn_on_hw_aicl_rerun;
	}
	vusbin1 = smb1360_get_usbin(the_chip)/1000;
	vusbin2 = smb1360_get_usbin(the_chip)/1000;
	vusbin3 = smb1360_get_usbin(the_chip)/1000;
	vusbin_mv_3 = (vusbin1 + vusbin2 + vusbin3)/3;
	iusb3 = 300;
	pr_info("[Cable impedance] Iusb=300mA(%d), Vusb3-1=%d,Vusb3-2=%d,Vusb3-3=%d,Vusb3=%d\n",
			iusb3, vusbin1, vusbin2, vusbin3, vusbin_mv_3);

	rRes1 = ((vusbin_mv_3 - vusbin_mv_1) * 1000 / (iusb1 - iusb3)) - gs_R_HW_MB_Impedance;
	rRes2 = ((vusbin_mv_3 - vusbin_mv_2) * 1000 / (iusb2 - iusb3)) - gs_R_HW_MB_Impedance;

	pr_info("[Cable impedance] R1=%d, R2=%d, Vusb1=%d, Vusb2=%d, Vusb3=%d, AICL=%d\n",
			rRes1, rRes2, vusbin_mv_1, vusbin_mv_2, vusbin_mv_3, aicl_result);

	gs_R_cable_impedance = (rRes1 + rRes2)/2;

	pr_info("[Cable impedance] R=%d\n", gs_R_cable_impedance);

	if (gs_R_cable_impedance >= 1000) {
		gs_cable_impedance = 3;
		htc_charger_event_notify(HTC_CHARGER_EVENT_BAD_CABLE);
	} else
		gs_cable_impedance = 2;

	gs_measure_cable_impedance = false;

turn_on_hw_aicl_rerun:
	rc = smb1360_input_current_limit_set(the_chip, icl_cfg_ma);
	if (rc < 0)
		pr_err("Couldn't recover %dmA ICL before run cable impedance rc=%d\n", icl_cfg_ma, rc);

	smb1360_chg_aicl_enable(the_chip, true);

endWorker:
	gs_stop_eoc = false;
	pr_info("[Cable impedance]End.\n");

	smb1360_ext_charging_disable(the_chip, false);
}
#endif 

static int bound(int val, int min, int max)
{
	if (val < min)
		return min;
	if (val > max)
		return max;

	return val;
}

static int __smb1360_read(struct smb1360_chip *chip, int reg,
				u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		*val = ret;
	}
	pr_debug("Reading 0x%02x=0x%02x\n", reg, *val);

	return 0;
}

static int __smb1360_write(struct smb1360_chip *chip, int reg,
						u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	pr_debug("Writing 0x%02x=0x%02x\n", reg, val);
	return 0;
}

static int smb1360_read(struct smb1360_chip *chip, int reg,
				u8 *val)
{
	int rc;

	if (chip->skip_reads) {
		*val = 0;
		return 0;
	}
	mutex_lock(&chip->read_write_lock);
	rc = __smb1360_read(chip, reg, val);
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

static int smb1360_write(struct smb1360_chip *chip, int reg,
						u8 val)
{
	int rc;

	if (chip->skip_writes)
		return 0;

	mutex_lock(&chip->read_write_lock);
	rc = __smb1360_write(chip, reg, val);
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

static int smb1360_fg_read(struct smb1360_chip *chip, int reg,
				u8 *val)
{
	int rc;

	if (chip->skip_reads) {
		*val = 0;
		return 0;
	}

	mutex_lock(&chip->read_write_lock);
	chip->client->addr = chip->fg_i2c_addr;
	rc = __smb1360_read(chip, reg, val);
	chip->client->addr = chip->default_i2c_addr;
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

static int smb1360_fg_write(struct smb1360_chip *chip, int reg,
						u8 val)
{
	int rc;

	if (chip->skip_writes)
		return 0;

	mutex_lock(&chip->read_write_lock);
	chip->client->addr = chip->fg_i2c_addr;
	rc = __smb1360_write(chip, reg, val);
	chip->client->addr = chip->default_i2c_addr;
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

static int smb1360_read_bytes(struct smb1360_chip *chip, int reg,
						u8 *val, u8 bytes)
{
	s32 rc;

	if (chip->skip_reads) {
		*val = 0;
		return 0;
	}

	mutex_lock(&chip->read_write_lock);
	rc = i2c_smbus_read_i2c_block_data(chip->client, reg, bytes, val);
	if (rc < 0)
		dev_err(chip->dev,
			"i2c read fail: can't read %d bytes from %02x: %d\n",
							bytes, reg, rc);
	mutex_unlock(&chip->read_write_lock);

	return (rc < 0) ? rc : 0;
}

static int smb1360_write_bytes(struct smb1360_chip *chip, int reg,
						u8 *val, u8 bytes)
{
	s32 rc;

	if (chip->skip_writes) {
		*val = 0;
		return 0;
	}

	mutex_lock(&chip->read_write_lock);
	rc = i2c_smbus_write_i2c_block_data(chip->client, reg, bytes, val);
	if (rc < 0)
		dev_err(chip->dev,
			"i2c write fail: can't read %d bytes from %02x: %d\n",
							bytes, reg, rc);
	mutex_unlock(&chip->read_write_lock);

	return (rc < 0) ? rc : 0;
}

static int smb1360_masked_write(struct smb1360_chip *chip, int reg,
						u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	if (chip->skip_writes || chip->skip_reads)
		return 0;

	mutex_lock(&chip->read_write_lock);
	rc = __smb1360_read(chip, reg, &temp);
	if (rc < 0) {
		dev_err(chip->dev, "read failed: reg=%03X, rc=%d\n", reg, rc);
		goto out;
	}
	temp &= ~mask;
	temp |= val & mask;
	rc = __smb1360_write(chip, reg, temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"write failed: reg=%03X, rc=%d\n", reg, rc);
	}
out:
	mutex_unlock(&chip->read_write_lock);
	return rc;
}

#define EXPONENT_MASK		0xF800
#define MANTISSA_MASK		0x3FF
#define SIGN_MASK		0x400
#define EXPONENT_SHIFT		11
#define SIGN_SHIFT		10
#define MICRO_UNIT		1000000ULL
static int64_t float_decode(u16 reg)
{
	int64_t final_val, exponent_val, mantissa_val;
	int exponent, mantissa, n;
	bool sign;

	exponent = (reg & EXPONENT_MASK) >> EXPONENT_SHIFT;
	mantissa = (reg & MANTISSA_MASK);
	sign = !!(reg & SIGN_MASK);

	pr_debug("exponent=%d mantissa=%d sign=%d\n", exponent, mantissa, sign);

	mantissa_val = mantissa * MICRO_UNIT;

	n = exponent - 15;
	if (n < 0)
		exponent_val = MICRO_UNIT >> -n;
	else
		exponent_val = MICRO_UNIT << n;

	n = n - 10;
	if (n < 0)
		mantissa_val >>= -n;
	else
		mantissa_val <<= n;

	final_val = exponent_val + mantissa_val;

	if (sign)
		final_val *= -1;

	return final_val;
}

#define MAX_MANTISSA    (1023 * 1000000ULL)
unsigned int float_encode(int64_t float_val)
{
	int exponent = 0, sign = 0;
	unsigned int final_val = 0;

	if (float_val == 0)
		return 0;

	if (float_val < 0) {
		sign = 1;
		float_val = -float_val;
	}

	
	while (float_val >= MAX_MANTISSA) {
		exponent++;
		float_val >>= 1;
	}

	
	while (float_val < MAX_MANTISSA && exponent > -25) {
		exponent--;
		float_val <<= 1;
	}

	exponent = exponent + 25;

	
	float_val = div_s64((float_val + MICRO_UNIT), (int)MICRO_UNIT);

	if (float_val == 1024) {
		exponent--;
		float_val <<= 1;
	}

	float_val -= 1024;

	
	if (float_val > MANTISSA_MASK)
		float_val = MANTISSA_MASK;

	
	final_val = (float_val & MANTISSA_MASK) | (sign << SIGN_SHIFT) |
		((exponent << EXPONENT_SHIFT) & EXPONENT_MASK);

	return final_val;
}

static int smb1360_enable_fg_access(struct smb1360_chip *chip)
{
	int rc;
	u8 reg = 0, timeout = 50;

	rc = smb1360_masked_write(chip, CMD_I2C_REG, FG_ACCESS_ENABLED_BIT,
							FG_ACCESS_ENABLED_BIT);
	if (rc) {
		pr_err("Couldn't enable FG access rc=%d\n", rc);
		return rc;
	}

	while (timeout) {
		
		msleep(200);
		rc = smb1360_read(chip, IRQ_I_REG, &reg);
		if (rc)
			pr_err("Could't read IRQ_I_REG rc=%d\n", rc);
		else if (reg & FG_ACCESS_ALLOWED_BIT)
			break;
		timeout--;
	}

	pr_debug("timeout=%d\n", timeout);

	if (!timeout)
		return -EBUSY;

	return 0;
}

static inline bool is_device_suspended(struct smb1360_chip *chip)
{
	return !chip->resume_completed;
}

static int smb1360_disable_fg_access(struct smb1360_chip *chip)
{
	int rc;

	rc = smb1360_masked_write(chip, CMD_I2C_REG, FG_ACCESS_ENABLED_BIT, 0);
	if (rc)
		pr_err("Couldn't disable FG access rc=%d\n", rc);

	return rc;
}

static int smb1360_enable_volatile_writes(struct smb1360_chip *chip)
{
	int rc;

	rc = smb1360_masked_write(chip, CMD_I2C_REG,
		ALLOW_VOLATILE_BIT, ALLOW_VOLATILE_BIT);
	if (rc < 0)
		dev_err(chip->dev,
			"Couldn't set VOLATILE_W_PERM_BIT rc=%d\n", rc);

	return rc;
}

#ifndef CONFIG_HTC_BATT_8960
#define TRIM_1C_REG		0x1C
#define CHECK_USB100_GOOD_BIT	BIT(6)
static bool is_usb100_broken(struct smb1360_chip *chip)
{
	int rc;
	u8 reg;

	rc = smb1360_read(chip, TRIM_1C_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read trim 1C reg rc = %d\n", rc);
		return rc;
	}
	return !!(reg & CHECK_USB100_GOOD_BIT);
}
#endif 

static int read_revision(struct smb1360_chip *chip, u8 *revision)
{
	int rc;

	*revision = 0;
	rc = smb1360_read(chip, REVISION_CTRL_REG, revision);
	if (rc)
		dev_err(chip->dev, "Couldn't read REVISION_CTRL_REG rc=%d", rc);

	*revision &= DEVICE_REV_MASK;

	return rc;
}

#define MIN_FLOAT_MV		3460
#define MAX_FLOAT_MV		4730
#define VFLOAT_STEP_MV		10
static int smb1360_float_voltage_set(struct smb1360_chip *chip, int vfloat_mv)
{
	u8 temp;

	if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
		dev_err(chip->dev, "bad float voltage mv =%d asked to set\n",
					vfloat_mv);
		return -EINVAL;
	}

	temp = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV;

	pr_info("voltage=%d setting 0x%02X\n", vfloat_mv, temp);

	return smb1360_masked_write(chip, BATT_CHG_FLT_VTG_REG,
				VFLOAT_MASK, temp);
}

#define MIN_RECHG_MV		50
#define MAX_RECHG_MV		300
static int smb1360_recharge_threshold_set(struct smb1360_chip *chip,
							int resume_mv)
{
	u8 temp;

	if ((resume_mv < MIN_RECHG_MV) || (resume_mv > MAX_RECHG_MV)) {
		dev_err(chip->dev, "bad rechg_thrsh =%d asked to set\n",
							resume_mv);
		return -EINVAL;
	}

	temp = resume_mv / 100;

	pr_info("resume_mv=%dmV setting 0x%02X\n", resume_mv, temp);

	return smb1360_masked_write(chip, CFG_BATT_CHG_REG,
		RECHG_MV_MASK, temp << RECHG_MV_SHIFT);
}

static int __smb1360_charging_disable(struct smb1360_chip *chip, bool disable)
{
	int rc;

	if (disable && !g_chg_error) {
		
		
		rc = smb1360_masked_write(chip, CFG_WD_CTRL,
						CFG_WD_CTRL_BIT,
						0);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set CHGR_WD_CTRL = %d\n", rc);
		

		
		rc = smb1360_masked_write(chip, CMD_IL_REG,
				USB_CTRL_MASK, USB_500_BIT);
		if (rc) {
			pr_err("Couldn't configure for USB500 rc=%d\n", rc);
			return rc;
		}

		
		rc = smb1360_masked_write(chip, CMD_CHG_REG,
				CMD_CHG_EN, CMD_CHG_EN);
		if (rc < 0)
			pr_err("Couldn't set CHG_ENABLE_BIT rc = %d\n", rc);
		else
			pr_debug("CHG_EN status=Enable\n");

		return rc;
	}


	rc = smb1360_masked_write(chip, CMD_CHG_REG,
			CMD_CHG_EN, disable ? 0 : CMD_CHG_EN);
	if (rc < 0)
		pr_err("Couldn't set CHG_ENABLE_BIT disable=%d rc = %d\n",
							disable, rc);
	else
		pr_debug("CHG_EN status=%d\n", !disable);

	return rc;
}

static int smb1360_charging_disable(struct smb1360_chip *chip, int reason,
								int disable)
{
	int rc = 0;
	int disabled;

	mutex_lock(&chip->charging_disable_lock);

	disabled = chip->charging_disabled_status;

	pr_debug("reason=%d requested_disable=%d disabled_status=%d\n",
					reason, disable, disabled);

	if (disable == true)
		disabled |= reason;
	else
		disabled &= ~reason;

	if (disabled)
		rc = __smb1360_charging_disable(chip, true);
	else
		rc = __smb1360_charging_disable(chip, false);

	if (rc)
		pr_err("Couldn't disable charging for reason=%d rc=%d\n",
							reason, rc);
	else
		chip->charging_disabled_status = disabled;

	mutex_unlock(&chip->charging_disable_lock);

	return rc;
}

static int smb1360_ext_charging_disable(struct smb1360_chip *chip, int disable)
{
	int rc = 0;

	if (!chip->pm8916_enabled)
		return rc;

	if (disable || chg_limit_current != 0 || iusb_limit_enable) {
		pr_info("disable=%d, chg_limit_current=%d, iusb_limit_enable=%d\n",
					disable, chg_limit_current, iusb_limit_enable);

		
		if (delayed_work_pending(&chip->aicl_work))
			cancel_delayed_work_sync(&chip->aicl_work);

		if (is_aicl_ongoing)
			chip->pm8916_psy_ma = 0; 
		else
			pr_info("pm8916_psy_ma=%d\n", chip->pm8916_psy_ma);

		smb1360_clean_aicl_flag(chip);

		rc = pm8916_charger_enable(0, chip->pm8916_psy_ma);

		if (rc)
			pr_err("Couldn't disable pm8916 charging rc=%d\n", rc);

		
		rc = smb1360_chg_aicl_enable(chip, 1);
		if (rc < 0)
			pr_err("enable/disable AICL failed rc=%d\n", rc);
	} else {
		
		if (is_ac_online())
			schedule_delayed_work(&chip->aicl_work, msecs_to_jiffies(SMB1360_AICL_START_PM8916_DELAY_MS));
		else
			pr_info("do not enable pm8916 charging due to cable type = %d\n", pwr_src);
	}

	return rc;
}


#ifndef CONFIG_HTC_BATT_8960
static enum power_supply_property smb1360_battery_properties[] = {
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_RESISTANCE,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
};
#endif

static int smb1360_get_prop_batt_present(struct smb1360_chip *chip)
{
	return chip->batt_present;
}

static int smb1360_get_prop_batt_status(struct smb1360_chip *chip)
{
	int rc;
	u8 reg = 0, chg_type;

	if (is_device_suspended(chip))
		return POWER_SUPPLY_STATUS_UNKNOWN;

	if (chip->batt_full)
		return POWER_SUPPLY_STATUS_FULL;

	rc = smb1360_read(chip, STATUS_3_REG, &reg);
	if (rc) {
		pr_err("Couldn't read STATUS_3_REG rc=%d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	pr_debug("STATUS_3_REG = %x\n", reg);

	if (reg & CHG_HOLD_OFF_BIT)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;

	if (chg_type == BATT_NOT_CHG_VAL)
		return POWER_SUPPLY_STATUS_DISCHARGING;
	else
		return POWER_SUPPLY_STATUS_CHARGING;
}

#ifndef CONFIG_HTC_BATT_8960
static int smb1360_get_prop_charging_status(struct smb1360_chip *chip)
{
	int rc;
	u8 reg = 0;

	rc = smb1360_read(chip, STATUS_3_REG, &reg);
	if (rc) {
		pr_err("Couldn't read STATUS_3_REG rc=%d\n", rc);
		return 0;
	}

	return (reg & CHG_EN_BIT) ? 1 : 0;
}
#endif 

static int smb1360_get_prop_charge_type(struct smb1360_chip *chip)
{
	int rc;
	u8 reg = 0;
	u8 chg_type;

	if (is_device_suspended(chip))
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	rc = smb1360_read(chip, STATUS_3_REG, &reg);
	if (rc) {
		pr_err("Couldn't read STATUS_3_REG rc=%d\n", rc);
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;
	if (chg_type == BATT_NOT_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	else if ((chg_type == BATT_FAST_CHG_VAL) ||
			(chg_type == BATT_TAPER_CHG_VAL))
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if (chg_type == BATT_PRE_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int smb1360_get_prop_batt_health(struct smb1360_chip *chip)
{
	union power_supply_propval ret = {0, };

	if (chip->batt_hot)
		ret.intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (chip->batt_cold)
		ret.intval = POWER_SUPPLY_HEALTH_COLD;
	else if (chip->batt_warm)
		ret.intval = POWER_SUPPLY_HEALTH_WARM;
	else if (chip->batt_cool)
		ret.intval = POWER_SUPPLY_HEALTH_COOL;
	else
		ret.intval = POWER_SUPPLY_HEALTH_GOOD;

	return ret.intval;
}

static int smb1360_get_prop_batt_capacity(struct smb1360_chip *chip)
{
	u8 reg;
	u32 temp = 0;
	int rc, soc = 0;

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;

	if (is_device_suspended(chip))
		return chip->soc_now;

	rc = smb1360_read(chip, SHDW_FG_MSYS_SOC, &reg);
	if (rc) {
		pr_err("Failed to read FG_MSYS_SOC rc=%d\n", rc);
		return rc;
	}
	soc = (100 * reg) / MAX_8_BITS;

	temp = (100 * reg) % MAX_8_BITS;
	if (temp > (MAX_8_BITS / 2))
		soc += 1;

	if (chip->empty_soc) {
		pr_debug("empty_soc\n");
		soc = 0;
	}

	pr_debug("msys_soc_reg=0x%02x, fg_soc=%d batt_full = %d\n", reg,
						soc, chip->batt_full);

	if (chip->wa_to_re_calc_soc_in_low_temp)
		soc = smb1360_re_calculate_soc_in_low_temperature(chip, soc);

	soc = smb1360_clamp_soc_based_on_voltage(chip, soc);

	chip->soc_now = (chip->batt_full ? 100 : bound(soc, 0, 100));

	return chip->soc_now;
}

#ifndef CONFIG_HTC_BATT_8960
static int smb1360_get_prop_chg_full_design(struct smb1360_chip *chip)
{
	u8 reg[2];
	int rc, fcc_mah = 0;

	if (is_device_suspended(chip))
		return chip->fcc_mah;

	rc = smb1360_read_bytes(chip, SHDW_FG_CAPACITY, reg, 2);
	if (rc) {
		pr_err("Failed to read SHDW_FG_CAPACITY rc=%d\n", rc);
		return rc;
	}
	fcc_mah = (reg[1] << 8) | reg[0];

	pr_debug("reg[0]=0x%02x reg[1]=0x%02x fcc_mah=%d\n",
				reg[0], reg[1], fcc_mah);

	chip->fcc_mah = fcc_mah * 1000;

	return chip->fcc_mah;
}
#endif 

static int smb1360_get_prop_batt_temp(struct smb1360_chip *chip)
{
	u8 reg[2];
	int rc, temp = 0;

	if (is_device_suspended(chip))
		return chip->temp_now;

	rc = smb1360_read_bytes(chip, SHDW_FG_BATT_TEMP, reg, 2);
	if (rc) {
		pr_err("Failed to read SHDW_FG_BATT_TEMP rc=%d\n", rc);
		return rc;
	}

	temp = (reg[1] << 8) | reg[0];
	temp = div_u64(temp * 625, 10000UL);	
	temp = (temp - 273) * 10;		

	pr_debug("reg[0]=0x%02x reg[1]=0x%02x temperature=%d\n",
					reg[0], reg[1], temp);

	chip->temp_now = temp;

	return chip->temp_now;
}

static int smb1360_get_prop_voltage_now(struct smb1360_chip *chip)
{
	u8 reg[2];
	int rc, temp = 0;

	if (is_device_suspended(chip))
		return chip->voltage_now;

	rc = smb1360_read_bytes(chip, SHDW_FG_VTG_NOW, reg, 2);
	if (rc) {
		pr_err("Failed to read SHDW_FG_VTG_NOW rc=%d\n", rc);
		return rc;
	}

	temp = (reg[1] << 8) | reg[0];
	temp = div_u64(temp * 5000, 0x7FFF);

	pr_debug("reg[0]=0x%02x reg[1]=0x%02x voltage=%d\n",
				reg[0], reg[1], temp);

	chip->voltage_now = temp;

	return chip->voltage_now;
}

#ifndef CONFIG_HTC_BATT_8960
static int smb1360_get_prop_batt_resistance(struct smb1360_chip *chip)
{
	u8 reg[2];
	u16 temp;
	int rc;
	int64_t resistance;

	if (is_device_suspended(chip))
		return chip->resistance_now;

	rc = smb1360_read_bytes(chip, SHDW_FG_ESR_ACTUAL, reg, 2);
	if (rc) {
		pr_err("Failed to read FG_ESR_ACTUAL rc=%d\n", rc);
		return rc;
	}
	temp = (reg[1] << 8) | reg[0];

	resistance = float_decode(temp) * 2;

	pr_debug("reg=0x%02x resistance=%lld\n", temp, resistance);

	
	chip->resistance_now = resistance;

	return chip->resistance_now;
}
#endif 

static int smb1360_get_prop_current_now(struct smb1360_chip *chip)
{
	u8 reg[2];
	int rc, temp = 0;

	if (is_device_suspended(chip))
		return chip->current_now;

	rc = smb1360_read_bytes(chip, SHDW_FG_CURR_NOW, reg, 2);
	if (rc) {
		pr_err("Failed to read SHDW_FG_CURR_NOW rc=%d\n", rc);
		return rc;
	}

	temp = ((s8)reg[1] << 8) | reg[0];
	temp = div_s64(temp * 2500, 0x7FFF);

	pr_debug("reg[0]=0x%02x reg[1]=0x%02x current=%d\n",
				reg[0], reg[1], temp * 1000);

	chip->current_now = temp * 1000;

	return chip->current_now;
}

#ifndef CONFIG_HTC_BATT_8960
static int smb1360_set_minimum_usb_current(struct smb1360_chip *chip)
{
	int rc = 0;

	if (chip->min_icl_usb100) {
		pr_debug("USB min current set to 100mA\n");
		
		rc = smb1360_masked_write(chip, CFG_BATT_CHG_ICL_REG,
						INPUT_CURR_LIM_MASK,
						INPUT_CURR_LIM_300MA);
		if (rc)
			pr_err("Couldn't set ICL mA rc=%d\n", rc);

		if (!(chip->workaround_flags & WRKRND_USB100_FAIL))
			rc = smb1360_masked_write(chip, CMD_IL_REG,
					USB_CTRL_MASK, USB_100_BIT);
			if (rc)
				pr_err("Couldn't configure for USB100 rc=%d\n",
								rc);
	} else {
		pr_debug("USB min current set to 500mA\n");
		rc = smb1360_masked_write(chip, CMD_IL_REG,
				USB_CTRL_MASK, USB_500_BIT);
		if (rc)
			pr_err("Couldn't configure for USB100 rc=%d\n",
							rc);
	}

	return rc;
}
#endif 

static int smb1360_set_appropriate_usb_current(struct smb1360_chip *chip)
{
	int rc = 0, i, therm_ma, current_ma;
	int path_current = chip->usb_psy_ma;

	if (!chip->batt_present) {
		pr_debug("ignoring current request since battery is absent\n");
		return 0;
	}

		therm_ma = path_current;

	current_ma = min(therm_ma, path_current);

	if (current_ma <= 2) {
#ifndef CONFIG_HTC_BATT_8960
		pr_debug("current_ma=%d <= 2 set USB current to minimum\n",
								current_ma);
		rc = smb1360_set_minimum_usb_current(chip);
		if (rc < 0)
			pr_err("Couldn't to set minimum USB current rc = %d\n",
								rc);
#else
		pr_debug("current_ma=%d <= 2 disable charging\n", current_ma);

		smb1360_ext_charging_disable(chip, true);
		rc = smb1360_charging_disable(chip, CURRENT, true);
		if (rc < 0)
			pr_err("Unable to disable charging rc=%d\n", rc);
#endif 

		return rc;
	}

	for (i = ARRAY_SIZE(input_current_limit) - 1; i >= 0; i--) {
		if (input_current_limit[i] <= current_ma)
			break;
	}
	if (i < 0) {
		pr_warn("Couldn't find ICL mA rc=%d\n", rc);
		i = 0;
	}
	
	rc = smb1360_masked_write(chip, CFG_BATT_CHG_ICL_REG,
					INPUT_CURR_LIM_MASK, i);
	if (rc)
		pr_err("Couldn't set ICL mA rc=%d\n", rc);

	pr_info("ICL set to = %d\n", input_current_limit[i]);

	if ((current_ma <= CURRENT_100_MA) &&
		((chip->workaround_flags & WRKRND_USB100_FAIL) ||
				!chip->min_icl_usb100)) {
		pr_debug("usb100 not supported: usb100_wrkrnd=%d min_icl_100=%d\n",
			!!(chip->workaround_flags & WRKRND_USB100_FAIL),
						chip->min_icl_usb100);
		current_ma = CURRENT_500_MA;
	}

	if (current_ma <= CURRENT_100_MA) {
		
		rc = smb1360_masked_write(chip, CMD_IL_REG,
				USB_CTRL_MASK, USB_100_BIT);
		if (rc)
			pr_err("Couldn't configure for USB100 rc=%d\n", rc);
		pr_debug("Setting USB 100\n");
	} else if (current_ma <= CURRENT_500_MA) {
		
		rc = smb1360_masked_write(chip, CMD_IL_REG,
				USB_CTRL_MASK, USB_500_BIT);
		if (rc)
			pr_err("Couldn't configure for USB500 rc=%d\n", rc);
		pr_debug("Setting USB 500\n");
	} else {
#ifndef CONFIG_HTC_BATT_8960
		
		if (chip->rsense_10mohm)
			current_ma /= 2;

		for (i = ARRAY_SIZE(fastchg_current) - 1; i >= 0; i--) {
			if (fastchg_current[i] <= current_ma)
				break;
		}
		if (i < 0) {
			pr_debug("Couldn't find fastchg mA rc=%d\n", rc);
			i = 0;
		}
		
		rc = smb1360_masked_write(chip, CHG_CURRENT_REG,
			FASTCHG_CURR_MASK, i << FASTCHG_CURR_SHIFT);
		if (rc)
			pr_err("Couldn't set fastchg mA rc=%d\n", rc);

		rc = smb1360_masked_write(chip, CMD_IL_REG,
				USB_CTRL_MASK, USB_500_BIT);
		if (rc)
			pr_err("Couldn't configure for USB500 rc=%d\n", rc);

		rc = smb1360_masked_write(chip, CMD_IL_REG,
				USB_CTRL_MASK, USB_AC_BIT);
		if (rc)
			pr_err("Couldn't configure for USB AC rc=%d\n", rc);

		pr_info("fast-chg current set to = %d\n", fastchg_current[i]);
#else
		if (chip->pm8916_enabled) {
			
			rc = smb1360_chg_aicl_enable(the_chip, 1);
			if (rc < 0)
				pr_err("enable/disable AICL failed rc=%d\n", rc);
		}

		qpnp_chg_set_appropriate_battery_current(chip);
#endif 
	}

	
	rc = smb1360_charging_disable(chip, CURRENT, false);
	if (rc < 0)
		pr_err("Unable to enable charging rc=%d\n", rc);

	return rc;
}

#ifndef CONFIG_HTC_BATT_8960
static int smb1360_system_temp_level_set(struct smb1360_chip *chip,
							int lvl_sel)
{
	int rc = 0;
	int prev_therm_lvl;

	if (!chip->thermal_mitigation) {
		pr_err("Thermal mitigation not supported\n");
		return -EINVAL;
	}

	if (lvl_sel < 0) {
		pr_err("Unsupported level selected %d\n", lvl_sel);
		return -EINVAL;
	}

	if (lvl_sel >= chip->thermal_levels) {
		pr_err("Unsupported level selected %d forcing %d\n", lvl_sel,
				chip->thermal_levels - 1);
		lvl_sel = chip->thermal_levels - 1;
	}

	if (lvl_sel == chip->therm_lvl_sel)
		return 0;

	mutex_lock(&chip->current_change_lock);
	prev_therm_lvl = chip->therm_lvl_sel;
	chip->therm_lvl_sel = lvl_sel;

	if (chip->therm_lvl_sel == (chip->thermal_levels - 1)) {
		rc = smb1360_set_minimum_usb_current(chip);
		if (rc)
			pr_err("Couldn't set USB current to minimum rc = %d\n",
							rc);
	} else {
		rc = smb1360_set_appropriate_usb_current(chip);
		if (rc)
			pr_err("Couldn't set USB current rc = %d\n", rc);
	}

	mutex_unlock(&chip->current_change_lock);
	return rc;
}

static int smb1360_battery_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct smb1360_chip *chip = container_of(psy,
				struct smb1360_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		smb1360_charging_disable(chip, USER, !val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->fake_battery_soc = val->intval;
		pr_info("fake_soc set to %d\n", chip->fake_battery_soc);
		power_supply_changed(&chip->batt_psy);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		smb1360_system_temp_level_set(chip, val->intval);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smb1360_battery_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

static int smb1360_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smb1360_chip *chip = container_of(psy,
				struct smb1360_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = smb1360_get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = smb1360_get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = smb1360_get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = smb1360_get_prop_charging_status(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = smb1360_get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = smb1360_get_prop_batt_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = smb1360_get_prop_chg_full_design(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = smb1360_get_prop_voltage_now(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = smb1360_get_prop_current_now(chip);
		break;
	case POWER_SUPPLY_PROP_RESISTANCE:
		val->intval = smb1360_get_prop_batt_resistance(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = smb1360_get_prop_batt_temp(chip);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = chip->therm_lvl_sel;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void smb1360_external_power_changed(struct power_supply *psy)
{
	struct smb1360_chip *chip = container_of(psy,
				struct smb1360_chip, batt_psy);
	union power_supply_propval prop = {0,};
	int rc, current_limit = 0;

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (rc < 0)
		dev_err(chip->dev,
			"could not read USB current_max property, rc=%d\n", rc);
	else
		current_limit = prop.intval / 1000;

	pr_debug("current_limit = %d\n", current_limit);

	if (chip->usb_psy_ma != current_limit) {
		mutex_lock(&chip->current_change_lock);
		chip->usb_psy_ma = current_limit;
		rc = smb1360_set_appropriate_usb_current(chip);
		if (rc < 0)
			pr_err("Couldn't set usb current rc = %d\n", rc);
		mutex_unlock(&chip->current_change_lock);
	}

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_ONLINE, &prop);
	if (rc < 0)
		pr_err("could not read USB ONLINE property, rc=%d\n", rc);

	
	rc = 0;
	if (chip->usb_present && !chip->charging_disabled_status
					&& chip->usb_psy_ma != 0) {
		if (prop.intval == 0)
			rc = power_supply_set_online(chip->usb_psy, true);
	} else {
		if (prop.intval == 1)
			rc = power_supply_set_online(chip->usb_psy, false);
	}
	if (rc < 0)
		pr_err("could not set usb online, rc=%d\n", rc);
}
#endif 

static int hot_hard_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	int batt_temp;

	batt_temp = smb1360_get_prop_batt_temp(chip);
	pr_info("batt_temp=%d degreeC, rt_stat = 0x%02x\n", batt_temp, rt_stat);
	chip->batt_hot = !!rt_stat;

	if (!(flag_keep_charge_on || flag_pa_recharge))
		
		smb1360_ext_charging_disable(chip, chip->batt_hot);

	htc_gauge_event_notify(HTC_GAUGE_EVENT_TEMP_ZONE_CHANGE);
	return 0;
}

static int cold_hard_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	int batt_temp;

	batt_temp = smb1360_get_prop_batt_temp(chip);
	pr_info("batt_temp=%d degreeC, rt_stat = 0x%02x\n", batt_temp, rt_stat);
	chip->batt_cold = !!rt_stat;

	if (!(flag_keep_charge_on || flag_pa_recharge))
		
		smb1360_ext_charging_disable(chip, chip->batt_cold);

	htc_gauge_event_notify(HTC_GAUGE_EVENT_TEMP_ZONE_CHANGE);
	return 0;
}

static int hot_soft_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	int batt_temp;
	batt_temp = smb1360_get_prop_batt_temp(chip);
	pr_info("batt_temp=%d degreeC, rt_stat = 0x%02x\n", batt_temp, rt_stat);

	chip->batt_warm = !!rt_stat;
	htc_gauge_event_notify(HTC_GAUGE_EVENT_TEMP_ZONE_CHANGE);
	return 0;
}

static int cold_soft_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_info("rt_stat = 0x%02x\n", rt_stat);
	chip->batt_cool = !!rt_stat;
	return 0;
}

static int battery_missing_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_info("rt_stat = 0x%02x\n", rt_stat);
	chip->batt_present = !rt_stat;
	return 0;
}

static int vbat_low_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_info("vbat low\n");

	return 0;
}

static int chg_hot_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_warn_ratelimited("chg hot\n");
	return 0;
}

static int chg_term_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_info("rt_stat = 0x%02x\n", rt_stat);
	if (rt_stat & IRQ_C_CHG_TERM) {
		is_batt_full_eoc_stop = true;
		htc_gauge_event_notify(HTC_GAUGE_EVENT_EOC_STOP_CHG);
	}
	chip->batt_full = !!rt_stat;
	return 0;
}

static int recharge_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_info("rt_stat = 0x%02x, is_batt_full_eoc_stop=%d\n",
			rt_stat, is_batt_full_eoc_stop);

	is_batt_full_eoc_stop = false;
	htc_start_eoc_work(chip);

	return 0;
}

static int chg_fastchg_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_debug("rt_stat = 0x%02x\n", rt_stat);

	return 0;
}

static int wd_timeout_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_info("watchdog timeout, rt_stat = 0x%02x\n", rt_stat);

	return 0;
}

static int chg_safety_timeout_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	int rc;
	pr_info("rt_stat = 0x%02x\n", rt_stat);

	if(is_ac_online() && (rt_stat & IRQ_D_CHARGE_TIMEOUT_BIT)) {
		
		
		rc = smb1360_masked_write(chip, CFG_WD_CTRL,
						CFG_WD_CTRL_BIT, CFG_WD_CTRL_BIT);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set CHGR_WD_CTRL = %d\n", rc);

		
		schedule_delayed_work(&chip->kick_wd_work, 0);
		

		is_ac_safety_timeout = true;
		htc_charger_event_notify(HTC_CHARGER_EVENT_SAFETY_TIMEOUT);
	}

	return 0;
}

static int aicl_done_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_info("rt_stat = 0x%02x\n", rt_stat);


	return 0;
}

static int usbin_uv_handler(struct smb1360_chip *chip, u8 rt_stat)
{
#ifndef CONFIG_HTC_BATT_8960
	bool usb_present = !rt_stat;

	pr_debug("chip->usb_present = %d usb_present = %d\n",
				chip->usb_present, usb_present);
	if (chip->usb_present && !usb_present) {
		
		chip->usb_present = usb_present;
		power_supply_set_present(chip->usb_psy, usb_present);
	}

	if (!chip->usb_present && usb_present) {
		
		chip->usb_present = usb_present;
		power_supply_set_present(chip->usb_psy, usb_present);
	}
#else
	pr_info("rt_stat = 0x%02x\n", rt_stat);
	schedule_delayed_work(&chip->update_ovp_uvp_work,
			msecs_to_jiffies(1000));
#endif 

	return 0;
}

static int usbin_ov_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_info("rt_stat = 0x%02x\n", rt_stat);
	schedule_delayed_work(&chip->update_ovp_uvp_work,
			msecs_to_jiffies(1000));

	return 0;
}

static int chg_inhibit_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_info("rt_stat = 0x%02x\n", rt_stat);
	chip->batt_full = !!rt_stat;
	return 0;
}

#define VIN_MIN_COLLAPSE_CHECK_MS	350
static int power_ok_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_info("chip->usb_present = %d usb_present = %d\n",
			chip->usb_present, !!rt_stat);

	if (rt_stat == 0) {
		if (delayed_work_pending(&the_chip->cable_impedance_work))
			cancel_delayed_work_sync(&the_chip->cable_impedance_work);
	}

	if (chip->pm8916_enabled &&
			chip->usb_present &&
			rt_stat == 0 &&
			(pwr_src == HTC_PWR_SOURCE_TYPE_AC ||
				pwr_src == HTC_PWR_SOURCE_TYPE_9VAC ||
				pwr_src == HTC_PWR_SOURCE_TYPE_MHL_AC)) {
		if (delayed_work_pending(&chip->aicl_work))
			cancel_delayed_work_sync(&chip->aicl_work);

		smb1360_ext_charging_disable(chip, true);

		schedule_delayed_work(&chip->vin_collapse_check_work,
			msecs_to_jiffies(VIN_MIN_COLLAPSE_CHECK_MS));
		wake_lock_timeout(&chip->vin_collapse_check_wake_lock, HZ/2);
	} else {
		cable_detection_vbus_irq_handler();
	}

	return 0;
}

static int delta_soc_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_info("SOC changed! - rt_stat = 0x%02x\n", rt_stat);

	return 0;
}

static int chg_error_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	u8 val;
	int rc;
	bool restart_ac_chg = false;

	pr_info("chg error! - rt_stat = 0x%02x\n", rt_stat);

	rc = smb1360_read(chip, CMD_IL_REG, &val);
	if (rc)
		pr_err("Couldn't configure for USB500 rc=%d\n", rc);
	printk("CMD_IL_REG = 0x%2X\n", val);

	if (rc)
		pr_err("Couldn't configure for USB500 rc=%d\n", rc);

	if(is_ac_safety_timeout && is_ac_online()
			&& !is_batt_full_eoc_stop) {
		pr_info("Safety timer (%dmins) timeout with AC cable\n",
				chip->safety_time);

		
		chip->pm8916_psy_ma = 0;
		rc = pm8916_charger_enable(0, chip->pm8916_psy_ma);
		if (rc)
			pr_err("Couldn't disable pm8916 charging, rc=%d\n", rc);
	} else {
		if (is_ac_safety_timeout && is_ac_online()
				&& is_batt_full_eoc_stop) {
			restart_ac_chg = true;
			pr_info("Safety timer (%dmins) timeout with AC cable "
				"but enable charging again due to chg_term was not triggered\n",
					chip->safety_time);
		}

		smb1360_ext_charging_disable(chip, true);
		g_chg_error = 1;
		
		rc = smb1360_charging_disable(chip, USER, true);
		if (rc < 0)
			pr_err("Unable to disable charging rc=%d\n", rc);
		g_chg_error = 0;

		rc = smb1360_masked_write(chip, CMD_IL_REG,
				USB_CTRL_MASK, USB_500_BIT);
		if (rc)
			pr_err("Couldn't configure for USB500 rc=%d\n", rc);

		if (restart_ac_chg
			|| (is_ac_online() && !rt_stat)) {
			is_ac_safety_timeout = false;
			eoc_count_by_curr = eoc_count = dischg_count = 0;
			is_batt_full_eoc_stop = false;

			rc = smb1360_masked_write(chip, CMD_IL_REG,
					USB_CTRL_MASK, USB_AC_BIT);
			if (rc)
				pr_err("Couldn't configure for USB AC rc=%d\n", rc);

			smb1360_ext_charging_disable(chip, false);
		}

		
		rc = smb1360_charging_disable(chip, USER, false);
		if (rc < 0)
			pr_err("Unable to enable charging rc=%d\n", rc);

		htc_start_eoc_work(chip);
	}

	return 0;
}

static int min_soc_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_info("SOC dropped below min SOC\n");

	return 0;
}

static int empty_soc_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	int batt_vol;

	batt_vol = smb1360_get_prop_voltage_now(chip);

	pr_info("SOC empty! rt_stat = 0x%02x, batt_vol=%dmV\n", rt_stat, batt_vol);

	if (!chip->empty_soc_disabled) {
		if (rt_stat) {
			if (chip->v_cutoff_mv != -EINVAL
					&& batt_vol > chip->v_cutoff_mv) {
				pr_info("batt_vol > %dmV, ignore the empty_soc IRQ\n",
					chip->v_cutoff_mv);
				return 0;
			}

			chip->empty_soc = true;
			
			pr_warn_ratelimited("SOC is 0\n");

			
			if (chip->voltage_empty_re_check_mv != -EINVAL
					&& batt_vol <= chip->voltage_empty_re_check_mv)
				htc_gauge_event_notify(HTC_GAUGE_EVENT_LOW_VOLTAGE_ALARM);
		} else {
			chip->empty_soc = false;
			
		}
	}

	return 0;
}

static int full_soc_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	if (rt_stat)
		pr_info("SOC is 100\n");

	return 0;
}

static int fg_access_allowed_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_info("stat=%d\n", !!rt_stat);

	return 0;
}

#ifndef CONFIG_HTC_BATT_8960
static int batt_id_complete_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	pr_debug("batt_id = %x\n", (rt_stat & BATT_ID_RESULT_BIT)
						>> BATT_ID_SHIFT);

	return 0;
}
#endif

static int smb1360_select_fg_i2c_address(struct smb1360_chip *chip)
{
	unsigned short addr = chip->default_i2c_addr << 0x1;

	switch (chip->fg_access_type) {
	case FG_ACCESS_CFG:
		addr = (addr & ~FG_I2C_CFG_MASK) | FG_CFG_I2C_ADDR;
		break;
	case FG_ACCESS_PROFILE_A:
		addr = (addr & ~FG_I2C_CFG_MASK) | FG_PROFILE_A_ADDR;
		break;
	case FG_ACCESS_PROFILE_B:
		addr = (addr & ~FG_I2C_CFG_MASK) | FG_PROFILE_B_ADDR;
		break;
	default:
		pr_err("Invalid FG access type=%d\n", chip->fg_access_type);
		return -EINVAL;
	}

	chip->fg_i2c_addr = addr >> 0x1;
	pr_debug("FG_access_type=%d fg_i2c_addr=%x\n", chip->fg_access_type,
							chip->fg_i2c_addr);

	return 0;
}

static int smb1360_adjust_current_gain(struct smb1360_chip *chip,
							int gain_factor)
{
	int i, rc;
	int64_t current_gain, new_current_gain;
	u8 reg[2];
	u16 reg_value1 = 0, reg_value2 = 0;
	u8 reg_val_mapping[][2] = {
			{0xE0, 0x1D},
			{0xE1, 0x00},
			{0xE2, 0x1E},
			{0xE3, 0x00},
			{0xE4, 0x00},
			{0xE5, 0x00},
			{0xE6, 0x00},
			{0xE7, 0x00},
			{0xE8, 0x00},
			{0xE9, 0x00},
			{0xEA, 0x00},
			{0xEB, 0x00},
			{0xEC, 0x00},
			{0xED, 0x00},
			{0xEF, 0x00},
			{0xF0, 0x50},
			{0xF1, 0x00},
	};

	if (gain_factor) {
		rc = smb1360_fg_read(chip, CURRENT_GAIN_LSB_REG, &reg[0]);
		if (rc) {
			pr_err("Unable to set FG access I2C address rc=%d\n",
									rc);
			return rc;
		}

		rc = smb1360_fg_read(chip, CURRENT_GAIN_MSB_REG, &reg[1]);
		if (rc) {
			pr_err("Unable to set FG access I2C address rc=%d\n",
									rc);
			return rc;
		}

		reg_value1 = (reg[1] << 8) | reg[0];
		current_gain = float_decode(reg_value1);
		new_current_gain = MICRO_UNIT  + (gain_factor * current_gain);
		reg_value2 = float_encode(new_current_gain);
		reg[0] = reg_value2 & 0xFF;
		reg[1] = (reg_value2 & 0xFF00) >> 8;
		pr_debug("current_gain_reg=0x%x current_gain_decoded=%lld new_current_gain_decoded=%lld new_current_gain_reg=0x%x\n",
			reg_value1, current_gain, new_current_gain, reg_value2);

		for (i = 0; i < ARRAY_SIZE(reg_val_mapping); i++) {
			if (reg_val_mapping[i][0] == 0xE1)
				reg_val_mapping[i][1] = reg[0];
			if (reg_val_mapping[i][0] == 0xE3)
				reg_val_mapping[i][1] = reg[1];

			pr_debug("Writing reg_add=%x value=%x\n",
				reg_val_mapping[i][0], reg_val_mapping[i][1]);

			rc = smb1360_fg_write(chip, reg_val_mapping[i][0],
					reg_val_mapping[i][1]);
			if (rc) {
				pr_err("Write fg address 0x%x failed, rc = %d\n",
						reg_val_mapping[i][0], rc);
				return rc;
			}
		}
	} else {
		pr_debug("Disabling gain correction\n");
		rc = smb1360_fg_write(chip, 0xF0, 0x00);
		if (rc) {
			pr_err("Write fg address 0x%x failed, rc = %d\n",
								0xF0, rc);
			return rc;
		}
	}

	return 0;
}

static int smb1360_otp_gain_config(struct smb1360_chip *chip, int gain_factor)
{
	int rc = 0;

	rc = smb1360_enable_fg_access(chip);
	if (rc) {
		pr_err("Couldn't request FG access rc = %d\n", rc);
		return rc;
	}
	chip->fg_access_type = FG_ACCESS_CFG;

	rc = smb1360_select_fg_i2c_address(chip);
	if (rc) {
		pr_err("Unable to set FG access I2C address\n");
		goto restore_fg;
	}

	rc = smb1360_adjust_current_gain(chip, gain_factor);
	if (rc) {
		pr_err("Unable to modify current gain rc=%d\n", rc);
		goto restore_fg;
	}

	rc = smb1360_masked_write(chip, CFG_FG_BATT_CTRL_REG,
			CFG_FG_OTP_BACK_UP_ENABLE, CFG_FG_OTP_BACK_UP_ENABLE);
	if (rc) {
		pr_err("Write reg 0x0E failed, rc = %d\n", rc);
		goto restore_fg;
	}

restore_fg:
	rc = smb1360_disable_fg_access(chip);
	if (rc) {
		pr_err("Couldn't disable FG access rc = %d\n", rc);
		return rc;
	}

	return rc;
}

static int smb1360_otg_disable(struct smb1360_chip *chip)
{
	int rc;

	rc = smb1360_masked_write(chip, CMD_CHG_REG, CMD_OTG_EN_BIT, 0);
	if (rc) {
		pr_err("Couldn't disable OTG mode rc=%d\n", rc);
		return rc;
	}

	
	if (chip->otg_fet_present && chip->fet_gain_enabled) {
		
		gpio_set_value(chip->otg_fet_enable_gpio, 1);
		rc = smb1360_otp_gain_config(chip, 0);
		if (rc < 0)
			pr_err("Couldn't config OTP gain config rc=%d\n", rc);
		else
			chip->fet_gain_enabled = false;
	}

	return rc;
}

static int otg_fail_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	int rc;

	pr_debug("OTG Failed stat=%d\n", rt_stat);
	rc = smb1360_otg_disable(chip);
	if (rc)
		pr_err("Couldn't disable OTG mode rc=%d\n", rc);

	return 0;
}

static int otg_oc_handler(struct smb1360_chip *chip, u8 rt_stat)
{
	int rc;

	pr_debug("OTG over-current stat=%d\n", rt_stat);
	rc = smb1360_otg_disable(chip);
	if (rc)
		pr_err("Couldn't disable OTG mode rc=%d\n", rc);

	return 0;
}

struct smb_irq_info {
	const char		*name;
	int			(*smb_irq)(struct smb1360_chip *chip,
							u8 rt_stat);
	int			high;
	int			low;
};

struct irq_handler_info {
	u8			stat_reg;
	u8			val;
	u8			prev_val;
	struct smb_irq_info	irq_info[4];
};

static struct irq_handler_info handlers[] = {
	{IRQ_A_REG, 0, 0,
		{
			{
				.name		= "cold_soft",
				.smb_irq	= cold_soft_handler,
			},
			{
				.name		= "hot_soft",
				.smb_irq	= hot_soft_handler,
			},
			{
				.name		= "cold_hard",
				.smb_irq	= cold_hard_handler,
			},
			{
				.name		= "hot_hard",
				.smb_irq	= hot_hard_handler,
			},
		},
	},
	{IRQ_B_REG, 0, 0,
		{
			{
				.name		= "chg_hot",
				.smb_irq	= chg_hot_handler,
			},
			{
				.name		= "vbat_low",
				.smb_irq	= vbat_low_handler,
			},
			{
				.name		= "battery_missing",
				.smb_irq	= battery_missing_handler,
			},
			{
				.name		= "battery_missing",
				.smb_irq	= battery_missing_handler,
			},
		},
	},
	{IRQ_C_REG, 0, 0,
		{
			{
				.name		= "chg_term",
				.smb_irq	= chg_term_handler,
			},
			{
				.name		= "taper",
			},
			{
				.name		= "recharge",
				.smb_irq	= recharge_handler,
			},
			{
				.name		= "fast_chg",
				.smb_irq	= chg_fastchg_handler,
			},
		},
	},
	{IRQ_D_REG, 0, 0,
		{
			{
				.name		= "prechg_timeout",
			},
			{
				.name		= "safety_timeout",
				.smb_irq	= chg_safety_timeout_handler,
			},
			{
				.name		= "aicl_done",
				.smb_irq	= aicl_done_handler,
			},
			{
				.name		= "battery_ov",
			},
		},
	},
	{IRQ_E_REG, 0, 0,
		{
			{
				.name		= "usbin_uv",
				.smb_irq	= usbin_uv_handler,
			},
			{
				.name		= "usbin_ov",
				.smb_irq	= usbin_ov_handler,
			},
			{
				.name		= "unused",
			},
			{
				.name		= "chg_inhibit",
				.smb_irq	= chg_inhibit_handler,
			},
		},
	},
	{IRQ_F_REG, 0, 0,
		{
			{
				.name		= "power_ok",
				.smb_irq	= power_ok_handler,
			},
			{
				.name		= "unused",
			},
			{
				.name		= "otg_fail",
				.smb_irq	= otg_fail_handler,
			},
			{
				.name		= "otg_oc",
				.smb_irq	= otg_oc_handler,
			},
		},
	},
	{IRQ_G_REG, 0, 0,
		{
			{
				.name		= "delta_soc",
				.smb_irq	= delta_soc_handler,
			},
			{
				.name		= "chg_error",
				.smb_irq	= chg_error_handler,
			},
			{
				.name		= "wd_timeout",
				.smb_irq	= wd_timeout_handler,
			},
			{
				.name		= "unused",
			},
		},
	},
	{IRQ_H_REG, 0, 0,
		{
			{
				.name		= "min_soc",
				.smb_irq	= min_soc_handler,
			},
			{
				.name		= "max_soc",
			},
			{
				.name		= "empty_soc",
				.smb_irq	= empty_soc_handler,
			},
			{
				.name		= "full_soc",
				.smb_irq	= full_soc_handler,
			},
		},
	},
	{IRQ_I_REG, 0, 0,
		{
			{
				.name		= "fg_access_allowed",
				.smb_irq	= fg_access_allowed_handler,
			},
			{
				.name		= "fg_data_recovery",
			},
			{
				.name		= "batt_id_complete",
#ifndef CONFIG_HTC_BATT_8960
				.smb_irq	= batt_id_complete_handler,
#endif
			},
		},
	},
};

#define IRQ_LATCHED_MASK	0x02
#define IRQ_STATUS_MASK		0x01
#define BATT_ID_LATCHED_MASK	0x08
#define BATT_ID_STATUS_MASK	0x07
#define BITS_PER_IRQ		2
static irqreturn_t smb1360_stat_handler(int irq, void *dev_id)
{
	struct smb1360_chip *chip = dev_id;
	int i, j;
	u8 triggered;
	u8 changed;
	u8 rt_stat, prev_rt_stat, irq_latched_mask, irq_status_mask;
	int rc;
	int handler_count = 0;

	mutex_lock(&chip->irq_complete);
	chip->irq_waiting = true;
	if (!chip->resume_completed) {
		dev_dbg(chip->dev, "IRQ triggered before device-resume\n");
		disable_irq_nosync(irq);
		mutex_unlock(&chip->irq_complete);
		return IRQ_HANDLED;
	}
	chip->irq_waiting = false;

	for (i = 0; i < ARRAY_SIZE(handlers); i++) {
		rc = smb1360_read(chip, handlers[i].stat_reg,
					&handlers[i].val);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't read %d rc = %d\n",
					handlers[i].stat_reg, rc);
			continue;
		}

		for (j = 0; j < ARRAY_SIZE(handlers[i].irq_info); j++) {
			if (handlers[i].stat_reg == IRQ_I_REG && j == 2) {
				irq_latched_mask = BATT_ID_LATCHED_MASK;
				irq_status_mask = BATT_ID_STATUS_MASK;
			} else {
				irq_latched_mask = IRQ_LATCHED_MASK;
				irq_status_mask = IRQ_STATUS_MASK;
			}
			triggered = handlers[i].val
			       & (irq_latched_mask << (j * BITS_PER_IRQ));
			rt_stat = handlers[i].val
				& (irq_status_mask << (j * BITS_PER_IRQ));
			prev_rt_stat = handlers[i].prev_val
				& (irq_status_mask << (j * BITS_PER_IRQ));
			changed = prev_rt_stat ^ rt_stat;

			if (triggered || changed) {
				rt_stat ? handlers[i].irq_info[j].high++ :
						handlers[i].irq_info[j].low++;
				
				if (!((i == IRQ_C_REG_SEQUENCE && (irq_status_mask << (j * BITS_PER_IRQ)) == IRQ_C_TAPER_CHG_BIT)
						|| (i == IRQ_H_REG_SEQUENCE && (irq_status_mask << (j * BITS_PER_IRQ)) == IRQ_H_FULL_SOC_BIT))) {
					pr_info("IRQ 0x%02X reg =0x%02X, %s irq is triggered=0x%02X, rt_stat=0x%02X, changed=0x%02X\n",
						handlers[i].stat_reg, handlers[i].val,
						handlers[i].irq_info[j].name, triggered, rt_stat, changed);
				}
			}

			if ((triggered || changed)
				&& handlers[i].irq_info[j].smb_irq != NULL) {
				handler_count++;
				rc = handlers[i].irq_info[j].smb_irq(chip,
								rt_stat);
				if (rc < 0)
					dev_err(chip->dev,
						"Couldn't handle %d irq for reg 0x%02x rc = %d\n",
						j, handlers[i].stat_reg, rc);
			}
		}
		handlers[i].prev_val = handlers[i].val;
	}

	pr_debug("handler count = %d\n", handler_count);
#ifndef CONFIG_HTC_BATT_8960
	if (handler_count)
		power_supply_changed(&chip->batt_psy);
#endif

	mutex_unlock(&chip->irq_complete);

	return IRQ_HANDLED;
}

static int smb1360_chg_is_usb_chg_plugged_in(struct smb1360_chip *chip)
{
	return (handlers[IRQ_F_REG_SEQUENCE].val & IRQ_F_POWER_OK_BIT) ? true : false;
}

static int get_prop_usb_valid_status(struct smb1360_chip *chip,
					int *ov, int *v, int *uv)
{
	*v = (handlers[IRQ_F_REG_SEQUENCE].val & IRQ_F_POWER_OK_BIT) ? true : false;
	*ov = (handlers[IRQ_E_REG_SEQUENCE].val & IRQ_E_USBIN_OV_BIT) ? true : false;
	*uv = (handlers[IRQ_E_REG_SEQUENCE].val & IRQ_E_USBIN_UV_BIT) ? true : false;

	pr_debug("v=%d, ov=%d, uv=%d\n", *v, *ov, *uv);
	return 0;
}


static int show_irq_count(struct seq_file *m, void *data)
{
	int i, j, total = 0;

	for (i = 0; i < ARRAY_SIZE(handlers); i++)
		for (j = 0; j < 4; j++) {
			if (!handlers[i].irq_info[j].name)
				continue;
			seq_printf(m, "%s=%d\t(high=%d low=%d)\n",
						handlers[i].irq_info[j].name,
						handlers[i].irq_info[j].high
						+ handlers[i].irq_info[j].low,
						handlers[i].irq_info[j].high,
						handlers[i].irq_info[j].low);
			total += (handlers[i].irq_info[j].high
					+ handlers[i].irq_info[j].low);
		}

	seq_printf(m, "\n\tTotal = %d\n", total);

	return 0;
}

static int irq_count_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb1360_chip *chip = inode->i_private;

	return single_open(file, show_irq_count, chip);
}

static const struct file_operations irq_count_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= irq_count_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int get_reg(void *data, u64 *val)
{
	struct smb1360_chip *chip = data;
	int rc;
	u8 temp;

	rc = smb1360_read(chip, chip->peek_poke_address, &temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read reg %x rc = %d\n",
			chip->peek_poke_address, rc);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int set_reg(void *data, u64 val)
{
	struct smb1360_chip *chip = data;
	int rc;
	u8 temp;

	temp = (u8) val;
	rc = smb1360_write(chip, chip->peek_poke_address, temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't write 0x%02x to 0x%02x rc= %d\n",
			chip->peek_poke_address, temp, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(poke_poke_debug_ops, get_reg, set_reg, "0x%02llx\n");

static int fg_get_reg(void *data, u64 *val)
{
	struct smb1360_chip *chip = data;
	int rc;
	u8 temp;

	rc = smb1360_select_fg_i2c_address(chip);
	if (rc) {
		pr_err("Unable to set FG access I2C address\n");
		return -EINVAL;
	}

	rc = smb1360_fg_read(chip, chip->fg_peek_poke_address, &temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read reg %x rc = %d\n",
			chip->fg_peek_poke_address, rc);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int fg_set_reg(void *data, u64 val)
{
	struct smb1360_chip *chip = data;
	int rc;
	u8 temp;

	rc = smb1360_select_fg_i2c_address(chip);
	if (rc) {
		pr_err("Unable to set FG access I2C address\n");
		return -EINVAL;
	}

	temp = (u8) val;
	rc = smb1360_fg_write(chip, chip->fg_peek_poke_address, temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't write 0x%02x to 0x%02x rc= %d\n",
			chip->fg_peek_poke_address, temp, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fg_poke_poke_debug_ops, fg_get_reg,
				fg_set_reg, "0x%02llx\n");

#define LAST_CNFG_REG	0x17
static int show_cnfg_regs(struct seq_file *m, void *data)
{
	struct smb1360_chip *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
		rc = smb1360_read(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cnfg_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb1360_chip *chip = inode->i_private;

	return single_open(file, show_cnfg_regs, chip);
}

static const struct file_operations cnfg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cnfg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_CMD_REG	0x40
#define LAST_CMD_REG	0x42
static int show_cmd_regs(struct seq_file *m, void *data)
{
	struct smb1360_chip *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_CMD_REG; addr <= LAST_CMD_REG; addr++) {
		rc = smb1360_read(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cmd_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb1360_chip *chip = inode->i_private;

	return single_open(file, show_cmd_regs, chip);
}

static const struct file_operations cmd_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cmd_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_STATUS_REG	0x48
#define LAST_STATUS_REG		0x4B
static int show_status_regs(struct seq_file *m, void *data)
{
	struct smb1360_chip *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_STATUS_REG; addr <= LAST_STATUS_REG; addr++) {
		rc = smb1360_read(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int status_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb1360_chip *chip = inode->i_private;

	return single_open(file, show_status_regs, chip);
}

static const struct file_operations status_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= status_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_IRQ_REG		0x50
#define LAST_IRQ_REG		0x58
static int show_irq_stat_regs(struct seq_file *m, void *data)
{
	struct smb1360_chip *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_IRQ_REG; addr <= LAST_IRQ_REG; addr++) {
		rc = smb1360_read(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int irq_stat_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb1360_chip *chip = inode->i_private;

	return single_open(file, show_irq_stat_regs, chip);
}

static const struct file_operations irq_stat_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= irq_stat_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int data_8(u8 *reg)
{
	return reg[0];
}
static int data_16(u8 *reg)
{
	return (reg[1] << 8) | reg[0];
}
static int data_24(u8 *reg)
{
	return  (reg[2] << 16) | (reg[1] << 8) | reg[0];
}
static int data_28(u8 *reg)
{
	return  ((reg[3] & 0xF) << 24) | (reg[2] << 16) |
					(reg[1] << 8) | reg[0];
}
static int data_32(u8 *reg)
{
	return  (reg[3]  << 24) | (reg[2] << 16) |
				(reg[1] << 8) | reg[0];
}

struct fg_regs {
	int index;
	int length;
	char *param_name;
	int (*calc_func) (u8 *);
};

static struct fg_regs fg_scratch_pad[] = {
	{0, 2, "v_current_predicted", data_16},
	{2, 2, "v_cutoff_predicted", data_16},
	{4, 2, "v_full_predicted", data_16},
	{6, 2, "ocv_estimate", data_16},
	{8, 2, "rslow_drop", data_16},
	{10, 2, "voltage_old", data_16},
	{12, 2, "current_old", data_16},
	{14, 4, "current_average_full", data_32},
	{18, 2, "temperature", data_16},
	{20, 2, "temp_last_track", data_16},
	{22, 2, "ESR_nominal", data_16},
	{26, 2, "Rslow", data_16},
	{28, 2, "counter_imptr", data_16},
	{30, 2, "counter_pulse", data_16},
	{32, 1, "IRQ_delta_prev", data_8},
	{33, 1, "cap_learning_counter", data_8},
	{34, 4, "Vact_int_error", data_32},
	{38, 3, "SOC_cutoff", data_24},
	{41, 3, "SOC_full", data_24},
	{44, 3, "SOC_auto_rechrge_temp", data_24},
	{47, 3, "Battery_SOC", data_24},
	{50, 4, "CC_SOC", data_28},
	{54, 2, "SOC_filtered", data_16},
	{56, 2, "SOC_Monotonic", data_16},
	{58, 2, "CC_SOC_coeff", data_16},
	{60, 2, "nominal_capacity", data_16},
	{62, 2, "actual_capacity", data_16},
	{68, 1, "temperature_counter", data_8},
	{69, 3, "Vbatt_filtered", data_24},
	{72, 3, "Ibatt_filtered", data_24},
	{75, 2, "Current_CC_shadow", data_16},
	{79, 2, "Ibatt_standby", data_16},
	{82, 1, "Auto_recharge_SOC_threshold", data_8},
	{83, 2, "System_cutoff_voltage", data_16},
	{85, 2, "System_CC_to_CV_voltage", data_16},
	{87, 2, "System_term_current", data_16},
	{89, 2, "System_fake_term_current", data_16},
	{91, 2, "thermistor_c1_coeff", data_16},
};

static struct fg_regs fg_cfg[] = {
	{0, 2, "ESR_actual", data_16},
	{4, 1, "IRQ_SOC_max", data_8},
	{5, 1, "IRQ_SOC_min", data_8},
	{6, 1, "IRQ_volt_empty", data_8},
	{7, 1, "Temp_external", data_8},
	{8, 1, "IRQ_delta_threshold", data_8},
	{9, 1, "JIETA_soft_cold", data_8},
	{10, 1, "JIETA_soft_hot", data_8},
	{11, 1, "IRQ_volt_min", data_8},
	{14, 2, "ESR_sys_replace", data_16},
};

static struct fg_regs fg_shdw[] = {
	{0, 1, "Latest_battery_info", data_8},
	{1, 1, "Latest_Msys_SOC", data_8},
	{2, 2, "Battery_capacity", data_16},
	{4, 2, "Rslow_drop", data_16},
	{6, 1, "Latest_SOC", data_8},
	{7, 1, "Latest_Cutoff_SOC", data_8},
	{8, 1, "Latest_full_SOC", data_8},
	{9, 2, "Voltage_shadow", data_16},
	{11, 2, "Current_shadow", data_16},
	{13, 2, "Latest_temperature", data_16},
	{15, 1, "Latest_system_sbits", data_8},
};

#define FIRST_FG_CFG_REG		0x20
#define LAST_FG_CFG_REG			0x2F
#define FIRST_FG_SHDW_REG		0x60
#define LAST_FG_SHDW_REG		0x6F
#define FG_SCRATCH_PAD_MAX		93
#define FG_SCRATCH_PAD_BASE_REG		0x80
#define SMB1360_I2C_READ_LENGTH		32

static int smb1360_check_cycle_stretch(struct smb1360_chip *chip)
{
	int rc = 0;
	u8 reg;

	rc = smb1360_read(chip, STATUS_4_REG, &reg);
	if (rc) {
		pr_err("Unable to read status regiseter\n");
	} else if (reg & CYCLE_STRETCH_ACTIVE_BIT) {
		
		rc = smb1360_masked_write(chip, CMD_I2C_REG,
			CYCLE_STRETCH_CLEAR_BIT, CYCLE_STRETCH_CLEAR_BIT);
		if (rc)
			pr_err("Unable to clear cycle stretch\n");
	}

	return rc;
}

static int show_fg_regs(struct seq_file *m, void *data)
{
	struct smb1360_chip *chip = m->private;
	int rc, i , j, rem_length;
	u8 reg[FG_SCRATCH_PAD_MAX];

	rc = smb1360_check_cycle_stretch(chip);
	if (rc)
		pr_err("Unable to check cycle-stretch\n");

	rc = smb1360_enable_fg_access(chip);
	if (rc) {
		pr_err("Couldn't request FG access rc=%d\n", rc);
		return rc;
	}

	for (i = 0; i < (FG_SCRATCH_PAD_MAX / SMB1360_I2C_READ_LENGTH); i++) {
		j = i * SMB1360_I2C_READ_LENGTH;
		rc = smb1360_read_bytes(chip, FG_SCRATCH_PAD_BASE_REG + j,
					&reg[j], SMB1360_I2C_READ_LENGTH);
		if (rc) {
			pr_err("Couldn't read scratch registers rc=%d\n", rc);
			break;
		}
	}

	j = i * SMB1360_I2C_READ_LENGTH;
	rem_length = (FG_SCRATCH_PAD_MAX % SMB1360_I2C_READ_LENGTH);
	if (rem_length) {
		rc = smb1360_read_bytes(chip, FG_SCRATCH_PAD_BASE_REG + j,
						&reg[j], rem_length);
		if (rc)
			pr_err("Couldn't read scratch registers rc=%d\n", rc);
	}

	rc = smb1360_disable_fg_access(chip);
	if (rc) {
		pr_err("Couldn't disable FG access rc=%d\n", rc);
		return rc;
	}

	rc = smb1360_check_cycle_stretch(chip);
	if (rc)
		pr_err("Unable to check cycle-stretch\n");


	seq_puts(m, "FG scratch-pad registers\n");
	for (i = 0; i < ARRAY_SIZE(fg_scratch_pad); i++)
		seq_printf(m, "\t%s = %x\n", fg_scratch_pad[i].param_name,
		fg_scratch_pad[i].calc_func(&reg[fg_scratch_pad[i].index]));

	rem_length = LAST_FG_CFG_REG - FIRST_FG_CFG_REG + 1;
	rc = smb1360_read_bytes(chip, FIRST_FG_CFG_REG,
					&reg[0], rem_length);
	if (rc)
		pr_err("Couldn't read config registers rc=%d\n", rc);

	seq_puts(m, "FG config registers\n");
	for (i = 0; i < ARRAY_SIZE(fg_cfg); i++)
		seq_printf(m, "\t%s = %x\n", fg_cfg[i].param_name,
				fg_cfg[i].calc_func(&reg[fg_cfg[i].index]));

	rem_length = LAST_FG_SHDW_REG - FIRST_FG_SHDW_REG + 1;
	rc = smb1360_read_bytes(chip, FIRST_FG_SHDW_REG,
					&reg[0], rem_length);
	if (rc)
		pr_err("Couldn't read shadow registers rc=%d\n", rc);

	seq_puts(m, "FG shadow registers\n");
	for (i = 0; i < ARRAY_SIZE(fg_shdw); i++)
		seq_printf(m, "\t%s = %x\n", fg_shdw[i].param_name,
				fg_shdw[i].calc_func(&reg[fg_shdw[i].index]));

	return rc;
}

static int fg_regs_open(struct inode *inode, struct file *file)
{
	struct smb1360_chip *chip = inode->i_private;

	return single_open(file, show_fg_regs, chip);
}

static const struct file_operations fg_regs_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= fg_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int smb1360_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smb1360_chip *chip = rdev_get_drvdata(rdev);

	rc = smb1360_masked_write(chip, CMD_CHG_REG, CMD_OTG_EN_BIT,
						CMD_OTG_EN_BIT);
	if (rc) {
		pr_err("Couldn't enable  OTG mode rc=%d\n", rc);
		return rc;
	}

	
	if (chip->otg_fet_present) {
		
		gpio_set_value(chip->otg_fet_enable_gpio, 0);
		rc = smb1360_otp_gain_config(chip, 3);
		if (rc < 0)
			pr_err("Couldn't config OTP gain config rc=%d\n", rc);
		else
			chip->fet_gain_enabled = true;
	}

	pr_debug("OTG mode enabled\n");
	return rc;
}

static int smb1360_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smb1360_chip *chip = rdev_get_drvdata(rdev);

	rc = smb1360_otg_disable(chip);
	if (rc)
		pr_err("Couldn't disable OTG regulator rc=%d\n", rc);

	pr_debug("OTG mode disabled\n");
	return rc;
}

static int smb1360_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	u8 reg = 0;
	int rc = 0;
	struct smb1360_chip *chip = rdev_get_drvdata(rdev);

	rc = smb1360_read(chip, CMD_CHG_REG, &reg);
	if (rc) {
		pr_err("Couldn't read OTG enable bit rc=%d\n", rc);
		return rc;
	}

	return  (reg & CMD_OTG_EN_BIT) ? 1 : 0;
}

struct regulator_ops smb1360_otg_reg_ops = {
	.enable		= smb1360_otg_regulator_enable,
	.disable	= smb1360_otg_regulator_disable,
	.is_enabled	= smb1360_otg_regulator_is_enable,
};

#ifdef CONFIG_HTC_BATT_8960
static int smb1360_get_batt_id_info(struct smb1360_chip *chip,
								int *batt_id, int *batt_id_mv)
{
	u8 val;
	int64_t batt_id_uv;
	struct qpnp_vadc_result result;
	int rc;

	*batt_id = 0;
	*batt_id_mv = 0;

	rc = smb1360_read_bytes(chip, IRQ_I_REG, &val, sizeof(val));
	if (rc) {
		pr_err("Couldn't read IRQ_I_REG rc=%d\n", rc);
		return rc;
	}

	val &= BATT_ID_RESULT_BIT;
	val >>= BATT_ID_SHIFT;

	
	if (val != A51_BATT_ID_RESULT_ERR) {
		*batt_id = 1; 
		return rc;
	}

	if (IS_ERR(chip->vadc_dev)) {
		rc = PTR_ERR(chip->vadc_dev);
		if (rc == -EPROBE_DEFER)
			pr_err("vadc not found - defer rc=%d\n", rc);
		else
			pr_err("vadc property missing, rc=%d\n", rc);

		return rc;
	}

	
	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX2_BAT_ID, &result);
	if (rc) {
		pr_err("error reading batt id channel = %d, rc = %d\n",
					LR_MUX2_BAT_ID, rc);
		return rc;
	}

	batt_id_uv = result.physical;

	*batt_id_mv = (int)batt_id_uv / 1000;
	*batt_id = htc_battery_cell_find_and_set_id_auto((int)batt_id_uv / 1000);

	return rc;
}

static int smb1360_get_usbin(struct smb1360_chip *chip)
{
	struct qpnp_vadc_result result;
	int usbin = 0, rc;

	if (IS_ERR(chip->vadc_dev)) {
		rc = PTR_ERR(chip->vadc_dev);
		if (rc == -EPROBE_DEFER)
			pr_err("vadc not found - defer rc=%d\n", rc);
		else
			pr_err("vadc property missing, rc=%d\n", rc);

		return rc;
	}

	
	rc = qpnp_vadc_read(chip->vadc_dev, USBIN, &result);
	if (rc) {
		pr_err("error reading USBIN channel = %d, rc = %d\n",
					USBIN, rc);
	} else {
		usbin = (int)result.physical;
	}

	return (rc) ? rc : usbin;
}

static int smb1360_get_input_current_limit(struct smb1360_chip *chip)
{
	u8 val;
	int input_curr_lmt = 0;
	int rc;

	rc = smb1360_read_bytes(chip, CFG_BATT_CHG_ICL_REG, &val, 1);
	if (rc)
		pr_err("Couldn't read ICL mA rc=%d\n", rc);
	else {
		val &= INPUT_CURR_LIM_MASK;
		input_curr_lmt = input_current_limit[val];
	}

	return input_curr_lmt;
}

static int smb1360_get_input_current_limit_status(struct smb1360_chip *chip)
{
	u8 val;
	int input_curr_lmt_status = 0;
	int rc;

	rc = smb1360_read_bytes(chip, 0x48, &val, 1);
	if (rc)
		pr_err("Couldn't read ICL mA rc=%d\n", rc);
	else {
		val &= INPUT_CURR_LIM_MASK;
		input_curr_lmt_status = input_current_limit[val];
	}

	return input_curr_lmt_status;
}

static int smb1360_get_fast_chg_current_limit(struct smb1360_chip *chip)
{
	u8 val;
	int fastchg_curr_limit = 0;
	int rc;

	rc = smb1360_read_bytes(chip, CHG_CURRENT_REG, &val, 1);
	if (rc)
		pr_err("Couldn't read fastchg mA rc=%d\n", rc);
	else {
		val &= FASTCHG_CURR_MASK;
		val >>= FASTCHG_CURR_SHIFT;
		fastchg_curr_limit = fastchg_current[val];
	}

	return fastchg_curr_limit;
}

static int set_battery_data(struct smb1360_chip *chip)
{
	int rc = 0;
	struct smb1360_battery_data *batt_data;
	struct device_node *node;

	node = of_find_node_by_name(chip->dev->of_node,
			"qcom,battery-data");
	if (!node) {
		pr_err("No available batterydata\n");
		return -EINVAL;
	}

	batt_data = devm_kzalloc(chip->dev,
			sizeof(struct smb1360_battery_data), GFP_KERNEL);
	if (!batt_data) {
		pr_err("Could not alloc battery data\n");
		return -EINVAL;
	}

	batt_data->vth_sf_lut = devm_kzalloc(chip->dev,
			sizeof(struct single_row_lut),
			GFP_KERNEL);

	batt_data->rbatt_temp_lut = devm_kzalloc(chip->dev,
			sizeof(struct single_row_lut),
			GFP_KERNEL);

	rc = of_batterydata_read_data_by_id_result(node, batt_data, chip->batt_id);

	if (rc == 0
			&& batt_data->vth_sf_lut
			&& batt_data->rbatt_temp_lut) {
		chip->vth_sf_lut = batt_data->vth_sf_lut;
		chip->rbatt_temp_lut = batt_data->rbatt_temp_lut;
		devm_kfree(chip->dev, batt_data);
	} else {
		pr_err("battery data load failed\n");
		devm_kfree(chip->dev, batt_data->vth_sf_lut);
		devm_kfree(chip->dev, batt_data->rbatt_temp_lut);
		devm_kfree(chip->dev, batt_data);
		return -EINVAL;
	}

	return 0;
}

static void dump_reg(void)
{
	static u8 chgval[26];
	static u8 cmdval[2];
	static u8 statval[5];
	static u8 partval[2];
	static u8 fgval[5];

	memset(chgval, 0, sizeof(chgval));
	memset(statval, 0, sizeof(statval));
	memset(partval, 0, sizeof(partval));
	memset(fgval, 0, sizeof(fgval));

	
	smb1360_read_bytes(the_chip, CFG_BATT_CHG_REG, chgval, sizeof(chgval));
	
	smb1360_read_bytes(the_chip, CMD_IL_REG, cmdval, sizeof(cmdval));
	
	smb1360_read_bytes(the_chip, FIRST_STATUS_REG, statval, sizeof(statval));
	
	smb1360_read_bytes(the_chip, 0x4D, partval, sizeof(partval));

	pr_info("ChgReg:00h~19h,"
		"[00h]=[0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X],"
		"[08h]=[0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X],"
		"[10h]=[0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X],"
		"[18h]=[0x%02X,0x%02X], "
		"CmdReg:41h~42h=[0x%02X,0x%02X], "
		"StatusReg:48h~4Ch=[0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X], "
		"PartID:4Dh=[0x%02X],4Eh=[0x%02X]\n",
		chgval[0x0], chgval[0x1], chgval[0x2], chgval[0x3], chgval[0x4], chgval[0x5], chgval[0x6], chgval[0x7], 
		chgval[0x8], chgval[0x9], chgval[0xA], chgval[0xB], chgval[0xC], chgval[0xD], chgval[0xE], chgval[0xF], 
		chgval[0x10], chgval[0x11], chgval[0x12], chgval[0x13], chgval[0x14], chgval[0x15], chgval[0x16], chgval[0x17], 
		chgval[0x18], chgval[0x19],
		cmdval[0], cmdval[1],
		statval[0], statval[1], statval[2], statval[3], statval[4],
		partval[0], partval[1]);
}

static void dump_irq(void)
{
	pr_info("IRQreg:50h~58h,"
		"[0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X]\n",
		handlers[0].val, handlers[1].val, handlers[2].val, handlers[3].val,
		handlers[4].val, handlers[5].val, handlers[6].val, handlers[7].val,
		handlers[8].val);
}

static void dump_all(void)
{
	int vbat_mv, ibat_ma, tbat_deg, soc, batt_id, batt_id_mv;
	int health, present, charger_type, status;
	int ac_online=0, vbus_online, iusb_max_ma, ibat_max_ma;
	int temp_fault = 0, host_mode;
	int usbin = 0;

	vbat_mv = smb1360_get_prop_voltage_now(the_chip);
	ibat_ma = smb1360_get_prop_current_now(the_chip) / 1000;
	tbat_deg = smb1360_get_prop_batt_temp(the_chip) / 10;
	soc = smb1360_get_prop_batt_capacity(the_chip);
	smb1360_get_batt_id_info(the_chip, &batt_id, &batt_id_mv);
	health = smb1360_get_prop_batt_health(the_chip);
	present = smb1360_get_prop_batt_present(the_chip);
	charger_type = smb1360_get_prop_charge_type(the_chip);
	status = smb1360_get_prop_batt_status(the_chip);
	ac_online = is_ac_online();
	vbus_online = smb1360_chg_is_usb_chg_plugged_in(the_chip);
	iusb_max_ma = smb1360_get_input_current_limit(the_chip);
	ibat_max_ma = smb1360_get_fast_chg_current_limit(the_chip);
	smb1360_is_batt_temperature_fault(&temp_fault);
	host_mode = smb1360_otg_regulator_is_enable(the_chip->otg_vreg.rdev);
	usbin = smb1360_get_usbin(the_chip);

	pr_info("V=%d mV,I=%d mA,T=%d C,SoC=%d%%,id=%d,id_mv=%d mV,"
			"H=%d,P=%d,CHG=%d,S=%d,"
			"ac_online=%d,vbus_online=%d,iusb_max_ma=%d,ibat_max=%d,"
			"OVP=%d,UVP=%d,TM=%d,"
			"eoc_count/by_curr/dischg_count=%d/%d/%d,is_ac_ST=%d,is_full=%d,"
			"temp_fault=%d,batt_hot/warm/cool/cold=%d/%d/%d/%d,"
			"flag=%d%d%d%d,"
			"host=%d,"
			"fastchg=%d,taperchg=%d, usbin=%d\n",
			vbat_mv, ibat_ma, tbat_deg, soc, batt_id, batt_id_mv,
			health, present, charger_type, status,
			ac_online, vbus_online, iusb_max_ma, ibat_max_ma,
			ovp, uvp, the_chip->therm_lvl_sel,
			eoc_count, eoc_count_by_curr, dischg_count, is_ac_safety_timeout, is_batt_full,
			temp_fault, the_chip->batt_hot, the_chip->batt_warm, the_chip->batt_cool, the_chip->batt_cold,
			flag_keep_charge_on, flag_pa_recharge, flag_enable_fast_charge, the_chip->charging_disabled,
			host_mode,
			smb1360_is_fastchg_on(the_chip), smb1360_chg_is_taper(the_chip), usbin);

	dump_reg();
	dump_irq();
}
#endif
static int smb1360_regulator_init(struct smb1360_chip *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(chip->dev, chip->dev->of_node);
	if (!init_data) {
		dev_err(chip->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &smb1360_otg_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = chip->dev->of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->otg_vreg.rdev = regulator_register(
					&chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"OTG reg failed, rc=%d\n", rc);
		}
	}

	return rc;
}

static int smb1360_check_batt_profile(struct smb1360_chip *chip)
{
#ifndef CONFIG_HTC_BATT_8960
	int rc, i, timeout = 50;
#else
	int rc, timeout = 50;
#endif
	u8 reg = 0, loaded_profile, new_profile = 0, bid_mask;

#ifndef CONFIG_HTC_BATT_8960
	if (!chip->connected_rid) {
		pr_debug("Skip batt-profile loading connected_rid=%d\n",
						chip->connected_rid);
		return 0;
	}
#endif

	rc = smb1360_read(chip, SHDW_FG_BATT_STATUS, &reg);
	if (rc) {
		pr_err("Couldn't read FG_BATT_STATUS rc=%d\n", rc);
		goto fail_profile;
	}

	loaded_profile = !!(reg & BATTERY_PROFILE_BIT) ?
			BATTERY_PROFILE_B : BATTERY_PROFILE_A;

	pr_info("fg_batt_status=%x loaded_profile=%d\n", reg, loaded_profile);

#ifndef CONFIG_HTC_BATT_8960
	for (i = 0; i < BATTERY_PROFILE_MAX; i++) {
		pr_debug("profile=%d profile_rid=%d connected_rid=%d\n", i,
						chip->profile_rid[i],
						chip->connected_rid);
		if (abs(chip->profile_rid[i] - chip->connected_rid) <
				(div_u64(chip->connected_rid, 10)))
			break;
	}

	if (i == BATTERY_PROFILE_MAX) {
		pr_err("None of the battery-profiles match the connected-RID\n");
		return 0;
	} else {
		if (i == loaded_profile) {
			pr_debug("Loaded Profile-RID == connected-RID\n");
			return 0;
		} else {
			new_profile = (loaded_profile == BATTERY_PROFILE_A) ?
					BATTERY_PROFILE_B : BATTERY_PROFILE_A;
			bid_mask = (new_profile == BATTERY_PROFILE_A) ?
					BATT_PROFILEA_MASK : BATT_PROFILEB_MASK;
			pr_info("Loaded Profile-RID != connected-RID, switch-profile old_profile=%d new_profile=%d\n",
						loaded_profile, new_profile);
		}
	}
#else
	switch(chip->batt_id) {
		case 2:
			new_profile = BATTERY_PROFILE_B;
			bid_mask = BATT_PROFILEB_MASK;
			break;
		case 255:
			pr_info("batt id is UNKNOWN\n");
		case 1:
		default:
			new_profile = BATTERY_PROFILE_A;
			bid_mask = BATT_PROFILEA_MASK;
			break;
	}

	if (new_profile == loaded_profile) {
		pr_info("Loaded Profile-RID == connected-RID\n");
		return 0;
	} else
		pr_info("Loaded Profile-RID != connected-RID, switch-profile old_profile=%d new_profile=%d\n",
				loaded_profile, new_profile);
#endif 

	
	rc = smb1360_masked_write(chip, CFG_FG_BATT_CTRL_REG,
				BATT_PROFILE_SELECT_MASK, bid_mask);
	if (rc) {
		pr_err("Couldn't reset battery-profile rc=%d\n", rc);
		goto fail_profile;
	}

	
	rc = smb1360_masked_write(chip, CMD_I2C_REG, FG_ACCESS_ENABLED_BIT,
							FG_ACCESS_ENABLED_BIT);
	if (rc) {
		pr_err("Couldn't enable FG access rc=%d\n", rc);
		goto fail_profile;
	}

	while (timeout) {
		
		msleep(100);
		rc = smb1360_read(chip, IRQ_I_REG, &reg);
		if (rc) {
			pr_err("Could't read IRQ_I_REG rc=%d\n", rc);
			goto restore_fg;
		}
		if (reg & FG_ACCESS_ALLOWED_BIT)
			break;
		timeout--;
	}
	if (!timeout) {
		pr_err("FG access timed-out\n");
		rc = -EAGAIN;
		goto restore_fg;
	}

	
	msleep(1500);

	
	rc = smb1360_masked_write(chip, CMD_I2C_REG, FG_RESET_BIT,
						FG_RESET_BIT);
	if (rc) {
		pr_err("Couldn't reset FG rc=%d\n", rc);
		goto restore_fg;
	}

	
	rc = smb1360_masked_write(chip, CMD_I2C_REG, FG_RESET_BIT, 0);
	if (rc) {
		pr_err("Couldn't un-reset FG rc=%d\n", rc);
		goto restore_fg;
	}

	
	rc = smb1360_masked_write(chip, CMD_I2C_REG, FG_ACCESS_ENABLED_BIT, 0);
	if (rc) {
		pr_err("Couldn't disable FG access rc=%d\n", rc);
		goto restore_fg;
	}

	timeout = 10;
	while (timeout) {
		
		msleep(500);
		rc = smb1360_read(chip, SHDW_FG_BATT_STATUS, &reg);
		if (rc) {
			pr_err("Could't read FG_BATT_STATUS rc=%d\n", rc);
			goto restore_fg;
		}

		reg = !!(reg & BATTERY_PROFILE_BIT);
		if (reg == new_profile) {
			pr_info("New profile=%d loaded\n", new_profile);
			break;
		}
		timeout--;
	}

	if (!timeout) {
		pr_err("New profile could not be loaded\n");
		return -EBUSY;
	}

	return 0;

restore_fg:
	smb1360_masked_write(chip, CMD_I2C_REG, FG_ACCESS_ENABLED_BIT, 0);
fail_profile:
	return rc;
}

#define UPDATE_IRQ_STAT(irq_reg, value) \
		handlers[irq_reg - IRQ_A_REG].prev_val = value;

static int determine_initial_status(struct smb1360_chip *chip)
{
	int rc;
	u8 reg = 0;

	chip->batt_present = true;
	rc = smb1360_read(chip, IRQ_B_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read IRQ_B_REG rc = %d\n", rc);
		return rc;
	}
	UPDATE_IRQ_STAT(IRQ_B_REG, reg);

	if (reg & IRQ_B_BATT_TERMINAL_BIT || reg & IRQ_B_BATT_MISSING_BIT)
		chip->batt_present = false;

	rc = smb1360_read(chip, IRQ_C_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read IRQ_C_REG rc = %d\n", rc);
		return rc;
	}
	UPDATE_IRQ_STAT(IRQ_C_REG, reg);

	if (reg & IRQ_C_CHG_TERM)
		chip->batt_full = true;

	rc = smb1360_read(chip, IRQ_A_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read irq A rc = %d\n", rc);
		return rc;
	}
	UPDATE_IRQ_STAT(IRQ_A_REG, reg);

	if (reg & IRQ_A_HOT_HARD_BIT)
		chip->batt_hot = true;
	if (reg & IRQ_A_COLD_HARD_BIT)
		chip->batt_cold = true;
	if (reg & IRQ_A_HOT_SOFT_BIT)
		chip->batt_warm = true;
	if (reg & IRQ_A_COLD_SOFT_BIT)
		chip->batt_cool = true;

	rc = smb1360_read(chip, IRQ_E_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read irq E rc = %d\n", rc);
		return rc;
	}
	UPDATE_IRQ_STAT(IRQ_E_REG, reg);

#ifdef CONFIG_HTC_BATT_8960
	
	cable_detection_vbus_irq_handler();
#else
	chip->usb_present = (reg & IRQ_E_USBIN_UV_BIT) ? false : true;
	power_supply_set_present(chip->usb_psy, chip->usb_present);
#endif

	return 0;
}

static int smb1360_fg_config(struct smb1360_chip *chip)
{
	int rc = 0, temp, fcc_mah;
	u8 reg = 0, reg2[2];

	if (!(chip->workaround_flags & WRKRND_FG_CONFIG_FAIL)) {
		if (chip->delta_soc != -EINVAL) {
			reg = abs(((chip->delta_soc * MAX_8_BITS) / 100) - 1);
			pr_debug("delta_soc=%d reg=%x\n", chip->delta_soc, reg);
			rc = smb1360_write(chip, SOC_DELTA_REG, reg);
			if (rc) {
				dev_err(chip->dev, "Couldn't write to SOC_DELTA_REG rc=%d\n",
						rc);
				return rc;
			}
		}

		if (chip->soc_min != -EINVAL) {
			if (is_between(0, 100, chip->soc_min)) {
				reg = DIV_ROUND_UP(chip->soc_min * MAX_8_BITS,
									100);
				pr_info("soc_min=%d reg=%x\n",
						chip->soc_min, reg);
				rc = smb1360_write(chip, SOC_MIN_REG, reg);
				if (rc) {
					dev_err(chip->dev, "Couldn't write to SOC_MIN_REG rc=%d\n",
							rc);
					return rc;
				}
			}
		}

		if (chip->soc_max != -EINVAL) {
			if (is_between(0, 100, chip->soc_max)) {
				reg = DIV_ROUND_UP(chip->soc_max * MAX_8_BITS,
									100);
				pr_info("soc_max=%d reg=%x\n",
						chip->soc_max, reg);
				rc = smb1360_write(chip, SOC_MAX_REG, reg);
				if (rc) {
					dev_err(chip->dev, "Couldn't write to SOC_MAX_REG rc=%d\n",
							rc);
					return rc;
				}
			}
		}

		if (chip->voltage_min_mv != -EINVAL) {
			temp = (chip->voltage_min_mv - 2500) * MAX_8_BITS;
			reg = DIV_ROUND_UP(temp, 2500);
			pr_info("voltage_min=%d reg=%x\n",
					chip->voltage_min_mv, reg);
			rc = smb1360_write(chip, VTG_MIN_REG, reg);
			if (rc) {
				dev_err(chip->dev, "Couldn't write to VTG_MIN_REG rc=%d\n",
							rc);
				return rc;
			}
		}

		if (chip->voltage_empty_mv != -EINVAL) {
			temp = (chip->voltage_empty_mv - 2500) * MAX_8_BITS;
			reg = DIV_ROUND_UP(temp, 2500);
			pr_info("voltage_empty=%d reg=%x\n",
					chip->voltage_empty_mv, reg);
			rc = smb1360_write(chip, VTG_EMPTY_REG, reg);
			if (rc) {
				dev_err(chip->dev, "Couldn't write to VTG_EMPTY_REG rc=%d\n",
							rc);
				return rc;
			}
		}
	}

	
	if (chip->cool_bat_decidegc != -EINVAL) {
		reg = ZERO_DEGREE_BASE_REG + chip->cool_bat_decidegc;
		pr_info("batt cool=%d degreeC reg=%x\n",
					chip->cool_bat_decidegc, reg);
		smb1360_write(chip, JEITA_SOFT_COLD_REG, reg);
	}

	
	if (chip->warm_bat_decidegc != -EINVAL) {
		reg = ZERO_DEGREE_BASE_REG + chip->warm_bat_decidegc;
		pr_info("batt warm=%d degreeC reg=%x\n",
					chip->warm_bat_decidegc, reg);
		smb1360_write(chip, JEITA_SOFT_HOT_REG, reg);
	}

	
	if (chip->batt_capacity_mah != -EINVAL
		|| chip->v_cutoff_mv != -EINVAL
		|| chip->fg_iterm_ma != -EINVAL
		|| chip->fg_ibatt_standby_ma != -EINVAL
		|| chip->fg_thermistor_c1_coeff != -EINVAL
		|| chip->fg_cc_to_cv_mv != -EINVAL
		|| chip->fg_auto_recharge_soc != -EINVAL) {

		rc = smb1360_enable_fg_access(chip);
		if (rc) {
			pr_err("Couldn't enable FG access rc=%d\n", rc);
			return rc;
		}

		
		if (chip->batt_capacity_mah != -EINVAL) {
			rc = smb1360_read_bytes(chip, ACTUAL_CAPACITY_REG,
								reg2, 2);
			if (rc) {
				pr_err("Failed to read ACTUAL CAPACITY rc=%d\n",
									rc);
				goto disable_fg;
			}
			fcc_mah = (reg2[1] << 8) | reg2[0];
			if (fcc_mah == chip->batt_capacity_mah) {
				pr_debug("battery capacity correct\n");
			} else {
				
				reg2[1] =
					(chip->batt_capacity_mah & 0xFF00) >> 8;
				reg2[0] = (chip->batt_capacity_mah & 0xFF);
				rc = smb1360_write_bytes(chip,
					ACTUAL_CAPACITY_REG, reg2, 2);
				if (rc) {
					pr_err("Couldn't write batt-capacity rc=%d\n",
									rc);
					goto disable_fg;
				}
				rc = smb1360_write_bytes(chip,
					NOMINAL_CAPACITY_REG, reg2, 2);
				if (rc) {
					pr_err("Couldn't write batt-capacity rc=%d\n",
									rc);
					goto disable_fg;
				}

				
				if (chip->cc_soc_coeff != -EINVAL) {
					reg2[1] =
					(chip->cc_soc_coeff & 0xFF00) >> 8;
					reg2[0] = (chip->cc_soc_coeff & 0xFF);
					rc = smb1360_write_bytes(chip,
						CC_TO_SOC_COEFF, reg2, 2);
					if (rc) {
						pr_err("Couldn't write cc_soc_coeff rc=%d\n",
									rc);
						goto disable_fg;
					}
				}
			}
		}

		
		if (chip->v_cutoff_mv != -EINVAL) {
			temp = (u16) div_u64(chip->v_cutoff_mv * 0x7FFF, 5000);
			reg2[1] = (temp & 0xFF00) >> 8;
			reg2[0] = temp & 0xFF;
			pr_info("v_cutoff_mv=%dmV reg=%x%x\n",
					chip->v_cutoff_mv, reg2[1], reg2[0]);
			rc = smb1360_write_bytes(chip, FG_SYS_CUTOFF_V_REG,
								reg2, 2);
			if (rc) {
				pr_err("Couldn't write cutoff_mv rc=%d\n", rc);
				goto disable_fg;
			}
		}

		if (chip->fg_iterm_ma != -EINVAL) {
			int iterm = chip->fg_iterm_ma * -1;
			temp = (s16) div_s64(iterm * 0x7FFF, 2500);
			reg2[1] = (temp & 0xFF00) >> 8;
			reg2[0] = temp & 0xFF;
			rc = smb1360_write_bytes(chip, FG_ITERM_REG,
							reg2, 2);
			if (rc) {
				pr_err("Couldn't write fg_iterm rc=%d\n", rc);
				goto disable_fg;
			}
		}

		if (chip->fg_ibatt_standby_ma != -EINVAL) {
			int iterm = chip->fg_ibatt_standby_ma;
			temp = (u16) div_u64(iterm * 0x7FFF, 2500);
			reg2[1] = (temp & 0xFF00) >> 8;
			reg2[0] = temp & 0xFF;
			rc = smb1360_write_bytes(chip, FG_IBATT_STANDBY_REG,
								reg2, 2);
			if (rc) {
				pr_err("Couldn't write fg_iterm rc=%d\n", rc);
				goto disable_fg;
			}
		}

		
		if (chip->fg_cc_to_cv_mv != -EINVAL) {
			temp = (u16) div_u64(chip->fg_cc_to_cv_mv * 0x7FFF,
								5000);
			reg2[1] = (temp & 0xFF00) >> 8;
			reg2[0] = temp & 0xFF;
			rc = smb1360_write_bytes(chip, FG_CC_TO_CV_V_REG,
								reg2, 2);
			if (rc) {
				pr_err("Couldn't write cc_to_cv_mv rc=%d\n",
								rc);
				goto disable_fg;
			}
		}

		
		if (chip->fg_thermistor_c1_coeff != -EINVAL) {
			reg2[1] = (chip->fg_thermistor_c1_coeff & 0xFF00) >> 8;
			reg2[0] = (chip->fg_thermistor_c1_coeff & 0xFF);
			rc = smb1360_write_bytes(chip, FG_THERM_C1_COEFF_REG,
								reg2, 2);
			if (rc) {
				pr_err("Couldn't write thermistor_c1_coeff rc=%d\n",
							rc);
				goto disable_fg;
			}
		}

		
		if (chip->fg_auto_recharge_soc != -EINVAL) {
			rc = smb1360_masked_write(chip, CFG_CHG_FUNC_CTRL_REG,
						CHG_RECHG_THRESH_FG_SRC_BIT,
						CHG_RECHG_THRESH_FG_SRC_BIT);
			if (rc) {
				dev_err(chip->dev, "Couldn't write to CFG_CHG_FUNC_CTRL_REG rc=%d\n",
									rc);
				goto disable_fg;
			}

			reg = DIV_ROUND_UP(chip->fg_auto_recharge_soc *
							MAX_8_BITS, 100);
			pr_info("fg_auto_recharge_soc=%d reg=%x\n",
					chip->fg_auto_recharge_soc, reg);
			rc = smb1360_write(chip, FG_AUTO_RECHARGE_SOC, reg);
			if (rc) {
				dev_err(chip->dev, "Couldn't write to FG_AUTO_RECHARGE_SOC rc=%d\n",
									rc);
				goto disable_fg;
			}
		}

disable_fg:
		
		smb1360_disable_fg_access(chip);
	}

	return rc;
}

static void smb1360_check_feature_support(struct smb1360_chip *chip)
{

#ifndef CONFIG_HTC_BATT_8960
	if (is_usb100_broken(chip)) {
#else
	if (1) {
#endif
		pr_debug("USB100 is not supported\n");
		chip->workaround_flags |= WRKRND_USB100_FAIL;
	}

	if (chip->revision == SMB1360_REV_1) {
		pr_debug("FG config and Battery detection is not supported\n");
		chip->workaround_flags |=
			WRKRND_FG_CONFIG_FAIL | WRKRND_BATT_DET_FAIL;
	}
}

static int smb1360_enable(struct smb1360_chip *chip, bool enable)
{
	int rc = 0;
	u8 val = 0, shdn_cmd_polar;

	rc = smb1360_read(chip, SHDN_CTRL_REG, &val);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read 0x1A reg rc = %d\n", rc);
		return rc;
	}

	
	if (!(val & SHDN_CMD_USE_BIT)) {
		pr_debug("SMB not configured for CMD based shutdown\n");
		return 0;
	}

	shdn_cmd_polar = !!(val & SHDN_CMD_POLARITY_BIT);
	val = (shdn_cmd_polar ^ enable) ? SHDN_CMD_BIT : 0;

	pr_debug("enable=%d shdn_polarity=%d value=%d\n", enable,
						shdn_cmd_polar, val);

	rc = smb1360_masked_write(chip, CMD_IL_REG, SHDN_CMD_BIT, val);
	if (rc < 0)
		pr_err("Couldn't shutdown smb1360 rc = %d\n", rc);

	return rc;
}

static inline int smb1360_poweroff(struct smb1360_chip *chip)
{
	pr_info("power off smb1360\n");
	return smb1360_enable(chip, false);
}

static inline int smb1360_poweron(struct smb1360_chip *chip)
{
	pr_info("power on smb1360\n");
	return smb1360_enable(chip, true);
}

static int smb1360_hw_init(struct smb1360_chip *chip)
{
	int rc;
	int i;
	u8 reg, mask;

	smb1360_check_feature_support(chip);

	rc = smb1360_enable_volatile_writes(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't configure for volatile rc = %d\n",
				rc);
		return rc;
	}

	if (chip->shdn_after_pwroff) {
		rc = smb1360_poweron(chip);
		if (rc < 0) {
			pr_err("smb1360 power on failed\n");
			return rc;
		}
	}

	if (chip->rsense_10mohm) {
		rc = smb1360_otp_gain_config(chip, 2);
		if (rc < 0) {
			pr_err("Couldn't config OTP rc=%d\n", rc);
			return rc;
		}
	}

	if (chip->otg_fet_present) {
		
		rc = devm_gpio_request_one(chip->dev,
				chip->otg_fet_enable_gpio,
				GPIOF_OPEN_DRAIN | GPIOF_INIT_HIGH,
				"smb1360_otg_fet_gpio");
		if (rc) {
			pr_err("Unable to request gpio rc=%d\n", rc);
			return rc;
		}

		
		rc = smb1360_otp_gain_config(chip, 0);
		if (rc < 0) {
			pr_err("Couldn't config OTP gain rc=%d\n", rc);
			return rc;
		}
	}

	rc = smb1360_check_batt_profile(chip);
	if (rc)
		pr_err("Unable to modify battery profile\n");

	rc = smb1360_masked_write(chip, CFG_CHG_MISC_REG,
					CHG_EN_BY_PIN_BIT
					| CHG_EN_ACTIVE_LOW_BIT
					| PRE_TO_FAST_REQ_CMD_BIT,
					0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set CFG_CHG_MISC_REG rc=%d\n", rc);
		return rc;
	}

	
	rc = smb1360_masked_write(chip, CFG_BATT_CHG_ICL_REG,
					AC_INPUT_ICL_PIN_BIT
					| AC_INPUT_PIN_HIGH_BIT
					| RESET_STATE_USB_500,
					AC_INPUT_PIN_HIGH_BIT
					| RESET_STATE_USB_500);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set CFG_BATT_CHG_ICL_REG rc=%d\n",
				rc);
		return rc;
	}

	
	reg = AICL_ENABLED_BIT | INPUT_UV_GLITCH_FLT_20MS_BIT;
	rc = smb1360_masked_write(chip, CFG_GLITCH_FLT_REG, reg, reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set CFG_GLITCH_FLT_REG rc=%d\n",
				rc);
		return rc;
	}

	
	if (chip->vfloat_mv != -EINVAL) {
		rc = smb1360_float_voltage_set(chip, chip->vfloat_mv);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set float voltage rc = %d\n", rc);
			return rc;
		}
	}

	if (chip->iterm_ma != -EINVAL) {
		if (chip->iterm_disabled) {
			dev_err(chip->dev, "Error: Both iterm_disabled and iterm_ma set\n");
			return -EINVAL;
		} else {
			if (chip->rsense_10mohm)
				chip->iterm_ma /= 2;

			if (chip->iterm_ma < 25)
				reg = CHG_ITERM_25MA;
			else if (chip->iterm_ma > 200)
				reg = CHG_ITERM_200MA;
			else
				reg = DIV_ROUND_UP(chip->iterm_ma, 25) - 1;

			pr_info("iterm_ma= %dmA setting 0x%X\n", chip->iterm_ma, reg);
			rc = smb1360_masked_write(chip, CFG_BATT_CHG_REG,
						CHG_ITERM_MASK, reg);
			if (rc) {
				dev_err(chip->dev,
					"Couldn't set iterm rc = %d\n", rc);
				return rc;
			}

			rc = smb1360_masked_write(chip, CFG_CHG_MISC_REG,
					CHG_CURR_TERM_DIS_BIT, 0);
			if (rc) {
				dev_err(chip->dev,
					"Couldn't enable iterm rc = %d\n", rc);
				return rc;
			}
		}
	} else  if (chip->iterm_disabled) {
		rc = smb1360_masked_write(chip, CFG_CHG_MISC_REG,
						CHG_CURR_TERM_DIS_BIT,
						CHG_CURR_TERM_DIS_BIT);
		if (rc) {
			dev_err(chip->dev, "Couldn't set iterm rc = %d\n",
								rc);
			return rc;
		}
	}

	
	
	if (flag_keep_charge_on || flag_pa_recharge) {
		
		rc = smb1360_masked_write(chip, CFG_SFY_TIMER_CTRL_REG,
					SAFETY_TIME_DISABLE_BIT, SAFETY_TIME_DISABLE_BIT);
		if (rc < 0) {
			dev_err(chip->dev,
			"Couldn't disable safety timer rc = %d\n",
							rc);
			return rc;
		}
	} else if (chip->safety_time != -EINVAL) {
		if (chip->safety_time == 0) {
			
			rc = smb1360_masked_write(chip, CFG_SFY_TIMER_CTRL_REG,
			SAFETY_TIME_DISABLE_BIT, SAFETY_TIME_DISABLE_BIT);
			if (rc < 0) {
				dev_err(chip->dev,
				"Couldn't disable safety timer rc = %d\n",
								rc);
				return rc;
			}
		} else {
			for (i = 0; i < ARRAY_SIZE(chg_time); i++) {
				if (chip->safety_time <= chg_time[i]) {
					reg = i << SAFETY_TIME_MINUTES_SHIFT;
					break;
				}
			}

			pr_info("Safety_timer: %dmins, reg:0x%X\n", chg_time[i], reg);
			rc = smb1360_masked_write(chip, CFG_SFY_TIMER_CTRL_REG,
			SAFETY_TIME_DISABLE_BIT | SAFETY_TIME_MINUTES_MASK,
								reg);
			if (rc < 0) {
				dev_err(chip->dev,
					"Couldn't set safety timer rc = %d\n",
									rc);
				return rc;
			}
		}
	} else {
		
		rc = smb1360_masked_write(chip, CFG_SFY_TIMER_CTRL_REG,
					SAFETY_TIME_DISABLE_BIT, 0);
		if (rc < 0) {
			dev_err(chip->dev,
			"Couldn't enable default maximum safety timer rc = %d\n",
							rc);
			return rc;
		}
	}

	
	if (chip->resume_delta_mv != -EINVAL) {
		if (chip->recharge_disabled && chip->chg_inhibit_disabled) {
			dev_err(chip->dev, "Error: Both recharge_disabled and recharge_mv set\n");
			return -EINVAL;
		} else {
			rc = smb1360_recharge_threshold_set(chip,
						chip->resume_delta_mv);
			if (rc) {
				dev_err(chip->dev,
					"Couldn't set rechg thresh rc = %d\n",
									rc);
				return rc;
			}
		}
	}

	rc = smb1360_masked_write(chip, CFG_CHG_MISC_REG,
					CFG_AUTO_RECHG_DIS_BIT,
					chip->recharge_disabled ?
					CFG_AUTO_RECHG_DIS_BIT : 0);
	if (rc) {
		dev_err(chip->dev, "Couldn't set rechg-cfg rc = %d\n", rc);
		return rc;
	}
	rc = smb1360_masked_write(chip, CFG_CHG_MISC_REG,
					CFG_CHG_INHIBIT_EN_BIT,
					chip->chg_inhibit_disabled ?
					0 : CFG_CHG_INHIBIT_EN_BIT);
	if (rc) {
		dev_err(chip->dev, "Couldn't set chg_inhibit rc = %d\n", rc);
		return rc;
	}

	
	rc = smb1360_masked_write(chip, CFG_BATT_MISSING_REG,
				BATT_MISSING_SRC_THERM_BIT,
				BATT_MISSING_SRC_THERM_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set batt_missing config = %d\n",
									rc);
		return rc;
	}

	
	if (chip->client->irq) {
		mask = CHG_STAT_IRQ_ONLY_BIT
			| CHG_STAT_ACTIVE_HIGH_BIT
			| CHG_STAT_DISABLE_BIT
			| CHG_TEMP_CHG_ERR_BLINK_BIT;

		if (!chip->pulsed_irq)
			reg = CHG_STAT_IRQ_ONLY_BIT;
		else
			reg = CHG_TEMP_CHG_ERR_BLINK_BIT;
		rc = smb1360_masked_write(chip, CFG_STAT_CTRL_REG, mask, reg);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set irq config rc = %d\n",
					rc);
			return rc;
		}

		
		rc = smb1360_write(chip, IRQ_CFG_REG,
				IRQ_BAT_HOT_COLD_HARD_BIT
				| IRQ_BAT_HOT_COLD_SOFT_BIT
				| IRQ_INTERNAL_TEMPERATURE_BIT
				| IRQ_DCIN_UV_BIT);
		if (rc) {
			dev_err(chip->dev, "Couldn't set irq1 config rc = %d\n",
					rc);
			return rc;
		}

		rc = smb1360_write(chip, IRQ2_CFG_REG,
				IRQ2_SAFETY_TIMER_BIT
				| IRQ2_CHG_ERR_BIT
				| IRQ2_CHG_PHASE_CHANGE_BIT
				| IRQ2_POWER_OK_BIT
				| IRQ2_BATT_MISSING_BIT
				| IRQ2_VBAT_LOW_BIT);
		if (rc) {
			dev_err(chip->dev, "Couldn't set irq2 config rc = %d\n",
					rc);
			return rc;
		}

		rc = smb1360_write(chip, IRQ3_CFG_REG,
				IRQ3_SOC_CHANGE_BIT
				| IRQ3_SOC_MIN_BIT
				| IRQ3_SOC_MAX_BIT
				| IRQ3_SOC_EMPTY_BIT
				| IRQ3_SOC_FULL_BIT);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set irq3 enable rc = %d\n",
					rc);
			return rc;
		}
	}

	
	if (chip->batt_id_disabled) {
		mask = BATT_ID_ENABLED_BIT | CHG_BATT_ID_FAIL;
		reg = CHG_BATT_ID_FAIL;
		rc = smb1360_masked_write(chip, CFG_FG_BATT_CTRL_REG,
						mask, reg);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set batt_id_reg rc = %d\n",
					rc);
			return rc;
		}
	}

	
	if (chip->otg_batt_curr_limit != -EINVAL) {
		for (i = 0; i < ARRAY_SIZE(otg_curr_ma); i++) {
			if (otg_curr_ma[i] >= chip->otg_batt_curr_limit)
				break;
		}

		if (i == ARRAY_SIZE(otg_curr_ma))
			i = i - 1;

		pr_info("otg_current= %dmA setting 0x%X\n",
				chip->otg_batt_curr_limit, (i << OTG_CURRENT_SHIFT));
		rc = smb1360_masked_write(chip, CFG_BATT_CHG_REG,
						OTG_CURRENT_MASK,
						i << OTG_CURRENT_SHIFT);
		if (rc)
			pr_err("Couldn't set OTG current limit, rc = %d\n", rc);
	}

	
	mask = CHG_JEITA_DISABLED_BIT;
	reg = (flag_keep_charge_on || flag_pa_recharge) ? CHG_JEITA_DISABLED_BIT : 0;
	rc = smb1360_masked_write(chip, CFG_FG_BATT_CTRL_REG,
					mask, reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set batt_ctrl_reg rc = %d\n",
				rc);
		return rc;
	}

	
	
	rc = smb1360_masked_write(chip, TEMP_COMPEN_REG,
				TEMP_HOT_VOL_COMPEN_BIT, TEMP_HOT_VOL_COMPEN_BIT);

	
	rc = smb1360_masked_write(chip, TEMP_COMPEN_REG,
				TEMP_COLD_VOL_COMPEN_BIT |
				TEMP_HOT_CURR_COMPEN_BIT |
				TEMP_COLD_CURR_COMPEN_BIT , 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set batt temp compensation config = %d\n",
									rc);
		return rc;
	}

	rc = smb1360_fg_config(chip);
	if (rc < 0) {
		pr_err("Couldn't configure FG rc=%d\n", rc);
		return rc;
	}

	rc = smb1360_charging_disable(chip, USER, !!chip->charging_disabled);
	if (rc)
		dev_err(chip->dev, "Couldn't '%s' charging rc = %d\n",
			chip->charging_disabled ? "disable" : "enable", rc);

	return rc;
}

static int smb_parse_batt_id(struct smb1360_chip *chip)
{
#ifndef CONFIG_HTC_BATT_8960
	int rc = 0, rpull = 0, vref = 0;
	int64_t denom, batt_id_uv;
	struct device_node *node = chip->dev->of_node;
	struct qpnp_vadc_result result;
#else
	int rc = 0;
	int64_t batt_id_uv;
	struct qpnp_vadc_result result;
	u8 val;
#endif
	chip->vadc_dev = qpnp_get_vadc(chip->dev, "smb1360");
	if (IS_ERR(chip->vadc_dev)) {
		rc = PTR_ERR(chip->vadc_dev);
		if (rc == -EPROBE_DEFER)
			pr_err("vadc not found - defer rc=%d\n", rc);
		else
			pr_err("vadc property missing, rc=%d\n", rc);

		return rc;
	}

#ifndef CONFIG_HTC_BATT_8960
	rc = of_property_read_u32(node, "qcom,profile-a-rid-kohm",
						&chip->profile_rid[0]);
	if (rc < 0) {
		pr_err("Couldn't read profile-a-rid-kohm rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_u32(node, "qcom,profile-b-rid-kohm",
						&chip->profile_rid[1]);
	if (rc < 0) {
		pr_err("Couldn't read profile-b-rid-kohm rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_u32(node, "qcom,batt-id-vref-uv", &vref);
	if (rc < 0) {
		pr_err("Couldn't read batt-id-vref-uv rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_u32(node, "qcom,batt-id-rpullup-kohm", &rpull);
	if (rc < 0) {
		pr_err("Couldn't read batt-id-rpullup-kohm rc=%d\n", rc);
		return rc;
	}
#else
	rc = smb1360_read_bytes(chip, IRQ_I_REG, &val, sizeof(val));
	if (rc) {
		pr_err("Couldn't read IRQ_I_REG rc=%d\n", rc);
		return rc;
	}

	val &= BATT_ID_RESULT_BIT;
	val >>= BATT_ID_SHIFT;

	
	if (val != A51_BATT_ID_RESULT_ERR) {
		chip->batt_id = 1; 
		return rc;
	}
#endif 

	
	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX2_BAT_ID, &result);
	if (rc) {
		pr_err("error reading batt id channel = %d, rc = %d\n",
					LR_MUX2_BAT_ID, rc);
		return rc;
	}
	batt_id_uv = result.physical;
#ifndef CONFIG_HTC_BATT_8960
	if (batt_id_uv == 0) {
		
		pr_err("batt_id_uv = 0, batt-id grounded using same profile\n");
		return 0;
	}

	denom = div64_s64(vref * 1000000LL, batt_id_uv) - 1000000LL;
	if (denom == 0) {
		
		return 0;
	}
	chip->connected_rid = div64_s64(rpull * 1000000LL + denom/2, denom);

	pr_debug("batt_id_voltage = %lld, connected_rid = %d\n",
			batt_id_uv, chip->connected_rid);
#else
	chip->batt_id = htc_battery_cell_find_and_set_id_auto((int)batt_id_uv / 1000);
	pr_info("batt_id=%d, batt_id_mv=%d\n", chip->batt_id, (int)batt_id_uv / 1000);
#endif 

	return 0;
}

static int smb_parse_dt(struct smb1360_chip *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	chip->rsense_10mohm = of_property_read_bool(node, "qcom,rsense-10mhom");

	if (of_property_read_bool(node, "qcom,batt-profile-select")) {
		rc = smb_parse_batt_id(chip);
		if (rc < 0) {
			if (rc != -EPROBE_DEFER)
				pr_err("Unable to parse batt-id rc=%d\n", rc);
			return rc;
		}
	}

	chip->otg_fet_present = of_property_read_bool(node,
						"qcom,otg-fet-present");
	if (chip->otg_fet_present) {
		chip->otg_fet_enable_gpio = of_get_named_gpio(node,
						"qcom,otg-fet-enable-gpio", 0);
		if (!gpio_is_valid(chip->otg_fet_enable_gpio)) {
			if (chip->otg_fet_enable_gpio != -EPROBE_DEFER)
				pr_err("Unable to get OTG FET enable gpio=%d\n",
						chip->otg_fet_enable_gpio);
			return chip->otg_fet_enable_gpio;
		}
	}

	chip->pulsed_irq = of_property_read_bool(node, "qcom,stat-pulsed-irq");

#ifndef CONFIG_HTC_BATT_8960
	rc = of_property_read_u32(node, "qcom,float-voltage-mv",
						&chip->vfloat_mv);
	if (rc < 0)
		chip->vfloat_mv = -EINVAL;
#else
	rc = htc_battery_cell_set_cur_cell_by_id(chip->batt_id);
	if (rc)
		chip->vfloat_mv = 4200;
	else
		chip->vfloat_mv = htc_battery_cell_get_cur_cell()->voltage_max;
#endif 

	rc = of_property_read_u32(node, "qcom,charging-timeout",
						&chip->safety_time);
	if (rc < 0)
		chip->safety_time = -EINVAL;

	if (!rc && (chip->safety_time > chg_time[ARRAY_SIZE(chg_time) - 1])) {
		dev_err(chip->dev, "Bad charging-timeout %d\n",
						chip->safety_time);
		return -EINVAL;
	}

	rc = of_property_read_u32(node, "qcom,recharge-thresh-mv",
						&chip->resume_delta_mv);
	if (rc < 0)
		chip->resume_delta_mv = -EINVAL;

	chip->recharge_disabled = of_property_read_bool(node,
						"qcom,recharge-disabled");

	rc = of_property_read_u32(node, "qcom,iterm-ma", &chip->iterm_ma);
	if (rc < 0)
		chip->iterm_ma = -EINVAL;

	chip->iterm_disabled = of_property_read_bool(node,
						"qcom,iterm-disabled");

	chip->chg_inhibit_disabled = of_property_read_bool(node,
						"qcom,chg-inhibit-disabled");

	chip->charging_disabled = of_property_read_bool(node,
						"qcom,charging-disabled");

	chip->batt_id_disabled = of_property_read_bool(node,
						"qcom,batt-id-disabled");

	chip->shdn_after_pwroff = of_property_read_bool(node,
						"qcom,shdn-after-pwroff");

	chip->min_icl_usb100 = of_property_read_bool(node,
						"qcom,min-icl-100ma");

	if (of_find_property(node, "qcom,thermal-mitigation",
					&chip->thermal_levels)) {
		chip->thermal_mitigation = devm_kzalloc(chip->dev,
					chip->thermal_levels,
						GFP_KERNEL);

		if (chip->thermal_mitigation == NULL) {
			pr_err("thermal mitigation kzalloc() failed.\n");
			return -ENOMEM;
		}

		chip->thermal_levels /= sizeof(int);
		rc = of_property_read_u32_array(node,
				"qcom,thermal-mitigation",
				chip->thermal_mitigation, chip->thermal_levels);
		if (rc) {
			pr_err("Couldn't read threm limits rc = %d\n", rc);
			return rc;
		}
	}

	rc = of_property_read_u32(node, "qcom,warm-bat-mv", &chip->warm_bat_mv);
	if (rc < 0)
		chip->warm_bat_mv = -EINVAL;

	
	chip->empty_soc_disabled = of_property_read_bool(node,
						"qcom,empty-soc-disabled");

	rc = of_property_read_u32(node, "qcom,fg-delta-soc", &chip->delta_soc);
	if (rc < 0)
		chip->delta_soc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,fg-soc-max", &chip->soc_max);
	if (rc < 0)
		chip->soc_max = -EINVAL;

	rc = of_property_read_u32(node, "qcom,fg-soc-min", &chip->soc_min);
	if (rc < 0)
		chip->soc_min = -EINVAL;

	rc = of_property_read_u32(node, "qcom,fg-voltage-min-mv",
					&chip->voltage_min_mv);
	if (rc < 0)
		chip->voltage_min_mv = -EINVAL;

	rc = of_property_read_u32(node, "qcom,fg-voltage-empty-mv",
					&chip->voltage_empty_mv);
	if (rc < 0)
		chip->voltage_empty_mv = -EINVAL;

	rc = of_property_read_u32(node, "qcom,fg-batt-capacity-mah",
					&chip->batt_capacity_mah);
	if (rc < 0)
		chip->batt_capacity_mah = -EINVAL;

	rc = of_property_read_u32(node, "qcom,fg-cc-soc-coeff",
					&chip->cc_soc_coeff);
	if (rc < 0)
		chip->cc_soc_coeff = -EINVAL;

	rc = of_property_read_u32(node, "qcom,fg-cutoff-voltage-mv",
						&chip->v_cutoff_mv);
	if (rc < 0)
		chip->v_cutoff_mv = -EINVAL;

	rc = of_property_read_u32(node, "qcom,fg-iterm-ma",
					&chip->fg_iterm_ma);
	if (rc < 0)
		chip->fg_iterm_ma = -EINVAL;

	rc = of_property_read_u32(node, "qcom,fg-ibatt-standby-ma",
					&chip->fg_ibatt_standby_ma);
	if (rc < 0)
		chip->fg_ibatt_standby_ma = -EINVAL;

	rc = of_property_read_u32(node, "qcom,thermistor-c1-coeff",
					&chip->fg_thermistor_c1_coeff);
	if (rc < 0)
		chip->fg_thermistor_c1_coeff = -EINVAL;

	rc = of_property_read_u32(node, "qcom,fg-cc-to-cv-mv",
					&chip->fg_cc_to_cv_mv);
	if (rc < 0)
		chip->fg_cc_to_cv_mv = -EINVAL;

	rc = of_property_read_u32(node, "qcom,otg-batt-curr-limit",
					&chip->otg_batt_curr_limit);
	if (rc < 0)
		chip->otg_batt_curr_limit = -EINVAL;

	rc = of_property_read_u32(node, "qcom,fg-auto-recharge-soc",
					&chip->fg_auto_recharge_soc);
	if (rc < 0)
		chip->fg_auto_recharge_soc = -EINVAL;

	rc = of_property_read_u32(node, "htc,batt-full-criteria",
					&chip->batt_full_criteria);
	if (rc < 0)
		chip->batt_full_criteria = -EINVAL;

	rc = of_property_read_u32(node, "htc,batt-eoc-criteria",
					&chip->batt_eoc_criteria);
	if (rc < 0)
		chip->batt_eoc_criteria = -EINVAL;

	rc = of_property_read_u32(node, "qcom,cool-bat-decidegc",
					&chip->cool_bat_decidegc);
	if (rc < 0)
		chip->cool_bat_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,warm-bat-decidegc",
					&chip->warm_bat_decidegc);
	if (rc < 0)
		chip->warm_bat_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,clamp-soc-voltage-mv",
					&chip->clamp_soc_voltage_mv);
	if (rc < 0)
		chip->clamp_soc_voltage_mv = -EINVAL;

	rc = of_property_read_u32(node, "qcom,max-fastchg-ma",
					&chip->max_fastchg_ma);
	if (rc < 0)
		chip->max_fastchg_ma = -EINVAL;

	rc = of_property_read_u32(node, "htc,fg-voltage-empty-re-check-mv",
					&chip->voltage_empty_re_check_mv);
	if (rc < 0)
		chip->voltage_empty_re_check_mv = -EINVAL;

	chip->wa_to_re_calc_soc_in_low_temp = of_property_read_bool(node,
					"htc,wa-to-re-calc-soc-in-low-temp");

	chip->pm8916_enabled = of_property_read_bool(node,
						"htc,pm8916-enabled");

	rc = of_property_read_u32(node, "htc,combined-chgr-max-aicl-input-curr-limit",
					&chip->combined_chgr_max_aicl_input_curr_limit);
	if (rc < 0)
		chip->combined_chgr_max_aicl_input_curr_limit = -EINVAL;

	rc = of_property_read_u32(node, "htc,pm8916-max-input-curr-limit",
					&chip->pm8916_max_input_curr_limit);
	if (rc < 0)
		chip->pm8916_max_input_curr_limit = PM8916_MAX_INPUT_CURR_LIMIT;

	rc = of_property_read_u32(node, "htc,r-hw-mb-impedance",
					&gs_R_HW_MB_Impedance);
	if (rc < 0)
		gs_R_HW_MB_Impedance = -EINVAL;

	return 0;
}

static int smb1360_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	u8 reg;
	int rc;
	struct smb1360_chip *chip;
#ifndef CONFIG_HTC_BATT_8960
	struct power_supply *usb_psy;
#endif

	flag_keep_charge_on =
		(get_kernel_flag() & KERNEL_FLAG_KEEP_CHARG_ON) ? 1 : 0;
	flag_pa_recharge =
		(get_kernel_flag() & KERNEL_FLAG_PA_RECHARG_TEST) ? 1 : 0;
	flag_enable_fast_charge =
		(get_kernel_flag() & KERNEL_FLAG_ENABLE_FAST_CHARGE) ? 1 : 0;
	flag_enable_bms_charger_log =
		(get_kernel_flag() & KERNEL_FLAG_ENABLE_BMS_CHARGER_LOG) ? 1 : 0;

#ifndef CONFIG_HTC_BATT_8960
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&client->dev, "USB supply not found; defer probe\n");
		return -EPROBE_DEFER;
	}
#endif

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	chip->resume_completed = true;
	chip->client = client;
	chip->dev = &client->dev;
#ifndef CONFIG_HTC_BATT_8960
	chip->usb_psy = usb_psy;
#endif
	chip->fake_battery_soc = -EINVAL;
	mutex_init(&chip->read_write_lock);

	
	rc = smb1360_read(chip, CFG_BATT_CHG_REG, &reg);
	if (rc) {
		pr_err("Failed to detect SMB1360 0x%X, device may be absent\n", client->addr);

		client->addr = 0x14;
		rc = smb1360_read(chip, CFG_BATT_CHG_REG, &reg);
		if (rc) {
			pr_err("Failed to detect SMB1360 0x%X, device may be absent\n", client->addr);
			return -ENODEV;
		}
	}

	rc = read_revision(chip, &chip->revision);
	pr_info("smb1360 revision = %d\n", chip->revision);
	if (rc)
		dev_err(chip->dev, "Couldn't read revision rc = %d\n", rc);

	rc = smb_parse_dt(chip);
	if (rc < 0) {
		dev_err(&client->dev, "Unable to parse DT nodes\n");
		return rc;
	}

	if (chip->wa_to_re_calc_soc_in_low_temp) {
		rc = set_battery_data(chip);
		if (rc) {
			pr_err("Bad battery data %d\n", rc);
			return rc;
		}
	}

	device_init_wakeup(chip->dev, 1);
	i2c_set_clientdata(client, chip);
	mutex_init(&chip->irq_complete);
	mutex_init(&chip->charging_disable_lock);
	mutex_init(&chip->current_change_lock);
	chip->default_i2c_addr = client->addr;

	pr_info("default_i2c_addr=%x\n", chip->default_i2c_addr);

	rc = smb1360_hw_init(chip);
	if (rc < 0) {
		dev_err(&client->dev,
			"Unable to intialize hardware rc = %d\n", rc);
		goto fail_hw_init;
	}

	rc = smb1360_regulator_init(chip);
	if  (rc) {
		dev_err(&client->dev,
			"Couldn't initialize smb349 ragulator rc=%d\n", rc);
		return rc;
	}

	rc = determine_initial_status(chip);
	if (rc < 0) {
		dev_err(&client->dev,
			"Unable to determine init status rc = %d\n", rc);
		goto fail_hw_init;
	}

	the_chip = chip;

#ifndef CONFIG_HTC_BATT_8960
	chip->batt_psy.name		= "battery";
	chip->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.get_property	= smb1360_battery_get_property;
	chip->batt_psy.set_property	= smb1360_battery_set_property;
	chip->batt_psy.properties	= smb1360_battery_properties;
	chip->batt_psy.num_properties  = ARRAY_SIZE(smb1360_battery_properties);
	chip->batt_psy.external_power_changed = smb1360_external_power_changed;
	chip->batt_psy.property_is_writeable = smb1360_battery_is_writeable;

	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		dev_err(&client->dev,
			"Unable to register batt_psy rc = %d\n", rc);
		goto fail_hw_init;
	}
#endif

	INIT_DELAYED_WORK(&chip->update_ovp_uvp_work, smb1360_update_ovp_uvp_work);
	INIT_DELAYED_WORK(&chip->kick_wd_work, smb1360_kick_wd_work);
	INIT_DELAYED_WORK(&chip->eoc_work, smb1360_eoc_work);
	INIT_DELAYED_WORK(&chip->aicl_work, smb1360_aicl_work);
	INIT_DELAYED_WORK(&chip->vin_collapse_check_work,
					vin_collapse_check_worker);
	INIT_DELAYED_WORK(&chip->cable_impedance_work, cable_impedance_worker);

	wake_lock_init(&chip->eoc_worker_lock, WAKE_LOCK_SUSPEND,
			"smb1360_eoc_worker_lock");
	wake_lock_init(&chip->vin_collapse_check_wake_lock,
			WAKE_LOCK_SUSPEND, "vin_collapse_check_wake_lock");

	
	if (client->irq)
		smb1360_stat_handler(client->irq, chip);

	
	if (client->irq) {
		rc = devm_request_threaded_irq(&client->dev, client->irq, NULL,
				smb1360_stat_handler, IRQF_ONESHOT,
				"smb1360_stat_irq", chip);
		if (rc < 0) {
			dev_err(&client->dev,
				"request_irq for irq=%d  failed rc = %d\n",
				client->irq, rc);
			goto unregister_batt_psy;
		}
		enable_irq_wake(client->irq);
	}

	chip->debug_root = debugfs_create_dir("smb1360", NULL);
	if (!chip->debug_root)
		dev_err(chip->dev, "Couldn't create debug dir\n");

	if (chip->debug_root) {
		struct dentry *ent;

		ent = debugfs_create_file("config_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &cnfg_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create cnfg debug file rc = %d\n",
				rc);

		ent = debugfs_create_file("status_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &status_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create status debug file rc = %d\n",
				rc);

		ent = debugfs_create_file("irq_status", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &irq_stat_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create irq_stat debug file rc = %d\n",
				rc);

		ent = debugfs_create_file("cmd_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &cmd_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create cmd debug file rc = %d\n",
				rc);

		ent = debugfs_create_file("fg_regs",
				S_IFREG | S_IRUGO, chip->debug_root, chip,
					  &fg_regs_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create fg_scratch_pad debug file rc = %d\n",
				rc);

		ent = debugfs_create_x32("address", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root,
					  &(chip->peek_poke_address));
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create address debug file rc = %d\n",
				rc);

		ent = debugfs_create_file("data", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root, chip,
					  &poke_poke_debug_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create data debug file rc = %d\n",
				rc);

		ent = debugfs_create_x32("fg_address",
					S_IFREG | S_IWUSR | S_IRUGO,
					chip->debug_root,
					&(chip->fg_peek_poke_address));
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create address debug file rc = %d\n",
				rc);

		ent = debugfs_create_file("fg_data",
					S_IFREG | S_IWUSR | S_IRUGO,
					chip->debug_root, chip,
					&fg_poke_poke_debug_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create data debug file rc = %d\n",
				rc);

		ent = debugfs_create_x32("fg_access_type",
					S_IFREG | S_IWUSR | S_IRUGO,
					chip->debug_root,
					&(chip->fg_access_type));
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create data debug file rc = %d\n",
				rc);

		ent = debugfs_create_x32("skip_writes",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root,
					  &(chip->skip_writes));
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create data debug file rc = %d\n",
				rc);

		ent = debugfs_create_x32("skip_reads",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root,
					  &(chip->skip_reads));
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create data debug file rc = %d\n",
				rc);

		ent = debugfs_create_file("irq_count", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &irq_count_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create count debug file rc = %d\n",
				rc);
	}

#ifndef CONFIG_HTC_BATT_8960
	dev_info(chip->dev, "SMB1360 revision=0x%x probe success! batt=%d usb=%d soc=%d\n",
			chip->revision,
			smb1360_get_prop_batt_present(chip),
			chip->usb_present,
			smb1360_get_prop_batt_capacity(chip));
#endif

	htc_charger_event_notify(HTC_CHARGER_EVENT_READY);
	htc_gauge_event_notify(HTC_GAUGE_EVENT_READY);

	if (chip->batt_full) {
		is_batt_full_eoc_stop = true;
		htc_gauge_event_notify(HTC_GAUGE_EVENT_EOC_STOP_CHG);
	}

	return 0;

unregister_batt_psy:
#ifndef CONFIG_HTC_BATT_8960
	power_supply_unregister(&chip->batt_psy);
#endif
fail_hw_init:
	regulator_unregister(chip->otg_vreg.rdev);
	return rc;
}

static int smb1360_remove(struct i2c_client *client)
{
	struct smb1360_chip *chip = i2c_get_clientdata(client);

	regulator_unregister(chip->otg_vreg.rdev);
#ifndef CONFIG_HTC_BATT_8960
	power_supply_unregister(&chip->batt_psy);
#endif
	cancel_delayed_work_sync(&chip->update_ovp_uvp_work);
	cancel_delayed_work_sync(&chip->kick_wd_work);
	cancel_delayed_work_sync(&chip->eoc_work);
	cancel_delayed_work_sync(&chip->aicl_work);
	mutex_destroy(&chip->charging_disable_lock);
	mutex_destroy(&chip->current_change_lock);
	mutex_destroy(&chip->read_write_lock);
	mutex_destroy(&chip->irq_complete);
	debugfs_remove_recursive(chip->debug_root);

	return 0;
}

static int smb1360_suspend(struct device *dev)
{
	int i, rc;
	struct i2c_client *client = to_i2c_client(dev);
	struct smb1360_chip *chip = i2c_get_clientdata(client);

	
	for (i = 0; i < 3; i++) {
		rc = smb1360_read(chip, IRQ_CFG_REG + i,
					&chip->irq_cfg_mask[i]);
		if (rc)
			pr_err("Couldn't save irq cfg regs rc=%d\n", rc);
	}

	
	rc = smb1360_write(chip, IRQ_CFG_REG, IRQ_DCIN_UV_BIT
						| IRQ_BAT_HOT_COLD_HARD_BIT
						| IRQ_BAT_HOT_COLD_SOFT_BIT);
	if (rc < 0)
		pr_err("Couldn't set irq_cfg rc=%d\n", rc);

	rc = smb1360_write(chip, IRQ2_CFG_REG, IRQ2_BATT_MISSING_BIT
						| IRQ2_VBAT_LOW_BIT
						| IRQ2_POWER_OK_BIT);
	if (rc < 0)
		pr_err("Couldn't set irq2_cfg rc=%d\n", rc);

	rc = smb1360_write(chip, IRQ3_CFG_REG, IRQ3_SOC_FULL_BIT
					| IRQ3_SOC_MIN_BIT
					| IRQ3_SOC_EMPTY_BIT);
	if (rc < 0)
		pr_err("Couldn't set irq3_cfg rc=%d\n", rc);

	mutex_lock(&chip->irq_complete);
	chip->resume_completed = false;
	mutex_unlock(&chip->irq_complete);

	return 0;
}

static int smb1360_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb1360_chip *chip = i2c_get_clientdata(client);

	if (chip->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int smb1360_resume(struct device *dev)
{
	int i, rc;
	struct i2c_client *client = to_i2c_client(dev);
	struct smb1360_chip *chip = i2c_get_clientdata(client);

	
	for (i = 0; i < 3; i++) {
		rc = smb1360_write(chip, IRQ_CFG_REG + i,
					chip->irq_cfg_mask[i]);
		if (rc)
			pr_err("Couldn't restore irq cfg regs rc=%d\n", rc);
	}

	mutex_lock(&chip->irq_complete);
	chip->resume_completed = true;
	if (chip->irq_waiting) {
		mutex_unlock(&chip->irq_complete);
		smb1360_stat_handler(client->irq, chip);
		enable_irq(client->irq);
	} else {
		mutex_unlock(&chip->irq_complete);
	}

	return 0;
}

static void smb1360_shutdown(struct i2c_client *client)
{
	int rc;
	struct smb1360_chip *chip = i2c_get_clientdata(client);

	rc = smb1360_otg_disable(chip);
	if (rc)
		pr_err("Couldn't disable OTG mode rc=%d\n", rc);

	if (chip->shdn_after_pwroff) {
		rc = smb1360_poweroff(chip);
		if (rc)
			pr_err("Couldn't shutdown smb1360, rc = %d\n", rc);
		pr_info("smb1360 power off\n");
	}
}

static const struct dev_pm_ops smb1360_pm_ops = {
	.resume		= smb1360_resume,
	.suspend_noirq	= smb1360_suspend_noirq,
	.suspend	= smb1360_suspend,
};

static struct of_device_id smb1360_match_table[] = {
	{ .compatible = "qcom,smb1360-chg-fg",},
	{ },
};

static const struct i2c_device_id smb1360_id[] = {
	{"smb1360-chg-fg", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, smb1360_id);

static struct i2c_driver smb1360_driver = {
	.driver		= {
		.name		= "smb1360-chg-fg",
		.owner		= THIS_MODULE,
		.of_match_table	= smb1360_match_table,
		.pm		= &smb1360_pm_ops,
	},
	.probe		= smb1360_probe,
	.remove		= smb1360_remove,
	.shutdown	= smb1360_shutdown,
	.id_table	= smb1360_id,
};

module_i2c_driver(smb1360_driver);

MODULE_DESCRIPTION("SMB1360 Charger and Fuel Gauge");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:smb1360-chg-fg");
