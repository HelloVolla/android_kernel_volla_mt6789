// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 MediaTek Inc.
 */

#include <linux/bits.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/consumer.h>
#include <linux/atomic.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/phy/phy.h>

#if IS_ENABLED(CONFIG_MTK_CHARGER)
#include "charger_class.h"
#include "mtk_charger.h"
#endif /* CONFIG_MTK_CHARGER */

#define PHY_MODE_BC11_SET 1
#define PHY_MODE_BC11_CLR 2

#define RT9467_REG_CHG_CTRL1		0x01
#define RT9467_REG_CHG_CTRL2		0x02
#define RT9467_REG_CHG_CTRL3		0x03
#define RT9467_REG_CHG_CTRL4		0x04
#define RT9467_REG_CHG_CTRL5		0x05
#define RT9467_REG_CHG_CTRL6		0x06
#define RT9467_REG_CHG_CTRL7		0x07
#define RT9467_REG_CHG_CTRL8		0x08
#define RT9467_REG_CHG_CTRL9		0x09
#define RT9467_REG_CHG_CTRL10		0x0A
#define RT9467_REG_CHG_CTRL12		0x0C
#define RT9467_REG_CHG_CTRL13		0x0D
#define RT9467_REG_CHG_CTRL14		0x0E
#define RT9467_REG_CHG_CTRL16		0x10
#define RT9467_REG_CHG_DPDM1		0x12
#define RT9467_REG_CHG_DPDM2		0x13
#define RT9467_REG_CHG_CTRL17		0x19
#define RT9467_REG_CHG_CTRL18		0x1A
#define RT9467_REG_CHG_HIDDEN_CTRL1	0x20
#define RT9467_REG_CHG_HIDDEN_CTRL2	0x21
#define RT9467_REG_CHG_HIDDEN_CTRL4	0x23
#define RT9467_REG_CHG_HIDDEN_CTRL6	0x25
#define RT9467_REG_CHG_HIDDEN_CTRL7	0x26
#define RT9467_REG_CHG_HIDDEN_CTRL9	0x28
#define RT9467_REG_CHG_HIDDEN_CTRL15	0x2E
#define RT9467_REG_DEVICE_ID		0x40
#define RT9467_REG_CHG_STAT		0x42
#define RT9467_REG_CHG_STATC		0x50
#define RT9467_REG_CHG_STATC_CTRL	0x60

#define RT9467_ICHG_MINUA		100000
#define RT9467_ICHG_MAXUA		5000000
#define RT9467_CV_MAXUV			4710000
#define RT9467_MANUFACTURER		"Richtek Technology Corp"
#define RT9467_MASK_EN_PUMPX		BIT(7)

enum rt9467_fields {
	/* RT9467_REG_CHG_CTRL1 */
	F_HZ = 0, F_OPA_MODE,
	/* RT9467_REG_CHG_CTRL2 */
	F_SHIP_MODE, F_BATDET_DIS_DLY, F_TE, F_IINLMTSEL, F_CFO_EN, F_CHG_EN,
	/* RT9467_REG_CHG_CTRL3 */
	F_IAICR, F_ILIM_EN,
	/* RT9467_REG_CHG_CTRL4 */
	F_VOREG,
	/* RT9467_REG_CHG_CTRL6 */
	F_VMIVR,
	/* RT9467_REG_CHG_CTRL7 */
	F_ICHG,
	/* RT9467_REG_CHG_CTRL8 */
	F_IPREC,
	/* RT9467_REG_CHG_CTRL9 */
	F_IEOC,
	/* RT9467_REG_CHG_CTRL10 */
	F_OTG_OLP,
	/* RT9467_REG_CHG_CTRL12 */
	F_WT_FC, F_TMR_EN,
	/* RT9467_REG_CHG_CTRL13 */
	F_WDT_EN,
	/* RT9467_REG_CHG_CTRL14 */
	F_AICL_MEAS, F_AICL_VTH,
	/* RT9467_REG_CHG_CTRL16 */
	F_JEITA_EN,
	/* RT9467_REG_CHG_DPDM1 */
	F_USBCHGEN, F_ATT_DLY,
	/* RT9467_REG_CHG_DPDM2 */
	F_USB_STATUS,
	/* RT9467_REG_CHG_CTRL17 */
	F_EN_PUMPX, F_PUMPX_20_10, F_PUMPX_UP_DN, F_PUMPX_DEC,
	/* RT9467_REG_CHG_CTRL18 */
	F_BAT_COMP, F_VCLAMP,
	/* RT9467_REG_DEVICE_ID */
	F_VENDOR, F_CHIP_REV,
	/* RT9467_REG_CHG_STAT */
	F_CHG_STAT,
	/* RT9467_REG_CHG_STATC */
	F_PWR_RDY, F_CHG_MIVR,
	/* RT9467_REG_CHG_HIDDEN_CTRL1 */
	F_EOC_RST,
	/* RT9467_REG_CHG_HIDDEN_CTRL2 */
	F_DISCHG_EN,
	/* RT9467_REG_CHG_HIDDEN_CTRL9 */
	F_EN_PSK,
	/* RT9467_REG_CHG_HIDDEN_CTRL15*/
	F_AUTO_SENSE,
	F_MAX_FIELDS
};

enum rt9467_ranges{
	RT9467_RANGE_IAICR = 0,
	RT9467_RANGE_VOREG,
	RT9467_RANGE_VMIVR,
	RT9467_RANGE_ICHG,
	RT9467_RANGE_IPREC,
	RT9467_RANGE_IEOC,
	RT9467_RANGE_AICL_VTH,
	RT9467_RANGE_PUMPX_DEC,
	RT9467_RANGE_BAT_COMP,
	RT9467_RANGE_VCLAMP,
	RT9467_MAX_RANGES
};

static const u32 rt9467_otg_cc[] = {
	500000, 700000, 1100000, 1300000, 1800000, 2100000, 2400000, 3000000,
};

static const u32 rt9467_safety_timer[] = {
	4, 6, 8, 10, 12, 14, 16, 20,
};

static const u8 rt9467_hm_passcode[] = {
	0x49, 0x32, 0xB6, 0x27, 0x48, 0x18, 0x03, 0xE2,
};

/* All converted to microvoltage and microamp */
static const struct linear_range rt9467_lranges[F_MAX_FIELDS] = {
	[RT9467_RANGE_IAICR] = { 100000, 0, 63, 50000 },
	[RT9467_RANGE_VOREG] = { 3900000, 0, 81, 10000 },
	[RT9467_RANGE_VMIVR] = { 3900000, 0, 95, 100000 },
	[RT9467_RANGE_ICHG] = { 100000, 0, 49, 100000 },
	[RT9467_RANGE_IPREC] = { 100000, 0, 15, 50000 },
	[RT9467_RANGE_IEOC] = { 100000, 0, 15, 50000 },
	[RT9467_RANGE_AICL_VTH] = { 4100000, 0, 7, 100000 },
	[RT9467_RANGE_PUMPX_DEC] = { 5500000, 0, 18, 500000 },
	[RT9467_RANGE_BAT_COMP] = { 0, 0, 7, 25000 },
	[RT9467_RANGE_VCLAMP] = { 0, 0, 7, 32000 }
};

static struct reg_field rt9467_chg_fields[] = {
	[F_HZ]			= REG_FIELD(RT9467_REG_CHG_CTRL1, 2, 2),
	[F_OPA_MODE]		= REG_FIELD(RT9467_REG_CHG_CTRL1, 0, 0),
	[F_SHIP_MODE]		= REG_FIELD(RT9467_REG_CHG_CTRL2, 7, 7),
	[F_BATDET_DIS_DLY]	= REG_FIELD(RT9467_REG_CHG_CTRL2, 6, 6),
	[F_TE]			= REG_FIELD(RT9467_REG_CHG_CTRL2, 4, 4),
	[F_IINLMTSEL]		= REG_FIELD(RT9467_REG_CHG_CTRL2, 2, 3),
	[F_CFO_EN]		= REG_FIELD(RT9467_REG_CHG_CTRL2, 1, 1),
	[F_CHG_EN]		= REG_FIELD(RT9467_REG_CHG_CTRL2, 0, 0),
	[F_IAICR]		= REG_FIELD(RT9467_REG_CHG_CTRL3, 2, 7),
	[F_ILIM_EN]		= REG_FIELD(RT9467_REG_CHG_CTRL3, 0, 0),
	[F_VOREG]		= REG_FIELD(RT9467_REG_CHG_CTRL4, 1, 7),
	[F_VMIVR] 		= REG_FIELD(RT9467_REG_CHG_CTRL6, 1, 7),
	[F_ICHG] 		= REG_FIELD(RT9467_REG_CHG_CTRL7, 2, 7),
	[F_IPREC] 		= REG_FIELD(RT9467_REG_CHG_CTRL8, 0, 3),
	[F_IEOC]		= REG_FIELD(RT9467_REG_CHG_CTRL9, 4, 7),
	[F_OTG_OLP] 		= REG_FIELD(RT9467_REG_CHG_CTRL10, 0, 2),
	[F_WT_FC] 		= REG_FIELD(RT9467_REG_CHG_CTRL12, 5, 7),
	[F_TMR_EN] 		= REG_FIELD(RT9467_REG_CHG_CTRL12, 1, 1),
	[F_WDT_EN] 		= REG_FIELD(RT9467_REG_CHG_CTRL13, 7, 7),
	[F_AICL_MEAS] 		= REG_FIELD(RT9467_REG_CHG_CTRL14, 7, 7),
	[F_AICL_VTH] 		= REG_FIELD(RT9467_REG_CHG_CTRL14, 0, 2),
	[F_JEITA_EN] 		= REG_FIELD(RT9467_REG_CHG_CTRL16, 4, 4),
	[F_USBCHGEN] 		= REG_FIELD(RT9467_REG_CHG_DPDM1, 7, 7),
	[F_ATT_DLY] 		= REG_FIELD(RT9467_REG_CHG_DPDM1, 6, 6),
	[F_USB_STATUS] 		= REG_FIELD(RT9467_REG_CHG_DPDM2, 0, 2),
	[F_EN_PUMPX] 		= REG_FIELD(RT9467_REG_CHG_CTRL17, 7, 7),
	[F_PUMPX_20_10] 	= REG_FIELD(RT9467_REG_CHG_CTRL17, 6, 6),
	[F_PUMPX_UP_DN] 	= REG_FIELD(RT9467_REG_CHG_CTRL17, 5, 5),
	[F_PUMPX_DEC] 		= REG_FIELD(RT9467_REG_CHG_CTRL17, 0, 4),
	[F_BAT_COMP] 		= REG_FIELD(RT9467_REG_CHG_CTRL18, 3, 5),
	[F_VCLAMP] 		= REG_FIELD(RT9467_REG_CHG_CTRL18, 0, 2),
	[F_VENDOR] 		= REG_FIELD(RT9467_REG_DEVICE_ID, 4, 7),
	[F_CHIP_REV] 		= REG_FIELD(RT9467_REG_DEVICE_ID, 0, 3),
	[F_CHG_STAT]		= REG_FIELD(RT9467_REG_CHG_STAT, 6, 7),
	[F_PWR_RDY] 		= REG_FIELD(RT9467_REG_CHG_STATC, 7, 7),
	[F_CHG_MIVR] 		= REG_FIELD(RT9467_REG_CHG_STATC, 6, 6),
	[F_EOC_RST]		= REG_FIELD(RT9467_REG_CHG_HIDDEN_CTRL1, 7, 7),
	[F_DISCHG_EN]		= REG_FIELD(RT9467_REG_CHG_HIDDEN_CTRL2, 2, 2),
	[F_EN_PSK]		= REG_FIELD(RT9467_REG_CHG_HIDDEN_CTRL9, 7, 7),
	[F_AUTO_SENSE]		= REG_FIELD(RT9467_REG_CHG_HIDDEN_CTRL15, 0, 0),
};

enum {
	RT9467_STAT_READY = 0,
	RT9467_STAT_PROGRESS,
	RT9467_STAT_CHARGE_DONE,
	RT9467_STAT_FAULT,
};

enum rt9467_adc_chan {
	RT9467_ADC_VBUS_DIV5 = 0,
	RT9467_ADC_VBUS_DIV2,
	RT9467_ADC_VSYS,
	RT9467_ADC_VBAT,
	RT9467_ADC_TS_BAT,
	RT9467_ADC_IBUS,
	RT9467_ADC_IBAT,
	RT9467_ADC_REGN,
	RT9467_ADC_TEMP_JC,
	RT9467_ADC_MAX
};

enum rt9467_attach_trigger {
	ATTACH_TRIG_IGNORE,
	ATTACH_TRIG_PWR_RDY,
	ATTACH_TRIG_TYPEC,
};

enum rt9467_usbsw {
	USBSW_CHG = 0,
	USBSW_USB,
};

static const char *const rt9467_attach_trig_names[] = {
	"ignore", "pwr_rdy", "typec",
};

enum rt9467_chg_type {
	RT9467_CHG_TYPE_NOVBUS = 0,
	RT9467_CHG_TYPE_UNDER_GOING,
	RT9467_CHG_TYPE_SDP,
	RT9467_CHG_TYPE_SDPNSTD,
	RT9467_CHG_TYPE_DCP,
	RT9467_CHG_TYPE_CDP,
	RT9467_CHG_TYPE_MAX,
};

static const char *const rt9467_port_stat_names[RT9467_CHG_TYPE_MAX] = {
	[RT9467_CHG_TYPE_NOVBUS]	= "No Vbus",
	[RT9467_CHG_TYPE_UNDER_GOING]	= "Under Going",
	[RT9467_CHG_TYPE_SDP]		= "SDP",
	[RT9467_CHG_TYPE_SDPNSTD]	= "SDPNSTD",
	[RT9467_CHG_TYPE_DCP]		= "DCP",
	[RT9467_CHG_TYPE_CDP]		= "CDP",
};

enum rt9467_iin_limit_sel {
	RT9467_IINLMTSEL_3_2A = 0,
	RT9467_IINLMTSEL_CHG_TYP,
	RT9467_IINLMTSEL_AICR,
	RT9467_IINLMTSEL_LOWER_LEVEL, /* lower of above three */
};

struct rt9467_chg_data {
	struct device *dev;
	struct regmap *regmap;
	struct regmap_field *rm_field[F_MAX_FIELDS];
	struct power_supply *psy;
	struct power_supply_desc psy_desc;
	enum power_supply_usb_type psy_usb_type;
	struct iio_channel *adc_chans;
	struct mutex hm_lock;
	struct mutex attach_lock;
	struct mutex ichg_lock;
	struct mutex ieoc_lock;
	struct mutex aicr_lock;
	struct mutex pp_lock;
	struct gpio_desc *ceb_gpio;
	struct gpio_desc *switch_en_gpio;
	struct gpio_desc *otg_en_gpio;
	struct regulator_dev *rdev;
	struct workqueue_struct *wq;
	struct work_struct bc12_work;
	struct work_struct init_work;
	struct delayed_work mivr_dwork;
	struct completion aicl_done;
	atomic_t attach;
	atomic_t eoc_cnt;
	enum rt9467_attach_trigger attach_trig;
	unsigned int ichg;
	unsigned int ichg_dis_chg;
	unsigned int mivr;
	unsigned int ieoc;
	unsigned int vbus_rdy;
	unsigned int safety_timer;
	int tchg;
	bool ieoc_wkard;
	bool bc12_dn;
	bool pp_en;
#if IS_ENABLED(CONFIG_MTK_CHARGER)
	struct charger_device *chg_dev;
	const char *chg_name;
	struct charger_properties chg_prop;
	struct mutex pe_lock;
	u32 boot_mode;
	struct wakeup_source *cdp_work_ws;
#endif /* CONFIG_MTK_CHARGER */
};

static void linear_range_get_selector_within(const struct linear_range *r,
					     unsigned int val,
					     unsigned int *selector)
{
	if (r->min > val) {
		*selector = r->min_sel;
		return;
	}

	if (linear_range_get_max_value(r) < val) {
		*selector = r->max_sel;
		return;
	}

	if (r->step == 0)
		*selector = r->min_sel;
	else
		*selector = (val - r->min) / r->step + r->min_sel;
}

static int rt9467_enable_hidden_mode(struct rt9467_chg_data *data, bool en)
{
	int ret = 0;

	mutex_lock(&data->hm_lock);

	if (en) {
		ret = regmap_raw_write(data->regmap, 0x70,
				       rt9467_hm_passcode,
				       ARRAY_SIZE(rt9467_hm_passcode));
		if (ret) {
			dev_err(data->dev, "%s: Failed to enable hidden mode (%d)\n", __func__, ret);
			goto out;
		}
	} else {
		ret = regmap_write(data->regmap, 0x70, 0x00);
		if (ret) {
			dev_err(data->dev, "%s: Failed to disable hidden mode (%d)\n", __func__, ret);
			goto out;
		}
	}
out:
	mutex_unlock(&data->hm_lock);
	return ret;
}

static int __rt9467_get_adc(struct rt9467_chg_data *data,
			  enum rt9467_adc_chan chan_idx, int *val)
{
	int ret;
	enum power_supply_property psp;
	union power_supply_propval psy_val;

	/* Workaround for IBUS and IBAT */
	if (chan_idx == RT9467_ADC_IBUS) {
		mutex_lock(&data->aicr_lock);
		psp = POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT;
		ret = power_supply_get_property(data->psy, psp, &psy_val);
		if (ret) {
			dev_err(data->dev, "Failed to get aicr\n");
			goto out;
		}
	} else if (chan_idx == RT9467_ADC_IBAT) {
		mutex_lock(&data->ichg_lock);
		psp = POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT;
		ret = power_supply_get_property(data->psy, psp, &psy_val);
		if (ret) {
			dev_err(data->dev, "Failed to get ichg\n");
			goto out;
		}
	}

	ret = iio_read_channel_processed(&data->adc_chans[chan_idx], val);
	if (ret)
		dev_err(data->dev, "Failed to get chan %d\n", chan_idx);

out:
	/* Coefficient of IBUS and IBAT */
	if (chan_idx == RT9467_ADC_IBUS) {
		if (psy_val.intval < 400000)
			*val = *val * 67 / 100;
		else
			*val = *val * 88 / 100;
		mutex_unlock(&data->aicr_lock);
	} else if (chan_idx == RT9467_ADC_IBAT) {
		if (psy_val.intval >= 100000 && psy_val.intval <= 450000)
			*val = *val * 57 / 100;
		else if (psy_val.intval >= 500000 && psy_val.intval <= 850000)
			*val = *val * 63 / 100;
		mutex_unlock(&data->ichg_lock);
	}
	return ret;
}

static int __rt9467_enable_otg(struct rt9467_chg_data *data, bool en)
{
	int ret;
	u8 hd_val = en ? 0x00 : 0x0F;
	u8 lg_slew_rate = en ? 0x7c : 0x73;

	dev_info(data->dev, "%s en = %d\n", __func__, en);

	ret = rt9467_enable_hidden_mode(data, true);
	if (ret) {
		dev_err(data->dev, "Failed to enable hidden mode\n");
		return ret;
	}

	/*
	 * Workaround : slow Low side mos Gate driver slew rate
	 * for decline VBUS noise
	 * reg[0x23] = 0x7c after entering OTG mode
	 * reg[0x23] = 0x73 after leaving OTG mode
	 */
	ret = regmap_write(data->regmap, RT9467_REG_CHG_HIDDEN_CTRL4,
			   lg_slew_rate);
	if (ret) {
		dev_err(data->dev, "Failed to set slew rate (%d)\n", ret);
		goto out;
	}

	ret = (en ? regulator_enable_regmap : regulator_disable_regmap)
	      (data->rdev);
	if (ret) {
		dev_err(data->dev, "Failed to enable otg\n");
		goto err_en_otg;
	}

	msleep(20);
	/*
	 * Workaround reg[0x25] = 0x00 after entering OTG mode
	 * reg[0x25] = 0x0F after leaving OTG mode
	 */
	ret = regmap_write(data->regmap, RT9467_REG_CHG_HIDDEN_CTRL6, hd_val);
	if (ret)
		dev_err(data->dev, "Failed to write otg workaround(%d)\n", ret);

	goto out;

err_en_otg:
	/* Recover Low side mos Gate slew rate */
	ret = regmap_write(data->regmap, RT9467_REG_CHG_HIDDEN_CTRL4, 0x73);
	if (ret)
		dev_err(data->dev, "Failed to set slew rate 0x73 (%d)\n", ret);
out:
	return rt9467_enable_hidden_mode(data, false);
}

/* OTG regulator */
static int rt9467_regulator_enable(struct regulator_dev *rdev)
{
	struct rt9467_chg_data *data = rdev_get_drvdata(rdev);

	return __rt9467_enable_otg(data, true);
}

static int rt9467_regulator_disable(struct regulator_dev *rdev)
{
	struct rt9467_chg_data *data = rdev_get_drvdata(rdev);

	return __rt9467_enable_otg(data, false);
}

static const struct regulator_ops rt9467_otg_regulator_ops = {
	.enable = rt9467_regulator_enable,
	.disable = rt9467_regulator_disable,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_current_limit = regulator_set_current_limit_regmap,
	.get_current_limit = regulator_get_current_limit_regmap,
};

static const struct regulator_desc rt9467_otg_desc = {
	.name = "usb-otg-vbus",
	.of_match = of_match_ptr("usb-otg-vbus"),
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.min_uV = 4425000,
	.uV_step = 25000,
	.n_voltages = 56,
	.curr_table = rt9467_otg_cc,
	.n_current_limits = ARRAY_SIZE(rt9467_otg_cc),
	.csel_reg = RT9467_REG_CHG_CTRL10,
	.csel_mask = GENMASK(2, 0),
	.vsel_reg = RT9467_REG_CHG_CTRL5,
	.vsel_mask = GENMASK(7, 1),
	.enable_reg = RT9467_REG_CHG_CTRL1,
	.enable_mask = BIT(0),
	.ops = &rt9467_otg_regulator_ops,
};

static int rt9467_register_regulator(struct rt9467_chg_data *data)
{
	struct regulator_config cfg = {
		.dev = data->dev,
		.driver_data = data,
		.regmap = data->regmap,
	};

	data->rdev = devm_regulator_register(data->dev, &rt9467_otg_desc, &cfg);
	return IS_ERR(data->rdev) ? PTR_ERR(data->rdev) : 0;
}

static int rt9467_chg_get_iio_adc(struct rt9467_chg_data *data)
{
	dev_info(data->dev, "%s\n", __func__);

	data->adc_chans = devm_iio_channel_get_all(data->dev);
	if (IS_ERR(data->adc_chans)) {
		dev_err(data->dev, "%s: get iio_dev fail\n", __func__);
		return PTR_ERR(data->adc_chans);
	}

	return 0;
}

/* Power supply */
static char *rt9467_psy_supplied_to[] = {
	"battery",
	"mtk-master-charger"
};

static const enum power_supply_usb_type rt9467_chg_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
};

static const enum power_supply_property rt9467_chg_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_PRECHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_TYPE,
};

static int rt9467_chg_prop_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	case POWER_SUPPLY_PROP_STATUS:
		return 1;
	default:
		return 0;
	}
	return 0;
}

static int rt9467_psy_get_status(struct rt9467_chg_data *data,
				 union power_supply_propval *val)
{
	unsigned int status;
	int ret;

	ret = regmap_field_read(data->rm_field[F_CHG_STAT], &status);
	if (ret)
		return ret;

	switch (status) {
	case RT9467_STAT_READY:
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case RT9467_STAT_PROGRESS:
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case RT9467_STAT_CHARGE_DONE:
		val->intval = POWER_SUPPLY_STATUS_FULL;
		break;
	default:
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	return 0;
}

static int rt9467_get_ranges(struct rt9467_chg_data *data,
			     enum rt9467_fields fd,  enum rt9467_ranges rg,
			     unsigned int *value)
{
	const struct linear_range *lrg = rt9467_lranges + rg;
	unsigned int sel;
	int ret;

	ret = regmap_field_read(data->rm_field[fd], &sel);
	if (ret)
		return ret;

	return linear_range_get_value(lrg, sel, value);
}

static int __rt9467_get_ichg(struct rt9467_chg_data *data, unsigned int *ichg)
{
	return rt9467_get_ranges(data, F_ICHG, RT9467_RANGE_ICHG, ichg);
}

static int rt9467_psy_get_ichg(struct rt9467_chg_data *data,
			       union power_supply_propval *val)
{
	unsigned int ichg;
	int ret;

	ret = __rt9467_get_ichg(data, &ichg);
	if (ret)
		return ret;

	val->intval = ichg;
	return 0;
}

static int rt9467_psy_get_cv(struct rt9467_chg_data *data,
			     union power_supply_propval *val)
{
	unsigned int cv;
	int ret;

	ret = rt9467_get_ranges(data, F_VOREG, RT9467_RANGE_VOREG, &cv);
	if (ret)
		return ret;

	val->intval = cv;
	return 0;
}

static int rt9467_psy_get_aicr(struct rt9467_chg_data *data,
			       union power_supply_propval *val)
{
	unsigned int aicr;
	int ret;

	ret = rt9467_get_ranges(data, F_IAICR, RT9467_RANGE_IAICR, &aicr);
	if (ret)
		return ret;

	val->intval = aicr;
	return 0;
}

static int rt9467_psy_get_mivr(struct rt9467_chg_data *data,
			       union power_supply_propval *val)
{
	unsigned int mivr;
	int ret;

	ret = rt9467_get_ranges(data, F_VMIVR, RT9467_RANGE_VMIVR, &mivr);
	if (ret)
		return ret;

	val->intval = mivr;
	return 0;
}

static int rt9467_psy_get_iprec(struct rt9467_chg_data *data,
				union power_supply_propval *val)
{
	unsigned int iprec;
	int ret;

	ret = rt9467_get_ranges(data, F_IPREC, RT9467_RANGE_IPREC, &iprec);
	if (ret)
		return ret;

	val->intval = iprec;
	return 0;
}

static int rt9467_psy_get_ieoc(struct rt9467_chg_data *data,
			       union power_supply_propval *val)
{
	unsigned int ieoc;
	int ret;

	ret = rt9467_get_ranges(data, F_IEOC, RT9467_RANGE_IEOC, &ieoc);
	if (ret)
		return ret;

	val->intval = ieoc;
	return 0;
}

static int rt9467_psy_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct rt9467_chg_data *data = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = rt9467_psy_get_status(data, val);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = atomic_read(&data->attach);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = rt9467_psy_get_ichg(data, val);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = RT9467_ICHG_MAXUA;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = rt9467_psy_get_cv(data, val);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		val->intval = RT9467_CV_MAXUV;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = rt9467_psy_get_aicr(data, val);
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = rt9467_psy_get_mivr(data, val);
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		mutex_lock(&data->attach_lock);
		val->intval = data->psy_usb_type;
		mutex_unlock(&data->attach_lock);
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = data->psy_desc.type;
		break;
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		ret = rt9467_psy_get_iprec(data, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = rt9467_psy_get_ieoc(data, val);
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = RT9467_MANUFACTURER;
		break;
	default:
		ret = -ENODATA;
		break;
	}
	return ret;
}

static int __rt9467_set_ieoc(struct rt9467_chg_data *data, unsigned int ieoc)
{
	int ret;
	unsigned int sel = 0;

	/* IEOC workaround */
	if (data->ieoc_wkard)
		ieoc += 100000; /* 100mA */

	linear_range_get_selector_within(rt9467_lranges + RT9467_RANGE_IEOC,
					 ieoc, &sel);
	dev_info(data->dev, "%s ieoc = %d(0x%02X)\n", __func__, ieoc, sel);
	ret = regmap_field_write(data->rm_field[F_IEOC], sel);
	if (ret)
		return ret;

	data->ieoc = ieoc;
	return 0;
}

static inline int rt9467_ichg_workaround(struct rt9467_chg_data *data,
					 unsigned int uA)
{
	int ret;

	/* VSYS short protection */
	ret = rt9467_enable_hidden_mode(data, true);
	if (ret) {
		dev_err(data->dev, "Failed to enable hidden mode\n");
		return ret;
	}

	if (data->ichg >= 900000 && uA < 900000) {
		ret = regmap_update_bits(data->regmap,
					 RT9467_REG_CHG_HIDDEN_CTRL7,
					 0x60, 0x00);
		if (ret)
			dev_err(data->dev, "Failed to clear hidden7 bit65\n");
	} else if (uA >= 900000 && data->ichg < 900000) {
		ret = regmap_update_bits(data->regmap,
					 RT9467_REG_CHG_HIDDEN_CTRL7,
					 0x60, 0x40);
		if (ret)
			dev_err(data->dev, "Failed to set hidden7 bit6\n");
	}

	return rt9467_enable_hidden_mode(data, false);
}

static int __rt9467_set_ichg(struct rt9467_chg_data *data, unsigned int ichg)
{
	int ret;
	unsigned int sel = 0;

	ichg = (ichg < 500000) ? 500000 : ichg;
	ret = rt9467_ichg_workaround(data, ichg);
	if (ret)
		return ret;

	linear_range_get_selector_within(rt9467_lranges + RT9467_RANGE_ICHG,
					 ichg, &sel);
	dev_info(data->dev, "%s ichg = %d(0x%02X)\n", __func__, ichg, sel);
	ret = regmap_field_write(data->rm_field[F_ICHG], sel);
	if (ret)
		return ret;

	/* Store ichg setting */
	ret = __rt9467_get_ichg(data, &data->ichg);
	if (ret)
		return ret;

	/* Workaround to mask IEOC accurate */
	if (ichg < 900000 && !data->ieoc_wkard) { /* 900mA */
		ret = __rt9467_set_ieoc(data, data->ieoc + 100000);
		data->ieoc_wkard = true;
	} else if (ichg >= 900000 && data->ieoc_wkard) {
		data->ieoc_wkard = false;
		ret = __rt9467_set_ieoc(data, data->ieoc - 100000);
	}

	return ret;
}

static int rt9467_psy_enable_charging(struct rt9467_chg_data *data, bool en)
{
	int ret;
	u32 ichg_ramp_t = 0;

	dev_info(data->dev, "%s en = %d\n", __func__, en);

	/* Workaround for vsys overshoot */
	mutex_lock(&data->ichg_lock);
	mutex_lock(&data->ieoc_lock);

	if (data->ichg <= 500000)
		goto out;

	if (!en) {
		data->ichg_dis_chg = data->ichg;
		ichg_ramp_t = (data->ichg - 500000) / 50000 * 2;
		ret = __rt9467_set_ichg(data, 500000);
		if (ret) {
			dev_err(data->dev, "set ichg failed\n");
			goto vsys_overshot_err;
		}
		msleep(ichg_ramp_t);
	} else {
		if (data->ichg == data->ichg_dis_chg) {
			ret = __rt9467_set_ichg(data, data->ichg);
			if (ret) {
				dev_err(data->dev, "set ichg failed\n");
				goto vsys_overshot_err;
			}
		}
	}
out:
	ret = regmap_field_write(data->rm_field[F_WDT_EN], en);
	if (ret) {
		dev_err(data->dev, "Failed to enable WDT\n");
		goto vsys_overshot_err;
	}

	ret = regmap_field_write(data->rm_field[F_CHG_EN], en);
vsys_overshot_err:
	mutex_unlock(&data->ieoc_lock);
	mutex_unlock(&data->ichg_lock);
	return ret;
}

static int rt9467_psy_set_ichg(struct rt9467_chg_data *data, unsigned int ichg)
{
	int ret;

	mutex_lock(&data->ichg_lock);
	mutex_lock(&data->ieoc_lock);
	ret = __rt9467_set_ichg(data, ichg);
	mutex_unlock(&data->ichg_lock);
	mutex_unlock(&data->ieoc_lock);

	return ret;
}

static int rt9467_psy_set_cv(struct rt9467_chg_data *data, unsigned int cv)
{
	unsigned int sel = 0;

	linear_range_get_selector_within(rt9467_lranges + RT9467_RANGE_VOREG, cv,
					 &sel);
	dev_info(data->dev, "%s cv = %d(0x%02X)\n", __func__, cv, sel);
	return regmap_field_write(data->rm_field[F_VOREG], sel);
}

static int rt9467_psy_set_aicr(struct rt9467_chg_data *data, unsigned int aicr)
{
	unsigned int sel = 0;
	int ret;

	mutex_lock(&data->aicr_lock);
	linear_range_get_selector_within(rt9467_lranges + RT9467_RANGE_IAICR,
					 aicr, &sel);
	dev_info(data->dev, "%s aicr = %d(0x%02X)\n", __func__, aicr, sel);
	ret = regmap_field_write(data->rm_field[F_IAICR], sel);
	mutex_unlock(&data->aicr_lock);

	return ret;
}

static int rt9467_psy_set_mivr(struct rt9467_chg_data *data, unsigned int mivr)
{
	unsigned int sel = 0;

	linear_range_get_selector_within(rt9467_lranges + RT9467_RANGE_VMIVR,
					 mivr, &sel);
	dev_info(data->dev, "%s mivr = %d(0x%02X)\n", __func__, mivr, sel);
	return regmap_field_write(data->rm_field[F_VMIVR], sel);
}

static int rt9467_psy_set_ieoc(struct rt9467_chg_data *data, unsigned int ieoc)
{
	int ret;

	mutex_lock(&data->ichg_lock);
	mutex_lock(&data->ieoc_lock);
	ret = __rt9467_set_ieoc(data, ieoc);
	mutex_unlock(&data->ieoc_lock);
	mutex_unlock(&data->ichg_lock);

	return ret;
}

static void rt9467_psy_attach_pre_process(struct rt9467_chg_data *data,
					 enum rt9467_attach_trigger trig,
					 bool attach);
static int rt9467_charger_set_online(struct rt9467_chg_data *data,
				const union power_supply_propval *val)
{
	dev_info(data->dev, "%s\n", __func__);
	rt9467_psy_attach_pre_process(data, ATTACH_TRIG_TYPEC, val->intval);
	return 0;
}

static int rt9467_psy_set_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	struct rt9467_chg_data *data = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = rt9467_psy_enable_charging(data, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = rt9467_psy_set_ichg(data, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = rt9467_psy_set_cv(data, val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = rt9467_psy_set_aicr(data, val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = rt9467_psy_set_mivr(data, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = rt9467_psy_set_ieoc(data, val->intval);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = rt9467_charger_set_online(data, val);
		break;
	case POWER_SUPPLY_PROP_TYPE:
		data->psy_desc.type = val->intval;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int rt9467_register_psy(struct rt9467_chg_data *data)
{
	struct power_supply_config cfg = {
		.drv_data		= data,
		.of_node		= data->dev->of_node,
		.supplied_to		= rt9467_psy_supplied_to,
		.num_supplicants	= ARRAY_SIZE(rt9467_psy_supplied_to),
	};
	struct power_supply_desc desc = {
		.name			= dev_name(data->dev),
		.type			= POWER_SUPPLY_TYPE_USB,
		.usb_types		= rt9467_chg_usb_types,
		.num_usb_types		= ARRAY_SIZE(rt9467_chg_usb_types),
		.properties		= rt9467_chg_properties,
		.num_properties		= ARRAY_SIZE(rt9467_chg_properties),
		.property_is_writeable	= rt9467_chg_prop_is_writeable,
		.get_property		= rt9467_psy_get_property,
		.set_property		= rt9467_psy_set_property,
	};

	memcpy(&data->psy_desc, &desc, sizeof(data->psy_desc));
	
	data->psy_desc.name = "charger";
	
	data->psy = devm_power_supply_register(data->dev, &data->psy_desc,&cfg);
	return IS_ERR(data->psy) ? PTR_ERR(data->psy) : 0;
}

#if IS_ENABLED(CONFIG_MTK_CHARGER)
static int rt9467_set_usbsw_state(struct rt9467_chg_data *data, int usbsw)
{
	struct phy *phy;
	int ret, mode = (usbsw == USBSW_CHG) ? PHY_MODE_BC11_SET :
					       PHY_MODE_BC11_CLR;

	dev_info(data->dev, "usbsw=%d\n", usbsw);
	phy = phy_get(data->dev, "usb2-phy");
	if (IS_ERR_OR_NULL(phy)) {
		dev_notice(data->dev, "failed to get usb2-phy\n");
		return -ENODEV;
	}
	ret = phy_set_mode_ext(phy, PHY_MODE_USB_DEVICE, mode);
	if (ret)
		dev_notice(data->dev, "failed to set phy ext mode\n");
	phy_put(data->dev, phy);
	return ret;
}

static bool is_usb_rdy(struct device *dev)
{
	bool ready = true;
	struct device_node *node = NULL;

	node = of_parse_phandle(dev->of_node, "usb", 0);
	if (node) {
		ready = !of_property_read_bool(node, "cdp-block");
		dev_info(dev, "usb ready = %d\n", ready);
	} else
		dev_notice(dev, "usb node missing or invalid\n");
	return ready;
}
#endif /*CONFIG_MTK_CHARGER*/

static int rt9467_chg_enable_bc12(struct rt9467_chg_data *data, bool en)
{
#if IS_ENABLED(CONFIG_MTK_CHARGER)
	const int max_wait_cnt = 200;
	int i = 0, ret = 0;

	__pm_stay_awake(data->cdp_work_ws);

	if (en) {
		/* Workaround for CDP port */
		for (i = 0; i < max_wait_cnt && !is_usb_rdy(data->dev); i++) {
			dev_dbg(data->dev, "%s CDP block\n", __func__);
			if (!atomic_read(&data->attach)) {
				dev_info(data->dev, "%s plug out\n", __func__);
				goto relax_and_wait;
			}
			msleep(100);
		}
		if (i >= max_wait_cnt)
			dev_err(data->dev, "%s CDP timeout\n", __func__);
		else
			dev_info(data->dev, "%s CDP free\n", __func__);

		ret = regmap_field_write(data->rm_field[F_USBCHGEN], false);
		if (ret < 0) {
			dev_err(data->dev, "%s disable USBCHGEN fail(%d)\n",
				__func__, ret);
			goto relax_and_wait;
		}
		udelay(40);
	}
	rt9467_set_usbsw_state(data, en ? USBSW_CHG : USBSW_USB);
#endif /*CONFIG_MTK_CHARGER*/
	ret = regmap_field_write(data->rm_field[F_USBCHGEN], en);
	if (ret < 0)
		dev_err(data->dev, "%s en = %d fail(%d)\n",
				      __func__, en, ret);
#if IS_ENABLED(CONFIG_MTK_CHARGER)
relax_and_wait:
	__pm_relax(data->cdp_work_ws);
#endif /*CONFIG_MTK_CHARGER*/

	return ret;
}

static void rt9467_chg_bc12_work_func(struct work_struct *work)
{
	int ret;
	unsigned int val;
	bool bc12_ctrl = true, bc12_en = false, rpt_psy = true, attach;
	struct rt9467_chg_data *data = container_of(work,
						    struct rt9467_chg_data,
						    bc12_work);

	mutex_lock(&data->attach_lock);
	attach = atomic_read(&data->attach);
	if (attach) {
#if IS_ENABLED(CONFIG_MTK_CHARGER)
		if (data->boot_mode == 5) {
			/* skip bc12 to speed up ADVMETA_BOOT */
			dev_notice(data->dev, "Force SDP in meta mode\n");
			data->psy_desc.type = POWER_SUPPLY_TYPE_USB;
			data->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
			data->bc12_dn = false;
			goto out;
		}
#endif /* CONFIG_MTK_CHARGER */
		if (!data->bc12_dn) {
			bc12_en = true;
			rpt_psy = false;
			goto out;
		}
		data->bc12_dn = false;
		ret = regmap_field_read(data->rm_field[F_USB_STATUS], &val);
		if (ret) {
			dev_err(data->dev, "Failed to get port stat\n");
			rpt_psy = false;
			goto out;
		}
		switch (val) {
		case RT9467_CHG_TYPE_NOVBUS:
			bc12_ctrl = false;
			rpt_psy = false;
			break;
		case RT9467_CHG_TYPE_UNDER_GOING:
			bc12_en = true;
			rpt_psy = false;
			break;
		case RT9467_CHG_TYPE_SDP:
			data->psy_desc.type = POWER_SUPPLY_TYPE_USB;
			data->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
			break;
		case RT9467_CHG_TYPE_SDPNSTD:
			data->psy_desc.type = POWER_SUPPLY_TYPE_USB;
			data->psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
			break;
		case RT9467_CHG_TYPE_DCP:
			data->psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
			data->psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
			break;
		case RT9467_CHG_TYPE_CDP:
			data->psy_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
			data->psy_usb_type = POWER_SUPPLY_USB_TYPE_CDP;
			break;
		default:
			bc12_ctrl = false;
			rpt_psy = false;
			dev_err(data->dev, "Unknown port stat %d\n", val);
			goto out;
		}
		dev_notice(data->dev, "port stat = %s\n",
			   rt9467_port_stat_names[val]);
	} else {
		data->bc12_dn = false;
		data->psy_desc.type = POWER_SUPPLY_TYPE_USB;
		data->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	}
out:
	mutex_unlock(&data->attach_lock);
	if (bc12_ctrl && (rt9467_chg_enable_bc12(data, bc12_en) < 0))
		dev_err(data->dev, "Failed to set bc12 = %d\n", bc12_en);

	if (rpt_psy)
		power_supply_changed(data->psy);
}

static void rt9467_psy_attach_pre_process(struct rt9467_chg_data *data,
					 enum rt9467_attach_trigger trig,
					 bool attach)
{
	dev_err(data->dev, "trig=%s,attach=%d\n",
		 rt9467_attach_trig_names[trig], attach);
	/* if attach trigger is not match, ignore it */
	if (data->attach_trig != trig) {
		dev_notice(data->dev, "trig=%s ignore\n",
			   rt9467_attach_trig_names[trig]);
		return;
	}
	atomic_set(&data->attach, attach);
	if (!queue_work(data->wq, &data->bc12_work))
		dev_notice(data->dev, "bc12 work already queued\n");
}

static void rt9467_chg_pwr_rdy_process(struct rt9467_chg_data *data)
{
	int ret = 0;

	dev_notice(data->dev, "%s\n", __func__);
	mutex_lock(&data->attach_lock);
	ret = regmap_field_read(data->rm_field[F_PWR_RDY], &data->vbus_rdy);
	if (ret) {
		dev_err(data->dev, "Falied to read pwr_rdy state\n");
		mutex_unlock(&data->attach_lock);
		return;
	}
	rt9467_psy_attach_pre_process(data, ATTACH_TRIG_PWR_RDY,
				      data->vbus_rdy);
	mutex_unlock(&data->attach_lock);
	return;
}

/* Prevent back boost */
static int rt9467_toggle_cfo(struct rt9467_chg_data *data)
{
	int ret;

	dev_info(data->dev, "%s\n", __func__);
	/* CFO off */
	ret = regmap_field_write(data->rm_field[F_CFO_EN], 0);
	if (ret) {
		dev_err(data->dev, "Failed to do cfo off\n", ret);
		return ret;
	}

	/* CFO on */
	return regmap_field_write(data->rm_field[F_CFO_EN], 1);
}

static int __rt9467_kick_wdt(struct rt9467_chg_data *data)
{
	union power_supply_propval val;

	return power_supply_get_property(data->psy, POWER_SUPPLY_PROP_STATUS,
					 &val);
}

static int __maybe_unused __rt9467_run_aicl(struct rt9467_chg_data *data,
					    unsigned int *uA)
{
	int ret = 0;
	unsigned int mivr_act, aicl_vth, sel = 0;
	enum power_supply_property psp = POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT;
	union power_supply_propval psy_val;
	long ret_comp;

	ret = regmap_field_read(data->rm_field[F_CHG_MIVR], &mivr_act);
	if (ret) {
		dev_err(data->dev, "Failed to read CHG_MIVR\n");
		return ret;;
	}

	if (!mivr_act) {
		dev_info(data->dev, "mivr loop is not activr\n");
		return 0;
	}

	ret = power_supply_get_property(data->psy, psp, &psy_val);
	if (ret) {
		dev_err(data->dev, "Failed to get mivr\n");
		return ret;
	}

	/* Check if there's a suitable AICL_VTH */
	aicl_vth = psy_val.intval + 200000;
	linear_range_get_selector_within(rt9467_lranges + RT9467_RANGE_AICL_VTH,
					 aicl_vth, &sel);
	dev_info(data->dev, "%s: vth = %d(0x%02X)\n", __func__, aicl_vth, sel);
	ret = regmap_field_write(data->rm_field[F_AICL_VTH], sel);
	if (ret) {
		dev_err(data->dev, "Failed to set vth\n");
		return ret;
	}

	ret = regmap_field_write(data->rm_field[F_AICL_MEAS], 1);
	if (ret) {
		dev_err(data->dev, "Failed to set aicl measure\n");
		return ret;
	}

	mutex_lock(&data->aicr_lock);
	reinit_completion(&data->aicl_done);
	ret_comp = wait_for_completion_interruptible_timeout(&data->aicl_done,
							msecs_to_jiffies(3500));
	if (ret_comp == 0)
		ret = -ETIMEDOUT;
	else if (ret_comp < 0)
		ret = -EINTR;
	else
		ret = 0;
	if (ret < 0) {
		dev_err(data->dev, "Failed to wait aicl(%d)\n", ret);
		goto out;
	}

	psp = POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT;
	ret = power_supply_get_property(data->psy, psp, &psy_val);
	if (ret) {
		dev_err(data->dev, "Failed to get aicr\n");
		goto out;
	}

	*uA = psy_val.intval;
	dev_info(data->dev, "AICR upper bound = %dmA\n", *uA / 1000);
out:
	mutex_unlock(&data->aicr_lock);
	return ret;
}

static void rt9467_enable_irq(struct rt9467_chg_data *data,
			      const char *name, bool en)
{
	int irqno = 0;
	struct platform_device *pdev = to_platform_device(data->dev);

	dev_info(data->dev, "%s: en = %d, name = %s\n", __func__, en, name);
	irqno = platform_get_irq_byname(pdev, name);
	if (irqno < 0)
		dev_err(data->dev, "%s: cannot find irq\n", __func__);
	(en ? enable_irq : disable_irq_nosync)(irqno);
}

static void rt9467_chg_mivr_dwork_handler(struct work_struct *work)
{
	struct delayed_work *delayed_work = to_delayed_work(work);
	struct rt9467_chg_data *data = container_of(delayed_work,
			struct rt9467_chg_data, mivr_dwork);

	rt9467_enable_irq(data, "chg_mivr", true);
}

static irqreturn_t rt9467_chg_mivr_handler(int irqno, void *priv)
{
	struct rt9467_chg_data *data = priv;
	unsigned int mivr_act;
	int ret, ibus;

	dev_notice(data->dev, "%s\n", __func__);
	ret = regmap_field_read(data->rm_field[F_CHG_MIVR], &mivr_act);
	if (ret) {
		dev_err(data->dev, "Failed to read CHG_MIVR\n");
		goto out;
	}

	if (!mivr_act) {
		dev_info(data->dev, "mivr loop is not active\n");
		goto out;
	}

	ret = __rt9467_get_adc(data, RT9467_ADC_IBUS, &ibus);
	if (ret) {
		dev_err(data->dev, "Failed to get IBUS\n");
		goto out;
	}

	if (ibus < 100000) { /* 100mA */
		ret = rt9467_toggle_cfo(data);
		if (ret)
			dev_err(data->dev, "Failed to toggle cfo\n");
		goto out;
	}

out:
	rt9467_enable_irq(data, "chg_mivr", false);
	schedule_delayed_work(&data->mivr_dwork, msecs_to_jiffies(500));
	return IRQ_HANDLED;
}

static irqreturn_t rt9467_pwr_rdy_handler(int irqno, void *priv)
{
	struct rt9467_chg_data *data = priv;

	dev_notice(data->dev, "%s\n", __func__);
	rt9467_chg_pwr_rdy_process(data);

	return IRQ_HANDLED;
}

static irqreturn_t rt9467_wdt_handler(int irqno, void *priv)
{
	struct rt9467_chg_data *data = priv;
	int ret;

	dev_notice(data->dev, "%s\n", __func__);
	ret = __rt9467_kick_wdt(data);
	if (ret) {
		dev_err(data->dev, "Failed to kick wdt (%d)\n", ret);
		return IRQ_RETVAL(ret);
	}

	return IRQ_HANDLED;
}

static irqreturn_t rt9467_attach_handler(int irqno, void *priv)
{
	struct rt9467_chg_data *data = priv;

	dev_info(data->dev, "%s\n", __func__);
	mutex_lock(&data->attach_lock);
	data->bc12_dn = true;
	mutex_unlock(&data->attach_lock);
	if (!queue_work(data->wq, &data->bc12_work))
		dev_notice(data->dev, "%s bc12 work already queued\n",
			   __func__);

	return IRQ_HANDLED;
}

static irqreturn_t rt9467_detach_handler(int irqno, void *priv)
{
	struct rt9467_chg_data *data = priv;

	dev_notice(data->dev, "%s\n", __func__);
	rt9467_chg_pwr_rdy_process(data);
	return IRQ_HANDLED;
}

static irqreturn_t rt9467_aiclmeas_handler(int irqno, void *priv)
{
	struct rt9467_chg_data *data = priv;

	dev_notice(data->dev, "%s\n", __func__);
	complete(&data->aicl_done);
	return IRQ_HANDLED;
}

static const struct {
	const char *irq_name;
	irq_handler_t handler;
} rt9467_irqs[] = {
	{ "chg_mivr",	rt9467_chg_mivr_handler },
	{ "pwr_rdy",	rt9467_pwr_rdy_handler },
	{ "wdt",	rt9467_wdt_handler },
	{ "attach", 	rt9467_attach_handler },
	{ "detach", 	rt9467_detach_handler },
	{ "aiclmeas",	rt9467_aiclmeas_handler },
};

/*
 * This function is used in shutdown function
 * Use i2c smbus directly
 */
static int rt9467_sw_reset(struct rt9467_chg_data *data)
{
	int ret;
	u8 evt[7];
	/* Register 0x01 ~ 0x10 */
	u8 rst_data[] = {
		0x10, 0x03, 0x23, 0x3C, 0x67, 0x0B, 0x4C, 0xA1,
		0x3C, 0x58, 0x2C, 0x02, 0x52, 0x05, 0x00, 0x10
	};
	u8 rt9467_irq_maskall[7] = {0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	dev_info(data->dev, "%s\n", __func__);
	/* Mask all irq */
	ret = regmap_raw_write(data->regmap, RT9467_REG_CHG_STATC_CTRL,
			       rt9467_irq_maskall,
			       ARRAY_SIZE(rt9467_irq_maskall));
	if (ret)
		dev_err(data->dev, "Failed to mask all irqs(%d)\n", ret);

	/* Read all irq */
	ret = regmap_raw_read(data->regmap, RT9467_REG_CHG_STATC, evt,
			      ARRAY_SIZE(evt));
	if (ret)
		dev_err(data->dev, "Failed to read all irqs(%d)\n", ret);

	/* Reset necessary registers */
	return regmap_raw_write(data->regmap, RT9467_REG_CHG_CTRL1, rst_data,
				ARRAY_SIZE(rst_data));
}

static int rt9467_disable_auto_sensing(struct rt9467_chg_data *data)
{
	int ret;

	ret = rt9467_enable_hidden_mode(data, true);
	if (ret) {
		dev_err(data->dev, "Failed to enter hidden mode\n");
		return ret;
	}

	ret = regmap_field_write(data->rm_field[F_AUTO_SENSE], 0);
	if (ret)
		dev_err(data->dev, "Failed to disable auto sense\n");

	return rt9467_enable_hidden_mode(data, false);
}

static ssize_t shipping_mode_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct rt9467_chg_data *data = dev_get_drvdata(dev);
	int ret;

	ret = rt9467_disable_auto_sensing(data);
	if (ret) {
		dev_err(data->dev, "Failed to disable auto sensing\n");
		return ret;
	}

	ret = rt9467_sw_reset(data);
	if (ret) {
		dev_err(data->dev, "Failed to do sw reset\n");
		return ret;
	}

	return regmap_field_write(data->rm_field[F_SHIP_MODE], 1);
}
static const DEVICE_ATTR_WO(shipping_mode);

#define DUMP_REG_BUF_SIZE	1024
static int __rt9467_dump_register(struct rt9467_chg_data *data)
{
	int ret, i;
	char buf[DUMP_REG_BUF_SIZE] = "\0";
	unsigned int val;
	union power_supply_propval psy_val;
	static struct {
		const char *name;
		const char *unit;
		enum power_supply_property psp;
	} settings[] = {
		{ .psp = POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
		  .name = "ICHG", .unit = "mA"},
		{ .psp = POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
		  .name = "MIVR", .unit = "mV"},
		{ .psp = POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
		  .name = "IEOC", .unit = "mA"},
		{ .psp = POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
		  .name = "CV", .unit = "mV"},
	};
	static struct {
		const char *name;
		const char *unit;
		enum rt9467_adc_chan chan;
	} adcs[] = {
		{ .chan = RT9467_ADC_VBUS_DIV5, .name = "VBUS", .unit = "mV" },
		{ .chan = RT9467_ADC_IBUS, .name = "IBUS", .unit = "mA" },
		{ .chan = RT9467_ADC_VBAT, .name = "VBAT", .unit = "mV" },
		{ .chan = RT9467_ADC_IBAT, .name = "IBAT", .unit = "mA" },
		{ .chan = RT9467_ADC_VSYS, .name = "VSYS", .unit = "mV" },
	};
	static const struct {
		const u8 reg;
		const char *name;
	} regs[] = {
		{ .reg = RT9467_REG_CHG_CTRL1, .name = "CHG_CTRL1" },
		{ .reg = RT9467_REG_CHG_CTRL2, .name = "CHG_CTRL2" },
		{ .reg = RT9467_REG_CHG_STATC, .name = "CHG_STATC" },
		{ .reg = RT9467_REG_CHG_STAT, .name = "CHG_STAT" },
	};

	for (i = 0; i < ARRAY_SIZE(settings); i++) {
		ret = power_supply_get_property(data->psy, settings[i].psp,
						&psy_val);
		if (ret) {
			dev_err(data->dev, "Failed to get %s\n",
				settings[i].name);
			return ret;
		}
		val = psy_val.intval / 1000;
		if (i == ARRAY_SIZE(settings) - 1)
			scnprintf(buf + strlen(buf), DUMP_REG_BUF_SIZE,
				  "%s = %d%s\n", settings[i].name, val,
				  settings[i].unit);
		else
			scnprintf(buf + strlen(buf), DUMP_REG_BUF_SIZE,
				  "%s = %d%s, ", settings[i].name, val,
				  settings[i].unit);
	}

	for (i = 0; i < ARRAY_SIZE(adcs); i++) {
		ret = __rt9467_get_adc(data, adcs[i].chan, &val);
		if (ret) {
			dev_err(data->dev, "Failed to read adc %s\n",
				adcs[i].name);
			return ret;
		}
		val /= 1000;
		if (i == ARRAY_SIZE(settings) - 1)
			scnprintf(buf + strlen(buf), DUMP_REG_BUF_SIZE,
				  "%s = %d%s\n", adcs[i].name, val,
				  adcs[i].unit);
		else
			scnprintf(buf + strlen(buf), DUMP_REG_BUF_SIZE,
				  "%s = %d%s, ", adcs[i].name, val,
				  adcs[i].unit);
	}

	for (i =0; i < ARRAY_SIZE(regs); i++) {
		ret = regmap_read(data->regmap, regs[i].reg, &val);
		if (ret) {
			dev_err(data->dev, "Failed to get %s\n", regs[i].name);
			return ret;
		}
		if (i == ARRAY_SIZE(regs) - 1)
			scnprintf(buf + strlen(buf), DUMP_REG_BUF_SIZE,
				  "%s = 0x%02X\n", regs[i].name, val);
		else
			scnprintf(buf + strlen(buf), DUMP_REG_BUF_SIZE,
				  "%s = 0x%02X, ", regs[i].name, val);
	}
	dev_info(data->dev, "%s %s", __func__, buf);
	return 0;
}

static void rt9467_init_setting_work_handler(struct work_struct *work)
{
	int ret;
	struct rt9467_chg_data *data = container_of(work,
						    struct rt9467_chg_data,
						    init_work);
#if IS_ENABLED(CONFIG_MTK_CHARGER)
	union power_supply_propval val;
	enum power_supply_property psp = POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT;
#endif /* CONFIG_MTK_CHARGER  */

	ret = rt9467_disable_auto_sensing(data);
	if (ret)
		dev_err(data->dev, "Failed to disable auto sensing\n");

	ret = regmap_field_write(data->rm_field[F_TE], 1);
	if (ret)
		dev_err(data->dev, "Failed to set TE\n");

	/* Select IINLMTSEL to use AICR */
	ret = regmap_field_write(data->rm_field[F_IINLMTSEL],
				 RT9467_IINLMTSEL_AICR);
	if (ret)
		dev_err(data->dev, "Failed to set iinlmtsel to AICR\n");
	msleep(150);

	/* Disable hardware ILIM */
	ret = regmap_field_write(data->rm_field[F_ILIM_EN], 0);
	if (ret)
		dev_err(data->dev, "Failed to disable hardware ILIM\n");

	/* Set 12hrs fast charger timer */
	ret = regmap_field_write(data->rm_field[F_WT_FC], 4);
	if (ret)
		dev_err(data->dev, "Failed to set WT_FC\n");

#if IS_ENABLED(CONFIG_MTK_CHARGER)
	/* Set AICR = 200mA in 1:META_BOOT, 5:ADVMETA_BOOT */
	if (data->boot_mode == 1 || data->boot_mode == 5) {
		val.intval = 200000;
		ret = power_supply_set_property(data->psy, psp, &val);
		if (ret)
			dev_err(data->dev, "Failed to set aicr 200mA\n");
	}
#endif /* CONFIG_MTK_CHARGER  */

	/* unmask pwrdy, mivr */
	ret = regmap_write(data->regmap, RT9467_REG_CHG_STATC_CTRL, 0x3f);
	if (ret)
		dev_err(data->dev, "Failed to unmask pwrdy and mivr\n");
	__rt9467_dump_register(data);
}

#if IS_ENABLED(CONFIG_MTK_CHARGER)
static int rt9467_plug_in(struct charger_device *chg_dev)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	union power_supply_propval val;

	dev_info(data->dev, "%s\n", __func__);
	val.intval = 1;
	return power_supply_set_property(data->psy, POWER_SUPPLY_PROP_STATUS,
					 &val);
}

static int rt9467_plug_out(struct charger_device *chg_dev)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	union power_supply_propval val;


	dev_info(data->dev, "%s\n", __func__);
	val.intval = 0;
	return power_supply_set_property(data->psy, POWER_SUPPLY_PROP_STATUS,
					 &val);
}

static int rt9467_dump_register(struct charger_device *chg_dev)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);

	return __rt9467_dump_register(data);
}

static int rt9467_enable_charging(struct charger_device *chg_dev, bool en)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);

	dev_info(data->dev, "%s: en = %d\n", __func__, en);
	return regmap_field_write(data->rm_field[F_CHG_EN], en);
}


static int rt9467_enable_hz(struct charger_device *chg_dev, bool en)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);

	dev_info(data->dev, "%s: en = %d\n", __func__, en);
	return regmap_field_write(data->rm_field[F_HZ], en);
}

static int rt9467_is_charging_enable(struct charger_device *chg_dev, bool *en)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	unsigned int val;
	int ret;

	ret = regmap_field_read(data->rm_field[F_CHG_EN], &val);
	if (ret)
		return ret;

	*en = val;
	dev_info(data->dev, "%s en = %d\n", __func__, *en);
	return 0;
}

static int rt9467_is_chip_enabled(struct charger_device *chg_dev, bool *en)
{
	*en = true;
	return 0;
}

static int rt9467_get_ichg(struct charger_device *chg_dev, u32 *ichg)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	union power_supply_propval val;
	enum power_supply_property psp =
				      POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT;
	int ret;

	ret = power_supply_get_property(data->psy, psp, &val);
	if (ret) {
		dev_err(data->dev, "%s get ichg failed\n", __func__);
		return ret;
	}

	*ichg = val.intval;
	dev_info(data->dev, "%s: ichg = %d\n", __func__, *ichg);
	return 0;
}

static int rt9467_set_ichg(struct charger_device *chg_dev, u32 ichg)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	union power_supply_propval val;
	enum power_supply_property psp =
				      POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT;

	dev_info(data->dev, "%s: ichg = %d\n", __func__, ichg);

	val.intval = ichg;
	return power_supply_set_property(data->psy, psp, &val);
}

static int rt9467_get_aicr(struct charger_device *chg_dev, u32 *aicr)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	union power_supply_propval val;
	enum power_supply_property psp = POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT;
	int ret;

	ret = power_supply_get_property(data->psy, psp, &val);
	if (ret) {
		dev_err(data->dev, "%s get aicr failed\n", __func__);
		return ret;
	}

	*aicr = val.intval;
	dev_info(data->dev, "%s: aicr = %d\n", __func__, *aicr);
	return 0;
}

static int rt9467_set_aicr(struct charger_device *chg_dev, u32 aicr)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	union power_supply_propval val;
	enum power_supply_property psp = POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT;

	dev_info(data->dev, "%s: aicr = %d\n", __func__, aicr);

	val.intval = aicr;
	return power_supply_set_property(data->psy, psp, &val);
}

static int rt9467_get_cv(struct charger_device *chg_dev, u32 *cv)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	union power_supply_propval val;
	enum power_supply_property psp =
				      POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE;
	int ret;

	ret = power_supply_get_property(data->psy, psp, &val);
	if (ret) {
		dev_err(data->dev, "%s get cv failed\n", __func__);
		return ret;
	}

	*cv = val.intval;
	dev_info(data->dev, "%s: cv = %d\n", __func__, *cv);
	return 0;
}

static int rt9467_set_cv(struct charger_device *chg_dev, u32 cv)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	union power_supply_propval val;
	enum power_supply_property psp =
				      POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE;

	dev_info(data->dev, "%s: cv = %d\n", __func__, cv);

	val.intval = cv;
	return power_supply_set_property(data->psy, psp, &val);
}

static int rt9467_kick_wdt(struct charger_device *chg_dev)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);

	return __rt9467_kick_wdt(data);
}
static int rt9467_get_mivr(struct charger_device *chg_dev, u32 *mivr)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	union power_supply_propval val;
	enum power_supply_property psp = POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT;
	int ret;

	ret = power_supply_get_property(data->psy, psp, &val);
	if (ret) {
		dev_err(data->dev, "%s get mivr failed\n", __func__);
		return ret;
	}

	*mivr = val.intval;
	dev_info(data->dev, "%s: mivr = %d\n", __func__, *mivr);
	return 0;
}

static int rt9467_set_mivr(struct charger_device *chg_dev, u32 mivr)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	union power_supply_propval val = {.intval = mivr};
	enum power_supply_property psp = POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT;
	int ret = 0;

	mutex_lock(&data->pp_lock);
	if (!data->pp_en) {
		dev_err(data->dev, "%s: power path is disabled\n",
			__func__);
		goto out;
	} else
		dev_info(data->dev, "%s: mivr = %d\n", __func__, mivr);

	data->mivr = mivr;
	ret =  power_supply_set_property(data->psy, psp, &val);
out:
	mutex_unlock(&data->pp_lock);
	return ret;
}

static int rt9467_get_mivr_state(struct charger_device *chg_dev, bool *in_loop)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	unsigned int val;
	int ret;

	ret = regmap_field_read(data->rm_field[F_CHG_MIVR], &val);
	if (ret) {
		dev_err(data->dev, "%s: failed to get mivr_state\n");
		return ret;
	}

	*in_loop = !!val;
	return 0;
}
static int rt9467_is_charging_done(struct charger_device *chg_dev, bool *done)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	union power_supply_propval val;
	int ret;

	ret = power_supply_get_property(data->psy, POWER_SUPPLY_PROP_STATUS,
					&val);
	if (ret) {
		dev_err(data->dev, "Failed to get charging state\n");
		return ret;
	}

	*done = val.intval == POWER_SUPPLY_STATUS_FULL;
	dev_info(data->dev, "%s: done = %d\n", __func__, *done);
	return ret;
}

static int rt9467_get_min_ichg(struct charger_device *chg_dev, u32 *uA)
{
	*uA = RT9467_ICHG_MINUA;
	return 0;
}

static int rt9467_set_ieoc(struct charger_device *chg_dev, u32 ieoc)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	union power_supply_propval val;
	enum power_supply_property psp = POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT;

	dev_info(data->dev, "%s: ieoc = %d\n", __func__, ieoc);

	val.intval = ieoc;
	return power_supply_set_property(data->psy, psp, &val);
}

static int rt9467_enable_te(struct charger_device *chg_dev, bool en)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);

	return regmap_field_write(data->rm_field[F_TE], en);
}

static int rt9467_run_aicl(struct charger_device *chg_dev, u32 *uA)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);

	return __rt9467_run_aicl(data, uA);
}

static int rt9467_reset_eoc_state(struct charger_device *chg_dev)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	int ret;

	/* Toggle EOC_RST */
	ret = rt9467_enable_hidden_mode(data, true);
	if (ret) {
		dev_err(data->dev, "Failed to enable hidden mode\n");
		return ret;
	}

	ret = regmap_field_write(data->rm_field[F_EOC_RST], 1);
	if (ret) {
		dev_err(data->dev, "Failed to set eoc rst\n");
		goto out;
	}

	ret = regmap_field_write(data->rm_field[F_EOC_RST], 0);
	if (ret) {
		dev_err(data->dev, "Failed to clear eoc rst\n");
		goto out;
	}

out:
	return rt9467_enable_hidden_mode(data, false);
}

static int rt9467_safety_check(struct charger_device *chg_dev, u32 uA)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	int ret, ibat = 0;

	ret = __rt9467_get_adc(data, RT9467_ADC_IBAT, &ibat);
	if (ret) {
		dev_err(data->dev, "Failed to get ibat\n");
		return ret;
	}
	if (ibat <= uA) {
		/* if it happens 3 times, trigger EOC event */
		if (atomic_read(&data->eoc_cnt) == 2) {
			atomic_set(&data->eoc_cnt, 0);
			dev_notice(data->dev, "ieoc=%d, ibat=%d\n", uA, ibat);
			charger_dev_notify(data->chg_dev,
					   CHARGER_DEV_NOTIFY_EOC);
		} else
			atomic_inc(&data->eoc_cnt);
	} else
		atomic_set(&data->eoc_cnt, 0);
	return 0;
}

static int rt9467_enable_safety_timer(struct charger_device *chg_dev, bool en)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);

	return regmap_field_write(data->rm_field[F_TMR_EN], en);
}

static int rt9467_is_safety_timer_enable(struct charger_device *chg_dev,
					 bool *en)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	unsigned int val;
	int ret;

	ret = regmap_field_read(data->rm_field[F_TMR_EN], &val);
	if (ret) {
		dev_err(data->dev, "Failed to read F_TMR_EN\n");
		return ret;
	}

	*en = !!val;
	return 0;
}

static int rt9467_enable_power_path(struct charger_device *chg_dev, bool en)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	u32 mivr = (en ? data->mivr: 13400000);
	union power_supply_propval val = {.intval = mivr};
	int ret = 0;

	mutex_lock(&data->pp_lock);
	dev_info(data->dev, "%s: en = %d, pp_en = %d\n",
				__func__, en, data->pp_en);
	if (en == data->pp_en)
		goto out;

	if (!en)
		rt9467_enable_irq(data, "chg_mivr", false);
	ret = power_supply_set_property(data->psy,
			POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT, &val);
	if (en)
		rt9467_enable_irq(data, "chg_mivr", true);
	data->pp_en = en;
out:
	mutex_unlock(&data->pp_lock);
	return ret;
}

static int rt9467_is_power_path_enable(struct charger_device *chg_dev, bool *en)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);

	mutex_lock(&data->pp_lock);
	*en = data->pp_en;
	mutex_unlock(&data->pp_lock);

	return 0;
}

static int rt9467_enable_chg_type_det(struct charger_device *chg_dev, bool en)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);

	dev_info(data->dev, "%s\n", __func__);
	rt9467_psy_attach_pre_process(data, ATTACH_TRIG_TYPEC, en);
	return 0;
}

static int rt9467_enable_otg(struct charger_device *chg_dev, bool en)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	struct regulator *regulator;
	int ret;

	dev_info(data->dev, "%s: en = %d\n", __func__, en);
	regulator = devm_regulator_get(data->dev, "usb-otg-vbus");
	if (IS_ERR(regulator)) {
		dev_err(data->dev, "failed to get otg regulator\n");
		return PTR_ERR(regulator);
	}
	ret = (en ? regulator_enable : regulator_disable)(regulator);
	devm_regulator_put(regulator);
	return 0;
}

static int rt9467_set_otg_cc(struct charger_device *chg_dev, u32 uA)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	struct regulator *regulator;

	dev_info(data->dev, "%s: uA = %d\n", __func__, uA);
	regulator = devm_regulator_get(data->dev, "usb-otg-vbus");
	if (IS_ERR(regulator)) {
		dev_err(data->dev, "failed to get otg regulator\n");
		return PTR_ERR(regulator);
	}
	return regulator_set_current_limit(regulator, uA, uA);
}

static int rt9467_enable_discharge(struct charger_device *chg_dev, bool en)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	int ret;

	dev_info(data->dev, "%s: en = %d\n", __func__, en);

	ret = rt9467_enable_hidden_mode(data, true);
	if (ret) {
		dev_err(data->dev, "Failed to enable hidden mode\n");
		return ret;
	}

	ret = regmap_field_write(data->rm_field[F_DISCHG_EN], en);
	if (ret)
		dev_err(data->dev, "Failed to set discharge = %d\n", en);

	return rt9467_enable_hidden_mode(data, false);
}

static int rt9467_enable_pump_express(struct rt9467_chg_data *data, bool en)
{
	int ret = 0, i = 0;
	bool pumpx_en = false;
	const int max_wait_times = 3;

	dev_info(data->dev, "%s: en = %d\n", __func__, en);
	ret = rt9467_set_aicr(data->chg_dev, 800000);
	if (ret)
		return ret;

	ret = rt9467_set_ichg(data->chg_dev, 2000000);
	if (ret)
		return ret;

	ret = rt9467_enable_charging(data->chg_dev, true);
	if (ret)
		return ret;

	ret = rt9467_enable_hidden_mode(data, true);
	if (ret)
		return ret;

	ret = regmap_field_write(data->rm_field[F_EN_PSK], 0);
	if (ret)
		dev_notice(data->dev, "Failed to disable skip mode\n");

	ret = regmap_field_write(data->rm_field[F_EN_PUMPX], en);
	if (ret)
		goto out;

	for (i = 0; i < max_wait_times; i++) {
		msleep(2500);
		pumpx_en = regmap_test_bits(data->regmap, RT9467_REG_CHG_CTRL17,
					    RT9467_MASK_EN_PUMPX);
		if (pumpx_en == 0)
			break;
	}
	if (i == max_wait_times)
		dev_notice(data->dev, "Falied to wait pumpx done\n");
out:

	ret = regmap_field_write(data->rm_field[F_EN_PSK], 1);
	if (ret)
		dev_notice(data->dev, "Failed to enable skip mode\n");
	return rt9467_enable_hidden_mode(data, false);
}

static int rt9467_set_pep_current_pattern(struct charger_device *chg_dev,
					  bool inc)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	int ret;

	dev_info(data->dev, "%s: inc=%d\n", __func__, inc);
	mutex_lock(&data->pe_lock);
	/* Set to PE1.0 */
	ret = regmap_field_write(data->rm_field[F_PUMPX_20_10], 0);
	if (ret) {
		dev_err(data->dev, "Failed to set pe1.0\n");
		goto out;
	}

	/* Set Pump Up/Down */
	ret = regmap_field_write(data->rm_field[F_PUMPX_UP_DN], inc);
	if (ret) {
		dev_err(data->dev, "Faild to set up/dn\n");
		goto out;
	}

	/* Enable Pumpx */
	ret = rt9467_enable_pump_express(data, true);
out:
	mutex_unlock(&data->pe_lock);
	return ret;
}

static int rt9467_set_pep20_efficiency_table(struct charger_device *chg_dev)
{
	return 0;
}

static int rt9467_set_pep20_current_pattern(struct charger_device *chg_dev,
					    u32 uV)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	int ret;
	unsigned int sel = 0;

	mutex_lock(&data->pe_lock);
	linear_range_get_selector_within(rt9467_lranges +
					 RT9467_RANGE_PUMPX_DEC, uV, &sel);
	dev_info(data->dev, "volt = %d(0x%02X)\n", uV, sel);

	ret = regmap_field_write(data->rm_field[F_PUMPX_20_10], 1);
	if (ret)
		goto out;

	ret = regmap_field_write(data->rm_field[F_PUMPX_DEC], sel);
	if (ret)
		goto out;

	ret = rt9467_enable_pump_express(data, true);
out:
	mutex_unlock(&data->pe_lock);
	return ret;
}

static int rt9467_set_pep20_reset(struct charger_device *chg_dev)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	int ret;

	mutex_lock(&data->pe_lock);
	ret = rt9467_set_mivr(chg_dev, 4500000);
	if (ret)
		goto out;

	/* Disable PSK mode */
	ret = rt9467_enable_hidden_mode(data, true);
	if (ret)
		goto out;

	ret = regmap_field_write(data->rm_field[F_EN_PSK], 0);
	if (ret)
		dev_notice(data->dev, "Failed to disable skip mode\n");

	ret = rt9467_set_aicr(chg_dev, 100000);
	if (ret)
		goto psk_out;

	msleep(250);
	ret = rt9467_set_aicr(chg_dev, 700000);
psk_out:
	ret = regmap_field_write(data->rm_field[F_EN_PSK], 1);
	ret |= rt9467_enable_hidden_mode(data, false);
out:
	mutex_unlock(&data->pe_lock);
	return ret;
}

static int rt9467_enable_cable_drop_comp(struct charger_device *chg_dev,
					 bool en)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	int ret;

	dev_info(data->dev, "%s: en = %d\n", __func__, en);
	mutex_lock(&data->pe_lock);
	/* Set to PEP2.0 */
	ret = regmap_field_write(data->rm_field[F_PUMPX_20_10], 1);
	if (ret)
		goto out;

	/* Set Voltage */
	ret = regmap_field_write(data->rm_field[F_PUMPX_DEC], 0x1F);
	if (ret)
		goto out;

	ret = rt9467_enable_pump_express(data, true);
out:
	mutex_unlock(&data->pe_lock);
	return ret;
}

static int rt9467_get_tchg(struct charger_device *chg_dev, int *min, int *max)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	int ret, adc_tmp;
	u8 rty_cnt = 3;

	/* Check unusual temperature */
	do {
		ret = __rt9467_get_adc(data, RT9467_ADC_TEMP_JC, &adc_tmp);
		if (ret)
			return ret;
		if (adc_tmp < 120)
			break;
		dev_notice(data->dev, "%s: [WARNING] t = %d\n", __func__,
			   adc_tmp);
		rty_cnt--;
	} while (adc_tmp >= 120 && rty_cnt > 0);

	/* Use previous one to prevent system from rebooting */
	if (adc_tmp >= 120)
		adc_tmp = data->tchg;
	else
		data->tchg = adc_tmp;
	*min = *max = adc_tmp;
	dev_info(data->dev, "tchg=%d\n", adc_tmp);
	return 0;
}

static int rt9467_get_vbus(struct charger_device *chg_dev, u32 *vbus)
{
	int ret = 0, adc_vbus = 0;
	struct rt9467_chg_data *data = charger_get_data(chg_dev);

	/* Get value from ADC */
	ret = __rt9467_get_adc(data, RT9467_ADC_VBUS_DIV5, &adc_vbus);
	if (ret < 0)
		return ret;

	*vbus = adc_vbus;
	dev_info(data->dev, "%s: vbus = %dmV\n", __func__, adc_vbus / 1000);
	return ret;
}

static int rt9467_get_ibus(struct charger_device *chg_dev, u32 *ibus)
{
	int ret = 0, adc_ibus = 0;
	struct rt9467_chg_data *data = charger_get_data(chg_dev);

	/* Get value from ADC */
	ret = __rt9467_get_adc(data, RT9467_ADC_IBUS, &adc_ibus);
	if (ret < 0)
		return ret;

	*ibus = adc_ibus;
	dev_info(data->dev, "%s: ibus = %dmA\n", __func__, adc_ibus / 1000);
	return ret;
}

static int rt9467_do_event(struct charger_device *chg_dev, u32 event, u32 args)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);

	switch (event) {
	case EVENT_FULL:
	case EVENT_RECHARGE:
	case EVENT_DISCHARGE:
		power_supply_changed(data->psy);
		break;
	default:
		break;
	}
	return 0;
}

static int rt9467_get_adc(struct charger_device *chg_dev, enum adc_channel chan,
			   int *min, int *max)
{
	struct rt9467_chg_data *data = charger_get_data(chg_dev);
	enum rt9467_adc_chan rt9467_chan;
	int val = 0, ret;

	switch (chan) {
	case ADC_CHANNEL_VBUS:
		rt9467_chan = RT9467_ADC_VBUS_DIV5;
		break;
	case ADC_CHANNEL_VSYS:
		rt9467_chan = RT9467_ADC_VSYS;
		break;
	case ADC_CHANNEL_VBAT:
		rt9467_chan = RT9467_ADC_VBAT;
		break;
	case ADC_CHANNEL_IBUS:
		rt9467_chan = RT9467_ADC_IBUS;
		break;
	case ADC_CHANNEL_IBAT:
		rt9467_chan = RT9467_ADC_IBAT;
		break;
	case ADC_CHANNEL_TEMP_JC:
		rt9467_chan = RT9467_ADC_TEMP_JC;
		break;
	case ADC_CHANNEL_TBAT:
		rt9467_chan = RT9467_ADC_TS_BAT;
		break;
	default:
		return -EINVAL;
	};

	ret = __rt9467_get_adc(data, rt9467_chan, &val);
	if(ret) {
		dev_err(data->dev, "Failed to get adc chan=%d (%d)\n", rt9467_chan, ret);
		return ret;
	}

	dev_info(data->dev, "%s: rt9467_chan = %d, val = %d\n", __func__, rt9467_chan, val / 1000);
	*min = *max = val;
	return ret;
}

static struct charger_ops rt9467_chg_ops = {
	/* Normal charging */
	.plug_in = rt9467_plug_in,
	.plug_out = rt9467_plug_out,
	.dump_registers = rt9467_dump_register,
	.enable = rt9467_enable_charging,
	.is_enabled = rt9467_is_charging_enable,
	.is_chip_enabled = rt9467_is_chip_enabled,
	.get_charging_current = rt9467_get_ichg,
	.set_charging_current = rt9467_set_ichg,
	.get_input_current = rt9467_get_aicr,
	.set_input_current = rt9467_set_aicr,
	.get_constant_voltage = rt9467_get_cv,
	.set_constant_voltage = rt9467_set_cv,
	.kick_wdt = rt9467_kick_wdt,
	.get_mivr = rt9467_get_mivr,
	.set_mivr = rt9467_set_mivr,
	.get_mivr_state = rt9467_get_mivr_state,
	.is_charging_done = rt9467_is_charging_done,
	.get_min_charging_current = rt9467_get_min_ichg,
	.set_eoc_current = rt9467_set_ieoc,
	.enable_termination = rt9467_enable_te,
	.run_aicl = rt9467_run_aicl,
	.reset_eoc_state = rt9467_reset_eoc_state,
	.safety_check = rt9467_safety_check,

	/* Safety timer */
	.enable_safety_timer = rt9467_enable_safety_timer,
	.is_safety_timer_enabled = rt9467_is_safety_timer_enable,

	/* Power path */
	.enable_powerpath = rt9467_enable_power_path,
	.is_powerpath_enabled = rt9467_is_power_path_enable,

	/* Charger type detection */
	.enable_chg_type_det = rt9467_enable_chg_type_det,

	/* OTG */
	.enable_otg = rt9467_enable_otg,
	.set_boost_current_limit = rt9467_set_otg_cc,
	.enable_discharge = rt9467_enable_discharge,

	/* PE+/PE+20 */
	.send_ta_current_pattern = rt9467_set_pep_current_pattern,
	.set_pe20_efficiency_table = rt9467_set_pep20_efficiency_table,
	.send_ta20_current_pattern = rt9467_set_pep20_current_pattern,
	.reset_ta = rt9467_set_pep20_reset,
	.enable_cable_drop_comp = rt9467_enable_cable_drop_comp,
	
	/*hz*/
	.enable_hz = rt9467_enable_hz,

	/* ADC */
	.get_adc = rt9467_get_adc,
	.get_tchg_adc = rt9467_get_tchg,
	.get_ibus_adc = rt9467_get_ibus,
	.get_vbus_adc = rt9467_get_vbus,

	/* Event */
	.event = rt9467_do_event,
};

static int rt9467_get_mtk_pdata(struct rt9467_chg_data *data)
{
	u32 val;
	const struct {
		u32 size;
		u32 tag;
		u32 boot_mode;
		u32 boot_type;
	} *tag;
	struct device_node *bc12_np, *boot_np, *np = data->dev->of_node;

	dev_info(data->dev, "%s\n", __func__);
	if (np) {
		/* mediatek chgdev name */
		if (of_property_read_string(np, "chg_name", &data->chg_name))
			dev_notice(data->dev, "Failed to get chg_name\n");

		/* mediatek boot mode */
		boot_np = of_parse_phandle(np, "boot_mode", 0);
		if (!boot_np) {
			dev_err(data->dev, "failed to get bootmode phandle\n");
			return -ENODEV;
		}
		tag = of_get_property(boot_np, "atag,boot", NULL);
		if (!tag) {
			dev_err(data->dev, "failed to get atag,boot\n");
			return -EINVAL;
		}
		dev_info(data->dev, "sz:0x%x tag:0x%x mode:0x%x type:0x%x\n",
			 tag->size, tag->tag, tag->boot_mode, tag->boot_type);
		data->boot_mode = tag->boot_mode;

		/*
		 * mediatek bc12_sel
		 * 0 means bc12 owner is THIS_MODULE,
		 * if it is not 0, always ignore
		 */
		bc12_np = of_parse_phandle(np, "bc12_sel", 0);
		if (!bc12_np) {
			dev_err(data->dev, "failed to get bc12_sel phandle\n");
			return -ENODEV;
		}
		if (of_property_read_u32(bc12_np, "bc12_sel", &val) < 0) {
			dev_err(data->dev, "property bc12_sel not found\n");
			return -EINVAL;
		}
		if (val != 0)
			data->attach_trig = ATTACH_TRIG_IGNORE;
		else if (IS_ENABLED(CONFIG_TCPC_CLASS))
			data->attach_trig = ATTACH_TRIG_TYPEC;
		else
			data->attach_trig = ATTACH_TRIG_PWR_RDY;
		data->dev->platform_data = data;
	}
	return 0;
}

static int rt9467_register_mtk_charger_dev(struct rt9467_chg_data *data)
{
	int ret;

	ret = rt9467_get_mtk_pdata(data);
	if (ret)
		return ret;
	mutex_init(&data->pe_lock);
	data->chg_prop.alias_name = data->chg_name;
	data->chg_dev = charger_device_register(data->chg_name, data->dev,
						data, &rt9467_chg_ops,
						&data->chg_prop);
	return IS_ERR(data->chg_dev) ? PTR_ERR(data->chg_dev) : 0;
}
#endif /* CONFIG_MTK_CHARGER */

static int rt9467_charger_probe(struct platform_device *pdev)
{
	struct rt9467_chg_data *data;
	int ret, i, irqno;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!data->regmap) {
		dev_err(&pdev->dev, "Failed to get parent regmap\n");
		return -ENODEV;
	}

	data->dev = &pdev->dev;
	mutex_init(&data->attach_lock);
	mutex_init(&data->hm_lock);
	mutex_init(&data->ichg_lock);
	mutex_init(&data->ieoc_lock);
	mutex_init(&data->aicr_lock);
	mutex_init(&data->pp_lock);
	atomic_set(&data->attach, 0);
	atomic_set(&data->eoc_cnt, 0);
	data->tchg = 25;
	data->mivr = 4500000;
	data->pp_en = true;
	init_completion(&data->aicl_done);
	data->wq = create_singlethread_workqueue(dev_name(data->dev));
	if (!data->wq) {
		dev_err(&pdev->dev, "Failed to create workqueue\n");
		return -ENOMEM;
	}
#if IS_ENABLED(CONFIG_MTK_CHARGER)
	data->cdp_work_ws =
		wakeup_source_register(data->dev,
				       devm_kasprintf(data->dev, GFP_KERNEL,
				       "rt9467_cdp_work_ws.%s",
				       dev_name(data->dev)));
#endif /* CONFIG_MTK_CHARGER */
	INIT_WORK(&data->bc12_work, rt9467_chg_bc12_work_func);
	INIT_WORK(&data->init_work, rt9467_init_setting_work_handler);
	INIT_DELAYED_WORK(&data->mivr_dwork, rt9467_chg_mivr_dwork_handler);
	platform_set_drvdata(pdev, data);

	ret = devm_regmap_field_bulk_alloc(&pdev->dev, data->regmap,
					   data->rm_field, rt9467_chg_fields,
					   ARRAY_SIZE(rt9467_chg_fields));
	if (ret) {
		dev_err(&pdev->dev, "Failed to alloc regmap fields\n");
		goto out;
	}

	data->attach_trig = ATTACH_TRIG_PWR_RDY;
	data->ceb_gpio = devm_gpiod_get(&pdev->dev, "ceb",
						 GPIOD_OUT_LOW);
	if (IS_ERR(data->ceb_gpio)) {
		dev_err(&pdev->dev, "Config ceb-gpio fail\n");
		ret = PTR_ERR(data->ceb_gpio);
		goto out;
	}

	ret = rt9467_register_regulator(data);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register regulator\n");
		goto out;
	}

	ret = rt9467_chg_get_iio_adc(data);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to get iio adc\n");
		goto out;
	}

	ret = rt9467_register_psy(data);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register power supply\n");
		goto out;
	}

	for (i = 0; i < ARRAY_SIZE(rt9467_irqs); i++) {
		irqno = platform_get_irq_byname(pdev, rt9467_irqs[i].irq_name);
		if (irqno < 0) {
			ret = irqno;
			goto out;
		}

		ret = devm_request_threaded_irq(&pdev->dev, irqno, NULL,
						rt9467_irqs[i].handler, 0,
						dev_name(&pdev->dev), data);
		if (ret) {
			dev_err(&pdev->dev, "Failed to request irq %d", irqno);
			goto out;
		}
	}

#if IS_ENABLED(CONFIG_MTK_CHARGER)
	ret = rt9467_register_mtk_charger_dev(data);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register chgdev\n");
		goto out;
	}
#endif /* CONFIG_MTK_CHARGER */

	ret = device_create_file(&pdev->dev, &dev_attr_shipping_mode);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create shipping attr\n");
		goto out_dev_attr;
	}

	schedule_work(&data->init_work);
	rt9467_chg_pwr_rdy_process(data);
	dev_info(&pdev->dev, "successfully\n");
	return 0;
out_dev_attr:
#if IS_ENABLED(CONFIG_MTK_CHARGER)
	charger_device_unregister(data->chg_dev);
	mutex_destroy(&data->pe_lock);
#endif /* CONFIG_MTK_CHARGER */
out:
	mutex_destroy(&data->attach_lock);
	mutex_destroy(&data->hm_lock);
	mutex_destroy(&data->ichg_lock);
	mutex_destroy(&data->ieoc_lock);
	mutex_destroy(&data->aicr_lock);
	mutex_destroy(&data->pp_lock);
	destroy_workqueue(data->wq);
	return ret;
}

static int rt9467_charger_remove(struct platform_device *pdev)
{
	struct rt9467_chg_data *data = platform_get_drvdata(pdev);

	dev_info(data->dev, "%s\n", __func__);

	device_remove_file(data->dev, &dev_attr_shipping_mode);
	mutex_destroy(&data->attach_lock);
	mutex_destroy(&data->hm_lock);
	mutex_destroy(&data->ichg_lock);
	mutex_destroy(&data->ieoc_lock);
	mutex_destroy(&data->aicr_lock);
	mutex_destroy(&data->pp_lock);
	destroy_workqueue(data->wq);
#if IS_ENABLED(CONFIG_MTK_CHARGER)
	charger_device_unregister(data->chg_dev);
	mutex_destroy(&data->pe_lock);
	wakeup_source_unregister(data->cdp_work_ws);
#endif /* CONFIG_MTK_CHARGER */
	return 0;
}

static void rt9467_charger_shutdown(struct platform_device *pdev)
{
	struct rt9467_chg_data *data = platform_get_drvdata(pdev);
	int ret;

	ret = rt9467_sw_reset(data);
	if (ret)
		dev_err(data->dev, "Failed to do sw reset\n");
}

static const struct of_device_id rt9467_charger_of_match_table[] = {
	{ .compatible = "richtek,rt9467-chg", },
	{ }
};
MODULE_DEVICE_TABLE(of, rt9467_charger_of_match_table);

static struct platform_driver rt9467_charger_driver = {
	.driver = {
		.name = "rt9467-charger",
		.of_match_table = rt9467_charger_of_match_table,
	},
	.probe = rt9467_charger_probe,
	.remove = rt9467_charger_remove,
	.shutdown = rt9467_charger_shutdown,
};
module_platform_driver(rt9467_charger_driver);

MODULE_DESCRIPTION("Richtek RT9467 charger driver");
MODULE_AUTHOR("ChiYuan Huang <cy_huang@richtek.com>");
MODULE_LICENSE("GPL");
