/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2022 Southchip Semiconductor Technology(Shanghai) Co., Ltd.
 */

#define pr_fmt(fmt)	"[sc89601a]:%s: " fmt, __func__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/bitops.h>
#include <linux/math64.h>


#include "sc89601a_reg.h"
//prize add by lipengpeng 20220621 start 
#include "mtk_charger.h"
//prize add by lipengpeng 20220621 end 
#include "charger_class.h"
enum {
    PN_SC89601A, 
};

static int pn_data[] = {
	[PN_SC89601A] = 0x05,
};

struct chg_para{
	int vlim;
	int ilim;

	int vreg;
	int ichg;
};

struct sc89601a_platform_data {
	int iprechg;
	int iterm;

	int boostv;
	int boosti;

	struct chg_para usb;
};

struct sc89601a {
	struct device *dev;
	struct i2c_client *client;

	int part_no;
	int revision;

	const char *chg_dev_name;
	const char *eint_name;

    int irq_gpio;
	int chg_en_gpio;

	int psy_usb_type;

	int status;
	int irq;

	struct mutex i2c_rw_lock;

	bool charge_enabled;	/* Register bit status */
	bool power_good;
	bool vbus_good;
	int input_curr_limit;

	struct sc89601a_platform_data *platform_data;
	struct charger_device *chg_dev;

	struct power_supply_desc psy_desc;
	struct power_supply *psy;
	struct power_supply *pdpe_psy;
	struct power_supply *chg_psy;
};

static const struct charger_properties sc89601a_chg_props = {
	.alias_name = "sc89601a",
};

static int __sc89601a_read_reg(struct sc89601a *sc, u8 reg, u8 *data)
{
	s32 ret;
	int retry_cnt = 5;
	
	do
	{

		ret = i2c_smbus_read_byte_data(sc->client, reg);
		if (ret < 0) {
			pr_err("i2c read fail: can't read from reg 0x%02X,ret = %d,retry_cnt = %d\n", reg,ret,retry_cnt);
			retry_cnt--;
			msleep(2);
		}
		else{
			*data = (u8) ret;
			return 0;
		}
	}while(retry_cnt > 0);

	return ret;
}

static int __sc89601a_write_reg(struct sc89601a *sc, int reg, u8 val)
{
	s32 ret;
	int retry_cnt = 5;
	
	do
	{
		if(reg == 0x00){
			//pr_err("%s write reg[0x00]:%x\n",__func__, val);
			if(val & 0x80){
				val &= 0x7f;
				pr_err("%s write reg[0x00]:%x\n",__func__, val);
			}
		}
		ret = i2c_smbus_write_byte_data(sc->client, reg, val);
		if (ret < 0) {
			pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d,retry_cnt = %d\n",val, reg, ret,retry_cnt);
			retry_cnt--;
			msleep(2);
		}
		else{
			return 0;
		}
	}while(retry_cnt > 0);
		
	return ret;
}

static int sc89601a_read_byte(struct sc89601a *sc, u8 reg, u8 *data)
{
	int ret;

	mutex_lock(&sc->i2c_rw_lock);
	ret = __sc89601a_read_reg(sc, reg, data);
	mutex_unlock(&sc->i2c_rw_lock);

	return ret;
}

static int sc89601a_write_byte(struct sc89601a *sc, u8 reg, u8 data)
{
	int ret;

	mutex_lock(&sc->i2c_rw_lock);
	ret = __sc89601a_write_reg(sc, reg, data);
	mutex_unlock(&sc->i2c_rw_lock);

	if (ret){
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
	}

	return ret;
}

static int sc89601a_update_bits(struct sc89601a *sc, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	mutex_lock(&sc->i2c_rw_lock);
	ret = __sc89601a_read_reg(sc, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __sc89601a_write_reg(sc, reg, tmp);
	if (ret){
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
	}

out:
	mutex_unlock(&sc->i2c_rw_lock);
	return ret;
}

static int sc89601a_set_key(struct sc89601a *sc)
{
	sc89601a_write_byte(sc, 0x7D, 0x48);
	sc89601a_write_byte(sc, 0x7D, 0x54);
	sc89601a_write_byte(sc, 0x7D, 0x53);
	return sc89601a_write_byte(sc, 0x7D, 0x38);
}

static int sc89601a_enable_otg(struct sc89601a *sc)
{

	u8 val = SC89601A_OTG_ENABLE << SC89601A_OTG_CONFIG_SHIFT;

	return sc89601a_update_bits(sc, SC89601A_REG_03,
				   SC89601A_OTG_CONFIG_MASK, val);
}

static int sc89601a_disable_otg(struct sc89601a *sc)
{
	u8 val = SC89601A_OTG_DISABLE << SC89601A_OTG_CONFIG_SHIFT;

	return sc89601a_update_bits(sc, SC89601A_REG_03,
				   SC89601A_OTG_CONFIG_MASK, val);
}

static int sc89601a_disable_hvdcp(struct sc89601a *sc)
{
	int ret;
	u8 val = SC89601A_HVDCP_DISABLE << SC89601A_HVDCPEN_SHIFT;

	ret = sc89601a_update_bits(sc, SC89601A_REG_02, 
				SC89601A_HVDCPEN_MASK, val);
	return ret;
}

static int sc89601a_enable_charger(struct sc89601a *sc)
{
	int ret;

	u8 val = SC89601A_CHG_ENABLE << SC89601A_CHG_CONFIG_SHIFT;

	ret = sc89601a_update_bits(sc, SC89601A_REG_03, 
				SC89601A_CHG_CONFIG_MASK, val);
	return ret;
}

static int sc89601a_disable_charger(struct sc89601a *sc)
{
	int ret;

	u8 val = SC89601A_CHG_DISABLE << SC89601A_CHG_CONFIG_SHIFT;

	ret = sc89601a_update_bits(sc, SC89601A_REG_03, 
				SC89601A_CHG_CONFIG_MASK, val);
	return ret;
}

static int sc89601a_adc_start(struct sc89601a *sc, bool oneshot)
{
	u8 val;
	int ret;
	
	ret = sc89601a_read_byte(sc, SC89601A_REG_02, &val);
	if (ret < 0) {
		dev_err(sc->dev, "%s failed to read register 0x02:%d\n", __func__, ret);
	}
	
	if (((val & SC89601A_CONV_RATE_MASK) >> SC89601A_CONV_RATE_SHIFT) == SC89601A_ADC_CONTINUE_ENABLE)
		return 0;

	if (oneshot) {
		ret = sc89601a_update_bits(sc, SC89601A_REG_02, SC89601A_CONV_START_MASK,
				SC89601A_CONV_START << SC89601A_CONV_START_SHIFT);
	}
	else {
		ret = sc89601a_update_bits(sc, SC89601A_REG_02, SC89601A_CONV_RATE_MASK,
				SC89601A_ADC_CONTINUE_ENABLE << SC89601A_CONV_RATE_SHIFT);
	}
	
	return ret;
}

static int sc89601a_adc_stop(struct sc89601a *sc)
{
	return sc89601a_update_bits(sc, SC89601A_REG_02, SC89601A_CONV_RATE_MASK,
				SC89601A_ADC_CONTINUE_DISABLE << SC89601A_CONV_RATE_SHIFT);
}

int sc89601a_set_chargecurrent(struct sc89601a *sc, int curr)
{
	u8 ichg;

	if (curr < SC89601A_ICHG_BASE)
		curr = SC89601A_ICHG_BASE;

    ichg = (curr - SC89601A_ICHG_BASE)/SC89601A_ICHG_LSB;

	return sc89601a_update_bits(sc, SC89601A_REG_04, 
						SC89601A_ICHG_MASK, ichg << SC89601A_ICHG_SHIFT);

}

int sc89601a_set_term_current(struct sc89601a *sc, int curr)
{
	u8 iterm;

	if (curr < SC89601A_ITERM_BASE)
		curr = SC89601A_ITERM_BASE;

    iterm = (curr - SC89601A_ITERM_BASE) / SC89601A_ITERM_LSB;

	return sc89601a_update_bits(sc, SC89601A_REG_05, 
						SC89601A_ITERM_MASK, iterm << SC89601A_ITERM_SHIFT);

}

int sc89601a_get_term_current(struct sc89601a *sc, int *curr)
{
    u8 reg_val;
	int iterm;
	int ret;

	ret = sc89601a_read_byte(sc, SC89601A_REG_05, &reg_val);
	if (!ret) {
		iterm = (reg_val & SC89601A_ITERM_MASK) >> SC89601A_ITERM_SHIFT;
        iterm = iterm * SC89601A_ITERM_LSB + SC89601A_ITERM_BASE;

		*curr = iterm * 1000;
	}
    return ret;
}

int sc89601a_set_prechg_current(struct sc89601a *sc, int curr)
{
	u8 iprechg;

	if (curr < SC89601A_IPRECHG_BASE)
		curr = SC89601A_IPRECHG_BASE;

    iprechg = (curr - SC89601A_IPRECHG_BASE) / SC89601A_IPRECHG_LSB;

	return sc89601a_update_bits(sc, SC89601A_REG_05, 
						SC89601A_IPRECHG_MASK, iprechg << SC89601A_IPRECHG_SHIFT);

}

int sc89601a_set_chargevolt(struct sc89601a *sc, int volt)
{
	u8 val;

	if (volt < SC89601A_VREG_BASE)
		volt = SC89601A_VREG_BASE;

	val = (volt - SC89601A_VREG_BASE)/SC89601A_VREG_LSB;
	return sc89601a_update_bits(sc, SC89601A_REG_06, 
						SC89601A_VREG_MASK, val << SC89601A_VREG_SHIFT);
}

int sc89601a_get_chargevol(struct sc89601a *sc, int *volt)
{
    u8 reg_val;
	int vchg;
	int ret;

	ret = sc89601a_read_byte(sc, SC89601A_REG_06, &reg_val);
	if (!ret) {
		vchg = (reg_val & SC89601A_VREG_MASK) >> SC89601A_VREG_SHIFT;
		vchg = vchg * SC89601A_VREG_LSB + SC89601A_VREG_BASE;
		*volt = vchg * 1000;
	}
    return ret;
}

static int sc89601a_get_boot_mode(struct sc89601a *sc)
{
	int ret = 0;
	union power_supply_propval val = {0,};
	
	 if (!sc->pdpe_psy) {
        sc->pdpe_psy = power_supply_get_by_name("pdpe-state");
        if (!sc->pdpe_psy) {
            pr_err("gezi get sc->pdpe_psy failed---%s\n",__func__);
			return -ENODEV;
        }
    }
	
	ret = power_supply_get_property(sc->pdpe_psy, POWER_SUPPLY_PROP_TEMP, &val);
	if (!ret){
		ret = val.intval;
	}
	else{
		pr_err("gezi get POWER_SUPPLY_PROP_TEMP failed---%s\n",__func__);
		ret = -1;
	}
	return ret;
}

static int sc89601a_get_state(struct sc89601a *sc)
{
	int ret = 0;
	union power_supply_propval val = {0,};
	
	 if (!sc->pdpe_psy) {
        sc->pdpe_psy = power_supply_get_by_name("pdpe-state");
        if (!sc->pdpe_psy) {
            pr_err("gezi get sc->pdpe_psy failed---%s\n",__func__);
			return -ENODEV;
        }
    }
	
	ret = power_supply_get_property(sc->pdpe_psy, POWER_SUPPLY_PROP_ONLINE, &val);
	if (!ret){
		ret = val.intval;
	}
	else{
		pr_err("gezi get POWER_SUPPLY_PROP_TEMP failed---%s\n",__func__);
		ret = -1;
	}
	return ret;
}

int sc89601a_adc_read_vbus_volt(struct sc89601a *sc, u32 *vol)
{
	uint8_t val;
	int volt;
	int ret;
	int boot_mode = 0,connect_state = 0;
	int read_vbus_retry = 3;
	boot_mode = sc89601a_get_boot_mode(sc);
	connect_state = sc89601a_get_state(sc);
	do
	{
		ret = sc89601a_read_byte(sc, SC89601A_REG_11, &val);
		pr_err("%s:boot_mode=%d,connect_state=%d,val = 0x%x,retry:%d\n", __func__, boot_mode,connect_state,val,read_vbus_retry);
		volt = SC89601A_VBUSV_BASE + ((val & SC89601A_VBUSV_MASK) >> SC89601A_VBUSV_SHIFT) * SC89601A_VBUSV_LSB;
		
		dev_err(sc->dev, "gezi read vbus voltage:%d,retry:%d\n", volt,read_vbus_retry);
	
		if(volt >= 10500){
			read_vbus_retry--;
			msleep(2);
		}
		else{
			break;
		}
	}while(read_vbus_retry > 0);
	if (ret < 0) {
		dev_err(sc->dev, "read vbus voltage failed :%d\n", ret);
	} else{

		if(boot_mode == 8 || boot_mode == 9){
			
		
		}
		else{
			if(volt < 2700){
				volt = 0;
			}
		}
//prize add by lipengpeng 20220708 start 
		//if(volt < 2700){
		//	volt=0;
		//}
//prize add by lipengpeng 20220708 end 
		*vol = volt * 1000;
	}

    return ret;
}

int sc89601a_adc_read_charge_current(struct sc89601a *sc, u32 *cur)
{
	uint8_t val;
	int curr;
	int ret;
	ret = sc89601a_read_byte(sc, SC89601A_REG_12, &val);
	if (ret < 0) {
		dev_err(sc->dev, "read charge current failed :%d\n", ret);
	} else{
		curr = (int)(SC89601A_ICHGR_BASE + ((val & SC89601A_ICHGR_MASK) >> SC89601A_ICHGR_SHIFT) * SC89601A_ICHGR_LSB) ;
		*cur = curr * 1000; 
	}

    return ret;
}

int sc89601a_set_force_vindpm(struct sc89601a *sc, bool en)
{
    u8 val;

    if (en)
        val = SC89601A_FORCE_VINDPM_ENABLE;
    else
        val = SC89601A_FORCE_VINDPM_DISABLE;
    

    return sc89601a_update_bits(sc, SC89601A_REG_0D, 
						SC89601A_FORCE_VINDPM_MASK, val << SC89601A_FORCE_VINDPM_SHIFT);
}

int sc89601a_set_input_volt_limit(struct sc89601a *sc, int volt)
{
	u8 val;

	if (volt < SC89601A_VINDPM_BASE)
		volt = SC89601A_VINDPM_BASE;
    
    sc89601a_set_force_vindpm(sc, true);

	val = (volt - SC89601A_VINDPM_BASE) / SC89601A_VINDPM_LSB;
	return sc89601a_update_bits(sc, SC89601A_REG_0D, 
						SC89601A_VINDPM_MASK, val << SC89601A_VINDPM_SHIFT);
}

int sc89601a_set_input_current_limit(struct sc89601a *sc, int curr)
{
	u8 val;

	if (curr > sc->input_curr_limit)
		curr = sc->input_curr_limit;

	if (curr < SC89601A_IINLIM_BASE)
		curr = SC89601A_IINLIM_BASE;

	val = (curr - SC89601A_IINLIM_BASE) / SC89601A_IINLIM_LSB;

	return sc89601a_update_bits(sc, SC89601A_REG_00, SC89601A_IINLIM_MASK, 
						val << SC89601A_IINLIM_SHIFT);
}

int sc89601a_get_input_volt_limit(struct sc89601a *sc, u32 *volt)
{
    u8 reg_val;
	int vchg;
	int ret;

	ret = sc89601a_read_byte(sc, SC89601A_REG_0D, &reg_val);
	if (!ret) {
		vchg = (reg_val & SC89601A_VINDPM_MASK) >> SC89601A_VINDPM_SHIFT;
		vchg = vchg * SC89601A_VINDPM_LSB + SC89601A_VINDPM_BASE;
		*volt = vchg * 1000;
	}
    return ret;
}

int sc89601a_get_input_current_limit(struct  sc89601a *sc, u32 *curr)
{
    u8 reg_val;
	int icl;
	int ret;

	ret = sc89601a_read_byte(sc, SC89601A_REG_00, &reg_val);
	if (!ret) {
		icl = (reg_val & SC89601A_IINLIM_MASK) >> SC89601A_IINLIM_SHIFT;
		icl = icl * SC89601A_IINLIM_LSB + SC89601A_IINLIM_BASE;
		*curr = icl * 1000;
	}

    return ret;
}

int sc89601a_set_watchdog_timer(struct sc89601a *sc, u8 timeout)
{
	u8 val;

	val = (timeout - SC89601A_WDT_BASE) / SC89601A_WDT_LSB;
	val <<= SC89601A_WDT_SHIFT;

	return sc89601a_update_bits(sc, SC89601A_REG_07, 
						SC89601A_WDT_MASK, val); 
}

int sc89601a_disable_watchdog_timer(struct sc89601a *sc)
{
	u8 val = SC89601A_WDT_DISABLE << SC89601A_WDT_SHIFT;

	return sc89601a_update_bits(sc, SC89601A_REG_07, 
						SC89601A_WDT_MASK, val);
}

int sc89601a_reset_watchdog_timer(struct sc89601a *sc)
{
	u8 val = SC89601A_WDT_RESET << SC89601A_WDT_RESET_SHIFT;

	return sc89601a_update_bits(sc, SC89601A_REG_03, 
						SC89601A_WDT_RESET_MASK, val);
}

int sc89601a_force_dpdm(struct sc89601a *sc)
{
	int ret;
	u8 val = SC89601A_FORCE_DPDM << SC89601A_FORCE_DPDM_SHIFT;

	ret = sc89601a_update_bits(sc, SC89601A_REG_02, 
						SC89601A_FORCE_DPDM_MASK, val);

	sc->power_good = false;
	pr_info("Force DPDM %s\n", !ret ? "successfully" : "failed");
	
	return ret;

}

int sc89601a_reset_chip(struct sc89601a *sc)
{
	int ret;
	u8 val = SC89601A_RESET << SC89601A_RESET_SHIFT;

	ret = sc89601a_update_bits(sc, SC89601A_REG_14, 
						SC89601A_RESET_MASK, val);
	return ret;
}

int sc89601a_enable_hiz_mode(struct sc89601a *sc, bool en)
{
	u8 val;

    if (en) {
        val = SC89601A_HIZ_ENABLE << SC89601A_ENHIZ_SHIFT;
    } else {
        val = SC89601A_HIZ_DISABLE << SC89601A_ENHIZ_SHIFT;
    }

	return sc89601a_update_bits(sc, SC89601A_REG_00, 
						SC89601A_ENHIZ_MASK, val);

}

int sc89601a_exit_hiz_mode(struct sc89601a *sc)
{

	u8 val = SC89601A_HIZ_DISABLE << SC89601A_ENHIZ_SHIFT;

	return sc89601a_update_bits(sc, SC89601A_REG_00, 
						SC89601A_ENHIZ_MASK, val);

}

int sc89601a_get_hiz_mode(struct sc89601a *sc, u8 *state)
{
	u8 val;
	int ret;

	ret = sc89601a_read_byte(sc, SC89601A_REG_00, &val);
	if (ret)
		return ret;
	*state = (val & SC89601A_ENHIZ_MASK) >> SC89601A_ENHIZ_SHIFT;

	return 0;
}

static int sc89601a_enable_term(struct sc89601a *sc, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = SC89601A_TERM_ENABLE << SC89601A_EN_TERM_SHIFT;
	else
		val = SC89601A_TERM_DISABLE << SC89601A_EN_TERM_SHIFT;

	ret = sc89601a_update_bits(sc, SC89601A_REG_07, 
						SC89601A_EN_TERM_MASK, val);

	return ret;
}

int sc89601a_set_boost_current(struct sc89601a *sc, int curr)
{
	u8 temp;

	if (curr == 500)
		temp = SC89601A_BOOST_LIM_500MA;
	else if (curr == 750)
		temp = SC89601A_BOOST_LIM_750MA;
	else if (curr == 1200)
		temp = SC89601A_BOOST_LIM_1200MA;
	else
		temp = SC89601A_BOOST_LIM_1400MA;

	return sc89601a_update_bits(sc, SC89601A_REG_0A, 
				SC89601A_BOOST_LIM_MASK, 
				temp << SC89601A_BOOST_LIM_SHIFT);

}

int sc89601a_set_boost_voltage(struct sc89601a *sc, int volt)
{
	u8 val = 0;

    if (volt < SC89601A_BOOSTV_BASE)
        volt = SC89601A_BOOSTV_BASE;
    if (volt > SC89601A_BOOSTV_BASE 
            + (SC89601A_BOOSTV_MASK >> SC89601A_BOOSTV_SHIFT) 
            * SC89601A_BOOSTV_LSB)
        volt = SC89601A_BOOSTV_BASE 
            + (SC89601A_BOOSTV_MASK >> SC89601A_BOOSTV_SHIFT) 
            * SC89601A_BOOSTV_LSB;

    val = ((volt - SC89601A_BOOSTV_BASE) / SC89601A_BOOSTV_LSB) 
            << SC89601A_BOOSTV_SHIFT;

	return sc89601a_update_bits(sc, SC89601A_REG_0A, 
				SC89601A_BOOSTV_MASK, val);


}

static int sc89601a_enable_ico(struct sc89601a* sc, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = SC89601A_ICO_ENABLE << SC89601A_ICOEN_SHIFT;
	else
		val = SC89601A_ICO_DISABLE << SC89601A_ICOEN_SHIFT;

	ret = sc89601a_update_bits(sc, SC89601A_REG_02, SC89601A_ICOEN_MASK, val);

	return ret;

}


static int sc89601a_enable_safety_timer(struct sc89601a *sc)
{
	const u8 val = SC89601A_CHG_TIMER_ENABLE << SC89601A_EN_TIMER_SHIFT;

	return sc89601a_update_bits(sc, SC89601A_REG_07, SC89601A_EN_TIMER_MASK,
				   val);
}

static int sc89601a_disable_safety_timer(struct sc89601a *sc)
{
	const u8 val = SC89601A_CHG_TIMER_DISABLE << SC89601A_EN_TIMER_SHIFT;

	return sc89601a_update_bits(sc, SC89601A_REG_07, SC89601A_EN_TIMER_MASK,
				   val);

}

static struct sc89601a_platform_data *sc89601a_parse_dt(struct device_node *np,
						      struct sc89601a *sc)
{
	int ret;
	struct sc89601a_platform_data *pdata;
	pdata = devm_kzalloc(sc->dev, sizeof(struct sc89601a_platform_data),
			     GFP_KERNEL);
	if (!pdata)
		return NULL;

	if (of_property_read_string(np, "charger_name", &sc->chg_dev_name) < 0) {
		sc->chg_dev_name = "primary_chg";
		pr_warn("no charger name\n");
	}

	if (of_property_read_string(np, "eint_name", &sc->eint_name) < 0) {
		sc->eint_name = "chr_stat";
		pr_warn("no eint name\n");
	}

	ret = of_property_read_u32(np, "sc,sc89601a,usb-vlim", &pdata->usb.vlim);
	if (ret) {
		pdata->usb.vlim = 4500;
		pr_err("Failed to read node of sc,sc89601a,usb-vlim\n");
	}

    sc->irq_gpio = of_get_named_gpio(np, "sc,intr-gpio", 0);
	if (sc->irq_gpio < 0)
		pr_err("sc,intr-gpio is not available\n");

	sc->chg_en_gpio = of_get_named_gpio(np, "sc,chg-en-gpio", 0);
	if (sc->chg_en_gpio < 0)
		pr_err("sc,chg-en-gpio is not available\n");
	gpio_direction_output(sc->chg_en_gpio, 0);

	ret = of_property_read_u32(np, "sc,sc89601a,usb-ilim", &pdata->usb.ilim);
	if (ret) {
		pdata->usb.ilim = 2000;
		pr_err("Failed to read node of sc,sc89601a,usb-ilim\n");
	}

	ret = of_property_read_u32(np, "sc,sc89601a,usb-vreg", &pdata->usb.vreg);
	if (ret) {
		pdata->usb.vreg = 4200;
		pr_err("Failed to read node of sc,sc89601a,usb-vreg\n");
	}

	ret = of_property_read_u32(np, "sc,sc89601a,usb-ichg", &pdata->usb.ichg);
	if (ret) {
		pdata->usb.ichg = 2000;
		pr_err("Failed to read node of sc,sc89601a,usb-ichg\n");
	}

	ret = of_property_read_u32(np, "sc,sc89601a,precharge-current",
				   &pdata->iprechg);
	if (ret) {
		pdata->iprechg = 180;
		pr_err("Failed to read node of sc,sc89601a,precharge-current\n");
	}

	ret = of_property_read_u32(np, "sc,sc89601a,termination-current",
				   &pdata->iterm);
	if (ret) {
		pdata->iterm = 180;
		pr_err
		    ("Failed to read node of sc,sc89601a,termination-current\n");
	}

	ret =
	    of_property_read_u32(np, "sc,sc89601a,boost-voltage",
				 &pdata->boostv);
	if (ret) {
		pdata->boostv = 5000;
		pr_err("Failed to read node of sc,sc89601a,boost-voltage\n");
	}

	ret =
	    of_property_read_u32(np, "sc,sc89601a,boost-current",
				 &pdata->boosti);
	if (ret) {
		pdata->boosti = 1200;
		pr_err("Failed to read node of sc,sc89601a,boost-current\n");
	}


	return pdata;
}

static bool sc89601a_get_cmd_stat(struct sc89601a *sc)
{
	struct mtk_charger *info = NULL;

	if (!sc->chg_psy) {
		sc->chg_psy = power_supply_get_by_name("mtk-master-charger");
		if(sc->chg_psy == NULL){
			pr_err("sc89601 get chg_psy err\n");
			return false;
		}
	}
	
	info = (struct mtk_charger *)power_supply_get_drvdata(sc->chg_psy);
	
	if(!info){
		return false;
	}

	return info->cmd_discharging;
}


static bool sc89601a_is_charging_disabled(struct sc89601a *sc)
{
	int ret = 0;
	u8 val = 0;
	bool en = false;
	
	ret = sc89601a_read_byte(sc, SC89601A_REG_03, &val);

	if (!ret){
		 en = !!(val & SC89601A_CHG_CONFIG_MASK);
	}
	
	pr_err("sc89601a_is_charging_disabled:%d\n", !en);
	
	return !en;
}

static int sc89601a_get_charge_stat(struct sc89601a *sc, int *state)
{
    int ret;
	u8 val;
	

	ret = sc89601a_read_byte(sc, SC89601A_REG_0B, &val);
	if (!ret) {
        if ((val & SC89601A_VBUS_STAT_MASK) >> SC89601A_VBUS_STAT_SHIFT 
                == SC89601A_VBUS_TYPE_OTG) {
            *state = POWER_SUPPLY_STATUS_DISCHARGING;
            return ret;
        }
		val = val & SC89601A_CHRG_STAT_MASK;
		val = val >> SC89601A_CHRG_STAT_SHIFT;
		switch (val)
        {
        case SC89601A_CHRG_STAT_IDLE:
			ret = sc89601a_read_byte(sc, SC89601A_REG_11, &val);
			if (val & SC89601A_VBUS_GD_MASK) {
				if(sc89601a_get_cmd_stat(sc) || sc89601a_is_charging_disabled(sc)){
					*state = POWER_SUPPLY_STATUS_NOT_CHARGING;
				}
				else{
					*state = POWER_SUPPLY_STATUS_CHARGING;
				}
			} else {
				*state = POWER_SUPPLY_STATUS_NOT_CHARGING;
			}
            break;
        case SC89601A_CHRG_STAT_PRECHG:
        case SC89601A_CHRG_STAT_FASTCHG:
            *state = POWER_SUPPLY_STATUS_CHARGING;
            break;
        case SC89601A_CHRG_STAT_CHGDONE:
            *state = POWER_SUPPLY_STATUS_FULL;
            break;
        default:
            *state = POWER_SUPPLY_STATUS_UNKNOWN;
            break;
        }
	}

	return ret;
}

static int sc89601a_get_charger_type(struct sc89601a *sc, int *type)
{
	int ret;

	u8 reg_val = 0;
	int vbus_stat = 0;
	int chg_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;

	ret = sc89601a_read_byte(sc, SC89601A_REG_0B, &reg_val);

	if (ret)
		return ret;

	vbus_stat = (reg_val & SC89601A_VBUS_STAT_MASK);
	vbus_stat >>= SC89601A_VBUS_STAT_SHIFT;
//prize add by lipengpeng 20220621 start 	
	if (vbus_stat != SC89601A_VBUS_TYPE_NONE && vbus_stat != SC89601A_VBUS_TYPE_OTG) {
		if (get_MT5725_status() == 0) {
			chg_type = POWER_SUPPLY_USB_TYPE_DCP;
			sc->psy_desc.type =POWER_SUPPLY_TYPE_USB;
			
			printk("wireless ---->0x%02x  %d  %d\n", reg_val, chg_type, sc->psy_desc.type);

			*type = chg_type;
			return 0;
		}
	}
//prize add by lipengpeng 20220621 end 
	switch (vbus_stat) {

	case SC89601A_VBUS_TYPE_NONE:
		chg_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		sc->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	case SC89601A_VBUS_TYPE_SDP:
		chg_type = POWER_SUPPLY_USB_TYPE_SDP;
		sc->psy_desc.type = POWER_SUPPLY_TYPE_USB;
		break;
	case SC89601A_VBUS_TYPE_CDP:
		chg_type = POWER_SUPPLY_USB_TYPE_CDP;
		sc->psy_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case SC89601A_VBUS_TYPE_DCP:
    case SC89601A_VBUS_TYPE_HVDCP:
		chg_type = POWER_SUPPLY_USB_TYPE_DCP;
		sc->psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case SC89601A_VBUS_TYPE_UNKNOWN:
		chg_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		sc->psy_desc.type = POWER_SUPPLY_TYPE_USB;
		break;
	case SC89601A_VBUS_TYPE_NON_STD:
		chg_type = POWER_SUPPLY_USB_TYPE_DCP;
		sc->psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	default:
		chg_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		sc->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	}
	
	pr_err("%s ---->0x%02x  %d  %d\n", __func__, reg_val, chg_type, sc->psy_desc.type);

	*type = chg_type;

	return 0;
}

static void sc89601a_dump_regs(struct sc89601a *sc);
static irqreturn_t sc89601a_irq_handler(int irq, void *data)
{
	int ret;
	u8 reg_val;
	bool prev_pg;
	bool prev_vbus_pg;
	struct sc89601a *sc = data;

	ret = sc89601a_read_byte(sc, SC89601A_REG_0B, &reg_val);
	if (ret)
		return IRQ_HANDLED;

	prev_pg = sc->power_good;

	sc->power_good = !!(reg_val & SC89601A_PG_STAT_MASK);

	ret = sc89601a_read_byte(sc, SC89601A_REG_11, &reg_val);
        if (ret)
                return IRQ_HANDLED;

        prev_vbus_pg = sc->vbus_good;

        sc->vbus_good = !!(reg_val & SC89601A_VBUS_GD_MASK);

	if (!prev_vbus_pg && sc->vbus_good){
		sc->input_curr_limit = 3000;
		pr_err("adapter/usb inserted\n");
		sc89601a_adc_start(sc, false);
		//sc89601a_enable_charger(sc);
		//ret = sc89601a_get_charger_type(sc, &sc->psy_usb_type);
	} else if (prev_vbus_pg && !sc->vbus_good) {
		pr_err("adapter/usb removed\n");
		sc89601a_adc_stop(sc);
	//ret = sc89601a_get_charger_type(sc, &sc->psy_usb_type);
	}

	//if (!prev_pg && sc->power_good) {
	ret = sc89601a_get_charger_type(sc, &sc->psy_usb_type);
	//}
	
	pr_err("%s", __func__);
	sc89601a_dump_regs(sc);

    power_supply_changed(sc->psy);
/*	
	if (!sc->chg_psy) {
		sc->chg_psy = power_supply_get_by_name("mtk-master-charger");
	}
	
	if (sc->chg_psy) {
		pr_err("--->power_supply changed mtk-master-charger\n");
		power_supply_changed(sc->chg_psy);
	}
*/
	return IRQ_HANDLED;
}

static int sc89601a_register_interrupt(struct sc89601a *sc)
{
	int ret = 0;

    ret = devm_gpio_request(sc->dev, sc->irq_gpio, "chr-irq");
    if (ret < 0) {
        pr_err("failed to request GPIO%d ; ret = %d", sc->irq_gpio, ret);
        return ret;
    }

    ret = gpio_direction_input(sc->irq_gpio);
    if (ret < 0) {
        pr_err("failed to set GPIO%d ; ret = %d", sc->irq_gpio, ret);
        return ret;
    }

    sc->irq = gpio_to_irq(sc->irq_gpio);
    if (ret < 0) {
        pr_err("failed gpio to irq GPIO%d ; ret = %d", sc->irq_gpio, ret);
        return ret;
    }


	ret = devm_request_threaded_irq(sc->dev, sc->irq, NULL,
					sc89601a_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"chr_stat", sc);
	if (ret < 0) {
		pr_err("request thread irq failed:%d\n", ret);
		return ret;
	}else{
		pr_err("request thread irq pass:%d  sc->irq =%d\n", ret, sc->irq);
	}

	enable_irq_wake(sc->irq);

	return 0;
}

static int sc89601a_init_device(struct sc89601a *sc)
{
	int ret;
	
	sc89601a_reset_chip(sc);

	sc89601a_set_key(sc);
	sc89601a_update_bits(sc, 0x88, 0x60, 0x60);

	sc89601a_set_key(sc);

	sc->input_curr_limit = 3000;

	sc89601a_disable_watchdog_timer(sc);
    sc89601a_disable_hvdcp(sc);
    sc89601a_enable_ico(sc, false);

	ret = sc89601a_set_prechg_current(sc, sc->platform_data->iprechg);
	if (ret)
		pr_err("Failed to set prechg current, ret = %d\n", ret);

	ret = sc89601a_set_term_current(sc, sc->platform_data->iterm);
	if (ret)
		pr_err("Failed to set termination current, ret = %d\n", ret);

	ret = sc89601a_set_boost_voltage(sc, sc->platform_data->boostv);
	if (ret)
		pr_err("Failed to set boost voltage, ret = %d\n", ret);

	ret = sc89601a_set_boost_current(sc, sc->platform_data->boosti);
	if (ret)
		pr_err("Failed to set boost current, ret = %d\n", ret);

	return 0;
}

static void determine_initial_status(struct sc89601a *sc)
{
	sc89601a_irq_handler(sc->irq, (void *) sc);
}

static int sc89601a_detect_device(struct sc89601a *sc)
{
	int ret;
	u8 data;

	ret = sc89601a_read_byte(sc, SC89601A_REG_14, &data);
	if (!ret) {
		sc->part_no = (data & SC89601A_PN_MASK) >> SC89601A_PN_SHIFT;
		sc->revision =
		    (data & SC89601A_DEV_REV_MASK) >> SC89601A_DEV_REV_SHIFT;
	}

	return ret;
}



static void sc89601a_update_chg_state(struct sc89601a *sc)
{
	static bool recode_state = 0;
	bool state = false;
	u8 val = 0;
	int ret = 0;
	
	ret = sc89601a_read_byte(sc, SC89601A_REG_03, &val);

	if (!ret){
		state = !!(val & SC89601A_CHG_CONFIG_MASK);
		if(state != recode_state){
			pr_err("gezi %s-------\n",__func__);
			power_supply_changed(sc->psy);
		}
		recode_state = state;
	}
	else{
		pr_err("gezi %s---get REG03 err----\n",__func__);
	}
}

static void sc89601a_dump_regs(struct sc89601a *sc)
{
	int addr;
	u8 val;
	int ret;

	for (addr = 0x0; addr <= 0x14; addr++) {
		ret = sc89601a_read_byte(sc, addr, &val);
		if (ret == 0)
			pr_err("Reg[%.2x] = 0x%.2x\n", addr, val);
	}
	
	sc89601a_update_chg_state(sc);
}

static ssize_t
sc89601a_show_registers(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	struct sc89601a *sc = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[200];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "sc89601a Reg");
	for (addr = 0x0; addr <= 0x14; addr++) {
		ret = sc89601a_read_byte(sc, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,
				       "Reg[%.2x] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t
sc89601a_store_registers(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct sc89601a *sc = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg < 0x14) {
		sc89601a_write_byte(sc, (unsigned char) reg,
				   (unsigned char) val);
	}

	return count;
}

static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR, sc89601a_show_registers,
		   sc89601a_store_registers);

static struct attribute *sc89601a_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group sc89601a_attr_group = {
	.attrs = sc89601a_attributes,
};

static int sc89601a_charging(struct charger_device *chg_dev, bool enable)
{

	struct sc89601a *sc = dev_get_drvdata(&chg_dev->dev);
	int ret = 0;
	u8 val;

	if (enable)
		ret = sc89601a_enable_charger(sc);
	else
		ret = sc89601a_disable_charger(sc);

	pr_err("%s charger %s\n", enable ? "enable" : "disable", !ret ? "successfully" : "failed");

	ret = sc89601a_read_byte(sc, SC89601A_REG_03, &val);

	if (!ret)
		sc->charge_enabled = !!(val & SC89601A_CHG_CONFIG_MASK);

	return ret;
}

static int sc89601a_plug_in(struct charger_device *chg_dev)
{

	int ret;

	ret = sc89601a_charging(chg_dev, true);

	if (ret)
		pr_err("Failed to enable charging:%d\n", ret);

	return ret;
}

static int sc89601a_plug_out(struct charger_device *chg_dev)
{
	int ret;

	ret = sc89601a_charging(chg_dev, false);

	if (ret)
		pr_err("Failed to disable charging:%d\n", ret);

	return ret;
}

static int sc89601a_dump_register(struct charger_device *chg_dev)
{
	struct sc89601a *sc = dev_get_drvdata(&chg_dev->dev);

	sc89601a_dump_regs(sc);

	return 0;
}

static int sc89601a_is_charging_enable(struct charger_device *chg_dev, bool *en)
{
	int ret;
	u8 val;
	struct sc89601a *sc = dev_get_drvdata(&chg_dev->dev);
	ret = sc89601a_read_byte(sc, SC89601A_REG_03, &val);

	if (!ret)
		sc->charge_enabled = !!(val & SC89601A_CHG_CONFIG_MASK);
	*en = sc->charge_enabled;
	pr_err("sc89601a_is_charging_enable:%d\n", sc->charge_enabled);
	return sc->charge_enabled;
}

static int sc89601a_is_charging_done(struct charger_device *chg_dev, bool *done)
{
	struct sc89601a *sc = dev_get_drvdata(&chg_dev->dev);
	int ret = 0;
	u8 val;
	int retry = 3;

	do{

		ret = sc89601a_read_byte(sc, SC89601A_REG_0B, &val);
		if (!ret) {
			pr_err("gezi ---%s ----reg0B=0x%x\n",__func__,val);
			val = val & SC89601A_CHRG_STAT_MASK;
			val = val >> SC89601A_CHRG_STAT_SHIFT;
			*done = (val == SC89601A_CHRG_STAT_CHGDONE);
			
			pr_err("gezi ---%s ----%d %d\n",__func__,*done,retry);
			
			if(*done){
				retry--;
				msleep(2);
			}
			else{
				break;
			}
		}
		else{
			pr_err("gezi-i2c read err--%s ----ret = %d\n",__func__,ret);
			ret = 0;
		}
	}while(retry > 0);

	return ret;
}

static int sc89601a_set_ichg(struct charger_device *chg_dev, u32 curr)
{
	struct sc89601a *sc = dev_get_drvdata(&chg_dev->dev);

	pr_err("charge curr = %d\n", curr);

	return sc89601a_set_chargecurrent(sc, curr / 1000);
}

static int sc89601a_get_ichg(struct charger_device *chg_dev, u32 *curr)
{
	struct sc89601a *sc = dev_get_drvdata(&chg_dev->dev);
	u8 reg_val;
	int ichg;
	int ret;

	ret = sc89601a_read_byte(sc, SC89601A_REG_04, &reg_val);
	if (!ret) {
        ichg = (reg_val & SC89601A_ICHG_MASK) >> SC89601A_ICHG_SHIFT;
        ichg = ichg * SC89601A_ICHG_LSB + SC89601A_ICHG_BASE;
        *curr = ichg * 1000;
	}

	return ret;
}

static int sc89601a_get_min_ichg(struct charger_device *chg_dev, u32 *curr)
{
	*curr = 60 * 1000;

	return 0;
}

static int sc89601a_set_vchg(struct charger_device *chg_dev, u32 volt)
{
	struct sc89601a *sc = dev_get_drvdata(&chg_dev->dev);

	pr_err("charge volt = %d\n", volt);

	return sc89601a_set_chargevolt(sc, volt / 1000);
}

static int sc89601a_get_vchg(struct charger_device *chg_dev, u32 *volt)
{
	struct sc89601a *sc = dev_get_drvdata(&chg_dev->dev);

	return sc89601a_get_chargevol(sc, volt);
}

static int sc89601a_set_ivl(struct charger_device *chg_dev, u32 volt)
{
	struct sc89601a *sc = dev_get_drvdata(&chg_dev->dev);

	pr_err("vindpm volt = %d\n", volt);
	
	if(volt > 8000000){
		return 0;
	}

	return sc89601a_set_input_volt_limit(sc, volt / 1000);
}

static int sc89601a_get_ivl(struct charger_device *chgdev, u32 *volt)
{
    struct sc89601a *sc = dev_get_drvdata(&chgdev->dev);

    return sc89601a_get_input_volt_limit(sc, volt);
}

static int sc89601a_get_vbus_adc(struct charger_device *chgdev, u32 *vbus)
{
	struct sc89601a *sc = dev_get_drvdata(&chgdev->dev);
	
	return sc89601a_adc_read_vbus_volt(sc, vbus);
}

static int sc89601a_get_ibus_adc(struct charger_device *chgdev, u32 *ibus)
{
	struct sc89601a *sc = dev_get_drvdata(&chgdev->dev);
	
	return sc89601a_adc_read_charge_current(sc, ibus);
}

static int sc89601a_set_icl(struct charger_device *chg_dev, u32 curr)
{
	struct sc89601a *sc = dev_get_drvdata(&chg_dev->dev);

	pr_err("indpm curr = %d\n", curr);

	return sc89601a_set_input_current_limit(sc, curr / 1000);
}

static int sc89601a_get_icl(struct charger_device *chg_dev, u32 *curr)
{
	struct sc89601a *sc = dev_get_drvdata(&chg_dev->dev);
	
    return sc89601a_get_input_current_limit(sc, curr);
}

static int sc89601a_kick_wdt(struct charger_device *chg_dev)
{
	struct sc89601a *sc = dev_get_drvdata(&chg_dev->dev);

	return sc89601a_reset_watchdog_timer(sc);
}

static int sc89601a_set_ieoc(struct charger_device *chgdev, u32 ieoc)
{
    struct sc89601a *sc = dev_get_drvdata(&chgdev->dev);
    
    return sc89601a_set_term_current(sc, ieoc / 1000);
}

static int sc89601a_enable_te(struct charger_device *chgdev, bool en)
{
    struct sc89601a *sc = dev_get_drvdata(&chgdev->dev);

    return sc89601a_enable_term(sc, en);
}

static int sc89601a_enable_hz(struct charger_device *chgdev, bool en)
{
    struct sc89601a *sc = dev_get_drvdata(&chgdev->dev);

    return sc89601a_enable_hiz_mode(sc, en);
}

static int sc89601a_event(struct charger_device *chgdev, u32 event, u32 args)
{
	struct sc89601a *sc = dev_get_drvdata(&chgdev->dev);

	pr_info("%s event = %d\n", __func__, event);

	power_supply_changed(sc->psy);
	
	return 0;
}

static int sc89601a_set_otg(struct charger_device *chg_dev, bool en)
{
	int ret;
	struct sc89601a *sc = dev_get_drvdata(&chg_dev->dev);

	if (en) {
        ret = sc89601a_disable_charger(sc);
        ret = sc89601a_enable_otg(sc);
    }
	else {
        ret = sc89601a_disable_otg(sc);
        ret = sc89601a_enable_charger(sc);
    }

	pr_err("%s OTG %s\n", en ? "enable" : "disable",
	       !ret ? "successfully" : "failed");

	return ret;
}

static int sc89601a_set_safety_timer(struct charger_device *chg_dev, bool en)
{
	struct sc89601a *sc = dev_get_drvdata(&chg_dev->dev);
	int ret;

	if (en)
		ret = sc89601a_enable_safety_timer(sc);
	else
		ret = sc89601a_disable_safety_timer(sc);

	return ret;
}

static int sc89601a_is_safety_timer_enabled(struct charger_device *chg_dev,
					   bool *en)
{
	struct sc89601a *sc = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 reg_val;

	ret = sc89601a_read_byte(sc, SC89601A_REG_07, &reg_val);

	if (!ret)
		*en = !!(reg_val & SC89601A_EN_TIMER_MASK);

	return ret;
}

static int sc89601a_set_boost_ilmt(struct charger_device *chg_dev, u32 curr)
{
	struct sc89601a *sc = dev_get_drvdata(&chg_dev->dev);
	int ret;

	pr_err("otg curr = %d\n", curr);

	ret = sc89601a_set_boost_current(sc, curr / 1000);

	return ret;
}

static int sc8960x_pumpx_up(struct sc89601a *sc)
{
	int curr = 0,temp_curr = 0;
	
	sc89601a_get_input_current_limit(sc, &temp_curr);
	
	pr_err("gezi----%s-----%d----start---temp_curr = %d\n",__func__,__LINE__,temp_curr);
	
	//if(curr < 1000){
		curr = 1500;
	//}
	
	sc89601a_set_input_current_limit(sc, 100);
	msleep(100);
	sc89601a_set_input_current_limit(sc, curr);
	msleep(100);
	sc89601a_set_input_current_limit(sc, 100);
	msleep(100);
	sc89601a_set_input_current_limit(sc, curr);
	msleep(100);
	sc89601a_set_input_current_limit(sc, 100);
	msleep(100);
	sc89601a_set_input_current_limit(sc, curr);
	msleep(300);
	sc89601a_set_input_current_limit(sc, 100);
	msleep(100);
	sc89601a_set_input_current_limit(sc, curr);
	msleep(300);
	sc89601a_set_input_current_limit(sc, 100);
	msleep(100);
	sc89601a_set_input_current_limit(sc, curr);
	msleep(300);
	sc89601a_set_input_current_limit(sc, 100);
	msleep(100);
    sc89601a_set_input_current_limit(sc, curr);
	msleep(500);
	sc89601a_set_input_current_limit(sc, 100);
	msleep(100);
	pr_err("gezi----%s-----%d---end----temp_curr = %d\n",__func__,__LINE__,temp_curr);
	return sc89601a_set_input_current_limit(sc, temp_curr);
}

static int sc8960x_pumpx_dn(struct sc89601a *sc)
{
	int curr;
	
	sc89601a_get_input_current_limit(sc, &curr);
	
	sc89601a_set_input_current_limit(sc, 100);
	msleep(100);
	sc89601a_set_input_current_limit(sc, curr);
	msleep(300);
	sc89601a_set_input_current_limit(sc, 100);
	msleep(100);
	sc89601a_set_input_current_limit(sc, curr);
	msleep(300);
	sc89601a_set_input_current_limit(sc, 100);
	msleep(100);
	sc89601a_set_input_current_limit(sc, curr);
	msleep(300);
	sc89601a_set_input_current_limit(sc, 100);
	msleep(100);
	sc89601a_set_input_current_limit(sc, curr);
	msleep(100);
	sc89601a_set_input_current_limit(sc, 100);
	msleep(100);
	sc89601a_set_input_current_limit(sc, curr);
	msleep(100);
	sc89601a_set_input_current_limit(sc, 100);
	msleep(100);
    sc89601a_set_input_current_limit(sc, curr);
	msleep(500);
	sc89601a_set_input_current_limit(sc, 100);
	msleep(100);
	return sc89601a_set_input_current_limit(sc, curr);
}

static int sc8960x_set_pe_current_pattern(struct charger_device *chgdev,
					 bool inc)
{
    struct sc89601a *sc = dev_get_drvdata(&chgdev->dev);
	int ret;

    if (inc) {
        ret = sc8960x_pumpx_up(sc);
    } else {
        ret = sc8960x_pumpx_dn(sc);
    }
	msleep(500);
    return ret;
}

static int sc8960x_reset_pe_ta(struct charger_device *chgdev)
{
    int ret = 0;
	int curr = 0;
	struct sc89601a *chip = dev_get_drvdata(&chgdev->dev);
	sc89601a_get_input_current_limit(chip, &curr);
	pr_info("sc89601a_set_pep_reset curr %d \n",curr);
	ret = sc89601a_set_input_current_limit(chip, 100);
	if (ret < 0)
		goto out;
	msleep(250);
	ret = sc89601a_set_input_current_limit(chip, curr);
out:
	return ret;
}

static enum power_supply_usb_type sc89601a_chg_psy_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_DCP,
};


static enum power_supply_property sc89601a_chg_psy_properties[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
};

static int sc89601a_chg_property_is_writeable(struct power_supply *psy,
					    enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_ENERGY_EMPTY:
		return 1;
	default:
		return 0;
	}
	return 0;
}

static int sc89601a_chg_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	int ret = 0;
	u32 _val;
	int data;
	struct sc89601a *sc = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "SouthChip";
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		//val->intval = sc->power_good;
		val->intval = sc->vbus_good;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = sc89601a_get_charge_stat(sc, &data);
		if (ret < 0)
			break;
		val->intval = data;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
        val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = sc89601a_get_chargevol(sc, &data);
        if (ret < 0)
            break;
        val->intval = data;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = sc89601a_get_input_current_limit(sc, &_val);
        if (ret < 0)
            break;
        val->intval = _val;
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = sc89601a_get_input_volt_limit(sc, &_val);
        if (ret < 0)
            break;
        val->intval = _val;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = sc89601a_get_term_current(sc, &data);
        if (ret < 0)
            break;
        val->intval = data;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		pr_err("---->POWER_SUPPLY_PROP_USB_TYPE : %d\n", sc->psy_usb_type);
		val->intval = sc->psy_usb_type;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (sc->psy_desc.type == POWER_SUPPLY_TYPE_USB)
			val->intval = 500000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		if (sc->psy_desc.type == POWER_SUPPLY_TYPE_USB)
			val->intval = 5000000;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		pr_err("---->POWER_SUPPLY_PROP_TYPE : %d\n", sc->psy_desc.type);
		val->intval = sc->psy_desc.type;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int sc89601a_chg_set_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	int ret = 0;
	struct sc89601a *sc = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		sc89601a_force_dpdm(sc);
		break;
	case POWER_SUPPLY_PROP_STATUS:
        if (val->intval) {
            ret = sc89601a_enable_charger(sc);
        } else {
            ret = sc89601a_disable_charger(sc);
        }
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = sc89601a_set_chargecurrent(sc, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = sc89601a_set_chargevolt(sc, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		sc->input_curr_limit = val->intval / 1000;
		ret = sc89601a_set_input_current_limit(sc, sc->input_curr_limit);
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = sc89601a_set_input_volt_limit(sc, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
        ret = sc89601a_set_term_current(sc, val->intval / 1000);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static char *sc89601a_psy_supplied_to[] = {
	"battery",
	"mtk-master-charger",
};

static const struct power_supply_desc sc89601a_psy_desc = {
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = sc89601a_chg_psy_usb_types,
	.num_usb_types = ARRAY_SIZE(sc89601a_chg_psy_usb_types),
	.properties = sc89601a_chg_psy_properties,
	.num_properties = ARRAY_SIZE(sc89601a_chg_psy_properties),
	.property_is_writeable = sc89601a_chg_property_is_writeable,
	.get_property = sc89601a_chg_get_property,
	.set_property = sc89601a_chg_set_property,
};

static int sc89601a_chg_init_psy(struct sc89601a *sc)
{
	struct power_supply_config cfg = {
		.drv_data = sc,
		.of_node = sc->dev->of_node,
		.supplied_to = sc89601a_psy_supplied_to,
		.num_supplicants = ARRAY_SIZE(sc89601a_psy_supplied_to),
	};

	memcpy(&sc->psy_desc, &sc89601a_psy_desc, sizeof(sc->psy_desc));
	sc->psy_desc.name = "charger";//dev_name(sc->dev);
	sc->psy = devm_power_supply_register(sc->dev, &sc->psy_desc,&cfg);
	
	sc->pdpe_psy = power_supply_get_by_name("pdpe-state");
	if(sc->pdpe_psy == NULL){
		pr_err("gezi get info->pdpe_psy failed\n");
	}
	
	if (!sc->chg_psy) {
		sc->chg_psy = power_supply_get_by_name("mtk-master-charger");
	}
	
	return IS_ERR(sc->psy) ? PTR_ERR(sc->psy) : 0;
}

static struct charger_ops sc89601a_chg_ops = {
	/* cable plug in/out */
	.plug_in = sc89601a_plug_in,
	.plug_out = sc89601a_plug_out,
    /* enable */
	.enable = sc89601a_charging,
	.is_enabled = sc89601a_is_charging_enable,
    /* charging current */
	.set_charging_current = sc89601a_set_ichg,
   	.get_charging_current = sc89601a_get_ichg,
	.get_min_charging_current = sc89601a_get_min_ichg,
	/* charging voltage */
	.set_constant_voltage = sc89601a_set_vchg,
	.get_constant_voltage = sc89601a_get_vchg,
	/* input current limit */
	.set_input_current = sc89601a_set_icl,
	.get_input_current = sc89601a_get_icl,
	.get_min_input_current = NULL,
	/* MIVR */
	.set_mivr = sc89601a_set_ivl,
	.get_mivr = sc89601a_get_ivl,
	.get_mivr_state = NULL,
	/* ADC */
	.get_adc = NULL,
	.get_vbus_adc = sc89601a_get_vbus_adc,
	.get_ibus_adc = sc89601a_get_ibus_adc,
	.get_ibat_adc = NULL,
	.get_tchg_adc = NULL,
	.get_zcv = NULL,
	/* charing termination */
	.set_eoc_current = sc89601a_set_ieoc,
	.enable_termination = sc89601a_enable_te,
	.reset_eoc_state = NULL,
	.safety_check = NULL,
	.is_charging_done = sc89601a_is_charging_done,
	/* power path */
	.enable_powerpath = NULL,
	.is_powerpath_enabled = NULL,
	/* timer */
	.enable_safety_timer = sc89601a_set_safety_timer,
	.is_safety_timer_enabled = sc89601a_is_safety_timer_enabled,
	.kick_wdt = sc89601a_kick_wdt,
	/* AICL */
	.run_aicl = NULL,
	/* PE+/PE+20 */
	.send_ta_current_pattern = sc8960x_set_pe_current_pattern,
	.set_pe20_efficiency_table = NULL,
	.send_ta20_current_pattern = NULL,
	.reset_ta = sc8960x_reset_pe_ta,
	.enable_cable_drop_comp = NULL,
	/* OTG */
	.set_boost_current_limit = sc89601a_set_boost_ilmt,
	.enable_otg = sc89601a_set_otg,
	.enable_discharge = NULL,
	/* charger type detection */
	.enable_chg_type_det = NULL,
	/* misc */
	.dump_registers = sc89601a_dump_register,
	.enable_hz = sc89601a_enable_hz,
	/* event */
	.event = sc89601a_event,
	/* 6pin battery */
	.enable_6pin_battery_charging = NULL,
};

static struct of_device_id sc89601a_charger_match_table[] = {
	 {
	 .compatible = "sc,sc89601a_charger",
	 .data = &pn_data[PN_SC89601A],
	 },
	{},
};
MODULE_DEVICE_TABLE(of, sc89601a_charger_match_table);


static int sc89601a_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct sc89601a *sc;
	const struct of_device_id *match;
	struct device_node *node = client->dev.of_node;

	int ret = 0;

	sc = devm_kzalloc(&client->dev, sizeof(struct sc89601a), GFP_KERNEL);
	if (!sc)
		return -ENOMEM;

	sc->dev = &client->dev;
	sc->client = client;

	i2c_set_clientdata(client, sc);

	mutex_init(&sc->i2c_rw_lock);

	ret = sc89601a_detect_device(sc);
	if (ret) {
		pr_err("No sc89601a device found!\n");
		return -ENODEV;
	}

	match = of_match_node(sc89601a_charger_match_table, node);
	if (match == NULL) {
		pr_err("device tree match not found\n");
		return -EINVAL;
	}

	sc->platform_data = sc89601a_parse_dt(node, sc);

	if (!sc->platform_data) {
		pr_err("No platform data provided.\n");
		return -EINVAL;
	}

    ret = sc89601a_chg_init_psy(sc);
	if (ret < 0) {
		dev_err(sc->dev, "failed to init power supply\n");
		return -EINVAL;
	}

	ret = sc89601a_init_device(sc);
	if (ret) {
		pr_err("Failed to init device\n");
		return ret;
	}

	sc89601a_register_interrupt(sc);

	sc->chg_dev = charger_device_register(sc->chg_dev_name,
					      &client->dev, sc,
					      &sc89601a_chg_ops,
					      &sc89601a_chg_props);
	if (IS_ERR_OR_NULL(sc->chg_dev)) {
		ret = PTR_ERR(sc->chg_dev);
		return ret;
	}

	ret = sysfs_create_group(&sc->dev->kobj, &sc89601a_attr_group);
	if (ret)
		dev_err(sc->dev, "failed to register sysfs. err: %d\n", ret);

	determine_initial_status(sc);

	pr_err("sc89601a probe successfully, Part Num:%d, Revision:%d\n!",
	       sc->part_no, sc->revision);

	return 0;
}

static int sc89601a_charger_remove(struct i2c_client *client)
{
	struct sc89601a *sc = i2c_get_clientdata(client);

	sysfs_remove_group(&sc->dev->kobj, &sc89601a_attr_group);

	return 0;
}

static void sc89601a_charger_shutdown(struct i2c_client *client)
{
	struct sc89601a *sc = i2c_get_clientdata(client);
	
	sc89601a_adc_stop(sc);
}

static struct i2c_driver sc89601a_charger_driver = {
	.driver = {
		   .name = "sc89601a-charger",
		   .owner = THIS_MODULE,
		   .of_match_table = sc89601a_charger_match_table,
		   },

	.probe = sc89601a_charger_probe,
	.remove = sc89601a_charger_remove,
	.shutdown = sc89601a_charger_shutdown,

};

module_i2c_driver(sc89601a_charger_driver);

MODULE_DESCRIPTION("SC SC89601A Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("South Chip");

