// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/power_supply.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

#include "../../../misc/mediatek/gate_ic/gate_i2c.h"

/* enable this to check panel self -bist pattern */
/* #define PANEL_BIST_PATTERN */
/****************TPS65132***********/
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
//#include "lcm_i2c.h"

static char bl_tb0[] = { 0x51, 0xff };

//TO DO: You have to do that remove macro BYPASSI2C and solve build error
//otherwise voltage will be unstable
//#define BYPASSI2C

#ifndef BYPASSI2C
/* i2c control start */
#define LCM_I2C_ID_NAME "I2C_LCD_BIAS"
static struct i2c_client *_lcm_i2c_client;

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int _lcm_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id);
static int _lcm_i2c_remove(struct i2c_client *client);

/*****************************************************************************
 * Data Structure
 *****************************************************************************/
struct _lcm_i2c_dev {
	struct i2c_client *client;
};

static const struct of_device_id _lcm_i2c_of_match[] = {
	{
		.compatible = "mediatek,I2C_LCD_BIAS",
	},
	{},
};

static const struct i2c_device_id _lcm_i2c_id[] = { { LCM_I2C_ID_NAME, 0 },
						    {} };

static struct i2c_driver _lcm_i2c_driver = {
	.id_table = _lcm_i2c_id,
	.probe = _lcm_i2c_probe,
	.remove = _lcm_i2c_remove,
	/* .detect		   = _lcm_i2c_detect, */
	.driver = {
		.owner = THIS_MODULE,
		.name = LCM_I2C_ID_NAME,
		.of_match_table = _lcm_i2c_of_match,
	},
};

/*****************************************************************************
 * Function
 *****************************************************************************/

#ifdef VENDOR_EDIT
// shifan@bsp.tp 20191226 add for loading tp fw when screen lighting on
extern void lcd_queue_load_tp_fw(void);
#endif /*VENDOR_EDIT*/

static int _lcm_i2c_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	pr_err("[LCM][I2C] %s\n", __func__);
	pr_err("[LCM][I2C] NT: info==>name=%s addr=0x%x\n", client->name, client->addr);
	_lcm_i2c_client = client;
	return 0;
}

static int _lcm_i2c_remove(struct i2c_client *client)
{
	pr_debug("[LCM][I2C] %s\n", __func__);
	_lcm_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}

static int _lcm_i2c_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = _lcm_i2c_client;
	char write_data[2] = { 0 };

	if (client == NULL) {
		pr_err("ERROR!! _lcm_i2c_client is null\n");
		return 0;
	}

	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		pr_info("[LCM][ERROR] _lcm_i2c write data fail !!\n");
	else{
		pr_info("[LCM][ERROR] _lcm_i2c write data success !!\n");
	}

	return ret;
}

/*
 * module load/unload record keeping
 */
static int _lcm_i2c_init(void)
{
	pr_err("[LCM][I2C] %s\n", __func__);
	i2c_add_driver(&_lcm_i2c_driver);
	pr_err("[LCM][I2C] %s success\n", __func__);
	return 0;
}

static void _lcm_i2c_exit(void)
{
	pr_debug("[LCM][I2C] %s\n", __func__);
	i2c_del_driver(&_lcm_i2c_driver);
}

//module_init(_lcm_i2c_init);
//module_exit(_lcm_i2c_exit);
/***********************************/
#endif

struct jdi {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos;
	struct gpio_desc *bias_neg;
	bool prepared;
	bool enabled;

	unsigned int gate_ic;

	int error;
};

#define jdi_dcs_write_seq(ctx, seq...)                                         \
	({                                                                     \
		const u8 d[] = { seq };                                        \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64,                           \
				 "DCS sequence too big for stack");            \
		jdi_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

#define jdi_dcs_write_seq_static(ctx, seq...)                                  \
	({                                                                     \
		static const u8 d[] = { seq };                                 \
		jdi_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

static inline struct jdi *panel_to_jdi(struct drm_panel *panel)
{
	return container_of(panel, struct jdi, panel);
}

#ifdef PANEL_SUPPORT_READBACK
static int jdi_dcs_read(struct jdi *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret,
			 cmd);
		ctx->error = ret;
	}

	return ret;
}

static void jdi_panel_get_data(struct jdi *ctx)
{
	u8 buffer[3] = { 0 };
	static int ret;

	pr_info("%s+\n", __func__);

	if (ret == 0) {
		ret = jdi_dcs_read(ctx, 0x0A, buffer, 1);
		pr_info("%s  0x%08x\n", __func__, buffer[0] | (buffer[1] << 8));
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

static void jdi_dcs_write(struct jdi *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static void jdi_panel_init(struct jdi *ctx)
{
	pr_err("gezi %s---------------------------%d\n", __func__,__LINE__);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	usleep_range(10 * 1000, 15 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(10 * 1000, 15 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10 * 1000, 15 * 1000);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	jdi_dcs_write_seq_static(ctx,0x0A,0x08),
	jdi_dcs_write_seq_static(ctx,0xF0,0x5A,0x59),
	jdi_dcs_write_seq_static(ctx,0xF1,0xA5,0xA6),
	jdi_dcs_write_seq_static(ctx,0xB0,0x82,0x81,0x05,0x04,0x87,0x86,0x84,0x85,0x66,0x66,0x33,0x33,0x20,0x01,0x01,0x78,0x01,0x01,0x0F,0x05,0x04,0x03,0x02,0x01,0x02,0x03,0x04,0x00,0x00,0x00),	 
	jdi_dcs_write_seq_static(ctx,0xB1,0x11,0x44,0x86,0x00,0x01,0x00,0x01,0x7C,0x01,0x01,0x04,0x08,0x54,0x00,0x00,0x00,0x44,0x40,0x02,0x01,0x40,0x02,0x01,0x40,0x02,0x01,0x40,0x02,0x01),	 
	jdi_dcs_write_seq_static(ctx,0xB2,0x54,0xC4,0x82,0x05,0x40,0x02,0x01,0x40,0x02,0x01,0x05,0x05,0x54,0x0C,0x0C,0x0D,0x0B),
	jdi_dcs_write_seq_static(ctx,0xB3,0x02,0x00,0x00,0x00,0x00,0x26,0x26,0x91,0xA2,0x33,0x44,0x00,0x26,0x00,0x18,0x01,0x02,0x08,0x20,0x30,0x08,0x09,0x44,0x20,0x40,0x20,0x40,0x08,0x09,0x22,0x33),
	jdi_dcs_write_seq_static(ctx,0xB4,0x0A,0x02,0xDC,0x1D,0x00,0x02,0x02,0x02,0x02,0x12,0x10,0x02,0x02,0x0E,0x0C,0x04,0x03,0x03,0x03,0x03,0x03,0x03,0xFF,0xFF,0xFC,0x00,0x00,0x00),
	jdi_dcs_write_seq_static(ctx,0xB5,0x0B,0x02,0xDC,0x1D,0x00,0x02,0x02,0x02,0x02,0x13,0x11,0x02,0x02,0x0F,0x0D,0x05,0x03,0x03,0x03,0x03,0x03,0x03,0xFF,0xFF,0xFC,0x00,0x00,0x00),
	jdi_dcs_write_seq_static(ctx,0xB8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00),
	jdi_dcs_write_seq_static(ctx,0xBB,0x01,0x05,0x09,0x11,0x0D,0x19,0x1D,0x55,0x25,0x69,0x00,0x21,0x25),
	jdi_dcs_write_seq_static(ctx,0xBC,0x00,0x00,0x00,0x00,0x02,0x20,0xFF,0x00,0x03,0x33,0x01,0x73,0x33,0x00),
	jdi_dcs_write_seq_static(ctx,0xBD,0xE9,0x02,0x4F,0xCF,0x72,0xA4,0x08,0x44,0xAE,0x15),
	jdi_dcs_write_seq_static(ctx,0xBE,0x65,0x65,0x50,0x46,0x0C,0x77,0x43,0x07,0x0E,0x0E),
	jdi_dcs_write_seq_static(ctx,0xBF,0x07,0x25,0x07,0x25,0x7F,0x00,0x11,0x04),
	jdi_dcs_write_seq_static(ctx,0xC0,0x10,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0xFF,0x00),
	jdi_dcs_write_seq_static(ctx,0xC1,0xC0,0x0C,0x20,0x96,0x04,0x30,0x30,0x04,0x2A,0x18,0x36,0x00,0x07,0xCF,0xFF,0xFF,0xC0,0x00,0xC0),
	jdi_dcs_write_seq_static(ctx,0xC2,0x00),
	jdi_dcs_write_seq_static(ctx,0xC3,0x06,0x00,0xFF,0x00,0xFF,0x00,0x00,0x81,0x01),
	jdi_dcs_write_seq_static(ctx,0xC4,0x84,0x01,0x2B,0x41,0x00,0x3C,0x00,0x03,0x03,0x2E),
	jdi_dcs_write_seq_static(ctx,0xC5,0x03,0x1C,0xC0,0xB8,0x50,0x10,0x64,0x44,0x08,0x09,0x26),
	jdi_dcs_write_seq_static(ctx,0xC6,0x87,0x9A,0x2A,0x29,0x29,0x33,0x7F,0x04,0x08,0x00),
	jdi_dcs_write_seq_static(ctx,0xC7,0xF7,0xA8,0x81,0x65,0x37,0x18,0xEA,0x43,0x14,0xF0,0xCA,0x9F,0xF9,0xCF,0xB2,0x88,0x71,0x4E,0x1A,0x7E,0xC0,0x00),
	jdi_dcs_write_seq_static(ctx,0xC8,0xF7,0xA8,0x81,0x65,0x37,0x18,0xEA,0x43,0x14,0xF0,0xCA,0x9F,0xF9,0xCF,0xB2,0x88,0x71,0x4E,0x1A,0x7E,0xC0,0x00),
	jdi_dcs_write_seq_static(ctx,0xCB,0x00),
	jdi_dcs_write_seq_static(ctx,0xD0,0x80,0x0D,0xFF,0x0F,0x61),
	jdi_dcs_write_seq_static(ctx,0xD2,0x42),
	jdi_dcs_write_seq_static(ctx,0xFE,0xFF,0xFF,0xFF,0x40),
	jdi_dcs_write_seq_static(ctx,0xF1,0x5A,0x59),
	jdi_dcs_write_seq_static(ctx,0xF0,0xA5,0xA6),
	jdi_dcs_write_seq_static(ctx,0x35,0x00),
	jdi_dcs_write_seq_static(ctx,0x26,0x01),

	jdi_dcs_write_seq_static(ctx, 0x11);
	msleep(200);
	jdi_dcs_write_seq_static(ctx, 0x29);
	msleep(100);
	jdi_dcs_write_seq_static(ctx, 0x26,0x01);	

	jdi_dcs_write_seq(ctx, bl_tb0[0], bl_tb0[1]);
	
	pr_err("gezi %s---------------------------%d\n", __func__,__LINE__);
	pr_info("%s-\n", __func__);
}

static int jdi_disable(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}
static bool is_gesture_wakeup_enabled(void)
{
	struct power_supply *sw_psy = NULL;
	union power_supply_propval val = {0,};
	int ret = 0;
    if (!sw_psy) { 
       sw_psy = power_supply_get_by_name("sar_cali");
        if (!sw_psy){
            pr_err("sw_psy psy not found!\n");
			return false;
		}
    }
	
	//val.intval = 0;
    ret = power_supply_get_property(sw_psy, POWER_SUPPLY_PROP_TEMP, &val);
	
	return val.intval;
}

static int jdi_unprepare(struct drm_panel *panel)
{

	struct jdi *ctx = panel_to_jdi(panel);

	if (!ctx->prepared)
		return 0;
	pr_err("gezi %s----------------000-----------%d\n", __func__,__LINE__);
	
	jdi_dcs_write_seq_static(ctx, 0x26,0x08);
	jdi_dcs_write_seq_static(ctx, 0x28);
	msleep(200);
	jdi_dcs_write_seq_static(ctx, 0x10);
	msleep(50);
	jdi_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	msleep(50);
	jdi_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	msleep(150);

	/*
	 * ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	 * gpiod_set_value(ctx->reset_gpio, 0);
	 * devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	 */
	if(!is_gesture_wakeup_enabled()){
		pr_err("gezi is_gesture_wakeup_enabled false...\n");
		if (ctx->gate_ic == 0) {
			ctx->bias_neg =
				devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
			gpiod_set_value(ctx->bias_neg, 0);
			devm_gpiod_put(ctx->dev, ctx->bias_neg);

			usleep_range(2000, 2001);

			ctx->bias_pos =
				devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
			gpiod_set_value(ctx->bias_pos, 0);
			devm_gpiod_put(ctx->dev, ctx->bias_pos);
		} else if (ctx->gate_ic == 4831) {
			_gate_ic_i2c_panel_bias_enable(0);
			_gate_ic_Power_off();
		}
	}
	else{
		pr_err("gezi is_gesture_wakeup_enabled true...\n");
	}

	ctx->error = 0;
	ctx->prepared = false;
	pr_err("gezi %s--------------000-------------%d\n", __func__,__LINE__);
	return 0;
}

static int jdi_prepare(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);
	int ret;
	pr_err("gezi %s----------------000-----------%d\n", __func__,__LINE__);
	pr_info("%s+\n", __func__);
	if (ctx->prepared)
		return 0;

	// lcd reset H -> L -> L
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10000, 10001);
	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(20);
	gpiod_set_value(ctx->reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	// end
	if (ctx->gate_ic == 0) {
		ctx->bias_pos =
			devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->bias_pos, 1);
		devm_gpiod_put(ctx->dev, ctx->bias_pos);

		usleep_range(2000, 2001);
		ctx->bias_neg =
			devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->bias_neg, 1);
		devm_gpiod_put(ctx->dev, ctx->bias_neg);
	} else if (ctx->gate_ic == 4831) {
		_gate_ic_Power_on();
		_gate_ic_i2c_panel_bias_enable(1);
	}
#ifndef BYPASSI2C
	_lcm_i2c_write_bytes(0x0, 0x14);
	_lcm_i2c_write_bytes(0x1, 0x14);
#endif
	jdi_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		jdi_unprepare(panel);

	ctx->prepared = true;
#ifdef PANEL_SUPPORT_READBACK
	jdi_panel_get_data(ctx);
#endif

#ifdef VENDOR_EDIT
	// shifan@bsp.tp 20191226 add for loading tp fw when screen lighting on
	lcd_queue_load_tp_fw();
#endif
	pr_err("gezi %s---------------000------------%d\n", __func__,__LINE__);
	pr_info("%s-\n", __func__);
	return ret;
}

static int jdi_enable(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

#define HFP (48)
#define HSA (4)
#define HBP (48)
#define VFP (150)
#define VSA (4)
#define VBP (12)
#define VAC (1560)
#define HAC (720)

#define PCLK_IN_KHZ \
    ((HAC+HFP+HSA+HBP)*(VAC+VFP+VSA+VBP)*(60)/1000) //2500

static const struct drm_display_mode default_mode = {
	.clock = PCLK_IN_KHZ,
	.hdisplay = HAC,
	.hsync_start = HAC + HFP,
	.hsync_end = HAC + HFP + HSA,
	.htotal = HAC + HFP + HSA + HBP,
	.vdisplay = VAC,
	.vsync_start = VAC + VFP,
	.vsync_end = VAC + VFP + VSA,
	.vtotal = VAC + VFP + VSA + VBP,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
	.pll_clk = 278,
	.vfp_low_power = 124,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
/*
	.lane_swap_en = 1,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.data_rate = 1190,
*/
};

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

static int jdi_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				 unsigned int level)
{

	if (level > 255)
		level = 255;
	pr_info("%s backlight = -%d\n", __func__, level);
	bl_tb0[1] = (u8)level;

	if (!cb)
		return -1;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	return 0;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct jdi *ctx = panel_to_jdi(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = jdi_setbacklight_cmdq,
	.ata_check = panel_ata_check,
};
#endif

static int jdi_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			 default_mode.hdisplay, default_mode.vdisplay,
			 drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = 65;
	connector->display_info.height_mm = 140;

	return 1;
}

static const struct drm_panel_funcs jdi_drm_funcs = {
	.disable = jdi_disable,
	.unprepare = jdi_unprepare,
	.prepare = jdi_prepare,
	.enable = jdi_enable,
	.get_modes = jdi_get_modes,
};

static int jdi_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	struct jdi *ctx;
	struct device_node *backlight;
	unsigned int value;
	int ret;
	int vbus_gpio = 0;

	pr_info("%s+\n", __func__);
	pr_err("gezi %s------------------------%d\n", __func__,__LINE__);
	
	_lcm_i2c_init();
	
	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_info("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_info("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct jdi), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET |
			MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ret = of_property_read_u32(dev->of_node, "gate-ic", &value);
	if (ret < 0)
		value = 0;
	else
		ctx->gate_ic = value;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}
	
	vbus_gpio = of_get_named_gpio(dev->of_node, "vbus-gpio", 0);

	if (vbus_gpio < 0) {
		pr_err("%s no vbus_gpio info\n", __func__);
		//return ret;
	}
	else{
		ret = gpio_request(vbus_gpio, "vbus_gpio_enable");
		if (ret < 0) {
			pr_err("%s gpio_request err\n", __func__);
		}
		else{
			gpio_direction_output(vbus_gpio, 0);
		}
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(dev, "cannot get reset-gpios %ld\n",
			 PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);
	if (ctx->gate_ic == 0) {
		ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->bias_pos)) {
			dev_info(dev, "cannot get bias-gpios 0 %ld\n",
				 PTR_ERR(ctx->bias_pos));
			return PTR_ERR(ctx->bias_pos);
		}
		devm_gpiod_put(dev, ctx->bias_pos);

		ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
		if (IS_ERR(ctx->bias_neg)) {
			dev_info(dev, "cannot get bias-gpios 1 %ld\n",
				 PTR_ERR(ctx->bias_neg));
			return PTR_ERR(ctx->bias_neg);
		}
		devm_gpiod_put(dev, ctx->bias_neg);
	}
	ctx->prepared = true;
	ctx->enabled = true;
	drm_panel_init(&ctx->panel, dev, &jdi_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;

#endif

	pr_info("%s- jdi,nt36672e,vdo,60hz\n", __func__);
	pr_err("gezi %s---------------------------%d\n", __func__,__LINE__);
	return ret;
}

static int jdi_remove(struct mipi_dsi_device *dsi)
{
	struct jdi *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
#endif
#ifndef BYPASSI2C
	_lcm_i2c_exit();
#endif

	return 0;
}

static const struct of_device_id jdi_of_match[] = {
	{
	    .compatible = "jdi,nt36672e,vdo,60hz",
	},
	{}
};

MODULE_DEVICE_TABLE(of, jdi_of_match);

static struct mipi_dsi_driver jdi_driver = {
	.probe = jdi_probe,
	.remove = jdi_remove,
	.driver = {
		.name = "panel-jdi-nt36672e-vdo-60hz",
		.owner = THIS_MODULE,
		.of_match_table = jdi_of_match,
	},
};

module_mipi_dsi_driver(jdi_driver);

MODULE_AUTHOR("shaohua deng <shaohua.deng@mediatek.com>");
MODULE_DESCRIPTION("JDI NT36672E VDO 60HZ AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
