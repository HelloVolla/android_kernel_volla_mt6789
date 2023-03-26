/*
 * Copyright (c) 2015 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

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
//#endif

//#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
//#include "../mediatek/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"
//#endif


struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos, *bias_neg;

	bool prepared;
	bool enabled;

	int error;
};


#define lcm_dcs_write_seq(ctx, seq...)                                         \
	({                                                                     \
		const u8 d[] = {seq};                                          \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64,                           \
				 "DCS sequence too big for stack");            \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

#define lcm_dcs_write_seq_static(ctx, seq...)                                  \
	({                                                                     \
		static const u8 d[] = {seq};                                   \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
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
		dev_err(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

#ifdef PANEL_SUPPORT_READBACK
static int lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret, cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = {0};
	static int ret;

	if (ret == 0) {
		ret = lcm_dcs_read(ctx, 0x0A, buffer, 1);
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif
/*
	lcm_dcs_write_seq_static(ctx,0XB9,0XFF,0X83,0X99);
	lcm_dcs_write_seq_static(ctx,0XBA,0X63,0X23);
	lcm_dcs_write_seq_static(ctx,0XB1,0X00,0X04,0X71,0X91,0X01,0X32,0X33,0X11,0X11,0X4D,0X57,0X06);
	lcm_dcs_write_seq_static(ctx,0XB2,0X00,0X80,0X80,0XCC,0X05,0X07,0X5A,0X11,0X00,0X00,0X10);
	lcm_dcs_write_seq_static(ctx,0XB4,0X00,0XFF,0X02,0XA7,0X02,0XA7,0X02,0XA7,0X02,0X00,0X03,0X05,0X00,0X2D,0X03,0X0E,0X0A,0X21,0X03,0X02,0X00,0X0B,0XA5,0X87,0X02,0XA7,0X02,0XA7,0X02,0XA7,0X02,0X00,0X03,0X05,0X00,0X2D,0X03,0X0E,0X0A,0X02,0X00,0X0B,0XA5,0X01);
	lcm_dcs_write_seq_static(ctx,0XD3,0X00,0X0C,0X03,0X03,0X00,0X00,0X14,0X04,0X32,0X10,0X09,0X00,0X09,0X32,0X10,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X11,0X00,0X02,0X02,0X03,0X00,0X00,0X00,0X0A,0X40);
	lcm_dcs_write_seq_static(ctx,0XD5,0X18,0X18,0X18,0X18,0X03,0X02,0X01,0X00,0X18,0X18,0X18,0X18,0X18,0X18,0X19,0X19,0X21,0X20,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X2F,0X2F,0X30,0X30,0X31,0X31);
	lcm_dcs_write_seq_static(ctx,0XD6,0X18,0X18,0X18,0X18,0X00,0X01,0X02,0X03,0X18,0X18,0X40,0X40,0X19,0X19,0X18,0X18,0X20,0X21,0X40,0X40,0X18,0X18,0X18,0X18,0X18,0X18,0X2F,0X2F,0X30,0X30,0X31,0X31);
	lcm_dcs_write_seq_static(ctx,0XD8,0XAF,0XAA,0XEA,0XAA,0XAF,0XAA,0XEA,0XAA,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00);
	lcm_dcs_write_seq_static(ctx,0XBD,0X01);
	lcm_dcs_write_seq_static(ctx,0XD8,0XFF,0XEF,0XEA,0XBF,0XFF,0XEF,0XEA,0XBF,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00);
	lcm_dcs_write_seq_static(ctx,0XBD,0X02);
	lcm_dcs_write_seq_static(ctx,0XD8,0XFF,0XEF,0XEA,0XBF,0XFF,0XEF,0XEA,0XBF);
	lcm_dcs_write_seq_static(ctx,0XBD,0X00);
	lcm_dcs_write_seq_static(ctx,0XE0,0X01,0X20,0X2B,0X26,0X53,0X5D,0X6D,0X6A,0X72,0X7C,0X84,0X8A,0X8F,0X99,0XA1,0XA5,0XAA,0XB5,0XBB,0XC1,0XB6,0XC4,0XC7,0X66,0X61,0X6B,0X73,0X01,0X11,0X21,0X20,0X53,0X5D,0X6D,0X6A,0X72,0X7C,0X84,0X8A,0X8F,0X99,0XA1,0XA5,0XAA,0XB3,0XB2,0XC1,0XB6,0XC4,0XC7,0X66,0X61,0X6B,0X73);
	lcm_dcs_write_seq_static(ctx,0XB6,0X81,0X81);
	lcm_dcs_write_seq_static(ctx,0XD2,0X66);
	lcm_dcs_write_seq_static(ctx,0XCC,0X08);
*/
static void lcm_panel_init(struct lcm *ctx)
{
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return;
	}
	//gpiod_set_value(ctx->reset_gpio, 0);
	//udelay(15 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	mdelay(20);
	gpiod_set_value(ctx->reset_gpio, 0);
	mdelay(20);
	gpiod_set_value(ctx->reset_gpio, 1);
	mdelay(150);//ili9882q at least 10ms
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	lcm_dcs_write_seq_static(ctx,0XB9,0XFF,0X83,0X99);
	lcm_dcs_write_seq_static(ctx,0XBA,0X63,0X23);
	lcm_dcs_write_seq_static(ctx,0XB1,0X00,0X04,0X71,0X91,0X01,0X32,0X33,0X11,0X11,0X4D,0X57,0X06);
	lcm_dcs_write_seq_static(ctx,0XB2,0X00,0X80,0X80,0XCC,0X05,0X07,0X5A,0X11,0X00,0X00,0X10);
	lcm_dcs_write_seq_static(ctx,0XB4,0X00,0XFF,0X02,0XA7,0X02,0XA7,0X02,0XA7,0X02,0X00,0X03,0X05,0X00,0X2D,0X03,0X0E,0X0A,0X21,0X03,0X02,0X00,0X0B,0XA5,0X87,0X02,0XA7,0X02,0XA7,0X02,0XA7,0X02,0X00,0X03,0X05,0X00,0X2D,0X03,0X0E,0X0A,0X02,0X00,0X0B,0XA5,0X01);
	lcm_dcs_write_seq_static(ctx,0XD3,0X00,0X0C,0X03,0X03,0X00,0X00,0X14,0X04,0X32,0X10,0X09,0X00,0X09,0X32,0X10,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X11,0X00,0X02,0X02,0X03,0X00,0X00,0X00,0X0A,0X40);
	lcm_dcs_write_seq_static(ctx,0XD5,0X18,0X18,0X18,0X18,0X03,0X02,0X01,0X00,0X18,0X18,0X18,0X18,0X18,0X18,0X19,0X19,0X21,0X20,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X18,0X2F,0X2F,0X30,0X30,0X31,0X31);
	lcm_dcs_write_seq_static(ctx,0XD6,0X18,0X18,0X18,0X18,0X00,0X01,0X02,0X03,0X18,0X18,0X40,0X40,0X19,0X19,0X18,0X18,0X20,0X21,0X40,0X40,0X18,0X18,0X18,0X18,0X18,0X18,0X2F,0X2F,0X30,0X30,0X31,0X31);
	lcm_dcs_write_seq_static(ctx,0XD8,0XAF,0XAA,0XEA,0XAA,0XAF,0XAA,0XEA,0XAA,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00);
	lcm_dcs_write_seq_static(ctx,0XBD,0X01);
	lcm_dcs_write_seq_static(ctx,0XD8,0XFF,0XEF,0XEA,0XBF,0XFF,0XEF,0XEA,0XBF,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00);
	lcm_dcs_write_seq_static(ctx,0XBD,0X02);
	lcm_dcs_write_seq_static(ctx,0XD8,0XFF,0XEF,0XEA,0XBF,0XFF,0XEF,0XEA,0XBF);
	lcm_dcs_write_seq_static(ctx,0XBD,0X00);
	lcm_dcs_write_seq_static(ctx,0XE0,0X01,0X20,0X2B,0X26,0X53,0X5D,0X6D,0X6A,0X72,0X7C,0X84,0X8A,0X8F,0X99,0XA1,0XA5,0XAA,0XB5,0XBB,0XC1,0XB6,0XC4,0XC7,0X66,0X61,0X6B,0X73,0X01,0X11,0X21,0X20,0X53,0X5D,0X6D,0X6A,0X72,0X7C,0X84,0X8A,0X8F,0X99,0XA1,0XA5,0XAA,0XB3,0XB2,0XC1,0XB6,0XC4,0XC7,0X66,0X61,0X6B,0X73);
	lcm_dcs_write_seq_static(ctx,0XB6,0X81,0X81);
	lcm_dcs_write_seq_static(ctx,0XD2,0X66);
	lcm_dcs_write_seq_static(ctx,0XCC,0X08);
	lcm_dcs_write_seq_static(ctx,0x11,0x00);
	mdelay(120);
	lcm_dcs_write_seq_static(ctx,0x29,0x00);
	mdelay(20);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_info("%s\n", __func__);
	if (!ctx->prepared)
		return 0;
	//lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x81, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x28);
	msleep(50);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(150);

	ctx->error = 0;
	ctx->prepared = false;

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

#if defined(CONFIG_PRIZE_LCD_BIAS)
	//display_bias_disable();
#else
	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);

	udelay(1000);

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);
#endif

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("%s\n", __func__);
	if (ctx->prepared)
		return 0;

#if defined(CONFIG_PRIZE_LCD_BIAS)
	display_ldo18_enable(1);
	display_bias_enable_v(5800);
#else
	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	udelay(2000);

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
#endif

	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif
	pr_info("%s-\n", __func__);
	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	pr_info("%s+\n", __func__);
	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}
/*
	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 5;
	params->dsi.vertical_frontporch = 9;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 20; //50--40
	params->dsi.horizontal_backporch = 20;
	params->dsi.horizontal_frontporch = 80;//  60-->75
*/
#define HFP (80)
#define HSA (20)
#define HBP (20)
#define VFP (9)
#define VSA (2)
#define VBP (5)
#define VAC (2160)
#define HAC (1080)

static struct drm_display_mode default_mode = {
	.clock = 156672,//htotal*vtotal*vrefresh/1000   163943   182495
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
	.physical_width_um = 67932,
	.physical_height_um = 150960,
	.pll_clk = 480,
	.vfp_low_power = VFP,
	.cust_esd_check = 0,
	.esd_check_enable = 0,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x9c,
	},

};

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static int panel_ata_check(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	unsigned char data[3] = {0x00, 0x00, 0x00};
	unsigned char id[3] = {0x83, 0x83, 0x83};
	ssize_t ret;

	ret = mipi_dsi_dcs_read(dsi, 0xDA, data, 3);
	if (ret < 0) {
		pr_err("%s error\n", __func__);
		return 0;
	}

	pr_info("ATA read data %x %x %x\n", data[0], data[1], data[2]);

	if (data[0] == id[0] &&
	    data[1] == id[1] &&
	    data[2] == id[2])
		return 1;

	pr_info("ATA expect data is %x %x %x\n", id[0], id[1], id[2]);

	return 0;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				 unsigned int level)
{
	char bl_tb0[] = {0x51, 0xFF};

	bl_tb0[1] = level;

	if (!cb)
		return -1;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 0;
}

static int lcm_get_virtual_heigh(void)
{
	return VAC;
}

static int lcm_get_virtual_width(void)
{
	return HAC;
}


static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = panel_ata_check,
	.get_virtual_heigh = lcm_get_virtual_heigh,
	.get_virtual_width = lcm_get_virtual_width,
};
#endif

static int lcm_get_modes(struct drm_panel *panel,
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

	connector->display_info.width_mm = 68;
	connector->display_info.height_mm = 151;

	return 2;
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;

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

	pr_info("%s+\n", __func__);
	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE
			 | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET
			 | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

	ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(dev, "%s: cannot get bias-pos 0 %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	devm_gpiod_put(dev, ctx->bias_pos);

	ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(dev, "%s: cannot get bias-neg 1 %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	devm_gpiod_put(dev, ctx->bias_neg);

	ctx->prepared = true;
	ctx->enabled = true;
	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);

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

	pr_info("%s-\n", __func__);

	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif
	pr_info("%s+\n", __func__);
	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
#endif

	return 0;
}

static const struct of_device_id lcm_of_match[] = {
	{ .compatible = "fhd,hx8399c,vdo", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel-fhd-hx8399c-vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("Cui Zhang <cui.zhang@mediatek.com>");
MODULE_DESCRIPTION("truly ili9882q VDO LCD Panel Driver");
MODULE_LICENSE("GPL v2");
