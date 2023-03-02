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

#define lcm_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

//prize add by lipengpeng 20210420 start 
//extern int fts_reset_proc_on(int delay);
//extern int fts_reset_proc(int hdelayms);
//extern void fts_enter_low_power(void);
//extern int fts_reset_proc_off(int delay);
//extern int fts_pinctrl_select_i2c_gpio(int n);
//prize add by lipengpeng 20210420 end 
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
		ret = lcm_dcs_read(ctx,  0x0A, buffer, 1);
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

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
	msleep(10);
	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(10);
	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(5);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

/*----------------------LCD initial code start----------------------*/
//CMD2 ENABLE
lcm_dcs_write_seq_static(ctx, 0x00,0x00);
lcm_dcs_write_seq_static(ctx, 0xFF,0x87,0x56,0x01);

lcm_dcs_write_seq_static(ctx, 0x00,0x80);
lcm_dcs_write_seq_static(ctx, 0xFF,0x87,0x56);

//Panel resulotion
lcm_dcs_write_seq_static(ctx, 0x00,0xA1);
lcm_dcs_write_seq_static(ctx, 0xB3,0x04,0x38,0x09,0x60,0x00,0xFC);   
//----------------------------------------------//
//TCON Setting
lcm_dcs_write_seq_static(ctx, 0x00,0x80);
lcm_dcs_write_seq_static(ctx, 0xC0,0x00,0x86,0x00,0x24,0x00,0x46);

lcm_dcs_write_seq_static(ctx, 0x00,0x90);
lcm_dcs_write_seq_static(ctx, 0xC0,0x00,0x86,0x00,0x24,0x00,0x46);

lcm_dcs_write_seq_static(ctx, 0x00,0xA0);
lcm_dcs_write_seq_static(ctx, 0xC0,0x01,0x07,0x00,0x24,0x00,0x46);

lcm_dcs_write_seq_static(ctx, 0x00,0xB0);
lcm_dcs_write_seq_static(ctx, 0xC0,0x00,0x86,0x00,0x24,0x46);

lcm_dcs_write_seq_static(ctx, 0x00,0xC1);
lcm_dcs_write_seq_static(ctx, 0xC0,0x00,0xC4,0x00,0x99,0x00,0x83,0x00,0xE5);

lcm_dcs_write_seq_static(ctx, 0x00,0xD7);
lcm_dcs_write_seq_static(ctx, 0xC0,0x00,0x83,0x00,0x24,0x00,0x46);

lcm_dcs_write_seq_static(ctx, 0x00,0xA3);
lcm_dcs_write_seq_static(ctx, 0xC1,0x00,0x34,0x00,0x34,0x00,0x02);

lcm_dcs_write_seq_static(ctx, 0x00,0x80);
lcm_dcs_write_seq_static(ctx, 0xCE,0x01,0x81,0x09,0x13,0x00,0xEC,0x00,0xEC,0x00,0x85,0x00,0xA0,0x00,0x74,0x00,0x74);

lcm_dcs_write_seq_static(ctx, 0x00,0x90);
lcm_dcs_write_seq_static(ctx, 0xCE,0x00,0x8E,0x0E,0xB6,0x00,0x8E,0x80,0x09,0x13,0x00,0x04,0x00,0x3A,0x39,0x2B);

lcm_dcs_write_seq_static(ctx, 0x00,0xA0);
lcm_dcs_write_seq_static(ctx, 0xCE,0x20,0x00,0x00);

lcm_dcs_write_seq_static(ctx, 0x00,0xB0);
lcm_dcs_write_seq_static(ctx, 0xCE,0x22,0x00,0x00);

lcm_dcs_write_seq_static(ctx, 0x00,0xD1);
lcm_dcs_write_seq_static(ctx, 0xCE,0x00,0x00,0x01,0x00,0x00,0x00,0x00);

lcm_dcs_write_seq_static(ctx, 0x00,0xE1);
lcm_dcs_write_seq_static(ctx, 0xCE,0x08,0x02,0x4D,0x02,0x4D,0x02,0x4D,0x00,0x00,0x00,0x00);

lcm_dcs_write_seq_static(ctx, 0x00,0xF1);
lcm_dcs_write_seq_static(ctx, 0xCE,0x16,0x0B,0x0F,0x01,0x43,0x01,0x43,0x01,0x3D);

lcm_dcs_write_seq_static(ctx, 0x00,0xB0);
lcm_dcs_write_seq_static(ctx, 0xCF,0x00,0x00,0xDA,0xDE);

lcm_dcs_write_seq_static(ctx, 0x00,0xB5);
lcm_dcs_write_seq_static(ctx, 0xCF,0x05,0x05,0x12,0x16);

lcm_dcs_write_seq_static(ctx, 0x00,0xC0);
lcm_dcs_write_seq_static(ctx, 0xCF,0x09,0x09,0x4C,0x50);

lcm_dcs_write_seq_static(ctx, 0x00,0xC5);
lcm_dcs_write_seq_static(ctx, 0xCF,0x00,0x00,0x08,0x0C);
//---------------------------------------------------------------------------------//
//Panel scan mode
lcm_dcs_write_seq_static(ctx, 0x00,0xE8);
lcm_dcs_write_seq_static(ctx, 0xC0,0x40);

//BOE GIP setting
lcm_dcs_write_seq_static(ctx, 0x00,0xCA);
lcm_dcs_write_seq_static(ctx, 0xC0,0x80);

//VST
lcm_dcs_write_seq_static(ctx, 0x00,0x80);
lcm_dcs_write_seq_static(ctx, 0xc2,0x83,0x01,0x01,0x81,0x82,0x01,0x01,0x81);

//CKV1-3
lcm_dcs_write_seq_static(ctx, 0x00,0xa0);
lcm_dcs_write_seq_static(ctx, 0xc2,0x8a,0x0a,0x00,0x00,0x86,0x89,0x0a,0x00,0x00,0x86,0x88,0x0a,0x00,0x00,0x86);
//CKV4
lcm_dcs_write_seq_static(ctx, 0x00,0xb0);
lcm_dcs_write_seq_static(ctx, 0xc2,0x87,0x0a,0x00,0x00,0x86);

lcm_dcs_write_seq_static(ctx, 0x00,0xe0);
lcm_dcs_write_seq_static(ctx, 0xc2,0x33,0x33,0x33,0x33,0x33,0x33,0x00,0x00,0x03,0x09,0x10,0x6a,0x01,0x81);

lcm_dcs_write_seq_static(ctx, 0x00,0x80);
lcm_dcs_write_seq_static(ctx, 0xcb,0xfd,0xfd,0x3c,0x30,0xfd,0x0c,0x1c,0xfd,0x02,0xfe,0xfd,0xfd,0xfd,0xfd,0x00,0xfc);

lcm_dcs_write_seq_static(ctx, 0x00,0x90);
lcm_dcs_write_seq_static(ctx, 0xcb,0xff,0x00,0x00,0x00,0xff,0x00,0x00,0x00,0x00,0xff,0x00,0x00,0x00,0x00,0x00,0x00);

lcm_dcs_write_seq_static(ctx, 0x00,0xa0);
lcm_dcs_write_seq_static(ctx, 0xcb,0x00,0x00,0x00,0x00);

lcm_dcs_write_seq_static(ctx, 0x00,0xb0);
lcm_dcs_write_seq_static(ctx, 0xcb,0x50,0x41,0xA4,0x50);

lcm_dcs_write_seq_static(ctx, 0x00,0xc0);
lcm_dcs_write_seq_static(ctx, 0xcb,0x50,0x41,0xA4,0x50);

//============================================================================//
//BOE CGOUT Select
lcm_dcs_write_seq_static(ctx, 0x00,0x80);//U2D Left CGOUT Select
lcm_dcs_write_seq_static(ctx, 0xcc,0x25,0x25,0x00,0x24,0x29,0x29,0x1b,0x18,0x1a,0x17,0x19,0x16,0x00,0x00,0x00,0x00);
	
lcm_dcs_write_seq_static(ctx, 0x00,0x90);//U2D Left CGOUT Select
lcm_dcs_write_seq_static(ctx, 0xcc,0x07,0x09,0x01,0x26,0x26,0x22,0x24,0x03);

lcm_dcs_write_seq_static(ctx, 0x00,0x80);//U2D Right CGOUT Select
lcm_dcs_write_seq_static(ctx, 0xcd,0x25,0x25,0x00,0x24,0x29,0x29,0x1b,0x18,0x1a,0x17,0x19,0x16,0x00,0x00,0x00,0x00);

lcm_dcs_write_seq_static(ctx, 0x00,0x90);//U2D Right CGOUT Select
lcm_dcs_write_seq_static(ctx, 0xcd,0x06,0x08,0x01,0x26,0x26,0x22,0x24,0x02);

lcm_dcs_write_seq_static(ctx, 0x00,0xa0);//D2U Left CGOUT Select
lcm_dcs_write_seq_static(ctx, 0xcc,0x25,0x25,0x00,0x24,0x00,0x00,0x1b,0x18,0x1a,0x17,0x19,0x16,0x00,0x00,0x00,0x00);

lcm_dcs_write_seq_static(ctx, 0x00,0xb0);//D2U Left CGOUT Select
lcm_dcs_write_seq_static(ctx, 0xcc,0x08,0x06,0x01,0x26,0x26,0x24,0x23,0x02);

lcm_dcs_write_seq_static(ctx, 0x00,0xa0);//D2U Right CGOUT Select
lcm_dcs_write_seq_static(ctx, 0xcd,0x25,0x25,0x00,0x24,0x00,0x00,0x1b,0x18,0x1a,0x17,0x19,0x16,0x00,0x00,0x00,0x00);

lcm_dcs_write_seq_static(ctx, 0x00,0xb0);//D2U Right CGOUT Select
lcm_dcs_write_seq_static(ctx, 0xcd,0x09,0x07,0x01,0x26,0x26,0x24,0x23,0x03);
//============================================================================//
//Source mapping
lcm_dcs_write_seq_static(ctx, 0x00,0xA0);
lcm_dcs_write_seq_static(ctx, 0xC4,0x8d,0xd8,0x8d);

//pump x2
lcm_dcs_write_seq_static(ctx, 0x00,0x93); 
lcm_dcs_write_seq_static(ctx, 0xC5,0x5A);

//Gamma 32
lcm_dcs_write_seq_static(ctx, 0x00,0xA4);  
lcm_dcs_write_seq_static(ctx, 0xD7,0x00);

//EN_VGHO1 follow EN_VGH
lcm_dcs_write_seq_static(ctx, 0x00,0x9B);  
lcm_dcs_write_seq_static(ctx, 0xF5,0x4B);

//VGHO1 precharge off
lcm_dcs_write_seq_static(ctx, 0x00,0x93);  
lcm_dcs_write_seq_static(ctx, 0xF5,0x00,0x00);

//EN_VGLO1 follow EN_VGL
lcm_dcs_write_seq_static(ctx, 0x00,0x9D);  
lcm_dcs_write_seq_static(ctx, 0xF5,0x49);

//VGLO1 precharge off
lcm_dcs_write_seq_static(ctx, 0x00,0x82);  
lcm_dcs_write_seq_static(ctx, 0xF5,0x00,0x00);

//ahung:hvreg_en_vglo1s,hvreg_en_vglo2s
lcm_dcs_write_seq_static(ctx, 0x00,0x80);
lcm_dcs_write_seq_static(ctx, 0xF5,0x59,0x59);

//hvreg_en_vgho1s,hvreg_en_vgho2s,pump_en_vgh_s
lcm_dcs_write_seq_static(ctx, 0x00,0x84);
lcm_dcs_write_seq_static(ctx, 0xF5,0x59,0x59,0x59);

//pump_en_vgl_s
lcm_dcs_write_seq_static(ctx, 0x00,0x96);
lcm_dcs_write_seq_static(ctx, 0xF5,0x59);

//vclreg_en_s
lcm_dcs_write_seq_static(ctx, 0x00,0xA6);
lcm_dcs_write_seq_static(ctx, 0xF5,0x59);

//pwron 3->5
lcm_dcs_write_seq_static(ctx, 0x00,0xCA);
lcm_dcs_write_seq_static(ctx, 0xC0,0x80);

//ahung:vcom_en_vcom
lcm_dcs_write_seq_static(ctx, 0x00,0xB1);
lcm_dcs_write_seq_static(ctx, 0xF5,0x1f);

//GVDD=4.6V
//NGVDD=-4.6V
lcm_dcs_write_seq_static(ctx, 0x00,0x00);
lcm_dcs_write_seq_static(ctx, 0xD8,0x2B,0x2B);

lcm_dcs_write_seq_static(ctx, 0x00,0x00);
lcm_dcs_write_seq_static(ctx, 0xD9,0x22,0x22);

lcm_dcs_write_seq_static(ctx, 0x00,0x00);
lcm_dcs_write_seq_static(ctx, 0xE1,0x00,0x06,0x08,0x0F,0x2F,0x19,0x20,0x26,0x30,0x69,0x38,0x3E,0x44,0x49,0x20,0x4D,0x55,0x5C,0x63,0x99,0x6A,0x71,0x78,0x81,0x1C,0x8A,0x90,0x96,0x9D,0xDB,0xA6,0xB1,0xBF,0xC8,0x9A,0xD5,0xE7,0xF5,0xFF,0x07);
lcm_dcs_write_seq_static(ctx, 0x00,0x00);
lcm_dcs_write_seq_static(ctx, 0xE2,0x00,0x06,0x08,0x0F,0x2F,0x1B,0x22,0x28,0x32,0x69,0x3A,0x40,0x46,0x4B,0x20,0x4F,0x57,0x5E,0x65,0x99,0x6C,0x71,0x78,0x81,0x1C,0x8A,0x90,0x96,0x9D,0xDB,0xA6,0xB1,0xBF,0xC8,0x9A,0xD5,0xE7,0xF5,0xFF,0x07);
lcm_dcs_write_seq_static(ctx, 0x00,0x00);
lcm_dcs_write_seq_static(ctx, 0xE3,0x00,0x06,0x08,0x0F,0x2F,0x19,0x20,0x26,0x30,0x69,0x38,0x3E,0x44,0x49,0x20,0x4D,0x55,0x5C,0x63,0x99,0x6A,0x71,0x78,0x81,0x1C,0x8A,0x90,0x96,0x9D,0xDB,0xA6,0xB1,0xBF,0xC8,0x9A,0xD5,0xE7,0xF5,0xFF,0x07);
lcm_dcs_write_seq_static(ctx, 0x00,0x00);
lcm_dcs_write_seq_static(ctx, 0xE4,0x00,0x06,0x08,0x0F,0x2F,0x1B,0x22,0x28,0x32,0x69,0x3A,0x40,0x46,0x4B,0x20,0x4F,0x57,0x5E,0x65,0x99,0x6C,0x71,0x78,0x81,0x1C,0x8A,0x90,0x96,0x9D,0xDB,0xA6,0xB1,0xBF,0xC8,0x9A,0xD5,0xE7,0xF5,0xFF,0x07);
lcm_dcs_write_seq_static(ctx, 0x00,0x00);
lcm_dcs_write_seq_static(ctx, 0xE5,0x00,0x06,0x08,0x0F,0x2F,0x19,0x20,0x26,0x30,0x69,0x38,0x3E,0x44,0x49,0x20,0x4D,0x55,0x5C,0x63,0x99,0x6A,0x71,0x78,0x81,0x1C,0x8A,0x90,0x96,0x9D,0xDB,0xA6,0xB1,0xBF,0xC8,0x9A,0xD5,0xE7,0xF5,0xFF,0x07);
lcm_dcs_write_seq_static(ctx, 0x00,0x00);
lcm_dcs_write_seq_static(ctx, 0xE6,0x00,0x06,0x08,0x0F,0x2F,0x1B,0x22,0x28,0x32,0x69,0x3A,0x40,0x46,0x4B,0x20,0x4F,0x57,0x5E,0x65,0x99,0x6C,0x71,0x78,0x81,0x1C,0x8A,0x90,0x96,0x9D,0xDB,0xA6,0xB1,0xBF,0xC8,0x9A,0xD5,0xE7,0xF5,0xFF,0x07);

//VGHO1=8V & VGLO1=-8V
lcm_dcs_write_seq_static(ctx, 0x00,0xB6);
lcm_dcs_write_seq_static(ctx, 0xC5,0x19,0x19,0x19,0x19);

//Reduce Source ring for COB
lcm_dcs_write_seq_static(ctx, 0x00,0x85);
lcm_dcs_write_seq_static(ctx, 0xC4,0x1C);

lcm_dcs_write_seq_static(ctx, 0x00,0x80);
lcm_dcs_write_seq_static(ctx, 0xA4,0x6C);

lcm_dcs_write_seq_static(ctx, 0x00,0x8D);
lcm_dcs_write_seq_static(ctx, 0xA5,0x06);

lcm_dcs_write_seq_static(ctx, 0x00,0xc6);
lcm_dcs_write_seq_static(ctx, 0xF5,0x02);

lcm_dcs_write_seq_static(ctx, 0x00,0xa9);
lcm_dcs_write_seq_static(ctx, 0xF5,0xc1);

lcm_dcs_write_seq_static(ctx, 0x00,0xb1);
lcm_dcs_write_seq_static(ctx, 0xc5,0x8a);

lcm_dcs_write_seq_static(ctx, 0x00,0xb4);
lcm_dcs_write_seq_static(ctx, 0xc5,0x8a);

lcm_dcs_write_seq_static(ctx, 0x00,0xA0);
lcm_dcs_write_seq_static(ctx, 0xC3,0x15,0x01,0x23,0x45,0x54,0x32,0x10,0x00,0x00,0x00,0x34,0x50,0x12,0x21,0x05,0x43);

lcm_dcs_write_seq_static(ctx, 0x00,0x8C); 
lcm_dcs_write_seq_static(ctx, 0xC3,0x00,0x00,0x00);
//whs-chk-gap
//lcm_dcs_write_seq_static(ctx, 0x00,0x86);
//lcm_dcs_write_seq_static(ctx, 0xc0,0x00,0x00,0x00);
//lcm_dcs_write_seq_static(ctx, 0x00,0x89);
//lcm_dcs_write_seq_static(ctx, 0xc0,0x02,0x12,0x01);
lcm_dcs_write_seq_static(ctx, 0x00,0x89);
lcm_dcs_write_seq_static(ctx, 0xc0,0x02,0x0c,0x07);

lcm_dcs_write_seq_static(ctx, 0x00,0xCB);
lcm_dcs_write_seq_static(ctx, 0xC0,0x19);//power off插十帧黑画面

lcm_dcs_write_seq_static(ctx, 0x00,0x84);
lcm_dcs_write_seq_static(ctx, 0xc5,0x26,0x26);

lcm_dcs_write_seq_static(ctx, 0x00,0x80);
lcm_dcs_write_seq_static(ctx, 0xA4,0x6C);

lcm_dcs_write_seq_static(ctx, 0x00,0x97);
lcm_dcs_write_seq_static(ctx, 0xC4,0x01);
////Rotate[4:0]
//lcm_dcs_write_seq_static(ctx, 0x00,0x80);
//lcm_dcs_write_seq_static(ctx, 0xa7,0x10);
//GOFF width
lcm_dcs_write_seq_static(ctx, 0x00,0xD0);
lcm_dcs_write_seq_static(ctx, 0xC3,0x46,0x00,0x00,0x00,0x46,0x00,0x00,0x30,0x46,0x00,0x00,0x30);
//----------------------LCD initial code End----------------------//
lcm_dcs_write_seq_static(ctx,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0xFF,0xFF,0xFF,0xFF);
lcm_dcs_write_seq_static(ctx, 0x11,0);
msleep(120);
lcm_dcs_write_seq_static(ctx, 0x29,0);
msleep(30);
lcm_dcs_write_seq_static(ctx, 0x35,0x00);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
		
	if (!ctx->enabled)
		return 0;
	pr_info("%s\n", __func__);
	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}
	pr_info("%s_end\n", __func__);
	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	pr_info("%s\n", __func__);
	if (!ctx->prepared)
		return 0;

	//fts_enter_low_power();
	//msleep(50);
	lcm_dcs_write_seq_static(ctx, 0x28);
	msleep(50);
	lcm_dcs_write_seq_static(ctx, 0x10);
//	display_bias_disable();
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
	gpiod_set_value(ctx->reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
#if defined(CONFIG_PRIZE_LCD_BIAS)
	//display_bias_disable();
	//fts_reset_proc_off(5);
	//display_ldo18_enable(0);
	//fts_pinctrl_select_i2c_gpio(0);
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
	//msleep(20);

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
	//fts_pinctrl_select_i2c_gpio(1);
	//fts_reset_proc(120);
	display_ldo18_enable(1);
	msleep(10);
	//display_bias_enable();
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

	msleep(10);
	msleep(10);
	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	pr_info("%s\n", __func__);
	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}
	pr_info("%s_end\n", __func__);
	ctx->enabled = true;

	return 0;
}

#define HFP (50)
#define HSA (16)
#define HBP (50)
#define VFP (36)     //8  prize--huangjiwu --shanping
#define VSA (10)
#define VBP (60)
#define VAC (2400)//1920
#define HAC (1080)//1080
static struct drm_display_mode default_mode = {
	.clock = 179832,     //htotal*vtotal*vrefresh/1000   163943   182495
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
	.physical_width_um = 69500,
	.physical_height_um = 154440,
	.pll_clk = 540,
	.vfp_low_power = 36,
	.cust_esd_check = 0,
	.esd_check_enable = 0,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x9C,
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
	unsigned char id[3] = {0x00, 0x00, 0x00};
	ssize_t ret;

	ret = mipi_dsi_dcs_read(dsi, 0x4, data, 3);
	if (ret < 0) {
		pr_err("%s error\n", __func__);
		return 0;
	}

	printk("ATA read data %x %x %x\n", data[0], data[1], data[2]);

	if (data[0] == id[0] &&
			data[1] == id[1] &&
			data[2] == id[2])
		return 1;

	printk("ATA expect read data is %x %x %x\n",
			id[0], id[1], id[2]);

	return 0;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
	void *handle, unsigned int level)
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

	connector->display_info.width_mm = 69;
	connector->display_info.height_mm = 154;

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
	{ .compatible = "csot,ft8756,vdo,xbd", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel-csot-ft8756-vdo-xbd",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("Yi-Lun Wang <Yi-Lun.Wang@mediatek.com>");
MODULE_DESCRIPTION("truly td4330 CMD LCD Panel Driver");
MODULE_LICENSE("GPL v2");
