/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 ov32b40gmsmipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
/* #include <asm/system.h> */
/* #include <linux/xlog.h> */
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov32b40gmsmipiraw_Sensor.h"

#ifdef CONFIG_MTK_CAM_CAL
extern int read_ov32b40gms_eeprom_mtk_fmt(void);
#endif


/****************************Modify following Strings for debug****************************/
#define PFX "ov32b40gms_camera_sensor"
#define LOG_1 LOG_INF("ov32b40gms,MIPI 2LANE\n")
#define LOG_2 LOG_INF("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n")
/****************************   Modify end    *******************************************/

#define LOG_INF(format, args...)	pr_err(PFX "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);
static MUINT32 g_sync_mode = SENSOR_MASTER_SYNC_MODE;

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = OV32B40GMS_SENSOR_ID,		//record sensor id defined in Kd_imgsensor.h

	.checksum_value = 0x9e08861c,		//checksum value for Camera Auto Test

	.pre = {
		.pclk = 58000000,
		.linelength = 720,
		.framelength = 2664,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000,
	},
	.cap = {
		.pclk = 58000000,
		.linelength = 720,
		.framelength = 2664,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000,
	},
	.cap1 = {
		.pclk = 58000000,
		.linelength = 720,
		.framelength = 2664,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000,
	},
	.normal_video = {
		.pclk = 58000000,
		.linelength = 720,
		.framelength = 2664,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000,
	},
	.hs_video = {
		.pclk = 58000000,
		.linelength = 720,
		.framelength = 2664,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000,


	},
	.slim_video = {
		.pclk = 58000000,
		.linelength = 720,
		.framelength = 2664,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000,


	},
	.margin = 5,			//sensor framelength & shutter margin
	.min_shutter = 2,		//min shutter
	.min_gain = 64,	  /*1x gain*/
	.max_gain = 1024, /*16x gain*/
	.min_gain_iso = 100,
	.gain_step = 2,
	.gain_type = 2,
	.max_frame_length = 0xffff,//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,	//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 1,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.temperature_support = 0,/* 1, support; 0,not support */
	.sensor_mode_num = 5,	  //support sensor mode num
	
	.cap_delay_frame = 2,		//enter capture delay frame num
	.pre_delay_frame = 2, 		//enter preview delay frame num
	.video_delay_frame = 2,		//enter video delay frame num
	.hs_video_delay_frame = 2,	//enter high speed video  delay frame num
	.slim_video_delay_frame = 2,//enter slim video delay frame num
	.frame_time_delay_frame = 2,
	
	.isp_driving_current = ISP_DRIVING_8MA, //mclk driving current
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
	.mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	.mipi_settle_delay_mode = 1,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,//sensor output first pixel color  SENSOR_OUTPUT_FORMAT_RAW_R SENSOR_OUTPUT_FORMAT_RAW_Gr SENSOR_OUTPUT_FORMAT_RAW_Gb 
	.mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num = SENSOR_MIPI_4_LANE,//mipi lane num
	.i2c_speed = 400,
	.i2c_addr_table = {0x20, 0x6c, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};


static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,//IMAGE_HV_MIRROR,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x3D0,					//current shutter
	.gain = 0x100,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x20,//record current sensor's i2c write id
	.current_ae_effective_frame = 2,
};


/* Sensor output window information */
#if 1
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =
{
 { 6528, 4896,	0,	0, 6528, 4896, 3264, 2448, 0, 0, 3264, 2448,	0,	0, 3264, 2448},  // 
 //{ 6528, 4896,	0, 	0, 6528, 4896, 1640, 1232, 0, 0, 1640, 1232,	2,	2, 1632,  1224}, // Preview 
 { 6528, 4896,	0,	0, 6528, 4896, 3264, 2448, 0, 0, 3264, 2448,	0,	0, 3264, 2448}, // capture 
 { 6528, 4896,	0,	0, 6528, 4896, 3264, 2448, 0, 0, 3264, 2448,	0,	0, 3264, 2448},  // video 
 { 6528, 4896,	0,	0, 6528, 4896, 3264, 2448, 0, 0, 3264, 2448,	0,	0, 3264, 2448}, //hight speed video 
 { 6528, 4896,	0,	0, 6528, 4896, 3264, 2448, 0, 0, 3264, 2448,	0,	0, 3264, 2448}};// slim video 
#else
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
/* Preview */
{ 3296, 2480, 0, 12, 3296, 2456, 1648, 1228, 8,	2, 1632, 1224, 0, 0, 1632, 1224},
/* capture */
{ 3296, 2480, 0, 12, 3296, 2456, 3296, 2456, 16, 4, 3264, 2448, 0, 0, 3264, 2448},
/* video*/
{ 3296, 2480, 0, 12, 3296, 2456, 3296, 2456, 16, 4, 3264, 2448, 0, 0, 3264, 2448},
/*hight speed video */
{ 3296, 2480, 336, 272, 2624, 1936, 656, 484, 8, 2, 640, 480, 0, 0,  640,  480},
/* slim video  */
{ 3296, 2480, 0, 12, 3296, 2456, 1648, 1228, 8,	2, 1632, 1224, 0, 0, 1632, 1224}
};
#endif
/*
#define OV32B40GMSMIPI_MaxGainIndex (97)
kal_uint16 OV32B40GMSMIPI_sensorGainMapping[OV32B40GMSMIPI_MaxGainIndex][2] ={
{ 64 ,0  },
{ 68 ,12 },
{ 71 ,23 },
{ 74 ,33 },
{ 77 ,42 },
{ 81 ,52 },
{ 84 ,59 },
{ 87 ,66 },
{ 90 ,73 },
{ 93 ,79 },
{ 96 ,85 },
{ 100,91 },
{ 103,96 },
{ 106,101},
{ 109,105},
{ 113,110},
{ 116,114},
{ 120,118},
{ 122,121},
{ 125,125},
{ 128,128},
{ 132,131},
{ 135,134},
{ 138,137},
{ 141,139},
{ 144,142},
{ 148,145},
{ 151,147},
{ 153,149},
{ 157,151},
{ 160,153},
{ 164,156},
{ 168,158},
{ 169,159},
{ 173,161},
{ 176,163},
{ 180,165},
{ 182,166},
{ 187,168},
{ 189,169},
{ 193,171},
{ 196,172},
{ 200,174},
{ 203,175},
{ 205,176},
{ 208,177},
{ 213,179},
{ 216,180},
{ 219,181},
{ 222,182},
{ 225,183},
{ 228,184},
{ 232,185},
{ 235,186},
{ 238,187},
{ 241,188},
{ 245,189},
{ 249,190},
{ 253,191},
{ 256,192},
{ 260,193},
{ 265,194},
{ 269,195},
{ 274,196},
{ 278,197},
{ 283,198},
{ 288,199},
{ 293,200},
{ 298,201},
{ 304,202},
{ 310,203},
{ 315,204},
{ 322,205},
{ 328,206},
{ 335,207},
{ 342,208},
{ 349,209},
{ 357,210},
{ 365,211},
{ 373,212},
{ 381,213},
{ 400,215},
{ 420,217},
{ 432,218},
{ 443,219},
{ 468,221},
{ 482,222},
{ 497,223},
{ 512,224},
{ 529,225},
{ 546,226},
{ 566,227},
{ 585,228},
{ 607,229},
{ 631,230},
{ 656,231},
{ 683,232}
};
*/

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);

	write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
	write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);	  
	write_cmos_sensor(0x380c, imgsensor.line_length >> 8);
	write_cmos_sensor(0x380d, imgsensor.line_length & 0xFF);
	
}	/*	set_dummy  */

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor(0x300a) << 8) | read_cmos_sensor(0x300b));
	//int sensorid;
	//sensorid =  ((read_cmos_sensor(0x0000) << 8) | read_cmos_sensor(0x0001));
	//LOG_INF("read sensor id:%x", sensorid);
	//return 0x0219;
}
static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable = %d\n", framerate,min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
		//imgsensor.dummy_line = 0;
	//else
		//imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */


#if 0
static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;
	//kal_uint32 frame_length = 0;
	   
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	
	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)		
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	if (shutter < imgsensor_info.min_shutter) shutter = imgsensor_info.min_shutter;

	if (imgsensor.autoflicker_en) { 
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
		else {
		// Extend frame length
		write_cmos_sensor(0x0160, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0161, imgsensor.frame_length & 0xFF);
		}
	} else {
		// Extend frame length
		write_cmos_sensor(0x0160, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0161, imgsensor.frame_length & 0xFF);
	}

	// Update Shutter
	write_cmos_sensor(0x015a, (shutter >> 8) & 0xFF);
	write_cmos_sensor(0x015b, (shutter ) & 0xFF);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

	//LOG_INF("frame_length = %d ", frame_length);

}	/*	write_shutter  */
#endif

/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
    unsigned long flags;
    kal_uint16 realtime_fps = 0;
    //kal_uint32 frame_length = 0;
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);


    spin_lock(&imgsensor_drv_lock);
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	
	if (imgsensor.autoflicker_en){ 
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146,0);
        else {
        // Extend frame length
        write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
        }
    } else {
        // Extend frame length
        write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
    }

    // Update Shutter
    write_cmos_sensor(0x3502, (shutter << 4) & 0xFF);
    write_cmos_sensor(0x3501, (shutter >> 4) & 0xFF);
    write_cmos_sensor(0x3500, (shutter >> 12) & 0x0F);
    LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}



static kal_uint16 gain2reg(const kal_uint16 gain)
{

	kal_uint16 reg_gain = 0x0000;
	kal_uint16 OV32B40GMS_GAIN_BASE = 128;


	reg_gain = gain*OV32B40GMS_GAIN_BASE/BASEGAIN;
	return (kal_uint16)reg_gain;

	
}

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT16 iPreGain = 0;
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;


	if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
		LOG_INF("Error gain setting");

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 16 * BASEGAIN)
			gain = 16 * BASEGAIN;		 
	}
 
	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain; 
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor(0x3508,(reg_gain>>8));
	write_cmos_sensor(0x3509,(reg_gain&0xff));
	
	return gain;
}	/*	set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
}
			


static void set_shutter_frame_length(
				kal_uint16 shutter, kal_uint16 frame_length,
				kal_bool auto_extend_en)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK
	 * to get exposure larger than frame exposure
	 */
	/* AE doesn't update sensor gain at capture mode,
	 * thus extra exposure lines must be updated here.
	 */

	/* OV Recommend Solution */
/* if shutter bigger than frame_length, should extend frame length first */

	spin_lock(&imgsensor_drv_lock);
	/* Change frame time */
	dummy_line = frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;
	imgsensor.min_frame_length = imgsensor.frame_length;

	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter)
		  ? imgsensor_info.min_shutter : shutter;

	shutter =
	  (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
	  ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk
		/ imgsensor.line_length * 10 / imgsensor.frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor(0x0104, 0x01);
			write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		  write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
			write_cmos_sensor(0x0104, 0x00);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor(0x0104, 0x01);
		write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x0104, 0x00);
	}

	/* Update Shutter */
	write_cmos_sensor(0x0104, 0x01);
	if (auto_extend_en)
		write_cmos_sensor(0x0350, 0x01); /* Enable auto extend */
	else
		write_cmos_sensor(0x0350, 0x00); /* Disable auto extend */
	//write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
	//write_cmos_sensor(0x0203, shutter  & 0xFF);
	
	  write_cmos_sensor(0x3502, (shutter << 4) & 0xFF);
    write_cmos_sensor(0x3501, (shutter >> 4) & 0xFF);
    write_cmos_sensor(0x3500, (shutter >> 12) & 0x0F);
	write_cmos_sensor(0x0104, 0x00);

	pr_info(
	"Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, auto_extend=%d\n",
		shutter, imgsensor.frame_length, frame_length,
		dummy_line, read_cmos_sensor(0x0350));

}	/* set_shutter_frame_length */


#if 1
static void set_mirror_flip(kal_uint8 image_mirror)
{
//	LOG_INF("image_mirror = %d\n", image_mirror);
	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/
	kal_uint8  iTemp; 
	LOG_INF("set_mirror_flip function\n");
    iTemp = read_cmos_sensor(0x0172) & 0x03;	//Clear the mirror and flip bits.
    switch (image_mirror)
    {
        case IMAGE_NORMAL:
            write_cmos_sensor(0x0172, iTemp | 0x03);	//Set normal
            break;
        case IMAGE_V_MIRROR:
            write_cmos_sensor(0x0172, iTemp | 0x01);	//Set flip
            break;
        case IMAGE_H_MIRROR:
            write_cmos_sensor(0x0172, iTemp | 0x02);	//Set mirror
            break;
        case IMAGE_HV_MIRROR:
            write_cmos_sensor(0x0172, iTemp);	//Set mirror and flip
            break;
    }
	LOG_INF("Error image_mirror setting\n");

}
#endif
/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}	/*	night_mode	*/

static void sensor_init(void)
{
	LOG_INF("E\n");
	
	write_cmos_sensor(0x0103,0x01);
	write_cmos_sensor(0x5000,0x5d);
	write_cmos_sensor(0x5001,0x20);
	write_cmos_sensor(0x5002,0x37);
	write_cmos_sensor(0x5003,0x00);
	write_cmos_sensor(0x5004,0xc0);
	write_cmos_sensor(0x5005,0x42);
	write_cmos_sensor(0x5006,0x04);
	write_cmos_sensor(0x5015,0x20);
	write_cmos_sensor(0x5053,0x04);
	write_cmos_sensor(0x5061,0x50);
	write_cmos_sensor(0x5068,0x03);
	write_cmos_sensor(0x506c,0x04);
	write_cmos_sensor(0x506d,0x0c);
	write_cmos_sensor(0x506e,0x0c);
	write_cmos_sensor(0x50c1,0x00);
	write_cmos_sensor(0x5115,0x0b);
	write_cmos_sensor(0x511c,0x01);
	write_cmos_sensor(0x511d,0x01);
	write_cmos_sensor(0x511e,0x01);
	write_cmos_sensor(0x511f,0x01);
	write_cmos_sensor(0x5180,0xc1);
	write_cmos_sensor(0x518a,0x04);
	write_cmos_sensor(0x51b0,0x30);
	write_cmos_sensor(0x51d0,0xc2);
	write_cmos_sensor(0x51d1,0x68);
	write_cmos_sensor(0x51d8,0x08);
	write_cmos_sensor(0x51d9,0x10);
	write_cmos_sensor(0x51da,0x02);
	write_cmos_sensor(0x51db,0x02);
	write_cmos_sensor(0x51dc,0x06);
	write_cmos_sensor(0x51dd,0x06);
	write_cmos_sensor(0x51de,0x02);
	write_cmos_sensor(0x51df,0x06);
	write_cmos_sensor(0x51e0,0x0a);
	write_cmos_sensor(0x51e1,0x0e);
	write_cmos_sensor(0x51e2,0x00);
	write_cmos_sensor(0x51e3,0x00);
	write_cmos_sensor(0x5200,0x12);
	write_cmos_sensor(0x5201,0x20);
	write_cmos_sensor(0x5202,0x0d);
	write_cmos_sensor(0x5203,0xa0);
	write_cmos_sensor(0x5205,0x10);
	write_cmos_sensor(0x5207,0x10);
	write_cmos_sensor(0x5208,0x12);
	write_cmos_sensor(0x5209,0x00);
	write_cmos_sensor(0x520a,0x0d);
	write_cmos_sensor(0x520b,0x80);
	write_cmos_sensor(0x520d,0x08);
	write_cmos_sensor(0x5250,0x14);
	write_cmos_sensor(0x5331,0x06);
	write_cmos_sensor(0x5332,0x43);
	write_cmos_sensor(0x5333,0x45);
	write_cmos_sensor(0x53c1,0x00);
	write_cmos_sensor(0x5415,0x0b);
	write_cmos_sensor(0x541c,0x01);
	write_cmos_sensor(0x541d,0x01);
	write_cmos_sensor(0x541e,0x01);
	write_cmos_sensor(0x541f,0x01);
	write_cmos_sensor(0x5480,0xc1);
	write_cmos_sensor(0x548a,0x04);
	write_cmos_sensor(0x54b0,0x30);
	write_cmos_sensor(0x54d0,0xc2);
	write_cmos_sensor(0x54d1,0x68);
	write_cmos_sensor(0x54d8,0x08);
	write_cmos_sensor(0x54d9,0x10);
	write_cmos_sensor(0x54da,0x02);
	write_cmos_sensor(0x54db,0x02);
	write_cmos_sensor(0x54dc,0x06);
	write_cmos_sensor(0x54dd,0x06);
	write_cmos_sensor(0x54de,0x02);
	write_cmos_sensor(0x54df,0x06);
	write_cmos_sensor(0x54e0,0x0a);
	write_cmos_sensor(0x54e1,0x0e);
	write_cmos_sensor(0x54e2,0x00);
	write_cmos_sensor(0x54e3,0x00);
	write_cmos_sensor(0x5500,0x12);
	write_cmos_sensor(0x5501,0x20);
	write_cmos_sensor(0x5502,0x0d);
	write_cmos_sensor(0x5503,0xa0);
	write_cmos_sensor(0x5505,0x10);
	write_cmos_sensor(0x5507,0x10);
	write_cmos_sensor(0x5508,0x12);
	write_cmos_sensor(0x5509,0x00);
	write_cmos_sensor(0x550a,0x0d);
	write_cmos_sensor(0x550b,0x80);
	write_cmos_sensor(0x550d,0x08);
	write_cmos_sensor(0x5550,0x14);
	write_cmos_sensor(0x5631,0x06);
	write_cmos_sensor(0x5632,0x43);
	write_cmos_sensor(0x5633,0x45);
	write_cmos_sensor(0x56c1,0x00);
	write_cmos_sensor(0x5715,0x0b);
	write_cmos_sensor(0x571c,0x01);
	write_cmos_sensor(0x571d,0x01);
	write_cmos_sensor(0x571e,0x01);
	write_cmos_sensor(0x571f,0x01);
	write_cmos_sensor(0x5780,0xc1);
	write_cmos_sensor(0x578a,0x04);
	write_cmos_sensor(0x57b0,0x30);
	write_cmos_sensor(0x57d0,0xc2);
	write_cmos_sensor(0x57d1,0x68);
	write_cmos_sensor(0x57d8,0x08);
	write_cmos_sensor(0x57d9,0x10);
	write_cmos_sensor(0x57da,0x02);
	write_cmos_sensor(0x57db,0x02);
	write_cmos_sensor(0x57dc,0x06);
	write_cmos_sensor(0x57dd,0x06);
	write_cmos_sensor(0x57de,0x02);
	write_cmos_sensor(0x57df,0x06);
	write_cmos_sensor(0x57e0,0x0a);
	write_cmos_sensor(0x57e1,0x0e);
	write_cmos_sensor(0x57e2,0x00);
	write_cmos_sensor(0x57e3,0x00);
	write_cmos_sensor(0x5800,0x12);
	write_cmos_sensor(0x5801,0x20);
	write_cmos_sensor(0x5802,0x0d);
	write_cmos_sensor(0x5803,0xa0);
	write_cmos_sensor(0x5805,0x10);
	write_cmos_sensor(0x5807,0x10);
	write_cmos_sensor(0x5808,0x12);
	write_cmos_sensor(0x5809,0x00);
	write_cmos_sensor(0x580a,0x0d);
	write_cmos_sensor(0x580b,0x80);
	write_cmos_sensor(0x580d,0x08);
	write_cmos_sensor(0x5850,0x14);
	write_cmos_sensor(0x5931,0x06);
	write_cmos_sensor(0x5932,0x43);
	write_cmos_sensor(0x5933,0x45);
	write_cmos_sensor(0x598a,0x00);
	write_cmos_sensor(0x598b,0x08);
	write_cmos_sensor(0x598c,0x00);
	write_cmos_sensor(0x598d,0x04);
	write_cmos_sensor(0x598e,0x00);
	write_cmos_sensor(0x598f,0x02);
	write_cmos_sensor(0x5990,0x00);
	write_cmos_sensor(0x5991,0x00);
	write_cmos_sensor(0x5992,0x00);
	write_cmos_sensor(0x5993,0x02);
	write_cmos_sensor(0x5994,0x04);
	write_cmos_sensor(0x5995,0x06);
	write_cmos_sensor(0x5996,0x08);
	write_cmos_sensor(0x5997,0x0c);
	write_cmos_sensor(0x5998,0x0f);
	write_cmos_sensor(0x5999,0x12);
	write_cmos_sensor(0x599a,0x16);
	write_cmos_sensor(0x599b,0x1a);
	write_cmos_sensor(0x599c,0x1e);
	write_cmos_sensor(0x599d,0x22);
	write_cmos_sensor(0x599e,0x27);
	write_cmos_sensor(0x599f,0x2c);
	write_cmos_sensor(0x59c4,0x08);
	write_cmos_sensor(0x59c5,0x08);
	write_cmos_sensor(0x59c6,0x0a);
	write_cmos_sensor(0x59c7,0x0c);
	write_cmos_sensor(0x59c8,0x0c);
	write_cmos_sensor(0x59c9,0x0e);
	write_cmos_sensor(0x59ca,0x0e);
	write_cmos_sensor(0x59cb,0x00);
	write_cmos_sensor(0x59fd,0xf1);
	write_cmos_sensor(0x5a12,0x0f);
	write_cmos_sensor(0x5a1a,0x19);
	write_cmos_sensor(0x5a1b,0x80);
	write_cmos_sensor(0x5a1c,0x13);
	write_cmos_sensor(0x5a1d,0x20);
	write_cmos_sensor(0x6525,0x00);
	write_cmos_sensor(0x5f80,0x40);
	write_cmos_sensor(0x5f81,0x40);
	write_cmos_sensor(0x5f82,0x40);
	write_cmos_sensor(0x5f83,0x40);
	write_cmos_sensor(0x5f84,0x40);
	write_cmos_sensor(0x5f85,0x40);
	write_cmos_sensor(0x5f86,0x40);
	write_cmos_sensor(0x5f87,0x40);
	write_cmos_sensor(0x5f88,0x40);
	write_cmos_sensor(0x5f89,0x40);
	write_cmos_sensor(0x5f8a,0x40);
	write_cmos_sensor(0x5f8b,0x40);
	write_cmos_sensor(0x5f8c,0x40);
	write_cmos_sensor(0x5f8d,0x40);
	write_cmos_sensor(0x5f8e,0x40);
	write_cmos_sensor(0x5f8f,0x40);
	write_cmos_sensor(0x5f90,0x40);
	write_cmos_sensor(0x5f91,0x40);
	write_cmos_sensor(0x5f92,0x40);
	write_cmos_sensor(0x5f93,0x40);
	write_cmos_sensor(0x5f94,0x40);
	write_cmos_sensor(0x5f95,0x40);
	write_cmos_sensor(0x5f96,0x40);
	write_cmos_sensor(0x5f97,0x40);
	write_cmos_sensor(0x5f98,0x40);
	write_cmos_sensor(0x5f99,0x40);
	write_cmos_sensor(0x5f9a,0x40);
	write_cmos_sensor(0x5f9b,0x40);
	write_cmos_sensor(0x5f9c,0x40);
	write_cmos_sensor(0x5f9d,0x40);
	write_cmos_sensor(0x5f9e,0x40);
	write_cmos_sensor(0x5f9f,0x40);
	write_cmos_sensor(0x5fa0,0x40);
	write_cmos_sensor(0x5fa1,0x40);
	write_cmos_sensor(0x5fa2,0x40);
	write_cmos_sensor(0x5fa3,0x40);
	write_cmos_sensor(0x5fa4,0x40);
	write_cmos_sensor(0x5fa5,0x40);
	write_cmos_sensor(0x5fa6,0x40);
	write_cmos_sensor(0x5fa7,0x40);
	write_cmos_sensor(0x5fa8,0x40);
	write_cmos_sensor(0x5fa9,0x40);
	write_cmos_sensor(0x5faa,0x40);
	write_cmos_sensor(0x5fab,0x40);
	write_cmos_sensor(0x5fac,0x40);
	write_cmos_sensor(0x5fad,0x40);
	write_cmos_sensor(0x5fae,0x40);
	write_cmos_sensor(0x5faf,0x40);
	write_cmos_sensor(0x5fb0,0x40);
	write_cmos_sensor(0x5fb1,0x40);
	write_cmos_sensor(0x5fb2,0x40);
	write_cmos_sensor(0x5fb3,0x40);
	write_cmos_sensor(0x5fb4,0x40);
	write_cmos_sensor(0x5fb5,0x40);
	write_cmos_sensor(0x5fb6,0x40);
	write_cmos_sensor(0x5fb7,0x40);
	write_cmos_sensor(0x5fb8,0x40);
	write_cmos_sensor(0x5fb9,0x40);
	write_cmos_sensor(0x5fba,0x40);
	write_cmos_sensor(0x5fbb,0x40);
	write_cmos_sensor(0x5fbc,0x40);
	write_cmos_sensor(0x5fbd,0x40);
	write_cmos_sensor(0x5fbe,0x40);
	write_cmos_sensor(0x5fbf,0x40);
	write_cmos_sensor(0x5fc0,0x40);
	write_cmos_sensor(0x5fc1,0x40);
	write_cmos_sensor(0x5fc2,0x40);
	write_cmos_sensor(0x5fc3,0x40);
	write_cmos_sensor(0x5fc4,0x40);
	write_cmos_sensor(0x5fc5,0x40);
	write_cmos_sensor(0x5fc6,0x40);
	write_cmos_sensor(0x5fc7,0x40);
	write_cmos_sensor(0x5fc8,0x40);
	write_cmos_sensor(0x5fc9,0x40);
	write_cmos_sensor(0x5fca,0x40);
	write_cmos_sensor(0x5fcb,0x40);
	write_cmos_sensor(0x5fcc,0x40);
	write_cmos_sensor(0x5fcd,0x40);
	write_cmos_sensor(0x5fce,0x40);
	write_cmos_sensor(0x5fcf,0x40);
	write_cmos_sensor(0x5fd0,0x40);
	write_cmos_sensor(0x5fd1,0x40);
	write_cmos_sensor(0x5fd2,0x40);
	write_cmos_sensor(0x5fd3,0x40);
	write_cmos_sensor(0x5fd4,0x40);
	write_cmos_sensor(0x5fd5,0x40);
	write_cmos_sensor(0x5fd6,0x40);
	write_cmos_sensor(0x5fd7,0x40);
	write_cmos_sensor(0x5fd8,0x40);
	write_cmos_sensor(0x5fd9,0x40);
	write_cmos_sensor(0x5fda,0x40);
	write_cmos_sensor(0x5fdb,0x40);
	write_cmos_sensor(0x5fdc,0x40);
	write_cmos_sensor(0x5fdd,0x40);
	write_cmos_sensor(0x5fde,0x40);
	write_cmos_sensor(0x5fdf,0x40);
	write_cmos_sensor(0x5fe0,0x40);
	write_cmos_sensor(0x5fe1,0x40);
	write_cmos_sensor(0x5fe2,0x40);
	write_cmos_sensor(0x5fe3,0x40);
	write_cmos_sensor(0x5fe4,0x40);
	write_cmos_sensor(0x5fe5,0x40);
	write_cmos_sensor(0x5fe6,0x40);
	write_cmos_sensor(0x5fe7,0x40);
	write_cmos_sensor(0x5fe8,0x40);
	write_cmos_sensor(0x5fe9,0x40);
	write_cmos_sensor(0x5fea,0x40);
	write_cmos_sensor(0x5feb,0x40);
	write_cmos_sensor(0x5fec,0x40);
	write_cmos_sensor(0x5fed,0x40);
	write_cmos_sensor(0x5fee,0x40);
	write_cmos_sensor(0x5fef,0x40);
	write_cmos_sensor(0x5ff0,0x40);
	write_cmos_sensor(0x5ff1,0x40);
	write_cmos_sensor(0x5ff2,0x40);
	write_cmos_sensor(0x5ff3,0x40);
	write_cmos_sensor(0x5ff4,0x40);
	write_cmos_sensor(0x5ff5,0x40);
	write_cmos_sensor(0x5ff6,0x40);
	write_cmos_sensor(0x5ff7,0x40);
	write_cmos_sensor(0x5ff8,0x40);
	write_cmos_sensor(0x5ff9,0x40);
	write_cmos_sensor(0x5ffa,0x40);
	write_cmos_sensor(0x5ffb,0x40);
	write_cmos_sensor(0x5ffc,0x40);
	write_cmos_sensor(0x5ffd,0x40);
	write_cmos_sensor(0x5ffe,0x40);
	write_cmos_sensor(0x5fff,0x40);
	write_cmos_sensor(0x6000,0x40);
	write_cmos_sensor(0x6001,0x40);
	write_cmos_sensor(0x6002,0x40);
	write_cmos_sensor(0x6003,0x40);
	write_cmos_sensor(0x6004,0x40);
	write_cmos_sensor(0x6005,0x40);
	write_cmos_sensor(0x6006,0x40);
	write_cmos_sensor(0x6007,0x40);
	write_cmos_sensor(0x6008,0x40);
	write_cmos_sensor(0x6009,0x40);
	write_cmos_sensor(0x600a,0x40);
	write_cmos_sensor(0x600b,0x40);
	write_cmos_sensor(0x600c,0x40);
	write_cmos_sensor(0x600d,0x40);
	write_cmos_sensor(0x600e,0x40);
	write_cmos_sensor(0x600f,0x40);
	write_cmos_sensor(0x6010,0x40);
	write_cmos_sensor(0x6011,0x40);
	write_cmos_sensor(0x6012,0x40);
	write_cmos_sensor(0x6013,0x40);
	write_cmos_sensor(0x6014,0x40);
	write_cmos_sensor(0x6015,0x40);
	write_cmos_sensor(0x6016,0x40);
	write_cmos_sensor(0x6017,0x40);
	write_cmos_sensor(0x6018,0x40);
	write_cmos_sensor(0x6019,0x40);
	write_cmos_sensor(0x601a,0x40);
	write_cmos_sensor(0x601b,0x40);
	write_cmos_sensor(0x601c,0x40);
	write_cmos_sensor(0x601d,0x40);
	write_cmos_sensor(0x601e,0x40);
	write_cmos_sensor(0x601f,0x40);
	write_cmos_sensor(0x6020,0x40);
	write_cmos_sensor(0x6021,0x40);
	write_cmos_sensor(0x6022,0x40);
	write_cmos_sensor(0x6023,0x40);
	write_cmos_sensor(0x6024,0x40);
	write_cmos_sensor(0x6025,0x40);
	write_cmos_sensor(0x6026,0x40);
	write_cmos_sensor(0x6027,0x40);
	write_cmos_sensor(0x6028,0x40);
	write_cmos_sensor(0x6029,0x40);
	write_cmos_sensor(0x602a,0x40);
	write_cmos_sensor(0x602b,0x40);
	write_cmos_sensor(0x602c,0x40);
	write_cmos_sensor(0x602d,0x40);
	write_cmos_sensor(0x602e,0x40);
	write_cmos_sensor(0x602f,0x40);
	write_cmos_sensor(0x6030,0x40);
	write_cmos_sensor(0x6031,0x40);
	write_cmos_sensor(0x6032,0x40);
	write_cmos_sensor(0x6033,0x40);
	write_cmos_sensor(0x6034,0xcd);
	write_cmos_sensor(0x6035,0xcd);
	write_cmos_sensor(0x6036,0xcd);
	write_cmos_sensor(0x6037,0xcd);
	write_cmos_sensor(0x6038,0xcd);
	write_cmos_sensor(0x6039,0xcd);
	write_cmos_sensor(0x603a,0xcd);
	write_cmos_sensor(0x603b,0xcd);
	write_cmos_sensor(0x603c,0xcd);
	write_cmos_sensor(0x603d,0xcd);
	write_cmos_sensor(0x603e,0xcd);
	write_cmos_sensor(0x603f,0xcd);
	write_cmos_sensor(0x6040,0xcd);
	write_cmos_sensor(0x6041,0xcd);
	write_cmos_sensor(0x6042,0xcd);
	write_cmos_sensor(0x6043,0xcd);
	write_cmos_sensor(0x6044,0xcd);
	write_cmos_sensor(0x6045,0xcd);
	write_cmos_sensor(0x6046,0xcd);
	write_cmos_sensor(0x6047,0xcd);
	write_cmos_sensor(0x6048,0xcd);
	write_cmos_sensor(0x6049,0xcd);
	write_cmos_sensor(0x604a,0xcd);
	write_cmos_sensor(0x604b,0xcd);
	write_cmos_sensor(0x604c,0xcd);
	write_cmos_sensor(0x604d,0xcd);
	write_cmos_sensor(0x604e,0xcd);
	write_cmos_sensor(0x604f,0xcd);
	write_cmos_sensor(0x6050,0xcd);
	write_cmos_sensor(0x6051,0xcd);
	write_cmos_sensor(0x6052,0xcd);
	write_cmos_sensor(0x6053,0xcd);
	write_cmos_sensor(0x6054,0xcd);
	write_cmos_sensor(0x6055,0xcd);
	write_cmos_sensor(0x6056,0xcd);
	write_cmos_sensor(0x6057,0xcd);
	write_cmos_sensor(0x6058,0xcd);
	write_cmos_sensor(0x6059,0xcd);
	write_cmos_sensor(0x605a,0xcd);
	write_cmos_sensor(0x605b,0xcd);
	write_cmos_sensor(0x605c,0xcd);
	write_cmos_sensor(0x605d,0xcd);
	write_cmos_sensor(0x605e,0xcd);
	write_cmos_sensor(0x605f,0xcd);
	write_cmos_sensor(0x6060,0xcd);
	write_cmos_sensor(0x6061,0xcd);
	write_cmos_sensor(0x6062,0xcd);
	write_cmos_sensor(0x6063,0xcd);
	write_cmos_sensor(0x6064,0xcd);
	write_cmos_sensor(0x6065,0xcd);
	write_cmos_sensor(0x6066,0xcd);
	write_cmos_sensor(0x6067,0xcd);
	write_cmos_sensor(0x6068,0xcd);
	write_cmos_sensor(0x6069,0xcd);
	write_cmos_sensor(0x606a,0xcd);
	write_cmos_sensor(0x606b,0xcd);
	write_cmos_sensor(0x600c,0xcd);
	write_cmos_sensor(0x606d,0xcd);
	write_cmos_sensor(0x606e,0xcd);
	write_cmos_sensor(0x606f,0xcd);
	write_cmos_sensor(0x6070,0xcd);
	write_cmos_sensor(0x6071,0xcd);
	write_cmos_sensor(0x6072,0xcd);
	write_cmos_sensor(0x6073,0xcd);
	write_cmos_sensor(0x6074,0xcd);
	write_cmos_sensor(0x6075,0xcd);
	write_cmos_sensor(0x6076,0xcd);
	write_cmos_sensor(0x6077,0xcd);
	write_cmos_sensor(0x6078,0xcd);
	write_cmos_sensor(0x6079,0xcd);
	write_cmos_sensor(0x607a,0xcd);
	write_cmos_sensor(0x607b,0xcd);
	write_cmos_sensor(0x607c,0xcd);
	write_cmos_sensor(0x607d,0xcd);
	write_cmos_sensor(0x607e,0xcd);
	write_cmos_sensor(0x607f,0xcd);
	write_cmos_sensor(0x6080,0xcd);
	write_cmos_sensor(0x6081,0xcd);
	write_cmos_sensor(0x6082,0xcd);
	write_cmos_sensor(0x6083,0xcd);
	write_cmos_sensor(0x6084,0xcd);
	write_cmos_sensor(0x6085,0xcd);
	write_cmos_sensor(0x6086,0xcd);
	write_cmos_sensor(0x6087,0xcd);
	write_cmos_sensor(0x6088,0xcd);
	write_cmos_sensor(0x6089,0xcd);
	write_cmos_sensor(0x608a,0xcd);
	write_cmos_sensor(0x608b,0xcd);
	write_cmos_sensor(0x608c,0xcd);
	write_cmos_sensor(0x608d,0xcd);
	write_cmos_sensor(0x608e,0xcd);
	write_cmos_sensor(0x608f,0xcd);
	write_cmos_sensor(0x6090,0xcd);
	write_cmos_sensor(0x6091,0xcd);
	write_cmos_sensor(0x6092,0xcd);
	write_cmos_sensor(0x6093,0xcd);
	write_cmos_sensor(0x6094,0xcd);
	write_cmos_sensor(0x6095,0xcd);
	write_cmos_sensor(0x6096,0xcd);
	write_cmos_sensor(0x6097,0xcd);
	write_cmos_sensor(0x6098,0xcd);
	write_cmos_sensor(0x6099,0xcd);
	write_cmos_sensor(0x609a,0xcd);
	write_cmos_sensor(0x609b,0xcd);
	write_cmos_sensor(0x609c,0xcd);
	write_cmos_sensor(0x609d,0xcd);
	write_cmos_sensor(0x609e,0xcd);
	write_cmos_sensor(0x609f,0xcd);
	write_cmos_sensor(0x60a0,0xcd);
	write_cmos_sensor(0x60a1,0xcd);
	write_cmos_sensor(0x60a2,0xcd);
	write_cmos_sensor(0x60a3,0xcd);
	write_cmos_sensor(0x60a4,0xcd);
	write_cmos_sensor(0x60a5,0xcd);
	write_cmos_sensor(0x60a6,0xcd);
	write_cmos_sensor(0x60a7,0xcd);
	write_cmos_sensor(0x60a8,0xcd);
	write_cmos_sensor(0x60a9,0xcd);
	write_cmos_sensor(0x60aa,0xcd);
	write_cmos_sensor(0x60ab,0xcd);
	write_cmos_sensor(0x60ac,0xcd);
	write_cmos_sensor(0x60ad,0xcd);
	write_cmos_sensor(0x60ae,0xcd);
	write_cmos_sensor(0x60af,0xcd);
	write_cmos_sensor(0x60b0,0xcd);
	write_cmos_sensor(0x60b1,0xcd);
	write_cmos_sensor(0x60b2,0xcd);
	write_cmos_sensor(0x60b3,0xcd);
	write_cmos_sensor(0x60b4,0xcd);
	write_cmos_sensor(0x60b5,0xcd);
	write_cmos_sensor(0x60b6,0xcd);
	write_cmos_sensor(0x60b7,0xcd);
	write_cmos_sensor(0x60b8,0xcd);
	write_cmos_sensor(0x60b9,0xcd);
	write_cmos_sensor(0x60ba,0xcd);
	write_cmos_sensor(0x60bb,0xcd);
	write_cmos_sensor(0x60bc,0xcd);
	write_cmos_sensor(0x60bd,0xcd);
	write_cmos_sensor(0x60be,0xcd);
	write_cmos_sensor(0x60bf,0xcd);
	write_cmos_sensor(0x60c0,0xcd);
	write_cmos_sensor(0x60c1,0xcd);
	write_cmos_sensor(0x60c2,0xcd);
	write_cmos_sensor(0x60c3,0xcd);
	write_cmos_sensor(0x60c4,0xcd);
	write_cmos_sensor(0x60c5,0xcd);
	write_cmos_sensor(0x60c6,0xcd);
	write_cmos_sensor(0x60c7,0xcd);
	write_cmos_sensor(0x60c8,0xcd);
	write_cmos_sensor(0x60c9,0xcd);
	write_cmos_sensor(0x60ca,0xcd);
	write_cmos_sensor(0x60cb,0xcd);
	write_cmos_sensor(0x60cc,0xcd);
	write_cmos_sensor(0x60cd,0xcd);
	write_cmos_sensor(0x60ce,0xcd);
	write_cmos_sensor(0x60cf,0xcd);
	write_cmos_sensor(0x60d0,0xcd);
	write_cmos_sensor(0x60d1,0xcd);
	write_cmos_sensor(0x60d2,0xcd);
	write_cmos_sensor(0x60d3,0xcd);
	write_cmos_sensor(0x60d4,0xcd);
	write_cmos_sensor(0x60d5,0xcd);
	write_cmos_sensor(0x60d6,0xcd);
	write_cmos_sensor(0x60d7,0xcd);
	write_cmos_sensor(0x60d8,0xcd);
	write_cmos_sensor(0x60d9,0xcd);
	write_cmos_sensor(0x60da,0xcd);
	write_cmos_sensor(0x60db,0xcd);
	write_cmos_sensor(0x60dc,0xcd);
	write_cmos_sensor(0x60dd,0xcd);
	write_cmos_sensor(0x60de,0xcd);
	write_cmos_sensor(0x60df,0xcd);
	write_cmos_sensor(0x60e0,0xcd);
	write_cmos_sensor(0x60e1,0xcd);
	write_cmos_sensor(0x60e2,0xcd);
	write_cmos_sensor(0x60e3,0xcd);
	write_cmos_sensor(0x60e4,0xcd);
	write_cmos_sensor(0x60e5,0xcd);
	write_cmos_sensor(0x60e6,0xcd);
	write_cmos_sensor(0x60e7,0xcd);
	write_cmos_sensor(0x60e8,0xcd);
	write_cmos_sensor(0x60e9,0xcd);
	write_cmos_sensor(0x60ea,0xcd);
	write_cmos_sensor(0x60eb,0xcd);
	write_cmos_sensor(0x60ec,0xcd);
	write_cmos_sensor(0x60ed,0xcd);
	write_cmos_sensor(0x60ee,0xcd);
	write_cmos_sensor(0x60ef,0xcd);
	write_cmos_sensor(0x60f0,0xcd);
	write_cmos_sensor(0x60f1,0xcd);
	write_cmos_sensor(0x60f2,0xcd);
	write_cmos_sensor(0x60f3,0xcd);
	write_cmos_sensor(0x60f4,0xcd);
	write_cmos_sensor(0x60f5,0xcd);
	write_cmos_sensor(0x60f6,0xcd);
	write_cmos_sensor(0x60f7,0xcd);
	write_cmos_sensor(0x60f8,0xcd);
	write_cmos_sensor(0x60f9,0xcd);
	write_cmos_sensor(0x60fa,0xcd);
	write_cmos_sensor(0x60fb,0xcd);
	write_cmos_sensor(0x60fc,0xcd);
	write_cmos_sensor(0x60fd,0xcd);
	write_cmos_sensor(0x60fe,0xcd);
	write_cmos_sensor(0x60ff,0xcd);
	write_cmos_sensor(0x6100,0xcd);
	write_cmos_sensor(0x6101,0xcd);
	write_cmos_sensor(0x6102,0xcd);
	write_cmos_sensor(0x6103,0xcd);
	write_cmos_sensor(0x6104,0xcd);
	write_cmos_sensor(0x6105,0xcd);
	write_cmos_sensor(0x6106,0xcd);
	write_cmos_sensor(0x6107,0xcd);
	write_cmos_sensor(0x6108,0xcd);
	write_cmos_sensor(0x6109,0xcd);
	write_cmos_sensor(0x610a,0xcd);
	write_cmos_sensor(0x610b,0xcd);
	write_cmos_sensor(0x610c,0xcd);
	write_cmos_sensor(0x610d,0xcd);
	write_cmos_sensor(0x610e,0xcd);
	write_cmos_sensor(0x610f,0xcd);
	write_cmos_sensor(0x6110,0xcd);
	write_cmos_sensor(0x6111,0xcd);
	write_cmos_sensor(0x6112,0xcd);
	write_cmos_sensor(0x6113,0xcd);
	write_cmos_sensor(0x6114,0xcd);
	write_cmos_sensor(0x6115,0xcd);
	write_cmos_sensor(0x6116,0xcd);
	write_cmos_sensor(0x6117,0xcd);
	write_cmos_sensor(0x6118,0xcd);
	write_cmos_sensor(0x6119,0xcd);
	write_cmos_sensor(0x611a,0xcd);
	write_cmos_sensor(0x611b,0xcd);
	write_cmos_sensor(0x611c,0xcd);
	write_cmos_sensor(0x611d,0xcd);
	write_cmos_sensor(0x611e,0xcd);
	write_cmos_sensor(0x611f,0xcd);
	write_cmos_sensor(0x6120,0xcd);
	write_cmos_sensor(0x6121,0xcd);
	write_cmos_sensor(0x6122,0xcd);
	write_cmos_sensor(0x6123,0xcd);
	write_cmos_sensor(0x6124,0xcd);
	write_cmos_sensor(0x6125,0xcd);
	write_cmos_sensor(0x6126,0xcd);
	write_cmos_sensor(0x6127,0xcd);
	write_cmos_sensor(0x6128,0xcd);
	write_cmos_sensor(0x6129,0xcd);
	write_cmos_sensor(0x612a,0xcd);
	write_cmos_sensor(0x612b,0xcd);
	write_cmos_sensor(0x612c,0xcd);
	write_cmos_sensor(0x612d,0xcd);
	write_cmos_sensor(0x612e,0xcd);
	write_cmos_sensor(0x612f,0xcd);
	write_cmos_sensor(0x6130,0xcd);
	write_cmos_sensor(0x6131,0xcd);
	write_cmos_sensor(0x6132,0xcd);
	write_cmos_sensor(0x6133,0xcd);
	write_cmos_sensor(0x6134,0xcd);
	write_cmos_sensor(0x6135,0xcd);
	write_cmos_sensor(0x6136,0xcd);
	write_cmos_sensor(0x6137,0xcd);
	write_cmos_sensor(0x6138,0xcd);
	write_cmos_sensor(0x6139,0xcd);
	write_cmos_sensor(0x613a,0xcd);
	write_cmos_sensor(0x613b,0xcd);
	write_cmos_sensor(0x613c,0xcd);
	write_cmos_sensor(0x613d,0xcd);
	write_cmos_sensor(0x613e,0xcd);
	write_cmos_sensor(0x613f,0xcd);
	write_cmos_sensor(0x6140,0xcd);
	write_cmos_sensor(0x6141,0xcd);
	write_cmos_sensor(0x6142,0xcd);
	write_cmos_sensor(0x6143,0xcd);
	write_cmos_sensor(0x6144,0xcd);
	write_cmos_sensor(0x6145,0xcd);
	write_cmos_sensor(0x6146,0xcd);
	write_cmos_sensor(0x6147,0xcd);
	write_cmos_sensor(0x6148,0xcd);
	write_cmos_sensor(0x6149,0xcd);
	write_cmos_sensor(0x614a,0xcd);
	write_cmos_sensor(0x614b,0xcd);
	write_cmos_sensor(0x614c,0xcd);
	write_cmos_sensor(0x614d,0xcd);
	write_cmos_sensor(0x614e,0xcd);
	write_cmos_sensor(0x614f,0xcd);
	write_cmos_sensor(0x6150,0xcd);
	write_cmos_sensor(0x6151,0xcd);
	write_cmos_sensor(0x6152,0xcd);
	write_cmos_sensor(0x6153,0xcd);
	write_cmos_sensor(0x6154,0xcd);
	write_cmos_sensor(0x6155,0xcd);
	write_cmos_sensor(0x6156,0xcd);
	write_cmos_sensor(0x6157,0xcd);
	write_cmos_sensor(0x6158,0xcd);
	write_cmos_sensor(0x6159,0xcd);
	write_cmos_sensor(0x615a,0xcd);
	write_cmos_sensor(0x615b,0xcd);
	write_cmos_sensor(0x615c,0xcd);
	write_cmos_sensor(0x615d,0xcd);
	write_cmos_sensor(0x615e,0xcd);
	write_cmos_sensor(0x615f,0xcd);
	write_cmos_sensor(0x6160,0xcd);
	write_cmos_sensor(0x6161,0xcd);
	write_cmos_sensor(0x6162,0xcd);
	write_cmos_sensor(0x6163,0xcd);
	write_cmos_sensor(0x6164,0xcd);
	write_cmos_sensor(0x6165,0xcd);
	write_cmos_sensor(0x6166,0xcd);
	write_cmos_sensor(0x6167,0xcd);
	write_cmos_sensor(0x6168,0xcd);
	write_cmos_sensor(0x6169,0xcd);
	write_cmos_sensor(0x616a,0xcd);
	write_cmos_sensor(0x616b,0xcd);
	write_cmos_sensor(0x610c,0xcd);
	write_cmos_sensor(0x616d,0xcd);
	write_cmos_sensor(0x616e,0xcd);
	write_cmos_sensor(0x616f,0xcd);
	write_cmos_sensor(0x6170,0xcd);
	write_cmos_sensor(0x6171,0xcd);
	write_cmos_sensor(0x6172,0xcd);
	write_cmos_sensor(0x6173,0xcd);
	write_cmos_sensor(0x6174,0xcd);
	write_cmos_sensor(0x6175,0xcd);
	write_cmos_sensor(0x6176,0xcd);
	write_cmos_sensor(0x6177,0xcd);
	write_cmos_sensor(0x6178,0xcd);
	write_cmos_sensor(0x6179,0xcd);
	write_cmos_sensor(0x617a,0xcd);
	write_cmos_sensor(0x617b,0xcd);
	write_cmos_sensor(0x617c,0xcd);
	write_cmos_sensor(0x617d,0xcd);
	write_cmos_sensor(0x617e,0xcd);
	write_cmos_sensor(0x617f,0xcd);
	write_cmos_sensor(0x6180,0xcd);
	write_cmos_sensor(0x6181,0xcd);
	write_cmos_sensor(0x6182,0xcd);
	write_cmos_sensor(0x6183,0xcd);
	write_cmos_sensor(0x6184,0xcd);
	write_cmos_sensor(0x6185,0xcd);
	write_cmos_sensor(0x6186,0xcd);
	write_cmos_sensor(0x6187,0xcd);
	write_cmos_sensor(0x6188,0xcd);
	write_cmos_sensor(0x6189,0xcd);
	write_cmos_sensor(0x618a,0xcd);
	write_cmos_sensor(0x618b,0xcd);
	write_cmos_sensor(0x618c,0xcd);
	write_cmos_sensor(0x618d,0xcd);
	write_cmos_sensor(0x618e,0xcd);
	write_cmos_sensor(0x618f,0xcd);
	write_cmos_sensor(0x6190,0xcd);
	write_cmos_sensor(0x6191,0xcd);
	write_cmos_sensor(0x6192,0xcd);
	write_cmos_sensor(0x6193,0xcd);
	write_cmos_sensor(0x6194,0xcd);
	write_cmos_sensor(0x6195,0xcd);
	write_cmos_sensor(0x6196,0xcd);
	write_cmos_sensor(0x6197,0xcd);
	write_cmos_sensor(0x6198,0xcd);
	write_cmos_sensor(0x6199,0xcd);
	write_cmos_sensor(0x619a,0xcd);
	write_cmos_sensor(0x619b,0xcd);
	write_cmos_sensor(0x619c,0xcd);
	write_cmos_sensor(0x619d,0xcd);
	write_cmos_sensor(0x619e,0xcd);
	write_cmos_sensor(0x619f,0xcd);
	write_cmos_sensor(0x61a0,0xcd);
	write_cmos_sensor(0x61a1,0xcd);
	write_cmos_sensor(0x61a2,0xcd);
	write_cmos_sensor(0x61a3,0xcd);
	write_cmos_sensor(0x61a4,0xcd);
	write_cmos_sensor(0x61a5,0xcd);
	write_cmos_sensor(0x61a6,0xcd);
	write_cmos_sensor(0x61a7,0xcd);
	write_cmos_sensor(0x61a8,0xcd);
	write_cmos_sensor(0x61a9,0xcd);
	write_cmos_sensor(0x61aa,0xcd);
	write_cmos_sensor(0x61ab,0xcd);
	write_cmos_sensor(0x61ac,0xcd);
	write_cmos_sensor(0x61ad,0xcd);
	write_cmos_sensor(0x61ae,0xcd);
	write_cmos_sensor(0x61af,0xcd);
	write_cmos_sensor(0x61b0,0xcd);
	write_cmos_sensor(0x61b1,0xcd);
	write_cmos_sensor(0x61b2,0xcd);
	write_cmos_sensor(0x61b3,0xcd);
	write_cmos_sensor(0x61b4,0xcd);
	write_cmos_sensor(0x61b5,0xcd);
	write_cmos_sensor(0x61b6,0xcd);
	write_cmos_sensor(0x61b7,0xcd);
	write_cmos_sensor(0x61b8,0xcd);
	write_cmos_sensor(0x61b9,0xcd);
	write_cmos_sensor(0x61ba,0xcd);
	write_cmos_sensor(0x61bb,0xcd);
	write_cmos_sensor(0x61bc,0xcd);
	write_cmos_sensor(0x61bd,0xcd);
	write_cmos_sensor(0x61be,0xcd);
	write_cmos_sensor(0x61bf,0xcd);
	write_cmos_sensor(0x61c0,0xcd);
	write_cmos_sensor(0x61c1,0xcd);
	write_cmos_sensor(0x61c2,0xcd);
	write_cmos_sensor(0x61c3,0xcd);
	write_cmos_sensor(0x61c4,0xcd);
	write_cmos_sensor(0x61c5,0xcd);
	write_cmos_sensor(0x61c6,0xcd);
	write_cmos_sensor(0x61c7,0xcd);
	write_cmos_sensor(0x61c8,0xcd);
	write_cmos_sensor(0x61c9,0xcd);
	write_cmos_sensor(0x61ca,0xcd);
	write_cmos_sensor(0x61cb,0xcd);
	write_cmos_sensor(0x61cc,0xcd);
	write_cmos_sensor(0x61cd,0xcd);
	write_cmos_sensor(0x61ce,0xcd);
	write_cmos_sensor(0x61cf,0xcd);
	write_cmos_sensor(0x61d0,0xcd);
	write_cmos_sensor(0x61d1,0xcd);
	write_cmos_sensor(0x61d2,0xcd);
	write_cmos_sensor(0x61d3,0xcd);
	write_cmos_sensor(0x61d4,0xcd);
	write_cmos_sensor(0x61d5,0xcd);
	write_cmos_sensor(0x61d6,0xcd);
	write_cmos_sensor(0x61d7,0xcd);
	write_cmos_sensor(0x61d8,0xcd);
	write_cmos_sensor(0x61d9,0xcd);
	write_cmos_sensor(0x61da,0xcd);
	write_cmos_sensor(0x61db,0xcd);
	write_cmos_sensor(0x61dc,0xcd);
	write_cmos_sensor(0x61dd,0xcd);
	write_cmos_sensor(0x61de,0xcd);
	write_cmos_sensor(0x61df,0xcd);
	write_cmos_sensor(0x61e0,0xcd);
	write_cmos_sensor(0x61e1,0xcd);
	write_cmos_sensor(0x61e2,0xcd);
	write_cmos_sensor(0x61e3,0xcd);
	write_cmos_sensor(0x61e4,0xcd);
	write_cmos_sensor(0x61e5,0xcd);
	write_cmos_sensor(0x61e6,0xcd);
	write_cmos_sensor(0x61e7,0xcd);
	write_cmos_sensor(0x61e8,0xcd);
	write_cmos_sensor(0x61e9,0xcd);
	write_cmos_sensor(0x61ea,0xcd);
	write_cmos_sensor(0x61eb,0xcd);
	write_cmos_sensor(0x61ec,0xcd);
	write_cmos_sensor(0x61ed,0xcd);
	write_cmos_sensor(0x61ee,0xcd);
	write_cmos_sensor(0x61ef,0xcd);
	write_cmos_sensor(0x61f0,0xcd);
	write_cmos_sensor(0x61f1,0xcd);
	write_cmos_sensor(0x61f2,0xcd);
	write_cmos_sensor(0x61f3,0xcd);
	write_cmos_sensor(0x61f4,0xcd);
	write_cmos_sensor(0x61f5,0xcd);
	write_cmos_sensor(0x61f6,0xcd);
	write_cmos_sensor(0x61f7,0xcd);
	write_cmos_sensor(0x61f8,0xcd);
	write_cmos_sensor(0x61f9,0xcd);
	write_cmos_sensor(0x61fa,0xcd);
	write_cmos_sensor(0x61fb,0xcd);
	write_cmos_sensor(0x61fc,0xcd);
	write_cmos_sensor(0x61fd,0xcd);
	write_cmos_sensor(0x61fe,0xcd);
	write_cmos_sensor(0x61ff,0xcd);
	write_cmos_sensor(0x6200,0xcd);
	write_cmos_sensor(0x6201,0xcd);
	write_cmos_sensor(0x6202,0xcd);
	write_cmos_sensor(0x6203,0xcd);
	write_cmos_sensor(0x6204,0xcd);
	write_cmos_sensor(0x6205,0xcd);
	write_cmos_sensor(0x6206,0xcd);
	write_cmos_sensor(0x6207,0xcd);
	write_cmos_sensor(0x6208,0xcd);
	write_cmos_sensor(0x6209,0xcd);
	write_cmos_sensor(0x620a,0xcd);
	write_cmos_sensor(0x620b,0xcd);
	write_cmos_sensor(0x620c,0xcd);
	write_cmos_sensor(0x620d,0xcd);
	write_cmos_sensor(0x620e,0xcd);
	write_cmos_sensor(0x620f,0xcd);
	write_cmos_sensor(0x6210,0xcd);
	write_cmos_sensor(0x6211,0xcd);
	write_cmos_sensor(0x6212,0xcd);
	write_cmos_sensor(0x6213,0xcd);
	write_cmos_sensor(0x6214,0xcd);
	write_cmos_sensor(0x6215,0xcd);
	write_cmos_sensor(0x6216,0xcd);
	write_cmos_sensor(0x6217,0xcd);
	write_cmos_sensor(0x6218,0xcd);
	write_cmos_sensor(0x6219,0xcd);
	write_cmos_sensor(0x621a,0xcd);
	write_cmos_sensor(0x621b,0xcd);
	write_cmos_sensor(0x621c,0xcd);
	write_cmos_sensor(0x621d,0xcd);
	write_cmos_sensor(0x621e,0xcd);
	write_cmos_sensor(0x621f,0xcd);
	write_cmos_sensor(0x6220,0xcd);
	write_cmos_sensor(0x6221,0xcd);
	write_cmos_sensor(0x6222,0xcd);
	write_cmos_sensor(0x6223,0xcd);
	write_cmos_sensor(0x6224,0xcd);
	write_cmos_sensor(0x6225,0xcd);
	write_cmos_sensor(0x6226,0xcd);
	write_cmos_sensor(0x6227,0xcd);
	write_cmos_sensor(0x6228,0xcd);
	write_cmos_sensor(0x6229,0xcd);
	write_cmos_sensor(0x622a,0xcd);
	write_cmos_sensor(0x622b,0xcd);
	write_cmos_sensor(0x622c,0xcd);
	write_cmos_sensor(0x622d,0xcd);
	write_cmos_sensor(0x622e,0xcd);
	write_cmos_sensor(0x622f,0xcd);
	write_cmos_sensor(0x6230,0xcd);
	write_cmos_sensor(0x6231,0xcd);
	write_cmos_sensor(0x6232,0xcd);
	write_cmos_sensor(0x6233,0xcd);
	write_cmos_sensor(0x6234,0xcd);
	write_cmos_sensor(0x6235,0xcd);
	write_cmos_sensor(0x6236,0xcd);
	write_cmos_sensor(0x6237,0xcd);
	write_cmos_sensor(0x6238,0xcd);
	write_cmos_sensor(0x6239,0xcd);
	write_cmos_sensor(0x623a,0xcd);
	write_cmos_sensor(0x623b,0xcd);
	write_cmos_sensor(0x623c,0xcd);
	write_cmos_sensor(0x623d,0xcd);
	write_cmos_sensor(0x623e,0xcd);
	write_cmos_sensor(0x623f,0xcd);
	write_cmos_sensor(0x6240,0xcd);
	write_cmos_sensor(0x6241,0xcd);
	write_cmos_sensor(0x6242,0xcd);
	write_cmos_sensor(0x6243,0xcd);
	write_cmos_sensor(0x6244,0xcd);
	write_cmos_sensor(0x6245,0xcd);
	write_cmos_sensor(0x6246,0xcd);
	write_cmos_sensor(0x6247,0xcd);
	write_cmos_sensor(0x6248,0xcd);
	write_cmos_sensor(0x6249,0xcd);
	write_cmos_sensor(0x624a,0xcd);
	write_cmos_sensor(0x624b,0xcd);
	write_cmos_sensor(0x624c,0xcd);
	write_cmos_sensor(0x624d,0xcd);
	write_cmos_sensor(0x624e,0xcd);
	write_cmos_sensor(0x624f,0xcd);
	write_cmos_sensor(0x5a40,0x75);
	write_cmos_sensor(0x5a41,0x75);
	write_cmos_sensor(0x5a42,0x75);
	write_cmos_sensor(0x5a43,0x75);
	write_cmos_sensor(0x5a44,0x75);
	write_cmos_sensor(0x5a45,0x75);
	write_cmos_sensor(0x5a46,0x75);
	write_cmos_sensor(0x5a47,0x75);
	write_cmos_sensor(0x5a48,0x75);
	write_cmos_sensor(0x5a49,0x75);
	write_cmos_sensor(0x5a4a,0x75);
	write_cmos_sensor(0x5a4b,0x75);
	write_cmos_sensor(0x5a4c,0x75);
	write_cmos_sensor(0x5a4d,0x75);
	write_cmos_sensor(0x5a4e,0x75);
	write_cmos_sensor(0x5a4f,0x75);
	write_cmos_sensor(0x5a50,0x75);
	write_cmos_sensor(0x5a51,0x75);
	write_cmos_sensor(0x5a52,0x75);
	write_cmos_sensor(0x5a53,0x75);
	write_cmos_sensor(0x5a54,0x75);
	write_cmos_sensor(0x5a55,0x75);
	write_cmos_sensor(0x5a56,0x75);
	write_cmos_sensor(0x5a57,0x75);
	write_cmos_sensor(0x5a58,0x75);
	write_cmos_sensor(0x5a59,0x75);
	write_cmos_sensor(0x5a5a,0x75);
	write_cmos_sensor(0x5a5b,0x75);
	write_cmos_sensor(0x5a5c,0x75);
	write_cmos_sensor(0x5a5d,0x75);
	write_cmos_sensor(0x5a5e,0x75);
	write_cmos_sensor(0x5a5f,0x75);
	write_cmos_sensor(0x5a60,0x75);
	write_cmos_sensor(0x5a61,0x75);
	write_cmos_sensor(0x5a62,0x75);
	write_cmos_sensor(0x5a63,0x75);
	write_cmos_sensor(0x5a64,0x75);
	write_cmos_sensor(0x5a65,0x75);
	write_cmos_sensor(0x5a66,0x75);
	write_cmos_sensor(0x5a67,0x75);
	write_cmos_sensor(0x5a68,0x75);
	write_cmos_sensor(0x5a69,0x75);
	write_cmos_sensor(0x5a6a,0x75);
	write_cmos_sensor(0x5a6b,0x75);
	write_cmos_sensor(0x5a0c,0x75);
	write_cmos_sensor(0x5a6d,0x75);
	write_cmos_sensor(0x5a6e,0x75);
	write_cmos_sensor(0x5a6f,0x75);
	write_cmos_sensor(0x5a70,0x75);
	write_cmos_sensor(0x5a71,0x75);
	write_cmos_sensor(0x5a72,0x75);
	write_cmos_sensor(0x5a73,0x75);
	write_cmos_sensor(0x5a74,0x75);
	write_cmos_sensor(0x5a75,0x75);
	write_cmos_sensor(0x5a76,0x75);
	write_cmos_sensor(0x5a77,0x75);
	write_cmos_sensor(0x5a78,0x75);
	write_cmos_sensor(0x5a79,0x75);
	write_cmos_sensor(0x5a7a,0x75);
	write_cmos_sensor(0x5a7b,0x75);
	write_cmos_sensor(0x5a7c,0x75);
	write_cmos_sensor(0x5a7d,0x75);
	write_cmos_sensor(0x5a7e,0x75);
	write_cmos_sensor(0x5a7f,0x75);
	write_cmos_sensor(0x5a80,0x75);
	write_cmos_sensor(0x5a81,0x75);
	write_cmos_sensor(0x5a82,0x75);
	write_cmos_sensor(0x5a83,0x75);
	write_cmos_sensor(0x5a84,0x75);
	write_cmos_sensor(0x5a85,0x75);
	write_cmos_sensor(0x5a86,0x75);
	write_cmos_sensor(0x5a87,0x75);
	write_cmos_sensor(0x5a88,0x75);
	write_cmos_sensor(0x5a89,0x75);
	write_cmos_sensor(0x5a8a,0x75);
	write_cmos_sensor(0x5a8b,0x75);
	write_cmos_sensor(0x5a8c,0x75);
	write_cmos_sensor(0x5a8d,0x75);
	write_cmos_sensor(0x5a8e,0x75);
	write_cmos_sensor(0x5a8f,0x75);
	write_cmos_sensor(0x5a90,0x75);
	write_cmos_sensor(0x5a91,0x75);
	write_cmos_sensor(0x5a92,0x75);
	write_cmos_sensor(0x5a93,0x75);
	write_cmos_sensor(0x5a94,0x75);
	write_cmos_sensor(0x5a95,0x75);
	write_cmos_sensor(0x5a96,0x75);
	write_cmos_sensor(0x5a97,0x75);
	write_cmos_sensor(0x5a98,0x75);
	write_cmos_sensor(0x5a99,0x75);
	write_cmos_sensor(0x5a9a,0x75);
	write_cmos_sensor(0x5a9b,0x75);
	write_cmos_sensor(0x5a9c,0x75);
	write_cmos_sensor(0x5a9d,0x75);
	write_cmos_sensor(0x5a9e,0x75);
	write_cmos_sensor(0x5a9f,0x75);
	write_cmos_sensor(0x5aa0,0x75);
	write_cmos_sensor(0x5aa1,0x75);
	write_cmos_sensor(0x5aa2,0x75);
	write_cmos_sensor(0x5aa3,0x75);
	write_cmos_sensor(0x5aa4,0x75);
	write_cmos_sensor(0x5aa5,0x75);
	write_cmos_sensor(0x5aa6,0x75);
	write_cmos_sensor(0x5aa7,0x75);
	write_cmos_sensor(0x5aa8,0x75);
	write_cmos_sensor(0x5aa9,0x75);
	write_cmos_sensor(0x5aaa,0x75);
	write_cmos_sensor(0x5aab,0x75);
	write_cmos_sensor(0x5aac,0x75);
	write_cmos_sensor(0x5aad,0x75);
	write_cmos_sensor(0x5aae,0x75);
	write_cmos_sensor(0x5aaf,0x75);
	write_cmos_sensor(0x5ab0,0x75);
	write_cmos_sensor(0x5ab1,0x75);
	write_cmos_sensor(0x5ab2,0x75);
	write_cmos_sensor(0x5ab3,0x75);
	write_cmos_sensor(0x5ab4,0x75);
	write_cmos_sensor(0x5ab5,0x75);
	write_cmos_sensor(0x5ab6,0x75);
	write_cmos_sensor(0x5ab7,0x75);
	write_cmos_sensor(0x5ab8,0x75);
	write_cmos_sensor(0x5ab9,0x75);
	write_cmos_sensor(0x5aba,0x75);
	write_cmos_sensor(0x5abb,0x75);
	write_cmos_sensor(0x5abc,0x75);
	write_cmos_sensor(0x5abd,0x75);
	write_cmos_sensor(0x5abe,0x75);
	write_cmos_sensor(0x5abf,0x75);
	write_cmos_sensor(0x5ac0,0x75);
	write_cmos_sensor(0x5ac1,0x75);
	write_cmos_sensor(0x5ac2,0x75);
	write_cmos_sensor(0x5ac3,0x75);
	write_cmos_sensor(0x5ac4,0x75);
	write_cmos_sensor(0x5ac5,0x75);
	write_cmos_sensor(0x5ac6,0x75);
	write_cmos_sensor(0x5ac7,0x75);
	write_cmos_sensor(0x5ac8,0x75);
	write_cmos_sensor(0x5ac9,0x75);
	write_cmos_sensor(0x5aca,0x75);
	write_cmos_sensor(0x5acb,0x75);
	write_cmos_sensor(0x5acc,0x75);
	write_cmos_sensor(0x5acd,0x75);
	write_cmos_sensor(0x5ace,0x75);
	write_cmos_sensor(0x5acf,0x75);
	write_cmos_sensor(0x5ad0,0x75);
	write_cmos_sensor(0x5ad1,0x75);
	write_cmos_sensor(0x5ad2,0x75);
	write_cmos_sensor(0x5ad3,0x75);
	write_cmos_sensor(0x5ad4,0x75);
	write_cmos_sensor(0x5ad5,0x75);
	write_cmos_sensor(0x5ad6,0x75);
	write_cmos_sensor(0x5ad7,0x75);
	write_cmos_sensor(0x5ad8,0x75);
	write_cmos_sensor(0x5ad9,0x75);
	write_cmos_sensor(0x5ada,0x75);
	write_cmos_sensor(0x5adb,0x75);
	write_cmos_sensor(0x5adc,0x75);
	write_cmos_sensor(0x5add,0x75);
	write_cmos_sensor(0x5ade,0x75);
	write_cmos_sensor(0x5adf,0x75);
	write_cmos_sensor(0x5ae0,0x75);
	write_cmos_sensor(0x5ae1,0x75);
	write_cmos_sensor(0x5ae2,0x75);
	write_cmos_sensor(0x5ae3,0x75);
	write_cmos_sensor(0x5ae4,0x75);
	write_cmos_sensor(0x5ae5,0x75);
	write_cmos_sensor(0x5ae6,0x75);
	write_cmos_sensor(0x5ae7,0x75);
	write_cmos_sensor(0x5ae8,0x75);
	write_cmos_sensor(0x5ae9,0x75);
	write_cmos_sensor(0x5aea,0x75);
	write_cmos_sensor(0x5aeb,0x75);
	write_cmos_sensor(0x5aec,0x75);
	write_cmos_sensor(0x5aed,0x75);
	write_cmos_sensor(0x5aee,0x75);
	write_cmos_sensor(0x5aef,0x75);
	write_cmos_sensor(0x5af0,0x75);
	write_cmos_sensor(0x5af1,0x75);
	write_cmos_sensor(0x5af2,0x75);
	write_cmos_sensor(0x5af3,0x75);
	write_cmos_sensor(0x5af4,0x75);
	write_cmos_sensor(0x5af5,0x75);
	write_cmos_sensor(0x5af6,0x75);
	write_cmos_sensor(0x5af7,0x75);
	write_cmos_sensor(0x5af8,0x75);
	write_cmos_sensor(0x5af9,0x75);
	write_cmos_sensor(0x5afa,0x75);
	write_cmos_sensor(0x5afb,0x75);
	write_cmos_sensor(0x5afc,0x75);
	write_cmos_sensor(0x5afd,0x75);
	write_cmos_sensor(0x5afe,0x75);
	write_cmos_sensor(0x5aff,0x75);
	write_cmos_sensor(0x5b00,0x75);
	write_cmos_sensor(0x5b01,0x75);
	write_cmos_sensor(0x5b02,0x75);
	write_cmos_sensor(0x5b03,0x75);
	write_cmos_sensor(0x5b04,0x75);
	write_cmos_sensor(0x5b05,0x75);
	write_cmos_sensor(0x5b06,0x75);
	write_cmos_sensor(0x5b07,0x75);
	write_cmos_sensor(0x5b08,0x75);
	write_cmos_sensor(0x5b09,0x75);
	write_cmos_sensor(0x5b0a,0x75);
	write_cmos_sensor(0x5b0b,0x75);
	write_cmos_sensor(0x5b0c,0x75);
	write_cmos_sensor(0x5b0d,0x75);
	write_cmos_sensor(0x5b0e,0x75);
	write_cmos_sensor(0x5b0f,0x75);
	write_cmos_sensor(0x5b10,0x75);
	write_cmos_sensor(0x5b11,0x75);
	write_cmos_sensor(0x5b12,0x75);
	write_cmos_sensor(0x5b13,0x75);
	write_cmos_sensor(0x5b14,0x75);
	write_cmos_sensor(0x5b15,0x75);
	write_cmos_sensor(0x5b16,0x75);
	write_cmos_sensor(0x5b17,0x75);
	write_cmos_sensor(0x5b18,0x75);
	write_cmos_sensor(0x5b19,0x75);
	write_cmos_sensor(0x5b1a,0x75);
	write_cmos_sensor(0x5b1b,0x75);
	write_cmos_sensor(0x5b1c,0x75);
	write_cmos_sensor(0x5b1d,0x75);
	write_cmos_sensor(0x5b1e,0x75);
	write_cmos_sensor(0x5b1f,0x75);
	write_cmos_sensor(0x5b20,0x75);
	write_cmos_sensor(0x5b21,0x75);
	write_cmos_sensor(0x5b22,0x75);
	write_cmos_sensor(0x5b23,0x75);
	write_cmos_sensor(0x5b24,0x75);
	write_cmos_sensor(0x5b25,0x75);
	write_cmos_sensor(0x5b26,0x75);
	write_cmos_sensor(0x5b27,0x75);
	write_cmos_sensor(0x5b28,0x75);
	write_cmos_sensor(0x5b29,0x75);
	write_cmos_sensor(0x5b2a,0x75);
	write_cmos_sensor(0x5b2b,0x75);
	write_cmos_sensor(0x5b2c,0x75);
	write_cmos_sensor(0x5b2d,0x75);
	write_cmos_sensor(0x5b2e,0x75);
	write_cmos_sensor(0x5b2f,0x75);
	write_cmos_sensor(0x5b30,0x75);
	write_cmos_sensor(0x5b31,0x75);
	write_cmos_sensor(0x5b32,0x75);
	write_cmos_sensor(0x5b33,0x75);
	write_cmos_sensor(0x5b34,0x75);
	write_cmos_sensor(0x5b35,0x75);
	write_cmos_sensor(0x5b36,0x75);
	write_cmos_sensor(0x5b37,0x75);
	write_cmos_sensor(0x5b38,0x75);
	write_cmos_sensor(0x5b39,0x75);
	write_cmos_sensor(0x5b3a,0x75);
	write_cmos_sensor(0x5b3b,0x75);
	write_cmos_sensor(0x5b3c,0x75);
	write_cmos_sensor(0x5b3d,0x75);
	write_cmos_sensor(0x5b3e,0x75);
	write_cmos_sensor(0x5b3f,0x75);
	write_cmos_sensor(0x5b40,0x75);
	write_cmos_sensor(0x5b41,0x75);
	write_cmos_sensor(0x5b42,0x75);
	write_cmos_sensor(0x5b43,0x75);
	write_cmos_sensor(0x5b44,0x75);
	write_cmos_sensor(0x5b45,0x75);
	write_cmos_sensor(0x5b46,0x75);
	write_cmos_sensor(0x5b47,0x75);
	write_cmos_sensor(0x5b48,0x75);
	write_cmos_sensor(0x5b49,0x75);
	write_cmos_sensor(0x5b4a,0x75);
	write_cmos_sensor(0x5b4b,0x75);
	write_cmos_sensor(0x5b4c,0x75);
	write_cmos_sensor(0x5b4d,0x75);
	write_cmos_sensor(0x5b4e,0x75);
	write_cmos_sensor(0x5b4f,0x75);
	write_cmos_sensor(0x5b50,0x75);
	write_cmos_sensor(0x5b51,0x75);
	write_cmos_sensor(0x5b52,0x75);
	write_cmos_sensor(0x5b53,0x75);
	write_cmos_sensor(0x5b54,0x75);
	write_cmos_sensor(0x5b55,0x75);
	write_cmos_sensor(0x5b56,0x75);
	write_cmos_sensor(0x5b57,0x75);
	write_cmos_sensor(0x5b58,0x75);
	write_cmos_sensor(0x5b59,0x75);
	write_cmos_sensor(0x5b5a,0x75);
	write_cmos_sensor(0x5b5b,0x75);
	write_cmos_sensor(0x5b5c,0x75);
	write_cmos_sensor(0x5b5d,0x75);
	write_cmos_sensor(0x5b5e,0x75);
	write_cmos_sensor(0x5b5f,0x75);
	write_cmos_sensor(0x5b80,0x75);
	write_cmos_sensor(0x5b81,0x75);
	write_cmos_sensor(0x5b82,0x75);
	write_cmos_sensor(0x5b83,0x75);
	write_cmos_sensor(0x5b84,0x75);
	write_cmos_sensor(0x5b85,0x75);
	write_cmos_sensor(0x5b86,0x75);
	write_cmos_sensor(0x5b87,0x75);
	write_cmos_sensor(0x5b88,0x75);
	write_cmos_sensor(0x5b89,0x75);
	write_cmos_sensor(0x5b8a,0x75);
	write_cmos_sensor(0x5b8b,0x75);
	write_cmos_sensor(0x5b8c,0x75);
	write_cmos_sensor(0x5b8d,0x75);
	write_cmos_sensor(0x5b8e,0x75);
	write_cmos_sensor(0x5b8f,0x75);
	write_cmos_sensor(0x5b90,0x75);
	write_cmos_sensor(0x5b91,0x75);
	write_cmos_sensor(0x5b92,0x75);
	write_cmos_sensor(0x5b93,0x75);
	write_cmos_sensor(0x5b94,0x75);
	write_cmos_sensor(0x5b95,0x75);
	write_cmos_sensor(0x5b96,0x75);
	write_cmos_sensor(0x5b97,0x75);
	write_cmos_sensor(0x5b98,0x75);
	write_cmos_sensor(0x5b99,0x75);
	write_cmos_sensor(0x5b9a,0x75);
	write_cmos_sensor(0x5b9b,0x75);
	write_cmos_sensor(0x5b9c,0x75);
	write_cmos_sensor(0x5b9d,0x75);
	write_cmos_sensor(0x5b9e,0x75);
	write_cmos_sensor(0x5b9f,0x75);
	write_cmos_sensor(0x5ba0,0x75);
	write_cmos_sensor(0x5ba1,0x75);
	write_cmos_sensor(0x5ba2,0x75);
	write_cmos_sensor(0x5ba3,0x75);
	write_cmos_sensor(0x5ba4,0x75);
	write_cmos_sensor(0x5ba5,0x75);
	write_cmos_sensor(0x5ba6,0x75);
	write_cmos_sensor(0x5ba7,0x75);
	write_cmos_sensor(0x5ba8,0x75);
	write_cmos_sensor(0x5ba9,0x75);
	write_cmos_sensor(0x5baa,0x75);
	write_cmos_sensor(0x5bab,0x75);
	write_cmos_sensor(0x5bac,0x75);
	write_cmos_sensor(0x5bad,0x75);
	write_cmos_sensor(0x5bae,0x75);
	write_cmos_sensor(0x5baf,0x75);
	write_cmos_sensor(0x5bb0,0x75);
	write_cmos_sensor(0x5bb1,0x75);
	write_cmos_sensor(0x5bb2,0x75);
	write_cmos_sensor(0x5bb3,0x75);
	write_cmos_sensor(0x5bb4,0x75);
	write_cmos_sensor(0x5bb5,0x75);
	write_cmos_sensor(0x5bb6,0x75);
	write_cmos_sensor(0x5bb7,0x75);
	write_cmos_sensor(0x5bb8,0x75);
	write_cmos_sensor(0x5bb9,0x75);
	write_cmos_sensor(0x5bba,0x75);
	write_cmos_sensor(0x5bbb,0x75);
	write_cmos_sensor(0x5bbc,0x75);
	write_cmos_sensor(0x5bbd,0x75);
	write_cmos_sensor(0x5bbe,0x75);
	write_cmos_sensor(0x5bbf,0x75);
	write_cmos_sensor(0x5bc0,0x75);
	write_cmos_sensor(0x5bc1,0x75);
	write_cmos_sensor(0x5bc2,0x75);
	write_cmos_sensor(0x5bc3,0x75);
	write_cmos_sensor(0x5bc4,0x75);
	write_cmos_sensor(0x5bc5,0x75);
	write_cmos_sensor(0x5bc6,0x75);
	write_cmos_sensor(0x5bc7,0x75);
	write_cmos_sensor(0x5bc8,0x75);
	write_cmos_sensor(0x5bc9,0x75);
	write_cmos_sensor(0x5bca,0x75);
	write_cmos_sensor(0x5bcb,0x75);
	write_cmos_sensor(0x5bcc,0x75);
	write_cmos_sensor(0x5bcd,0x75);
	write_cmos_sensor(0x5bce,0x75);
	write_cmos_sensor(0x5bcf,0x75);
	write_cmos_sensor(0x5bd0,0x75);
	write_cmos_sensor(0x5bd1,0x75);
	write_cmos_sensor(0x5bd2,0x75);
	write_cmos_sensor(0x5bd3,0x75);
	write_cmos_sensor(0x5bd4,0x75);
	write_cmos_sensor(0x5bd5,0x75);
	write_cmos_sensor(0x5bd6,0x75);
	write_cmos_sensor(0x5bd7,0x75);
	write_cmos_sensor(0x5bd8,0x75);
	write_cmos_sensor(0x5bd9,0x75);
	write_cmos_sensor(0x5bda,0x75);
	write_cmos_sensor(0x5bdb,0x75);
	write_cmos_sensor(0x5bdc,0x75);
	write_cmos_sensor(0x5bdd,0x75);
	write_cmos_sensor(0x5bde,0x75);
	write_cmos_sensor(0x5bdf,0x75);
	write_cmos_sensor(0x5be0,0x75);
	write_cmos_sensor(0x5be1,0x75);
	write_cmos_sensor(0x5be2,0x75);
	write_cmos_sensor(0x5be3,0x75);
	write_cmos_sensor(0x5be4,0x75);
	write_cmos_sensor(0x5be5,0x75);
	write_cmos_sensor(0x5be6,0x75);
	write_cmos_sensor(0x5be7,0x75);
	write_cmos_sensor(0x5be8,0x75);
	write_cmos_sensor(0x5be9,0x75);
	write_cmos_sensor(0x5bea,0x75);
	write_cmos_sensor(0x5beb,0x75);
	write_cmos_sensor(0x5bec,0x75);
	write_cmos_sensor(0x5bed,0x75);
	write_cmos_sensor(0x5bee,0x75);
	write_cmos_sensor(0x5bef,0x75);
	write_cmos_sensor(0x5bf0,0x75);
	write_cmos_sensor(0x5bf1,0x75);
	write_cmos_sensor(0x5bf2,0x75);
	write_cmos_sensor(0x5bf3,0x75);
	write_cmos_sensor(0x5bf4,0x75);
	write_cmos_sensor(0x5bf5,0x75);
	write_cmos_sensor(0x5bf6,0x75);
	write_cmos_sensor(0x5bf7,0x75);
	write_cmos_sensor(0x5bf8,0x75);
	write_cmos_sensor(0x5bf9,0x75);
	write_cmos_sensor(0x5bfa,0x75);
	write_cmos_sensor(0x5bfb,0x75);
	write_cmos_sensor(0x5bfc,0x75);
	write_cmos_sensor(0x5bfd,0x75);
	write_cmos_sensor(0x5bfe,0x75);
	write_cmos_sensor(0x5bff,0x75);
	write_cmos_sensor(0x5c00,0x75);
	write_cmos_sensor(0x5c01,0x75);
	write_cmos_sensor(0x5c02,0x75);
	write_cmos_sensor(0x5c03,0x75);
	write_cmos_sensor(0x5c04,0x75);
	write_cmos_sensor(0x5c05,0x75);
	write_cmos_sensor(0x5c06,0x75);
	write_cmos_sensor(0x5c07,0x75);
	write_cmos_sensor(0x5c08,0x75);
	write_cmos_sensor(0x5c09,0x75);
	write_cmos_sensor(0x5c0a,0x75);
	write_cmos_sensor(0x5c0b,0x75);
	write_cmos_sensor(0x5c0c,0x75);
	write_cmos_sensor(0x5c0d,0x75);
	write_cmos_sensor(0x5c0e,0x75);
	write_cmos_sensor(0x5c0f,0x75);
	write_cmos_sensor(0x5c10,0x75);
	write_cmos_sensor(0x5c11,0x75);
	write_cmos_sensor(0x5c12,0x75);
	write_cmos_sensor(0x5c13,0x75);
	write_cmos_sensor(0x5c14,0x75);
	write_cmos_sensor(0x5c15,0x75);
	write_cmos_sensor(0x5c16,0x75);
	write_cmos_sensor(0x5c17,0x75);
	write_cmos_sensor(0x5c18,0x75);
	write_cmos_sensor(0x5c19,0x75);
	write_cmos_sensor(0x5c1a,0x75);
	write_cmos_sensor(0x5c1b,0x75);
	write_cmos_sensor(0x5c1c,0x75);
	write_cmos_sensor(0x5c1d,0x75);
	write_cmos_sensor(0x5c1e,0x75);
	write_cmos_sensor(0x5c1f,0x75);
	write_cmos_sensor(0x5c20,0x75);
	write_cmos_sensor(0x5c21,0x75);
	write_cmos_sensor(0x5c22,0x75);
	write_cmos_sensor(0x5c23,0x75);
	write_cmos_sensor(0x5c24,0x75);
	write_cmos_sensor(0x5c25,0x75);
	write_cmos_sensor(0x5c26,0x75);
	write_cmos_sensor(0x5c27,0x75);
	write_cmos_sensor(0x5c28,0x75);
	write_cmos_sensor(0x5c29,0x75);
	write_cmos_sensor(0x5c2a,0x75);
	write_cmos_sensor(0x5c2b,0x75);
	write_cmos_sensor(0x5c2c,0x75);
	write_cmos_sensor(0x5c2d,0x75);
	write_cmos_sensor(0x5c2e,0x75);
	write_cmos_sensor(0x5c2f,0x75);
	write_cmos_sensor(0x5c30,0x75);
	write_cmos_sensor(0x5c31,0x75);
	write_cmos_sensor(0x5c32,0x75);
	write_cmos_sensor(0x5c33,0x75);
	write_cmos_sensor(0x5c34,0x75);
	write_cmos_sensor(0x5c35,0x75);
	write_cmos_sensor(0x5c36,0x75);
	write_cmos_sensor(0x5c37,0x75);
	write_cmos_sensor(0x5c38,0x75);
	write_cmos_sensor(0x5c39,0x75);
	write_cmos_sensor(0x5c3a,0x75);
	write_cmos_sensor(0x5c3b,0x75);
	write_cmos_sensor(0x5c3c,0x75);
	write_cmos_sensor(0x5c3d,0x75);
	write_cmos_sensor(0x5c3e,0x75);
	write_cmos_sensor(0x5c3f,0x75);
	write_cmos_sensor(0x5c40,0x75);
	write_cmos_sensor(0x5c41,0x75);
	write_cmos_sensor(0x5c42,0x75);
	write_cmos_sensor(0x5c43,0x75);
	write_cmos_sensor(0x5c44,0x75);
	write_cmos_sensor(0x5c45,0x75);
	write_cmos_sensor(0x5c46,0x75);
	write_cmos_sensor(0x5c47,0x75);
	write_cmos_sensor(0x5c48,0x75);
	write_cmos_sensor(0x5c49,0x75);
	write_cmos_sensor(0x5c4a,0x75);
	write_cmos_sensor(0x5c4b,0x75);
	write_cmos_sensor(0x5c4c,0x75);
	write_cmos_sensor(0x5c4d,0x75);
	write_cmos_sensor(0x5c4e,0x75);
	write_cmos_sensor(0x5c4f,0x75);
	write_cmos_sensor(0x5c50,0x75);
	write_cmos_sensor(0x5c51,0x75);
	write_cmos_sensor(0x5c52,0x75);
	write_cmos_sensor(0x5c53,0x75);
	write_cmos_sensor(0x5c54,0x75);
	write_cmos_sensor(0x5c55,0x75);
	write_cmos_sensor(0x5c56,0x75);
	write_cmos_sensor(0x5c57,0x75);
	write_cmos_sensor(0x5c58,0x75);
	write_cmos_sensor(0x5c59,0x75);
	write_cmos_sensor(0x5c5a,0x75);
	write_cmos_sensor(0x5c5b,0x75);
	write_cmos_sensor(0x5c5c,0x75);
	write_cmos_sensor(0x5c5d,0x75);
	write_cmos_sensor(0x5c5e,0x75);
	write_cmos_sensor(0x5c5f,0x75);
	write_cmos_sensor(0x5c60,0x75);
	write_cmos_sensor(0x5c61,0x75);
	write_cmos_sensor(0x5c62,0x75);
	write_cmos_sensor(0x5c63,0x75);
	write_cmos_sensor(0x5c64,0x75);
	write_cmos_sensor(0x5c65,0x75);
	write_cmos_sensor(0x5c66,0x75);
	write_cmos_sensor(0x5c67,0x75);
	write_cmos_sensor(0x5c68,0x75);
	write_cmos_sensor(0x5c69,0x75);
	write_cmos_sensor(0x5c6a,0x75);
	write_cmos_sensor(0x5c6b,0x75);
	write_cmos_sensor(0x5c0c,0x75);
	write_cmos_sensor(0x5c6d,0x75);
	write_cmos_sensor(0x5c6e,0x75);
	write_cmos_sensor(0x5c6f,0x75);
	write_cmos_sensor(0x5c70,0x75);
	write_cmos_sensor(0x5c71,0x75);
	write_cmos_sensor(0x5c72,0x75);
	write_cmos_sensor(0x5c73,0x75);
	write_cmos_sensor(0x5c74,0x75);
	write_cmos_sensor(0x5c75,0x75);
	write_cmos_sensor(0x5c76,0x75);
	write_cmos_sensor(0x5c77,0x75);
	write_cmos_sensor(0x5c78,0x75);
	write_cmos_sensor(0x5c79,0x75);
	write_cmos_sensor(0x5c7a,0x75);
	write_cmos_sensor(0x5c7b,0x75);
	write_cmos_sensor(0x5c7c,0x75);
	write_cmos_sensor(0x5c7d,0x75);
	write_cmos_sensor(0x5c7e,0x75);
	write_cmos_sensor(0x5c7f,0x75);
	write_cmos_sensor(0x5c80,0x75);
	write_cmos_sensor(0x5c81,0x75);
	write_cmos_sensor(0x5c82,0x75);
	write_cmos_sensor(0x5c83,0x75);
	write_cmos_sensor(0x5c84,0x75);
	write_cmos_sensor(0x5c85,0x75);
	write_cmos_sensor(0x5c86,0x75);
	write_cmos_sensor(0x5c87,0x75);
	write_cmos_sensor(0x5c88,0x75);
	write_cmos_sensor(0x5c89,0x75);
	write_cmos_sensor(0x5c8a,0x75);
	write_cmos_sensor(0x5c8b,0x75);
	write_cmos_sensor(0x5c8c,0x75);
	write_cmos_sensor(0x5c8d,0x75);
	write_cmos_sensor(0x5c8e,0x75);
	write_cmos_sensor(0x5c8f,0x75);
	write_cmos_sensor(0x5c90,0x75);
	write_cmos_sensor(0x5c91,0x75);
	write_cmos_sensor(0x5c92,0x75);
	write_cmos_sensor(0x5c93,0x75);
	write_cmos_sensor(0x5c94,0x75);
	write_cmos_sensor(0x5c95,0x75);
	write_cmos_sensor(0x5c96,0x75);
	write_cmos_sensor(0x5c97,0x75);
	write_cmos_sensor(0x5c98,0x75);
	write_cmos_sensor(0x5c99,0x75);
	write_cmos_sensor(0x5c9a,0x75);
	write_cmos_sensor(0x5c9b,0x75);
	write_cmos_sensor(0x5c9c,0x75);
	write_cmos_sensor(0x5c9d,0x75);
	write_cmos_sensor(0x5c9e,0x75);
	write_cmos_sensor(0x5c9f,0x75);
	write_cmos_sensor(0x5ca0,0x75);
	write_cmos_sensor(0x5ca1,0x75);
	write_cmos_sensor(0x5ca2,0x75);
	write_cmos_sensor(0x5ca3,0x75);
	write_cmos_sensor(0x5ca4,0x75);
	write_cmos_sensor(0x5ca5,0x75);
	write_cmos_sensor(0x5ca6,0x75);
	write_cmos_sensor(0x5ca7,0x75);
	write_cmos_sensor(0x5ca8,0x75);
	write_cmos_sensor(0x5ca9,0x75);
	write_cmos_sensor(0x5caa,0x75);
	write_cmos_sensor(0x5cab,0x75);
	write_cmos_sensor(0x5cac,0x75);
	write_cmos_sensor(0x5cad,0x75);
	write_cmos_sensor(0x5cae,0x75);
	write_cmos_sensor(0x5caf,0x75);
	write_cmos_sensor(0x5cb0,0x75);
	write_cmos_sensor(0x5cb1,0x75);
	write_cmos_sensor(0x5cb2,0x75);
	write_cmos_sensor(0x5cb3,0x75);
	write_cmos_sensor(0x5cb4,0x75);
	write_cmos_sensor(0x5cb5,0x75);
	write_cmos_sensor(0x5cb6,0x75);
	write_cmos_sensor(0x5cb7,0x75);
	write_cmos_sensor(0x5cb8,0x75);
	write_cmos_sensor(0x5cb9,0x75);
	write_cmos_sensor(0x5cba,0x75);
	write_cmos_sensor(0x5cbb,0x75);
	write_cmos_sensor(0x5cbc,0x75);
	write_cmos_sensor(0x5cbd,0x75);
	write_cmos_sensor(0x5cbe,0x75);
	write_cmos_sensor(0x5cbf,0x75);
	write_cmos_sensor(0x5cc0,0x75);
	write_cmos_sensor(0x5cc1,0x75);
	write_cmos_sensor(0x5cc2,0x75);
	write_cmos_sensor(0x5cc3,0x75);
	write_cmos_sensor(0x5cc4,0x75);
	write_cmos_sensor(0x5cc5,0x75);
	write_cmos_sensor(0x5cc6,0x75);
	write_cmos_sensor(0x5cc7,0x75);
	write_cmos_sensor(0x5cc8,0x75);
	write_cmos_sensor(0x5cc9,0x75);
	write_cmos_sensor(0x5cca,0x75);
	write_cmos_sensor(0x5ccb,0x75);
	write_cmos_sensor(0x5ccc,0x75);
	write_cmos_sensor(0x5ccd,0x75);
	write_cmos_sensor(0x5cce,0x75);
	write_cmos_sensor(0x5ccf,0x75);
	write_cmos_sensor(0x5cd0,0x75);
	write_cmos_sensor(0x5cd1,0x75);
	write_cmos_sensor(0x5cd2,0x75);
	write_cmos_sensor(0x5cd3,0x75);
	write_cmos_sensor(0x5cd4,0x75);
	write_cmos_sensor(0x5cd5,0x75);
	write_cmos_sensor(0x5cd6,0x75);
	write_cmos_sensor(0x5cd7,0x75);
	write_cmos_sensor(0x5cd8,0x75);
	write_cmos_sensor(0x5cd9,0x75);
	write_cmos_sensor(0x5cda,0x75);
	write_cmos_sensor(0x5cdb,0x75);
	write_cmos_sensor(0x5cdc,0x75);
	write_cmos_sensor(0x5cdd,0x75);
	write_cmos_sensor(0x5cde,0x75);
	write_cmos_sensor(0x5cdf,0x75);
	write_cmos_sensor(0x5ce0,0x75);
	write_cmos_sensor(0x5ce1,0x75);
	write_cmos_sensor(0x5ce2,0x75);
	write_cmos_sensor(0x5ce3,0x75);
	write_cmos_sensor(0x5ce4,0x75);
	write_cmos_sensor(0x5ce5,0x75);
	write_cmos_sensor(0x5ce6,0x75);
	write_cmos_sensor(0x5ce7,0x75);
	write_cmos_sensor(0x5ce8,0x75);
	write_cmos_sensor(0x5ce9,0x75);
	write_cmos_sensor(0x5cea,0x75);
	write_cmos_sensor(0x5ceb,0x75);
	write_cmos_sensor(0x5cec,0x75);
	write_cmos_sensor(0x5ced,0x75);
	write_cmos_sensor(0x5cee,0x75);
	write_cmos_sensor(0x5cef,0x75);
	write_cmos_sensor(0x5cf0,0x75);
	write_cmos_sensor(0x5cf1,0x75);
	write_cmos_sensor(0x5cf2,0x75);
	write_cmos_sensor(0x5cf3,0x75);
	write_cmos_sensor(0x5cf4,0x75);
	write_cmos_sensor(0x5cf5,0x75);
	write_cmos_sensor(0x5cf6,0x75);
	write_cmos_sensor(0x5cf7,0x75);
	write_cmos_sensor(0x5cf8,0x75);
	write_cmos_sensor(0x5cf9,0x75);
	write_cmos_sensor(0x5cfa,0x75);
	write_cmos_sensor(0x5cfb,0x75);
	write_cmos_sensor(0x5cfc,0x75);
	write_cmos_sensor(0x5cfd,0x75);
	write_cmos_sensor(0x5cfe,0x75);
	write_cmos_sensor(0x5cff,0x75);
	write_cmos_sensor(0x5d00,0x75);
	write_cmos_sensor(0x5d01,0x75);
	write_cmos_sensor(0x5d02,0x75);
	write_cmos_sensor(0x5d03,0x75);
	write_cmos_sensor(0x5d04,0x75);
	write_cmos_sensor(0x5d05,0x75);
	write_cmos_sensor(0x5d06,0x75);
	write_cmos_sensor(0x5d07,0x75);
	write_cmos_sensor(0x5d08,0x75);
	write_cmos_sensor(0x5d09,0x75);
	write_cmos_sensor(0x5d0a,0x75);
	write_cmos_sensor(0x5d0b,0x75);
	write_cmos_sensor(0x5d0c,0x75);
	write_cmos_sensor(0x5d0d,0x75);
	write_cmos_sensor(0x5d0e,0x75);
	write_cmos_sensor(0x5d0f,0x75);
	write_cmos_sensor(0x5d10,0x75);
	write_cmos_sensor(0x5d11,0x75);
	write_cmos_sensor(0x5d12,0x75);
	write_cmos_sensor(0x5d13,0x75);
	write_cmos_sensor(0x5d14,0x75);
	write_cmos_sensor(0x5d15,0x75);
	write_cmos_sensor(0x5d16,0x75);
	write_cmos_sensor(0x5d17,0x75);
	write_cmos_sensor(0x5d18,0x75);
	write_cmos_sensor(0x5d19,0x75);
	write_cmos_sensor(0x5d1a,0x75);
	write_cmos_sensor(0x5d1b,0x75);
	write_cmos_sensor(0x5d1c,0x75);
	write_cmos_sensor(0x5d1d,0x75);
	write_cmos_sensor(0x5d1e,0x75);
	write_cmos_sensor(0x5d1f,0x75);
	write_cmos_sensor(0x5d20,0x75);
	write_cmos_sensor(0x5d21,0x75);
	write_cmos_sensor(0x5d22,0x75);
	write_cmos_sensor(0x5d23,0x75);
	write_cmos_sensor(0x5d24,0x75);
	write_cmos_sensor(0x5d25,0x75);
	write_cmos_sensor(0x5d26,0x75);
	write_cmos_sensor(0x5d27,0x75);
	write_cmos_sensor(0x5d28,0x75);
	write_cmos_sensor(0x5d29,0x75);
	write_cmos_sensor(0x5d2a,0x75);
	write_cmos_sensor(0x5d2b,0x75);
	write_cmos_sensor(0x5d2c,0x75);
	write_cmos_sensor(0x5d2d,0x75);
	write_cmos_sensor(0x5d2e,0x75);
	write_cmos_sensor(0x5d2f,0x75);
	write_cmos_sensor(0x5d30,0x75);
	write_cmos_sensor(0x5d31,0x75);
	write_cmos_sensor(0x5d32,0x75);
	write_cmos_sensor(0x5d33,0x75);
	write_cmos_sensor(0x5d34,0x75);
	write_cmos_sensor(0x5d35,0x75);
	write_cmos_sensor(0x5d36,0x75);
	write_cmos_sensor(0x5d37,0x75);
	write_cmos_sensor(0x5d38,0x75);
	write_cmos_sensor(0x5d39,0x75);
	write_cmos_sensor(0x5d3a,0x75);
	write_cmos_sensor(0x5d3b,0x75);
	write_cmos_sensor(0x5d3c,0x75);
	write_cmos_sensor(0x5d3d,0x75);
	write_cmos_sensor(0x5d3e,0x75);
	write_cmos_sensor(0x5d3f,0x75);
	write_cmos_sensor(0x5d40,0x75);
	write_cmos_sensor(0x5d41,0x75);
	write_cmos_sensor(0x5d42,0x75);
	write_cmos_sensor(0x5d43,0x75);
	write_cmos_sensor(0x5d44,0x75);
	write_cmos_sensor(0x5d45,0x75);
	write_cmos_sensor(0x5d46,0x75);
	write_cmos_sensor(0x5d47,0x75);
	write_cmos_sensor(0x5d48,0x75);
	write_cmos_sensor(0x5d49,0x75);
	write_cmos_sensor(0x5d4a,0x75);
	write_cmos_sensor(0x5d4b,0x75);
	write_cmos_sensor(0x5d4c,0x75);
	write_cmos_sensor(0x5d4d,0x75);
	write_cmos_sensor(0x5d4e,0x75);
	write_cmos_sensor(0x5d4f,0x75);
	write_cmos_sensor(0x5d50,0x75);
	write_cmos_sensor(0x5d51,0x75);
	write_cmos_sensor(0x5d52,0x75);
	write_cmos_sensor(0x5d53,0x75);
	write_cmos_sensor(0x5d54,0x75);
	write_cmos_sensor(0x5d55,0x75);
	write_cmos_sensor(0x5d56,0x75);
	write_cmos_sensor(0x5d57,0x75);
	write_cmos_sensor(0x5d58,0x75);
	write_cmos_sensor(0x5d59,0x75);
	write_cmos_sensor(0x5d5a,0x75);
	write_cmos_sensor(0x5d5b,0x75);
	write_cmos_sensor(0x5d5c,0x75);
	write_cmos_sensor(0x5d5d,0x75);
	write_cmos_sensor(0x5d5e,0x75);
	write_cmos_sensor(0x5d5f,0x75);
	write_cmos_sensor(0x5d60,0x75);
	write_cmos_sensor(0x5d61,0x75);
	write_cmos_sensor(0x5d62,0x75);
	write_cmos_sensor(0x5d63,0x75);
	write_cmos_sensor(0x5d64,0x75);
	write_cmos_sensor(0x5d65,0x75);
	write_cmos_sensor(0x5d66,0x75);
	write_cmos_sensor(0x5d67,0x75);
	write_cmos_sensor(0x5d68,0x75);
	write_cmos_sensor(0x5d69,0x75);
	write_cmos_sensor(0x5d6a,0x75);
	write_cmos_sensor(0x5d6b,0x75);
	write_cmos_sensor(0x5d0c,0x75);
	write_cmos_sensor(0x5d6d,0x75);
	write_cmos_sensor(0x5d6e,0x75);
	write_cmos_sensor(0x5d6f,0x75);
	write_cmos_sensor(0x5d70,0x75);
	write_cmos_sensor(0x5d71,0x75);
	write_cmos_sensor(0x5d72,0x75);
	write_cmos_sensor(0x5d73,0x75);
	write_cmos_sensor(0x5d74,0x75);
	write_cmos_sensor(0x5d75,0x75);
	write_cmos_sensor(0x5d76,0x75);
	write_cmos_sensor(0x5d77,0x75);
	write_cmos_sensor(0x5d78,0x75);
	write_cmos_sensor(0x5d79,0x75);
	write_cmos_sensor(0x5d7a,0x75);
	write_cmos_sensor(0x5d7b,0x75);
	write_cmos_sensor(0x5d7c,0x75);
	write_cmos_sensor(0x5d7d,0x75);
	write_cmos_sensor(0x5d7e,0x75);
	write_cmos_sensor(0x5d7f,0x75);
	write_cmos_sensor(0x5d80,0x75);
	write_cmos_sensor(0x5d81,0x75);
	write_cmos_sensor(0x5d82,0x75);
	write_cmos_sensor(0x5d83,0x75);
	write_cmos_sensor(0x5d84,0x75);
	write_cmos_sensor(0x5d85,0x75);
	write_cmos_sensor(0x5d86,0x75);
	write_cmos_sensor(0x5d87,0x75);
	write_cmos_sensor(0x5d88,0x75);
	write_cmos_sensor(0x5d89,0x75);
	write_cmos_sensor(0x5d8a,0x75);
	write_cmos_sensor(0x5d8b,0x75);
	write_cmos_sensor(0x5d8c,0x75);
	write_cmos_sensor(0x5d8d,0x75);
	write_cmos_sensor(0x5d8e,0x75);
	write_cmos_sensor(0x5d8f,0x75);
	write_cmos_sensor(0x5d90,0x75);
	write_cmos_sensor(0x5d91,0x75);
	write_cmos_sensor(0x5d92,0x75);
	write_cmos_sensor(0x5d93,0x75);
	write_cmos_sensor(0x5d94,0x75);
	write_cmos_sensor(0x5d95,0x75);
	write_cmos_sensor(0x5d96,0x75);
	write_cmos_sensor(0x5d97,0x75);
	write_cmos_sensor(0x5d98,0x75);
	write_cmos_sensor(0x5d99,0x75);
	write_cmos_sensor(0x5d9a,0x75);
	write_cmos_sensor(0x5d9b,0x75);
	write_cmos_sensor(0x5d9c,0x75);
	write_cmos_sensor(0x5d9d,0x75);
	write_cmos_sensor(0x5d9e,0x75);
	write_cmos_sensor(0x5d9f,0x75);
	write_cmos_sensor(0x5da0,0x75);
	write_cmos_sensor(0x5da1,0x75);
	write_cmos_sensor(0x5da2,0x75);
	write_cmos_sensor(0x5da3,0x75);
	write_cmos_sensor(0x5da4,0x75);
	write_cmos_sensor(0x5da5,0x75);
	write_cmos_sensor(0x5da6,0x75);
	write_cmos_sensor(0x5da7,0x75);
	write_cmos_sensor(0x5da8,0x75);
	write_cmos_sensor(0x5da9,0x75);
	write_cmos_sensor(0x5daa,0x75);
	write_cmos_sensor(0x5dab,0x75);
	write_cmos_sensor(0x5dac,0x75);
	write_cmos_sensor(0x5dad,0x75);
	write_cmos_sensor(0x5dae,0x75);
	write_cmos_sensor(0x5daf,0x75);
	write_cmos_sensor(0x5db0,0x75);
	write_cmos_sensor(0x5db1,0x75);
	write_cmos_sensor(0x5db2,0x75);
	write_cmos_sensor(0x5db3,0x75);
	write_cmos_sensor(0x5db4,0x75);
	write_cmos_sensor(0x5db5,0x75);
	write_cmos_sensor(0x5db6,0x75);
	write_cmos_sensor(0x5db7,0x75);
	write_cmos_sensor(0x5db8,0x75);
	write_cmos_sensor(0x5db9,0x75);
	write_cmos_sensor(0x5dba,0x75);
	write_cmos_sensor(0x5dbb,0x75);
	write_cmos_sensor(0x5dbc,0x75);
	write_cmos_sensor(0x5dbd,0x75);
	write_cmos_sensor(0x5dbe,0x75);
	write_cmos_sensor(0x5dbf,0x75);
	write_cmos_sensor(0x5dc0,0x75);
	write_cmos_sensor(0x5dc1,0x75);
	write_cmos_sensor(0x5dc2,0x75);
	write_cmos_sensor(0x5dc3,0x75);
	write_cmos_sensor(0x5dc4,0x75);
	write_cmos_sensor(0x5dc5,0x75);
	write_cmos_sensor(0x5dc6,0x75);
	write_cmos_sensor(0x5dc7,0x75);
	write_cmos_sensor(0x5dc8,0x75);
	write_cmos_sensor(0x5dc9,0x75);
	write_cmos_sensor(0x5dca,0x75);
	write_cmos_sensor(0x5dcb,0x75);
	write_cmos_sensor(0x5dcc,0x75);
	write_cmos_sensor(0x5dcd,0x75);
	write_cmos_sensor(0x5dce,0x75);
	write_cmos_sensor(0x5dcf,0x75);
	write_cmos_sensor(0x5dd0,0x75);
	write_cmos_sensor(0x5dd1,0x75);
	write_cmos_sensor(0x5dd2,0x75);
	write_cmos_sensor(0x5dd3,0x75);
	write_cmos_sensor(0x5dd4,0x75);
	write_cmos_sensor(0x5dd5,0x75);
	write_cmos_sensor(0x5dd6,0x75);
	write_cmos_sensor(0x5dd7,0x75);
	write_cmos_sensor(0x5dd8,0x75);
	write_cmos_sensor(0x5dd9,0x75);
	write_cmos_sensor(0x5dda,0x75);
	write_cmos_sensor(0x5ddb,0x75);
	write_cmos_sensor(0x5ddc,0x75);
	write_cmos_sensor(0x5ddd,0x75);
	write_cmos_sensor(0x5dde,0x75);
	write_cmos_sensor(0x5ddf,0x75);
	write_cmos_sensor(0x5de0,0x75);
	write_cmos_sensor(0x5de1,0x75);
	write_cmos_sensor(0x5de2,0x75);
	write_cmos_sensor(0x5de3,0x75);
	write_cmos_sensor(0x5de4,0x75);
	write_cmos_sensor(0x5de5,0x75);
	write_cmos_sensor(0x5de6,0x75);
	write_cmos_sensor(0x5de7,0x75);
	write_cmos_sensor(0x5de8,0x75);
	write_cmos_sensor(0x5de9,0x75);
	write_cmos_sensor(0x5dea,0x75);
	write_cmos_sensor(0x5deb,0x75);
	write_cmos_sensor(0x5dec,0x75);
	write_cmos_sensor(0x5ded,0x75);
	write_cmos_sensor(0x5dee,0x75);
	write_cmos_sensor(0x5def,0x75);
	write_cmos_sensor(0x5df0,0x75);
	write_cmos_sensor(0x5df1,0x75);
	write_cmos_sensor(0x5df2,0x75);
	write_cmos_sensor(0x5df3,0x75);
	write_cmos_sensor(0x5df4,0x75);
	write_cmos_sensor(0x5df5,0x75);
	write_cmos_sensor(0x5df6,0x75);
	write_cmos_sensor(0x5df7,0x75);
	write_cmos_sensor(0x5df8,0x75);
	write_cmos_sensor(0x5df9,0x75);
	write_cmos_sensor(0x5dfa,0x75);
	write_cmos_sensor(0x5dfb,0x75);
	write_cmos_sensor(0x5dfc,0x75);
	write_cmos_sensor(0x5dfd,0x75);
	write_cmos_sensor(0x5dfe,0x75);
	write_cmos_sensor(0x5dff,0x75);
	write_cmos_sensor(0x5e00,0x75);
	write_cmos_sensor(0x5e01,0x75);
	write_cmos_sensor(0x5e02,0x75);
	write_cmos_sensor(0x5e03,0x75);
	write_cmos_sensor(0x5e04,0x75);
	write_cmos_sensor(0x5e05,0x75);
	write_cmos_sensor(0x5e06,0x75);
	write_cmos_sensor(0x5e07,0x75);
	write_cmos_sensor(0x5e08,0x75);
	write_cmos_sensor(0x5e09,0x75);
	write_cmos_sensor(0x5e0a,0x75);
	write_cmos_sensor(0x5e0b,0x75);
	write_cmos_sensor(0x5e0c,0x75);
	write_cmos_sensor(0x5e0d,0x75);
	write_cmos_sensor(0x5e0e,0x75);
	write_cmos_sensor(0x5e0f,0x75);
	write_cmos_sensor(0x5e10,0x75);
	write_cmos_sensor(0x5e11,0x75);
	write_cmos_sensor(0x5e12,0x75);
	write_cmos_sensor(0x5e13,0x75);
	write_cmos_sensor(0x5e14,0x75);
	write_cmos_sensor(0x5e15,0x75);
	write_cmos_sensor(0x5e16,0x75);
	write_cmos_sensor(0x5e17,0x75);
	write_cmos_sensor(0x5e18,0x75);
	write_cmos_sensor(0x5e19,0x75);
	write_cmos_sensor(0x5e1a,0x75);
	write_cmos_sensor(0x5e1b,0x75);
	write_cmos_sensor(0x5e1c,0x75);
	write_cmos_sensor(0x5e1d,0x75);
	write_cmos_sensor(0x5e1e,0x75);
	write_cmos_sensor(0x5e1f,0x75);
	write_cmos_sensor(0x5e20,0x75);
	write_cmos_sensor(0x5e21,0x75);
	write_cmos_sensor(0x5e22,0x75);
	write_cmos_sensor(0x5e23,0x75);
	write_cmos_sensor(0x5e24,0x75);
	write_cmos_sensor(0x5e25,0x75);
	write_cmos_sensor(0x5e26,0x75);
	write_cmos_sensor(0x5e27,0x75);
	write_cmos_sensor(0x5e28,0x75);
	write_cmos_sensor(0x5e29,0x75);
	write_cmos_sensor(0x5e2a,0x75);
	write_cmos_sensor(0x5e2b,0x75);
	write_cmos_sensor(0x5e2c,0x75);
	write_cmos_sensor(0x5e2d,0x75);
	write_cmos_sensor(0x5e2e,0x75);
	write_cmos_sensor(0x5e2f,0x75);
	write_cmos_sensor(0x5e30,0x75);
	write_cmos_sensor(0x5e31,0x75);
	write_cmos_sensor(0x5e32,0x75);
	write_cmos_sensor(0x5e33,0x75);
	write_cmos_sensor(0x5e34,0x75);
	write_cmos_sensor(0x5e35,0x75);
	write_cmos_sensor(0x5e36,0x75);
	write_cmos_sensor(0x5e37,0x75);
	write_cmos_sensor(0x5e38,0x75);
	write_cmos_sensor(0x5e39,0x75);
	write_cmos_sensor(0x5e3a,0x75);
	write_cmos_sensor(0x5e3b,0x75);
	write_cmos_sensor(0x5e3c,0x75);
	write_cmos_sensor(0x5e3d,0x75);
	write_cmos_sensor(0x5e3e,0x75);
	write_cmos_sensor(0x5e3f,0x75);
	write_cmos_sensor(0x5e40,0x75);
	write_cmos_sensor(0x5e41,0x75);
	write_cmos_sensor(0x5e42,0x75);
	write_cmos_sensor(0x5e43,0x75);
	write_cmos_sensor(0x5e44,0x75);
	write_cmos_sensor(0x5e45,0x75);
	write_cmos_sensor(0x5e46,0x75);
	write_cmos_sensor(0x5e47,0x75);
	write_cmos_sensor(0x5e48,0x75);
	write_cmos_sensor(0x5e49,0x75);
	write_cmos_sensor(0x5e4a,0x75);
	write_cmos_sensor(0x5e4b,0x75);
	write_cmos_sensor(0x5e4c,0x75);
	write_cmos_sensor(0x5e4d,0x75);
	write_cmos_sensor(0x5e4e,0x75);
	write_cmos_sensor(0x5e4f,0x75);
	write_cmos_sensor(0x5e50,0x75);
	write_cmos_sensor(0x5e51,0x75);
	write_cmos_sensor(0x5e52,0x75);
	write_cmos_sensor(0x5e53,0x75);
	write_cmos_sensor(0x5e54,0x75);
	write_cmos_sensor(0x5e55,0x75);
	write_cmos_sensor(0x5e56,0x75);
	write_cmos_sensor(0x5e57,0x75);
	write_cmos_sensor(0x5e58,0x75);
	write_cmos_sensor(0x5e59,0x75);
	write_cmos_sensor(0x5e5a,0x75);
	write_cmos_sensor(0x5e5b,0x75);
	write_cmos_sensor(0x5e5c,0x75);
	write_cmos_sensor(0x5e5d,0x75);
	write_cmos_sensor(0x5e5e,0x75);
	write_cmos_sensor(0x5e5f,0x75);
	write_cmos_sensor(0x5e60,0x75);
	write_cmos_sensor(0x5e61,0x75);
	write_cmos_sensor(0x5e62,0x75);
	write_cmos_sensor(0x5e63,0x75);
	write_cmos_sensor(0x5e64,0x75);
	write_cmos_sensor(0x5e65,0x75);
	write_cmos_sensor(0x5e66,0x75);
	write_cmos_sensor(0x5e67,0x75);
	write_cmos_sensor(0x5e68,0x75);
	write_cmos_sensor(0x5e69,0x75);
	write_cmos_sensor(0x5e6a,0x75);
	write_cmos_sensor(0x5e6b,0x75);
	write_cmos_sensor(0x5e0c,0x75);
	write_cmos_sensor(0x5e6d,0x75);
	write_cmos_sensor(0x5e6e,0x75);
	write_cmos_sensor(0x5e6f,0x75);
	write_cmos_sensor(0x5e70,0x75);
	write_cmos_sensor(0x5e71,0x75);
	write_cmos_sensor(0x5e72,0x75);
	write_cmos_sensor(0x5e73,0x75);
	write_cmos_sensor(0x5e74,0x75);
	write_cmos_sensor(0x5e75,0x75);
	write_cmos_sensor(0x5e76,0x75);
	write_cmos_sensor(0x5e77,0x75);
	write_cmos_sensor(0x5e78,0x75);
	write_cmos_sensor(0x5e79,0x75);
	write_cmos_sensor(0x5e7a,0x75);
	write_cmos_sensor(0x5e7b,0x75);
	write_cmos_sensor(0x5e7c,0x75);
	write_cmos_sensor(0x5e7d,0x75);
	write_cmos_sensor(0x5e7e,0x75);
	write_cmos_sensor(0x5e7f,0x75);
	write_cmos_sensor(0x5e80,0x75);
	write_cmos_sensor(0x5e81,0x75);
	write_cmos_sensor(0x5e82,0x75);
	write_cmos_sensor(0x5e83,0x75);
	write_cmos_sensor(0x5e84,0x75);
	write_cmos_sensor(0x5e85,0x75);
	write_cmos_sensor(0x5e86,0x75);
	write_cmos_sensor(0x5e87,0x75);
	write_cmos_sensor(0x5e88,0x75);
	write_cmos_sensor(0x5e89,0x75);
	write_cmos_sensor(0x5e8a,0x75);
	write_cmos_sensor(0x5e8b,0x75);
	write_cmos_sensor(0x5e8c,0x75);
	write_cmos_sensor(0x5e8d,0x75);
	write_cmos_sensor(0x5e8e,0x75);
	write_cmos_sensor(0x5e8f,0x75);
	write_cmos_sensor(0x5e90,0x75);
	write_cmos_sensor(0x5e91,0x75);
	write_cmos_sensor(0x5e92,0x75);
	write_cmos_sensor(0x5e93,0x75);
	write_cmos_sensor(0x5e94,0x75);
	write_cmos_sensor(0x5e95,0x75);
	write_cmos_sensor(0x5e96,0x75);
	write_cmos_sensor(0x5e97,0x75);
	write_cmos_sensor(0x5e98,0x75);
	write_cmos_sensor(0x5e99,0x75);
	write_cmos_sensor(0x5e9a,0x75);
	write_cmos_sensor(0x5e9b,0x75);
	write_cmos_sensor(0x5e9c,0x75);
	write_cmos_sensor(0x5e9d,0x75);
	write_cmos_sensor(0x5e9e,0x75);
	write_cmos_sensor(0x5e9f,0x75);
	write_cmos_sensor(0x5ea0,0x75);
	write_cmos_sensor(0x5ea1,0x75);
	write_cmos_sensor(0x5ea2,0x75);
	write_cmos_sensor(0x5ea3,0x75);
	write_cmos_sensor(0x5ea4,0x75);
	write_cmos_sensor(0x5ea5,0x75);
	write_cmos_sensor(0x5ea6,0x75);
	write_cmos_sensor(0x5ea7,0x75);
	write_cmos_sensor(0x5ea8,0x75);
	write_cmos_sensor(0x5ea9,0x75);
	write_cmos_sensor(0x5eaa,0x75);
	write_cmos_sensor(0x5eab,0x75);
	write_cmos_sensor(0x5eac,0x75);
	write_cmos_sensor(0x5ead,0x75);
	write_cmos_sensor(0x5eae,0x75);
	write_cmos_sensor(0x5eaf,0x75);
	write_cmos_sensor(0x5eb0,0x75);
	write_cmos_sensor(0x5eb1,0x75);
	write_cmos_sensor(0x5eb2,0x75);
	write_cmos_sensor(0x5eb3,0x75);
	write_cmos_sensor(0x5eb4,0x75);
	write_cmos_sensor(0x5eb5,0x75);
	write_cmos_sensor(0x5eb6,0x75);
	write_cmos_sensor(0x5eb7,0x75);
	write_cmos_sensor(0x5eb8,0x75);
	write_cmos_sensor(0x5eb9,0x75);
	write_cmos_sensor(0x5eba,0x75);
	write_cmos_sensor(0x5ebb,0x75);
	write_cmos_sensor(0x5ebc,0x75);
	write_cmos_sensor(0x5ebd,0x75);
	write_cmos_sensor(0x5ebe,0x75);
	write_cmos_sensor(0x5ebf,0x75);
	write_cmos_sensor(0x5ec0,0x75);
	write_cmos_sensor(0x5ec1,0x75);
	write_cmos_sensor(0x5ec2,0x75);
	write_cmos_sensor(0x5ec3,0x75);
	write_cmos_sensor(0x5ec4,0x75);
	write_cmos_sensor(0x5ec5,0x75);
	write_cmos_sensor(0x5ec6,0x75);
	write_cmos_sensor(0x5ec7,0x75);
	write_cmos_sensor(0x5ec8,0x75);
	write_cmos_sensor(0x5ec9,0x75);
	write_cmos_sensor(0x5eca,0x75);
	write_cmos_sensor(0x5ecb,0x75);
	write_cmos_sensor(0x5ecc,0x75);
	write_cmos_sensor(0x5ecd,0x75);
	write_cmos_sensor(0x5ece,0x75);
	write_cmos_sensor(0x5ecf,0x75);
	write_cmos_sensor(0x5ed0,0x75);
	write_cmos_sensor(0x5ed1,0x75);
	write_cmos_sensor(0x5ed2,0x75);
	write_cmos_sensor(0x5ed3,0x75);
	write_cmos_sensor(0x5ed4,0x75);
	write_cmos_sensor(0x5ed5,0x75);
	write_cmos_sensor(0x5ed6,0x75);
	write_cmos_sensor(0x5ed7,0x75);
	write_cmos_sensor(0x5ed8,0x75);
	write_cmos_sensor(0x5ed9,0x75);
	write_cmos_sensor(0x5eda,0x75);
	write_cmos_sensor(0x5edb,0x75);
	write_cmos_sensor(0x5edc,0x75);
	write_cmos_sensor(0x5edd,0x75);
	write_cmos_sensor(0x5ede,0x75);
	write_cmos_sensor(0x5edf,0x75);
	write_cmos_sensor(0x0301,0x48);
	write_cmos_sensor(0x0304,0x01);
	write_cmos_sensor(0x0305,0xa1);
	write_cmos_sensor(0x0306,0x04);
	write_cmos_sensor(0x0307,0x00);
	write_cmos_sensor(0x0309,0x51);
	write_cmos_sensor(0x0316,0x33);
	write_cmos_sensor(0x0320,0x02);
	write_cmos_sensor(0x0321,0x01);
	write_cmos_sensor(0x0323,0x03);
	write_cmos_sensor(0x0324,0x01);
	write_cmos_sensor(0x0325,0xe0);
	write_cmos_sensor(0x0326,0xc4);
	write_cmos_sensor(0x0327,0x04);
	write_cmos_sensor(0x0329,0x00);
	write_cmos_sensor(0x032c,0x00);
	write_cmos_sensor(0x032d,0x01);
	write_cmos_sensor(0x032e,0x02);
	write_cmos_sensor(0x032f,0xc1);
	write_cmos_sensor(0x0350,0x00);
	write_cmos_sensor(0x0360,0x09);
	write_cmos_sensor(0x0361,0x07);
	write_cmos_sensor(0x3009,0x00);
	write_cmos_sensor(0x3012,0x41);
	write_cmos_sensor(0x3015,0x00);
	write_cmos_sensor(0x3019,0xd2);
	write_cmos_sensor(0x301a,0xb0);
	write_cmos_sensor(0x301e,0x88);
	write_cmos_sensor(0x3025,0x89);
	write_cmos_sensor(0x3026,0x30);
	write_cmos_sensor(0x3027,0x08);
	write_cmos_sensor(0x3044,0xc2);
	write_cmos_sensor(0x3400,0x0c);
	write_cmos_sensor(0x340c,0x10);
	write_cmos_sensor(0x340d,0x00);
	write_cmos_sensor(0x340e,0xff);
	write_cmos_sensor(0x3421,0x08);
	write_cmos_sensor(0x3422,0x00);
	write_cmos_sensor(0x3423,0x00);
	write_cmos_sensor(0x3424,0x15);
	write_cmos_sensor(0x3425,0x40);
	write_cmos_sensor(0x3426,0x10);
	write_cmos_sensor(0x3427,0x10);
	write_cmos_sensor(0x3500,0x00);
	write_cmos_sensor(0x3501,0x14);
	write_cmos_sensor(0x3502,0xb4);
	write_cmos_sensor(0x3504,0x48);
	write_cmos_sensor(0x3506,0x20);
	write_cmos_sensor(0x3507,0x00);
	write_cmos_sensor(0x3508,0x08);
	write_cmos_sensor(0x3509,0x00);
	write_cmos_sensor(0x3540,0x00);
	write_cmos_sensor(0x3541,0x00);
	write_cmos_sensor(0x3542,0x10);
	write_cmos_sensor(0x3546,0x20);
	write_cmos_sensor(0x3548,0x01);
	write_cmos_sensor(0x3549,0x00);
	write_cmos_sensor(0x3580,0x00);
	write_cmos_sensor(0x3581,0x00);
	write_cmos_sensor(0x3582,0x08);
	write_cmos_sensor(0x3586,0x20);
	write_cmos_sensor(0x3588,0x01);
	write_cmos_sensor(0x3589,0x00);
	write_cmos_sensor(0x3605,0xd0);
	write_cmos_sensor(0x360a,0x5d);
	write_cmos_sensor(0x360b,0xc0);
	write_cmos_sensor(0x360c,0x9e);
	write_cmos_sensor(0x360d,0x25);
	write_cmos_sensor(0x360e,0x08);
	write_cmos_sensor(0x3618,0x80);
	write_cmos_sensor(0x3622,0x0a);
	write_cmos_sensor(0x3623,0x08);
	write_cmos_sensor(0x3624,0x14);
	write_cmos_sensor(0x3628,0x99);
	write_cmos_sensor(0x362b,0x77);
	write_cmos_sensor(0x362d,0x0a);
	write_cmos_sensor(0x3635,0xcb);
	write_cmos_sensor(0x3659,0x6a);
	write_cmos_sensor(0x3684,0x81);
	write_cmos_sensor(0x3688,0x02);
	write_cmos_sensor(0x3689,0x88);
	write_cmos_sensor(0x368a,0x2e);
	write_cmos_sensor(0x368e,0x71);
	write_cmos_sensor(0x3694,0x7f);
	write_cmos_sensor(0x3695,0x00);
	write_cmos_sensor(0x3696,0x51);
	write_cmos_sensor(0x369a,0x00);
	write_cmos_sensor(0x36a4,0x00);
	write_cmos_sensor(0x36a6,0x00);
	write_cmos_sensor(0x36a7,0x00);
	write_cmos_sensor(0x36a8,0x00);
	write_cmos_sensor(0x3700,0x2c);
	write_cmos_sensor(0x3701,0x30);
	write_cmos_sensor(0x3702,0x4b);
	write_cmos_sensor(0x3703,0x42);
	write_cmos_sensor(0x3706,0x31);
	write_cmos_sensor(0x3708,0x3a);
	write_cmos_sensor(0x3709,0xf6);
	write_cmos_sensor(0x370b,0x6a);
	write_cmos_sensor(0x3711,0x00);
	write_cmos_sensor(0x3712,0x51);
	write_cmos_sensor(0x3713,0x00);
	write_cmos_sensor(0x3714,0x6f);
	write_cmos_sensor(0x3716,0x40);
	write_cmos_sensor(0x3717,0x02);
	write_cmos_sensor(0x371d,0x24);
	write_cmos_sensor(0x371e,0x13);
	write_cmos_sensor(0x371f,0x0c);
	write_cmos_sensor(0x3720,0x08);
	write_cmos_sensor(0x3721,0x15);
	write_cmos_sensor(0x3725,0x32);
	write_cmos_sensor(0x3730,0x00);
	write_cmos_sensor(0x3731,0x00);
	write_cmos_sensor(0x3732,0x00);
	write_cmos_sensor(0x3734,0x20);
	write_cmos_sensor(0x3736,0x04);
	write_cmos_sensor(0x3743,0x20);
	write_cmos_sensor(0x3745,0xff);
	write_cmos_sensor(0x3753,0x21);
	write_cmos_sensor(0x3754,0x14);
	write_cmos_sensor(0x3755,0x0c);
	write_cmos_sensor(0x375e,0x00);
	write_cmos_sensor(0x375f,0x82);
	write_cmos_sensor(0x3760,0x08);
	write_cmos_sensor(0x3761,0x10);
	write_cmos_sensor(0x3762,0x08);
	write_cmos_sensor(0x3763,0x08);
	write_cmos_sensor(0x3764,0x08);
	write_cmos_sensor(0x3765,0x10);
	write_cmos_sensor(0x3766,0x18);
	write_cmos_sensor(0x3767,0x28);
	write_cmos_sensor(0x3768,0x00);
	write_cmos_sensor(0x3769,0x08);
	write_cmos_sensor(0x376b,0x00);
	write_cmos_sensor(0x3770,0x0d);
	write_cmos_sensor(0x3774,0x0e);
	write_cmos_sensor(0x3791,0x31);
	write_cmos_sensor(0x3793,0x6a);
	write_cmos_sensor(0x3795,0x00);
	write_cmos_sensor(0x3797,0x00);
	write_cmos_sensor(0x3799,0x0d);
	write_cmos_sensor(0x379a,0x0d);
	write_cmos_sensor(0x379b,0x0d);
	write_cmos_sensor(0x37a4,0x0d);
	write_cmos_sensor(0x37ff,0x00);
	write_cmos_sensor(0x3800,0x00);
	write_cmos_sensor(0x3801,0x00);
	write_cmos_sensor(0x3802,0x00);
	write_cmos_sensor(0x3803,0x00);
	write_cmos_sensor(0x3804,0x19);
	write_cmos_sensor(0x3805,0x9f);
	write_cmos_sensor(0x3806,0x13);
	write_cmos_sensor(0x3807,0x3f);
	write_cmos_sensor(0x3808,0x19);
	write_cmos_sensor(0x3809,0x80);
	write_cmos_sensor(0x380a,0x13);
	write_cmos_sensor(0x380b,0x20);
	write_cmos_sensor(0x380c,0x05);
	write_cmos_sensor(0x380d,0xa0);
	write_cmos_sensor(0x380e,0x14);
	write_cmos_sensor(0x380f,0xd4);
	write_cmos_sensor(0x3810,0x00);
	write_cmos_sensor(0x3811,0x11);
	write_cmos_sensor(0x3812,0x00);
	write_cmos_sensor(0x3813,0x10);
	write_cmos_sensor(0x3814,0x11);
	write_cmos_sensor(0x3815,0x11);
	write_cmos_sensor(0x3820,0x00);
	write_cmos_sensor(0x3821,0x06);
	write_cmos_sensor(0x3822,0x00);
	write_cmos_sensor(0x3823,0x04);
	write_cmos_sensor(0x3824,0x00);
	write_cmos_sensor(0x3825,0x00);
	write_cmos_sensor(0x3826,0x00);
	write_cmos_sensor(0x3827,0x40);
	write_cmos_sensor(0x3828,0x0f);
	write_cmos_sensor(0x382a,0x82);
	write_cmos_sensor(0x382e,0x41);
	write_cmos_sensor(0x3835,0x00);
	write_cmos_sensor(0x3836,0x00);
	write_cmos_sensor(0x3837,0x1f);
	write_cmos_sensor(0x383a,0x00);
	write_cmos_sensor(0x383b,0x00);
	write_cmos_sensor(0x383e,0x00);
	write_cmos_sensor(0x3840,0x00);
	write_cmos_sensor(0x3847,0x00);
	write_cmos_sensor(0x384a,0x00);
	write_cmos_sensor(0x384c,0x05);
	write_cmos_sensor(0x384d,0xa0);
	write_cmos_sensor(0x3856,0x50);
	write_cmos_sensor(0x3857,0x30);
	write_cmos_sensor(0x3858,0x80);
	write_cmos_sensor(0x3859,0x40);
	write_cmos_sensor(0x3865,0x00);
	write_cmos_sensor(0x3888,0x00);
	write_cmos_sensor(0x3889,0x00);
	write_cmos_sensor(0x388a,0x00);
	write_cmos_sensor(0x388b,0x00);
	write_cmos_sensor(0x388c,0x00);
	write_cmos_sensor(0x388d,0x00);
	write_cmos_sensor(0x388e,0x00);
	write_cmos_sensor(0x388f,0x00);
	write_cmos_sensor(0x3894,0x00);
	write_cmos_sensor(0x390c,0x3f);
	write_cmos_sensor(0x390f,0x57);
	write_cmos_sensor(0x3911,0x3c);
	write_cmos_sensor(0x3914,0x40);
	write_cmos_sensor(0x3920,0x80);
	write_cmos_sensor(0x3921,0x00);
	write_cmos_sensor(0x3922,0x00);
	write_cmos_sensor(0x3924,0xa5);
	write_cmos_sensor(0x3973,0x06);
	write_cmos_sensor(0x3976,0x06);
	write_cmos_sensor(0x3978,0x06);
	write_cmos_sensor(0x397b,0x06);
	write_cmos_sensor(0x3994,0xee);
	write_cmos_sensor(0x399b,0x04);
	write_cmos_sensor(0x39cd,0x18);
	write_cmos_sensor(0x39ce,0x48);
	write_cmos_sensor(0x39cf,0x48);
	write_cmos_sensor(0x39d0,0x48);
	write_cmos_sensor(0x39d2,0x1b);
	write_cmos_sensor(0x39d3,0x4b);
	write_cmos_sensor(0x39d4,0x4b);
	write_cmos_sensor(0x39d5,0x4b);
	write_cmos_sensor(0x39f5,0x04);
	write_cmos_sensor(0x39f6,0x0e);
	write_cmos_sensor(0x39fd,0x01);
	write_cmos_sensor(0x39ff,0x00);
	write_cmos_sensor(0x3a01,0x03);
	write_cmos_sensor(0x3a03,0x00);
	write_cmos_sensor(0x3a05,0x05);
	write_cmos_sensor(0x3a07,0x07);
	write_cmos_sensor(0x3a2d,0x20);
	write_cmos_sensor(0x3a2e,0x20);
	write_cmos_sensor(0x3a2f,0x20);
	write_cmos_sensor(0x3a30,0x20);
	write_cmos_sensor(0x3a31,0x20);
	write_cmos_sensor(0x3a32,0x20);
	write_cmos_sensor(0x3a33,0x20);
	write_cmos_sensor(0x3a34,0x20);
	write_cmos_sensor(0x3a35,0x55);
	write_cmos_sensor(0x3a43,0x11);
	write_cmos_sensor(0x3a44,0x0a);
	write_cmos_sensor(0x3a45,0x06);
	write_cmos_sensor(0x3a46,0x02);
	write_cmos_sensor(0x3a47,0x11);
	write_cmos_sensor(0x3a48,0x0a);
	write_cmos_sensor(0x3a49,0x06);
	write_cmos_sensor(0x3a4a,0x02);
	write_cmos_sensor(0x3a4b,0x07);
	write_cmos_sensor(0x3a4c,0x08);
	write_cmos_sensor(0x3a4d,0x08);
	write_cmos_sensor(0x3a4e,0x08);
	write_cmos_sensor(0x3a4f,0x08);
	write_cmos_sensor(0x3a50,0x08);
	write_cmos_sensor(0x3a51,0x08);
	write_cmos_sensor(0x3a57,0x00);
	write_cmos_sensor(0x3a5a,0x00);
	write_cmos_sensor(0x3a5f,0x00);
	write_cmos_sensor(0x3a60,0x00);
	write_cmos_sensor(0x3a61,0x00);
	write_cmos_sensor(0x3a62,0x00);
	write_cmos_sensor(0x3a63,0x00);
	write_cmos_sensor(0x3a64,0x00);
	write_cmos_sensor(0x3a65,0x00);
	write_cmos_sensor(0x3a66,0x00);
	write_cmos_sensor(0x3a67,0x00);
	write_cmos_sensor(0x3a68,0x00);
	write_cmos_sensor(0x3a69,0x00);
	write_cmos_sensor(0x3a6a,0x00);
	write_cmos_sensor(0x3a6b,0x00);
	write_cmos_sensor(0x3a0c,0x00);
	write_cmos_sensor(0x3a6d,0x00);
	write_cmos_sensor(0x3a6e,0x00);
	write_cmos_sensor(0x3a7d,0xf0);
	write_cmos_sensor(0x3a7f,0x00);
	write_cmos_sensor(0x3a80,0x0e);
	write_cmos_sensor(0x3a82,0x00);
	write_cmos_sensor(0x3a85,0x00);
	write_cmos_sensor(0x3a86,0x00);
	write_cmos_sensor(0x3a8c,0x0c);
	write_cmos_sensor(0x3aa3,0x14);
	write_cmos_sensor(0x3aa4,0x12);
	write_cmos_sensor(0x3aa5,0x0f);
	write_cmos_sensor(0x3aa6,0x08);
	write_cmos_sensor(0x3aad,0xff);
	write_cmos_sensor(0x3d85,0x8b);
	write_cmos_sensor(0x3d8c,0x75);
	write_cmos_sensor(0x3d8d,0xa0);
	write_cmos_sensor(0x3daa,0x70);
	write_cmos_sensor(0x3dab,0x10);
	write_cmos_sensor(0x3dac,0x74);
	write_cmos_sensor(0x3dad,0xe8);
	write_cmos_sensor(0x3dae,0x75);
	write_cmos_sensor(0x3daf,0x67);
	write_cmos_sensor(0x3f00,0x10);
	write_cmos_sensor(0x4009,0x02);
	write_cmos_sensor(0x4010,0xe8);
	write_cmos_sensor(0x4011,0x81);
	write_cmos_sensor(0x4012,0x0c);
	write_cmos_sensor(0x4015,0x02);
	write_cmos_sensor(0x4016,0x1d);
	write_cmos_sensor(0x4017,0x00);
	write_cmos_sensor(0x4018,0x0f);
	write_cmos_sensor(0x401a,0x40);
	write_cmos_sensor(0x401e,0x00);
	write_cmos_sensor(0x401f,0xf4);
	write_cmos_sensor(0x4020,0x04);
	write_cmos_sensor(0x4021,0x00);
	write_cmos_sensor(0x4022,0x04);
	write_cmos_sensor(0x4023,0x00);
	write_cmos_sensor(0x4024,0x04);
	write_cmos_sensor(0x4025,0x00);
	write_cmos_sensor(0x4026,0x04);
	write_cmos_sensor(0x4027,0x00);
	write_cmos_sensor(0x402e,0x02);
	write_cmos_sensor(0x4288,0xa7);
	write_cmos_sensor(0x4289,0x00);
	write_cmos_sensor(0x428e,0x04);
	write_cmos_sensor(0x4300,0x00);
	write_cmos_sensor(0x4301,0x00);
	write_cmos_sensor(0x4302,0x00);
	write_cmos_sensor(0x4303,0x00);
	write_cmos_sensor(0x4304,0x00);
	write_cmos_sensor(0x4305,0x00);
	write_cmos_sensor(0x4306,0x00);
	write_cmos_sensor(0x4307,0x00);
	write_cmos_sensor(0x4308,0x00);
	write_cmos_sensor(0x430b,0xff);
	write_cmos_sensor(0x430c,0xff);
	write_cmos_sensor(0x430d,0x00);
	write_cmos_sensor(0x430e,0x00);
	write_cmos_sensor(0x4500,0x07);
	write_cmos_sensor(0x4502,0x00);
	write_cmos_sensor(0x4504,0x80);
	write_cmos_sensor(0x4506,0x01);
	write_cmos_sensor(0x4509,0x05);
	write_cmos_sensor(0x450c,0x00);
	write_cmos_sensor(0x450d,0x20);
	write_cmos_sensor(0x450e,0x00);
	write_cmos_sensor(0x450f,0x00);
	write_cmos_sensor(0x4510,0x00);
	write_cmos_sensor(0x4523,0x00);
	write_cmos_sensor(0x4526,0x00);
	write_cmos_sensor(0x4542,0x00);
	write_cmos_sensor(0x4543,0x00);
	write_cmos_sensor(0x4544,0x00);
	write_cmos_sensor(0x4545,0x00);
	write_cmos_sensor(0x4546,0x00);
	write_cmos_sensor(0x4547,0x10);
	write_cmos_sensor(0x4602,0x00);
	write_cmos_sensor(0x4603,0x15);
	write_cmos_sensor(0x4609,0x20);
	write_cmos_sensor(0x460b,0x03);
	write_cmos_sensor(0x4640,0x00);
	write_cmos_sensor(0x4641,0x32);
	write_cmos_sensor(0x4643,0x08);
	write_cmos_sensor(0x4644,0x40);
	write_cmos_sensor(0x4645,0xb3);
	write_cmos_sensor(0x464a,0x00);
	write_cmos_sensor(0x464b,0x30);
	write_cmos_sensor(0x464c,0x01);
	write_cmos_sensor(0x4800,0x64);
	write_cmos_sensor(0x480b,0x10);
	write_cmos_sensor(0x480c,0x80);
	write_cmos_sensor(0x480e,0x00);
	write_cmos_sensor(0x480f,0x32);
	write_cmos_sensor(0x481f,0x30);
	write_cmos_sensor(0x4829,0x64);
	write_cmos_sensor(0x4833,0x20);
	write_cmos_sensor(0x4837,0x08);
	write_cmos_sensor(0x484b,0x27);
	write_cmos_sensor(0x4850,0x47);
	write_cmos_sensor(0x4853,0x04);
	write_cmos_sensor(0x4854,0x07);
	write_cmos_sensor(0x4860,0x00);
	write_cmos_sensor(0x4861,0xec);
	write_cmos_sensor(0x4862,0x05);
	write_cmos_sensor(0x4883,0x00);
	write_cmos_sensor(0x4885,0x3d);
	write_cmos_sensor(0x4888,0x10);
	write_cmos_sensor(0x4889,0x03);
	write_cmos_sensor(0x4d00,0x04);
	write_cmos_sensor(0x4d01,0xea);
	write_cmos_sensor(0x4d02,0xb9);
	write_cmos_sensor(0x4d03,0x32);
	write_cmos_sensor(0x4d04,0xfb);
	write_cmos_sensor(0x4d05,0x50);
	write_cmos_sensor(0x3220,0x12);
	write_cmos_sensor(0x3221,0x28);
	write_cmos_sensor(0x36aa,0x12);
	write_cmos_sensor(0x36ab,0x28);
	write_cmos_sensor(0x3d86,0x12);
	write_cmos_sensor(0x3d87,0x28);
	write_cmos_sensor(0x40c2,0x12);
	write_cmos_sensor(0x40c3,0x28);
	write_cmos_sensor(0x4540,0x12);
	write_cmos_sensor(0x4541,0x28);
	write_cmos_sensor(0x4606,0x12);
	write_cmos_sensor(0x4607,0x28);
	write_cmos_sensor(0x4648,0x12);
	write_cmos_sensor(0x4649,0x28);
	write_cmos_sensor(0x5110,0x00);
	write_cmos_sensor(0x5111,0x10);
	write_cmos_sensor(0x5112,0x05);
	write_cmos_sensor(0x5113,0x67);
	
	

}	/*	sensor_init  */


static void preview_setting(void)
{
	LOG_INF("E\n");
  write_cmos_sensor(0x0305,0x63);
	write_cmos_sensor(0x0307,0x00);
	write_cmos_sensor(0x4837,0x09);
	write_cmos_sensor(0x5000,0x55);
	write_cmos_sensor(0x5001,0x00);
	write_cmos_sensor(0x5002,0x37);
	write_cmos_sensor(0x5003,0x00);
	write_cmos_sensor(0x5015,0x20);
	write_cmos_sensor(0x5068,0x02);
	write_cmos_sensor(0x5115,0x0b);
	write_cmos_sensor(0x511c,0x01);
	write_cmos_sensor(0x511d,0x01);
	write_cmos_sensor(0x511e,0x01);
	write_cmos_sensor(0x511f,0x01);
	write_cmos_sensor(0x5180,0xc1);
	write_cmos_sensor(0x518a,0x04);
	write_cmos_sensor(0x51b0,0x30);
	write_cmos_sensor(0x51d0,0x28);
	write_cmos_sensor(0x51d8,0x08);
	write_cmos_sensor(0x51d9,0x10);
	write_cmos_sensor(0x51da,0x02);
	write_cmos_sensor(0x51db,0x02);
	write_cmos_sensor(0x51dc,0x06);
	write_cmos_sensor(0x51dd,0x06);
	write_cmos_sensor(0x51de,0x02);
	write_cmos_sensor(0x51df,0x06);
	write_cmos_sensor(0x51e0,0x0a);
	write_cmos_sensor(0x51e1,0x0e);
	write_cmos_sensor(0x51e2,0x00);
	write_cmos_sensor(0x51e3,0x00);
	write_cmos_sensor(0x5200,0x12);
	write_cmos_sensor(0x5201,0x20);
	write_cmos_sensor(0x5202,0x0d);
	write_cmos_sensor(0x5203,0xa0);
	write_cmos_sensor(0x5205,0x10);
	write_cmos_sensor(0x5207,0x10);
	write_cmos_sensor(0x5208,0x12);
	write_cmos_sensor(0x5209,0x00);
	write_cmos_sensor(0x520a,0x0d);
	write_cmos_sensor(0x520b,0x80);
	write_cmos_sensor(0x520d,0x08);
	write_cmos_sensor(0x5250,0x14);
	write_cmos_sensor(0x5331,0x06);
	write_cmos_sensor(0x5332,0x42);
	write_cmos_sensor(0x5333,0x24);
	write_cmos_sensor(0x53c1,0x00);
	write_cmos_sensor(0x5415,0x0b);
	write_cmos_sensor(0x541c,0x01);
	write_cmos_sensor(0x541d,0x01);
	write_cmos_sensor(0x541e,0x01);
	write_cmos_sensor(0x541f,0x01);
	write_cmos_sensor(0x5480,0xc1);
	write_cmos_sensor(0x548a,0x04);
	write_cmos_sensor(0x54b0,0x30);
	write_cmos_sensor(0x54d0,0x28);
	write_cmos_sensor(0x54d1,0x68);
	write_cmos_sensor(0x54d8,0x08);
	write_cmos_sensor(0x54d9,0x10);
	write_cmos_sensor(0x54da,0x02);
	write_cmos_sensor(0x54db,0x02);
	write_cmos_sensor(0x54dc,0x06);
	write_cmos_sensor(0x54dd,0x06);
	write_cmos_sensor(0x54de,0x02);
	write_cmos_sensor(0x54df,0x06);
	write_cmos_sensor(0x54e0,0x0a);
	write_cmos_sensor(0x54e1,0x0e);
	write_cmos_sensor(0x54e2,0x00);
	write_cmos_sensor(0x54e3,0x00);
	write_cmos_sensor(0x5500,0x12);
	write_cmos_sensor(0x5501,0x20);
	write_cmos_sensor(0x5502,0x0d);
	write_cmos_sensor(0x5503,0xa0);
	write_cmos_sensor(0x5505,0x10);
	write_cmos_sensor(0x5507,0x10);
	write_cmos_sensor(0x5508,0x12);
	write_cmos_sensor(0x5509,0x00);
	write_cmos_sensor(0x550a,0x0d);
	write_cmos_sensor(0x550b,0x80);
	write_cmos_sensor(0x550d,0x08);
	write_cmos_sensor(0x5550,0x14);
	write_cmos_sensor(0x5631,0x06);
	write_cmos_sensor(0x5632,0x42);
	write_cmos_sensor(0x5633,0x24);
	write_cmos_sensor(0x56c1,0x00);
	write_cmos_sensor(0x5715,0x0b);
	write_cmos_sensor(0x571c,0x01);
	write_cmos_sensor(0x571d,0x01);
	write_cmos_sensor(0x571e,0x01);
	write_cmos_sensor(0x571f,0x01);
	write_cmos_sensor(0x5780,0xc1);
	write_cmos_sensor(0x578a,0x04);
	write_cmos_sensor(0x57b0,0x30);
	write_cmos_sensor(0x57d0,0x28);
	write_cmos_sensor(0x57d8,0x08);
	write_cmos_sensor(0x57d9,0x10);
	write_cmos_sensor(0x57da,0x02);
	write_cmos_sensor(0x57db,0x02);
	write_cmos_sensor(0x57dc,0x06);
	write_cmos_sensor(0x57dd,0x06);
	write_cmos_sensor(0x57de,0x02);
	write_cmos_sensor(0x57df,0x06);
	write_cmos_sensor(0x57e0,0x0a);
	write_cmos_sensor(0x57e1,0x0e);
	write_cmos_sensor(0x57e2,0x00);
	write_cmos_sensor(0x57e3,0x00);
	write_cmos_sensor(0x5800,0x12);
	write_cmos_sensor(0x5801,0x20);
	write_cmos_sensor(0x5802,0x0d);
	write_cmos_sensor(0x5803,0xa0);
	write_cmos_sensor(0x5805,0x10);
	write_cmos_sensor(0x5807,0x10);
	write_cmos_sensor(0x5808,0x12);
	write_cmos_sensor(0x5809,0x00);
	write_cmos_sensor(0x580a,0x0d);
	write_cmos_sensor(0x580b,0x80);
	write_cmos_sensor(0x580d,0x08);
	write_cmos_sensor(0x5850,0x14);
	write_cmos_sensor(0x5931,0x06);
	write_cmos_sensor(0x5932,0x42);
	write_cmos_sensor(0x5933,0x24);
	write_cmos_sensor(0x3400,0x0c);
	write_cmos_sensor(0x340c,0x10);
	write_cmos_sensor(0x340d,0x00);
	write_cmos_sensor(0x3500,0x00);
	write_cmos_sensor(0x3501,0x0a);
	write_cmos_sensor(0x3502,0x48);
	write_cmos_sensor(0x3541,0x00);
	write_cmos_sensor(0x3542,0x10);
	write_cmos_sensor(0x360b,0x50);
	write_cmos_sensor(0x360d,0x05);
	write_cmos_sensor(0x3635,0xac);
	write_cmos_sensor(0x3712,0x50);
	write_cmos_sensor(0x3714,0x63);
	write_cmos_sensor(0x3800,0x00);
	write_cmos_sensor(0x3801,0x00);
	write_cmos_sensor(0x3802,0x00);
	write_cmos_sensor(0x3803,0x00);
	write_cmos_sensor(0x3804,0x19);
	write_cmos_sensor(0x3805,0x9f);
	write_cmos_sensor(0x3806,0x13);
	write_cmos_sensor(0x3807,0x3f);
	write_cmos_sensor(0x3808,0x0c);
	write_cmos_sensor(0x3809,0xc0);
	write_cmos_sensor(0x380a,0x09);
	write_cmos_sensor(0x380b,0x90);
	write_cmos_sensor(0x380c,0x02);
	write_cmos_sensor(0x380d,0xd0);
	write_cmos_sensor(0x380e,0x0a);
	write_cmos_sensor(0x380f,0x68);
	write_cmos_sensor(0x3810,0x00);
	write_cmos_sensor(0x3811,0x09);
	write_cmos_sensor(0x3812,0x00);
	write_cmos_sensor(0x3813,0x08);
	write_cmos_sensor(0x3814,0x11);
	write_cmos_sensor(0x3815,0x11);
	write_cmos_sensor(0x3820,0x02);
	write_cmos_sensor(0x3821,0x14);
	write_cmos_sensor(0x3822,0x00);
	write_cmos_sensor(0x3823,0x04);
	write_cmos_sensor(0x3837,0x07);
	write_cmos_sensor(0x384c,0x02);
	write_cmos_sensor(0x384d,0xd0);
	write_cmos_sensor(0x3856,0x50);
	write_cmos_sensor(0x3857,0x30);
	write_cmos_sensor(0x3858,0x80);
	write_cmos_sensor(0x3859,0x40);
	write_cmos_sensor(0x3894,0x00);
	write_cmos_sensor(0x3922,0x01);
	write_cmos_sensor(0x3976,0x01);
	write_cmos_sensor(0x397b,0x01);
	write_cmos_sensor(0x3994,0xff);
	write_cmos_sensor(0x39f5,0x01);
	write_cmos_sensor(0x39fd,0x02);
	write_cmos_sensor(0x3a01,0x04);
	write_cmos_sensor(0x3a4d,0x0c);
	write_cmos_sensor(0x3a4e,0x0c);
	write_cmos_sensor(0x3a4f,0x0c);
	write_cmos_sensor(0x3a50,0x0c);
	write_cmos_sensor(0x3a51,0x0c);
	write_cmos_sensor(0x3a57,0x15);
	write_cmos_sensor(0x3a5a,0x01);
	write_cmos_sensor(0x3a7d,0x60);
	write_cmos_sensor(0x3a80,0x0f);
	write_cmos_sensor(0x3a82,0x01);
	write_cmos_sensor(0x3a85,0x01);
	write_cmos_sensor(0x3aa3,0x13);
	write_cmos_sensor(0x3aa4,0x10);
	write_cmos_sensor(0x3aa5,0x0b);
	write_cmos_sensor(0x3aa6,0x00);
	write_cmos_sensor(0x3aad,0x00);
	write_cmos_sensor(0x4016,0x0d);
	write_cmos_sensor(0x4018,0x07);
	write_cmos_sensor(0x4504,0x80);
	write_cmos_sensor(0x4542,0x00);
	write_cmos_sensor(0x4602,0x00);
	write_cmos_sensor(0x4603,0x15);
	write_cmos_sensor(0x460b,0x03);
	write_cmos_sensor(0x4641,0x19);
	write_cmos_sensor(0x4643,0x0c);
	write_cmos_sensor(0x480e,0x00);
	write_cmos_sensor(0x0100,0x01);
	
	


	 
}	/*	preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
		LOG_INF("E\n");
  write_cmos_sensor(0x0305,0x63);
	write_cmos_sensor(0x0307,0x00);
	write_cmos_sensor(0x4837,0x09);
	write_cmos_sensor(0x5000,0x55);
	write_cmos_sensor(0x5001,0x00);
	write_cmos_sensor(0x5002,0x37);
	write_cmos_sensor(0x5003,0x00);
	write_cmos_sensor(0x5015,0x20);
	write_cmos_sensor(0x5068,0x02);
	write_cmos_sensor(0x5115,0x0b);
	write_cmos_sensor(0x511c,0x01);
	write_cmos_sensor(0x511d,0x01);
	write_cmos_sensor(0x511e,0x01);
	write_cmos_sensor(0x511f,0x01);
	write_cmos_sensor(0x5180,0xc1);
	write_cmos_sensor(0x518a,0x04);
	write_cmos_sensor(0x51b0,0x30);
	write_cmos_sensor(0x51d0,0x28);
	write_cmos_sensor(0x51d8,0x08);
	write_cmos_sensor(0x51d9,0x10);
	write_cmos_sensor(0x51da,0x02);
	write_cmos_sensor(0x51db,0x02);
	write_cmos_sensor(0x51dc,0x06);
	write_cmos_sensor(0x51dd,0x06);
	write_cmos_sensor(0x51de,0x02);
	write_cmos_sensor(0x51df,0x06);
	write_cmos_sensor(0x51e0,0x0a);
	write_cmos_sensor(0x51e1,0x0e);
	write_cmos_sensor(0x51e2,0x00);
	write_cmos_sensor(0x51e3,0x00);
	write_cmos_sensor(0x5200,0x12);
	write_cmos_sensor(0x5201,0x20);
	write_cmos_sensor(0x5202,0x0d);
	write_cmos_sensor(0x5203,0xa0);
	write_cmos_sensor(0x5205,0x10);
	write_cmos_sensor(0x5207,0x10);
	write_cmos_sensor(0x5208,0x12);
	write_cmos_sensor(0x5209,0x00);
	write_cmos_sensor(0x520a,0x0d);
	write_cmos_sensor(0x520b,0x80);
	write_cmos_sensor(0x520d,0x08);
	write_cmos_sensor(0x5250,0x14);
	write_cmos_sensor(0x5331,0x06);
	write_cmos_sensor(0x5332,0x42);
	write_cmos_sensor(0x5333,0x24);
	write_cmos_sensor(0x53c1,0x00);
	write_cmos_sensor(0x5415,0x0b);
	write_cmos_sensor(0x541c,0x01);
	write_cmos_sensor(0x541d,0x01);
	write_cmos_sensor(0x541e,0x01);
	write_cmos_sensor(0x541f,0x01);
	write_cmos_sensor(0x5480,0xc1);
	write_cmos_sensor(0x548a,0x04);
	write_cmos_sensor(0x54b0,0x30);
	write_cmos_sensor(0x54d0,0x28);
	write_cmos_sensor(0x54d1,0x68);
	write_cmos_sensor(0x54d8,0x08);
	write_cmos_sensor(0x54d9,0x10);
	write_cmos_sensor(0x54da,0x02);
	write_cmos_sensor(0x54db,0x02);
	write_cmos_sensor(0x54dc,0x06);
	write_cmos_sensor(0x54dd,0x06);
	write_cmos_sensor(0x54de,0x02);
	write_cmos_sensor(0x54df,0x06);
	write_cmos_sensor(0x54e0,0x0a);
	write_cmos_sensor(0x54e1,0x0e);
	write_cmos_sensor(0x54e2,0x00);
	write_cmos_sensor(0x54e3,0x00);
	write_cmos_sensor(0x5500,0x12);
	write_cmos_sensor(0x5501,0x20);
	write_cmos_sensor(0x5502,0x0d);
	write_cmos_sensor(0x5503,0xa0);
	write_cmos_sensor(0x5505,0x10);
	write_cmos_sensor(0x5507,0x10);
	write_cmos_sensor(0x5508,0x12);
	write_cmos_sensor(0x5509,0x00);
	write_cmos_sensor(0x550a,0x0d);
	write_cmos_sensor(0x550b,0x80);
	write_cmos_sensor(0x550d,0x08);
	write_cmos_sensor(0x5550,0x14);
	write_cmos_sensor(0x5631,0x06);
	write_cmos_sensor(0x5632,0x42);
	write_cmos_sensor(0x5633,0x24);
	write_cmos_sensor(0x56c1,0x00);
	write_cmos_sensor(0x5715,0x0b);
	write_cmos_sensor(0x571c,0x01);
	write_cmos_sensor(0x571d,0x01);
	write_cmos_sensor(0x571e,0x01);
	write_cmos_sensor(0x571f,0x01);
	write_cmos_sensor(0x5780,0xc1);
	write_cmos_sensor(0x578a,0x04);
	write_cmos_sensor(0x57b0,0x30);
	write_cmos_sensor(0x57d0,0x28);
	write_cmos_sensor(0x57d8,0x08);
	write_cmos_sensor(0x57d9,0x10);
	write_cmos_sensor(0x57da,0x02);
	write_cmos_sensor(0x57db,0x02);
	write_cmos_sensor(0x57dc,0x06);
	write_cmos_sensor(0x57dd,0x06);
	write_cmos_sensor(0x57de,0x02);
	write_cmos_sensor(0x57df,0x06);
	write_cmos_sensor(0x57e0,0x0a);
	write_cmos_sensor(0x57e1,0x0e);
	write_cmos_sensor(0x57e2,0x00);
	write_cmos_sensor(0x57e3,0x00);
	write_cmos_sensor(0x5800,0x12);
	write_cmos_sensor(0x5801,0x20);
	write_cmos_sensor(0x5802,0x0d);
	write_cmos_sensor(0x5803,0xa0);
	write_cmos_sensor(0x5805,0x10);
	write_cmos_sensor(0x5807,0x10);
	write_cmos_sensor(0x5808,0x12);
	write_cmos_sensor(0x5809,0x00);
	write_cmos_sensor(0x580a,0x0d);
	write_cmos_sensor(0x580b,0x80);
	write_cmos_sensor(0x580d,0x08);
	write_cmos_sensor(0x5850,0x14);
	write_cmos_sensor(0x5931,0x06);
	write_cmos_sensor(0x5932,0x42);
	write_cmos_sensor(0x5933,0x24);
	write_cmos_sensor(0x3400,0x0c);
	write_cmos_sensor(0x340c,0x10);
	write_cmos_sensor(0x340d,0x00);
	write_cmos_sensor(0x3500,0x00);
	write_cmos_sensor(0x3501,0x0a);
	write_cmos_sensor(0x3502,0x48);
	write_cmos_sensor(0x3541,0x00);
	write_cmos_sensor(0x3542,0x10);
	write_cmos_sensor(0x360b,0x50);
	write_cmos_sensor(0x360d,0x05);
	write_cmos_sensor(0x3635,0xac);
	write_cmos_sensor(0x3712,0x50);
	write_cmos_sensor(0x3714,0x63);
	write_cmos_sensor(0x3800,0x00);
	write_cmos_sensor(0x3801,0x00);
	write_cmos_sensor(0x3802,0x00);
	write_cmos_sensor(0x3803,0x00);
	write_cmos_sensor(0x3804,0x19);
	write_cmos_sensor(0x3805,0x9f);
	write_cmos_sensor(0x3806,0x13);
	write_cmos_sensor(0x3807,0x3f);
	write_cmos_sensor(0x3808,0x0c);
	write_cmos_sensor(0x3809,0xc0);
	write_cmos_sensor(0x380a,0x09);
	write_cmos_sensor(0x380b,0x90);
	write_cmos_sensor(0x380c,0x02);
	write_cmos_sensor(0x380d,0xd0);
	write_cmos_sensor(0x380e,0x0a);
	write_cmos_sensor(0x380f,0x68);
	write_cmos_sensor(0x3810,0x00);
	write_cmos_sensor(0x3811,0x09);
	write_cmos_sensor(0x3812,0x00);
	write_cmos_sensor(0x3813,0x08);
	write_cmos_sensor(0x3814,0x11);
	write_cmos_sensor(0x3815,0x11);
	write_cmos_sensor(0x3820,0x02);
	write_cmos_sensor(0x3821,0x14);
	write_cmos_sensor(0x3822,0x00);
	write_cmos_sensor(0x3823,0x04);
	write_cmos_sensor(0x3837,0x07);
	write_cmos_sensor(0x384c,0x02);
	write_cmos_sensor(0x384d,0xd0);
	write_cmos_sensor(0x3856,0x50);
	write_cmos_sensor(0x3857,0x30);
	write_cmos_sensor(0x3858,0x80);
	write_cmos_sensor(0x3859,0x40);
	write_cmos_sensor(0x3894,0x00);
	write_cmos_sensor(0x3922,0x01);
	write_cmos_sensor(0x3976,0x01);
	write_cmos_sensor(0x397b,0x01);
	write_cmos_sensor(0x3994,0xff);
	write_cmos_sensor(0x39f5,0x01);
	write_cmos_sensor(0x39fd,0x02);
	write_cmos_sensor(0x3a01,0x04);
	write_cmos_sensor(0x3a4d,0x0c);
	write_cmos_sensor(0x3a4e,0x0c);
	write_cmos_sensor(0x3a4f,0x0c);
	write_cmos_sensor(0x3a50,0x0c);
	write_cmos_sensor(0x3a51,0x0c);
	write_cmos_sensor(0x3a57,0x15);
	write_cmos_sensor(0x3a5a,0x01);
	write_cmos_sensor(0x3a7d,0x60);
	write_cmos_sensor(0x3a80,0x0f);
	write_cmos_sensor(0x3a82,0x01);
	write_cmos_sensor(0x3a85,0x01);
	write_cmos_sensor(0x3aa3,0x13);
	write_cmos_sensor(0x3aa4,0x10);
	write_cmos_sensor(0x3aa5,0x0b);
	write_cmos_sensor(0x3aa6,0x00);
	write_cmos_sensor(0x3aad,0x00);
	write_cmos_sensor(0x4016,0x0d);
	write_cmos_sensor(0x4018,0x07);
	write_cmos_sensor(0x4504,0x80);
	write_cmos_sensor(0x4542,0x00);
	write_cmos_sensor(0x4602,0x00);
	write_cmos_sensor(0x4603,0x15);
	write_cmos_sensor(0x460b,0x03);
	write_cmos_sensor(0x4641,0x19);
	write_cmos_sensor(0x4643,0x0c);
	write_cmos_sensor(0x480e,0x00);
	write_cmos_sensor(0x0100,0x01);

}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E\n");
  write_cmos_sensor(0x0305,0x63);
	write_cmos_sensor(0x0307,0x00);
	write_cmos_sensor(0x4837,0x09);
	write_cmos_sensor(0x5000,0x55);
	write_cmos_sensor(0x5001,0x00);
	write_cmos_sensor(0x5002,0x37);
	write_cmos_sensor(0x5003,0x00);
	write_cmos_sensor(0x5015,0x20);
	write_cmos_sensor(0x5068,0x02);
	write_cmos_sensor(0x5115,0x0b);
	write_cmos_sensor(0x511c,0x01);
	write_cmos_sensor(0x511d,0x01);
	write_cmos_sensor(0x511e,0x01);
	write_cmos_sensor(0x511f,0x01);
	write_cmos_sensor(0x5180,0xc1);
	write_cmos_sensor(0x518a,0x04);
	write_cmos_sensor(0x51b0,0x30);
	write_cmos_sensor(0x51d0,0x28);
	write_cmos_sensor(0x51d8,0x08);
	write_cmos_sensor(0x51d9,0x10);
	write_cmos_sensor(0x51da,0x02);
	write_cmos_sensor(0x51db,0x02);
	write_cmos_sensor(0x51dc,0x06);
	write_cmos_sensor(0x51dd,0x06);
	write_cmos_sensor(0x51de,0x02);
	write_cmos_sensor(0x51df,0x06);
	write_cmos_sensor(0x51e0,0x0a);
	write_cmos_sensor(0x51e1,0x0e);
	write_cmos_sensor(0x51e2,0x00);
	write_cmos_sensor(0x51e3,0x00);
	write_cmos_sensor(0x5200,0x12);
	write_cmos_sensor(0x5201,0x20);
	write_cmos_sensor(0x5202,0x0d);
	write_cmos_sensor(0x5203,0xa0);
	write_cmos_sensor(0x5205,0x10);
	write_cmos_sensor(0x5207,0x10);
	write_cmos_sensor(0x5208,0x12);
	write_cmos_sensor(0x5209,0x00);
	write_cmos_sensor(0x520a,0x0d);
	write_cmos_sensor(0x520b,0x80);
	write_cmos_sensor(0x520d,0x08);
	write_cmos_sensor(0x5250,0x14);
	write_cmos_sensor(0x5331,0x06);
	write_cmos_sensor(0x5332,0x42);
	write_cmos_sensor(0x5333,0x24);
	write_cmos_sensor(0x53c1,0x00);
	write_cmos_sensor(0x5415,0x0b);
	write_cmos_sensor(0x541c,0x01);
	write_cmos_sensor(0x541d,0x01);
	write_cmos_sensor(0x541e,0x01);
	write_cmos_sensor(0x541f,0x01);
	write_cmos_sensor(0x5480,0xc1);
	write_cmos_sensor(0x548a,0x04);
	write_cmos_sensor(0x54b0,0x30);
	write_cmos_sensor(0x54d0,0x28);
	write_cmos_sensor(0x54d1,0x68);
	write_cmos_sensor(0x54d8,0x08);
	write_cmos_sensor(0x54d9,0x10);
	write_cmos_sensor(0x54da,0x02);
	write_cmos_sensor(0x54db,0x02);
	write_cmos_sensor(0x54dc,0x06);
	write_cmos_sensor(0x54dd,0x06);
	write_cmos_sensor(0x54de,0x02);
	write_cmos_sensor(0x54df,0x06);
	write_cmos_sensor(0x54e0,0x0a);
	write_cmos_sensor(0x54e1,0x0e);
	write_cmos_sensor(0x54e2,0x00);
	write_cmos_sensor(0x54e3,0x00);
	write_cmos_sensor(0x5500,0x12);
	write_cmos_sensor(0x5501,0x20);
	write_cmos_sensor(0x5502,0x0d);
	write_cmos_sensor(0x5503,0xa0);
	write_cmos_sensor(0x5505,0x10);
	write_cmos_sensor(0x5507,0x10);
	write_cmos_sensor(0x5508,0x12);
	write_cmos_sensor(0x5509,0x00);
	write_cmos_sensor(0x550a,0x0d);
	write_cmos_sensor(0x550b,0x80);
	write_cmos_sensor(0x550d,0x08);
	write_cmos_sensor(0x5550,0x14);
	write_cmos_sensor(0x5631,0x06);
	write_cmos_sensor(0x5632,0x42);
	write_cmos_sensor(0x5633,0x24);
	write_cmos_sensor(0x56c1,0x00);
	write_cmos_sensor(0x5715,0x0b);
	write_cmos_sensor(0x571c,0x01);
	write_cmos_sensor(0x571d,0x01);
	write_cmos_sensor(0x571e,0x01);
	write_cmos_sensor(0x571f,0x01);
	write_cmos_sensor(0x5780,0xc1);
	write_cmos_sensor(0x578a,0x04);
	write_cmos_sensor(0x57b0,0x30);
	write_cmos_sensor(0x57d0,0x28);
	write_cmos_sensor(0x57d8,0x08);
	write_cmos_sensor(0x57d9,0x10);
	write_cmos_sensor(0x57da,0x02);
	write_cmos_sensor(0x57db,0x02);
	write_cmos_sensor(0x57dc,0x06);
	write_cmos_sensor(0x57dd,0x06);
	write_cmos_sensor(0x57de,0x02);
	write_cmos_sensor(0x57df,0x06);
	write_cmos_sensor(0x57e0,0x0a);
	write_cmos_sensor(0x57e1,0x0e);
	write_cmos_sensor(0x57e2,0x00);
	write_cmos_sensor(0x57e3,0x00);
	write_cmos_sensor(0x5800,0x12);
	write_cmos_sensor(0x5801,0x20);
	write_cmos_sensor(0x5802,0x0d);
	write_cmos_sensor(0x5803,0xa0);
	write_cmos_sensor(0x5805,0x10);
	write_cmos_sensor(0x5807,0x10);
	write_cmos_sensor(0x5808,0x12);
	write_cmos_sensor(0x5809,0x00);
	write_cmos_sensor(0x580a,0x0d);
	write_cmos_sensor(0x580b,0x80);
	write_cmos_sensor(0x580d,0x08);
	write_cmos_sensor(0x5850,0x14);
	write_cmos_sensor(0x5931,0x06);
	write_cmos_sensor(0x5932,0x42);
	write_cmos_sensor(0x5933,0x24);
	write_cmos_sensor(0x3400,0x0c);
	write_cmos_sensor(0x340c,0x10);
	write_cmos_sensor(0x340d,0x00);
	write_cmos_sensor(0x3500,0x00);
	write_cmos_sensor(0x3501,0x0a);
	write_cmos_sensor(0x3502,0x48);
	write_cmos_sensor(0x3541,0x00);
	write_cmos_sensor(0x3542,0x10);
	write_cmos_sensor(0x360b,0x50);
	write_cmos_sensor(0x360d,0x05);
	write_cmos_sensor(0x3635,0xac);
	write_cmos_sensor(0x3712,0x50);
	write_cmos_sensor(0x3714,0x63);
	write_cmos_sensor(0x3800,0x00);
	write_cmos_sensor(0x3801,0x00);
	write_cmos_sensor(0x3802,0x00);
	write_cmos_sensor(0x3803,0x00);
	write_cmos_sensor(0x3804,0x19);
	write_cmos_sensor(0x3805,0x9f);
	write_cmos_sensor(0x3806,0x13);
	write_cmos_sensor(0x3807,0x3f);
	write_cmos_sensor(0x3808,0x0c);
	write_cmos_sensor(0x3809,0xc0);
	write_cmos_sensor(0x380a,0x09);
	write_cmos_sensor(0x380b,0x90);
	write_cmos_sensor(0x380c,0x02);
	write_cmos_sensor(0x380d,0xd0);
	write_cmos_sensor(0x380e,0x0a);
	write_cmos_sensor(0x380f,0x68);
	write_cmos_sensor(0x3810,0x00);
	write_cmos_sensor(0x3811,0x09);
	write_cmos_sensor(0x3812,0x00);
	write_cmos_sensor(0x3813,0x08);
	write_cmos_sensor(0x3814,0x11);
	write_cmos_sensor(0x3815,0x11);
	write_cmos_sensor(0x3820,0x02);
	write_cmos_sensor(0x3821,0x14);
	write_cmos_sensor(0x3822,0x00);
	write_cmos_sensor(0x3823,0x04);
	write_cmos_sensor(0x3837,0x07);
	write_cmos_sensor(0x384c,0x02);
	write_cmos_sensor(0x384d,0xd0);
	write_cmos_sensor(0x3856,0x50);
	write_cmos_sensor(0x3857,0x30);
	write_cmos_sensor(0x3858,0x80);
	write_cmos_sensor(0x3859,0x40);
	write_cmos_sensor(0x3894,0x00);
	write_cmos_sensor(0x3922,0x01);
	write_cmos_sensor(0x3976,0x01);
	write_cmos_sensor(0x397b,0x01);
	write_cmos_sensor(0x3994,0xff);
	write_cmos_sensor(0x39f5,0x01);
	write_cmos_sensor(0x39fd,0x02);
	write_cmos_sensor(0x3a01,0x04);
	write_cmos_sensor(0x3a4d,0x0c);
	write_cmos_sensor(0x3a4e,0x0c);
	write_cmos_sensor(0x3a4f,0x0c);
	write_cmos_sensor(0x3a50,0x0c);
	write_cmos_sensor(0x3a51,0x0c);
	write_cmos_sensor(0x3a57,0x15);
	write_cmos_sensor(0x3a5a,0x01);
	write_cmos_sensor(0x3a7d,0x60);
	write_cmos_sensor(0x3a80,0x0f);
	write_cmos_sensor(0x3a82,0x01);
	write_cmos_sensor(0x3a85,0x01);
	write_cmos_sensor(0x3aa3,0x13);
	write_cmos_sensor(0x3aa4,0x10);
	write_cmos_sensor(0x3aa5,0x0b);
	write_cmos_sensor(0x3aa6,0x00);
	write_cmos_sensor(0x3aad,0x00);
	write_cmos_sensor(0x4016,0x0d);
	write_cmos_sensor(0x4018,0x07);
	write_cmos_sensor(0x4504,0x80);
	write_cmos_sensor(0x4542,0x00);
	write_cmos_sensor(0x4602,0x00);
	write_cmos_sensor(0x4603,0x15);
	write_cmos_sensor(0x460b,0x03);
	write_cmos_sensor(0x4641,0x19);
	write_cmos_sensor(0x4643,0x0c);
	write_cmos_sensor(0x480e,0x00);
	write_cmos_sensor(0x0100,0x01);

}
static void hs_video_setting(void)
{
	//LOG_INF("E\n");

}

static void slim_video_setting(void)
{
	
}
//
kal_uint8  test_pattern_flag=0;

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

		   

	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID 
*
* PARAMETERS
*	*sensorID : return the sensor ID 
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id) 
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id()+1;
			if (*sensor_id == imgsensor_info.sensor_id) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);	  
				return ERROR_NONE;
			}	
			LOG_INF("Read sensor id fail, i2c write id: 0x%x id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;
	LOG_1;
	LOG_2;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id()+1;
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, write id:0x%x id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;
	
	/* initail sequence write in  */
	sensor_init();
	
	iPreGain = 0;
	
	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/

	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap1.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}

	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps); 
	set_mirror_flip(imgsensor.mirror);
	
	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);

	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();

	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	slim_video	 */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);


	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
	//imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

			break;
		default:			
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}

	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			//preview(image_window, sensor_config_data);
			normal_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;	  
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();		
/*			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();*/
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();	
			break;		
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
			break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			//*framerate = imgsensor_info.pre.max_framerate;
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*framerate = imgsensor_info.cap.max_framerate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*framerate = imgsensor_info.hs_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*framerate = imgsensor_info.slim_video.max_framerate;
			break;
		default:
			break;
	}

	return ERROR_NONE;
}

static char *features[300] = {

	"SENSOR_FEATURE_BEGIN,                              ",
	"SENSOR_FEATURE_GET_RESOLUTION,                     ",
	"SENSOR_FEATURE_GET_PERIOD,                         ",
	"SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ,               ",
	"SENSOR_FEATURE_SET_ESHUTTER,                       ",
	"SENSOR_FEATURE_SET_NIGHTMODE,                      ",
	"SENSOR_FEATURE_SET_GAIN,                           ",
	"SENSOR_FEATURE_SET_DUAL_GAIN,                      ",
	"SENSOR_FEATURE_SET_GAIN_AND_ESHUTTER,              ",
	"SENSOR_FEATURE_SET_FLASHLIGHT,                     ",
	"SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ,          ",
	"SENSOR_FEATURE_SET_REGISTER,                       ",
	"SENSOR_FEATURE_GET_REGISTER,                       ",
	"SENSOR_FEATURE_SET_CCT_REGISTER,                   ",
	"SENSOR_FEATURE_GET_CCT_REGISTER,                   ",
	"SENSOR_FEATURE_SET_ENG_REGISTER,                   ",
	"SENSOR_FEATURE_GET_ENG_REGISTER,                   ",
	"SENSOR_FEATURE_GET_REGISTER_DEFAULT,               ",
	"SENSOR_FEATURE_GET_CONFIG_PARA,                    ",
	"SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR,              ",
	"SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA,              ",
	"SENSOR_FEATURE_GET_GROUP_COUNT,                    ",
	"SENSOR_FEATURE_GET_GROUP_INFO,                     ",
	"SENSOR_FEATURE_GET_ITEM_INFO,                      ",
	"SENSOR_FEATURE_SET_ITEM_INFO,                      ",
	"SENSOR_FEATURE_GET_ENG_INFO,                       ",
	"SENSOR_FEATURE_GET_LENS_DRIVER_ID,                 ",
	"SENSOR_FEATURE_SET_YUV_CMD,                        ",
	"SENSOR_FEATURE_SET_VIDEO_MODE,                     ",
	"SENSOR_FEATURE_SET_TARGET_FRAME_RATE,              ",
	"SENSOR_FEATURE_SET_CALIBRATION_DATA,               ",
	"SENSOR_FEATURE_SET_SENSOR_SYNC,                    ",
	"SENSOR_FEATURE_INITIALIZE_AF,                      ",
	"SENSOR_FEATURE_CONSTANT_AF,                        ",
	"SENSOR_FEATURE_INFINITY_AF,                        ",
	"SENSOR_FEATURE_MOVE_FOCUS_LENS,                    ",
	"SENSOR_FEATURE_GET_AF_STATUS,                      ",
	"SENSOR_FEATURE_GET_AE_STATUS,                      ",
	"SENSOR_FEATURE_GET_AWB_STATUS,                     ",
	"SENSOR_FEATURE_GET_AF_INF,                         ",
	"SENSOR_FEATURE_GET_AF_MACRO,                       ",
	"SENSOR_FEATURE_CHECK_SENSOR_ID,                    ",
	"SENSOR_FEATURE_SET_AUTO_FLICKER_MODE,              ",
	"SENSOR_FEATURE_SET_TEST_PATTERN,                   ",
	"SENSOR_FEATURE_SET_SOFTWARE_PWDN,                  ",
	"SENSOR_FEATURE_SINGLE_FOCUS_MODE,                  ",
	"SENSOR_FEATURE_CANCEL_AF,                          ",
	"SENSOR_FEATURE_SET_AF_WINDOW,                      ",
	"SENSOR_FEATURE_GET_EV_AWB_REF,                     ",
	"SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN,          ",
	"SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS,         ",
	"SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS,      ",
	"SENSOR_FEATURE_SET_AE_WINDOW,                      ",
	"SENSOR_FEATURE_GET_EXIF_INFO,                      ",
	"SENSOR_FEATURE_GET_DELAY_INFO,                     ",
	"SENSOR_FEATURE_SET_SLAVE_I2C_ID,                   ",
	"SENSOR_FEATURE_SUSPEND,                            ",
	"SENSOR_FEATURE_RESUME,                             ",
	"SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO,     ",
	"SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO, ",
	"SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO,               ",
	"SENSOR_FEATURE_AUTOTEST_CMD,                       ",
	"SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE,    ",
	"SENSOR_FEATURE_GET_TEMPERATURE_VALUE,              ",
	"SENSOR_FEATURE_GET_SENSOR_CURRENT_TEMPERATURE,     ",
	"SENSOR_FEATURE_GET_AE_FLASHLIGHT_INFO,             ",
	"SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO,        ",
	"SENSOR_FEATURE_SET_YUV_3A_CMD,                     ",
	"SENSOR_FEATURE_SET_N3D_I2C_STREAM_REGDATA,         ",
	"SENSOR_FEATURE_SET_N3D_STOP_STREAMING,             ",
	"SENSOR_FEATURE_SET_N3D_START_STREAMING,            ",
	"SENSOR_FEATURE_GET_SENSOR_N3D_STREAM_TO_VSYNC_TIME,",
	"SENSOR_FEATURE_SET_ESHUTTER_GAIN,                  ",
	"SENSOR_FEATURE_SET_OB_LOCK,                        ",
	"SENSOR_FEATURE_SET_SENSOR_OTP_AWB_CMD,             ",
	"SENSOR_FEATURE_SET_SENSOR_OTP_LSC_CMD,             ",
	"SENSOR_FEATURE_GET_YUV_CAPTURE_OUTPUT_JPEG,        ",
	"SENSOR_FEATURE_SET_YUV_JPEG_PARA,                  ",
	"SENSOR_FEATURE_GET_YUV_JPEG_INFO,                  ",
	"SENSOR_FEATURE_SET_FRAMERATE,                      ",
	"SENSOR_FEATURE_SET_HDR,                            ",
	"SENSOR_FEATURE_GET_CROP_INFO,                      ",
	"SENSOR_FEATURE_GET_VC_INFO,                        ",
	"SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN,              ",
	"SENSOR_FEATURE_SET_AWB_GAIN,                       ",
	"SENSOR_FEATURE_SET_MIN_MAX_FPS,                    ",
	"SENSOR_FEATURE_GET_PDAF_INFO,                      ",
	"SENSOR_FEATURE_GET_PDAF_DATA,                      ",
	"SENSOR_FEATURE_SET_PDFOCUS_AREA,                   ",
	"SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY,           ",
	"SENSOR_FEATURE_DEBUG_IMGSENSOR,                    ",
	"SENSOR_FEATURE_SET_HDR_SHUTTER,                    ",
	"SENSOR_FEATURE_SET_ISO,                            ",
	"SENSOR_FEATURE_SET_PDAF,                           ",
	"SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME,             ",
	"SENSOR_FEATURE_SET_SHUTTER_BUF_MODE,               ",
	"SENSOR_FEATURE_SET_GAIN_BUF_MODE,                  ",
	"SENSOR_FEATURE_SET_I2C_BUF_MODE_EN,                ",
	"SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY,            ",
	"SENSOR_FEATURE_GET_PDAF_TYPE,                      ",
	"SENSOR_FEATURE_SET_PDAF_TYPE,                      ",
	"SENSOR_FEATURE_GET_PDAF_REG_SETTING,               ",
	"SENSOR_FEATURE_SET_PDAF_REG_SETTING,               ",
	"SENSOR_FEATURE_SET_STREAMING_SUSPEND,              ",
	"SENSOR_FEATURE_SET_STREAMING_RESUME,               ",
	"SENSOR_FEATURE_OPEN,                               ",
	"SENSOR_FEATURE_CLOSE,                              ",
	"SENSOR_FEATURE_SET_DRIVER,                         ",
	"SENSOR_FEATURE_CHECK_IS_ALIVE,                     ",
	"SENSOR_FEATURE_GET_4CELL_DATA,                     ",
	"SENSOR_FEATURE_SET_WAKE_LOCK,                      ",
	"SENSOR_FEATURE_GET_MIPI_PIXEL_RATE,                ",
	"SENSOR_FEATURE_SET_HDR_ATR,                        ",
	"SENSOR_FEATURE_SET_HDR_TRI_GAIN,                   ",
	"SENSOR_FEATURE_SET_HDR_TRI_SHUTTER,                ",
	"SENSOR_FEATURE_SET_LSC_TBL,                        ",
	"SENSOR_FEATURE_GET_SENSOR_SYNC_MODE_CAPACITY,      ",
	"SENSOR_FEATURE_GET_SENSOR_SYNC_MODE,               ",
	"SENSOR_FEATURE_SET_SENSOR_SYNC_MODE,               ",
	"SENSOR_FEATURE_GET_PIXEL_RATE,                     ",
	"SENSOR_FEATURE_MAX									"};

static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable)
		write_cmos_sensor(0x0100, 0x01);
	else
		write_cmos_sensor(0x0100, 0x00);
	mdelay(10);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("feature_id = %d[%s]\n", feature_id, features[feature_id - SENSOR_FEATURE_START]);
	switch (feature_id)
	{
		case SENSOR_FEATURE_GET_ANA_GAIN_TABLE:
		/*	if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL) {
				*(feature_data + 0) =
					sizeof(ana_gain_table_16x);
			} else {
				memcpy((void *)(uintptr_t) (*(feature_data + 1)),
				(void *)ana_gain_table_16x,
				sizeof(ana_gain_table_16x));
			}*/
			break;
		case SENSOR_FEATURE_GET_AWB_REQ_BY_SCENARIO:
			switch (*feature_data) {
			case MSDK_SCENARIO_ID_CUSTOM3:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1;
				break;
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
				break;
			}
			break;
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_gain;
		*(feature_data + 2) = imgsensor_info.max_gain;
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = imgsensor_info.min_gain_iso;
		*(feature_data + 1) = imgsensor_info.gain_step;
		*(feature_data + 2) = imgsensor_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_shutter;
		switch (*feature_data)
		{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:
		case MSDK_SCENARIO_ID_CUSTOM4:
		case MSDK_SCENARIO_ID_CUSTOM5:
			*(feature_data + 2) = 2;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
		default:
			*(feature_data + 2) = 1;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
			*feature_return_para_32 = imgsensor.current_ae_effective_frame;
			break;
				
	case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
			memcpy(feature_return_para_32, &imgsensor.ae_frm_mode,sizeof(struct IMGSENSOR_AE_FRM_MODE));
			break;

	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data)
		{
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = (imgsensor_info.cap.framelength << 16) + imgsensor_info.cap.linelength;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = (imgsensor_info.normal_video.framelength << 16) + imgsensor_info.normal_video.linelength;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = (imgsensor_info.hs_video.framelength << 16) + imgsensor_info.hs_video.linelength;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = (imgsensor_info.slim_video.framelength << 16) + imgsensor_info.slim_video.linelength;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = (imgsensor_info.pre.framelength << 16) + imgsensor_info.pre.linelength;
			break;
		}
		break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len=4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data)
		{
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.cap.pclk;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.normal_video.pclk;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.hs_video.pclk;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.slim_video.pclk;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.pre.pclk;
			break;
		}
		break;
		case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL) (*feature_data));
			break;
		case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
			break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			get_imgsensor_id(feature_return_para_32);
			break;
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_FRAMERATE:
			LOG_INF("current fps :%d\n", (UINT32)*feature_data_32);
			spin_lock(&imgsensor_drv_lock);
			imgsensor.current_fps = *feature_data_32;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_SET_HDR:
			LOG_INF("ihdr enable :%d\n", *feature_data_32);
			spin_lock(&imgsensor_drv_lock);
			imgsensor.ihdr_en = (UINT8)*feature_data_32;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId---sunritel:%d\n", (UINT32)*feature_data);
            wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

			switch (*feature_data_32) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
					break;
				default:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
					break;
			}
		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1),
			(UINT16)*(feature_data+2));
		ihdr_write_shutter_gain((UINT16)*feature_data,
			(UINT16)*(feature_data+1), (UINT16)*(feature_data+2));
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length(
		    (UINT16) (*feature_data), (UINT16) (*(feature_data + 1)),
			(BOOL) (*(feature_data + 2)));
		break;
	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
			/*
			 * 1, if driver support new sw frame sync
			 * set_shutter_frame_length() support third para auto_extend_en
			 */
			*(feature_data + 1) = 1;
			/* margin info by scenario */
			*(feature_data + 2) = imgsensor_info.margin;
		break;
	case SENSOR_FEATURE_GET_SENSOR_SYNC_MODE_CAPACITY:
		*feature_return_para_32 = SENSOR_MASTER_SYNC_MODE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_SENSOR_SYNC_MODE:
		*feature_return_para_32 = g_sync_mode;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_SENSOR_SYNC_MODE:
		g_sync_mode = (MUINT32)(*feature_data_32);
		LOG_INF("[hwadd]mode = %d\n", g_sync_mode);
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
			*feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_PIXEL_RATE:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.cap.pclk /
			(imgsensor_info.cap.linelength - 80))*
			imgsensor_info.cap.grabwindow_width;

			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.normal_video.pclk /
			(imgsensor_info.normal_video.linelength - 80))*
			imgsensor_info.normal_video.grabwindow_width;

			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.hs_video.pclk /
			(imgsensor_info.hs_video.linelength - 80))*
			imgsensor_info.hs_video.grabwindow_width;

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.slim_video.pclk /
			(imgsensor_info.slim_video.linelength - 80))*
			imgsensor_info.slim_video.grabwindow_width;

			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.pre.pclk /
			(imgsensor_info.pre.linelength - 80))*
			imgsensor_info.pre.grabwindow_width;
			break;
		}
		break;
		case SENSOR_FEATURE_GET_BINNING_TYPE:
			switch (*(feature_data + 1)) {
			case MSDK_SCENARIO_ID_CUSTOM3:
				*feature_return_para_32 = 1; /*BINNING_NONE*/
				break;
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			case MSDK_SCENARIO_ID_CUSTOM4:
			default:
				*feature_return_para_32 = 1; /*BINNING_AVERAGED*/
				break;
			}
			pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
				*feature_return_para_32);
			*feature_para_len = 4;
		
			break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.cap.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.hs_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.slim_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
		break;
		case SENSOR_FEATURE_SET_AWB_GAIN:
			break;
		case SENSOR_FEATURE_SET_HDR_SHUTTER:
		    LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1));
			/* ihdr_write_shutter((UINT16)*feature_data,
			 *(UINT16)*(feature_data+1));
			 */
			break;
		default:
			break;
	}

	return ERROR_NONE;
}	/*	feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 OV32B40GMS_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	OV5693_MIPI_RAW_SensorInit	*/
EXPORT_SYMBOL(OV32B40GMS_MIPI_RAW_SensorInit);
