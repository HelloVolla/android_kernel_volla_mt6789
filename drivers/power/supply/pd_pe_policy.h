
/*
 * Copyright (c) 2022 Coosea Group Technology(ShenZhen) Co., Ltd.
 * zhaopengge@cooseagroup.com
 * 2022/07/20
 * GPL-2.0
 */


#ifndef __PD_PE_POLICY_MANAGER_H__
#define __PD_PE_POLICY_MANAGER_H__

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <../../misc/mediatek/typec/tcpc/inc/tcpm.h>

enum pdpe_work_state {
	PDPE_WORK_IDLE,  //0
	PDPE_WORK_INIT_DONE,  //1
	PDPE_WORK_PE_NEED_NOT,  //2
	PDPE_WORK_PE_CHECK,  //3
	PDPE_WORK_PE_CHECK_DONE,  //4
	PDPE_WORK_PE_RUN,		 //5
	PDPE_WORK_PE_RUN_END,	 //6
	PDPE_WORK_PE_NOT_SUPPORT,	 //7
	PDPE_WORK_PD_CHECK,	 //8
	PDPE_WORK_PD_CHECK_DONE, //9
	PDPE_WORK_PD_RUN,	 //10
	PDPE_WORK_PD_RUN_OK,	 //11
	PDPE_WORK_PD_NOT_SUPPORT,	 //12
};

struct pdpe_config {
	
	/*struct notifier_block nb;*/
	struct power_supply *usb_psy;
	struct power_supply *sw_psy;
	struct power_supply_desc sw_desc;
	struct power_supply_config sw_cfg;
	struct delayed_work pd_detect_work;
	struct delayed_work kpoc_detect_work;
	struct notifier_block nb;
	
	enum pdpe_work_state pdpe_state;
	int boot_mode;
	bool psy_change_running;
	unsigned char kpoc_cnt;
	
};

extern int pdpe_init(struct platform_device *pdev);

extern void pdpe_exit(void);

#endif


