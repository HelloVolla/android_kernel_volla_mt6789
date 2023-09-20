#include <linux/init.h>	
#include <linux/module.h>	
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/reboot.h>

#include "pd_pe_policy.h"
#include "pd_policy_manager.h"

#define PDPE_WORK_RUN_INTERVAL        		300
#define PDPE_KPROC_WORK_RUN_INTERVAL        200

static enum power_supply_property pd_psy_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TEMP,
};

static int psy_pd_property_is_writeable(struct power_supply *psy,
					       enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:/*connect state*/
	case POWER_SUPPLY_PROP_PRESENT:/*hw reset cnt*/
	case POWER_SUPPLY_PROP_TEMP:/*boot mode*/
		return 1;
	default:
		return 0;
	}
}

static int psy_pd_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	
	struct pdpe_config *pdpe = NULL;
	pdpe = (struct pdpe_config *)power_supply_get_drvdata(psy);
	if (pdpe == NULL) {
		pr_err("%s: get info failed\n", __func__);
		return -EINVAL;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pdpe->pdpe_state;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = usbpd_get_hrst_cnt();
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = pdpe->boot_mode;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void set_pd_state(struct pdpe_config *pdpe,int state)
{
	/*unlikely(!pdpe){
		pr_err("%s: set_pd_state failed\n", __func__);
		return;
	}
	*/
	pr_err("gezi %s ori state = %d,new state = %d\n",__func__,pdpe->pdpe_state,state);
	pdpe->pdpe_state = state;
	
	if(pdpe->pdpe_state == PDPE_WORK_PE_NOT_SUPPORT || pdpe->pdpe_state == PDPE_WORK_PD_CHECK){
		pr_err("gezi %s state = %d schedule_delayed_work\n",__func__,pdpe->pdpe_state);
		schedule_delayed_work(&pdpe->pd_detect_work, msecs_to_jiffies(PDPE_WORK_RUN_INTERVAL));
	}
	else{
		pr_err("gezi %s----111---state = %d\n", __func__,pdpe->pdpe_state);
		cancel_delayed_work(&pdpe->pd_detect_work);
	}
}


int psy_pd_set_property(struct power_supply *psy,
			enum power_supply_property psp,
			const union power_supply_propval *val)
{
	struct pdpe_config *pdpe = NULL;
	pdpe = (struct pdpe_config *)power_supply_get_drvdata(psy);

	if (pdpe == NULL) {
		pr_err("%s: failed to get info\n", __func__);
		return -EINVAL;
	}
	
	
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		pr_err("gezi %s------%d\n",__func__,val->intval);
		set_pd_state(pdpe,val->intval);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		pdpe->boot_mode = val->intval;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int pdpe_psy_reg(struct pdpe_config *pdpe,struct platform_device *pdev)
{
	pdpe->sw_desc.name = "pdpe-state";
	pdpe->sw_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
	pdpe->sw_desc.properties = pd_psy_properties;
	pdpe->sw_desc.num_properties = ARRAY_SIZE(pd_psy_properties);
	pdpe->sw_desc.get_property = psy_pd_get_property;
	pdpe->sw_desc.set_property = psy_pd_set_property;
	pdpe->sw_desc.property_is_writeable = psy_pd_property_is_writeable;

	pdpe->sw_cfg.drv_data = pdpe;

	pdpe->sw_psy = power_supply_register(&pdev->dev, &pdpe->sw_desc,&pdpe->sw_cfg);
	if (IS_ERR(pdpe->sw_psy)){
		pr_err("gezi register sw_psy fail\n");
	}
	else{
		pr_err("gezi register sw_psy success\n");
	}
	return 0;
}

static void pd_detect_work(struct work_struct *work)
{
    struct pdpe_config *pdpe = container_of(work, struct pdpe_config, pd_detect_work.work);
	
	if(pdpe->pdpe_state == PDPE_WORK_PE_NOT_SUPPORT || pdpe->pdpe_state == PDPE_WORK_PD_CHECK){
		pr_err("gezi %s----000---state = %d\n", __func__,pdpe->pdpe_state);
		usbpd_pm_schedule();
		schedule_delayed_work(&pdpe->pd_detect_work, msecs_to_jiffies(PDPE_WORK_RUN_INTERVAL));
	}

}

static void pdpe_check_usb_psy(struct pdpe_config *pdpe)
{
    if (!pdpe->usb_psy) { 
        pdpe->usb_psy = power_supply_get_by_name("charger");
        if (!pdpe->usb_psy)
            pr_err("usb psy not found!\n");
    }
}

static int is_kpoc_mode(struct pdpe_config *pdpe)
{
	if(!pdpe){
		return 0;
	}
	if(pdpe->boot_mode == 8 || pdpe->boot_mode == 9){
		return 1;
	}
	else{
		return 0;
	}
}

static int pdpe_get_charge_state(struct pdpe_config *pdpe)
{
	int ret = 0;
    union power_supply_propval val = {0,};

	pdpe_check_usb_psy(pdpe);
	
	if(!pdpe->usb_psy){
		return -1;
	}

    ret = power_supply_get_property(pdpe->usb_psy, POWER_SUPPLY_PROP_ONLINE, &val);
	
    if (!ret){
        ret = val.intval;
	}
	else{
		ret = -1;
	}

    return ret;
}


static void kpoc_detect_work(struct work_struct *work)
{
    struct pdpe_config *pdpe = container_of(work, struct pdpe_config, kpoc_detect_work.work);
	
	if((pdpe->pdpe_state == PDPE_WORK_IDLE || pdpe->pdpe_state == PDPE_WORK_INIT_DONE)&& (pdpe_get_charge_state(pdpe) == 0)){
		pr_err("gezi %s----000---state = %d,kpoc_cnt = %d\n", __func__,pdpe->pdpe_state,pdpe->kpoc_cnt);
		pdpe->kpoc_cnt++;
		schedule_delayed_work(&pdpe->kpoc_detect_work, msecs_to_jiffies(PDPE_KPROC_WORK_RUN_INTERVAL));
		if(pdpe->kpoc_cnt >= 8){ //drv add by lipengpeng 20230315 C to C connection charging and data disconnection
			pr_err("gezi KERNEL POWER OFF %s state = %d,kpoc_cnt = %d\n", __func__,pdpe->pdpe_state,pdpe->kpoc_cnt);
			kernel_power_off();
		}
	}
	else{
		pr_err("gezi %s----000---state = %d\n", __func__,pdpe->pdpe_state);
		pdpe->psy_change_running = false;
		pdpe->kpoc_cnt = 0;
	}
}

static int pdpe_psy_notifier_cb(struct notifier_block *nb, 
            unsigned long event, void *data)
{
    struct pdpe_config *pdpe = container_of(nb, struct pdpe_config, nb);
	int state = 0;
	
    if (event != PSY_EVENT_PROP_CHANGED)
        return NOTIFY_OK;

    pdpe_check_usb_psy(pdpe);

    if (!pdpe->usb_psy){
        return NOTIFY_OK;
	}
	
	state = pdpe_get_charge_state(pdpe);
	
    if ((!pdpe->psy_change_running) && (!state) && is_kpoc_mode(pdpe))
	{
		pr_err(" gezi %s ,charger state = %d\n",__func__,state);
		pdpe->psy_change_running = true;
		schedule_delayed_work(&pdpe->kpoc_detect_work, msecs_to_jiffies(PDPE_KPROC_WORK_RUN_INTERVAL / 10));
    }
	else{
		pr_err(" gezi %s charger state = %d,boot_mode = %d\n",__func__,state,pdpe->boot_mode);
	}

    return NOTIFY_OK;
}



int pdpe_init(struct platform_device *pdev)
{
    struct pdpe_config *pdpe;
	
	pr_err("gezi %s------------------------%d\n", __func__,__LINE__);

    pdpe = kzalloc(sizeof(*pdpe), GFP_KERNEL);
    if (!pdpe){
        return -ENOMEM;
	}
	
	pdpe->pdpe_state = PDPE_WORK_IDLE;
	pdpe->psy_change_running = false;
	
	INIT_DELAYED_WORK(&pdpe->pd_detect_work, pd_detect_work);
	INIT_DELAYED_WORK(&pdpe->kpoc_detect_work, kpoc_detect_work);
/*
    pdpe->nb.notifier_call = pdpe_psy_notifier_cb;
    power_supply_reg_notifier(&pdpe->nb);
*/

	pdpe_psy_reg(pdpe,pdev);
	pdpe_check_usb_psy(pdpe);
	pdpe->nb.notifier_call = pdpe_psy_notifier_cb;
    power_supply_reg_notifier(&pdpe->nb);
	
    return 0;
}

void pdpe_exit(void)
{

}

