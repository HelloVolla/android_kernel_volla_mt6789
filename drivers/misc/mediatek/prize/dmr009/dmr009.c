#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>
#include <linux/semaphore.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/kthread.h>
#include <linux/input.h>
#if defined(CONFIG_PM_WAKELOCKS)
#include <linux/pm_wakeup.h>
#else
#include <linux/wakelock.h>
#endif
#include <linux/time.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/input.h>

#define dmr009_DEVNAME "dmr009_device"

struct dmr009_dev{
	struct device * dev;
	int dmr009_state;
	unsigned int dmr009_irq;
	int dmr009_pwd_pin;
	int dmr009_ptt_pin;
	int dmr009_3v3_pin;
	int dmr009_irq_pro;
};

static struct dmr009_dev *dmr009_local = NULL;

//irq
static ssize_t dmr_show(struct device *dev,struct device_attribute *attr, char *buf)
{
 	return scnprintf(buf, PAGE_SIZE, "%d\n", dmr009_local->dmr009_state);
}

static ssize_t dmr_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    int error;
	unsigned int temp;
    error = kstrtouint(buf, 10, &temp);
	if(error < 0)
		goto err;
err:
	return count;
}
static DEVICE_ATTR(debug, S_IRUGO|S_IWUSR, dmr_show, dmr_store);

//pwd
static ssize_t pwd_show(struct device *dev,struct device_attribute *attr, char *buf)
{
 	return scnprintf(buf, PAGE_SIZE, "pwd :%d\n", gpio_get_value(dmr009_local->dmr009_pwd_pin));
}

static ssize_t pwd_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    int error;
	unsigned int temp;
    error = kstrtouint(buf, 10, &temp);
	if(temp)
	{
		gpio_direction_output(dmr009_local->dmr009_3v3_pin,1); //3v3-gpio  high
		mdelay(10);
		gpio_direction_output(dmr009_local->dmr009_pwd_pin,1); //pwd-gpio  high
	}else{
		gpio_direction_output(dmr009_local->dmr009_pwd_pin,0); //pwd  low
		mdelay(10);
		gpio_direction_output(dmr009_local->dmr009_3v3_pin,0); //3v3-gpio  high
	}
		
	
	if(error < 0)
		goto err;
err:
	return count;
}
static DEVICE_ATTR(pwd, S_IRUGO|S_IWUSR, pwd_show, pwd_store);

//ptt
static ssize_t ptt_show(struct device *dev,struct device_attribute *attr, char *buf)
{
 	return scnprintf(buf, PAGE_SIZE, "ptt :%d\n", gpio_get_value(dmr009_local->dmr009_ptt_pin));
}

static ssize_t ptt_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    int error;
	unsigned int temp;
    error = kstrtouint(buf, 10, &temp);
	if(temp)
		gpio_direction_output(dmr009_local->dmr009_ptt_pin,1); //pwd  high
	else
		gpio_direction_output(dmr009_local->dmr009_ptt_pin,0); //pwd  low
	
	if(error < 0)
		goto err;
err:
	return count;
}
static DEVICE_ATTR(ptt, S_IRUGO|S_IWUSR, ptt_show, ptt_store);


static const struct attribute *dmr009_event_attr[] = {
        &dev_attr_debug.attr,
        &dev_attr_pwd.attr,
        &dev_attr_ptt.attr,
        NULL,
};

static const struct attribute_group dmr009_event_attr_group = {
        .attrs = (struct attribute **) dmr009_event_attr,
};

static irqreturn_t irq_dmr009_handler(int irq, void *dev_id)
{
	printk("[sy8801_dev] %s: +++ \n",__func__);

	disable_irq_nosync(irq);
	
	if(dmr009_local->dmr009_irq_pro){
			printk("[sy8801_dev] %s: start \n",__func__);
		dmr009_local->dmr009_state = 1;
		dmr009_local->dmr009_irq_pro = 0;
		irq_set_irq_type(dmr009_local->dmr009_irq,IRQF_TRIGGER_FALLING);
	}else{
			printk("[sy8801_dev] %s: stop \n",__func__);
		dmr009_local->dmr009_state = 0;
		dmr009_local->dmr009_irq_pro = 1;
		irq_set_irq_type(dmr009_local->dmr009_irq,IRQF_TRIGGER_RISING);	
	}
	enable_irq(dmr009_local->dmr009_irq);
	printk("[sy8801_dev] %s: --- \n",__func__);

	return IRQ_HANDLED;
}


static int dmr009_probe(struct platform_device *pdev){

    int ret = 0;
	// int irq_flags = 0;

	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;

		
    dmr009_local = devm_kzalloc(&pdev->dev, sizeof(struct dmr009_dev), GFP_KERNEL);
	dmr009_local->dev = &pdev->dev;
	if(IS_ERR_OR_NULL(dmr009_local)) 
    { 
       ret = PTR_ERR(dmr009_local);
       printk("[dmr009_local]:failed to devm_kzalloc dmr009_local %d\n", ret);
	   return ret;
    }
		
	if (dmr009_local->dev) {
		ret = sysfs_create_group(&(dmr009_local->dev)->kobj, &dmr009_event_attr_group);
	if(ret < 0) {
		printk("song_event:sysfs_create_group fail\r\n");
	return ret;
	}
	} else {
		printk("song_event:device_create fail\r\n");
	}

	dmr009_local->dmr009_irq = irq_of_parse_and_map(node, 0);
	printk("dmr009_irq<%d>\n", dmr009_local->dmr009_irq);
	if (dmr009_local->dmr009_irq < 0)
		return -ENODEV;

	ret = request_irq(dmr009_local->dmr009_irq, irq_dmr009_handler,
					IRQF_TRIGGER_RISING , "DMR009", NULL);		
	if (ret) {
		printk("request EINT <%d> fail, ret<%d>\n", dmr009_local->dmr009_irq, ret);
		return ret;
	}		
	
	dmr009_local->dmr009_pwd_pin = of_get_named_gpio(node, "pwd-gpio", 0);
	if (dmr009_local->dmr009_pwd_pin < 0){
		printk("pwd-gpio fail\n");
		return -ENODEV;
	}
	
	dmr009_local->dmr009_3v3_pin = of_get_named_gpio(node, "3v3-gpio", 0);
	if (dmr009_local->dmr009_3v3_pin < 0){
		printk("3v3-gpio fail\n");
		return -ENODEV;
	}
	dmr009_local->dmr009_ptt_pin = of_get_named_gpio(node, "ptt-gpio", 0);
	if (dmr009_local->dmr009_ptt_pin < 0){
		printk("ptt-gpio fail\n");
		return -ENODEV;
	}
	
	dmr009_local->dmr009_irq_pro = 1;


    return 0;
}

static int dmr009_remove(struct platform_device *pdev){
	printk("[dmr009_dev]:dmr009_remove begin!\n");
	printk("[dmr009_dev]:dmr009_remove Done!\n");
    
	return 0;
}


const struct of_device_id dmr009_of_match[] = {
	{ .compatible = "coosea,dmr009", },
	{},
};

struct platform_device dmr009_device = {
	.name		= dmr009_DEVNAME,
	.id			= -1,
};

static struct platform_driver dmr009_driver = {
	.remove = dmr009_remove,
	.probe = dmr009_probe,
	.driver = {
			.name = dmr009_DEVNAME,
			.owner = THIS_MODULE,
			.of_match_table = dmr009_of_match,
	},
};

static int __init dmr009_init(void)
{
	int ret = 0;
	
	printk("COOSEA dmr009 dmr009_init\n");
	
	ret = platform_driver_register(&dmr009_driver);
	if (ret < 0)
		printk("dmr009 : dmr009_init failed ret:%d\n", ret);
	return 0;
}
/* should never be called */
static void __exit dmr009_device_exit(void)
{
	printk("COOSEA dmr009 dmr009_device_exit \n");
	platform_driver_unregister(&dmr009_driver);

}

module_init(dmr009_init);
module_exit(dmr009_device_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("COOSEA dmr009 driver");
MODULE_AUTHOR("Liao Jie<liaojie@cooseagroup.com>");
