#define pr_fmt(fmt) "[pix_manager]" fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/poll.h>
#include <linux/bitmap.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <uapi/linux/sched/types.h>
#include <linux/sched_clock.h>
#include <linux/log2.h>

struct pixel_data {
    int16_t pixelA;
    int16_t pixelR;
    int16_t pixelG;
    int16_t pixelB;
} g_pix_param;

static int dev_major;
static struct class *pixel_manager_class;

static int pixel_manager_open(struct inode *inode, struct file *filp)
{
	nonseekable_open(inode, filp);
	return 0;
}

static int pixel_manager_release(struct inode *inode, struct file *filp)
{
    return 0;
}

static int string_is_digital(const char *str)
{
	int i = 0;

	if (str == NULL) {
		return -1;
	}

	while (str[i] != '\0') {
		if (str[i] < '0' || str[i] > '9') {
			return -1;
		}
		i++;
	}

	return 0;
}

static int pixel_atoi(const char *str)
{
	int i = 0;
    int ret_value = 0;

	if (str == NULL) {
		return -1;
	}

	while (str[i] != '\0') {
		ret_value = ret_value * 10 + (str[i] - '0');
		i++;
	}

	return ret_value;
}

static ssize_t pixel_manager_write(struct file *filp,
		const char __user *buf, size_t count, loff_t *f_pos)
{
	char rgb_buf[64] = {0};
	char *find_str1 = NULL;
	char *find_str2 = NULL;
	char *find_str3 = NULL;
	char *find_str4 = NULL;
	char tmp_buf[16] = {0};

	if (copy_from_user(rgb_buf, buf, count))
		return -EFAULT;

	pr_info("pixel_manager_write:%s\n", rgb_buf);
	find_str1 = strstr(rgb_buf, "A:");
	find_str2 = strstr(rgb_buf, "R:");
	find_str3 = strstr(rgb_buf, "G:");
	find_str4 = strstr(rgb_buf, "B:");
	if (find_str1 != NULL && find_str2 != NULL && find_str2 - find_str1 > 2) {
		strncpy(tmp_buf, find_str1 + 2, find_str2 - find_str1 - 2);
		if (string_is_digital(tmp_buf) == 0) {
			g_pix_param.pixelA = pixel_atoi(tmp_buf);
		}
	}

	if (find_str2 != NULL && find_str3 != NULL && find_str3 - find_str2 > 2) {
		memset(tmp_buf, 0, sizeof(tmp_buf));
		strncpy(tmp_buf, find_str2 + 2, find_str3 - find_str2 - 2);
		if (string_is_digital(tmp_buf) == 0) {
			g_pix_param.pixelR = pixel_atoi(tmp_buf);
		}
	}

	if (find_str3 != NULL && find_str4 != NULL && find_str4 - find_str3 > 2) {
		memset(tmp_buf, 0, sizeof(tmp_buf));
		strncpy(tmp_buf, find_str3 + 2, find_str4 - find_str3 - 2);
		if (string_is_digital(tmp_buf) == 0) {
			g_pix_param.pixelG = pixel_atoi(tmp_buf);
		}
	}

	if (find_str4 != NULL) {
		memset(tmp_buf, 0, sizeof(tmp_buf));
		strcpy(tmp_buf, find_str4 + 2);
		if (string_is_digital(tmp_buf) == 0) {
			g_pix_param.pixelB = pixel_atoi(tmp_buf);
		}
	}
	return count;
}

void get_pix_rgb(int16_t *R, int16_t *G, int16_t *B)
{
	if (R == NULL || G == NULL || B == NULL) {
		return;
	}

	if (g_pix_param.pixelR > 255 || g_pix_param.pixelG > 255 || g_pix_param.pixelB > 255) {
		*R = -1;
		*G = -1;
		*B = -1;
		return;
	}

	*R = g_pix_param.pixelR;
	*G = g_pix_param.pixelG;
	*B = g_pix_param.pixelB;
	pr_info("get_pix_rgb %d:%d:%d\n", *R, *G, *B);
}
EXPORT_SYMBOL_GPL(get_pix_rgb);

static const struct file_operations pixel_manager_fops = {
	.owner          = THIS_MODULE,
	.open           = pixel_manager_open,
	.release        = pixel_manager_release,
	.write          = pixel_manager_write
};

static int __init pixel_manager_init(void)
{
	int ret;
	struct device *dev;

	dev_major = register_chrdev(0, "pix_manager", &pixel_manager_fops);
	if (dev_major < 0) {
		pr_err("Unable to get major\n");
		ret = dev_major;
		goto err_exit;
	}

	pixel_manager_class = class_create(THIS_MODULE, "pix_manager");
	if (IS_ERR(pixel_manager_class)) {
		pr_err("Failed to create class\n");
		ret = PTR_ERR(pixel_manager_class);
		goto err_chredev;
	}

	dev = device_create(pixel_manager_class, NULL, MKDEV(dev_major, 0),
		NULL, "pix_manager");
	if (IS_ERR(dev)) {
		pr_err("Failed to create device\n");
		ret = PTR_ERR(dev);
		goto err_class;
	}

	memset(&g_pix_param, -1, sizeof(g_pix_param));

	return 0;

err_class:
	class_destroy(pixel_manager_class);
err_chredev:
	unregister_chrdev(dev_major, "pix_manager");
err_exit:
	return ret;
}

static void __exit pixel_manager_exit(void)
{
	device_destroy(pixel_manager_class, MKDEV(dev_major, 0));
	class_destroy(pixel_manager_class);
	unregister_chrdev(dev_major, "pix_manager");
}

subsys_initcall(pixel_manager_init);
module_exit(pixel_manager_exit);

MODULE_DESCRIPTION("transfer rgb value");
MODULE_AUTHOR("Coosea");
MODULE_LICENSE("GPL v2");

