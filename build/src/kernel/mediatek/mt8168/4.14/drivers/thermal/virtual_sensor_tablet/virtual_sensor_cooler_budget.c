#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dmi.h>
#include <linux/acpi.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>
#include <mt-plat/aee.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/wait.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/thermal_framework.h>

#if defined (CONFIG_AMAZON_METRICS_LOG) || defined (CONFIG_AMAZON_MINERVA_METRICS_LOG)
#include <linux/metricslog.h>

#ifndef THERMO_METRICS_STR_LEN
#define THERMO_METRICS_STR_LEN 128
#endif
#endif

#define DRIVER_NAME "thermal_budget_cooler"
#define MAXIMUM_CPU_POWER 4600

struct cooler_sort_list thermal_budget_list_head;

struct thermal_dev_t {
	int major, minor;
	int flag;
	dev_t devno;
	struct class *cls;
	struct mutex mutex;
	struct cdev cdev;
	wait_queue_head_t queue;
	struct vs_cooler_platform_data *pdata;
};

static struct thermal_dev_t *tdev;
static int _thermal_budget;
static int _thermal_budget_cooler;
static int _thermal_budget_cpu;

static DEFINE_MUTEX(notify_mutex);

#define USE_KERNEL_API_DIRECTLY 1

static void _thermal_budget_notify(int budget1, int budget2)
{
	int thermal_budget = min(budget1, budget2);

	if (thermal_budget == _thermal_budget)
		return;

	_thermal_budget = thermal_budget;

#if USE_KERNEL_API_DIRECTLY
	vs_set_cooling_level(NULL, VS_THERMAL_COOLER_BUDGET,
		(_thermal_budget < MAXIMUM_CPU_POWER) ? _thermal_budget : 0);
#else
	if (tdev) {
		tdev->flag = 1;
		wake_up_interruptible(&tdev->queue);
	}
#endif
}

void thermal_budget_notify(int budget)
{
	mutex_lock(&notify_mutex);

	_thermal_budget_cpu = budget;
	if (budget == 0)
		_thermal_budget_cpu = MAXIMUM_CPU_POWER;
	_thermal_budget_notify(_thermal_budget_cpu, _thermal_budget_cooler);

	mutex_unlock(&notify_mutex);
}

int thermal_budget_get(void)
{
	return _thermal_budget;
}

static int thermal_budget_open(struct inode *inod, struct file *filp)
{
	return 0;
}

static ssize_t thermal_budget_write(struct file *filp,
				    const char __user *buf,
				    size_t len, loff_t *off)
{
	unsigned int budget = 0;
	int ret;

	ret = kstrtouint_from_user(buf, len, 10, &budget);
	if (budget)
		thermal_budget_notify(budget);
	else
		thermal_budget_notify(MAXIMUM_CPU_POWER);
	return len;
}

static ssize_t thermal_budget_read(struct file *filp,
				   char __user *buf, size_t len, loff_t *off)
{
	char budget[30];
	int s;

	wait_event_interruptible(tdev->queue, (tdev->flag == 1));
	tdev->flag = 0;
	s = snprintf(budget, 30, "%d\n", _thermal_budget);
	if (s > len)
		s = len;
	if (copy_to_user(buf, budget, s))
		return -EFAULT;
	return s;
}

static unsigned int thermal_budget_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;

	poll_wait(filp, &tdev->queue, wait);
	if (tdev->flag == 1)
		mask = POLLIN | POLLRDNORM;
	return mask;
}

static int thermal_budget_release(struct inode *inod, struct file *filp)
{
	return 0;
}

static ssize_t thermal_budget_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, 30, "%d\n", _thermal_budget);
}

static int thermal_budget_get_max_state(struct thermal_cooling_device *cdev,
					unsigned long *state)
{
	struct vs_cooler_platform_data *pdata;

	if (!cdev || !(cdev->devdata)) {
		pr_err("%s cdev: %p or cdev->devdata is NULL!\n", __func__, cdev);
		return -EINVAL;
	}
	pdata = cdev->devdata;
	*state = pdata->max_state;
	return 0;
}

static int thermal_budget_get_cur_state(struct thermal_cooling_device *cdev,
					unsigned long *state)
{
	struct vs_cooler_platform_data *pdata;

	if (!cdev || !(cdev->devdata)) {
		pr_err("%s cdev: %p or cdev->devdata is NULL!\n", __func__, cdev);
		return -EINVAL;
	}
	pdata = cdev->devdata;
	*state = pdata->state;
	return 0;
}

static int thermal_budget_set_cur_state(struct thermal_cooling_device *cdev,
					unsigned long state)
{
	int level;
	struct vs_cooler_platform_data *pdata;

	if (!cdev || !(cdev->devdata)) {
		pr_err("%s cdev: %p or cdev->devdata is NULL!\n", __func__, cdev);
		return -EINVAL;
	}
	pdata = cdev->devdata;
	if (pdata->state == state)
		return 0;

	pdata->state = (state > pdata->max_state) ? pdata->max_state : state;
	if (!pdata->state)
		level = MAXIMUM_CPU_POWER;
	else
		level = pdata->levels[pdata->state - 1];

	pdata->level = level;
	mutex_lock(&notify_mutex);
	level = thermal_level_compare(pdata, &thermal_budget_list_head, true);
	_thermal_budget_cooler = level;
	_thermal_budget_notify(_thermal_budget_cpu, level);
	mutex_unlock(&notify_mutex);
	return 0;
}

static ssize_t levels_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct thermal_cooling_device *cdev =
	    container_of(dev, struct thermal_cooling_device, device);
	struct vs_cooler_platform_data *pdata;
	int i;
	int offset = 0;
	size_t bufsz = 0;

	if (!cdev || !(cdev->devdata)) {
		pr_err("%s cdev: %p or cdev->devdata is NULL!\n", __func__, cdev);
		return -EINVAL;
	}
	pdata = cdev->devdata;
	bufsz = pdata->max_state * 20;
	for (i = 0; i < pdata->max_state; i++)
		offset +=
		    scnprintf(buf + offset, bufsz - offset, "%d %d\n", i + 1, pdata->levels[i]);
	return offset;
}

static ssize_t levels_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	int level, state;
	struct thermal_cooling_device *cdev =
	    container_of(dev, struct thermal_cooling_device, device);
	struct vs_cooler_platform_data *pdata;

	if (!cdev || !(cdev->devdata)) {
		pr_err("%s cdev: %p or cdev->devdata is NULL!\n", __func__, cdev);
		return -EINVAL;
	}
	pdata = cdev->devdata;
	if (sscanf(buf, "%d %d\n", &state, &level) != 2)
		return -EINVAL;
	if (state >= pdata->max_state)
		return -EINVAL;
	pdata->levels[state] = level;
	return count;
}

static struct thermal_cooling_device_ops cooling_ops = {
	.get_max_state = thermal_budget_get_max_state,
	.get_cur_state = thermal_budget_get_cur_state,
	.set_cur_state = thermal_budget_set_cur_state,
};

static const struct file_operations thermal_budget_fops = {
	.owner = THIS_MODULE,
	.open = thermal_budget_open,
	.read = thermal_budget_read,
	.write = thermal_budget_write,
	.poll = thermal_budget_poll,
	.release = thermal_budget_release,
};

static DEVICE_ATTR(thermal_budget, 0444, thermal_budget_show, NULL);
static DEVICE_ATTR(levels, 0644, levels_show, levels_store);

static int thermal_budget_probe(struct platform_device *pdev)
{
	int result;

	tdev = kzalloc(sizeof(struct thermal_dev_t), GFP_KERNEL);
	if (!tdev) {
		pr_err("%s Failed to create thermal budget driver\n",
			__func__);
		return -ENOMEM;
	}

	result = alloc_chrdev_region(&tdev->devno, 0, 1, "thermal_budget");
	if (result) {
		pr_err("%s: Can't get major %d\n", __func__, tdev->major);
		goto alloc_chrdev_err;
	}

	cdev_init(&tdev->cdev, &thermal_budget_fops);
	tdev->cdev.owner = THIS_MODULE;

	result = cdev_add(&tdev->cdev, tdev->devno, 1);
	if (result) {
		pr_err("%s: Failed to add thermal_budget device\n", __func__);
		goto cdev_add_err;
	}

	tdev->cls = class_create(THIS_MODULE, "thermal_notify");
	if (IS_ERR(tdev->cls)) {
		pr_alert("thermal_budget: failed in creating class\n");
		result = -EINVAL;
		goto class_create_err;
	}

	tdev->major = MAJOR(tdev->devno);
	tdev->minor = MINOR(tdev->devno);
	device_create(tdev->cls, NULL, MKDEV(tdev->major, 0),
		      NULL, "thermal_budget");

	mutex_init(&tdev->mutex);
	init_waitqueue_head(&tdev->queue);
	result = device_create_file(&pdev->dev, &dev_attr_thermal_budget);
	if (result)
		pr_err("%s Failed to create thermal_budget attr\n", __func__);

	_thermal_budget = _thermal_budget_cpu =
	    _thermal_budget_cooler = MAXIMUM_CPU_POWER;
	return result;

class_create_err:
	cdev_del(&tdev->cdev);
cdev_add_err:
	unregister_chrdev_region(tdev->devno, 1);
alloc_chrdev_err:
	kfree(tdev);
	return result;
}

static int thermal_budget_remove(struct platform_device *pdev)
{
	cdev_del(&tdev->cdev);
	unregister_chrdev_region(tdev->devno, 1);
	if (tdev->cls) {
		device_destroy(tdev->cls, MKDEV(tdev->major, 0));
		class_destroy(tdev->cls);
	}
	mutex_destroy(&tdev->mutex);
	kfree(tdev);
	if (pdev)
		device_remove_file(&pdev->dev, &dev_attr_thermal_budget);
	return 0;
}

static struct platform_driver thermal_budget_driver = {
	.driver = {
		   .name = "thermal_budget",
		   .owner = THIS_MODULE,
		   },
	.probe = thermal_budget_probe,
	.remove = thermal_budget_remove,
};

static struct platform_device thermal_budget_device = {
	.name = "thermal_budget",
	.id = -1,
};

static int budget_cooler_probe(struct platform_device *pdev)
{
	struct vs_cooler_platform_data *pdata = NULL;
	struct cooler_sort_list *tz_list = NULL;
	int ret = 0;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

#ifdef CONFIG_OF
	pr_notice("cooler custom init by DTS!\n");
	cooler_init_cust_data_from_dt(pdev, pdata);
#endif

	pdata->cdev = thermal_cooling_device_register(pdata->type,
							pdata,
							&cooling_ops);
	if (!pdata->cdev) {
		pr_err("%s Failed to create thermal_budget cooling device\n",
			__func__);
		ret = -EINVAL;
		goto register_err;
	}

	tz_list = kzalloc(sizeof(struct cooler_sort_list), GFP_KERNEL);
	if (!tz_list) {
		pr_err("%s Failed to allocate cooler_sort_list memory\n",
			__func__);
		ret = -ENOMEM;
		goto list_mem_err;
	}
	tz_list->pdata = pdata;
	mutex_lock(&notify_mutex);
	list_add(&tz_list->list, &thermal_budget_list_head.list);
	mutex_unlock(&notify_mutex);
	ret = device_create_file(&pdata->cdev->device, &dev_attr_levels);
	if (ret)
		pr_err("%s Failed to create budget cooler levels attr\n", __func__);

	platform_set_drvdata(pdev, pdata);
	return 0;

list_mem_err:
	thermal_cooling_device_unregister(pdata->cdev);
register_err:
	devm_kfree(&pdev->dev, pdata);
	return ret;
}

static int budget_cooler_remove(struct platform_device *pdev)
{
	struct vs_cooler_platform_data *pdata = platform_get_drvdata(pdev);
	struct cooler_sort_list *cooler_list, *tmp;

	if (pdata) {
		mutex_lock(&notify_mutex);
		list_for_each_entry_safe(cooler_list, tmp,
						&thermal_budget_list_head.list, list) {
			if (cooler_list->pdata == pdata) {
				list_del(&cooler_list->list);
				kfree(cooler_list);
				cooler_list = NULL;
				break;
			}
		}
		mutex_unlock(&notify_mutex);
		if (pdata->cdev) {
			device_remove_file(&pdata->cdev->device, &dev_attr_levels);
			thermal_cooling_device_unregister(pdata->cdev);
		}
		devm_kfree(&pdev->dev, pdata);
	}
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id thermal_budget_of_match_table[] = {
	{.compatible = "amazon,thermal_budget_cooler", },
	{},
};
MODULE_DEVICE_TABLE(of, thermal_budget_of_match_table);
#endif

static struct platform_driver budget_cooler_driver = {
	.probe = budget_cooler_probe,
	.remove = budget_cooler_remove,
	.driver     = {
		.name  = DRIVER_NAME,
#ifdef CONFIG_OF
		.of_match_table = thermal_budget_of_match_table,
#endif
		.owner = THIS_MODULE,
	},
};

static int __init thermal_budget_init(void)
{
	int ret;

	ret = platform_driver_register(&thermal_budget_driver);
	if (ret) {
		pr_err("Unable to register thermal budget driver (%d)\n", ret);
		return ret;
	}
	ret = platform_device_register(&thermal_budget_device);
	if (ret) {
		pr_err("Unable to register thermal budget device (%d)\n", ret);
		platform_driver_unregister(&thermal_budget_driver);
		return ret;
	}

	INIT_LIST_HEAD(&thermal_budget_list_head.list);

	ret = platform_driver_register(&budget_cooler_driver);
	if (ret) {
		pr_err("%s: Failed to register driver %s\n", __func__,
			budget_cooler_driver.driver.name);
		platform_driver_unregister(&thermal_budget_driver);
		platform_device_unregister(&thermal_budget_device);
		return ret;
	}

	return 0;
}

static void __exit thermal_budget_exit(void)
{
	platform_driver_unregister(&thermal_budget_driver);
	platform_device_unregister(&thermal_budget_device);
	platform_driver_unregister(&budget_cooler_driver);
}

module_init(thermal_budget_init);
module_exit(thermal_budget_exit);
