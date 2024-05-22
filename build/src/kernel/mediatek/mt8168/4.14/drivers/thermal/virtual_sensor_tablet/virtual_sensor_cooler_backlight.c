#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/types.h>
#include <linux/kobject.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/thermal_framework.h>

#define virtual_sensor_cooler_backlight_dprintk(fmt, args...) pr_debug("thermal/cooler/backlight " fmt, ##args)

#define MAX_BRIGHTNESS	LED_FULL
#define DRIVER_NAME "backlight_cooler"

static struct mutex bl_update_lock;
struct cooler_sort_list backlight_list_head;

static int virtual_sensor_get_max_state(struct thermal_cooling_device *cdev,
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

static int virtual_sensor_get_cur_state(struct thermal_cooling_device *cdev,
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

static int virtual_sensor_set_cur_state(struct thermal_cooling_device *cdev,
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
		level = MAX_BRIGHTNESS;
	else
		level = pdata->levels[pdata->state - 1];

	if (level == pdata->level)
		goto out;

	pdata->level = level;
	mutex_lock(&bl_update_lock);
	level = thermal_level_compare(pdata, &backlight_list_head, true);
	vs_set_cooling_level(cdev, VS_THERMAL_COOLER_BACKLIGHT, level);
	mutex_unlock(&bl_update_lock);

out:
	return 0;
}

static ssize_t levels_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
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

static ssize_t levels_store(struct device *dev, struct device_attribute *attr,
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
	.get_max_state = virtual_sensor_get_max_state,
	.get_cur_state = virtual_sensor_get_cur_state,
	.set_cur_state = virtual_sensor_set_cur_state,
};

static DEVICE_ATTR(levels, 0644, levels_show, levels_store);

static int backlight_probe(struct platform_device *pdev)
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
		pr_err("%s Failed to create backlight cooling device\n",
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
	mutex_lock(&bl_update_lock);
	list_add(&tz_list->list, &backlight_list_head.list);
	mutex_unlock(&bl_update_lock);
	ret = device_create_file(&pdata->cdev->device, &dev_attr_levels);
	if (ret)
		pr_err("%s Failed to create backlight cooler levels attr\n", __func__);

	platform_set_drvdata(pdev, pdata);
	return 0;

list_mem_err:
	thermal_cooling_device_unregister(pdata->cdev);
register_err:
	devm_kfree(&pdev->dev, pdata);

	return ret;
}

static int backlight_remove(struct platform_device *pdev)
{
	struct vs_cooler_platform_data *pdata = platform_get_drvdata(pdev);
	struct cooler_sort_list *cooler_list, *tmp;

	if (pdata) {
		mutex_lock(&bl_update_lock);
		list_for_each_entry_safe(cooler_list, tmp, &backlight_list_head.list, list) {
			if (cooler_list->pdata == pdata) {
				list_del(&cooler_list->list);
				kfree(cooler_list);
				cooler_list = NULL;
				break;
			}
		}
		mutex_unlock(&bl_update_lock);
		if (pdata->cdev) {
			device_remove_file(&pdata->cdev->device, &dev_attr_levels);
			thermal_cooling_device_unregister(pdata->cdev);
		}
		devm_kfree(&pdev->dev, pdata);
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id backlight_of_match_table[] = {
	{.compatible = "amazon,backlight_cooler", },
	{},
};
MODULE_DEVICE_TABLE(of, backlight_of_match_table);
#endif

static struct platform_driver backlight_driver = {
	.probe = backlight_probe,
	.remove = backlight_remove,
	.driver     = {
		.name  = DRIVER_NAME,
#ifdef CONFIG_OF
		.of_match_table = backlight_of_match_table,
#endif
		.owner = THIS_MODULE,
	},
};


static int __init virtual_sensor_cooler_backlight_init(void)
{
	int err = 0;

	virtual_sensor_cooler_backlight_dprintk("init\n");

	INIT_LIST_HEAD(&backlight_list_head.list);
	mutex_init(&bl_update_lock);
	err = platform_driver_register(&backlight_driver);
	if (err) {
		pr_err("%s: Failed to register driver %s\n", __func__,
			backlight_driver.driver.name);
		return err;
	}

	return 0;
}

static void __exit virtual_sensor_cooler_backlight_exit(void)
{
	virtual_sensor_cooler_backlight_dprintk("exit\n");

	platform_driver_unregister(&backlight_driver);
	mutex_destroy(&bl_update_lock);
}

module_init(virtual_sensor_cooler_backlight_init);
module_exit(virtual_sensor_cooler_backlight_exit);
