/**
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/extcon.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/pinctrl/consumer.h>

#include <linux/sched.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include "typec.h"
#include "tcpm.h"
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <charger_class.h>

#define TCPC_OTG_DEV_NAME "type_c_port0"

struct usb_extcon_info {
	struct device *dev;
	struct extcon_dev *edev;
	struct charger_device *primary_charger;

	struct tcpc_device *otg_tcpc_dev;
	struct notifier_block otg_nb;
};

static const unsigned int usb_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

static int otg_tcp_notifier_call(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct tcp_notify *noti = data;
	struct usb_extcon_info *info =
			container_of(nb, struct usb_extcon_info, otg_nb);
	struct device *dev = info->dev;
	struct charger_device *charger = info->primary_charger;
	int old_state, new_state, new_role;
	int ret;

	switch (event) {
	case TCP_NOTIFY_SOURCE_VBUS:
		if (charger) {
			if (noti->vbus_state.mv)
				ret = charger_dev_enable_otg(charger, true);
			else
				ret = charger_dev_enable_otg(charger, false);
			dev_dbg(dev, "charger dev enable %s\n",
					(ret ? "fail":"success"));
		}
		break;
	case TCP_NOTIFY_TYPEC_STATE:
		old_state = noti->typec_state.old_state;
		new_state = noti->typec_state.new_state;
		dev_dbg(dev, "TCP_NOTIFY_TYPEC_STATE, old:%d, new:%d\n",
				old_state, new_state);
		if (old_state == TYPEC_UNATTACHED) {
			if (new_state == TYPEC_ATTACHED_SRC)
				extcon_set_state_sync(info->edev,
							EXTCON_USB_HOST, true);
		} else if (new_state == TYPEC_UNATTACHED) {
			if (old_state == TYPEC_ATTACHED_SRC)
				extcon_set_state_sync(info->edev,
							EXTCON_USB_HOST, false);
		}
		break;
	case TCP_NOTIFY_DR_SWAP:
		new_role = noti->swap_state.new_role;
		dev_dbg(dev, "TCP_NOTIFY_DR_SWAP, new role=%d\n", new_role);
		if (new_role == PD_ROLE_UFP) {
			extcon_set_state_sync(info->edev,
						EXTCON_USB_HOST, false);
			extcon_set_state_sync(info->edev,
						EXTCON_USB, true);
		} else if (new_role == PD_ROLE_DFP) {
			extcon_set_state_sync(info->edev,
						EXTCON_USB, false);
			extcon_set_state_sync(info->edev,
						EXTCON_USB_HOST, true);
		}
		break;
	#ifdef TYPEC_EVENT_DEBUG
	case TCP_NOTIFY_SINK_VBUS:
		dev_dbg(dev, "source vbus = %dmv\n", noti->vbus_state.mv);
		break;
	case TCP_NOTIFY_ENTER_MODE:
		dev_dbg(dev, "TCP_NOTIFY_ENTER_MODE\n");
		break;
	case TCP_NOTIFY_EXIT_MODE:
		dev_dbg(dev, "TCP_NOTIFY_EXIT_MODE\n");
		break;
	case TCP_NOTIFY_AMA_DP_STATE:
		dev_dbg(dev, "TCP_NOTIFY_AMA_DP_STATE\n");
		break;
	case TCP_NOTIFY_AMA_DP_ATTENTION:
		dev_dbg(dev, "TCP_NOTIFY_AMA_DP_ATTENTION\n");
		break;
	case TCP_NOTIFY_AMA_DP_HPD_STATE:
		dev_dbg(dev, "TCP_NOTIFY_AMA_DP_HPD_STATE\n");
		break;
	case TCP_NOTIFY_DC_EN_UNLOCK:
		dev_dbg(dev, "TCP_NOTIFY_DC_EN_UNLOCK\n");
		break;
	case TCP_NOTIFY_UVDM:
		dev_dbg(dev, "TCP_NOTIFY_UVDM\n");
		break;
	case TCP_NOTIFY_DIS_VBUS_CTRL:
		dev_dbg(dev, "TCP_NOTIFY_DIS_VBUS_CTRL\n");
		break;
	case TCP_NOTIFY_SOURCE_VCONN:
		dev_dbg(dev, "TCP_NOTIFY_SOURCE_VCONN\n");
		break;
	case TCP_NOTIFY_EXT_DISCHARGE:
		dev_dbg(dev, "TCP_NOTIFY_EXT_DISCHARGE\n");
		break;
	case TCP_NOTIFY_ATTACHWAIT_SNK:
		dev_dbg(dev, "TCP_NOTIFY_ATTACHWAIT_SNK\n");
		break;
	case TCP_NOTIFY_ATTACHWAIT_SRC:
		dev_dbg(dev, "TCP_NOTIFY_ATTACHWAIT_SRC\n");
		break;
	case TCP_NOTIFY_PD_STATE:
		dev_dbg(dev, "TCP_NOTIFY_PD_STATE\n");
		break;
	case TCP_NOTIFY_PR_SWAP:
		dev_dbg(dev, "TCP_NOTIFY_PR_SWAP\n");
		break;
	case TCP_NOTIFY_VCONN_SWAP:
		dev_dbg(dev, "TCP_NOTIFY_VCONN_SWAP\n");
		break;
	case TCP_NOTIFY_HARD_RESET_STATE:
		dev_dbg(dev, "TCP_NOTIFY_HARD_RESET_STATE\n");
		break;
	case TCP_NOTIFY_ALERT:
		dev_dbg(dev, "TCP_NOTIFY_ALERT\n");
		break;
	case TCP_NOTIFY_STATUS:
		dev_dbg(dev, "TCP_NOTIFY_STATUS\n");
		break;
	case TCP_NOTIFY_REQUEST_BAT_INFO:
		dev_dbg(dev, "TCP_NOTIFY_REQUEST_BAT_INFO\n");
		break;
	#endif
	default:
		dev_dbg(dev, "default, event:%ld\n", event);
	}
	return NOTIFY_OK;
}

static void register_extcon_typec(struct usb_extcon_info *info)
{
	static int ret;
	struct device *dev = info->dev;

	info->otg_tcpc_dev = tcpc_dev_get_by_name(TCPC_OTG_DEV_NAME);
	if (!info->otg_tcpc_dev) {
		dev_err(dev, "get type_c_port0 fail\n");
		return;
	}

	info->otg_nb.notifier_call = otg_tcp_notifier_call;
	ret = register_tcp_dev_notifier(info->otg_tcpc_dev, &info->otg_nb,
			TCP_NOTIFY_TYPE_VBUS | TCP_NOTIFY_TYPE_USB |
			TCP_NOTIFY_TYPE_MISC);
	if (ret < 0) {
		dev_err(dev, "register tcp notifier fail\n");
		return;
	}
}

static int usb_extcon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct usb_extcon_info *info;
	int ret;

	if (!np)
		return -EINVAL;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = dev;
	info->edev = devm_extcon_dev_allocate(dev, usb_extcon_cable);
	if (IS_ERR(info->edev)) {
		dev_err(dev, "failed to allocate extcon device\n");
		return -ENOMEM;
	}

	ret = devm_extcon_dev_register(dev, info->edev);
	if (ret < 0) {
		dev_err(dev, "failed to register extcon device\n");
		return ret;
	}

	info->primary_charger = get_charger_by_name("primary_chg");
	if (!info->primary_charger)
		dev_err(dev, "fail to get primary charger\n");

	platform_set_drvdata(pdev, info);
	register_extcon_typec(info);

	return 0;
}

static int usb_extcon_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int usb_extcon_suspend(struct device *dev)
{
	return 0;
}

static int usb_extcon_resume(struct device *dev)
{
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(usb_extcon_pm_ops,
		usb_extcon_suspend, usb_extcon_resume);

static const struct of_device_id usb_extcon_dt_match[] = {
	{ .compatible = "linux,extcon-usb-typec", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, usb_extcon_dt_match);

static const struct platform_device_id usb_extcon_platform_ids[] = {
	{ .name = "extcon-usb-typec", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, usb_extcon_platform_ids);

static struct platform_driver usb_extcon_driver = {
	.probe          = usb_extcon_probe,
	.remove         = usb_extcon_remove,
	.driver         = {
		.name   = "extcon-usb-typec",
		.pm     = &usb_extcon_pm_ops,
		.of_match_table = usb_extcon_dt_match,
	},
	.id_table = usb_extcon_platform_ids,
};

module_platform_driver(usb_extcon_driver);

MODULE_AUTHOR("Jumin Li <jumin.li@mediatek.com>");
MODULE_DESCRIPTION("USB typec extcon driver");
MODULE_LICENSE("GPL v2");
