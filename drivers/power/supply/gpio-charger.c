// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *  Driver for chargers which report their online status through a GPIO pin
 */

#include <linux/device.h>
#include <linux/gpio.h> /* For legacy platform data */
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>

#include <linux/power/gpio-charger.h>

struct gpio_charger {
	unsigned int irq;
	unsigned int charge_status_irq;
	unsigned int capacity_level_irq;
	bool wakeup_enabled;

	struct power_supply *charger;
	struct power_supply_desc charger_desc;
	struct gpio_desc *gpiod;
	struct gpio_desc *charge_status;
	struct gpio_desc *capacity_level;
};

static irqreturn_t gpio_charger_irq(int irq, void *devid)
{
	struct power_supply *charger = devid;

	power_supply_changed(charger);

	return IRQ_HANDLED;
}

static inline struct gpio_charger *psy_to_gpio_charger(struct power_supply *psy)
{
	return power_supply_get_drvdata(psy);
}

static int gpio_charger_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct gpio_charger *gpio_charger = psy_to_gpio_charger(psy);
	int online;

	online = gpiod_get_value_cansleep(gpio_charger->gpiod);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = online;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (gpio_charger->charge_status && online) {
			if (gpiod_get_value_cansleep(gpio_charger->charge_status))
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		} else
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		if (gpio_charger->capacity_level && online) {
			if (gpiod_get_value_cansleep(gpio_charger->capacity_level))
				val->intval = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
			else
				val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		} else
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_type gpio_charger_get_type(struct device *dev)
{
	const char *chargetype;

	if (!device_property_read_string(dev, "charger-type", &chargetype)) {
		if (!strcmp("unknown", chargetype))
			return POWER_SUPPLY_TYPE_UNKNOWN;
		if (!strcmp("battery", chargetype))
			return POWER_SUPPLY_TYPE_BATTERY;
		if (!strcmp("ups", chargetype))
			return POWER_SUPPLY_TYPE_UPS;
		if (!strcmp("mains", chargetype))
			return POWER_SUPPLY_TYPE_MAINS;
		if (!strcmp("usb-sdp", chargetype))
			return POWER_SUPPLY_TYPE_USB;
		if (!strcmp("usb-dcp", chargetype))
			return POWER_SUPPLY_TYPE_USB;
		if (!strcmp("usb-cdp", chargetype))
			return POWER_SUPPLY_TYPE_USB;
		if (!strcmp("usb-aca", chargetype))
			return POWER_SUPPLY_TYPE_USB;
	}
	dev_warn(dev, "unknown charger type %s\n", chargetype);

	return POWER_SUPPLY_TYPE_UNKNOWN;
}

static int gpio_charger_get_irq(struct device *dev, void *dev_id,
				struct gpio_desc *gpio)
{
	int ret, irq = gpiod_to_irq(gpio);

	if (irq > 0) {
		ret = devm_request_any_context_irq(dev, irq, gpio_charger_irq,
						   IRQF_TRIGGER_RISING |
						   IRQF_TRIGGER_FALLING,
						   dev_name(dev),
						   dev_id);
		if (ret < 0) {
			dev_warn(dev, "Failed to request irq: %d\n", ret);
			irq = 0;
		}
	}

	return irq;
}

static enum power_supply_property gpio_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_STATUS /* Must always be last in the array. */
};

static int gpio_charger_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct gpio_charger_platform_data *pdata = dev->platform_data;
	struct power_supply_config psy_cfg = {};
	struct gpio_charger *gpio_charger;
	struct power_supply_desc *charger_desc;
	struct gpio_desc *charge_status, *capacity_level;
	int charge_status_irq, capacity_level_irq;
	unsigned long flags;
	int ret;

	if (!pdata && !dev->of_node) {
		dev_err(dev, "No platform data\n");
		return -ENOENT;
	}

	gpio_charger = devm_kzalloc(dev, sizeof(*gpio_charger), GFP_KERNEL);
	if (!gpio_charger)
		return -ENOMEM;

	/*
	 * This will fetch a GPIO descriptor from device tree, ACPI or
	 * boardfile descriptor tables. It's good to try this first.
	 */
	gpio_charger->gpiod = devm_gpiod_get(dev, NULL, GPIOD_IN);

	/*
	 * If this fails and we're not using device tree, try the
	 * legacy platform data method.
	 */
	if (IS_ERR(gpio_charger->gpiod) && !dev->of_node) {
		/* Non-DT: use legacy GPIO numbers */
		if (!gpio_is_valid(pdata->gpio)) {
			dev_err(dev, "Invalid gpio pin in pdata\n");
			return -EINVAL;
		}
		flags = GPIOF_IN;
		if (pdata->gpio_active_low)
			flags |= GPIOF_ACTIVE_LOW;
		ret = devm_gpio_request_one(dev, pdata->gpio, flags,
					    dev_name(dev));
		if (ret) {
			dev_err(dev, "Failed to request gpio pin: %d\n", ret);
			return ret;
		}
		/* Then convert this to gpiod for now */
		gpio_charger->gpiod = gpio_to_desc(pdata->gpio);
	} else if (IS_ERR(gpio_charger->gpiod)) {
		/* Just try again if this happens */
		if (PTR_ERR(gpio_charger->gpiod) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_err(dev, "error getting GPIO descriptor\n");
		return PTR_ERR(gpio_charger->gpiod);
	}

	charge_status = devm_gpiod_get_optional(dev, "charge-status", GPIOD_IN);
	gpio_charger->charge_status = charge_status;
	if (IS_ERR(gpio_charger->charge_status))
		return PTR_ERR(gpio_charger->charge_status);

	capacity_level = devm_gpiod_get_optional(dev, "capacity-level", GPIOD_IN);
	gpio_charger->capacity_level = capacity_level;
	if (IS_ERR(gpio_charger->capacity_level))
		return PTR_ERR(gpio_charger->capacity_level);

	charger_desc = &gpio_charger->charger_desc;
	charger_desc->properties = gpio_charger_properties;
	charger_desc->num_properties = ARRAY_SIZE(gpio_charger_properties);
	/* Remove POWER_SUPPLY_PROP_STATUS from the supported properties. */
	if (!gpio_charger->charge_status)
		charger_desc->num_properties -= 1;
	charger_desc->get_property = gpio_charger_get_property;

	psy_cfg.of_node = dev->of_node;
	psy_cfg.drv_data = gpio_charger;

	if (pdata) {
		charger_desc->name = pdata->name;
		charger_desc->type = pdata->type;
		psy_cfg.supplied_to = pdata->supplied_to;
		psy_cfg.num_supplicants = pdata->num_supplicants;
	} else {
		charger_desc->name = dev->of_node->name;
		charger_desc->type = gpio_charger_get_type(dev);
	}

	if (!charger_desc->name)
		charger_desc->name = pdev->name;

	gpio_charger->charger = devm_power_supply_register(dev, charger_desc,
							   &psy_cfg);
	if (IS_ERR(gpio_charger->charger)) {
		ret = PTR_ERR(gpio_charger->charger);
		dev_err(dev, "Failed to register power supply: %d\n", ret);
		return ret;
	}

	gpio_charger->irq = gpio_charger_get_irq(dev, gpio_charger->charger,
						 gpio_charger->gpiod);

	charge_status_irq = gpio_charger_get_irq(dev, gpio_charger->charger,
						 gpio_charger->charge_status);
	gpio_charger->charge_status_irq = charge_status_irq;

	capacity_level_irq = gpio_charger_get_irq(dev, gpio_charger->charger,
						 gpio_charger->capacity_level);
	gpio_charger->capacity_level_irq = capacity_level_irq;

	platform_set_drvdata(pdev, gpio_charger);

	device_init_wakeup(dev, 1);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int gpio_charger_suspend(struct device *dev)
{
	struct gpio_charger *gpio_charger = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		gpio_charger->wakeup_enabled =
			!enable_irq_wake(gpio_charger->irq);

	return 0;
}

static int gpio_charger_resume(struct device *dev)
{
	struct gpio_charger *gpio_charger = dev_get_drvdata(dev);

	if (device_may_wakeup(dev) && gpio_charger->wakeup_enabled)
		disable_irq_wake(gpio_charger->irq);
	power_supply_changed(gpio_charger->charger);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(gpio_charger_pm_ops,
		gpio_charger_suspend, gpio_charger_resume);

static const struct of_device_id gpio_charger_match[] = {
	{ .compatible = "gpio-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, gpio_charger_match);

static struct platform_driver gpio_charger_driver = {
	.probe = gpio_charger_probe,
	.driver = {
		.name = "gpio-charger",
		.pm = &gpio_charger_pm_ops,
		.of_match_table = gpio_charger_match,
	},
};

module_platform_driver(gpio_charger_driver);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("Driver for chargers which report their online status through a GPIO");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:gpio-charger");
