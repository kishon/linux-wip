// SPDX-License-Identifier: GPL-2.0
/**
 * pci-j721e - PCIe controller driver for TI's J721E SoCs
 *
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
 */

#include <dt-bindings/pci/pci.h>
#include <linux/io.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/mfd/syscon.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include "../pci.h"
#include "pcie-cadence.h"

#define J721E_PCIE_USER_CMD_STATUS	0x4
#define LINK_TRAINING_ENABLE		BIT(0)

#define J721E_PCIE_USER_LINKSTATUS	0x14
#define LINK_STATUS			GENMASK(1, 0)

enum link_status {
	NO_RECIEVERS_DETECTED,
	LINK_TRAINING_IN_PROGRESS,
	LINK_UP_DL_IN_PROGRESS,
	LINK_UP_DL_COMPLETED,
};

#define J721E_MODE_RC			BIT(7)
#define LANE_COUNT_MASK			BIT(8)
#define LANE_COUNT(n)			((n) << 8)

#define GENERATION_SEL_MASK		GENMASK(1, 0)

#define MAX_LANES			2

struct j721e_pcie {
	struct device		*dev;
	struct device_node	*node;
	u32			mode;
	u32			num_lanes;
	void __iomem		*intd_cfg_base;
	void __iomem		*user_cfg_base;
};

struct j721e_pcie_data {
	u32			mode;
};

static inline u32 j721e_pcie_intd_readl(struct j721e_pcie *pcie, u32 offset)
{
	return readl(pcie->intd_cfg_base + offset);
}

static inline void j721e_pcie_intd_writel(struct j721e_pcie *pcie, u32 offset,
					  u32 value)
{
	writel(value, pcie->intd_cfg_base + offset);
}

static inline u32 j721e_pcie_user_readl(struct j721e_pcie *pcie, u32 offset)
{
	return readl(pcie->user_cfg_base + offset);
}

static inline void j721e_pcie_user_writel(struct j721e_pcie *pcie, u32 offset,
					  u32 value)
{
	writel(value, pcie->user_cfg_base + offset);
}

static int j721e_pcie_link_control(struct cdns_pcie *cdns_pcie, bool start)
{
	struct j721e_pcie *pcie = dev_get_drvdata(cdns_pcie->dev);
	u32 reg;

	reg = j721e_pcie_user_readl(pcie, J721E_PCIE_USER_CMD_STATUS);
	if (start)
		reg |= LINK_TRAINING_ENABLE;
	else
		reg &= ~LINK_TRAINING_ENABLE;
	j721e_pcie_user_writel(pcie, J721E_PCIE_USER_CMD_STATUS, reg);

	return 0;
}

static bool j721e_pcie_is_link_up(struct cdns_pcie *cdns_pcie)
{
	struct j721e_pcie *pcie = dev_get_drvdata(cdns_pcie->dev);
	u32 reg;

	reg = j721e_pcie_user_readl(pcie, J721E_PCIE_USER_LINKSTATUS);
	reg &= LINK_STATUS;
	if (reg == LINK_UP_DL_COMPLETED)
		return true;

	return false;
}

static const struct cdns_pcie_common_ops j721e_ops_ops = {
	.cdns_start_link = j721e_pcie_link_control,
	.cdns_is_link_up = j721e_pcie_is_link_up,
};

static int j721e_pcie_set_mode(struct j721e_pcie *pcie, struct regmap *syscon)
{
	struct device *dev = pcie->dev;
	u32 mask = J721E_MODE_RC;
	u32 mode = pcie->mode;
	u32 val = 0;
	int ret = 0;

	if (mode == PCI_MODE_RC)
		val = J721E_MODE_RC;

	ret = regmap_update_bits(syscon, 0, mask, val);
	if (ret)
		dev_err(dev, "failed to set pcie mode\n");

	return ret;
}

static int j721e_pcie_set_link_speed(struct j721e_pcie *pcie,
				     struct regmap *syscon)
{
	struct device *dev = pcie->dev;
	struct device_node *np = dev->of_node;
	int link_speed;
	u32 val = 0;
	int ret;

	link_speed = of_pci_get_max_link_speed(np);
	if (link_speed < 2)
		link_speed = 2;

	val = link_speed - 1;
	ret = regmap_update_bits(syscon, 0, GENERATION_SEL_MASK, val);
	if (ret)
		dev_err(dev, "failed to set link speed\n");

	return ret;
}

static int j721e_pcie_set_lane_count(struct j721e_pcie *pcie,
				     struct regmap *syscon)
{
	struct device *dev = pcie->dev;
	u32 lanes = pcie->num_lanes;
	u32 val = 0;
	int ret;

	val = LANE_COUNT(lanes - 1);
	ret = regmap_update_bits(syscon, 0, LANE_COUNT_MASK, val);
	if (ret)
		dev_err(dev, "failed to set link count\n");

	return ret;
}

static int j721e_pcie_ctrl_init(struct j721e_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct device_node *node = dev->of_node;
	struct regmap *syscon;
	int ret;

	syscon = syscon_regmap_lookup_by_phandle(node, "ti,syscon-pcie-ctrl");
	if (IS_ERR(syscon)) {
		dev_err(dev, "Unable to get ti,syscon-pcie-ctrl regmap\n");
		return PTR_ERR(syscon);
	}

	ret = j721e_pcie_set_mode(pcie, syscon);
	if (ret < 0) {
		dev_err(dev, "Failed to set pci mode\n");
		return ret;
	}

	ret = j721e_pcie_set_link_speed(pcie, syscon);
	if (ret < 0) {
		dev_err(dev, "Failed to set link speed\n");
		return ret;
	}

	ret = j721e_pcie_set_lane_count(pcie, syscon);
	if (ret < 0) {
		dev_err(dev, "Failed to set num-lanes\n");
		return ret;
	}

	return 0;
}

static int cdns_ti_pcie_config_read(struct pci_bus *bus, unsigned int devfn,
				    int where, int size, u32 *value)
{
	struct pci_host_bridge *bridge = pci_find_host_bridge(bus);
	struct cdns_pcie_rc *rc = pci_host_bridge_priv(bridge);
	unsigned int busn = bus->number;

	if (busn == rc->bus_range->start)
		return pci_generic_config_read32(bus, devfn, where, size,
						 value);

	return pci_generic_config_read(bus, devfn, where, size, value);
}

static int cdns_ti_pcie_config_write(struct pci_bus *bus, unsigned int devfn,
				     int where, int size, u32 value)
{
	struct pci_host_bridge *bridge = pci_find_host_bridge(bus);
	struct cdns_pcie_rc *rc = pci_host_bridge_priv(bridge);
	unsigned int busn = bus->number;

	if (busn == rc->bus_range->start)
		return pci_generic_config_write32(bus, devfn, where, size,
						  value);

	return pci_generic_config_write(bus, devfn, where, size, value);
}

static struct pci_ops cdns_ti_pcie_host_ops = {
	.map_bus	= cdns_pci_map_bus,
	.read		= cdns_ti_pcie_config_read,
	.write		= cdns_ti_pcie_config_write,
};

static const struct j721e_pcie_data j721e_pcie_rc_data = {
	.mode = PCI_MODE_RC,
};

static const struct j721e_pcie_data j721e_pcie_ep_data = {
	.mode = PCI_MODE_EP,
};

static const struct of_device_id of_j721e_pcie_match[] = {
	{
		.compatible = "ti,j721e-cdns-pcie-host",
		.data = &j721e_pcie_rc_data,
	},
	{},
};

static int j721e_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	const struct of_device_id *match;
	struct pci_host_bridge *bridge;
	struct j721e_pcie_data *data;
	struct j721e_pcie *pcie;
	struct cdns_pcie_rc *rc;
	struct cdns_pcie_ep *ep;
	struct resource *res;
	void __iomem *base;
	u32 num_lanes;
	u32 mode;
	int ret;

        match = of_match_device(of_match_ptr(of_j721e_pcie_match), dev);
        if (!match)
                return -EINVAL;

	data = (struct j721e_pcie_data *)match->data;
	mode = (u32)data->mode;

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pcie->dev = dev;
	pcie->node = node;
	pcie->mode = mode;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "intd_cfg");
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);
	pcie->intd_cfg_base = base;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "user_cfg");
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);
	pcie->user_cfg_base = base;

	ret = of_property_read_u32(node, "num-lanes", &num_lanes);
	if (ret || num_lanes > MAX_LANES)
		num_lanes = 1;
	pcie->num_lanes = num_lanes;

	dev_set_drvdata(dev, pcie);
	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_get_sync failed\n");
		goto err_get_sync;
	}

	ret = j721e_pcie_ctrl_init(pcie);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_get_sync failed\n");
		goto err_get_sync;
	}

	switch (mode) {
	case PCI_MODE_RC:
		if (!IS_ENABLED(CONFIG_PCIE_CADENCE_HOST)) {
			ret = -ENODEV;
			goto err_get_sync;
		}

		bridge = devm_pci_alloc_host_bridge(dev, sizeof(*rc));
		if (!bridge) {
			ret = -ENOMEM;
			goto err_get_sync;
		}

		bridge->ops = &cdns_ti_pcie_host_ops;
		rc = pci_host_bridge_priv(bridge);
		rc->dev = dev;
		rc->pcie.dev = dev;
		rc->pcie.ops = &j721e_ops_ops;

		ret = cdns_pcie_host_setup(rc);
		if (ret < 0)
			goto err_get_sync;

		break;
	case PCI_MODE_EP:
		if (!IS_ENABLED(CONFIG_PCIE_CADENCE_EP)) {
			ret = -ENODEV;
			goto err_get_sync;
		}

		ep = devm_kzalloc(dev, sizeof(*ep), GFP_KERNEL);
		if (!ep) {
			ret = -ENOMEM;
			goto err_get_sync;
		}

		ep->dev = dev;
		ep->pcie.dev = dev;
		ep->pcie.ops = &j721e_ops_ops;

		ret = cdns_pcie_ep_setup(ep);
		if (ret < 0)
			goto err_get_sync;

		break;
	default:
		dev_err(dev, "INVALID device type %d\n", mode);
	}

	return 0;

err_get_sync:
	pm_runtime_put(dev);
	pm_runtime_disable(dev);

	return ret;
}

static int j721e_pcie_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	pm_runtime_put(dev);
	pm_runtime_disable(dev);
	of_platform_depopulate(dev);

	return 0;
}

static struct platform_driver j721e_pcie_driver = {
	.probe  = j721e_pcie_probe,
	.remove = j721e_pcie_remove,
	.driver = {
		.name	= "j721e-pcie",
		.of_match_table = of_j721e_pcie_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(j721e_pcie_driver);
