// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 Cadence
// Cadence PCIe platform  driver.
// Author: Tom Joseph <tjoseph@cadence.com>

#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/of_device.h>
#include "pcie-cadence.h"

/**
 * struct cdns_plat_pcie - private data for this PCIe platform driver
 * @pcie: Cadence PCIe controller
 * @regmap: pointer to PCIe device
 * @is_rc: Set to 1 indicates the PCIe controller mode is Root Complex,
 *         if 0 it is in Endpoint mode.
 */
struct cdns_plat_pcie {
	struct cdns_pcie        *pcie;
	bool is_rc;
};

struct cdns_plat_pcie_of_data {
	bool is_rc;
};

static const struct of_device_id cdns_plat_pcie_of_match[];

int cdns_plat_pcie_link_control(struct cdns_pcie *pcie, bool start)
{
	pr_debug(" %s called\n", __func__);
	return 0;
}

bool cdns_plat_pcie_link_status(struct cdns_pcie *pcie)
{
	pr_debug(" %s called\n", __func__);
	return 0;
}

static const struct cdns_pcie_common_ops cdns_pcie_common_ops = {
	.cdns_start_link = cdns_plat_pcie_link_control,
	.cdns_is_link_up = cdns_plat_pcie_link_status,
};

static int cdns_plat_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cdns_plat_pcie *cdns_plat_pcie;
	const struct of_device_id *match;
	const struct cdns_plat_pcie_of_data *data;
	struct pci_host_bridge *bridge;
	struct cdns_pcie_rc *rc;
	struct cdns_pcie_ep *ep;
	int ret;
	bool is_rc;

	match = of_match_device(cdns_plat_pcie_of_match, dev);
	if (!match)
		return -EINVAL;
	data = (struct cdns_plat_pcie_of_data *)match->data;
	is_rc = data->is_rc;

	pr_debug(" Started %s with is_rc: %d\n", __func__, is_rc);
	cdns_plat_pcie = devm_kzalloc(dev, sizeof(*cdns_plat_pcie), GFP_KERNEL);
	if (!cdns_plat_pcie)
		return -ENOMEM;

	platform_set_drvdata(pdev, cdns_plat_pcie);
	if (is_rc) {
		if (!IS_ENABLED(CONFIG_PCIE_CADENCE_PLAT_HOST))
			return -ENODEV;

		bridge = devm_pci_alloc_host_bridge(dev, sizeof(*rc));
		if (!bridge)
			return -ENOMEM;

		rc = pci_host_bridge_priv(bridge);
		rc->dev = dev;
		rc->pcie.ops = &cdns_pcie_common_ops;
		cdns_plat_pcie->pcie = &rc->pcie;
		cdns_plat_pcie->is_rc = is_rc;

		ret = cdns_pcie_host_setup(rc);
		if (ret < 0)
			return ret;
	} else {
		if (!IS_ENABLED(CONFIG_PCIE_CADENCE_PLAT_EP))
			return -ENODEV;

		ep = devm_kzalloc(dev, sizeof(*ep), GFP_KERNEL);
		if (!ep)
			return -ENOMEM;
		ep->dev = dev;
		ep->pcie.ops = &cdns_pcie_common_ops;
		cdns_plat_pcie->pcie = &ep->pcie;
		cdns_plat_pcie->is_rc = is_rc;

		ret = cdns_pcie_ep_setup(ep);
		if (ret < 0)
			return ret;
	}
	return 0;
}


static void cdns_plat_pcie_shutdown(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cdns_pcie *pcie = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_put_sync(dev);
	if (ret < 0)
		dev_dbg(dev, "pm_runtime_put_sync failed\n");

	pm_runtime_disable(dev);

	cdns_pcie_disable_phy(pcie);
}

static const struct cdns_plat_pcie_of_data cdns_plat_pcie_host_of_data = {
	.is_rc = true,
};

static const struct cdns_plat_pcie_of_data cdns_plat_pcie_ep_of_data = {
	.is_rc = false,
};

static const struct of_device_id cdns_plat_pcie_of_match[] = {
	{
		.compatible = "cdns,cdns-pcie-host",
		.data = &cdns_plat_pcie_host_of_data,
	},
	{
		.compatible = "cdns,cdns-pcie-ep",
		.data = &cdns_plat_pcie_ep_of_data,
	},
	{},
};

static struct platform_driver cdns_plat_pcie_driver = {
	.driver = {
		.name = "cdns-pcie",
		.of_match_table = cdns_plat_pcie_of_match,
		.pm	= &cdns_pcie_pm_ops,
	},
	.probe = cdns_plat_pcie_probe,
	.shutdown = cdns_plat_pcie_shutdown,
};
builtin_platform_driver(cdns_plat_pcie_driver);
