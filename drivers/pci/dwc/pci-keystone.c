// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2018 Texas Instruments
// PCIe host controller driver for Texas Instruments Keystone SoCs.
// Author:  Murali Karicheri <m-karicheri2@ti.com>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/mfd/syscon.h>
#include <linux/msi.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/resource.h>
#include <linux/signal.h>

#include "pcie-designware.h"
#include "pci-keystone.h"

#define PCIE_VENDORID_MASK	0xffff
#define PCIE_DEVICEID_SHIFT	16

#define to_keystone_pcie(x)	dev_get_drvdata((x)->dev)

/*
 * Keystone PCI controller has a h/w limitation of
 * 256 bytes maximum read request size. It can't handle
 * anything higher than this. So force this limit on
 * all downstream devices.
 */
static void ks_pcie_quirk(struct pci_dev *dev)
{
	struct pci_bus *bus = dev->bus;
	struct pci_dev *bridge;
	static const struct pci_device_id rc_pci_devids[] = {
		{ PCI_DEVICE(PCI_VENDOR_ID_TI, PCI_DEVICE_ID_TI_K2HK),
		 .class = PCI_CLASS_BRIDGE_PCI << 8, .class_mask = ~0, },
		{ PCI_DEVICE(PCI_VENDOR_ID_TI, PCI_DEVICE_ID_TI_K2E),
		 .class = PCI_CLASS_BRIDGE_PCI << 8, .class_mask = ~0, },
		{ PCI_DEVICE(PCI_VENDOR_ID_TI, PCI_DEVICE_ID_TI_K2L),
		 .class = PCI_CLASS_BRIDGE_PCI << 8, .class_mask = ~0, },
		{ PCI_DEVICE(PCI_VENDOR_ID_TI, PCI_DEVICE_ID_TI_K2G),
		 .class = PCI_CLASS_BRIDGE_PCI << 8, .class_mask = ~0, },
		{ 0, },
	};

	if (pci_is_root_bus(bus))
		bridge = dev;

	/* look for the host bridge */
	while (!pci_is_root_bus(bus)) {
		bridge = bus->self;
		bus = bus->parent;
	}

	if (!bridge)
		return;

	if (pci_match_id(rc_pci_devids, bridge)) {
		if (pcie_get_readrq(dev) > 256) {
			dev_info(&dev->dev, "limiting MRRS to 256\n");
			pcie_set_readrq(dev, 256);
		}
	}
}
DECLARE_PCI_FIXUP_ENABLE(PCI_ANY_ID, PCI_ANY_ID, ks_pcie_quirk);

static void ks_pcie_msi_irq_handler(struct irq_desc *desc)
{
	unsigned int irq = irq_desc_get_irq(desc);
	struct keystone_pcie *ks_pcie = irq_desc_get_handler_data(desc);
	u32 offset = irq - ks_pcie->msi_host_irqs[0];
	struct dw_pcie *pci = ks_pcie->pci;
	struct device *dev = pci->dev;
	struct irq_chip *chip = irq_desc_get_chip(desc);

	dev_dbg(dev, "%s, irq %d\n", __func__, irq);

	chained_irq_enter(chip, desc);
	ks_dw_pcie_handle_msi_irq(ks_pcie, offset);
	chained_irq_exit(chip, desc);
}

static void ks_pcie_legacy_irq_handler(struct irq_desc *desc)
{
	unsigned int irq = irq_desc_get_irq(desc);
	struct keystone_pcie *ks_pcie = irq_desc_get_handler_data(desc);
	struct dw_pcie *pci = ks_pcie->pci;
	struct device *dev = pci->dev;
	u32 offset = irq - ks_pcie->legacy_host_irqs[0];
	struct irq_chip *chip = irq_desc_get_chip(desc);

	dev_dbg(dev, ": Handling legacy irq %d\n", irq);

	chained_irq_enter(chip, desc);
	ks_dw_pcie_handle_legacy_irq(ks_pcie, offset);
	chained_irq_exit(chip, desc);
}

static int ks_pcie_get_irq_controller_info(struct keystone_pcie *ks_pcie,
					   bool legacy)
{
	int i;
	char *controller;
	int irq_count;
	int *num_irqs, **host_irqs;
	struct device *dev = ks_pcie->pci->dev;
	struct device_node *np = ks_pcie->np, **intc_np;

	if (legacy) {
		intc_np = &ks_pcie->legacy_intc_np;
		host_irqs = &ks_pcie->legacy_host_irqs;
		num_irqs = &ks_pcie->num_legacy_host_irqs;
		controller = "legacy-interrupt-controller";
	} else {
		intc_np = &ks_pcie->msi_intc_np;
		host_irqs =  &ks_pcie->msi_host_irqs;
		num_irqs = &ks_pcie->num_msi_host_irqs;
		controller = "msi-interrupt-controller";
	}

	*intc_np = of_find_node_by_name(np, controller);
	if (!(*intc_np)) {
		dev_err(dev, "Node for %s is absent\n", controller);
		return -EINVAL;
	}

	irq_count = of_irq_count(*intc_np);
	if (!irq_count) {
		dev_err(dev, "No IRQ entries in %s\n", controller);
		return -EINVAL;
	}

	*host_irqs = devm_kzalloc(dev, sizeof(**host_irqs) * irq_count,
				  GFP_KERNEL);
	if (!(*host_irqs))
		return -ENOMEM;

	for (i = 0; i < irq_count; i++) {
		(*host_irqs)[i] = irq_of_parse_and_map(*intc_np, i);
		if (!(*host_irqs)[i])
			return -EINVAL;
	}

	*num_irqs = irq_count;

	return 0;
}

static void ks_pcie_setup_interrupts(struct keystone_pcie *ks_pcie)
{
	int i;

	/* Legacy IRQ */
	for (i = 0; i < ks_pcie->num_legacy_host_irqs; i++) {
		irq_set_chained_handler_and_data(ks_pcie->legacy_host_irqs[i],
						 ks_pcie_legacy_irq_handler,
						 ks_pcie);
	}
	ks_dw_pcie_enable_legacy_irqs(ks_pcie);

	/* MSI IRQ */
	for (i = 0; i < ks_pcie->num_msi_host_irqs; i++) {
		irq_set_chained_handler_and_data(ks_pcie->msi_host_irqs[i],
						 ks_pcie_msi_irq_handler,
						 ks_pcie);
	}
}

/*
 * When a PCI device does not exist during config cycles, keystone host gets a
 * bus error instead of returning 0xffffffff. This handler always returns 0
 * for this kind of faults.
 */
static int ks_pcie_fault(unsigned long addr, unsigned int fsr,
				struct pt_regs *regs)
{
	unsigned long instr = *(unsigned long *) instruction_pointer(regs);

	if ((instr & 0x0e100090) == 0x00100090) {
		int reg = (instr >> 12) & 15;

		regs->uregs[reg] = -1;
		regs->ARM_pc += 4;
	}

	return 0;
}

static int __init ks_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);

	dw_pcie_setup_rc(pp);
	ks_dw_pcie_setup_rc_app_regs(ks_pcie);
	ks_pcie_setup_interrupts(ks_pcie);
	writew(PCI_IO_RANGE_TYPE_32 | (PCI_IO_RANGE_TYPE_32 << 8),
			pci->dbi_base + PCI_IO_BASE);

        dw_pcie_writew_dbi(pci, PCI_VENDOR_ID,
			   ks_pcie->id & PCIE_VENDORID_MASK);
        dw_pcie_writew_dbi(pci, PCI_DEVICE_ID,
			   ks_pcie->id >> PCIE_DEVICEID_SHIFT);

	/*
	 * PCIe access errors that result into OCP errors are caught by ARM as
	 * "External aborts"
	 */
	hook_fault_code(17, ks_pcie_fault, SIGBUS, 0,
			"Asynchronous external abort");

	ks_dw_pcie_start_link(pci);
	if (!dw_pcie_wait_for_link(pci))
		return 0;

	return 0;
}

static const struct dw_pcie_host_ops ks_pcie_host_ops = {
	.rd_other_conf = ks_dw_pcie_rd_other_conf,
	.wr_other_conf = ks_dw_pcie_wr_other_conf,
	.host_init = ks_pcie_host_init,
	.msi_set_irq = ks_dw_pcie_msi_set_irq,
	.msi_clear_irq = ks_dw_pcie_msi_clear_irq,
	.get_msi_addr = ks_dw_pcie_get_msi_addr,
	.msi_host_init = ks_dw_pcie_msi_host_init,
	.msi_irq_ack = ks_dw_pcie_msi_irq_ack,
	.scan_bus = ks_dw_pcie_v3_65_scan_bus,
};

static irqreturn_t ks_pcie_err_handler(int irq, void *priv)
{
	struct keystone_pcie *ks_pcie = priv;

	return ks_dw_pcie_handle_error_irq(ks_pcie);
}

static int __init ks_add_pcie_port(struct keystone_pcie *ks_pcie,
			 struct platform_device *pdev)
{
	struct dw_pcie *pci = ks_pcie->pci;
	struct pcie_port *pp = &pci->pp;
	struct device *dev = &pdev->dev;
	int ret;

	/* Get legacy interrupt controller info */
	ret = ks_pcie_get_irq_controller_info(ks_pcie, true);
	if (ret)
		return ret;

	/* Get MSI interrupt controller info */
	ret = ks_pcie_get_irq_controller_info(ks_pcie, false);
	if (ret)
		return ret;

	pp->root_bus_nr = -1;
	pp->ops = &ks_pcie_host_ops;
	ret = ks_dw_pcie_host_init(ks_pcie);
	if (ret) {
		dev_err(dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static const struct dw_pcie_ops ks_pcie_dw_pcie_ops = {
	.start_link = ks_dw_pcie_start_link,
	.stop_link = ks_dw_pcie_stop_link,
	.link_up = ks_dw_pcie_link_up,
};

static void ks_pcie_disable_phy(struct keystone_pcie *ks_pcie)
{
	int num_lanes = ks_pcie->num_lanes;

	while (num_lanes--) {
		phy_power_off(ks_pcie->phy[num_lanes]);
		phy_exit(ks_pcie->phy[num_lanes]);
	}
}

static int ks_pcie_enable_phy(struct keystone_pcie *ks_pcie)
{
	int i;
	int ret;
	int num_lanes = ks_pcie->num_lanes;

	for (i = 0; i < num_lanes; i++) {
		ret = phy_init(ks_pcie->phy[i]);
		if (ret < 0)
			goto err_phy;

		ret = phy_power_on(ks_pcie->phy[i]);
		if (ret < 0) {
			phy_exit(ks_pcie->phy[i]);
			goto err_phy;
		}
	}

	return 0;

err_phy:
	while (--i >= 0) {
		phy_power_off(ks_pcie->phy[i]);
		phy_exit(ks_pcie->phy[i]);
	}

	return ret;
}

static int __init ks_pcie_probe(struct platform_device *pdev)
{
	int ret;
	int irq;
	unsigned id;
	char name[10];
	u32 index;
	u32 num_lanes;
	struct phy **phy;
	struct device_link **link;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct dw_pcie *pci;
	struct regmap *devctrl_regs;
	struct keystone_pcie *ks_pcie;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pci->dev = dev;
	pci->ops = &ks_pcie_dw_pcie_ops;

	ks_pcie = devm_kzalloc(dev, sizeof(*ks_pcie), GFP_KERNEL);
	if (!ks_pcie)
		return -ENOMEM;

        ret = of_property_read_u32(np, "num-lanes", &num_lanes);
        if (ret)
                num_lanes = 1;

	phy = devm_kzalloc(dev, sizeof(*phy) * num_lanes, GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	link = devm_kzalloc(dev, sizeof(*link) * num_lanes, GFP_KERNEL);
	if (!link)
		return -ENOMEM;

	for (index = 0; index < num_lanes; index++) {
		snprintf(name, sizeof(name), "pcie-phy%d", index);
		phy[index] = devm_phy_optional_get(dev, name);
		if (IS_ERR(phy[index]))
			return PTR_ERR(phy[index]);

		if (!phy[index])
			continue;

		link[index] = device_link_add(dev, &phy[index]->dev, DL_FLAG_STATELESS);
		if (!link[index]) {
			ret = -EINVAL;
			goto err_link;
		}
	}

	devctrl_regs = syscon_regmap_lookup_by_phandle(np, "ti,syscon-dev");
	if (IS_ERR(devctrl_regs)) {
		ret = PTR_ERR(devctrl_regs);
		goto err_link;
	}

	ret = of_property_read_u32_index(np, "ti,syscon-dev", 1, &index);
	if (ret) {
		dev_err(dev, "Failed to get pcie vendor/device id offset!\n");
		goto err_link;
	}

	ret = regmap_read(devctrl_regs, index, &id);
	if (ret) {
		dev_err(dev, "Failed to read pcie vendor id and device id!\n");
		goto err_link;
	}

	ks_pcie->np = np;
	ks_pcie->pci = pci;
	ks_pcie->phy = phy;
	ks_pcie->link = link;
	ks_pcie->id = id;
	ks_pcie->num_lanes = num_lanes;

	ret = ks_pcie_enable_phy(ks_pcie);
	if (ret) {
		dev_err(dev, "failed to enable phy\n");
		goto err_link;
	}

	ks_pcie->clk = devm_clk_get(dev, "pcie");
	if (IS_ERR(ks_pcie->clk)) {
		dev_err(dev, "Failed to get pcie rc clock\n");
		goto err_clk_get;
	}

	ret = clk_prepare_enable(ks_pcie->clk);
	if (ret)
		goto err_clk_get;

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_get_sync failed\n");
		goto err_get_sync;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "missing IRQ resource: %d\n", irq);
		ret = -EINVAL;
		goto err_get_sync;
	}
	
	ret = devm_request_irq(dev, irq, ks_pcie_err_handler, IRQF_SHARED,
			       "ks-pcie-error-irq", ks_pcie);
	if (ret < 0) {
		dev_err(dev, "failed to request error IRQ %d\n", irq);
		goto err_get_sync;
	}
	ks_dw_pcie_enable_error_irq(ks_pcie);

	platform_set_drvdata(pdev, ks_pcie);

	ret = ks_add_pcie_port(ks_pcie, pdev);
	if (ret < 0)
		goto err_get_sync;

	return 0;

err_get_sync:
	pm_runtime_put(dev);
	pm_runtime_disable(dev);
	clk_disable_unprepare(ks_pcie->clk);

err_clk_get:
	ks_pcie_disable_phy(ks_pcie);

err_link:
	while (--index >= 0 && link[index])
		device_link_del(link[index]);

	return ret;
}

void ks_pcie_shutdown(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct keystone_pcie *ks_pcie = dev_get_drvdata(dev);
	int ret;

	ks_dw_pcie_stop_link(ks_pcie->pci);

	ret = pm_runtime_put_sync(dev);
	if (ret < 0)
		dev_dbg(dev, "pm_runtime_put_sync failed\n");

	pm_runtime_disable(dev);
	ks_pcie_disable_phy(ks_pcie);
}

static const struct of_device_id ks_pcie_of_match[] = {
	{
		.type = "pci",
		.compatible = "ti,keystone-pcie",
	},
	{},
};

static struct platform_driver ks_pcie_driver __refdata = {
	.probe  = ks_pcie_probe,
	.driver = {
		.name = "keystone-pcie",
		.of_match_table = of_match_ptr(ks_pcie_of_match),
	},
};
builtin_platform_driver(ks_pcie_driver);
