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

#define CMD_STATUS		0x004
#define LTSSM_EN_VAL		BIT(0)
#define OB_XLAT_EN_VAL		BIT(1)
#define DBI_CS2			BIT(5)

#define CFG_SETUP		0x008
#define CFG_BUS(x)		(((x) & 0xff) << 16)
#define CFG_DEVICE(x)		(((x) & 0x1f) << 8)
#define CFG_FUNC(x)		((x) & 0x7)
#define CFG_TYPE1		BIT(24)

#define OB_SIZE			0x030
#define OB_WIN_SIZE		8	/* 8MB */

#define IRQ_EOI			0x050

#define IRQ_STATUS(n)		(0x184 + ((n) << 4))
#define IRQ_ENABLE_SET(n)	(0x188 + ((n) << 4))
#define INTx_EN			BIT(0)

#define ERR_IRQ_STATUS		0x1c4
#define ERR_IRQ_ENABLE_SET	0x1c8
#define ERR_IRQ_ENABLE_CLR	0x1cc
#define ERR_AER			BIT(5)	/* ECRC error */
#define ERR_AXI			BIT(4)	/* AXI tag lookup fatal error */
#define ERR_CORR		BIT(3)	/* Correctable error */
#define ERR_NONFATAL		BIT(2)	/* Non-fatal error */
#define ERR_FATAL		BIT(1)	/* Fatal error */
#define ERR_SYS			BIT(0)	/* System (fatal, non-fatal, or correctable) */
#define ERR_IRQ_ALL		(ERR_AER | ERR_AXI | ERR_CORR | \
				 ERR_NONFATAL | ERR_FATAL | ERR_SYS)
#define ERR_FATAL_IRQ		(ERR_FATAL | ERR_AXI)

#define OB_OFFSET_INDEX(n)	(0x200 + (8 * (n)))
#define OB_ENABLEN		BIT(0)

#define OB_OFFSET_HI(n)		(0x204 + (8 * (n)))

#define to_keystone_pcie(x)	dev_get_drvdata((x)->dev)

static int ks_pcie_start_link(struct dw_pcie *pci);
static void ks_pcie_stop_link(struct dw_pcie *pci);

static inline u32 ks_pcie_app_readl(struct keystone_pcie *ks_pcie, u32 offset)
{
	return readl(ks_pcie->va_app_base + offset);
}

static inline void ks_pcie_app_writel(struct keystone_pcie *ks_pcie, u32 offset,
				      u32 value)
{
	writel(value, ks_pcie->va_app_base + offset);
}

static void ks_pcie_set_dbi_mode(struct keystone_pcie *ks_pcie)
{
	u32 val;

	val = ks_pcie_app_readl(ks_pcie, CMD_STATUS);
	val |= DBI_CS2;
	ks_pcie_app_writel(ks_pcie, CMD_STATUS, val);

	do {
		val = ks_pcie_app_readl(ks_pcie, CMD_STATUS);
	} while (!(val & DBI_CS2));
}

static void ks_pcie_clear_dbi_mode(struct keystone_pcie *ks_pcie)
{
	u32 val;

	val = ks_pcie_app_readl(ks_pcie, CMD_STATUS);
	val &= ~DBI_CS2;
	ks_pcie_app_writel(ks_pcie, CMD_STATUS, val);

	do {
		val = ks_pcie_app_readl(ks_pcie, CMD_STATUS);
	} while (val & DBI_CS2);
}

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

static int ks_pcie_rd_other_conf(struct pcie_port *pp, struct pci_bus *bus,
				 unsigned int devfn, int where, int size,
				 u32 *val)
{
	u32 reg;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);

	reg = CFG_BUS(bus->number) | CFG_DEVICE(PCI_SLOT(devfn)) |
		CFG_FUNC(PCI_FUNC(devfn));
	if (bus->parent->number != pp->root_bus_nr)
		reg |= CFG_TYPE1;
	ks_pcie_app_writel(ks_pcie, CFG_SETUP, reg);

	return dw_pcie_read(pp->va_cfg0_base + where, size, val);
}

static int ks_pcie_wr_other_conf(struct pcie_port *pp, struct pci_bus *bus,
				 unsigned int devfn, int where, int size,
				 u32 val)
{
	u32 reg;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);

	reg = CFG_BUS(bus->number) | CFG_DEVICE(PCI_SLOT(devfn)) |
		CFG_FUNC(PCI_FUNC(devfn));
	if (bus->parent->number != pp->root_bus_nr)
		reg |= CFG_TYPE1;
	ks_pcie_app_writel(ks_pcie, CFG_SETUP, reg);

	return dw_pcie_write(pp->va_cfg0_base + where, size, val);
}

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
	u32 reg;
	int virq;
	unsigned int irq = irq_desc_get_irq(desc);
	struct keystone_pcie *ks_pcie = irq_desc_get_handler_data(desc);
	struct dw_pcie *pci = ks_pcie->pci;
	struct device *dev = pci->dev;
	u32 offset = irq - ks_pcie->legacy_host_irqs[0];
	struct irq_chip *chip = irq_desc_get_chip(desc);

	dev_dbg(dev, ": Handling legacy irq %d\n", irq);

	chained_irq_enter(chip, desc);

	reg = ks_pcie_app_readl(ks_pcie, IRQ_STATUS(offset));
	if (!(reg & INTx_EN))
		goto ret;

	virq = irq_linear_revmap(ks_pcie->legacy_irq_domain, offset);
	dev_dbg(dev, ": irq: irq_offset %d, virq %d\n", offset, virq);
	generic_handle_irq(virq);

	ks_pcie_app_writel(ks_pcie, IRQ_EOI, offset);

ret:
	chained_irq_exit(chip, desc);
}

static void ks_dw_pcie_v3_65_scan_bus(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);

	ks_pcie_set_dbi_mode(ks_pcie);
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_0, 1);
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_0, SZ_4K - 1);
	ks_pcie_clear_dbi_mode(ks_pcie);

	 /*
	  * For BAR0, just setting bus address for inbound writes (MSI) should
	  * be sufficient.  Use physical address to avoid any conflicts.
	  */
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_0, ks_pcie->app.start);
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

static void ks_pcie_enable_legacy_irqs(struct keystone_pcie *ks_pcie)
{
	int i;

	for (i = 0; i < PCI_NUM_INTX; i++)
		ks_pcie_app_writel(ks_pcie, IRQ_ENABLE_SET(i), INTx_EN);
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
	ks_pcie_enable_legacy_irqs(ks_pcie);

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

static void ks_pcie_setup_mem_space(struct keystone_pcie *ks_pcie)
{
	u32 val;
	u32 num_ob_windows = ks_pcie->num_ob_windows;
	struct dw_pcie *pci = ks_pcie->pci;
	struct pcie_port *pp = &pci->pp;
	u64 start = pp->mem->start;
	u64 end = pp->mem->end;
	int i;

	val = ilog2(OB_WIN_SIZE);
	ks_pcie_app_writel(ks_pcie, OB_SIZE, val);

	/* Using Direct 1:1 mapping of RC <-> PCI memory space */
	for (i = 0; i < num_ob_windows && (start < end); i++) {
		ks_pcie_app_writel(ks_pcie, OB_OFFSET_INDEX(i),
				   lower_32_bits(start) | OB_ENABLEN);
		ks_pcie_app_writel(ks_pcie, OB_OFFSET_HI(i),
				   upper_32_bits(start));
		start += OB_WIN_SIZE;
	}

	val = ks_pcie_app_readl(ks_pcie, CMD_STATUS);
	val |= OB_XLAT_EN_VAL;
	ks_pcie_app_writel(ks_pcie, CMD_STATUS, val);
}

static int ks_pcie_intx_map(struct irq_domain *domain, unsigned int irq,
			    irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &dummy_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops ks_pcie_intx_domain_ops = {
	.map = ks_pcie_intx_map,
};

static int __init ks_pcie_host_init(struct pcie_port *pp)
{
	int ret;
	struct irq_domain *legacy_irq_domain;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct device *dev = pci->dev;
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);

	/* Get legacy interrupt controller info */
	ret = ks_pcie_get_irq_controller_info(ks_pcie, true);
	if (ret)
		return ret;

	/* Get MSI interrupt controller info */
	ret = ks_pcie_get_irq_controller_info(ks_pcie, false);
	if (ret)
		return ret;

	legacy_irq_domain = irq_domain_add_linear(ks_pcie->legacy_intc_np,
						  PCI_NUM_INTX,
						  &ks_pcie_intx_domain_ops,
						  NULL);
	if (!legacy_irq_domain) {
		dev_err(dev, "Failed to add irq domain for legacy irqs\n");
		return -EINVAL;
	}
	ks_pcie->legacy_irq_domain = legacy_irq_domain;

	dw_pcie_setup_rc(pp);

        ks_pcie_set_dbi_mode(ks_pcie);
        dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_0, 0);
        dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_1, 0);
        ks_pcie_clear_dbi_mode(ks_pcie);

	ks_pcie_setup_mem_space(ks_pcie);
	ks_pcie_setup_interrupts(ks_pcie);

	/*
	 * PCIe access errors that result into OCP errors are caught by ARM as
	 * "External aborts"
	 */
	hook_fault_code(17, ks_pcie_fault, SIGBUS, 0,
			"Asynchronous external abort");

	ks_pcie_start_link(pci);
	dw_pcie_wait_for_link(pci);

	return 0;
}

static const struct dw_pcie_host_ops ks_pcie_host_ops = {
	.rd_other_conf = ks_pcie_rd_other_conf,
	.wr_other_conf = ks_pcie_wr_other_conf,
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
	u32 reg;
	struct keystone_pcie *ks_pcie = priv;
	struct device *dev = ks_pcie->pci->dev;

	reg = ks_pcie_app_readl(ks_pcie, ERR_IRQ_STATUS);

	if (reg & ERR_SYS)
		dev_err(dev, "System Error\n");

	if (reg & ERR_FATAL)
		dev_dbg(dev, "Fatal Error\n");

	if (reg & ERR_NONFATAL)
		dev_dbg(dev, "Non Fatal Error\n");

	if (reg & ERR_CORR)
		dev_dbg(dev, "Correctable Error\n");

	if (reg & ERR_AXI)
		dev_dbg(dev, "AXI tag lookup fatal Error\n");

	if (reg & ERR_AER)
		dev_dbg(dev, "ECRC Error\n");

	ks_pcie_app_writel(ks_pcie, ERR_IRQ_STATUS, reg);

	return IRQ_HANDLED;
}

static void ks_pcie_enable_error_irq(struct keystone_pcie *ks_pcie)
{
	ks_pcie_app_writel(ks_pcie, ERR_IRQ_ENABLE_SET, ERR_IRQ_ALL);
}

static int __init ks_add_pcie_port(struct keystone_pcie *ks_pcie,
			 struct platform_device *pdev)
{
	int ret;
	struct dw_pcie *pci = ks_pcie->pci;
	struct pcie_port *pp = &pci->pp;
	struct device *dev = &pdev->dev;
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "config");
	pp->va_cfg0_base = devm_pci_remap_cfg_resource(dev, res);
	if (IS_ERR(pp->va_cfg0_base))
		return PTR_ERR(pp->va_cfg0_base);

	pp->va_cfg1_base = pp->va_cfg0_base;
	pp->root_bus_nr = -1;
	pp->ops = &ks_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static int ks_pcie_init_id(struct keystone_pcie *ks_pcie)
{
	int ret;
	u32 index;
	unsigned id;
	struct regmap *devctrl_regs;
	struct dw_pcie *pci = ks_pcie->pci;
	struct device *dev = pci->dev;
	struct device_node *np = dev->of_node;

	devctrl_regs = syscon_regmap_lookup_by_phandle(np, "ti,syscon-dev");
	if (IS_ERR(devctrl_regs))
		return PTR_ERR(devctrl_regs);

	ret = of_property_read_u32_index(np, "ti,syscon-dev", 1, &index);
	if (ret)
		return ret;

	ret = regmap_read(devctrl_regs, index, &id);
	if (ret)
		return ret;

        dw_pcie_writew_dbi(pci, PCI_VENDOR_ID, id & PCIE_VENDORID_MASK);
        dw_pcie_writew_dbi(pci, PCI_DEVICE_ID, id >> PCIE_DEVICEID_SHIFT);

	return 0;
}

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

static int ks_pcie_link_up(struct dw_pcie *pci)
{
	u32 val;

	val = dw_pcie_readl_dbi(pci, PCIE_PORT_DEBUG0);
	val &= PORT_LOGIC_LTSSM_STATE_MASK;
	return (val == PORT_LOGIC_LTSSM_STATE_L0);
}

static int ks_pcie_start_link(struct dw_pcie *pci)
{
	u32 val;
	struct device *dev = pci->dev;
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);

        if (dw_pcie_link_up(pci)) {
                dev_err(dev, "Link already up\n");
                return 0;
        }

	val = ks_pcie_app_readl(ks_pcie, CMD_STATUS);
	val |= LTSSM_EN_VAL;
	ks_pcie_app_writel(ks_pcie, CMD_STATUS, val);

	return 0;
}

static void ks_pcie_stop_link(struct dw_pcie *pci)
{
	u32 val;
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);

	val = ks_pcie_app_readl(ks_pcie, CMD_STATUS);
	val &= ~LTSSM_EN_VAL;
	ks_pcie_app_writel(ks_pcie, CMD_STATUS, val);
}

static const struct dw_pcie_ops ks_pcie_dw_pcie_ops = {
	.start_link = ks_pcie_start_link,
	.stop_link = ks_pcie_stop_link,
	.link_up = ks_pcie_link_up,
};

static int __init ks_pcie_probe(struct platform_device *pdev)
{
	int i;
	int ret;
	int irq;
	char name[10];
	u32 num_lanes;
	u32 num_ob_windows;
	void __iomem *base;
	struct resource *res;
	struct phy **phy;
	struct device_link **link;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct dw_pcie *pci;
	struct keystone_pcie *ks_pcie;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbics");
	base = devm_pci_remap_cfg_resource(dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	pci->dev = dev;
	pci->dbi_base = base;
	pci->ops = &ks_pcie_dw_pcie_ops;

	ks_pcie = devm_kzalloc(dev, sizeof(*ks_pcie), GFP_KERNEL);
	if (!ks_pcie)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "app");
	base = devm_ioremap_nocache(dev, res->start, resource_size(res));
	if (!base)
		return -ENOMEM;

        ret = of_property_read_u32(np, "num-lanes", &num_lanes);
        if (ret)
                num_lanes = 1;

	ret = of_property_read_u32(np, "num-ob-windows", &num_ob_windows);
	if (ret < 0) {
		dev_err(dev, "unable to read *num-ob-windows* property\n");
		return ret;
	}

	phy = devm_kzalloc(dev, sizeof(*phy) * num_lanes, GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	link = devm_kzalloc(dev, sizeof(*link) * num_lanes, GFP_KERNEL);
	if (!link)
		return -ENOMEM;

	for (i = 0; i < num_lanes; i++) {
		snprintf(name, sizeof(name), "pcie-phy%d", i);
		phy[i] = devm_phy_optional_get(dev, name);
		if (IS_ERR(phy[i]))
			return PTR_ERR(phy[i]);

		if (!phy[i])
			continue;

		link[i] = device_link_add(dev, &phy[i]->dev, DL_FLAG_STATELESS);
		if (!link[i]) {
			ret = -EINVAL;
			goto err_link;
		}
	}

	ks_pcie->np = np;
	ks_pcie->pci = pci;
	ks_pcie->phy = phy;
	ks_pcie->link = link;
	ks_pcie->num_lanes = num_lanes;
	ks_pcie->num_ob_windows = num_ob_windows;
	ks_pcie->va_app_base = base;
	ks_pcie->app = *res;

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
	platform_set_drvdata(pdev, ks_pcie);

	ret = ks_pcie_init_id(ks_pcie);
	if (ret < 0)
		goto err_get_sync;

	ret = ks_add_pcie_port(ks_pcie, pdev);
	if (ret < 0)
		goto err_get_sync;

	ks_pcie_enable_error_irq(ks_pcie);

	return 0;

err_get_sync:
	pm_runtime_put(dev);
	pm_runtime_disable(dev);
	clk_disable_unprepare(ks_pcie->clk);

err_clk_get:
	ks_pcie_disable_phy(ks_pcie);

err_link:
	while (--i >= 0 && link[i])
		device_link_del(link[i]);

	return ret;
}

void ks_pcie_shutdown(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct keystone_pcie *ks_pcie = dev_get_drvdata(dev);
	int ret;

	ks_pcie_stop_link(ks_pcie->pci);

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
