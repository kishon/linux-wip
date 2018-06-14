// SPDX-License-Identifier: GPL-2.0
/**
 * USB 3.0, PCIe and SGMII SERDES driver for K3 Platform
 *
 * Copyright (C) 2018 Texas Instruments
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
 */

#include <dt-bindings/phy/phy.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mux/consumer.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#define CMU_R07C		0x7c
#define CMU_MASTER_CDN_O	BIT(24)

#define COMLANE_R138		0xb38
#define CONFIG_VERSION_REG	GENMASK(23, 16)
#define CONFIG_VERSION_REG_SHIFT 16
#define VERSION			0x70

#define COMLANE_R190		0xb90
#define L1_MASTER_CDN_O		BIT(9)

#define COMLANE_R194		0xb94
#define CMU_OK_I_0		BIT(19)

#define SERDES_CTRL		0x1fd0
#define POR_EN			BIT(29)

#define WIZ_LANEXCTL_STS	0x1fe0
#define TX0_ENABLE_OVL		BIT(31)
#define TX0_ENABLE_VAL 		GENMASK(30, 29)
#define TX0_ENABLE_SHIFT	29
#define TX0_DISABLE_STATE	0x0
#define TX0_SLEEP_STATE		0x1
#define TX0_SNOOZE_STATE	0x2
#define TX0_ENABLE_STATE	0x3
#define RX0_ENABLE_OVL		BIT(15)
#define RX0_ENABLE_VAL 		GENMASK(14, 13)
#define RX0_ENABLE_SHIFT	13
#define RX0_DISABLE_STATE	0x0
#define RX0_SLEEP_STATE		0x1
#define RX0_SNOOZE_STATE	0x2
#define RX0_ENABLE_STATE	0x3

#define WIZ_PLL_CTRL		0x1ff4
#define PLL_ENABLE_OVL		BIT(31)
#define PLL_ENABLE_VAL 		GENMASK(30, 29)
#define PLL_ENABLE_SHIFT	29
#define PLL_DISABLE_STATE	0x0
#define PLL_SLEEP_STATE		0x1
#define PLL_SNOOZE_STATE	0x2
#define PLL_ENABLE_STATE	0x3
#define PLL_OK			BIT(28)

#define PLL_LOCK_TIME	100	/* in milliseconds */

#define LANE_USB3		0x0
#define LANE_PCIE0_LANE0	0x1

#define LANE_PCIE1_LANE0	0x0
#define LANE_PCIE0_LANE1	0x1

struct phy_k3 {
	void __iomem		*base;
	struct device		*dev;
	u8			id;
	struct mux_control	*control;
};

struct phy_k3_of_data {
	u8	id;
};

static inline u32 phy_k3_readl(void __iomem *addr, unsigned offset)
{
	return readl(addr + offset);
}

static inline void phy_k3_writel(void __iomem *addr, unsigned offset,
				       u32 data)
{
	writel(data, addr + offset);
}

static int phy_k3_enable_pll(struct phy_k3 *phy)
{
	struct device *dev = phy->dev;
	unsigned long timeout;
	u32 val;

	val = phy_k3_readl(phy->base, WIZ_PLL_CTRL);
	val &= ~(PLL_ENABLE_OVL | PLL_ENABLE_VAL);
	val |= PLL_ENABLE_OVL | (PLL_ENABLE_STATE << PLL_ENABLE_SHIFT);
	phy_k3_writel(phy->base, WIZ_PLL_CTRL, val);

	timeout = jiffies + msecs_to_jiffies(PLL_LOCK_TIME);
	do {
		cpu_relax();
		val = phy_k3_readl(phy->base, WIZ_PLL_CTRL);
		if (val & PLL_OK)
			return 0;
	} while (!time_after(jiffies, timeout));

	dev_err(dev, "PLL enable failed\n");
	return -EBUSY;
}

static void phy_k3_disable_pll(struct phy_k3 *phy)
{
	u32 val;

	val = phy_k3_readl(phy->base, WIZ_PLL_CTRL);
	val &= ~(PLL_ENABLE_OVL | PLL_ENABLE_VAL);
	phy_k3_writel(phy->base, WIZ_PLL_CTRL, val);
}

static int phy_k3_enable_txrx(struct phy_k3 *phy)
{
	u32 val;

	/* Enable TX */
	val = phy_k3_readl(phy->base, WIZ_LANEXCTL_STS);
	val &= ~(TX0_ENABLE_OVL | TX0_ENABLE_VAL);
	val |= TX0_ENABLE_OVL | (TX0_ENABLE_STATE << TX0_ENABLE_SHIFT);
	phy_k3_writel(phy->base, WIZ_LANEXCTL_STS, val);

	/* Enable RX */
	val = phy_k3_readl(phy->base, WIZ_LANEXCTL_STS);
	val &= ~(RX0_ENABLE_OVL | RX0_ENABLE_VAL);
	val |= RX0_ENABLE_OVL | (RX0_ENABLE_STATE << RX0_ENABLE_SHIFT);
	phy_k3_writel(phy->base, WIZ_LANEXCTL_STS, val);

	return 0;
}

static int phy_k3_disable_txrx(struct phy_k3 *phy)
{
	u32 val;

	/* Disable TX */
	val = phy_k3_readl(phy->base, WIZ_LANEXCTL_STS);
	val &= ~(TX0_ENABLE_OVL | TX0_ENABLE_VAL);
	phy_k3_writel(phy->base, WIZ_LANEXCTL_STS, val);

	/* Disable RX */
	val = phy_k3_readl(phy->base, WIZ_LANEXCTL_STS);
	val &= ~(RX0_ENABLE_OVL | RX0_ENABLE_VAL);
	phy_k3_writel(phy->base, WIZ_LANEXCTL_STS, val);

	return 0;
}

static int phy_k3_power_on(struct phy *x)
{
	struct phy_k3 *phy = phy_get_drvdata(x);
	struct device *dev = phy->dev;
	unsigned long timeout;
	int ret;
	u32 val;

	ret = phy_k3_enable_pll(phy);
	if (ret) {
		dev_err(dev, "Failed to enable PLL\n");
		return ret;
	}

	ret = phy_k3_enable_txrx(phy);
	if (ret) {
		dev_err(dev, "Failed to enable TX RX\n");
		return ret;
	}

	timeout = jiffies + msecs_to_jiffies(PLL_LOCK_TIME);
	do {
		cpu_relax();
		val = phy_k3_readl(phy->base, COMLANE_R194);
		if (val & CMU_OK_I_0)
			return 0;
	} while (!time_after(jiffies, timeout));

	dev_err(dev, "CMU Not Enabled\n");
	return -EBUSY;
}

static int phy_k3_power_off(struct phy *x)
{
	struct phy_k3 *phy = phy_get_drvdata(x);

	phy_k3_disable_txrx(phy);
	phy_k3_disable_pll(phy);

	return 0;
}

static int phy_k3_init(struct phy *x)
{
	struct phy_k3 *phy = phy_get_drvdata(x);
	u32 val;

	val = phy_k3_readl(phy->base, COMLANE_R138);
	val &= ~CONFIG_VERSION_REG;
	val |= VERSION << CONFIG_VERSION_REG_SHIFT;
	phy_k3_writel(phy->base, COMLANE_R138, val);

	val = phy_k3_readl(phy->base, CMU_R07C);
	val |= CMU_MASTER_CDN_O;
	phy_k3_writel(phy->base, CMU_R07C, val);

	val = phy_k3_readl(phy->base, COMLANE_R190);
	val |= L1_MASTER_CDN_O;
	phy_k3_writel(phy->base, COMLANE_R190, val);

	return 0;
}

static int phy_k3_reset(struct phy *x)
{
	struct phy_k3 *phy = phy_get_drvdata(x);
	u32 val;

	val = phy_k3_readl(phy->base, SERDES_CTRL);
	val |= POR_EN;
	phy_k3_writel(phy->base, SERDES_CTRL, val);
	mdelay(1);
	val &= ~POR_EN;
	phy_k3_writel(phy->base, SERDES_CTRL, val);

	return 0;
}

static int phy_k3_set_mode(struct phy_k3 *phy, int mode, int lane)
{
	struct mux_control *control = phy->control;
	unsigned int lane_func_sel = 9;

	if (mode == PHY_TYPE_PCIE) {
		if (phy->id == 0) {
			lane_func_sel = LANE_PCIE0_LANE0;
		} else if (phy->id == 1) {
			if (lane == 0)
				lane_func_sel = LANE_PCIE1_LANE0;
			else
				lane_func_sel = LANE_PCIE0_LANE1;
		}
	}

	return mux_control_select(control, lane_func_sel);
}

struct phy *phy_k3_simple_xlate(struct device *dev, struct of_phandle_args
				*args) {
	struct phy_k3 *k3_phy;
	struct phy *phy;
	int ret;

	phy = of_phy_simple_xlate(dev, args);
	if (IS_ERR(phy))
		return phy;

	k3_phy = phy_get_drvdata(phy);
	ret = phy_k3_set_mode(k3_phy, args->args[0], args->args[1]);
	if (ret < 0)
		return ERR_PTR(-ENODEV);

	return phy;
}

static const struct phy_ops ops = {
	.reset		= phy_k3_reset,
	.init		= phy_k3_init,
	.power_on	= phy_k3_power_on,
	.power_off	= phy_k3_power_off,
	.owner		= THIS_MODULE,
};

static const struct phy_k3_of_data phy_k3_of_data_serdes0 = {
	.id = 0,
};

static const struct phy_k3_of_data phy_k3_of_data_serdes1 = {
	.id = 1,
};

static const struct of_device_id phy_k3_id_table[] = {
	{
		.compatible = "ti,phy-k3-serdes0",
		.data = &phy_k3_of_data_serdes0,
	},
	{
		.compatible = "ti,phy-k3-serdes1",
		.data = &phy_k3_of_data_serdes1,
	},
	{}
};
MODULE_DEVICE_TABLE(of, phy_k3_id_table);

static int phy_k3_probe(struct platform_device *pdev)
{
	struct phy_provider *phy_provider;
	struct phy_k3 *k3_phy;
	struct device *dev = &pdev->dev;
        const struct phy_k3_of_data *data;
        const struct of_device_id *match;
	struct resource *res;
	struct phy *phy;
	u32 id = 0;

	match = of_match_device(of_match_ptr(phy_k3_id_table), dev);
	data = (struct phy_k3_of_data *)match->data;
	if (data)
		id = (u8)data->id;

	k3_phy = devm_kzalloc(dev, sizeof(*k3_phy), GFP_KERNEL);
	if (!k3_phy)
		return -ENOMEM;

	k3_phy->dev = dev;
	k3_phy->id = id;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "serdes");
	k3_phy->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(k3_phy->base))
		return PTR_ERR(k3_phy->base);

	k3_phy->control = devm_mux_control_get(dev, NULL);
	if (IS_ERR(k3_phy->control))
		return PTR_ERR(k3_phy->control);

	pm_runtime_enable(dev);

	phy = devm_phy_create(dev, NULL, &ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);

	phy_set_drvdata(phy, k3_phy);
	phy_provider = devm_of_phy_provider_register(dev, phy_k3_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static int phy_k3_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static struct platform_driver phy_k3_driver = {
	.probe		= phy_k3_probe,
	.remove		= phy_k3_remove,
	.driver		= {
		.name	= "phy-k3",
		.of_match_table = phy_k3_id_table,
	},
};
module_platform_driver(phy_k3_driver);

MODULE_ALIAS("platform:phy-k3");
MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TI K3 SERDES driver");
MODULE_LICENSE("GPL v2");
