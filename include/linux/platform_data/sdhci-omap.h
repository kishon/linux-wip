/**
 * SDHCI Controller Platform Data for TI's OMAP SoCs
 *
 * Copyright (C) 2017 Texas Instruments
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __SDHCI_OMAP_PDATA_H__
#define __SDHCI_OMAP_PDATA_H__

struct sdhci_omap_platform_data {
	const char *name;

	/*
	 * set if your board has components or wiring that limits the
	 * maximum frequency on the MMC bus
	 */
	unsigned int max_freq;

	/* string specifying a particular variant of hardware */
	char *version;
};

#endif
