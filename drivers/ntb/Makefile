# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_NTB) += ntb.o hw/ test/
obj-$(CONFIG_NTB_TRANSPORT) += ntb_transport.o
obj-$(CONFIG_NTB_VIRTIO) += ntb_virtio.o

ntb-y			:= core.o
ntb-$(CONFIG_NTB_MSI)	+= msi.o
