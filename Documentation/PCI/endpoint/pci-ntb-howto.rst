.. SPDX-License-Identifier: GPL-2.0

======================
PCI NTB EPF User Guide
======================

:Author: Kishon Vijay Abraham I <kishon@ti.com>

This document is a guide to help users use pci-epf-ntb function driver
and ntb_hw_epf host driver for NTB functionality. The list of steps to
be followed in the host side and EP side is given below. For the hardware
configuration and internals of NTB using configurable endpoints see
Documentation/PCI/endpoint/pci-ntb-function.rst

Endpoint Device
===============

Endpoint Controller Devices
---------------------------

For implementing NTB functionality atleast two endpoint controller devices
are required.
To find the list of endpoint controller devices in the system::

        # ls /sys/class/pci_epc/
          2900000.pcie-ep  2910000.pcie-ep

If PCI_ENDPOINT_CONFIGFS is enabled::

	# ls /sys/kernel/config/pci_ep/controllers
	  2900000.pcie-ep  2910000.pcie-ep


Endpoint Function Drivers
-------------------------

To find the list of endpoint function drivers in the system::

	# ls /sys/bus/pci-epf/drivers
	  pci_epf_ntb   pci_epf_ntb

If PCI_ENDPOINT_CONFIGFS is enabled::

	# ls /sys/kernel/config/pci_ep/functions
	  pci_epf_ntb   pci_epf_ntb


Creating pci-epf-ntb Device
----------------------------

PCI endpoint function device can be created using the configfs. To create
pci-epf-ntb device, the following commands can be used::

	# mount -t configfs none /sys/kernel/config
	# cd /sys/kernel/config/pci_ep/
	# mkdir functions/pci_epf_ntb/func1

The "mkdir func1" above creates the pci-epf-ntb function device that will
be probed by pci_epf_ntb driver.

The PCI endpoint framework populates the directory with the following
configurable fields::

	# ls functions/pci_epf_ntb/func1
          baseclass_code    deviceid          msi_interrupts    pci-epf-ntb.0
          progif_code       secondary         subsys_id         vendorid
          cache_line_size   interrupt_pin     msix_interrupts   primary
          revid             subclass_code     subsys_vendor_id

The PCI endpoint function driver populates these entries with default values
when the device is bound to the driver. The pci-epf-ntb driver populates
vendorid with 0xffff and interrupt_pin with 0x0001::

	# cat functions/pci_epf_ntb/func1/vendorid
	  0xffff
	# cat functions/pci_epf_ntb/func1/interrupt_pin
	  0x0001


Configuring pci-epf-ntb Device
-------------------------------

The user can configure the pci-epf-ntb device using configfs entry. In order
to change the vendorid and the number of MSI interrupts device, the following
commands can be used::

	# echo 0x104c > functions/pci_epf_ntb/func1/vendorid
	# echo 0xb00d > functions/pci_epf_ntb/func1/deviceid


Binding pci-epf-ntb Device to EP Controller
--------------------------------------------

NTB function device should be attached to two PCIe endpoint controllers
connected to the two hosts. Use the 'primary' and 'secondary' entries
inside NTB function device to attach one PCIe endpoint controller to
primary interface and the other PCIe endpoint controller to the secondary
interface. ::

        # ln -s controllers/2900000.pcie-ep/ functions/pci-epf-ntb/func1/primary
        # ln -s controllers/2910000.pcie-ep/ functions/pci-epf-ntb/func1/secondary

Once the above step is completed, both the PCI endpoint controllers is ready to
establish a link with the host.


Start the Link
--------------

In order for the endpoint device to establish a link with the host, the _start_
field should be populated with '1'. For NTB, both the PCIe endpoint controllers
should establish link with the host::

        #echo 1 > controllers/2900000.pcie-ep/start
        #echo 1 > controllers/2910000.pcie-ep/start


RootComplex Device
==================

lspci Output
------------

Note that the devices listed here correspond to the value populated in 1.4
above::

        # lspci
        0000:00:00.0 PCI bridge: Texas Instruments Device b00d
        0000:01:00.0 RAM memory: Texas Instruments Device b00d


Using ntb_hw_epf Device
-----------------------

The host side software follows the standard NTB software architecture in Linux.
All the existing client side NTB utilities like NTB Transport Client and NTB
Netdev, NTB Ping Pong Test Client and NTB Tool Test Clientcan be used with NTB
function device.

For more information on NTB see
Documentation/driver-api/ntb.rst
