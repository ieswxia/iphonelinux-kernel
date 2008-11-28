/*
 *  arch/arm/mach-apple_iphone/usb.c
 *
 *  Copyright (C) 2008 Yiduo Wang
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>

#include <asm/byteorder.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include <asm/cacheflush.h>

#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/platform_device.h>

#include "clock.h"
#include "usb.h"
#include <mach/usb.h>

#define GET_BITS(x, start, length) ((((u32)(x)) << (32 - ((start) + (length)))) >> (32 - (length)))

#define DRIVER_VERSION          __DATE__

struct iphone_usb {
        spinlock_t lock;
};

typedef struct RingBuffer {
	s8* writePtr;
	s8* readPtr;
	u32 count;
	u32 size;
	s8* bufferStart;
	s8* bufferEnd;
	spinlock_t lock;
} RingBuffer;

struct iphone_usb iphone_usb_state;

static const char driver_name[] = "iphoneusb";

static USBEPRegisters* InEPRegs;
static USBEPRegisters* OutEPRegs;

static USBState usb_state;
static USBDirection endpoint_directions[USB_NUM_ENDPOINTS];
static USBEndpointBidirHandlerInfo endpoint_handlers[USB_NUM_ENDPOINTS];

static u32 inInterruptStatus[USB_NUM_ENDPOINTS];
static u32 outInterruptStatus[USB_NUM_ENDPOINTS];

static int usb_speed;
static int usb_max_packet_size;

static u8 currentlySending;

static USBDeviceDescriptor deviceDescriptor;
static USBDeviceQualifierDescriptor deviceQualifierDescriptor;

static int numStringDescriptors;
static USBStringDescriptor** stringDescriptors;
static USBFirstStringDescriptor* firstStringDescriptor;

static USBConfiguration* configurations;

static u8* ep0controlSendBuffer = NULL;
static u8* ep0controlRecvBuffer = NULL;

static RingBuffer* txQueue = NULL;

static USBEnumerateHandler enumerateHandler;
static USBStartHandler startHandler;

static void change_state(USBState new_state)
{
	printk("iphoneusb: USB state change: %d -> %d\r\n", usb_state, new_state);
	usb_state = new_state;
	if(usb_state == USBConfigured) {
		// TODO: set to host powered
	}
}

static RingBuffer* createRingBuffer(int size) {
	RingBuffer* buffer;
	buffer = (RingBuffer*) kmalloc(sizeof(RingBuffer), GFP_KERNEL);
	buffer->bufferStart = (s8*) kmalloc(size, GFP_KERNEL | GFP_DMA);
	buffer->bufferEnd = buffer->bufferStart + size;
	buffer->size = size;
	buffer->count = 0;
	buffer->readPtr = buffer->bufferStart;
	buffer->writePtr = buffer->bufferStart;
        spin_lock_init(&buffer->lock);
	return buffer;
}

static int ringBufferDequeue(RingBuffer* buffer) {
	s8 value;
	unsigned long flags;
        spin_lock_irqsave(&buffer->lock, flags);
	if(buffer->count == 0) {
        	spin_unlock_irqrestore(&buffer->lock, flags);
		return -1;
	}

	value = *buffer->readPtr;
	buffer->readPtr++;
	buffer->count--;

	if(buffer->readPtr == buffer->bufferEnd) {
		buffer->readPtr = buffer->bufferStart;
	}

	//uartPrintf("queue(dequeue): %d %x %x %x %x\r\n", buffer->count, buffer->readPtr, buffer->writePtr, buffer->bufferStart, buffer->bufferEnd);
        spin_unlock_irqrestore(&buffer->lock, flags);
	return value;
}

static int8_t ringBufferEnqueue(RingBuffer* buffer, u8 value) {
	unsigned long flags;
        spin_lock_irqsave(&buffer->lock, flags);
	if(buffer->count == buffer->size) {
        	spin_unlock_irqrestore(&buffer->lock, flags);
		return -1;
	}

	*buffer->writePtr = value;
	buffer->writePtr++;
	buffer->count++;

	if(buffer->writePtr == buffer->bufferEnd) {
		buffer->writePtr = buffer->bufferStart;
	}

	//uartPrintf("queue(enqueue): %d %x %x %x %x\r\n", buffer->count, buffer->readPtr, buffer->writePtr, buffer->bufferStart, buffer->bufferEnd);
        spin_unlock_irqrestore(&buffer->lock, flags);

	return value;
}

static u16 packetsizeFromSpeed(int speed_id)
{
	switch(speed_id) {
		case USB_HIGHSPEED:
			return 512;
		case USB_FULLSPEED:
		case USB_FULLSPEED_48_MHZ:
			return 64;
		case USB_LOWSPEED:
			return 32;
		default:
			return -1;
	}
}

static int advanceTxQueue(void)
{
	unsigned long flags;
	s8 nextEP;
	u32 controlStatus;
	struct iphone_usb *dev = &iphone_usb_state;
        spin_lock_irqsave(&dev->lock, flags);
	if(currentlySending != 0xFF) {
        	spin_unlock_irqrestore(&dev->lock, flags);
		return -1;
	}

	nextEP = ringBufferDequeue(txQueue);
	if(nextEP < 0) {
        	spin_unlock_irqrestore(&dev->lock, flags);
		return -1;
	}

	currentlySending = nextEP;
        spin_unlock_irqrestore(&dev->lock, flags);

	/*int endpoint;
	for(endpoint = 0; endpoint < USB_NUM_ENDPOINTS; endpoint++) {
		if(endpoint_directions[endpoint] == USBIn || endpoint_directions[endpoint] == USBBiDir) {
			InEPRegs[endpoint].control = (InEPRegs[endpoint].control & ~(DCTL_NEXTEP_MASK << DCTL_NEXTEP_SHIFT)) | ((nextEP & DCTL_NEXTEP_MASK) << DCTL_NEXTEP_SHIFT);
		}
	}*/

	InEPRegs[0].control = (InEPRegs[0].control & ~(DCTL_NEXTEP_MASK << DCTL_NEXTEP_SHIFT)) | ((nextEP & DCTL_NEXTEP_MASK) << DCTL_NEXTEP_SHIFT);
	InEPRegs[1].control = (InEPRegs[1].control & ~(DCTL_NEXTEP_MASK << DCTL_NEXTEP_SHIFT)) | ((nextEP & DCTL_NEXTEP_MASK) << DCTL_NEXTEP_SHIFT);
	InEPRegs[3].control = (InEPRegs[3].control & ~(DCTL_NEXTEP_MASK << DCTL_NEXTEP_SHIFT)) | ((nextEP & DCTL_NEXTEP_MASK) << DCTL_NEXTEP_SHIFT);
	InEPRegs[5].control = (InEPRegs[5].control & ~(DCTL_NEXTEP_MASK << DCTL_NEXTEP_SHIFT)) | ((nextEP & DCTL_NEXTEP_MASK) << DCTL_NEXTEP_SHIFT);

	// clear all the interrupts
	InEPRegs[nextEP].interrupt = USB_EPINT_ALL;

	// we're ready to transmit!
	controlStatus = InEPRegs[nextEP].control;
	InEPRegs[nextEP].control = controlStatus | USB_EPCON_ENABLE | USB_EPCON_CLEARNAK;

	return 0;
}

static void setConfiguration(int i) {
	int j;
	int k;
	int endpoint;
	for(j = 0; j < configurations[i].descriptor.bNumInterfaces; j++) {
		for(k = 0; k < configurations[i].interfaces[j].descriptor.bNumEndpoints; k++) {
			endpoint = configurations[i].interfaces[j].endpointDescriptors[k].bEndpointAddress & 0x3;
			if((configurations[i].interfaces[j].endpointDescriptors[k].bEndpointAddress & (0x1 << 7)) == (0x1 << 7)) {
				InEPRegs[endpoint].control = InEPRegs[endpoint].control | DCTL_SETD0PID;
			} else {
				OutEPRegs[endpoint].control = OutEPRegs[endpoint].control | DCTL_SETD0PID;
			}
		}
	}
}

static void receive(int endpoint, USBTransferType transferType, void* buffer, int packetLength, int bufferLen)
{
	if(endpoint == USB_CONTROLEP) {
		OutEPRegs[endpoint].transferSize = ((USB_SETUP_PACKETS_AT_A_TIME & DOEPTSIZ0_SUPCNT_MASK) << DOEPTSIZ0_SUPCNT_SHIFT)
			| ((USB_SETUP_PACKETS_AT_A_TIME & DOEPTSIZ0_PKTCNT_MASK) << DEPTSIZ_PKTCNT_SHIFT) | (bufferLen & DEPTSIZ0_XFERSIZ_MASK);
	} else {
		// divide our buffer into packets. Apple uses fancy bitwise arithmetic while we call huge libgcc integer arithmetic functions
		// for the sake of code readability. Will this matter?
		int packetCount = bufferLen / packetLength;
		if((bufferLen % packetLength) != 0)
			++packetCount;

		OutEPRegs[endpoint].transferSize = ((packetCount & DEPTSIZ_PKTCNT_MASK) << DEPTSIZ_PKTCNT_SHIFT) | (bufferLen & DEPTSIZ_XFERSIZ_MASK);
	}

	__raw_writel(__raw_readl(USB + DAINTMSK) | (1 << (DAINTMSK_OUT_SHIFT + endpoint)), USB + DAINTMSK);

	OutEPRegs[endpoint].dmaAddress = buffer;

	// start the transfer
	OutEPRegs[endpoint].control = USB_EPCON_ENABLE | USB_EPCON_CLEARNAK | USB_EPCON_ACTIVE
		| ((transferType & USB_EPCON_TYPE_MASK) << USB_EPCON_TYPE_SHIFT) | (packetLength & USB_EPCON_MPS_MASK);
}

static void receiveControl(void* buffer, int bufferLen)
{
	dmac_flush_range(buffer, ((u8*)buffer) + bufferLen);
	receive(USB_CONTROLEP, USBControl, buffer, USB_MAX_PACKETSIZE, bufferLen);
}

static void usbTxRx(int endpoint, USBDirection direction, USBTransferType transferType, void* buffer, int bufferLen) {
	int packetLength;
	int packetCount;

	if(transferType == USBControl || transferType == USBInterrupt) {
		packetLength = USB_MAX_PACKETSIZE;
	} else {
		packetLength = packetsizeFromSpeed(usb_speed);
	}

	dmac_flush_range(buffer, ((u8*)buffer) + bufferLen);

	if(direction == USBOut) {
		receive(endpoint, transferType, buffer, packetLength, bufferLen);
		return;
	}

	if(GNPTXFSTS_GET_TXQSPCAVAIL(__raw_readl(USB + GNPTXFSTS)) == 0) {
		// no space available
		return;
	}

	// enable interrupts for this endpoint
	__raw_writel(__raw_readl(USB + DAINTMSK) | ((1 << endpoint) << DAINTMSK_IN_SHIFT), USB + DAINTMSK);
	
	InEPRegs[endpoint].dmaAddress = buffer;

	if(endpoint == USB_CONTROLEP) {
		// we'll only send one packet at a time on CONTROLEP
		InEPRegs[endpoint].transferSize = ((1 & DEPTSIZ_PKTCNT_MASK) << DEPTSIZ_PKTCNT_SHIFT) | (bufferLen & DEPTSIZ0_XFERSIZ_MASK);
		InEPRegs[endpoint].control = USB_EPCON_CLEARNAK;
		return;
	}

	packetCount = bufferLen / packetLength;
	if((bufferLen % packetLength) != 0)
		++packetCount;


	InEPRegs[endpoint].transferSize = ((packetCount & DEPTSIZ_PKTCNT_MASK) << DEPTSIZ_PKTCNT_SHIFT)
		| (bufferLen & DEPTSIZ_XFERSIZ_MASK) | ((USB_MULTICOUNT & DIEPTSIZ_MC_MASK) << DIEPTSIZ_MC_SHIFT);


	InEPRegs[endpoint].control = USB_EPCON_CLEARNAK | USB_EPCON_ACTIVE | ((transferType & USB_EPCON_TYPE_MASK) << USB_EPCON_TYPE_SHIFT) | (packetLength & USB_EPCON_MPS_MASK);
	
}

static void sendControl(void* buffer, int bufferLen) {
	usbTxRx(USB_CONTROLEP, USBIn, USBControl, buffer, bufferLen);
	ringBufferEnqueue(txQueue, USB_CONTROLEP);
	advanceTxQueue();
}

static void usb_send_bulk(int endpoint, void* buffer, int bufferLen) {
	usbTxRx(endpoint, USBIn, USBBulk, buffer, bufferLen);
	ringBufferEnqueue(txQueue, endpoint);
	advanceTxQueue();
}

static void usb_send_interrupt(int endpoint, void* buffer, int bufferLen) {
	usbTxRx(endpoint, USBIn, USBInterrupt, buffer, bufferLen);
	ringBufferEnqueue(txQueue, endpoint);
	advanceTxQueue();
}


static void usb_receive_bulk(int endpoint, void* buffer, int bufferLen) {
	usbTxRx(endpoint, USBOut, USBBulk, buffer, bufferLen);
}

static void usb_receive_interrupt(int endpoint, void* buffer, int bufferLen) {
	usbTxRx(endpoint, USBOut, USBInterrupt, buffer, bufferLen);
}

static int resetUSB(void)
{
	int endpoint;
	__raw_writel(__raw_readl(USB + DCFG) & ~DCFG_DEVICEADDRMSK, USB + DCFG);

	for(endpoint = 0; endpoint < USB_NUM_ENDPOINTS; endpoint++) {
		OutEPRegs[endpoint].control = OutEPRegs[endpoint].control | USB_EPCON_SETNAK;
	}

	// enable interrupts for endpoint 0
	__raw_writel(__raw_readl(USB + DAINTMSK) | ((1 << USB_CONTROLEP) << DAINTMSK_OUT_SHIFT) | ((1 << USB_CONTROLEP) << DAINTMSK_IN_SHIFT), USB + DAINTMSK);

	__raw_writel(USB_EPINT_XferCompl | USB_EPINT_SetUp | USB_EPINT_Back2BackSetup, USB + DOEPMSK);
	__raw_writel(USB_EPINT_XferCompl | USB_EPINT_AHBErr | USB_EPINT_TimeOUT, USB + DIEPMSK);

	receiveControl(ep0controlRecvBuffer, sizeof(USBSetupPacket));

	return 0;
}

static USBInterface* addInterfaceDescriptor(USBConfiguration* configuration, u8 bInterfaceNumber, u8 bAlternateSetting, u8 bInterfaceClass, u8 bInterfaceSubClass, u8 bInterfaceProtocol, u8 iInterface) {
	u8 newIndex = configuration->descriptor.bNumInterfaces;
	configuration->descriptor.bNumInterfaces++;

	configuration->interfaces = (USBInterface*) krealloc(configuration->interfaces, sizeof(USBInterface) * configuration->descriptor.bNumInterfaces, GFP_KERNEL);
	configuration->interfaces[newIndex].descriptor.bLength = sizeof(USBInterfaceDescriptor);
	configuration->interfaces[newIndex].descriptor.bDescriptorType = USBInterfaceDescriptorType;
	configuration->interfaces[newIndex].descriptor.bInterfaceNumber = bInterfaceNumber;
	configuration->interfaces[newIndex].descriptor.bAlternateSetting = bAlternateSetting;
	configuration->interfaces[newIndex].descriptor.bInterfaceClass = bInterfaceClass;
	configuration->interfaces[newIndex].descriptor.bInterfaceSubClass = bInterfaceSubClass;
	configuration->interfaces[newIndex].descriptor.bInterfaceProtocol = bInterfaceProtocol;
	configuration->interfaces[newIndex].descriptor.iInterface = iInterface;
	configuration->interfaces[newIndex].descriptor.bNumEndpoints = 0;
	configuration->interfaces[newIndex].endpointDescriptors = NULL;

	return &configuration->interfaces[newIndex];
}

static u8 addStringDescriptor(const char* descriptorString) {
	int sLen;
	u8 newIndex = numStringDescriptors;
	numStringDescriptors++;

	stringDescriptors = (USBStringDescriptor**) krealloc(stringDescriptors, sizeof(USBStringDescriptor*) * numStringDescriptors, GFP_KERNEL);

	sLen = strlen(descriptorString);
	stringDescriptors[newIndex] = (USBStringDescriptor*) kmalloc(sizeof(USBStringDescriptor) + sLen, GFP_KERNEL);
	stringDescriptors[newIndex]->bLength = sizeof(USBStringDescriptor) + sLen;
	stringDescriptors[newIndex]->bDescriptorType = USBStringDescriptorType;
	memcpy(stringDescriptors[newIndex]->bString, descriptorString, sLen);

	firstStringDescriptor = (USBFirstStringDescriptor*) krealloc(firstStringDescriptor, sizeof(USBFirstStringDescriptor) + (sizeof(uint16_t) * numStringDescriptors), GFP_KERNEL);
	firstStringDescriptor->bLength = sizeof(USBFirstStringDescriptor) + (sizeof(uint16_t) * numStringDescriptors);
	firstStringDescriptor->bDescriptorType = USBStringDescriptorType;
	firstStringDescriptor->wLANGID[newIndex] = USB_LANGID_ENGLISH_US;

	return (newIndex + 1);
}

USBStringDescriptor* usb_get_string_descriptor(int index) {
	if(index == 0) {
		return (USBStringDescriptor*) firstStringDescriptor;
	} else {
		return stringDescriptors[index - 1];
	}
}

static void releaseStringDescriptors(void) {
	int i;

	if(stringDescriptors == NULL) {
		return;
	}

	for(i = 0; i < numStringDescriptors; i++) {
		kfree(stringDescriptors[i]);
	}

	kfree(stringDescriptors);

	numStringDescriptors = 0;
	stringDescriptors = NULL;
}

static void releaseConfigurations(void) {
	int i;
	int j;
	if(configurations == NULL) {
		return;
	}

	for(i = 0; i < deviceDescriptor.bNumConfigurations; i++) {
		for(j = 0; j < configurations[i].descriptor.bNumInterfaces; j++) {
			kfree(configurations[i].interfaces[j].endpointDescriptors);
		}
		kfree(configurations[i].interfaces);
	}

	kfree(configurations);
	deviceDescriptor.bNumConfigurations = 0;
	configurations = NULL;
}

static int addConfiguration(u8 bConfigurationValue, uint8_t iConfiguration, uint8_t selfPowered, uint8_t remoteWakeup, uint16_t maxPower) {
	int newIndex = deviceDescriptor.bNumConfigurations;
	deviceDescriptor.bNumConfigurations++;
	deviceQualifierDescriptor.bNumConfigurations++;

	configurations = (USBConfiguration*) krealloc(configurations, sizeof(USBConfiguration) * deviceDescriptor.bNumConfigurations, GFP_KERNEL);
	configurations[newIndex].descriptor.bLength = sizeof(USBConfigurationDescriptor);
	configurations[newIndex].descriptor.bDescriptorType = USBConfigurationDescriptorType;
	configurations[newIndex].descriptor.wTotalLength = 0;
	configurations[newIndex].descriptor.bNumInterfaces = 0;
	configurations[newIndex].descriptor.bConfigurationValue = bConfigurationValue;
	configurations[newIndex].descriptor.iConfiguration = iConfiguration;
	configurations[newIndex].descriptor.bmAttributes = ((0x1) << 7) | ((selfPowered & 0x1) << 6) | ((remoteWakeup & 0x1) << 5);
	configurations[newIndex].descriptor.bMaxPower = maxPower / 2;
	configurations[newIndex].interfaces = NULL;

	return newIndex;
}

static void endConfiguration(USBConfiguration* configuration) {
	int i;
	configuration->descriptor.wTotalLength = sizeof(USBConfigurationDescriptor);

	for(i = 0; i < configurations->descriptor.bNumInterfaces; i++) {
		configuration->descriptor.wTotalLength += sizeof(USBInterfaceDescriptor) + (configuration->interfaces[i].descriptor.bNumEndpoints * sizeof(USBEndpointDescriptor));
	}
}

USBConfigurationDescriptor* usb_get_configuration_descriptor(int index, u8 speed_id) {
	if(index == 0 && configurations[0].interfaces == NULL) {
		USBInterface* interface = addInterfaceDescriptor(&configurations[0], 0, 0,
			OPENIBOOT_INTERFACE_CLASS, OPENIBOOT_INTERFACE_SUBCLASS, OPENIBOOT_INTERFACE_PROTOCOL, addStringDescriptor("IF0"));

		enumerateHandler(interface);
		endConfiguration(&configurations[0]);
	}

	return &configurations[index].descriptor;
}

static void create_descriptors(void) {
	if(configurations == NULL) {
		deviceDescriptor.bLength = sizeof(USBDeviceDescriptor);
		deviceDescriptor.bDescriptorType = USBDeviceDescriptorType;
		deviceDescriptor.bcdUSB = USB_2_0;
		deviceDescriptor.bDeviceClass = 0;
		deviceDescriptor.bDeviceSubClass = 0;
		deviceDescriptor.bDeviceProtocol = 0;
		deviceDescriptor.bMaxPacketSize = USB_MAX_PACKETSIZE;
		deviceDescriptor.idVendor = VENDOR_APPLE;
		deviceDescriptor.idProduct = PRODUCT_IPHONE;
		deviceDescriptor.bcdDevice = DEVICE_IPHONE;
		deviceDescriptor.iManufacturer = addStringDescriptor("Apple Inc.");
		deviceDescriptor.iProduct = addStringDescriptor("Apple Mobile Device (iPhone Linux)");
		deviceDescriptor.iSerialNumber = addStringDescriptor("");
		deviceDescriptor.bNumConfigurations = 0;

		deviceQualifierDescriptor.bDescriptorType = USBDeviceDescriptorType;
		deviceQualifierDescriptor.bcdUSB = USB_2_0;
		deviceQualifierDescriptor.bDeviceClass = 0;
		deviceQualifierDescriptor.bDeviceSubClass = 0;
		deviceQualifierDescriptor.bDeviceProtocol = 0;
		deviceDescriptor.bMaxPacketSize = USB_MAX_PACKETSIZE;
		deviceDescriptor.bNumConfigurations = 0;

		addConfiguration(1, addStringDescriptor("iPhone Linux Configuration"), 0, 0, 500);
	}
}

USBDeviceDescriptor* usb_get_device_descriptor(void) {
	create_descriptors();
	return &deviceDescriptor;
}

USBDeviceQualifierDescriptor* usb_get_device_qualifier_descriptor(void) {
	create_descriptors();
	return &deviceQualifierDescriptor;
}

static u32 getConfigurationTree(int i, u8 speed_id, void* buffer) {
	int j;
	int k;
	u8 *buf = (u8*) buffer;
	u32 pos = 0;

	if(configurations == NULL) {
		return 0;
	}

	memcpy(buf + pos, usb_get_configuration_descriptor(i, speed_id), sizeof(USBConfigurationDescriptor));
	pos += sizeof(USBConfigurationDescriptor);

	for(j = 0; j < configurations[i].descriptor.bNumInterfaces; j++) {
		memcpy(buf + pos, &configurations[i].interfaces[j].descriptor, sizeof(USBInterfaceDescriptor));
		pos += sizeof(USBInterfaceDescriptor);
		for(k = 0; k < configurations[i].interfaces[j].descriptor.bNumEndpoints; k++) {
			memcpy(buf + pos, &configurations[i].interfaces[j].endpointDescriptors[k], sizeof(USBEndpointDescriptor));
			pos += sizeof(USBEndpointDescriptor);
		}
	}

	return pos;
}

static void getEndpointInterruptStatuses(void) {
	// To not mess up the interrupt controller, we can only read the interrupt status once per interrupt, so we need to cache them here
	int endpoint;
	for(endpoint = 0; endpoint < USB_NUM_ENDPOINTS; endpoint++) {
		if(endpoint_directions[endpoint] == USBIn || endpoint_directions[endpoint] == USBBiDir) {
			inInterruptStatus[endpoint] = InEPRegs[endpoint].interrupt;
		}
		if(endpoint_directions[endpoint] == USBOut || endpoint_directions[endpoint] == USBBiDir) {
			outInterruptStatus[endpoint] = OutEPRegs[endpoint].interrupt;
		}
	}
}

static int isSetupPhaseDone(void) {
	u32 status = __raw_readl(USB + DAINTMSK);
	int isDone = 0;

	if((status & ((1 << USB_CONTROLEP) << DAINTMSK_OUT_SHIFT)) == ((1 << USB_CONTROLEP) << DAINTMSK_OUT_SHIFT)) {
		if((outInterruptStatus[USB_CONTROLEP] & USB_EPINT_SetUp) == USB_EPINT_SetUp) {
			isDone = 1;
		}
	}

	// clear interrupt
	OutEPRegs[USB_CONTROLEP].interrupt = USB_EPINT_SetUp;

	return isDone;
}

static void handleTxInterrupts(int endpoint) {
	if(!inInterruptStatus[endpoint]) {
		return;
	}

	//uartPrintf("\tendpoint %d has an TX interrupt\r\n", endpoint);

	// clear pending interrupts
	if((inInterruptStatus[endpoint] & USB_EPINT_INEPNakEff) == USB_EPINT_INEPNakEff) {
		InEPRegs[endpoint].interrupt = USB_EPINT_INEPNakEff;
		//uartPrintf("\t\tUSB_EPINT_INEPNakEff\r\n");
	}

	if((inInterruptStatus[endpoint] & USB_EPINT_INTknEPMis) == USB_EPINT_INTknEPMis) {
		InEPRegs[endpoint].interrupt = USB_EPINT_INTknEPMis;
		//uartPrintf("\t\tUSB_EPINT_INTknEPMis\r\n");

		// clear the corresponding core interrupt
		__raw_writel(__raw_readl(USB + GINTSTS) | GINTMSK_EPMIS, USB + GINTSTS);
	}

	if((inInterruptStatus[endpoint] & USB_EPINT_INTknTXFEmp) == USB_EPINT_INTknTXFEmp) {
		InEPRegs[endpoint].interrupt = USB_EPINT_INTknTXFEmp;
		//uartPrintf("\t\tUSB_EPINT_INTknTXFEmp\r\n");
	}

	if((inInterruptStatus[endpoint] & USB_EPINT_TimeOUT) == USB_EPINT_TimeOUT) {
		InEPRegs[endpoint].interrupt = USB_EPINT_TimeOUT;
		//uartPrintf("\t\tUSB_EPINT_TimeOUT\r\n");
		currentlySending = 0xFF;
		//ringBufferDequeue(txQueue);
	}

	if((inInterruptStatus[endpoint] & USB_EPINT_AHBErr) == USB_EPINT_AHBErr) {
		InEPRegs[endpoint].interrupt = USB_EPINT_AHBErr;
		//uartPrintf("\t\tUSB_EPINT_AHBErr\r\n");
	}

	if((inInterruptStatus[endpoint] & USB_EPINT_EPDisbld) == USB_EPINT_EPDisbld) {
		InEPRegs[endpoint].interrupt = USB_EPINT_EPDisbld;
		//uartPrintf("\t\tUSB_EPINT_EPDisbldr\n");
	}

	if((inInterruptStatus[endpoint] & USB_EPINT_XferCompl) == USB_EPINT_XferCompl) {
		InEPRegs[endpoint].interrupt = USB_EPINT_XferCompl;
		//uartPrintf("\t\tUSB_EPINT_XferCompl\n");
		currentlySending = 0xFF;
		advanceTxQueue();
	}
	
}

static void handleRxInterrupts(int endpoint) {
	if(!outInterruptStatus[endpoint]) {
		return;
	}

	//uartPrintf("\tendpoint %d has an RX interrupt\r\n", endpoint);
		
	if((outInterruptStatus[endpoint] & USB_EPINT_Back2BackSetup) == USB_EPINT_Back2BackSetup) {
		OutEPRegs[endpoint].interrupt = USB_EPINT_Back2BackSetup;
	}

	if((outInterruptStatus[endpoint] & USB_EPINT_OUTTknEPDis) == USB_EPINT_OUTTknEPDis) {
		OutEPRegs[endpoint].interrupt = USB_EPINT_OUTTknEPDis;
	}

	if((outInterruptStatus[endpoint] & USB_EPINT_AHBErr) == USB_EPINT_AHBErr) {
		OutEPRegs[endpoint].interrupt = USB_EPINT_AHBErr;
	}

	if((outInterruptStatus[endpoint] & USB_EPINT_EPDisbld) == USB_EPINT_EPDisbld) {
		OutEPRegs[endpoint].interrupt = USB_EPINT_EPDisbld;
	}

	if((outInterruptStatus[endpoint] & USB_EPINT_XferCompl) == USB_EPINT_XferCompl) {
		OutEPRegs[endpoint].interrupt = USB_EPINT_XferCompl;
		if(endpoint == 0) {
			receiveControl(ep0controlRecvBuffer, sizeof(USBSetupPacket));
		}
	}
}

static void callEndpointHandlers(void) {
	u32 status = __raw_readl(USB + DAINTMSK);

	int endpoint;
	for(endpoint = 0; endpoint < USB_NUM_ENDPOINTS; endpoint++) {
		if((status & ((1 << endpoint) << DAINTMSK_OUT_SHIFT)) == ((1 << endpoint) << DAINTMSK_OUT_SHIFT)) {
			if((outInterruptStatus[endpoint] & USB_EPINT_XferCompl) == USB_EPINT_XferCompl) {
				if(endpoint_handlers[endpoint].out.handler != NULL) {
					endpoint_handlers[endpoint].out.handler(endpoint_handlers[endpoint].out.token);
				}
			}
		}

		if((status & ((1 << endpoint) << DAINTMSK_IN_SHIFT)) == ((1 << endpoint) << DAINTMSK_IN_SHIFT)) {
			if((inInterruptStatus[endpoint] & USB_EPINT_XferCompl) == USB_EPINT_XferCompl) {
				if(endpoint_handlers[endpoint].in.handler != NULL) {
					endpoint_handlers[endpoint].in.handler(endpoint_handlers[endpoint].in.token);
				}
			}
		}

		if(endpoint_directions[endpoint] == USBOut || endpoint_directions[endpoint] == USBBiDir) {
			handleRxInterrupts(endpoint);
		}

		if(endpoint_directions[endpoint] == USBIn || endpoint_directions[endpoint] == USBBiDir) {
			handleTxInterrupts(endpoint);
		}
	}
}

static irqreturn_t usbIRQHandler(int irq, void* _dev)
{
	int retval;
	u32 status;
	struct iphone_usb *dev = _dev;
	int process = 0;
        spin_lock(&dev->lock);
	status = __raw_readl(USB + GINTSTS) & __raw_readl(USB + GINTMSK);

	//uartPrintf("<begin interrupt: %x>\r\n", status);

	if(status) {
		process = 1;
	}

	while(process) {
		if((status & GINTMSK_OTG) == GINTMSK_OTG) {
			// acknowledge OTG interrupt (these bits are all R_SS_WC which means Write Clear, a write of 1 clears the bits)
			__raw_writel(__raw_readl(USB + GOTGINT), USB + GOTGINT);

			// acknowledge interrupt (this bit is actually RO, but should've been cleared when we cleared GOTGINT. Still, iBoot pokes it as if it was WC, so we will too)
			__raw_writel(GINTMSK_OTG, USB + GINTSTS);

			process = 1;
		} else {
			// we only care about OTG
			process = 0;
		}

		if((status & GINTMSK_RESET) == GINTMSK_RESET) {
			if(usb_state < USBError) {
				printk("iphoneusb: reset detected\r\n");
				change_state(USBPowered);
			}

			retval = resetUSB();

			__raw_writel(GINTMSK_RESET, USB + GINTSTS);

			if(retval) {
				printk("iphoneusb: listening for further usb events\r\n");
				spin_unlock(&dev->lock);
				return IRQ_HANDLED;	
			}

			process = 1;
		}

		if(((status & GINTMSK_INEP) == GINTMSK_INEP) || ((status & GINTMSK_OEP) == GINTMSK_OEP)) {
			// aha, got something on one of the endpoints. Now the real fun begins

			// first, let's get the interrupt status of individual endpoints
			getEndpointInterruptStatuses();

			if(isSetupPhaseDone()) {
				// recall our earlier receiveControl calls. We now should have 8 bytes of goodness in controlRecvBuffer.
				USBSetupPacket* setupPacket = (USBSetupPacket*) ep0controlRecvBuffer;

				uint16_t length;
				u32 totalLength;
				USBStringDescriptor* strDesc;
				if(USBSetupPacketRequestTypeType(setupPacket->bmRequestType) != USBSetupPacketVendor) {
					switch(setupPacket->bRequest) {
						case USB_GET_DESCRIPTOR:
							length = setupPacket->wLength;
							// descriptor type is high, descriptor index is low
							switch(setupPacket->wValue >> 8) {
								case USBDeviceDescriptorType:
									if(length > sizeof(USBDeviceDescriptor))
										length = sizeof(USBDeviceDescriptor);

									memcpy(ep0controlSendBuffer, usb_get_device_descriptor(), length);
									break;
								case USBConfigurationDescriptorType:
									// hopefully SET_ADDRESS was received beforehand to set the speed
									totalLength = getConfigurationTree(setupPacket->wValue & 0xFF, usb_speed, ep0controlSendBuffer);
									if(length > totalLength)
										length = totalLength;
									break;
								case USBStringDescriptorType:
									strDesc = usb_get_string_descriptor(setupPacket->wValue & 0xFF);
									if(length > strDesc->bLength)
										length = strDesc->bLength;
									memcpy(ep0controlSendBuffer, strDesc, length);
									break;
								case USBDeviceQualifierDescriptorType:
									if(length > sizeof(USBDeviceQualifierDescriptor))
										length = sizeof(USBDeviceQualifierDescriptor);

									memcpy(ep0controlSendBuffer, usb_get_device_qualifier_descriptor(), length);
									break;
								default:
									printk("iphoneusb: Unknown descriptor request: %d\r\n", setupPacket->wValue >> 8);
									if(usb_state < USBError) {
										change_state(USBUnknownDescriptorRequest);
									}
							}

							if(usb_state < USBError) {
								sendControl(ep0controlSendBuffer, length);
							}

							break;

						case USB_SET_ADDRESS:
							usb_speed = DSTS_GET_SPEED(__raw_readl(USB + DSTS));
							usb_max_packet_size = packetsizeFromSpeed(usb_speed);
							__raw_writel((__raw_readl(USB + DCFG) & ~DCFG_DEVICEADDRMSK)
								| ((setupPacket->wValue & DCFG_DEVICEADDR_UNSHIFTED_MASK) << DCFG_DEVICEADDR_SHIFT), USB + DCFG);

							// send an acknowledgement
							sendControl(ep0controlSendBuffer, 0);

							if(usb_state < USBError) {
								change_state(USBAddress);
							}
							break;

						case USB_SET_INTERFACE:
							// send an acknowledgement
							sendControl(ep0controlSendBuffer, 0);
							break;

						case USB_GET_STATUS:
							// FIXME: iBoot doesn't really care about this status
							*((uint16_t*) ep0controlSendBuffer) = 0;
							sendControl(ep0controlSendBuffer, sizeof(uint16_t));
							break;

						case USB_GET_CONFIGURATION:
							// FIXME: iBoot just puts out a debug message on console for this request.
							break;

						case USB_SET_CONFIGURATION:
							setConfiguration(0);
							// send an acknowledgment
							sendControl(ep0controlSendBuffer, 0);

							if(usb_state < USBError) {
								change_state(USBConfigured);
								startHandler();
							}
							break;
						default:
							if(usb_state < USBError) {
								change_state(USBUnknownRequest);
							}
					}

					// get the next SETUP packet
					receiveControl(ep0controlRecvBuffer, sizeof(USBSetupPacket));
				}
			} else {
				//uartPrintf("\t<begin callEndpointHandlers>\r\n");
				callEndpointHandlers();
				//uartPrintf("\t<end callEndpointHandlers>\r\n");
			}

			process = 1;
		}

		if((status & GINTMSK_SOF) == GINTMSK_SOF) {
			__raw_writel(GINTMSK_SOF, USB + GINTSTS);
			process = 1;
		}

		if((status & GINTMSK_SUSPEND) == GINTMSK_SUSPEND) {
			__raw_writel(GINTMSK_SUSPEND, USB + GINTSTS);
			process = 1;
		}

		status = __raw_readl(USB + GINTSTS) & __raw_readl(USB + GINTMSK);
	}
	spin_unlock(&dev->lock);
	return IRQ_HANDLED;
}

static int iphone_usb_setup(struct platform_device *pdev)
{
        int retval = 0;
	int i;

	struct iphone_usb *dev = &iphone_usb_state;
        spin_lock_init(&dev->lock);

	InEPRegs = (USBEPRegisters*)(USB + USB_INREGS);
	OutEPRegs = (USBEPRegisters*)(USB + USB_OUTREGS);

	change_state(USBStart);

	// Power on hardware
	iphone_power_ctrl(POWER_USB, 1);
	mdelay(USB_START_DELAYMS);

	// Initialize our data structures
	for(i = 0; i < USB_NUM_ENDPOINTS; i++) {
		switch(USB_EP_DIRECTION(i)) {
			case USB_ENDPOINT_DIRECTIONS_BIDIR:
				endpoint_directions[i] = USBBiDir;
				break;
			case USB_ENDPOINT_DIRECTIONS_IN:
				endpoint_directions[i] = USBIn;
				break;
			case USB_ENDPOINT_DIRECTIONS_OUT:
				endpoint_directions[i] = USBOut;
				break;
		}
		printk("iphoneusb: EP %d: %d\r\n", i, endpoint_directions[i]);
	}

	memset(endpoint_handlers, 0, sizeof(endpoint_handlers));

	// Set up the hardware
	iphone_clock_gate_switch(USB_OTGCLOCKGATE, 1);
	iphone_clock_gate_switch(USB_PHYCLOCKGATE, 1);

	// Generate a soft disconnect on host
	__raw_writel(__raw_readl(USB + DCTL) | DCTL_SFTDISCONNECT, USB + DCTL);
	mdelay(USB_SFTDISCONNECT_DELAYMS);

	// power on OTG
	__raw_writel(__raw_readl(USB + USB_ONOFF) & (~USB_ONOFF_OFF), USB + USB_ONOFF);
	udelay(USB_ONOFFSTART_DELAYUS);

	// power on PHY
	__raw_writel(OPHYPWR_POWERON, USB_PHY + OPHYPWR);
	udelay(USB_PHYPWRPOWERON_DELAYUS);

	// select clock
	__raw_writel((__raw_readl(USB_PHY + OPHYCLK) & OPHYCLK_CLKSEL_MASK) | OPHYCLK_CLKSEL_48MHZ, USB_PHY + OPHYCLK);

	// reset phy
	__raw_writel(__raw_readl(USB_PHY + ORSTCON) | ORSTCON_PHYSWRESET, USB_PHY + ORSTCON);
	udelay(USB_RESET2_DELAYUS);
	__raw_writel(__raw_readl(USB_PHY + ORSTCON) & (~ORSTCON_PHYSWRESET), USB_PHY + ORSTCON);
	udelay(USB_RESET_DELAYUS);

	__raw_writel(GRSTCTL_CORESOFTRESET, USB + GRSTCTL);

	// wait until reset takes
	while((__raw_readl(USB + GRSTCTL) & GRSTCTL_CORESOFTRESET) == GRSTCTL_CORESOFTRESET);

	// wait until reset completes
	while((__raw_readl(USB + GRSTCTL) & ~GRSTCTL_AHBIDLE) != 0);

	udelay(USB_RESETWAITFINISH_DELAYUS);

	// allow host to reconnect
	__raw_writel(__raw_readl(USB + DCTL) & (~DCTL_SFTDISCONNECT), USB + DCTL);
	udelay(USB_SFTCONNECT_DELAYUS);

	// flag all interrupts as positive, maybe to disable them

	// Set 7th EP? This is what iBoot does
	InEPRegs[USB_NUM_ENDPOINTS].interrupt = USB_EPINT_INEPNakEff | USB_EPINT_INTknEPMis | USB_EPINT_INTknTXFEmp
		| USB_EPINT_TimeOUT | USB_EPINT_AHBErr | USB_EPINT_EPDisbld | USB_EPINT_XferCompl;
	OutEPRegs[USB_NUM_ENDPOINTS].interrupt = USB_EPINT_OUTTknEPDis
		| USB_EPINT_SetUp | USB_EPINT_AHBErr | USB_EPINT_EPDisbld | USB_EPINT_XferCompl;

	for(i = 0; i < USB_NUM_ENDPOINTS; i++) {
		InEPRegs[i].interrupt = USB_EPINT_INEPNakEff | USB_EPINT_INTknEPMis | USB_EPINT_INTknTXFEmp
			| USB_EPINT_TimeOUT | USB_EPINT_AHBErr | USB_EPINT_EPDisbld | USB_EPINT_XferCompl;
		OutEPRegs[i].interrupt = USB_EPINT_OUTTknEPDis
			| USB_EPINT_SetUp | USB_EPINT_AHBErr | USB_EPINT_EPDisbld | USB_EPINT_XferCompl;
	}

	// disable all interrupts until endpoint descriptors and configuration structures have been setup
	__raw_writel(GINTMSK_NONE, USB + GINTMSK);
	__raw_writel(USB_EPINT_NONE, USB + DIEPMSK);
	__raw_writel(USB_EPINT_NONE, USB + DOEPMSK);

        retval = request_irq(USB_INTERRUPT, usbIRQHandler, IRQF_DISABLED, driver_name, dev);
	disable_irq(USB_INTERRUPT);
	return retval;
}

static int iphone_usb_stop(struct platform_device *pdev)
{
        int retval = 0;
	iphone_power_ctrl(POWER_USB, 1);
	iphone_clock_gate_switch(USB_OTGCLOCKGATE, 1);
	iphone_clock_gate_switch(USB_PHYCLOCKGATE, 1);

	__raw_writel(__raw_readl(USB + USB_ONOFF) | USB_ONOFF_OFF, USB + USB_ONOFF); // reset link
	__raw_writel(OPHYPWR_FORCESUSPEND | OPHYPWR_PLLPOWERDOWN
		| OPHYPWR_XOPOWERDOWN | OPHYPWR_ANALOGPOWERDOWN | OPHYPWR_UNKNOWNPOWERDOWN, USB_PHY + OPHYPWR); // power down phy

	__raw_writel(ORSTCON_PHYSWRESET | ORSTCON_LINKSWRESET | ORSTCON_PHYLINKSWRESET, USB_PHY + ORSTCON); // reset phy/link

	udelay(USB_RESET_DELAYUS);	// wait a millisecond for the changes to stick

	iphone_clock_gate_switch(USB_OTGCLOCKGATE, 0);
	iphone_clock_gate_switch(USB_PHYCLOCKGATE, 0);
	iphone_power_ctrl(POWER_USB, 0);

	releaseConfigurations();
	releaseStringDescriptors();
	return retval;
}

static void initializeDescriptors(void) {
	numStringDescriptors = 0;
	stringDescriptors = NULL;
	configurations = NULL;
	firstStringDescriptor = NULL;
}

static int iphone_usb_start(USBEnumerateHandler hEnumerate, USBStartHandler hStart) {
	int i;
	enumerateHandler = hEnumerate;
	startHandler = hStart;
	currentlySending = 0xFF;

	if(txQueue == NULL)
		txQueue = createRingBuffer(TX_QUEUE_LEN);

	initializeDescriptors();

	if(ep0controlSendBuffer == NULL)
		ep0controlSendBuffer = kmalloc(CONTROL_SEND_BUFFER_LEN, GFP_KERNEL | GFP_DMA);

	if(ep0controlRecvBuffer == NULL)
		ep0controlRecvBuffer = kmalloc(CONTROL_RECV_BUFFER_LEN, GFP_KERNEL | GFP_DMA);

	__raw_writel(GAHBCFG_DMAEN | GAHBCFG_BSTLEN_INCR8 | GAHBCFG_MASKINT, USB + GAHBCFG);
	__raw_writel(GUSBCFG_PHYIF16BIT | GUSBCFG_SRPENABLE | GUSBCFG_HNPENABLE | ((5 & GUSBCFG_TURNAROUND_MASK) << GUSBCFG_TURNAROUND_SHIFT), USB + GUSBCFG);
	__raw_writel(DCFG_NZSTSOUTHSHK, USB + DCFG); // some random setting. See specs
	__raw_writel(__raw_readl(USB + DCFG) & ~(DCFG_DEVICEADDRMSK), USB + DCFG);
	InEPRegs[0].control = USB_EPCON_ACTIVE;
	OutEPRegs[0].control = USB_EPCON_ACTIVE;

	__raw_writel(RX_FIFO_DEPTH, USB + GRXFSIZ);
	__raw_writel((TX_FIFO_DEPTH << GNPTXFSIZ_DEPTH_SHIFT) | TX_FIFO_STARTADDR, USB + GNPTXFSIZ);

	for(i = 0; i < USB_NUM_ENDPOINTS; i++) {
		InEPRegs[i].interrupt = USB_EPINT_INEPNakEff | USB_EPINT_INTknEPMis | USB_EPINT_INTknTXFEmp
			| USB_EPINT_TimeOUT | USB_EPINT_AHBErr | USB_EPINT_EPDisbld | USB_EPINT_XferCompl;
		OutEPRegs[i].interrupt = USB_EPINT_OUTTknEPDis
			| USB_EPINT_SetUp | USB_EPINT_AHBErr | USB_EPINT_EPDisbld | USB_EPINT_XferCompl;
		
	}

	__raw_writel(GINTMSK_OTG | GINTMSK_SUSPEND | GINTMSK_RESET | GINTMSK_INEP | GINTMSK_OEP | GINTMSK_DISCONNECT, USB + GINTMSK);
	__raw_writel(DAINTMSK_ALL, USB + DAINTMSK);

	__raw_writel(USB_EPINT_XferCompl | USB_EPINT_SetUp | USB_EPINT_Back2BackSetup, USB + DOEPMSK);
	__raw_writel(USB_EPINT_XferCompl | USB_EPINT_AHBErr | USB_EPINT_TimeOUT, USB + DIEPMSK);

	InEPRegs[0].interrupt = USB_EPINT_ALL;
	OutEPRegs[0].interrupt = USB_EPINT_ALL;

	__raw_writel(DCTL_PROGRAMDONE + DCTL_CGOUTNAK + DCTL_CGNPINNAK, USB + DCTL);
	udelay(USB_PROGRAMDONE_DELAYUS);
	__raw_writel(__raw_readl(USB + GOTGCTL) | GOTGCTL_SESSIONREQUEST, USB + GOTGCTL);

	receiveControl(ep0controlRecvBuffer, sizeof(USBSetupPacket));

	change_state(USBPowered);

	enable_irq(USB_INTERRUPT);

	return 0;
}

static int addEndpointDescriptor(USBInterface* interface, int endpoint, USBDirection direction, USBTransferType transferType, USBSynchronisationType syncType, USBUsageType usageType, uint16_t wMaxPacketSize, int bInterval) {
	int newIndex;
	if(direction > 2)
		return -1;

	newIndex = interface->descriptor.bNumEndpoints;
	interface->descriptor.bNumEndpoints++;

	interface->endpointDescriptors = (USBEndpointDescriptor*) krealloc(interface->endpointDescriptors, sizeof(USBEndpointDescriptor) * interface->descriptor.bNumEndpoints, GFP_KERNEL);
	interface->endpointDescriptors[newIndex].bLength = sizeof(USBEndpointDescriptor);
	interface->endpointDescriptors[newIndex].bDescriptorType = USBEndpointDescriptorType;
	interface->endpointDescriptors[newIndex].bEndpointAddress = (endpoint & 0xF) | ((direction & 0x1) << 7);	// see USB specs for the bitfield spec
	interface->endpointDescriptors[newIndex].bmAttributes = (transferType & 0x3) | ((syncType & 0x3) << 2) | ((usageType & 0x3) << 4);
	interface->endpointDescriptors[newIndex].wMaxPacketSize = wMaxPacketSize;
	interface->endpointDescriptors[newIndex].bInterval = bInterval;

	return newIndex;
}

static void usb_add_endpoint(USBInterface* interface, int endpoint, USBDirection direction, USBTransferType transferType) {
	if(transferType == USBInterrupt) { 
		addEndpointDescriptor(interface, endpoint, direction, transferType, USBNoSynchronization, USBDataEndpoint, packetsizeFromSpeed(usb_speed), 32);
	} else {
		addEndpointDescriptor(interface, endpoint, direction, transferType, USBNoSynchronization, USBDataEndpoint, packetsizeFromSpeed(usb_speed), 0);
	}
}

static int usb_install_ep_handler(int endpoint, USBDirection direction, USBEndpointHandler handler, uint32_t token) {
	if(endpoint >= USB_NUM_ENDPOINTS) {
		return -1;
	}

	if(endpoint_directions[endpoint] != direction && endpoint_directions[endpoint] != USBBiDir) {
		return -1; // that endpoint can't handle this kind of direction
	}

	if(direction == USBIn) {
		endpoint_handlers[endpoint].in.handler = handler;
		endpoint_handlers[endpoint].in.token = token;
	} else if(direction == USBOut) {
		endpoint_handlers[endpoint].out.handler = handler;
		endpoint_handlers[endpoint].out.token = token;
	} else {
		return -1; // can only register IN or OUt directions
	}

	return 0;
}

static int MyBufferLen = 0;
static spinlock_t MyBufferLock;
static char* MyBuffer;
static char* pMyBuffer = NULL;

#define SCROLLBACK_LEN (1024*16)

static int getScrollbackLen(void)
{
	return MyBufferLen;
}

static void bufferFlush(char* buffer, int len)
{
	unsigned long flags;
	if(len == 0)
		return;
	spin_lock_irqsave(&MyBufferLock, flags);
	memcpy(buffer, MyBuffer, len);
	memmove(MyBuffer, MyBuffer + len, MyBufferLen - len);
	MyBufferLen -= len;
	pMyBuffer -= len;
	spin_unlock_irqrestore(&MyBufferLock, flags);
}

static void bufferPrint(const char* buffer, int len)
{
	unsigned long flags;
	spin_lock_irqsave(&MyBufferLock, flags);
	if((MyBufferLen + len) > SCROLLBACK_LEN) {
		spin_unlock_irqrestore(&MyBufferLock, flags);
		return;
	}

	MyBufferLen += len;
	memcpy(pMyBuffer, buffer, len);
	pMyBuffer += len;

	spin_unlock_irqrestore(&MyBufferLock, flags);
}

static void bufferSetup(void) {
	MyBuffer = (char*) kmalloc(SCROLLBACK_LEN, GFP_KERNEL);
	MyBufferLen = 0;
	pMyBuffer = MyBuffer;
	spin_lock_init(&MyBufferLock);
}

static struct uart_port iphone_usb_uart_port;

static void iphone_usb_serial_write(char* start, int bufferLen) {
	bufferPrint(start, bufferLen);
}

static void iphone_usb_serial_read(char* start, int bufferLen) {
	struct tty_struct* tty;

	tty = iphone_usb_uart_port.info->port.tty;
	if(tty) {
		tty_insert_flip_string(tty, start, bufferLen);
		iphone_usb_uart_port.icount.rx += bufferLen;
		tty_flip_buffer_push(tty);
	}
}

/**
 * iphone_usb_serial_type - What type of console are we?
 * @port: Port to operate with (we ignore since we only have one port)
 *
 */
static const char *iphone_usb_serial_type(struct uart_port *port)
{
	return ("iPhone USB Serial");
}

/**
 * iphone_usb_serial_tx_empty - Is the transmitter empty?  We pretend we're always empty
 * @port: Port to operate on (we ignore since we only have one port)
 *
 */
static unsigned int iphone_usb_serial_tx_empty(struct uart_port *port)
{
	return 1;
}

/**
 * iphone_usb_serial_stop_tx - stop the transmitter - no-op for us
 * @port: Port to operat eon - we ignore - no-op function
 *
 */
static void iphone_usb_serial_stop_tx(struct uart_port *port)
{
}

/**
 * iphone_usb_serial_release_port - Free i/o and resources for port - no-op for us
 * @port: Port to operate on - we ignore - no-op function
 *
 */
static void iphone_usb_serial_release_port(struct uart_port *port)
{
}

/**
 * iphone_usb_serial_enable_ms - Force modem status interrupts on - no-op for us
 * @port: Port to operate on - we ignore - no-op function
 *
 */
static void iphone_usb_serial_enable_ms(struct uart_port *port)
{
}

/**
 * iphone_usb_serial_shutdown - shut down the port - free irq and disable - no-op for us
 * @port: Port to shut down - we ignore
 *
 */
static void iphone_usb_serial_shutdown(struct uart_port *port)
{
}

/**
 * iphone_usb_serial_set_mctrl - set control lines (dtr, rts, etc) - no-op for our console
 * @port: Port to operate on - we ignore
 * @mctrl: Lines to set/unset - we ignore
 *
 */
static void iphone_usb_serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

/**
 * iphone_usb_serial_get_mctrl - get control line info, we just return a static value
 * @port: port to operate on - we only have one port so we ignore this
 *
 */
static unsigned int iphone_usb_serial_get_mctrl(struct uart_port *port)
{
	return TIOCM_CAR | TIOCM_RNG | TIOCM_DSR | TIOCM_CTS;
}

/**
 * iphone_usb_serial_stop_rx - Stop the receiver - we ignor ethis
 * @port: Port to operate on - we ignore
 *
 */
static void iphone_usb_serial_stop_rx(struct uart_port *port)
{
}

/**
 * iphone_usb_serial_start_tx - Start transmitter
 * @port: Port to operate on
 *
 */
static void iphone_usb_serial_start_tx(struct uart_port *port)
{
	int xmit_count, tail, head, loops, ii;
	char *start;
	struct circ_buf *xmit;

	xmit = &port->info->xmit;
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		return;
	}

	head = xmit->head;
	tail = xmit->tail;
	start = &xmit->buf[tail];
	loops = (head < tail) ? 2 : 1;

	for (ii = 0; ii < loops; ii++) {
		xmit_count = (head < tail) ?
		    (UART_XMIT_SIZE - tail) : (head - tail);
		
		iphone_usb_serial_write(start, xmit_count);
		port->icount.tx += xmit_count;
		tail += xmit_count;
		tail &= UART_XMIT_SIZE - 1;
		xmit->tail = tail;
		start = &xmit->buf[tail];
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		iphone_usb_serial_stop_tx(port);	/* no-op for us */
}

/**
 * iphone_usb_serial_break_ctl - handle breaks - ignored by us
 * @port: Port to operate on
 * @break_state: Break state
 *
 */
static void iphone_usb_serial_break_ctl(struct uart_port *port, int break_state)
{
}

/**
 * iphone_usb_serial_startup - Start up the serial port - always return 0 (We're always on)
 * @port: Port to operate on
 *
 */
static int iphone_usb_serial_startup(struct uart_port *port)
{
	return 0;
}

/**
 * iphone_usb_serial_set_termios - set termios stuff - we ignore these
 * @port: port to operate on
 * @termios: New settings
 * @termios: Old
 *
 */
static void
iphone_usb_serial_set_termios(struct uart_port *port, struct ktermios *termios,
		struct ktermios *old)
{
}

/**
 * iphone_usb_serial_request_port - allocate resources for port - ignored by us
 * @port: port to operate on
 *
 */
static int iphone_usb_serial_request_port(struct uart_port *port)
{
	return 0;
}

/**
 * iphone_usb_serial_config_port - allocate resources, set up - we ignore,  we're always on
 * @port: Port to operate on
 * @flags: flags used for port setup
 *
 */
static void iphone_usb_serial_config_port(struct uart_port *port, int flags)
{
}

static struct uart_ops iphone_usb_serial_ops = {
	.tx_empty     = iphone_usb_serial_tx_empty,
	.set_mctrl    = iphone_usb_serial_set_mctrl,
	.get_mctrl    = iphone_usb_serial_get_mctrl,
	.stop_tx      = iphone_usb_serial_stop_tx,
	.start_tx     = iphone_usb_serial_start_tx,
	.stop_rx      = iphone_usb_serial_stop_rx,
	.enable_ms    = iphone_usb_serial_enable_ms,
	.break_ctl    = iphone_usb_serial_break_ctl,
	.startup      = iphone_usb_serial_startup,
	.shutdown     = iphone_usb_serial_shutdown,
	.type         = iphone_usb_serial_type,
	.release_port = iphone_usb_serial_release_port,
	.request_port = iphone_usb_serial_request_port,
	.config_port  = iphone_usb_serial_config_port,
	.verify_port  = NULL,
	.set_termios  = iphone_usb_serial_set_termios,
};

static u8* controlSendBuffer = NULL;
static u8* controlRecvBuffer = NULL;
static u8* dataSendBuffer = NULL;
static u8* dataRecvBuffer = NULL;
static u8* commandRecvBuffer = NULL;
static u8* dataRecvPtr = NULL;
static u32 left = 0;
static u32 rxLeft = 0;

static u8* sendFilePtr = NULL;
static u32 sendFileBytesLeft = 0;

#define USB_BYTES_AT_A_TIME 0x80

static void controlReceived(u32 token) {
	u32 toRead;
	OpenIBootCmd* cmd = (OpenIBootCmd*)controlRecvBuffer;
	OpenIBootCmd* reply = (OpenIBootCmd*)controlSendBuffer;

	if(cmd->command == OPENIBOOTCMD_DUMPBUFFER) {
		int length;

		if(sendFileBytesLeft > 0) {
			length = sendFileBytesLeft;
		} else {
			length = getScrollbackLen(); // getScrollbackLen();// 0x80;
		}

		reply->command = OPENIBOOTCMD_DUMPBUFFER_LEN;
		reply->dataLen = length;
		usb_send_interrupt(3, controlSendBuffer, sizeof(OpenIBootCmd));
		//uartPrintf("got dumpbuffer cmd, returning length: %d\r\n", length);
	} else if(cmd->command == OPENIBOOTCMD_DUMPBUFFER_GOAHEAD) {
		left = cmd->dataLen;

		//uartPrintf("got dumpbuffer goahead, writing length: %d\r\n", (int)left);

		toRead = (left > USB_BYTES_AT_A_TIME) ? USB_BYTES_AT_A_TIME: left;
		if(sendFileBytesLeft > 0) {
			usb_send_bulk(1, sendFilePtr, toRead);
			sendFilePtr += toRead;
			sendFileBytesLeft -= toRead;
		} else {
			bufferFlush((char*) dataSendBuffer, toRead);
			usb_send_bulk(1, dataSendBuffer, toRead);
		}
		left -= toRead;
	} else if(cmd->command == OPENIBOOTCMD_SENDCOMMAND) {
		dataRecvPtr = dataRecvBuffer;
		rxLeft = cmd->dataLen;

		//uartPrintf("got sendcommand, receiving length: %d\r\n", (int)rxLeft);

		reply->command = OPENIBOOTCMD_SENDCOMMAND_GOAHEAD;
		reply->dataLen = cmd->dataLen;
		usb_send_interrupt(3, controlSendBuffer, sizeof(OpenIBootCmd));

		toRead = (rxLeft > USB_BYTES_AT_A_TIME) ? USB_BYTES_AT_A_TIME: rxLeft;
		usb_receive_bulk(2, dataRecvPtr, toRead);
		rxLeft -= toRead;
		dataRecvPtr += toRead;
	}

	usb_receive_interrupt(4, controlRecvBuffer, sizeof(OpenIBootCmd));
}

static void dataReceived(u32 token) {
	//uartPrintf("receiving remainder: %d\r\n", (int)rxLeft);
	if(rxLeft > 0) {
		u32 toRead = (rxLeft > USB_BYTES_AT_A_TIME) ? USB_BYTES_AT_A_TIME: rxLeft;
		usb_receive_bulk(2, dataRecvPtr, toRead);
		rxLeft -= toRead;
		dataRecvPtr += toRead;
	} else {
		iphone_usb_serial_read((char*)dataRecvBuffer, (u32)dataRecvPtr - (u32)dataRecvBuffer);
	}	
}

static void dataSent(u32 token) {
	if(left > 0) {
		u32 toRead = (left > USB_BYTES_AT_A_TIME) ? USB_BYTES_AT_A_TIME: left;
		if(sendFileBytesLeft > 0) {
			usb_send_bulk(1, sendFilePtr, toRead);
			sendFilePtr += toRead;
			sendFileBytesLeft -= toRead;
		} else {
			bufferFlush((char*) dataSendBuffer, toRead);
			usb_send_bulk(1, dataSendBuffer, toRead);
		}
		left -= toRead;
	}	
}

static void controlSent(u32 token) {
	//uartPrintf("control sent\r\n");
}

static void serialEnumerateHandler(USBInterface* interface) {
	usb_add_endpoint(interface, 1, USBIn, USBBulk);
	usb_add_endpoint(interface, 2, USBOut, USBBulk);
	usb_add_endpoint(interface, 3, USBIn, USBInterrupt);
	usb_add_endpoint(interface, 4, USBOut, USBInterrupt);

	if(!controlSendBuffer)
		controlSendBuffer = kmalloc(0x80, GFP_KERNEL | GFP_DMA);

	if(!controlRecvBuffer)
		controlRecvBuffer = kmalloc(0x80, GFP_KERNEL | GFP_DMA);

	if(!dataSendBuffer)
		dataSendBuffer = kmalloc(0x80, GFP_KERNEL | GFP_DMA);

	if(!dataRecvBuffer)
		dataRecvBuffer = commandRecvBuffer = kmalloc(0x80, GFP_KERNEL | GFP_DMA);
}

static void serialStartHandler(void) {
	printk("iphoneusb: got serial USB start\n");
	usb_receive_interrupt(4, controlRecvBuffer, sizeof(OpenIBootCmd));
}

static int iphone_usb_probe(struct platform_device *pdev)
{
	int retval = 0;
	retval = iphone_usb_setup(pdev);

	usb_install_ep_handler(4, USBOut, controlReceived, 0);
	usb_install_ep_handler(2, USBOut, dataReceived, 0);
	usb_install_ep_handler(3, USBIn, controlSent, 0);
	usb_install_ep_handler(1, USBIn, dataSent, 0);
	iphone_usb_start(serialEnumerateHandler, serialStartHandler);

	return retval;
}

static int iphone_usb_remove(struct platform_device *pdev)
{
	int retval = 0;
	retval = iphone_usb_stop(pdev);

	return retval;
}

static struct platform_driver iphone_usb_driver = {
        .probe          = iphone_usb_probe,
        .remove         = iphone_usb_remove,
        .suspend        = NULL,
        .resume         = NULL,
        .driver         = {
                .owner  = THIS_MODULE,
                .name   = "iphoneusb",
        },
};

struct uart_driver iphone_usb_uart_driver = {
	.owner        = THIS_MODULE,
	.driver_name  = "iphone_usb_serial",
	.dev_name     = "ttyUSB",
	.major        = TTY_MAJOR,
	.minor        = 128,
	.nr           = 1,
};

static void iphone_usb_console_write(struct console *co, const char *s, unsigned int count)
{
	int i;
	char ch;
	char cr = '\r';
	for(i = 0; i < count; i++) {
		ch = s[i];
		if(ch == '\n')
			bufferPrint(&cr, 1);

		bufferPrint(&ch, 1);
	}
}

static int __init iphone_usb_console_setup(struct console *co, char *options) {
	return 0;
}

static struct console iphone_usb_console = {
	.name     = "ttyUSB",
	.write    = iphone_usb_console_write,
	.device   = uart_console_device,
	.setup    = iphone_usb_console_setup,
	.flags    = CON_PRINTBUFFER,
	.index    = -1,
};

static struct platform_device *iphone_usb_device;

static int __init iphone_usb_init(void)
{
	int ret;

	ret = platform_driver_register(&iphone_usb_driver);
	if(!ret) {
		iphone_usb_device = platform_device_register_simple("iphoneusb", 0,
				NULL, 0);

		if (IS_ERR(iphone_usb_device)) {
			platform_driver_unregister(&iphone_usb_driver);
			ret = PTR_ERR(iphone_usb_device);
			printk("%s: Error loading version %s\n", driver_name, DRIVER_VERSION);
		} else {
			bufferSetup();

			uart_register_driver(&iphone_usb_uart_driver);
			spin_lock_init(&iphone_usb_uart_port.lock);
			iphone_usb_uart_port.membase = (char *)1;	/* just needs to be non-zero */
			iphone_usb_uart_port.type = PORT_16550A;
			iphone_usb_uart_port.fifosize = 1024;
			iphone_usb_uart_port.ops = &iphone_usb_serial_ops;
			iphone_usb_uart_port.line = 0;

			uart_add_one_port(&iphone_usb_uart_driver, &iphone_usb_uart_port);

			iphone_usb_console.data = &iphone_usb_uart_driver;
			register_console(&iphone_usb_console);
			printk("%s: loaded version %s\n", driver_name, DRIVER_VERSION);
		}
	}

	return ret;
}

static void __exit iphone_usb_exit(void)
{
	uart_unregister_driver(&iphone_usb_uart_driver);
        platform_device_unregister(iphone_usb_device);
        platform_driver_unregister(&iphone_usb_driver);
        printk("Unloaded %s version %s\n", driver_name, DRIVER_VERSION);
}

module_init(iphone_usb_init);
module_exit(iphone_usb_exit);

MODULE_LICENSE("GPL");
