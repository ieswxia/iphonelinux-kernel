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

#include <mach/iphone-clock.h>
#include "usb.h"
#include <mach/usb.h>

#define GET_BITS(x, start, length) ((((u32)(x)) << (32 - ((start) + (length)))) >> (32 - (length)))

#define DRIVER_VERSION          __DATE__

#define driver_name "iphoneudc"

static USBEPRegisters* InEPRegs;
static USBEPRegisters* OutEPRegs;
static u32 inInterruptStatus[USB_NUM_ENDPOINTS];
static u32 outInterruptStatus[USB_NUM_ENDPOINTS];
static u8* ep0controlSendBuffer = NULL;
static u8* ep0controlRecvBuffer = NULL;

struct iphone_udc;
static u8 currentlySending;

struct iphone_udc_request {
	struct usb_request			req;
	struct list_head			queue;

};

struct iphone_udc_ep {
	struct usb_ep				ep;
	struct list_head			queue;
	const struct usb_endpoint_descriptor*	desc;
	struct iphone_udc			*dev;
	USBTransferType				transferType;
	u8*					txBuf;
	u8*					rxBuf;
	unsigned				num:8;
};

struct iphone_udc {
	spinlock_t				lock;
	struct usb_gadget			gadget;
	struct iphone_udc_ep			ep[6]; 

	USBState				usb_state;

	struct usb_gadget_driver*		driver;

	struct platform_device*			pdev;
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

struct iphone_udc* the_controller;
static RingBuffer* txQueue = NULL;

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
static void change_state(USBState new_state)
{
	struct iphone_udc *dev = the_controller;

	printk("%s: USB state change: %d -> %d\r\n", driver_name, dev->usb_state, new_state);
	dev->usb_state = new_state;
	if(dev->usb_state == USBConfigured) {
		// TODO: set to host powered
	}
}

static u16 packetsizeFromSpeed(int speed_id)
{
	switch(speed_id) {
		case USB_SPEED_HIGH:
			return 512;
		case USB_SPEED_FULL:
			return 64;
		case USB_SPEED_LOW:
			return 32;
		default:
			return -1;
	}
}

static void receive(int endpoint, USBTransferType transferType, void* buffer, int packetLength, int bufferLen)
{
	//printk("receive: %d %d %p %d\n", packetLength, bufferLen, buffer, transferType);
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
		packetLength = packetsizeFromSpeed(the_controller->gadget.speed);
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

static int advanceTxQueue(void)
{
	s8 nextEP;
	u32 controlStatus;
	if(currentlySending != 0xFF) {
		return -1;
	}

	nextEP = ringBufferDequeue(txQueue);
	if(nextEP < 0) {
		return -1;
	}

	currentlySending = nextEP;

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

static void sendControl(void* buffer, int bufferLen) {
	usbTxRx(USB_CONTROLEP, USBIn, USBControl, buffer, bufferLen);
	ringBufferEnqueue(txQueue, USB_CONTROLEP);
	advanceTxQueue();
}

static void getEndpointInterruptStatuses(void) {
	// To not mess up the interrupt controller, we can only read the interrupt status once per interrupt, so we need to cache them here
	int endpoint;
	for(endpoint = 0; endpoint < USB_NUM_ENDPOINTS; endpoint++) {
		inInterruptStatus[endpoint] = InEPRegs[endpoint].interrupt;
		outInterruptStatus[endpoint] = OutEPRegs[endpoint].interrupt;
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
			receiveControl(ep0controlRecvBuffer, sizeof(struct usb_ctrlrequest));
		}
	}
}

static void done(struct iphone_udc_ep *ep, struct iphone_udc_request *req, int status)
{
	//printk("%s: %p\n", __FUNCTION__, ep);
	udelay(1000);
	list_del_init(&req->queue);

	if (likely(req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

	spin_unlock(&ep->dev->lock);
	req->req.complete(&ep->ep, &req->req);
	spin_lock(&ep->dev->lock);
}

void nuke(struct iphone_udc_ep *ep, int status)
{
	struct iphone_udc_request *req;

	/* called with irqs blocked */
	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct iphone_udc_request, queue);
		done(ep, req, status);
	}
}


static void write_packet(struct iphone_udc_ep* ep, struct iphone_udc_request* req, int max)
{
	int length;
	if(req->req.length > 0) {
		length = req->req.length - req->req.actual;
		length = min(length, max);
		memcpy(ep->txBuf, req->req.buf + req->req.actual, length);
	} else {
		length = 0;
	}
	//printk("%s (%d): %d - %d (%d)\n", __FUNCTION__, ep->num, req->req.actual, length, req->req.length);
	req->req.actual += length;
	if(ep->num == 0) {
		usbTxRx(ep->num, USBIn, USBControl, ep->txBuf, length);
	} else {
		usbTxRx(ep->num, USBIn, ep->transferType, ep->txBuf, length);
	}
	ringBufferEnqueue(txQueue, ep->num);
	advanceTxQueue();
}

static void receive_packet(struct iphone_udc_ep* ep, struct iphone_udc_request* req, int max)
{
	int length;
	length = req->req.length - req->req.actual;
	length = min(length, max);
	dmac_flush_range(ep->rxBuf, ((u8*)ep->rxBuf) + length);
	receive(ep->num, ep->transferType, ep->rxBuf, ep->ep.maxpacket, length);
}

static void advanceQueue(struct iphone_udc* dev, struct iphone_udc_ep* ep)
{
	struct iphone_udc_request *req;

	//printk("advanceQueue on ep %d\n", ep->num);

	if (list_empty(&ep->queue))
		req = NULL;
	else
		req = list_entry(ep->queue.next, struct iphone_udc_request, queue);

	if(req == NULL)
		return;

	if(ep->num == 0)
	{
		write_packet(ep, req, ep->ep.maxpacket);
	} else if(ep->desc->bEndpointAddress & USB_DIR_IN) {
		write_packet(ep, req, ep->ep.maxpacket);
	} else {
		receive_packet(ep, req, ep->ep.maxpacket);
	}
}

static void txCompleteRequest(int endpoint)
{
	int max;
	struct iphone_udc* dev = the_controller;
	struct iphone_udc_request *req;
	struct iphone_udc_ep *ep = &dev->ep[endpoint];

	//printk("txCompleteRequest on ep %d!\n", endpoint);

	if (list_empty(&ep->queue))
		req = NULL;
	else
		req = list_entry(ep->queue.next, struct iphone_udc_request, queue);

	if(req == NULL)
		return;

	if(req->req.length == 0) {
		//printk("%s: done with ZLP\n", __FUNCTION__);
		done(ep, req, 0);
		advanceQueue(dev, ep);
		return;
	}

	if(req->req.actual == req->req.length) {
		//printk("%s: done with non-ZLP\n", __FUNCTION__);
		done(ep, req, 0);
		advanceQueue(dev, ep);
		return;
	}

	max = ep->ep.maxpacket;
	write_packet(ep, req, max);
}

static void rxCompleteRequest(int endpoint, int leftToTransfer)
{
	int max;
	struct iphone_udc* dev = the_controller;
	struct iphone_udc_request *req;
	struct iphone_udc_ep *ep = &dev->ep[endpoint];
	int transferred;
	int requested;

	//printk("rxCompleteRequest on ep %d!\n", endpoint);

	if (list_empty(&ep->queue))
		req = NULL;
	else
		req = list_entry(ep->queue.next, struct iphone_udc_request, queue);

	if(req == NULL)
		return;

	max = ep->ep.maxpacket;
	requested = req->req.length - req->req.actual;
	requested = min(requested, max);
	transferred = requested - leftToTransfer;

	memcpy(req->req.buf + req->req.actual, ep->rxBuf, transferred);
	req->req.actual += transferred;

	done(ep, req, 0);

	advanceQueue(dev, ep);
}

static void callEndpointHandlers(void) {
	u32 status = __raw_readl(USB + DAINTMSK);

	int endpoint;
	for(endpoint = 0; endpoint < USB_NUM_ENDPOINTS; endpoint++) {
		if((status & ((1 << endpoint) << DAINTMSK_OUT_SHIFT)) == ((1 << endpoint) << DAINTMSK_OUT_SHIFT)) {
			if((outInterruptStatus[endpoint] & USB_EPINT_XferCompl) == USB_EPINT_XferCompl) {
				rxCompleteRequest(endpoint, OutEPRegs[endpoint].transferSize & DEPTSIZ_XFERSIZ_MASK);
			}
		}

		if((status & ((1 << endpoint) << DAINTMSK_IN_SHIFT)) == ((1 << endpoint) << DAINTMSK_IN_SHIFT)) {
			if((inInterruptStatus[endpoint] & USB_EPINT_XferCompl) == USB_EPINT_XferCompl) {
				txCompleteRequest(endpoint);
			}
		}

		handleRxInterrupts(endpoint);
		handleTxInterrupts(endpoint);
	}
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

	receiveControl(ep0controlRecvBuffer, sizeof(struct usb_ctrlrequest));

	return 0;
}

static int iphone_udc_ep_enable(struct usb_ep *_ep, const struct usb_endpoint_descriptor *desc)
{
	struct iphone_udc *dev;
	struct iphone_udc_ep *ep;

	printk("%s\n", __FUNCTION__);

	ep = container_of(_ep, struct iphone_udc_ep, ep);
	if (!_ep || !desc || ep->desc
			|| desc->bDescriptorType != USB_DT_ENDPOINT)
		return -EINVAL;
	dev = ep->dev;
	if (ep == &dev->ep[0])
		return -EINVAL;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;
	if (ep->num != (desc->bEndpointAddress & 0x0f))
		return -EINVAL;

	switch (desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) {
	case USB_ENDPOINT_XFER_BULK:
		ep->transferType = USBBulk;
		break;
	case USB_ENDPOINT_XFER_INT:
		ep->transferType = USBInterrupt;
		break;
	default:
		return -EINVAL;
	}

	if(ep->transferType == USBControl || ep->transferType == USBInterrupt) {
		ep->ep.maxpacket = USB_MAX_PACKETSIZE;
	} else {
		ep->ep.maxpacket = packetsizeFromSpeed(dev->gadget.speed);
	}

	ep->desc = desc;

	printk("%s: endpoint enabled: %d\n", driver_name, ep->num);
	return 0;
}

static int iphone_udc_ep_disable(struct usb_ep *_ep)
{
	struct iphone_udc_ep *ep;

	printk("%s\n", __FUNCTION__);

	ep = container_of(_ep, struct iphone_udc_ep, ep);
	if (!_ep || !ep->desc)
		return -ENODEV;

	printk("%s: endpoint disabled: %d\n", driver_name, ep->num);

	return 0;
}

static struct usb_request* iphone_udc_alloc_request(struct usb_ep *_ep, gfp_t gfp_flags)
{
	struct iphone_udc_request *req;

	//printk("%s\n", __FUNCTION__);

	if (!_ep)
		return NULL;
	req = kzalloc(sizeof(struct iphone_udc_request), gfp_flags);
	if (!req)
		return NULL;

	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}

static void iphone_udc_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct iphone_udc_request* req;

	//printk("%s\n", __FUNCTION__);

	if (!_ep || !_req)
		return;

	req = container_of(_req, struct iphone_udc_request, req);
	WARN_ON(!list_empty(&req->queue));
	kfree(req);
}

static int iphone_udc_queue(struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
	struct iphone_udc_request* req;
	struct iphone_udc_ep* ep;
	struct iphone_udc* dev;
	unsigned long flags;

	//printk("%s: EP-%p\n", __FUNCTION__, _ep);

	req = container_of(_req, struct iphone_udc_request, req);
	if (unlikely
			(!_req || !_req->complete || !_req->buf
			 || !list_empty(&req->queue))) {
		printk("%s: bad params: %p %p %p %d\n", __FUNCTION__, _req, _req->complete, _req->buf, list_empty(&req->queue));
		printk("%s: BAD PARAMS: %s queue req %p, len %d buf %p\n",
				__FUNCTION__, _ep->name, _req, _req->length, _req->buf);
		return -EINVAL;
	}

	ep = container_of(_ep, struct iphone_udc_ep, ep);
	if (unlikely(!_ep || (!ep->desc && ep->num != 0))) {
		printk("%s: bad ep\n", __FUNCTION__);
		return -EINVAL;
	}

	dev = ep->dev;
	if (unlikely(!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)) {
		printk("%s: bogus device state %p\n", __FUNCTION__, dev->driver);
		return -ESHUTDOWN;
	}

/*	printk("%s: %s queue req %p, len %d buf %p\n",
			__FUNCTION__, _ep->name, _req, _req->length, _req->buf);*/

	spin_lock_irqsave(&dev->lock, flags);
	_req->status = -EINPROGRESS;
	_req->actual = 0;
	
	if(list_empty(&ep->queue))
	{
		list_add_tail(&req->queue, &ep->queue);
		advanceQueue(dev, ep);
	} else {
		//printk("not writing due to queue not empty\n");
		list_add_tail(&req->queue, &ep->queue);
	}

	if (&req->req != _req) {
		spin_unlock_irqrestore (&dev->lock, flags);
		return -EINVAL;
	}

	spin_unlock_irqrestore (&dev->lock, flags);
	return 0;
}

static int iphone_udc_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct iphone_udc_request* req;
	struct iphone_udc_ep* ep;
	struct iphone_udc* dev;
	unsigned long flags;

	printk("%s\n", __FUNCTION__);

	ep = container_of(_ep, struct iphone_udc_ep, ep);
	if (!_ep || !_req || (!ep->desc && ep->num != 0))
		return -EINVAL;
	dev = ep->dev;
	if (!dev->driver)
		return -ESHUTDOWN;

        spin_lock_irqsave(&ep->dev->lock, flags);

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req) {
		spin_unlock_irqrestore(&ep->dev->lock, flags);
		return -EINVAL;
	}

	done(ep, req, -ECONNRESET);

	spin_unlock_irqrestore(&ep->dev->lock, flags);

	return 0;
}

static int iphone_udc_set_halt(struct usb_ep *_ep, int value)
{
	printk("%s: attempted halt!\n", driver_name);
	return 0;
}
	
static struct usb_ep_ops iphone_udc_ep_ops = {
	.enable		= iphone_udc_ep_enable,
	.disable	= iphone_udc_ep_disable,

	.alloc_request	= iphone_udc_alloc_request,
	.free_request	= iphone_udc_free_request,

	.queue		= iphone_udc_queue,
	.dequeue	= iphone_udc_dequeue,

	.set_halt	= iphone_udc_set_halt,
};

static void udc_reinit(struct iphone_udc *dev)
{
	static char *names[] = { "ep0", "ep1in-bulk", "ep2out-bulk", "ep3in-bulk", "ep4out-bulk", "ep5in-bulk" };

	unsigned i;

	INIT_LIST_HEAD (&dev->gadget.ep_list);
	dev->gadget.ep0 = &dev->ep[0].ep;
	dev->gadget.speed = USB_SPEED_UNKNOWN;

	for (i = 0; i < USB_NUM_ENDPOINTS; i++) {
		struct iphone_udc_ep* ep = &dev->ep[i];

		ep->num = i;
		ep->ep.name = names[i];

		ep->ep.ops = &iphone_udc_ep_ops;
		list_add_tail (&ep->ep.ep_list, &dev->gadget.ep_list);
		ep->dev = dev;
		INIT_LIST_HEAD (&ep->queue);

		InEPRegs[i].interrupt = USB_EPINT_INEPNakEff | USB_EPINT_INTknEPMis | USB_EPINT_INTknTXFEmp
			| USB_EPINT_TimeOUT | USB_EPINT_AHBErr | USB_EPINT_EPDisbld | USB_EPINT_XferCompl;
		OutEPRegs[i].interrupt = USB_EPINT_OUTTknEPDis
			| USB_EPINT_SetUp | USB_EPINT_AHBErr | USB_EPINT_EPDisbld | USB_EPINT_XferCompl;

		ep->ep.maxpacket = 512;

		if(ep->txBuf == NULL)
			ep->txBuf = kmalloc(ep->ep.maxpacket, GFP_KERNEL | GFP_DMA);
	
		if(ep->rxBuf == NULL)
			ep->rxBuf = kmalloc(ep->ep.maxpacket, GFP_KERNEL | GFP_DMA);
	}

	dev->ep[0].ep.maxpacket = USB_MAX_PACKETSIZE;
	list_del_init (&dev->ep[0].ep.ep_list);
}

static irqreturn_t usbIRQHandler(int irq, void* _dev);
static int iphone_udc_start(struct iphone_udc* dev) {
	int i;
	int retval;
	currentlySending = 0xFF;

	if(txQueue == NULL)
		txQueue = createRingBuffer(TX_QUEUE_LEN);

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

	change_state(USBPowered);

        retval = request_irq(USB_INTERRUPT, usbIRQHandler, IRQF_DISABLED, driver_name, dev);

	receiveControl(ep0controlRecvBuffer, sizeof(struct usb_ctrlrequest));

        return retval;
}

int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct iphone_udc *dev = the_controller;
	int retval = 0;

	if (!driver
			|| (driver->speed != USB_SPEED_FULL && driver->speed != USB_SPEED_HIGH)
			|| !driver->bind
			|| !driver->disconnect
			|| !driver->setup)
	{
		printk("%s: invalid gadget driver!\n", driver_name);
		return -EINVAL;
	}

	if (!dev)
	{
		printk("%s: no controller device yet!\n", driver_name);
		return -ENODEV;
	}

	if (dev->driver)
	{
		printk("%s: gadget driver already registered!\n", driver_name);
		return -EBUSY;
	}

	dev->driver = driver;
	dev->gadget.dev.driver = &driver->driver;

	udc_reinit(dev);
	if ((retval = device_add(&dev->gadget.dev)) != 0) {
		printk(KERN_ERR "Error in device_add() : %d\n",retval);
		return retval;
	}

	retval = driver->bind(&dev->gadget);
	if (retval) {
		printk("%s: unable to bind to driver, error %d\n", driver_name, retval);
		dev->driver = NULL;
		dev->gadget.dev.driver = NULL;
		return retval;
	}

	retval = iphone_udc_start(dev);

	printk("%s: gadget driver registered\n", driver_name);

	return retval;
}
EXPORT_SYMBOL(usb_gadget_register_driver);

static irqreturn_t usbIRQHandler(int irq, void* _dev)
{
	int retval;
	u32 status;
	struct iphone_udc *dev = _dev;
	int process = 0;
	int usb_speed;
        spin_lock(&dev->lock);
	status = __raw_readl(USB + GINTSTS) & __raw_readl(USB + GINTMSK);

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
			if(dev->usb_state < USBError) {
				printk("%s: reset detected\r\n", driver_name);
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
				struct usb_ctrlrequest* setupPacket = (struct usb_ctrlrequest*) ep0controlRecvBuffer;
				if(dev->gadget.speed == USB_SPEED_UNKNOWN)
				{
					usb_speed = DSTS_GET_SPEED(__raw_readl(USB + DSTS));
					switch(usb_speed) {
						case USB_HIGHSPEED:
							printk("USB speed detected as HIGH\n");
							dev->gadget.speed = USB_SPEED_HIGH;
							break;
						case USB_FULLSPEED:
						case USB_FULLSPEED_48_MHZ:
							printk("USB speed detected as FULL\n");
							dev->gadget.speed = USB_SPEED_FULL;
							break;
						case USB_LOWSPEED:
							printk("USB speed detected as LOW\n");
							dev->gadget.speed = USB_SPEED_LOW;
							break;
					}
				}

				nuke(&dev->ep[0], -EPROTO);

				if(setupPacket->bRequest == USB_SET_ADDRESS)
				{
					usb_speed = DSTS_GET_SPEED(__raw_readl(USB + DSTS));
					switch(usb_speed) {
						case USB_HIGHSPEED:
							printk("USB speed detected as HIGH\n");
							dev->gadget.speed = USB_SPEED_HIGH;
							break;
						case USB_FULLSPEED:
						case USB_FULLSPEED_48_MHZ:
							printk("USB speed detected as FULL\n");
							dev->gadget.speed = USB_SPEED_FULL;
							break;
						case USB_LOWSPEED:
							printk("USB speed detected as LOW\n");
							dev->gadget.speed = USB_SPEED_LOW;
							break;
					}

					__raw_writel((__raw_readl(USB + DCFG) & ~DCFG_DEVICEADDRMSK)
							| ((setupPacket->wValue & DCFG_DEVICEADDR_UNSHIFTED_MASK) << DCFG_DEVICEADDR_SHIFT), USB + DCFG);

					// send an acknowledgement
					sendControl(ep0controlSendBuffer, 0);

					if(dev->usb_state < USBError) {
						change_state(USBAddress);
					}
				} else {
					/*if(setupPacket->bRequest == USB_SET_CONFIGURATION)
					{
						// send an acknowledgement
						sendControl(ep0controlSendBuffer, 0);
					}*/

					spin_unlock(&dev->lock);
					dev->driver->setup(&dev->gadget, setupPacket);
					spin_lock(&dev->lock);
				}

				// get the next SETUP packet
				receiveControl(ep0controlRecvBuffer, sizeof(struct usb_ctrlrequest));
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

static int iphone_udc_get_frame(struct usb_gadget *_gadget)
{
	return -EOPNOTSUPP;
}

static const struct usb_gadget_ops iphone_udc_ops = {
	.get_frame	= iphone_udc_get_frame,
	// no remote wakeup
	// not selfpowered
};

static void gadget_release(struct device *_dev)
{
	struct iphone_udc* dev = the_controller;

	kfree(dev);
}

static int iphone_udc_remove(struct platform_device *pdev)
{
	int retval = 0;

	return retval;
}

static int iphone_udc_setup(struct iphone_udc* dev)
{
        int retval = 0;
	int i;

	InEPRegs = (USBEPRegisters*)(USB + USB_INREGS);
	OutEPRegs = (USBEPRegisters*)(USB + USB_OUTREGS);

	change_state(USBStart);

	// Power on hardware
	iphone_power_ctrl(POWER_USB, 1);
	mdelay(USB_START_DELAYMS);

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

	return retval;
}

static int iphone_udc_probe(struct platform_device *pdev)
{
	int retval = 0;
	struct iphone_udc *dev;

	dev = kzalloc(sizeof(struct iphone_udc), GFP_KERNEL);
	if(dev == NULL) {
		retval = -ENOMEM;
		goto done;
	}

	spin_lock_init(&dev->lock);
	dev->pdev = pdev;
	dev->gadget.ops = &iphone_udc_ops;

	device_initialize(&dev->gadget.dev);
	dev_set_name(&dev->gadget.dev, "gadget");
	dev->gadget.dev.parent = &pdev->dev;
	dev->gadget.dev.dma_mask = pdev->dev.dma_mask;
	dev->gadget.dev.release = gadget_release;
	dev->gadget.name = driver_name;
	dev->gadget.is_dualspeed = 1;

	the_controller = dev;

	retval = iphone_udc_setup(dev);
	if(retval == 0)
	{
		printk("%s: controller probed\n", driver_name);
		return 0;
	} else
	{
		printk("%s: controller probe failed!\n", driver_name);
	}

done:
	if(dev)
		iphone_udc_remove(pdev);

	return retval;
}

static struct platform_driver iphone_udc_driver = {
        .probe          = iphone_udc_probe,
        .remove         = iphone_udc_remove,
        .suspend        = NULL,
        .resume         = NULL,
        .driver         = {
                .owner  = THIS_MODULE,
                .name   = driver_name,
        },
};

static struct platform_device* iphone_udc_device;

static int __init iphone_udc_init(void)
{
	int ret;

	ret = platform_driver_register(&iphone_udc_driver);
	if(!ret) {
		iphone_udc_device = platform_device_register_simple("iphoneudc", 0,
				NULL, 0);

		if (IS_ERR(iphone_udc_device)) {
			platform_driver_unregister(&iphone_udc_driver);
			ret = PTR_ERR(iphone_udc_device);
			printk("%s: Error loading version %s\n", driver_name, DRIVER_VERSION);
		} else {
			printk("%s: loaded version %s\n", driver_name, DRIVER_VERSION);
		}
	}

	return ret;
}

static void __exit iphone_udc_exit(void)
{
        platform_device_unregister(iphone_udc_device);
        platform_driver_unregister(&iphone_udc_driver);
        printk("Unloaded %s version %s\n", driver_name, DRIVER_VERSION);
}

module_init(iphone_udc_init);
module_exit(iphone_udc_exit);

MODULE_LICENSE("GPL");
