#include <linux/init.h>
#include <linux/module.h>
#include <asm/irq.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <asm/cacheflush.h>
#include <linux/delay.h>
#include <linux/sched.h>

#include <mach/hardware.h>
#include <mach/iphone-dma.h>
#include <mach/dma.h>
#include <mach/iphone-clock.h>

#define GET_BITS(x, start, length) ((((u32)(x)) << (32 - ((start) + (length)))) >> (32 - (length)))

static const int ControllerLookupTable[] = {
	1, 1, 1, 1, 1,
	1, 1, 1, 1, 1,
	2, 2, 2, 2, 2,
	2, 2, 2, 2, 3,
	3, 3, 3, 3, 3,
	3
};

static const u32 AddressLookupTable[] = {
	0x3CC00024, 0x3CC00020, 0x3CC08024, 0x3CC08020, 0x3CC0C024,
	0x3CC0C020, 0x3CC10024, 0x3CC10020, 0x38A00080, 0x38300040,
	0x3CE00020, 0x3CE00010, 0x3D200020, 0x3D200010, 0x3CD00038,
	0x3CD00010, 0x3D400038, 0x3D400010, 0x3CB00010, 0x3CA00038,
	0x3CA00010, 0x3C300020, 0x3C300010, 0x3CC04024, 0x3CC04020,
	0
};

static const u32 PeripheralLookupTable[][IPHONE_DMA_NUMCONTROLLERS] = {
	{0x07, 0x10}, {0x06, 0x10}, {0x0B, 0x10}, {0x0A, 0x10}, {0x0D, 0x10},
	{0x0C, 0x10}, {0x0F, 0x10}, {0x0E, 0x10}, {0x02, 0x10}, {0x03, 0x10},
	{0x10, 0x0D}, {0x10, 0x0C}, {0x10, 0x0F}, {0x10, 0x0E}, {0x10, 0x03},
	{0x10, 0x02}, {0x10, 0x05}, {0x10, 0x04}, {0x10, 0x06}, {0x01, 0x01},
	{0x00, 0x00}, {0x05, 0x0B}, {0x04, 0x0A}, {0x09, 0x09}, {0x08, 0x08},
	{0x00, 0x00}
};

static volatile DMARequest requests[IPHONE_DMA_NUMCONTROLLERS][IPHONE_DMA_NUMCHANNELS];
static DMALinkedList* DMALists[IPHONE_DMA_NUMCONTROLLERS][IPHONE_DMA_NUMCHANNELS];
static dma_addr_t DMAListsPhys[IPHONE_DMA_NUMCONTROLLERS][IPHONE_DMA_NUMCHANNELS];
static size_t DMAListsSize[IPHONE_DMA_NUMCONTROLLERS][IPHONE_DMA_NUMCHANNELS];

static void dispatchRequest(volatile DMARequest *request);

static irqreturn_t dmaIRQHandler(int irq, void* pController);

static volatile int Controller0FreeChannels[IPHONE_DMA_NUMCHANNELS] = {0};
static volatile int Controller1FreeChannels[IPHONE_DMA_NUMCHANNELS] = {0};
static spinlock_t freeChannelsLock;

static struct device* dma_dev;

int iphone_dma_setup(void) {
	int ret;

	iphone_clock_gate_switch(DMAC0_CLOCKGATE, 1);
	iphone_clock_gate_switch(DMAC1_CLOCKGATE, 1);

	memset(requests, 0, sizeof(requests));

        ret = request_irq(DMAC0_INTERRUPT, dmaIRQHandler, IRQF_DISABLED, "iphone_dma", (void*) 1);
        ret = request_irq(DMAC0_INTERRUPT, dmaIRQHandler, IRQF_DISABLED, "iphone_dma", (void*) 2);

	spin_lock_init(&freeChannelsLock);

	return ret;
}

static DMALinkedList* get_new_linked_list(int controller, int channel, size_t size, dma_addr_t* phys)
{
	if(DMALists[controller][channel])
	{
		if(DMAListsSize[controller][channel] == size)
		{
			*phys = DMAListsPhys[controller][channel];
			return DMALists[controller][channel];
		}

		dma_free_writecombine(dma_dev, size, DMALists[controller][channel], DMAListsPhys[controller][channel]);
	}

	DMALists[controller][channel] = dma_alloc_writecombine(dma_dev, size, &DMAListsPhys[controller][channel], GFP_KERNEL);
	*phys = DMAListsPhys[controller][channel];
	return DMALists[controller][channel];
}

static irqreturn_t dmaIRQHandler(int irq, void* pController) {
	u32 intTCStatusReg;
	u32 intTCClearReg;
	u32 intTCStatus;
	int channel;
	u32 controller = (u32)pController;

	if(controller == 1) {
		intTCStatusReg = DMAC0 + DMACIntTCStatus;
		intTCClearReg = DMAC0 + DMACIntTCClear;
	} else {
		intTCStatusReg = DMAC1 + DMACIntTCStatus;
		intTCClearReg = DMAC1 + DMACIntTCClear;
	}

	intTCStatus = __raw_readl(intTCStatusReg);

	for(channel = 0; channel < IPHONE_DMA_NUMCHANNELS; channel++) {
		if((intTCStatus & (1 << channel)) != 0) {
			dispatchRequest(&requests[controller - 1][channel]);
			__raw_writel(1 << channel, intTCClearReg);
		}
		
	}

	return IRQ_HANDLED;	
}

static int getFreeChannel(int* controller, int* channel) {
	int i;
	unsigned long flags;

	spin_lock_irqsave(&freeChannelsLock, flags);
	if((*controller & (1 << 0)) != 0) {
		for(i = 0; i < IPHONE_DMA_NUMCHANNELS; i++) {
			if(Controller0FreeChannels[i] == 0) {
				*controller = 1;
				*channel = i;
				Controller0FreeChannels[i] = 1;
				spin_unlock_irqrestore(&freeChannelsLock, flags);
				return 0;
			}
		}
	}

	if((*controller & (1 << 1)) != 0) {
		for(i = 0; i < IPHONE_DMA_NUMCHANNELS; i++) {
			if(Controller1FreeChannels[i] == 0) {
				*controller = 2;
				*channel = i;
				Controller1FreeChannels[i] = 1;
				spin_unlock_irqrestore(&freeChannelsLock, flags);
				return 0;
			}
		}
	}
	spin_unlock_irqrestore(&freeChannelsLock, flags);

	return ERROR_BUSY;
}

int iphone_dma_request(int Source, int SourceTransferWidth, int SourceBurstSize, int Destination, int DestinationTransferWidth, int DestinationBurstSize, int* controller, int* channel) {
	u32 DMACControl;
	u32 DMACConfig;
	u32 config;
	int x;

	if(*controller == 0) {
		*controller = ControllerLookupTable[Source] & ControllerLookupTable[Destination];

		if(*controller == 0) {
			return ERROR_DMA;
		}

		while(getFreeChannel(controller, channel) == ERROR_BUSY);
		requests[*controller - 1][*channel].started = true;
		requests[*controller - 1][*channel].done = false;
	}

	if(*controller == 1) {
		DMACControl = DMAC0 + DMAC0Control0 + (*channel * DMAChannelRegSize);
		DMACConfig = DMAC0 + DMACConfiguration;
	} else if(*controller == 2) {
		DMACControl = DMAC1 + DMAC0Control0 + (*channel * DMAChannelRegSize);
		DMACConfig = DMAC1 + DMACConfiguration;
	} else {
		return ERROR_DMA;
	}

	config = 0;

	__raw_writel(DMACConfiguration_ENABLE, DMACConfig);

	config |= (SourceTransferWidth / 2) << DMAC0Control0_SWIDTHSHIFT; // 4 -> 2, 2 -> 1, 1 -> 0
	config |= (DestinationTransferWidth / 2) << DMAC0Control0_DWIDTHSHIFT; // 4 -> 2, 2 -> 1, 1 -> 0

	for(x = 0; x < 7; x++) {
		if((1 << (x + 1)) >= SourceBurstSize) {
			config |= x << DMAC0Control0_SBSIZESHIFT;
			break;
		}
	}
	for(x = 0; x < 7; x++) {
		if((1 << (x + 1)) >= DestinationBurstSize) {
			config |= x << DMAC0Control0_DBSIZESHIFT;
			break;
		}
	}

	config |= 1 << DMAC0Control0_TERMINALCOUNTINTERRUPTENABLE;
	config |= 1 << DMAC0Control0_SOURCEAHBMASTERSELECT;

	__raw_writel(config, DMACControl);

	return 0;
}

dma_addr_t iphone_dma_dstpos(int controller, int channel) {
	u32 regOffset = channel * DMAChannelRegSize;

	if(controller == 1) {
		regOffset += DMAC0;
	} else if(controller == 2) {
		regOffset += DMAC1;
	}

	return __raw_readl(regOffset + DMAC0DestAddress);
}

dma_addr_t iphone_dma_srcpos(int controller, int channel) {
	u32 regOffset = channel * DMAChannelRegSize;

	if(controller == 1) {
		regOffset += DMAC0;
	} else if(controller == 2) {
		regOffset += DMAC1;
	}

	return __raw_readl(regOffset + DMAC0SrcAddress);
}

int iphone_dma_prepare(dma_addr_t Source, dma_addr_t Destination, int size, dma_addr_t continueList, int* controller, int* channel) {
	u32 regSrcAddress;
	u32 regDestAddress;
	u32 regLLI;
	u32 regControl0;
	u32 regConfiguration;
	int transfers;
	u32 sourcePeripheral = 0;
	u32 destPeripheral = 0;
	u32 flowControl = 0;
	u32 sourceIncrement = 0;
	u32 destinationIncrement = 0;

	const u32 regControl0Mask = ~(DMAC0Control0_SIZEMASK | DMAC0Control0_SOURCEINCREMENT | DMAC0Control0_DESTINATIONINCREMENT);

	u32 regOffset = (*channel * DMAChannelRegSize);

	if(*controller == 1) {
		regOffset += DMAC0;
	} else if(*controller == 2) {
		regOffset += DMAC1;
	}

	regSrcAddress = regOffset + DMAC0SrcAddress;
	regDestAddress = regOffset + DMAC0DestAddress;
	regLLI = regOffset + DMAC0LLI;
	regControl0 = regOffset + DMAC0Control0;
	regConfiguration = regOffset + DMAC0Configuration;

	transfers = size/(1 << DMAC0Control0_DWIDTH(__raw_readl(regControl0)));

	if(Source <= (sizeof(AddressLookupTable)/sizeof(u32))) {
		if(Destination <= (sizeof(AddressLookupTable)/sizeof(u32))) {
			__raw_writel(AddressLookupTable[Source], regSrcAddress);
			__raw_writel(AddressLookupTable[Destination], regDestAddress);
			sourcePeripheral = PeripheralLookupTable[Source][*controller - 1];
			destPeripheral = PeripheralLookupTable[Destination][*controller - 1];
			flowControl =  DMAC0Configuration_FLOWCNTRL_P2P;
		} else {
			__raw_writel(AddressLookupTable[Source], regSrcAddress);
			__raw_writel(Destination, regDestAddress);
			sourcePeripheral = PeripheralLookupTable[Source][*controller - 1];
			destPeripheral = PeripheralLookupTable[IPHONE_DMA_MEMORY][*controller - 1];
			flowControl =  DMAC0Configuration_FLOWCNTRL_P2M;
			destinationIncrement = 1 << DMAC0Control0_DESTINATIONINCREMENT;
		}
	} else {
		if(Destination <= (sizeof(AddressLookupTable)/sizeof(u32))) {
			__raw_writel(Source, regSrcAddress);
			__raw_writel(AddressLookupTable[Destination], regDestAddress);
			sourcePeripheral = PeripheralLookupTable[IPHONE_DMA_MEMORY][*controller - 1];
			destPeripheral = PeripheralLookupTable[Destination][*controller - 1];
			flowControl =  DMAC0Configuration_FLOWCNTRL_M2P;
			sourceIncrement = 1 << DMAC0Control0_SOURCEINCREMENT;
		} else {
			__raw_writel(Source, regSrcAddress);
			__raw_writel(Destination, regDestAddress);
			sourcePeripheral = PeripheralLookupTable[IPHONE_DMA_MEMORY][*controller - 1];
			destPeripheral = PeripheralLookupTable[IPHONE_DMA_MEMORY][*controller - 1];
			flowControl =  DMAC0Configuration_FLOWCNTRL_M2M;
			sourceIncrement = 1 << DMAC0Control0_SOURCEINCREMENT;
			destinationIncrement = 1 << DMAC0Control0_DESTINATIONINCREMENT;
		}
	}

	if(!continueList) {
		u32 src = __raw_readl(regSrcAddress);
		u32 dest = __raw_readl(regDestAddress);
		if(transfers > 0xFFF) {
			DMALinkedList* item;
			dma_addr_t phys;

			__raw_writel(__raw_readl(regControl0) & ~(1 << DMAC0Control0_TERMINALCOUNTINTERRUPTENABLE), regControl0);

			if(DMALists[*controller - 1][*channel])
				kfree(DMALists[*controller - 1][*channel]);

			item = get_new_linked_list(*controller - 1, *channel,
					((transfers + 0xDFF) / 0xE00) * sizeof(DMALinkedList), &phys);
			
			__raw_writel(phys, regLLI);

			do {
				transfers -= 0xE00;
				
				if(sourceIncrement != 0) {
					src += 0xE00 << DMAC0Control0_DWIDTH(__raw_readl(regControl0));
				}
				if(destinationIncrement != 0) {
					dest += 0xE00 << DMAC0Control0_DWIDTH(__raw_readl(regControl0));
				}

				item->source = src;
				item->destination = dest;

				if(transfers <= 0xE00) {
					item->control = destinationIncrement | sourceIncrement | (regControl0Mask & __raw_readl(regControl0)) | (transfers & DMAC0Control0_SIZEMASK)
						| (1 << DMAC0Control0_TERMINALCOUNTINTERRUPTENABLE);
					item->next = NULL;
				} else {
					item->control = destinationIncrement | sourceIncrement | (regControl0Mask & __raw_readl(regControl0)) | 0xE00;
					item->next = item + 1;
					item = item->next;
				}
			} while(transfers > 0xE00);

			flush_cache_all();
			transfers = 0xE00;
		} else {
			__raw_writel(0, regLLI);
		}
	} else {
		__raw_writel(continueList, regLLI);
	}

	__raw_writel((__raw_readl(regControl0) & regControl0Mask) | destinationIncrement | sourceIncrement | (transfers & DMAC0Control0_SIZEMASK), regControl0);
	__raw_writel(DMAC0Configuration_TERMINALCOUNTINTERRUPTMASK
			| (flowControl << DMAC0Configuration_FLOWCNTRLSHIFT)
			| (destPeripheral << DMAC0Configuration_DESTPERIPHERALSHIFT)
			| (sourcePeripheral << DMAC0Configuration_SRCPERIPHERALSHIFT), regConfiguration);

	return 0;
}

int iphone_dma_perform(dma_addr_t Source, dma_addr_t Destination, int size, dma_addr_t continueList, int* controller, int* channel) {
	int ret;

	ret = iphone_dma_prepare(Source, Destination, size, continueList, controller, channel);

	if(ret != 0)
		return ret;

	iphone_dma_resume(*controller, *channel);
	return 0;
}

void iphone_dma_pause(int controller, int channel) {
	u32 regOffset = channel * DMAChannelRegSize;

	if(controller == 1) {
		regOffset += DMAC0;
	} else if(controller == 2) {
		regOffset += DMAC1;
	}

	__raw_writel(__raw_readl(regOffset + DMAC0Configuration) & ~DMAC0Configuration_CHANNELENABLED,
			regOffset + DMAC0Configuration);
}

void iphone_dma_resume(int controller, int channel) {
	u32 regOffset = channel * DMAChannelRegSize;

	if(controller == 1) {
		regOffset += DMAC0;
	} else if(controller == 2) {
		regOffset += DMAC1;
	}

	__raw_writel(__raw_readl(regOffset + DMAC0Configuration) | DMAC0Configuration_CHANNELENABLED,
			regOffset + DMAC0Configuration);
}

int iphone_dma_finish(int controller, int channel, int timeout) {
	unsigned long flags;
	int ret = 0;
	unsigned long timeout_jiffies;

	timeout_jiffies = msecs_to_jiffies(timeout) + 1;
	requests[controller - 1][channel].task = current;

	set_current_state(TASK_INTERRUPTIBLE);
	if(!requests[controller - 1][channel].done)
		timeout_jiffies = schedule_timeout(timeout_jiffies);
	set_current_state(TASK_RUNNING);

	if(!requests[controller - 1][channel].done)
		ret = -1;

	requests[controller - 1][channel].started = false;
	requests[controller - 1][channel].done = false;
	requests[controller - 1][channel].task = NULL;

	spin_lock_irqsave(&freeChannelsLock, flags);
	if(controller == 1)
		Controller0FreeChannels[channel] = 0;
	else if(controller == 2)
		Controller1FreeChannels[channel] = 0;
	spin_unlock_irqrestore(&freeChannelsLock, flags);

	return ret;
}

static void dispatchRequest(volatile DMARequest *request) {
	// TODO: Implement this
	request->done = true;

	if(request->task)
	{
		wake_up_process(request->task);
	}
}

static int __devinit iphone_dma_probe(struct platform_device *pdev)
{
	dma_dev = &pdev->dev;
	return 0;
}

static int __devexit iphone_dma_remove(struct platform_device *pdev)
{
	return 0;
}

static struct resource iphone_dma_resources[] = {
	[0] = {
		.start  = DMAC0_PA,
		.end    = DMAC0_PA + 0x1000 - 1,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = DMAC1_PA,
		.end    = DMAC1_PA + 0x1000 - 1,
		.flags  = IORESOURCE_MEM,
	},
	[2] = {
		.start  = DMAC0_INTERRUPT,
		.end    = DMAC1_INTERRUPT,
		.flags  = IORESOURCE_IRQ,
	},
};

struct platform_device iphone_dma = {
	.name           = "iphone-dma",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(iphone_dma_resources),
	.resource       = iphone_dma_resources,
};

static struct platform_driver iphone_dma_driver = {
	.driver         = {
		.name   = "iphone-dma",
		.owner  = THIS_MODULE,
	},
	.probe          = iphone_dma_probe,
	.remove         = __devexit_p(iphone_dma_remove),
	.suspend        = NULL,
	.resume         = NULL,
};

static int __init iphone_dma_modinit(void)
{
	return platform_driver_register(&iphone_dma_driver);
}

static void __exit iphone_dma_modexit(void)
{
	platform_driver_unregister(&iphone_dma_driver);
}

module_init(iphone_dma_modinit);
module_exit(iphone_dma_modexit);

MODULE_DESCRIPTION("iPhone DMA Controller");
MODULE_LICENSE("GPL");
