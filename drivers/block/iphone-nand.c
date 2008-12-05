#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/stat.h>
#include <linux/errno.h>
#include <linux/major.h>
#include <linux/wait.h>
#include <linux/blkdev.h>
#include <linux/blkpg.h>
#include <linux/init.h>
#include <linux/smp_lock.h>
#include <linux/swap.h>
#include <linux/slab.h>
#include <linux/loop.h>
#include <linux/compat.h>
#include <linux/suspend.h>
#include <linux/freezer.h>
#include <linux/writeback.h>
#include <linux/buffer_head.h>		/* for invalidate_bdev() */
#include <linux/completion.h>
#include <linux/highmem.h>
#include <linux/gfp.h>
#include <linux/kthread.h>
#include <linux/splice.h>
#include <linux/vmalloc.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/hdreg.h>
#include <linux/platform_device.h>

#include <mach/hardware.h>
#include <asm/uaccess.h>
#include <mach/nand.h>
#include "iphone-nand.h"
#include "iphone-ftl.h"
#include <mach/iphone-clock.h>
#include <mach/iphone-dma.h>

#define DebugPrintf(...)
//#define DebugPrintf printk
#define SECTOR_SHIFT		9
#define has_elapsed(startTime, timeout) ((iphone_microtime() - (startTime)) > (timeout))

#define driver_name "iphonenand"
#define DRIVER_VERSION          __DATE__

/*
 * NAND SECTION
 *
 *
 *
 */

int HasNANDInit = 0;

static int banksTable[NAND_NUM_BANKS];

static int ECCType = 0;
static u8 NANDSetting1;
static u8 NANDSetting2;
static u8 NANDSetting3;
static u8 NANDSetting4;
static u32 TotalECCDataSize;
static u32 ECCType2;
static int NumValidBanks = 0;
static const int NANDBankResetSetting = 1;
static int LargePages;

static NANDData Data;
static UnknownNANDType Data2;

static u8* aTemporaryReadEccBuf;
static u8* aTemporarySBuf;

#define SECTOR_SIZE 512

static const NANDDeviceType SupportedDevices[] = {
	{0x2555D5EC, 8192, 0x80, 4, 64, 4, 2, 4, 2, 7744, 4, 6},
	{0xB614D5EC, 4096, 0x80, 8, 128, 4, 2, 4, 2, 3872, 4, 6},
	{0xB655D7EC, 8192, 0x80, 8, 128, 4, 2, 4, 2, 7744, 4, 6},
	{0xA514D3AD, 4096, 0x80, 4, 64, 4, 2, 4, 2, 3872, 4, 6},
	{0xA555D5AD, 8192, 0x80, 4, 64, 4, 2, 4, 2, 7744, 4, 6},
	{0xA585D598, 8320, 0x80, 4, 64, 6, 2, 4, 2, 7744, 4, 6},
	{0xBA94D598, 4096, 0x80, 8, 216, 6, 2, 4, 2, 3872, 8, 8},
	{0xBA95D798, 8192, 0x80, 8, 216, 6, 2, 4, 2, 7744, 8, 8},
	{0x3ED5D789, 8192, 0x80, 8, 216, 4, 2, 4, 2, 7744, 8, 8},
	{0x3E94D589, 4096, 0x80, 8, 216, 4, 2, 4, 2, 3872, 8, 8},
	{0x3ED5D72C, 8192, 0x80, 8, 216, 4, 2, 4, 2, 7744, 8, 8},
	{0x3E94D52C, 4096, 0x80, 8, 216, 4, 2, 4, 2, 3872, 8, 8},
	{0}
};

static int wait_for_ready(int timeout) {
	u64 startTime;

	if((__raw_readl(NAND + NAND_STATUS) & NAND_STATUS_READY) != 0) {
		return 0;
	}

	startTime = iphone_microtime();
	while((__raw_readl(NAND + NAND_STATUS) & NAND_STATUS_READY) == 0) {
		if(has_elapsed(startTime, timeout * 1000)) {
			return ERROR_TIMEOUT;
		}
	}

	return 0;
}

static int wait_for_status_bit_2(int timeout) {
	u64 startTime;

	if((__raw_readl(NAND + NAND_STATUS) & (1 << 2)) != 0) {
		__raw_writel(1 << 2, NAND + NAND_STATUS);
		return 0;
	}

	startTime = iphone_microtime();
	while((__raw_readl(NAND + NAND_STATUS) & (1 << 2)) == 0) {
		if(has_elapsed(startTime, timeout * 1000)) {
			return ERROR_TIMEOUT;
		}
	}

	__raw_writel(1 << 2, NAND + NAND_STATUS);

	return 0;
}

static int wait_for_status_bit_3(int timeout) {
	u64 startTime;

	if((__raw_readl(NAND + NAND_STATUS) & (1 << 3)) != 0) {
		__raw_writel(1 << 3, NAND + NAND_STATUS);
		return 0;
	}

	startTime = iphone_microtime();
	while((__raw_readl(NAND + NAND_STATUS) & (1 << 3)) == 0) {
		if(has_elapsed(startTime, timeout * 1000)) {
			return ERROR_TIMEOUT;
		}
	}

	__raw_writel(1 << 3, NAND + NAND_STATUS);

	return 0;
}

static int nand_bank_reset_helper(int bank, int timeout) {
	u64 startTime = iphone_microtime();
	u32 toTest;

	if(NANDBankResetSetting)
		bank = 0;
	else
		bank &= 0xffff;

	toTest = 1 << (bank + 4);

	while((__raw_readl(NAND + NAND_STATUS) & toTest) == 0) {
		if(has_elapsed(startTime, timeout * 1000)) {
			return ERROR_TIMEOUT;
		}
	}

	__raw_writel(toTest, NAND + NAND_STATUS);

	return 0;
}

static int nand_bank_reset(int bank, int timeout) {
	int ret;

	__raw_writel(((NANDSetting1 & NAND_CONFIG_SETTING1MASK) << NAND_CONFIG_SETTING1SHIFT) | ((NANDSetting2 & NAND_CONFIG_SETTING2MASK) << NAND_CONFIG_SETTING2SHIFT)
			| (1 << (banksTable[bank] + 1)) | NAND_CONFIG_DEFAULTS, NAND + NAND_CONFIG);

	__raw_writel(NAND_CMD_RESET, NAND + NAND_CMD);

	ret = wait_for_ready(timeout);
	if(ret == 0) {
		ret = nand_bank_reset_helper(bank, timeout);
		msleep(1);
		return ret;
	} else {
		msleep(1);
		return ret;
	}
}

static int bank_setup(int bank) {
	u32 toTest;
	u64 startTime;
	u32 data;

	__raw_writel(((NANDSetting1 & NAND_CONFIG_SETTING1MASK) << NAND_CONFIG_SETTING1SHIFT) | ((NANDSetting2 & NAND_CONFIG_SETTING2MASK) << NAND_CONFIG_SETTING2SHIFT)
			| (1 << (banksTable[bank] + 1)) | NAND_CONFIG_DEFAULTS, NAND + NAND_CONFIG);

	toTest = 1 << (bank + 4);
	if((__raw_readl(NAND + NAND_STATUS) & toTest) != 0) {
		__raw_writel(toTest, NAND + NAND_STATUS);
	}

	__raw_writel(NAND_CON_SETTING1, NAND + NAND_CON); 
	__raw_writel(NAND_CMD_SETTING2, NAND + NAND_CMD);
	wait_for_ready(500);

	startTime = iphone_microtime();
	while(1) {
		__raw_writel(0, NAND + NAND_TRANSFERSIZE);
		__raw_writel(NAND_CON_BEGINTRANSFER, NAND + NAND_CON);

		if(wait_for_status_bit_3(500) != 0) {
			printk("nand: bank_setup: wait for status bit 3 timed out\r\n");
			return ERROR_TIMEOUT;
		}


		data = __raw_readl(NAND + NAND_DMA_SOURCE);
		__raw_writel(NAND_CON_SETTING2, NAND + NAND_CON);
		if((data & (1 << 6)) == 0) {
			if(has_elapsed(startTime, 500 * 1000)) {
				printk("nand: bank_setup: wait for bit 6 of DMA timed out\r\n");
				return ERROR_TIMEOUT;
			}
		} else {
			break;
		}
	}

	__raw_writel(0, NAND + NAND_CMD);
	wait_for_ready(500);
	return 0;
}

static int nand_setup(void) {
	int bank;
	int i;
	int bits;
	const NANDDeviceType* nandType = NULL;
	const NANDDeviceType* candidate;

	if(HasNANDInit)
		return 0;

	NANDSetting1 = 7;
	NANDSetting2 = 7;
	NANDSetting3 = 7;
	NANDSetting4 = 7;

	printk("nand: Probing flash controller...\r\n");

	iphone_clock_gate_switch(NAND_CLOCK_GATE1, 1);
	iphone_clock_gate_switch(NAND_CLOCK_GATE2, 1);

	for(bank = 0; bank < NAND_NUM_BANKS; bank++) {
		banksTable[bank] = bank;
	}

	NumValidBanks = 0;

	__raw_writel(0, NAND + NAND_SETUP);
	__raw_writel(__raw_readl(NAND + NAND_SETUP) | (ECCType << 4), NAND + NAND_SETUP);

	for(bank = 0; bank < NAND_NUM_BANKS; bank++) {
		u32 id;

		nand_bank_reset(bank, 100);

		__raw_writel(NAND_CON_SETTING1, NAND + NAND_CON);
		__raw_writel(((NANDSetting1 & NAND_CONFIG_SETTING1MASK) << NAND_CONFIG_SETTING1SHIFT) | ((NANDSetting2 & NAND_CONFIG_SETTING2MASK) << NAND_CONFIG_SETTING2SHIFT)
				| (1 << (banksTable[bank] + 1)) | NAND_CONFIG_DEFAULTS, NAND + NAND_CONFIG);

		__raw_writel(NAND_CMD_ID, NAND + NAND_CMD);

		wait_for_ready(500);

		__raw_writel(0, NAND + NAND_CONFIG4);
		__raw_writel(0, NAND + NAND_CONFIG3);
		__raw_writel(NAND_CON_SETUPTRANSFER, NAND + NAND_CON);

		wait_for_status_bit_2(500);
		nand_bank_reset_helper(bank, 100);

		__raw_writel(8, NAND + NAND_TRANSFERSIZE);
		__raw_writel(NAND_CON_BEGINTRANSFER, NAND + NAND_CON);

		wait_for_status_bit_3(500);
		id = __raw_readl(NAND + NAND_DMA_SOURCE);
		candidate = SupportedDevices;
		while(candidate->id != 0) {
			if(candidate->id == id) {
				if(nandType == NULL) {
					nandType = candidate;
				} else if(nandType != candidate) {
					printk("nand: Mismatched device IDs (0x%08x after 0x%08x)\r\n", id, nandType->id);
					return ERROR_ARG;
				}
				banksTable[NumValidBanks++] = bank;
			}
			candidate++;
		}

		__raw_writel(NAND_CON_SETTING1, NAND + NAND_CON);
	}

	if(nandType == NULL) {
		printk("nand: No supported NAND found\r\n");
		return ERROR_ARG;
	}

	Data.DeviceID = nandType->id;

	NANDSetting2 = (((FREQUENCY_BUS * (nandType->NANDSetting2 + 1)) + 99999999)/100000000) - 1;
	NANDSetting1 = (((FREQUENCY_BUS * (nandType->NANDSetting1 + 1)) + 99999999)/100000000) - 1;
	NANDSetting3 = (((FREQUENCY_BUS * (nandType->NANDSetting3 + 1)) + 99999999)/100000000) - 1;
	NANDSetting4 = (((FREQUENCY_BUS * (nandType->NANDSetting4 + 1)) + 99999999)/100000000) - 1;

	if(NANDSetting2 > 7)
		NANDSetting2 = 7;

	if(NANDSetting1 > 7)
		NANDSetting1 = 7;

	if(NANDSetting3 > 7)
		NANDSetting3 = 7;

	if(NANDSetting4 > 7)
		NANDSetting4 = 7;

	Data.blocksPerBank = nandType->blocksPerBank;
	Data.banksTotal = NumValidBanks;
	Data.sectorsPerPage = nandType->sectorsPerPage;
	Data.userSubBlksTotal = nandType->userSubBlksTotal;
	Data.bytesPerSpare = nandType->bytesPerSpare;
	Data.field_2E = 4;
	Data.field_2F = 3;
	Data.pagesPerBlock = nandType->pagesPerBlock;

	if(Data.sectorsPerPage > 4) {
		LargePages = 1;
	} else {
		LargePages = 0;
	}

	if(nandType->ecc1 == 6) {
		ECCType = 4;
		TotalECCDataSize = Data.sectorsPerPage * 15;
	} else if(nandType->ecc1 == 8) {
		ECCType = 8;
		TotalECCDataSize = Data.sectorsPerPage * 20;
	} else if(nandType->ecc1 == 4) {
		ECCType = 0;
		TotalECCDataSize = Data.sectorsPerPage * 10;
	}

	if(nandType->ecc2 == 6) {
		ECCType2 = 4;
	} else if(nandType->ecc2 == 8) {
		ECCType2 = 8;
	} else if(nandType->ecc2 == 4) {
		ECCType2 = 0;
	}

	Data.field_4 = 5;
	Data.bytesPerPage = SECTOR_SIZE * Data.sectorsPerPage;
	Data.pagesPerBank = Data.pagesPerBlock * Data.blocksPerBank;
	Data.pagesTotal = Data.pagesPerBank * Data.banksTotal;
	Data.pagesPerSubBlk = Data.pagesPerBlock * Data.banksTotal;
	Data.userPagesTotal = Data.userSubBlksTotal * Data.pagesPerSubBlk;
	Data.subBlksTotal = (Data.banksTotal * Data.blocksPerBank) / Data.banksTotal;

	Data2.field_2 = Data.subBlksTotal - Data.userSubBlksTotal - 28;
	Data2.field_0 = Data2.field_2 + 4;
	Data2.field_4 = Data2.field_2 + 5;
	Data2.field_6 = 3;
	Data2.field_8 = 23;
	if(Data2.field_8 == 0)
		Data.field_22 = 0;

	bits = 0;
	i = Data2.field_8;
	while((i <<= 1) != 0) {
		bits++;
	}

	Data.field_22 = bits;

	printk("nand: DEVICE: %08x\r\n", Data.DeviceID);
	printk("nand: BANKS_TOTAL: %d\r\n", Data.banksTotal);
	printk("nand: BLOCKS_PER_BANK: %d\r\n", Data.blocksPerBank);
	printk("nand: SUBLKS_TOTAL: %d\r\n", Data.subBlksTotal);
	printk("nand: USER_SUBLKS_TOTAL: %d\r\n", Data.userSubBlksTotal);
	printk("nand: PAGES_PER_SUBLK: %d\r\n", Data.pagesPerSubBlk);
	printk("nand: PAGES_PER_BANK: %d\r\n", Data.pagesPerBank);
	printk("nand: SECTORS_PER_PAGE: %d\r\n", Data.sectorsPerPage);
	printk("nand: BYTES_PER_SPARE: %d\r\n", Data.bytesPerSpare);
	printk("nand: BYTES_PER_PAGE: %d\r\n", Data.bytesPerPage);

	aTemporaryReadEccBuf = (u8*) kmalloc(Data.bytesPerPage, GFP_KERNEL);
	memset(aTemporaryReadEccBuf, 0xFF, Data.bytesPerPage);

	aTemporarySBuf = (u8*) kmalloc(Data.bytesPerSpare, GFP_KERNEL);

	HasNANDInit = 1;

	return 0;
}

static int transferFromFlash(void* buffer, int size) {
	int controller = 0;
	int channel = 0;

	if((((u32)buffer) & 0x3) != 0) {
		// the buffer needs to be aligned for DMA, last two bits have to be clear
		return ERROR_ALIGN;
	}

	__raw_writel(__raw_readl(NAND + NAND_CONFIG) | (1 << NAND_CONFIG_DMASETTINGSHIFT), NAND + NAND_CONFIG);
	__raw_writel(size - 1, NAND + NAND_TRANSFERSIZE);
	__raw_writel(NAND_CON_BEGINTRANSFER, NAND + NAND_CON);

	dmac_clean_range(buffer, (u8*)buffer + size);

	iphone_dma_request(IPHONE_DMA_NAND, 4, 4, IPHONE_DMA_MEMORY, 4, 4, &controller, &channel);
	iphone_dma_perform(IPHONE_DMA_NAND, virt_to_phys(buffer), size, 0, &controller, &channel);

	if(iphone_dma_finish(controller, channel, 500) != 0) {
		printk("nand: dma timed out\r\n");
		return ERROR_TIMEOUT;
	}

	if(wait_for_status_bit_3(500) != 0) {
		printk("nand: waiting for status bit 3 timed out\r\n");
		return ERROR_TIMEOUT;
	}

	__raw_writel(NAND_CON_SETTING1, NAND + NAND_CON);

	dmac_flush_range(buffer, (u8*)buffer + size);

	return 0;
}

static void ecc_perform(int setting, int sectors, u8* sectorData, u8* eccData) {
	dmac_flush_range(sectorData, (u8*)sectorData + (sectors * SECTOR_SIZE));
	dmac_flush_range(eccData, (u8*)eccData + (sectors * 20));

	__raw_writel(1, NANDECC + NANDECC_CLEARINT);
	__raw_writel(((sectors - 1) & 0x3) | setting, NANDECC + NANDECC_SETUP);
	__raw_writel(virt_to_phys(sectorData), NANDECC + NANDECC_DATA);
	__raw_writel(virt_to_phys(eccData), NANDECC + NANDECC_ECC);

	dmac_clean_range(sectorData, (u8*)sectorData + (sectors * SECTOR_SIZE));
	dmac_clean_range(eccData, (u8*)eccData + (sectors * 20));

	__raw_writel(1, NANDECC + NANDECC_START);
}

#define VIC1 IO_ADDRESS(0x38E01000)
#define VICRAWINTR 0x8
#define VIC_InterruptSeparator 0x20
static int wait_for_ecc_interrupt(int timeout) {
	u64 startTime = iphone_microtime();
	u32 mask = (1 << (NANDECC_INT - VIC_InterruptSeparator));
	while((__raw_readl(VIC1 + VICRAWINTR) & mask) == 0) {
		if(has_elapsed(startTime, timeout * 1000)) {
			return ERROR_TIMEOUT;
		}
	}

	__raw_writel(1, NANDECC + NANDECC_CLEARINT);

	if((__raw_readl(VIC1 + VICRAWINTR) & mask) == 0) {
		return 0;
	} else {
		return ERROR_TIMEOUT;
	}
}

static int ecc_finish(void) {
	int ret;
	if((ret = wait_for_ecc_interrupt(500)) != 0)
		return ret;

	if((__raw_readl(NANDECC + NANDECC_STATUS) & 0x1) != 0)
		return ERROR_ECC;

	return 0;
}

static int checkECC(int setting, u8* data, u8* ecc) {
	int eccSize = 0;
	u8* dataPtr = data;
	u8* eccPtr = ecc;
	int sectorsLeft = Data.sectorsPerPage;

	if(setting == 4) {
		eccSize = 15;
	} else if(setting == 8) {
		eccSize = 20;
	} else if(setting == 0) {
		eccSize = 10;
	} else {
		return ERROR_ECC;
	}

	while(sectorsLeft > 0) {
		int toCheck;
		if(sectorsLeft > 4)
			toCheck = 4;
		else
			toCheck = sectorsLeft;

		if(LargePages) {
			// If there are more than 4 sectors in a page...
			int i;
			for(i = 0; i < toCheck; i++) {
				// loop through each sector that we have to check this time's ECC
				u8* x = &eccPtr[eccSize * i]; // first byte of ECC
				u8* y = x + eccSize - 1; // last byte of ECC
				while(x < y) {
					// swap the byte order of them
					u8 t = *y;
					*y = *x;
					*x = t;
					x++;
					y--;
				}
			}
		}

		ecc_perform(setting, toCheck, dataPtr, eccPtr);
		if(ecc_finish() != 0)
			return ERROR_ECC;

		dataPtr += toCheck * SECTOR_SIZE;
		eccPtr += toCheck * eccSize;
		sectorsLeft -= toCheck;
	}

	return 0;
}

static int isEmptyBlock(u8* buffer, int size) {
	int i;
	int found = 0;
	for(i = 0; i < size; i++) {
		if(buffer[i] != 0xFF) {
			found++;
		}
	}

	if(found <= 1)
		return 1;
	else
		return 0;
}

static int nand_read(int bank, int page, u8* buffer, u8* spare, int doECC, int checkBadBlocks) {
	int eccFailed;

	if(bank >= Data.banksTotal)
		return ERROR_ARG;

	if(page >= Data.pagesPerBank)
		return ERROR_ARG;

	if(buffer == NULL && spare == NULL)
		return ERROR_ARG;

	__raw_writel(((NANDSetting1 & NAND_CONFIG_SETTING1MASK) << NAND_CONFIG_SETTING1SHIFT) | ((NANDSetting2 & NAND_CONFIG_SETTING2MASK) << NAND_CONFIG_SETTING2SHIFT)
		| (1 << (banksTable[bank] + 1)) | NAND_CONFIG_DEFAULTS, NAND + NAND_CONFIG);

	__raw_writel(0, NAND + NAND_CMD);
	if(wait_for_ready(500) != 0) {
		printk("nand: bank setting failed\r\n");
		goto FIL_read_error;
	}

	__raw_writel(NAND_CONFIG4_TRANSFERSETTING, NAND + NAND_CONFIG4);

	if(buffer) {
		__raw_writel(page << 16, NAND + NAND_CONFIG3); // lower bits of the page number to the upper bits of CONFIG3
		__raw_writel((page >> 16) & 0xFF, NAND + NAND_CONFIG5); // upper bits of the page number

	} else {
		__raw_writel((page << 16) | Data.bytesPerPage, NAND + NAND_CONFIG3); // lower bits of the page number to the upper bits of CONFIG3
		__raw_writel((page >> 16) & 0xFF, NAND + NAND_CONFIG5); // upper bits of the page number	
	}

	__raw_writel(NAND_CON_SETUPTRANSFER, NAND + NAND_CON);
	if(wait_for_status_bit_2(500) != 0) {
		printk("nand: setup transfer failed\r\n");
		goto FIL_read_error;
	}
	
	__raw_writel(NAND_CMD_READ, NAND + NAND_CMD);
	if(wait_for_ready(500) != 0) {
		printk("nand: setting config2 failed\r\n");
		goto FIL_read_error;
	}

	if(bank_setup(bank) != 0) {
		printk("nand: bank setup failed\r\n");
		goto FIL_read_error;
	}

	if(buffer) {
		if(transferFromFlash(aTemporaryReadEccBuf, Data.bytesPerPage) != 0) {
			goto FIL_read_error;
		}
		memcpy(buffer, aTemporaryReadEccBuf, Data.bytesPerPage);
	}

	if(transferFromFlash(aTemporarySBuf, Data.bytesPerSpare) != 0) {
		goto FIL_read_error;
	}

	eccFailed = 0;
	if(doECC) {
		if(buffer) {
			eccFailed = (checkECC(ECCType, aTemporaryReadEccBuf, aTemporarySBuf + sizeof(SpareData)) != 0);
		}

		memset(aTemporaryReadEccBuf, 0xFF, SECTOR_SIZE);
		memcpy(aTemporaryReadEccBuf, aTemporarySBuf, sizeof(SpareData));
		ecc_perform(ECCType, 1, aTemporaryReadEccBuf, aTemporarySBuf + sizeof(SpareData) + TotalECCDataSize);
		if(ecc_finish() != 0) {
			eccFailed |= 1;
		}
	}

	if(spare) {
		if(doECC) {
			// We can only copy the first 12 bytes because the rest is probably changed by the ECC check routine
			memcpy(spare, aTemporaryReadEccBuf, sizeof(SpareData));
		} else {
			memcpy(spare, aTemporarySBuf, Data.bytesPerSpare);
		}
	}

	if(eccFailed || checkBadBlocks) {
		if(isEmptyBlock(aTemporarySBuf, Data.bytesPerSpare) != 0) {
			return ERROR_EMPTYBLOCK;
		} else if(eccFailed) {
			return ERROR_NAND;
		}
	}

	return 0;

FIL_read_error:
	printk("nand: some sort of read error\n");
	nand_bank_reset(bank, 100);
	return ERROR_NAND;
}

static int nand_read_multiple(u16* bank, u32* pages, u8* main, SpareData* spare, int pagesCount) {
	int i;
	unsigned int ret;
	for(i = 0; i < pagesCount; i++) {
		ret = nand_read(bank[i], pages[i], main, (u8*) &spare[i], 1, 1);
		if(ret > 1)
			return ret;

		main += Data.bytesPerPage;
	}

	return 0;
}

static int nand_read_alternate_ecc(int bank, int page, u8* buffer) {
	int ret;
	if((ret = nand_read(bank, page, buffer, aTemporarySBuf, 0, 1)) != 0) {
		DebugPrintf("nand: Raw read failed.\r\n");
		return ret;
	}

	memcpy(aTemporaryReadEccBuf, buffer, Data.bytesPerPage);
	if(checkECC(ECCType2, aTemporaryReadEccBuf, aTemporarySBuf) != 0) {
		DebugPrintf("nand: Alternate ECC check failed, but raw read succeeded.\r\n");
		return ERROR_NAND;
	}

	return 0;
}

/*
 * FTL SECTION
 *
 *
 *
 */

#define FTL_ID_V1 0x43303033
#define FTL_ID_V2 0x43303034

int HasFTLInit = 0;

static int findDeviceInfoBBT(int bank, void* deviceInfoBBT) {
	u8* buffer = vmalloc(Data.bytesPerPage);
	int lowestBlock = Data.blocksPerBank - (Data.blocksPerBank / 10);
	int block;
	int ret;
	for(block = Data.blocksPerBank - 1; block >= lowestBlock; block--) {
		int page;
		int badBlockCount = 0;
		for(page = 0; page < Data.pagesPerBlock; page++) {
			if(badBlockCount > 2) {
				DebugPrintf("ftl: findDeviceInfoBBT - too many bad pages, skipping block %d\r\n", block);
				break;
			}

			ret = nand_read_alternate_ecc(bank, (block * Data.pagesPerBlock) + page, buffer);
			if(ret != 0) {
				if(ret == 1) {
					DebugPrintf("ftl: findDeviceInfoBBT - found 'badBlock' on bank %d, page %d\r\n", (block * Data.pagesPerBlock) + page);
					badBlockCount++;
				}

				DebugPrintf("ftl: findDeviceInfoBBT - skipping bank %d, page %d\r\n", (block * Data.pagesPerBlock) + page);
				continue;
			}

			if(memcmp(buffer, "DEVICEINFOBBT\0\0\0", 16) == 0) {
				if(deviceInfoBBT) {
					memcpy(deviceInfoBBT, buffer + 0x38, *((u32*)(buffer + 0x34)));
				}

				vfree(buffer);
				return 1;
			} else {
				DebugPrintf("ftl: did not find signature on bank %d, page %d\r\n", (block * Data.pagesPerBlock) + page);
			}
		}
	}

	vfree(buffer);
	return 0;
}

static int hasDeviceInfoBBT(void) {
	int bank;
	int good = 1;
	for(bank = 0; bank < Data.banksTotal; bank++) {
		good = findDeviceInfoBBT(bank, NULL);
		if(!good)
			return 0;
	}

	return good;
}

static VFLData1Type VFLData1;
static VFLCxt* pstVFLCxt = NULL;
static u8* pstBBTArea = NULL;
static u32* ScatteredPageNumberBuffer = NULL;
static u16* ScatteredBankNumberBuffer = NULL;
static int VFLData4 = 0;
static u8 VFLData5[0xF8];

static int VFL_Init(void) {
	memset(&VFLData1, 0, sizeof(VFLData1));
	if(pstVFLCxt == NULL) {
		pstVFLCxt = vmalloc(Data.banksTotal * sizeof(VFLCxt));
		if(pstVFLCxt == NULL)
			return -1;
	}

	if(pstBBTArea == NULL) {
		pstBBTArea = (u8*) vmalloc((Data.blocksPerBank + 7) / 8);
		if(pstBBTArea == NULL)
			return -1;
	}

	if(ScatteredPageNumberBuffer == NULL && ScatteredBankNumberBuffer == NULL) {
		ScatteredPageNumberBuffer = (u32*) vmalloc(Data.pagesPerSubBlk * 4);
		ScatteredBankNumberBuffer = (u16*) vmalloc(Data.pagesPerSubBlk * 4);
		if(ScatteredPageNumberBuffer == NULL || ScatteredBankNumberBuffer == NULL)
			return -1;
	}

	VFLData4 = 0;

	return 0;
}

static FTLData1Type FTLData1;
static FTLCxt* pstFTLCxt;
static FTLCxt* FTLCxtBuffer;
static SpareData* FTLSpareBuffer;
static u32* ScatteredVirtualPageNumberBuffer;
static u8* StoreCxt;

static int FTL_Init(void) {
	int i, x, y, z, pagesPerSimpleMergeBuffer;
	int numPagesToWriteInStoreCxt = 0;

	x = ((Data.userSubBlksTotal + 23) * sizeof(u16)) / Data.bytesPerPage;
	if((((Data.userSubBlksTotal + 23) * sizeof(u16)) % Data.bytesPerPage) != 0) {
		x++;
	}

	numPagesToWriteInStoreCxt = x * 2;

	y = (Data.userSubBlksTotal * 2) / Data.bytesPerPage;
	if(((Data.userSubBlksTotal * 2) % Data.bytesPerPage) != 0) {
		y++;
	}

	numPagesToWriteInStoreCxt += y;

	z = (Data.pagesPerSubBlk * 34) / Data.bytesPerPage;
	if(((Data.pagesPerSubBlk * 34) % Data.bytesPerPage) != 0) {
		z++;
	}

	numPagesToWriteInStoreCxt += z + 2;

	if(numPagesToWriteInStoreCxt >= Data.pagesPerSubBlk) {
		printk("nand: error - FTL_NUM_PAGES_TO_WRITE_IN_STORECXT >= PAGES_PER_SUBLK\r\n");
		return -1;
	}

	pagesPerSimpleMergeBuffer = Data.pagesPerSubBlk / 8;
	if((pagesPerSimpleMergeBuffer * 2) >= Data.pagesPerSubBlk) {
		printk("nand: error - (PAGES_PER_SIMPLE_MERGE_BUFFER * 2) >=  PAGES_PER_SUBLK\r\n");
		return -1;
	}

	memset(&FTLData1, 0, 0x58);

	if(pstFTLCxt == NULL) {
		pstFTLCxt = FTLCxtBuffer = (FTLCxt*) vmalloc(sizeof(FTLCxt));
		if(pstFTLCxt == NULL)
			return -1;
		memset(pstFTLCxt->field_3D8, 0, sizeof(pstFTLCxt->field_3D8)); 
	}

	pstFTLCxt->field_31C = 0;

	pstFTLCxt->dataVbn = (u16*) vmalloc(Data.userSubBlksTotal * sizeof(u16));
	pstFTLCxt->field_1A0 = (u16*) vmalloc((Data.pagesPerSubBlk * 18) * sizeof(u16));
	pstFTLCxt->field_19C = (u16*) vmalloc((Data.userSubBlksTotal + 23) * sizeof(u16));
	pstFTLCxt->field_3B0 = (u16*) vmalloc((Data.userSubBlksTotal + 23) * sizeof(u16));

	FTLSpareBuffer = (SpareData*) vmalloc(Data.pagesPerSubBlk * sizeof(SpareData));

	if((Data.pagesPerSubBlk / 8) >= numPagesToWriteInStoreCxt) {
		numPagesToWriteInStoreCxt = Data.pagesPerSubBlk / 8;
	}

	StoreCxt = vmalloc(Data.bytesPerPage * numPagesToWriteInStoreCxt);
	ScatteredVirtualPageNumberBuffer = (u32*) vmalloc(Data.pagesPerSubBlk * sizeof(u32*));

	if(!pstFTLCxt->dataVbn || !pstFTLCxt->field_1A0 || !pstFTLCxt->field_19C || !FTLCxtBuffer->field_3B0 || ! FTLSpareBuffer || !StoreCxt || !ScatteredVirtualPageNumberBuffer)
		return -1;

	for(i = 0; i < 18; i++) {
		pstFTLCxt->pLog[i].field_8 = pstFTLCxt->field_1A0 + (i * Data.pagesPerSubBlk);
		memset(pstFTLCxt->pLog[i].field_8, 0xFF, Data.pagesPerSubBlk * 2);
		pstFTLCxt->pLog[i].field_10 = 1;
		pstFTLCxt->pLog[i].field_C = 0;
		pstFTLCxt->pLog[i].field_E = 0;
	}

	return 0;
}

// pageBuffer and spareBuffer are represented by single BUF struct within Whimory
static int vfl_read_page(int bank, int block, int page, u8* pageBuffer, u8* spareBuffer) {
	int i;
	memset(spareBuffer, 0, Data.bytesPerSpare);
	for(i = 0; i < 8; i++) {
		if(nand_read(bank, (block * Data.pagesPerBlock) + page + i, pageBuffer, spareBuffer, 1, 1) == 0) {
			SpareData* spareData = (SpareData*) spareBuffer;
			if(spareData->field_8 == 0 && spareData->field_9 == 0x80)
				return 1;
		}
	}
	return 0;
}

static void vfl_checksum(void* data, int size, u32* a, u32* b) {
	int i;
	u32* buffer = (u32*) data;
	u32 x = 0;
	u32 y = 0;
	for(i = 0; i < (size / 4); i++) {
		x += buffer[i];
		y ^= buffer[i];
	}

	*a = x + 0xAABBCCDD;
	*b = y ^ 0xAABBCCDD;
}

static int vfl_gen_checksum(int bank) {
	vfl_checksum(&pstVFLCxt[bank], (u32)&pstVFLCxt[bank].checksum1 - (u32)&pstVFLCxt[bank], &pstVFLCxt[bank].checksum1, &pstVFLCxt[bank].checksum2);
	return 0;
}

static int vfl_check_checksum(int bank) {
	u32 checksum1, checksum2;
	static int counter = 0;

	counter++;

	vfl_checksum(&pstVFLCxt[bank], (u32)&pstVFLCxt[bank].checksum1 - (u32)&pstVFLCxt[bank], &checksum1, &checksum2);

	// Yeah, this looks fail, but this is actually the logic they use
	if(checksum1 == pstVFLCxt[bank].checksum1)
		return 1;

	if(checksum2 != pstVFLCxt[bank].checksum2)
		return 1;

	return 0;
}

static void virtual_page_number_to_virtual_address(u32 dwVpn, u16* virtualBank, u16* virtualBlock, u16* virtualPage) {
	*virtualBank = dwVpn % Data.banksTotal;
	*virtualBlock = dwVpn / Data.pagesPerSubBlk;
	*virtualPage = (dwVpn / Data.banksTotal) % Data.pagesPerBlock;
}

// badBlockTable is a bit array with 8 virtual blocks in one bit entry
static int isGoodBlock(u8* badBlockTable, u16 virtualBlock) {
	int index = virtualBlock/8;
	return ((badBlockTable[index / 8] >> (7 - (index % 8))) & 0x1) == 0x1;
}

static u16 virtual_block_to_physical_block(u16 virtualBank, u16 virtualBlock) {
	int pwDesPbn;
	if(isGoodBlock(pstVFLCxt[virtualBank].badBlockTable, virtualBlock))
		return virtualBlock;

	for(pwDesPbn = 0; pwDesPbn < pstVFLCxt[virtualBank].numReservedBlocks; pwDesPbn++) {
		if(pstVFLCxt[virtualBank].reservedBlockPoolMap[pwDesPbn] == virtualBlock) {
			if(pwDesPbn >= Data.blocksPerBank) {
				printk("ftl: Destination physical block for remapping is greater than number of blocks per bank!");
			}
			return pstVFLCxt[virtualBank].reservedBlockPoolStart + pwDesPbn;
		}
	}

	return virtualBlock;
}

static int VFL_Read(u32 virtualPageNumber, u8* buffer, u8* spare, int empty_ok, int* refresh_page) {
	u16 virtualBank;
	u16 virtualBlock;
	u16 virtualPage;
	u16 physicalBlock;
	u32 dwVpn;
	int page, ret;

	if(refresh_page) {
		*refresh_page = 0;
	}

	VFLData1.field_8++;
	VFLData1.field_20++;

	dwVpn = virtualPageNumber + (Data.pagesPerSubBlk * Data2.field_4);
	if(dwVpn >= Data.pagesTotal) {
		printk("ftl: dwVpn overflow: %d\r\n", dwVpn);
		return ERROR_ARG;
	}

	if(dwVpn < Data.pagesPerSubBlk) {
		printk("ftl: dwVpn underflow: %d\r\n", dwVpn);
	}

	virtual_page_number_to_virtual_address(dwVpn, &virtualBank, &virtualBlock, &virtualPage);
	physicalBlock = virtual_block_to_physical_block(virtualBank, virtualBlock);

	page = physicalBlock * Data.pagesPerBlock + virtualPage;

	ret = nand_read(virtualBank, page, buffer, spare, 1, 1);

	if(!empty_ok && ret == ERROR_EMPTYBLOCK) {
		ret = ERROR_NAND;
	}

	if(refresh_page) {
		if((Data.field_2F <= 0 && ret == 0) || ret == ERROR_NAND) {
			printk("ftl: setting refresh_page to TRUE due to the following factors: Data.field_2F = %x, ret = %d\r\n", Data.field_2F, ret);
			*refresh_page = 1;
		}
	}

	if(ret == ERROR_ARG || ret == ERROR_NAND) {
		nand_bank_reset(virtualBank, 100);
		ret = nand_read(virtualBank, page, buffer, spare, 1, 1);
		if(!empty_ok && ret == ERROR_EMPTYBLOCK) {
			return ERROR_NAND;
		}

		if(ret == ERROR_ARG || ret == ERROR_NAND)
			return ret;
	}

	if(ret == ERROR_EMPTYBLOCK) {
		if(spare) {
			memset(spare, 0xFF, sizeof(SpareData));
		}
	}

	return ret;
}

static int VFL_ReadMultiplePagesInVb(int logicalBlock, int logicalPage, int count, u8* main, SpareData* spare, int* refresh_page) {
	int i;
	int currentPage = logicalPage; 
	for(i = 0; i < count; i++) {
		int ret = VFL_Read((logicalBlock * Data.pagesPerSubBlk) + currentPage, main + (Data.bytesPerPage * i), (u8*) &spare[i], 1, refresh_page);
		currentPage++;
		if(ret != 0)
			return 0;
	}
	return 1;
}

static int VFL_ReadScatteredPagesInVb(u32* virtualPageNumber, int count, u8* main, SpareData* spare, int* refresh_page) {
	int i;
	int ret;
	VFLData1.field_8 += count;
	VFLData1.field_20++;

	if(refresh_page) {
		*refresh_page = 0;
	}

	for(i = 0; i < count; i++) {
		u32 dwVpn = virtualPageNumber[i] + (Data.pagesPerSubBlk * Data2.field_4);

		u16 virtualBlock;
		u16 virtualPage;
		u16 physicalBlock;

		virtual_page_number_to_virtual_address(dwVpn, &ScatteredBankNumberBuffer[i], &virtualBlock, &virtualPage);
		physicalBlock = virtual_block_to_physical_block(ScatteredBankNumberBuffer[i], virtualBlock);
		ScatteredPageNumberBuffer[i] = physicalBlock * Data.pagesPerBlock + virtualPage;
	}

	ret = nand_read_multiple(ScatteredBankNumberBuffer, ScatteredPageNumberBuffer, main, spare, count);
	if(Data.field_2F <= 0 && refresh_page != NULL) {
		printk("ftl: VFL_ReadScatteredPagesInVb mark page for refresh\r\n");
		*refresh_page = 1;
	}

	if(ret != 0)
		return 0;
	else
		return 1;
}

// sub_18015A9C
static u16* VFL_get_maxThing(void) {
	int bank = 0;
	int max = 0;
	u16* maxThing = NULL;
	for(bank = 0; bank < Data.banksTotal; bank++) {
		int cur = pstVFLCxt[bank].field_0;
		if(max <= cur) {
			max = cur;
			maxThing = pstVFLCxt[bank].field_4;
		}
	}

	return maxThing;
}

static int VFL_Open(void) {
	int bank = 0;
	int i;
	int minLpn;
	int VFLCxtIdx;
	int page, last;
	u8* pageBuffer;
	u8* spareBuffer;
	VFLCxt* curVFLCxt;
	void* maxThing;
	SpareData* spareData;
	u16 buffer[3];

	for(bank = 0; bank < Data.banksTotal; bank++) {
		if(!findDeviceInfoBBT(bank, pstBBTArea)) {
			printk("ftl: findDeviceInfoBBT failed\r\n");
			return -1;
		}

		if(bank >= Data.banksTotal) {
			return -1;
		}


		curVFLCxt = &pstVFLCxt[bank];
		pageBuffer = vmalloc(Data.bytesPerPage);
		spareBuffer = vmalloc(Data.bytesPerSpare);
		if(pageBuffer == NULL || spareBuffer == NULL) {
			printk("ftl: cannot allocate page and spare buffer\r\n");
			return -1;
		}

		i = 1;
		for(i = 1; i < Data2.field_0; i++) {
			// so pstBBTArea is a bit array of some sort
			if(!(pstBBTArea[i / 8] & (1 << (i  & 0x7))))
				continue;

			if(vfl_read_page(bank, i, 0, pageBuffer, spareBuffer) == 1) {
				memcpy(curVFLCxt->VFLCxtBlock, ((VFLCxt*)pageBuffer)->VFLCxtBlock, sizeof(curVFLCxt->VFLCxtBlock));
				break;
			}
		}

		if(i == Data2.field_0) {
			printk("ftl: cannot find readable VFLCxtBlock\r\n");
			vfree(pageBuffer);
			vfree(spareBuffer);
			return -1;
		}

		minLpn = 0xFFFFFFFF;
		VFLCxtIdx = 4;
		for(i = 0; i < 4; i++) {
			u16 block = curVFLCxt->VFLCxtBlock[i];
			if(block == 0xFFFF)
				continue;

			if(vfl_read_page(bank, block, 0, pageBuffer, spareBuffer) != 1)
				continue;

			spareData = (SpareData*) spareBuffer;
			if(spareData->logicalPageNumber > 0 && spareData->logicalPageNumber <= minLpn) {
				minLpn = spareData->logicalPageNumber;
				VFLCxtIdx = i;
			}
		}

		if(VFLCxtIdx == 4) {
			printk("ftl: cannot find readable VFLCxtBlock index in spares\r\n");
			vfree(pageBuffer);
			vfree(spareBuffer);
			return -1;
		}

		page = 8;
		last = 0;
		for(page = 8; page < Data.pagesPerBlock; page += 8) {
			if(vfl_read_page(bank, curVFLCxt->VFLCxtBlock[VFLCxtIdx], page, pageBuffer, spareBuffer) == 0) {
				break;
			}
			
			last = page;
		}

		if(vfl_read_page(bank, curVFLCxt->VFLCxtBlock[VFLCxtIdx], last, pageBuffer, spareBuffer) == 0) {
			printk("ftl: cannot find readable VFLCxt\n");
			vfree(pageBuffer);
			vfree(spareBuffer);
			return -1;
		}

		// Aha, so the upshot is that this finds the VFLCxt and copies it into pstVFLCxt
		memcpy(&pstVFLCxt[bank], pageBuffer, sizeof(VFLCxt));
		if(curVFLCxt->field_0 >= VFLData4) {
			VFLData4 = curVFLCxt->field_0;
		}

		vfree(pageBuffer);
		vfree(spareBuffer);

		if(vfl_check_checksum(bank) == 0) {
			printk("ftl: VFLCxt has bad checksum\n");
			return -1;
		}
	} 

	maxThing = VFL_get_maxThing();

	memcpy(buffer, maxThing, 6);

	for(bank = 0; bank < Data.banksTotal; bank++) {
		memcpy(pstVFLCxt[bank].field_4, buffer, sizeof(buffer));
		vfl_gen_checksum(bank);
	}

	return 0;
}

void FTL_64bit_sum(u64* src, u64* dest, int size) {
	int i;
	for(i = 0; i < size / sizeof(u64); i++) {
		dest[i] += src[i];
	}
}

static int FTL_Restore(void) {
	return 0;
}

static int FTL_GetStruct(FTLStruct type, void** data, int* size) {
	switch(type) {
		case FTLData1SID:
			*data = &FTLData1;
			*size = sizeof(FTLData1);
			return 1;
		default:
			return 0;
	}
}

static int VFL_GetStruct(FTLStruct type, void** data, int* size) {
	switch(type) {
		case VFLData1SID:
			*data = &VFLData1;
			*size = sizeof(VFLData1);
			return 1;
		case VFLData5SID:
			*data = VFLData5;
			*size = 0xF8;
			return 1;
		default:
			return 0;
	}
}

static int sum_data(u8* pageBuffer) {
	void* data;
	int size;
	FTL_GetStruct(FTLData1SID, &data, &size);
	FTL_64bit_sum((u64*)pageBuffer, (u64*)data, size);
	VFL_GetStruct(VFLData1SID, &data, &size);
	FTL_64bit_sum((u64*)(pageBuffer + 0x200), (u64*)data, size);
	VFL_GetStruct(VFLData5SID, &data, &size);
	FTL_64bit_sum((u64*)(pageBuffer + 0x400), (u64*)data, size);
	return 1;
}

static int FTL_Open(int* pagesAvailable, int* bytesPerPage) {
	int refreshPage;
	int ret;
	int i;
	u32 minLpn;
	u32 ftlCxtBlock;
	u8* pageBuffer;
	u8* spareBuffer;

	u16* dataVbn = pstFTLCxt->dataVbn;
	u16* field_19C = pstFTLCxt->field_19C;
	void* field_3B0 = pstFTLCxt->field_3B0;
	u16* field_1A0 = pstFTLCxt->field_1A0;

	int pagesToRead;
	int ftlCxtFound;
	int toRead;
	int x;
	void* thing;
	SpareData* spareData;

	int success = 0;

	if((thing = VFL_get_maxThing()) == NULL)
		goto FTL_Open_Error;
	
	memcpy(pstFTLCxt->thing, thing, sizeof(pstFTLCxt->thing));

	pageBuffer = vmalloc(Data.bytesPerPage);
	spareBuffer = vmalloc(Data.bytesPerSpare);
	if(!pageBuffer || !spareBuffer) {
		printk("ftl: FTL_Open ran out of memory!\r\n");
		return ERROR_ARG;
	}

	ftlCxtBlock = 0xffff;
	minLpn = 0xffffffff;
	for(i = 0; i < sizeof(pstFTLCxt->thing)/sizeof(u16); i++) {
		ret = VFL_Read(Data.pagesPerSubBlk * pstFTLCxt->thing[i], pageBuffer, spareBuffer, 1, &refreshPage);
		if(ret == ERROR_ARG) {
			vfree(pageBuffer);
			vfree(spareBuffer);
			goto FTL_Open_Error;
		}

		spareData = (SpareData*) spareBuffer;
		if((spareData->field_9 - 0x43) > 0xC)
			continue;

		if(ret != 0)
			continue;

		if(ftlCxtBlock != 0xffff && spareData->logicalPageNumber >= minLpn)
			continue;

		minLpn = spareData->logicalPageNumber;
		ftlCxtBlock = pstFTLCxt->thing[i];
	}


	if(ftlCxtBlock == 0xffff) {
		printk("ftl: Cannot find context!\r\n");
		goto FTL_Open_Error_Release;
	}

	printk("ftl: Successfully found FTL context block: %d\r\n", ftlCxtBlock);

	ftlCxtFound = 0;
	for(i = Data.pagesPerSubBlk - 1; i > 0; i--) {
		ret = VFL_Read(Data.pagesPerSubBlk * ftlCxtBlock + i, pageBuffer, spareBuffer, 1, &refreshPage);
		if(ret == 1) {
			continue;
		} else if(ret == 0 && ((SpareData*)spareBuffer)->field_9 == 0x43) {
			memcpy(FTLCxtBuffer, pageBuffer, sizeof(FTLCxt));
			ftlCxtFound = 1;
			break;
		} else {
			ftlCxtFound = 0;
			break;
		}
	}

	// Restore now possibly overwritten (by data from NAND) pointers from backed up copies
	pstFTLCxt->dataVbn = dataVbn;
	pstFTLCxt->field_19C = field_19C;
	pstFTLCxt->field_3B0 = field_3B0;
	pstFTLCxt->field_1A0 = field_1A0;

	for(i = 0; i < 18; i++) {
		pstFTLCxt->pLog[i].field_8 = pstFTLCxt->field_1A0 + (i * Data.pagesPerSubBlk);
	}

	if(!ftlCxtFound)
		goto FTL_Open_Error_Release;

	printk("ftl: Successfully read FTL context block.\r\n");

	pagesToRead = (Data.userSubBlksTotal * 2) / Data.bytesPerPage;
	if(((Data.userSubBlksTotal * 2) % Data.bytesPerPage) != 0)
		pagesToRead++;

	for(i = 0; i < pagesToRead; i++) {
		if(VFL_Read(pstFTLCxt->pages_for_dataVbn[i], pageBuffer, spareBuffer, 1, &refreshPage) != 0)
			goto FTL_Open_Error_Release;

		toRead = Data.bytesPerPage;
		if(toRead > ((Data.userSubBlksTotal * 2) - (i * Data.bytesPerPage))) {
			toRead = (Data.userSubBlksTotal * 2) - (i * Data.bytesPerPage);
		}

		memcpy(((u8*)pstFTLCxt->dataVbn) + (i * Data.bytesPerPage), pageBuffer, toRead);	
	}

	pagesToRead = (Data.pagesPerSubBlk * 34) / Data.bytesPerPage;
	if(((Data.pagesPerSubBlk * 34) % Data.bytesPerPage) != 0)
		pagesToRead++;

	for(i = 0; i < pagesToRead; i++) {
		if(VFL_Read(pstFTLCxt->pages_for_1A0[i], pageBuffer, spareBuffer, 1, &refreshPage) != 0)
			goto FTL_Open_Error_Release;

		toRead = Data.bytesPerPage;
		if(toRead > ((Data.pagesPerSubBlk * 34) - (i * Data.bytesPerPage))) {
			toRead = (Data.pagesPerSubBlk * 34) - (i * Data.bytesPerPage);
		}

		memcpy(((u8*)pstFTLCxt->field_1A0) + (i * Data.bytesPerPage), pageBuffer, toRead);	
	}

	pagesToRead = ((Data.userSubBlksTotal + 23) * sizeof(u16)) / Data.bytesPerPage;
	if((((Data.userSubBlksTotal + 23) * sizeof(u16)) % Data.bytesPerPage) != 0)
		pagesToRead++;

	for(i = 0; i < pagesToRead; i++) {
		if(VFL_Read(pstFTLCxt->pages_for_19C[i], pageBuffer, spareBuffer, 1, &refreshPage) != 0)
			goto FTL_Open_Error_Release;

		toRead = Data.bytesPerPage;
		if(toRead > (((Data.pagesPerSubBlk + 23) * sizeof(u16)) - (i * Data.bytesPerPage))) {
			toRead = ((Data.pagesPerSubBlk + 23) * sizeof(u16)) - (i * Data.bytesPerPage);
		}

		memcpy(((u8*)pstFTLCxt->field_19C) + (i * Data.bytesPerPage), pageBuffer, toRead);	
	}

	printk("ftl: Detected version %x %x\r\n", FTLCxtBuffer->versionLower, FTLCxtBuffer->versionUpper);
	if(FTLCxtBuffer->versionLower == 0x46560001 && FTLCxtBuffer->versionUpper == 0xB9A9FFFE) {
		pagesToRead = ((Data.userSubBlksTotal + 23) * sizeof(u16)) / Data.bytesPerPage;
		if((((Data.userSubBlksTotal + 23) * sizeof(u16)) % Data.bytesPerPage) != 0)
			pagesToRead++;

		success = 1;
		for(i = 0; i < pagesToRead; i++) {
			if(VFL_Read(pstFTLCxt->pages_for_3B0[i], pageBuffer, spareBuffer, 1, &refreshPage) != 0) {
				success = 0;
				break;
			}

			toRead = Data.bytesPerPage;
			if(toRead > (((Data.pagesPerSubBlk + 23) * sizeof(u16)) - (i * Data.bytesPerPage))) {
				toRead = ((Data.pagesPerSubBlk + 23) * sizeof(u16)) - (i * Data.bytesPerPage);
			}

			memcpy(((u8*)pstFTLCxt->field_3B0) + (i * Data.bytesPerPage), pageBuffer, toRead);	
		}

		if((pstFTLCxt->field_3D4 + 1) == 0) {
			x = pstFTLCxt->field_3D0 / Data.pagesPerSubBlk;
			if(x == 0 || x <= Data.userSubBlksTotal) {
				if(VFL_Read(pstFTLCxt->field_3D0, pageBuffer, spareBuffer, 1, &refreshPage) != 0)
					goto FTL_Open_Error_Release;

				sum_data(pageBuffer);
			}
		}
	} else {
		printk("ftl: updating the FTL from seemingly compatible version\r\n");
		for(i = 0; i < (Data.userSubBlksTotal + 23); i++) {
			pstFTLCxt->field_3B0[i] = 0x1388;
		}

		for(i = 0; i < 5; i++) {
			pstFTLCxt->elements2[i].field_0 = -1;
			pstFTLCxt->elements2[i].field_2 = -1;
		}

		pstFTLCxt->field_3C8 = 0;
		pstFTLCxt->field_31C = 0;
		FTLCxtBuffer->versionLower = 0x46560000;
		FTLCxtBuffer->versionUpper = 0xB9A9FFFF;

		success = 1;
	}

	if(success) {
		printk("ftl: FTL successfully opened!\r\n");
		vfree(pageBuffer);
		vfree(spareBuffer);
		*pagesAvailable = Data.userPagesTotal;
		*bytesPerPage = Data.bytesPerPage;
		return 0;
	}

FTL_Open_Error_Release:
	vfree(pageBuffer);
	vfree(spareBuffer);

FTL_Open_Error:
	printk("ftl: FTL_Open cannot load FTLCxt!\r\n");
	if(FTL_Restore() != 0) {
		*pagesAvailable = Data.userPagesTotal;
		*bytesPerPage = Data.bytesPerPage;
		return 0;
	} else {
		return ERROR_ARG;
	}
}

u32 FTL_map_page(FTLCxtLog* pLog, int lbn, int offset) {
	if(pLog && pLog->field_8[offset] != 0xFFFF) {
		if(((pLog->wVbn * Data.pagesPerSubBlk) + pLog->field_8[offset] + 1) != 0)
			return (pLog->wVbn * Data.pagesPerSubBlk) + pLog->field_8[offset];
	}

	return (pstFTLCxt->dataVbn[lbn] * Data.pagesPerSubBlk) + offset;
}

static int FTL_Read(int logicalPageNumber, int totalPagesToRead, u8* pBuf) {
	int i;
	int hasError = 0;
	int lbn, offset;
	int ret, pagesRead, pagesToRead, refreshPage, currentLogicalPageNumber;
	int readSuccessful;
	int loop;
	u8* pageBuffer;
	u8* spareBuffer;
	FTLCxtLog* pLog;

	FTLData1.field_8 += totalPagesToRead;
	FTLData1.field_18++;
	pstFTLCxt->field_3CC++;

	if(!pBuf) {
		return ERROR_ARG;
	}

	if(totalPagesToRead == 0 || (logicalPageNumber + totalPagesToRead) >= Data.userPagesTotal) {
		printk("ftl: invalid input parameters\r\n");
		return ERROR_INPUT;
	}

	lbn = logicalPageNumber / Data.pagesPerSubBlk;
	offset = logicalPageNumber - (lbn * Data.pagesPerSubBlk);
	
	pageBuffer = vmalloc(Data.bytesPerPage);
	spareBuffer = vmalloc(Data.bytesPerSpare);
	if(!pageBuffer || !spareBuffer) {
		printk("ftl: FTL_Read ran out of memory!\r\n");
		return ERROR_ARG;
	}

	pLog = NULL;
	for(i = 0; i < 17; i++) {
		if(pstFTLCxt->pLog[i].wVbn == 0xFFFF)
			continue;

		if(pstFTLCxt->pLog[i].field_6 == lbn) {
			pLog = &pstFTLCxt->pLog[i];
			break;
		}
	}

	ret = 0;
	pagesRead = 0;
	currentLogicalPageNumber = logicalPageNumber;

	while(1) {
		pagesToRead = Data.pagesPerSubBlk - offset;
		if(pagesToRead >= (totalPagesToRead - pagesRead))
			pagesToRead = totalPagesToRead - pagesRead;

		if(pLog != NULL) {
			for(i = 0; i < pagesToRead; i++) {
				ScatteredVirtualPageNumberBuffer[i] = FTL_map_page(pLog, lbn, offset + i);
				if((ScatteredVirtualPageNumberBuffer[i] / Data.pagesPerSubBlk) == pLog->wVbn) {
					pstFTLCxt->field_3B0[ScatteredVirtualPageNumberBuffer[i] / Data.pagesPerSubBlk]++;
				} else {
					pstFTLCxt->field_3B0[pstFTLCxt->dataVbn[ScatteredVirtualPageNumberBuffer[i] / Data.pagesPerSubBlk]]++;
				}
			}

			readSuccessful = VFL_ReadScatteredPagesInVb(ScatteredVirtualPageNumberBuffer, pagesToRead, pBuf + (pagesRead * Data.bytesPerPage), FTLSpareBuffer, &refreshPage);
			if(refreshPage) {
				printk("ftl: _AddLbnToRefreshList (0x%x, 0x%x, 0x%x)\r\n", lbn, pstFTLCxt->dataVbn[lbn], pLog->wVbn);
			}
		} else {
			// VFL_ReadMultiplePagesInVb has a different calling convention and implementation than the equivalent iBoot function.
			// Ours is a bit less optimized, and just calls VFL_Read for each page.
			readSuccessful = VFL_ReadMultiplePagesInVb(pstFTLCxt->dataVbn[lbn], offset, pagesToRead, pBuf + (pagesRead * Data.bytesPerPage), FTLSpareBuffer, &refreshPage);
			if(refreshPage) {
				printk("ftl: _AddLbnToRefreshList (0x%x, 0x%x)\r\n", lbn, pstFTLCxt->dataVbn[lbn]);
			}
		}

		loop = 0;
		if(readSuccessful) {
			for(i = 0; i < pagesToRead; i++) {
				if(FTLSpareBuffer[i].eccMark == 0xFF)
					continue;

				printk("ftl: CHECK_FTL_ECC_MARK (0x%x, 0x%x, 0x%x, 0x%x)\r\n", lbn, offset, i, FTLSpareBuffer[i].eccMark);
				hasError = 1;
			}

			pagesRead += pagesToRead;
			currentLogicalPageNumber += pagesToRead;
			offset += pagesToRead;

			if(pagesRead == totalPagesToRead) {
				goto FTL_Read_Done;
			}

			loop = 0;
		} else {
			loop = 1;
		}

		do {
			if(pagesRead != totalPagesToRead && Data.pagesPerSubBlk != offset) {
				int virtualPage = FTL_map_page(pLog, lbn, offset);
				ret = VFL_Read(virtualPage, pBuf + (Data.bytesPerPage * pagesRead), spareBuffer, 1, &refreshPage);
				if(refreshPage) {
					printk("ftl: _AddLbnToRefreshList (0x%x, 0x%x)\r\n", lbn, virtualPage / Data.pagesPerSubBlk);
				}

				if(ret == ERROR_ARG)
					goto FTL_Read_Error_Release;

				if(ret == ERROR_NAND || ((SpareData*) spareBuffer)->eccMark != 0xFF) {
					printk("ftl: ECC error, ECC mark is: %x\r\n", ((SpareData*) spareBuffer)->eccMark);
					hasError = 1;
					if(pLog) {
						virtualPage = FTL_map_page(pLog, lbn, offset);
						printk("ftl: lbn 0x%x pLog->wVbn 0x%x dataVbn 0x%x offset 0x%x vpn 0x%x\r\n", lbn, pLog->wVbn, pstFTLCxt->dataVbn[lbn], offset, virtualPage);
					} else {
						virtualPage = FTL_map_page(NULL, lbn, offset);
						printk("ftl: lbn 0x%x dataVbn 0x%x offset 0x%x vpn 0x%x\r\n", lbn, pstFTLCxt->dataVbn[lbn], offset, virtualPage);
					}
				}

				if(ret == 0) {
					if(((SpareData*) spareBuffer)->logicalPageNumber != offset) {
						printk("ftl: error, dwWrittenLpn(0x%x) != dwLpn(0x%x)\r\n", ((SpareData*) spareBuffer)->logicalPageNumber, offset);
					}
				}

				pagesRead++;
				currentLogicalPageNumber++;
				offset++;
				if(pagesRead == totalPagesToRead) {
					goto FTL_Read_Done;
				}
			}

			if(offset == Data.pagesPerSubBlk) {
				lbn++;
				if(lbn >= Data.userSubBlksTotal)
					goto FTL_Read_Error_Release;

				pLog = NULL;
				for(i = 0; i < 17; i++) {
					if(pstFTLCxt->pLog[i].wVbn != 0xFFFF && pstFTLCxt->pLog[i].field_6 == lbn) {
						pLog = &pstFTLCxt->pLog[i];
						break;
					}
				}

				offset = 0;
				break;
			}
		} while(loop);
	}

FTL_Read_Done:
	vfree(pageBuffer);
	vfree(spareBuffer);
	if(hasError) {
		printk("ftl: USER_DATA_ERROR, failed with (0x%x, 0x%x, 0x%p)\r\n", logicalPageNumber, totalPagesToRead, pBuf);
		return ERROR_NAND;
	}

	return 0;

FTL_Read_Error_Release:
	vfree(pageBuffer);
	vfree(spareBuffer);
	printk("ftl: _FTLRead error!\r\n");
	return ret;
}

static int ftl_setup(void) {
	int i;
	int foundSignature;
	int pagesAvailable;
	int bytesPerPage;
	u8* buffer;
	int ret;

	if(HasFTLInit)
		return 0;

	nand_setup();

	if(VFL_Init() != 0) {
		printk("ftl: VFL_Init failed\r\n");
		return -1;
	}

	if(FTL_Init() != 0) {
		printk("ftl: FTL_Init failed\r\n");
		return -1;
	}

	foundSignature = 0;

	DebugPrintf("ftl: Attempting to read %d pages from first block of first bank.\r\n", Data.pagesPerBlock);
	buffer = vmalloc(Data.bytesPerPage);
	for(i = 0; i < Data.pagesPerBlock; i++) {
		if((ret = nand_read_alternate_ecc(0, i, buffer)) == 0) {
			u32 id = *((u32*) buffer);
			if(id == FTL_ID_V1 || id == FTL_ID_V2) {
				printk("ftl: Found production format: %x\r\n", id);
				foundSignature = 1;
				break;
			} else {
				DebugPrintf("ftl: Found non-matching signature: %x\r\n", ((u32*) buffer));
			}
		} else {
			DebugPrintf("ftl: page %d of first bank is unreadable: %x!\r\n", i, ret);
		}
	}
	vfree(buffer);

	if(!foundSignature || !hasDeviceInfoBBT()) {
		printk("ftl: no signature or production format.\r\n");
		return -1;
	}

	if(VFL_Open() != 0) {
		printk("ftl: VFL_Open failed\r\n");
		return -1;
	}

	if(FTL_Open(&pagesAvailable, &bytesPerPage) != 0) {
		printk("ftl: FTL_Open failed\r\n");
		return -1;
	}

	HasFTLInit = 1;

	return 0;
}

/*
 * LINUX BLOCK DEVICE SECTION
 *
 *
 *
 */

static struct iphone_nand_device {
	spinlock_t lock;
	struct gendisk* gd;
	struct block_device* bdev;
	struct request_queue* queue;
	int sectorSize;
} Device;

static int iphone_nand_ioctl(struct block_device *bdev, fmode_t mode,
			unsigned int cmd, unsigned long arg)
{
	long size;
	struct hd_geometry geo;

	switch(cmd) {
		/*
		 * The only command we need to interpret is HDIO_GETGEO, since
		 * we can't partition the drive otherwise.  We have no real
		 * geometry, of course, so make something up.
		 */
		case HDIO_GETGEO:
			size = Data.pagesPerSubBlk * Data.userSubBlksTotal * (Device.sectorSize >> SECTOR_SHIFT);
			geo.cylinders = (size & ~0x3f) >> 6;
			geo.heads = 4;
			geo.sectors = 16;
			geo.start = 4;
			if (copy_to_user((void *) arg, &geo, sizeof(geo)))
				return -EFAULT;
			return 0;
	}

	return -ENOTTY;
}

static struct block_device_operations iphone_nand_fops = {
	.owner =		THIS_MODULE,
	.locked_ioctl =		iphone_nand_ioctl,
};

static void iphone_nand_read(struct iphone_nand_device* dev, unsigned long sectorNum, unsigned long len, char* buffer) {
	printk("iphone_nand_read: sector %ld (%ld), len %ld (%ld) to %p\n", sectorNum, sectorNum >> 3, len, len >> 3, buffer);
	FTL_Read(sectorNum >> 3, len >> 3, buffer);
}

static void iphone_nand_write(struct iphone_nand_device* dev, unsigned long sectorNum, unsigned long len, char* buffer) {

}

static void iphone_nand_request(struct request_queue* q)
{
	struct request *req;

	while ((req = elv_next_request(q)) != NULL)
	{
		if (! blk_fs_request(req))
		{
			end_request(req, 0);
			continue;
		}
		if(rq_data_dir(req))
		{
			// WRITE
			iphone_nand_write(&Device, req->sector, req->current_nr_sectors,
					req->buffer);
		} else {
			// READ
			iphone_nand_read(&Device, req->sector, req->current_nr_sectors,
					req->buffer);
		}
		end_request(req, 1);
	}
}

static int major_num = 0;

static int __init iphone_nand_init(void)
{
	spin_lock_init(&Device.lock);
	major_num = register_blkdev(major_num, "nand");

	if(major_num <= 0)
	{
		printk("%s: unable to get major number\n", driver_name);
		goto out;
	}

	nand_setup();
	ftl_setup();

	Device.sectorSize = Data.bytesPerPage;

	Device.gd = alloc_disk(1);
	if(!Device.gd)
		goto out_unregister;

	Device.gd->major = major_num;
	Device.gd->first_minor = 0;
	Device.gd->fops = &iphone_nand_fops;
	Device.gd->private_data = &Device;
	strcpy(Device.gd->disk_name, "nand0");
	Device.queue = blk_init_queue(iphone_nand_request, &Device.lock);
	if (!Device.queue)
		goto out_put_disk;

	blk_queue_max_sectors(Device.queue, Data.sectorsPerPage);
	blk_queue_bounce_limit(Device.queue, BLK_BOUNCE_ANY);
	blk_queue_hardsect_size(Device.queue, Device.sectorSize);

	Device.gd->queue = Device.queue;

	set_capacity(Device.gd, Data.pagesPerSubBlk * Data.userSubBlksTotal * (Device.sectorSize >> SECTOR_SHIFT));
	add_disk(Device.gd);

out_put_disk:
	put_disk(Device.gd);
out_unregister:
	unregister_blkdev(major_num, "nand");
out:
	return -ENOMEM;
}

static void __exit iphone_nand_exit(void)
{
	del_gendisk(Device.gd);
	put_disk(Device.gd);
	blk_cleanup_queue(Device.queue);
	unregister_blkdev(major_num, "nand");
}

module_init(iphone_nand_init);
module_exit(iphone_nand_exit);

MODULE_LICENSE("GPL");
