#include <mach/hardware.h>
#include <ftl/nand.h>
#include <mach/iphone-dma.h>
#include <mach/iphone-clock.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>

#define LOG printk
#define LOGDBG(format, ...)

// Device
#define NAND IO_ADDRESS(0x38A00000)
#define NANDECC IO_ADDRESS(0x38F00000)
#define NAND_CLOCK_GATE1 0x8
#define NAND_CLOCK_GATE2 0xC
#define NANDECC_INT 0x2B
#define NAND_INT 0x14

// Registers
#define FMCTRL0 0x0
#define FMCTRL1 0x4
#define NAND_CMD 0x8
#define FMADDR0 0xC
#define FMANUM 0x2C
#define FMADDR1 0x10
#define FMDNUM 0x30
#define NAND_REG_44 0x44
#define FMCSTAT 0x48
#define FMFIFO 0x80
#define RSCTRL 0x100

#define NANDECC_DATA 0x4
#define NANDECC_ECC 0x8
#define NANDECC_START 0xC
#define NANDECC_STATUS 0x10
#define NANDECC_SETUP 0x14
#define NANDECC_CLEARINT 0x40

// Values

#define FMCTRL_TWH_SHIFT 12
#define FMCTRL_TWP_SHIFT 16
#define FMCTRL_TWH_MASK 0x7
#define FMCTRL_TWP_MASK 0x7
#define FMCTRL0_DMASETTINGSHIFT 10
#define FMCTRL0_ON 1
#define FMCTRL0_WPB 0x800

#define NAND_CMD_RESET 0xFF
#define NAND_CMD_ID 0x90
#define NAND_CMD_READSTATUS 0x70
#define NAND_CMD_READ 0x30

#define FMANUM_TRANSFERSETTING 4

#define FMCTRL1_DOTRANSADDR (1 << 0)
#define FMCTRL1_CLEARALL (FMCTRL1_FLUSHFIFOS | 0x720)
#define FMCTRL1_FLUSHFIFOS (FMCTRL1_FLUSHRXFIFO | FMCTRL1_FLUSHTXFIFO)
#define FMCTRL1_FLUSHTXFIFO 0x40
#define FMCTRL1_FLUSHRXFIFO 0x80
#define FMCTRL1_DOREADDATA (1 << 1)

#define FMCSTAT_READY 0x1

#define NAND_NUM_BANKS 8

// NAND database

typedef struct NANDDeviceType {
	uint32_t id;
	uint16_t blocksPerBank;
	uint16_t pagesPerBlock;
	uint16_t sectorsPerPage;
	uint16_t bytesPerSpare;
	uint8_t WPPulseTime;
	uint8_t WEHighHoldTime;
	uint8_t NANDSetting3;
	uint8_t NANDSetting4;
	uint32_t userSuBlksTotal;
	uint32_t ecc1;
	uint32_t ecc2;
} NANDDeviceType;

static const NANDDeviceType SupportedDevices[] = {
	{0x2555D5EC, 8192, 128, 4, 64, 4, 2, 4, 2, 7744, 4, 6},
	{0xB614D5EC, 4096, 128, 8, 128, 4, 2, 4, 2, 3872, 4, 6},
	{0xB655D7EC, 8192, 128, 8, 128, 4, 2, 4, 2, 7744, 4, 6},
	{0xA514D3AD, 4096, 128, 4, 64, 4, 2, 4, 2, 3872, 4, 6},
	{0xA555D5AD, 8192, 128, 4, 64, 4, 2, 4, 2, 7744, 4, 6},
	{0xB614D5AD, 4096, 128, 8, 128, 4, 2, 4, 2, 3872, 4, 6},
	{0xB655D7AD, 8192, 128, 8, 128, 4, 2, 4, 2, 7744, 4, 6},
	{0xA585D598, 8320, 128, 4, 64, 6, 2, 4, 2, 7744, 4, 6},
	{0xBA94D598, 4096, 128, 8, 216, 6, 2, 4, 2, 3872, 8, 8},
	{0xBA95D798, 8192, 128, 8, 216, 6, 2, 4, 2, 7744, 8, 8},
	{0x3ED5D789, 8192, 128, 8, 216, 4, 2, 4, 2, 7744, 8, 8},
	{0x3E94D589, 4096, 128, 8, 216, 4, 2, 4, 2, 3872, 8, 8},
	{0x3ED5D72C, 8192, 128, 8, 216, 4, 2, 4, 2, 7744, 8, 8},
	{0x3E94D52C, 4096, 128, 8, 216, 4, 2, 4, 2, 3872, 8, 8},
	{0}
};

// NAND configuration

static u8 WEHighHoldTime;
static u8 WPPulseTime;
static bool LargePages;
static int ECCType = 0;
static int ECCType2;
static u32 TotalECCDataSize;
static const bool NoMultibankCmdStatus = true;
static int NumValidBanks = 0;

static NANDData Geometry;
static NANDFTLData FTLData;

// NAND state data

static int banksTable[NAND_NUM_BANKS];

// Global buffers

static u8* aTemporaryReadEccBuf;
static u8* aTemporarySBuf;

// Linux stuff

struct device *nand_dev;

#define VIC1 IO_ADDRESS(0x38E01000)
#define VICRAWINTR 0x8
#define VIC_InterruptSeparator 0x20
static int wait_for_ecc_interrupt(int timeout)
{
	u64 startTime = iphone_microtime();
	u32 mask = (1 << (NANDECC_INT - VIC_InterruptSeparator));
	while((readl(VIC1 + VICRAWINTR) & mask) == 0) {
		if(iphone_has_elapsed(startTime, timeout * 1000)) {
			return -ETIMEDOUT;
		}
	}

	writel(1, NANDECC + NANDECC_CLEARINT);

	if((readl(VIC1 + VICRAWINTR) & mask) == 0) {
		return 0;
	} else {
		return -ETIMEDOUT;
	}
}

static int ecc_finish(dma_addr_t sectorDMA, dma_addr_t eccDMA, int sectors) {
	int ret;
	if((ret = wait_for_ecc_interrupt(500)) != 0)
		return ret;

	dma_unmap_single(nand_dev, sectorDMA, sectors * SECTOR_SIZE, DMA_BIDIRECTIONAL);
	dma_unmap_single(nand_dev, eccDMA, sectors* 20, DMA_BIDIRECTIONAL);

	if((readl(NANDECC + NANDECC_STATUS) & 0x1) != 0)
		return ERROR_ECC;

	return 0;
}

static int ecc_perform(int setting, int sectors, u8* sectorData, u8* eccData) {
	dma_addr_t sectorDMA = dma_map_single(nand_dev, sectorData, sectors * SECTOR_SIZE, DMA_BIDIRECTIONAL);
	dma_addr_t eccDMA = dma_map_single(nand_dev, eccData, sectors * 20, DMA_BIDIRECTIONAL);

	writel(1, NANDECC + NANDECC_CLEARINT);
	writel(((sectors - 1) & 0x3) | setting, NANDECC + NANDECC_SETUP);
	writel(sectorDMA, NANDECC + NANDECC_DATA);
	writel(eccDMA, NANDECC + NANDECC_ECC);

	writel(1, NANDECC + NANDECC_START);

	return ecc_finish(sectorDMA, eccDMA, sectors);
}

static int ecc_generate(int setting, int sectors, u8* sectorData, u8* eccData) {
	dma_addr_t sectorDMA = dma_map_single(nand_dev, sectorData, sectors * SECTOR_SIZE, DMA_BIDIRECTIONAL);
	dma_addr_t eccDMA = dma_map_single(nand_dev, eccData, sectors * 20, DMA_BIDIRECTIONAL);

	writel(1, NANDECC + NANDECC_CLEARINT);
	writel(((sectors - 1) & 0x3) | setting, NANDECC + NANDECC_SETUP);
	writel(virt_to_phys(sectorData), NANDECC + NANDECC_DATA);
	writel(virt_to_phys(eccData), NANDECC + NANDECC_ECC);

	writel(2, NANDECC + NANDECC_START);
	
	return ecc_finish(sectorDMA, eccDMA, sectors);
}

static int checkECC(int setting, u8* data, u8* ecc) {
	int eccSize = 0;
	u8* dataPtr = data;
	u8* eccPtr = ecc;
	int sectorsLeft = Geometry.sectorsPerPage;

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

		if(ecc_perform(setting, toCheck, dataPtr, eccPtr) != 0)
			return ERROR_ECC;

		dataPtr += toCheck * SECTOR_SIZE;
		eccPtr += toCheck * eccSize;
		sectorsLeft -= toCheck;
	}

	return 0;
}

static int generateECC(int setting, u8* data, u8* ecc) {
	int eccSize = 0;
	u8* dataPtr = data;
	u8* eccPtr = ecc;
	int sectorsLeft = Geometry.sectorsPerPage;

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

		ecc_generate(setting, toCheck, dataPtr, eccPtr);

		if(LargePages) {
			// If there are more than 4 sectors in a page...
			int i;
			for(i = 0; i < toCheck; i++) {
				// loop through each sector that we have generated this time's ECC
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

		dataPtr += toCheck * SECTOR_SIZE;
		eccPtr += toCheck * eccSize;
		sectorsLeft -= toCheck;
	}

	return 0;
}

static int wait_for_ready(int timeout) {
	u64 startTime;
	if((readl(NAND + FMCSTAT) & FMCSTAT_READY) != 0) {
		return 0;
	}

	startTime = iphone_microtime();
	while((readl(NAND + FMCSTAT) & FMCSTAT_READY) == 0) {
		if(iphone_has_elapsed(startTime, timeout * 1000)) {
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int wait_for_address_done(int timeout) {
	u64 startTime;
	if((readl(NAND + FMCSTAT) & (1 << 2)) != 0) {
		writel(1 << 2, NAND + FMCSTAT);
		return 0;
	}

	startTime = iphone_microtime();
	while((readl(NAND + FMCSTAT) & (1 << 2)) == 0) {
		if(iphone_has_elapsed(startTime, timeout * 1000)) {
			return -ETIMEDOUT;
		}
	}

	writel(1 << 2, NAND + FMCSTAT);

	return 0;
}

static int wait_for_command_done(int bank, int timeout) {
	u32 toTest;

	u64 startTime = iphone_microtime();
	if(NoMultibankCmdStatus)
		bank = 0;
	else
		bank &= 0xffff;

	toTest = 1 << (bank + 4);

	while((readl(NAND + FMCSTAT) & toTest) == 0) {
		if(iphone_has_elapsed(startTime, timeout * 1000)) {
			return -ETIMEDOUT;
		}
	}

	writel(toTest, NAND + FMCSTAT);

	return 0;
}

static int wait_for_transfer_done(int timeout) {
	u64 startTime;
	if((readl(NAND + FMCSTAT) & (1 << 3)) != 0) {
		writel(1 << 3, NAND + FMCSTAT);
		return 0;
	}

	startTime = iphone_microtime();
	while((readl(NAND + FMCSTAT) & (1 << 3)) == 0) {
		if(iphone_has_elapsed(startTime, timeout * 1000)) {
			return -ETIMEDOUT;
		}
	}

	writel(1 << 3, NAND + FMCSTAT);

	return 0;
}

int nand_read_status(void)
{
	int status;

	writel(readl(NAND + NAND_REG_44) & ~(1 << 4), NAND + NAND_REG_44);
	writel(FMCTRL1_CLEARALL, NAND + FMCTRL1);
	writel(NAND_CMD_READSTATUS, NAND + NAND_CMD);
	writel(0, NAND + FMDNUM);
	writel(FMCTRL1_CLEARALL | FMCTRL1_DOREADDATA, NAND + FMCTRL1);

	wait_for_transfer_done(500);

	status = readl(NAND + FMFIFO);
	writel(FMCTRL1_CLEARALL, NAND + FMCTRL1);
	writel(1 << 3, NAND + FMCSTAT);
	writel(readl(NAND + NAND_REG_44) | (1 << 2), NAND + NAND_REG_44);

	return status;
}

static int wait_for_nand_bank_ready(int bank)
{
	u32 toTest;
	u64 startTime;

	writel(((WEHighHoldTime & FMCTRL_TWH_MASK) << FMCTRL_TWH_SHIFT) | ((WPPulseTime & FMCTRL_TWP_MASK) << FMCTRL_TWP_SHIFT)
			| (1 << (banksTable[bank] + 1)) | FMCTRL0_ON | FMCTRL0_WPB, NAND + FMCTRL0);

	toTest = 1 << (bank + 4);
	if((readl(NAND + FMCSTAT) & toTest) != 0)
	{
		writel(toTest, NAND + FMCSTAT);
	}

	writel(FMCTRL1_FLUSHFIFOS, NAND + FMCTRL1); 
	writel(NAND_CMD_READSTATUS, NAND + NAND_CMD);
	wait_for_ready(500);

	startTime = iphone_microtime();
	while(true)
	{
		u32 data;

		writel(0, NAND + FMDNUM);
		writel(FMCTRL1_DOREADDATA, NAND + FMCTRL1);

		if(wait_for_transfer_done(500) != 0)
		{
			LOG("nand: wait_for_nand_bank_ready: wait for transfer done timed out\n");
			return -ETIMEDOUT;
		}


		data = readl(NAND + FMFIFO);
		writel(FMCTRL1_FLUSHRXFIFO, NAND + FMCTRL1);
		if((data & (1 << 6)) == 0)
		{
			if(iphone_has_elapsed(startTime, 500 * 1000))
			{
				LOG("nand: wait_for_nand_bank_ready: wait for bit 6 of DMA timed out\n");
				return -ETIMEDOUT;
			}
		} else
		{
			break;
		}
	}

	writel(0, NAND + NAND_CMD);
	wait_for_ready(500);
	return 0;
}

static int transferFromFlash(void* buffer, int size) {
	int controller = 0;
	int channel = 0;
	dma_addr_t dma;

	if((((u32)buffer) & 0x3) != 0) {
		// the buffer needs to be aligned for DMA, last two bits have to be clear
		return -EINVAL;
	}

	writel(readl(NAND + FMCTRL0) | (1 << FMCTRL0_DMASETTINGSHIFT), NAND + FMCTRL0);
	writel(size - 1, NAND + FMDNUM);
	writel(FMCTRL1_DOREADDATA, NAND + FMCTRL1);

	dma = dma_map_single(nand_dev, buffer, size, DMA_FROM_DEVICE);

	iphone_dma_request(IPHONE_DMA_NAND, 4, 4, IPHONE_DMA_MEMORY, 4, 4, &controller, &channel);
	iphone_dma_perform(IPHONE_DMA_NAND, (u32)dma, size, 0, &controller, &channel);

	if(iphone_dma_finish(controller, channel, 500) != 0) {
		LOG("nand: dma timed out\n");
		return -ETIMEDOUT;
	}

	if(wait_for_transfer_done(500) != 0) {
		LOG("nand: waiting for transfer done timed out\n");
		return -ETIMEDOUT;
	}

	writel(FMCTRL1_FLUSHFIFOS, NAND + FMCTRL1);

	dma_unmap_single(nand_dev, dma, size, DMA_FROM_DEVICE);

	return 0;
}

static int transferToFlash(void* buffer, int size) {
	int controller = 0;
	int channel = 0;
	dma_addr_t dma;

	if((((u32)buffer) & 0x3) != 0) {
		// the buffer needs to be aligned for DMA, last two bits have to be clear
		return -EINVAL;
	}

	writel(readl(NAND + FMCTRL0) | (1 << FMCTRL0_DMASETTINGSHIFT), NAND + FMCTRL0);
	writel(size - 1, NAND + FMDNUM);
	writel(0x7F4, NAND + FMCTRL1);

	dma = dma_map_single(nand_dev, buffer, size, DMA_TO_DEVICE);

	iphone_dma_request(IPHONE_DMA_MEMORY, 4, 4, IPHONE_DMA_NAND, 4, 4, &controller, &channel);
	iphone_dma_perform((u32)dma, IPHONE_DMA_NAND, size, 0, &controller, &channel);

	if(iphone_dma_finish(controller, channel, 500) != 0) {
		LOG("nand: dma timed out\n");
		return -ETIMEDOUT;
	}

	if(wait_for_transfer_done(500) != 0) {
		LOG("nand: waiting for transfer done timed out\n");
		return -ETIMEDOUT;
	}

	writel(FMCTRL1_FLUSHFIFOS, NAND + FMCTRL1);

	dma_unmap_single(nand_dev, dma, size, DMA_TO_DEVICE);

	return 0;
}

static bool isEmptyBlock(u8* buffer, int size) {
	int i;
	int found = 0;
	for(i = 0; i < size; i++) {
		if(buffer[i] != 0xFF) {
			found++;
		}
	}

	if(found <= 1)
		return true;
	else
		return false;
}

int nand_read(int bank, int page, u8* buffer, u8* spare, bool doECC, bool checkBlank)
{
	bool eccFailed;

	if(bank >= Geometry.banksTotal)
		return -EINVAL;

	if(page >= Geometry.pagesPerBank)
		return -EINVAL;

	if(buffer == NULL && spare == NULL)
		return -EINVAL;

	writel(((WEHighHoldTime & FMCTRL_TWH_MASK) << FMCTRL_TWH_SHIFT) | ((WPPulseTime & FMCTRL_TWP_MASK) << FMCTRL_TWP_SHIFT)
		| (1 << (banksTable[bank] + 1)) | FMCTRL0_ON | FMCTRL0_WPB, NAND + FMCTRL0);

	writel(0, NAND + NAND_CMD);
	if(wait_for_ready(500) != 0) {
		LOG("nand: bank setting failed\n");
		goto FIL_read_error;
	}

	writel(FMANUM_TRANSFERSETTING, NAND + FMANUM);

	if(buffer) {
		writel(page << 16, NAND + FMADDR0); // lower bits of the page number to the upper bits of CONFIG3
		writel((page >> 16) & 0xFF, NAND + FMADDR1); // upper bits of the page number

	} else {
		writel((page << 16) | Geometry.bytesPerPage, NAND + FMADDR0); // lower bits of the page number to the upper bits of CONFIG3
		writel((page >> 16) & 0xFF, NAND + FMADDR1); // upper bits of the page number	
	}

	writel(FMCTRL1_DOTRANSADDR, NAND + FMCTRL1);
	if(wait_for_address_done(500) != 0) {
		LOG("nand: sending address failed\n");
		goto FIL_read_error;
	}
	
	writel(NAND_CMD_READ, NAND + NAND_CMD);
	if(wait_for_ready(500) != 0) {
		LOG("nand: sending read command failed\n");
		goto FIL_read_error;
	}

	if(wait_for_nand_bank_ready(bank) != 0) {
		LOG("nand: nand bank not ready after a long time\n");
		goto FIL_read_error;
	}

	if(buffer) {
		if(transferFromFlash(buffer, Geometry.bytesPerPage) != 0) {
			LOG("nand: transferFromFlash failed\n");
			goto FIL_read_error;
		}
	}

	if(transferFromFlash(aTemporarySBuf, Geometry.bytesPerSpare) != 0) {
		LOG("nand: transferFromFlash for spare failed\n");
		goto FIL_read_error;
	}

	eccFailed = false;
	if(doECC) {
		if(buffer) {
			eccFailed = (checkECC(ECCType, buffer, aTemporarySBuf + sizeof(SpareData)) != 0);
		}

		memcpy(aTemporaryReadEccBuf, aTemporarySBuf, sizeof(SpareData));
		if(ecc_perform(ECCType, 1, aTemporaryReadEccBuf, aTemporarySBuf + sizeof(SpareData) + TotalECCDataSize) != 0)
		{
			memset(aTemporaryReadEccBuf, 0xFF, SECTOR_SIZE);
			eccFailed |= 1;
		}
	}

	if(spare) {
		if(doECC) {
			// We can only copy the first 12 bytes because the rest is probably changed by the ECC check routine
			memcpy(spare, aTemporaryReadEccBuf, sizeof(SpareData));
		} else {
			memcpy(spare, aTemporarySBuf, Geometry.bytesPerSpare);
		}
	}

	if(eccFailed || checkBlank) {
		if(isEmptyBlock(aTemporarySBuf, Geometry.bytesPerSpare) != 0) {
			return ERROR_EMPTYBLOCK;
		} else if(eccFailed) {
			return -EIO;
		}
	}

	return 0;

FIL_read_error:
	nand_bank_reset(bank, 100);
	return -EIO;
}

int nand_read_multiple(u16* bank, u32* pages, u8* main, SpareData* spare, int pagesCount)
{
	int i;
	unsigned int ret;
	for(i = 0; i < pagesCount; i++) {
		ret = nand_read(bank[i], pages[i], main, (u8*) &spare[i], true, true);
		if(ret > 1)
			return ret;

		main += Geometry.bytesPerPage;
	}

	return 0;
}

int nand_read_alternate_ecc(int bank, int page, u8* buffer) {
	int ret;
	if((ret = nand_read(bank, page, buffer, aTemporarySBuf, false, true)) != 0) {
		LOGDBG("nand: Raw read failed.\n");
		return ret;
	}

	if(checkECC(ECCType2, buffer, aTemporarySBuf) != 0) {
		LOGDBG("nand: Alternate ECC check failed, but raw read succeeded.\n");
		return -EIO;
	}

	return 0;
}

int nand_erase(int bank, int block)
{
	int pageAddr;

	if(bank >= Geometry.banksTotal)
		return -EINVAL;

	if(block >= Geometry.blocksPerBank)
		return -EINVAL;

	pageAddr = block * Geometry.pagesPerBlock;

	writel(((WEHighHoldTime & FMCTRL_TWH_MASK) << FMCTRL_TWH_SHIFT) | ((WPPulseTime & FMCTRL_TWP_MASK) << FMCTRL_TWP_SHIFT)
		| (1 << (banksTable[bank] + 1)) | FMCTRL0_ON | FMCTRL0_WPB, NAND + FMCTRL0);

	writel(FMCTRL1_CLEARALL, NAND + FMCTRL1);
	writel(0x60, NAND + NAND_CMD);

	writel(2, NAND + FMANUM);
	writel(pageAddr, NAND + FMADDR0);
	writel(FMCTRL1_DOTRANSADDR, NAND + FMCTRL1);

	if(wait_for_address_done(500) != 0) {
		LOG("nand (nand_erase): wait for address complete failed\n");
		goto FIL_erase_error;
	}

	writel(0xD0, NAND + NAND_CMD);
	wait_for_ready(500);

	while((nand_read_status() & (1 << 6)) == 0);

	if(nand_read_status() & 0x1)
		return -1;
	else
		return 0;

FIL_erase_error:
	return -1;
}

int nand_write(int bank, int page, u8* buffer, u8* spare, bool doECC)
{
	if(bank >= Geometry.banksTotal)
		return -EINVAL;

	if(page >= Geometry.pagesPerBank)
		return -EINVAL;

	if(buffer == NULL && spare == NULL)
		return -EINVAL;

	if(doECC) {
		memcpy(aTemporarySBuf, spare, sizeof(SpareData));
		if(generateECC(ECCType, buffer, aTemporarySBuf + sizeof(SpareData)) != 0) {
			LOG("nand: Unexpected error during ECC generation\n");
			return -EINVAL;
		}

		memset(aTemporaryReadEccBuf, 0xFF, SECTOR_SIZE);
		memcpy(aTemporaryReadEccBuf, spare, sizeof(SpareData));

		ecc_generate(ECCType, 1, aTemporaryReadEccBuf, aTemporarySBuf + sizeof(SpareData) + TotalECCDataSize);
	}

	writel(((WEHighHoldTime & FMCTRL_TWH_MASK) << FMCTRL_TWH_SHIFT) | ((WPPulseTime & FMCTRL_TWP_MASK) << FMCTRL_TWP_SHIFT)
		| (1 << (banksTable[bank] + 1)) | FMCTRL0_ON | FMCTRL0_WPB, NAND + FMCTRL0);

	writel(0x80, NAND + NAND_CMD);
	if(wait_for_ready(500) != 0) {
		LOG("nand: bank setting failed\n");
		goto FIL_write_error;
	}

	writel(FMANUM_TRANSFERSETTING, NAND + FMANUM);

	if(buffer) {
		writel(page << 16, NAND + FMADDR0); // lower bits of the page number to the upper bits of CONFIG3
		writel((page >> 16) & 0xFF, NAND + FMADDR1); // upper bits of the page number
	} else {
		writel((page << 16) | Geometry.bytesPerPage, NAND + FMADDR0); // lower bits of the page number to the upper bits of CONFIG3
		writel((page >> 16) & 0xFF, NAND + FMADDR1); // upper bits of the page number	
	}

	writel(FMCTRL1_DOTRANSADDR, NAND + FMCTRL1);
	if(wait_for_address_done(500) != 0) {
		LOG("nand: setup transfer failed\n");
		goto FIL_write_error;
	}

	if(buffer) {
		if(transferToFlash(buffer, Geometry.bytesPerPage) != 0) {
			LOG("nand: transferToFlash failed\n");
			goto FIL_write_error;
		}
	}

	if(transferToFlash(aTemporarySBuf, Geometry.bytesPerSpare) != 0) {
		LOG("nand: transferToFlash for spare failed\n");
		goto FIL_write_error;
	}

	writel(0x10, NAND + NAND_CMD);
	wait_for_ready(500);

	while((nand_read_status() & (1 << 6)) == 0);

	if(nand_read_status() & 0x1)
		return -1;
	else
		return 0;

	return 0;

FIL_write_error:
	nand_bank_reset(bank, 100);
	return -EIO;
}

int nand_bank_reset(int bank, int timeout)
{
	int ret;

	writel(((WEHighHoldTime & FMCTRL_TWH_MASK) << FMCTRL_TWH_SHIFT) | ((WPPulseTime & FMCTRL_TWP_MASK) << FMCTRL_TWP_SHIFT)
			| (1 << (banksTable[bank] + 1)) | FMCTRL0_ON | FMCTRL0_WPB, NAND + FMCTRL0);

	writel(NAND_CMD_RESET, NAND + NAND_CMD);

	ret = wait_for_ready(timeout);
	if(ret == 0) {
		ret = wait_for_command_done(bank, timeout);
		msleep(1);
		return ret;
	} else {
		msleep(1);
		return ret;
	}
}

NANDFTLData* nand_get_ftl_data(void)
{
	return &FTLData;
}

NANDData* nand_get_geometry(void)
{
	return &Geometry;
}

int nand_setup(void)
{
	int bank;
	const NANDDeviceType* nandType = NULL;
	const NANDDeviceType* candidate;
	u32 id;

	WEHighHoldTime = 7;
	WPPulseTime = 7;

	LOG("nand: Probing flash controller...\n");

	iphone_clock_gate_switch(NAND_CLOCK_GATE1, true);
	iphone_clock_gate_switch(NAND_CLOCK_GATE2, true);

	for(bank = 0; bank < NAND_NUM_BANKS; bank++) {
		banksTable[bank] = bank;
	}

	NumValidBanks = 0;

	writel(0, NAND + RSCTRL);
	writel(readl(NAND + RSCTRL) | (ECCType << 4), NAND + RSCTRL);

	for(bank = 0; bank < NAND_NUM_BANKS; bank++) {
		nand_bank_reset(bank, 100);

		writel(FMCTRL1_FLUSHFIFOS, NAND + FMCTRL1);
		writel(((WEHighHoldTime & FMCTRL_TWH_MASK) << FMCTRL_TWH_SHIFT) | ((WPPulseTime & FMCTRL_TWP_MASK) << FMCTRL_TWP_SHIFT)
			| (1 << (banksTable[bank] + 1)) | FMCTRL0_ON | FMCTRL0_WPB, NAND + FMCTRL0);

		writel(NAND_CMD_ID, NAND + NAND_CMD);

		wait_for_ready(500);

		writel(0, NAND + FMANUM);
		writel(0, NAND + FMADDR0);
		writel(FMCTRL1_DOTRANSADDR, NAND + FMCTRL1);

		wait_for_address_done(500);
		wait_for_command_done(bank, 100);

		writel(8, NAND + FMDNUM);
		writel(FMCTRL1_DOREADDATA, NAND + FMCTRL1);

		wait_for_transfer_done(500);
		id = readl(NAND + FMFIFO);
		candidate = SupportedDevices;
		while(candidate->id != 0) {
			if(candidate->id == id) {
				if(nandType == NULL) {
					nandType = candidate;
				} else if(nandType != candidate) {
					LOG("nand: Mismatched device IDs (0x%08x after 0x%08x)\n", id, nandType->id);
					return -EINVAL;
				}
				banksTable[NumValidBanks++] = bank;
			}
			candidate++;
		}

		writel(FMCTRL1_FLUSHFIFOS, NAND + FMCTRL1);
	}

	if(nandType == NULL) {
		LOG("nand: No supported NAND found\n");
		return -EINVAL;
	}

	Geometry.DeviceID = nandType->id;
	Geometry.banksTable = banksTable;

	WPPulseTime = (((FREQUENCY_BUS * (nandType->WPPulseTime + 1)) + 99999999)/100000000) - 1;
	WEHighHoldTime = (((FREQUENCY_BUS * (nandType->WEHighHoldTime + 1)) + 99999999)/100000000) - 1;

	if(WPPulseTime > 7)
		WPPulseTime = 7;

	if(WEHighHoldTime > 7)
		WEHighHoldTime = 7;

	Geometry.blocksPerBank = nandType->blocksPerBank;
	Geometry.banksTotal = NumValidBanks;
	Geometry.sectorsPerPage = nandType->sectorsPerPage;
	Geometry.userSuBlksTotal = nandType->userSuBlksTotal;
	Geometry.bytesPerSpare = nandType->bytesPerSpare;
	Geometry.field_2E = 4;
	Geometry.field_2F = 3;
	Geometry.pagesPerBlock = nandType->pagesPerBlock;

	if(Geometry.sectorsPerPage > 4) {
		LargePages = true;
	} else {
		LargePages = false;
	}

	if(nandType->ecc1 == 6) {
		ECCType = 4;
		TotalECCDataSize = Geometry.sectorsPerPage * 15;
	} else if(nandType->ecc1 == 8) {
		ECCType = 8;
		TotalECCDataSize = Geometry.sectorsPerPage * 20;
	} else if(nandType->ecc1 == 4) {
		ECCType = 0;
		TotalECCDataSize = Geometry.sectorsPerPage * 10;
	}

	if(nandType->ecc2 == 6) {
		ECCType2 = 4;
	} else if(nandType->ecc2 == 8) {
		ECCType2 = 8;
	} else if(nandType->ecc2 == 4) {
		ECCType2 = 0;
	}

	Geometry.field_4 = 5;
	Geometry.bytesPerPage = SECTOR_SIZE * Geometry.sectorsPerPage;
	Geometry.pagesPerBank = Geometry.pagesPerBlock * Geometry.blocksPerBank;
	Geometry.pagesTotal = Geometry.pagesPerBank * Geometry.banksTotal;
	Geometry.pagesPerSuBlk = Geometry.pagesPerBlock * Geometry.banksTotal;
	Geometry.userPagesTotal = Geometry.userSuBlksTotal * Geometry.pagesPerSuBlk;
	Geometry.suBlksTotal = (Geometry.banksTotal * Geometry.blocksPerBank) / Geometry.banksTotal;

	FTLData.field_2 = Geometry.suBlksTotal - Geometry.userSuBlksTotal - 28;
	FTLData.sysSuBlks = FTLData.field_2 + 4;
	FTLData.field_4 = FTLData.field_2 + 5;
	FTLData.field_6 = 3;
	FTLData.field_8 = 23;
	if(FTLData.field_8 == 0)
		Geometry.field_22 = 0;

	{
		int bits = 0;
		int i = FTLData.field_8;
		while((i <<= 1) != 0) {
			bits++;
		}

		Geometry.field_22 = bits;
	}

	LOG("nand: DEVICE: %08x\n", Geometry.DeviceID);
	LOG("nand: BANKS_TOTAL: %d\n", Geometry.banksTotal);
	LOG("nand: BLOCKS_PER_BANK: %d\n", Geometry.blocksPerBank);
	LOG("nand: SUBLKS_TOTAL: %d\n", Geometry.suBlksTotal);
	LOG("nand: USER_SUBLKS_TOTAL: %d\n", Geometry.userSuBlksTotal);
	LOG("nand: PAGES_PER_SUBLK: %d\n", Geometry.pagesPerSuBlk);
	LOG("nand: PAGES_PER_BANK: %d\n", Geometry.pagesPerBank);
	LOG("nand: SECTORS_PER_PAGE: %d\n", Geometry.sectorsPerPage);
	LOG("nand: BYTES_PER_SPARE: %d\n", Geometry.bytesPerSpare);
	LOG("nand: BYTES_PER_PAGE: %d\n", Geometry.bytesPerPage);
	LOG("nand: PAGES_PER_BLOCK: %d\n", Geometry.pagesPerBlock);

	aTemporaryReadEccBuf = (uint8_t*) kmalloc(Geometry.bytesPerPage, GFP_KERNEL | GFP_DMA);
	memset(aTemporaryReadEccBuf, 0xFF, SECTOR_SIZE);

	aTemporarySBuf = (uint8_t*) kmalloc(Geometry.bytesPerSpare, GFP_KERNEL | GFP_DMA);

	return 0;
}

