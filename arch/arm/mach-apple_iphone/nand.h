#ifndef IPHONE_NAND_H
#define IPHONE_NAND_H

#define ERROR_ARG 0x80010000
#define ERROR_NAND 0x80020000
#define ERROR_TIMEOUT 0x1F
#define ERROR_ECC 0x17
#define ERROR_EMPTYBLOCK 0x1

typedef struct NANDDeviceType {
	u32 id;
	u16 blocksPerBank;
	u16 pagesPerBlock;
	u16 sectorsPerPage;
	u16 bytesPerSpare;
	u8 NANDSetting2;
	u8 NANDSetting1;
	u8 NANDSetting3;
	u8 NANDSetting4;
	u32 userSubBlksTotal;
	u32 ecc1;
	u32 ecc2;
} NANDDeviceType;

typedef struct UnknownNANDType {
	u16 field_0;
	u16 field_2;
	u16 field_4;		// reservoir blocks?
	u16 field_6;
	u16 field_8;
} UnknownNANDType;

typedef struct SpareData {
	u32 logicalPageNumber;
	u8 field_4;
	u8 field_5;
	u8 field_6;
	u8 field_7;
	u8 field_8;
	u8 field_9;
	u8 eccMark;
	u8 field_B;
} __attribute__ ((packed)) SpareData;

typedef struct NANDData {
	u32 field_0;
	u16 field_4;
	u16 sectorsPerPage;
	u16 pagesPerBlock;
	u16 pagesPerSubBlk;
	u32 pagesPerBank;
	u32 pagesTotal;
	u16 subBlksTotal;
	u16 userSubBlksTotal;
	u32 userPagesTotal;
	u16 blocksPerBank;
	u16 bytesPerPage;
	u16 bytesPerSpare;
	u16 field_22;
	u32 field_24;
	u32 DeviceID;
	u16 banksTotal;
	u8 field_2E;
	u8 field_2F;
} NANDData;

#endif
