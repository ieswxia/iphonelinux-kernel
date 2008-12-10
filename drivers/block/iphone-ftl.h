#ifndef FTL_H
#define FTL_H

#define ERROR_INPUT 0x80030000

typedef struct VFLCxt {
	u32 field_0;				// 0x000
	u16 field_4[3];				// 0x004
	u8 unk1[0x10];				// 0x00A
	u16 numReservedBlocks;			// 0x01A
	u16 reservedBlockPoolStart;		// 0x01C
	u16 field_1E;				// 0x01E
	u16 reservedBlockPoolMap[0x334];		// 0x020
	u8 badBlockTable[0x11a];			// 0x688
	u16 VFLCxtBlock[4];			// 0x7A2
	u8 unk3[0x4E];				// 0x7AA
	u32 checksum1;				// 0x7F8
	u32 checksum2;				// 0x7FC
} VFLCxt;

typedef struct FTLCxtLog {
	void* field_0;					// 0x0
	uint16_t wVbn;					// 0x4
	uint16_t wLbn;					// 0x6
	uint16_t* wPageOffsets;				// 0x8
	uint16_t field_C;				// 0xC
	uint16_t field_E;				// 0xE
	uint32_t field_10;				// 0x10
} FTLCxtLog;

typedef struct FTLCxtElement2 {
	u16 field_0;				// 0x0
	u16 field_2;				// 0x2
} FTLCxtElement2;

typedef struct FTLCxt {
	uint32_t unk0;					// 0x0
	uint32_t unk1;					// 0x4
	uint16_t wNumOfFreeVb;				// 0x8
	uint16_t wUnk2;					// 0xA
	uint16_t wUnk3;					// 0xC
	uint16_t awUnkBlockList1[4];			// 0xE
	uint16_t awFreeVb[10];				// 0x16
	uint16_t awUnkBlockList2[7];			// 0x2A
	uint32_t pages_for_pawMapTable[18];		// 0x38
	uint32_t pages_for_pawEraseCounterTable[36];	// 0x80
	uint32_t pages_for_wPageOffsets[34];		// 0x110
	uint16_t* pawMapTable;				// 0x198
	uint16_t* pawEraseCounterTable;			// 0x19C
	uint16_t* wPageOffsets;				// 0x1A0
	FTLCxtLog pLog[18];				// 0x1A4
	uint8_t unk2[6];				// 0x30C
	uint16_t thing[3];				// 0x312
	uint32_t field_318;				// 0x318
	uint32_t field_31C;				// 0x31C
	uint32_t pages_for_pawReadCounterTable[36];	// 0x320
	uint16_t* pawReadCounterTable;			// 0x3B0
	FTLCxtElement2 elements2[5];			// 0x3B4
	uint32_t field_3C8;				// 0x3C8
	uint32_t field_3CC;				// 0x3CC
	uint32_t field_3D0;				// 0x3D0
	uint32_t field_3D4;				// 0x3D4
	uint8_t field_3D8[0x420];			// 0x3D8
	uint32_t versionLower;				// 0x7F8
	uint32_t versionUpper;				// 0x7FC
} FTLCxt;

typedef struct VFLData1Type {
	u64 field_0;
	u64 field_8;
	u64 field_10;
	u64 field_18;
	u64 field_20;
	u64 field_28;
	u64 field_30;
	u64 field_38;
	u64 field_40;
} VFLData1Type;

typedef struct FTLData1Type {
	u64 field_0;
	u64 field_8;
	u64 field_10;
	u64 field_18;
	u64 field_20;
	u64 field_28;
	u64 field_30;
	u64 field_38;
	u64 field_40;
	u64 field_48;
	u64 field_50;
} FTLData1Type;

typedef enum FTLStruct {
	FTLData1SID = 0x1000200
} FTLStruct;

typedef enum VFLStruct {
	VFLData1SID = 0x2000200,
	VFLData5SID = 0x2000500
} VFLStruct;

extern int HasFTLInit;

#endif
