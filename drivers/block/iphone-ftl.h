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
	u16 wVbn;					// 0x4
	u16 field_6;				// 0x6
	u16* field_8;				// 0x8
	u16 field_C;				// 0xC
	u16 field_E;				// 0xE
	u32 field_10;				// 0x10
} FTLCxtLog;

typedef struct FTLCxtElement2 {
	u16 field_0;				// 0x0
	u16 field_2;				// 0x2
} FTLCxtElement2;

typedef struct FTLCxt {
	u8 unk1[0x38];				// 0x0
	u32 pages_for_dataVbn[18];			// 0x38
	u32 pages_for_19C[36];			// 0x80
	u32 pages_for_1A0[34];			// 0x110
	u16* dataVbn;				// 0x198
	u16* field_19C;				// 0x19C
	u16* field_1A0;				// 0x1A0
	FTLCxtLog pLog[18];				// 0x1A4
	u8 unk2[6];				// 0x30C
	u16 thing[3];				// 0x312
	u32 field_318;				// 0x318
	u32 field_31C;				// 0x31C
	u32 pages_for_3B0[36];			// 0x320
	u16* field_3B0;				// 0x3B0
	FTLCxtElement2 elements2[5];			// 0x3B4
	u32 field_3C8;				// 0x3C8
	u32 field_3CC;				// 0x3CC
	u32 field_3D0;				// 0x3D0
	u32 field_3D4;				// 0x3D4
	u8 field_3D8[0x420];			// 0x3D8
	u32 versionLower;				// 0x7F8
	u32 versionUpper;				// 0x7FC
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
