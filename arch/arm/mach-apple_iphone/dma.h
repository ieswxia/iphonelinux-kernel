#ifndef IPHONE_DMA_H
#define IPHONE_DMA_H
#include <linux/sched.h>

#define ERROR_DMA 0x13
#define ERROR_BUSY 0x15
#define ERROR_ALIGN 0x9

typedef volatile struct DMARequest {
	int started;
	int done;
	struct task_struct* task;
	// TODO: fill this thing out
} DMARequest;

typedef struct DMALinkedList {
    u32 source;	
    u32 destination;
    struct DMALinkedList* next;
    u32 control;
} DMALinkedList;

#define IPHONE_DMA_MEMORY 25
#define IPHONE_DMA_NAND 8

int iphone_dma_setup(void);
int iphone_dma_request(int Source, int SourceTransferWidth, int SourceBurstSize, int Destination, int DestinationTransferWidth, int DestinationBurstSize, int* controller, int* channel);
int iphone_dma_perform(u32 Source, u32 Destination, int size, int continueList, int* controller, int* channel);
int iphone_dma_finish(int controller, int channel, int timeout);

#endif

