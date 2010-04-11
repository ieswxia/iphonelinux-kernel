#ifndef IPHONE_DMA_H
#define IPHONE_DMA_H
#include <linux/sched.h>

#define ERROR_DMA 0x13
#define ERROR_BUSY 0x15
#define ERROR_ALIGN 0x9

typedef volatile struct DMARequest {
	bool started;
	bool done;
	struct task_struct* task;
	// TODO: fill this thing out
} DMARequest;

typedef struct DMALinkedList {
    u32 source;	
    u32 destination;
    struct DMALinkedList* next;
    u32 control;
} DMALinkedList;

#define IPHONE_DMA_I2S0_RX 19
#define IPHONE_DMA_I2S0_TX 20
#define IPHONE_DMA_I2S1_RX 14
#define IPHONE_DMA_I2S1_TX 15
#define IPHONE_DMA_MEMORY 25
#define IPHONE_DMA_NAND 8

extern struct platform_device iphone_dma;

int iphone_dma_setup(void);
int iphone_dma_request(int Source, int SourceTransferWidth, int SourceBurstSize, int Destination, int DestinationTransferWidth, int DestinationBurstSize, int* controller, int* channel);
int iphone_dma_prepare(dma_addr_t Source, dma_addr_t Destination, int size, dma_addr_t continueList, int* controller, int* channel);
int iphone_dma_perform(dma_addr_t Source, dma_addr_t Destination, int size, dma_addr_t continueList, int* controller, int* channel);
int iphone_dma_finish(int controller, int channel, int timeout);
dma_addr_t iphone_dma_dstpos(int controller, int channel);
dma_addr_t iphone_dma_srcpos(int controller, int channel);
void iphone_dma_resume(int controller, int channel);
void iphone_dma_pause(int controller, int channel);

#endif

