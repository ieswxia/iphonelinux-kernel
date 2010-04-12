#ifndef IPHONE_DMA_H
#define IPHONE_DMA_H

typedef enum SPIOption13 {
	SPIOption13Setting0 = 8,
	SPIOption13Setting1 = 16,
	SPIOption13Setting2 = 32
} SPIOption13;

void iphone_spi_set_baud(int port, int baud, SPIOption13 option13, bool isMaster, bool isActiveLow, bool lastClockEdgeMissing);
int iphone_spi_tx(int port, const u8* buffer, int len, bool block, bool unknown);
int iphone_spi_rx(int port, u8* buffer, int len, bool block, bool noTransmitJunk);
int iphone_spi_txrx(int port, const u8* outBuffer, int outLen, u8* inBuffer, int inLen, bool block);

#endif

