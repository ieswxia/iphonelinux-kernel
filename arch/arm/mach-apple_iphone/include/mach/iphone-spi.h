#ifndef IPHONE_SPI_H
#define IPHONE_SPI_H

#define GPIO_SPI0_CS0_IPHONE 0x400
#define GPIO_SPI0_CS0_IPOD 0x700

#ifdef CONFIG_IPOD
#define GPIO_SPI2_CS0 0x1804
#define GPIO_SPI2_CS1 0x705
#endif

#ifdef CONFIG_IPHONE
#define GPIO_SPI2_CS0 0x705
#endif

#ifdef CONFIG_IPOD
#define GPIO_SPI0_CS0 GPIO_SPI0_CS0_IPOD
#else
#define GPIO_SPI0_CS0 GPIO_SPI0_CS0_IPHONE
#endif

#define GPIO_SPI1_CS0 0x1800

#ifdef CONFIG_3G
#define GPIO_SPI0_CS1 0x705
#define GPIO_SPI0_CS2 0x706
#endif

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

