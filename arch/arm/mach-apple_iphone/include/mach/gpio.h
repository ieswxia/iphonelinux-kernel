#ifndef IPHONE_HW_GPIO_H
#define IPHONE_HW_GPIO_H

#include <mach/hardware.h>

// Device
#define GPIO_POWER IO_ADDRESS(0x39A00000)	/* probably a part of the system controller */
#define GPIO IO_ADDRESS(0x3E400000)

// Registers
#define POWER_GPIO_CONFIG0 0xA0
#define POWER_GPIO_CONFIG1 0xC0
#define GPIO_IO 0x320

// Values
#define NUM_GPIO 7
#define POWER_GPIO_CONFIG0_RESET 0
#define POWER_GPIO_CONFIG1_RESET 0xFFFFFFFF

#define GPIO_IO_MAJSHIFT 16
#define GPIO_IO_MAJMASK 0x1F
#define GPIO_IO_MINSHIFT 8
#define GPIO_IO_MINMASK 0x7
#define GPIO_IO_USHIFT 0
#define GPIO_IO_UMASK 0xF

#define GPIO_CLOCKGATE 0x2C

#define GPIO_DETECT1 0xE04
#define GPIO_DETECT2 0xE05
#define GPIO_DETECT3 0xE06

#endif

