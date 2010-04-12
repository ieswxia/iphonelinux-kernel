#ifndef IPHONE_HW_GPIO_H
#define IPHONE_HW_GPIO_H

#include <linux/interrupt.h>
#include <mach/hardware.h>

// Device
#define GPIOIC IO_ADDRESS(0x39A00000)	/* probably a part of the system controller */
#define GPIO IO_ADDRESS(0x3E400000)

// Registers
#define GPIO_INTLEVEL 0x80
#define GPIO_INTSTAT 0xA0
#define GPIO_INTEN 0xC0
#define GPIO_INTTYPE 0xE0
#define GPIO_FSEL 0x320

#define GPIO_IO 0x320

// Values
#define GPIO_NUMINTGROUPS 7
#define GPIO_INTSTAT_RESET 0xFFFFFFFF
#define GPIO_INTEN_RESET 0

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

typedef struct GPIORegisters {
	volatile uint32_t CON;
	volatile uint32_t DAT;
	volatile uint32_t PUD;
	volatile uint32_t CONSLP;
	volatile uint32_t PUDSLP;
	volatile uint32_t unused1;
	volatile uint32_t unused2;
	volatile uint32_t unused3;
} GPIORegisters;

int iphone_gpio_pin_state(int port);
void iphone_gpio_custom_io(int port, int bits);
void iphone_gpio_pin_reset(int port);
void iphone_gpio_pin_output(int port, int bit);
int iphone_gpio_detect_configuration(void);

void iphone_gpio_register_interrupt(u32 interrupt, int type, int level, int autoflip, irq_handler_t handler, void* token);
void iphone_gpio_interrupt_enable(u32 interrupt);
void iphone_gpio_interrupt_disable(u32 interrupt);

#endif

