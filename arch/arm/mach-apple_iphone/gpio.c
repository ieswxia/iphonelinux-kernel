#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/sysdev.h>
#include <linux/io.h>

#include <mach/iphone-clock.h>
#include <mach/gpio.h>
#include "gpio.h"

#define GET_BITS(x, start, length) ((((u32)(x)) << (32 - ((start) + (length)))) >> (32 - (length)))

static GPIORegisters* GPIORegs = (GPIORegisters*) GPIO;

static int iphone_gpio_setup(void) {
	int i;

	for(i = 0; i < NUM_GPIO; i++) {
		__raw_writel(POWER_GPIO_CONFIG0_RESET, GPIO_POWER + POWER_GPIO_CONFIG0);
		__raw_writel(POWER_GPIO_CONFIG1_RESET, GPIO_POWER + POWER_GPIO_CONFIG1);
	}

	// iBoot also sets up interrupt handlers, but those are never unmasked
	
	iphone_clock_gate_switch(GPIO_CLOCKGATE, 1);

	return 0;
}

module_init(iphone_gpio_setup);

int iphone_gpio_pin_state(int port) {
	return ((GPIORegs[GET_BITS(port, 8, 5)].DAT & (1 << GET_BITS(port, 0, 3))) != 0);
}

void iphone_gpio_custom_io(int port, int bits) {
	__raw_writel(((GET_BITS(port, 8, 5) & GPIO_IO_MAJMASK) << GPIO_IO_MAJSHIFT)
				| ((GET_BITS(port, 0, 3) & GPIO_IO_MINMASK) << GPIO_IO_MINSHIFT)
				| ((bits & GPIO_IO_UMASK) << GPIO_IO_USHIFT), GPIO + GPIO_IO);
}

void iphone_gpio_pin_reset(int port) {
	iphone_gpio_custom_io(port, 0);
}

void iphone_gpio_pin_output(int port, int bit) {
	iphone_gpio_custom_io(port, 0xE | bit); // 0b111U, where U is the argument
}

int iphone_gpio_detect_configuration(void) {
	static int hasDetected = 0;
	static int detectedConfig = 0;

	if(hasDetected) {
		return detectedConfig;
	}

	detectedConfig = (iphone_gpio_pin_state(GPIO_DETECT3) ? 1 : 0) | ((iphone_gpio_pin_state(GPIO_DETECT2) ? 1 : 0) << 1) | ((iphone_gpio_pin_state(GPIO_DETECT1) ? 1 : 0) << 2);
	hasDetected = 1;
	return detectedConfig;
}

