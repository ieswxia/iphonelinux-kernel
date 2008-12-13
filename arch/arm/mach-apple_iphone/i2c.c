#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include "i2c.h"
#include "gpio.h"
#include <mach/iphone-clock.h>
#include <mach/hardware.h>
#include <mach/i2c.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/sysdev.h>
#include <linux/io.h>

static const I2CInfo I2CInit[] = {
			{0, 0, I2CNoError, 0, 0, I2CDone, 0, 0, 0, 0, 0, NULL, 0, NULL, I2C0_SCL_GPIO, I2C0_SDA_GPIO, I2C0 + IICCON, I2C0 + IICSTAT, I2C0 + IICADD, I2C0 + IICDS, I2C0 + IICLC,
				I2C0 + IICREG14, I2C0 + IICREG18, I2C0 + IICREG1C, I2C0 + IICREG20},
			{0, 0, I2CNoError, 0, 0, I2CDone, 0, 0, 0, 0, 0, NULL, 0, NULL, I2C1_SCL_GPIO, I2C1_SDA_GPIO, I2C1 + IICCON, I2C1 + IICSTAT, I2C1 + IICADD, I2C1 + IICDS, I2C1 + IICLC,
				I2C1 + IICREG14, I2C1 + IICREG18, I2C1 + IICREG1C, I2C1 + IICREG20}
		};

static I2CInfo I2C[2];

static void init_i2c(I2CInfo* i2c, u32 frequency);
static I2CError iphone_i2c_readwrite(I2CInfo* i2c);
static void do_i2c(I2CInfo* i2c);

int __init iphone_i2c_setup(void) {
	memcpy(I2C, I2CInit, sizeof(I2CInfo) * 2);

	iphone_clock_gate_switch(I2C0_CLOCKGATE, 1);
	iphone_clock_gate_switch(I2C1_CLOCKGATE, 1);

	init_i2c(&I2C[0], FREQUENCY_PERIPHERAL / 100000);
	init_i2c(&I2C[1], FREQUENCY_PERIPHERAL / 100000);

	return 0;
}
module_init(iphone_i2c_setup);

I2CError iphone_i2c_rx(int bus, int iicaddr, const uint8_t* registers, int num_regs, void* buffer, int len) {
	I2C[bus].address = iicaddr;
	I2C[bus].is_write = 0;
	I2C[bus].registers = registers;
	I2C[bus].num_regs = num_regs;
	I2C[bus].bufferLen = len;
	I2C[bus].buffer = (uint8_t*) buffer;
	return iphone_i2c_readwrite(&I2C[bus]);
}

I2CError iphone_i2c_tx(int bus, int iicaddr, void* buffer, int len) {
	I2C[bus].address = iicaddr;
	I2C[bus].is_write = 1;
	I2C[bus].registers = NULL;
	I2C[bus].num_regs = 0;
	I2C[bus].bufferLen = len;
	I2C[bus].buffer = (uint8_t*) buffer;
	return iphone_i2c_readwrite(&I2C[bus]);
}

static void init_i2c(I2CInfo* i2c, u32 frequency) {
	int prescaler, divisorRequired;
	int i;
	i2c->frequency = 2560000 / frequency;
	divisorRequired = 640000 / i2c->frequency;
	if(divisorRequired < 512) {
		// round up
		i2c->iiccon_settings = IICCON_INIT | IICCON_TXCLKSRC_FPCLK16;
		prescaler = ((divisorRequired + 0x1F) >> 5) - 1;
	} else {
		i2c->iiccon_settings = IICCON_INIT | IICCON_TXCLKSRC_FPCLK512;
		prescaler = ((divisorRequired + 0x1FF) >> 9) - 1;
	}

	if(prescaler == 0)
		prescaler = 1;

	i2c->iiccon_settings |= prescaler;

	iphone_gpio_custom_io(i2c->iic_sda_gpio, 0xE); // pull sda low?

	for(i = 0; i < 19; i++) {
		iphone_gpio_custom_io(i2c->iic_scl_gpio, ((i % 2) == 1) ? 0xE : 0x0);
		udelay(5);
	}

	iphone_gpio_custom_io(i2c->iic_scl_gpio, 0x2);	// generate stop condition?
	iphone_gpio_custom_io(i2c->iic_sda_gpio, 0x2);
}

static I2CError iphone_i2c_readwrite(I2CInfo* i2c) {
	__raw_writel(i2c->iiccon_settings, i2c->register_IICCON);

	i2c->iiccon_settings |= IICCON_ACKGEN;
	i2c->state = I2CStart;
	i2c->operation_result = 0;
	i2c->error_code = I2CNoError;

	__raw_writel(IICCON_INIT, i2c->register_20);

	do_i2c(i2c);

	if(i2c->error_code != I2CNoError)
		return i2c->error_code;

	while(1) {
		int hardware_status;
		do {
			hardware_status = __raw_readl(i2c->register_20);
			__raw_writel(hardware_status, i2c->register_20);
			i2c->operation_result &= ~hardware_status;
			udelay(5);
		} while(hardware_status != 0);

		if(i2c->state == I2CDone) {
			break;
		}

		do_i2c(i2c);
	}

	return i2c->error_code;
}

static void do_i2c(I2CInfo* i2c) {
	int proceed = 0;
	while(i2c->operation_result == 0 || proceed) {
		proceed = 0;
		switch(i2c->state) {
			case I2CStart:
				if(i2c->num_regs != 0) {
					__raw_writel(i2c->iiccon_settings, i2c->register_IICCON);
					__raw_writel(i2c->address, i2c->register_IICDS);
					i2c->operation_result = OPERATION_SEND;
					i2c->current_iicstat =
						(IICSTAT_MODE_MASTERTX << IICSTAT_MODE_SHIFT)
						| (IICSTAT_STARTSTOPGEN_START << IICSTAT_STARTSTOPGEN_SHIFT)
						| (1 << IICSTAT_DATAOUTPUT_ENABLE_SHIFT);
					__raw_writel(i2c->current_iicstat, i2c->register_IICSTAT);
					i2c->cursor = 0;
					i2c->state = I2CSendRegister;
				} else {
					i2c->operation_result = 0;
					i2c->state = I2CSetup;
					proceed = 1;
				}
				break;
			case I2CSendRegister:
				if((__raw_readl(i2c->register_IICSTAT) & IICSTAT_LASTRECEIVEDBIT) == 0) {
					// ack received
					if(i2c->cursor != i2c->num_regs) {
						// need to send more from the register list
						i2c->operation_result = OPERATION_SEND;
						__raw_writel(i2c->registers[i2c->cursor++], i2c->register_IICDS);
						__raw_writel(i2c->iiccon_settings | IICCON_INTPENDING, i2c->register_IICCON);
					} else {
						i2c->state = I2CRegistersDone;
						proceed = 1;	
					}
				} else {
					// ack not received
					printk("iphone-i2c: ack not received before register/command %d\n", i2c->cursor);
					i2c->error_code = -1;
					i2c->state = I2CFinish;
					proceed = 1;
				}
				break;
			case I2CRegistersDone:
				if(i2c->is_write) {
					i2c->state = I2CSetup;
					proceed = 1;
				} else {
					i2c->operation_result = OPERATION_CONDITIONCHANGE;
					i2c->current_iicstat &= (IICSTAT_MODE_SLAVETX << IICSTAT_MODE_SHIFT) | (IICSTAT_STARTSTOPGEN_MASK << IICSTAT_STARTSTOPGEN_SHIFT);
					__raw_writel(i2c->current_iicstat, i2c->register_IICSTAT);
					__raw_writel(i2c->iiccon_settings | IICCON_INTPENDING, i2c->register_IICCON);
					i2c->state = I2CSetup;
				}
				break;
			case I2CSetup:
				__raw_writel(i2c->iiccon_settings | IICCON_ACKGEN, i2c->register_IICCON);
				__raw_writel(i2c->address, i2c->register_IICDS);
				i2c->operation_result = OPERATION_SEND;
				if(i2c->is_write) {
					i2c->current_iicstat =
						(IICSTAT_MODE_MASTERTX << IICSTAT_MODE_SHIFT)
						| (IICSTAT_STARTSTOPGEN_START << IICSTAT_STARTSTOPGEN_SHIFT)
						| (1 << IICSTAT_DATAOUTPUT_ENABLE_SHIFT);
				} else {
					i2c->current_iicstat =
						(IICSTAT_MODE_MASTERRX << IICSTAT_MODE_SHIFT)
						| (IICSTAT_STARTSTOPGEN_START << IICSTAT_STARTSTOPGEN_SHIFT)
						| (1 << IICSTAT_DATAOUTPUT_ENABLE_SHIFT);
				}
				__raw_writel(i2c->current_iicstat, i2c->register_IICSTAT);
				i2c->cursor = 0;
				if(i2c->is_write) {
					i2c->state = I2CTx;
				} else {
					i2c->state = I2CRxSetup;
				}
				break;
			case I2CTx:
				if((__raw_readl(i2c->register_IICSTAT) & IICSTAT_LASTRECEIVEDBIT) == 0) {
					if(i2c->cursor != i2c->bufferLen) {
						// need to send more from the register list
						i2c->operation_result = OPERATION_SEND;
						__raw_writel(i2c->buffer[i2c->cursor++], i2c->register_IICDS);
						__raw_writel(i2c->iiccon_settings | IICCON_INTPENDING | IICCON_ACKGEN, i2c->register_IICCON);
					} else {
						i2c->state = I2CFinish;
						proceed = 1;	
					}
				} else {
					// ack not received
					printk("iphone-i2c: ack not received before byte %d\n", i2c->cursor);
					i2c->error_code = -1;
					i2c->state = I2CFinish;
					proceed = 1;
				}
				break;
			case I2CRx:
				i2c->buffer[i2c->cursor++] = __raw_readl(i2c->register_IICDS);
				// fall into I2CRxSetup
			case I2CRxSetup:
				if(i2c->cursor != 0 || (__raw_readl(i2c->register_IICSTAT) & IICSTAT_LASTRECEIVEDBIT) == 0) {
					if(i2c->cursor != i2c->bufferLen) {
						if((i2c->bufferLen - i2c->cursor) == 1) {
							// last byte
							i2c->iiccon_settings &= ~IICCON_ACKGEN;
						}
						i2c->operation_result = OPERATION_SEND;
						__raw_writel(i2c->iiccon_settings | IICCON_INTPENDING, i2c->register_IICCON);
						i2c->state = I2CRx;
					} else {
						i2c->state = I2CFinish;
					}
				} else {
					i2c->error_code = -1;
					i2c->state = I2CFinish;
					proceed = 1;
				}
				break;
			case I2CFinish:
				i2c->operation_result = OPERATION_CONDITIONCHANGE;
				i2c->current_iicstat &= ~(IICSTAT_STARTSTOPGEN_MASK << IICSTAT_STARTSTOPGEN_SHIFT);
				if(i2c->is_write) {
					// turn off the tx bit in the mode
					i2c->current_iicstat &= ~(IICSTAT_MODE_SLAVETX << IICSTAT_MODE_SHIFT);
				}
				__raw_writel(i2c->current_iicstat, i2c->register_IICSTAT);
				__raw_writel(i2c->iiccon_settings | IICCON_INTPENDING, i2c->register_IICCON);
				i2c->state =I2CDone;
				break;
			case I2CDone:
				// should not occur
				break;
		}
	}
}

