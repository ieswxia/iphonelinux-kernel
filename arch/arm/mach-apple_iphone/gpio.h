#ifndef GPIO_H
#define GPIO_H

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

#endif
