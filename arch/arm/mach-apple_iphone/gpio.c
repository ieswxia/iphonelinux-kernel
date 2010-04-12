#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/sysdev.h>
#include <linux/interrupt.h>
#include <linux/io.h>

#include <mach/iphone-clock.h>
#include <mach/gpio.h>

#define GET_BITS(x, start, length) ((((u32)(x)) << (32 - ((start) + (length)))) >> (32 - (length)))

#define GPIO_INTTYPE_MASK 0x1
#define GPIO_INTTYPE_EDGE 0x0
#define GPIO_INTTYPE_LEVEL GPIO_INTTYPE_MASK

#define GPIO_INTLEVEL_MASK 0x2
#define GPIO_INTLEVEL_LOW 0x0
#define GPIO_INTLEVEL_HIGH GPIO_INTLEVEL_MASK

#define GPIO_AUTOFLIP_MASK 0x4
#define GPIO_AUTOFLIP_NO 0x0
#define GPIO_AUTOFLIP_YES GPIO_AUTOFLIP_MASK

typedef struct {
	u32 flags[32];
	void* token[32];
	irq_handler_t handler[32];
} GPIOInterruptGroup;

GPIOInterruptGroup InterruptGroups[GPIO_NUMINTGROUPS];

static GPIORegisters* GPIORegs = (GPIORegisters*) GPIO;

static irqreturn_t gpio_handle_interrupt(int irq, void* pToken);

static int iphone_gpio_setup(void) {
	int i;
	int ret;

	for(i = 0; i < GPIO_NUMINTGROUPS; i++) {
		// writes to all the interrupt status register to acknowledge and discard any pending
		writel(GPIO_INTSTAT_RESET, GPIOIC + GPIO_INTSTAT + (i * 0x4));

		// disable all interrupts
		writel(GPIO_INTEN_RESET, GPIOIC + GPIO_INTEN + (i * 0x4));
	}

	memset(InterruptGroups, 0, sizeof(InterruptGroups));

        ret = request_irq(0x21, gpio_handle_interrupt, IRQF_DISABLED, "iphone_gpio", (void*) 0);
	if(ret)
		return ret;

        ret = request_irq(0x20, gpio_handle_interrupt, IRQF_DISABLED, "iphone_gpio", (void*) 1);
	if(ret)
		return ret;

        ret = request_irq(0x1f, gpio_handle_interrupt, IRQF_DISABLED, "iphone_gpio", (void*) 2);
	if(ret)
		return ret;

        ret = request_irq(0x03, gpio_handle_interrupt, IRQF_DISABLED, "iphone_gpio", (void*) 3);
	if(ret)
		return ret;

        ret = request_irq(0x02, gpio_handle_interrupt, IRQF_DISABLED, "iphone_gpio", (void*) 4);
	if(ret)
		return ret;

        ret = request_irq(0x01, gpio_handle_interrupt, IRQF_DISABLED, "iphone_gpio", (void*) 5);
	if(ret)
		return ret;

        ret = request_irq(0x00, gpio_handle_interrupt, IRQF_DISABLED, "iphone_gpio", (void*) 6);
	if(ret)
		return ret;

	// iBoot also sets up interrupt handlers, but those are never unmasked
	
	iphone_clock_gate_switch(GPIO_CLOCKGATE, 1);

	return 0;
}

module_init(iphone_gpio_setup);

int iphone_gpio_pin_state(int port) {
	return ((GPIORegs[GET_BITS(port, 8, 5)].DAT & (1 << GET_BITS(port, 0, 3))) != 0);
}

void iphone_gpio_custom_io(int port, int bits) {
	writel(((GET_BITS(port, 8, 5) & GPIO_IO_MAJMASK) << GPIO_IO_MAJSHIFT)
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

void iphone_gpio_register_interrupt(u32 interrupt, int type, int level, int autoflip, irq_handler_t handler, void* token)
{
	int group = interrupt >> 5;
	int index = interrupt & 0x1f;
	int mask = ~(1 << index);
	unsigned long flags;

	local_irq_save(flags);

	InterruptGroups[group].flags[index] = (type ? GPIO_INTTYPE_LEVEL : 0) | (level ? GPIO_INTLEVEL_HIGH : 0) | (level ? GPIO_AUTOFLIP_YES : 0);
	InterruptGroups[group].handler[index] = handler;
	InterruptGroups[group].token[index] = token;

	// set whether or not the interrupt is level or edge
	writel((readl(GPIOIC + GPIO_INTTYPE + (0x4 * group)) & mask) | ((type ? 1 : 0) << index), GPIOIC + GPIO_INTTYPE + (0x4 * group));

	// set whether to trigger the interrupt high or low
	writel((readl(GPIOIC + GPIO_INTLEVEL + (0x4 * group)) & mask) | ((level ? 1 : 0) << index), GPIOIC + GPIO_INTLEVEL + (0x4 * group));
	local_irq_restore(flags);
}

void iphone_gpio_interrupt_enable(u32 interrupt)
{
	int group = interrupt >> 5;
	int index = interrupt & 0x1f;
	unsigned long flags;

	local_irq_save(flags);
	writel(1 << index, GPIOIC + GPIO_INTSTAT + (0x4 * group));
	writel(readl(GPIOIC + GPIO_INTEN + (0x4 * group)) | (1 << index), GPIOIC + GPIO_INTEN + (0x4 * group));
	local_irq_restore(flags);
}

void iphone_gpio_interrupt_disable(u32 interrupt)
{
	int group = interrupt >> 5;
	int index = interrupt & 0x1f;
	int mask = ~(1 << index);
	unsigned long flags;

	local_irq_save(flags);
	writel(readl(GPIOIC + GPIO_INTEN + (0x4 * group)) & mask, GPIOIC + GPIO_INTEN + (0x4 * group));
	local_irq_restore(flags);
}

static irqreturn_t gpio_handle_interrupt(int irq, void* pToken)
{
	u32 token = (u32) pToken;
	u32 statReg = GPIOIC + GPIO_INTSTAT + (0x4 * token);

	int stat;

	while((stat = readl(statReg)) != 0)
	{
		int i;
		for(i = 0; i < 32; ++i)
		{
			if(stat & 1)
			{
				u32 flags = InterruptGroups[token].flags[i];

				if((flags & GPIO_INTTYPE_MASK) == GPIO_INTTYPE_EDGE)
					writel(1 << i, statReg);

				if((flags & GPIO_AUTOFLIP_MASK) == GPIO_AUTOFLIP_YES)
				{
					// flip the corresponding bit in GPIO_INTLEVEL
					writel(readl(GPIOIC + GPIO_INTLEVEL + (0x4 * token)) ^ (1 << i), GPIOIC + GPIO_INTLEVEL + (0x4 * token));

					// flip the bit in the flags too
					InterruptGroups[token].flags[i] = flags ^ GPIO_INTLEVEL_MASK;
				}

				if(InterruptGroups[token].handler[i])
				{
					InterruptGroups[token].handler[i]((token << 5) | i, InterruptGroups[token].token[i]);
				}

				writel(1 << i, statReg);
			}

			stat >>= 1;
		}
	}

	return IRQ_HANDLED;
}

