#include <linux/init.h>
#include <linux/device.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include "clock.h"

#define POWER IO_ADDRESS(0x39A00000)	/* probably a part of the system controller */

#define POWER_ONCTRL 0xC
#define POWER_OFFCTRL 0x10
#define POWER_SETSTATE 0x8
#define POWER_STATE 0x14

int iphone_power_ctrl(u32 device, int on_off) {
	if(on_off) {
		__raw_writel(device, POWER + POWER_ONCTRL);
	} else {
		__raw_writel(device, POWER + POWER_OFFCTRL);
	}

	/* wait for the new state to take effect */
	while((__raw_readl(POWER + POWER_SETSTATE) & device) != (__raw_readl(POWER + POWER_STATE) & device));

	return 0;
}
