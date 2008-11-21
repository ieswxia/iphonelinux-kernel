/*
 *  linux/arch/arm/mach-versatile/versatile_ab.c
 *
 *  Copyright (C) 2004 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/mach/irq.h>

#include "core.h"
extern void printascii(const char* str);
extern void printhex8(uint32_t num);

static struct map_desc iphone_io_desc[] __initdata = {
	{
		.virtual	=  IO_ADDRESS(0x3CC00000),
		.pfn		= __phys_to_pfn(0x3CC00000),
		.length		= SZ_128K,
		.type		= MT_DEVICE
	},
};

void __init iphone_map_io(void)
{
	printascii("iphone_map_io with new weird map\r\n");
	iotable_init(iphone_io_desc, ARRAY_SIZE(iphone_io_desc));
}

void __init iphone_init_irq(void)
{
	printascii("iphone_init_irq\r\n");
}

static void __init iphone_timer_init(void)
{
	printascii("iphone_timer_init\r\n");
}

void __init iphone_init(void)
{
	printascii("iphone_init\r\n");
}

struct sys_timer iphone_timer = {
	.init		= iphone_timer_init,
};

MACHINE_START(APPLE_IPHONE, "Apple iPhone")
	/* Maintainer: iPhone Linux */
	.phys_io	= 0x38000000,
	.io_pg_offst	= (IO_ADDRESS(0x38000000) >> 18) & 0xfffc,
/*	.io_pg_offst	= (0xF0000000  >> 18) & 0xfffc,*/
	.boot_params	= 0x09000000,
	.map_io		= iphone_map_io,
	.init_irq	= iphone_init_irq,
	.timer		= &iphone_timer,
	.init_machine	= iphone_init,
MACHINE_END
