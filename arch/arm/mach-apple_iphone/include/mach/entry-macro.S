/*
 * arch/arm/mach-apple_iphone/include/mach/entry-macro.S
 *
 * Low-level IRQ helper macros for the Apple iPhone
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <mach/hardware.h>

		.macro	disable_fiq
		.endm

		.macro  get_irqnr_preamble, base, tmp
		.endm

		.macro  arch_ret_to_user, tmp1, tmp2
		.endm

		.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp
		mov	\tmp, #0
		ldr	\base, =0x38E00000
		ldr	\irqstat, [\base, #0]           @ get masked status
		cmp	\irqstat, \tmp
		addeq	\base, \base, #0x100000
		ldreq	\irqstat, [\base, #0]           @ get masked status
		cmpeq	\irqstat, #0
		ldr	\irqnr, [\base, #0xF00]         @ read for VICADDR[01]
		.endm

		.macro irq_prio_table
		.endm
