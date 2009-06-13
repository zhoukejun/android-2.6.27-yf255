/*
 *  arch/arm/mach-pxa/include/mach/yf255.h //zkj copy lubbock.h
 *
 *  Author:	Nicolas Pitre
 *  Created:	Jun 15, 2001
 *  Copyright:	MontaVista Software Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define YF255_ETH_PHYS	PXA_CS3_PHYS

#define YF255_FPGA_PHYS	PXA_CS2_PHYS
#define YF255_FPGA_VIRT	(0xf0000000)
#define YF255_P2V(x)		((x) - YF255_FPGA_PHYS + YF255_FPGA_VIRT)
#define YF255_V2P(x)		((x) - YF255_FPGA_VIRT + YF255_FPGA_PHYS)

#ifndef __ASSEMBLY__
#  define __YF255_REG(x)		(*((volatile unsigned long *)YF255_P2V(x)))
#else
#  define __YF255_REG(x)		YF255_P2V(x)
#endif

/* FPGA register virtual addresses */
#define YF255_WHOAMI		__YF255_REG(YF255_FPGA_PHYS + 0x000)
#define YF255_HEXLED		__YF255_REG(YF255_FPGA_PHYS + 0x010)
#define YF255_DISC_BLNK_LED	__YF255_REG(YF255_FPGA_PHYS + 0x040)
#define YF255_CONF_SWITCHES	__YF255_REG(YF255_FPGA_PHYS + 0x050)
#define YF255_USER_SWITCHES	__YF255_REG(YF255_FPGA_PHYS + 0x060)
#define YF255_MISC_WR		__YF255_REG(YF255_FPGA_PHYS + 0x080)
#define YF255_MISC_RD		__YF255_REG(YF255_FPGA_PHYS + 0x090)
#define YF255_IRQ_MASK_EN	__YF255_REG(YF255_FPGA_PHYS + 0x0c0)
#define YF255_IRQ_SET_CLR	__YF255_REG(YF255_FPGA_PHYS + 0x0d0)
#define YF255_GP		__YF255_REG(YF255_FPGA_PHYS + 0x100)

#ifndef __ASSEMBLY__
extern void yf255_set_misc_wr(unsigned int mask, unsigned int set);
#endif
