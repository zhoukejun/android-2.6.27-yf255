/*
 *  linux/arch/arm/mach-pxa/yf255.c  zkj copy lubbock.c
 *
 *  Support for the Intel DBPXA250 Development Platform.
 *
 *  Author:	Nicolas Pitre
 *  Created:	Jun 15, 2001
 *  Copyright:	MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/major.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>

#include <asm/setup.h>
#include <asm/memory.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/sizes.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/mach/flash.h>

#include <mach/pxa-regs.h>
#include <mach/pxa2xx-regs.h>
#include <mach/mfp-pxa25x.h>
#include <mach/audio.h>
#include <mach/yf255.h>
#include <mach/udc.h>
#include <mach/irda.h>
#include <mach/pxafb.h>
#include <mach/mmc.h>

#include "generic.h"
#include "clock.h"
#include "devices.h"

static unsigned long yf255_pin_config[] __initdata = {
	GPIO15_nCS_1,	/* CS1 - Flash */
	GPIO79_nCS_3,	/* CS3 - SMC ethernet */

	/* SSP data pins */
	GPIO23_SSP1_SCLK,
	GPIO25_SSP1_TXD,
	GPIO26_SSP1_RXD,

#if 0
	/* AC97 */
	GPIO28_AC97_BITCLK,
	GPIO29_AC97_SDATA_IN_0,
	GPIO30_AC97_SDATA_OUT,
	GPIO31_AC97_SYNC,
#endif
	/* BTUART */
	GPIO42_BTUART_RXD,
	GPIO43_BTUART_TXD,
	GPIO44_BTUART_CTS,
	GPIO45_BTUART_RTS,

	/* PC Card */
	GPIO48_nPOE,
	GPIO49_nPWE,
	GPIO50_nPIOR,
	GPIO51_nPIOW,
	GPIO52_nPCE_1,
	GPIO53_nPCE_2,
	GPIO54_nPSKTSEL,
	GPIO55_nPREG,
	GPIO56_nPWAIT,
	GPIO57_nIOIS16,

#if 0
	/* LCD */
	GPIO58_LCD_LDD_0, 
	GPIO59_LCD_LDD_1, 
	GPIO60_LCD_LDD_2, 
	GPIO61_LCD_LDD_3, 
	GPIO62_LCD_LDD_4, 
	GPIO63_LCD_LDD_5, 
	GPIO64_LCD_LDD_6, 
	GPIO65_LCD_LDD_7, 
	GPIO66_LCD_LDD_8, 
	GPIO67_LCD_LDD_9, 
	GPIO68_LCD_LDD_10,
	GPIO69_LCD_LDD_11,
	GPIO70_LCD_LDD_12,
	GPIO71_LCD_LDD_13,
	GPIO72_LCD_LDD_14,
	GPIO73_LCD_LDD_15,
	GPIO74_LCD_FCLK,
	GPIO75_LCD_LCLK,
	GPIO76_LCD_PCLK,
	GPIO77_LCD_ACBIAS,
#endif
#if 0
	/* MMC */
	GPIO6_MMC_CLK,
	GPIO8_MMC_CS0,
	GPIO12_nMCCCD,
#endif

	/* wakeup */
	GPIO1_GPIO | WAKEUP_ON_EDGE_RISE,

	GPIO27_GPIO | MFP_DIR_IN | WAKEUP_ON_LEVEL_HIGH,
	GPIO32_GPIO | MFP_DIR_IN | WAKEUP_ON_LEVEL_HIGH,
	GPIO42_GPIO | MFP_DIR_IN | WAKEUP_ON_LEVEL_HIGH,
	GPIO43_GPIO | MFP_DIR_IN | WAKEUP_ON_LEVEL_HIGH,
	GPIO44_GPIO | MFP_DIR_IN | WAKEUP_ON_LEVEL_HIGH,
	GPIO45_GPIO | MFP_DIR_IN | WAKEUP_ON_LEVEL_HIGH,
};

#define YF255_MISC_WR		__YF255_REG(YF255_FPGA_PHYS + 0x080)

static unsigned long yf255_irq_enabled;

static void yf255_mask_irq(unsigned int irq)
{
	int yf255_irq = (irq - YF255_IRQ(0));
	YF255_IRQ_MASK_EN = (yf255_irq_enabled &= ~(1 << yf255_irq));
}

static void yf255_unmask_irq(unsigned int irq)
{
	int yf255_irq = (irq - YF255_IRQ(0));
	/* the irq can be acknowledged only if deasserted, so it's done here */
	YF255_IRQ_SET_CLR &= ~(1 << yf255_irq);
	YF255_IRQ_MASK_EN = (yf255_irq_enabled |= (1 << yf255_irq));
}

static struct irq_chip yf255_irq_chip = {
	.name		= "FPGA",
	.ack		= yf255_mask_irq,
	.mask		= yf255_mask_irq,
	.unmask		= yf255_unmask_irq,
};

static void yf255_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	unsigned long pending = YF255_IRQ_SET_CLR & yf255_irq_enabled;
	do {
		GEDR(0) = GPIO_bit(0);	/* clear our parent irq */
		if (likely(pending)) {
			irq = YF255_IRQ(0) + __ffs(pending);
			desc = irq_desc + irq;
			desc_handle_irq(irq, desc);
		}
		pending = YF255_IRQ_SET_CLR & yf255_irq_enabled;
	} while (pending);
}

static void __init yf255_init_irq(void)
{
	int irq;

	pxa25x_init_irq();
#if 1
	/* setup extra yf255 irqs */
	for (irq = YF255_IRQ(0); irq <= YF255_LAST_IRQ; irq++) {
		set_irq_chip(irq, &yf255_irq_chip);
		set_irq_handler(irq, handle_level_irq);
		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
	}
#endif

	/*For Ethernet_INT*/
	set_irq_type(IRQ_GPIO(0), IRQ_TYPE_EDGE_RISING);

	/*For RTC_INT*/
	set_irq_type(IRQ_GPIO(1), IRQ_TYPE_EDGE_RISING);

	/*For AC97_IRQ*/
//	set_irq_type(IRQ_GPIO(3), IRQ_TYPE_EDGE_RISING);

	/*For PCMCIA_DECT*/
	set_irq_type(IRQ_GPIO(4), IRQ_TYPE_EDGE_RISING);
	/*For PCMCIA_RDY*/
	set_irq_type(IRQ_GPIO(5), IRQ_TYPE_EDGE_RISING);

	/*For MMC*/
	set_irq_type(IRQ_GPIO(12), IRQ_TYPE_LEVEL_LOW);// IRQ_TYPE_EDGE_FALLING);//zkj
}

#ifdef CONFIG_PM

static int yf255_irq_resume(struct sys_device *dev)
{
	YF255_IRQ_MASK_EN = yf255_irq_enabled;
	return 0;
}

static struct sysdev_class yf255_irq_sysclass = {
	.name = "cpld_irq",
	.resume = yf255_irq_resume,
};

static struct sys_device yf255_irq_device = {
	.cls = &yf255_irq_sysclass,
};

static int __init yf255_irq_device_init(void)
{
	int ret = -ENODEV;

	if (machine_is_yf255()) {
		ret = sysdev_class_register(&yf255_irq_sysclass);
		if (ret == 0)
			ret = sysdev_register(&yf255_irq_device);
	}
	return ret;
}

device_initcall(yf255_irq_device_init);

#endif

static int yf255_udc_is_connected(void)
{
	return (YF255_MISC_RD & (1 << 9)) == 0;
}

static struct pxa2xx_udc_mach_info udc_info __initdata = {
	.udc_is_connected	= yf255_udc_is_connected,
	// no D+ pullup; yf255 can't connect/disconnect in software
};

static struct resource lan91c113_resources[] = {
	[0] = {
		.name	= "lan91c113-regs",
		.start	= YF255_ETH_PHYS,
		.end	= YF255_ETH_PHYS + 0x0000fffff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_GPIO0, //YF255_ETH_IRQ,
		.end	= IRQ_GPIO0, //YF255_ETH_IRQ,
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

static struct platform_device lan91c113_device = {
	.name		= "lan91c113",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(lan91c113_resources),
	.resource	= lan91c113_resources,
	.dev		= {
		.platform_data = NULL,
	},
};

static struct resource flash_resources[] = {
	[0] = {
		.start	= 0x00000000,
		.end	= SZ_32M - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct mtd_partition yf255_partitions[] = {
	{
		.name =		"Bootloader",
		.size =		0x00040000,
		.offset =	0,
		.mask_flags =	MTD_WRITEABLE  /* force read-only */
	},{
		.name =		"Kernel",
		.size =		0x00100000,
		.offset =	0x00080000,//zkj
	},{
		.name =		"Filesystem",
		.size =		MTDPART_SIZ_FULL,
		.offset =	0x01800000//0x00180000 //zkj
	}
};

static struct flash_platform_data yf255_flash_data[] = {
	{
		.map_name	= "cfi_probe",
		.parts		= yf255_partitions,
		.nr_parts	= ARRAY_SIZE(yf255_partitions),
	},
};

static struct platform_device yf255_flash_device = {
	.name		= "pxa2xx-flash",
	.id		= 0,
	.dev = {
		.platform_data = &yf255_flash_data[0],
	},
	.resource = &flash_resources[0],
	.num_resources = 1,
};

#if defined(CONFIG_SND_PXA2XX_AC97) || defined(CONFIG_SND_PXA2XX_AC97_MODULE)
static void __init yf255_init_ac97(void)
{
	pxa_set_ac97_info(NULL);
}
#else
static inline void yf255_init_ac97(void) {}
#endif

/* UCB1400 touchscreen controller */
#if defined(CONFIG_TOUCHSCREEN_UCB1400) || defined(CONFIG_TOUCHSCREEN_UCB1400_MODULE)
/* UCB1400 touchscreen controller */
static struct platform_device yf255_ts_device = {
	.name		= "ucb1400_ts",
	.id		= -1,
};

static void __init yf255_init_touchscreen(void)
{
        platform_device_register(&yf255_ts_device);
}
#else
static inline void yf255_init_touchscreen(void) {}
#endif

static struct gpio_keys_button gpio_keys_button[] = {
	[0] = {
		.desc	= "Back",
//		.code	= 59,
		.code	= 82,//158,
		.type	= EV_KEY,
		.gpio	= 43,
		.active_low = 0,
		.wakeup = 1,
		.debounce_interval = 1,
	},
#if 0
	[1] = {
		.desc	= "Menu",
		.code	= 82,//158,
//		.code	= 139,
		.type	= EV_KEY,
		.gpio	= 44,
		.active_low = 0,
		.wakeup = 1,
		.debounce_interval = 1,
	},
	[2] = {
		.desc	= "Menu",
		.code	= 82,//0,
		.type	= EV_KEY,
		.gpio	= 45,
		.active_low = 0,
		.wakeup = 1,
		.debounce_interval = 1,
	},
	[3] = {
		.desc	= "Menu",
		.code	= 82,//0,
//		.code	= 229,
		.type	= EV_KEY,
		.gpio	= 42,
		.active_low = 0,
		.wakeup = 1,
		.debounce_interval = 1,
	},
	[4] = {
		.desc	= "Power",
		.code	= 82,// 59,
//		.code	= 116,
		.type	= EV_KEY,
		.gpio	= 32,
		.active_low = 0,
		.wakeup = 1,
		.debounce_interval = 1,
	},
	[5] = {
		.desc	= "Power",
		.code	= 82,//59,
//		.code	= 227,
		.type	= EV_KEY,
		.gpio	= 27,
		.active_low = 0,
		.wakeup = 1,
		.debounce_interval = 1,
	},
#endif
};

static struct gpio_keys_platform_data yf255_gpio_keys = {
	.buttons	= gpio_keys_button,
	.nbuttons	= ARRAY_SIZE(gpio_keys_button),
};

static struct platform_device yf255_gpio_keys_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
		.platform_data	= &yf255_gpio_keys,
	},
};

static void __init yf255_init_gpiokeys(void)
{
        platform_device_register(&yf255_gpio_keys_device);
}

static struct pxafb_mode_info sharp_lq043_mode = {
	.pixclock	= 108696,
	.xres		= 480,
	.yres		= 272,
	.bpp		= 16,
	.hsync_len	= 41,
	.left_margin	= 1,
	.right_margin	= 3,
	.vsync_len	= 10,
	.upper_margin	= 2,
	.lower_margin	= 2,
	.sync		= 0,
	.cmap_greyscale	= 0,
};

static struct pxafb_mach_info sharp_lq043 = {
	.modes		= &sharp_lq043_mode,
	.num_modes	= 1,
	.lcd_conn	= LCD_COLOR_TFT_16BPP | LCD_PCLK_EDGE_FALL,
	.lccr0		= LCCR0_Act | LCCR0_Sngl | LCCR0_Color,
	.lccr3		= LCCR3_OutEnH | LCCR3_PixFlEdg,
};


static struct platform_device *devices[] __initdata = {
	&lan91c113_device,
	&yf255_flash_device,
};

#define	MMC_POLL_RATE		msecs_to_jiffies(1000)

static void yf255_mmc_poll(unsigned long);
static irq_handler_t mmc_detect_int;

static struct timer_list mmc_timer = {
	.function	= yf255_mmc_poll,
};

static void yf255_mmc_poll(unsigned long data)
{
	unsigned long flags;

	/* clear any previous irq state, then ... */
	local_irq_save(flags);
	YF255_IRQ_SET_CLR &= ~(1 << 0);
	local_irq_restore(flags);

	/* poll until mmc/sd card is removed */
	if (YF255_IRQ_SET_CLR & (1 << 0))
		mod_timer(&mmc_timer, jiffies + MMC_POLL_RATE);
	else {
		(void) mmc_detect_int(IRQ_GPIO(12), (void *)data);
		enable_irq(IRQ_GPIO(12));
	}
}

static irqreturn_t yf255_detect_int(int irq, void *data)
{
	/* IRQ is level triggered; disable, and poll for removal */
	disable_irq(irq);
	mod_timer(&mmc_timer, jiffies + MMC_POLL_RATE);

	return mmc_detect_int(irq, data);
}

static int yf255_mci_init(struct device *dev,
		irq_handler_t detect_int,
		void *data)
{
	/* detect card insert/eject */
	mmc_detect_int = detect_int;
	init_timer(&mmc_timer);
	mmc_timer.data = (unsigned long) data;
	return request_irq(IRQ_GPIO(12), yf255_detect_int,
			IRQF_SAMPLE_RANDOM, "yf255-sd-detect", data);
}

static int yf255_mci_get_ro(struct device *dev)
{
	return (YF255_MISC_RD & (1 << 2)) != 0;
}

static void yf255_mci_exit(struct device *dev, void *data)
{
	free_irq(IRQ_GPIO(12), data);
	del_timer_sync(&mmc_timer);
}

static struct pxamci_platform_data yf255_mci_platform_data = {
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
	.detect_delay	= 1,
	.init 		= yf255_mci_init,
	.get_ro		= yf255_mci_get_ro,
	.exit 		= yf255_mci_exit,
};

static void yf255_irda_transceiver_mode(struct device *dev, int mode)
{
	unsigned long flags;

	local_irq_save(flags);
	if (mode & IR_SIRMODE) {
		YF255_MISC_WR &= ~(1 << 4);
	} else if (mode & IR_FIRMODE) {
		YF255_MISC_WR |= 1 << 4;
	}
	pxa2xx_transceiver_mode(dev, mode);
	local_irq_restore(flags);
}

static struct pxaficp_platform_data yf255_ficp_platform_data = {
	.transceiver_cap  = IR_SIRMODE | IR_FIRMODE,
	.transceiver_mode = yf255_irda_transceiver_mode,
};

static void __init yf255_init(void)
{
	int flashboot = (YF255_CONF_SWITCHES & 1);

	pxa2xx_mfp_config(ARRAY_AND_SIZE(yf255_pin_config));

	pxa_set_udc_info(&udc_info);
	set_pxa_fb_info(&sharp_lq043);
	pxa_set_mci_info(&yf255_mci_platform_data);
	pxa_set_ficp_info(&yf255_ficp_platform_data);

	yf255_init_gpiokeys();
	yf255_init_ac97();
	yf255_init_touchscreen();

	yf255_flash_data[0].width = (BOOT_DEF & 1) ? 2 : 4;
	/* Compensate for the nROMBT switch which swaps the flash banks */
	printk(KERN_NOTICE "YF255 configured to boot from %s (bank %d)\n",
	       flashboot?"Flash":"ROM", flashboot);

	yf255_flash_data[flashboot].name = "boot-rom";
	(void) platform_add_devices(devices, ARRAY_SIZE(devices));
}

static struct map_desc yf255_io_desc[] __initdata = {
  	{	/* CPLD */
		.virtual	=  YF255_FPGA_VIRT,
		.pfn		= __phys_to_pfn(YF255_FPGA_PHYS),
		.length		= 0x00100000,
		.type		= MT_DEVICE
	}
};

static void __init yf255_map_io(void)
{
	pxa_map_io();
	iotable_init(yf255_io_desc, ARRAY_SIZE(yf255_io_desc));
#if 1
//zkj add start --------------------------------------
        /* low power mode setting */
        /* if you can make the power consumption less, change the setting below */
        PGSR0 = 0xC3E39FFC;
        PGSR1 = 0xFCFFAB8C;
        PGSR2 = 0x0001FFFF;
//zkj add end  --------------------------------------
#endif
	PCFR |= PCFR_OPDE;
#if 1
	OSCC |= OSCC_OON; //zkj.
#endif
}

MACHINE_START(YF255, "YFDVK-255-I Development Platform (PXA255)")
	/* Maintainer: zhoukejun@gmail.com */
	.phys_io	= 0x40000000,
	.io_pg_offst	= (io_p2v(0x40000000) >> 18) & 0xfffc,
	.map_io		= yf255_map_io,
	.init_irq	= yf255_init_irq,
	.timer		= &pxa_timer,
	.init_machine	= yf255_init,
MACHINE_END
