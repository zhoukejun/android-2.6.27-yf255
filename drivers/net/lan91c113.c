/*------------------------------------------------------------------------
 .lan91c113.c  Updated from smc91111.c by zhoukejun (zhoukeju@gmail.com)
 . This is a driver for SMSC's 91C111 single-chip Ethernet device.
 .
 . Copyright (C) 2001-2004 Standard Microsystems Corporation (SMSC)
 .       Developed by Simple Network Magic Corporation (SNMC)
 . Copyright (C) 1996 by Erik Stahlman (ES)
 .
 . This program is free software; you can redistribute it and/or modify
 . it under the terms of the GNU General Public License as published by
 . the Free Software Foundation; either version 2 of the License, or
 . (at your option) any later version.
 .
 . This program is distributed in the hope that it will be useful,
 . but WITHOUT ANY WARRANTY; without even the implied warranty of
 . MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 . GNU General Public License for more details.
 .
 . You should have received a copy of the GNU General Public License
 . along with this program; if not, write to the Free Software
 . Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 .
 . Information contained in this file was obtained from the LAN91C111
 . manual from SMC.  To get a copy, if you really want one, you can find 
 . information under www.smsc.com.
 . 
 .
 . "Features" of the SMC chip:
 .   Integrated PHY/MAC for 10/100BaseT Operation
 .   Supports internal and external MII
 .   Integrated 8K packet memory
 .   EEPROM interface for configuration
 .
 . Arguments:
 . 	io	= for the base address
 .	irq	= for the IRQ
 .	nowait	= 0 for normal wait states, 1 eliminates additional wait states
 .
 . author:
 . 	Erik Stahlman				( erik@vt.edu )
 . 	Daris A Nevil				( dnevil@snmc.com )
 .  	Pramod B Bhardwaj   			(pramod.bhardwaj@smsc.com)
 .  	M David Gelbman   			(david.gelbman@smsc.com)
 .
 .
 . Hardware multicast code from Peter Cammaert ( pc@denkart.be )
 .
 . Sources:
 .    o   SMSC LAN91C111 databook (www.smsc.com)
 .    o   smc9194.c by Erik Stahlman
 .    o   skeleton.c by Donald Becker ( becker@cesdis.gsfc.nasa.gov )
 .
 . History:
 .    17/08/06 (v2.02) by Frank Meisenbach (f.meisenbach@cbl.de)
 .             - Bugs fixed to improve the stability
 .             --> Sometimes the system hung up, e.g. under heavy broadcast load condition
 .             - Added the function eth_change_mtu()
 .             - Added support for RevC
 .             --> smc_recv() was buggy, RevC was handled as RevA
 .             - Modified smc_timeout()
 .             --> The interface will have the same behavior as before (PROMISC, MULTICAST)
 .	  06/23/06  M David Gelbman add "insmod" link_mode parameter
 .    11/03/05  M David Gelbman  Added support for RevC of LAN91C111
 .    12/13/04  Steven Chew, Added code to reset PHY upon link down
 .    09/24/01  Pramod B Bhardwaj, Added the changes for Kernel 2.4
 .    08/21/01  Pramod B Bhardwaj Added support for RevB of LAN91C111
 .	04/25/01  Daris A Nevil  Initial public release through SMSC
 .	03/16/01  Daris A Nevil  Modified smc9194.c for use with LAN91C111
 ----------------------------------------------------------------------------*/

// Use power-down feature of the chip
#define POWER_DOWN	1
#define CONFIG_SMC16BITONLY	1

static const char version[] =
    "SMSC LAN91C113 Driver (For YFDVK-255-I by zhoukejun@gmail.com), (Linux Kernel 2.6 + Support for Odd Byte) 03/17/2009\n\n";

/* Debugging options */
#if 0
#define ENABLE_SMC_DEBUG_RX		1
#define ENABLE_SMC_DEBUG_TX		1
#define ENABLE_SMC_DEBUG_DMA		1
#define ENABLE_SMC_DEBUG_PKTS		1
#define ENABLE_SMC_DEBUG_MISC		1
#define ENABLE_SMC_DEBUG_FUNC		1
#else
#define ENABLE_SMC_DEBUG_RX		0
#define ENABLE_SMC_DEBUG_TX		0
#define ENABLE_SMC_DEBUG_DMA		0
#define ENABLE_SMC_DEBUG_PKTS		0
#define ENABLE_SMC_DEBUG_MISC		0
#define ENABLE_SMC_DEBUG_FUNC		0
#endif

#define SMC_DEBUG_RX		((ENABLE_SMC_DEBUG_RX	? 1 : 0) << 0)
#define SMC_DEBUG_TX		((ENABLE_SMC_DEBUG_TX	? 1 : 0) << 1)
#define SMC_DEBUG_DMA		((ENABLE_SMC_DEBUG_DMA	? 1 : 0) << 2)
#define SMC_DEBUG_PKTS		((ENABLE_SMC_DEBUG_PKTS ? 1 : 0) << 3)
#define SMC_DEBUG_MISC		((ENABLE_SMC_DEBUG_MISC ? 1 : 0) << 4)
#define SMC_DEBUG_FUNC		((ENABLE_SMC_DEBUG_FUNC ? 1 : 0) << 5)

#ifndef SMC_DEBUG
#define SMC_DEBUG	 ( SMC_DEBUG_RX	  | \
			   SMC_DEBUG_TX	  | \
			   SMC_DEBUG_DMA  | \
			   SMC_DEBUG_PKTS | \
			   SMC_DEBUG_MISC | \
			   SMC_DEBUG_FUNC   \
			 )
#endif

#include <linux/module.h>
#include <linux/version.h>

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/in.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/init.h>
#include <asm/bitops.h>

#include <mach/dma.h>
#include <mach/hardware.h>

#include <asm/irq.h>

#include <linux/errno.h>
#include <linux/delay.h>

#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>

#ifdef CONFIG_SYSCTL
#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#endif

#include <asm/io.h>
#include "lan91c113.h"
/*------------------------------------------------------------------------
 .
 . Configuration options, for the experienced user to change.
 .
 -------------------------------------------------------------------------*/

/*
 . Do you want to use 32 bit xfers?  This should work on all chips, as
 . the chipset is designed to accommodate them.
*/
#undef USE_32_BIT

/*
 . Wait time for memory to be free.  This probably shouldn't be
 . tuned that much, as waiting for this means nothing else happens
 . in the system
*/
#define MEMORY_WAIT_TIME 16

/*
 . DEBUGGING LEVELS
 .
 . 0 for normal operation
 . 1 for slightly more details
 . >2 for various levels of increasingly useless information
 .    2 for interrupt tracking, status flags
 .    3 for packet info
 .    4 for complete packet dumps
*/

#if (SMC_DEBUG > 2 )
#define PRINTK3(args...) printk(args)
#else
#define PRINTK3(args...)
#endif

#if SMC_DEBUG > 1
#define PRINTK2(args...) printk(args)
#else
#define PRINTK2(args...)
#endif

#ifdef SMC_DEBUG
#define PRINTK(args...) printk(args)
#else
#define PRINTK(args...)
#endif

#if SMC_DEBUG > 0
#define DBG(n, args...)				 \
	do {					 \
		if (SMC_DEBUG & (n))		 \
			printk(args);		 \
	} while (0)

#else
#define DBG(n, args...)   do { } while (0)
#endif

/*------------------------------------------------------------------------
 .
 . The internal workings of the driver.  If you are changing anything
 . here with the SMC stuff, you should have the datasheet and know
 . what you are doing.
 .
 -------------------------------------------------------------------------*/
#define CARDNAME "lan91c113"

// Memory sizing constant
#define LAN91C111_MEMORY_MULTIPLIER     (1024*2)

/* store this information for the driver.. */
MODULE_LICENSE("GPL");

/*-----------------------------------------------------------------
 .
 .  The driver can be entered at any of the following entry points.
 .
 .------------------------------------------------------------------  */

static int smc_detect_phy(struct net_device *dev);
/*
 . This is called by  register_netdev().  It is responsible for
 . checking the portlist for the SMC9000 series chipset.  If it finds
 . one, then it will initialize the device, find the hardware information,
 . and sets up the appropriate device parameters.
 . NOTE: Interrupts are *OFF* when this procedure is called.
 .
 . NB:This shouldn't be static since it is referred to externally.
*/
static int __init smc_init(void);

/*
 . This is called by  unregister_netdev().  It is responsible for
 . cleaning up before the driver is finally unregistered and discarded.
*/
void smc_destructor(struct net_device *dev);

/*
 . The kernel calls this function when someone wants to use the net_device,
 . typically 'ifconfig ethX up'.
*/
static int smc_open(struct net_device *dev);

/*
 . This is called by the kernel to send a packet out into the net.  it's
 . responsible for doing a best-effort send, but if it's simply not possible
 . to send it, the packet gets dropped.
*/
static void smc_timeout(struct net_device *dev);
/*
 . This is called by the kernel in response to 'ifconfig ethX down'.  It
 . is responsible for cleaning up everything that the open routine
 . does, and maybe putting the card into a powerdown state.
*/
static int smc_close(struct net_device *dev);

/*
 . This routine allows the proc file system to query the driver's
 . statistics.
*/
static struct net_device_stats *smc_query_statistics(struct net_device *dev);

/*
 . Finally, a call to set promiscuous mode ( for TCPDUMP and related
 . programs ) and multicast modes.
*/
static void smc_set_multicast_list(struct net_device *dev);

/*
 . Configures the PHY through the MII Management interface
*/
static void smc_phy_configure(struct net_device *dev);

/*---------------------------------------------------------------
 .
 . Interrupt level calls..
 .
 ----------------------------------------------------------------*/

/*
 . Handles the actual interrupt
*/
static irqreturn_t smc_interrupt(int irq, void *);
/*
 . This is a separate procedure to handle the receipt of a packet, to
 . leave the interrupt code looking slightly cleaner
*/
inline static void smc_rcv(struct net_device *dev);
/*
 . This handles a TX interrupt, which is only called when an error
 . relating to a packet is sent.
*/
inline static void smc_tx(struct net_device *dev);

/*
 . This handles interrupts generated from PHY register 18
*/
static void smc_phy_interrupt(struct net_device *dev);

/*
 ------------------------------------------------------------
 .
 . Internal routines
 .
 ------------------------------------------------------------
*/

/*
 . Test if a given location contains a chip, trying to cause as
 . little damage as possible if it's not a SMC chip.
*/
static int smc_probe(struct net_device *dev, unsigned int ioaddr);

/*
 . A rather simple routine to print out a packet for debugging purposes.
*/

#if SMC_DEBUG > 2
static void print_packet(byte *, int);
#endif

#define tx_done(dev) 1

/* this is called to actually send the packet to the chip */
static void smc_hardware_send_packet(struct net_device *dev);

/* Since I am not sure if I will have enough room in the chip's ram
 . to store the packet, I call this routine, which either sends it
 . now, or generates an interrupt when the card is ready for the
 . packet */
static int smc_wait_to_send_packet(struct sk_buff *skb, struct net_device *dev);

/* this does a soft reset on the device */
static void smc_reset(struct net_device *dev);

/* Enable Interrupts, Receive, and Transmit */
static void smc_enable(struct net_device *dev);

/* this puts the device in an inactive state */
static void smc_shutdown(unsigned int ioaddr);

/*
  this routine will set the hardware multicast table to the specified
  values given it by the higher level routines
*/
static void smc_setmulticast(unsigned int ioaddr, int count,
			     struct dev_mc_list *);
static int crc32(char *, int);

/* Routines to Read and Write the PHY Registers across the
   MII Management Interface
*/

static word smc_read_phy_register(unsigned int ioaddr, byte phyaddr,
				  byte phyreg);
static void smc_write_phy_register(unsigned int ioaddr, byte phyaddr,
				   byte phyreg, word phydata);

/* Initilizes our device's sysctl proc filesystem */

#ifdef CONFIG_SYSCTL
static void smc_sysctl_register(struct net_device *);
static void smc_sysctl_unregister(struct net_device *);
#endif				/* CONFIG_SYSCTL */

#ifdef CONFIG_NET_POLL_CONTROLLER
static void net_poll_controller(struct net_device *dev);

/*
 * Polling receive - used by netconsole and other diagnostic tools
 * to allow network i/o with interrupts disabled.
 */
static void net_poll_controller(struct net_device *dev)
{
	disable_irq(dev->irq);
	net_interrupt(dev->irq, dev);
	enable_irq(dev->irq);
}
#endif

/*
 . Function: smc_reset( struct device* dev )
 . Purpose:
 .      This sets the SMC91111 chip to its normal state, hopefully from whatever
 .      mess that any other DOS driver has put it in.
 .
 . Maybe I should reset more registers to defaults in here?  SOFTRST  should
 . do that for me.
 .
 . Method:
 .      1.  send a SOFT RESET
 .      2.  wait for it to finish
 .      3.  enable autorelease mode
 .      4.  reset the memory management unit
 .      5.  clear all interrupts
 .
*/
static void smc_reset(struct net_device *dev)
{
	struct smc_local *lp = netdev_priv(dev);
	unsigned int ioaddr = lp->base;
	unsigned short status_test;

	PRINTK3("smc_reset()\n");

	status_test = SMC_inw(BANK_SELECT);
	if ((status_test & 0xff00) != 0x3300) {
		printk("ethid=%2x...\n", status_test);
	}
	PRINTK2("%s:smc_reset\n", dev->name);

	/* This resets the registers mostly to defaults, but doesn't
	   affect EEPROM.  That seems unnecessary */
	SMC_SELECT_BANK(0);
	SMC_outw(RCR_SOFTRST, RCR_REG);

	/* Setup the Configuration Register */
	/* This is necessary because the CONFIG_REG is not affected */
	/* by a soft reset */

	SMC_SELECT_BANK(1);
	SMC_outw(CONFIG_DEFAULT, CONFIG_REG);

	/* Setup for fast accesses if requested */
	/* If the card/system can't handle it then there will */
	/* be no recovery except for a hard reset or power cycle */

	if (dev->dma)
		SMC_outw(SMC_inw(CONFIG_REG) | CONFIG_NO_WAIT, CONFIG_REG);

#ifdef POWER_DOWN
	/* Release from possible power-down state */
	/* Configuration register is not affected by Soft Reset */
	SMC_SELECT_BANK(1);
	SMC_outw(SMC_inw(CONFIG_REG) | CONFIG_EPH_POWER_EN, CONFIG_REG);
#endif

	SMC_SELECT_BANK(0);

	/* this should pause enough for the chip to be happy */
	mdelay(10);

	/* Disable transmit and receive functionality */
	SMC_outw(RCR_CLEAR, RCR_REG);
	SMC_outw(TCR_CLEAR, TCR_REG);

	/* set the control register to automatically
	   release successfully transmitted packets, to make the best
	   use out of our limited memory */
	SMC_SELECT_BANK(1);
	SMC_outw(SMC_inw(CTL_REG) | CTL_AUTO_RELEASE, CTL_REG);

	/* Reset the MMU */
	SMC_SELECT_BANK(2);
	SMC_outw(MC_RESET, MMU_CMD_REG);

	/* Note:  It doesn't seem that waiting for the MMU busy is needed here,
	   but this is a place where future chipsets _COULD_ break.  Be wary
	   of issuing another MMU command right after this */

	/* Disable all interrupts */
#if defined(CONFIG_SMC16BITONLY)
	SMC_outw(0, INT_REG);
#else
	SMC_outb(0, IM_REG);
#endif
//	iounmap (ioaddr);
}

/*
 . Function: smc_enable
 . Purpose: let the chip talk to the outside work
 . Method:
 .      1.  Enable the transmitter
 .      2.  Enable the receiver
 .      3.  Enable interrupts
*/
static void smc_enable(struct net_device *dev)
{
	struct smc_local *lp = netdev_priv(dev);
	unsigned int ioaddr = lp->base;

	PRINTK2("%s:smc_enable\n", dev->name);

	SMC_SELECT_BANK(0);
	/* see the header file for options in TCR/RCR DEFAULT */
	// SMC_outw( lp->tcr_cur_mode,   TCR_REG );
	//  SMC_outw( lp->rcr_cur_mode,   RCR_REG );
	SMC_outw(0x0001, TCR_REG);
	SMC_outw(0x0300, RCR_REG);

	/* now, enable interrupts */
	SMC_SELECT_BANK(2);
#if defined(CONFIG_SMC16BITONLY)
	SMC_outw((SMC_INTERRUPT_MASK << 8), INT_REG);
#else
	SMC_outb(SMC_INTERRUPT_MASK, IM_REG);
#endif
}

/*
 . Function: smc_shutdown
 . Purpose:  closes down the SMC91xxx chip.
 . Method:
 .      1. zero the interrupt mask
 .      2. clear the enable receive flag
 .      3. clear the enable xmit flags
 .
 . TODO:
 .   (1) maybe utilize power down mode.
 .      Why not yet?  Because while the chip will go into power down mode,
 .      the manual says that it will wake up in response to any I/O requests
 .      in the register space.   Empirical results do not show this working.
*/
static void smc_shutdown(unsigned int ioaddr)
{
	PRINTK2(CARDNAME ":smc_shutdown\n");

	/* no more interrupts for me */
	SMC_SELECT_BANK(2);
#if defined(CONFIG_SMC16BITONLY)
	SMC_outw(0, INT_REG);
#else
	SMC_outb(0, IM_REG);
#endif

	/* and tell the card to stay away from that nasty outside world */
	SMC_SELECT_BANK(0);
#if defined(CONFIG_SMC16BITONLY)
	SMC_outw(RCR_CLEAR, RCR_REG);
	SMC_outw(TCR_CLEAR, TCR_REG);
#else
	SMC_outb(RCR_CLEAR, RCR_REG);
	SMC_outb(TCR_CLEAR, TCR_REG);
#endif

#ifdef POWER_DOWN
	/* finally, shut the chip down */
	SMC_SELECT_BANK(1);
	SMC_outw(SMC_inw(CONFIG_REG) & ~CONFIG_EPH_POWER_EN, CONFIG_REG);
#endif
}

/*
 . Function: smc_setmulticast( unsigned int ioaddr, int count, dev_mc_list * adds )
 . Purpose:
 .    This sets the internal hardware table to filter out unwanted multicast
 .    packets before they take up memory.
 .
 .    The SMC chip uses a hash table where the high 6 bits of the CRC of
 .    address are the offset into the table.  If that bit is 1, then the
 .    multicast packet is accepted.  Otherwise, it's dropped silently.
 .
 .    To use the 6 bits as an offset into the table, the high 3 bits are the
 .    number of the 8 bit register, while the low 3 bits are the bit within
 .    that register.
 .
 . This routine is based very heavily on the one provided by Peter Cammaert.
*/

static void
smc_setmulticast(unsigned int ioaddr, int count, struct dev_mc_list *addrs)
{
	int i;
	unsigned char multicast_table[8];
	struct dev_mc_list *cur_addr;
	/* table for flipping the order of 3 bits */
	unsigned char invert3[] = { 0, 4, 2, 6, 1, 5, 3, 7 };

	PRINTK2(CARDNAME ":smc_setmulticast\n");

	/* start with a table of all zeros: reject all */
	memset(multicast_table, 0, sizeof(multicast_table));

	cur_addr = addrs;
	for (i = 0; i < count; i++, cur_addr = cur_addr->next) {
		int position;

		/* do we have a pointer here? */
		if (!cur_addr)
			break;
		/* make sure this is a multicast address - shouldn't this
		   be a given if we have it here ? */
		if (!(*cur_addr->dmi_addr & 1))
			continue;

		/* only use the low order bits */
		position = crc32(cur_addr->dmi_addr, 6) & 0x3f;

		/* do some messy swapping to put the bit in the right spot */
		multicast_table[invert3[position & 7]] |=
		    (1 << invert3[(position >> 3) & 7]);

	}
	/* now, the table can be loaded into the chipset */
	SMC_SELECT_BANK(3);

#if defined(CONFIG_SMC16BITONLY)
	for (i = 0; i < 8; i += 2) {
		SMC_outw(((multicast_table[i + 1] << 8) | (multicast_table[i])),
			 MCAST_REG1 + i);
	}
#else
	for (i = 0; i < 8; i++) {
		SMC_outb(multicast_table[i], MCAST_REG1 + i);
	}
#endif
}

/*
  Finds the CRC32 of a set of bytes.
  Again, from Peter Cammaert's code.
*/
static int crc32(char *s, int length)
{
	/* indices */
	int perByte;
	int perBit;
	/* crc polynomial for Ethernet */
	const unsigned long poly = 0xedb88320;
	/* crc value - preinitialized to all 1's */
	unsigned long crc_value = 0xffffffff;

	for (perByte = 0; perByte < length; perByte++) {
		unsigned char c;

		c = *(s++);
		for (perBit = 0; perBit < 8; perBit++) {
			crc_value = (crc_value >> 1) ^
			    (((crc_value ^ c) & 0x01) ? poly : 0);
			c >>= 1;
		}
	}
	return crc_value;
}

/*
 . Function: smc_wait_to_send_packet( struct sk_buff * skb, struct device * )
 . Purpose:
 .    Attempt to allocate memory for a packet, if chip-memory is not
 .    available, then tell the card to generate an interrupt when it
 .    is available.
 .
 . Algorithm:
 .
 . o    if the saved_skb is not currently null, then drop this packet
 .      on the floor.  This should never happen, because of TBUSY.
 . o    if the saved_skb is null, then replace it with the current packet,
 . o    See if I can sending it now.
 . o    (NO): Enable interrupts and let the interrupt handler deal with it.
 . o    (YES):Send it now.
*/
static int smc_wait_to_send_packet(struct sk_buff *skb, struct net_device *dev)
{
	struct smc_local *lp = netdev_priv(dev);
	unsigned int ioaddr = lp->base;
	word length;
	unsigned short numPages;
	word time_out;
	word status;

	PRINTK3("%s: smc_wait_to_send_packet\n", dev->name);

	netif_stop_queue(dev);	//zkj. 2009-03-14

	if (lp->saved_skb) {
		/* THIS SHOULD NEVER HAPPEN. */
		lp->stats.tx_aborted_errors++;
                printk("%s: Bad Craziness - sent packet while busy.\n",
		                       dev->name);
		return 1;
	}
	lp->saved_skb = skb;

	length = ETH_ZLEN < skb->len ? skb->len : ETH_ZLEN;

	/*
	 ** The MMU wants the number of pages to be the number of 256 bytes
	 ** 'pages', minus 1 ( since a packet can't ever have 0 pages :) )
	 **
	 ** The 91C111 ignores the size bits, but the code is left intact
	 ** for backwards and future compatibility.
	 **
	 ** Pkt size for allocating is data length +6 (for additional status
	 ** words, length and ctl!)
	 **
	 ** If odd size then last byte is included in this header.
	 */
	numPages = ((length & 0xfffe) + 6);
	numPages >>= 8;		// Divide by 256

	if (numPages > 7) {
		printk("%s: Far too big packet error. \n", dev->name);
		/* freeing the packet is a good thing here... but should
		   . any packets of this size get down here?   */
		dev_kfree_skb(skb);
		lp->saved_skb = NULL;
		/* this IS an error, but, i don't want the skb saved */
		netif_wake_queue(dev);
		return 0;
	}
	/* either way, a packet is waiting now */
	lp->packets_waiting++;

	/* now, try to allocate the memory */
	SMC_SELECT_BANK(2);
	SMC_outw(MC_ALLOC | numPages, MMU_CMD_REG);
	/*
	   . Performance Hack
	   .
	   . wait a short amount of time.. if I can send a packet now, I send
	   . it now.  Otherwise, I enable an interrupt and wait for one to be
	   . available.
	   .
	   . I could have handled this a slightly different way, by checking to
	   . see if any memory was available in the FREE MEMORY register.  However,
	   . either way, I need to generate an allocation, and the allocation works
	   . no matter what, so I saw no point in checking free memory.
	 */
	time_out = MEMORY_WAIT_TIME;
	do {
		status = SMC_inb(INT_REG);
		if (status & IM_ALLOC_INT) {
			/* acknowledge the interrupt */
#if defined(CONFIG_SMC16BITONLY)
			SMC_outw(IM_ALLOC_INT | (SMC_inb(IM_REG) << 8),
				 INT_REG);
#else
			SMC_outb(IM_ALLOC_INT, INT_REG);
#endif
			break;
		}
	}
	while (--time_out);

	if (!time_out) {
		/* oh well, wait until the chip finds memory later */
		SMC_ENABLE_INT(IM_ALLOC_INT);

		/* Check the status bit one more time just in case */
		/* it snuk in between the time we last checked it */
		/* and when we set the interrupt bit */
		status = SMC_inb(INT_REG);
		if (!(status & IM_ALLOC_INT)) {
			PRINTK2("%s: memory allocation deferred. \n",
				dev->name);
			/* it's deferred, but I'll handle it later */
			return 0;
		}

		/* Looks like it did sneak in, so disable */
		/* the interrupt */
		SMC_DISABLE_INT(IM_ALLOC_INT);
	}
	/* or YES! I can send the packet now.. */
	smc_hardware_send_packet(dev);
	netif_wake_queue(dev);
	return 0;
}

/*
 . Function:  smc_hardware_send_packet(struct device * )
 . Purpose:
 .      This sends the actual packet to the SMC9xxx chip.
 .
 . Algorithm:
 .      First, see if a saved_skb is available.
 .              ( this should NOT be called if there is no 'saved_skb'
 .      Now, find the packet number that the chip allocated
 .      Point the data pointers at it in memory
 .      Set the length word in the chip's memory
 .      Dump the packet to chip memory
 .      Check if a last byte is needed ( odd length packet )
 .              if so, set the control flag right
 .      Tell the card to send it
 .      Enable the transmit interrupt, so I know if it failed
 .      Free the kernel data if I actually sent it.
*/
static void smc_hardware_send_packet(struct net_device *dev)
{
	struct smc_local *lp = netdev_priv(dev);
	unsigned int ioaddr = lp->base;
	byte packet_no;
	struct sk_buff *skb = lp->saved_skb;
	word length;
	byte *buf;

	PRINTK3("%s:smc_hardware_send_packet\n", dev->name);

	if (!skb) {
		PRINTK("%s: In XMIT with no packet to send \n", dev->name);
		return;
	}
	length = ETH_ZLEN < skb->len ? skb->len : ETH_ZLEN;
	buf = skb->data;

	/* If I get here, I _know_ there is a packet slot waiting for me */
	packet_no = SMC_inb(AR_REG);
	if (packet_no & AR_FAILED) {
		/* or isn't there?  BAD CHIP! */
		printk(KERN_DEBUG "%s: Memory allocation failed. \n",
		       dev->name);
		dev_kfree_skb_any(skb);
		lp->saved_skb = NULL;
		netif_wake_queue(dev);
		return;
	}

	/* we have a packet address, so tell the card to use it */
	SMC_outb(packet_no, PN_REG);

	/* point to the beginning of the packet */
	SMC_outw(PTR_AUTOINC, PTR_REG);

	PRINTK3("%s: Trying to xmit packet of length %x\n", dev->name, length);

#if SMC_DEBUG > 2
	printk("Transmitting Packet\n");
	print_packet(buf, length);
#endif

	/* send the packet length ( +6 for status, length and ctl byte )
	   and the status word ( set to zeros ) */
#ifdef USE_32_BIT
#if defined(CONFIG_SMC16BITONLY)
	SMC_outl((length + 6), DATA_REG);
#else
	SMC_outl((length + 6) << 16, DATA_REG);
#endif
#else
	SMC_outw(0, DATA_REG);
	/* send the packet length ( +6 for status words, length, and ctl */
#if defined(CONFIG_SMC16BITONLY)
//        SMC_outw( (length+6) & 0xFFFF,  DATA_REG );
	SMC_outw((length + 6), DATA_REG);
#else
	SMC_outb((length + 6) & 0xFF, DATA_REG);
	SMC_outb((length + 6) >> 8, DATA_REG);
#endif
#endif

	/* send the actual data
	   . I _think_ it's faster to send the longs first, and then
	   . mop up by sending the last word.  It depends heavily
	   . on alignment, at least on the 486.  Maybe it would be
	   . a good idea to check which is optimal?  But that could take
	   . almost as much time as is saved?
	 */
#ifdef USE_32_BIT
	SMC_outsl(DATA_REG, buf, length >> 2);
	if (length & 0x2)
#if defined(CONFIG_SMC16BITONLY)
		SMC_outw(*((word *) (buf + (length & 0xFFFFFFFC))), DATA_REG);
#else
		SMC_outw(*((word *) (buf + (length & 0xFFFFFFFC))), DATA_REG);
#endif
#else
	SMC_outsw(DATA_REG, buf, (length) >> 1);
#endif				// USE_32_BIT

	/* Send the last byte, if there is one.   */
	if ((length & 1) == 0) {
		SMC_outw(0, DATA_REG);
	} else {
#if defined(CONFIG_SMC16BITONLY)
		SMC_outw(buf[length - 1] | 0x2000, DATA_REG);
#else
		SMC_outb(buf[length - 1], DATA_REG);
		SMC_outb(0x20, DATA_REG);	// Set odd bit in CONTROL BYTE
#endif
	}

	/* enable the interrupts */
	SMC_ENABLE_INT((IM_TX_INT | IM_TX_EMPTY_INT));

	/* and let the chipset deal with it */
	SMC_outw(MC_ENQUEUE, MMU_CMD_REG);

	PRINTK2("%s: Sent packet of length %d \n", dev->name, length);

	lp->saved_skb = NULL;
	dev_kfree_skb_any(skb);

	dev->trans_start = jiffies;

	/* we can send another packet */
	netif_wake_queue(dev);

	return;
}

/*----------------------------------------------------------------------
 . Function: smc_probe( unsigned int ioaddr )
 .
 . Purpose:
 .      Tests to see if a given ioaddr points to an SMC91111 chip.
 .      Returns a 0 on success
 .
 . Algorithm:
 .      (1) see if the high byte of BANK_SELECT is 0x33
 .      (2) compare the ioaddr with the base register's address
 .      (3) see if I recognize the chip ID in the appropriate register
 .
 .---------------------------------------------------------------------
 */
/*---------------------------------------------------------------
 . Here I do typical initialization tasks.
 .
 . o  Initialize the structure if needed
 . o  print out my vanity message if not done so already
 . o  print out what type of hardware is detected
 . o  print out the ethernet address
 . o  find the IRQ
 . o  set up my private data
 . o  configure the dev structure with my subroutines
 . o  actually GRAB the irq.
 . o  GRAB the region
 .-----------------------------------------------------------------*/
static int __init smc_probe(struct net_device *dev, unsigned int ioaddr)
{
	struct smc_local *lp = netdev_priv(dev);
	int i, memory, retval;
	static unsigned version_printed = 0;
	unsigned int bank;
	const char *version_string;
	DECLARE_MAC_BUF(mac);

	/*registers */
	word revision_register;
	word base_address_register;
	word memory_info_register;

	printk("smc_probe()\n");
	DBG(SMC_DEBUG_FUNC, "%s: --> %s\n", CARDNAME, __FUNCTION__);

	/* First, see if the high byte is 0x33 */
	bank = SMC_inw(BANK_SELECT);
	if ((bank & 0xFF00) != 0x3300) {
		PRINTK2("SMC_wrong_1\n");
		printk("Can't detect : %04x\n", bank);
		return -ENODEV;
	}

	/* The above MIGHT indicate a device, but I need to write to further test this.  */
	SMC_outw(0x0, BANK_SELECT);
	bank = SMC_inw(BANK_SELECT);
	if ((bank & 0xFF00) != 0x3300) {
		retval = -ENODEV;
		PRINTK2("SMC_wrong_2\n");
		goto err_out;
	}

	/* well, we've already written once, so hopefully another time won't
	   hurt.  This time, I need to switch the bank register to bank 1,
	   so I can access the base address register */
	SMC_SELECT_BANK(1);

	base_address_register = SMC_inw(BASE_REG);
	if ((ioaddr & 0xfff) !=
	    (base_address_register >> 3 & 0x3E0)) {
		printk(CARDNAME ": IOADDR %x doesn't match configuration (%x)."
		       "Probably not a SMC chip\n", ioaddr,
		       base_address_register >> 3 & 0x3E0);
		/* well, the base address register didn't match.  Must not have
		   been a SMC chip after all. */
		retval = -ENODEV;
		goto err_out;
	}

	/*  check if the revision register is something that I recognize.
	   These might need to be added to later, as future revisions
	   could be added.  */
	SMC_SELECT_BANK(3);
	revision_register = SMC_inw(REV_REG);
	if (!chip_ids[(revision_register >> 4) & 0xF]) {
		/* I don't recognize this chip, so... */
		printk(CARDNAME ": IO %x: Unrecognized revision register:"
		       " %x, Contact author. \n", ioaddr, revision_register);
		retval = -ENODEV;
		goto err_out;
	}

	/* at this point I'll assume that the chip is an SMC9xxx.
	   It might be prudent to check a listing of MAC addresses
	   against the hardware address, or do some other tests. */

	if (version_printed++ == 0)
		printk("%s", version);

	/* fill in some of the fields */
	dev->base_addr = ioaddr;
	lp->base = ioaddr;
	lp->version = revision_register & 0xff;
	spin_lock_init(&lp->lock);

	/* Program MAC address if not set... */
	SMC_SELECT_BANK(1);
	for (i = 0; (i < 6); i += 2) {
		word address;
		address = SMC_inw(ADDR0_REG + i);
		if ((address != 0x0000) && (address != 0xffff))
			break;
	}
	if (i >= 6) {
		/* Set a default MAC address */
		SMC_outw(0xCF00, ADDR0_REG);
		SMC_outw(0x4952, ADDR0_REG + 2);
		SMC_outw(0x01C3, ADDR0_REG + 4);
	}

	/*
	   . Get the MAC address ( bank 1, regs 4 - 9 )
	 */
	SMC_SELECT_BANK(1);
	for (i = 0; i < 6; i += 2) {
		word address;

		address = SMC_inw(ADDR0_REG + i);
		dev->dev_addr[i + 1] = address >> 8;
		dev->dev_addr[i] = address & 0xFF;
	}

	/* get the memory information */

	SMC_SELECT_BANK(0);
	memory_info_register = SMC_inw(MIR_REG);
	memory = memory_info_register & (word) 0x00ff;
	memory *= LAN91C111_MEMORY_MULTIPLIER;

	/*
	   Now, I want to find out more about the chip.  This is sort of
	   redundant, but it's cleaner to have it in both, rather than having
	   one VERY long probe procedure.
	 */
	SMC_SELECT_BANK(3);
	revision_register = SMC_inw(REV_REG);
	version_string = chip_ids[(revision_register >> 4) & 0xF];
	if (!version_string) {
		/* I shouldn't get here because this call was done before.... */
		retval = -ENODEV;
		PRINTK2("SMC_wrong_3\n");
		goto err_out;
	}

	/* now, reset the chip, and put it into a known state */
	smc_reset(dev);

	/* now, print out the card info, in a short format.. */

	printk("%s: %s(rev:%d) at %#3x IRQ:%d MEMSIZE:%db NOWAIT:%d\n",
	       dev->name,
	       version_string, revision_register & 0xF, ioaddr, dev->irq,
	       memory, dev->dma);
	/*
	   . Print the Ethernet address
	 */
	printk("ADDR: ");
	for (i = 0; i < 5; i++)
		printk("%2.2x:", dev->dev_addr[i]);
	printk("%2.2x \n", dev->dev_addr[5]);

	/* Fill in the fields of the device structure with ethernet values. */
	ether_setup(dev);

	//GRER |= IRQ_SMC91C111;
	//GFER &= ~IRQ_SMC91C111;

	dev->open = smc_open;
	dev->stop = smc_close;
	dev->hard_start_xmit = smc_wait_to_send_packet;
	dev->tx_timeout = smc_timeout;
	dev->watchdog_timeo = msecs_to_jiffies(1000);
	dev->get_stats = smc_query_statistics;
#ifdef  HAVE_MULTICAST
	dev->set_multicast_list = smc_set_multicast_list;
#endif
#ifdef CONFIG_NET_POLL_CONTROLLER
	dev->poll_controller	= net_poll_controller;
#endif

	/* => Store the ChipRevision and ChipID, to be used in resolving the Odd-Byte issue in RevB of LAN91C111; Pramod */
	SMC_SELECT_BANK(3);
	revision_register = SMC_inw(REV_REG);
	lp->dev = dev;
	lp->ChipID = (revision_register >> 4) & 0xF;
	lp->ChipRev = revision_register & 0xF;

	smc_detect_phy(dev);

	/* Grab the IRQ */
	retval = request_irq(dev->irq, &smc_interrupt, 0, dev->name, dev);
	if (retval) {
		printk("%s: unable to get IRQ %d (irqval=%d).\n",
		       dev->name, dev->irq, retval);
		kfree(dev->priv);
		dev->priv = NULL;
		goto err_out;
	}

	retval = register_netdev(dev);

	if (retval == 0) {
		/* now, print out the card info, in a short format.. */
		printk("%s: %s (rev %d) at %x IRQ %d",
			dev->name, version_string, revision_register & 0x0f,
			lp->base, dev->irq);

		if (dev->dma != (unsigned char)-1)
			printk(" DMA %d", dev->dma);

		if (!is_valid_ether_addr(dev->dev_addr)) {
			printk("%s: Invalid ethernet MAC address.  Please "
			       "set using ifconfig\n", dev->name);
		} else {
			/* Print the Ethernet address */
			printk("%s: Ethernet addr: %s\n",
			       dev->name, print_mac(mac, dev->dev_addr));
		}
	}

err_out:
#ifdef CONFIG_ARCH_PXA
	if (retval && dev->dma != (unsigned char)-1)
		pxa_free_dma(dev->dma);
#endif
	return retval;
}

/*-------------------------------------------------------------------------
 |
 | smc_init( struct device * dev )
 |   Input parameters:
 |      dev->base_addr == 0, try to find all possible locations
 |      dev->base_addr == 1, return failure code
 |      dev->base_addr == 2, always allocate space,  and return success
 |      dev->base_addr == <anything else>   this is the address to check
 |
 |   Output:
 |      0 --> there is a device
 |      anything else, error
 |
 ---------------------------------------------------------------------------
*/
static int smc_drv_probe(struct platform_device *pdev)
{
	struct smc91x_platdata *pd = pdev->dev.platform_data;
	struct net_device *ndev;
	struct smc_local *lp; 
	struct resource *res, *ires;
	unsigned int addr;
	unsigned long irq_flags = SMC_IRQ_FLAGS;
	int ret;

	printk("smc_drv_probe()\n");

	ndev = alloc_etherdev(sizeof(struct smc_local));
	if (!ndev) {
		printk("%s: could not allocate device.\n", CARDNAME);
		ret = -ENOMEM;
		goto out;
	}
	SET_NETDEV_DEV(ndev, &pdev->dev);

	lp = netdev_priv(ndev);

	if (pd) {
		memcpy(&lp->cfg, pd, sizeof(lp->cfg));
	} else {
		lp->cfg.flags |= (SMC_CAN_USE_16BIT) ? SMC_USE_16BIT : 0;
		lp->cfg.flags |= SMC_NOWAIT;
	}

	ndev->dma = (unsigned char)-1;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					 "lan91c113-regs");

	if (!res) {
		ret = -ENODEV;
		goto out_free_netdev;
	}

	if (!request_mem_region(res->start, SMC_IO_EXTENT, CARDNAME)) {
		ret = -EBUSY;
		goto out_free_netdev;
	}

	ires = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!ires) {
		ret = -ENODEV;
		goto out_release_io;
	}

	ndev->irq = ires->start;

	if (ires->flags & IRQF_TRIGGER_MASK)
		irq_flags = ires->flags & IRQF_TRIGGER_MASK;

	platform_set_drvdata(pdev, ndev);

	addr = ioremap(res->start, SMC_IO_EXTENT);
	if (!addr) {
		ret = -ENOMEM;
		goto out_release_io;
	}
#ifdef CONFIG_ARCH_PXA
	lp->device = &pdev->dev;
	lp->physaddr = res->start;
#endif

	ret = smc_probe(ndev, addr + 0x300);
	if (ret != 0)
		goto out_iounmap;
	printk("smc_drv_probe end()\n");
	return 0;

 out_iounmap:
	platform_set_drvdata(pdev, NULL);
	iounmap(addr);
 out_release_io:
	release_mem_region(res->start, SMC_IO_EXTENT);
 out_free_netdev:
	free_netdev(ndev);
 out:
	printk("%s: not found (%d).\n", CARDNAME, ret);

	return ret;
}

/*-------------------------------------------------------------------------
 |
 | smc_destructor( struct device * dev )
 |   Input parameters:
 |      dev, pointer to the device structure
 |
 |   Output:
 |      None.
 |
 ---------------------------------------------------------------------------
*/
void smc_destructor(struct net_device *dev)
{
	PRINTK2(CARDNAME ":smc_destructor\n");
}

#if SMC_DEBUG > 2
static void print_packet(byte * buf, int length)
{
	int i;
	int remainder;
	int lines;

	printk("Packet of length %d \n", length);

	lines = length / 16;
	remainder = length % 16;

	for (i = 0; i < lines; i++) {
		int cur;

		for (cur = 0; cur < 8; cur++) {
			byte a, b;

			a = *(buf++);
			b = *(buf++);
			printk("%02x%02x ", a, b);
		}
		printk("\n");
	}
	for (i = 0; i < remainder / 2; i++) {
		byte a, b;

		a = *(buf++);
		b = *(buf++);
		printk("%02x%02x ", a, b);
	}
	printk("\n");
}
#endif

/*
 * Open and Initialize the board
 *
 * Set up everything, reset the card, etc ..
 *
 */
static int smc_open(struct net_device *dev)
{
	struct smc_local *lp = netdev_priv(dev);
	unsigned int ioaddr = lp->base;
	int i;	/* used to set hw ethernet address */

	PRINTK2("%s:smc_open\n", dev->name);

	/* Setup the default Register Modes */
	lp->tcr_cur_mode = TCR_DEFAULT;
	lp->rcr_cur_mode = RCR_DEFAULT;
	lp->rpc_cur_mode = RPC_DEFAULT;

#ifdef CONFIG_SYSCTL
	/* Set default parameters (files) */
	lp->ctl_swfdup = 0;
	lp->ctl_ephloop = 0;
	lp->ctl_miiop = 0;
	lp->ctl_autoneg = 1;
	lp->ctl_rfduplx = 1;
	lp->ctl_rspeed = 100;
	lp->ctl_afduplx = 1;
	lp->ctl_aspeed = 100;
	lp->ctl_lnkfail = 1;
	lp->ctl_forcol = 0;
	lp->ctl_filtcar = 0;
#endif

	/* reset the hardware */

	smc_reset(dev);
	smc_enable(dev);

	/* Configure the PHY */
	smc_phy_configure(dev);

	/*
	   According to Becker, I have to set the hardware address
	   at this point, because the (l)user can set it with an
	   ioctl.  Easily done...
	 */
	SMC_SELECT_BANK(1);
	for (i = 0; i < 6; i += 2) {
		word address;

		address = dev->dev_addr[i + 1] << 8;
		address |= dev->dev_addr[i];
		SMC_outw(address, ADDR0_REG + i);
	}

#ifdef CONFIG_SYSCTL
	smc_sysctl_register(dev);
#endif				/* CONFIG_SYSCTL */

	netif_start_queue(dev);
	return 0;
}

/*--------------------------------------------------------
 . Called by the kernel to send a packet out into the void
 . of the net.  This routine is largely based on
 . skeleton.c, from Becker.
 .--------------------------------------------------------
*/
static void smc_timeout(struct net_device *dev)
{
	struct smc_local *lp = netdev_priv(dev);
	PRINTK3("%s:smc_timeout()\n", dev->name);

	/* If we get here, some higher level has decided we are broken.
	   There should really be a "kick me" function call instead. */
	printk(KERN_WARNING "%s: transmit timed out, %s?\n", dev->name,
	       tx_done(dev) ? "IRQ conflict" : "network cable problem");

	/* "kick" the adaptor */
	smc_reset(dev);
	smc_enable(dev);

	/* Reconfigure the PHY */
	smc_phy_configure(dev);

	netif_wake_queue(dev);
	dev->trans_start = jiffies;
	/* clear anything saved */
	lp->saved_skb = NULL;
}

/*--------------------------------------------------------------------
 .
 . This is the main routine of the driver, to handle the net_device when
 . it needs some attention.
 .
 . So:
 .   first, save state of the chipset
 .   branch off into routines to handle each case, and acknowledge
 .          each to the interrupt register
 .   and finally restore state.
 .
 ---------------------------------------------------------------------*/
static irqreturn_t smc_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct smc_local *lp = netdev_priv(dev);
	unsigned int ioaddr = lp->base;

	byte status;
	word card_stats;
	byte mask;
	int timeout;
	/* state registers */
	word saved_bank;
	word saved_pointer;

	PRINTK3("%s: SMC interrupt started \n", dev->name);
	if (dev == NULL) {
		printk(KERN_WARNING "%s: irq %d for unknown device.\n",
		       dev->name, irq);
		return IRQ_NONE;
	}

	/* will Linux let this happen ??  If not, this costs some speed
	   if ( dev->interrupt ) {
	   printk(KERN_WARNING "%s: interrupt inside interrupt.\n",
	   dev->name);
	   return;
	   }

	   dev->interrupt = 1; */

	spin_lock(&lp->lock);

	saved_bank = SMC_inw(BANK_SELECT);

	SMC_SELECT_BANK(2);
	saved_pointer = SMC_inw(PTR_REG);

	/* read the interrupt status register */
	mask = SMC_inb(IM_REG);

	/* disable all interrupts */
#if defined(CONFIG_SMC16BITONLY)
	SMC_outw(0, INT_REG);
#else
	SMC_outb(0, IM_REG);
#endif

	/* set a timeout value, so I don't stay here forever */
	timeout = 4;

	PRINTK2(KERN_WARNING "%s: MASK IS %x \n", dev->name, mask);
	do {
		/* read the status flag, and mask it */
		status = SMC_inb(INT_REG) & mask;
		if (!status)
			break;

		PRINTK3(KERN_WARNING "%s: Handling interrupt status %x \n",
			dev->name, status);

		if (status & IM_RCV_INT) {
			/* Got a packet(s). */
			PRINTK2(KERN_WARNING "%s: Receive Interrupt\n",
				dev->name);
			smc_rcv(dev);
		} else if (status & IM_TX_INT) {
			PRINTK2(KERN_WARNING "%s: TX ERROR handled\n",
				dev->name);
			smc_tx(dev);
			// Acknowledge the interrupt
#if defined(CONFIG_SMC16BITONLY)
			// MASK is 0 here, so 16bit write is safe
			SMC_outw(IM_TX_INT, INT_REG);
#else
			SMC_outb(IM_TX_INT, INT_REG);
#endif
		} else if (status & IM_TX_EMPTY_INT) {
			/* update stats */
			SMC_SELECT_BANK(0);
			card_stats = SMC_inw(COUNTER_REG);
			/* single collisions */
			lp->stats.collisions += card_stats & 0xF;
			card_stats >>= 4;
			/* multiple collisions */
			lp->stats.collisions += card_stats & 0xF;

			/* these are for when linux supports these statistics */
#if 0
			card_stats >>= 4;
			/* deferred */
			card_stats >>= 4;
			/* excess deferred */
#endif
			SMC_SELECT_BANK(2);
			PRINTK2(KERN_WARNING "%s: TX_BUFFER_EMPTY handled\n",
				dev->name);
			// Acknowledge the interrupt
#if defined(CONFIG_SMC16BITONLY)
			// MASK is 0 here, so 16bit write is safe
			SMC_outw(IM_TX_EMPTY_INT, INT_REG);
#else
			SMC_outb(IM_TX_EMPTY_INT, INT_REG);
#endif
			mask &= ~IM_TX_EMPTY_INT;
			lp->stats.tx_packets += lp->packets_waiting;
			lp->packets_waiting = 0;

		} else if (status & IM_ALLOC_INT) {
			PRINTK2(KERN_DEBUG "%s: Allocation interrupt \n",
				dev->name);
			/* clear this interrupt so it doesn't happen again */
			mask &= ~IM_ALLOC_INT;

			smc_hardware_send_packet(dev);

			/* enable xmit interrupts based on this */
			mask |= (IM_TX_EMPTY_INT | IM_TX_INT);

			/* and let the card send more packets to me */
			netif_wake_queue(dev);

			PRINTK2("%s: Handoff done successfully.\n", dev->name);
		} else if (status & IM_RX_OVRN_INT) {
			lp->stats.rx_errors++;
			lp->stats.rx_fifo_errors++;
			// Acknowledge the interrupt
#if defined(CONFIG_SMC16BITONLY)
			// MASK is 0 here, so 16bit write is safe
			SMC_outw(IM_RX_OVRN_INT, INT_REG);
#else
			SMC_outb(IM_RX_OVRN_INT, INT_REG);
#endif
		} else if (status & IM_EPH_INT) {
			PRINTK("%s: UNSUPPORTED: EPH INTERRUPT \n", dev->name);
		} else if (status & IM_MDINT) {
			smc_phy_interrupt(dev);
			// Acknowledge the interrupt
#if defined(CONFIG_SMC16BITONLY)
			// MASK is 0 here, so 16bit write is safe
			SMC_outw(IM_MDINT, INT_REG);
#else
			SMC_outb(IM_MDINT, INT_REG);
#endif
		} else if (status & IM_ERCV_INT) {
			PRINTK("%s: UNSUPPORTED: ERCV INTERRUPT \n", dev->name);
			// Acknowledge the interrupt
#if defined(CONFIG_SMC16BITONLY)
			// MASK is 0 here, so 16bit write is safe
			SMC_outw(IM_ERCV_INT, INT_REG);
#else
			SMC_outb(IM_ERCV_INT, INT_REG);
#endif
		}
	}
	while (timeout--);

	/* restore register states */

	SMC_SELECT_BANK(2);


#if defined(CONFIG_SMC16BITONLY)
	SMC_outw(mask << 8, INT_REG);
#else
	SMC_outb(mask, IM_REG);
#endif

	PRINTK3(KERN_WARNING "%s: MASK is now %x \n", dev->name, mask);
	SMC_outw(saved_pointer, PTR_REG);

	SMC_SELECT_BANK(saved_bank);

	spin_unlock(&lp->lock);

	//dev->interrupt = 0;
	PRINTK3("%s: Interrupt done\n", dev->name);
	return IRQ_HANDLED;
}

/*-------------------------------------------------------------
 .
 . smc_rcv -  receive a packet from the card
 .
 . There is ( at least ) a packet waiting to be read from
 . chip-memory.
 .
 . o Read the status
 . o If an error, record it
 . o otherwise, read in the packet
 --------------------------------------------------------------
*/
static void smc_rcv(struct net_device *dev)
{
	struct smc_local *lp = netdev_priv(dev);
	unsigned int ioaddr = lp->base;
	int packet_number;
	word status;
	word packet_length;

	PRINTK3("%s:smc_rcv\n", dev->name);

	/* assume bank 2 */

	packet_number = SMC_inw(RXFIFO_REG);

	if (packet_number & RXFIFO_REMPTY) {

		/* we got called , but nothing was on the FIFO */
		PRINTK("%s: WARNING: smc_rcv with nothing on FIFO. \n",
		       dev->name);
		/* don't need to restore anything */
		return;
	}

	/*  start reading from the start of the packet */
	SMC_outw(PTR_READ | PTR_RCV | PTR_AUTOINC, PTR_REG);

	/* First two words are status and packet_length */
	status = SMC_inw(DATA_REG);
	packet_length = SMC_inw(DATA_REG);

	packet_length &= 0x07ff;	/* mask off top bits */

	PRINTK2("RCV: STATUS %4x LENGTH %4x\n", status, packet_length);

	if (!(status & RS_ERRORS)) {
		/* do stuff to make a new packet */
		struct sk_buff *skb;
		byte *data;

		/* set multicast stats */
		if (status & RS_MULTICAST)
			lp->stats.multicast++;

		// Allocate enough memory for entire receive frame, to be safe
		skb = dev_alloc_skb(packet_length);

		/* Adjust for having already read the first two words */
		packet_length -= 4;

		if (skb == NULL) {
			printk(KERN_NOTICE "%s: Low memory, packet dropped.\n",
			       dev->name);
			lp->stats.rx_dropped++;
			goto done;
		}

		/*
		   ! This should work without alignment, but it could be
		   ! in the worse case
		 */
		/* TODO: Should I use 32bit alignment here ? */
		skb_reserve(skb, 2);	/* 16 bit alignment */

		skb->dev = dev;

		/* =>
		   ODD-BYTE ISSUE : The odd byte problem has been fixed in the LAN91C111 Rev B.
		   So we check if the Chip Revision, stored in smsc_local->ChipRev, is = 1.
		   If so then we increment the packet length only if RS_ODDFRAME is set.
		   If the Chip's revision is equal to 0, then we blindly increment the packet length
		   by 1, thus always assuming that the packet is odd length, leaving the higher layer
		   to decide the actual length.
		   -- Pramod
		   <= */
		if ((9 == lp->ChipID) && (1 == lp->ChipRev)) {
			if (status & RS_ODDFRAME)
				data = skb_put(skb, packet_length + 1);
			else
				data = skb_put(skb, packet_length);

		} else {
			// set odd length for bug in LAN91C111, REV A
			// which never sets RS_ODDFRAME
			data = skb_put(skb, packet_length + 1);
		}

#ifdef USE_32_BIT
		PRINTK3(" Reading %d dwords (and %d bytes) \n",
			packet_length >> 2, packet_length & 3);
		/* QUESTION:  Like in the TX routine, do I want
		   to send the DWORDs or the bytes first, or some
		   mixture.  A mixture might improve already slow PIO
		   performance  */
		insl(DATA_REG, data, packet_length >> 2);
		/* read the left over bytes */
		insb(DATA_REG, data + (packet_length & 0xFFFFFC),
		     packet_length & 0x3);
#else
		PRINTK3(" Reading %d words and %d byte(s) \n",
			(packet_length >> 1), packet_length & 1);
		SMC_insw(DATA_REG, data, packet_length >> 1);

#endif				// USE_32_BIT

#if SMC_DEBUG > 2
		printk("Receiving Packet\n");
		print_packet(data, packet_length);
#endif
		skb->protocol = eth_type_trans(skb, dev);
		netif_rx(skb);
		lp->stats.rx_packets++;
	} else {
		/* error ... */
		lp->stats.rx_errors++;

		if (status & RS_ALGNERR)
			lp->stats.rx_frame_errors++;
		if (status & (RS_TOOSHORT | RS_TOOLONG))
			lp->stats.rx_length_errors++;
		if (status & RS_BADCRC)
			lp->stats.rx_crc_errors++;
	}

	while (SMC_inw(MMU_CMD_REG) & MC_BUSY)
		udelay(1);	// Wait until not busy
 done:
	/*  error or good, tell the card to get rid of this packet */
	SMC_outw(MC_RELEASE, MMU_CMD_REG);

	return;
}

/*************************************************************************
 . smc_tx
 .
 . Purpose:  Handle a transmit error message.   This will only be called
 .   when an error, because of the AUTO_RELEASE mode.
 .
 . Algorithm:
 .      Save pointer and packet no
 .      Get the packet no from the top of the queue
 .      check if it's valid ( if not, is this an error??? )
 .      read the status word
 .      record the error
 .      ( resend?  Not really, since we don't want old packets around )
 .      Restore saved values
 ************************************************************************/
static void smc_tx(struct net_device *dev)
{
	struct smc_local *lp = netdev_priv(dev);
	unsigned int ioaddr = lp->base;
	byte saved_packet;
	byte packet_no;
	word tx_status;

	PRINTK3("%s:smc_tx\n", dev->name);

	/* assume bank 2  */

	saved_packet = SMC_inb(PN_REG);
	packet_no = SMC_inw(RXFIFO_REG);
	packet_no &= 0x7F;

	/* If the TX FIFO is empty then nothing to do */
	if (packet_no & TXFIFO_TEMPTY)
		return;

	/* select this as the packet to read from */
#if defined(CONFIG_SMC16BITONLY)
	SMC_outw(packet_no, PN_REG);
#else
	SMC_outb(packet_no, PN_REG);
#endif

	/* read the first word (status word) from this packet */
	SMC_outw(PTR_AUTOINC | PTR_READ, PTR_REG);

	tx_status = SMC_inw(DATA_REG);
	PRINTK3("%s: TX DONE STATUS: %4x \n", dev->name, tx_status);

	lp->stats.tx_errors++;
	if (tx_status & TS_LOSTCAR)
		lp->stats.tx_carrier_errors++;
	if (tx_status & TS_LATCOL) {
		printk(KERN_DEBUG
		       "%s: Late collision occurred on last xmit.\n",
		       dev->name);
		lp->stats.tx_window_errors++;
#ifdef CONFIG_SYSCTL
		lp->ctl_forcol = 0;	// Reset forced collsion
#endif
	}
#if 0
	if (tx_status & TS_16COL) {
	...}
#endif

	if (tx_status & TS_SUCCESS) {
		printk("%s: Successful packet caused interrupt \n", dev->name);
	}
	/* re-enable transmit */
	SMC_SELECT_BANK(0);
	SMC_outw(SMC_inw(TCR_REG) | TCR_ENABLE, TCR_REG);

	/* kill the packet */
	SMC_SELECT_BANK(2);
	SMC_outw(MC_FREEPKT, MMU_CMD_REG);

	/* one less packet waiting for me */
	lp->packets_waiting--;

	/* Don't change Packet Number Reg until busy bit is cleared */
	/* Per LAN91C111 Spec, Page 50 */
	while (SMC_inw(MMU_CMD_REG) & MC_BUSY) ;

#if defined(CONFIG_SMC16BITONLY)
	SMC_outw(saved_packet, PN_REG);
#else
	SMC_outb(saved_packet, PN_REG);
#endif
	return;
}

/*----------------------------------------------------
 . smc_close
 .
 . this makes the board clean up everything that it can
 . and not talk to the outside world.   Caused by
 . an 'ifconfig ethX down'
 .
 -----------------------------------------------------*/
static int smc_close(struct net_device *dev)
{
	netif_stop_queue(dev);
	//dev->start = 0;

	PRINTK2("%s:smc_close\n", dev->name);

#ifdef CONFIG_SYSCTL
	smc_sysctl_unregister(dev);
#endif				/* CONFIG_SYSCTL */

	/* clear everything */
	smc_shutdown(dev->base_addr);

	return 0;
}

/*------------------------------------------------------------
 . Get the current statistics.
 . This may be called with the card open or closed.
 .-------------------------------------------------------------*/
static struct net_device_stats *smc_query_statistics(struct net_device *dev)
{
	struct smc_local *lp = netdev_priv(dev);

	PRINTK2("%s:smc_query_statistics\n", dev->name);

	return &lp->stats;
}

/*-----------------------------------------------------------
 . smc_set_multicast_list
 .
 . This routine will, depending on the values passed to it,
 . either make it accept multicast packets, go into
 . promiscuous mode ( for TCPDUMP and cousins ) or accept
 . a select set of multicast packets
*/
static void smc_set_multicast_list(struct net_device *dev)
{
	struct smc_local *lp = netdev_priv(dev);
	unsigned int ioaddr = lp->base;

	PRINTK2("%s:smc_set_multicast_list\n", dev->name);

	SMC_SELECT_BANK(0);
	if (dev->flags & IFF_PROMISC) {
		PRINTK2("%s:smc_set_multicast_list:RCR_PRMS\n", dev->name);
		SMC_outw(SMC_inw(RCR_REG) | RCR_PRMS, RCR_REG);
	}

/* BUG?  I never disable promiscuous mode if multicasting was turned on.
   Now, I turn off promiscuous mode, but I don't do anything to multicasting
   when promiscuous mode is turned on.
*/

	/* Here, I am setting this to accept all multicast packets.
	   I don't need to zero the multicast table, because the flag is
	   checked before the table is
	 */
	else if (dev->flags & IFF_ALLMULTI) {
		SMC_outw(SMC_inw(RCR_REG) | RCR_ALMUL, RCR_REG);
		PRINTK2("%s:smc_set_multicast_list:RCR_ALMUL\n", dev->name);
	}

	/* We just get all multicast packets even if we only want them
	   . from one source.  This will be changed at some future
	   . point. */
	else if (dev->mc_count) {
		/* support hardware multicasting */

		/* be sure I get rid of flags I might have set */
		SMC_outw(SMC_inw(RCR_REG) & ~(RCR_PRMS | RCR_ALMUL), RCR_REG);
		/* NOTE: this has to set the bank, so make sure it is the
		   last thing called.  The bank is set to zero at the top */
		smc_setmulticast(ioaddr, dev->mc_count, dev->mc_list);
	} else {
		PRINTK2("%s:smc_set_multicast_list:~(RCR_PRMS|RCR_ALMUL)\n",
			dev->name);
		SMC_outw(SMC_inw(RCR_REG) & ~(RCR_PRMS | RCR_ALMUL), RCR_REG);

		/*
		   since I'm disabling all multicast entirely, I need to
		   clear the multicast list
		 */
		SMC_SELECT_BANK(3);
		SMC_outw(0, MCAST_REG1);
		SMC_outw(0, MCAST_REG2);
		SMC_outw(0, MCAST_REG3);
		SMC_outw(0, MCAST_REG4);
	}
}

#ifdef CONFIG_SYSCTL
/*------------------------------------------------------------
 . Modify a bit in the LAN91C111 register set
 .-------------------------------------------------------------*/
static word
smc_modify_regbit(int bank, unsigned int ioaddr, int reg,
		  unsigned int bit, int val)
{
	word regval;

	SMC_SELECT_BANK(bank);

	regval = SMC_inw(reg);
	if (val)
		regval |= bit;
	else
		regval &= ~bit;

	SMC_outw(regval, ioaddr);
	return (regval);
}

/*------------------------------------------------------------
 . Retrieve a bit in the LAN91C111 register set
 .-------------------------------------------------------------*/
static int
smc_get_regbit(int bank, unsigned int ioaddr, int reg, unsigned int bit)
{
	SMC_SELECT_BANK(bank);
	if (SMC_inw(reg) & bit)
		return (1);
	else
		return (0);
}

/*------------------------------------------------------------
 . Modify a LAN91C111 register (word access only)
 .-------------------------------------------------------------*/
static void smc_modify_reg(int bank, unsigned int ioaddr, int reg, word val)
{
	SMC_SELECT_BANK(bank);
	SMC_outw(val, reg);
}

/*------------------------------------------------------------
 . Retrieve a LAN91C111 register (word access only)
 .-------------------------------------------------------------*/
static int smc_get_reg(int bank, unsigned int ioaddr, int reg)
{
	SMC_SELECT_BANK(bank);
	return (SMC_inw(reg));
}

static const char smc_info_string[] =
    "\n"
    "info           Provides this information blurb\n"
    "swver          Prints the software version information of this driver\n"
    "autoneg        Auto-negotiate Mode = 1\n"
    "rspeed         Requested Speed, 100=100Mbps, 10=10Mpbs\n"
    "rfduplx        Requested Full Duplex Operation\n"
    "aspeed         Actual Speed, 100=100Mbps, 10=10Mpbs\n"
    "afduplx        Actual Full Duplex Operation\n"
    "lnkfail        PHY Link Failure when 1\n"
    "miiop          External MII when 1, Internal PHY when 0\n"
    "swfdup         Switched Full Duplex Mode (allowed only in MII operation)\n"
    "ephloop        EPH Block Loopback\n"
    "forcol         Force a collision\n"
    "filtcar        Filter leading edge of carrier sense for 12 bit times\n"
    "freemem        Free buffer memory in bytes\n"
    "totmem         Total buffer memory in bytes\n"
    "leda           Output of LED-A (green)\n"
    "ledb           Output of LED-B (yellow)\n"
    "chiprev        Revision ID of the LAN91C111 chip\n" "";

/*------------------------------------------------------------
 . Sysctl handler for all integer parameters
 .-------------------------------------------------------------*/
static int
smc_sysctl_handler(ctl_table * ctl, int write, struct file *filp,
		   void *buffer, size_t * lenp)
{
	struct net_device *dev = (struct net_device *)ctl->extra1;
	struct smc_local *lp = (struct smc_local *)ctl->extra2;
	unsigned int ioaddr = lp->base;
	int *valp = ctl->data;
	int val;
	int ret;

	// Update parameters from the real registers
	switch (ctl->ctl_name) {
	case CTL_SMC_FORCOL:
		*valp = smc_get_regbit(0, ioaddr, TCR_REG, TCR_FORCOL);
		break;

	case CTL_SMC_FREEMEM:
		*valp = ((word) smc_get_reg(0, ioaddr, MIR_REG) >> 8)
		    * LAN91C111_MEMORY_MULTIPLIER;
		break;

	case CTL_SMC_TOTMEM:
		*valp = (smc_get_reg(0, ioaddr, MIR_REG) & (word) 0x00ff)
		    * LAN91C111_MEMORY_MULTIPLIER;
		break;

	case CTL_SMC_CHIPREV:
		*valp = smc_get_reg(3, ioaddr, REV_REG);
		break;

	case CTL_SMC_AFDUPLX:
		*valp = (lp->lastPhy18 & PHY_INT_DPLXDET) ? 1 : 0;
		break;

	case CTL_SMC_ASPEED:
		*valp = (lp->lastPhy18 & PHY_INT_SPDDET) ? 100 : 10;
		break;

	case CTL_SMC_LNKFAIL:
		*valp = (lp->lastPhy18 & PHY_INT_LNKFAIL) ? 1 : 0;
		break;

	case CTL_SMC_LEDA:
		*valp = (lp->rpc_cur_mode >> RPC_LSXA_SHFT) & (word) 0x0007;
		break;

	case CTL_SMC_LEDB:
		*valp = (lp->rpc_cur_mode >> RPC_LSXB_SHFT) & (word) 0x0007;
		break;

	case CTL_SMC_MIIOP:
		*valp = smc_get_regbit(1, ioaddr, CONFIG_REG, CONFIG_EXT_PHY);
		break;

#ifdef SMC_DEBUG
	case CTL_SMC_REG_BSR:	// Bank Select
		*valp = smc_get_reg(0, ioaddr, BSR_REG);
		break;

	case CTL_SMC_REG_TCR:	// Transmit Control
		*valp = smc_get_reg(0, ioaddr, TCR_REG);
		break;

	case CTL_SMC_REG_ESR:	// EPH Status
		*valp = smc_get_reg(0, ioaddr, EPH_STATUS_REG);
		break;

	case CTL_SMC_REG_RCR:	// Receive Control
		*valp = smc_get_reg(0, ioaddr, RCR_REG);
		break;

	case CTL_SMC_REG_CTRR:	// Counter
		*valp = smc_get_reg(0, ioaddr, COUNTER_REG);
		break;

	case CTL_SMC_REG_MIR:	// Memory Information
		*valp = smc_get_reg(0, ioaddr, MIR_REG);
		break;

	case CTL_SMC_REG_RPCR:	// Receive/Phy Control
		*valp = smc_get_reg(0, ioaddr, RPC_REG);
		break;

	case CTL_SMC_REG_CFGR:	// Configuration
		*valp = smc_get_reg(1, ioaddr, CONFIG_REG);
		break;

	case CTL_SMC_REG_BAR:	// Base Address
		*valp = smc_get_reg(1, ioaddr, BASE_REG);
		break;

	case CTL_SMC_REG_IAR0:	// Individual Address
		*valp = smc_get_reg(1, ioaddr, ADDR0_REG);
		break;

	case CTL_SMC_REG_IAR1:	// Individual Address
		*valp = smc_get_reg(1, ioaddr, ADDR1_REG);
		break;

	case CTL_SMC_REG_IAR2:	// Individual Address
		*valp = smc_get_reg(1, ioaddr, ADDR2_REG);
		break;

	case CTL_SMC_REG_GPR:	// General Purpose
		*valp = smc_get_reg(1, ioaddr, GP_REG);
		break;

	case CTL_SMC_REG_CTLR:	// Control
		*valp = smc_get_reg(1, ioaddr, CTL_REG);
		break;

	case CTL_SMC_REG_MCR:	// MMU Command
		*valp = smc_get_reg(2, ioaddr, MMU_CMD_REG);
		break;

	case CTL_SMC_REG_PNR:	// Packet Number
		*valp = smc_get_reg(2, ioaddr, PN_REG);
		break;

	case CTL_SMC_REG_FPR:	// Allocation Result/FIFO Ports
		*valp = smc_get_reg(2, ioaddr, RXFIFO_REG);
		break;

	case CTL_SMC_REG_PTR:	// Pointer
		*valp = smc_get_reg(2, ioaddr, PTR_REG);
		break;

	case CTL_SMC_REG_DR:	// Data 
		*valp = smc_get_reg(2, ioaddr, DATA_REG);
		break;

	case CTL_SMC_REG_ISR:	// Interrupt Status/Mask
		*valp = smc_get_reg(2, ioaddr, INT_REG);
		break;

	case CTL_SMC_REG_MTR1:	// Multicast Table Entry 1
		*valp = smc_get_reg(3, ioaddr, MCAST_REG1);
		break;

	case CTL_SMC_REG_MTR2:	// Multicast Table Entry 2
		*valp = smc_get_reg(3, ioaddr, MCAST_REG2);
		break;

	case CTL_SMC_REG_MTR3:	// Multicast Table Entry 3
		*valp = smc_get_reg(3, ioaddr, MCAST_REG3);
		break;

	case CTL_SMC_REG_MTR4:	// Multicast Table Entry 4
		*valp = smc_get_reg(3, ioaddr, MCAST_REG4);
		break;

	case CTL_SMC_REG_MIIR:	// Management Interface
		*valp = smc_get_reg(3, ioaddr, MII_REG);
		break;

	case CTL_SMC_REG_REVR:	// Revision
		*valp = smc_get_reg(3, ioaddr, REV_REG);
		break;

	case CTL_SMC_REG_ERCVR:	// Early RCV
		*valp = smc_get_reg(3, ioaddr, ERCV_REG);
		break;

	case CTL_SMC_REG_EXTR:	// External
		*valp = smc_get_reg(7, ioaddr, EXT_REG);
		break;

	case CTL_SMC_PHY_CTRL:
		*valp =
		    smc_read_phy_register(ioaddr, lp->phyaddr, PHY_CNTL_REG);
		break;

	case CTL_SMC_PHY_STAT:
		*valp =
		    smc_read_phy_register(ioaddr, lp->phyaddr, PHY_STAT_REG);
		break;

	case CTL_SMC_PHY_ID1:
		*valp = smc_read_phy_register(ioaddr, lp->phyaddr, PHY_ID1_REG);
		break;

	case CTL_SMC_PHY_ID2:
		*valp = smc_read_phy_register(ioaddr, lp->phyaddr, PHY_ID2_REG);
		break;

	case CTL_SMC_PHY_ADC:
		*valp = smc_read_phy_register(ioaddr, lp->phyaddr, PHY_AD_REG);
		break;

	case CTL_SMC_PHY_REMC:
		*valp = smc_read_phy_register(ioaddr, lp->phyaddr, PHY_RMT_REG);
		break;

	case CTL_SMC_PHY_CFG1:
		*valp =
		    smc_read_phy_register(ioaddr, lp->phyaddr, PHY_CFG1_REG);
		break;

	case CTL_SMC_PHY_CFG2:
		*valp =
		    smc_read_phy_register(ioaddr, lp->phyaddr, PHY_CFG2_REG);
		break;

	case CTL_SMC_PHY_INT:
		*valp = smc_read_phy_register(ioaddr, lp->phyaddr, PHY_INT_REG);
		break;

	case CTL_SMC_PHY_MASK:
		*valp =
		    smc_read_phy_register(ioaddr, lp->phyaddr, PHY_MASK_REG);
		break;

#endif				// SMC_DEBUG

	default:
		// Just ignore unsupported parameters
		break;
	}

	// Save old state
	val = *valp;

	// Perform the generic integer operation        
	if ((ret = proc_dointvec(ctl, write, filp, buffer, lenp, 0)) != 0)
		return (ret);

	// Write changes out to the registers
	if (write && *valp != val) {

		val = *valp;
		switch (ctl->ctl_name) {

		case CTL_SMC_SWFDUP:
			if (val)
				lp->tcr_cur_mode |= TCR_SWFDUP;
			else
				lp->tcr_cur_mode &= ~TCR_SWFDUP;

			smc_modify_regbit(0, ioaddr, TCR_REG, TCR_SWFDUP, val);
			break;

		case CTL_SMC_EPHLOOP:
			if (val)
				lp->tcr_cur_mode |= TCR_EPH_LOOP;
			else
				lp->tcr_cur_mode &= ~TCR_EPH_LOOP;

			smc_modify_regbit(0, ioaddr, TCR_REG, TCR_EPH_LOOP,
					  val);
			break;

		case CTL_SMC_FORCOL:
			if (val)
				lp->tcr_cur_mode |= TCR_FORCOL;
			else
				lp->tcr_cur_mode &= ~TCR_FORCOL;

			// Update the EPH block
			smc_modify_regbit(0, ioaddr, TCR_REG, TCR_FORCOL, val);
			break;

		case CTL_SMC_FILTCAR:
			if (val)
				lp->rcr_cur_mode |= RCR_FILT_CAR;
			else
				lp->rcr_cur_mode &= ~RCR_FILT_CAR;

			// Update the EPH block
			smc_modify_regbit(0, ioaddr, RCR_REG, RCR_FILT_CAR,
					  val);
			break;

		case CTL_SMC_RFDUPLX:
			// Disallow changes if in auto-negotiation mode
			if (lp->ctl_autoneg)
				break;

			if (val) {
				lp->rpc_cur_mode |= RPC_DPLX;
			} else {
				lp->rpc_cur_mode &= ~RPC_DPLX;
			}

			// Reconfigure the PHY
			smc_phy_configure(dev);

			break;

		case CTL_SMC_RSPEED:
			// Disallow changes if in auto-negotiation mode
			if (lp->ctl_autoneg)
				break;

			if (val > 10)
				lp->rpc_cur_mode |= RPC_SPEED;
			else
				lp->rpc_cur_mode &= ~RPC_SPEED;

			// Reconfigure the PHY
			smc_phy_configure(dev);

			break;

		case CTL_SMC_AUTONEG:
			if (val)
				lp->rpc_cur_mode |= RPC_ANEG;
			else
				lp->rpc_cur_mode &= ~RPC_ANEG;

			// Reconfigure the PHY
			smc_phy_configure(dev);

			break;

		case CTL_SMC_LEDA:
			val &= 0x07;	// Restrict to 3 ls bits
			lp->rpc_cur_mode &= ~(word) (0x07 << RPC_LSXA_SHFT);
			lp->rpc_cur_mode |= (word) (val << RPC_LSXA_SHFT);

			// Update the Internal PHY block
			smc_modify_reg(0, ioaddr, RPC_REG, lp->rpc_cur_mode);
			break;

		case CTL_SMC_LEDB:
			val &= 0x07;	// Restrict to 3 ls bits
			lp->rpc_cur_mode &= ~(word) (0x07 << RPC_LSXB_SHFT);
			lp->rpc_cur_mode |= (word) (val << RPC_LSXB_SHFT);

			// Update the Internal PHY block
			smc_modify_reg(0, ioaddr, RPC_REG, lp->rpc_cur_mode);
			break;

		case CTL_SMC_MIIOP:
			// Update the Internal PHY block
			smc_modify_regbit(1, ioaddr, CONFIG_REG, CONFIG_EXT_PHY,
					  val);
			break;

#ifdef SMC_DEBUG
		case CTL_SMC_REG_BSR:	// Bank Select
			smc_modify_reg(0, ioaddr, BSR_REG, val);
			break;

		case CTL_SMC_REG_TCR:	// Transmit Control
			smc_modify_reg(0, ioaddr, TCR_REG, val);
			break;

		case CTL_SMC_REG_ESR:	// EPH Status
			smc_modify_reg(0, ioaddr, EPH_STATUS_REG, val);
			break;

		case CTL_SMC_REG_RCR:	// Receive Control
			smc_modify_reg(0, ioaddr, RCR_REG, val);
			break;

		case CTL_SMC_REG_CTRR:	// Counter
			smc_modify_reg(0, ioaddr, COUNTER_REG, val);
			break;

		case CTL_SMC_REG_MIR:	// Memory Information
			smc_modify_reg(0, ioaddr, MIR_REG, val);
			break;

		case CTL_SMC_REG_RPCR:	// Receive/Phy Control
			smc_modify_reg(0, ioaddr, RPC_REG, val);
			break;

		case CTL_SMC_REG_CFGR:	// Configuration
			smc_modify_reg(1, ioaddr, CONFIG_REG, val);
			break;

		case CTL_SMC_REG_BAR:	// Base Address
			smc_modify_reg(1, ioaddr, BASE_REG, val);
			break;

		case CTL_SMC_REG_IAR0:	// Individual Address
			smc_modify_reg(1, ioaddr, ADDR0_REG, val);
			break;

		case CTL_SMC_REG_IAR1:	// Individual Address
			smc_modify_reg(1, ioaddr, ADDR1_REG, val);
			break;

		case CTL_SMC_REG_IAR2:	// Individual Address
			smc_modify_reg(1, ioaddr, ADDR2_REG, val);
			break;

		case CTL_SMC_REG_GPR:	// General Purpose
			smc_modify_reg(1, ioaddr, GP_REG, val);
			break;

		case CTL_SMC_REG_CTLR:	// Control
			smc_modify_reg(1, ioaddr, CTL_REG, val);
			break;

		case CTL_SMC_REG_MCR:	// MMU Command
			smc_modify_reg(2, ioaddr, MMU_CMD_REG, val);
			break;

		case CTL_SMC_REG_PNR:	// Packet Number
			smc_modify_reg(2, ioaddr, PN_REG, val);
			break;

		case CTL_SMC_REG_FPR:	// Allocation Result/FIFO Ports
			smc_modify_reg(2, ioaddr, RXFIFO_REG, val);
			break;

		case CTL_SMC_REG_PTR:	// Pointer
			smc_modify_reg(2, ioaddr, PTR_REG, val);
			break;

		case CTL_SMC_REG_DR:	// Data 
			smc_modify_reg(2, ioaddr, DATA_REG, val);
			break;

		case CTL_SMC_REG_ISR:	// Interrupt Status/Mask
			smc_modify_reg(2, ioaddr, INT_REG, val);
			break;

		case CTL_SMC_REG_MTR1:	// Multicast Table Entry 1
			smc_modify_reg(3, ioaddr, MCAST_REG1, val);
			break;

		case CTL_SMC_REG_MTR2:	// Multicast Table Entry 2
			smc_modify_reg(3, ioaddr, MCAST_REG2, val);
			break;

		case CTL_SMC_REG_MTR3:	// Multicast Table Entry 3
			smc_modify_reg(3, ioaddr, MCAST_REG3, val);
			break;

		case CTL_SMC_REG_MTR4:	// Multicast Table Entry 4
			smc_modify_reg(3, ioaddr, MCAST_REG4, val);
			break;

		case CTL_SMC_REG_MIIR:	// Management Interface
			smc_modify_reg(3, ioaddr, MII_REG, val);
			break;

		case CTL_SMC_REG_REVR:	// Revision
			smc_modify_reg(3, ioaddr, REV_REG, val);
			break;

		case CTL_SMC_REG_ERCVR:	// Early RCV
			smc_modify_reg(3, ioaddr, ERCV_REG, val);
			break;

		case CTL_SMC_REG_EXTR:	// External
			smc_modify_reg(7, ioaddr, EXT_REG, val);
			break;

		case CTL_SMC_PHY_CTRL:
			smc_write_phy_register(ioaddr, lp->phyaddr,
					       PHY_CNTL_REG, val);
			break;

		case CTL_SMC_PHY_STAT:
			smc_write_phy_register(ioaddr, lp->phyaddr,
					       PHY_STAT_REG, val);
			break;

		case CTL_SMC_PHY_ID1:
			smc_write_phy_register(ioaddr, lp->phyaddr, PHY_ID1_REG,
					       val);
			break;

		case CTL_SMC_PHY_ID2:
			smc_write_phy_register(ioaddr, lp->phyaddr, PHY_ID2_REG,
					       val);
			break;

		case CTL_SMC_PHY_ADC:
			smc_write_phy_register(ioaddr, lp->phyaddr, PHY_AD_REG,
					       val);
			break;

		case CTL_SMC_PHY_REMC:
			smc_write_phy_register(ioaddr, lp->phyaddr, PHY_RMT_REG,
					       val);
			break;

		case CTL_SMC_PHY_CFG1:
			smc_write_phy_register(ioaddr, lp->phyaddr,
					       PHY_CFG1_REG, val);
			break;

		case CTL_SMC_PHY_CFG2:
			smc_write_phy_register(ioaddr, lp->phyaddr,
					       PHY_CFG2_REG, val);
			break;

		case CTL_SMC_PHY_INT:
			smc_write_phy_register(ioaddr, lp->phyaddr, PHY_INT_REG,
					       val);
			break;

		case CTL_SMC_PHY_MASK:
			smc_write_phy_register(ioaddr, lp->phyaddr,
					       PHY_MASK_REG, val);
			break;

#endif				// SMC_DEBUG

		default:
			// Just ignore unsupported parameters
			break;
		}		// end switch

	}			// end if

	return ret;
}

/*------------------------------------------------------------
 . Sysctl registration function for all parameters (files)
 .-------------------------------------------------------------*/
static void smc_sysctl_register(struct net_device *dev)
{
	struct smc_local *lp = netdev_priv(dev);
	static int ctl_name = CTL_SMC;
	ctl_table *ct;
	int i;

	// Make sure the ctl_tables start out as all zeros
	memset(lp->root_table, 0, sizeof lp->root_table);
	memset(lp->eth_table, 0, sizeof lp->eth_table);
	memset(lp->param_table, 0, sizeof lp->param_table);

	// Initialize the root table
	ct = lp->root_table;
	ct->ctl_name = CTL_DEV;
	ct->procname = "dev";
	ct->maxlen = 0;
	ct->mode = 0555;
	ct->child = lp->eth_table;
	// remaining fields are zero

	// Initialize the ethX table (this device's table)
	ct = lp->eth_table;
	ct->ctl_name = ctl_name++;	// Must be unique
	ct->procname = dev->name;
	ct->maxlen = 0;
	ct->mode = 0555;
	ct->child = lp->param_table;
	// remaining fields are zero

	// Initialize the parameter (files) table
	// Make sure the last entry remains null
	ct = lp->param_table;
	for (i = 0; i < (CTL_SMC_LAST_ENTRY - 1); ++i) {
		// Initialize fields common to all table entries
		ct[i].proc_handler = smc_sysctl_handler;
		ct[i].extra1 = (void *)dev;	// Save our device pointer
		ct[i].extra2 = (void *)lp;	// Save our smc_local data pointer
	}

	// INFO - this is our only string parameter
	i = 0;
	ct[i].proc_handler = proc_dostring;	// use default handler
	ct[i].ctl_name = CTL_SMC_INFO;
	ct[i].procname = "info";
	ct[i].data = (void *)smc_info_string;
	ct[i].maxlen = sizeof smc_info_string;
	ct[i].mode = 0444;	// Read only

	// SWVER
	++i;
	ct[i].proc_handler = proc_dostring;	// use default handler
	ct[i].ctl_name = CTL_SMC_SWVER;
	ct[i].procname = "swver";
	ct[i].data = (void *)version;
	ct[i].maxlen = sizeof version;
	ct[i].mode = 0444;	// Read only

	// SWFDUP
	++i;
	ct[i].ctl_name = CTL_SMC_SWFDUP;
	ct[i].procname = "swfdup";
	ct[i].data = (void *)&(lp->ctl_swfdup);
	ct[i].maxlen = sizeof lp->ctl_swfdup;
	ct[i].mode = 0644;	// Read by all, write by root

	// EPHLOOP
	++i;
	ct[i].ctl_name = CTL_SMC_EPHLOOP;
	ct[i].procname = "ephloop";
	ct[i].data = (void *)&(lp->ctl_ephloop);
	ct[i].maxlen = sizeof lp->ctl_ephloop;
	ct[i].mode = 0644;	// Read by all, write by root

	// MIIOP
	++i;
	ct[i].ctl_name = CTL_SMC_MIIOP;
	ct[i].procname = "miiop";
	ct[i].data = (void *)&(lp->ctl_miiop);
	ct[i].maxlen = sizeof lp->ctl_miiop;
	ct[i].mode = 0644;	// Read by all, write by root

	// AUTONEG
	++i;
	ct[i].ctl_name = CTL_SMC_AUTONEG;
	ct[i].procname = "autoneg";
	ct[i].data = (void *)&(lp->ctl_autoneg);
	ct[i].maxlen = sizeof lp->ctl_autoneg;
	ct[i].mode = 0644;	// Read by all, write by root

	// RFDUPLX
	++i;
	ct[i].ctl_name = CTL_SMC_RFDUPLX;
	ct[i].procname = "rfduplx";
	ct[i].data = (void *)&(lp->ctl_rfduplx);
	ct[i].maxlen = sizeof lp->ctl_rfduplx;
	ct[i].mode = 0644;	// Read by all, write by root

	// RSPEED
	++i;
	ct[i].ctl_name = CTL_SMC_RSPEED;
	ct[i].procname = "rspeed";
	ct[i].data = (void *)&(lp->ctl_rspeed);
	ct[i].maxlen = sizeof lp->ctl_rspeed;
	ct[i].mode = 0644;	// Read by all, write by root

	// AFDUPLX
	++i;
	ct[i].ctl_name = CTL_SMC_AFDUPLX;
	ct[i].procname = "afduplx";
	ct[i].data = (void *)&(lp->ctl_afduplx);
	ct[i].maxlen = sizeof lp->ctl_afduplx;
	ct[i].mode = 0444;	// Read only

	// ASPEED
	++i;
	ct[i].ctl_name = CTL_SMC_ASPEED;
	ct[i].procname = "aspeed";
	ct[i].data = (void *)&(lp->ctl_aspeed);
	ct[i].maxlen = sizeof lp->ctl_aspeed;
	ct[i].mode = 0444;	// Read only

	// LNKFAIL
	++i;
	ct[i].ctl_name = CTL_SMC_LNKFAIL;
	ct[i].procname = "lnkfail";
	ct[i].data = (void *)&(lp->ctl_lnkfail);
	ct[i].maxlen = sizeof lp->ctl_lnkfail;
	ct[i].mode = 0444;	// Read only

	// FORCOL
	++i;
	ct[i].ctl_name = CTL_SMC_FORCOL;
	ct[i].procname = "forcol";
	ct[i].data = (void *)&(lp->ctl_forcol);
	ct[i].maxlen = sizeof lp->ctl_forcol;
	ct[i].mode = 0644;	// Read by all, write by root

	// FILTCAR
	++i;
	ct[i].ctl_name = CTL_SMC_FILTCAR;
	ct[i].procname = "filtcar";
	ct[i].data = (void *)&(lp->ctl_filtcar);
	ct[i].maxlen = sizeof lp->ctl_filtcar;
	ct[i].mode = 0644;	// Read by all, write by root

	// FREEMEM
	++i;
	ct[i].ctl_name = CTL_SMC_FREEMEM;
	ct[i].procname = "freemem";
	ct[i].data = (void *)&(lp->ctl_freemem);
	ct[i].maxlen = sizeof lp->ctl_freemem;
	ct[i].mode = 0444;	// Read only

	// TOTMEM
	++i;
	ct[i].ctl_name = CTL_SMC_TOTMEM;
	ct[i].procname = "totmem";
	ct[i].data = (void *)&(lp->ctl_totmem);
	ct[i].maxlen = sizeof lp->ctl_totmem;
	ct[i].mode = 0444;	// Read only

	// LEDA
	++i;
	ct[i].ctl_name = CTL_SMC_LEDA;
	ct[i].procname = "leda";
	ct[i].data = (void *)&(lp->ctl_leda);
	ct[i].maxlen = sizeof lp->ctl_leda;
	ct[i].mode = 0644;	// Read by all, write by root

	// LEDB
	++i;
	ct[i].ctl_name = CTL_SMC_LEDB;
	ct[i].procname = "ledb";
	ct[i].data = (void *)&(lp->ctl_ledb);
	ct[i].maxlen = sizeof lp->ctl_ledb;
	ct[i].mode = 0644;	// Read by all, write by root

	// CHIPREV
	++i;
	ct[i].ctl_name = CTL_SMC_CHIPREV;
	ct[i].procname = "chiprev";
	ct[i].data = (void *)&(lp->ctl_chiprev);
	ct[i].maxlen = sizeof lp->ctl_chiprev;
	ct[i].mode = 0444;	// Read only

#ifdef SMC_DEBUG
	// REG_BSR
	++i;
	ct[i].ctl_name = CTL_SMC_REG_BSR;
	ct[i].procname = "reg_bsr";
	ct[i].data = (void *)&(lp->ctl_reg_bsr);
	ct[i].maxlen = sizeof lp->ctl_reg_bsr;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_TCR
	++i;
	ct[i].ctl_name = CTL_SMC_REG_TCR;
	ct[i].procname = "reg_tcr";
	ct[i].data = (void *)&(lp->ctl_reg_tcr);
	ct[i].maxlen = sizeof lp->ctl_reg_tcr;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_ESR
	++i;
	ct[i].ctl_name = CTL_SMC_REG_ESR;
	ct[i].procname = "reg_esr";
	ct[i].data = (void *)&(lp->ctl_reg_esr);
	ct[i].maxlen = sizeof lp->ctl_reg_esr;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_RCR
	++i;
	ct[i].ctl_name = CTL_SMC_REG_RCR;
	ct[i].procname = "reg_rcr";
	ct[i].data = (void *)&(lp->ctl_reg_rcr);
	ct[i].maxlen = sizeof lp->ctl_reg_rcr;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_CTRR
	++i;
	ct[i].ctl_name = CTL_SMC_REG_CTRR;
	ct[i].procname = "reg_ctrr";
	ct[i].data = (void *)&(lp->ctl_reg_ctrr);
	ct[i].maxlen = sizeof lp->ctl_reg_ctrr;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_MIR
	++i;
	ct[i].ctl_name = CTL_SMC_REG_MIR;
	ct[i].procname = "reg_mir";
	ct[i].data = (void *)&(lp->ctl_reg_mir);
	ct[i].maxlen = sizeof lp->ctl_reg_mir;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_RPCR
	++i;
	ct[i].ctl_name = CTL_SMC_REG_RPCR;
	ct[i].procname = "reg_rpcr";
	ct[i].data = (void *)&(lp->ctl_reg_rpcr);
	ct[i].maxlen = sizeof lp->ctl_reg_rpcr;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_CFGR
	++i;
	ct[i].ctl_name = CTL_SMC_REG_CFGR;
	ct[i].procname = "reg_cfgr";
	ct[i].data = (void *)&(lp->ctl_reg_cfgr);
	ct[i].maxlen = sizeof lp->ctl_reg_cfgr;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_BAR
	++i;
	ct[i].ctl_name = CTL_SMC_REG_BAR;
	ct[i].procname = "reg_bar";
	ct[i].data = (void *)&(lp->ctl_reg_bar);
	ct[i].maxlen = sizeof lp->ctl_reg_bar;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_IAR0
	++i;
	ct[i].ctl_name = CTL_SMC_REG_IAR0;
	ct[i].procname = "reg_iar0";
	ct[i].data = (void *)&(lp->ctl_reg_iar0);
	ct[i].maxlen = sizeof lp->ctl_reg_iar0;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_IAR1
	++i;
	ct[i].ctl_name = CTL_SMC_REG_IAR1;
	ct[i].procname = "reg_iar1";
	ct[i].data = (void *)&(lp->ctl_reg_iar1);
	ct[i].maxlen = sizeof lp->ctl_reg_iar1;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_IAR2
	++i;
	ct[i].ctl_name = CTL_SMC_REG_IAR2;
	ct[i].procname = "reg_iar2";
	ct[i].data = (void *)&(lp->ctl_reg_iar2);
	ct[i].maxlen = sizeof lp->ctl_reg_iar2;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_GPR
	++i;
	ct[i].ctl_name = CTL_SMC_REG_GPR;
	ct[i].procname = "reg_gpr";
	ct[i].data = (void *)&(lp->ctl_reg_gpr);
	ct[i].maxlen = sizeof lp->ctl_reg_gpr;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_CTLR
	++i;
	ct[i].ctl_name = CTL_SMC_REG_CTLR;
	ct[i].procname = "reg_ctlr";
	ct[i].data = (void *)&(lp->ctl_reg_ctlr);
	ct[i].maxlen = sizeof lp->ctl_reg_ctlr;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_MCR
	++i;
	ct[i].ctl_name = CTL_SMC_REG_MCR;
	ct[i].procname = "reg_mcr";
	ct[i].data = (void *)&(lp->ctl_reg_mcr);
	ct[i].maxlen = sizeof lp->ctl_reg_mcr;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_PNR
	++i;
	ct[i].ctl_name = CTL_SMC_REG_PNR;
	ct[i].procname = "reg_pnr";
	ct[i].data = (void *)&(lp->ctl_reg_pnr);
	ct[i].maxlen = sizeof lp->ctl_reg_pnr;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_FPR
	++i;
	ct[i].ctl_name = CTL_SMC_REG_FPR;
	ct[i].procname = "reg_fpr";
	ct[i].data = (void *)&(lp->ctl_reg_fpr);
	ct[i].maxlen = sizeof lp->ctl_reg_fpr;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_PTR
	++i;
	ct[i].ctl_name = CTL_SMC_REG_PTR;
	ct[i].procname = "reg_ptr";
	ct[i].data = (void *)&(lp->ctl_reg_ptr);
	ct[i].maxlen = sizeof lp->ctl_reg_ptr;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_DR
	++i;
	ct[i].ctl_name = CTL_SMC_REG_DR;
	ct[i].procname = "reg_dr";
	ct[i].data = (void *)&(lp->ctl_reg_dr);
	ct[i].maxlen = sizeof lp->ctl_reg_dr;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_ISR
	++i;
	ct[i].ctl_name = CTL_SMC_REG_ISR;
	ct[i].procname = "reg_isr";
	ct[i].data = (void *)&(lp->ctl_reg_isr);
	ct[i].maxlen = sizeof lp->ctl_reg_isr;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_MTR1
	++i;
	ct[i].ctl_name = CTL_SMC_REG_MTR1;
	ct[i].procname = "reg_mtr1";
	ct[i].data = (void *)&(lp->ctl_reg_mtr1);
	ct[i].maxlen = sizeof lp->ctl_reg_mtr1;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_MTR2
	++i;
	ct[i].ctl_name = CTL_SMC_REG_MTR2;
	ct[i].procname = "reg_mtr2";
	ct[i].data = (void *)&(lp->ctl_reg_mtr2);
	ct[i].maxlen = sizeof lp->ctl_reg_mtr2;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_MTR3
	++i;
	ct[i].ctl_name = CTL_SMC_REG_MTR3;
	ct[i].procname = "reg_mtr3";
	ct[i].data = (void *)&(lp->ctl_reg_mtr3);
	ct[i].maxlen = sizeof lp->ctl_reg_mtr3;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_MTR4
	++i;
	ct[i].ctl_name = CTL_SMC_REG_MTR4;
	ct[i].procname = "reg_mtr4";
	ct[i].data = (void *)&(lp->ctl_reg_mtr4);
	ct[i].maxlen = sizeof lp->ctl_reg_mtr4;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_MIIR
	++i;
	ct[i].ctl_name = CTL_SMC_REG_MIIR;
	ct[i].procname = "reg_miir";
	ct[i].data = (void *)&(lp->ctl_reg_miir);
	ct[i].maxlen = sizeof lp->ctl_reg_miir;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_REVR
	++i;
	ct[i].ctl_name = CTL_SMC_REG_REVR;
	ct[i].procname = "reg_revr";
	ct[i].data = (void *)&(lp->ctl_reg_revr);
	ct[i].maxlen = sizeof lp->ctl_reg_revr;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_ERCVR
	++i;
	ct[i].ctl_name = CTL_SMC_REG_ERCVR;
	ct[i].procname = "reg_ercvr";
	ct[i].data = (void *)&(lp->ctl_reg_ercvr);
	ct[i].maxlen = sizeof lp->ctl_reg_ercvr;
	ct[i].mode = 0644;	// Read by all, write by root

	// REG_EXTR
	++i;
	ct[i].ctl_name = CTL_SMC_REG_EXTR;
	ct[i].procname = "reg_extr";
	ct[i].data = (void *)&(lp->ctl_reg_extr);
	ct[i].maxlen = sizeof lp->ctl_reg_extr;
	ct[i].mode = 0644;	// Read by all, write by root

	// PHY Control
	++i;
	ct[i].ctl_name = CTL_SMC_PHY_CTRL;
	ct[i].procname = "phy_ctrl";
	ct[i].data = (void *)&(lp->ctl_phy_ctrl);
	ct[i].maxlen = sizeof lp->ctl_phy_ctrl;
	ct[i].mode = 0644;	// Read by all, write by root

	// PHY Status
	++i;
	ct[i].ctl_name = CTL_SMC_PHY_STAT;
	ct[i].procname = "phy_stat";
	ct[i].data = (void *)&(lp->ctl_phy_stat);
	ct[i].maxlen = sizeof lp->ctl_phy_stat;
	ct[i].mode = 0644;	// Read by all, write by root

	// PHY ID1
	++i;
	ct[i].ctl_name = CTL_SMC_PHY_ID1;
	ct[i].procname = "phy_id1";
	ct[i].data = (void *)&(lp->ctl_phy_id1);
	ct[i].maxlen = sizeof lp->ctl_phy_id1;
	ct[i].mode = 0644;	// Read by all, write by root

	// PHY ID2
	++i;
	ct[i].ctl_name = CTL_SMC_PHY_ID2;
	ct[i].procname = "phy_id2";
	ct[i].data = (void *)&(lp->ctl_phy_id2);
	ct[i].maxlen = sizeof lp->ctl_phy_id2;
	ct[i].mode = 0644;	// Read by all, write by root

	// PHY Advertise Capabilities
	++i;
	ct[i].ctl_name = CTL_SMC_PHY_ADC;
	ct[i].procname = "phy_adc";
	ct[i].data = (void *)&(lp->ctl_phy_adc);
	ct[i].maxlen = sizeof lp->ctl_phy_adc;
	ct[i].mode = 0644;	// Read by all, write by root

	// PHY Remote Capabilities
	++i;
	ct[i].ctl_name = CTL_SMC_PHY_REMC;
	ct[i].procname = "phy_remc";
	ct[i].data = (void *)&(lp->ctl_phy_remc);
	ct[i].maxlen = sizeof lp->ctl_phy_remc;
	ct[i].mode = 0644;	// Read by all, write by root

	// PHY Configuration 1
	++i;
	ct[i].ctl_name = CTL_SMC_PHY_CFG1;
	ct[i].procname = "phy_cfg1";
	ct[i].data = (void *)&(lp->ctl_phy_cfg1);
	ct[i].maxlen = sizeof lp->ctl_phy_cfg1;
	ct[i].mode = 0644;	// Read by all, write by root

	// PHY Configuration 2
	++i;
	ct[i].ctl_name = CTL_SMC_PHY_CFG2;
	ct[i].procname = "phy_cfg2";
	ct[i].data = (void *)&(lp->ctl_phy_cfg2);
	ct[i].maxlen = sizeof lp->ctl_phy_cfg2;
	ct[i].mode = 0644;	// Read by all, write by root

	// PHY Interrupt/Status Output
	++i;
	ct[i].ctl_name = CTL_SMC_PHY_INT;
	ct[i].procname = "phy_int";
	ct[i].data = (void *)&(lp->ctl_phy_int);
	ct[i].maxlen = sizeof lp->ctl_phy_int;
	ct[i].mode = 0644;	// Read by all, write by root

	// PHY Interrupt/Status Mask
	++i;
	ct[i].ctl_name = CTL_SMC_PHY_MASK;
	ct[i].procname = "phy_mask";
	ct[i].data = (void *)&(lp->ctl_phy_mask);
	ct[i].maxlen = sizeof lp->ctl_phy_mask;
	ct[i].mode = 0644;	// Read by all, write by root

#endif				// SMC_DEBUG

	// Register /proc/sys/dev/ethX
	lp->sysctl_header = register_sysctl_table(lp->root_table);
}

/*------------------------------------------------------------
 . Sysctl unregistration when driver is closed
 .-------------------------------------------------------------*/
static void smc_sysctl_unregister(struct net_device *dev)
{
	struct smc_local *lp = netdev_priv(dev);

	unregister_sysctl_table(lp->sysctl_header);
}

#endif				/* endif CONFIG_SYSCTL */

//---PHY CONTROL AND CONFIGURATION-----------------------------------------

#if (SMC_DEBUG > 2 )

/*------------------------------------------------------------
 . Debugging function for viewing MII Management serial bitstream
 .-------------------------------------------------------------*/
static void smc_dump_mii_stream(byte * bits, int size)
{
	int i;

	printk("BIT#:");
	for (i = 0; i < size; ++i) {
		printk("%d", i % 10);
	}

	printk("\nMDOE:");
	for (i = 0; i < size; ++i) {
		if (bits[i] & MII_MDOE)
			printk("1");
		else
			printk("0");
	}

	printk("\nMDO :");
	for (i = 0; i < size; ++i) {
		if (bits[i] & MII_MDO)
			printk("1");
		else
			printk("0");
	}

	printk("\nMDI :");
	for (i = 0; i < size; ++i) {
		if (bits[i] & MII_MDI)
			printk("1");
		else
			printk("0");
	}

	printk("\n");
}
#endif

/*------------------------------------------------------------
 . Reads a register from the MII Management serial interface
 .-------------------------------------------------------------*/
static word
smc_read_phy_register(unsigned int ioaddr, byte phyaddr, byte phyreg)
{
	int oldBank;
	int i;
	byte mask;
	word mii_reg;
	byte bits[64];
	int clk_idx = 0;
	int input_idx;
	word phydata;

	// 32 consecutive ones on MDO to establish sync
	for (i = 0; i < 32; ++i)
		bits[clk_idx++] = MII_MDOE | MII_MDO;

	// Start code <01>
	bits[clk_idx++] = MII_MDOE;
	bits[clk_idx++] = MII_MDOE | MII_MDO;

	// Read command <10>
	bits[clk_idx++] = MII_MDOE | MII_MDO;
	bits[clk_idx++] = MII_MDOE;

	// Output the PHY address, msb first
	mask = (byte) 0x10;
	for (i = 0; i < 5; ++i) {
		if (phyaddr & mask)
			bits[clk_idx++] = MII_MDOE | MII_MDO;
		else
			bits[clk_idx++] = MII_MDOE;

		// Shift to next lowest bit
		mask >>= 1;
	}

	// Output the phy register number, msb first
	mask = (byte) 0x10;
	for (i = 0; i < 5; ++i) {
		if (phyreg & mask)
			bits[clk_idx++] = MII_MDOE | MII_MDO;
		else
			bits[clk_idx++] = MII_MDOE;

		// Shift to next lowest bit
		mask >>= 1;
	}

	// Tristate and turnaround (2 bit times)
	bits[clk_idx++] = 0;
	//bits[clk_idx++] = 0;

	// Input starts at this bit time
	input_idx = clk_idx;

	// Will input 16 bits
	for (i = 0; i < 16; ++i)
		bits[clk_idx++] = 0;

	// Final clock bit
	bits[clk_idx++] = 0;

	// Save the current bank
	oldBank = SMC_inw(BANK_SELECT);

	// Select bank 3
	SMC_SELECT_BANK(3);

	// Get the current MII register value
	mii_reg = SMC_inw(MII_REG);

	// Turn off all MII Interface bits
	mii_reg &= ~(MII_MDOE | MII_MCLK | MII_MDI | MII_MDO);

	// Clock all 64 cycles
	for (i = 0; i < sizeof bits; ++i) {
		// Clock Low - output data
		SMC_outw(mii_reg | bits[i], MII_REG);
		udelay(50);

		// Clock Hi - input data
		SMC_outw(mii_reg | bits[i] | MII_MCLK, MII_REG);
		udelay(50);
		bits[i] |= SMC_inw(MII_REG) & MII_MDI;
	}

	// Return to idle state
	// Set clock to low, data to low, and output tristated
	SMC_outw(mii_reg, MII_REG);
	udelay(50);

	// Restore original bank select
	SMC_SELECT_BANK(oldBank);

	// Recover input data
	phydata = 0;
	for (i = 0; i < 16; ++i) {
		phydata <<= 1;

		if (bits[input_idx++] & MII_MDI)
			phydata |= 0x0001;
	}

#if (SMC_DEBUG > 2 )
	printk("smc_read_phy_register(): phyaddr=%x,phyreg=%x,phydata=%x\n",
	       phyaddr, phyreg, phydata);
	smc_dump_mii_stream(bits, sizeof bits);
#endif

	return (phydata);
}

/*------------------------------------------------------------
 . Writes a register to the MII Management serial interface
 .-------------------------------------------------------------*/
static void
smc_write_phy_register(unsigned int ioaddr,
		       byte phyaddr, byte phyreg, word phydata)
{
	int oldBank;
	int i;
	word mask;
	word mii_reg;
	byte bits[65];
	int clk_idx = 0;

	// 32 consecutive ones on MDO to establish sync
	for (i = 0; i < 32; ++i)
		bits[clk_idx++] = MII_MDOE | MII_MDO;

	// Start code <01>
	bits[clk_idx++] = MII_MDOE;
	bits[clk_idx++] = MII_MDOE | MII_MDO;

	// Write command <01>
	bits[clk_idx++] = MII_MDOE;
	bits[clk_idx++] = MII_MDOE | MII_MDO;

	// Output the PHY address, msb first
	mask = (byte) 0x10;
	for (i = 0; i < 5; ++i) {
		if (phyaddr & mask)
			bits[clk_idx++] = MII_MDOE | MII_MDO;
		else
			bits[clk_idx++] = MII_MDOE;

		// Shift to next lowest bit
		mask >>= 1;
	}

	// Output the phy register number, msb first
	mask = (byte) 0x10;
	for (i = 0; i < 5; ++i) {
		if (phyreg & mask)
			bits[clk_idx++] = MII_MDOE | MII_MDO;
		else
			bits[clk_idx++] = MII_MDOE;

		// Shift to next lowest bit
		mask >>= 1;
	}

	// Tristate and turnaround (2 bit times)
	bits[clk_idx++] = 0;
	bits[clk_idx++] = 0;

	// Write out 16 bits of data, msb first
	mask = 0x8000;
	for (i = 0; i < 16; ++i) {
		if (phydata & mask)
			bits[clk_idx++] = MII_MDOE | MII_MDO;
		else
			bits[clk_idx++] = MII_MDOE;

		// Shift to next lowest bit
		mask >>= 1;
	}

	// Final clock bit (tristate)
	bits[clk_idx++] = 0;

	// Save the current bank
	oldBank = SMC_inw(BANK_SELECT);

	// Select bank 3
	SMC_SELECT_BANK(3);

	// Get the current MII register value
	mii_reg = SMC_inw(MII_REG);

	// Turn off all MII Interface bits
	mii_reg &= ~(MII_MDOE | MII_MCLK | MII_MDI | MII_MDO);

	// Clock all cycles
	for (i = 0; i < sizeof bits; ++i) {
		// Clock Low - output data
		SMC_outw(mii_reg | bits[i], MII_REG);
		udelay(50);

		// Clock Hi - input data
		SMC_outw(mii_reg | bits[i] | MII_MCLK, MII_REG);
		udelay(50);
		bits[i] |= SMC_inw(MII_REG) & MII_MDI;
	}

	// Return to idle state
	// Set clock to low, data to low, and output tristated
	SMC_outw(mii_reg, MII_REG);
	udelay(50);

	// Restore original bank select
	SMC_SELECT_BANK(oldBank);

#if (SMC_DEBUG > 2 )
	printk("smc_write_phy_register(): phyaddr=%x,phyreg=%x,phydata=%x\n",
	       phyaddr, phyreg, phydata);
	smc_dump_mii_stream(bits, sizeof bits);
#endif
}

/*------------------------------------------------------------
 . Finds and reports the PHY address
 .-------------------------------------------------------------*/
static int smc_detect_phy(struct net_device *dev)
{
	struct smc_local *lp = netdev_priv(dev);
	unsigned int ioaddr = lp->base;
	word phy_id1;
	word phy_id2;
	int phyaddr;
	int found = 0;

	PRINTK3("%s:smc_detect_phy()\n", dev->name);

	lp->phy_type = 0;

	// Scan all 32 PHY addresses if necessary
	for (phyaddr = 0; phyaddr < 32; ++phyaddr) {
		// Read the PHY identifiers
		phy_id1 = smc_read_phy_register(ioaddr, phyaddr, PHY_ID1_REG);
		phy_id2 = smc_read_phy_register(ioaddr, phyaddr, PHY_ID2_REG);

		PRINTK3("%s: phy_id1=%x, phy_id2=%x\n", dev->name, phy_id1,
			phy_id2);

		// Make sure it is a valid identifier   
		if ((phy_id2 > 0x0000) && (phy_id2 < 0xffff) &&
		    (phy_id1 > 0x0000) && (phy_id1 < 0xffff)) {
			if ((phy_id1 != 0x8000) && (phy_id2 != 0x8000)) {
				// Save the PHY's address
				lp->phyaddr = phyaddr;
				found = 1;
				break;
			}
		}
	}

	if (!found) {
		PRINTK("%s: No PHY found\n", dev->name);
		return (0);
	}
	// Set the PHY type
	if ((phy_id1 == 0x0016) && ((phy_id2 & 0xFFF0) == 0xF840)) {
		lp->phytype = PHY_LAN83C183;
		PRINTK("%s: PHY=LAN83C183 (LAN91C111 Internal)\n", dev->name);
	}

	if ((phy_id1 == 0x0282) && ((phy_id2 & 0xFFF0) == 0x1C50)) {
		lp->phytype = PHY_LAN83C180;
		PRINTK("%s: PHY=LAN83C180\n", dev->name);
	}

	return (1);
}

/*------------------------------------------------------------
 . Waits the specified number of milliseconds - kernel friendly
 .-------------------------------------------------------------*/
static void smc_wait_ms(unsigned int ms)
{

	if (!in_interrupt()) {
		current->state = TASK_UNINTERRUPTIBLE;
		schedule_timeout(1 + ms * HZ / 1000);
	} else {
		current->state = TASK_INTERRUPTIBLE;
		schedule_timeout(1 + ms * HZ / 1000);
		current->state = TASK_RUNNING;
	}
}

#ifdef CONFIG_SYSCTL
/*------------------------------------------------------------
 . Sets the PHY to a configuration as determined by the user
 .-------------------------------------------------------------*/
static int smc_phy_fixed(struct net_device *dev)
{
	struct smc_local *lp = netdev_priv(dev);
	unsigned int ioaddr = lp->base;
	byte phyaddr = lp->phyaddr;
	word my_fixed_caps;
	word cfg1;

	PRINTK3("%s:smc_phy_fixed()\n", dev->name);

	// Enter Link Disable state
	cfg1 = smc_read_phy_register(ioaddr, phyaddr, PHY_CFG1_REG);
	cfg1 |= PHY_CFG1_LNKDIS;
	smc_write_phy_register(ioaddr, phyaddr, PHY_CFG1_REG, cfg1);

	// Set our fixed capabilities
	// Disable auto-negotiation
	my_fixed_caps = 0;

#ifdef CONFIG_SYSCTL
	if (lp->ctl_rfduplx)
		my_fixed_caps |= PHY_CNTL_DPLX;

	if (lp->ctl_rspeed == 100)
		my_fixed_caps |= PHY_CNTL_SPEED;
#endif

	// Write our capabilities to the phy control register
	smc_write_phy_register(ioaddr, phyaddr, PHY_CNTL_REG, my_fixed_caps);

	// Re-Configure the Receive/Phy Control register
	SMC_outw(lp->rpc_cur_mode, RPC_REG);

	// Success
	return (1);
}
#endif

/*------------------------------------------------------------
 . Configures the specified PHY using Autonegotiation. Calls
 . smc_phy_fixed() if the user has requested a certain config.
 .-------------------------------------------------------------*/
static void smc_phy_configure(struct net_device *dev)
{
	struct smc_local *lp = netdev_priv(dev);
	unsigned int ioaddr = lp->base;
	int timeout;
	byte phyaddr;
	word my_phy_caps;	// My PHY capabilities
	word my_ad_caps;	// My Advertised capabilities
	word status = 0;
	int failed = 0;

	PRINTK3("%s:smc_phy_configure()\n", dev->name);

	// Set the blocking flag
	lp->autoneg_active = 1;

	// Find the address and type of our phy
	if (!smc_detect_phy(dev)) {
		goto smc_phy_configure_exit;
	}
	// Get the detected phy address
	phyaddr = lp->phyaddr;

	// Reset the PHY, setting all other bits to zero
	smc_write_phy_register(ioaddr, phyaddr, PHY_CNTL_REG, PHY_CNTL_RST);

	// Wait for the reset to complete, or time out
	timeout = 6;		// Wait up to 3 seconds
	while (timeout--) {
		if (!(smc_read_phy_register(ioaddr, phyaddr, PHY_CNTL_REG)
		      & PHY_CNTL_RST)) {
			// reset complete
			break;
		}

		smc_wait_ms(500);	// wait 500 millisecs
		if (signal_pending(current))	// Exit anyway if signaled
		{
			PRINTK2("%s:PHY reset interrupted by signal\n",
				dev->name);
			timeout = 0;
			break;
		}
	}

	if (timeout < 1) {
		PRINTK2("%s:PHY reset timed out\n", dev->name);
		goto smc_phy_configure_exit;
	}
	// Read PHY Register 18, Status Output
	lp->lastPhy18 = smc_read_phy_register(ioaddr, phyaddr, PHY_INT_REG);

	// Enable PHY Interrupts (for register 18)
	// Interrupts listed here are disabled
	smc_write_phy_register(ioaddr, phyaddr, PHY_MASK_REG,
			       PHY_INT_LOSSSYNC | PHY_INT_CWRD | PHY_INT_SSD |
			       PHY_INT_ESD | PHY_INT_RPOL | PHY_INT_JAB |
			       PHY_INT_SPDDET | PHY_INT_DPLXDET);

	/* Configure the Receive/Phy Control register */
	SMC_SELECT_BANK(0);
	SMC_outw(lp->rpc_cur_mode, RPC_REG);

	// Copy our capabilities from PHY_STAT_REG to PHY_AD_REG
	my_phy_caps = smc_read_phy_register(ioaddr, phyaddr, PHY_STAT_REG);
	my_ad_caps = PHY_AD_CSMA;	// I am CSMA capable

	if (my_phy_caps & PHY_STAT_CAP_T4)
		my_ad_caps |= PHY_AD_T4;

	if (my_phy_caps & PHY_STAT_CAP_TXF)
		my_ad_caps |= PHY_AD_TX_FDX;

	if (my_phy_caps & PHY_STAT_CAP_TXH)
		my_ad_caps |= PHY_AD_TX_HDX;

	if (my_phy_caps & PHY_STAT_CAP_TF)
		my_ad_caps |= PHY_AD_10_FDX;

	if (my_phy_caps & PHY_STAT_CAP_TH)
		my_ad_caps |= PHY_AD_10_HDX;

#ifdef CONFIG_SYSCTL
	// Disable capabilities not selected by our user
	if (lp->ctl_rspeed != 100) {
		my_ad_caps &= ~(PHY_AD_T4 | PHY_AD_TX_FDX | PHY_AD_TX_HDX);
	}

	if (!lp->ctl_rfduplx) {
		my_ad_caps &= ~(PHY_AD_TX_FDX | PHY_AD_10_FDX);
	}
#endif

	// Update our Auto-Neg Advertisement Register
	smc_write_phy_register(ioaddr, phyaddr, PHY_AD_REG, my_ad_caps);

	PRINTK2("%s:phy caps=%x\n", dev->name, my_phy_caps);
	PRINTK2("%s:phy advertised caps=%x\n", dev->name, my_ad_caps);

#ifdef CONFIG_SYSCTL
	// If the user requested no auto neg, then go set his request
	if (!(lp->ctl_autoneg)) {
		smc_phy_fixed(dev);
		goto smc_phy_configure_exit;
	}
#endif

	// Restart auto-negotiation process in order to advertise my caps
	smc_write_phy_register(ioaddr, phyaddr, PHY_CNTL_REG,
			       PHY_CNTL_ANEG_EN | PHY_CNTL_ANEG_RST);

	// Wait for the auto-negotiation to complete.  This may take from
	// 2 to 3 seconds.
	// Wait for the reset to complete, or time out
	timeout = 20;		// Wait up to 10 seconds
	while (timeout--) {
		status = smc_read_phy_register(ioaddr, phyaddr, PHY_STAT_REG);
		if (status & PHY_STAT_ANEG_ACK) {
			// auto-negotiate complete
			break;
		}

		smc_wait_ms(500);	// wait 500 millisecs
		if (signal_pending(current))	// Exit anyway if signaled
		{
			printk(KERN_DEBUG
			       "%s:PHY auto-negotiate interrupted by signal\n",
			       dev->name);
			timeout = 0;
			break;
		}
		// Restart auto-negotiation if remote fault
		if (status & PHY_STAT_REM_FLT) {
			PRINTK2("%s:PHY remote fault detected\n", dev->name);

			// Restart auto-negotiation
			PRINTK2("%s:PHY restarting auto-negotiation\n",
				dev->name);
			smc_write_phy_register(ioaddr, phyaddr, PHY_CNTL_REG,
					       PHY_CNTL_ANEG_EN |
					       PHY_CNTL_ANEG_RST |
					       PHY_CNTL_SPEED | PHY_CNTL_DPLX);
		}
	}

	if (timeout < 1) {
		printk(KERN_DEBUG "%s:PHY auto-negotiate timed out\n",
		       dev->name);
		PRINTK2("%s:PHY auto-negotiate timed out\n", dev->name);
		failed = 1;
	}
	// Fail if we detected an auto-negotiate remote fault
	if (status & PHY_STAT_REM_FLT) {
		printk(KERN_DEBUG "%s:PHY remote fault detected\n", dev->name);
		PRINTK2("%s:PHY remote fault detected\n", dev->name);
		failed = 1;
	}
	// The smc_phy_interrupt() routine will be called to update lastPhy18

	// Set our sysctl parameters to match auto-negotiation results
	if (lp->lastPhy18 & PHY_INT_SPDDET) {
		PRINTK2("%s:PHY 100BaseT\n", dev->name);
		lp->rpc_cur_mode |= RPC_SPEED;
	} else {
		PRINTK2("%s:PHY 10BaseT\n", dev->name);
		lp->rpc_cur_mode &= ~RPC_SPEED;
	}

	if (lp->lastPhy18 & PHY_INT_DPLXDET) {
		PRINTK2("%s:PHY Full Duplex\n", dev->name);
		lp->rpc_cur_mode |= RPC_DPLX;
	} else {
		PRINTK2("%s:PHY Half Duplex\n", dev->name);
		lp->rpc_cur_mode &= ~RPC_DPLX;
	}

	// Re-Configure the Receive/Phy Control register
	SMC_outw(lp->rpc_cur_mode, RPC_REG);

 smc_phy_configure_exit:

	// Exit auto-negotiation
	lp->autoneg_active = 0;
}

/*************************************************************************
 . smc_phy_interrupt
 .
 . Purpose:  Handle interrupts relating to PHY register 18. This is
 .  called from the "hard" interrupt handler.
 .
 ************************************************************************/
static void smc_phy_interrupt(struct net_device *dev)
{
	struct smc_local *lp = netdev_priv(dev);
	unsigned int ioaddr = lp->base;
	byte phyaddr = lp->phyaddr;
	word phy18;

	PRINTK2("%s: smc_phy_interrupt\n", dev->name);

	while (1) {
		// Read PHY Register 18, Status Output
		phy18 = smc_read_phy_register(ioaddr, phyaddr, PHY_INT_REG);

		// Exit if not more changes
		if (phy18 == lp->lastPhy18)
			break;

#if (SMC_DEBUG > 1 )

		PRINTK2("%s:     phy18=0x%x\n", dev->name, phy18);
		PRINTK2("%s: lastPhy18=0x%x\n", dev->name, lp->lastPhy18);

		// Handle events
		if ((phy18 & PHY_INT_LNKFAIL) !=
		    (lp->lastPhy18 & PHY_INT_LNKFAIL)) {
			PRINTK2("%s: PHY Link Fail=%x\n", dev->name,
				phy18 & PHY_INT_LNKFAIL);
		}

		if ((phy18 & PHY_INT_LOSSSYNC) !=
		    (lp->lastPhy18 & PHY_INT_LOSSSYNC)) {
			PRINTK2("%s: PHY LOSS SYNC=%x\n", dev->name,
				phy18 & PHY_INT_LOSSSYNC);
		}

		if ((phy18 & PHY_INT_CWRD) != (lp->lastPhy18 & PHY_INT_CWRD)) {
			PRINTK2("%s: PHY INVALID 4B5B code=%x\n", dev->name,
				phy18 & PHY_INT_CWRD);
		}

		if ((phy18 & PHY_INT_SSD) != (lp->lastPhy18 & PHY_INT_SSD)) {
			PRINTK2("%s: PHY No Start Of Stream=%x\n", dev->name,
				phy18 & PHY_INT_SSD);
		}

		if ((phy18 & PHY_INT_ESD) != (lp->lastPhy18 & PHY_INT_ESD)) {
			PRINTK2("%s: PHY No End Of Stream=%x\n", dev->name,
				phy18 & PHY_INT_ESD);
		}

		if ((phy18 & PHY_INT_RPOL) != (lp->lastPhy18 & PHY_INT_RPOL)) {
			PRINTK2("%s: PHY Reverse Polarity Detected=%x\n",
				dev->name, phy18 & PHY_INT_RPOL);
		}

		if ((phy18 & PHY_INT_JAB) != (lp->lastPhy18 & PHY_INT_JAB)) {
			PRINTK2("%s: PHY Jabber Detected=%x\n", dev->name,
				phy18 & PHY_INT_JAB);
		}

		if ((phy18 & PHY_INT_SPDDET) !=
		    (lp->lastPhy18 & PHY_INT_SPDDET)) {
			PRINTK2("%s: PHY Speed Detect=%x\n", dev->name,
				phy18 & PHY_INT_SPDDET);
		}

		if ((phy18 & PHY_INT_DPLXDET) !=
		    (lp->lastPhy18 & PHY_INT_DPLXDET)) {
			PRINTK2("%s: PHY Duplex Detect=%x\n", dev->name,
				phy18 & PHY_INT_DPLXDET);
		}
#endif

		// Update the last phy 18 variable
		lp->lastPhy18 = phy18;

	}			// end while
}

/*------------------------------------------------------------
 . Cleanup when module is removed with rmmod
 .-------------------------------------------------------------*/
static int smc_drv_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct smc_local *lp = netdev_priv(ndev);
	struct resource *res;

	DBG(SMC_DEBUG_FUNC, "--> %s\n", __FUNCTION__);

	platform_set_drvdata(pdev, NULL);

	unregister_netdev(ndev);

	free_irq(ndev->irq, ndev);

#ifdef CONFIG_ARCH_PXA
	if (ndev->dma != (unsigned char)-1)
		pxa_free_dma(ndev->dma);
#endif
	iounmap(lp->base);

	res =
	    platform_get_resource_byname(pdev, IORESOURCE_MEM,
					 "lan91c113-regs");
	if (!res)
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, SMC_IO_EXTENT);

	free_netdev(ndev);
	return 0;
}

static int smc_drv_suspend(struct platform_device *dev, pm_message_t state)
{
	struct net_device *ndev = platform_get_drvdata(dev);
	struct smc_local *lp = netdev_priv(ndev);
	unsigned int ioaddr = lp->base;

	DBG(SMC_DEBUG_FUNC, "--> %s\n", __FUNCTION__);

	if (ndev) {
		if (netif_running(ndev)) {
			netif_device_detach(ndev);
			smc_shutdown(ndev);
#ifdef POWER_DOWN
			/* finally, shut the chip down */
			SMC_SELECT_BANK(1);
			SMC_outw(SMC_inw(CONFIG_REG) & ~CONFIG_EPH_POWER_EN,
				 CONFIG_REG);
#endif
		}
	}
	return 0;
}

static int smc_drv_resume(struct platform_device *dev)
{
	struct net_device *ndev = platform_get_drvdata(dev);

	DBG(SMC_DEBUG_FUNC, "--> %s\n", __FUNCTION__);
	if (ndev) {
		struct smc_local *lp = netdev_priv(ndev);
		if (netif_running(ndev)) {
			smc_reset(ndev);
			smc_enable(ndev);
			if (lp->phy_type != 0)
				smc_phy_configure(&lp->phy_configure);
			netif_device_attach(ndev);
		}
	}
	return 0;
}

static struct platform_driver smc_driver = {
	.probe = smc_drv_probe,
	.remove = smc_drv_remove,
	.suspend = smc_drv_suspend,
	.resume = smc_drv_resume,
	.driver = {
		   .name = CARDNAME,
		   .owner = THIS_MODULE,
		   },
};

static int __init smc_init(void)
{
	printk("smc_init\n");
	return platform_driver_register(&smc_driver);
}

static void __exit smc_cleanup(void)
{
	printk("smc_cleanup\n");
	platform_driver_unregister(&smc_driver);
}

module_init(smc_init);
module_exit(smc_cleanup);
