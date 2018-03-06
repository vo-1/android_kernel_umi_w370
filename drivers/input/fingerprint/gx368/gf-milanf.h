#ifndef __GF_MILANF_H
#define __GF_MILANF_H

#include <linux/types.h>
#include "gf-common.h"
/****************GF Macro interface*******************/
#define GF_DEBUG             1
#define CONFIG_HAS_EARLYSUSPEND  0
#define GF_FASYNC            1//If support fasync mechanism.
#define PROCESSOR_64_BIT         0


#define GF_DRIVER_VERSION    "V1.2<20151207>"

/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */

/*spi device name*/
#define SPI_DEV_NAME        "spidev"
/*device name after register in charater*/
#define DEV_NAME            "goodix_fp"
#define	CHRD_DRIVER_NAME    "goodix_fp_spi"
#define	CLASS_NAME          "goodix_fp"
#define GF_INPUT_NAME       "gf-input"
#define SPIDEV_MAJOR        156	/* assigned */
#define N_SPI_MINORS        32	/* ... up to 256 */

struct gf_configs {
	unsigned short addr;
	unsigned short value;
};

#define CONFIG_FDT_DOWN  1
#define CONFIG_FDT_UP    2
#define CONFIG_FF        3
#define CONFIG_NAV       4
#define CONFIG_IMG       5
#define CONFIG_NAV_IMG   6

#define GF_BUF_STA_MASK		(0x1<<7)
#define	GF_BUF_STA_READY	(0x1<<7)
#define	GF_BUF_STA_BUSY		(0x0<<7)

#define	GF_IMAGE_MASK		(1<<6)
#define	GF_IMAGE_ENABLE		(1<<6)
#define	GF_IMAGE_DISABLE	(0x0)

#define	GF_KEY_MASK			(0x1<<5)
#define	GF_KEY_ENABLE		(0x1<<5)
#define	GF_KEY_DISABLE		(0x0)

#define	GF_KEY_STA			(0x1<<4)

/**************************debug******************************/
#define DEFAULT_DEBUG   (0)
#define SUSPEND_DEBUG   (1)
#define SPI_DEBUG       (2)
#define TIME_DEBUG      (3)
#define FLOW_DEBUG      (4)
static int g_debug_level = FLOW_DEBUG;
//static int gf_debug_level(int level)
//{
//	g_debug_level=level;
//	printk("=======sonia g_debug_level=%d",g_debug_level);
//	return 0;
//}
#if GF_DEBUG
#define gf_debug(level, fmt, args...) do{ \
    if(g_debug_level >= level) {\
	printk("gf3208 "fmt"\n", ##args); \
    } \
}while(0)
#define gf_error(fmt,arg...)          printk("<<gf3208_error>> "fmt"\n",##arg)  

#define FUNC_ENTRY()  gf_debug(FLOW_DEBUG, "gf3208:%s, entry\n", __func__)
#define FUNC_EXIT()  gf_debug(FLOW_DEBUG,"gf3208:%s, exit\n", __func__)
#endif


/*************************************************************/


/**********************IO Magic**********************/
#define  GF_IOC_MAGIC    'g'  //define magic number
struct gf_ioc_transfer {
	u8	cmd;
	u8 reserve;
	u16	addr;
	u32 len;
#if PROCESSOR_64_BIT
	u64  buf;
#else
	u8 *buf;
#endif
};
struct gf_key {
	unsigned int key;
	int value;
};

struct gf_key_map
{
    char *name;
    unsigned short val;
};

#define  GF_IOC_RESET	     _IO(GF_IOC_MAGIC, 0)
#define  GF_IOC_RW           _IOWR(GF_IOC_MAGIC, 1, struct gf_ioc_transfer)
#define  GF_IOC_CMD          _IOW(GF_IOC_MAGIC,2,unsigned char)
#define  GF_IOC_CONFIG       _IOW(GF_IOC_MAGIC,3,unsigned char)
#define  GF_IOC_ENABLE_IRQ   _IO(GF_IOC_MAGIC,4)
#define  GF_IOC_DISABLE_IRQ  _IO(GF_IOC_MAGIC,5)
#define  GF_IOC_SENDKEY      _IOW(GF_IOC_MAGIC,6,struct gf_key)

#define  GF_IOC_MAXNR        7
#endif
