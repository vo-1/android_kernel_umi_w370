/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/spi/spi.h>
#include <linux/types.h>
#include <mt_spi.h>
#include <asm/atomic.h>
#include <linux/irq.h>
#include <linux/poll.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
//#include <mach/eint.h>
//#include <cust_eint.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <compat.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/seq_file.h>
#include <linux/input.h>

#include <linux/rtpm_prio.h>

#include <linux/jiffies.h>
#include <linux/wakelock.h>
#include <asm/uaccess.h>
#include <linux/sched.h>
#include <linux/input.h>
#include <mach/gpio_const.h>
//#include "upmu_sw.h"
//#include "upmu_common.h"
//#include <mach/gpio.h>
#include <mt_gpio.h>
//#include <plat/gpio-cfg.h>
//#include "cust_gpio_usage.h" //daizhiling add 
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>


#include <linux/cdev.h>
#include "slspi.h"
#include "sl_proc.h"
#include "spi_debug.h"
//Singh add for common resource for fingerprint

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
 //Singh add for fingerprint compatible
#define COMPATIBLE_SUPPORT
//#define SPIDEV_MAJOR			153	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */
#define SL_MAX_FRAME_NUM 		2
#define VERBOSE  			0
#define SILEAD_PROC

//#define SILEAD_DEBUF

static DECLARE_BITMAP(minors, N_SPI_MINORS);
/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *	is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY)

//static char tmp[1024];
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static struct spidev_data	*fp_spidev = NULL;
static unsigned int spidev_major = 0;
struct cdev spicdev;

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");
static const char input_name[]= "sl_fp_key";

#define SL_READ  0x00 //read flags
#define SL_WRITE 0xFF //write flags

//extern void mt_eint_unmask(unsigned int line);
//extern void mt_eint_mask(unsigned int line);
static void spidev_work(struct work_struct *work);
static int spidev_reset_hw(struct spidev_data *spidev);
static int suspend_flag = 0;
//static void silead_early_suspend (struct early_suspend *h);
//static void silead_late_resume (struct early_suspend *h);
static inline void sl_fp_read_init(struct spidev_data *spidev, u8 addr);

static inline ssize_t sl_fp_write(struct spidev_data *spidev, uint8_t reg, uint32_t w_data);
static inline ssize_t sl_fp_read(struct spidev_data *spidev, u8 addr, uint32_t *pdata, size_t len);
//static int check_id(struct spidev_data	*spidev);
//static int enter_sleepmode(struct spidev_data *spidev);// daizhiling remove for not be unsed
static DECLARE_WAIT_QUEUE_HEAD(waiter);
//static struct task_struct *thread = NULL; //daizhiling remove
//static int fp_flag = 0;//daizhiling remove
//static unsigned char * pgframe = NULL;
//static int getoneframe(struct spidev_data *spidev);
//static int check_id(struct spidev_data	*spidev);
static struct platform_device *fingerprint_device = NULL;
static unsigned int fingerprint_irq = 0;
static int fingerprint_irq_registration(void);
struct of_device_id fingerprint_of_match[] = {
	{ .compatible = "mediatek,dtv-eint",},
//	{ .compatible = "mediatek,hct_finger",},
	{},
};
MODULE_DEVICE_TABLE(of,fingerprint_of_match);

typedef u32 compat_uptr_t;
static inline void __user *compat_ptr(compat_uptr_t uptr)
{
  return (void __user *)(unsigned long)uptr;
}

static void spidev_work(struct work_struct *work);
struct mt_chip_conf chip_config = {
		.setuptime =15,//6, //10,//15,//10 ,//3,
		.holdtime =15, //6,//10,//15,//10,//3,
		.high_time =60,//4,//6,//8,//12, //25,//8,      //10--6m   15--4m   20--3m  30--2m  [ 60--1m 120--0.5m  300--0.2m]
		.low_time = 60,//4,//6,//8,//12,//25,//8,
		.cs_idletime = 60,//30,// 60,//100,//12,
		.ulthgh_thrsh = 0,

		.rx_mlsb = SPI_MSB, 
		.tx_mlsb = SPI_MSB,		 
		.tx_endian = 0,
		.rx_endian = 0,

		.cpol = SPI_CPOL_0, 
		.cpha = SPI_CPHA_0,//SPI_CPHA_0 ok!!
		
		.com_mod = DMA_TRANSFER, //FIFO_TRANSFER, origin
		.pause = 0,
		.finish_intr = 1,
};
//set spi mode 

/*daizhiling remove for build error begin,will be used for get picture*/

///////////////////////////////////////////////////
static inline void spidev_schedule_work(struct spidev_data *spidev)
{
    if (work_pending(&spidev->work)) {
        return;
    }
    if (spidev->wqueue) {
        queue_work(spidev->wqueue, &spidev->work);
    } else {
        schedule_work(&spidev->work);
    }
}

static int  put_buffer(struct spidev_data *spidev)
{
    spidev->k_mmap_buf += sizeof(struct sl_frame);
    if (spidev->k_mmap_buf - spidev->mmap_buf == sizeof(struct sl_frame)*spidev->max_frame_num) {
        spidev->k_mmap_buf = spidev->mmap_buf;
    }

    atomic_inc(&spidev->frame_num);
    if (atomic_read(&spidev->frame_num) == spidev->max_frame_num-1) {//spidev->k_mmap_buf == spidev->u_mmap_buf) {
        //buffer is full
        dev_dbg(&spidev->spi->dev, "Receive buffer is full\n");
        return 0;
    }
    return 0;
}
static int get_buffer(struct spidev_data *spidev)
{
    int offset = -EAGAIN;
    if (atomic_read(&spidev->frame_num) == 0) {//buffer is empty
        dev_dbg(&spidev->spi->dev, "Receive buffer is empyt\n");
        spidev_schedule_work(spidev);
        return offset;
    }

    atomic_dec(&spidev->frame_num);
    offset = spidev->u_mmap_buf- spidev->mmap_buf;
    spidev->u_mmap_buf += sizeof(struct sl_frame);
    if ((spidev->u_mmap_buf - spidev->mmap_buf) == sizeof(struct sl_frame)*spidev->max_frame_num) {
        spidev->u_mmap_buf = spidev->mmap_buf;
    }
    spidev_schedule_work(spidev);
    return offset;
}


static int gsl_fp_rdinit(struct spidev_data *spidev, unsigned char reg)
{
    uint8_t tx[] = {
        reg, SL_READ,
    };
    unsigned rx[ARRAY_SIZE(tx)] = {0};
    struct spi_message	m;
    struct spi_transfer	t = {
        .rx_buf		= rx,
        .tx_buf		= tx,
        .len		= ARRAY_SIZE(tx),
        .bits_per_word = SPI_BITS,
        .delay_usecs = SPI_DELAY,
        .speed_hz = SPI_SPEED,
    };

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    return spidev_sync(spidev, &m);
}

void init_frame(struct spidev_data *spidev)
{
    unsigned int ret=0;
    unsigned long timeout;
// init page point
    spidev_write_reg(spidev, 0x00, 0xBF);
// start scanning
    spidev_write_reg(spidev, (0xFF080024>>7), 0xF0);
    spidev_write_reg(spidev, 0x2007FFFF, (0xFF080024%0x80));
    /* Wait  2 seconds for scanning done */
    timeout = jiffies + 2*HZ;
    while (time_before(jiffies, timeout)) {
        ret = spidev_read_reg(spidev, 0xBF);
        dev_dbg(&spidev->spi->dev, "0xBF=0x%02x\n", ret);
        if (ret != 0) {
            break;
        }
        udelay(100);
    }
    dev_dbg(&spidev->spi->dev, "last ret 0xBF=0x%02x\n", ret);
    spidev_write_reg(spidev, 0x00, 0xF0);
    gsl_fp_rdinit(spidev, 0);
}

struct spi_transfer	t_silead[SL_ONE_FRAME_PAGES];

static void spidev_work(struct work_struct *work)
{
    struct spidev_data *spidev = container_of(work, struct spidev_data, work);
    struct spi_message	m;
    //struct spi_transfer	t[SL_ONE_FRAME_PAGES];
    int ret = 0, i;

	struct sched_param param = {.sched_priority = 1};
	sched_setscheduler(current, SCHED_RR, &param);

    if (atomic_read(&spidev->is_cal_mode)){
        return ;
    }

    t_silead[0].rx_buf	= spidev->k_mmap_buf;
    t_silead[0].tx_buf	= spidev->tx_mmap_buf;
    t_silead[0].len	= SL_HEAD_SIZE +SL_PAGE_SIZE;
    t_silead[0].bits_per_word = SPI_BITS;
    t_silead[0].delay_usecs = SPI_DELAY;
    t_silead[0].speed_hz = SPI_SPEED;
    spi_message_init(&m);
    spi_message_add_tail(&t_silead[0], &m);
    for (i=1; i < SL_ONE_FRAME_PAGES; ++i)
    {
        t_silead[i].rx_buf	= spidev->k_mmap_buf+i*SL_PAGE_SIZE + SL_HEAD_SIZE;
        t_silead[i].tx_buf	= spidev->tx_mmap_buf+i*SL_PAGE_SIZE + SL_HEAD_SIZE;
        t_silead[i].len    = SL_PAGE_SIZE;
        t_silead[i].bits_per_word = SPI_BITS;
        t_silead[i].delay_usecs = SPI_DELAY;
        t_silead[i].speed_hz = SPI_SPEED;
        spi_message_add_tail(&t_silead[i], &m);
    }

    if (!atomic_read(&spidev->frame_num)) {
        init_frame(spidev);
        ret = spidev_sync(spidev, &m);
        if (ret >0) {
            put_buffer(spidev);
        } else {
            dev_notice(&spidev->spi->dev, "sync fialed %d\n", ret);
        }
    } else {
        dev_dbg(&spidev->spi->dev, "Receive buffer is full=%d\n", atomic_read(&spidev->frame_num));
    }

    //if interrupt config, enable it
    //
}

static inline ssize_t sl_fp_write(struct spidev_data *spidev, uint8_t reg, uint32_t w_data);

/*daizhiling remove for build error,will be used for eint*/

static void spidev_irq_work(struct work_struct *work)
{
	struct spidev_data*		spidev = container_of(work,struct spidev_data,irq_work);
    char*					env_ext[2] = {"SILEAD_FP_EVENT=IRQ", NULL};
	printk("irq bottom half spidev_irq_work enter \n");
	kobject_uevent_env(&spidev->spi->dev.kobj, KOBJ_CHANGE, env_ext); 	
	return;
}

/*daizhiling remove for not be used begin*/
static irqreturn_t spidev_irq_routing(int irq, void* dev)
{
	struct spidev_data *spidev = fp_spidev;	
	printk("irq top half spidev_irq_routing enter \n");

	if (!spidev)
		return IRQ_HANDLED;

	disable_irq_nosync(spidev->irq);
	//int_top_time = sched_clock();
	schedule_work(&spidev->irq_work);
	return IRQ_HANDLED;

#if 0
	mt_eint_mask(spidev->irq); 
	//	disable_irq_nosync(spidev->irq);
	if(spidev->wqueue)
	{
		queue_work(spidev->wqueue,&spidev->irq_work);
	}
	else
	{
		schedule_work(&spidev->irq_work);
	}

	return IRQ_HANDLED;
#endif
}
/*daizhiling remove for not be used end*/

/*-------------------------------------------------------------------------*/

#ifdef LSB_TO_MSB
static inline unsigned char reversalBits(unsigned char reg)
{
    int i;
    unsigned char ret = 0;
    unsigned char tmp = 0;
    unsigned char tmp1 = 0;
    unsigned char mask = 1;
    for (i = 7; i >= 0; i--) {
        tmp = reg >> (7 - i);
        tmp1 = (tmp << i) & (mask << i);
        ret |= tmp1;
    }
    return ret;
}

static void Modifybuf(u8 *buf, size_t len)
{
    u8 tmp;
    int i;

    for (i = 0; i < len; i++) {
        tmp = reversalBits(buf[i]);
        //pr_err("%s:%d buf:%d\n", __func__, __LINE__, tmp);
        buf[i] = tmp;
    }
}
#endif

/*
 * We can't use the standard synchronous wrappers for file I/O; we
 * need to protect against async removal of the underlying spi_device.
 */
static void spidev_complete(void *arg)
{
    complete(arg);
}

ssize_t spidev_sync(struct spidev_data *spidev, struct spi_message *message)
{
    DECLARE_COMPLETION_ONSTACK(done);
    int status;

#ifdef LSB_TO_MSB
    struct list_head *p;
    struct spi_transfer *t;

    list_for_each(p, &message->transfers) {
        //pr_err("%s:%d\n", __func__, __LINE__);
        t = list_entry(p, struct spi_transfer, transfer_list);
        if (t->tx_buf) {
            Modifybuf((u8*)t->tx_buf, t->len);
        }
    }
#endif

    message->complete = spidev_complete;
    message->context = &done;

    if (spidev->spi == NULL) {
        status = -ESHUTDOWN;
    } else {
        spin_lock_irq(&spidev->spi_lock);
        status = spi_async(spidev->spi, message);
        spin_unlock_irq(&spidev->spi_lock);
    }

    if (status == 0) {
        //wait_for_completion(&done);------> deadlock
        wait_for_completion_timeout(&done, msecs_to_jiffies(3000)); 
        status = message->status;
        if (status == 0)
            status = message->actual_length;

#ifdef LSB_TO_MSB
        list_for_each(p, &message->transfers) {
            //pr_err("%s:%d\n", __func__, __LINE__);
            t = list_entry(p, struct spi_transfer, transfer_list);
            if (t->rx_buf) {
                Modifybuf((u8*)t->rx_buf, t->len);
            }
        }
#endif
    }

    return status;
}

static inline ssize_t
spidev_sync_write(struct spidev_data *spidev, size_t len)
{
    struct spi_transfer	t = {
        .tx_buf		= spidev->buffer,
        .len		= len,
    };
    struct spi_message	m;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    return spidev_sync(spidev, &m);
}

static inline ssize_t
spidev_sync_read(struct spidev_data *spidev, size_t len)
{
    struct spi_transfer	t = {
        .rx_buf		= spidev->buffer,
        .len		= len,
    };
    struct spi_message	m;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    return spidev_sync(spidev, &m);
}

static inline ssize_t
__spidev_sync_read(struct spidev_data *spidev, size_t offset, size_t len)
{
    struct spi_transfer	t = {
        .rx_buf		= spidev->mmap_buf + offset,
        .tx_buf		= spidev->mmap_buf + offset,
        .len		= len,
    };
    struct spi_message	m;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    return spidev_sync(spidev, &m);
}


static inline ssize_t
__spidev_async_read(struct spidev_data *spidev, size_t offset, size_t len)
{

    struct spi_transfer	t = {
        .rx_buf		= spidev->mmap_buf + offset,
        .tx_buf		= spidev->mmap_buf + offset,
        .len		= len,
    };
    struct spi_message	m;

    //unsigned		is_dma_mapped:1;
    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    return spidev_sync(spidev, &m);
}

static inline void sl_fp_read_init(struct spidev_data *spidev, u8 addr)
{
    struct spi_transfer		t;
    struct spi_message	m;
    u8 tx[6] = {0};
    u8 rx[6] = {0};

    tx[0] = addr;

    t.tx_buf = tx;
    t.rx_buf = rx;
    t.len = 6;
    t.bits_per_word = SPI_BITS;
    t.delay_usecs = SPI_DELAY;
    t.speed_hz = SPI_SPEED;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    spidev_sync(spidev, &m);
}

static inline ssize_t sl_fp_read(struct spidev_data *spidev, u8 addr, uint32_t *pdata, size_t len)
{
    ssize_t	status;
    struct spi_transfer		t;
    struct spi_message	m;
    u8 tx[SPI_BUF_SIZE + 3] = {0};
    u8 rx[SPI_BUF_SIZE + 3] = {0};
    u8 offset = 2;
    int i;

    if(len > SPI_BUF_SIZE) {
#ifdef thomasDEBUG
        printk("%s  too long len = %d!\n", __func__, (int)len);
#endif
        return -1;
    }

    if(addr < 0x80) {
        offset = 3;
        sl_fp_read_init(spidev, addr);
    }

    tx[0] = addr;

    t.tx_buf = tx;
    t.rx_buf = rx;
    t.len = len + offset;
    t.bits_per_word = SPI_BITS;
    t.delay_usecs = SPI_DELAY;
    t.speed_hz = SPI_SPEED;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    status = spidev_sync(spidev, &m);
    if(status > 0) {
        for(i = 0; i < len; i ++) {
            *(pdata + i) = rx[i + offset];
        }
    } else {
#ifdef thomasDEBUG
        printk("%s  error status = %d!\n", __func__, (int)status);
#endif
    }
    return status;
}

static inline ssize_t
sl_fp_write(struct spidev_data *spidev, uint8_t reg, uint32_t w_data)
{
    struct spi_transfer		t;
    struct spi_message	m;
    uint8_t tx[] = {
        reg, 0xFF,
        (w_data >>0) &0xFF,
        (w_data >>8) &0xFF,
        (w_data >>16) &0xFF,
        (w_data >>24) &0xFF,
    };
    u8 rx[sizeof(tx)] = {0};
    t.tx_buf = tx;
    t.rx_buf = rx;
    t.len = sizeof(tx);
    t.bits_per_word = SPI_BITS;
    t.delay_usecs = SPI_DELAY;
    t.speed_hz = SPI_SPEED;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    return spidev_sync(spidev, &m);
}

static union {
    unsigned char temp_char[4];
    unsigned int get_reg_data;
} temp_data;

unsigned int spidev_read_reg(struct spidev_data *spidev, unsigned char reg)
{
    struct spi_message	m;
    unsigned char rx[6];
    unsigned char tx[] = {
        reg, SL_READ, 0x00, 0x00, 0x00, 0x00
    };
    struct spi_transfer	t = {
        .rx_buf		= rx,
        .tx_buf		= tx,
        .len		= ARRAY_SIZE(tx),
        .bits_per_word = SPI_BITS,
        .delay_usecs = SPI_DELAY,
        .speed_hz = SPI_SPEED,
    };
    if (!(reg>0x80 && reg <0x100)) {
        gsl_fp_rdinit(spidev, reg);
    }
    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    spidev_sync(spidev, &m);
    memcpy(temp_data.temp_char, (rx+2), 4);
    return temp_data.get_reg_data;
}

int spidev_write_reg(struct spidev_data *spidev, unsigned int data, unsigned char reg)
{
    struct spi_message	m;
    uint8_t rx[6] = {0};
    uint8_t tx[] = {
        reg, SL_WRITE,
        (data >>0) &0xFF,
        (data >>8) &0xFF,
        (data >>16) &0xFF,
        (data >>24) &0xFF,
    };
    struct spi_transfer	t = {
        .rx_buf		= rx,
        .tx_buf		= tx,
        .len		= ARRAY_SIZE(tx),
        .bits_per_word = SPI_BITS,
        .delay_usecs = SPI_DELAY,
        .speed_hz = SPI_SPEED,
    };

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    return spidev_sync(spidev, &m);
}




/*-------------------------------------------------------------------------*/

/* Read-only message with current device setup */
static ssize_t
spidev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    struct spidev_data	*spidev;
    ssize_t			status = 0;

    /*chipselect only toggles at start or end of operation */
    if (count > bufsiz)
        return -EMSGSIZE;

    spidev = filp->private_data;

    mutex_lock(&spidev->buf_lock);
    status = spidev_sync_read(spidev, count);
    if (status > 0) {
        unsigned long	missing;

        missing = copy_to_user(buf, spidev->buffer, status);
        if (missing == status)
            status = -EFAULT;
        else
            status = status - missing;
    }
    mutex_unlock(&spidev->buf_lock);

    return status;
}

/* Write-only message with current device setup */
static ssize_t
spidev_write(struct file *filp, const char __user *buf,
             size_t count, loff_t *f_pos)
{
    struct spidev_data	*spidev;
    ssize_t			status = 0;
    unsigned long		missing;
  
    /* chipselect only toggles at start or end of operation */
    if (count > bufsiz)
        return -EMSGSIZE;

    spidev = filp->private_data;

    mutex_lock(&spidev->buf_lock);
    missing = copy_from_user(spidev->buffer, buf, count);
    if (missing == 0) {
        status = spidev_sync_write(spidev, count);
    } else
        status = -EFAULT;
    mutex_unlock(&spidev->buf_lock);

    return status;
}

static int spidev_message(struct spidev_data *spidev,
                          struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
    struct spi_message	msg;
    struct spi_transfer	*k_xfers;
    struct spi_transfer	*k_tmp;
    struct spi_ioc_transfer *u_tmp;
    unsigned		n, total;
    u8			*buf;
    int			status = -EFAULT;
   
    spi_message_init(&msg);
    k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
    if (k_xfers == NULL)
        return -ENOMEM;

    /* Construct spi_message, copying any tx data to bounce buffer.
     * We walk the array of user-provided transfers, using each one
     * to initialize a kernel version of the same transfer.
     */
    buf = spidev->buffer;
    total = 0;
    for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
            n;
            n--, k_tmp++, u_tmp++) {
        k_tmp->len = u_tmp->len;

        total += k_tmp->len;
        if (total > bufsiz) {
            status = -EMSGSIZE;
            goto done;
        }

        if (u_tmp->rx_buf) {
            k_tmp->rx_buf = buf;
            if (!access_ok(VERIFY_WRITE, (u8 __user *)
                           (uintptr_t) u_tmp->rx_buf,
                           u_tmp->len))
                goto done;
        }
        if (u_tmp->tx_buf) {
            k_tmp->tx_buf = buf;
            if (copy_from_user(buf, (const u8 __user *)
                               (uintptr_t) u_tmp->tx_buf,
                               u_tmp->len))
                goto done;
        }
        #if 0
		if (u_tmp->tx_buf)
		{
			int i = 0;
			if(u_tmp->len > 2)
			{
				printk("[FINGERP][line%d]:%s ", __LINE__, __FUNCTION__);
				printk("reg:%02x flag:%02x ", buf[i], buf[i+1]);
				for(i = u_tmp->len - 1; i > 1; i--)
				{
					printk("%02x", buf[i]);
				}
				printk("\n");
			}
			else
			{
				printk("%s:u_tmp->len < 2", __func__);
			}
        }
        #endif
        buf += k_tmp->len;

        k_tmp->cs_change = !!u_tmp->cs_change;
        k_tmp->bits_per_word = u_tmp->bits_per_word;
        k_tmp->delay_usecs = u_tmp->delay_usecs;
        k_tmp->speed_hz = u_tmp->speed_hz;
		
//#ifdef thomasDEBUG
#if 0
       // dev_dbg(&spidev->spi->dev,
	printk(
                "  xfer len %zd %s%s%s%dbits %u usec %uHz\n",
                u_tmp->len,
                u_tmp->rx_buf ? "rx " : "",
                u_tmp->tx_buf ? "tx " : "",
                u_tmp->cs_change ? "cs " : "",
                u_tmp->bits_per_word ? : spidev->spi->bits_per_word,
                u_tmp->delay_usecs,
                u_tmp->speed_hz ? : spidev->spi->max_speed_hz);
#endif
        spi_message_add_tail(k_tmp, &msg);
    }

    //retval =  __spidev_sync_read(spidev, 0, tmp);
    status = spidev_sync(spidev, &msg);
//    dev_err(&spidev->spi->dev, "spidev sync status %d\n", status);
    if (status < 0) {
        dev_err(&spidev->spi->dev, "spidev sync failed %d\n", status);
        goto done;
    }

    /* copy any rx data out of bounce buffer */
    buf = spidev->buffer;
    for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++) {
        if (u_tmp->rx_buf) {
            if (__copy_to_user((u8 __user *)
                               (uintptr_t) u_tmp->rx_buf, buf,
                               u_tmp->len)) {
                status = -EFAULT;
                goto done;
            }
        }
        buf += u_tmp->len;
    }
    status = total;
done:
    kfree(k_xfers);
    return status;
}

static int spidev_mmap(struct file* filep, struct vm_area_struct *vma)
{
    struct spidev_data	*spidev = filep->private_data;

    vma->vm_flags |= VM_RESERVED;
    vma->vm_flags |= VM_LOCKED;
    if (NULL == spidev->mmap_buf) {
        dev_err(&spidev->spi->dev,"frame buffer is not alloc\n");
        return -ENOMEM;
    }
    return remap_pfn_range( vma, vma->vm_start,
                            virt_to_phys((void*)((unsigned long)spidev->mmap_buf))>>PAGE_SHIFT,
                            vma->vm_end - vma->vm_start, PAGE_SHARED);
}
#if 0
struct pinctrl *hct_finger_pinctrl;
struct pinctrl_state *hct_finger_power_on, *hct_finger_power_off;//, *hct_finger_int_as_int;
static int spidev_power_on(struct platform_device* pdev,int power_on)
{
#if 0
	struct regulator *reg = NULL;
	int ret = 0;
	printk("nasri...power_on = %d %s %d",power_on,__func__,__LINE__);
	if(power_on)
	{
		//2.8V
		reg = regulator_get(&(fingerprint_device->dev),"vfingerprint");
		ret = regulator_set_voltage(reg, 2800000, 2800000);
		if(ret)
			printk("nasri....regulator_set_voltage failed!\n");
		ret = regulator_enable(reg);	/*enable regulator*/
		if (ret)
			printk("nasri....regulator enable\n");
		msleep(100);

	}
	else
	{
		if(reg != NULL)
		{
			regulator_disable(reg);
		}
		else
			printk("nasri...reg is NULL !!! something error???\n");
	}
	return 0;
#endif
    struct device_node *node;
    int ret;
    node = of_find_compatible_node(NULL, NULL, "mediatek,hct_finger");
    printk("node.name %s full name %s",node->name,node->full_name);


        hct_finger_pinctrl = devm_pinctrl_get(&pdev->dev);
        if (IS_ERR(hct_finger_pinctrl)) {
            ret = PTR_ERR(hct_finger_pinctrl);
            dev_err(&pdev->dev, "hct_finger cannot find pinctrl\n");
                return ret;
        }   

    printk("[%s] hct_finger_pinctrl+++++++++++++++++\n",pdev->name);

    hct_finger_power_on = pinctrl_lookup_state(hct_finger_pinctrl, "finger_power_en1");
    if (IS_ERR(hct_finger_power_on)) {
        ret = PTR_ERR(hct_finger_power_on);
        dev_err(&pdev->dev, " Cannot find hct_finger pinctrl hct_finger_power_on!\n");
        return ret;
    }   
    hct_finger_power_off = pinctrl_lookup_state(hct_finger_pinctrl, "finger_power_en0");
    if (IS_ERR(hct_finger_power_off)) {
        ret = PTR_ERR(hct_finger_power_off);
        dev_err(&pdev->dev, " Cannot find hct_finger pinctrl hct_finger_power_off!\n");
        return ret;
    } 

    	if(power_on)
	{
		pinctrl_select_state(hct_finger_pinctrl, hct_finger_power_on);
	}
	else
	{
		pinctrl_select_state(hct_finger_pinctrl, hct_finger_power_off);
	}
    return 0;
}
#endif


#define GPIO_SPI_33_V GPIO_MODE_00
#define GPIO_SPI_18_V GPIO_MODE_00

#define GPIO_SPI_33_V_PIN (GPIO11 | 0x80000000)
#define GPIO_SPI_18_V_PIN (GPIO12 | 0x80000000)
static int spidev_power_on(struct spidev_data *spidev,int power_on)
{
	//3.3v
	mt_set_gpio_mode(GPIO_SPI_33_V_PIN,GPIO_SPI_33_V);
	mt_set_gpio_dir(GPIO_SPI_33_V_PIN,GPIO_DIR_OUT);

	//2.8v
	mt_set_gpio_mode(GPIO_SPI_18_V_PIN,GPIO_SPI_18_V);
	mt_set_gpio_dir(GPIO_SPI_18_V_PIN,GPIO_DIR_OUT);

	if(power_on)
	{
		mt_set_gpio_out(GPIO_SPI_33_V_PIN,GPIO_OUT_ONE);
		mt_set_gpio_out(GPIO_SPI_18_V_PIN,GPIO_OUT_ONE);	
	}
	else
	{
		mt_set_gpio_out(GPIO_SPI_33_V_PIN,GPIO_OUT_ZERO);
		mt_set_gpio_out(GPIO_SPI_18_V_PIN,GPIO_OUT_ZERO);
	}
	return 0;
}


#define GPIO_FINGERPRINT_RST_PIN (GPIO17 | 0x80000000)
#define GPIO_FINGERPRINT_RST_PIN_M_GPIO GPIO_MODE_00
static int spidev_reset_hw(struct spidev_data *spidev)
{
   // hw reset  
	mt_set_gpio_mode(GPIO_FINGERPRINT_RST_PIN,GPIO_FINGERPRINT_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_FINGERPRINT_RST_PIN,GPIO_DIR_OUT);

	mt_set_gpio_out(GPIO_FINGERPRINT_RST_PIN,GPIO_OUT_ZERO);
    mdelay(5);
	mt_set_gpio_out(GPIO_FINGERPRINT_RST_PIN,GPIO_OUT_ONE);


   return 0;
}
static int spidev_shutdown_hw(struct spidev_data *spidev)
{
	mt_set_gpio_mode(GPIO_FINGERPRINT_RST_PIN,GPIO_FINGERPRINT_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_FINGERPRINT_RST_PIN,GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_FINGERPRINT_RST_PIN,GPIO_OUT_ZERO);
	return 0;
}

static long spidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int		err = 0;
    int			retval = 0;
    struct spidev_data	*spidev;
    struct spi_device	*spi;
    u32			tmp;
    unsigned		n_ioc;
    struct spi_ioc_transfer	*ioc;

    //unsigned int val;
    //char tmp_fc[9];
 
    /* Check type and command number */
    if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
        return -ENOTTY;

    /* Check access direction once here; don't repeat below.
     * IOC_DIR is from the user perspective, while access_ok is
     * from the kernel perspective; so they look reversed.
     */
    if (_IOC_DIR(cmd) & _IOC_READ)
        err = !access_ok(VERIFY_WRITE,
                         (void __user *)arg, _IOC_SIZE(cmd));
    if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
        err = !access_ok(VERIFY_READ,
                         (void __user *)arg, _IOC_SIZE(cmd));
    if (err)
        return -EFAULT;

    /* guard against device removal before, or while,
     * we issue this ioctl.
     */
    spidev = filp->private_data;
    spin_lock_irq(&spidev->spi_lock);
    spi = spi_dev_get(spidev->spi);
    spin_unlock_irq(&spidev->spi_lock);

    if (atomic_read(&spidev->is_cal_mode)){
        dev_dbg(&spidev->spi->dev, "Current stat is cal mode\n");
        return -EBUSY;
    }
     
    if (atomic_read(&spidev->is_suspend)){
        dev_dbg(&spidev->spi->dev, "device is suspend\n");
        return -EBUSY;
    }

    if (spi == NULL){
        return -ESHUTDOWN;
    }

    /* use the buffer lock here for triple duty:
     *  - prevent I/O (from us) so calling spi_setup() is safe;
     *  - prevent concurrent SPI_IOC_WR_* from morphing
     *    data fields while SPI_IOC_RD_* reads them;
     *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
     */
    mutex_lock(&spidev->buf_lock);

    switch (cmd) {
    case SPI_HW_RESET :
         spidev_reset_hw(spidev);
         break;
	case SPI_HW_SHUTDOWN:
		 spidev_shutdown_hw(spidev);
		 break;
    case SPI_SYNC_READ:
        retval =  __get_user(tmp,  (u32 __user *)arg);
        dev_dbg(&spi->dev, "SPI_SYNC_READ: pagesize=%d\n", tmp);
        if (retval == 0) {
            retval = __spidev_sync_read(spidev, 0, tmp);
        } else {
            dev_err(&spi->dev, "SPI_SYNC_READ:failed get_user\n");
        }
        break;
    case SPI_ASYNC_READ_PRE:
        dev_dbg(&spi->dev, "SPI_ASYNC_READ_PRE\n");
        spidev->k_mmap_buf = spidev->u_mmap_buf =spidev->mmap_buf;
        atomic_set(&spidev->frame_num, 0);
        cancel_work_sync(&spidev->work);
        spidev_schedule_work(spidev);
        retval = 0;
        break;
    case SPI_ASYNC_READ:
        dev_dbg(&spi->dev, "SPI_ASYNC_READ\n");
        retval = get_buffer(spidev);
        break;
    case SPI_GET_BUFFER_SIZE:
        dev_dbg(&spi->dev, "SPI_GET_BUFFER_SIZE\n");
        retval = __put_user(spidev->max_buf_size,
                            (__u32 __user *)arg);
        /* read requests */
    case SPI_IOC_RD_MODE:
        retval = __put_user(spi->mode & SPI_MODE_MASK,
                            (__u8 __user *)arg);
        break;
    case SPI_IOC_RD_LSB_FIRST:
        retval = __put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
                            (__u8 __user *)arg);
        break;
    case SPI_IOC_RD_BITS_PER_WORD:
        retval = __put_user(spi->bits_per_word, (__u8 __user *)arg);
        break;
    case SPI_IOC_RD_MAX_SPEED_HZ:
        retval = __put_user(spi->max_speed_hz, (__u32 __user *)arg);
        break;

        /* write requests */
    case SPI_IOC_WR_MODE:
        retval = __get_user(tmp, (u8 __user *)arg);
        if (retval == 0) {
            u8	save = spi->mode;

            if (tmp & ~SPI_MODE_MASK) {
                retval = -EINVAL;
                break;
            }

            tmp |= spi->mode & ~SPI_MODE_MASK;
            spi->mode = (u8)tmp;
            retval = spi_setup(spi);
            if (retval < 0)
                spi->mode = save;
            else
                dev_dbg(&spi->dev, "spi mode %02x\n", tmp);
        }
        break;
    case SPI_IOC_WR_LSB_FIRST:
        retval = __get_user(tmp, (__u8 __user *)arg);
        if (retval == 0) {
            u8	save = spi->mode;

            if (tmp)
                spi->mode |= SPI_LSB_FIRST;
            else
                spi->mode &= ~SPI_LSB_FIRST;
            retval = spi_setup(spi);
            if (retval < 0)
                spi->mode = save;
            else
                dev_dbg(&spi->dev, "%csb first\n",tmp ? 'l' : 'm');
        }
        break;
    case SPI_IOC_WR_BITS_PER_WORD:
        retval = __get_user(tmp, (__u8 __user *)arg);
        if (retval == 0) {
            u8	save = spi->bits_per_word;

            spi->bits_per_word = tmp;
            retval = spi_setup(spi);
            if (retval < 0)
                spi->bits_per_word = save;
            else
                dev_dbg(&spi->dev, "%d bits per word\n", tmp);
        }
        break;
    case SPI_IOC_WR_MAX_SPEED_HZ:
        retval = __get_user(tmp, (__u32 __user *)arg);
        if (retval == 0) {
            u32	save = spi->max_speed_hz;

            spi->max_speed_hz = tmp;
            retval = spi_setup(spi);
            if (retval < 0)
                spi->max_speed_hz = save;
            else
                dev_dbg(&spi->dev, "%d Hz (max)\n", tmp);
        }
        break;
	case SPI_HW_IRQ_ENBALE:
		//printk("%s:%d", __FUNCTION__, arg);
		if(arg)
		{
			//int mode
			printk("int mode");
			sl_fp_write(fp_spidev,0xbf,0);	
			enable_irq(spidev->irq);		//daizhiling open			
			//mt_eint_unmask(spidev->irq); //daizhiling remove
		}
		else
		{
			//polling mode
			printk("polling mode");
			//mt_eint_mask(spidev->irq);
			//disable_irq_nosync(spidev->irq);
		}
		break;

    default:
		printk("----------1111111111111111-------------------\n");
//		check_id(spidev);
        /* segmented and/or full-duplex I/O request */
        if (_IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
                || _IOC_DIR(cmd) != _IOC_WRITE) {
            retval = -ENOTTY;
            break;
        }

        tmp = _IOC_SIZE(cmd);
        if ((tmp % sizeof(struct spi_ioc_transfer)) != 0) {
            retval = -EINVAL;
 
            break;
        }
        n_ioc = tmp / sizeof(struct spi_ioc_transfer);
        if (n_ioc == 0){            
		break;
	}

        /* copy into scratch area */
        ioc = kmalloc(tmp, GFP_KERNEL);
        if (!ioc) {
            retval = -ENOMEM;
            break;
        }
        if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
            kfree(ioc);
            retval = -EFAULT;
            break;
        }

        /* translate to spi_message, execute */
        retval = spidev_message(spidev, ioc, n_ioc);
//		printk("==========retval = %d\n",retval);
        kfree(ioc);
        break;
    }

    mutex_unlock(&spidev->buf_lock);
    spi_dev_put(spi);

//	printk("retval = %d\n",retval);
    return retval;
}

#if 1 //def CONFIG_COMPAT
static long
spidev_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return spidev_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define spidev_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int spidev_open(struct inode *inode, struct file *filp)
{
    struct spidev_data	*spidev = NULL;
    int			status = -ENXIO;
    spidev = fp_spidev;

	if (atomic_read(&fp_spidev->is_cal_mode)){
	//	dev_dbg(&fp_spidev->spi->dev, "Current stat is cal mode\n");
		return -EACCES;
	}

	mutex_lock(&device_list_lock);

	list_for_each_entry(spidev, &device_list, device_entry) {
	if (spidev->devt == inode->i_rdev) {
		status = 0;
		break;
	}
	}
	if (status == 0) {
		if (!spidev->buffer) {
			spidev->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (!spidev->buffer) {
				dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		if (status == 0) {
			spidev->users++;
			filp->private_data = spidev;
			nonseekable_open(inode, filp);
		}
	} else {
		//pr_debug("spidev: nothing for minor %d\n", iminor(inode));
	}

	mutex_unlock(&device_list_lock);
	return status;
}

static int spidev_release(struct inode *inode, struct file *filp)
{
	struct spidev_data	*spidev;
	int			status = 0;

	mutex_lock(&device_list_lock);
	spidev = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	spidev->users--;
	if (!spidev->users) {
		int dofree;
		if (spidev->buffer) {
			kfree(spidev->buffer);
			spidev->buffer = NULL;
		}
		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&spidev->spi_lock);
		dofree = (spidev->spi == NULL);
		spin_unlock_irq(&spidev->spi_lock);

		if (dofree)
			kfree(spidev);
	}
	cancel_work_sync(&spidev->work);
	mutex_unlock(&device_list_lock);
	return status;
}

static const struct file_operations spidev_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	* gets more complete API coverage.  It'll simplify things
	* too, except for the locking.
	*/
	.write =	spidev_write,
	.read =		spidev_read,
	.unlocked_ioctl = spidev_ioctl,
	.compat_ioctl = spidev_compat_ioctl,
	.open =		spidev_open,
	.release =	spidev_release,
	.llseek =	no_llseek,
	.mmap = spidev_mmap,
};

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */
 #ifdef SILEAD_DEBUF
static void sl_unregister_input(struct spidev_data *spidev)
{
    if (spidev->input){
	    input_unregister_device(spidev->input);
	    input_free_device(spidev->input);
        spidev->input = NULL;
    }
}

static int sl_register_input(struct spidev_data *spidev)
{
    int ret = 0;

      printk("silead: sl_register_input\n");

	spidev->input = input_allocate_device();
	if (!spidev->input){
        printk("silead input alloc failed\n");
        spidev->input = NULL;
		return -ENOMEM;
    }

	spidev->input->name = input_name;
	spidev->input->phys = input_name;
	spidev->input->id.bustype = BUS_HOST; // 
	/*
	spi_sensor->input_dev->id.vendor = 0xDEAD;
	spi_sensor->input_dev->id.product = 0xBEEF;
	spi_sensor->input_dev->id.version = 0x0102;
	*/
	set_bit(EV_KEY, spidev->input->evbit);
	//set_bit(KEY_HOME, spidev->input->keybit);
	set_bit(KEY_WAKEUP, spidev->input->keybit);

	device_init_wakeup(&spidev->input->dev, 1);

	ret = input_register_device(spidev->input);
	if (ret) {
		printk("failed to register sl button %d\n", ret);
		input_free_device(spidev->input);
        spidev->input = NULL;
        return ret;
	}
#if 1
	//mt_eint_set_hw_debounce(CUST_EINT_FINGER_NUM,1);
	//mt_eint_registration(CUST_EINT_FINGER_NUM, IRQF_TRIGGER_RISING, spidev_irq_routing, 1);
#else
	ret = request_irq(spidev->irq, spidev_irq_routing,
				IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, input_name, spidev);
#endif
    if (ret){
	    printk("%s[%d]: %d\n", __func__, __LINE__, ret);
		input_free_device(spidev->input);
        spidev->input = NULL;
    }
    //disable_irq(spidev->irq);
	//mt_eint_unmask(spidev->irq);
	return ret;
}
#endif
//#if DEBUG_GPIO_MODE_MY
#if 1
#define GPIO_SPI_CS_PIN_M_SPI_CSB GPIO_MODE_01
#define GPIO_SPI_SCK_PIN_M_SPI_CKB GPIO_MODE_01
#define GPIO_SPI_MISO_PIN_M_SPI_MIB GPIO_MODE_01
#define GPIO_SPI_MOSI_PIN_M_SPI_MOB GPIO_MODE_01


#define GPIO_SPI_CS_PIN (GPIO0 | 0x80000000)
#define GPIO_SPI_SCK_PIN (GPIO1 | 0x80000000)
#define GPIO_SPI_MISO_PIN (GPIO3 | 0x80000000)
#define GPIO_SPI_MOSI_PIN (GPIO2 | 0x80000000)
#endif
static int mt_spi_gpio_set(void)
{
	//sip cs pin
	mt_set_gpio_mode(GPIO_SPI_CS_PIN, GPIO_SPI_CS_PIN_M_SPI_CSB);
	mt_set_gpio_dir(GPIO_SPI_CS_PIN, GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(GPIO_SPI_CS_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_SPI_CS_PIN,GPIO_PULL_UP);//GPIO_PULL_UP

	//sip clk pin
	mt_set_gpio_mode(GPIO_SPI_SCK_PIN, GPIO_SPI_SCK_PIN_M_SPI_CKB);
	mt_set_gpio_dir(GPIO_SPI_SCK_PIN, GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(GPIO_SPI_SCK_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_SPI_SCK_PIN,GPIO_PULL_DOWN);

	//sip miso pin
	mt_set_gpio_mode(GPIO_SPI_MISO_PIN, GPIO_SPI_MISO_PIN_M_SPI_MIB);
	mt_set_gpio_dir(GPIO_SPI_MISO_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_SPI_MISO_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_SPI_MISO_PIN,GPIO_PULL_UP);
	

	//sip mosi pin
	mt_set_gpio_mode(GPIO_SPI_MOSI_PIN, GPIO_SPI_MOSI_PIN_M_SPI_MOB);
	mt_set_gpio_dir(GPIO_SPI_MOSI_PIN, GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(GPIO_SPI_MOSI_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_SPI_MOSI_PIN,GPIO_PULL_UP); 
	printk("nasri...%s %d\n",__func__,__LINE__);
			
	
    return 0;
}	

#if 0 //def COMPATIBLE_SUPPORT
static int check_id(struct spidev_data	*spidev)
{
	struct spidev_data	*m_spidev = spidev;
	int m_repeat_count = 3;
	int m_ret = 0;
	int m_verno = 0;
	printk("check silead id!!!");
	while(m_repeat_count--)
	{
		m_verno = spidev_read_reg(m_spidev, 0xfc);
		//	m_ret = sl_fp_read(m_spidev, 0xfc, &m_verno, 4);
		printk("m_verno->0x%08x *** m_verno>>12->0x%08x----\n", m_verno, m_verno>>(4*3));
		m_verno >>= (4*3);
		//		if(0x6163b == m_verno)
		//			break;
	}
	if(m_repeat_count < 0)
	{
		m_ret = -ENODEV;
	}
	return m_ret;
}
#endif


static struct class *spidev_class;
/* -------------------------------------------------------------------- */


/*-------------------------------------------------------------------------*/
/*daizhiling add for silead eint begin*/
//static int spidev_setup_eint(struct spi_device *spi);
//static int __devinit spidev_probe(struct spi_device *spi)
//extern struct platform_device *fingerprint;//daizhiling add for power on silead
static int spidev_probe(struct spi_device *spi)
{
    struct spidev_data	*spidev;
    int			status;
    unsigned long		minor, page;
    //int ret  = 0;
    /* Allocate driver data */	
    spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
    if (!spidev)
    return -ENOMEM;
	
    /* Initialize the driver data */
    spidev->spi = spi;
    printk("spidev_probe\n");   
    spin_lock_init(&spidev->spi_lock);
    mutex_init(&spidev->buf_lock);

    INIT_LIST_HEAD(&spidev->device_entry);
    INIT_WORK(&spidev->work, spidev_work);
    INIT_WORK(&spidev->irq_work,spidev_irq_work);
    wake_lock_init(&spidev->wake_lock, WAKE_LOCK_SUSPEND, "silead_wake_lock");
    spidev->wqueue = create_singlethread_workqueue("silead_wq");

    spidev->max_frame_num = SL_MAX_FRAME_NUM;
    spidev->max_buf_size =((sizeof(struct sl_frame)*spidev->max_frame_num+PAGE_SIZE)/PAGE_SIZE)*PAGE_SIZE;
    spidev->mmap_buf = kmalloc(spidev->max_buf_size, GFP_KERNEL);
    spidev->tx_mmap_buf = kmalloc(spidev->max_buf_size, GFP_KERNEL);
    memset(spidev->tx_mmap_buf, 0, spidev->max_buf_size);
    memset(spidev->mmap_buf, 0, spidev->max_buf_size);
    if (!spidev->mmap_buf) {
        dev_err(&spi->dev, "alloc kebuffer failedn\n");
        return -ENOMEM;
    }
    for(page = (unsigned long)spidev->mmap_buf;
    page < (unsigned long)spidev->mmap_buf+spidev->max_buf_size; page+= PAGE_SIZE) {
        SetPageReserved(virt_to_page(page));
    }
    /* If we can allocate a minor number, hook up this device.
    * Reusing minors is fine so long as udev or mdev is working.
    */
    mutex_lock(&device_list_lock);
    minor = find_first_zero_bit(minors, N_SPI_MINORS);
    if (minor < N_SPI_MINORS) {
        struct device *dev;

        spidev->devt = MKDEV(spidev_major, minor);
        dev = device_create(spidev_class, &spi->dev, spidev->devt,
        spidev, "silead_fp_dev");
        status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
    } else {
        dev_dbg(&spi->dev, "no minor number available!\n");
        status = -ENODEV;
    }
    if (status == 0) {
        set_bit(minor, minors);
        list_add(&spidev->device_entry, &device_list);
    }

    mutex_unlock(&device_list_lock);

	/* power on */
	spidev_power_on(spidev, 1);
	spidev_reset_hw(spidev);//daizhiling modify reset

    if (status == 0){
		printk("silead: status == 0\n");
        spi_set_drvdata(spi, spidev);
        fp_spidev = spidev;
        #ifdef SILEAD_PROC
        sl_proc_init(spidev);
        #endif
        atomic_set(&spidev->is_cal_mode, 0);///default is enroll mode
        atomic_set(&spidev->is_suspend, 0);///default is enroll mode
        suspend_flag = 0;
        #ifdef SILEAD_PROC	
        //spidev->hw_reset_gpio = EXYNOS4_GPX3(3);
        //spidev->wake_up_gpio = EXYNOS4_GPX3(4);
        //spidev->irq = gpio_to_irq(spidev->wake_up_gpio);
        //spidev->wake_up_gpio = GPIO_FINGER_EINT_PIN;
        spidev->wake_up_enable = 1;	
        //spidev->irq = CUST_EINT_FINGER_NUM;
		
		#ifdef SILEAD_DEBUF //daizhiling add for build error
        sl_register_input(spidev);
		#endif
        #if defined(CONFIG_HAS_EARLYSUSPEND)
        printk("silead:register early suspend\n");
        si_early_suspend.suspend = silead_early_suspend,
        si_early_suspend.resume = silead_late_resume,
        si_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 2,
        register_early_suspend(&si_early_suspend);
        #endif

 //       pgframe = (unsigned char *)kmalloc(128*109+1024, GFP_KERNEL);
#if 0
//#ifdef COMPATIBLE_SUPPORT
		spidev_reset_hw(spidev);
		if(check_id(fp_spidev) < 0)
			goto error;
#endif
        printk("silead: register EINT\n");

//		spidev_setup_eint(spi);//daizhiling add for eint
		fingerprint_irq_registration();
			

        printk("silead: probe done\n");
        #endif
    }
    else
    {
		kfree(spidev);
		return status;
    }

	printk("exit!!!");
    return status;
#if 0//def  COMPATIBLE_SUPPORT
error:
	printk("silead dev non!!!");
    if(spidev->devt != 0) {
        mutex_lock(&device_list_lock);
        //sysfs_remove_group(); spi_set_drvdata(spi, NULL);
        list_del(&spidev->device_entry);
        device_destroy(spidev_class, spidev->devt);
    	clear_bit(MINOR(spidev->devt), minors);
        mutex_unlock(&device_list_lock);
    }
	mutex_lock(&device_list_lock);
    if (spidev->mmap_buf) {
        for(page = (unsigned long)spidev->mmap_buf;
                page < (unsigned long)spidev->mmap_buf+spidev->max_buf_size; page+= PAGE_SIZE) {
            ClearPageReserved(virt_to_page(page));
        }

        kfree(spidev->mmap_buf);
    }
    if (spidev->tx_mmap_buf){
        kfree(spidev->tx_mmap_buf);
    }
    wake_lock_destroy(&spidev->wake_lock);
#ifdef SILEAD_DEBUF 
        sl_unregister_input(spidev);
#endif
    //unregister_early_suspend(&si_early_suspend);//daizhiling remove for build error
	//spi_unregister_driver(&spidev_spi_driver);
    mutex_unlock(&device_list_lock);
    if(pgframe)
    {
        kfree(pgframe);
    }
	return status;	
#endif
}
//added by nasri.
//FINGER_PRINT_EINT_STRING should be same with cust_eint.dtsi eint string. 
#if 1
#define FINGER_PRINT_EINT_STRING "FINGERPRINT-eint"
static int fingerprint_irq_registration(void)
{

	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = { 0, 0 };
#if 0
	hct_finger_int_as_int = pinctrl_lookup_state(hct_finger_pinctrl, "finger_int_as_int");
	if (IS_ERR(hct_finger_int_as_int)) {
		ret = PTR_ERR(hct_finger_int_as_int);
		dev_err(&fingerprint_device->dev, " Cannot find hct_finger pinctrl hct_finger_spi0_cs_as_gpio!\n");
		return ret;
	}
	pinctrl_select_state(hct_finger_pinctrl, hct_finger_int_as_int);
#endif
	node = of_find_compatible_node(NULL, NULL, "mediatek, dtv-eint");
#if 1
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);
		printk("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);
		
		fingerprint_irq = irq_of_parse_and_map(node, 0);
	
		printk("nasri...fingerprint_irq = %d\n",fingerprint_irq);	
		fp_spidev->irq = fingerprint_irq;	
		ret = request_irq(fingerprint_irq, (irq_handler_t)spidev_irq_routing, IRQF_TRIGGER_NONE,FINGER_PRINT_EINT_STRING, NULL);
		//if (request_irq(spidev->irq, spidev_irq_routing,IRQF_TRIGGER_NONE,  "Silead-eint",NULL)) //IRQF_TRIGGER_RISING

		//disable_irq(fp_spidev->irq);

		if (ret > 0) {
			ret = -1;
		}
		
	} else {
		ret = -1;
	}
#endif
	printk("nasri....%s %d ret = %d\n",__func__,__LINE__,ret);

	return ret;
}
#endif

#if 0
#define GPIO_FINGERPRINT_EINT_PIN (GPIO9 | 0x80000000)
#define GPIO_FINGERPRINT_EINT_PIN_M_EINT GPIO_MODE_00
static int spidev_setup_eint(struct spi_device *spi)
{	
	struct device_node *node = NULL;
	struct spidev_data *spidev = fp_spidev;	
	//struct pinctrl *pinctrl;
	//struct pinctrl_state *pins_default;
	//struct pinctrl_state *pins_cfg;
	u32 ints[2] = {0, 0};
	printk("silead: spidev_setup_eint\n");
	/* Allocate driver data */	
	//spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
	//if (!spidev)
	//return -ENOMEM;
	/* gpio setting */
        mt_set_gpio_mode(GPIO_FINGERPRINT_EINT_PIN, GPIO_FINGERPRINT_EINT_PIN_M_EINT);
        mt_set_gpio_dir(GPIO_FINGERPRINT_EINT_PIN, GPIO_DIR_IN);
        mt_set_gpio_pull_enable(GPIO_FINGERPRINT_EINT_PIN, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(GPIO_FINGERPRINT_EINT_PIN, GPIO_PULL_DOWN);
	#if 0
	pinctrl = devm_pinctrl_get(&spi->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		printk("Cannot find spidev pinctrl!\n");
	}
	pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
	if (IS_ERR(pins_default)) {
		ret = PTR_ERR(pins_default);
		printk("Cannot find spidev pinctrl default!\n");

	}
	pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		printk("Cannot find spidev pinctrl pin_cfg!\n");

	}
	pinctrl_select_state(pinctrl, pins_cfg); //daizhiling add
	#endif
	printk("silead: spidev_setup_eint111\n");
	/* eint request */
	node = of_find_compatible_node(NULL, NULL, "mediatek, fingerprint-eint");
	printk("silead: spidev_setup_eint111\n");
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "Silead_fp");
		gpio_set_debounce(ints[0], ints[1]);
		printk("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

		spidev->irq = irq_of_parse_and_map(node, 0);
		printk("spidev->irq = %d\n", spidev->irq);
		if (!spidev->irq) {
			printk("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}

		if (request_irq(spidev->irq, (irq_handler_t)spidev_irq_routing, IRQF_TRIGGER_RISING, "Silead-eint", NULL))
		{
			printk("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
	} else {
			printk("null irq node!!\n");
		   }
	return 0;
}
/*daizhiling add for silead eint end*/
#endif

static int __exit spidev_remove(struct spi_device *spi)
//int spidev_remove(struct spi_device *spi)
{
    struct spidev_data	*spidev = spi_get_drvdata(spi);
    unsigned long page;

    /* make sure ops on existing fds can abort cleanly */
    spin_lock_irq(&spidev->spi_lock);
    spidev->spi = NULL;
    spi_set_drvdata(spi, NULL);
    spin_unlock_irq(&spidev->spi_lock);

    /* prevent new opens */
    mutex_lock(&device_list_lock);
    list_del(&spidev->device_entry);
    device_destroy(spidev_class, spidev->devt);
    clear_bit(MINOR(spidev->devt), minors);
    if (spidev->users == 0) {
        if (spidev->mmap_buf) {
            for(page = (unsigned long)spidev->mmap_buf;
                    page < (unsigned long)spidev->mmap_buf+spidev->max_buf_size; page+= PAGE_SIZE) {
                ClearPageReserved(virt_to_page(page));
            }

            kfree(spidev->mmap_buf);
        }
        if (spidev->tx_mmap_buf){
            kfree(spidev->tx_mmap_buf);
        }
        wake_lock_destroy(&spidev->wake_lock);
    #ifdef SILEAD_DEBUF	
        sl_unregister_input(spidev);
    #endif
	kfree(spidev);
       
    }
    mutex_unlock(&device_list_lock);

    return 0;
}

static int spidev_suspend(struct device *spi)//, pm_message_t mesg)
{
    struct spidev_data	*spidev = NULL; //spi_get_drvdata(spi);
    spidev = fp_spidev;
printk("spidev_suspend enter.....\n");
#if 0
    if (spidev->wake_up_enable) {
			printk("aaaaaaaaaaaaaaaaaaaaa\n");
//    	enter_sleepmode(fp_spidev);
    	suspend_flag = 1;
        enable_irq(spidev->irq);
    } else {
    	printk("bbbbbbbbbbbbbbbbbbb\n");
        disable_irq(spidev->irq);
    }
    //atomic_set(&spidev->is_suspend, 0); 
    atomic_set(&fp_spidev->is_suspend, 1);
    suspend_flag = 1;
    printk("spidev_suspend >>>>> ssuspend_flag--->%d\n",suspend_flag);
    dev_info(&spidev->spi->dev, "%s\n", __func__);
#if (DEBUG_GPIO_SHUTDOWN_STATE_MY)
	mt_set_gpio_mode(GPIO_FIGERPRINT_RST,GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_FIGERPRINT_RST,GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_FIGERPRINT_RST,0);
#endif
#endif
	//suspend_flag = 1;
	//atomic_set(&spidev->is_suspend, 1);
	printk("spidev_suspend >>>>> enable_irq --------by tony>> ssuspend_flag--->%d\n",suspend_flag);
    return 0;
}
static int spidev_resume(struct device *spi)
{

	printk("spidev_resume enter.....\n");
	//suspend_flag = 0;
	printk("spidev_resume >>>> disable_irq suspend_flag--->%d\n",suspend_flag);
//	disable_irq(spidev->irq);
	//enter_sleepmode(fp_spidev);
    return 0;
}

//struct spi_device_id silead_spi_id_table = {"silead_fp", 0};
/*daizhiling modify for build error begin*/

static const struct dev_pm_ops silead_pm = {
	.suspend = spidev_suspend,
	.resume  = spidev_resume
};
#if 0
struct of_device_id silead_of_match[] = {
	{ .compatible = "mediatek,silead_fp", },
	{},
};
#endif
static struct spi_driver spidev_spi_driver = {
    .driver = {
        .name =		"silead_fp",//spidev
        .owner =	THIS_MODULE,
        //.of_match_table = silead_of_match,//daizhiling add for power on
	.pm    =        &silead_pm
    },
    .probe =	spidev_probe,
    .remove =	__exit_p(spidev_remove),
    //.id_table = &silead_spi_id_table,

//    .suspend  = spidev_suspend,
//    .resume   = spidev_resume,
    
    /* NOTE:  suspend/resume methods are not necessary here.
     * We don't do anything except pass the requests to/from
     * the underlying controller.  The refrigerator handles
     * most issues; the controller driver handles the rest.
     */
};
/*daizhiling modify for build error end*/
static struct spi_board_info sfp_spi1_board_info[] __initdata = {
	[0] = {
		.modalias = "silead_fp",
//		.platform_data = NULL,
		.max_speed_hz = SPI_SPEED, // 4*1000*1000, origin
		.bus_num = 0,
		.chip_select = 0,// 0 daizhiling modify for CS can not be pull down
        .mode = SPI_MODE_0, //SPI_MODE_0, //SPI_MODE_1 origin ,
	    .controller_data=&chip_config
	},
};

/*-------------------------------------------------------------------------*/

static int fingerprint_dts_probe(struct platform_device *dev)
{
	printk("nasri...%s %d\n",__func__,__LINE__);
	fingerprint_device = dev;
//	fingerprint_get_gpio_info(dev);
	return 0;
}
#if 0
struct platform_device * fingerprint_get_dev()
{
	return fingerprint_device;
}
#endif
const struct dev_pm_ops fingerprint_pm_ops = {
	.suspend = NULL,
	.resume = NULL,
};
static struct platform_driver fingerprint_dts_driver = {
	.probe = fingerprint_dts_probe,
	.remove = NULL,
	.shutdown = NULL,
	.driver = {
		   .name = "dtv-eint",
		   .pm = &fingerprint_pm_ops,
		    .owner = THIS_MODULE,
		   .of_match_table = fingerprint_of_match,
		   },
};

static int __init spidev_init(void)
{
    int status;
    dev_t devno;

    printk("==============slspi===========\n");
    /* Claim our 256 reserved device numbers.  Then register a class
     * that will key udev/mdev to add/remove /dev nodes.  Last, register
     * the driver which manages those device numbers.
     */
    BUILD_BUG_ON(N_SPI_MINORS > 256);
	//#if (DEBUG_GPIO_MODE_MY)
    mt_spi_gpio_set();
	//#elif (DEBUG_GPIO_MODE_ORIGIN)
	//mt_spi_gpio_set();
	//#endif
	status = platform_driver_register(&fingerprint_dts_driver);

    spi_register_board_info(sfp_spi1_board_info, ARRAY_SIZE(sfp_spi1_board_info));
    status = alloc_chrdev_region(&devno, 0,255, "sileadfp");
    if(status <0 )
        return status;
    spidev_major = MAJOR(devno);
    printk("-----------------spidev_major = %d. \n",(int)spidev_major);
    cdev_init(&spicdev, &spidev_fops);
    spicdev.owner = THIS_MODULE;
    status = cdev_add(&spicdev,MKDEV(spidev_major, 0),N_SPI_MINORS);
    if(status != 0)
        return status;
    spidev_class = class_create(THIS_MODULE, "spidev");
    if (IS_ERR(spidev_class)) {
        unregister_chrdev(spidev_major, spidev_spi_driver.driver.name);
        return PTR_ERR(spidev_class);
    }
    status = spi_register_driver(&spidev_spi_driver);
	printk("status->%d", status);
    if (status < 0) {
        class_destroy(spidev_class);
        unregister_chrdev(spidev_major, spidev_spi_driver.driver.name);
    }
    return status;
}

static void __exit spidev_exit(void)
{
    cdev_del(&spicdev);
    spi_unregister_driver(&spidev_spi_driver);
    class_destroy(spidev_class);
    unregister_chrdev(spidev_major, spidev_spi_driver.driver.name);
}

module_init(spidev_init);
module_exit(spidev_exit);

MODULE_AUTHOR("Andrea Paterniani, <a.paterniani@swapp-eng.it>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spidev");
