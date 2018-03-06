/**
 * goodix common header file
 */

#ifndef __GF_COMMON_H
#define __GF_COMMON_H
#include <linux/of_irq.h>
#include <linux/notifier.h>
//#include <mach/mt_gpio.h>
//#include <mach/mt_spi.h>
//#include <mach/eint.h>
//#include <cust_eint.h>
#include <linux/platform_data/spi-mt65xx.h>
#include <linux/spi/spi.h>
//#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>

#include <linux/of.h>//add xielei
#include <mt_spi.h>//add xielei
#include <mt_spi_hal.h>//add xielei
//kernel-3.18/drivers/spi/mediatek/mt6735/mt_spi.h
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include "gf-milanf.h"

typedef struct{
	dev_t devt;
	struct spi_device *spi;
	struct list_head device_entry;
	struct input_dev *input;
	struct workqueue_struct *spi_wq;
	struct mutex buf_lock;
	struct mutex frame_lock;
	struct timer_list gf_timer;
#ifdef GF_FASYNC
	struct fasync_struct *async;
#endif
	struct notifier_block notifier;
	spinlock_t spi_lock;
	wait_queue_head_t waiter;
	unsigned users;
	u8 *buffer;	
	u8 buf_status;	

	/* buffer is NULL unless this device is open (users > 0) */
	unsigned long irq_gpio;
	unsigned long rst_gpio;
	unsigned int poweron;
	unsigned int esd_running;
	u8 mode;
#if CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_fp;
#endif
}gf_dev_t;


typedef enum {
	SPEED_500KHZ=0,
	SPEED_1MHZ,
	SPEED_2MHZ,
	SPEED_3MHZ,
	SPEED_4MHZ,
	SPEED_6MHZ,
	SPEED_8MHZ,
	SPEED_KEEP,
	SPEED_UNSUPPORTED
}SPI_SPEED;

static struct mt_chip_conf spi_conf_mt65xx = {
	.setuptime = 15,
	.holdtime = 15,
	.high_time = 21, 
	.low_time = 21,	
	.cs_idletime = 20,
	.ulthgh_thrsh = 0,
	.cpol = 0,
	.cpha = 0,
	.rx_mlsb = 1,
	.tx_mlsb = 1,
	.tx_endian = 0,
	.rx_endian = 0,
	.com_mod = FIFO_TRANSFER,
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};

#define MTK_SPI_ALIGN_MASK_NUM  10
#define MTK_SPI_ALIGN_MASK  ((0x1 << MTK_SPI_ALIGN_MASK_NUM) - 1)

/*******************Refering to hardware platform*****************************/
/*

#if 0
#define		GF_IRQ_NUM         1
#define		GF_IRQ_PIN_NUM     1
#define 	GF_IRQ_PIN         (GPIO1 | 0x80000000)
#define 	GF_IRQ_PIN_M_GPIO  GPIO_MODE_00
#define 	GF_IRQ_PIN_M_EINT  GPIO_MODE_04

#define 	GF_RST_PIN         (GPIO66 | 0x80000000)
#define 	GF_RST_PIN_M_GPIO  GPIO_MODE_00
#define 	GF_RST_PIN_M_DAIPCMOUT   GPIO_MODE_01

#define		GF_SCK_PIN         (GPIO6 | 0x80000000)            
#define		GF_SCK_PIN_M_GPIO  GPIO_MODE_00
#define		GF_SCK_PIN_M_SCK   GPIO_MODE_02

#define		GF_CS_PIN          (GPIO5 | 0x80000000)
#define		GF_CS_PIN_M_GPIO   GPIO_MODE_00
#define		GF_CS_PIN_M_CS     GPIO_MODE_02
	
#define		GF_MOSI_PIN        (GPIO4 | 0x80000000)
#define		GF_MOSI_PIN_M_GPIO GPIO_MODE_00
#define		GF_MOSI_PIN_M_MOSI GPIO_MODE_02

#define		GF_MISO_PIN        (GPIO3 | 0x80000000)
#define		GF_MISO_PIN_M_GPIO GPIO_MODE_00
#define		GF_MISO_PIN_M_MISO GPIO_MODE_02
#endif






#define     GF_IRQ_NUM         1
#define     GF_IRQ_PIN_NUM     1
#define     GF_IRQ_PIN         (GPIO17 | 0x80000000)
#define     GF_IRQ_PIN_M_GPIO  GPIO_MODE_00
#define     GF_IRQ_PIN_M_EINT  GPIO_MODE_04
  
#define     GF_RST_PIN         (GPIO16 | 0x80000000)
#define     GF_RST_PIN_M_GPIO  GPIO_MODE_00
#define     GF_RST_PIN_M_DAIPCMOUT   GPIO_MODE_01
 
#define     GF_SCK_PIN         (GPIO6 | 0x80000000)            
#define     GF_SCK_PIN_M_GPIO  GPIO_MODE_00
#define     GF_SCK_PIN_M_SCK   GPIO_MODE_02 

#define     GF_CS_PIN          (GPIO5 | 0x80000000)
#define     GF_CS_PIN_M_GPIO   GPIO_MODE_00
#define     GF_CS_PIN_M_CS     GPIO_MODE_02
     
#define     GF_MOSI_PIN        (GPIO4 | 0x80000000)
#define     GF_MOSI_PIN_M_GPIO GPIO_MODE_00
#define     GF_MOSI_PIN_M_MOSI GPIO_MODE_02
 
#define     GF_MISO_PIN        (GPIO3 | 0x80000000)
#define     GF_MISO_PIN_M_GPIO GPIO_MODE_00
#define     GF_MISO_PIN_M_MISO GPIO_MODE_02





















//#define     GF_FLASH_BYPASS_PIN EXYNOS4_GPK3(0)
*/
/****************Goodix hardware****************/

#define GF_W                0xF0
#define GF_R                0xF1
#define GF_WDATA_OFFSET     (0x3)
#define GF_RDATA_OFFSET     (0x5)

/**********************************************************/

/****************Function prototypes*****************/

extern int gf_spi_read_bytes(gf_dev_t *gf_dev,
                                u16 addr, u32 data_len, u8 *rx_buf);

extern int gf_spi_write_bytes(gf_dev_t *gf_dev,
                                u16 addr, u32 data_len, u8 *tx_buf);
extern int  gf_spi_read_word(gf_dev_t* gf_dev, u16 addr, u16* value);
extern int  gf_spi_write_word(gf_dev_t* gf_dev, u16 addr, u16 value);
//extern int  gf_spi_read_data(gf_dev_t* gf_dev, u16 addr, int len, u8* value, bool endian_exchange);
extern int  gf_spi_read_data(gf_dev_t* gf_dev, u16 addr, int len, u8* value);
extern int  gf_spi_read_data_bigendian(gf_dev_t* gf_dev, u16 addr, int len, u8* value);
extern int  gf_spi_write_data(gf_dev_t* gf_dev, u16 addr, int len, u8* value);
extern int  gf_spi_send_cmd(gf_dev_t* gf_dev, unsigned char* cmd, int len);
extern void gf_spi_set_mode(struct spi_device *spi, SPI_SPEED speed, int flag);
//////////////////////////////////////////////////////////////////////////////////
extern void gf_spi_setup(gf_dev_t *gf_dev, int max_speed_hz);
#endif //__GF_COMMON_H
