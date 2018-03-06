/* MicroArray Fingerprint
 * plat-mtk.h
 * date: 2015-08-20
 * version: v2.0
 * Author: czl
 */

#ifndef PLAT_MTK_H
#define PLAT_MTK_H

#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/of_irq.h>
#include "mt_spi.h"

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/ioctl.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/poll.h>
#include <linux/sort.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/spi/spidev.h>
#include <linux/spi/spi.h>
#include <linux/usb.h>
#include <linux/usb/ulpi.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/power_supply.h>
#include <linux/gpio.h>
#include <mt_gpio.h>

#include <asm/uaccess.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/of_irq.h>
#include <linux/completion.h>

#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(USE_EARLY_SUSPEND)
#include <linux/earlysuspend.h>
#endif

#include <linux/platform_device.h>


extern int hct_finger_get_gpio_info(struct platform_device *pdev);
extern int hct_finger_set_power(int cmd);
extern int hct_finger_set_reset(int cmd);
extern int hct_finger_set_spi_mode(int cmd);
extern int hct_finger_set_eint(int cmd);
extern int mas_probe(struct spi_device *spi);
extern int mas_remove(struct spi_device *spi);
extern int mas_plat_probe(struct platform_device *pdev);
extern int mas_plat_remove(struct platform_device *pdev);

//#ifdef CONFIG_OF
static struct of_device_id sof_match[] = {
	{ .compatible = "mediatek,hct_finger", },
	{}
};
MODULE_DEVICE_TABLE(of, sof_match);
//#endif

struct spi_device_id sdev_id = {"madev", 0};
struct spi_driver sdrv = {
	.driver = {
		.name =	"madev",
		.owner = THIS_MODULE,
	},
	.probe = mas_probe,
	.remove = mas_remove,
	.id_table = &sdev_id,
};

static struct platform_driver spdrv = {
	.probe	  = mas_plat_probe,
	.remove	 = mas_plat_remove,
	.driver = {
		.name  = "madev",
		.owner = THIS_MODULE,			
//#ifdef CONFIG_OF
		.of_match_table = sof_match,
//#endif
	}
};

struct mt_chip_conf smt_conf = {
	.setuptime = 1,
	.holdtime = 1,
	.high_time = 10, // 10--6m 15--4m 20--3m 30--2m [ 60--1m 120--0.5m  300--0.2m]
	.low_time = 10,
	.cs_idletime = 10,
	.ulthgh_thrsh = 0,
	.cpol = 0,
	.cpha = 0,
	.rx_mlsb = SPI_MSB,
	.tx_mlsb = SPI_MSB,
	.tx_endian = 0,
	.rx_endian = 0,
	.com_mod = DMA_TRANSFER,
	.pause = 0,
	.finish_intr = 5,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};

struct spi_board_info smt_info[] __initdata = {
	[0] = {
		.modalias = "madev",
		.max_speed_hz = SPI_SPEED,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.controller_data = &smt_conf
	}
};

void plat_set_gpio(struct platform_device *plat) {
	printk(KERN_EMERG"plat_set_gpio start\n");			
	hct_finger_get_gpio_info(plat);
	hct_finger_set_power(1);
	hct_finger_set_reset(1);
	hct_finger_set_spi_mode(1);
	hct_finger_set_eint(1);
	printk(KERN_EMERG"plat_set_gpio end\n");	
}

int plat_register_driver(void) {
	int ret;

	printd("%s: start\n", __func__);
	ret = platform_driver_register(&spdrv);
	msleep(200);
	
	

	printd("%s: end.\n", __func__);

	return ret;
}

void plat_unregister_driver(void) {
	spi_unregister_driver(&sdrv);
}

/* MTK电源开关
 * @power 1:开，0：关
 * @return 0成功，-1失败
 */
int plat_power(int power) {
	int ret = 0;

	// 仅PMU管理电源
//	if(power) {
//		ret = hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_3000, MODE_NAME);
//	} else {
//		ret = hwPowerDown(MT6323_POWER_LDO_VGP1, MODE_NAME);
//	}
	return (ret==1)? 0: -1;
}



void plat_tansfer(struct spi_device *spi, int len) {
	static int mode = -1;
	int tmp = len>32? DMA_TRANSFER: FIFO_TRANSFER;

	//printd("%s: start\n", __func__);

	if(tmp!=mode) {
		struct mt_chip_conf *conf = (struct mt_chip_conf *) spi->controller_data;
		conf->com_mod = tmp;
		spi_setup(spi);
		mode = tmp;
	}

	//printd("%s: end.\n", __func__);
}

void plat_enable_irq(struct spi_device *spi, u8 flag) {
	static int state = -1;

	//printd("%s: start\n", __func__);

	if (state != flag) {
		if (flag) {
			printd("%s: enable_irq.\n", __func__);
			enable_irq(spi->irq);
		} else {
			printd("%s: disable_irq.\n", __func__);
			disable_irq_nosync(spi->irq);
		}
		state = flag;
	} 

	//printd("%s: end.\n", __func__);
}

int plat_request_irq(unsigned int irq, irq_handler_t handler, unsigned long flags,
        const char *name, void *dev) {	
	u32 ints[2] = {0};
	struct device_node *node = NULL;
	const char*tname = "mediatek,hct_finger";
	node = of_find_compatible_node(NULL, NULL, "mediatek,hct_finger");
  of_property_read_u32_array(node,"debounce",ints,ARRAY_SIZE(ints));

	 gpio_set_debounce(ints[0], ints[1]);
	irq = irq_of_parse_and_map(node, 0);	
	
	return request_irq(irq, handler, flags, tname, dev);
}

#endif



