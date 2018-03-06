/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <linux/io.h>
#include <linux/uaccess.h>
#include <mach/gpio_const.h>
#include <mt_gpio.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/leds.h>
#include <linux/mutex.h>

/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */
#define TAG_NAME "leds_strobe.c"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME"%s: " fmt, __func__ , ##arg)
#define PK_WARN(fmt, arg...)        pr_warn(TAG_NAME"%s: " fmt, __func__ , ##arg)
#define PK_NOTICE(fmt, arg...)      pr_debug(TAG_NAME"%s: " fmt, __func__ , ##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME"%s: " fmt, __func__ , ##arg)
#define PK_TRC_FUNC(fmt)              pr_debug(TAG_NAME "<%s>\n", fmt, __func__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME"%s: " fmt, __func__ , ##arg)


#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#define PK_VER PK_TRC_VERBOSE
#define PK_ERR PK_ERROR
#else
#define PK_DBG(a, ...)
#define PK_VER(a, ...)
#define PK_ERR(a, ...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;

static int g_duty = -1;
static int g_timeOutTimeMs;

static DEFINE_MUTEX(g_strobeSem);

/* not used */
/* #define STROBE_DEVICE_ID 0x63 */


static struct work_struct workTimeOut;

#ifndef GPIO_CAMERA_FLASH_MODE_PIN
#define GPIO_CAMERA_FLASH_MODE_PIN (GPIO3 | 0x80000000)
#endif

#ifndef GPIO_CAMERA_FLASH_EN_PIN
#define GPIO_CAMERA_FLASH_EN_PIN   (GPIO1 | 0x80000000)
#endif

#define FLASH_GPIO_ENF GPIO_CAMERA_FLASH_MODE_PIN
#define FLASH_GPIO_ENT GPIO_CAMERA_FLASH_EN_PIN

/*****************************************************************************
Functions
*****************************************************************************/
/*
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData,
		       u16 i2cId);
*/
static void work_timeOutFunc(struct work_struct *data);



int FL_Enable(void)
{
	if (g_duty == 1)	/* torch mode */
		mt_set_gpio_out(FLASH_GPIO_ENF, GPIO_OUT_ZERO);
	else			/* flash mode */
		mt_set_gpio_out(FLASH_GPIO_ENF, GPIO_OUT_ONE);

	mt_set_gpio_out(FLASH_GPIO_ENT, GPIO_OUT_ONE);

	return 0;
}



int FL_Disable(void)
{
	mt_set_gpio_out(FLASH_GPIO_ENF, GPIO_OUT_ZERO);
	mt_set_gpio_out(FLASH_GPIO_ENT, GPIO_OUT_ZERO);
	/* PK_DBG(" FL_Disable line=%d\n", __LINE__); */
	return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	/* PK_DBG(" FL_dim_duty line=%d\n", __LINE__); */
	g_duty = duty;
	return 0;
}


int FL_Init(void)
{
	if (mt_set_gpio_mode(FLASH_GPIO_ENT, GPIO_MODE_00))
		PK_DBG("[constant_flashlight] set gpio mode failed!!\n");
	if (mt_set_gpio_dir(FLASH_GPIO_ENT, GPIO_DIR_OUT))
		PK_DBG("[constant_flashlight] set gpio dir failed!!\n");
	if (mt_set_gpio_out(FLASH_GPIO_ENT, GPIO_OUT_ZERO))
		PK_DBG("[constant_flashlight] set gpio failed!!\n");
	if (mt_set_gpio_mode(FLASH_GPIO_ENF, GPIO_MODE_00))
		PK_DBG("[constant_flashlight] set gpio mode failed!!\n");
	if (mt_set_gpio_dir(FLASH_GPIO_ENF, GPIO_DIR_OUT))
		PK_DBG("[constant_flashlight] set gpio dir failed!!\n");
	if (mt_set_gpio_out(FLASH_GPIO_ENF, GPIO_OUT_ZERO))
		PK_DBG("[constant_flashlight] set gpio failed!!\n");

	INIT_WORK(&workTimeOut, work_timeOutFunc);
	/* PK_DBG(" FL_Init line=%d\n", __LINE__); */
	return 0;
}


int FL_Uninit(void)
{
	FL_Disable();
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
	/* printk(KERN_ALERT "work handler function./n"); */
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	g_timeOutTimeMs = 1000;	/* 1s */
	hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_timeOutTimer.function = ledTimeOutCallback;

}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
	/* PK_DBG("constant_flashlight_ioctl() line=%d ior_shift=%d,
		iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg); */
	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		/* PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg); */
		g_timeOutTimeMs = arg;
		break;
	case FLASH_IOC_SET_DUTY:
		/* PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg); */
		FL_dim_duty(arg);
		break;
	case FLASH_IOC_SET_STEP:
		/* PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg); */
		break;
	case FLASH_IOC_SET_ONOFF:
		/* PK_DBG("FLASHLIGHT_ONOFF: %d\n", (int)arg); */
		if (arg == 1) {
			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;

				ktime = ktime_set(0, g_timeOutTimeMs * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			FL_Enable();
		} else {
			FL_Disable();
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;
	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;

	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_ERR(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}


FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);
