/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __MT65XX_LCM_LIST_H__
#define __MT65XX_LCM_LIST_H__

#include <lcm_drv.h>

extern LCM_DRIVER otm1283a_cmi50_tps65132_hd_lcm_drv;
extern LCM_DRIVER nt35521_boe50_huaxian_hd_lcm_drv;
extern LCM_DRIVER r63350_sharp50_wsc_hd_lcm_drv;
extern LCM_DRIVER r63315_sharp50_wsc_hd_lcm_drv;
extern LCM_DRIVER s6d7aa0_qijing50_jdf_hd_lcm_drv;
extern LCM_DRIVER s6d7aa0_qijing50_jdf_hd_n40_lcm_drv;

#ifdef BUILD_LK
extern void mdelay(unsigned long msec);
#endif

#endif
