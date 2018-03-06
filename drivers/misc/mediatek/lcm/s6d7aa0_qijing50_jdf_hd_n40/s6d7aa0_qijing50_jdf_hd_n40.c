/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
 *  Copyright Statement:
 *  --------------------
 *  This software is protected by Copyright and the information contained
 *  herein is confidential. The software may not be copied and the information
 *  contained herein may not be used or disclosed except with the written
 *  permission of MediaTek Inc. (C) 2008
 *
 *  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 *  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 *  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
 *  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 *  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 *  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 *  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 *  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
 *  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
 *  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
 *  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
 *
 *  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
 *  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 *  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 *  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
 *  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 *  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
 *  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
 *  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
 *  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
 *  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
 *
 *****************************************************************************/

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"
#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h> 
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>
#endif

#if defined(BUILD_LK)
#define LCM_DEBUG  printf
#define LCM_FUNC_TRACE() printf("huyl [uboot] %s\n",__func__)
#else
#define LCM_DEBUG  printk
#define LCM_FUNC_TRACE() printk("huyl [kernel] %s\n",__func__)
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)
//#define GPIO_65132_EN     GPIO78 | 0x80000000//GPIO_LCD_BIAS_ENP_PIN

#define GPIO_65132_EN     GPIO15 | 0x80000000//GPIO_LCD_BIAS_ENP_PIN

#define LCM_ID_S6D7AA0X62 0x0802
#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
static LCM_UTIL_FUNCS lcm_util = {0};
#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

#define REGFLAG_DELAY             							0XAA
#define REGFLAG_END_OF_TABLE      							0x100   // END OF REGISTERS MARKER

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

struct LCM_setting_table {
  unsigned cmd;
  unsigned char count;
  unsigned char para_list[64];
};

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static struct LCM_setting_table lcm_initialization_setting[] = {

  {0xF0,2,{0x5A,0x5A}},
  {0xF1,2,{0x5A,0x5A}},
  {0xFC,2,{0xA5,0xA5}},
  {0xB1,1,{0x12}},
	{0xB2,4,{0x14,0x22,0x2F,0x18}},
  {0xB3,1,{0x11}},
  {0xBA,9,{0x03,0x19,0x19,0x11,0x03,0x05,0x18,0x18,0x77}},
	{0xBB,6,{0x07,0x4D,0x22,0x77,0x00,0x00}},
	{0xBC,3,{0x00,0x4E,0x0B}},
	{0xC0,3,{0x80,0x80,0x30}},
  {0xC1,1,{0x03}},
  {0xC2,1,{0x00}},
  {0xC3,3,{0x40,0x00,0x28}},
  {0xE1,5,{0x03,0x10,0x1C,0xA0,0x10}},
	{0xEE,8,{0xA5,0x00,0x98,0x00,0xA5,0x00,0x98,0x00}},
	{0xF2,5,{0x02,0x0C,0x10,0x44,0x10}},
  {0xF3,10,{0x01,0x93,0x20,0x22,0x80,0x05,0x25,0x3C,0x26,0x00}},
	{0xF4,45,{0x02,0x02,0x10,0x26,0x10,0x02,0x03,0x03,0x03,0x10,
			  0x16,0x03,0x00,0x0C,0x0C,0x03,0x04,0x05,0x13,0x1E,
			  0x09,0x0A,0x05,0x05,0x01,0x04,0x02,0x61,0x74,0x75,
			  0x72,0x83,0x80,0x80,0x00,0x00,0x01,0x01,0x28,0x04,0x03,0x28,0x01,0xD1,0x32}},
	{0xF5,26,{0x91,0x23,0x2D,0x5F,0xBB,0x8B,0x55,0x0F,0x33,0x33,
			  0x03,0x59,0x54,0x52,0x45,0x57,0x60,0x10,0x60,0x80,0x27,0x26,0x52,0x25,0x6F,0x1B}},
	{0xF7,32,{0x01,0x1B,0x08,0x0C,0x09,0x0D,0x01,0x01,0x01,0x04,
			  0x06,0x01,0x01,0x00,0x00,0x1A,0x01,0x1B,0x0A,0x0E,


			  0x0B,0x0F,0x01,0x01,0x01,0x05,0x07,0x01,0x01,0x00,0x00,0x1A}},

 {0xF6,6,{0x60,0x25,0x05,0x00,0x00,0x00}},
 {0xFD,5,{0x16,0x10,0x11,0x23,0x09}},
	{0xFE,6,{0x00,0x02,0x03,0x21,0x80,0x78}},

	{0xFA,17,{0x10,0x30,0x17,0x1D,0x11,0x15,0x1A,0x18,0x18,0x20,0x21,0x20,0x20,0x1F,0x1F,0x25,0x20}},
	{0xFB,17,{0x10,0x30,0x17,0x1D,0x11,0x15,0x1A,0x18,0x18,0x20,0x21,0x20,0x20,0x1F,0x1F,0x25,0x20}},
	{0xEF,17,{0x34,0x12,0x98,0xBA,0x10,0x80,0x24,0x80,0x80,0x80,0x00,0x00,0x00,0x44,0xA0,0x82,0x00}},
	{0xCD,13,{0x2E,0x2E,0x2E,0x2E,0x2E,0x2E,0x2E,0x2E,0x2E,0x2E,0x2E,0x2E,0x2E}},
	{0xCE,13,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x51,1,{0xFF}},
	{0x53,1,{0x2C}},
	{0x55,1,{0x01}},
	{0x36,1,{0x1c}},
  {0x11,1, {0x00}},		// Sleep-Out
{REGFLAG_DELAY, 150,  {0}},      
{0x29,1, {0x00}},    //Display on            
{REGFLAG_DELAY, 50,  {0}},

{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_sleep_mode_in_setting[] = {

  {0x28, 0, {0x00}},
  {REGFLAG_DELAY, 50, {}},

  {0x10, 0, {0x00}},
  {REGFLAG_DELAY, 120, {}},
  {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
  unsigned int i;

  for(i = 0; i < count; i++) {

    unsigned cmd;
    cmd = table[i].cmd;

    switch (cmd) {

      case REGFLAG_DELAY :
        MDELAY(table[i].count);
        break;

      case REGFLAG_END_OF_TABLE :
        break;

      default:
        dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
    }
  }

}


static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
  memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}
static void lcm_get_params(LCM_PARAMS *params)
{
  memset(params, 0, sizeof(LCM_PARAMS));
  params->type = LCM_TYPE_DSI;
  params->width = FRAME_WIDTH;
  params->height = FRAME_HEIGHT;
  params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
  params->dbi.te_edge_polarity = LCM_POLARITY_RISING;
  params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;
  params->dsi.LANE_NUM = LCM_THREE_LANE;//LCM_FOUR_LANE
  params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
  params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
  params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
  params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
  params->dsi.intermediat_buffer_num = 0;	//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
  params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
  params->dsi.word_count = 720 * 3;
  params->dsi.vertical_sync_active                = 4;// 3    2
  params->dsi.vertical_backporch                  = 8;// 20   1
  params->dsi.vertical_frontporch                 = 16; // 1  12
  params->dsi.vertical_active_line                = FRAME_HEIGHT;
  params->dsi.horizontal_sync_active              = 16;// 50  2
	params->dsi.horizontal_backporch				=32;//16;
  params->dsi.horizontal_frontporch               = 16;//90
  params->dsi.horizontal_active_pixel             = FRAME_WIDTH;
  /*    params->dsi.ssc_disable                         = 1;
        params->dsi.ssc_range                         = 4;
        params->dsi.HS_TRAIL                             = 15;
        params->dsi.esd_check_enable = 0;
        params->dsi.customization_esd_check_enable = 0;
        params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
        params->dsi.lcm_esd_check_table[0].count        = 1;
        params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
   */
  params->dsi.PLL_CLOCK =255;

}


static void lcm_init(void)
{
  SET_RESET_PIN(1);
  MDELAY(10);
  SET_RESET_PIN(0);
  MDELAY(10);
  SET_RESET_PIN(1);
  MDELAY(60);
  push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

}                                       

static void lcm_suspend(void)
{

  push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
  SET_RESET_PIN(1);
  MDELAY(10);
  SET_RESET_PIN(0);
  MDELAY(10);
  SET_RESET_PIN(1); 
  MDELAY(120);  
}

static void lcm_resume(void)
{
  lcm_init();
}


static unsigned int lcm_compare_id(void)
{
  unsigned int id = 0;
  unsigned int id_high=0,id_low=0;

  unsigned char buffer[3];
  unsigned int data_array[16];

  SET_RESET_PIN(1);	
  MDELAY(10);
  SET_RESET_PIN(0);
  MDELAY(50);
  SET_RESET_PIN(1);
  MDELAY(50);

  data_array[0]= 0x00033902;
  data_array[1]= 0x005A5AF0;
  dsi_set_cmdq(data_array, 2, 1);

  data_array[0]= 0x00033902;
  data_array[1]= 0x005A5AF1;
  dsi_set_cmdq(data_array, 2, 1);


  data_array[0] = 0x00023700;// set return byte number
  dsi_set_cmdq(data_array, 1, 1);

  read_reg_v2(0xF2, buffer, 2);
  id_high = buffer[1];//0x08
  id_low = buffer[0];//0x02

  id = (id_high<<8) | id_low;

#if defined(BUILD_LK)
  printf("%s, S6D7AA0X62_id = 0x%d \n", id);
#else
  pr_debug("S6D7AA0X62_id = 0x%d \n", id);
#endif

  return (LCM_ID_S6D7AA0X62 == id) ? 1 : 0;
}
// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER s6d7aa0_qijing50_jdf_hd_n40_lcm_drv =
{
  .name           = "s6d7aa0_qijing50_jdf_hd_n40",
  .set_util_funcs = lcm_set_util_funcs,
  .get_params     = lcm_get_params,
  .init           = lcm_init,
  .suspend        = lcm_suspend,
  .resume         = lcm_resume,
  .compare_id    = lcm_compare_id,
};
