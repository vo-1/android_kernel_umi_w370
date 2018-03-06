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

#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <string.h>
#else
#include <linux/wait.h>
#include <linux/gpio.h>
#include <linux/string.h>
#include "ddp_irq.h"
#include <mt-plat/mt_boot.h>
#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>
#endif

#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL,fmt)
#else
#define LCD_DEBUG(fmt)  printk(fmt)
#endif

/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */

#define FRAME_WIDTH  			    (720)
#define FRAME_HEIGHT 			    (1280)

#define REGFLAG_DELAY           	(0XFEF)
#define REGFLAG_END_OF_TABLE    	(0xFFF)	// END OF REGISTERS MARKER

#define R63315_LCM_ID 	 	        (0x3315)

// ---------------------------------------------------------------------------
// Local Variables
// 

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#if defined(BUILD_LK)
#define MDELAY(n)   mdelay(n)  // (lcm_util.mdelay(n*20000))
#else
extern void msleep(unsigned int msecs);
#define MDELAY(n)   msleep(n)  // (lcm_util.mdelay(n*20000))
#endif

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */
static void lcm_init_power(void)
{

}
static void lcm_resume_power(void)
{

}

static void lcm_suspend_power(void)
{

}

// 
// LCM Driver Implementations
// 

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

  // enable tearing-free
  params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
  params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

  params->dsi.mode = SYNC_EVENT_VDO_MODE;//BURST_VDO_MODE;


  // DSI
  /* Command mode setting */
  params->dsi.LANE_NUM				= LCM_THREE_LANE;//LCM_THREE_LANE;
  //The following defined the fomat for data coming from LCD engine.
  params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
  params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
  params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
  params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
  // Highly depends on LCD driver capability.

  params->dsi.intermediat_buffer_num = 0;	//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage


  // Video mode setting
  params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
  params->dsi.word_count=720*3;

  params->dsi.vertical_sync_active				= 2;
  params->dsi.vertical_backporch					= 6;
  params->dsi.vertical_frontporch					= 8;
  params->dsi.vertical_active_line = FRAME_HEIGHT;

  params->dsi.horizontal_sync_active = 10;
  params->dsi.horizontal_backporch				= 80;
  params->dsi.horizontal_frontporch				= 80;
  params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
  /***********************/
  params->dsi.PLL_CLOCK = 280;//850 1296
  params->dsi.ssc_disable = 1;

#if 0
  params->dsi.noncont_clock = TRUE;
  params->dsi.noncont_clock_period =1;
  params->dsi.esd_check_enable = 1;
  params->dsi.customization_esd_check_enable = 1;
  params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
  params->dsi.lcm_esd_check_table[0].count        = 1;
  params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
#endif
}

static void lcm_init(void)
{

  unsigned int data_array[16];

  SET_RESET_PIN(1);
  MDELAY(10);
  SET_RESET_PIN(0);
  MDELAY(10);
  SET_RESET_PIN(1);
  MDELAY(120);

  data_array[0] = 0x00022902; 						  
  data_array[1] = 0x000000b0;
  dsi_set_cmdq(data_array, 2, 1);

  data_array[0] = 0x00022902; 						  
  data_array[1] = 0x000001D6;
  dsi_set_cmdq(data_array, 2, 1);

  data_array[0] = 0x00072902;
  data_array[1] = 0x000014B3;
  data_array[2] = 0x00000000;
  dsi_set_cmdq(&data_array, 3, 1);

  data_array[0] = 0x00032902;
  data_array[1] = 0x000008B4;//4///4LANE_0C,3LANE_08
  dsi_set_cmdq(&data_array, 2, 1);

  data_array[0] = 0x00032902;
  data_array[1] = 0x00B339B6;
  dsi_set_cmdq(&data_array, 2, 1);


  data_array[0] = 0x00032902;
  data_array[1] = 0x000400BE;
  dsi_set_cmdq(&data_array, 2, 1);

  data_array[0] = 0x00232902;
  data_array[1] = 0x006644C1;
  data_array[2] = 0xFFFF905F;
  data_array[3] = 0xFFFFFF07;
  data_array[4] = 0xB9AC635D;
  data_array[5] = 0xE157DF0F;
  data_array[6] = 0xF8CBFFFF;
  data_array[7] = 0x42424242;
  data_array[8] = 0x00002200;
  data_array[9] = 0x00D10362;
  dsi_set_cmdq(&data_array, 10, 1);

  data_array[0] = 0x00082902;
  data_array[1] = 0x00F531C2;
  data_array[2] = 0x00000808;
  dsi_set_cmdq(&data_array, 3, 1);

  data_array[0] = 0x00172902;
  data_array[1] = 0x000070C4;
  data_array[2] = 0x00000000;
  data_array[3] = 0x0C040000;
  data_array[4] = 0x00000000;
  data_array[5] = 0x00000000;
  data_array[6] = 0x000C0400;
  dsi_set_cmdq(&data_array, 7, 1);

  data_array[0] = 0x00292902;
  data_array[1] = 0xF202C8C6;
  data_array[2] = 0x0000E20A;
  data_array[3] = 0x00000000;
  data_array[4] = 0x00000000;
  data_array[5] = 0x2E120000;
  data_array[6] = 0xF202C820;
  data_array[7] = 0x0000E20A;
  data_array[8] = 0x00000000;
  data_array[9] = 0x00000000;
  data_array[10] = 0x2E120000;
  data_array[11] = 0x00000020;
  dsi_set_cmdq(&data_array, 12, 1);
/*
  data_array[0] = 0x00202902;
  data_array[1] = 0x1001C7C7;
  data_array[2] = 0x3E302218;
  data_array[3] = 0x453C5848;
  data_array[4] = 0x6B655C50;
  data_array[5] = 0x18100173;
  data_array[6] = 0x483E3022;
  data_array[7] = 0x50453C58;
  data_array[8] = 0x736B655C;
  dsi_set_cmdq(&data_array, 9, 1);
*/

  data_array[0] = 0x001F2902;
  data_array[1] = 0x181001C7;
  data_array[2] = 0x483E3022;
  data_array[3] = 0x50453C58;
  data_array[4] = 0x736B655C;
  data_array[5] = 0x22181001;
  data_array[6] = 0x58483E30;
  data_array[7] = 0x5C50453C;
  data_array[8] = 0x00736B65;
  dsi_set_cmdq(&data_array, 9, 1);

  data_array[0] = 0x00142902;
  data_array[1] = 0x000000C8;
  data_array[2] = 0x00FC0000;
  data_array[3] = 0x00000000;
  data_array[4] = 0x000000FC;
  data_array[5] = 0x00FC0000;
  dsi_set_cmdq(&data_array, 6, 1);

  data_array[0] = 0x000A2902;
  data_array[1] = 0xB7E086CB;
  data_array[2] = 0x00000061;
  data_array[3] = 0x0000C000;
  dsi_set_cmdq(&data_array, 4, 1);


  data_array[0] = 0x00022902; 						  
  data_array[1] = 0x000006CC;
  dsi_set_cmdq(data_array, 2, 1);

  data_array[0] = 0x000B2902;
  data_array[1] = 0xBB8144D0;
  data_array[2] = 0x194CCD58;
  data_array[3] = 0x00000419;
  dsi_set_cmdq(&data_array, 4, 1);

  data_array[0] = 0x001A2902;
  data_array[1] = 0xBB331BD3;
  data_array[2] = 0x3333B3BB;
  data_array[3] = 0x00010133;
  data_array[4] = 0x02A0D8A0;
  data_array[5] = 0x3B333E46;
  data_array[6] = 0x3D077237;
  data_array[7] = 0x000000BF;
  dsi_set_cmdq(&data_array, 8, 1);

  data_array[0] = 0x00082902;
  data_array[1] = 0x000006D5;
  data_array[2] = 0x30013001;///VCOM
  dsi_set_cmdq(&data_array, 3, 1);

  data_array[0] = 0x00290500; //set_display_on
  dsi_set_cmdq(data_array, 1, 1);
  MDELAY(20);

  data_array[0] = 0x00110500; // Sleep Out
  dsi_set_cmdq(data_array, 1, 1);	
  MDELAY(120);

}

static void lcm_suspend(void)
{

  unsigned int data_array[16];

  data_array[0]=0x00280500; 
  dsi_set_cmdq(data_array, 1, 1);
  MDELAY(20);

  data_array[0] = 0x001A2902;
  data_array[1] = 0xBB3313D3;
  data_array[2] = 0x3333B3BB;
  data_array[3] = 0x00010133;
  data_array[4] = 0x02A0D8A0;
  data_array[5] = 0x3B333E46;
  data_array[6] = 0x3D077237;
  data_array[7] = 0x000000BF;
  dsi_set_cmdq(&data_array, 8, 1);
  MDELAY(10);

  data_array[0] = 0x00100500; 
  dsi_set_cmdq(data_array, 1, 1);
  MDELAY(100);//delay more for 3 frames time  17*3=54ms

  data_array[0] = 0x00022902; 						  
  data_array[1] = 0x000000b0;
  dsi_set_cmdq(data_array, 2, 1);

  data_array[0] = 0x00022902; 						  
  data_array[1] = 0x000001B1;
  dsi_set_cmdq(data_array, 2, 1);
  MDELAY(30);

  SET_RESET_PIN(0);
  MDELAY(20);
}


static unsigned int lcm_compare_id(void);
static void lcm_resume(void)
{
  lcm_init();
}

static unsigned int lcm_compare_id(void)
{
  unsigned int id = 0;
  unsigned char buffer[5]; 
  unsigned int array[16];   
  unsigned char id_high = 0;
  unsigned char id_low = 0;
  int i=0;


  SET_RESET_PIN(1);
  MDELAY(5);
  SET_RESET_PIN(0);
  MDELAY(20);
  SET_RESET_PIN(1);
  MDELAY(50);

  array[0] = 0x00022902;                          
  array[1] = 0x000000b0; 
  dsi_set_cmdq(array, 2, 1);

  array[0] = 0x00053700;// read id return two byte,version and id

  dsi_set_cmdq(array, 1, 1);

  read_reg_v2(0xbf, buffer, 5);

  id = buffer[2]<<8 | buffer[3]; //we only need ID

#ifdef BUILD_LK
  printf("%s, R63315-=-LCM-LK: id= x%08x,id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x,id3 = 0x%08x,id4 = 0x%08x\n", __func__, id,buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);
#else
  printk("%s, R63315-=-LCM-KERNEL: id= x%08x,id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x,id3 = 0x%08x,id4 = 0x%08x\n", __func__, id,buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);
#endif
  return (R63315_LCM_ID == id)?1:0;
}

LCM_DRIVER r63315_sharp50_wsc_hd_lcm_drv =
{
  .name = "r63315_sharp50_wsc_hd",
  .set_util_funcs = lcm_set_util_funcs,
  .get_params = lcm_get_params,
  .init = lcm_init,
  .suspend = lcm_suspend,
  .resume = lcm_resume,
  .compare_id = lcm_compare_id,
  //.init_power     = lcm_init_power,
  //.resume_power   = lcm_resume_power,
  //.suspend_power  = lcm_suspend_power,
};
