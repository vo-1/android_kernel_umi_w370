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
#define LCD_DEBUG(fmt)  pr_err(fmt)
#endif
/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

#define REGFLAG_DELAY             							0XFFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

#define LCM_ID			(0x91)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

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

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifndef BUILD_LK
#define GPIO_LCM_PWR_EN (GPIO15 | 0x80000000)
#endif
/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */
static void lcm_init_power(void)
{
  mt_set_gpio_mode(GPIO_LCM_PWR_EN, GPIO_MODE_00);
  mt_set_gpio_dir(GPIO_LCM_PWR_EN, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ONE);
}

static void lcm_suspend_power(void)
{
  mt_set_gpio_mode(GPIO_LCM_PWR_EN, GPIO_MODE_00);
  mt_set_gpio_dir(GPIO_LCM_PWR_EN, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ZERO);
}

static void lcm_resume_power(void)
{
  mt_set_gpio_mode(GPIO_LCM_PWR_EN, GPIO_MODE_00);
  mt_set_gpio_dir(GPIO_LCM_PWR_EN, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ONE);
}
struct LCM_setting_table {
  unsigned cmd;
  unsigned char count;
  unsigned char para_list[64];
};
static struct LCM_setting_table lcm_initialization_setting[] = {
  {0xff, 4,{0xaa,0x55,0xa5,0x80}},
  {0x6f, 2,{0x11,0x00}},
  {0xf7, 2,{0x20,0x00}},

  {0x6f, 1,{0x06}},
  {0xf7, 1,{0xa0}},
  {0x6f, 1,{0x19}},
  {0xf7, 1,{0x12}},
  {0x6f, 1,{0x02}},
  {0xf7, 1,{0x47}},
  {0x6f, 1,{0x17}},
  {0xf4, 1,{0x70}},
  {0x6f, 1,{0x01}},
  {0xf9, 1,{0x46}},

  {0xf0, 5,{0x55,0xaa,0x52,0x08,0x00}},
  {0xbd, 5,{0x01,0xa0,0x10,0x10,0x01}},
  {0xb8, 4,{0x01,0x02,0x0c,0x02}},

  {0xbb, 2,{0x11,0x11}},
  {0xbc, 2,{0x00,0x00}},
  {0xb6, 1,{0x04}},
  {0xc8, 1,{0x80}},
  {0xf0, 5,{0x55,0xaa,0x52,0x08,0x01}},
  {0xb0, 2,{0x09,0x09}},
  {0xb1, 2,{0x09,0x09}},
  {0xbc, 2,{0xdd,0x00}},
  {0xbd, 2,{0xdd,0x00}},
  {0xca, 1,{0x00}},
  {0xc0, 1,{0x0c}},
  {0xb5, 2,{0x03,0x03}},
  {0xbe, 1,{0x30}},
  {0xb3, 2,{0x19,0x19}},
  {0xb4, 2,{0x39,0x39}},
  {0xb9, 2,{0x26,0x26}},
  {0xba, 2,{0x24,0x24}},
  {0xf0, 5,{0x55,0xaa,0x52,0x08,0x02}},
  {0xee, 1,{0x01}},
  {0xb0, 16,{0x00,0x20,0x00,0x45,0x00,0x6e,0x00,0x8c,0x00,0xa2,0x00,0xc5,0x00,0xe8,0x01,0x19}},
  {0xb1, 16,{0x01,0x41,0x01,0x7c,0x01,0xab,0x01,0xfa,0x02,0x35,0x02,0x38,0x02,0x6e,0x02,0xa7}},
  {0xb2, 16,{0x02,0xcf,0x03,0x04,0x03,0x2a,0x03,0x5c,0x03,0x7f,0x03,0xa6,0x03,0xbf,0x03,0xd8}},
  {0xb3, 4,{0x03,0xf4,0x03,0xff}},
  {0xc0, 1,{0x04}},
  {0xf0, 5,{0x55,0xaa,0x52,0x08,0x06}},
  {0xb0, 2,{0x10,0x12}},
  {0xb1, 2,{0x14,0x16}},
  {0xb2, 2,{0x00,0x02}},
  {0xb3, 2,{0x31,0x31}},
  {0xb4, 2,{0x31,0x34}},
  {0xb5, 2,{0x34,0x34}},
  {0xb6, 2,{0x34,0x31}},
  {0xb7, 2,{0x31,0x31}},
  {0xb8, 2,{0x31,0x31}},
  {0xb9, 2,{0x2d,0x2e}},
  {0xba, 2,{0x2e,0x2d}},
  {0xbb, 2,{0x31,0x31}},
  {0xbc, 2,{0x31,0x31}},
  {0xbd, 2,{0x31,0x34}},
  {0xbe, 2,{0x34,0x34}},
  {0xbf, 2,{0x34,0x31}},
  {0xc0, 2,{0x31,0x31}},
  {0xc1, 2,{0x03,0x01}},
  {0xc2, 2,{0x17,0x15}},
  {0xc3, 2,{0x13,0x11}},
  {0xe5, 2,{0x31,0x31}},
  {0xc4, 2,{0x17,0x15}},
  {0xc5, 2,{0x13,0x11}},
  {0xc6, 2,{0x03,0x01}},
  {0xc7, 2,{0x31,0x31}},
  {0xc8, 2,{0x31,0x34}},
  {0xc9, 2,{0x34,0x34}},
  {0xca, 2,{0x34,0x31}},
  {0xcb, 2,{0x31,0x31}},
  {0xcc, 2,{0x31,0x31}},
  {0xcd, 2,{0x2e,0x2d}},
  {0xce, 2,{0x2d,0x2e}},
  {0xcf, 2,{0x31,0x31}},
  {0xd0, 2,{0x31,0x31}},
  {0xd1, 2,{0x31,0x34}},
  {0xd2, 2,{0x34,0x34}},
  {0xd3, 2,{0x34,0x31}},
  {0xd4, 2,{0x31,0x31}},
  {0xd5, 2,{0x00,0x02}},
  {0xd6, 2,{0x10,0x12}},
  {0xd7, 2,{0x14,0x16}},
  {0xe6, 2,{0x32,0x32}},
  {0xd8, 5,{0x00,0x00,0x00,0x00,0x00}},
  {0xd9, 5,{0x00,0x00,0x00,0x00,0x00}},
  {0xe7, 1,{0x00}},
  {0xf0, 5,{0x55,0xaa,0x52,0x08,0x05}},
  {0xed, 1,{0x30}},
  {0xb0, 2,{0x17,0x06}},
  {0xb8, 1,{0x00}},
  {0xc0, 1,{0x0d}},
  {0xc1, 1,{0x0b}},
  {0xc2, 1,{0x23}},
  {0xc3, 1,{0x40}},
  {0xc4, 1,{0x84}},
  {0xc5, 1,{0x82}},
  {0xc6, 1,{0x82}},
  {0xc7, 1,{0x80}},
  {0xc8, 2,{0x0b,0x30}},
  {0xc9, 2,{0x05,0x10}},
  {0xca, 2,{0x01,0x10}},
  {0xcb, 2,{0x01,0x10}},
  {0xd1, 5,{0x03,0x05,0x05,0x07,0x00}},
  {0xd2, 5,{0x03,0x05,0x09,0x03,0x00}},
  {0xd3, 5,{0x00,0x00,0x6a,0x07,0x10}},
  {0xd4, 5,{0x30,0x00,0x6a,0x07,0x10}},
  {0xf0, 5,{0x55,0xaa,0x52,0x08,0x03}},
  {0xb0, 2,{0x00,0x00}},
  {0xb1, 2,{0x00,0x00}},
  {0xb2, 5,{0x05,0x00,0x0a,0x00,0x00}},
  {0xb3, 5,{0x05,0x00,0x0a,0x00,0x00}},
  {0xb4, 5,{0x05,0x00,0x0a,0x00,0x00}},
  {0xb5, 5,{0x05,0x00,0x0a,0x00,0x00}},
  {0xb6, 5,{0x02,0x00,0x0a,0x00,0x00}},
  {0xb7, 5,{0x02,0x00,0x0a,0x00,0x00}},
  {0xb8, 5,{0x02,0x00,0x0a,0x00,0x00}},
  {0xb9, 5,{0x02,0x00,0x0a,0x00,0x00}},
  {0xba, 5,{0x53,0x00,0x0a,0x00,0x00}},
  {0xbb, 5,{0x53,0x00,0x0a,0x00,0x00}},
  {0xbc, 5,{0x53,0x00,0x0a,0x00,0x00}},
  {0xbd, 5,{0x53,0x00,0x0a,0x00,0x00}},
  {0xc4, 1,{0x60}},
  {0xc5, 1,{0x40}},
  {0xc6, 1,{0x64}},
  {0xc7, 1,{0x44}},
  {0x6f, 1,{0x11}},
  {0xf3, 1,{0x01}},

  {0xff, 4,{0xaa,0x55,0xa5,0x80}},
  {0x6f, 1,{0x07}},    // 3lane
  {0xf7, 1,{0x50}},

  {0x11, 1 , {0x00}},
  {REGFLAG_DELAY, 120, {}},

  {0x29, 1 , {0x00}},
  {REGFLAG_DELAY, 20, {}},
  {REGFLAG_END_OF_TABLE, 0x00, {}}

};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {

  // Display off sequence
  {0x28, 1, {0x00}},
  {REGFLAG_DELAY, 10, {}},
  // Sleep Mode On
  {0x10, 1, {0x00}},
  {REGFLAG_DELAY, 120, {}},
  {REGFLAG_END_OF_TABLE, 0x00, {}}
};
/*static struct LCM_setting_table lcm_deep_sleep_mode_out_setting[] = {
// Display off sequence
{0x11, 1, {0x00}},
{REGFLAG_DELAY, 120, {}},
// Sleep Mode On
{0x29, 1, {0x00}},
{REGFLAG_DELAY, 20, {}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};*/

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
  unsigned int i;

  for(i = 0; i < count; i++) {

    unsigned int cmd;
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
/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
  memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS * params)
{
  memset(params, 0, sizeof(LCM_PARAMS));
  params->type = LCM_TYPE_DSI;
  params->width = FRAME_WIDTH;
  params->height = FRAME_HEIGHT;

  // enable tearing-free
  params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
  params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
  params->dsi.mode = CMD_MODE;
#else
  //params->dsi.mode   = SYNC_EVENT_VDO_MODE;
  params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif

  // DSI
  /* Command mode setting */
  params->dsi.LANE_NUM = LCM_THREE_LANE;
  //The following defined the fomat for data coming from LCD engine.
  params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
  params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
  params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
  params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

  params->dsi.intermediat_buffer_num = 0;	//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

  params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
  params->dsi.word_count = 720 * 3;

  params->dsi.vertical_sync_active				= 0x3;// 3    3
  params->dsi.vertical_backporch					= 0x0E;// 20   14
  params->dsi.vertical_frontporch					= 0x10; // 1  12 16
  params->dsi.vertical_active_line				= FRAME_HEIGHT; 

  params->dsi.horizontal_sync_active				= 0x04;// 50  2
  params->dsi.horizontal_backporch				= 0x28 ;//40
  params->dsi.horizontal_frontporch				= 0x1e ;//30
  params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

  // Bit rate calculation
  //1 Every lane speed
  params->dsi.PLL_CLOCK =280;
  params->dsi.ssc_disable = 1;  // ssc disable control (1: disable, 0: enable, default: 0)
  params->dsi.cont_clock = 1;
  params->dsi.esd_check_enable = 1;  
  params->dsi.customization_esd_check_enable      = 1;
  params->dsi.noncont_clock = 1;
  params->dsi.noncont_clock_period = 2;
  params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
  params->dsi.lcm_esd_check_table[0].count        = 1;  
  params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
}

static void lcm_init(void)
{
  SET_RESET_PIN(1);
  MDELAY(1);
  SET_RESET_PIN(0);
  MDELAY(50);
  SET_RESET_PIN(1);
  MDELAY(120);
  push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}
static unsigned int lcm_compare_id(void);

static void lcm_suspend(void)
{
  push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
  SET_RESET_PIN(0);
  MDELAY(10);

}

static void lcm_resume(void)
{
  lcm_init();
}

static unsigned int lcm_compare_id(void)
{
  unsigned int id=0;
  unsigned char buffer[3];
  unsigned int array[16];
  unsigned int data_array[16];
  SET_RESET_PIN(1);
  MDELAY(10);
  SET_RESET_PIN(0);
  MDELAY(50);

  SET_RESET_PIN(1);
  MDELAY(120);

  data_array[0] = 0x00063902;
  data_array[1] = 0x52AA55F0;
  data_array[2] = 0x00000108;
  dsi_set_cmdq(data_array, 3, 1);

  array[0] = 0x00033700;// read id return two byte,version and id
  dsi_set_cmdq(array, 1, 1);

  read_reg_v2(0xC5, buffer, 3);
  id = buffer[1]; //we only need ID
#ifdef BUILD_LK
  printf("%s, LK nt35521 debug: nt35521 id = 0x%08x buffer[0]=0x%08x,buffer[1]=0x%08x,buffer[2]=0x%08x\n", __func__, id,buffer[0],buffer[1],buffer[2]);
#else
  printk("%s, LK nt35521 debug: nt35521 id = 0x%08x buffer[0]=0x%08x,buffer[1]=0x%08x,buffer[2]=0x%08x\n", __func__, id,buffer[0],buffer[1],buffer[2]);
#endif

  // if(id == LCM_ID)
  if(buffer[0]==0x55 && buffer[1]==0x21)
    return 1;
  else
    return 0;
}

LCM_DRIVER nt35521_boe50_huaxian_hd_lcm_drv = {
  .name = "nt35521_boe50_huaxian_hd",
  .set_util_funcs = lcm_set_util_funcs,
  .get_params = lcm_get_params,
  .init = lcm_init,
  .suspend = lcm_suspend,
  .resume = lcm_resume,
  .compare_id    = lcm_compare_id,
  .init_power = lcm_init_power,
  .resume_power = lcm_resume_power,
  .suspend_power = lcm_suspend_power,
};
