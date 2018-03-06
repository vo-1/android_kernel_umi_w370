/**
 * gf_common.c
 * GOODIX SPI common handle file
 */
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/completion.h>
#include <linux/gpio.h>
//#include <mach/mt_gpio.h>
#include <linux/delay.h>
#include "gf-common.h"

/****************************************************
**Setup SPI speed and transfer mode
**SPEEND = SPI_MODULE_CLOCK/(high_time+low_time)  KHZ
**eg:SPI_MODULE_CLOCK = 104000khz
**   high_time = 10   low_time = 10
**SPEEND=104000/(10+10)=5200KHZ=5.2M
*****************************************************/
void gf_spi_set_mode(struct spi_device *spi, SPI_SPEED speed, int flag)
{
	struct mt_chip_conf *mcc = &spi_conf_mt65xx;
	if(flag == 0) {
		mcc->com_mod = FIFO_TRANSFER;
	} else {
		mcc->com_mod = DMA_TRANSFER;
	}
	switch(speed)
	{
		case SPEED_500KHZ:
			mcc->high_time = 120;
			mcc->low_time = 120;
			break;
		case SPEED_1MHZ:
			mcc->high_time = 60;
			mcc->low_time = 60;
			break;
		case SPEED_2MHZ:
			mcc->high_time = 30;
			mcc->low_time = 30;
			break;
		case SPEED_3MHZ:
			mcc->high_time = 20;
			mcc->low_time = 20;
			break;
		case SPEED_4MHZ:
			mcc->high_time = 15;
			mcc->low_time = 15;
			break;

		case SPEED_6MHZ:
			mcc->high_time = 10;
			mcc->low_time = 10;
			break;
		case SPEED_8MHZ:
		    mcc->high_time = 8;
			mcc->low_time = 8;
			break;  
		case SPEED_KEEP:
		case SPEED_UNSUPPORTED:
			break;
	}
	if(spi_setup(spi) < 0){
		gf_error("gf:Failed to set spi.");
	}
}

/********************************************
**SPI setup
*********************************************/
void gf_spi_setup(gf_dev_t *gf_dev, int max_speed_hz)
{
    gf_dev->spi->mode = SPI_MODE_0; //CPOL=CPHA=0
    gf_dev->spi->max_speed_hz = max_speed_hz; 
    gf_dev->spi->bits_per_word = 8;
    gf_dev->spi->controller_data  = (void*)&spi_conf_mt65xx;
    spi_setup(gf_dev->spi);
}

/**********************************************************
 *Message format:
 *	write cmd   |  ADDR_H |ADDR_L  |  data stream  |
 *    1B         |   1B    |  1B    |  length       |
 *
 * read buffer length should be 1 + 1 + 1 + data_length
 ***********************************************************/
int gf_spi_write_bytes(gf_dev_t *gf_dev,
				u16 addr, u32 data_len, u8 *tx_buf)
{
	struct spi_message msg;
	struct spi_transfer *xfer;
	u32 package_num = (data_len + 2*GF_WDATA_OFFSET)>>MTK_SPI_ALIGN_MASK_NUM;
	u32 reminder = (data_len + 2*GF_WDATA_OFFSET) & MTK_SPI_ALIGN_MASK;
	u8 *reminder_buf = NULL;
	u8 twice = 0;
	int ret = 0;

	printk("nasri....addr = 0x%x\n",addr);
	/*set spi mode.*/
	if((data_len + GF_WDATA_OFFSET) > 32) {
		gf_spi_set_mode(gf_dev->spi, SPEED_KEEP, 1); //FIFO
	} else {
		gf_spi_set_mode(gf_dev->spi, SPEED_KEEP, 0); //FIFO
	}
	printk("nasri...%s %d\n",__func__,__LINE__);
	if((package_num > 0) && (reminder != 0)) {
		twice = 1;
		/*copy the reminder data to temporarity buffer.*/
		reminder_buf = kzalloc(reminder + GF_WDATA_OFFSET, GFP_KERNEL);
		if(reminder_buf == NULL ) {
			gf_error("gf:No memory for exter data.");
			return -ENOMEM;
		}
		memcpy(reminder_buf + GF_WDATA_OFFSET, tx_buf + 2*GF_WDATA_OFFSET+data_len - reminder, reminder);
        gf_debug(SPI_DEBUG,"gf:w-reminder:0x%x-0x%x,0x%x", reminder_buf[GF_WDATA_OFFSET],reminder_buf[GF_WDATA_OFFSET+1],
                reminder_buf[GF_WDATA_OFFSET + 2]);
		xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
	} else {
		twice = 0;
		xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
	}
	if(xfer == NULL){
		gf_error("gf:No memory for command.");
		if(reminder_buf != NULL)
			kfree(reminder_buf);
		return -ENOMEM;
	}

	//gf_debug(SPI_DEBUG,"gf:write twice = %d. data_len = %d, package_num = %d, reminder = %d\n", (int)twice, (int)data_len, (int)package_num, (int)reminder);
	/*if the length is not align with 1024. Need 2 transfer at least.*/
	spi_message_init(&msg);
	tx_buf[0] = GF_W;
	tx_buf[1] = (u8)((addr >> 8)&0xFF);
	tx_buf[2] = (u8)(addr & 0xFF);
	xfer[0].tx_buf = tx_buf;
	//xfer[0].delay_usecs = 5;
	printk("nasri....%s %d\n",__func__,__LINE__);

	if(twice == 1) {
		xfer[0].len = package_num << MTK_SPI_ALIGN_MASK_NUM;
		spi_message_add_tail(&xfer[0], &msg);
		addr += (data_len - reminder + GF_WDATA_OFFSET);
		reminder_buf[0] = GF_W;
		reminder_buf[1] = (u8)((addr >> 8)&0xFF);
		reminder_buf[2] = (u8)(addr & 0xFF);
		xfer[1].tx_buf = reminder_buf;
		xfer[1].len = reminder + 2*GF_WDATA_OFFSET;
		//xfer[1].delay_usecs = 5;
		printk("nasri....%s %d\n",__func__,__LINE__);

		spi_message_add_tail(&xfer[1], &msg);
	} else {
		xfer[0].len = data_len + GF_WDATA_OFFSET;
		spi_message_add_tail(&xfer[0], &msg);
	}
	printk("nasri....%s %d\n",__func__,__LINE__);
	ret = spi_sync(gf_dev->spi, &msg);
	if(ret == 0) {
		if(twice == 1)
			ret = msg.actual_length - 2*GF_WDATA_OFFSET;
		else
			ret = msg.actual_length - GF_WDATA_OFFSET;
	} else 	{
		gf_debug(SPI_DEBUG,"gf:write async failed. ret = %d", ret);
	}
	printk("nasri....%s %d\n",__func__,__LINE__);

	if(xfer != NULL) {
		kfree(xfer);
		xfer = NULL;
	}
	printk("nasri....%s %d\n",__func__,__LINE__);

	if(reminder_buf != NULL) {
		kfree(reminder_buf);
		reminder_buf = NULL;
	}
	printk("nasri....%s %d\n",__func__,__LINE__);

	return ret;
}

/*************************************************************
 *First message:
 *	write cmd   |  ADDR_H |ADDR_L  |
 *    1B         |   1B    |  1B    |
 *Second message:
 *	read cmd   |  data stream  |
 *    1B        |   length    |
 *
 * read buffer length should be 1 + 1 + 1 + 1 + data_length
 **************************************************************/
int gf_spi_read_bytes(gf_dev_t *gf_dev,
				u16 addr, u32 data_len, u8 *rx_buf)
{
	struct spi_message msg;
	struct spi_transfer *xfer;
	u32 package_num = (data_len + 1 + 1)>>MTK_SPI_ALIGN_MASK_NUM;
	u32 reminder = (data_len + 1 + 1) & MTK_SPI_ALIGN_MASK;
	u8 *one_more_buff = NULL;
	u8 twice = 0;
	int ret = 0;		
	one_more_buff = kzalloc(15000, GFP_KERNEL);
	if(one_more_buff == NULL ) {
		gf_error("No memory for one_more_buff.");
		return -ENOMEM;
	}
	xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
	if((package_num > 0) && (reminder != 0)) {
		twice = 1;
		printk("stone package_num is %d reminder is %d\n",package_num,reminder);
	} else {
		twice = 0;
	}
	if( xfer == NULL){
		gf_error("No memory for command.");
		if(one_more_buff != NULL)
			kfree(one_more_buff);
		return -ENOMEM;
	}
	/*set spi mode.*/
	if((data_len + GF_RDATA_OFFSET) > 32) {
		gf_spi_set_mode(gf_dev->spi, SPEED_KEEP, 1); //DMA
	} else {
		gf_spi_set_mode(gf_dev->spi, SPEED_KEEP, 0); //FIFO
	}
	spi_message_init(&msg);
    /*send GF command to device.*/
	rx_buf[0] = GF_W;
	rx_buf[1] = (u8)((addr >> 8)&0xFF);
	rx_buf[2] = (u8)(addr & 0xFF);
	xfer[0].tx_buf = rx_buf;
	xfer[0].len = 3;
	spi_message_add_tail(&xfer[0], &msg);
	spi_sync(gf_dev->spi, &msg);
	spi_message_init(&msg);

	/*if wanted to read data from GF. 
	 *Should write Read command to device
	 *before read any data from device.
	 */
	//memset(rx_buf, 0xff, data_len);
	one_more_buff[0] = GF_R;
	xfer[1].tx_buf = &one_more_buff[0];
	xfer[1].rx_buf = &one_more_buff[0];	
	//read 1 additional package to ensure no even data read
	if(twice == 1)
		xfer[1].len = ((package_num+1) << MTK_SPI_ALIGN_MASK_NUM);
	else
		xfer[1].len = data_len + 1;
	spi_message_add_tail(&xfer[1], &msg);
	ret = spi_sync(gf_dev->spi, &msg);
	if(ret == 0) {
		memcpy(rx_buf + GF_RDATA_OFFSET,one_more_buff+1,data_len);
		ret = data_len;
	}else {
        gf_error("gf: read failed. ret = %d", ret);
  }

	kfree(xfer);
	if(xfer != NULL)
		xfer = NULL;
	if(one_more_buff != NULL) {
		kfree(one_more_buff);
		one_more_buff = NULL;
	}	
	//gf_debug(SPI_DEBUG,"gf:read twice = %d, data_len = %d, package_num = %d, reminder = %d\n",(int)twice, (int)data_len, (int)package_num, (int)reminder);
	//gf_debug(SPI_DEBUG,"gf:data_len = %d, msg.actual_length = %d, ret = %d\n", (int)data_len, (int)msg.actual_length, ret);
	return ret;
}

//static int gf_spi_read_byte(gf_dev_t *gf_dev, u16 addr, u8 *value)
//{
//    int status = 0;
//    mutex_lock(&gf_dev->buf_lock);
//
//    status = gf_spi_read_bytes(gf_dev, addr, 1, gf_dev->buffer);
//    *value = gf_dev->buffer[GF_RDATA_OFFSET];
//    mutex_unlock(&gf_dev->buf_lock);
//    return status;
//}
//static int gf_spi_write_byte(gf_dev_t *gf_dev, u16 addr, u8 value)
//{
//    int status = 0;
//    mutex_lock(&gf_dev->buf_lock);
//    gf_dev->buffer[GF_WDATA_OFFSET] = value;
//    status = gf_spi_write_bytes(gf_dev, addr, 1, gf_dev->buffer);
//    mutex_unlock(&gf_dev->buf_lock);
//    return status;
//}

int gf_spi_read_word(gf_dev_t *gf_dev, u16 addr, u16 *value)
{
	int status = 0;
	u8 *buf = NULL;
	mutex_lock(&gf_dev->buf_lock);
	status = gf_spi_read_bytes(gf_dev, addr, 2, gf_dev->buffer);
	buf = gf_dev->buffer + GF_RDATA_OFFSET;
	*value = (u16)buf[0]<<8 | buf[1];
	mutex_unlock(&gf_dev->buf_lock);
	return status;
}
int gf_spi_write_word(gf_dev_t* gf_dev, u16 addr, u16 value)
{
	int status = 0;
    mutex_lock(&gf_dev->buf_lock);
    gf_dev->buffer[GF_WDATA_OFFSET] = 0x00;
    gf_dev->buffer[GF_WDATA_OFFSET+1] = 0x01;
    gf_dev->buffer[GF_WDATA_OFFSET+2] = (u8)(value>>8);
    gf_dev->buffer[GF_WDATA_OFFSET+3] = (u8)(value & 0x00ff);
    status = gf_spi_write_bytes(gf_dev, addr, 4, gf_dev->buffer);
    mutex_unlock(&gf_dev->buf_lock);

    return status;
}

void endian_exchange(int len, u8* buf)
{
    int i;
    u8 buf_tmp;
    for(i=0; i< len/2; i++)
    {   
		buf_tmp = buf[2*i+1];
		buf[2*i+1] = buf[2*i] ;
		buf[2*i] = buf_tmp;
    }
}

int gf_spi_read_data(gf_dev_t* gf_dev, u16 addr, int len, u8* value)
{
    int status;

    mutex_lock(&gf_dev->buf_lock);
    status = gf_spi_read_bytes(gf_dev, addr, len, gf_dev->buffer);
    memcpy(value, gf_dev->buffer+GF_RDATA_OFFSET, len);
	if(len>=1022)
	{
		printk("stone value[len-5]=%x,%x,%x,%x,%x",value[len-5],value[len-4],value[len-3],value[len-2],value[len-1]);
	}
    mutex_unlock(&gf_dev->buf_lock);	
	
    return status;
}

int gf_spi_read_data_bigendian(gf_dev_t* gf_dev, u16 addr,int len, u8* value)
{
    int status;

    mutex_lock(&gf_dev->buf_lock);
    status = gf_spi_read_bytes(gf_dev, addr, len, gf_dev->buffer);
    memcpy(value, gf_dev->buffer+GF_RDATA_OFFSET, len);
    mutex_unlock(&gf_dev->buf_lock);	
	
   	endian_exchange(len,value);
    return status;
}


int gf_spi_write_data(gf_dev_t* gf_dev, u16 addr, int len, u8* value)
{
    int status =0;
    unsigned short addr_len = 0;
    unsigned char* buf = NULL;

    if (len > 1024 * 10){
		pr_err("%s length is large.\n",__func__);
		return -1;
    }

    addr_len = len / 2;  

    buf = kzalloc(len + 2, GFP_KERNEL);
    if (buf == NULL){
		pr_err("%s, No memory for buffer.\n",__func__);
		return -ENOMEM;
    }

    buf[0] = (unsigned char) ((addr_len & 0xFF00) >> 8);
    buf[1] = (unsigned char) (addr_len & 0x00FF);
    memcpy(buf+2, value, len);
    endian_exchange(len,buf+2);

    mutex_lock(&gf_dev->buf_lock);
    memcpy(gf_dev->buffer+GF_WDATA_OFFSET, buf, len+2);
    kfree(buf);

    status = gf_spi_write_bytes(gf_dev, addr, len+2, gf_dev->buffer);
    mutex_unlock(&gf_dev->buf_lock);

    return status;
}

int gf_spi_send_cmd(gf_dev_t* gf_dev, unsigned char* cmd, int len)
{
	struct spi_message msg;
	struct spi_transfer *xfer;
	int ret = 0;

	gf_spi_set_mode(gf_dev->spi, SPEED_KEEP, 0); //FIFO

	xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);

	if(xfer == NULL){
		gf_error("gf:No memory for command.");
		return -ENOMEM;
	}

	//gf_debug(SPI_DEBUG,"gf:write twice = %d. data_len = %d, package_num = %d, reminder = %d\n", (int)twice, (int)data_len, (int)package_num, (int)reminder);

	spi_message_init(&msg);
    xfer->tx_buf = cmd;
	xfer->len = len;
	spi_message_add_tail(xfer, &msg);
	ret = spi_sync(gf_dev->spi, &msg);

	kfree(xfer);
	if(xfer != NULL) {
		xfer = NULL;
	}
	
	return ret;
}
