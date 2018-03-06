/*
 * include/linux/spi/spidev.h
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
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

#ifndef SPI_DEBUG_H
#define SPI_DEBUG_H
//Singh_add for debug macro
//SET_GPIO_MODE
#define DEBUG_GPIO_MODE_MY	(0)
#define DEBUG_GPIO_MODE_ORIGIN	(0)  //origin should 0
//set fingerprint shutdown pin state
#define DEBUG_GPIO_SHUTDOWN_STATE_MY (0)
#define DEBUG_GPIO_SHUTDOWN_STATE_ORIGIN (0)
//chip_config
#define DEBUG_CHIP_CONFIG_MY (1)
#define DEBUG_CHIP_CONFIG_ORIGIN (0) //origin should 0
//spi_fifo/dma bufsize
#define DEBUG_FIFO_DMA_BUFSIZE_MY (0) //it's bad
#define DEBUG_FIFO_DMA_BUFSIZE_ORIGIN (1) //origin should 1
#endif /* SPIDEV_H */
