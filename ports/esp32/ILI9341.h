/**
	******************************************************************************
	* This file is part of the MicroPython project, http://micropython.org/
	* Copyright (C), 2021 -2023, 01studio Tech. Co., Ltd.http://bbs.01studio.org/
	* File Name 				 :	ili9341.h
	* Author						 :	Folktale
	* Version 					 :	v1.0
	* date							 :	2021/7/1
	* Description 			 :	
	******************************************************************************
**/

#ifndef MICROPY_INCLUDED_ESP32_ILI9341_H
#define MICROPY_INCLUDED_ESP32_ILI9341_H

#include "py/obj.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#if (MICROPY_HW_LCD32 && MICROPY_ENABLE_TFTLCD)
	
#include "modtftlcd.h"

typedef struct {
    mp_obj_base_t base;
    spi_device_handle_t spi;
		uint8_t spihost;
    uint8_t mhz;
    uint8_t miso;
    uint8_t mosi;
    uint8_t clk;
    uint8_t cs;
    uint8_t dc;
    uint8_t rst;
} ILI9341_t;
extern ILI9341_t *g_ILI9341;

extern Graphics_Display g_lcd;

extern const mp_obj_type_t ILI9341_type;

extern void lcd_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color);
extern void lcd_Full(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t *color);
extern void lcd_DrawPoint(uint16_t x,uint16_t y,uint16_t color);
extern uint16_t lcd_readPoint(uint16_t x, uint16_t y);
extern void lcd_draw_hline(uint16_t x0,uint16_t y0,uint16_t len,uint16_t color);
extern void hw_spi_deinit_internal(void);
extern void lcd_set_dir(uint8_t dir);

#endif

#endif // MICROPY_INCLUDED_ESP32_ILI9341_H
