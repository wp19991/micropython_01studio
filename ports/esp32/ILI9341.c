/**
	******************************************************************************
	* This file is part of the MicroPython project, http://micropython.org/
	* Copyright (C), 2021 -2023, 01studio Tech. Co., Ltd.http://bbs.01studio.org/
	* File Name 				 :	ili9341.c
	* Author						 :	Folktale
	* Version 					 :	v1.0
	* date							 :	2021/7/1
	* Description 			 :	
	******************************************************************************
**/

#include "mpconfigboard.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "py/obj.h"
#include <math.h>
#include "py/builtin.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "py/runtime.h"
#include "py/stream.h"
#include "py/mphal.h"
#include "modmachine.h"

#include "py/binary.h"
#include "py/objarray.h"
#include "py/mperrno.h"
#include "extmod/vfs.h"
#include "py/stream.h"
#include "lib/utils/pyexec.h"

#if (MICROPY_HW_LCD32 & MICROPY_ENABLE_TFTLCD)
	
#include "ILI9341.h"

#include "global.h"

#ifdef CONFIG_IDF_TARGET_ESP32
#define SPI_DMA_CH	1
#define LCD_HOST    HSPI_HOST
#elif defined CONFIG_IDF_TARGET_ESP32S2
#define LCD_HOST    SPI2_HOST
#define SPI_DMA_CH	3

#elif defined CONFIG_IDF_TARGET_ESP32C3
#define LCD_HOST    SPI2_HOST
#define SPI_DMA_CH	3
#endif

#ifdef MICROPY_PY_PICLIB
#include "piclib.h"
#define PICLIB_PY_QSTR (1)
#else
#define PICLIB_PY_QSTR (0)
#endif

/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

typedef enum {
    LCD_TYPE_ILI = 1,
    LCD_TYPE_ST,
    LCD_TYPE_MAX,
}type_lcd_t;


ILI9341_t *g_ILI9341 = NULL;

STATIC bool is_init = 0;

Graphics_Display g_lcd;

static uint16_t get_rgb565(uint8_t r_color, uint8_t g_color , uint8_t b_color)
{
	return ( ((uint16_t)(r_color & 0xF8)<<8) | ((uint16_t)(g_color & 0xFC)<<5) | ((uint16_t)(b_color & 0xF8)>>3) );
}

STATIC void disp_spi_init(ILI9341_t *self)
{
	esp_err_t ret;
	if(!is_init)
	{
		spi_bus_config_t buscfg={
			.miso_io_num=self->miso,
			.mosi_io_num=self->mosi,
			.sclk_io_num=self->clk,
			.quadwp_io_num=-1,
			.quadhd_io_num=-1,
			.max_transfer_sz=200*1024,
		};

		spi_device_interface_config_t devcfg={
			.clock_speed_hz=self->mhz*1000*1000,
			.mode=0,                             //SPI mode 0
			.spics_io_num=self->cs,              //CS pin
			.queue_size=7,
			.flags=SPI_DEVICE_HALFDUPLEX,
		};

		//Initialize the SPI bus
		ret=spi_bus_initialize(self->spihost, &buscfg, SPI_DMA_CH);
		if (ret != ESP_OK){
			mp_raise_ValueError(MP_ERROR_TEXT("ili9341 Failed initializing SPI bus"));
		}
		//Attach the LCD to the SPI bus
		ret=spi_bus_add_device(self->spihost, &devcfg, &self->spi);
		if (ret != ESP_OK){
			mp_raise_ValueError(MP_ERROR_TEXT("ili9341 Failed adding SPI device"));
		}
		is_init = 1;
	}

}
void hw_spi_deinit_internal(void) {
	if(is_init){
		switch (spi_bus_remove_device(g_ILI9341->spi)) {
			case ESP_ERR_INVALID_ARG:
				mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("invalid configuration"));
				return;

			case ESP_ERR_INVALID_STATE:
				mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("SPI device already freed"));
				return;
		}
    switch (spi_bus_free(g_ILI9341->spihost)) {
        case ESP_ERR_INVALID_ARG:
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("invalid configuration"));
            return;

        case ESP_ERR_INVALID_STATE:
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("SPI bus already freed"));
            return;
    }
		int8_t pins[6] = {g_ILI9341->miso, g_ILI9341->mosi, g_ILI9341->clk, g_ILI9341->cs, g_ILI9341->dc, g_ILI9341->rst};

		for (int i = 0; i < 6; i++) {
			if (pins[i] != -1) {
				gpio_pad_select_gpio(pins[i]);
				gpio_matrix_out(pins[i], SIG_GPIO_OUT_IDX, false, false);
				gpio_set_direction(pins[i], GPIO_MODE_INPUT);
			}
		}
		is_init = 0;
		m_free(g_ILI9341);
	}
}

STATIC const lcd_init_cmd_t ili_init_cmds[]={
		{0xCF, {0x00, 0x83, 0X30}, 3},
		{0xED, {0x64, 0x03, 0X12, 0X81}, 4},
		{0xE8, {0x85, 0x01, 0x79}, 3},
		{0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
		{0xF7, {0x20}, 1},
		{0xEA, {0x00, 0x00}, 2},
		{0xC0, {0x26}, 1},			/*Power control*/
		{0xC1, {0x11}, 1},			/*Power control */
		{0xC5, {0x35, 0x3E}, 2},	/*VCOM control*/
		{0xC7, {0xBE}, 1},			/*VCOM control*/
		{0x36, {0x48}, 1},			/*Memory Access Control*/
		{0x3A, {0x55}, 1},			/*Pixel Format Set*/
		{0xB1, {0x00, 0x1B}, 2},
		{0xF2, {0x08}, 1},
		{0x26, {0x01}, 1},
		{0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
		{0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
		{0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
		{0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
		{0x2C, {0}, 0},
		{0xB7, {0x07}, 1},
		{0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
		{0x11, {0}, 0x80},
		{0x29, {0}, 0x80},
		{0, {0}, 0xff},
	};

//--------------------------------------------------------

STATIC void disp_spi_send(ILI9341_t *self, const uint8_t * data, uint32_t length)
{
	if (length == 0) return;           //no need to send anything

	spi_transaction_t t;
  memset(&t, 0, sizeof(t));       	//Zero out the transaction
	t.length = length * 8;              //Length is in bytes, transaction length is in bits.
	t.tx_buffer = data;               	//Data

	spi_device_transmit(self->spi, &t);
}
//---------------------------------------------------------
STATIC void disp_fill_send(ILI9341_t *self, uint16_t data, uint32_t length)
{
	if (length == 0) return;           //no need to send anything

	gpio_set_level(self->dc, 1);	 /*Data mode*/

	uint8_t *t_data = (uint8_t *)m_malloc(length*2);
	if(t_data == NULL){
		printf("fill malloc error\r\n");
	}
	for(uint32_t i=0; i < length; i++){
		t_data[2*i] = (data >> 8);
		t_data[2*i+1] = (data & 0xFF);
	}

	spi_transaction_t t;
  memset(&t, 0, sizeof(t));       	//Zero out the transaction
	t.length = length * 8 *2;              //Length is in bytes, transaction length is in bits.
	t.tx_buffer = t_data;               	//Data

	spi_device_transmit(self->spi, &t);
	
	m_free(t_data);
}

//---------------------------------------------------------

STATIC void ili9441_send_cmd(ILI9341_t *self, uint8_t cmd)
{
	gpio_set_level(self->dc, 0);	 /*Command mode*/
	disp_spi_send(self, &cmd, 1);
}
//---------------------------------------------------------

//发送读取命令
STATIC void ili9441_read_comd(ILI9341_t *self, const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
		
		gpio_set_level(self->dc, 1);	 /*Command mode*/
		
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
		//t.flags = SPI_TRANS_USE_TXDATA;
		
    ret=spi_device_polling_transmit(self->spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}
uint16_t ili9441_read_data(ILI9341_t *self)
{
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length=8*2;
	//t.flags = SPI_TRANS_USE_RXDATA;
	t.user = (void*)1;

	esp_err_t ret = spi_device_polling_transmit(self->spi, &t);
	assert( ret == ESP_OK );

	return *(uint16_t*)t.rx_data;
}
//---------------------------------------------------------
STATIC void ili9341_send_data(ILI9341_t *self, const void * data, uint32_t length)
{
	gpio_set_level(self->dc, 1);	 /*Data mode*/
	disp_spi_send(self, data, length);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++=

void  mp_init_ILI9341(void)
{
	g_ILI9341 = (ILI9341_t *)m_malloc(sizeof(ILI9341_t));
	
	ILI9341_t *self = g_ILI9341;
	
	self->mhz			= 50;
	self->spi 		= NULL;
	self->miso 		= LCD_PIN_MISO;
	self->mosi 		= LCD_PIN_MOSI;
	self->clk  		= LCD_PIN_CLK;
	self->cs   		= LCD_PIN_CS;
	self->dc   		= LCD_PIN_DC;
	self->rst  		= LCD_PIN_RST;
	self->spihost = LCD_HOST;

	disp_spi_init(self);
	gpio_pad_select_gpio(self->dc);

	//Initialize non-SPI GPIOs
	gpio_set_direction(self->dc, GPIO_MODE_OUTPUT);
	gpio_set_direction(self->rst, GPIO_MODE_OUTPUT);

	//Reset the display
	gpio_set_level(self->rst, 0);
	vTaskDelay(100 / portTICK_RATE_MS);
	gpio_set_level(self->rst, 1);
	vTaskDelay(100 / portTICK_RATE_MS);

	//Send all the commands
	uint16_t cmd = 0;
	while (ili_init_cmds[cmd].databytes!=0xff) {
		ili9441_send_cmd(self, ili_init_cmds[cmd].cmd);
		ili9341_send_data(self, ili_init_cmds[cmd].data, ili_init_cmds[cmd].databytes & 0x1F);
		if (ili_init_cmds[cmd].databytes & 0x80) {
			vTaskDelay(100 / portTICK_RATE_MS);
		}
		cmd++;
	} 
}

//设置LCD显示方向
void lcd_set_dir(uint8_t dir)
{
	uint8_t dir_data = 0;
	lcddev.dir=dir;		//竖屏
	
	switch (dir)
		{
		case 2:
		dir_data = 0x28;
		lcddev.width=320;
		lcddev.height=240;
		break;
		case 3:
		dir_data = 0x88;
		lcddev.width=240;
		lcddev.height=320;
		break;
		case 4:
		dir_data = 0xE8;
		lcddev.width=320;
		lcddev.height=240;
		break;
		default:
		dir_data = 0x48;
		lcddev.width=240;
		lcddev.height=320;
		break;
		}

	uint8_t data[4] = {0};
	
	ili9441_send_cmd(g_ILI9341, 0x2A);
	data[0] = 0x00;
	data[1] = 0x00;
	data[2] = lcddev.width >> 8;
	data[3] = lcddev.width & 0xFF;
	ili9341_send_data(g_ILI9341, data, 4);
	
	ili9441_send_cmd(g_ILI9341, 0x2B); //2A
	data[0] = 0x00;
	data[1] = 0x00;
	data[2] = lcddev.height >> 8;
	data[3] = lcddev.height & 0xFF;
	ili9341_send_data(g_ILI9341, data, 4);

	ili9441_send_cmd(g_ILI9341, 0x36);
	data[0] = dir_data;
	ili9341_send_data(g_ILI9341, data, 1);
	
	g_lcd.width = lcddev.width;
	g_lcd.height = lcddev.height;

}	 

//画点
void lcd_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
{
	uint8_t data[2];

  ILI9341_t *self = g_ILI9341;

	/*Column addresses*/
	ili9441_send_cmd(self, 0x2A);
	data[0] = (x >> 8);
	data[1] = (x & 0xFF);
	ili9341_send_data(self, data, 2);
	
		/*Page addresses*/
	ili9441_send_cmd(self, 0x2B);
	data[0] = (y >> 8);
	data[1] = (y & 0xFF);
	ili9341_send_data(self, data, 2);
	
		/*Memory write*/
	ili9441_send_cmd(self, 0x2C);
	data[0] = (color >> 8);
	data[1] = (color & 0xFF);
	ili9341_send_data(self, data, 2);
}

//读点
uint16_t lcd_readPoint(uint16_t x, uint16_t y)
{
	uint8_t data[2];
	ILI9341_t *self = g_ILI9341;
	
	if(x > lcddev.width || y > lcddev.height)	return 0;

	/*Column addresses*/
	ili9441_send_cmd(self, 0x2A);
	data[0] = (x >> 8);
	data[1] = (x & 0xFF);
	ili9341_send_data(self, data, 2);

	/*Page addresses*/
	ili9441_send_cmd(self, 0x2B);
	data[0] = (y >> 8);
	data[1] = (y & 0xFF);
	ili9341_send_data(self, data, 2);
	
	/*read comd*/
	ili9441_read_comd(self, 0x2E);
	
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length=8*2;
	//t.flags = SPI_TRANS_USE_RXDATA;
	t.user = (void*)1;

	esp_err_t ret = spi_device_polling_transmit(self->spi, &t);
	assert( ret == ESP_OK );
	
	return *(uint16_t*)t.rx_data;
}

//填充指定颜色
void lcd_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color)
{

	uint8_t data[4];
	//uint32_t size = (ex - sx + 1) * (ey - sy + 1);
uint32_t size = (ex - sx) * (ey - sy);

  ILI9341_t *self = g_ILI9341;

	/*Column addresses*/
	ili9441_send_cmd(self, 0x2A);
	data[0] = (sx >> 8) & 0xFF;
	data[1] = sx & 0xFF;
	data[2] = (ex >> 8) & 0xFF;
	data[3] = ex & 0xFF;
	ili9341_send_data(self, data, 4);

	/*Page addresses*/
	ili9441_send_cmd(self, 0x2B);
	data[0] = (sy >> 8) & 0xFF;
	data[1] = sy & 0xFF;
	data[2] = (ey >> 8) & 0xFF;
	data[3] = ey & 0xFF;
	ili9341_send_data(self, data, 4);

	/*Memory write*/
	ili9441_send_cmd(self, 0x2C);

	if(size > 320*120){
		disp_fill_send(self, color, size>>1);
		disp_fill_send(self, color, size>>1);
	}else{
		disp_fill_send(self, color, size);
	}

}

//填充指定区域块颜色
//开始位置填充多少个
void lcd_Full(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t *color)
{
	uint8_t data[4];

  ILI9341_t *self = g_ILI9341;

	/*Column addresses*/
	ili9441_send_cmd(self, 0x2A);
	data[0] = (sx >> 8) & 0xFF;
	data[1] = sx & 0xFF;
	data[2] = ((sx+ex-1) >> 8) & 0xFF;
	data[3] = (sx+ex-1) & 0xFF;
	ili9341_send_data(self, data, 4);

	/*Page addresses*/
	ili9441_send_cmd(self, 0x2B);
	data[0] = (sy >> 8) & 0xFF;
	data[1] = sy & 0xFF;
	data[2] = ((sy+ey-1) >> 8) & 0xFF;
	data[3] = (sy+ey-1) & 0xFF;
	ili9341_send_data(self, data, 4);

	/*Memory write*/
	ili9441_send_cmd(self, 0x2C);

	uint32_t size = ex * ey;

	/*Byte swapping is required*/
	uint32_t i;
	uint8_t * color_u8 = (uint8_t *) color;
	uint8_t color_tmp;

	for(i = 0; i < size * 2; i += 2) {
		color_tmp = color_u8[i + 1];
		color_u8[i + 1] = color_u8[i];
		color_u8[i] = color_tmp;
	}
	
	if(size > 320*120){
		ili9341_send_data(self, (uint8_t*)color, size);
		color += size;
		ili9341_send_data(self, (uint8_t*)color, size);
	}else{
		ili9341_send_data(self, (uint8_t*)color, size * 2);
	}
	
	
}

//填充指定区域块颜色
//开始位置填充多少个
void lcd_cam_full(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t *color)
{
	uint8_t data[4];

  ILI9341_t *self = g_ILI9341;

	/*Column addresses*/
	ili9441_send_cmd(self, 0x2A);
	data[0] = (sx >> 8) & 0xFF;
	data[1] = sx & 0xFF;
	data[2] = ((sx+ex-1) >> 8) & 0xFF;
	data[3] = (sx+ex-1) & 0xFF;
	ili9341_send_data(self, data, 4);

	/*Page addresses*/
	ili9441_send_cmd(self, 0x2B);
	data[0] = (sy >> 8) & 0xFF;
	data[1] = sy & 0xFF;
	data[2] = ((sy+ey-1) >> 8) & 0xFF;
	data[3] = (sy+ey-1) & 0xFF;
	ili9341_send_data(self, data, 4);

	/*Memory write*/
	ili9441_send_cmd(self, 0x2C);

	uint32_t size = ex * ey;
	
	if(size > 320*120){
		ili9341_send_data(self, (uint8_t*)color, size);
		color += (size>>1);
		ili9341_send_data(self, (uint8_t*)color, size);
	}else{
		ili9341_send_data(self, (uint8_t*)color, size * 2);
	}
	
}


//绘制横线函数
void lcd_draw_hline(uint16_t x0,uint16_t y0,uint16_t len,uint16_t color)
{
	if((len==0)||(x0>lcddev.width)||(y0>lcddev.height)) return;
	lcd_Fill(x0, y0,x0+len, y0, color);
}
//
void lcd_draw_vline(uint16_t x0,uint16_t y0,uint16_t len,uint16_t color)
{
	if((len==0)||(x0>lcddev.width)||(y0>lcddev.height)) return;
	lcd_Fill(x0, y0,x0, y0+len, color);
}
//------------------------------------------------
Graphics_Display g_lcd =
{
	16,
	240,
	320,
	lcd_DrawPoint,
	lcd_readPoint,
	lcd_draw_hline,
	lcd_draw_vline,
	lcd_Fill,
	lcd_Full
};
//==============================================================================================
//mpy
STATIC mp_obj_t ILI9341_drawpPixel(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
	static const mp_arg_t drawp_args[] = {
			{ MP_QSTR_x,       MP_ARG_INT, {.u_int = 0} },
			{ MP_QSTR_y,       MP_ARG_INT, {.u_int = 0} },
			{ MP_QSTR_color,    MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
	};
	mp_arg_val_t args[MP_ARRAY_SIZE(drawp_args)];
	mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(drawp_args), drawp_args, args);

	if(args[2].u_obj !=MP_OBJ_NULL) 
	{
		size_t len;
		mp_obj_t *params;
		mp_obj_get_array(args[2].u_obj, &len, &params);
		if(len == 3){
			lcd_DrawPoint(args[0].u_int,args[1].u_int ,
			get_rgb565(mp_obj_get_int(params[0]), mp_obj_get_int(params[1]), mp_obj_get_int(params[2])));
		}else{
			mp_raise_ValueError(MP_ERROR_TEXT("lcd drawPixel parameter error \nCorrect call:drawPixel(x,y,(r,g,b)"));
		}
	}
  return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ILI9341_drawpPixel_obj, 1, ILI9341_drawpPixel);
//---------------------------华丽的分割线-------------------------------------------------------------------
STATIC mp_obj_t ILI9341_drawpFull(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)  {
	static const mp_arg_t clear_args[] = {
			{ MP_QSTR_fillcolor,    MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
	};
	mp_arg_val_t args[MP_ARRAY_SIZE(clear_args)];
	mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(clear_args), clear_args, args);

	if(args[0].u_obj !=MP_OBJ_NULL) 
	{
		size_t len;
		mp_obj_t *params;
		mp_obj_get_array(args[0].u_obj, &len, &params);
		if(len == 3){
			lcddev.backcolor = get_rgb565(mp_obj_get_int(params[0]), mp_obj_get_int(params[1]), mp_obj_get_int(params[2]));
			lcd_Fill(0,0,lcddev.width, lcddev.height, lcddev.backcolor);
			lcddev.clercolor = lcddev.backcolor;
		}else{
			mp_raise_ValueError(MP_ERROR_TEXT("lcd fill parameter error \nCorrect call:fill((r,g,b))"));
		}
	}
  return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ILI9341_drawpFull_obj, 1, ILI9341_drawpFull);
//---------------------------华丽的分割线-------------------------------------------------------------------
STATIC mp_obj_t ILI9341_drawLin(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t drawL_args[] = {
				{ MP_QSTR_x0,        	MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_y0,       	MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_x1,       	MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_y1,       	MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_color,   		MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(drawL_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(drawL_args), drawL_args, args);

    if(args[4].u_obj !=MP_OBJ_NULL) 
    {
          size_t len;
          mp_obj_t *params;
          mp_obj_get_array(args[4].u_obj, &len, &params);
          if(len == 3){
						grap_drawLine(&g_lcd,args[0].u_int ,args[1].u_int,args[2].u_int,args[3].u_int ,
             get_rgb565(mp_obj_get_int(params[0]), mp_obj_get_int(params[1]), mp_obj_get_int(params[2])));
          }else{
            mp_raise_ValueError(MP_ERROR_TEXT("lcd drawL parameter error \nCorrect call:drawPixel(x,y,(r,g,b)"));
          }
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ILI9341_drawLin_obj, 4, ILI9341_drawLin);
//---------------------------华丽的分割线-------------------------------------------------------------------
STATIC mp_obj_t ILI9341_drawRect(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {

  STATIC const mp_arg_t Rect_args[] = {
		{ MP_QSTR_x,        		MP_ARG_INT, {.u_int = 0} },
    { MP_QSTR_y,        		MP_ARG_INT, {.u_int = 0} },
    { MP_QSTR_width,     		MP_ARG_INT, {.u_int = 0} },
    { MP_QSTR_height,    		MP_ARG_INT, {.u_int = 0} },
    { MP_QSTR_color,    		MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    { MP_QSTR_border,    		MP_ARG_INT, {.u_int = 1} }, 
    { MP_QSTR_fillcolor,   	MP_ARG_OBJ,{.u_obj = MP_OBJ_NULL} }, 
  };

  mp_arg_val_t args[MP_ARRAY_SIZE(Rect_args)];
  mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(Rect_args), Rect_args, args);
  
  if(args[4].u_obj !=MP_OBJ_NULL) 
  {
    size_t len;
    mp_obj_t *params;
    mp_obj_get_array(args[4].u_obj, &len, &params);
    if(len == 3){
			grap_drawRect(&g_lcd,args[0].u_int,args[1].u_int,args[2].u_int,args[3].u_int,args[5].u_int,
          get_rgb565(mp_obj_get_int(params[0]), mp_obj_get_int(params[1]), mp_obj_get_int(params[2])));
    }else{
      mp_raise_ValueError(MP_ERROR_TEXT("lcd drawRect parameter error \n"));
    }
  }
    //MP_OBJ_NULL
  if(args[6].u_obj != mp_const_none && args[6].u_obj !=MP_OBJ_NULL) 
  {
    size_t len;
    mp_obj_t *params;
    mp_obj_get_array(args[6].u_obj, &len, &params);

    if (len != 3) { // Check params len
       mp_raise_ValueError(MP_ERROR_TEXT("lcd fillcolor parameter error"));
    }
    uint16_t color=get_rgb565(mp_obj_get_int(params[0]), mp_obj_get_int(params[1]), mp_obj_get_int(params[2]));
    for(uint16_t i=0 ; i <= (args[3].u_int-(args[5].u_int*2)); i++ ){ 
     grap_drawLine(&g_lcd,args[0].u_int+args[5].u_int,args[1].u_int+args[5].u_int+i,
					args[0].u_int+args[2].u_int-args[5].u_int,args[1].u_int+args[5].u_int+i,color);
		}
     
  }
  return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ILI9341_drawRect_obj, 1, ILI9341_drawRect);
//---------------------------华丽的分割线-------------------------------------------------------------------
STATIC mp_obj_t ILI9341_drawCircle(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
  STATIC const mp_arg_t tft_allowed_args[] = {
	{ MP_QSTR_x, 			 			MP_ARG_INT, {.u_int = 0} },
	{ MP_QSTR_y, 			 			MP_ARG_INT, {.u_int = 0} },
	{ MP_QSTR_radius, 	 		MP_ARG_INT, {.u_int = 0} },
	{ MP_QSTR_color,	  		MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
	{ MP_QSTR_border, 	 		MP_ARG_INT, {.u_int = 1} }, 
	{ MP_QSTR_fillcolor,	 	MP_ARG_OBJ,	{.u_obj = MP_OBJ_NULL} }, //7

  };

  mp_arg_val_t args[MP_ARRAY_SIZE(tft_allowed_args)];
  mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(tft_allowed_args), tft_allowed_args, args);
  
  uint16_t color;
//Circlecolor
  if(args[3].u_obj !=MP_OBJ_NULL) 
  {
    size_t len;
    mp_obj_t *params;
    mp_obj_get_array(args[3].u_obj, &len, &params);
    if(len == 3){
      color = get_rgb565(mp_obj_get_int(params[0]), mp_obj_get_int(params[1]), mp_obj_get_int(params[2]));
			
			
        for(uint16_t i=0; i < args[4].u_int ;i++) {
          grap_drawColorCircle(&g_lcd,
														args[0].u_int,args[1].u_int,args[2].u_int-i,color);
        }
    }else{
      mp_raise_ValueError(MP_ERROR_TEXT("lcd color parameter error \n"));
    }
  }
//fillcolor
  if(args[5].u_obj != mp_const_none && args[5].u_obj !=MP_OBJ_NULL) 
  {
    size_t len;
    mp_obj_t *params;
    mp_obj_get_array(args[5].u_obj, &len, &params);

    if (len != 3) { // Check params len
       mp_raise_ValueError(MP_ERROR_TEXT("lcd fillcolor parameter error"));
    }
    color = get_rgb565(mp_obj_get_int(params[0]), mp_obj_get_int(params[1]), mp_obj_get_int(params[2]));

    for(uint16_t i=0 ; i <= (args[2].u_int-args[4].u_int); i++ ) {
      grap_drawColorCircle(&g_lcd,
						args[0].u_int, args[1].u_int, args[2].u_int-args[4].u_int-i, color);
    }
  }
  return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ILI9341_drawCircle_obj, 1, ILI9341_drawCircle);
//---------------------------华丽的分割线-------------------------------------------------------------------
STATIC mp_obj_t ILI9341_drawStr(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {

  STATIC const mp_arg_t tft_allowed_args[] = {
		{ MP_QSTR_text,     		MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    { MP_QSTR_x,        		MP_ARG_REQUIRED |MP_ARG_INT, {.u_int = 0} },
    { MP_QSTR_y,        		MP_ARG_REQUIRED |MP_ARG_INT, {.u_int = 0} },
    { MP_QSTR_color,    		MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    { MP_QSTR_backcolor,   	MP_ARG_OBJ,{.u_obj = MP_OBJ_NULL} }, 
    { MP_QSTR_size,      		MP_ARG_INT, {.u_int = 2} },
  };

  mp_arg_val_t args[MP_ARRAY_SIZE(tft_allowed_args)];
  mp_arg_parse_all(n_args-1, pos_args+1, kw_args, MP_ARRAY_SIZE(tft_allowed_args), tft_allowed_args, args);

  uint16_t text_size = args[5].u_int;
  uint16_t color = 0;
  //color
  if(args[3].u_obj !=MP_OBJ_NULL) 
  {
    size_t len;
    mp_obj_t *params;
    mp_obj_get_array(args[3].u_obj, &len, &params);
    if(len == 3){
      color = get_rgb565(mp_obj_get_int(params[0]), mp_obj_get_int(params[1]), mp_obj_get_int(params[2]));
    }else{
      mp_raise_ValueError(MP_ERROR_TEXT("printStr color parameter error \n"));
    }
  }

  if(args[4].u_obj != mp_const_none && args[4].u_obj !=MP_OBJ_NULL) 
  {
    size_t len;
    mp_obj_t *params;
    mp_obj_get_array(args[4].u_obj, &len, &params);
  
    if (len != 3) { 
       mp_raise_ValueError(MP_ERROR_TEXT("lcd backolor parameter error"));
    }
    lcddev.backcolor = get_rgb565(mp_obj_get_int(params[0]), mp_obj_get_int(params[1]), mp_obj_get_int(params[2])); 
  }
//
  if(args[0].u_obj !=MP_OBJ_NULL) 
  {
    mp_buffer_info_t bufinfo;
    if (mp_obj_is_int(args[0].u_obj)) {
      mp_raise_ValueError(MP_ERROR_TEXT("lcd text parameter error"));

    } else {
        mp_get_buffer_raise(args[0].u_obj, &bufinfo, MP_BUFFER_READ);
        char *str = bufinfo.buf;

        if(text_size == 1)  text_size = 16;
        else if(text_size == 2) text_size = 24;
        else if(text_size == 3) text_size = 32;
        else if(text_size == 4) text_size = 48;
        else mp_raise_ValueError(MP_ERROR_TEXT("lcd size parameter error"));
        grap_drawStr(&g_lcd, args[1].u_int, args[2].u_int, 
									text_size* bufinfo.len, text_size , text_size,str ,color, lcddev.backcolor);
    }
  }
	else{
     mp_raise_ValueError(MP_ERROR_TEXT("lcd text parameter is empty"));
  }
  return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ILI9341_drawStr_obj, 1, ILI9341_drawStr);
//---------------------------华丽的分割线-------------------------------------------------------------------
//#ifdef MICROPY_PY_PICLIB
#if MICROPY_PY_PICLIB

// cached file
mp_obj_t ILI9341_CachePicture(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {

  STATIC const mp_arg_t tft_allowed_args[] = { 
    { MP_QSTR_file,     MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
		{ MP_QSTR_path,     MP_ARG_KW_ONLY 	| MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
		{ MP_QSTR_replace, 	MP_ARG_KW_ONLY 	| MP_ARG_BOOL,{.u_bool = false} },
  };

  uint8_t arg_num = MP_ARRAY_SIZE(tft_allowed_args);
  mp_arg_val_t args[arg_num];
  mp_arg_parse_all(n_args-1, pos_args+1, kw_args, arg_num, tft_allowed_args, args);

  return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ILI9341_CachePicture_obj, 1, ILI9341_CachePicture);

//---------------------------华丽的分割线-------------------------------------------------------------------
STATIC mp_obj_t ILI9341_drawPicture(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {

  STATIC const mp_arg_t ILI9341_allowed_args[] = { 
    { MP_QSTR_x,       MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
    { MP_QSTR_y,       MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
    { MP_QSTR_file,    MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
		{ MP_QSTR_cached,  MP_ARG_KW_ONLY  | MP_ARG_BOOL, {.u_bool = true} },
  };

  uint8_t arg_num = MP_ARRAY_SIZE(ILI9341_allowed_args);
  mp_arg_val_t args[arg_num];
  mp_arg_parse_all(n_args-1, pos_args+1, kw_args, arg_num, ILI9341_allowed_args, args);

  if(args[2].u_obj !=MP_OBJ_NULL) 
  {
    mp_buffer_info_t bufinfo;
    if (mp_obj_is_int(args[2].u_obj)) {
      mp_raise_ValueError(MP_ERROR_TEXT("picture parameter error"));
    } 
		else 
		{
			mp_get_buffer_raise(args[2].u_obj, &bufinfo, MP_BUFFER_READ);

			uint8_t res=0;
			mp_obj_t tuple[2];
			const char *file_path = (const char *)bufinfo.buf;
			const char *ftype = mp_obj_str_get_str(file_type(file_path));
			
			 //---------------------------------------------------------------
				if(args[3].u_bool == true){
					
					uint8_t file_len = strlen(file_path);
					char *file_buf = (char *)m_malloc(file_len+7); 
					memset(file_buf, '\0', file_len+7);
					sprintf(file_buf,"%s%s",file_path,".cache");
					res = check_sys_file((const char *)file_buf);
					 if(res){
							grap_drawCached(&g_lcd,NULL, args[0].u_int, args[1].u_int, (const char *)file_buf); 
					 }
					 
					 m_free(file_buf);
					 if(res) return mp_const_none;
				 }
			 //---------------------------------------------------------------

			 piclib_init();
			 
			if(strncmp(ftype,"jpg",3) == 0 || strncmp(ftype,"jpeg",4) == 0)
				{
					jpg_decode(NULL,file_path, args[0].u_int,args[1].u_int ,1);
				}else if(strncmp(ftype , "bmp" , 3) == 0)
				{
					minibmp_decode(NULL ,file_path, args[0].u_int, args[1].u_int,lcddev.width, lcddev.height,0);
				}else
				{
					mp_raise_ValueError(MP_ERROR_TEXT("picture file type error"));
					return mp_const_none;
				}

			tuple[0] = mp_obj_new_int(picinfo.S_Height);
			tuple[1] = mp_obj_new_int(picinfo.S_Width);
			return mp_obj_new_tuple(2, tuple);
    }
  }
	else{
      mp_raise_ValueError(MP_ERROR_TEXT("picture parameter is empty"));
  }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ILI9341_drawPicture_obj, 1, ILI9341_drawPicture);

#endif

//---------------------------华丽的分割线-------------------------------------------------------------------
STATIC mp_obj_t ILI9341_deinit(mp_obj_t self_in) {
	hw_spi_deinit_internal();
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(ILI9341_deinit_obj, ILI9341_deinit);
//---------------------------华丽的分割线-------------------------------------------------------------------

STATIC mp_obj_t ILI9341_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {

	enum { ARG_portrait };
	static const mp_arg_t allowed_args[] = {
			{ MP_QSTR_portrait, MP_ARG_INT, {.u_int = 1} },
	};
	
	mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
	mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

	//g_ILI9341 = (ILI9341_t *)m_malloc(sizeof(ILI9341_t));

	ILI9341_t *self = m_new_obj(ILI9341_t);
	
	mp_init_ILI9341();
	
	self = g_ILI9341;
	self->base.type = type;
	
	lcd_set_dir(args[ARG_portrait].u_int);
	
	lcddev.type = 4;
	lcddev.backcolor = 0x0000;
	lcd_Fill(0,0,lcddev.width,lcddev.height,lcddev.backcolor);
	
	lcddev.clercolor = lcddev.backcolor;
	
	return MP_OBJ_FROM_PTR(self);
}
//---------------------------华丽的分割线-------------------------------------------------------------------
STATIC const mp_rom_map_elem_t ILI9341_locals_dict_table[] = {
	// instance methods
	{ MP_ROM_QSTR(MP_QSTR___del__), MP_ROM_PTR(&ILI9341_deinit_obj) },
  { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&ILI9341_deinit_obj) },
	{ MP_ROM_QSTR(MP_QSTR_fill), MP_ROM_PTR(&ILI9341_drawpFull_obj) },
	{ MP_ROM_QSTR(MP_QSTR_drawPixel), MP_ROM_PTR(&ILI9341_drawpPixel_obj) },
	{ MP_ROM_QSTR(MP_QSTR_drawLine), MP_ROM_PTR(&ILI9341_drawLin_obj) },
	{ MP_ROM_QSTR(MP_QSTR_drawRect), MP_ROM_PTR(&ILI9341_drawRect_obj) },
	{ MP_ROM_QSTR(MP_QSTR_drawCircle), MP_ROM_PTR(&ILI9341_drawCircle_obj) },
	{ MP_ROM_QSTR(MP_QSTR_printStr), MP_ROM_PTR(&ILI9341_drawStr_obj) },
	#if MICROPY_PY_PICLIB
	{ MP_ROM_QSTR(MP_QSTR_Picture), MP_ROM_PTR(&ILI9341_drawPicture_obj) },
	{ MP_ROM_QSTR(MP_QSTR_CachePicture), MP_ROM_PTR(&ILI9341_CachePicture_obj) },
	#endif
};
STATIC MP_DEFINE_CONST_DICT(ILI9341_locals_dict, ILI9341_locals_dict_table);
//---------------------------华丽的分割线-------------------------------------------------------------------
const mp_obj_type_t ILI9341_type = {
    { &mp_type_type },
    .name = MP_QSTR_ILI9341,
    .make_new = ILI9341_make_new,
    .locals_dict = (mp_obj_dict_t*)&ILI9341_locals_dict,
};

//-------------------------------------------------------------------------------------------
#endif
