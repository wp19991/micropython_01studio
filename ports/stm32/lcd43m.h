/**
  ******************************************************************************
  * @file    __TFTLCD_H.h
  * @author  
  * @brief   
  ******************************************************************************
  
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD43M_H
#define __LCD43M_H

#ifdef __cplusplus
 extern "C" {
#endif 
#include "py/obj.h"

extern const mp_obj_type_t tftlcd_lcd43m_type;

/* Includes ------------------------------------------------------------------*/  

//LCD分辨率设置
#define SSD_HOR_RESOLUTION		800		//LCD水平分辨率
#define SSD_VER_RESOLUTION		480		//LCD垂直分辨率
//LCD驱动参数设置
#define SSD_HOR_PULSE_WIDTH		1		//水平脉宽
#define SSD_HOR_BACK_PORCH		46		//水平前廊
#define SSD_HOR_FRONT_PORCH		210		//水平后廊

#define SSD_VER_PULSE_WIDTH		1		//垂直脉宽
#define SSD_VER_BACK_PORCH		23		//垂直前廊
#define SSD_VER_FRONT_PORCH		22		//垂直前廊
//如下几个参数，自动计算
#define SSD_HT	(SSD_HOR_RESOLUTION+SSD_HOR_BACK_PORCH+SSD_HOR_FRONT_PORCH)
#define SSD_HPS	(SSD_HOR_BACK_PORCH)
#define SSD_VT 	(SSD_VER_RESOLUTION+SSD_VER_BACK_PORCH+SSD_VER_FRONT_PORCH)
#define SSD_VPS (SSD_VER_BACK_PORCH)


 //画笔颜色
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	 0x001F  
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40 //棕色
#define BRRED 			 0XFC07 //棕红色
#define GRAY  			 0X8430 //灰色


//LCD重要参数集
typedef struct  
{		 	 
	uint16_t width;			//LCD 宽度
	uint16_t height;			//LCD 高度
	uint16_t id;				//LCD ID
	uint8_t  dir;			//横屏还是竖屏控制：1，竖屏；0，横屏。	
	uint16_t wramcmd;		//开始写gram指令
	uint16_t setxcmd;		//设置x坐标指令
	uint16_t setycmd;		//设置y坐标指令 
	uint16_t backcolor; //默认字体背景色
	uint16_t clercolor; 	//清屏颜色
}_lcd_dev;


typedef struct
{
	uint8_t scan;
	uint8_t gray;
	uint16_t w;
	uint16_t h;
	uint8_t is565;
	uint8_t rgb;
}__attribute__((packed)) IMAGE2LCD; 

//LCD参数
extern _lcd_dev lcddev;	//管理LCD重要参数

#define LCD43M_REG	(*(volatile uint16_t *)(0x6C000000))
#define LCD43M_RAM	(*(volatile uint16_t *)(0x6C000002))

//扫描方向定义
#define L2R_U2D  0 		//从左到右,从上到下
#define L2R_D2U  1 		//从左到右,从下到上
#define R2L_U2D  2 		//从右到左,从上到下
#define R2L_D2U  3 		//从右到左,从下到上

#define U2D_L2R  4 		//从上到下,从左到右
#define U2D_R2L  5 		//从上到下,从右到左
#define D2U_L2R  6 		//从下到上,从左到右
#define D2U_R2L  7		//从下到上,从右到左	 

#define DFT_SCAN_DIR  L2R_U2D  //默认的扫描方向


extern void lcd43m_init();
extern void LCD_Display_Dir(uint8_t dir);
extern void LCD_DisplayStr(uint16_t x,uint16_t y,uint16_t width,uint16_t height,uint8_t size,char *p,uint16_t color);
extern void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);

extern void LCD_Clear(uint16_t color);
extern void LCD_Set_Window(uint16_t sx,uint16_t sy,uint16_t width,uint16_t height);

extern void LCD_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color);
extern void LCD_Color_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t *color);
extern uint16_t LCD_ReadPoint(uint16_t x , uint16_t y);
extern void LCD_Fast_DrawPoint(uint16_t x,uint16_t y,uint16_t color);
extern void LCD43M_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

extern uint16_t rgb888to565(uint8_t r_color, uint8_t g_color , uint8_t b_color);

/**
  * @}
  */    
#ifdef __cplusplus
}
#endif

#endif /* __TFTLCD_H */

