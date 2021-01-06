

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TP_TOUCH_H
#define __TP_TOUCH_H

#ifdef __cplusplus
 extern "C" {
#endif 


#if (MICROPY_HW_GT1151 && MICROPY_HW_LCD43M)

#define TP_SDA_PIN                    GPIO_PIN_11
#define TP_SDA_GPIO_PORT              GPIOF

#define TP_SCL_PIN                    GPIO_PIN_0
#define TP_SCL_GPIO_PORT              GPIOB

#define TP_SDA_IN()  {TP_SDA_GPIO_PORT->MODER&=~(3<<(2*11));TP_SDA_GPIO_PORT->MODER|=0<<(2*11);}	//INT MODE
#define TP_SDA_OUT() {TP_SDA_GPIO_PORT->MODER&=~(3<<(2*11));TP_SDA_GPIO_PORT->MODER|=1<<(2*11);} 	//OUT MODE

#define TP_IIC_SCL(Vlue) ((Vlue) ? (TP_SCL_GPIO_PORT->BSRR = TP_SCL_PIN) : (TP_SCL_GPIO_PORT->BSRR = (uint32_t)TP_SCL_PIN << 16U))
#define TP_IIC_SDA(Vlue) ((Vlue) ? (TP_SDA_GPIO_PORT->BSRR = TP_SDA_PIN) : (TP_SDA_GPIO_PORT->BSRR = (uint32_t)TP_SDA_PIN << 16U))

#define TP_SDA_READ (TP_SDA_GPIO_PORT->IDR & TP_SDA_PIN)


#define TP_PRES_DOWN 	0x80  //触屏被按下	  
#define TP_CATH_UP 		0x40  //按下弹起 
#define TP_PRES_MOVE	0x20  //有按键移动 
#define TP_INACTIVE  	0X00 //没有动作


#define CT_MAX_TOUCH  5    //电容屏支持的点数,固定为5点

//触摸屏控制器
typedef struct 
{
volatile	uint16_t x[CT_MAX_TOUCH];
volatile	uint16_t y[CT_MAX_TOUCH];	

volatile	uint8_t  sta;					//笔的状态 
								//b7:按下1/松开0; 
	                    //b6:0,没有按键按下;1,有按键按下. 
								//b5:保留
								//b4~b0:电容触摸屏按下的点数(0,表示未按下,1表示按下)
uint8_t id[4];				// ID
}TP_DEV;

extern TP_DEV tp_dev;
extern 	bool is_init;

extern const mp_obj_type_t touch_gt1151_type;
extern void gtxx_read_point(void);

#endif

/**
  * @}
  */    
#ifdef __cplusplus
}
#endif

#endif /* __TP_TOUCH_H */



