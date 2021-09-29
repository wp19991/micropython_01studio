

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


extern 	bool gt1151_is_init;

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



