#define MICROPY_HW_BOARD_NAME               "ESP32S2 module"
#define MICROPY_HW_MCU_NAME                 "ESP32S2"

#define MICROPY_PY_BLUETOOTH                (0)
#define MICROPY_HW_ENABLE_SDCARD            (0)

#define MICROPY_PY_PICLIB					(1)

#define MICROPY_ENABLE_TFTLCD			(1)
#define MICROPY_HW_LCD32					(1)	

#define LCD_PIN_DC    	35
#define LCD_PIN_RST    	36
#define LCD_PIN_CS    	37
#define LCD_PIN_CLK    	38
#define LCD_PIN_MISO    39
#define LCD_PIN_MOSI    40

#define	MICROPY_ENABLE_TOUCH			(1)
#define	MICROPY_HW_XPT2046				(1)

#define XPT_PIN_IRQ    	33
#define XPT_PIN_CS    	34

#define MICROPY_ENABLE_GUI				(1)
#define MICROPY_GUI_BUTTON				(1)
#define GUI_BTN_NUM_MAX 					(50)

