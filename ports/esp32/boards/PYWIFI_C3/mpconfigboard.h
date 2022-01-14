#define MICROPY_HW_BOARD_NAME "01Studio pyWiFi-ESP32-C3"
#define MICROPY_HW_MCU_NAME "ESP32C3"

#define MICROPY_HW_I2C0_SCL (GPIO_NUM_4)
#define MICROPY_HW_I2C0_SDA (GPIO_NUM_5)

#define MICROPY_HW_SPI1_SCK (14)
#define MICROPY_HW_SPI1_MOSI (13)
#define MICROPY_HW_SPI1_MISO (12)

#define MICROPY_HW_SPI2_SCK (18)
#define MICROPY_HW_SPI2_MOSI (21)
#define MICROPY_HW_SPI2_MISO (19)

#if 0
#define MICROPY_ENABLE_TFTLCD			(1)
#define MICROPY_HW_LCD15					(1)

#define MICROPY_PY_PICLIB					(1)
//spi2
#define LCD_PIN_DC    	18
#define LCD_PIN_RST    	19
#define LCD_PIN_CS    	6
#define LCD_PIN_CLK    	0
#define LCD_PIN_MISO    7//
#define LCD_PIN_MOSI    1
#endif

#define MICROPY_HW_ENABLE_SDCARD            (0)
