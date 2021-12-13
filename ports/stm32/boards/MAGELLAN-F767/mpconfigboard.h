#define MICROPY_HW_BOARD_NAME       "01Studio Magellan"
#define MICROPY_HW_MCU_NAME         "STM32F767IGT6"
#define	MICROPY_HW_FLASH_FS_LABEL		"MAGELLAN"

#define MICROPY_HW_ENABLE_RTC       (1)
#define MICROPY_HW_ENABLE_RNG       (1)
#define MICROPY_HW_ENABLE_ADC       (1)
#define MICROPY_HW_ENABLE_DAC       (1)
#define MICROPY_HW_ENABLE_USB       (1)
#define MICROPY_HW_ENABLE_SDCARD    (1)
#define MICROPY_HW_HAS_SWITCH       (1)
#define MICROPY_HW_HAS_FLASH        (1)
#define	MICROPY_HW_ENABLE_SERVO		(1) 

#define MICROPY_PY_THREAD_GIL		(1)
#define MICROPY_PY_THREAD			(1)

#define MICROPY_ENABLE_SDCARD_NIRQ  (1)

// The pyboard has a 32kHz crystal for the RTC
#define MICROPY_HW_RTC_USE_LSE      	(1)
#define MICROPY_HW_RTC_USE_US       	(0)
#define MICROPY_HW_RTC_USE_CALOUT   	(1)

#define MICROPY_HW_CLK_PLLM (6)
#define MICROPY_HW_CLK_PLLN (216)
#define MICROPY_HW_CLK_PLLP (RCC_PLLP_DIV2)
#define MICROPY_HW_CLK_PLLQ (9)
#define MICROPY_HW_FLASH_LATENCY (FLASH_LATENCY_7) // 210-216 MHz needs 7 wait states

// UART1 config
#define MICROPY_HW_UART1_NAME   		"UART1"    // on RX / TX
#define MICROPY_HW_UART1_TX  				(pin_B14)
#define MICROPY_HW_UART1_RX  				(pin_B15)
// UART config
#define MICROPY_HW_UART2_NAME   		"UART2"    // on RX / TX
#define MICROPY_HW_UART2_TX         (pin_A2)
#define MICROPY_HW_UART2_RX         (pin_A3)
#define MICROPY_HW_UART_REPL        PYB_UART_3
#define MICROPY_HW_UART_REPL_BAUD   115200

#define MICROPY_HW_UART3_NAME   		"UART3"    // on RX / TX
#define MICROPY_HW_UART3_TX         (pin_B10)
#define MICROPY_HW_UART3_RX         (pin_B11)

// I2C buses
#define MICROPY_HW_I2C1_NAME 					"I2C1"
#define MICROPY_HW_I2C1_SCL 					(pin_B8)
#define MICROPY_HW_I2C1_SDA 					(pin_B9)

#define MICROPY_HW_I2C2_NAME 					"I2C2"
#define MICROPY_HW_I2C2_SCL 					(pin_H4)
#define MICROPY_HW_I2C2_SDA 					(pin_H5)

// SPI buses
// #define MICROPY_HW_SPI3_NSS         (pin_A4)
// #define MICROPY_HW_SPI3_SCK         (pin_B3)
// #define MICROPY_HW_SPI3_MISO        (pin_B4)
// #define MICROPY_HW_SPI3_MOSI        (pin_B5)

// CAN buses
#define MICROPY_HW_CAN1_NAME "CAN1"
#define MICROPY_HW_CAN1_TX    (pin_B9)
#define MICROPY_HW_CAN1_RX    (pin_B8)

// USRSW is pulled low. Pressing the button makes the input go high.
#define MICROPY_HW_USRSW_PIN        (pin_E3)
#define MICROPY_HW_USRSW_PULL       (GPIO_PULLUP)
#define MICROPY_HW_USRSW_EXTI_MODE  (GPIO_MODE_IT_FALLING)
#define MICROPY_HW_USRSW_PRESSED    (0)

// LEDs
#define MICROPY_HW_LED1             (pin_E2)
#define MICROPY_HW_LED2             (pin_B14)
#define MICROPY_HW_LED3             (pin_B15)
#define MICROPY_HW_LED4             (pin_D11)
#define MICROPY_HW_LED_ON(pin)      (mp_hal_pin_high(pin))
#define MICROPY_HW_LED_OFF(pin)     (mp_hal_pin_low(pin))

// USB config (CN13 - USB OTG FS)
#define MICROPY_HW_USB_FS              (1)
#define MICROPY_HW_USB_VBUS_DETECT_PIN (pin_A9)
#define MICROPY_HW_USB_OTG_ID_PIN      (pin_A10)

// SD card detect switch
#define MICROPY_HW_SDCARD_DETECT_PIN        (pin_D7)
#define MICROPY_HW_SDCARD_DETECT_PULL       (GPIO_NOPULL)
#define MICROPY_HW_SDCARD_DETECT_PRESENT    (GPIO_PIN_RESET)

#define MICROPY_HW_ETH_MDC          				(pin_C1)
#define MICROPY_HW_ETH_MDIO         				(pin_A2)
#define MICROPY_HW_ETH_RMII_REF_CLK 				(pin_A1)
#define MICROPY_HW_ETH_RMII_CRS_DV  				(pin_A7)
#define MICROPY_HW_ETH_RMII_RXD0    				(pin_C4)
#define MICROPY_HW_ETH_RMII_RXD1    				(pin_C5)
#define MICROPY_HW_ETH_RMII_TX_EN   				(pin_B11)
#define MICROPY_HW_ETH_RMII_TXD0    				(pin_G13)
#define MICROPY_HW_ETH_RMII_TXD1    				(pin_G14)

#if MICROPY_HW_HAS_FLASH
// QSPI Flash 
#define MICROPY_HW_ENABLE_INTERNAL_FLASH_STORAGE 			(0)
#define MICROPY_HW_SPIFLASH_ENABLE_CACHE					(1)
#define MICROPY_HW_SPIFLASH_SIZE_BITS   					(128 * 1024 * 1024)
#define MICROPY_HW_QSPIFLASH_SIZE_BITS_LOG2					(28)
#define MICROPY_HW_QSPIFLASH_CS         					(pin_B6)
#define MICROPY_HW_QSPIFLASH_SCK        					(pin_B2)
#define MICROPY_HW_QSPIFLASH_IO0        					(pin_F8)
#define MICROPY_HW_QSPIFLASH_IO1        					(pin_F9)
#define MICROPY_HW_QSPIFLASH_IO2        					(pin_F7)
#define MICROPY_HW_QSPIFLASH_IO3        					(pin_F6)

// block device config for SPI flash
extern const struct _mp_spiflash_config_t spiflash_config;
extern struct _spi_bdev_t spi_bdev;
#define MICROPY_HW_BDEV_IOCTL(op, arg) ( \
	(op) == BDEV_IOCTL_NUM_BLOCKS ? (MICROPY_HW_SPIFLASH_SIZE_BITS / 8 / FLASH_BLOCK_SIZE) : \
    (op) == BDEV_IOCTL_INIT ? spi_bdev_ioctl(&spi_bdev, (op), (uint32_t)&spiflash_config) : \
    spi_bdev_ioctl(&spi_bdev, (op), (arg)) \
)
#define MICROPY_HW_BDEV_READBLOCKS(dest, bl, n) spi_bdev_readblocks(&spi_bdev, (dest), (bl), (n))
#define MICROPY_HW_BDEV_WRITEBLOCKS(src, bl, n) spi_bdev_writeblocks(&spi_bdev, (src), (bl), (n))
#endif






