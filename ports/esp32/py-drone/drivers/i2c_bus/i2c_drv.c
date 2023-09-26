/**
 * ESP-Drone固件
 *
 * i2c_drv.c - i2c驱动实现
 *
 * @note
 * 由于某种原因，使用I2C_AcknowledgeConfig(I2C_SENSORS, ENABLE)和
 * I2C_GenerateSTART(I2C_SENSORS, ENABLE)按顺序设置CR1寄存器有时会
 * 创建一个瞬间的start->stop条件（3.9us长），通过I2C分析器发现。只有在CR1中同时设置
 * start和stop标志位的情况下才能生成这种快速的start->stop条件。因此，我尝试一次性设置CR1
 * 寄存器，即I2C_SENSORS->CR1 = (I2C_CR1_START | I2C_CR1_ACK | I2C_CR1_PE)，问题就解决了。
 * 不妨一试...
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "py/obj.h"

#include "i2c_drv.h"

// 传感器I2C总线的定义
#define I2C_DEFAULT_SENSORS_CLOCK_SPEED             400000

// EEPROM和板载I2C总线的定义，同时使用两个具有400KHz时钟的I2C总线可能会触发看门狗
#define I2C_DEFAULT_DECK_CLOCK_SPEED                100000

#include "esp_log.h"

static const char *TAG = "i2c_drv";

static bool isinit_i2cPort[2] = {0, 0};

// 总线的常量定义
static const I2cDef sensorBusDef = {
        .i2cPort            = I2C_NUM_0,
        .i2cClockSpeed      = I2C_DEFAULT_SENSORS_CLOCK_SPEED,
        .gpioSCLPin         = MICROPY_HW_SENSOR_I2C_PIN_SCL,
        .gpioSDAPin         = MICROPY_HW_SENSOR_I2C_PIN_SDA,
        .gpioPullup         = GPIO_PULLUP_DISABLE,
};

I2cDrv sensorsBus = {
        .def                = &sensorBusDef,
};

static const I2cDef deckBusDef = {
        .i2cPort            = I2C_NUM_1,
        .i2cClockSpeed      = I2C_DEFAULT_DECK_CLOCK_SPEED,
        .gpioSCLPin         = MICROPY_HW_DECK_I2C_PIN_SCL,
        .gpioSDAPin         = MICROPY_HW_DECK_I2C_PIN_SDA,
        .gpioPullup         = GPIO_PULLUP_ENABLE,
};

I2cDrv deckBus = {
        .def                = &deckBusDef,
};

// 初始化I2C总线
static void i2cdrvInitBus(I2cDrv *i2c) {
    if (isinit_i2cPort[i2c->def->i2cPort]) {
        return;
    }

    i2c_config_t conf = {0};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = i2c->def->gpioSDAPin;
    conf.sda_pullup_en = i2c->def->gpioPullup;
    conf.scl_io_num = i2c->def->gpioSCLPin;
    conf.scl_pullup_en = i2c->def->gpioPullup;
    conf.master.clk_speed = i2c->def->i2cClockSpeed;
    esp_err_t err = i2c_param_config(i2c->def->i2cPort, &conf);

    if (!err) {
        err = i2c_driver_install(i2c->def->i2cPort, conf.mode, 0, 0, 0);
    }

    ESP_LOGI(TAG, "i2c %d driver install return = %d", i2c->def->i2cPort, err);
    i2c->isBusFreeMutex = xSemaphoreCreateMutex();
    isinit_i2cPort[i2c->def->i2cPort] = true;
}

//-----------------------------------------------------------

// 初始化I2C驱动
void i2cdrvInit(I2cDrv *i2c) {
    i2cdrvInitBus(i2c);
}

// 反初始化I2C驱动
void i2cDrvDeInit(I2cDrv *i2c) {
    if (isinit_i2cPort[i2c->def->i2cPort]) {

        i2c_driver_delete(i2c->def->i2cPort);

        gpio_pad_select_gpio(i2c->def->gpioSDAPin);
        gpio_matrix_out(i2c->def->gpioSDAPin, SIG_GPIO_OUT_IDX, false, false);
        gpio_set_direction(i2c->def->gpioSDAPin, GPIO_MODE_INPUT);

        gpio_pad_select_gpio(i2c->def->gpioSCLPin);
        gpio_matrix_out(i2c->def->gpioSCLPin, SIG_GPIO_OUT_IDX, false, false);
        gpio_set_direction(i2c->def->gpioSCLPin, GPIO_MODE_INPUT);

        vSemaphoreDelete(i2c->isBusFreeMutex);
        isinit_i2cPort[i2c->def->i2cPort] = false;

    }
}
