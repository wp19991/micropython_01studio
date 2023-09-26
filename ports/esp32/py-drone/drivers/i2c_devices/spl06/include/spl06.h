/**
 * ESP-Drone固件
 */
#ifndef SPL06_H
#define SPL06_H

#include <stdbool.h>
#include "i2cdev.h"

// SPL06传感器的I2C地址和默认芯片ID
#define SPL06_I2C_ADDR                     (0x76)
#define SPL06_DEFAULT_CHIP_ID              (0x10)

// 寄存器地址定义
#define SPL06_PRESSURE_MSB_REG             (0x00)  /* 气压MSB寄存器 */
#define SPL06_PRESSURE_LSB_REG             (0x01)  /* 气压LSB寄存器 */
#define SPL06_PRESSURE_XLSB_REG            (0x02)  /* 气压XLSB寄存器 */
#define SPL06_TEMPERATURE_MSB_REG          (0x03)  /* 温度MSB寄存器 */
#define SPL06_TEMPERATURE_LSB_REG          (0x04)  /* 温度LSB寄存器 */
#define SPL06_TEMPERATURE_XLSB_REG         (0x05)  /* 温度XLSB寄存器 */
#define SPL06_PRESSURE_CFG_REG             (0x06)  /* 气压配置寄存器 */
#define SPL06_TEMPERATURE_CFG_REG          (0x07)  /* 温度配置寄存器 */
#define SPL06_MODE_CFG_REG                 (0x08)  /* 模式和状态配置寄存器 */
#define SPL06_INT_FIFO_CFG_REG             (0x09)  /* 中断和FIFO配置寄存器 */
#define SPL06_INT_STATUS_REG               (0x0A)  /* 中断状态寄存器 */
#define SPL06_FIFO_STATUS_REG              (0x0B)  /* FIFO状态寄存器 */
#define SPL06_RST_REG                      (0x0C)  /* 软重置寄存器 */
#define SPL06_CHIP_ID                      (0x0D)  /* 芯片ID寄存器 */
#define SPL06_COEFFICIENT_CALIB_REG        (0x10)  /* 系数校准寄存器 */

// 校准系数的长度和数据帧大小
#define SPL06_CALIB_COEFFICIENT_LENGTH     (18)
#define SPL06_DATA_FRAME_SIZE              (6)

// SPL06传感器的工作模式
#define SPL06_CONTINUOUS_MODE              (0x07)

// 温度传感器类型
#define TEMPERATURE_INTERNAL_SENSOR        (0)
#define TEMPERATURE_EXTERNAL_SENSOR        (1)

// 气压测量速率和过采样率
#define SPL06_MWASURE_1                    (0x00)
#define SPL06_MWASURE_2                    (0x01)
#define SPL06_MWASURE_4                    (0x02)
#define SPL06_MWASURE_8                    (0x03)
#define SPL06_MWASURE_16                   (0x04)
#define SPL06_MWASURE_32                   (0x05)
#define SPL06_MWASURE_64                   (0x06)
#define SPL06_MWASURE_128                  (0x07)

#define SPL06_OVERSAMP_1                   (0x00)
#define SPL06_OVERSAMP_2                   (0x01)
#define SPL06_OVERSAMP_4                   (0x02)
#define SPL06_OVERSAMP_8                   (0x03)
#define SPL06_OVERSAMP_16                  (0x04)
#define SPL06_OVERSAMP_32                  (0x05)
#define SPL06_OVERSAMP_64                  (0x06)
#define SPL06_OVERSAMP_128                 (0x07)

// 函数声明
float spl0601_get_temperature(int32_t rawTemperature);

float spl0601_get_pressure(int32_t rawPressure, int32_t rawTemperature);

bool SPL06Init(I2C_Dev *i2cPort);

bool SPL06GetData(float *pressure, float *temperature, float *asl);

void pressureFilter(float *in, float *out); /* 限幅平均滤波法 */
float SPL06PressureToAltitude(float pressure /*, float* groundPressure, float* groundTemp*/);

void SPL06DeInit(void);

#endif // SPL06_H
