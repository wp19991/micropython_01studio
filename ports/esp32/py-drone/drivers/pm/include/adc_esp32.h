/**
 * ESP-Drone Firmware
 * adc.h - Analog Digital Conversion header file
 *
 * 这些函数共同用于配置和操作模数转换器（ADC），以及处理ADC采样数据。
 */
#ifndef ADC_H_
#define ADC_H_

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "config.h"

/******** Types ********/

// 定义结构体 AdcPair，用于存储ADC采样值
typedef struct __attribute__((packed)) {
    uint16_t vref; // 参考电压值
    uint16_t val;  // ADC采样值
} AdcPair;

// 定义结构体 AdcGroup，用于存储ADC采样值组
typedef struct __attribute__((packed)) {
    AdcPair vbat;  // 电池电压的ADC采样值组
} AdcGroup;

// 定义结构体 AdcDeciGroup，用于存储ADC采样值的十分之一倍
typedef struct {
    uint16_t vbat;    // 电池电压的ADC采样值
    uint16_t vbatVref; // 参考电压的ADC采样值
} AdcDeciGroup;

/*** Public interface ***/

/**
 * 初始化模数转换器（Analog to Digital Converter）。配置陀螺仪和参考电压通道。
 */
void adcInit(void);

/**
 * 进行ADC测试。
 * @return 如果测试成功则返回true，否则返回false。
 */
bool adcTest(void);

/**
 * @brief 将12位ADC值转换为电池电压
 * @param vbat  12位ADC值
 * @param vref  内部电压参考的12位ADC值，通常为1.2V
 * @return 以浮点数表示的电压值
 */
float adcConvertToVoltageFloat(uint16_t vbat, uint16_t vref);

/**
 * 启动DMA以开始转换ADC样本。
 */
void adcDmaStart(void);

/**
 * 停止转换ADC样本。
 */
void adcDmaStop(void);

/**
 * ADC中断处理函数
 */
void adcInterruptHandler(void);

/**
 * ADC任务函数
 */
void adcTask(void *param);

/**
 * 从指定引脚读取电压值。
 * @param pin 要读取的引脚编号
 * @return 以浮点数表示的电压值
 */
float analogReadVoltage(uint32_t pin);  // 应该在deck_analog.c中实现

#endif /* ADC_H_ */
