/**
 * ESP-Drone firmware
 *
 * pm_esplane.h - 电源管理驱动和函数。
 *
 * 这些函数和宏定义用于管理系统的电源，包括电池状态、充电状态、电压测量、电流测量等功能。
 * 函数之间的调用关系在代码中是线性的，没有函数相互调用。
 * 函数主要用于配置和监测电源管理功能，以确保系统电源的稳定和可靠。
 */

#ifndef PM_H_
#define PM_H_

#include "driver/adc.h"

// 默认的临界低电压和超时时间
#ifndef CRITICAL_LOW_VOLTAGE
#define PM_BAT_CRITICAL_LOW_VOLTAGE   3.0f
#else
#define PM_BAT_CRITICAL_LOW_VOLTAGE   CRITICAL_LOW_VOLTAGE
#endif
#ifndef CRITICAL_LOW_TIMEOUT
#define PM_BAT_CRITICAL_LOW_TIMEOUT   M2T(1000 * 5) // 默认5秒
#else
#define PM_BAT_CRITICAL_LOW_TIMEOUT   CRITICAL_LOW_TIMEOUT
#endif

// 默认的低电压和超时时间
#ifndef LOW_VOLTAGE
#define PM_BAT_LOW_VOLTAGE   3.2f
#else
#define PM_BAT_LOW_VOLTAGE   LOW_VOLTAGE
#endif
#ifndef LOW_TIMEOUT
#define PM_BAT_LOW_TIMEOUT   M2T(1000 * 5) // 默认5秒
#else
#define PM_BAT_LOW_TIMEOUT   LOW_TIMEOUT
#endif

// 默认的系统关机超时时间
#ifndef SYSTEM_SHUTDOWN_TIMEOUT
#define PM_SYSTEM_SHUTDOWN_TIMEOUT    M2T(1000 * 60 * 5) // 默认5分钟
#else
#define PM_SYSTEM_SHUTDOWN_TIMEOUT    M2T(1000 * 60 * SYSTEM_SHUTDOWN_TIMEOUT)
#endif

#define PM_BAT_DIVIDER                3.0f
#define PM_BAT_ADC_FOR_3_VOLT         (int32_t)(((3.0f / PM_BAT_DIVIDER) / 2.8f) * 4096)
#define PM_BAT_ADC_FOR_1p2_VOLT       (int32_t)(((1.2f / PM_BAT_DIVIDER) / 2.8f) * 4096)

#define PM_BAT_IIR_SHIFT     8
/**
 * 设置 PM_BAT_WANTED_LPF_CUTOFF_HZ 为所需的截止频率（单位：Hz）。
 */
#define PM_BAT_WANTED_LPF_CUTOFF_HZ   1

/**
 * 衰减应在1到256之间。
 *
 * f0 = fs / 2*pi*attenuation.
 * attenuation = fs / 2*pi*f0
 */
#define PM_BAT_IIR_LPF_ATTENUATION (int)(ADC_SAMPLING_FREQ / (int)(2 * 3.1415f * PM_BAT_WANTED_LPF_CUTOFF_HZ))
#define PM_BAT_IIR_LPF_ATT_FACTOR  (int)((1<<PM_BAT_IIR_SHIFT) / PM_BAT_IIR_LPF_ATTENUATION)

// ADC1引脚编号
#define PM_ADC1_PIN 2

typedef enum
{
    battery,      // 电池状态
    charging,     // 充电中
    charged,      // 已充满
    lowPower,     // 低电量
    shutDown,     // 关机
} PMStates;

typedef enum
{
    charge100mA,  // 充电电流100mA
    charge300mA,  // 充电电流300mA
    charge500mA,  // 充电电流500mA
    chargeMax,    // 最大充电电流
} PMChargeStates;

typedef enum
{
    USBNone,         // 无USB连接
    USB500mA,        // USB电流为500mA
    USBWallAdapter,  // 使用USB壁挂适配器
} PMUSBPower;

// 初始化电源管理
void pmInit(void);

// 测试电源管理是否已初始化
bool pmTest(void);

/**
 * 电源管理任务
 */
void pmTask(void *param);

// 设置充电状态
void pmSetChargeState(PMChargeStates chgState);
// void pmSyslinkUpdate(SyslinkPacket *slp);

/**
 * 返回电池电压，以浮点数表示（单位：V）
 */
float pmGetBatteryVoltage(void);

/**
 * 返回最小电池电压，以浮点数表示（单位：V）
 */
float pmGetBatteryVoltageMin(void);

/**
 * 返回最大电池电压，以浮点数表示（单位：V）
 */
float pmGetBatteryVoltageMax(void);

/**
 * 更新和计算电池值。
 * 每次获取新的ADC采样值后都应调用此函数。
 */
//void pmBatteryUpdate(AdcGroup* adcValues);

/**
 * 如果电池电量低并持续一段时间，则返回true。
 */
bool pmIsBatteryLow(void);

/**
 * 如果充电器当前连接，则返回true。
 */
bool pmIsChargerConnected(void);

/**
 * 如果电池当前正在充电，则返回true。
 */
bool pmIsCharging(void);

/**
 * 如果电池当前正在放电，则返回true。
 */
bool pmIsDischarging(void);

/**
 * 启用或禁用外部电池电压测量。
 */
void pmEnableExtBatteryVoltMeasuring(uint8_t pin, float multiplier);

/**
 * 测量外部电压。
 */
float pmMeasureExtBatteryVoltage(void);

/**
 * 启用或禁用外部电池电流测量。
 */
void pmEnableExtBatteryCurrMeasuring(uint8_t pin, float ampPerVolt);

/**
 * 测量外部电流。
 */
float pmMeasureExtBatteryCurrent(void);

// 反初始化电源管理
void pmDeInit(void);
#endif /* PM_H_ */
