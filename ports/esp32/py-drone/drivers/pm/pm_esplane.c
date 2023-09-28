/**
 * ESP-Drone Firmware
 * pm.c - 电源管理驱动和函数。
 *
 * 这些函数用于管理电源状态、电池电压、外部电池电压和电流的测量等。
 * 电源管理任务 pmTask 是一个后台任务，用于持续监测电池状态并采取相应的行动，
 * 如充电、低电量关机等。其他函数则用于初始化和获取电池状态等。
 */

#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "sensfusion6.h"
#include "pm_esplane.h"
#include "adc_esp32.h"
#include "led.h"
#include "esp_log.h"
#include "ledseq.h"

#define TAG "PM"

// 定义了电池和外部电池的相关信息，包括电压、电流等
typedef struct _PmSyslinkInfo {
    union {
        uint8_t flags;
        struct {
            uint8_t chg: 1;   // 充电标志
            uint8_t pgood: 1;   // 电源状态标志
            uint8_t unused: 6;
        };
    };
    float vBat;               // 电池电压
    float chargeCurrent;      // 充电电流
#ifdef PM_SYSTLINK_INLCUDE_TEMP
    float temp;               // 温度
#endif
}__attribute__((packed)) PmSyslinkInfo;

static float batteryVoltage;         // 电池电压
static uint16_t batteryVoltageMV;       // 电池电压（毫伏）
static float batteryVoltageMin = 6.0;  // 电池电压的最小值，默认初始值为6.0
static float batteryVoltageMax = 0.0;  // 电池电压的最大值，默认初始值为0.0

static float extBatteryVoltage;      // 外部电池电压
static uint16_t extBatteryVoltageMV;    // 外部电池电压（毫伏）
static uint16_t extBatVoltDeckPin;        // 外部电池电压测量的引脚编号
static bool isExtBatVoltDeckPinSet = false;  // 是否设置了外部电池电压测量的引脚
static float extBatVoltMultiplier;    // 外部电池电压的倍数
static float extBatteryCurrent;       // 外部电池电流
static uint16_t extBatCurrDeckPin;        // 外部电池电流测量的引脚编号
static bool isExtBatCurrDeckPinSet = false;  // 是否设置了外部电池电流测量的引脚
static float extBatCurrAmpPerVolt;    // 外部电池电流的每伏特安倍数

#ifdef PM_SYSTLINK_INLCUDE_TEMP
// nRF51内部温度
static float    temp;               // 温度
#endif

static uint32_t batteryLowTimeStamp;         // 电池电量低的时间戳
static uint32_t batteryCriticalLowTimeStamp;  // 电池电量临界低的时间戳
static bool isInit;                          // 电源管理是否已初始化的标志
static PMStates pmState;                     // 电源状态
static PmSyslinkInfo pmSyslinkInfo;           // 电源系统连接信息

static uint8_t batteryLevel;                  // 电池电量级别
static bool isLowpower;                      // 是否低电量

const static float bat671723HS25C[10] =
        {
                3.00, // 00%
                3.78, // 10%
                3.83, // 20%
                3.87, // 30%
                3.89, // 40%
                3.92, // 50%
                3.96, // 60%
                4.00, // 70%
                4.04, // 80%
                4.10  // 90%
        };

// 测试电源管理是否已初始化
bool pmTest(void) {
    return isInit;
}

/**
 * 设置电池电压以及其最小和最大值
 */
static void pmSetBatteryVoltage(float voltage) {
    batteryVoltage = voltage;
    batteryVoltageMV = (uint16_t)(voltage * 1000);
    if (batteryVoltageMax < voltage) {
        batteryVoltageMax = voltage;
    }
    if (batteryVoltageMin > voltage) {
        batteryVoltageMin = voltage;
    }
}

/**
 * 关闭系统
 */
static void pmSystemShutdown(void) {
#ifdef ACTIVATE_AUTO_SHUTDOWN
    //TODO: 实现系统关机的syslink调用
#endif
}

/**
 * 根据电池电压返回一个从0到9的数字，其中0表示完全放电，9表示90%充电。
 */
static int32_t pmBatteryChargeFromVoltage(float voltage) {
    int charge = 0;

    if (voltage < bat671723HS25C[0]) {
        return 0;
    }
    if (voltage > bat671723HS25C[9]) {
        return 9;
    }
    while (voltage > bat671723HS25C[charge]) {
        charge++;
    }

    return charge;
}

// 获取电池电压（单位：V）
float pmGetBatteryVoltage(void) {
    return batteryVoltage;
}

// 获取电池电压的最小值（单位：V）
float pmGetBatteryVoltageMin(void) {
    return batteryVoltageMin;
}

// 获取电池电压的最大值（单位：V）
float pmGetBatteryVoltageMax(void) {
    return batteryVoltageMax;
}

// 设置充电状态
void pmSetChargeState(PMChargeStates chgState) {
    // TODO: 发送带有充电状态的syslink包
}

// 更新电源状态
PMStates pmUpdateState() {
    PMStates state;
    bool isCharging = pmSyslinkInfo.chg;
    bool isPgood = pmSyslinkInfo.pgood;
    uint32_t batteryLowTime;

    batteryLowTime = xTaskGetTickCount() - batteryLowTimeStamp;

    if (isPgood && !isCharging) {
        state = charged;
    } else if (isPgood && isCharging) {
        state = charging;
    } else if (!isPgood && !isCharging && (batteryLowTime > PM_BAT_LOW_TIMEOUT)) {
        state = lowPower;
    } else {
        state = battery;
    }

    return state;
}

// 启用外部电池电流测量
void pmEnableExtBatteryCurrMeasuring(uint8_t pin, float ampPerVolt) {
    extBatCurrDeckPin = pin;
    isExtBatCurrDeckPinSet = true;
    extBatCurrAmpPerVolt = ampPerVolt;
}

// 测量外部电池电流
float pmMeasureExtBatteryCurrent(void) {
    float current;

    if (isExtBatCurrDeckPinSet) {
        current = analogReadVoltage(extBatCurrDeckPin) * extBatCurrAmpPerVolt;
    } else {
        current = 0.0;
    }

    return current;
}

// 启用外部电池电压测量
void pmEnableExtBatteryVoltMeasuring(uint8_t pin, float multiplier) {
    extBatVoltDeckPin = pin;
    isExtBatVoltDeckPinSet = true;
    extBatVoltMultiplier = multiplier;
}

// 测量外部电池电压
float pmMeasureExtBatteryVoltage(void) {
    float voltage;

    if (isExtBatVoltDeckPinSet) {
        voltage = analogReadVoltage(extBatVoltDeckPin) * extBatVoltMultiplier;
    } else {
        voltage = 0.0;
    }
    return voltage;
}

// 如果电池电量低则返回true
bool pmIsBatteryLow(void) {
    return (pmState == lowPower);
}

// 如果充电器当前连接则返回true
bool pmIsChargerConnected(void) {
    return (pmState == charging) || (pmState == charged);
}

// 如果电池当前正在充电则返回true
bool pmIsCharging(void) {
    return (pmState == charging);
}

// 如果电池正在放电则返回true
bool pmIsDischarging(void) {
    PMStates pmState;
    pmState = pmUpdateState();
    return (pmState == lowPower) || (pmState == battery);
}

// 电源管理任务
void pmTask(void *param) {
    PMStates pmStateOld = battery;
    uint32_t tickCount = 0;
    uint32_t batteryCriticalLowTime = 0;

#ifdef configUSE_APPLICATION_TASK_TAG
#if configUSE_APPLICATION_TASK_TAG == 1
    vTaskSetApplicationTaskTag(0, (void *)TASK_PM_ID_NBR);
#endif
#endif

    tickCount = xTaskGetTickCount();
    batteryLowTimeStamp = tickCount;
    batteryCriticalLowTimeStamp = tickCount;
    pmSetChargeState(charge300mA);
    // systemWaitStart();

    while (1) {
        vTaskDelay(M2T(100));
        extBatteryVoltage = pmMeasureExtBatteryVoltage();
        extBatteryVoltageMV = (uint16_t)(extBatteryVoltage * 1000);
        extBatteryCurrent = pmMeasureExtBatteryCurrent();
        pmSetBatteryVoltage(extBatteryVoltage);
        batteryLevel = pmBatteryChargeFromVoltage(pmGetBatteryVoltage()) * 10;

        tickCount = xTaskGetTickCount();

        if (pmGetBatteryVoltage() > PM_BAT_LOW_VOLTAGE) {
            batteryLowTimeStamp = tickCount;
        }
        if (pmGetBatteryVoltage() > PM_BAT_CRITICAL_LOW_VOLTAGE) {
            batteryCriticalLowTimeStamp = tickCount;
        }

        pmState = pmUpdateState();
        // ESP_LOGI(TAG,"batteryLevel=%u extBatteryVoltageMV=%u ,pmState:%d\n", batteryLevel, extBatteryVoltageMV,pmState);
        if (pmState != pmStateOld) {
            // 状态变化时的操作
            switch (pmState) {
                case charged:
                    //ledseqStop(CHG_LED,seq_charging);
                    //ledseqRun(CHG_LED, seq_charged);
                    break;
                case charging:
                    isLowpower = false;
                    if (getIsCalibrated()) {
                        ledseqRun(SYS_LED, seq_calibrated);
                    } else {
                        ledseqRun(SYS_LED, seq_alive);
                    }
                    break;
                case lowPower:
                    batteryCriticalLowTime = tickCount - batteryCriticalLowTimeStamp;
                    if (batteryCriticalLowTime > PM_BAT_CRITICAL_LOW_TIMEOUT) {
                        pmSystemShutdown();
                    }
                    ledseqStop(SYS_LED, seq_alive);
                    ledseqStop(SYS_LED, seq_calibrated);
                    ledseqRun(LOWBAT_LED, seq_lowbat);
                    break;
                case battery:
                    //ledseqStop(CHG_LED, seq_charging);
                    //ledseqRun(CHG_LED, seq_charged);
                    break;
                default:
                    //systemSetCanFly(true);
                    break;
            }
            pmStateOld = pmState;
        }
    }
}

bool getIsLowpower(void) {
    return isLowpower;
}

void pmInit(void) {
    if (isInit) {
        return;
    }
    adcInit();

    pmEnableExtBatteryVoltMeasuring(PM_ADC1_PIN, 5); // ADC1引脚固定为ADC通道

    pmSyslinkInfo.pgood = false;
    pmSyslinkInfo.chg = false;
    pmSyslinkInfo.vBat = 3.7f;
    pmSetBatteryVoltage(pmSyslinkInfo.vBat);

    // xTaskCreate(pmTask, "pmTask", PM_TASK_STACKSIZE, NULL, PM_TASK_PRI, NULL);
    isInit = true;
}

void pmDeInit(void) {
    isInit = false;
}
