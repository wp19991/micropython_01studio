/**
 *
 * ESP-无人机固件
 * motors.c - 电机驱动程序
 *
 */

#include <stdbool.h>

// FreeRTOS 包含
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motors.h"

#include "esp_log.h"

//#define DEBUG_EP2
#define TAG "MOTORS"

static uint16_t motorsConvBitsTo16(uint16_t bits);

static uint16_t motorsConv16ToBits(uint16_t bits);

// 电机比例数组，用于存储各电机的比例
uint32_t motor_ratios[] = {0, 0, 0, 0};

void motorsPlayTone(uint16_t frequency, uint16_t duration_msec);

void motorsPlayMelody(uint16_t *notes);

void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio);

// 当前映射配置的指针数组
const MotorPerifDef **motorMap;

// 各电机对应的 GPIO 引脚常量数组
const uint32_t MOTORS[] = {MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4};

// 用于测试的音符频率数组
const uint16_t testsound[NBR_OF_MOTORS] = {A4, A5, F5, D5};

static bool isInit = false; // 是否已初始化标志
static bool isTimerInit = false; // 定时器是否已初始化标志

//---------------------------------------------
static const MotorPerifDef CONN_M1 = {
        .drvType = BRUSHED,
};

// 连接器 M2, PB11, TIM2_CH4
static const MotorPerifDef CONN_M2 = {
        .drvType = BRUSHED,
};

// 连接器 M3, PA15, TIM2_CH1
static const MotorPerifDef CONN_M3 = {
        .drvType = BRUSHED,
};

// 连接器 M4, PB9, TIM4_CH4
static const MotorPerifDef CONN_M4 = {
        .drvType = BRUSHED,
};

/**
 * 默认的刷子映射到 M1-M4 连接器。
 */
const MotorPerifDef *motorMapDefaultBrushed[NBR_OF_MOTORS] = {
        &CONN_M1,
        &CONN_M2,
        &CONN_M3,
        &CONN_M4
};
//---------------------------------------------

// 电机 PWM 通道配置数组
ledc_channel_config_t motors_channel[NBR_OF_MOTORS] = {
        {
                .channel = MOT_PWM_CH1,
                .duty = 0,
                .gpio_num = MOTOR1_GPIO,
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .timer_sel = LEDC_TIMER_0
        },
        {
                .channel = MOT_PWM_CH2,
                .duty = 0,
                .gpio_num = MOTOR2_GPIO,
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .timer_sel = LEDC_TIMER_0
        },
        {
                .channel = MOT_PWM_CH3,
                .duty = 0,
                .gpio_num = MOTOR3_GPIO,
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .timer_sel = LEDC_TIMER_0
        },
        {
                .channel = MOT_PWM_CH4,
                .duty = 0,
                .gpio_num = MOTOR4_GPIO,
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .timer_sel = LEDC_TIMER_0
        },
};
/* 私有函数 */

// 将 PWM 位值转换为 16 位值
static uint16_t motorsConvBitsTo16(uint16_t bits) {
    return ((bits) << (16 - MOTORS_PWM_BITS));
}

// 将 16 位值转换为 PWM 位值
static uint16_t motorsConv16ToBits(uint16_t bits) {
    return ((bits) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}

// 初始化 PWM 定时器
bool pwm_timmer_init() {
    if (isTimerInit) {
        // 第一个初始化将会配置它
        return TRUE;
    }

    /*
     * 准备和设置将由 MOTORS 控制器使用的计时器的配置
     */
    ledc_timer_config_t ledc_timer = {
            .duty_resolution = MOTORS_PWM_BITS, // PWM 占空比的分辨率
            .freq_hz = 15000,                  // PWM 信号的频率
            .speed_mode = LEDC_LOW_SPEED_MODE, // 计时器模式
            .timer_num = LEDC_TIMER_0,         // 计时器索引
            // .clk_cfg = LEDC_AUTO_CLK,              // 自动选择源时钟
    };

    // 设置计时器0的配置，用于高速通道
    if (ledc_timer_config(&ledc_timer) == ESP_OK) {
        isTimerInit = TRUE;
        return TRUE;
    }

    return FALSE;
}

/* 公共函数 */

// 初始化电机。将所有电机比例设置为 0%
void motorsInit(void) {
    int i;

    if (isInit) {
        // 第一个初始化将会配置它
        return;
    }

    //motorMap = motorMapSelect;

    if (pwm_timmer_init() != TRUE) {
        return;
    }

    for (i = 0; i < NBR_OF_MOTORS; i++) {
        ledc_channel_config(&motors_channel[i]);
    }

    isInit = true;
}

// 停止电机并反初始化
void motorsDeInit(void) {
    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        ledc_stop(motors_channel[i].speed_mode, motors_channel[i].channel, 0);
    }
    isInit = false;
}

// 测试电机是否正常工作
bool motorsTest(void) {
    int i;

    for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++) {
        //if (motorMap[i]->drvType == BRUSHED)
        {
#ifdef ACTIVATE_STARTUP_SOUND
            motorsBeep(MOTORS[i], true, testsound[i], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4) / 20);
            vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
            motorsBeep(MOTORS[i], false, 0, 0);
            vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
#else
            motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
            vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
            motorsSetRatio(MOTORS[i], 0);
            vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
#endif
        }
    }

    return isInit;
}

// 设置电机的比例（功率）
void motorsSetRatio(uint32_t id, uint16_t ithrust) {
    if (isInit) {
        uint16_t ratio;

        //ASSERT(id < NBR_OF_MOTORS);

        ratio = ithrust;

#ifdef ENABLE_THRUST_BAT_COMPENSATED

        if (motorMap[id]->drvType == BRUSHED) {
            float thrust = ((float)ithrust / 65536.0f) * 40; // 根据实际重量修改
            float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
            float supply_voltage = pmGetBatteryVoltage();
            float percentage = volts / supply_voltage;
            percentage = percentage > 1.0f ? 1.0f : percentage;
            ratio = percentage * UINT16_MAX;
            motor_ratios[id] = ratio;
        }

#endif
        ledc_set_duty(motors_channel[id].speed_mode, motors_channel[id].channel, (uint32_t) motorsConv16ToBits(ratio));
        ledc_update_duty(motors_channel[id].speed_mode, motors_channel[id].channel);
        motor_ratios[id] = ratio;
#ifdef DEBUG_EP2
        ESP_LOGI(TAG,"motors ID = %d ,ithrust_10bit = %d", id, (uint32_t)motorsConv16ToBits(ratio));
#endif
    }
}

// 获取电机的比例（功率）
int motorsGetRatio(uint32_t id) {
    int ratio;
    //ASSERT(id < NBR_OF_MOTORS);
    ratio = motorsConvBitsTo16((uint16_t) ledc_get_duty(motors_channel[id].speed_mode, motors_channel[id].channel));
    return ratio;
}

void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio) {
    uint32_t freq_hz = 15000;
    //ASSERT(id < NBR_OF_MOTORS);
    if (ratio != 0) {
        ratio = (uint16_t)(0.05 * (1 << 16));
    }

    if (enable) {
        freq_hz = frequency;
    }

    ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, freq_hz);
    ledc_set_duty(motors_channel[id].speed_mode, motors_channel[id].channel, (uint32_t) motorsConv16ToBits(ratio));
    ledc_update_duty(motors_channel[id].speed_mode, motors_channel[id].channel);
}

// 播放具有给定频率和特定持续时间（毫秒）的音调
void motorsPlayTone(uint16_t frequency, uint16_t duration_msec) {
    motorsBeep(MOTOR_M1, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    motorsBeep(MOTOR_M2, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    motorsBeep(MOTOR_M3, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    motorsBeep(MOTOR_M4, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    vTaskDelay(M2T(duration_msec));
    motorsBeep(MOTOR_M1, false, frequency, 0);
    motorsBeep(MOTOR_M2, false, frequency, 0);
    motorsBeep(MOTOR_M3, false, frequency, 0);
    motorsBeep(MOTOR_M4, false, frequency, 0);
}

// 播放音符数组的旋律
void motorsPlayMelody(uint16_t *notes) {
    int i = 0;
    uint16_t note;     // 音符的频率
    uint16_t duration; // 持续时间（毫秒）

    do {
        note = notes[i++];
        duration = notes[i++];
        motorsPlayTone(note, duration);
    } while (duration != 0);
}
