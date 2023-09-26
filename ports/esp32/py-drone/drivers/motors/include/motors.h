/**
 *
 * ESP-Drone Firmware
 *
 * Motors.h - 电机驱动器头文件
 *
 */
#ifndef __MOTORS_H__
#define __MOTORS_H__

#include <stdint.h>
#include <stdbool.h>

#include "driver/ledc.h"
#include "py/mphal.h"
#include "py/obj.h"
#include "config.h"

/******** 定义 ********/

// CF2 PWM波形在328kHz下滤波效果更好。在168kHz下，NCP702稳压器会受到影响。
#define CONFIG_TARGET_PYDRONE_S2    1

#define MOTORS_PWM_BITS           LEDC_TIMER_8_BIT
#define MOTORS_PWM_PERIOD         ((1<<MOTORS_PWM_BITS) - 1)
#define MOTORS_TIM_BEEP_CLK_FREQ  4000000

// 根据电池电压来补偿推力，使其在不同电压下产生大致相同的推力。基于推力测量。

//#define ENABLE_THRUST_BAT_COMPENSATED

#define NBR_OF_MOTORS 4
// 电机ID定义
#define MOTOR_M1  0
#define MOTOR_M2  1
#define MOTOR_M3  2
#define MOTOR_M4  3

#define MOTOR1_GPIO  MICROPY_MOTOR01_PIN
#define MOTOR2_GPIO  MICROPY_MOTOR02_PIN
#define MOTOR3_GPIO  MICROPY_MOTOR03_PIN
#define MOTOR4_GPIO  MICROPY_MOTOR04_PIN

#define MOT_PWM_CH1  4      // 电机 M1 PWM 通道
#define MOT_PWM_CH2  5      // 电机 M2 PWM 通道
#define MOT_PWM_CH3  6      // 电机 M3 PWM 通道
#define MOT_PWM_CH4  7      // 电机 M4 PWM 通道

// 测试定义
#define MOTORS_TEST_RATIO         (uint16_t)(0.2*(1<<16))
#define MOTORS_TEST_ON_TIME_MS    50
#define MOTORS_TEST_DELAY_TIME_MS 150

// 声音定义
#define C4    262
#define DES4  277
#define D4    294
#define ES4   311
#define E4    330
#define F4    349
#define GES4  370
#define G4    392
#define AS4   415
#define A4    440
#define B4    466
#define H4    493
#define C5    523
#define DES5  554
#define D5    587
#define ES5   622
#define E5    659
#define F5    698
#define GES5  740
#define G5    783
#define AS5   830
#define A5    880
#define B5    932
#define H5    987
#define C6    1046
#define DES6  1108
#define D6    1174
#define ES6   1244
#define E6    1318
#define F6    1396
#define GES6  1479
#define G6    1567
#define AS6   1661
#define A6    1760
#define B6    1864
#define H6    1975
#define C7    2093
#define DES7  2217
#define D7    2349
#define ES7   2489
#define E7    2637
#define F7    2793
#define GES7  2959
#define G7    3135
#define AS7   3322
#define A7    3520
#define H7    3729
#define B7    3951

// 声音持续时间定义
#define EIGHTS 125
#define QUAD 250
#define HALF 500
#define FULL 1000
#define STOP 0

typedef enum {
    BRUSHED,
    BRUSHLESS
} motorsDrvType;

typedef struct {
    motorsDrvType drvType;

} MotorPerifDef;

/**
 * 电机映射配置
 */
//extern const MotorPerifDef* motorMapNoMotors[NBR_OF_MOTORS];
extern const MotorPerifDef *motorMapDefaultBrushed[NBR_OF_MOTORS];
// extern const MotorPerifDef* motorMapDefaltConBrushless[NBR_OF_MOTORS];
// extern const MotorPerifDef* motorMapBigQuadDeck[NBR_OF_MOTORS];
// extern const MotorPerifDef* motorMapBoltBrushless[NBR_OF_MOTORS];

/**
 * 电机模块的测试声音
 */
extern const uint16_t testsound[NBR_OF_MOTORS];

/*** 公共接口 ***/

bool pwm_timmer_init();

/**
 * 初始化。将所有电机的比例设置为0%
 */
void motorsInit(void);

/**
 * 反初始化。恢复默认值
 */
void motorsDeInit(void);

/**
 * 电机模块的测试。测试会短暂地启动每个电机，顺序为M1到M4。
 */
bool motorsTest(void);

/**
 * 设置电机 'id' 的PWM比例
 */
void motorsSetRatio(uint32_t id, uint16_t ratio);

/**
 * 获取电机 'id' 的PWM比例。如果ID错误，返回-1。
 */
int motorsGetRatio(uint32_t id);

/**
 * FreeRTOS任务以测试电机驱动程序
 */
void motorsTestTask(void *params);

/* 设置电机控制器的PWM频率
 * 该函数将所有电机设置为“beep”模式，
 * 每个电机都会以给定的比例和频率启动。
 * 比例越高，电机的功率越大。
 * 注意：过高的比例可能会使你的飞行器升空并造成危险！
 * 示例：
 *     motorsBeep(true, 1000, (uint16_t)(72000000L / frequency)/ 20);
 *     motorsBeep(false, 0, 0); *
 * */
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio);

#endif /* __MOTORS_H__ */
