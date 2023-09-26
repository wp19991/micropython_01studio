/**
 * power_distribution_stock.c - Crazyflie 电源分配代码
 *
 * 这段代码主要实现了电机的功率分配和控制，根据不同的飞行姿态计算出每个电机的功率，并将其设置到相应的电机。
 * 如果开启了电机设置开关，将使用预先设置的电机功率；否则，如果电机功率低于空闲推力，将它们设置为空闲推力。
 * 函数还提供了初始化电机、测试电机、停止电机和获取电机PWM值等功能。
 */

#include <string.h>

#include "power_distribution.h"
#include <stdio.h>
#include <string.h>
#include "num.h"
#include "motors.h"
#include "config.h"

#define TAG "PWR_DIST"

static bool motorSetEnable = false; // 用于控制电机设置的开关

static motorPower_t motorPower; // 存储电机功率的结构体

static struct {
    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
    uint16_t m4;
} motorPowerSet; // 存储电机功率设置的结构体

#ifndef DEFAULT_IDLE_THRUST
#define DEFAULT_IDLE_THRUST 0
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST; // 电机空闲状态下的推力

void powerDistributionInit(void) {
    motorsInit(); // 初始化电机
}

bool powerDistributionTest(void) {
    bool pass = true;

    pass &= motorsTest(); // 测试电机是否正常工作

    return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerStop() {
    motorsSetRatio(MOTOR_M1, 0); // 停止电机1
    motorsSetRatio(MOTOR_M2, 0); // 停止电机2
    motorsSetRatio(MOTOR_M3, 0); // 停止电机3
    motorsSetRatio(MOTOR_M4, 0); // 停止电机4
}

void powerDistribution(const control_t *control) {
#ifdef QUAD_FORMATION_X
    int16_t r = control->roll / 2.0f; // 获取滚转角度的一半
    int16_t p = control->pitch / 2.0f; // 获取俯仰角度的一半

    motorPower.m1 = limitThrust(control->thrust - r - p + control->yaw); // 计算电机1的功率
    motorPower.m3 =  limitThrust(control->thrust + r + p + control->yaw); // 计算电机3的功率

    motorPower.m2 = limitThrust(control->thrust - r + p - control->yaw); // 计算电机2的功率
    motorPower.m4 =  limitThrust(control->thrust + r - p - control->yaw); // 计算电机4的功率
#else // QUAD_FORMATION_NORMAL
    motorPower.m1 = limitThrust(control->thrust + control->pitch + control->yaw); // 计算电机1的功率
    motorPower.m2 = limitThrust(control->thrust - control->roll - control->yaw); // 计算电机2的功率
    motorPower.m3 = limitThrust(control->thrust - control->pitch + control->yaw); // 计算电机3的功率
    motorPower.m4 = limitThrust(control->thrust + control->roll - control->yaw); // 计算电机4的功率
#endif

    if (motorSetEnable) {
        motorsSetRatio(MOTOR_M1, motorPowerSet.m1); // 设置电机1的功率
        motorsSetRatio(MOTOR_M2, motorPowerSet.m2); // 设置电机2的功率
        motorsSetRatio(MOTOR_M3, motorPowerSet.m3); // 设置电机3的功率
        motorsSetRatio(MOTOR_M4, motorPowerSet.m4); // 设置电机4的功率
    } else {
        if (motorPower.m1 < idleThrust) {
            motorPower.m1 = idleThrust; // 如果电机1的功率低于空闲推力，将其设置为空闲推力
        }
        if (motorPower.m2 < idleThrust) {
            motorPower.m2 = idleThrust; // 如果电机2的功率低于空闲推力，将其设置为空闲推力
        }
        if (motorPower.m3 < idleThrust) {
            motorPower.m3 = idleThrust; // 如果电机3的功率低于空闲推力，将其设置为空闲推力
        }
        if (motorPower.m4 < idleThrust) {
            motorPower.m4 = idleThrust; // 如果电机4的功率低于空闲推力，将其设置为空闲推力
        }

        motorsSetRatio(MOTOR_M1, motorPower.m1); // 设置电机1的功率
        motorsSetRatio(MOTOR_M2, motorPower.m2); // 设置电机2的功率
        motorsSetRatio(MOTOR_M3, motorPower.m3); // 设置电机3的功率
        motorsSetRatio(MOTOR_M4, motorPower.m4); // 设置电机4的功率
    }
}

void getMotorPWM(motorPower_t *get) {
    *get = motorPower; // 获取电机的PWM值
}
