// I2Cdev库集合 - HMC5883L I2C设备类头文件
// 基于Honeywell HMC5883L数据手册，10/2010（Form #900405 Rev B）
// 由Jeff Rowberg <jeff@rowberg.net>于6/12/2012编写
// 更新应始终可在https://github.com/jrowberg/i2cdevlib获得

#ifndef HMC5883L_H_
#define HMC5883L_H_

#include <stdbool.h>
#include "i2cdev.h"

// 设备地址
#define HMC5883L_ADDRESS            0x0D
#define HMC5883L_DEFAULT_ADDRESS    0xFF

// 寄存器地址
#define QMC5883L_RA_DATAX_L         0x00
#define QMC5883L_RA_DATAX_H         0x01
#define QMC5883L_RA_DATAY_L         0x02
#define QMC5883L_RA_DATAY_H         0x03
#define QMC5883L_RA_DATAZ_L         0x04
#define QMC5883L_RA_DATAZ_H         0x05
#define QMC5883L_RA_STATUS          0x06
#define QMC5883L_RA_DATA_TEMP_L     0x07
#define QMC5883L_RA_DATA_TEMP_H     0x08
#define QMC5883L_RA_CONFIG_1        0x09
#define QMC5883L_RA_CONFIG_2        0x0A
#define QMC5883L_RA_MODE            0x0B
#define QMC5883L_CHIP_ID            0x0D

// 模式
#define QMC5883L_MODE_STANDBY       0x00
#define QMC5883L_MODE_CONTINUOUS    0x01

// 数据输出速率
#define QMC5883L_OUTPUT_10HZ        0x00
#define QMC5883L_OUTPUT_50HZ        0x04
#define QMC5883L_OUTPUT_100HZ       0x08
#define QMC5883L_OUTPUT_200HZ       0x0C

// 增益
#define QMC5883L_OUTPUT_2G          0x00
#define QMC5883L_OUTPUT_8G          0x10

// 采样数
#define QMC5883L_SAMPLE_64          0xC0
#define QMC5883L_SAMPLE_128         0x80
#define QMC5883L_SAMPLE_256         0x40
#define QMC5883L_SAMPLE_512         0x00
#define QMC5883L_STATUS_DRDY_BIT    0x01

#define HMC5883L_RA_CONFIG_A        0x00
#define HMC5883L_RA_CONFIG_B        0x01
#define HMC5883L_RA_MODE            0x02
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAX_L         0x04
#define HMC5883L_RA_DATAZ_H         0x05
#define HMC5883L_RA_DATAZ_L         0x06
#define HMC5883L_RA_DATAY_H         0x07
#define HMC5883L_RA_DATAY_L         0x08
#define HMC5883L_RA_STATUS          0x09
#define HMC5883L_RA_ID_A            0x0A
#define HMC5883L_RA_ID_B            0x0B
#define HMC5883L_RA_ID_C            0x0C

// CONFIG_A寄存器位
#define HMC5883L_CRA_AVERAGE_BIT    6
#define HMC5883L_CRA_AVERAGE_LENGTH 2
#define HMC5883L_CRA_RATE_BIT       4
#define HMC5883L_CRA_RATE_LENGTH    3
#define HMC5883L_CRA_BIAS_BIT       1
#define HMC5883L_CRA_BIAS_LENGTH    2

// 数据输出速率选项
#define HMC5883L_AVERAGING_1        0x00
#define HMC5883L_AVERAGING_2        0x01
#define HMC5883L_AVERAGING_4        0x02
#define HMC5883L_AVERAGING_8        0x03

#define HMC5883L_RATE_0P75          0x00
#define HMC5883L_RATE_1P5           0x01
#define HMC5883L_RATE_3             0x02
#define HMC5883L_RATE_7P5           0x03
#define HMC5883L_RATE_15            0x04
#define HMC5883L_RATE_30            0x05
#define HMC5883L_RATE_75            0x06

// 偏差选项
#define HMC5883L_BIAS_NORMAL        0x00
#define HMC5883L_BIAS_POSITIVE      0x01
#define HMC5883L_BIAS_NEGATIVE      0x02

// CONFIG_B寄存器位
#define HMC5883L_CRB_GAIN_BIT       7
#define HMC5883L_CRB_GAIN_LENGTH    3

// 增益选项
#define HMC5883L_GAIN_1370          0x00
#define HMC5883L_GAIN_1090          0x01
#define HMC5883L_GAIN_820           0x02
#define HMC5883L_GAIN_660           0x03
#define HMC5883L_GAIN_440           0x04
#define HMC5883L_GAIN_390           0x05
#define HMC5883L_GAIN_330           0x06
#define HMC5883L_GAIN_220           0x07

// MODE寄存器位
#define HMC5883L_MODEREG_BIT        1
#define HMC5883L_MODEREG_LENGTH     2

// 测量模式选项
#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01
#define HMC5883L_MODE_IDLE          0x02

// STATUS寄存器位
#define HMC5883L_STATUS_LOCK_BIT    1
#define HMC5883L_STATUS_READY_BIT   0

// 自测试参数
#define HMC5883L_ST_GAIN            HMC5883L_GAIN_440  // 自测试时的增益值
#define HMC5883L_ST_GAIN_NBR        440
#define HMC5883L_ST_ERROR           0.2                // 最大误差
#define HMC5883L_ST_DELAY_MS        250                // 自测试延迟时间（毫秒）
#define HMC5883L_ST_X_NORM          (int32_t)(1.16 * HMC5883L_ST_GAIN_NBR)
#define HMC5883L_ST_X_MIN           (int32_t)(HMC5883L_ST_X_NORM - (HMC5883L_ST_X_NORM * HMC5883L_ST_ERROR))
#define HMC5883L_ST_X_MAX           (int32_t)(HMC5883L_ST_X_NORM + (HMC5883L_ST_X_NORM * HMC5883L_ST_ERROR))
#define HMC5883L_ST_Y_NORM          (int32_t)(1.16 * HMC5883L_ST_GAIN_NBR)
#define HMC5883L_ST_Y_MIN           (int32_t)(HMC5883L_ST_Y_NORM - (HMC5883L_ST_Y_NORM * HMC5883L_ST_ERROR))
#define HMC5883L_ST_Y_MAX           (int32_t)(HMC5883L_ST_Y_NORM + (HMC5883L_ST_Y_NORM * HMC5883L_ST_ERROR))
#define HMC5883L_ST_Z_NORM          (int32_t)(1.08 * HMC5883L_ST_GAIN_NBR)
#define HMC5883L_ST_Z_MIN           (int32_t)(HMC5883L_ST_Z_NORM - (HMC5883L_ST_Z_NORM * HMC5883L_ST_ERROR))
#define HMC5883L_ST_Z_MAX           (int32_t)(HMC5883L_ST_Z_NORM + (HMC5883L_ST_Z_NORM * HMC5883L_ST_ERROR))

// 函数声明
void hmc5883lInit(I2C_Dev *i2cPort);

void hmc5883lDeInit(void);

bool hmc5883lTestConnection();

bool hmc5883lSelfTest();

bool hmc5883lEvaluateSelfTest(int16_t min, int16_t max, int16_t value, char *string);

// CONFIG_A寄存器函数
uint8_t hmc5883lGetSampleAveraging();

void hmc5883lSetSampleAveraging(uint8_t averaging);

uint8_t hmc5883lGetDataRate();

void hmc5883lSetDataRate(uint8_t rate);

uint8_t hmc5883lGetMeasurementBias();

void hmc5883lSetMeasurementBias(uint8_t bias);

// CONFIG_B寄存器函数
uint8_t hmc5883lGetGain();

void hmc5883lSetGain(uint8_t gain);

// MODE寄存器函数
uint8_t hmc5883lGetMode();

void hmc5883lSetMode(uint8_t mode);

// DATA*寄存器函数
void hmc5883lGetHeading(int16_t *x, int16_t *y, int16_t *z);

int16_t hmc5883lGetHeadingX();

int16_t hmc5883lGetHeadingY();

int16_t hmc5883lGetHeadingZ();

// STATUS寄存器函数
bool hmc5883lGetLockStatus();

bool hmc5883lGetReadyStatus();

#endif /* HMC5883L_H_ */
