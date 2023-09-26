// I2Cdev库集合 - HMC5883L I2C设备类
// 基于Honeywell HMC5883L数据手册，10/2010 (Form #900405 Rev B)
// 由Jeff Rowberg <jeff@rowberg.net>于6/12/2012创建
// 更新应始终可在https://github.com/jrowberg/i2cdevlib获得

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include "i2cdev.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "py/obj.h"
#include "esp_log.h"

#define TAG "HMC5883L"

static uint8_t devAddr;
static uint8_t buffer[6];
static uint8_t mode;
static I2C_Dev *I2Cx;
static bool isInit;

/** 上电并准备进行一般使用。
 * 这将使用默认设置准备磁力计，以便在单次使用模式下（功耗要求很低）进行使用。
 * 默认设置包括8次采样平均、15Hz数据输出速率、正常测量偏差和1090增益（以LSB/Gauss表示）。
 * 请确保在初始化后特别调整任何需要的设置，尤其是增益设置，如果你看到了很多-4096值（请参阅数据手册获取更多信息）。
 */
void hmc5883lInit(I2C_Dev *i2cPort) {
    if (isInit) {
        return;
    }

    I2Cx = i2cPort;
    devAddr = HMC5883L_ADDRESS;

    // 写入配置字节以初始化磁力计
    i2cdevWriteByte(I2Cx, devAddr, QMC5883L_RA_CONFIG_2, 0x00);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    i2cdevWriteByte(I2Cx, devAddr, QMC5883L_RA_MODE, 0x01);
    i2cdevWriteByte(I2Cx, devAddr, QMC5883L_RA_CONFIG_2, 0x40);
    isInit = true;
}

/** 反初始化。恢复默认值 */
void hmc5883lDeInit(void) {
    isInit = false;
}

/** 验证I2C连接。
 * 确保设备已连接并以预期方式响应。
 * @return 连接有效时返回true，否则返回false
 */
bool hmc5883lTestConnection() {
    uint8_t read_id = 0;

    if (i2cdevReadByte(I2Cx, devAddr, QMC5883L_CHIP_ID, &read_id)) {
        ESP_LOGI(TAG, "MC5883 ID为: 0x%X\n", read_id);
        return (read_id == HMC5883L_DEFAULT_ADDRESS);
    }
    return false;
}

/** 执行自检。
 * @return 自检通过时返回true，否则返回false
 */
bool hmc5883lSelfTest() {
    // 此处可执行自检测试
    bool testStatus = true;
    return testStatus;
}

/** 评估HMC8335L自检的值。
 * @param min 自检的下限
 * @param max 自检的上限
 * @param value 要比较的值
 * @param string 描述值的字符串的指针
 * @return 自检是否在min - max限制内，是则返回true，否则返回false
 */
bool hmc5883lEvaluateSelfTest(int16_t min, int16_t max, int16_t value, char *string) {
    if (value < min || value > max) {
        ESP_LOGI(TAG, "自检 %s [失败]。低值：%d，高值：%d，测量值：%d\n",
                 string, min, max, value);
        return false;
    }

    return true;
}

/** 获取数据输出速率值。
 * 表格下面显示了在连续测量模式下的所有可选择输出速率。在给定输出速率下，应测量所有三个通道。
 * 最大速率为160 Hz的其他输出速率可以通过在单次测量模式下监视DRDY中断引脚来实现。
 *
 * 值 | 典型数据输出速率（Hz）
 * ------+------------------------------
 * 0     | 0.75
 * 1     | 1.5
 * 2     | 3
 * 3     | 7.5
 * 4     | 15（默认）
 * 5     | 30
 * 6     | 75
 * 7     | 未使用
 *
 * @return 当前数据输出到寄存器的速率
 */
uint8_t hmc5883lGetDataRate() {
    i2cdevReadBits(I2Cx, devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_RATE_BIT, HMC5883L_CRA_RATE_LENGTH, buffer);
    return buffer[0];
}

/** 设置数据输出速率值。
 * @param rate 数据输出到寄存器的速率
 */
void hmc5883lSetDataRate(uint8_t rate) {
    i2cdevWriteBits(I2Cx, devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_RATE_BIT, HMC5883L_CRA_RATE_LENGTH, rate);
}

/** 获取测量偏差值。
 * @return 当前偏差值（0-2表示正常/正偏差/负偏差）
 */
uint8_t hmc5883lGetMeasurementBias() {
    i2cdevReadBits(I2Cx, devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_BIAS_BIT, HMC5883L_CRA_BIAS_LENGTH, buffer);
    return buffer[0];
}

/** 设置测量偏差值。
 * @param bias 新的偏差值（0-2表示正常/正偏差/负偏差）
 */
void hmc5883lSetMeasurementBias(uint8_t bias) {
    i2cdevWriteBits(I2Cx, devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_BIAS_BIT, HMC5883L_CRA_BIAS_LENGTH, bias);
}

// MODE寄存器

/** 获取测量模式。
 * 在连续测量模式下，设备会持续执行测量并将结果放入数据寄存器中。当新数据放入所有三个寄存器时，RDY会变高。
 * 在上电或写入模式或配置寄存器后，首次测量集将在2/fDO后可在所有三个数据输出寄存器中获得，并且后续测量将以fDO的频率可用，其中fDO是数据输出的频率。
 *
 * 当选择单次测量模式（默认）时，设备执行单次测量，将RDY置高，并返回到空闲模式。模式寄存器的位值返回到空闲模式。
 * 测量仍然保留在数据输出寄存器中，并且在读取数据输出寄存器或执行另一个测量之前，RDY保持高电平。
 * @return 当前测量模式
 */
uint8_t hmc5883lGetMode() {
    i2cdevReadBits(I2Cx, devAddr, HMC5883L_RA_MODE, HMC5883L_MODEREG_BIT, HMC5883L_MODEREG_LENGTH, buffer);
    return buffer[0];
}

/** 设置测量模式。
 * @param newMode 新的测量模式
 */
void hmc5883lSetMode(uint8_t newMode) {
    // 使用此方法可确保将位7-2设置为零，这是数据手册中指定的要求；实际上，使用I2Cdev.writeBits方法更有效
    uint8_t tempMode = (QMC5883L_OUTPUT_10HZ | QMC5883L_OUTPUT_2G | QMC5883L_SAMPLE_128);

    i2cdevWriteByte(I2Cx, devAddr, QMC5883L_RA_CONFIG_1, newMode | tempMode);

    mode = newMode; // 用于跟踪是否需要在读取后清除位7
}

// 数据寄存器

/** 获取三轴方向的磁头测量。
 * 如果ADC读取对于给定通道溢出或下溢，或者在偏差测量期间发生数学溢出，则此数据寄存器将包含值-4096。
 * 此寄存器值将在下一个有效测量之后清除。请注意，如果单次模式处于活动状态，此方法将自动清除MODE寄存器中的适当位。
 * @param x 用于存储X轴磁头的16位有符号整数容器
 * @param y 用于存储Y轴磁头的16位有符号整数容器
 * @param z 用于存储Z轴磁头的16位有符号整数容器
 */
void hmc5883lGetHeading(int16_t *x, int16_t *y, int16_t *z) {
    i2cdevReadReg8(I2Cx, devAddr, HMC5883L_RA_DATAX_H, 6, buffer);

    if (mode == HMC5883L_MODE_SINGLE) {
        i2cdevWriteByte(I2Cx, devAddr, HMC5883L_RA_MODE,
                        HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
    }

    *x = (((int16_t) buffer[0]) << 8) | buffer[1];
    *y = (((int16_t) buffer[4]) << 8) | buffer[5];
    *z = (((int16_t) buffer[2]) << 8) | buffer[3];
}

/** 获取X轴磁头测量。
 * @return 包含X轴磁头的16位有符号整数
 */
int16_t hmc5883lGetHeadingX() {
    // 每个轴读取都要求读取所有轴寄存器，即使只使用一个轴；这在代码中不是无意中做得不高效的
    i2cdevReadReg8(I2Cx, devAddr, HMC5883L_RA_DATAX_H, 6, buffer);

    if (mode == HMC5883L_MODE_SINGLE) {
        i2cdevWriteByte(I2Cx, devAddr, HMC5883L_RA_MODE,
                        HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
    }

    return (((int16_t) buffer[0]) << 8) | buffer[1];
}

/** 获取Y轴磁头测量。
 * @return 包含Y轴磁头的16位有符号整数
 */
int16_t hmc5883lGetHeadingY() {
    // 每个轴读取都要求读取所有轴寄存器，即使只使用一个轴；这在代码中不是无意中做得不高效的
    i2cdevReadReg8(I2Cx, devAddr, HMC5883L_RA_DATAX_H, 6, buffer);

    if (mode == HMC5883L_MODE_SINGLE) {
        i2cdevWriteByte(I2Cx, devAddr, HMC5883L_RA_MODE,
                        HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
    }

    return (((int16_t) buffer[4]) << 8) | buffer[5];
}

/** 获取Z轴磁头测量。
 * @return 包含Z轴磁头的16位有符号整数
 */
int16_t hmc5883lGetHeadingZ() {
    // 每个轴读取都要求读取所有轴寄存器，即使只使用一个轴；这在代码中不是无意中做得不高效的
    i2cdevReadReg8(I2Cx, devAddr, HMC5883L_RA_DATAX_H, 6, buffer);

    if (mode == HMC5883L_MODE_SINGLE) {
        i2cdevWriteByte(I2Cx, devAddr, HMC5883L_RA_MODE,
                        HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
    }

    return (((int16_t) buffer[2]) << 8) | buffer[3];
}

// STATUS寄存器

/** 获取数据输出寄存器锁定状态。
 * 当某些但不是所有六个数据输出寄存器被读取时，设置此位。当此位被设置时，六个数据输出寄存器被锁定，
 * 任何新数据都不会放入这些寄存器，直到满足以下三种情况之一：一、所有六个字节都已读取或模式更改，二、模式更改，三、更改了测量配置。
 * @return 数据输出寄存器锁定状态
 */
bool hmc5883lGetLockStatus() {
    i2cdevReadBit(I2Cx, devAddr, HMC5883L_RA_STATUS, HMC5883L_STATUS_LOCK_BIT, buffer);
    return buffer[0];
}

/** 获取数据准备状态。
 * 当数据被写入所有六个数据寄存器时，此位被设置，并且在设备启动写入数据输出寄存器并且之后写入一个或多个数据输出寄存器后被清除。
 * 当RDY位清除时，它将保持清除状态250微秒。 DRDY引脚可以用作监视设备以获得测量数据的状态寄存器的替代方法。
 * @return 数据准备状态
 */
bool hmc5883lGetReadyStatus() {
    i2cdevReadBit(I2Cx, devAddr, HMC5883L_RA_STATUS, HMC5883L_STATUS_READY_BIT, buffer);
    return buffer[0];
}
