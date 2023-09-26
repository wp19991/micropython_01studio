/**
 * ESP-Drone固件
 *
 * i2cdev.c - 用于与I2C设备通信的函数
 */

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "i2cdev.h"
#include "i2c_drv.h"

// 初始化I2C设备
int i2cdevInit(I2C_Dev *dev) {
    // 调用i2cdrvInit函数进行初始化
    i2cdrvInit(dev);
    return true;
}

// 从I2C设备读取数据
bool i2cdevRead(I2C_Dev *dev, uint8_t devAddress, uint16_t len, uint8_t *data) {
    // 调用i2cdevReadReg8函数进行读取，内部地址设置为I2CDEV_NO_MEM_ADDR
    return i2cdevReadReg8(dev, devAddress, I2CDEV_NO_MEM_ADDR, len, data);
}

// 从I2C设备读取一个字节
bool i2cdevReadByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t *data) {
    // 调用i2cdevReadReg8函数进行读取，读取一个字节
    return i2cdevReadReg8(dev, devAddress, memAddress, 1, data);
}

// 从I2C设备读取一个位
bool i2cdevReadBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t bitNum, uint8_t *data) {
    uint8_t byte;
    bool status;

    // 调用i2cdevReadReg8函数进行读取，读取一个字节
    status = i2cdevReadReg8(dev, devAddress, memAddress, 1, &byte);
    // 将指定位的数据提取出来
    *data = byte & (1 << bitNum);

    return status;
}

// 从I2C设备读取指定位数的数据
bool
i2cdevReadBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t bitStart, uint8_t length, uint8_t *data) {
    bool status;
    uint8_t byte;

    // 先读取一个字节的数据
    if ((status = i2cdevReadByte(dev, devAddress, memAddress, &byte)) == true) {
        // 构建掩码来提取指定位的数据
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        byte &= mask;
        byte >>= (bitStart - length + 1);
        *data = byte;
    }

    return status;
}

// 从I2C设备读取一个8位寄存器的数据
bool i2cdevReadReg8(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t *data) {
    // 获取总线互斥锁，防止冲突访问
    if (xSemaphoreTake(dev->isBusFreeMutex, (TickType_t) 5) == pdFALSE) {
        return false;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (memAddress != I2CDEV_NO_MEM_ADDR) {
        // 如果指定了内部地址，首先写入内部地址
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
        i2c_master_write_byte(cmd, memAddress, I2C_MASTER_ACK_EN);
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_READ, I2C_MASTER_ACK_EN);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(dev->def->i2cPort, cmd, (TickType_t) 5);
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive(dev->isBusFreeMutex);

    // 打印读写日志和错误日志
#if defined CONFIG_I2CBUS_LOG_READWRITES
    if (!err) {
        char str[len * 5 + 1];

        for (size_t i = 0; i < len; i++) {
            sprintf(str + i * 5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
        }

        I2CBUS_LOG_RW("[port:%d, slave:0x%X] 从寄存器0x%X读取了%d字节数据：%s", dev->def->i2cPort, devAddress, memAddress, len, str);
    }
#endif
#if defined CONFIG_I2CBUS_LOG_ERRORS
#if defined CONFIG_I2CBUS_LOG_READWRITES
    else {
#else
    if (err) {
#endif
        I2CBUS_LOGE("[port:%d, slave:0x%X] 读取寄存器0x%X失败，错误码：0x%X", dev->def->i2cPort, devAddress, memAddress, err);
    }
#endif

    if (err == ESP_OK) {
        return true;
    } else {
        return false;
    }
}

// 从I2C设备读取一个16位寄存器的数据
bool i2cdevReadReg16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress, uint16_t len, uint8_t *data) {
    // 获取总线互斥锁，防止冲突访问
    if (xSemaphoreTake(dev->isBusFreeMutex, (TickType_t) 5) == pdFALSE) {
        return false;
    }

    uint8_t memAddress8[2];
    memAddress8[0] = (uint8_t)((memAddress >> 8) & 0x00FF);
    memAddress8[1] = (uint8_t)(memAddress & 0x00FF);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
    if (memAddress != I2C_NO_INTERNAL_ADDRESS) {
        // 如果指定了内部地址，首先写入内部地址
        i2c_master_write(cmd, memAddress8, 2, I2C_MASTER_ACK_EN);
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_READ, I2C_MASTER_ACK_EN);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(dev->def->i2cPort, cmd, (TickType_t) 5);
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive(dev->isBusFreeMutex);

    // 打印读写日志和错误日志
#if defined CONFIG_I2CBUS_LOG_READWRITES
    if (!err) {
        char str[len * 5 + 1];

        for (size_t i = 0; i < len; i++) {
            sprintf(str + i * 5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
        }

        I2CBUS_LOG_RW("[port:%d, slave:0x%X] 从寄存器0x%X读取了%d字节数据：%s", dev->def->i2cPort, devAddress, memAddress, len, str);
    }
#endif
#if defined CONFIG_I2CBUS_LOG_ERRORS
#if defined CONFIG_I2CBUS_LOG_READWRITES
    else {
#else
    if (err) {
#endif
        I2CBUS_LOGE("[port:%d, slave:0x%X] 读取寄存器0x%X失败，错误码：0x%X", dev->def->i2cPort, devAddress, memAddress, err);
    }
#endif

    if (err == ESP_OK) {
        return true;
    } else {
        return false;
    }
}

// 向I2C设备写入一个字节
bool i2cdevWriteByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t data) {
    // 调用i2cdevWriteReg8函数进行写入
    return i2cdevWriteReg8(dev, devAddress, memAddress, 1, &data);
}

// 向I2C设备写入一个位
bool i2cdevWriteBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t bitNum, uint8_t data) {
    uint8_t byte;
    // 先读取当前字节数据
    i2cdevReadByte(dev, devAddress, memAddress, &byte);
    // 根据bitNum设置或清除相应位
    byte = (data != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum));
    // 写回I2C设备
    return i2cdevWriteByte(dev, devAddress, memAddress, byte);
}

// 向I2C设备写入指定位数的数据
bool
i2cdevWriteBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint8_t bitStart, uint8_t length, uint8_t data) {
    bool status;
    uint8_t byte;

    // 先读取一个字节的数据
    if ((status = i2cdevReadByte(dev, devAddress, memAddress, &byte)) == true) {
        // 构建掩码来清除指定位的数据
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        // 将数据放入正确的位置
        data <<= (bitStart - length + 1);
        data &= mask;         // 将数据中的非重要位清零
        byte &= ~(mask);      // 将原字节中的重要位清零
        byte |= data;         // 将数据与原字节合并
        // 写回I2C设备
        status = i2cdevWriteByte(dev, devAddress, memAddress, byte);
    }

    return status;
}

// 向I2C设备写入一个8位寄存器的数据
bool i2cdevWriteReg8(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t *data) {
    // 获取总线互斥锁，防止冲突访问
    if (xSemaphoreTake(dev->isBusFreeMutex, (TickType_t) 5) == pdFALSE) {
        return false;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
    if (memAddress != I2CDEV_NO_MEM_ADDR) {
        // 如果指定了内部地址，首先写入内部地址
        i2c_master_write_byte(cmd, memAddress, I2C_MASTER_ACK_EN);
    }
    i2c_master_write(cmd, (uint8_t *) data, len, I2C_MASTER_ACK_EN);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(dev->def->i2cPort, cmd, (TickType_t) 5);
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive(dev->isBusFreeMutex);

    // 打印读写日志和错误日志
#if defined CONFIG_I2CBUS_LOG_READWRITES
    if (!err) {
        char str[len * 5 + 1];

        for (size_t i = 0; i < len; i++) {
            sprintf(str + i * 5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
        }

        I2CBUS_LOG_RW("[port:%d, slave:0x%X] 写入了%d字节数据到寄存器0x%X：%s", dev->def->i2cPort, devAddress, len, memAddress, str);
    }
#endif
#if defined CONFIG_I2CBUS_LOG_ERRORS
#if defined CONFIG_I2CBUS_LOG_READWRITES
    else {
#else
    if (err) {
#endif
        I2CBUS_LOGE("[port:%d, slave:0x%X] 写入%d字节数据到寄存器0x%X失败，错误码：0x%X", dev->def->i2cPort, devAddress, len, memAddress, err);
    }
#endif

    if (err == ESP_OK) {
        return true;
    } else {
        return false;
    }
}

// 向I2C设备写入一个16位寄存器的数据
bool i2cdevWriteReg16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress, uint16_t len, uint8_t *data) {
    // 获取总线互斥锁，防止冲突访问
    if (xSemaphoreTake(dev->isBusFreeMutex, (TickType_t) 5) == pdFALSE) {
        return false;
    }

    uint8_t memAddress8[2];
    memAddress8[0] = (uint8_t)((memAddress >> 8) & 0x00FF);
    memAddress8[1] = (uint8_t)(memAddress & 0x00FF);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
    if (memAddress != I2C_NO_INTERNAL_ADDRESS) {
        // 如果指定了内部地址，首先写入内部地址
        i2c_master_write(cmd, memAddress8, 2, I2C_MASTER_ACK_EN);
    }
    i2c_master_write(cmd, (uint8_t *) data, len, I2C_MASTER_ACK_EN);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(dev->def->i2cPort, cmd, (TickType_t) 5);
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive(dev->isBusFreeMutex);

    // 打印读写日志和错误日志
#if defined CONFIG_I2CBUS_LOG_READWRITES
    if (!err) {
        char str[len * 5 + 1];

        for (size_t i = 0; i < len; i++) {
            sprintf(str + i * 5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
        }

        I2CBUS_LOG_RW("[port:%d, slave:0x%X] 写入了%d字节数据到寄存器0x%X：%s", dev->def->i2cPort, devAddress, len, memAddress, str);
    }
#endif
#if defined CONFIG_I2CBUS_LOG_ERRORS
#if defined CONFIG_I2CBUS_LOG_READWRITES
    else {
#else
    if (err) {
#endif
        I2CBUS_LOGE("[port:%d, slave:0x%X] 写入%d字节数据到寄存器0x%X失败，错误码：0x%X", dev->def->i2cPort, devAddress, len, memAddress, err);
    }
#endif

    if (err == ESP_OK) {
        return true;
    } else {
        return false;
    }
}
