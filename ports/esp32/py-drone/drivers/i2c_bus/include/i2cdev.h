/**
 * i2cdev.h - 用于与I2C设备通信的函数
 */

#ifndef __I2CDEV_H__
#define __I2CDEV_H__

#include <stdint.h>
#include <stdbool.h>

#include "i2c_drv.h"

#define I2CDEV_NO_MEM_ADDR  0xFF

typedef I2cDrv I2C_Dev;
#define I2C1_DEV  &deckBus
#define I2C0_DEV  &sensorsBus

// 为了兼容性
#define i2cdevWrite16 i2cdevWriteReg16
#define i2cdevRead16  i2cdevReadReg16

#define I2C_TIMEOUT 5
#define I2CDEV_CLK_TS (1000000 / 100000)

#define I2C_MASTER_ACK_EN   true    /*!< 启用主机的应答检查 */
#define I2C_MASTER_ACK_DIS  false   /*!< 禁用主机的应答检查 */

/**
 * 从I2C外设读取字节
 * @param dev  要从中读取的I2C外设的指针
 * @param devAddress  要读取的设备地址
 * @param len  要读取的字节数。
 * @param data  用于存储读取数据的缓冲区的指针。
 *
 * @return 如果读取成功则返回TRUE，否则返回FALSE。
 */
bool i2cdevRead(I2C_Dev *dev, uint8_t devAddress, uint16_t len, uint8_t *data);

/**
 * 从I2C外设读取字节
 * @param dev  要从中读取的I2C外设的指针
 * @param devAddress  要读取的设备地址
 * @param memAddress  要读取的内部地址，如果没有则为I2CDEV_NO_MEM_ADDR。
 * @param len  要读取的字节数。
 * @param data  用于存储读取数据的缓冲区的指针。
 *
 * @return 如果读取成功则返回TRUE，否则返回FALSE。
 */
bool i2cdevReadReg8(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint16_t len, uint8_t *data);

/**
 * 从具有16位内部寄存器/内存地址的I2C外设读取字节
 * @param dev  要从中读取的I2C外设的指针
 * @param devAddress  要读取的设备地址
 * @param memAddress  要读取的内部地址，如果没有则为I2CDEV_NO_MEM_ADDR。
 * @param len  要读取的字节数。
 * @param data  用于存储读取数据的缓冲区的指针。
 *
 * @return 如果读取成功则返回TRUE，否则返回FALSE。
 */
bool i2cdevReadReg16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
                     uint16_t len, uint8_t *data);

/**
 * I2C设备初始化函数。
 * @param dev  要初始化的I2C外设的指针。
 *
 * @return 如果初始化成功则返回TRUE，否则返回FALSE。
 */
int i2cdevInit(I2C_Dev *dev);

/**
 * 从I2C外设读取一个字节
 * @param dev  要从中读取的I2C外设的指针
 * @param devAddress  要读取的设备地址
 * @param memAddress  要读取的内部地址，如果没有则为I2CDEV_NO_MEM_ADDR。
 * @param data  用于存储读取数据的缓冲区的指针。
 *
 * @return 如果读取成功则返回TRUE，否则返回FALSE。
 */
bool i2cdevReadByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t *data);

/**
 * 从I2C外设读取一个位
 * @param dev  要从中读取的I2C外设的指针
 * @param devAddress  要读取的设备地址
 * @param memAddress  要读取的内部地址，如果没有则为I2CDEV_NO_MEM_ADDR。
 * @param bitNum  要读取的位数，0 - 7。
 * @param data  用于存储读取数据的缓冲区的指针。
 *
 * @return 如果读取成功则返回TRUE，否则返回FALSE。
 */
bool i2cdevReadBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                   uint8_t bitNum, uint8_t *data);

/**
 * 从I2C外设读取最多8位
 * @param dev  要从中读取的I2C外设的指针
 * @param devAddress  要读取的设备地址
 * @param memAddress  要读取的内部地址，如果没有则为I2CDEV_NO_MEM_ADDR。
 * @param bitStart  要从中开始的位，0 - 7，MSB为0
 * @param length  要读取的位数，1 - 8。
 * @param data  用于存储读取数据的缓冲区的指针。
 *
 * @return 如果读取成功则返回TRUE，否则返回FALSE。
 */
bool i2cdevReadBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitStart, uint8_t length, uint8_t *data);

/**
 * 向I2C外设写入字节
 * @param dev  要写入的I2C外设的指针
 * @param devAddress  要写入的设备地址
 * @param len  要读取的字节数。
 * @param data  从中读取数据并写入的缓冲区的指针。
 *
 * @return 如果写入成功则返回TRUE，否则返回FALSE。
 */
bool i2cdevWrite(I2C_Dev *dev, uint8_t devAddress, uint16_t len, uint8_t *data);

/**
 * 向I2C外设写入字节
 * @param dev  要写入的I2C外设的指针
 * @param devAddress  要写入的设备地址
 * @param memAddress  要写入的内部地址，如果没有则为I2CDEV_NO_MEM_ADDR。
 * @param len  要读取的字节数。
 * @param data  从中读取数据并写入的缓冲区的指针。
 *
 * @return 如果写入成功则返回TRUE，否则返回FALSE。
 */
bool i2cdevWriteReg8(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint16_t len, uint8_t *data);

/**
 * 向具有16位内部寄存器/内存地址的I2C外设写入字节
 * @param dev  要写入的I2C外设的指针
 * @param devAddress  要写入的设备地址
 * @param memAddress  要写入的内部地址，如果没有则为I2CDEV_NO_MEM_ADDR。
 * @param len  要读取的字节数。
 * @param data  从中读取数据并写入的缓冲区的指针。
 *
 * @return 如果写入成功则返回TRUE，否则返回FALSE。
 */
bool i2cdevWriteReg16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
                      uint16_t len, uint8_t *data);

/**
 * 向I2C外设写入一个字节
 * @param dev  要写入的I2C外设的指针
 * @param devAddress  要写入的设备地址
 * @param memAddress  要写入的内部地址，如果没有则为I2CDEV_NO_MEM_ADDR。
 * @param data  要写入的字节。
 *
 * @return 如果写入成功则返回TRUE，否则返回FALSE。
 */
bool i2cdevWriteByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint8_t data);

/**
 * 向I2C外设写入一个位
 * @param dev  要写入的I2C外设的指针
 * @param devAddress  要写入的设备地址
 * @param memAddress  要写入的内部地址，如果没有则为I2CDEV_NO_MEM_ADDR。
 * @param bitNum  要写入的位数，0 - 7。
 * @param data  要写入的位。
 *
 * @return 如果写入成功则返回TRUE，否则返回FALSE。
 */
bool i2cdevWriteBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitNum, uint8_t data);

/**
 * 向I2C外设写入最多8位
 * @param dev  要写入的I2C外设的指针
 * @param devAddress  要写入的设备地址
 * @param memAddress  要写入的内部地址，如果没有则为I2CDEV_NO_MEM_ADDR。
 * @param bitStart  要从中开始的位，0 - 7。
 * @param length  要写入的位数，1 - 8。
 * @param data  包含要写入的位的字节。
 *
 * @return 如果写入成功则返回TRUE，否则返回FALSE。
 */
bool i2cdevWriteBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint8_t bitStart, uint8_t length, uint8_t data);

#endif //__I2CDEV_H__
