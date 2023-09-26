#ifndef I2C_DRV_H
#define I2C_DRV_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/i2c.h"

#define I2C_NO_INTERNAL_ADDRESS   0xFFFF

typedef enum {
    i2cAck,
    i2cNack
} I2cStatus;

typedef enum {
    i2cWrite,
    i2cRead
} I2cDirection;

/**
 * 结构体用于存储I2C消息的详细信息。然后将该结构体排队等待I2C ISR处理。
 */
typedef struct _I2cMessage {
    uint32_t messageLength;      //< 要发送或接收的数据字节数。
    uint8_t slaveAddress;       //< I2C总线上设备的从机地址。
    uint8_t nbrOfRetries;      //< 重试次数。
    I2cDirection direction;          //< 消息方向
    I2cStatus status;             //< I2C状态
    xQueueHandle clientQueue;        //< 发送接收消息的队列。
    bool isInternal16bit;    //< 是否是16位内部地址。如果为false，为8位。
    uint16_t internalAddress;    //< 设备的内部地址。
    uint8_t *buffer;            //< 数据缓冲区的指针，用于传输数据或接收数据。
} I2cMessage;

typedef struct {
    i2c_port_t i2cPort;
    uint32_t i2cClockSpeed;
    uint32_t gpioSCLPin;
    uint32_t gpioSDAPin;
    gpio_pullup_t gpioPullup;
} I2cDef;

typedef struct {
    const I2cDef *def;                    //< I2C定义
    SemaphoreHandle_t isBusFreeMutex;     //< 保护总线的互斥锁
} I2cDrv;

// c文件中定义的I2C总线
extern I2cDrv deckBus;
extern I2cDrv sensorsBus;

/**
 * 初始化I2C外设，根据静态的I2cDef结构定义。
 */
void i2cdrvInit(I2cDrv *i2c);

void i2cDrvDeInit(I2cDrv *i2c);

/**
 * 通过I2C总线发送或接收消息。
 *
 * 消息通过信号量同步，并使用中断来传输消息。
 *
 * @param i2c      要使用的I2C总线。
 * @param message	 包含所有I2C消息信息的I2cMessage结构体。
 *                 如果出现NACK，则会更改消息状态。
 * @return         如果成功返回true，否则返回false。
 */
bool i2cdrvMessageTransfer(I2cDrv *i2c, I2cMessage *message);


/**
 * 创建要传输的消息
 *
 * @param message       用于填充的消息结构体的指针。
 * @param slaveAddress  I2C从机地址
 * @param direction     i2cWrite或i2cRead
 * @param length        消息的长度
 * @param buffer        指向发送/接收数据的缓冲区的指针
 */
void i2cdrvCreateMessage(I2cMessage *message,
                         uint8_t slaveAddress,
                         I2cDirection direction,
                         uint32_t length,
                         uint8_t *buffer);

/**
 * 创建带有内部“寄存器”地址的消息。将首先写入1或2字节（取决于IsInternal16）的寄存器地址，然后写入/读取数据。
 *
 * @param message       用于填充的消息结构体的指针。
 * @param slaveAddress  I2C从机地址
 * @param IsInternal16  如果为true，则寄存器地址为16位，否则为8位。
 * @param direction     i2cWrite或i2cRead
 * @param length        消息的长度
 * @param buffer        指向发送/接收数据的缓冲区的指针
 */
void i2cdrvCreateMessageIntAddr(I2cMessage *message,
                                uint8_t slaveAddress,
                                bool IsInternal16,
                                uint16_t intAddress,
                                I2cDirection direction,
                                uint32_t length,
                                uint8_t *buffer);

#endif
