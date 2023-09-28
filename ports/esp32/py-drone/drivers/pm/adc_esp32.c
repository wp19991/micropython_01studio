/**
 * ESP-Drone Firmware
 *
 * adc.c - 模拟数字转换
 *
 * 这些函数共同用于配置和操作模数转换器（ADC），以及处理ADC采样数据。 adcInit 函数用于初始化ADC，
 * 然后可以使用 analogReadVoltage 函数来从指定引脚读取电压值。
 * 其他函数则提供了ADC初始化和测试的支持。函数之间的调用关系在代码中是线性的，没有函数相互调用。
 */

#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "adc_esp32.h"
#include "config.h"
#include "pm_esplane.h"

#include "esp_log.h"

#define TAG "ADC"

static bool isInit;  // 表示ADC是否已初始化的标志

static esp_adc_cal_characteristics_t *adc_chars;  // 存储ADC特性的指针

static const adc_channel_t channel = ADC_CHANNEL_1;  // 使用的ADC通道（GPIO2，如果是ADC1）
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;  // ADC位宽度，12位

static const adc_atten_t atten = ADC_ATTEN_DB_0;  // ADC信号的衰减，11dB衰减（ADC_ATTEN_DB_11）对应满量程电压3.9V
static const adc_unit_t unit = ADC_UNIT_1;  // 使用的ADC单元

#define DEFAULT_VREF		1100		// 通过adc2_vref_to_gpio()获取更好的估算值
#define NO_OF_SAMPLES		30			// 多次采样

// 检查eFuse中的校准信息是否可用
static void checkEfuse(void)
{
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        ESP_LOGI(TAG,"eFuse Two Point: Supported\n");
    } else {
        ESP_LOGI(TAG,"无法获取eFuse Two Point校准值。将使用默认校准值。\n");
    }
}

// 打印ADC特性值的类型
static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGI(TAG,"Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGI(TAG,"Characterized using eFuse Vref\n");
    } else {
        ESP_LOGI(TAG,"Characterized using Default Vref\n");
    }
}

// 从指定引脚读取电压值
float analogReadVoltage(uint32_t pin)
{
    uint32_t adc_reading = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        if (unit == ADC_UNIT_1) {
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
        } else {
            int raw;
            adc2_get_raw((adc2_channel_t)channel, width, &raw);
            adc_reading += raw;
        }
    }
    adc_reading /= NO_OF_SAMPLES;
    // 将adc_reading转换为以mV为单位的电压值
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    return voltage / 1000.0;  // 返回电压值（以V为单位）
}

// 初始化ADC
void adcInit(void)
{
    if (isInit) {
        return;
    }

    checkEfuse();
    // 配置ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(width);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    // 特性化ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    isInit = true;  // 设置ADC已初始化标志
}

// 测试ADC是否已初始化
bool adcTest(void)
{
    return isInit;
}
