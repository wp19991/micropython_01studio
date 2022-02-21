// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
	******************************************************************************
	* Copyright (C), 2021 -2023, 01studio Tech. Co., Ltd.https://www.01studio.cc/
	* File Name					:	psxcontroller.c
	* Author					:	Folktale
	* Version					:	v1.0
	* date						:	2021/7/1
	* Description				:	
	******************************************************************************
**/

#include "mpconfigboard.h"
#include "esp_system.h"
#include "py/obj.h"
#include <math.h>
#include "py/builtin.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "py/binary.h"
#include "py/objarray.h"
#include "py/mperrno.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "psxcontroller.h"
#include "sdkconfig.h"

// #define DELAY() asm("nop; nop; nop; nop;nop; nop; nop; nop;nop; nop; nop; nop;nop; nop; nop; nop;")

#if MICROPY_ENABLE_PSXCONTROLLER

#include "driver/adc.h"

#include "hal/adc_types.h"
#include "esp_log.h"

#include "py/runtime.h"
#include "py/mphal.h"
#include "modmachine.h"
#include "esp_adc_cal.h"

#define GPIO_PULL_DOWN (1)
#define GPIO_PULL_UP   (2)
#define GPIO_PULL_HOLD (4)

//ADC Attenuation
#define ADC_EXAMPLE_ATTEN           ADC_ATTEN_DB_0

//ADC Calibration
#if CONFIG_IDF_TARGET_ESP32
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_VREF
#elif CONFIG_IDF_TARGET_ESP32S2
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32C3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32S3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP_FIT
#endif

#define NO_OF_SAMPLES	10

static void joy_gpio_init(void)
{
	esp_err_t err;
	
	#if CONFIG_IDF_TARGET_ESP32S2
	adc1_config_width(ADC_WIDTH_BIT_13);
	#elif CONFIG_IDF_TARGET_ESP32C3
	adc1_config_width(ADC_WIDTH_BIT_12);
	#else
	adc1_config_width(ADC_WIDTH_BIT_12);
	#endif
	
	err = adc1_config_channel_atten(PSX_ADCL_LEFTRIGHT_CH, ADC_ATTEN_DB_11);
    if (err != ESP_OK) {
        printf("joy_gpio_init init error1\r\n");
    }
	err = adc1_config_channel_atten(PSX_ADCL_UPDOWN_CH, ADC_ATTEN_DB_11);
    if (err != ESP_OK) {
        printf("joy_gpio_init init error2\r\n");
    }
	
}
static void gpio_global_init(gpio_num_t gpio,gpio_mode_t io_mode,uint8_t mode)
{
	if (rtc_gpio_is_valid_gpio(gpio)) {
		#if !CONFIG_IDF_TARGET_ESP32C3
		rtc_gpio_deinit(gpio);
		#endif
	}
	gpio_pad_select_gpio(gpio);
	gpio_set_direction(gpio, io_mode);
	
	if (mode & GPIO_PULL_DOWN) {
		gpio_pulldown_en(gpio);
	} else {
		gpio_pulldown_dis(gpio);
	}
	if (mode & GPIO_PULL_UP) {
		gpio_pullup_en(gpio);
	} else {
		gpio_pullup_dis(gpio);
	}
	if (mode & GPIO_PULL_HOLD) {
		gpio_hold_en(gpio);
	} else if (GPIO_IS_VALID_OUTPUT_GPIO(gpio)) {
		gpio_hold_dis(gpio);
	}
}
static void input_gpio_init(void)
{
	gpio_global_init(PSX_GPIO_UP,GPIO_MODE_INPUT,GPIO_PULL_UP);
	gpio_global_init(PSX_GPIO_DOWN,GPIO_MODE_INPUT,GPIO_PULL_UP);
	gpio_global_init(PSX_GPIO_LEFT,GPIO_MODE_INPUT,GPIO_PULL_UP);
	gpio_global_init(PSX_GPIO_RIGHT,GPIO_MODE_INPUT,GPIO_PULL_UP);
	gpio_global_init(PSX_GPIO_X,GPIO_MODE_INPUT,GPIO_PULL_UP);
	gpio_global_init(PSX_GPIO_Y,GPIO_MODE_INPUT,GPIO_PULL_UP);
	gpio_global_init(PSX_GPIO_A,GPIO_MODE_INPUT,GPIO_PULL_UP);
	gpio_global_init(PSX_GPIO_B,GPIO_MODE_INPUT,GPIO_PULL_UP);
	gpio_global_init(PSX_GPIO_BACK,GPIO_MODE_INPUT,GPIO_PULL_UP);
	gpio_global_init(PSX_GPIO_START,GPIO_MODE_INPUT,GPIO_PULL_UP);
}


static void joy_adc_read(uint16_t *joy_leftright,uint16_t *joy_updown)
{	
	uint32_t adc_reading1 = 0;
	uint32_t adc_reading2 = 0;
	//Multisampling
	for (int i = 0; i < NO_OF_SAMPLES; i++) {
		adc_reading1 += adc1_get_raw(PSX_ADCL_LEFTRIGHT_CH);
		adc_reading2 += adc1_get_raw(PSX_ADCL_UPDOWN_CH);
	}
	adc_reading1 /= NO_OF_SAMPLES;
	adc_reading2 /= NO_OF_SAMPLES;
	
	*joy_leftright = (uint16_t)adc_reading1;
	*joy_updown = (uint16_t)adc_reading2;
}

/*********************************************************************
 ________________________________________________________________________________________________________
|  15 |   14  |   13  |   12   | 11| 10| 9 | 8 |   7  |   6   |  5  |   4   |   3   |   2    |  1  |  0  |
|-----|-------|-------|--------|---|---|---|---|------|-------|-----|-------|-------|--------|-----|-----|
| UP1 | DOWN1 | LEFT1 | RIGHT1 | A | B | X | Y | BACK | START | UP2 | DOWN2 | LEFT2 | RIGHT2 | TDB | TDB |
|-----|-------|-------|--------|---|---|---|---|------|-------|-----|-------|-------|--------|-----|-----|

*********************************************************************/
static int joy_read_data(void)
{
	int pxsInput = 0xffff;
	uint16_t leftright_voltage=0,updown_voltage=0;

	uint8_t lr_dir = 0,ud_dir = 0;
	
	joy_adc_read(&leftright_voltage,&updown_voltage);
	// printf("leftright_voltage: %dmV,updown_voltage:%dmV\n", leftright_voltage, updown_voltage);

	if(leftright_voltage >= PSX_JOY_RIGHT_VALUE){
		lr_dir = 1;  //right
	}else if(leftright_voltage <= PSX_JOY_LEFT_VALUE){
		lr_dir = 2; //left左
	}
	
	if(updown_voltage >= PSX_JOY_UP_VALUE){
		ud_dir = 1; //up
	}else if(updown_voltage <= PSX_JOY_DOWN_VALUE){
		ud_dir = 2; //down
	}
	//up1
	if(PSX_UP_PRESSED == gpio_get_level(PSX_GPIO_UP) || ud_dir == 1){
		pxsInput &= ~(1<<15);
	}
	//down1
	if(PSX_DOWN_PRESSED == gpio_get_level(PSX_GPIO_DOWN) || ud_dir == 2){
		pxsInput &= ~(1<<14);
	}
	//right1
	if(PSX_RIGHT_PRESSED == gpio_get_level(PSX_GPIO_RIGHT) || lr_dir == 1){
		pxsInput &= ~(1<<12);
	}
	//left1
	if(PSX_LEFT_PRESSED == gpio_get_level(PSX_GPIO_LEFT) || lr_dir == 2){
		pxsInput &= ~(1<<13);
	}
	if(PSX_A_PRESSED == gpio_get_level(PSX_GPIO_A)){
		pxsInput &= ~(1<<11);
	}
	if(PSX_B_PRESSED == gpio_get_level(PSX_GPIO_B)){
		pxsInput &= ~(1<<10);
	}
	if(PSX_X_PRESSED == gpio_get_level(PSX_GPIO_X)){
		pxsInput &= ~(1<<9);
	}
	if(PSX_Y_PRESSED == gpio_get_level(PSX_GPIO_Y)){
		pxsInput &= ~(1<<8);
	}
	if(PSX_BACK_PRESSED == gpio_get_level(PSX_GPIO_BACK)){
		pxsInput &= ~(1<<7);
	}
	if(PSX_START_PRESSED == gpio_get_level(PSX_GPIO_START)){
		pxsInput &= ~(1<<6);
	}

	return pxsInput;

}

int psxReadInput() {
	return joy_read_data();
}

void psxcontrollerInit() {
	input_gpio_init();
	joy_gpio_init();
}

#else
int psxReadInput() {
	return 0xFFFF;
}
void psxcontrollerInit() {

}

#endif