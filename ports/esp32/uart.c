/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * Development of the code in this file was sponsored by Microbric Pty Ltd
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdio.h>

#include "driver/uart.h"
#include "soc/uart_periph.h"

#include "py/runtime.h"
#include "py/mphal.h"

#include "esp_system.h"

#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 2, 0)
static const int RX_BUF_SIZE = 1024;
static void rx_task(void *arg)
{
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_0, data, RX_BUF_SIZE, 10 / portTICK_RATE_MS);
		if(rxBytes > 0){
			for(uint32_t i=0 ; i< rxBytes; i++){
				uint8_t c = data[i]; // UART0
				if (c == mp_interrupt_char) {
					mp_sched_keyboard_interrupt();
				} else {
					// this is an inline function so will be in IRAM
					ringbuf_put(&stdin_ringbuf, c);
				}
			}
		}
    }
    free(data);
}

void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
	
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
}
#else
STATIC void uart_irq_handler(void *arg);
void uart_init(void) {
    uart_isr_handle_t handle;
	uart_isr_register(UART_NUM_0, uart_irq_handler, NULL, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, &handle);
    uart_enable_rx_intr(UART_NUM_0);
}

// all code executed in ISR must be in IRAM, and any const data must be in DRAM
STATIC void IRAM_ATTR uart_irq_handler(void *arg) {
    volatile uart_dev_t *uart = &UART0;
    #if CONFIG_IDF_TARGET_ESP32S3
    uart->int_clr.rxfifo_full_int_clr = 1;
    uart->int_clr.rxfifo_tout_int_clr = 1;
    #else
    uart->int_clr.rxfifo_full = 1;
    uart->int_clr.rxfifo_tout = 1;
    uart->int_clr.frm_err = 1;
    #endif
    while (uart->status.rxfifo_cnt) {
        #if CONFIG_IDF_TARGET_ESP32
        uint8_t c = uart->fifo.rw_byte;
        #elif CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
        uint8_t c = READ_PERI_REG(UART_FIFO_AHB_REG(0)); // UART0
        #endif
        if (c == mp_interrupt_char) {
            mp_sched_keyboard_interrupt();
        } else {
            // this is an inline function so will be in IRAM
            ringbuf_put(&stdin_ringbuf, c);
        }
    }
}
#endif
