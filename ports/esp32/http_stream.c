/**
	******************************************************************************
	* This file is part of the MicroPython project, http://micropython.org/
	* Copyright (C), 2021 -2023, 01studio Tech. Co., Ltd.http://bbs.01studio.org/
	* File Name 				 :	http_stream.c
	* Author						 :	Folktale
	* Version 					 :	v1.0
	* date							 :	2021/8/02
	* Description 			 :	
	******************************************************************************
**/

#include "mpconfigboard.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "esp_system.h"

#include "http_stream.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_http_server.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include <math.h>

#include "py/mphal.h"

#include "sdkconfig.h"

#if (MICROPY_HW_OV2640)

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

httpd_handle_t stream_httpd = NULL;

static uint64_t jpg_len = 100*1024;
static uint8_t *_jpg_buf = NULL;
static char *part_buf=NULL;
	
static esp_err_t stream_handler(httpd_req_t *req)
{
	camera_fb_t *fb = NULL;
	struct timeval _timestamp;
	esp_err_t res = ESP_OK;
	
	size_t _jpg_buf_len = 0;

	if(_jpg_buf == NULL){
		mp_raise_ValueError(MP_ERROR_TEXT("stream_handler malloc _jpg_buf failed"));
	}
	
	res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
	if (res != ESP_OK)
	{
		mp_raise_ValueError(MP_ERROR_TEXT("httpd_resp_set_type failed"));
		return res;
	}
	
	httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
	httpd_resp_set_hdr(req, "X-Framerate", "60");
	while (true)
	{
		fb = esp_camera_fb_get();
		if (!fb){
			mp_raise_ValueError(MP_ERROR_TEXT("stream_handler Camera capture failed"));
			res = ESP_FAIL;
		}else
		{
			_timestamp.tv_sec = fb->timestamp.tv_sec;
			_timestamp.tv_usec = fb->timestamp.tv_usec;
			if (fb->format != PIXFORMAT_JPEG)
			{
				bool jpeg_converted = rgb565_2jpg(fb, 50, jpg_len, _jpg_buf, &_jpg_buf_len);
				esp_camera_fb_return(fb);
				if (!jpeg_converted)
				{
					mp_raise_ValueError(MP_ERROR_TEXT("stream_handler JPEG compression failed"));
					res = ESP_FAIL;
				}
			}
		}
		if (res == ESP_OK)
		{
			res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
		}

		if (res == ESP_OK)
		{
			size_t hlen = snprintf((char *)part_buf, 128, _STREAM_PART, _jpg_buf_len, _timestamp.tv_sec, _timestamp.tv_usec);
			res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
		}

		if (res == ESP_OK){
			res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
		}else{
			break;
		}
		
	}

  return res;
}

void init_httpd_app(uint16_t server_port)
{
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();
	config.max_uri_handlers = 16;
	
	_jpg_buf=(uint8_t *)m_malloc(jpg_len);
	part_buf=(char *)m_malloc(128);

	httpd_uri_t stream_uri = {
			.uri = "/",
			.method = HTTP_GET,
			.handler = stream_handler,
			.user_ctx = NULL};

	config.server_port = server_port;

	if (httpd_start(&stream_httpd, &config) == ESP_OK)
	{
		httpd_register_uri_handler(stream_httpd, &stream_uri);
	}
}
//======================================================

#endif

