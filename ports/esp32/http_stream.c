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
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "mpconfigboard.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "esp_system.h"
#include "py/mphal.h"
#include "sdkconfig.h"

#include "http_stream.h"

#if MICROPY_ENABLE_STREAM

#include "esp_timer.h"
#include "esp_http_server.h"

#if MICROPY_ENABLE_SENSOR
#include "esp_camera.h"
#include "img_converters.h"
#endif

#if MICROPY_HW_USB_CAM
#include "usb_uvc_port.h"
#endif

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

static uint8_t stream_type = 0; //0：sensor数据流，1:uvc cam 数据流

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static uint64_t jpg_len = 35*1024;
static uint8_t *_jpg_buf = NULL;
static char *part_buf=NULL;

#if MICROPY_ENABLE_SENSOR
STATIC camera_fb_t *ov2640_fb = NULL;
#endif

#if MICROPY_ENABLE_SENSOR
STATIC uvcam_fb_t *uvc_fb = NULL;
#endif

typedef struct {
	httpd_req_t *req;
	size_t len;
} jpg_chunking_t;

typedef struct {
    size_t size;  //number of values used for filtering
    size_t index; //current value index
    size_t count; //value count
    int sum;
    int *values; //array to be filled with values
} ra_filter_t;

static ra_filter_t ra_filter;

static ra_filter_t *ra_filter_init(ra_filter_t *filter, size_t sample_size)
{
    memset(filter, 0, sizeof(ra_filter_t));

    filter->values = (int *)malloc(sample_size * sizeof(int));

    if (!filter->values) {
        return NULL;
    }

    memset(filter->values, 0, sample_size * sizeof(int));

    filter->size = sample_size;
    return filter;
}
static int ra_filter_run(ra_filter_t *filter, int value)
{
    if (!filter->values) {
        return value;
    }

    filter->sum -= filter->values[filter->index];
    filter->values[filter->index] = value;
    filter->sum += filter->values[filter->index];
    filter->index++;
    filter->index = filter->index % filter->size;

    if (filter->count < filter->size) {
        filter->count++;
    }

    return filter->sum / filter->count;
}

static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len)
{
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if (!index)
    {
        j->len = 0;
    }
    if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK)
    {
        return 0;
    }
    j->len += len;
    return len;
}

static esp_err_t capture_handler(httpd_req_t *req)
{
	esp_err_t res = ESP_OK;
	int64_t fr_start = esp_timer_get_time();
	
	#if MICROPY_HW_USB_CAM
	if(stream_type){
		uvc_fb = uvc_camera_fb_get();
		if (!uvc_fb) {
			mp_raise_ValueError(MP_ERROR_TEXT("Camera capture failed"));
			httpd_resp_send_500(req);
			return ESP_FAIL;
		}
	}else
	#endif
	{
	#if MICROPY_ENABLE_SENSOR
		ov2640_fb = esp_camera_fb_get();
		if (!ov2640_fb)
		{
			mp_raise_ValueError(MP_ERROR_TEXT("Camera capture failed"));
			httpd_resp_send_500(req);
			return ESP_FAIL;
		}
	#endif
	}

	httpd_resp_set_type(req, "image/jpeg");
	httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
	httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

	char ts[32];
	#if MICROPY_HW_USB_CAM
	if(stream_type){
		snprintf(ts, 32, "%ld.%06ld", uvc_fb->timestamp.tv_sec, uvc_fb->timestamp.tv_usec);
	}else
	#endif
	{
	#if MICROPY_ENABLE_SENSOR
		snprintf(ts, 32, "%ld.%06ld", ov2640_fb->timestamp.tv_sec, ov2640_fb->timestamp.tv_usec);
	#endif
	}
	
	httpd_resp_set_hdr(req, "X-Timestamp", (const char *)ts);

	size_t fb_len = 0;
	#if MICROPY_HW_USB_CAM
	if(stream_type){
    res = httpd_resp_send(req, (const char *)uvc_fb->buf, uvc_fb->len);
    uvc_camera_fb_return(uvc_fb);
	}else
	#endif
	{
		#if MICROPY_ENABLE_SENSOR
		jpg_chunking_t jchunk = {req, 0};
		res = frame2jpg_cb(ov2640_fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
		httpd_resp_send_chunk(req, NULL, 0);
		fb_len = jchunk.len;
		esp_camera_fb_return(ov2640_fb);
		#endif
	}

	int64_t fr_end = esp_timer_get_time();
	printf("JPG: %uB %ums\r\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
	return res;
}

static esp_err_t stream_handler(httpd_req_t *req)
{

	struct timeval _timestamp;
	esp_err_t res = ESP_OK;
	size_t _jpg_buf_len = 0;
	
	// static int64_t last_frame = 0;
	// if (!last_frame) {
			// last_frame = esp_timer_get_time();
	// }

	if(_jpg_buf == NULL && stream_type==0){
		mp_raise_ValueError(MP_ERROR_TEXT("stream_handler malloc _jpg_buf failed"));
	}
	
	res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
	
	if (res != ESP_OK){
		mp_raise_ValueError(MP_ERROR_TEXT("httpd_resp_set_type failed"));
		return res;
	}
	
	httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
	httpd_resp_set_hdr(req, "X-Framerate", "60");
	
	while (true)
	{
	#if MICROPY_HW_USB_CAM
		if(stream_type){
			uvc_fb = uvc_camera_fb_get();
			if (!uvc_fb) {
				mp_raise_ValueError(MP_ERROR_TEXT("UVC Camera capture failed"));
				res = ESP_FAIL;
			} else {
				_timestamp.tv_sec = uvc_fb->timestamp.tv_sec;
				_timestamp.tv_usec = uvc_fb->timestamp.tv_usec;
				_jpg_buf_len = uvc_fb->len;
				_jpg_buf = uvc_fb->buf;
			}
		}
		else
	#endif
		{
		#if MICROPY_ENABLE_SENSOR
			ov2640_fb = esp_camera_fb_get();

			_timestamp.tv_sec = ov2640_fb->timestamp.tv_sec;
			_timestamp.tv_usec = ov2640_fb->timestamp.tv_usec;
			if (ov2640_fb->format != PIXFORMAT_JPEG)
			{
				bool jpeg_converted = rgb565_2jpg(ov2640_fb, 50, jpg_len, _jpg_buf, &_jpg_buf_len);
				esp_camera_fb_return(ov2640_fb);
				if (!jpeg_converted)
				{
					mp_raise_ValueError(MP_ERROR_TEXT("stream_handler JPEG compression failed"));
					res = ESP_FAIL;
				}
			}
		#endif
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
		}
		#if MICROPY_HW_USB_CAM
		if(stream_type)
		{
			if (uvc_fb) {
					uvc_camera_fb_return(uvc_fb);
					uvc_fb = NULL;
			} 
		}
		if (res != ESP_OK) {
			break;
		}
		#endif
		// int64_t fr_end = esp_timer_get_time();
		// int64_t frame_time = fr_end - last_frame;
		// last_frame = fr_end;
		// frame_time /= 1000;
		// uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
		// printf("MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps)\r\n",
					 // (uint32_t)(_jpg_buf_len),
					 // (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time,
					 // avg_frame_time, 1000.0 / avg_frame_time
					// );
	}

  return res;
}
static esp_err_t index_handler(httpd_req_t *req)
{
    extern const unsigned char index_uvc_html_gz_start[] asm("_binary_index_uvc_html_gz_start");
    extern const unsigned char index_uvc_html_gz_end[] asm("_binary_index_uvc_html_gz_end");
    size_t index_uvc_html_gz_len = index_uvc_html_gz_end - index_uvc_html_gz_start;

    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    return httpd_resp_send(req, (const char *)index_uvc_html_gz_start, index_uvc_html_gz_len);
}

void init_httpd_app(uint16_t server_port, uint8_t stream_t)
{
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();
	config.max_uri_handlers = 16;
	
stream_type = stream_t;

	httpd_uri_t index_uri = {
			.uri = "/",
			.method = HTTP_GET,
			.handler = index_handler,
			.user_ctx = NULL
	};

	httpd_uri_t capture_uri = {
			.uri = "/capture",
			.method = HTTP_GET,
			.handler = capture_handler,
			.user_ctx = NULL
	};
	if(!stream_type){
		_jpg_buf=(uint8_t *)m_malloc(jpg_len);
	}
	
	part_buf=(char *)m_malloc(128);

	httpd_uri_t stream_uri = {
			.uri = "/stream",
			.method = HTTP_GET,
			.handler = stream_handler,
			.user_ctx = NULL
	};

	config.server_port = server_port;

	//ra_filter_init(&ra_filter, 20);

	if (httpd_start(&camera_httpd, &config) == ESP_OK) {
		httpd_register_uri_handler(camera_httpd, &index_uri);
		httpd_register_uri_handler(camera_httpd, &capture_uri);
	}
	
	config.server_port += 1;
	config.ctrl_port += 1;
	if (httpd_start(&stream_httpd, &config) == ESP_OK)
	{
		httpd_register_uri_handler(stream_httpd, &stream_uri);
	}
}

#endif
//======================================================


