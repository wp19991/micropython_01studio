/**
	******************************************************************************
	* Copyright (C), 2021 -2023, 01studio Tech. Co., Ltd.http://bbs.01studio.org/
	* File Name 				 :	http_stream.h
	* Author						 :	Folktale
	* Version 					 :	v1.0
	* date							 :	2021/8/02
	* Description 			 :	
	******************************************************************************
**/

#ifndef MICROPY_INCLUDED_ESP32_HTTP_STREAM_H
#define MICROPY_INCLUDED_ESP32_HTTP_STREAM_H


#if MICROPY_ENABLE_STREAM

void init_httpd_app(uint16_t server_port, uint8_t stream_t);

#endif

#endif // MICROPY_INCLUDED_ESP32_HTTP_STREAM_H
