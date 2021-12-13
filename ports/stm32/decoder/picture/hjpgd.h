
#ifndef MICROPY_INCLUDED_STM32_HJPGD_H
#define MICROPY_INCLUDED_STM32_HJPGD_H

#include "py/obj.h"

#include "lib/oofatfs/ff.h"
#include "extmod/vfs.h"
#include "extmod/vfs_fat.h"
#if MICROPY_PY_HJPEG_DECODE

#define JPEG_DMA_INBUF_LEN				4096			//单个DMA IN  BUF的大小 
#define JPEG_DMA_INBUF_NB					10				//DMA IN  BUF的个数
#define JPEG_DMA_OUTBUF_NB				2				//DMA OUT BUF的个数

#define JPEG_STATE_NOHEADER				0					//HEADER未读取,初始状态
#define JPEG_STATE_HEADEROK				1					//HEADER读取成功
#define JPEG_STATE_FINISHED				2					//解码完成
#define JPEG_STATE_ERROR					3					//解码错误

#define JPEG_YCBCR_COLORSPACE		JPEG_CONFR1_COLORSPACE_0
#define JPEG_CMYK_COLORSPACE		JPEG_CONFR1_COLORSPACE

//JPEG数据缓冲结构体
typedef struct
{
    uint8_t sta;			//状态:0,无数据;1,有数据.
    uint8_t *buf;		//JPEG数据缓冲区
    uint16_t size; 		//JPEG数据长度 
}jpeg_databuf_type; 

//jpeg编解码控制结构体
typedef struct
{ 
	JPEG_ConfTypeDef	Conf;             			//当前JPEG文件相关参数
	jpeg_databuf_type inbuf[JPEG_DMA_INBUF_NB];		//DMA IN buf
	jpeg_databuf_type outbuf[JPEG_DMA_OUTBUF_NB];	//DMA OUT buf
	volatile uint8_t inbuf_read_ptr;								//DMA IN buf当前读取位置
	volatile uint8_t inbuf_write_ptr;							//DMA IN buf当前写入位置
	volatile uint8_t indma_pause;								//输入DMA暂停状态标识
	volatile uint8_t outbuf_read_ptr;							//DMA OUT buf当前读取位置
	volatile uint8_t outbuf_write_ptr;							//DMA OUT buf当前写入位置
	volatile uint8_t outdma_pause;								//输入DMA暂停状态标识
	volatile uint8_t state;										//解码状态;0,未识别到Header;1,识别到了Header;2,解码完成;
	uint32_t yuvblk_size;								//YUV输出的字节数,使得完成一次DMA2D YUV2RGB转换,刚好是图片宽度的整数倍
													//YUV420图片,每个像素占1.5个YUV字节,每次输出16行,yuvblk_size=图片宽度*16*1.5
													//YUV422图片,每个像素占2个YUV字节和RGB565一样,每次输出8行,yuvblk_size=图片宽度*8*2
													//YUV444图片,每个像素占3个YUV字节,每次输出8行,yuvblk_size=图片宽度*8*3
	
	uint16_t yuvblk_height;								//每个YUV块输出像素的高度,对于YUV420,为16,对于YUV422/YUV444为8	
	uint16_t yuvblk_curheight;							//当前输出高度,0~分辨率高度
}jpeg_codec_typedef;

extern jpeg_codec_typedef hjpgd;
void JPEG_Core_Destroy(void);
uint8_t JPEG_Core_Init(void);
uint8_t hjpgd_decode(FATFS *fs, const char *filename,uint16_t x,uint16_t y);
//uint8_t hvideo_decode(uint16_t x,uint16_t y,uint8_t *buf,uint32_t len);
//uint8_t mjpeg_init(uint16_t offx,uint16_t offy,uint16_t width,uint16_t height);
#endif
#endif