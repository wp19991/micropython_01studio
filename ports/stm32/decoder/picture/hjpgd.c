#include "hjpgd.h"
#include "piclib.h"  

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "py/obj.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "systick.h"
#include "py/objstr.h"
#include "py/objlist.h"
#include "py/stream.h"
#include "py/gc.h"

#if MICROPY_PY_HJPEG_DECODE


extern uint8_t _hjpgdbuf;
static uint8_t *hbuf = &_hjpgdbuf;

extern uint8_t _houtbuf;
static uint8_t* outbuf = &_houtbuf;

typedef struct _hjpgd_dma_buf_t {
	uint8_t buf[JPEG_DMA_INBUF_NB][JPEG_DMA_INBUF_LEN] 	__attribute__((aligned(4)));
} hjpgd_dma_buf_t;

static hjpgd_dma_buf_t hdma_buf __attribute__((aligned(64*1024)));


uint16_t imgoffx,imgoffy;	
volatile uint32_t mjpeg_remain_size;	
volatile uint8_t mjpeg_fileover=0;

jpeg_codec_typedef hjpgd;


const uint8_t JPEG_LUM_QuantTable[JPEG_QUANT_TABLE_SIZE] = 
{
	16,11,10,16,24,40,51,61,12,12,14,19,26,58,60,55,
	14,13,16,24,40,57,69,56,14,17,22,29,51,87,80,62,
	18,22,37,56,68,109,103,77,24,35,55,64,81,104,113,92,
	49,64,78,87,103,121,120,101,72,92,95,98,112,100,103,99
};
const uint8_t JPEG_ZIGZAG_ORDER[JPEG_QUANT_TABLE_SIZE]=
{
	0,1,8,16,9,2,3,10,17,24,32,25,18,11,4,5,
	12,19,26,33,40,48,41,34,27,20,13,6,7,14,21,28,
	35,42,49,56,57,50,43,36,29,22,15,23,30,37,44,51,
	58,59,52,45,38,31,39,46,53,60,61,54,47,55,62,63
}; 
// const uint8_t JPEG_ZIGZAG_ORDER[JPEG_QUANT_TABLE_SIZE]=
// {
	// 17, 18, 24, 47, 99, 99, 99, 99,
	// 18, 21, 26, 66, 99, 99, 99, 99,
	// 24, 26, 56, 99, 99, 99, 99, 99,
	// 47, 66, 99, 99, 99, 99, 99, 99,
	// 99, 99, 99, 99, 99, 99, 99, 99,
	// 99, 99, 99, 99, 99, 99, 99, 99,
	// 99, 99, 99, 99, 99, 99, 99, 99,
	// 99, 99, 99, 99, 99, 99, 99, 99
// }; 

static void JPEG_IN_DMA_Init(uint32_t meminaddr,uint32_t meminsize)
{ 
	uint32_t regval=0;
	uint32_t addrmask=0;
	RCC->AHB3ENR|=1<<0;		
	MDMA_Channel7->CCR=0;	
	while(MDMA_Channel7->CCR&0X01);	
	MDMA_Channel7->CIFCR=0X1F;
	MDMA_Channel7->CCR|=1<<2;	
	MDMA_Channel7->CCR|=2<<6;	
	MDMA_Channel7->CBNDTR=meminsize;
	MDMA_Channel7->CDAR=(uint32_t)&JPEG->DIR;
	MDMA_Channel7->CSAR=meminaddr;
	regval=0<<28;
	regval|=1<<25;
	regval|=(32-1)<<18;
	regval|=4<<15;
	regval|=4<<12;	
	regval|=0<<8;
	regval|=2<<6;	
	regval|=0<<4;	
	regval|=0<<2;	
	regval|=2<<0;	
	MDMA_Channel7->CTCR=regval;	
	MDMA_Channel7->CTBR=17<<0;

	addrmask=meminaddr&0XFF000000;
	if(addrmask==0X20000000||addrmask==0)MDMA_Channel7->CTBR|=1<<16;
	
    HAL_NVIC_SetPriority(MDMA_IRQn,1,2);  
    HAL_NVIC_EnableIRQ(MDMA_IRQn); 
}  

static void JPEG_DMA_Stop(void)
{ 
	JPEG->CONFR0&=~(1<<0);	
	JPEG->CR&=~(0X3F<<1);	
	JPEG->CFR=3<<5;			
}  

void JPEG_Core_Destroy(void)
{
	JPEG_DMA_Stop();
}

static void JPEG_OUT_DMA_Init(uint32_t memoutaddr,uint32_t memoutsize)
{ 
	uint32_t regval=0;
	uint32_t addrmask=0;
	RCC->AHB3ENR|=1<<0;		
	MDMA_Channel6->CCR=0;	
	while(MDMA_Channel6->CCR&0X01);
	MDMA_Channel6->CIFCR=0X1F;
	MDMA_Channel6->CCR|=3<<6;	
	MDMA_Channel6->CCR|=1<<2;	
	MDMA_Channel6->CBNDTR=memoutsize;	
	MDMA_Channel6->CDAR=memoutaddr;
	MDMA_Channel6->CSAR=(uint32_t)&JPEG->DOR;	
	regval=0<<28;			
	regval|=1<<25;			
	regval|=(32-1)<<18;		
	regval|=4<<15;		
	regval|=4<<12;				
	regval|=0<<10;
	regval|=0<<6;			
	regval|=2<<4;	
	regval|=2<<2;			
	regval|=0<<0;		
	MDMA_Channel6->CTCR=regval;	
	MDMA_Channel6->CTBR=19<<0;	

	addrmask=memoutaddr&0XFF000000;	
	if(addrmask==0X20000000||addrmask==0)MDMA_Channel6->CTBR|=1<<17;
    
    HAL_NVIC_SetPriority(MDMA_IRQn,1,2);   
    HAL_NVIC_EnableIRQ(MDMA_IRQn);  
}  

void (*jpeg_in_callback)(void);
void (*jpeg_out_callback)(void);
void (*jpeg_eoc_callback)(void);	
void (*jpeg_hdp_callback)(void);	


void MDMA_IRQHandler(void)
{        
	if(MDMA_Channel7->CISR&(1<<1))
 	if(MDMA_Channel7->CISR&(1<<1))
	{
		MDMA_Channel7->CIFCR|=1<<1;	
		JPEG->CR&=~(0X7E);	
      	jpeg_in_callback();	
		JPEG->CR|=3<<5;				

	}
	if(MDMA_Channel6->CISR&(1<<1))	
	{
		MDMA_Channel6->CIFCR|=1<<1; 
		JPEG->CR&=~(0X7E);
      	jpeg_out_callback();
		JPEG->CR|=3<<5;					
	
	} 

}   

void JPEG_IRQHandler(void)
{
	if(JPEG->SR&(1<<6))		
	{ 
		jpeg_hdp_callback();
		JPEG->CR&=~(1<<6);		
		JPEG->CFR|=1<<6;		
	}
	if(JPEG->SR&(1<<5))	 
	{
		JPEG_DMA_Stop();
		jpeg_eoc_callback();
		JPEG->CFR|=1<<5;		
		MDMA_Channel6->CCR&=~(1<<0);
		MDMA_Channel7->CCR&=~(1<<0);
	}
}

uint8_t JPEG_Core_Init(void)
{
	//uint8_t i;
	RCC->AHB3ENR|=1<<5;				//使能硬件jpeg时钟 	
	
	__HAL_RCC_JPGDECEN_CLK_ENABLE();
	
	JPEG->CR=0;				
	JPEG->CR|=1<<0;		
	JPEG->CONFR0&=~(1<<0);	
	JPEG->CR|=1<<13;
	JPEG->CR|=1<<14;		
	JPEG->CFR=3<<5;

	HAL_NVIC_SetPriority(JPEG_IRQn,1,3);   
	HAL_NVIC_EnableIRQ(JPEG_IRQn);   
	JPEG->CONFR1|=1<<8;			

	return 0;
}


static void JPEG_Decode_Init(jpeg_codec_typedef *tjpeg)
{ 
	uint8_t i;
	tjpeg->inbuf_read_ptr=0;
	tjpeg->inbuf_write_ptr=0;
	tjpeg->indma_pause=0;
	tjpeg->outbuf_read_ptr=0;
	tjpeg->outbuf_write_ptr=0;	
	tjpeg->outdma_pause=0;		
	tjpeg->state=JPEG_STATE_NOHEADER;
	for(i=0;i<JPEG_DMA_INBUF_NB;i++)
	{
		tjpeg->inbuf[i].sta=0;
		tjpeg->inbuf[i].size=0;

	}
	for(i=0;i<JPEG_DMA_OUTBUF_NB;i++)
	{
		tjpeg->outbuf[i].sta=0;
		tjpeg->outbuf[i].size=0;
	}		
	MDMA_Channel6->CCR=0;	
	MDMA_Channel7->CCR=0;		
	MDMA_Channel6->CIFCR=0X1F;
	MDMA_Channel7->CIFCR=0X1F;	
	
	JPEG->CONFR1|=1<<3;		
	JPEG->CONFR0&=~(1<<0);	
	JPEG->CR&=~(0X3F<<1);
	JPEG->CR|=1<<13;
	JPEG->CR|=1<<14;	
	JPEG->CR|=1<<6;		
	JPEG->CR|=1<<5;		
	JPEG->CFR=3<<5;		  
	JPEG->CONFR0|=1<<0;	
}

static void JPEG_IN_DMA_Start(void)
{ 
	MDMA_Channel7->CCR|=1<<0;	
}

static void JPEG_OUT_DMA_Start(void)
{
	MDMA_Channel6->CCR|=1<<0;	
}


static void JPEG_IN_DMA_Resume(uint32_t memaddr,uint32_t memlen)
{  
	if(memlen%4)memlen+=4-memlen%4;	
	MDMA_Channel7->CIFCR=0X1F;	
	MDMA_Channel7->CBNDTR=memlen;	
	MDMA_Channel7->CSAR=memaddr;
	MDMA_Channel7->CCR|=1<<0;	
} 

static void JPEG_OUT_DMA_Resume(uint32_t memaddr,uint32_t memlen)
{  
	if(memlen%4)memlen+=4-memlen%4;
	MDMA_Channel6->CIFCR=0X1F;	
	MDMA_Channel6->CBNDTR=memlen;	
	MDMA_Channel6->CDAR=memaddr;
	MDMA_Channel6->CCR|=1<<0;	
}

static void JPEG_Get_Info(jpeg_codec_typedef *tjpeg)
{ 
	uint32_t yblockNb,cBblockNb,cRblockNb; 
	switch(JPEG->CONFR1&0X03)
	{
		case 0://grayscale,1 color component
			tjpeg->Conf.ColorSpace=JPEG_GRAYSCALE_COLORSPACE;
			break;
		case 2://YUV/RGB,3 color component
			tjpeg->Conf.ColorSpace=JPEG_YCBCR_COLORSPACE;
			break;	
		case 3://CMYK,4 color component
			tjpeg->Conf.ColorSpace=JPEG_CMYK_COLORSPACE;
			break;			
	}
	tjpeg->Conf.ImageHeight=(JPEG->CONFR1&0XFFFF0000)>>16;	
	tjpeg->Conf.ImageWidth=(JPEG->CONFR3&0XFFFF0000)>>16;	

	if((tjpeg->Conf.ColorSpace==JPEG_YCBCR_COLORSPACE)||(tjpeg->Conf.ColorSpace==JPEG_CMYK_COLORSPACE))
	{
		yblockNb  =(JPEG->CONFR4&(0XF<<4))>>4;
		cBblockNb =(JPEG->CONFR5&(0XF<<4))>>4;
		cRblockNb =(JPEG->CONFR6&(0XF<<4))>>4;
		if((yblockNb==1)&&(cBblockNb==0)&&(cRblockNb==0))tjpeg->Conf.ChromaSubsampling=JPEG_422_SUBSAMPLING; //16x8 block
		else if((yblockNb==0)&&(cBblockNb==0)&&(cRblockNb==0))tjpeg->Conf.ChromaSubsampling=JPEG_444_SUBSAMPLING;
		else if((yblockNb==3)&&(cBblockNb==0)&&(cRblockNb==0))tjpeg->Conf.ChromaSubsampling = JPEG_420_SUBSAMPLING;
		else tjpeg->Conf.ChromaSubsampling=JPEG_444_SUBSAMPLING; 
	}else tjpeg->Conf.ChromaSubsampling=JPEG_444_SUBSAMPLING;	
	tjpeg->Conf.ImageQuality=0;	
}

uint8_t JPEG_Get_Quality(void)
{
	uint32_t quality=0;
	uint32_t quantRow,quantVal,scale,i,j;
	uint32_t *tableAddress=(uint32_t*)JPEG->QMEM0; 
	i=0;
	while(i<JPEG_QUANT_TABLE_SIZE)
	{
		quantRow=*tableAddress;
		for(j=0;j<4;j++)
		{
			quantVal=(quantRow>>(8*j))&0xFF;
			if(quantVal==1)quality+=100;	//100% 
			else
			{
				scale=(quantVal*100)/((uint32_t)JPEG_LUM_QuantTable[JPEG_ZIGZAG_ORDER[i+j]]);
				if(scale<=100)quality+=(200-scale)/2;  
				else quality+=5000/scale;      
			}      
		} 
		i+=4;
		tableAddress++;    
	} 
	return (quality/((uint32_t)64));   
}


static uint8_t JPEG_DMA2D_YUV2RGB_Conversion(jpeg_codec_typedef *tjpeg,uint32_t pdst)
{ 
	uint32_t regval=0;
	uint32_t cm=0;		
	uint32_t destination=0,timeout=0; 

	if(tjpeg->Conf.ChromaSubsampling==JPEG_420_SUBSAMPLING)cm=DMA2D_CSS_420;	
	if(tjpeg->Conf.ChromaSubsampling==JPEG_422_SUBSAMPLING)cm=DMA2D_CSS_422;
	else if(tjpeg->Conf.ChromaSubsampling==JPEG_444_SUBSAMPLING)cm=DMA2D_NO_CSS;
	destination=(uint32_t)pdst+(tjpeg->yuvblk_curheight*tjpeg->Conf.ImageWidth)*2;	

	RCC->AHB3ENR|=1<<4;		
	RCC->AHB3RSTR|=1<<4;
	RCC->AHB3RSTR&=~(1<<4);
	DMA2D->CR&=~(1<<0);
	DMA2D->CR=1<<16;	

	DMA2D->OPFCCR=2<<0;	

	if(lcddev.type == 3 && lcddev.dir == 1){
		DMA2D->OPFCCR |= 1<<21;
	}

	DMA2D->OOR=0;
	DMA2D->IFCR|=1<<1;		
	regval=11<<0;			

	regval|=cm<<18;	
	DMA2D->FGPFCCR=regval;	
	DMA2D->FGOR=0;			
	DMA2D->NLR=tjpeg->yuvblk_height|(tjpeg->Conf.ImageWidth<<16);	
	DMA2D->OMAR=destination;
	DMA2D->FGMAR=(uint32_t)tjpeg->outbuf[tjpeg->outbuf_read_ptr].buf;	
	DMA2D->CR|=1<<0;			
	while((DMA2D->ISR&(1<<1))==0)	
	{
		timeout++;
		if(timeout>0XFFFFFF){
			break;	
		}
	} 
	tjpeg->yuvblk_curheight+=tjpeg->yuvblk_height;
   
   RCC->AHB3RSTR|=1<<4;   
   RCC->AHB3RSTR&=~(1<<4); 
	if(timeout>0XFFFFFF) return 1;  	
	return 0;
}
//--------------------------------------------------------------------

static void jpeg_dma_in_callback(void)
{ 
	hjpgd.inbuf[hjpgd.inbuf_read_ptr].sta=0;	
	hjpgd.inbuf[hjpgd.inbuf_read_ptr].size=0;	

	hjpgd.inbuf_read_ptr++;		

	if(hjpgd.inbuf_read_ptr>=JPEG_DMA_INBUF_NB)	hjpgd.inbuf_read_ptr=0;
	if(hjpgd.inbuf[hjpgd.inbuf_read_ptr].sta==0)
	{
 		hjpgd.indma_pause=1;			
	}else									
	{
		JPEG_IN_DMA_Resume((uint32_t)hjpgd.inbuf[hjpgd.inbuf_read_ptr].buf,hjpgd.inbuf[hjpgd.inbuf_read_ptr].size);
	}
}


void jpeg_dma_out_callback(void)
{	  
	uint32_t *pdata=0; 
	hjpgd.outbuf[hjpgd.outbuf_write_ptr].sta=1;
	hjpgd.outbuf[hjpgd.outbuf_write_ptr].size=hjpgd.yuvblk_size-(MDMA_Channel6->CBNDTR&0X1FFFF);	
	if(hjpgd.state==JPEG_STATE_FINISHED)
	{
		pdata=(uint32_t*)(hjpgd.outbuf[hjpgd.outbuf_write_ptr].buf+hjpgd.outbuf[hjpgd.outbuf_write_ptr].size);
		while(JPEG->SR&(1<<4))
		{
			*pdata=JPEG->DOR;
			pdata++;
			hjpgd.outbuf[hjpgd.outbuf_write_ptr].size+=4; 
		}
	}  
	hjpgd.outbuf_write_ptr++;		
	if(hjpgd.outbuf_write_ptr>=JPEG_DMA_OUTBUF_NB)hjpgd.outbuf_write_ptr=0;
	if(hjpgd.outbuf[hjpgd.outbuf_write_ptr].sta==1)
	{ 
 		hjpgd.outdma_pause=1;		
	}else									
	{
		JPEG_OUT_DMA_Resume((uint32_t)hjpgd.outbuf[hjpgd.outbuf_write_ptr].buf,hjpgd.yuvblk_size);
	}
}


static void jpeg_endofcovert_callback(void)
{ 
	hjpgd.state=JPEG_STATE_FINISHED;
}

static void jpeg_hdrover_callback(void)
{
	uint8_t i=0;
	hjpgd.state=JPEG_STATE_HEADEROK;		
	JPEG_Get_Info(&hjpgd);		
	picinfo.ImgWidth=hjpgd.Conf.ImageWidth;
	picinfo.ImgHeight=hjpgd.Conf.ImageHeight; 

	switch(hjpgd.Conf.ChromaSubsampling)
	{
		case JPEG_420_SUBSAMPLING: 
			hjpgd.yuvblk_size=24*hjpgd.Conf.ImageWidth;
			hjpgd.yuvblk_height=16;					
			break;
		case JPEG_422_SUBSAMPLING:
			hjpgd.yuvblk_size=16*hjpgd.Conf.ImageWidth;	
			hjpgd.yuvblk_height=8;		
			break;
		case JPEG_444_SUBSAMPLING:
			hjpgd.yuvblk_size=24*hjpgd.Conf.ImageWidth;
			hjpgd.yuvblk_height=8;				
			break;
	}  
	hjpgd.yuvblk_curheight=0;
	for(i=0;i<JPEG_DMA_OUTBUF_NB;i++)
	{
		hjpgd.outbuf[i].buf = (uint8_t*)((uint32_t)outbuf+(i*hjpgd.yuvblk_size*hjpgd.yuvblk_height+32));
		if(hjpgd.outbuf[i].buf==NULL)		
		{
			hjpgd.state=JPEG_STATE_ERROR;
		}	
	}	

	if(hjpgd.outbuf[1].buf!=NULL)	
	{
		JPEG_OUT_DMA_Init((uint32_t)hjpgd.outbuf[0].buf,hjpgd.yuvblk_size);
		JPEG_OUT_DMA_Start();			
	}
} 
	
static void ltdc_full_jpgd(uint16_t x,uint16_t y,uint16_t width,uint16_t height,uint16_t *color)	 
{  
if(x >= lcddev.width || y >= lcddev.height) return;

	uint32_t offline =0 ;
	uint32_t ltdc_addr = 0;
	uint32_t ltdc_whlen = 0;
	if(lcddev.type == 2) 
	{
		offline = lcddev.x_pixel - width;
		ltdc_addr = ((uint32_t)ltdc_framebuf[ltdcdev.layer] + ltdcdev.pixsize*(lcddev.x_pixel*y+x));
		ltdc_whlen = ((uint32_t)width<<16)|((uint32_t)height);
	}else if(lcddev.type == 3)  //7寸
	{
		offline = lcddev.x_pixel - width;
		ltdc_addr = ((uint32_t)ltdc_framebuf[ltdcdev.layer] + ltdcdev.pixsize*(lcddev.x_pixel*y+x));
		ltdc_whlen = ((uint32_t)width<<16)|((uint32_t)height);
	}

	DMA2D->CR=0<<16;				
	DMA2D->FGPFCCR=ltdcdev.ltdc_format;//LTDC_PIXEL_FORMAT_RGB565;	
	DMA2D->FGOR=0;					
	DMA2D->OOR=offline;		
	DMA2D->CR&=~(1<<0);		
	DMA2D->FGMAR=(uint32_t)color;	
	DMA2D->OMAR=ltdc_addr;			
	DMA2D->NLR=ltdc_whlen;	
	DMA2D->CR|=1<<0;				
	while((DMA2D->ISR&(1<<1))==0);
	DMA2D->IFCR|=1<<1;				
}

uint8_t hjpgd_decode(FATFS *fs, const char *filename,uint16_t x,uint16_t y)
{
	UINT rb;

	volatile uint32_t timecnt=0;
	uint8_t fileover=0;
	uint8_t i=0;
	uint8_t res;  

	for(i=0;i<JPEG_DMA_INBUF_NB;i++){
		hjpgd.inbuf[i].buf = (uint8_t *)&hdma_buf.buf[i][0];
		memset(hjpgd.inbuf[i].buf,0,JPEG_DMA_INBUF_NB);
	} 
	
	res=JPEG_Core_Init();			
	if(res)return 1;
	
	FIL *f_hjpeg=(FIL*)m_malloc(sizeof(FIL));
	
	if(f_hjpeg==NULL)
	{
		printf("f_hjpeg m_malloc error\r\n");
		return 1;	
	}
	
	if(f_open(fs,f_hjpeg,(const TCHAR*)filename,FA_READ)!=FR_OK)	
  {
		JPEG_Core_Destroy();
		return 2;
	} 

	uint16_t *rgb565buf=( uint16_t *)hbuf;

	JPEG_Decode_Init(&hjpgd);						
	for(i=0;i<JPEG_DMA_INBUF_NB;i++)
	{
 		res=f_read(f_hjpeg,hjpgd.inbuf[i].buf,JPEG_DMA_INBUF_LEN,&rb);
		
		if(res==FR_OK&&rb)
		{
			hjpgd.inbuf[i].size=rb;		
			hjpgd.inbuf[i].sta=1;		
		}
		if(res != FR_OK){
			printf("f_read error\r\n");
		}
		if(rb==0)break; 
	}
	JPEG_IN_DMA_Init((uint32_t)hjpgd.inbuf[0].buf,hjpgd.inbuf[0].size);	

	jpeg_in_callback=jpeg_dma_in_callback;	
	jpeg_out_callback=jpeg_dma_out_callback; 	
	jpeg_eoc_callback=jpeg_endofcovert_callback;
	jpeg_hdp_callback=jpeg_hdrover_callback; 
 	JPEG_IN_DMA_Start();				

	while(1)
	{ 
		if(hjpgd.inbuf[hjpgd.inbuf_write_ptr].sta==0&&fileover==0)	
		{
			res=f_read(f_hjpeg,hjpgd.inbuf[hjpgd.inbuf_write_ptr].buf,JPEG_DMA_INBUF_LEN,&rb);

 			if(res==FR_OK&&rb){
				hjpgd.inbuf[hjpgd.inbuf_write_ptr].size=rb;	
				hjpgd.inbuf[hjpgd.inbuf_write_ptr].sta=1;

			}else if(res!=FR_OK){
				printf("f read error\r\n");
			}else if(rb==0){
				timecnt=0;	
				fileover=1;

			}
			if(hjpgd.indma_pause==1&&hjpgd.inbuf[hjpgd.inbuf_read_ptr].sta==1)
			{
 				JPEG_IN_DMA_Resume((uint32_t)hjpgd.inbuf[hjpgd.inbuf_read_ptr].buf,hjpgd.inbuf[hjpgd.inbuf_read_ptr].size);
 				hjpgd.indma_pause=0;
			}
			hjpgd.inbuf_write_ptr++;
			if(hjpgd.inbuf_write_ptr>=JPEG_DMA_INBUF_NB)hjpgd.inbuf_write_ptr=0;
			
		}
		if(hjpgd.outbuf[hjpgd.outbuf_read_ptr].sta==1)	
		{
			SCB_CleanInvalidateDCache();				
			if(JPEG_DMA2D_YUV2RGB_Conversion(&hjpgd,(uint32_t)rgb565buf))
			{
				printf("time out\r\n");
			}
			SCB_CleanInvalidateDCache();		
			hjpgd.outbuf[hjpgd.outbuf_read_ptr].sta=0;	
			hjpgd.outbuf[hjpgd.outbuf_read_ptr].size=0;	

			hjpgd.outbuf_read_ptr++;
			if(hjpgd.outbuf_read_ptr>=JPEG_DMA_OUTBUF_NB)hjpgd.outbuf_read_ptr=0;	
			if(hjpgd.yuvblk_curheight>=hjpgd.Conf.ImageHeight)break;			
		}else if(hjpgd.outdma_pause==1&&hjpgd.outbuf[hjpgd.outbuf_write_ptr].sta==0)
		{
 			JPEG_OUT_DMA_Resume((uint32_t)hjpgd.outbuf[hjpgd.outbuf_write_ptr].buf,hjpgd.yuvblk_size);
 			hjpgd.outdma_pause=0;
		}
		timecnt++;  
		if(hjpgd.state==JPEG_STATE_ERROR)	
		{
			res=2;
			break;
		} 
		if(fileover)	
		{
			if(hjpgd.state==JPEG_STATE_NOHEADER){
				break;	
			}
			if(timecnt>0XFFFFFF)
			{
				printf("timecnt\r\n");
				break;
			}
		}
	}

	if(hjpgd.state==JPEG_STATE_FINISHED)	
	{
		picinfo.S_Height=hjpgd.Conf.ImageHeight;
		picinfo.S_Width=hjpgd.Conf.ImageWidth;

		if((lcddev.type == 2 && lcddev.dir == 4) || (lcddev.type == 3 && lcddev.dir == 1)){
			ltdc_full_jpgd(x,y,hjpgd.Conf.ImageWidth,hjpgd.Conf.ImageHeight,rgb565buf);
		}else{
			pic_phy.fillcolor(x,y,hjpgd.Conf.ImageWidth,hjpgd.Conf.ImageHeight,(uint16_t*)rgb565buf);//color fill
		}
	}  
	f_close(f_hjpeg);			
	m_free(f_hjpeg);
	JPEG_Core_Destroy(); 
	return res;
}

#endif








