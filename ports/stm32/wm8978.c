/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
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
#include <string.h>
#include "py/mphal.h"
#include "py/runtime.h"
#include "pin.h"
#include "i2c.h"
#include "irq.h"
#include "genhdr/pins.h"
#include "dma.h"
#include "lib/oofatfs/ff.h"
#include "extmod/vfs.h"
#include "extmod/vfs_fat.h"
#include <errno.h> // used by mp_is_nonblocking_error
#include "py/nlr.h"
#include "py/gc.h"
#include "lib/utils/pyexec.h"
#include "gccollect.h"
#include "py/ringbuf.h"
#include "py/objstr.h"
#include "py/objlist.h"
#include "bufhelper.h"

#include "led.h"

#if MICROPY_HW_WM8978
#include "global.h"
#include "mp3play.h"

#include "wm8978.h"
//======================================================
STATIC volatile uint8_t wavtransferend=0; 
STATIC volatile uint8_t wavwitchbuf=0;		
volatile uint8_t Is_FileReadOk=1;	

I2S_HandleTypeDef I2S2_Handler;		
DMA_HandleTypeDef I2S2_TXDMA_Handler;   
DMA_HandleTypeDef I2S2_RXDMA_Handler;	

STATIC const uint16_t i2splaybuf[2]={0X0000,0X0000};

uint8_t *i2srecbuf1;
uint8_t *i2srecbuf2;

//--------------------------------------------

#define WM8978_REG_LEN	58

STATIC uint16_t WM8978_REG_VAL[WM8978_REG_LEN] = {

	0X0000,0X0000,0X0000,0X0000,0X0050,0X0000,0X0140,0X0000,
	0X0000,0X0000,0X0000,0X00FF,0X00FF,0X0000,0X0100,0X00FF,
	0X00FF,0X0000,0X012C,0X002C,0X002C,0X002C,0X002C,0X0000,
	0X0032,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,
	0X0038,0X000B,0X0032,0X0000,0X0008,0X000C,0X0093,0X00E9,
	0X0000,0X0000,0X0000,0X0000,0X0003,0X0010,0X0010,0X0100,
	0X0100,0X0002,0X0001,0X0001,0X0039,0X0039,0X0039,0X0039,
	0X0001,0X0001
};

#define I2C_TIMEOUT_MS (50)

#define WM8978_ADDR				(0x1A)  //0x1a

//==============================================================================================================
typedef struct _audio_wm8978_obj_t {
    mp_obj_base_t base;
    mp_obj_t callback;
} wm8978_obj_t;

//=========================================================

STATIC int wm8978_write_reg(uint8_t reg, uint16_t dat)
{
	int ret;
	uint8_t data[2];
	
	data[0] = (reg<<1)|((dat>>8)&0X01);
	data[1] = (dat&0xff);
	ret = i2c_writeto(I2C1, WM8978_ADDR, data, 2, true);
	WM8978_REG_VAL[reg] = dat;  //save wm8978 reg val
	return ret;
}

 void WM8978_I2S_CFG(uint8_t fmt,uint8_t len)
{
	fmt&=0X03;
	len&=0X03;//限定范围
	wm8978_write_reg(4,(fmt<<3)|(len<<5));	//R4,WM8978工作模式设置	
}	

//=========================================================
STATIC uint16_t wm8978_read_reg(uint8_t reg) {

	reg %= WM8978_REG_LEN;
	return WM8978_REG_VAL[reg];
}
//WM8978 L2/R2(也就是Line In)增益设置(L2/R2-->ADC输入部分的增益)
//gain:0~7,0表示通道禁止,1~7,对应-12dB~6dB,3dB/Step

STATIC void linein_gain(uint8_t gain)
{
	uint16_t regval;
	gain&=0X07;
	regval=wm8978_read_reg(47);	//读取R47
	regval&=~(7<<4);			//清除原来的设置 
 	wm8978_write_reg(47,regval|gain<<4);//设置R47
	regval=wm8978_read_reg(48);	//读取R48
	regval&=~(7<<4);			//清除原来的设置 
 	wm8978_write_reg(48,regval|gain<<4);//设置R48
} 
STATIC void wm8978_aux_gain(uint8_t gain)
{
	uint16_t regval;
	gain&=0X07;
	regval=wm8978_read_reg(47);	//读取R47
	regval&=~(7<<0);			//清除原来的设置 
 	wm8978_write_reg(47,regval|gain<<0);//设置R47
	regval=wm8978_read_reg(48);	//读取R48
	regval&=~(7<<0);			//清除原来的设置 
 	wm8978_write_reg(48,regval|gain<<0);//设置R48
}  
//--------------------------------------------------------
//WM8978 DAC/ADC配置
//adcen:adc使能(1)/关闭(0)
//dacen:dac使能(1)/关闭(0)
void wm8978_adda_cfg(uint8_t dacen,uint8_t adcen)
{
	uint16_t regval;
	regval=wm8978_read_reg(3);	//读取R3
	if(dacen)regval|=3<<0;		//R3最低2个位设置为1,开启DACR&DACL
	else regval&=~(3<<0);		//R3最低2个位清零,关闭DACR&DACL.
	wm8978_write_reg(3,regval);	//设置R3
	regval=wm8978_read_reg(2);	//读取R2
	if(adcen)regval|=3<<0;		//R2最低2个位设置为1,开启ADCR&ADCL
	else regval&=~(3<<0);		//R2最低2个位清零,关闭ADCR&ADCL.
	wm8978_write_reg(2,regval);	//设置R2	
}
//---------------------------------------------------------------------------------------
//WM8978 输入通道配置 
//micen:MIC开启(1)/关闭(0)
//lineinen:Line In开启(1)/关闭(0)
//auxen:aux开启(1)/关闭(0) 
void wm8978_input_cfg(uint8_t micen,uint8_t lineinen,uint8_t auxen)
{
	uint16_t regval;  
	regval=wm8978_read_reg(2);	//读取R2
	if(micen)regval|=3<<2;		//开启INPPGAENR,INPPGAENL(MIC的PGA放大)
	else regval&=~(3<<2);		//关闭INPPGAENR,INPPGAENL.
 	wm8978_write_reg(2,regval);	//设置R2 
	
	regval=wm8978_read_reg(44);	//读取R44
	if(micen)regval|=3<<4|3<<0;	//开启LIN2INPPGA,LIP2INPGA,RIN2INPPGA,RIP2INPGA.
	else regval&=~(3<<4|3<<0);	//关闭LIN2INPPGA,LIP2INPGA,RIN2INPPGA,RIP2INPGA.
	wm8978_write_reg(44,regval);//设置R44
	
	if(lineinen)linein_gain(5);//LINE IN 0dB增益
	else linein_gain(0);	//关闭LINE IN
	if(auxen)wm8978_aux_gain(7);//AUX 6dB增益
	else wm8978_aux_gain(0);	//关闭AUX输入  
} 
//---------------------------------------------------------------------------------------
//WM8978 输出配置 
//dacen:DAC输出(放音)开启(1)/关闭(0)
//bpsen:Bypass输出(录音,包括MIC,LINE IN,AUX等)开启(1)/关闭(0) 
void wm8978_output_cfg(uint8_t dacen,uint8_t bpsen)
{
	uint16_t regval=0;
	if(dacen)regval|=1<<0;	//DAC输出使能
	if(bpsen)
	{
		regval|=1<<1;		//BYPASS使能
		regval|=5<<2;		//0dB增益
	} 
	wm8978_write_reg(50,regval);//R50设置
	wm8978_write_reg(51,regval);//R51设置 
}
//---------------------------------------------------------------------------------------

//WM8978 MIC增益设置(不包括BOOST的20dB,MIC-->ADC输入部分的增益)
//gain:0~63,对应-12dB~35.25dB,0.75dB/Step
STATIC void wm8978_mic_gain(uint8_t gain)
{
	gain&=0X3F;
	wm8978_write_reg(45,gain);		//R45,左通道PGA设置 
	wm8978_write_reg(46,gain|1<<8);	//R46,右通道PGA设置
}

//设置耳机左右声道音量
//voll:左声道音量(0~63)
//volr:右声道音量(0~63)
void wm8978_hspk_vol(uint8_t voll,uint8_t volr)
{
	voll&=0X3F;
	volr&=0X3F;//限定范围
	if(voll==0)voll|=1<<6;//音量为0时,直接mute
	if(volr==0)volr|=1<<6;//音量为0时,直接mute 
	wm8978_write_reg(52,voll);			//R52,耳机左声道音量设置
	wm8978_write_reg(53,volr|(1<<8));	//R53,耳机右声道音量设置,同步更新(HPVU=1)
}
//-----------------------------------------------------------------------------------
//设置喇叭音量
//voll:左声道音量(0~63) 
void wm8978_spk_vol(uint8_t volx)
{ 
	volx&=0X3F;//限定范围
	if(volx==0)volx|=1<<6;				//音量为0时,直接mute 
 	wm8978_write_reg(54,volx);			//R54,喇叭左声道音量设置
	wm8978_write_reg(55,volx|(1<<8));	//R55,喇叭右声道音量设置,同步更新(SPKVU=1)	
} 
//-----------------------------------------------------------------------------------
//初始化wm8978
void wm8978_init(void) {
	GPIO_InitTypeDef GPIO_Initure;

	__HAL_RCC_SPI2_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_Initure.Pin=GPIO_PIN_12|GPIO_PIN_13;  
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;
	GPIO_Initure.Pull=GPIO_PULLUP;    
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;
	GPIO_Initure.Alternate=GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);

	GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6; 
	HAL_GPIO_Init(GPIOC,&GPIO_Initure);

	GPIO_Initure.Pin=GPIO_PIN_2; 
	GPIO_Initure.Alternate=GPIO_AF6_I2S2ext;
	HAL_GPIO_Init(GPIOC,&GPIO_Initure); 
	

    // start the I2C bus in master mode
    i2c_init(I2C1, MICROPY_HW_I2C1_SCL, MICROPY_HW_I2C1_SDA, 400000, I2C_TIMEOUT_MS);

    int ret;
    for (int i = 0; i < 4; i++) {
        ret = i2c_writeto(I2C1, WM8978_ADDR, NULL, 0, true);
        if (ret == 0) {
            break;
        }
    }
    if (ret != 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("wm8978 not found"));
    }
    ret = wm8978_write_reg(0 , 0); // 软件复位wm8978

	if(ret != 2) {
		mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("wm8978 reset err"));
	}
	mp_hal_delay_ms(30);

	wm8978_write_reg(1,0X1B);	//R1,MICEN设置为1(MIC使能),BIASEN设置为1(模拟器工作),VMIDSEL[1:0]设置为:11(5K)
	wm8978_write_reg(2,0X1B0);	//R2,ROUT1,LOUT1输出使能(耳机可以工作),BOOSTENR,BOOSTENL使能
	wm8978_write_reg(3,0X6C);	//R3,LOUT2,ROUT2输出使能(喇叭工作),RMIX,LMIX使能	
	wm8978_write_reg(6,0);		//R6,MCLK由外部提供
	wm8978_write_reg(43,1<<4);	//R43,INVROUT2反向,驱动喇叭
	wm8978_write_reg(47,1<<8);	//R47设置,PGABOOSTL,左通道MIC获得20倍增益
	wm8978_write_reg(48,1<<8);	//R48设置,PGABOOSTR,右通道MIC获得20倍增益
	wm8978_write_reg(49,3<<1);	//R49,SPEAKER BOOST使能，TSDEN,开启过热保护 
	wm8978_write_reg(10,1<<3);	//R10,SOFTMUTE关闭,128x采样,最佳SNR 
	wm8978_write_reg(14,1<<3);	//R14,ADC 128x采样率

	wm8978_adda_cfg(1,0);			//开启DAC
	//wm8978_adda_cfg(0,0);
	wm8978_input_cfg(0,0,0);		//关闭输入通道
	wm8978_output_cfg(1,0);			//开启DAC输出  
	wm8978_hspk_vol(50,50);
	wm8978_spk_vol(50);

}

//============================================================================================

FIL audio_rec;		//录音文件	

__wavctrl wavctrl;		//WAV控制结构体
__WaveHeader *wavhead; //录音

//音乐播放控制器
__audiodev audiodev;
uint32_t  wavsize = 0;		//wav数据大小(字节数,不包括文件头!!)
uint8_t 	rec_sta=0;		//录音状态
					//[7]:0,没有开启录音;1,已经开启录音;
					//[6]:1,停止录音
					//[5:1]:保留
					//[0]:0,正在录音;1,暂停录音;
					
 //===============================================================
 //I2S DMA回调函数指针
 void (*i2s_tx_callback)(void);  //TX回调函数 
 void (*i2s_rx_callback)(void);  //RX回调函数
//===============================================================
 //关闭音频播放
void wav_stop(void)
 {
	 audiodev.status=0;
	 __HAL_DMA_DISABLE(&I2S2_TXDMA_Handler);
 } 
 //暂停播放
void wav_pause(void)
 {
	 if(audiodev.status&0X01)
		 audiodev.status&=~(1<<0);
	 else 
		 audiodev.status|=0X01;  
 }
//---------------------------------------------------------------
 void wav_get_curtime(FIL*fx,__wavctrl *wavx)
 {
	 long long fpos;	 
	 wavx->totsec=wavx->datasize/(wavx->bitrate/8); 
	 fpos=fx->fptr-wavx->datastart; 				 
	 wavx->cursec=fpos*wavx->totsec/wavx->datasize;  
 }
 //==================================================================

void wav_i2s_dma_tx_callback(void) {	 
	 uint32_t bread;
	 uint32_t i;

	 if(DMA1_Stream4->CR&(1<<19))
	 {
			if((audiodev.status)==3){
				 f_read(audiodev.file,audiodev.i2sbuf1,WAV_I2S_TX_DMA_BUFSIZE,(UINT*)&bread);
			 }else if((audiodev.status&0X01)==0) {
					for(i=0;i<WAV_I2S_TX_DMA_BUFSIZE;i++) { //暂停
						audiodev.i2sbuf1[i]=0;//填充0
					}
			}
	 }else 
	 {
		 if((audiodev.status)==3){
				f_read(audiodev.file,audiodev.i2sbuf2,WAV_I2S_TX_DMA_BUFSIZE,(UINT*)&bread);
		 	}else if((audiodev.status&0X01)==0) {
				 for(i=0;i<WAV_I2S_TX_DMA_BUFSIZE;i++) { //暂停
					 audiodev.i2sbuf2[i]=0;//填充0
				 }
		 }

	 }
		if((audiodev.status)==3 && bread < WAV_I2S_TX_DMA_BUFSIZE) {
		 for(i=bread;i<WAV_I2S_TX_DMA_BUFSIZE-bread;i++) audiodev.i2sbuf2[i]=0; 
		 	wav_stop();
			wm8978_adda_cfg(0,0);
		 	f_close(audiodev.file);
			m_free(audiodev.i2sbuf1);
			m_free(audiodev.i2sbuf2);
			m_free(audiodev.file);
		}

	wav_get_curtime(audiodev.file , &wavctrl);
}

 //录音指定时间=数据大小(data)/(16000*4)
 void rec_i2s_dma_rx_callback(void) 
 { 
	 UINT bw;
	 uint8_t res = 0;
	 if(rec_sta&0X80)//录音模式
	 {	
		 if(DMA1_Stream3->CR&(1<<19))
		 {
			 res=f_write(&audio_rec,i2srecbuf1,I2S_RX_DMA_BUF_SIZE,&bw);
			 if(res)	printf("write error:%d\r\n",res);

		 }else 
		 {
			res=f_write(&audio_rec,i2srecbuf2,I2S_RX_DMA_BUF_SIZE,&bw);
			 if(res)	printf("write error:%d\r\n",res);
		 }
		 wavsize+=I2S_RX_DMA_BUF_SIZE;

		 if(rec_sta&0X40)
   //   if(wavsize>=wavhead->fmt.SampleRate*4*10)
      {
				__HAL_DMA_DISABLE(&I2S2_RXDMA_Handler); 			
				rec_sta=0; 
				wavhead->riff.ChunkSize=wavsize+36;	
				wavhead->data.ChunkSize=wavsize; 
				f_lseek(&audio_rec,0);	
				f_write(&audio_rec,(const void*)wavhead,sizeof(__WaveHeader),&bw);//写入头数据
				f_close(&audio_rec);
				wavsize=0;
	
				m_free(i2srecbuf1);
				m_free(i2srecbuf2);

		 }
		 
	 } 
	
 } 
 //--------------------------------------------------------------------
void audio_init(uint32_t I2S_Standard,uint32_t I2S_Mode,uint32_t I2S_Clock_Polarity,uint32_t I2S_DataFormat)
{
	I2S2_Handler.Instance=SPI2;
	I2S2_Handler.Init.Mode=I2S_Mode;
	I2S2_Handler.Init.Standard=I2S_Standard;	
	I2S2_Handler.Init.DataFormat=I2S_DataFormat;
	I2S2_Handler.Init.MCLKOutput=I2S_MCLKOUTPUT_ENABLE;	
	I2S2_Handler.Init.AudioFreq=I2S_AUDIOFREQ_DEFAULT;
	I2S2_Handler.Init.CPOL=I2S_Clock_Polarity;
	I2S2_Handler.Init.ClockSource=I2S_CLOCK_PLL;

	I2S2_Handler.Init.FullDuplexMode=I2S_FULLDUPLEXMODE_ENABLE;	
	HAL_I2S_Init(&I2S2_Handler); 

	SPI2->CR2|=1<<1;
	I2S2ext->CR2|=1<<0;
	__HAL_I2S_ENABLE(&I2S2_Handler);
	__HAL_I2SEXT_ENABLE(&I2S2_Handler);	
	
}
void auido_deinit(void) {
	HAL_I2S_DeInit(&I2S2_Handler);
	__HAL_I2SEXT_DISABLE(&I2S2_Handler);
	__HAL_DMA_DISABLE(&I2S2_RXDMA_Handler); 
	__HAL_DMA_DISABLE(&I2S2_TXDMA_Handler); 

	HAL_DMA_DeInit(&I2S2_TXDMA_Handler); 
	HAL_DMA_Init(&I2S2_RXDMA_Handler);
}

const uint16_t I2S_PSC_TBL[][5]=
{
	{800 ,256,5,12,1},		//8Khz采样率
	{1102,429,4,19,0},		//11.025Khz采样率 
	{1600,213,2,13,0},		//16Khz采样率
	{2205,429,4, 9,1},		//22.05Khz采样率
	{3200,213,2, 6,1},		//32Khz采样率
	{4410,271,2, 6,0},		//44.1Khz采样率
	{4800,258,3, 3,1},		//48Khz采样率
	{8820,316,2, 3,1},		//88.2Khz采样率
	{9600,344,2, 3,1},  	//96Khz采样率
	{17640,361,2,2,0},  	//176.4Khz采样率 
	{19200,393,2,2,0},  	//192Khz采样率
};

uint8_t I2S2_SampleRate_Set(uint32_t samplerate)
{   
    uint8_t i=0; 
	uint32_t tempreg=0;
    RCC_PeriphCLKInitTypeDef RCCI2S2_ClkInitSture;  
	
	for(i=0;i<(sizeof(I2S_PSC_TBL)/10);i++)
	{
		if((samplerate/10)==I2S_PSC_TBL[i][0])break;
	}
    if(i==(sizeof(I2S_PSC_TBL)/10))return 1;
	
    RCCI2S2_ClkInitSture.PeriphClockSelection=RCC_PERIPHCLK_I2S;
    RCCI2S2_ClkInitSture.PLLI2S.PLLI2SN=(uint32_t)I2S_PSC_TBL[i][1];
    RCCI2S2_ClkInitSture.PLLI2S.PLLI2SR=(uint32_t)I2S_PSC_TBL[i][2]; 
    HAL_RCCEx_PeriphCLKConfig(&RCCI2S2_ClkInitSture);  
	
	RCC->CR|=1<<26;
	while((RCC->CR&1<<27)==0);
	tempreg=I2S_PSC_TBL[i][3]<<0;
	tempreg|=I2S_PSC_TBL[i][4]<<8;
	tempreg|=1<<9;
	SPI2->I2SPR=tempreg;
	return 0;
} 

void I2S2_TX_DMA_Init(uint8_t* buf0,uint8_t *buf1,uint32_t num)
{  
    __HAL_RCC_DMA1_CLK_ENABLE();                                    	
    __HAL_LINKDMA(&I2S2_Handler,hdmatx,I2S2_TXDMA_Handler);         		
	
    I2S2_TXDMA_Handler.Instance=DMA1_Stream4;                       	                  
    I2S2_TXDMA_Handler.Init.Channel=DMA_CHANNEL_0;                  		
    I2S2_TXDMA_Handler.Init.Direction=DMA_MEMORY_TO_PERIPH;         		
    I2S2_TXDMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;             		
    I2S2_TXDMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                 		
    I2S2_TXDMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_HALFWORD;   	
    I2S2_TXDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_HALFWORD;    	
    I2S2_TXDMA_Handler.Init.Mode=DMA_CIRCULAR;                      		
    I2S2_TXDMA_Handler.Init.Priority=DMA_PRIORITY_LOW;             
    I2S2_TXDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;          	
    I2S2_TXDMA_Handler.Init.MemBurst=DMA_MBURST_SINGLE;             	
    I2S2_TXDMA_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;          		

    HAL_DMA_DeInit(&I2S2_TXDMA_Handler);                            		
    HAL_DMA_Init(&I2S2_TXDMA_Handler);	                            	
    
    HAL_DMAEx_MultiBufferStart(&I2S2_TXDMA_Handler,(uint32_t)buf0,(uint32_t)&SPI2->DR,(uint32_t)buf1,num);
    __HAL_DMA_DISABLE(&I2S2_TXDMA_Handler);                         	
    mp_hal_delay_us(10);                                          
    __HAL_DMA_ENABLE_IT(&I2S2_TXDMA_Handler,DMA_IT_TC);         	
    __HAL_DMA_CLEAR_FLAG(&I2S2_TXDMA_Handler,DMA_FLAG_TCIF0_4);     		
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn,2,1);                    		
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
} 
//uint8_t wav_decode_init( const char *filename , __wavctrl* wavx)
uint8_t wav_decode_init(FATFS *fs, const char *filename , __wavctrl* wavx) 
{
	UINT br=0;

	ChunkRIFF *riff;
	ChunkFMT *fmt;
	ChunkFACT *fact;
	ChunkDATA *data;

	uint8_t buf[512];

	FRESULT res;
	res = f_open(fs, audiodev.file, filename , FA_READ);  //打开文件
	if(res == FR_OK){
		res = f_read(audiodev.file, buf, 512, &br);
		riff=(ChunkRIFF *)buf;//获取RIFF块

		if(riff->Format==0X45564157){
				fmt=(ChunkFMT *)(buf+12);	 
				fact=(ChunkFACT *)(buf+12+8+fmt->ChunkSize);
				if(fact->ChunkID==0X74636166||fact->ChunkID==0X5453494C)wavx->datastart=12+8+fmt->ChunkSize+8+fact->ChunkSize;
				else wavx->datastart=12+8+fmt->ChunkSize;  
				data=(ChunkDATA *)(buf+wavx->datastart);	
				if(data->ChunkID==0X61746164){			
					wavx->audioformat=fmt->AudioFormat;	
					wavx->nchannels=fmt->NumOfChannels;	
					wavx->samplerate=fmt->SampleRate;		
					wavx->bitrate=fmt->ByteRate*8;		
					wavx->blockalign=fmt->BlockAlign;	
					wavx->bps=fmt->BitsPerSample;			
					
					wavx->datasize=data->ChunkSize;		
					wavx->datastart=wavx->datastart+8;	
					#if 0 
					printf("wavx->audioformat:%d\n",wavx->audioformat);
					printf("wavx->nchannels:%d\n",wavx->nchannels);
					printf("wavx->samplerate:%d\n",wavx->samplerate);
					printf("wavx->bitrate:%d\n",wavx->bitrate);
					printf("wavx->blockalign:%d\n",wavx->blockalign);
					printf("wavx->bps:%d\r\n",wavx->bps);
					printf("wavx->datasize:%d\n",wavx->datasize);
					printf("wavx->datastart:%d\n",wavx->datastart);  
					#endif

					wav_get_curtime(audiodev.file , &wavctrl);
				}
				else {
					mp_raise_ValueError(MP_ERROR_TEXT("wav decode data error"));
				}
			}
			else {
				mp_raise_ValueError(MP_ERROR_TEXT("not wav data"));
			}
	}
	else{
		mp_raise_ValueError(MP_ERROR_TEXT("wav decode open file "));
	}

	f_close(audiodev.file );
	return res;
}

//I2S2ext RX DMA配置
void I2S2ext_RX_DMA_Init(uint8_t* buf0,uint8_t *buf1,uint32_t num)
{  
	__HAL_RCC_DMA1_CLK_ENABLE();                                  
    __HAL_LINKDMA(&I2S2_Handler,hdmarx,I2S2_RXDMA_Handler);        
	
    I2S2_RXDMA_Handler.Instance=DMA1_Stream3;                                   
    I2S2_RXDMA_Handler.Init.Channel=DMA_CHANNEL_3;                 
    I2S2_RXDMA_Handler.Init.Direction=DMA_PERIPH_TO_MEMORY;         	
    I2S2_RXDMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;             	
    I2S2_RXDMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                 	
    I2S2_RXDMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_HALFWORD;  
    I2S2_RXDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_HALFWORD;  
    I2S2_RXDMA_Handler.Init.Mode=DMA_CIRCULAR;                      	
    I2S2_RXDMA_Handler.Init.Priority=DMA_PRIORITY_LOW;          
    I2S2_RXDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;        
    I2S2_RXDMA_Handler.Init.MemBurst=DMA_MBURST_SINGLE;          
    I2S2_RXDMA_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;          	
    HAL_DMA_DeInit(&I2S2_RXDMA_Handler);                            	
    HAL_DMA_Init(&I2S2_RXDMA_Handler);	                            	

		HAL_DMAEx_MultiBufferStart(&I2S2_RXDMA_Handler,(uint32_t)&I2S2ext->DR,(uint32_t)buf0,(uint32_t)buf1,num);
		__HAL_DMA_DISABLE(&I2S2_RXDMA_Handler);                     
		mp_hal_delay_us(10);  	
		__HAL_DMA_ENABLE_IT(&I2S2_RXDMA_Handler,DMA_IT_TC); 
		__HAL_DMA_CLEAR_FLAG(&I2S2_RXDMA_Handler,DMA_FLAG_TCIF3_7); 

		HAL_NVIC_SetPriority(DMA1_Stream3_IRQn,2,2); 
		HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);	
} 

//进入PCM 录音模式 		  
void recoder_enter_rec_mode(void)
{
	i2srecbuf1 = m_malloc(I2S_RX_DMA_BUF_SIZE);
	i2srecbuf2 = m_malloc(I2S_RX_DMA_BUF_SIZE);
	if(i2srecbuf1 == NULL || i2srecbuf2 == NULL) mp_raise_ValueError(MP_ERROR_TEXT("i2s buf malloc error"));

	WM8978_I2S_CFG(2,0);
	audio_init(I2S_STANDARD_PHILIPS,I2S_MODE_MASTER_TX,I2S_CPOL_LOW,I2S_DATAFORMAT_16B);
	I2S2_SampleRate_Set(8000);
 	I2S2_TX_DMA_Init((uint8_t*)&i2splaybuf[0],(uint8_t*)&i2splaybuf[1],1); 

	DMA1_Stream4->CR&=~(1<<4);
	I2S2ext_RX_DMA_Init(i2srecbuf1,i2srecbuf2,I2S_RX_DMA_BUF_SIZE/2);

  i2s_rx_callback=rec_i2s_dma_rx_callback;
 	__HAL_DMA_ENABLE(&I2S2_TXDMA_Handler);
	__HAL_DMA_ENABLE(&I2S2_RXDMA_Handler); 

}  

//----------------------------------------------------------------------------------

//录制音频
STATIC void wm8978_recorder(const char *name) 
{
	FRESULT res;
	UINT bw;

	const char *file_path = mp_obj_str_get_str(get_path(name ,&res));

  wavsize = 0;
#if 1
  	mp_vfs_mount_t *vfs = MP_STATE_VM(vfs_mount_table);

  	if(res == 1){
  		vfs = vfs->next;
  	}
  	fs_user_mount_t *vfs_fat = MP_OBJ_TO_PTR(vfs->obj);
    res=f_open(&vfs_fat->fatfs,&audio_rec,file_path,FA_WRITE|FA_CREATE_ALWAYS);
#else
  res = f_open_helper(&audio_rec,file_path,FA_WRITE|FA_CREATE_ALWAYS);
#endif
	

	if(res == FR_OK)
	{
		rec_sta=0X80;	//开始录音	
		wavhead = m_malloc(sizeof(__WaveHeader));
		i2srecbuf1 = m_malloc(I2S_RX_DMA_BUF_SIZE);
		i2srecbuf2 = m_malloc(I2S_RX_DMA_BUF_SIZE);
		if(wavhead == NULL ||i2srecbuf1 == NULL || i2srecbuf2 == NULL)	{
				mp_raise_ValueError(MP_ERROR_TEXT("i2s buf malloc error"));
		}

		wavhead->riff.ChunkID=0X46464952;	//"RIFF"
		wavhead->riff.ChunkSize=0;	
		wavhead->riff.Format=0X45564157; 	//"WAVE"
		
		wavhead->fmt.ChunkID=0X20746D66; 	//"fmt "
		wavhead->fmt.ChunkSize=16; 	
		wavhead->fmt.AudioFormat=0X01;
		wavhead->fmt.NumOfChannels=2;
		//wavhead->fmt.SampleRate=16000;
		wavhead->fmt.SampleRate=8000;		
		wavhead->fmt.ByteRate=wavhead->fmt.SampleRate*4;
		wavhead->fmt.BlockAlign=4;
		wavhead->fmt.BitsPerSample=16;
		wavhead->data.ChunkID=0X61746164;	//"data"
		wavhead->data.ChunkSize=0; 
		
		res = f_write(&audio_rec, (const void*)wavhead,sizeof(__WaveHeader),&bw);
		if(res != FR_OK){
			mp_raise_ValueError(MP_ERROR_TEXT("file write hard error"));
		}
		recoder_enter_rec_mode();	
	}
	else{
		mp_raise_ValueError(MP_ERROR_TEXT("recorder open file error"));
	}

}

//=======================================================



//进入PCM 放音模式 		  
void recoder_enter_play_mode(void)
{
	__HAL_DMA_DISABLE(&I2S2_TXDMA_Handler);
	__HAL_DMA_DISABLE(&I2S2_RXDMA_Handler); 	
}

//=======================================================
void wm8978_play_song(const char * file_name) {
	FRESULT res;
	uint32_t bread;

	const char *file_path = mp_obj_str_get_str(get_path(file_name ,&res));
#if 1
	mp_vfs_mount_t *vfs = MP_STATE_VM(vfs_mount_table);

	if(res == 1){
		vfs = vfs->next;
	}
	fs_user_mount_t *vfs_fat = MP_OBJ_TO_PTR(vfs->obj);
#else


#endif
	recoder_enter_play_mode();
	audiodev.file=(FIL*)m_malloc(sizeof(FIL));
	audiodev.i2sbuf1 = m_malloc(WAV_I2S_TX_DMA_BUFSIZE);
	audiodev.i2sbuf2 = m_malloc(WAV_I2S_TX_DMA_BUFSIZE);
	if(audiodev.i2sbuf1 == NULL || audiodev.i2sbuf2 == NULL || audiodev.file == NULL)
			mp_raise_ValueError(MP_ERROR_TEXT("wav play malloc error "));

	res = wav_decode_init(&vfs_fat->fatfs , file_path , &wavctrl); 

	if(res == FR_OK) {
		if(wavctrl.bps==16) {
			WM8978_I2S_CFG(2,0);
			audio_init(I2S_STANDARD_PHILIPS,I2S_MODE_MASTER_TX,I2S_CPOL_LOW,I2S_DATAFORMAT_16B_EXTENDED); 
		}
		else if(wavctrl.bps==24) {
			WM8978_I2S_CFG(2,2);
			audio_init(I2S_STANDARD_PHILIPS,I2S_MODE_MASTER_TX,I2S_CPOL_LOW,I2S_DATAFORMAT_24B);
		}
		res = I2S2_SampleRate_Set(wavctrl.samplerate);

		I2S2_TX_DMA_Init(audiodev.i2sbuf1,audiodev.i2sbuf2,WAV_I2S_TX_DMA_BUFSIZE/2); 
		i2s_tx_callback=wav_i2s_dma_tx_callback;
		wav_stop();	
		res = f_open(&vfs_fat->fatfs, audiodev.file, file_path , FA_READ);  //打开文件

		if(res==0) {
			f_lseek(audiodev.file, wavctrl.datastart);		//跳过文件头
			f_read(audiodev.file,audiodev.i2sbuf1,WAV_I2S_TX_DMA_BUFSIZE,(UINT*)&bread);
			f_read(audiodev.file,audiodev.i2sbuf2,WAV_I2S_TX_DMA_BUFSIZE,(UINT*)&bread);

			}else res=0XFF; 
	}
	else res=0XFF;
}
//-------------------------------------------------------

//将小写字母转为大写字母,如果是数字,则保持不变.
uint8_t char_upper(uint8_t c)
{
	if(c<'A')return c;//数字,保持不变.
	if(c>='a')return c-0x20;//变为大写.
	else return c;//大写,保持不变
}	
//===============================================================================
STATIC void audio_wm8978_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    mp_printf(print,"wm8978 printf\n");
}
//-----------------------------------------------------------------------------------------
STATIC mp_obj_t auido_wm8978_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
	// check arguments
	mp_arg_check_num(n_args, n_kw, 0, MP_OBJ_FUN_ARGS_MAX, true);
	wm8978_init();
	
  wm8978_obj_t *wm_obj;
  wm_obj = m_new_obj(wm8978_obj_t);
  wm_obj->base.type = &audio_wm8978_type;
  wm_obj->callback = mp_const_none;
	audiodev.status=0;
	audiodev.audioName = m_malloc(50);
	if(audiodev.audioName != NULL)	memset(audiodev.audioName, '\0', 50);
	else mp_raise_ValueError(MP_ERROR_TEXT("malloc audio Name file error"));

	return MP_OBJ_FROM_PTR(wm_obj);
}
//==============================================================================
//播放音乐
STATIC mp_obj_t audio_wm8978_play(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {

	if(audiodev.status==0){
		const char *type = mp_obj_str_get_str(file_type((const char *)audiodev.audioName));

		while(Is_FileReadOk){
			Is_FileReadOk = 0;
			mp_hal_delay_ms(1000);
		}
		if(strncmp(type , "wav" , 3) == 0 ||strncmp(type , "WAV" , 3) == 0) {
			wm8978_play_song(audiodev.audioName);
			audiodev.status = 3<<0;
			__HAL_DMA_ENABLE(&I2S2_TXDMA_Handler);
		}else if(strncmp(type , "mp3" , 3) == 0 ||strncmp(type , "MP3" , 3) == 0) {
			//wm8978_adda_cfg(1,0);
			mp3_play_song(audiodev.audioName);
			//wm8978_adda_cfg(0,0);
		}else{
			mp_raise_ValueError(MP_ERROR_TEXT("play audio type error"));
		}
	}else if((audiodev.status&0X01)==0){
		//audiodev.status = 3<<0;
		//__HAL_DMA_ENABLE(&I2S2_TXDMA_Handler);
	}

	return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(audio_wm8978_play_obj, 0, audio_wm8978_play);
//-----------------------------------------------------------------------------------
STATIC mp_obj_t audio_wm8978_continue_play(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
	audiodev.status = 3<<0;
	__HAL_DMA_ENABLE(&I2S2_TXDMA_Handler);
	return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(audio_wm8978_continue_play_obj, 0, audio_wm8978_continue_play);
//-----------------------------------------------------------------------------------

STATIC mp_obj_t audio_wm8978_pause(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
		audiodev.status&=~(1<<0);
		return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(audio_wm8978_pause_obj, 0, audio_wm8978_pause);
//-----------------------------------------------------------------------------------
STATIC mp_obj_t audio_wm8978_stop(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
	if(audiodev.status != 0){
		audiodev.status=0;
		__HAL_DMA_DISABLE(&I2S2_TXDMA_Handler);//结束播放; 	
		f_close(audiodev.file);
		memset(audiodev.i2sbuf1,0,WAV_I2S_TX_DMA_BUFSIZE);
		memset(audiodev.i2sbuf2,0,WAV_I2S_TX_DMA_BUFSIZE);
		m_free(audiodev.i2sbuf1);
		m_free(audiodev.i2sbuf2);
		m_free(audiodev.file);
	}

	return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(audio_wm8978_stop_obj, 0, audio_wm8978_stop);
//-----------------------------------------------------------------------------------
STATIC mp_obj_t audio_wm8978_load(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)  {

		STATIC const mp_arg_t wm8978_allowed_args[] = {
			{ MP_QSTR_filename, 		MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} }, 
		};

		mp_arg_val_t args[MP_ARRAY_SIZE(wm8978_allowed_args)];
		mp_arg_parse_all(n_args-1, pos_args+1, kw_args, MP_ARRAY_SIZE(wm8978_allowed_args), wm8978_allowed_args, args);
			//MP_OBJ_NULL
		if(args[0].u_obj !=MP_OBJ_NULL) 
		{
			mp_buffer_info_t bufinfo;
			if (mp_obj_is_int(args[0].u_obj)) {
				mp_raise_ValueError(MP_ERROR_TEXT("wav name parameter error"));
			} else {
					mp_get_buffer_raise(args[0].u_obj, &bufinfo, MP_BUFFER_READ);
					memset(audiodev.audioName, '\0', 50);
					strncpy(audiodev.audioName,(char *)bufinfo.buf,bufinfo.len);
			}
		}
		else{
			 mp_raise_ValueError(MP_ERROR_TEXT("load wav is empty"));
		}

		return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(audio_wm8978_load_obj, 1, audio_wm8978_load);
//-----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------
STATIC mp_obj_t audio_wm8978_volume(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
  STATIC const mp_arg_t volume_args[] = {
    { MP_QSTR_vol,   MP_ARG_INT, {.u_int = 80} }, 
  };
  mp_arg_val_t args[MP_ARRAY_SIZE(volume_args)];
  mp_arg_parse_all(n_args-1, pos_args+1, kw_args, MP_ARRAY_SIZE(volume_args), volume_args, args);

	uint8_t vul = (uint8_t)(args[0].u_int * 0.63);
	wm8978_spk_vol(vul);
	wm8978_hspk_vol(vul,vul);

  return mp_obj_new_int(vul);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(audio_wm8978_volume_obj, 0, audio_wm8978_volume);
//----------------------------------------------------------------------------------------------------
//获取 歌曲总播放时间
STATIC mp_obj_t audio_wm8978_totsec(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
	return mp_obj_new_int(wavctrl.totsec);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(audio_wm8978_totsec_obj,0, audio_wm8978_totsec);	
//----------------------------------------------------------------------------------------------------
//获取 目前播放时间
STATIC mp_obj_t audio_wm8978_cursec(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
	return mp_obj_new_int(wavctrl.cursec);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(audio_wm8978_cursec_obj,0, audio_wm8978_cursec);	
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

STATIC mp_obj_t audio_wm8978_record(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
  STATIC const mp_arg_t record_args[] = {
    { MP_QSTR_filename,     MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    { MP_QSTR_db,     			 MP_ARG_INT, {.u_int = 80} }, //
  };

  mp_arg_val_t args[MP_ARRAY_SIZE(record_args)];
  mp_arg_parse_all(n_args-1, pos_args+1, kw_args, MP_ARRAY_SIZE(record_args), record_args, args);

    //MP_OBJ_NULL
  if(args[0].u_obj !=MP_OBJ_NULL) 
  {
    mp_buffer_info_t bufinfo;
    if (mp_obj_is_int(args[0].u_obj)) {
      mp_raise_ValueError(MP_ERROR_TEXT("input record file name error"));
    } 
		else{
			wm8978_adda_cfg(0,1);
			wm8978_input_cfg(1,1,0);
			wm8978_output_cfg(0,1);
			wm8978_spk_vol(0);
			wm8978_mic_gain((uint8_t)(args[1].u_int * 0.63));
			mp_get_buffer_raise(args[0].u_obj, &bufinfo, MP_BUFFER_READ);
			char *str = bufinfo.buf;

			while(Is_FileReadOk){
				Is_FileReadOk = 0;
				mp_hal_delay_ms(1000);
			}
			wm8978_recorder(str);
    }
  }
	else	mp_raise_ValueError(MP_ERROR_TEXT("record file name parameter is empty"));
  return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(audio_wm8978_record_obj, 1, audio_wm8978_record);
//-----------------------------------------------------------------------------------
STATIC mp_obj_t audio_wm8978_record_stop(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
	uint32_t irq_state = disable_irq();
	rec_sta |= 0x40;
	//printf("rec_sta:%x\n",rec_sta);
	enable_irq(irq_state);
	//配置播放
	wm8978_adda_cfg(1,0);
	wm8978_input_cfg(0,0,0);
	wm8978_output_cfg(1,0);

	return mp_obj_new_int(rec_sta);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(audio_wm8978_record_stop_obj, 0, audio_wm8978_record_stop);
//-----------------------------------------------------------------------------------

STATIC mp_obj_t audio_wm8978_deinit(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
	audiodev.status=0;
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(audio_wm8978_deinit_obj,0, audio_wm8978_deinit);
//----------------------------------------------------------------------------------




/******************************************************************************/
STATIC const mp_rom_map_elem_t audio_wm8978_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR__name__), MP_ROM_QSTR(MP_QSTR_wm8978) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&audio_wm8978_deinit_obj) },
		{ MP_ROM_QSTR(MP_QSTR_load), MP_ROM_PTR(&audio_wm8978_load_obj) },
		{ MP_ROM_QSTR(MP_QSTR_play), MP_ROM_PTR(&audio_wm8978_play_obj) },
		{ MP_ROM_QSTR(MP_QSTR_continue_play), MP_ROM_PTR(&audio_wm8978_continue_play_obj)},
		{ MP_ROM_QSTR(MP_QSTR_pause), MP_ROM_PTR(&audio_wm8978_pause_obj)},
		{ MP_ROM_QSTR(MP_QSTR_stop), MP_ROM_PTR(&audio_wm8978_stop_obj) },
		{ MP_ROM_QSTR(MP_QSTR_volume), MP_ROM_PTR(&audio_wm8978_volume_obj)},
		{ MP_ROM_QSTR(MP_QSTR_totsec), MP_ROM_PTR(&audio_wm8978_totsec_obj)},
		{ MP_ROM_QSTR(MP_QSTR_cursec), MP_ROM_PTR(&audio_wm8978_cursec_obj)},

		{ MP_ROM_QSTR(MP_QSTR_record), MP_ROM_PTR(&audio_wm8978_record_obj)},
		{ MP_ROM_QSTR(MP_QSTR_record_stop), MP_ROM_PTR(&audio_wm8978_record_stop_obj)},
			
};STATIC MP_DEFINE_CONST_DICT(audio_wm8978_locals_dict,audio_wm8978_locals_dict_table);


const mp_obj_type_t audio_wm8978_type = {
    { &mp_type_type },
    .name = MP_QSTR_WM8978,
    .print = audio_wm8978_print,
    .make_new = auido_wm8978_make_new,
    .locals_dict = (mp_obj_dict_t *)&audio_wm8978_locals_dict,
};

/******************************************************************************/

#endif // MICROPY_HW_WM8978
