/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* USER CODE BEGIN 0 */

#include "main.h"
#include "usart.h" //包含UART_HandleTypeDef huart1定义
#include "string.h"
#include "spi.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RESET_Pin|PA_EN_Pin|GPIO_PIN_5|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|SPI_NSS_Pin|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_13|GPIO_PIN_14|RS485_T_R_Pin|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TX_RX_GPIO_Port, TX_RX_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PCPin PCPin */
  GPIO_InitStruct.Pin = RESET_Pin|TX_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = SPI_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = PA_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PA_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB10 PB11 PB12 
                           PB13 PB14 PB3 PB4 
                           PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = RS485_T_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_T_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 2 */
/*
文件使用说明：
外部文件调用Lora_Init()进行lora的参数和接口初始化
使用void Lora_Send(unsigned char *p_send_buf,unsigned char len);进行数据发送
使用void Lora_RecDataGet(unsigned char *p_recdata,unsigned char len);接收数据
使用unsigned char Lora_GetNumofRecData(void);获取接受到数据的字节长度
在对应IO中中断处理函数中调用void SX1278_Interupt(void);进行lora数据处理
		TxDone RxDone CADDone ```
		Lora module 的IO0 IO1 IO2 IO3 IO5对应的外部触发EXTI中断 调用SX1278_Interupt()
*/

//#include "stm32l1xx.h"

/* 宏定义*/
#define LORA_RECNUM_MAX 512
///* GPIO相关宏定义 */

#define  SX1278_SDO                         HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)

#define  RF_REST_L			    								HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_RESET)	     
#define  RF_REST_H			    								HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_SET)	  

#define  RF_CE_L                            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_RESET)      
#define  RF_CE_H                            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_SET)         

#define  RF_CKL_L                           HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_RESET)     
#define  RF_CKL_H                           HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_SET)      

#define  RF_SDI_L                           HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET)      
#define  RF_SDI_H                           HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET)         

#define  PA_TXD_OUT()                       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,GPIO_PIN_SET);\
                                            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,GPIO_PIN_RESET)
#define  PA_RXD_OUT()                       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,GPIO_PIN_SET);\
                                            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,GPIO_PIN_RESET)  
/*lora 初始化参数定义*/

#define   SPREADINGFACTOR  7    //7-12
#define   CODINGRATE    1        //1-4
#define   BW_FREQUENCY  7        //6-9
#define   POWERVALUE    7

//const unsigned char  power_data[8]={0X80,0X80,0X80,0X83,0X86,0x89,0x8c,0x8f};
const unsigned char  power_data[8]={0X80,0X80,0X80,0X83,0X86,0x89,0x8c,0x00};//4f
unsigned char   Frequency[3]={0x6c,0x80,0x00};//434MHz
//unsigned char   Frequency[3]={0x6c,0x40,0x00};//433MHz

/* 全局变量 定义 */
uint8_t flag=0x00;

uint8_t Recv_Num=0;
uint8_t RF_EX0_STATUS;

unsigned char   SX1278_RLEN;
unsigned char   recv[LORA_RECNUM_MAX];

lpCtrlTypefunc_t lpTypefunc = {0,0,0,0};

/*************************************************************
  Function   ：Lora_GPIO_Config  
  Description：lora module对应的MCU接口定义
	Input      : none
  return     : none 
*************************************************************/	
void Lora_GPIO_Config(void)
{

}

/*************************************************************
  Function   ：SX1276Reset  
  Description：lora module 复位操作
							        __      ____
               RESET :  |____|  
	Input      : none
  return     : none 
*************************************************************/
void SX1276Reset(void)
{
   //RF_REST_L;	
	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
   Delay1s(200);
   //RF_REST_H;
	 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
   Delay1s(500);
}
/*************************************************************
  Function   ：RF_SPI_WRITE_BYTE  
  Description：SPI 写操作
	Input      : data SPI写入的数据 
  return     : none 
*************************************************************/
void RF_SPI_WRITE_BYTE(unsigned char data)
{
   unsigned char i;
   for (i=0;i<8;i++){   
     if (data & 0x80)						/* check if MSB is high */
       RF_SDI_H;
     else RF_SDI_L;							/* if not, set to low */
		 
     RF_CKL_H;						  		/* toggle clock high */
     data = (data << 1);				/* shift 1 place for next bit */
     RF_CKL_L;									/* toggle clock low */
   }
}
/*************************************************************
  Function   ：RF_SPI_READ_BYTE  
  Description：SPI 读操作
	Input      : none
  return     : 读取的数据 
*************************************************************/
unsigned char RF_SPI_READ_BYTE()
{	 
   unsigned char j;
   unsigned char i;
   j=0;
   for (i = 0; i < 8; i++){	 
     RF_CKL_H; 
     j = (j << 1);						 // shift 1 place to the left or shift in 0 //
     if( SX1278_SDO )							 // check to see if bit is high //
       j = j | 0x01; 					   // if high, make bit high //
												  // toggle clock high // 
     RF_CKL_L; 							 // toggle clock low //  
   }
  
   return j;								// toggle clock low //
}
/*************************************************************
  Function   ：cmdSwitchEn  
  Description：SPI chip select
	Input      : cmdEntype 
								enClose  --- disable chip
								enOpen   --- enable chip
  return     : 
*************************************************************/
void cmdSwitchEn(cmdEntype_t cmd)
{
   switch(cmd)
   {
     case enOpen:{
       RF_CE_L;
     }break;
     case enClose:{
       RF_CE_H;
     }break;
     default:break;
   }
}
/*************************************************************
  Function   ：cmdSwitchEn  
  Description：SPI chip select
	Input      : cmdpatype 
								rxOpen  --- rx enable
								txOpen   ---tx enable
  return     : 
*************************************************************/
void cmdSwitchPA(cmdpaType_t cmd)
{
   switch(cmd)
   {
     case rxOpen:{
       PA_RXD_OUT();
     }break;
     case txOpen:{
       PA_TXD_OUT();
     }break;
     
     default:break;
   }
}


lpCtrlTypefunc_t  ctrlTypefunc = {
   RF_SPI_WRITE_BYTE,
   RF_SPI_READ_BYTE,
   cmdSwitchEn,
   cmdSwitchPA
};
void Delay1s(unsigned int ii)
{
   unsigned char j;
   while(ii--){
     for(j=0;j<100;j++);
   }
}
/*************************************************************
  Function   ：register_rf_func  
  Description：结构体变量赋值
	Input      : func lpCtrlTypefunc_t类型变量 
							 待赋值给全局变量lpTypefunc
  return     : none  
*************************************************************/	
void register_rf_func(lpCtrlTypefunc_t *func)
{
   if(func->lpByteWritefunc != 0){
      lpTypefunc.lpByteWritefunc = func->lpByteWritefunc;
   }
   if(func->lpByteReadfunc != 0){
      lpTypefunc.lpByteReadfunc = func->lpByteReadfunc;
   }
   if(func->lpSwitchEnStatus != 0){
      lpTypefunc.lpSwitchEnStatus = func->lpSwitchEnStatus;
   }
   if(func->paSwitchCmdfunc != 0){
      lpTypefunc.paSwitchCmdfunc = func->paSwitchCmdfunc;
   }
}
/*************************************************************
  Function   ：SX1276WriteBuffer  
  Description：SPI 向地址为addr的寄存器写入内容buffer
	Input      : addr sx1278的寄存器地址	
							 buffer 写入的内容
  return     : none  
*************************************************************/	
void SX1276WriteBuffer( unsigned char addr, unsigned char buffer)
{ 
//   //lpTypefunc.lpSwitchEnStatus(enOpen); //NSS = 0;
//	 RF_CE_L;
//   //lpTypefunc.lpByteWritefunc( addr | 0x80 );
//	 RF_SPI_WRITE_BYTE( addr | 0x80 );
//   //lpTypefunc.lpByteWritefunc( buffer);
//	 RF_SPI_WRITE_BYTE( buffer);
//   //lpTypefunc.lpSwitchEnStatus(enClose); //NSS = 1;
//	 RF_CE_H;
	uint8_t tmp;
	tmp =( addr | 0x80 );
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&tmp,1,1000);
	HAL_SPI_Transmit(&hspi1,&buffer,1,1000);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
}
/*************************************************************
  Function   ：SX1276ReadBuffer  
  Description：SPI 读取地址为addr的寄存器内容
	Input      : addr sx1278的寄存器地址							
  return     : 寄存器存放的数据  
*************************************************************/		
unsigned char SX1276ReadBuffer(unsigned char addr)
{
  unsigned char Value;
//  //lpTypefunc.lpSwitchEnStatus(enOpen); //NSS = 0;
//	RF_CE_L;
//  //lpTypefunc.lpByteWritefunc( addr & 0x7f  );
//	RF_SPI_WRITE_BYTE( addr & 0x7f );
//  //Value = lpTypefunc.lpByteReadfunc();
//	Value = RF_SPI_READ_BYTE();
//  //lpTypefunc.lpSwitchEnStatus(enClose);//NSS = 1;
//	RF_CE_H;

	uint8_t tmp;
	tmp =( addr & 0x7f );
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&tmp,1,1000);
	HAL_SPI_Receive(&hspi1,&Value,1,1000);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
  return Value; 
}
/*************************************************************
  Function   ：SX1276LoRaFsk  
  Description：lora工作参数设置 通信工作模式 TX RX CAD SLEEP IDLE
	Input      : opmode RFMode_SEt类型							
  return     : none   
*************************************************************/		
void SX1276LoRaSetOpMode( RFMode_SET opMode )
{
   unsigned char opModePrev;
   opModePrev=SX1276ReadBuffer(REG_LR_OPMODE);
   opModePrev &=0xf8;
   opModePrev |= (unsigned char)opMode;
   SX1276WriteBuffer( REG_LR_OPMODE, opModePrev);		
}
/*************************************************************
  Function   ：SX1276LoRaFsk  
  Description：lora工作参数设置 通信模式 FSK 或是lora
	Input      : opmode
							FSK_mode | LORA_mode
  return     : none   
*************************************************************/	
void SX1276LoRaFsk( Debugging_fsk_ook opMode )
{
   unsigned char opModePrev;
   opModePrev=SX1276ReadBuffer(REG_LR_OPMODE);
   opModePrev &=0x7F;
   opModePrev |= (unsigned char)opMode;
   SX1276WriteBuffer( REG_LR_OPMODE, opModePrev);		
}
/*************************************************************
  Function   ：SX1276LoRaSetRFFrequency  
  Description：lora工作参数设置 通信频率
							frequency = F(xosc)*Frf/2^19
	Input      : Frequency[i]
							Resolution is 61.035 Hz if F(XOSC) = 32 MHz. Default value is
							0x6c8000 = 434 MHz. Register values must be modified only
							when device is in SLEEP or STAND-BY mode.
  return     : none   
*************************************************************/		
void SX1276LoRaSetRFFrequency(void)
{
   SX1276WriteBuffer( REG_LR_FRFMSB, Frequency[0]);
   SX1276WriteBuffer( REG_LR_FRFMID, Frequency[1]);
   SX1276WriteBuffer( REG_LR_FRFLSB, Frequency[2]);
}
/*************************************************************
  Function   ：SX1276LoRaSetRFPower  
  Description：lora工作参数设置 发射功率设置
							PA_BOOST pin.
							Maximum power of +20 dBm
							maxpower  pmax=10.8+0.6*maxpower[dbm]
							Pout=17-(15-OutputPower)
	Input      : power :
							{0X80,0X80,0X80,0X83,0X86,0x89,0x8c,0x8f}&0x0F;
  return     : none   
*************************************************************/		
void SX1276LoRaSetRFPower(unsigned char power )
{
   SX1276WriteBuffer( REG_LR_PADAC, 0x87);
   SX1276WriteBuffer( REG_LR_PACONFIG,  power_data[power] );
}
/*************************************************************
  Function   ：SX1276LoRaSetSpreadingFactor  
  Description：lora工作参数设置  扩频因子设置
	Input      : value :
								SF rate (expressed as a base-2 logarithm)
								6 --- 64 chips / symbol
								7 --- 128 chips / symbol
								8 --- 256 chips / symbol
								9 --- 512 chips / symbol
								10 --- 1024 chips / symbol
								11 --- 2048 chips / symbol
								12 --- 4096 chips / symbol
								other values reserved
  return     : none   
*************************************************************/
void SX1276LoRaSetSpreadingFactor(unsigned char factor )
{
   unsigned char RECVER_DAT;
   SX1276LoRaSetNbTrigPeaks( 3 );
   RECVER_DAT=SX1276ReadBuffer( REG_LR_MODEMCONFIG2);	  
   RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG2_SF_MASK ) | ( factor << 4 );
   SX1276WriteBuffer( REG_LR_MODEMCONFIG2, RECVER_DAT );	 
}
	
/*************************************************************
  Function   ：SX1276LoRaSetNbTrigPeaks  
  Description：lora工作参数设置 
								2-0 bits of register 0x31
	Input      : value :

  return     : none   
*************************************************************/
void SX1276LoRaSetNbTrigPeaks(unsigned char value )
{
   unsigned char RECVER_DAT;
   RECVER_DAT = SX1276ReadBuffer( 0x31);
   RECVER_DAT = ( RECVER_DAT & 0xF8 ) | value;
   SX1276WriteBuffer( 0x31, RECVER_DAT );
}
/*************************************************************
  Function   ：SX1276LoRaSetErrorCoding  
  Description：lora工作参数设置 error coding rate
							In implicit header mode should be set on receiver to determine
							expected coding rate.
	Input      : value :
									001  ----  4/5
									010  ----  4/6
									011  ----  4/7
									100  ----  4/8	
  return     : none   
*************************************************************/
void SX1276LoRaSetErrorCoding(unsigned char value )
{	
   unsigned char RECVER_DAT;
   RECVER_DAT=SX1276ReadBuffer( REG_LR_MODEMCONFIG1);
   RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG1_CODINGRATE_MASK ) | ( value << 1 );
   SX1276WriteBuffer( REG_LR_MODEMCONFIG1, RECVER_DAT);
}
/*************************************************************
  Function   ：SX1276LoRaSetPacketCrcOn  
  Description：lora工作参数设置 packet crc on
	Input      : enabe :
									true  --- enable
									false --- disable
  return     : none   
*************************************************************/
void SX1276LoRaSetPacketCrcOn(BOOL enable )
{	
   unsigned char RECVER_DAT;
   RECVER_DAT= SX1276ReadBuffer( REG_LR_MODEMCONFIG2);
   RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) | ( enable << 2 );
   SX1276WriteBuffer( REG_LR_MODEMCONFIG2, RECVER_DAT );
}

/*************************************************************
  Function   ：SX1276LoRaSetSignalBandwidth  
  Description：lora工作参数设置 signal bandwidth
  Input      : bw --- 
								 0000  ----.  7.8khz
								 0001  ----.  10.4khz
								 0010  ----.  15.6khz
								 0011  ----.  20.8khz
								 0100  ----.  31.25khz
								 0101  ----.  41.7khz
								 0110  ----.  62.5khz
								 0111  ----.  125Khz
								 1000  ----.  250khz
								 1001  ----.  500khz

  return     : none   
*************************************************************/
void SX1276LoRaSetSignalBandwidth(unsigned char bw )
{
   unsigned char RECVER_DAT;
   RECVER_DAT=SX1276ReadBuffer( REG_LR_MODEMCONFIG1);
   RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG1_BW_MASK ) | ( bw << 4 );
   SX1276WriteBuffer( REG_LR_MODEMCONFIG1, RECVER_DAT );
}
	
/*************************************************************
  Function   ：SX1276LoRaSetImplicitHeaderOn  
  Description：lora工作参数设置 ImplicitHeaderOn
  Input      : enabe --- 
									true implicit header mode
									false  explicit header mode
  return     : none   
*************************************************************/
void SX1276LoRaSetImplicitHeaderOn(BOOL enable )
{
   unsigned char RECVER_DAT;
   RECVER_DAT=SX1276ReadBuffer( REG_LR_MODEMCONFIG1 );
   RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) | ( enable );
   SX1276WriteBuffer( REG_LR_MODEMCONFIG1, RECVER_DAT );
}
	
/*************************************************************
  Function   ：SX1276LoRaSetSymbTimeout  
  Description：lora工作参数设置 设置Rx operation time-out value 
							Timeout =  symboltimeout.Ts
  Input      : value  9bit valid 
  return     : none   
*************************************************************/
void SX1276LoRaSetSymbTimeout(unsigned int value )
{
   unsigned char RECVER_DAT[2];
   RECVER_DAT[0]=SX1276ReadBuffer( REG_LR_MODEMCONFIG2 );
   RECVER_DAT[1]=SX1276ReadBuffer( REG_LR_SYMBTIMEOUTLSB );
   RECVER_DAT[0] = ( RECVER_DAT[0] & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) | ( ( value >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK );
   RECVER_DAT[1] = value & 0xFF;
   SX1276WriteBuffer( REG_LR_MODEMCONFIG2, RECVER_DAT[0]);
   SX1276WriteBuffer( REG_LR_SYMBTIMEOUTLSB, RECVER_DAT[1]);
}
	
/*************************************************************
  Function   ：SX1276LoRaSetPayloadLength  
  Description：lora工作参数设置 设置payload length
  Input      : value  ‘0’ 不允许 
  return     : none   
*************************************************************/
void SX1276LoRaSetPayloadLength(unsigned char value )
{
   SX1276WriteBuffer( REG_LR_PAYLOADLENGTH, value );
} 
	
/*************************************************************
  Function   ：SX1276LoRaSetPreamLength  
  Description：lora工作参数设置 设置preamble length
  Input      : value 低字节设置MSB 高字节设置LSB MSB=preamble length +4.25symbol
  return     : none   
*************************************************************/
void SX1276LoRaSetPreamLength(unsigned int value )
{
   unsigned char a[2];
   a[0]=(value&0xff00)>>8;
   a[1]=value&0xff;
   SX1276WriteBuffer( REG_LR_PREAMBLEMSB, a[0] );
   SX1276WriteBuffer( REG_LR_PREAMBLELSB, a[1] );
}
	
/*************************************************************
  Function   ：SX1276LoRaSetMobileNode  
  Description：lora工作参数设置 数据优化否
  Input      : Bool true-优化 false --不优化
  return     : none   
*************************************************************/
void SX1276LoRaSetMobileNode(BOOL enable )
{	 
   unsigned char RECVER_DAT;
   RECVER_DAT=SX1276ReadBuffer( REG_LR_MODEMCONFIG3 );
   RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG3_MOBILE_NODE_MASK ) | ( enable << 3 );
   SX1276WriteBuffer( REG_LR_MODEMCONFIG3, RECVER_DAT );
}

/*************************************************************
  Function   ：SX1276LORA_INT  
  Description：lora初始化,通信参数
  Input      : none 
  return     : none   
*************************************************************/
void SX1276LORA_INT(void)
{
   SX1276LoRaSetOpMode(Sleep_mode);  									//设置睡眠模式
   SX1276LoRaFsk(LORA_mode);	     										// 设置扩频模式
   SX1276LoRaSetOpMode(Stdby_mode);  									// 设置为普通模式
   SX1276WriteBuffer( REG_LR_DIOMAPPING1,GPIO_VARE_1);// IO 标志配置 IO 0 1 2 3
   SX1276WriteBuffer( REG_LR_DIOMAPPING2,GPIO_VARE_2); // IO 5
   SX1276LoRaSetRFFrequency();								        //设置频率 434MHZ
   SX1276LoRaSetRFPower(POWERVALUE);					        // 设置功率
   SX1276LoRaSetSpreadingFactor(SPREADINGFACTOR);	    // 扩频因子设置
   SX1276LoRaSetErrorCoding(CODINGRATE);		          //有效数据比
   SX1276LoRaSetPacketCrcOn(true);			              //CRC 校验打开
   SX1276LoRaSetSignalBandwidth( BW_FREQUENCY );	    //设置扩频带宽
   SX1276LoRaSetImplicitHeaderOn(false);		          //同步头是显性模式
   SX1276LoRaSetPayloadLength( 0xff);									//最大payload length 255
   SX1276LoRaSetSymbTimeout( 0x3FF );									//超时设置
   SX1276LoRaSetMobileNode(true); 			             // 低数据的优化
   RF_RECEIVE(); 																		//进入RX 模式
}
/*************************************************************
  Function   ：FUN_RF_SENDPACKET  
  Description：lora 发送一定长度的数据
  Input      : RF_TRAN_P ---待发送数据区 LEN--待发送数据长度
  return     : none   
*************************************************************/	
void FUN_RF_SENDPACKET(unsigned char *RF_TRAN_P,unsigned char LEN)
{
   unsigned char ASM_i;
			uint8_t write_fifo_addr=0x80;
		//uint8_t read_fifo_addr=0x00;
	 uint8_t test_read_buff[10];
	memset(test_read_buff,0,10);
	
   lpTypefunc.paSwitchCmdfunc(txOpen);
   SX1276LoRaSetOpMode( Stdby_mode );
   SX1276WriteBuffer( REG_LR_HOPPERIOD, 0 );								//不做频率跳变
   SX1276WriteBuffer(REG_LR_IRQFLAGSMASK,IRQN_TXD_Value);		//打开发送中断
   SX1276WriteBuffer( REG_LR_PAYLOADLENGTH, LEN);	 					//最大数据包
   SX1276WriteBuffer( REG_LR_FIFOTXBASEADDR, 0);
   SX1276WriteBuffer( REG_LR_FIFOADDRPTR, 0 );
	
//   //lpTypefunc.lpSwitchEnStatus(enOpen);
//	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
//   //lpTypefunc.lpByteWritefunc( 0x80 );
//	 HAL_SPI_Transmit(&hspi1,(uint8_t *)0x80,1,1000); 
//   for( ASM_i = 0; ASM_i < LEN; ASM_i++ ){
//     //lpTypefunc.lpByteWritefunc( *RF_TRAN_P );RF_TRAN_P++;
//		 HAL_SPI_Transmit(&hspi1,RF_TRAN_P,1,1000);RF_TRAN_P++;
//   }
//   //lpTypefunc.lpSwitchEnStatus(enClose);
//	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);

	 ///////////////////////-----------Write FIFO---------///////////////////////////////
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);// NSS拉低
		HAL_SPI_Transmit(&hspi1,&write_fifo_addr,1,1000); //写+fifo地址0x00
		//lpTypefunc.lpByteWritefunc( 0x80 );
		 for( ASM_i = 0; ASM_i < LEN; ASM_i++ ){
			 HAL_SPI_Transmit(&hspi1,RF_TRAN_P,1,1000);
			 //HAL_SPI_TransmitReceive(&hspi1,&read_fifo_addr,&test_read_buff[ASM_i],1,1000);
			 //SX1276WriteBuffer( REG_LR_FIFO , *RF_TRAN_P );
			 RF_TRAN_P++;
		 }
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); // NSS拉高 
	 ////////////////////////////////////////////////////////////////////////////////////
	 
//		///////////////////////-----------Read FIFO---------///////////////////////////////
//		SX1276WriteBuffer( REG_LR_FIFOADDRPTR, 0 );
//		
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);// NSS拉低
//		HAL_SPI_Transmit(&hspi1, &read_fifo_addr,1,1000);
//		 for( ASM_i = 0; ASM_i < LEN; ASM_i++ ){
//			 HAL_SPI_TransmitReceive(&hspi1,&read_fifo_addr,&test_read_buff[ASM_i],1,1000);
//		 }
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); // NSS拉高 
//	 
//	////////////////////////////////////////////////////////////////////////////////////
	 
   SX1276WriteBuffer(REG_LR_DIOMAPPING1,0x40);
   SX1276WriteBuffer(REG_LR_DIOMAPPING2,0x00);
   SX1276LoRaSetOpMode( Transmitter_mode );
}

////////////Old Code//////////////
//{
//   unsigned char ASM_i;
//   lpTypefunc.paSwitchCmdfunc(txOpen);
//   SX1276LoRaSetOpMode( Stdby_mode );
//   SX1276WriteBuffer( REG_LR_HOPPERIOD, 0 );								//不做频率跳变
//   SX1276WriteBuffer(REG_LR_IRQFLAGSMASK,IRQN_TXD_Value);		//打开发送中断
//   SX1276WriteBuffer( REG_LR_PAYLOADLENGTH, LEN);	 					//最大数据包
//   SX1276WriteBuffer( REG_LR_FIFOTXBASEADDR, 0);
//   SX1276WriteBuffer( REG_LR_FIFOADDRPTR, 0 );
//	
//   //lpTypefunc.lpSwitchEnStatus(enOpen);
//	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
//   lpTypefunc.lpByteWritefunc( 0x80 );
//   for( ASM_i = 0; ASM_i < LEN; ASM_i++ ){
//     //lpTypefunc.lpByteWritefunc( *RF_TRAN_P );RF_TRAN_P++;
//		 HAL_SPI_Transmit(&hspi1,RF_TRAN_P,1,1000);RF_TRAN_P++;
//   }
//   //lpTypefunc.lpSwitchEnStatus(enClose);
//	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
//	 
//   SX1276WriteBuffer(REG_LR_DIOMAPPING1,0x40);
//   SX1276WriteBuffer(REG_LR_DIOMAPPING2,0x00);
//   SX1276LoRaSetOpMode( Transmitter_mode );
//}

/*************************************************************
  Function   ：RF_RECEIVE  
  Description：lora 接收模式 接收打开 中断方式
  Input      : none 
  return     : none   
*************************************************************/
void RF_RECEIVE (void)
{
   SX1276LoRaSetOpMode(Stdby_mode );
   SX1276WriteBuffer(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);  //打开Rx Done中断
   SX1276WriteBuffer(REG_LR_HOPPERIOD,	PACKET_MIAX_Value );
   SX1276WriteBuffer( REG_LR_DIOMAPPING1, 0X00 );
   SX1276WriteBuffer( REG_LR_DIOMAPPING2, 0X00 );	
	 /////////////////////////////////////////////////
	 SX1276WriteBuffer( REG_LR_FIFORXBASEADDR, 0);
   SX1276WriteBuffer( REG_LR_FIFOADDRPTR, 0 );
	 /////////////////////////////////////////////////
   SX1276LoRaSetOpMode( Receiver_mode );
   lpTypefunc.paSwitchCmdfunc(rxOpen);
}
	
/*************************************************************
  Function   ：RF_CAD_RECEIVE  
  Description：lora CAD 侦测 接收打开 
  Input      : none 
  return     : none   
*************************************************************/
void RF_CAD_RECEIVE (void)
{
   SX1276LoRaSetOpMode( Stdby_mode );
   SX1276WriteBuffer(REG_LR_IRQFLAGSMASK,  IRQN_CAD_Value);	//打开CAD Done 中断
   SX1276WriteBuffer( REG_LR_DIOMAPPING1, 0X80 );
   SX1276WriteBuffer( REG_LR_DIOMAPPING2, 0X00 );	
   SX1276LoRaSetOpMode( CAD_mode );
   lpTypefunc.paSwitchCmdfunc(rxOpen);
}
	
/*************************************************************
  Function   ：RF_SLEEP  
  Description：lora sleep模式进入 
  Input      : none 
  return     : none   
*************************************************************/
void RF_SLEEP(void)
{
   SX1276LoRaSetOpMode( Stdby_mode );
   SX1276WriteBuffer(REG_LR_IRQFLAGSMASK,  IRQN_SLEEP_Value);  //打开Sleep中断
   SX1276WriteBuffer( REG_LR_DIOMAPPING1, 0X00 );
   SX1276WriteBuffer( REG_LR_DIOMAPPING2, 0X00 );	
   SX1276LoRaSetOpMode( Sleep_mode );
}

//**************下面是中断里面处理的代码**********************
/*************************************************************
  Function   ：SX1278_Interupt  
  Description：lora对应中断标志处理，TXDONE RXDONE CADDONE ```
  Input      : none 
  return     : none   
*************************************************************/
void SX1278_Interupt(void)
{
	uint8_t fifo_addr=0x00;
	uint8_t rxaddrptr=0x00;
	unsigned char RF_REC_RLEN_i =0;
	unsigned char   CRC_Value;
	//unsigned char   RF_EX0_STATUS;
	
  RF_EX0_STATUS=SX1276ReadBuffer( REG_LR_IRQFLAGS ); 
	
  if((RF_EX0_STATUS&0x40)==0x40){														//接收 RX Done
		if((RF_EX0_STATUS&0x20)==0x20)                          //负载CRC错误
		{
			flag=0x04;
			
			
				rxaddrptr=SX1276ReadBuffer( REG_LR_FIFORXCURRENTADDR );
				SX1278_RLEN = SX1276ReadBuffer(REG_LR_NBRXBYTES);
				SX1276WriteBuffer (REG_LR_FIFOADDRPTR,rxaddrptr);
							
				for(RF_REC_RLEN_i=0;RF_REC_RLEN_i<SX1278_RLEN;RF_REC_RLEN_i++){
					recv[RF_REC_RLEN_i]=SX1276ReadBuffer( fifo_addr );							
				}
				len = 	SX1278_RLEN;	
				if(len)
				{
					Lora_RecDataGet(Loradata,len);
					Recv_Num++;
				}
			
			RS485_TX;	
			printf("A packet with wrong CRC!\r\n");
			RS485_RX;
		}
		else if((RF_EX0_STATUS&0x80)==0x80)
		{
			RS485_TX;
			printf("Rx Time Out!\r\n");
			RS485_RX;
		}
		else if((RF_EX0_STATUS&0x10)==0x10)
		{
			RS485_TX;
			printf("Receive an invalid header!\r\n");
			RS485_RX;
		}
		else
		{
			flag=0x01;
			CRC_Value=SX1276ReadBuffer( REG_LR_MODEMCONFIG2 );
			if((CRC_Value&0x04)==0x04){
				rxaddrptr=SX1276ReadBuffer( REG_LR_FIFORXCURRENTADDR );
				SX1278_RLEN = SX1276ReadBuffer(REG_LR_NBRXBYTES);
				SX1276WriteBuffer (REG_LR_FIFOADDRPTR,rxaddrptr);
							
				for(RF_REC_RLEN_i=0;RF_REC_RLEN_i<SX1278_RLEN;RF_REC_RLEN_i++){
					recv[RF_REC_RLEN_i]=SX1276ReadBuffer( fifo_addr );							
				}
				len = 	SX1278_RLEN;	
				if(len)
				{
					Lora_RecDataGet(Loradata,len);
					Recv_Num++;
					//串口发送接收信息
					//HAL_UART_Transmit_IT(&huart1,&Recv_Num,1);
					//HAL_UART_Transmit(&huart1,Loradata,len,1000);
				}
			}  
			else
			{
				RS485_TX;
				printf("CRC is not set!\r\n");		
				RS485_RX;
			}
		}
    SX1276LoRaSetOpMode( Stdby_mode );
    SX1276WriteBuffer(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value);  //打开RXD中断
    SX1276WriteBuffer(REG_LR_HOPPERIOD,    PACKET_MIAX_Value);
    SX1276WriteBuffer( REG_LR_DIOMAPPING1, 0X00 );
    SX1276WriteBuffer( REG_LR_DIOMAPPING2, 0x00 );	
    SX1276LoRaSetOpMode( Receiver_mode );
		flag=0x00;
  }else if((RF_EX0_STATUS&0x08)==0x08){											// TX Done
		flag=0x02;
    SX1276LoRaSetOpMode( Stdby_mode );
    SX1276WriteBuffer(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);  //打开RXD中断
    SX1276WriteBuffer(REG_LR_HOPPERIOD,   PACKET_MIAX_Value );
    SX1276WriteBuffer( REG_LR_DIOMAPPING1, 0X00 );
    SX1276WriteBuffer( REG_LR_DIOMAPPING2, 0x00 );	
    SX1276LoRaSetOpMode( Receiver_mode );
		flag=0x00;
  }else if((RF_EX0_STATUS&0x04)==0x04){  										// CAD Done
		flag=0x03;
    if((RF_EX0_STATUS&0x01)==0x01){     //表示CAD 检测到扩频信号 模块进入了接收状态来接收数据
      SX1276LoRaSetOpMode( Stdby_mode );
      SX1276WriteBuffer(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);  //打开RXD中断
      SX1276WriteBuffer(REG_LR_HOPPERIOD,   PACKET_MIAX_Value );
      SX1276WriteBuffer( REG_LR_DIOMAPPING1, 0X02 );
      SX1276WriteBuffer( REG_LR_DIOMAPPING2, 0x00 );	
      SX1276LoRaSetOpMode( Receiver_mode );
    }else{                           // 没检测到
      SX1276LoRaSetOpMode( Stdby_mode );
      SX1276WriteBuffer(REG_LR_IRQFLAGSMASK,  IRQN_SLEEP_Value);  //打开sleep中断
      SX1276WriteBuffer( REG_LR_DIOMAPPING1, 0X00 );
      SX1276WriteBuffer( REG_LR_DIOMAPPING2, 0X00 );	
      SX1276LoRaSetOpMode( Sleep_mode );
    }
		flag=0x00;
  }	
//	else if(((RF_EX0_STATUS&0x20)==0x20)||((RF_EX0_STATUS&0x60)==0x60)){                     //PayloadCRC错误
//		printf("Payload CRC is wrong!\r\n");
//		flag=0x04;
//    SX1276LoRaSetOpMode( Stdby_mode );
//    SX1276WriteBuffer(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);  //打开RXD中断
//    SX1276WriteBuffer(REG_LR_HOPPERIOD,   PACKET_MIAX_Value );
//    SX1276WriteBuffer( REG_LR_DIOMAPPING1, 0X02 );
//    SX1276WriteBuffer( REG_LR_DIOMAPPING2, 0x00 );	
//    SX1276LoRaSetOpMode( Receiver_mode );
//		flag=0x00;
//  }	
	else{
		flag=0x05;
    SX1276LoRaSetOpMode( Stdby_mode );
    SX1276WriteBuffer(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);  //打开RXD中断
    SX1276WriteBuffer(REG_LR_HOPPERIOD,   PACKET_MIAX_Value );
    SX1276WriteBuffer( REG_LR_DIOMAPPING1, 0X02 );
    SX1276WriteBuffer( REG_LR_DIOMAPPING2, 0x00 );	
    SX1276LoRaSetOpMode( Receiver_mode );
		flag=0x00;
//    lpTypefunc.paSwitchCmdfunc(rxOpen);
  }
  SX1276WriteBuffer( REG_LR_IRQFLAGS, 0xff  );
//	RF_EX0_STATUS=SX1276ReadBuffer( REG_LR_IRQFLAGS );
//	RF_EX0_STATUS++;
//	RF_EX0_STATUS++;
}
/*************************************************************
  Function   ：Lora_Init  
  Description：lora初始化，GPIO 、通信参数、复位
  Input      : none 
  return     : none   
*************************************************************/
void Lora_Init(void)
{
	 Lora_GPIO_Config();
	 register_rf_func(&ctrlTypefunc);
   SX1276Reset();
   SX1276LORA_INT();
}
/*************************************************************
  Function   ：Lora_Send  
  Description：发送一定长度的数据
  Input      : p_send_buf -待发送数据buffer  len --待发送数据长度
  return     : none    
*************************************************************/
void Lora_Send(unsigned char *p_send_buf,unsigned char len)
{
	FUN_RF_SENDPACKET(p_send_buf,len);
}
void Lora_RecDataGet(unsigned char *p_recdata,unsigned char len)
{
	memcpy(p_recdata,recv,len);
	memset(recv,0,len);
}

/*************************************************************
  Function   ：Lora_GetNumofRecData  
  Description：获取接收到数据长度
  Input      : 无  
  return     : len-接收数据长度    
*************************************************************/
unsigned char Lora_GetNumofRecData(void)
{
	unsigned char len;
	
	if(SX1278_RLEN) 
	{
		len = SX1278_RLEN;
		SX1278_RLEN = 0;
	}
	else 
	{
		len = 0;
	}
	return len;	
}


/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
