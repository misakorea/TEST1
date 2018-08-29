/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "main.h"
#include "stm32l1xx_hal.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
//#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define	TX 0
#define	RX 1

uint8_t mode;

uint8_t send_msg[10];
uint8_t serial_num=0x30,cicular=0x30;

//#define  RS485_TX      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_SET)	
//#define  RS485_RX      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_RESET)

#define MAX_LEN 20  //最大输入指令长度

int return_flag=0;
RECEIVE_CMD rx_cmd;

/////////////////读取温度//////////////////////
int32_t AD_Value;float temp,temp_value;

uint16_t *TS_CAL1=(uint16_t *)0x1FF8007A;
uint16_t *TS_CAL2=(uint16_t *)0x1FF8007E;

int32_t temp1,temp2;

///////////////////////////////////////////////
//uint8_t test_spi,test_irq,reg_addr,asc2_tmp,bw;//reg_value_l,reg_value_h;
uint8_t test_spi,test_irq,reg_addr,asc2_tmp,bw,asc2_power,asc2_cr;
uint16_t asc2_freq;
uint8_t reg_value_hex[2];
uint8_t uart_flag;//串口完成标志位
uint8_t recv_len,cmd_len;//串口接收数，指令长度
uint8_t recv_len_2;


uint8_t uart_buff[MAX_LEN],cmd_buff[MAX_LEN];//保存指令
uint8_t uart2_buff[MAX_LEN];//

uint8_t ready=0,complt=0;//串口接收完成标志位
uint8_t ready_2=0,complt_2=0;//串口接收完成标志位

uint8_t reg_value,res=0x00;

//uint8_t error_tip[]={0xff,0xff,0xff,0xff,0xff};
uint8_t msg[]="hello\r\n";
uint8_t error_tip[]={0x58,0x58,0x58,0x58,0x58};
uint8_t newline[]={0x0d,0x0a};
uint8_t tmp[10]={0x00};
//uint8_t lora_send[] = {"123456"};

//uint8_t cmd_help[]="help";
//uint8_t cmd_read_sx_reg[]="read reg";
//uint8_t sep[2]={0x0d,0x0a};
//uint8_t sep[2]={65,66};

//uint8_t reg_addr[BUF_LEN];
uint8_t len;
uint8_t Loradata[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Usart1SendData_DMA(uint8_t *pdata, uint16_t Length);  
void TestIO(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
	
	HAL_UART_Receive_DMA(&huart1, UsartType1.usartDMA_rxBuf, RECEIVELEN);  
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		
	Lora_Init();
	
	mode=RX;
	if(mode==RX)
	{
		RS485_TX;
		printf("\r\nAIR Terminal(Rx) initialization complete\r\n");
		RS485_RX;
	}
	else 
	{
		RS485_TX;
		printf("\r\nGROUND Terminal(Tx) initialization complete\r\n");
		RS485_RX;
	}
	
	HAL_UART_Transmit(&huart2,error_tip,sizeof(error_tip),1000);
	HAL_UART_Transmit(&huart2,newline,sizeof(newline),1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		//TestIO();
		if(mode==RX)
		{
			//HAL_Delay(1000);
	///////////////////////////////////打印接收信息///////////////////////////////////////////
			if(len>0)
			{
				RS485_TX;
				HAL_UART_Transmit(&huart1,Loradata,len,1000);
				RS485_RX;
				len=0;
				memset(Loradata,0,sizeof(Loradata));
			}
		}
		else if ( mode == TX)
		{
			memset(send_msg,serial_num,7);
			memset(&send_msg[7],cicular,1);
			memset(&send_msg[8],0x0d,1);
			memset(&send_msg[9],0x0a,1);
			if(serial_num<0x39)
				serial_num++;
			else
			{
				serial_num=0x30;
				cicular++;
				if(cicular>0x39)
					cicular=0x30;
			}				
			Lora_Send(send_msg,10);
			HAL_Delay(3000);			
		}
			
//////////////////////////////////////////////////////////////////////////////////////////

				if(UsartType1.receive_flag)//产生空闲中断 
        {  
					UsartType1.receive_flag=0;//清零标记
		//////////////////////////////////////////////		
					if(UsartType1.rx_len==4)
					{
						//reg_addr=asc2Hex(UsartType1.usartDMA_rxBuf,2);
						reg_addr=asc2base(UsartType1.usartDMA_rxBuf,2,16);
						if(reg_addr<0x71)
						{
							reg_value=SX1276ReadBuffer(reg_addr);
							reg_value_hex[0]=Hex2asc_h(reg_value);
							reg_value_hex[1]=Hex2asc_l(reg_value);
							RS485_TX;
							HAL_UART_Transmit(&huart1,((uint8_t *)"0x"),2,1000);
							HAL_UART_Transmit(&huart1,reg_value_hex,2,1000);
							printf("\r\n");
							RS485_RX;
//							reg_value=SX1276ReadBuffer(reg_addr);
//							reg_value_hex[0]=Hex2asc_h(reg_value);
//							reg_value_hex[1]=Hex2asc_l(reg_value);
//							printf("%s\r\n",reg_value_hex);
						}						
						else 
						{
							RS485_TX;
							printf("%s\r\n",error_tip);
							RS485_TX;
						}
					}	
					//////////////////////////////////配置参数///////////////////////////////////////
					else if((!memcmp(rx_cmd.cmdBuf,"sf",2)) || (!memcmp(rx_cmd.cmdBuf,"SF",2))) //改扩频因子
					{
						if(rx_cmd.cmd_Len==6)
							asc2_tmp=asc2base(&rx_cmd.cmdBuf[3],1,10);
						else if(rx_cmd.cmd_Len==7)
							asc2_tmp=asc2base(&rx_cmd.cmdBuf[3],2,10);
							Chg_SF(asc2_tmp);
					}
					
					else if((!memcmp(rx_cmd.cmdBuf,"bw",2)) || (!memcmp(rx_cmd.cmdBuf,"BW",2))) //改带宽
					{
						if(!memcmp(&rx_cmd.cmdBuf[3],"7.8",3))   bw=0x00;
						else if(!memcmp(&rx_cmd.cmdBuf[3],"10.4",4))  bw=0x01;
						else if(!memcmp(&rx_cmd.cmdBuf[3],"15.6",4))  bw=0x02;
						else if(!memcmp(&rx_cmd.cmdBuf[3],"20.8",4))  bw=0x03;
						else if(!memcmp(&rx_cmd.cmdBuf[3],"31.25",5)) bw=0x04;
						else if(!memcmp(&rx_cmd.cmdBuf[3],"41.7",4))  bw=0x05;
						else if(!memcmp(&rx_cmd.cmdBuf[3],"62.5",4))  bw=0x06;
						else if(!memcmp(&rx_cmd.cmdBuf[3],"125",3))   bw=0x07;
						else if(!memcmp(&rx_cmd.cmdBuf[3],"250",3))   bw=0x08;
						else if(!memcmp(&rx_cmd.cmdBuf[3],"500",3))   bw=0x09;
						else bw=0xff;
						Chg_BW(bw);
					}
					
					else if((!memcmp(rx_cmd.cmdBuf,"pw",2)) || (!memcmp(rx_cmd.cmdBuf,"PW",2))) //改功率
					{
						if(rx_cmd.cmd_Len==6)
							asc2_power=asc2base(&rx_cmd.cmdBuf[3],1,10);
						else if(rx_cmd.cmd_Len==7)
							asc2_power=asc2base(&rx_cmd.cmdBuf[3],2,10);
						Chg_Power(asc2_power);
					}
			
					else if((!memcmp(rx_cmd.cmdBuf,"freq",4)) || (!memcmp(rx_cmd.cmdBuf,"FREQ",4))) //改频率
					{
						//rx_cmd.cmd_Len++;
						if(rx_cmd.cmd_Len==10)
							asc2_freq=(asc2base(&rx_cmd.cmdBuf[5],1,10))*100+(asc2base(&rx_cmd.cmdBuf[6],1,10))*10+asc2base(&rx_cmd.cmdBuf[7],1,10);
						else if(rx_cmd.cmd_Len==11)
							asc2_freq=(asc2base(&rx_cmd.cmdBuf[5],1,10))*1000+(asc2base(&rx_cmd.cmdBuf[6],1,10))*100+(asc2base(&rx_cmd.cmdBuf[7],1,10))*10+asc2base(&rx_cmd.cmdBuf[8],1,10);
						Chg_Freq(asc2_freq);
					}	


					else if((!memcmp(rx_cmd.cmdBuf,"cr",2)) || (!memcmp(rx_cmd.cmdBuf,"CR",2))) //改编码率
					{
						asc2_cr=asc2base(&rx_cmd.cmdBuf[3],1,10);
						Chg_CR(asc2_cr);
					}			

					else if ((!memcmp(rx_cmd.cmdBuf,"tx_mode",7)) || (!memcmp(rx_cmd.cmdBuf,"TX_MODE",7))) //改变收发状态
					{
						mode=TX;
						RS485_TX;
						printf("RF Mode has changed to TX\r\n");
						RS485_RX;				
					}
			
					else if ((!memcmp(rx_cmd.cmdBuf,"rx_mode",7)) || (!memcmp(rx_cmd.cmdBuf,"RX_MODE",7))) //改变收发状态
					{
						mode=RX;
						RS485_TX;
						printf("RF Mode has changed to RX\r\n");
						RS485_RX;
					}
					
					////////////////////////////////////////////////////////////////////////
					else
					{
						if((!memcmp(UsartType1.usartDMA_rxBuf,"temp",4)) || (!memcmp(UsartType1.usartDMA_rxBuf,"TEMP",4)))
							Show_Temp();
						else
						{
							RS485_TX;
							printf("%s\r\n",error_tip);
							RS485_RX;
						}
					}
        }  
	
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	SX1278_Interupt();
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//void UART_IDLECallBack(UART_HandleTypeDef *huart)
{
	if (huart->Instance==USART1)
	{
		uart_buff[recv_len]=res;
		recv_len++;	
		//if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
		switch(res)
		{			
			case 0x0d :
				if(ready==0)
				{
					ready=1;
				}
				else
				{
					ready=0;memset(uart_buff,0,sizeof(MAX_LEN));recv_len=0;
				}
				break;
				
			case 0x0a :
				if(ready==1)
				{
					complt=1;
				}
				else 
				{
					memset(uart_buff,0,MAX_LEN);recv_len=0;
				}
				break;

			default :
				break;				
		}	
		if(complt==1)
		{
			memcpy(cmd_buff,uart_buff,MAX_LEN);
			memset(uart_buff,0,MAX_LEN);
			cmd_len=recv_len;uart_flag=1;
			complt=0;ready=0;recv_len=0;
		}
	HAL_UART_Receive_IT(&huart1,&res,1);	
	}
	if (huart->Instance==USART2)
	{
//		if((tmp[8]==0x0d)&&(tmp[9]==0x0a))
//			HAL_UART_Transmit(&huart2,msg,sizeof(msg),1000);
//		HAL_UART_Receive_IT(&huart2,tmp,10);
	}
			
}

////DMA发送函数
//void Usart1SendData_DMA(uint8_t *pdata, uint16_t Length)  
//{  
//    while(UsartType1.dmaSend_flag == USART_DMA_SENDING);  
//    UsartType1.dmaSend_flag = USART_DMA_SENDING;  
//    HAL_UART_Transmit_DMA(&huart1, pdata, Length);  
//}  
//  
////DMA发送完成中断回调函数
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)  
//{  
//     __HAL_DMA_DISABLE(huart->hdmatx);  
//    UsartType1.dmaSend_flag = USART_DMA_SENDOVER;  
//}  

////DMA接收空闲中断
//void UsartReceive_IDLE(UART_HandleTypeDef *huart)  
//{  
//    uint32_t temp;  
//  
//    if((__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET))  
//    {   
//        __HAL_UART_CLEAR_IDLEFLAG(&huart1);  
//        HAL_UART_DMAStop(&huart1);  
//        //temp = huart1.hdmarx->Instance->NDTR;
//				temp = huart1.hdmarx->Instance->CNDTR;
//        UsartType1.rx_len =  RECEIVELEN - temp;   
//        UsartType1.receive_flag=1;  
//        HAL_UART_Receive_DMA(&huart1,UsartType1.usartDMA_rxBuf,RECEIVELEN);  
//    }  
//}  

void Show_Temp(void)
{
//	uint8_t i,times;
//	times=50;
//	for(i=0;i<times;i++)
//	{
//		HAL_ADC_PollForConversion(&hadc, 50);
//		if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc), HAL_ADC_STATE_REG_EOC))
//		{
//			AD_Value = HAL_ADC_GetValue(&hadc);
//			temp1=((int32_t)(*TS_CAL1));
//			temp2=((int32_t)(*TS_CAL2));
//			temp_value=((110-30)/(((float)temp2)-((float)temp1)))*(AD_Value-(*TS_CAL2))+110;
//			
//			//AD_Value = ((HAL_ADC_GetValue(&hadc))*3300/4096-760)/2.5+25;
//			//printf("MCU Temperature : %.1f℃\r\n",((AD_Value*3300/4096-760)/2.5+25));
//			//printf("MCU Temperature : %.1f℃\r\n",AD_Value);
//		}	
//		temp=temp+temp_value;
//		if(i==(times-1))
//		{
//			temp=temp/times;
//			printf("Temprature : %f℃\r\n",temp);		
//		}
//	}
}

/*------------------------------------------------------/
将输入的ASCII字符直接转化为字面数字
参数；1.需要转换字符其实地址；2.转换字节个数；3.转换进制(10,16)
输出：转换完成字节
*------------------------------------------------------*/
uint8_t asc2base(uint8_t * cmd, uint8_t len, uint8_t base)
{
	uint8_t a,asc2base=0,temp=0;
	for(a=0;a<len;a++)
	{
		temp=cmd[a];
		if((cmd[a]>47)&&(cmd[a]<58)) temp=temp-48;
		if((cmd[a]>64)&&(cmd[a]<71)) temp=temp-55;
		if((cmd[a]>96)&&(cmd[a]<103)) temp=temp-87;				
		asc2base=asc2base*base+temp;
	}
	return asc2base;
//	uint8_t a,asc2base=0;
//	for(a=0;a<len;a++)
//	{
//		if((cmd[a]>47)&&(cmd[a]<58)) cmd[a]=cmd[a]-48;
//		if((cmd[a]>64)&&(cmd[a]<71)) cmd[a]=cmd[a]-55;
//		if((cmd[a]>96)&&(cmd[a]<103)) cmd[a]=cmd[a]-87;				
//		asc2base=asc2base*base+cmd[a];
//	}
//	return asc2base;
}


///*------------------------------------------------------/
//将asc字符转化为16进制数
//参数1.需要转换字符其实地址
//参数2.转换字节个数
//*------------------------------------------------------*/
//uint8_t asc2Hex(uint8_t * cmd, uint8_t len)
//{
//	uint8_t a,asc2hex=0;
//	for(a=0;a<len;a++)
//	{
//		if((cmd[a]>47)&&(cmd[a]<58)) cmd[a]=cmd[a]-48;
//		if((cmd[a]>64)&&(cmd[a]<71)) cmd[a]=cmd[a]-55;
//		if((cmd[a]>96)&&(cmd[a]<103)) cmd[a]=cmd[a]-87;
//		
//		asc2hex=(asc2hex<<(a*4))+cmd[a];
//	}
//	return asc2hex;
//}

uint8_t Hex2asc_l(uint8_t data)
{
	uint8_t low;
	low=(data&0x0f);
	low=(low>9)?(low+87):(low+48);
	return low;
}

uint8_t Hex2asc_h(uint8_t data)
{
	uint8_t high;
	high=(data>>4);
	high=(high>9)?(high+87):(high+48);
	return high;
}

void Chg_SF(uint8_t sf)
{
	if((asc2_tmp>5)&&(asc2_tmp<13))
	{
		SX1276LoRaSetOpMode(Stdby_mode);
		SX1276LoRaSetSpreadingFactor(sf);	
		RF_RECEIVE(); 
		RS485_TX;
		printf("SF has changed to %d\r\n",sf);	
		RS485_RX;
	}
	else 
	{
		RS485_TX;
		printf("SF input incorrect...(6~12)\r\n");
		RS485_RX;
	}
}

void Chg_BW(uint8_t bw)
{
	if(bw!=0xff)
	{
		SX1276LoRaSetOpMode(Stdby_mode);
		SX1276LoRaSetSignalBandwidth( bw );	
		RF_RECEIVE();
		RS485_TX;
		printf("BW has changed to %s",&rx_cmd.cmdBuf[3]);
		RS485_RX;
		//HAL_UART_Transmit(&huart1,&rx_cmd.cmdBuf[3],(rx_cmd.cmd_Len-5),1000);
		//printf("\r\n");	
	}
	else 
	{
		RS485_TX;
		printf("BW input incorrect... ( 7.8  10.4  15.6  20.8  31.25  41.7  62.5  125  250  500  )\r\n");
		RS485_RX;
	}
}

void Chg_Power(uint8_t power)  //Pout=(10.8+0.6*max)-(15-output)
{
	uint8_t powervalue,max,output;
	float output_tmp=0;
	switch (power)
	{
		case 0: // min (-4.2dbm)
			max=0;output=0;
			break;
		case 1: // -3.2dbm
			max=0;output=1; 
			break;
		case 2: // -2.2dbm
			max=0;output=2; 
			break;		
		case 3: // -1.2dbm
			max=0;output=3; 
			break;	
		case 4: // -0.2dbm
			max=0;output=4; 
			break;	
		case 5: // 0dbm
			max=7;output=0; 
			break;
		case 6: // 1dbm
			max=7;output=1; 
			break;
		case 7: // 2dbm
			max=7;output=2; 
			break;
		case 8: // 3dbm
			max=7;output=3; 
			break;
		case 9: // 4dbm
			max=7;output=4; 
			break;
		case 10: // 5dbm
			max=7;output=5; 
			break;
		case 11: // 6dbm
			max=7;output=6; 
			break;
		case 12: // 7dbm
			max=7;output=7; 
			break;		
		case 13: // 8dbm
			max=7;output=8; 
			break;
		case 14: // 9dbm
			max=7;output=9; 
			break;
		case 15: // 10dbm
			max=7;output=10; 
			break;	
		case 16: // 11dbm
			max=7;output=11; 
			break;	
		case 17: // 12dbm
			max=7;output=12; 
			break;			
		case 18: // 13dbm
			max=7;output=13; 
			break;			
		case 19: // 14dbm
			max=7;output=14; 
			break;			
		case 20: // 15dbm
			max=7;output=15; 
			break;	
		default:
			max=0xff;
			break;
	}
	
	if(max==0xff)
	{
		RS485_TX;
		printf("Power input incorrect... (0~20)\r\n");
		RS485_RX;
	}
	else
	{
		powervalue=(max<<4)+output;
		SX1276LoRaSetOpMode(Stdby_mode);
		SX1276WriteBuffer( REG_LR_PACONFIG,  powervalue);			
		output_tmp=(10.8+0.6*max)-(15-output);	
		RS485_TX;
		printf("Power has changed to %.1f dBm!\r\n",output_tmp);
		RS485_RX;
	}
		
//	if(power<15)
//	{
//		powervalue=0x50+(power+1);
//		SX1276LoRaSetOpMode(Stdby_mode);
//		SX1276WriteBuffer( REG_LR_PACONFIG,  powervalue);
//		if(rx_cmd.cmd_Len==6)
//		{
//			output_tmp=(float)asc2base(&rx_cmd.cmdBuf[3],1,10)-0.2;
//			RS485_TX;
//			printf("Power has changed to %.1f dBm!\r\n",output_tmp);
//			RS485_RX;
//			//HAL_UART_Transmit(&huart1,&rx_cmd.cmdBuf[3],(rx_cmd.cmd_Len-5),1000);
//			//printf(" dbm\r\n");		
//		}
//		else if(rx_cmd.cmd_Len==7)
//		{
//			output_tmp=(asc2base(&rx_cmd.cmdBuf[3],2,10)-0.2);
//			RS485_TX;
//			printf("Power has changed to %.1f dBm!\r\n",output_tmp);
//			RS485_RX;
//		}

//	}
//	else
//	{
//		RS485_TX;
//		printf("Power input incorrect... (0~14)\r\n");
//		RS485_RX;
//	}
}

//void Chg_Power(uint8_t power)
//{
//	uint8_t powervalue;
//	float output_tmp=0;
//	if(power<15)
//	{
//		powervalue=0x50+(power+1);
//		SX1276LoRaSetOpMode(Stdby_mode);
//		SX1276WriteBuffer( REG_LR_PACONFIG,  powervalue);
//		if(rx_cmd.cmd_Len==6)
//		{
//			output_tmp=(float)asc2base(&rx_cmd.cmdBuf[3],1,10)-0.2;
//			RS485_TX;
//			printf("Power has changed to %.1f dBm!\r\n",output_tmp);
//			RS485_RX;
//			//HAL_UART_Transmit(&huart1,&rx_cmd.cmdBuf[3],(rx_cmd.cmd_Len-5),1000);
//			//printf(" dbm\r\n");		
//		}
//		else if(rx_cmd.cmd_Len==7)
//		{
//			output_tmp=(asc2base(&rx_cmd.cmdBuf[3],2,10)-0.2);
//			RS485_TX;
//			printf("Power has changed to %.1f dBm!\r\n",output_tmp);
//			RS485_RX;
//		}

//	}
//	else
//	{
//		RS485_TX;
//		printf("Power input incorrect... (0~14)\r\n");
//		RS485_RX;
//	}
//}

void Chg_Freq(uint16_t freq)
{
	uint8_t frq[3]={0,0,0};
	uint32_t output_tmp;
	if((freq<1021)&&(freq>136))
	{
		output_tmp=freq<<14;
		frq[0]=output_tmp&(0x000000ff);
		frq[1]=(output_tmp&(0x0000ff00))>>8;
		frq[2]=(output_tmp&(0x00ff0000))>>16;
		SX1276WriteBuffer( REG_LR_FRFMSB, frq[2]);
		SX1276WriteBuffer( REG_LR_FRFMID, frq[1]);
		SX1276WriteBuffer( REG_LR_FRFLSB, frq[0]);
		RS485_TX;
		printf("Frequency has changed to %s",&rx_cmd.cmdBuf[5]);
		RS485_RX;
	}
	else 
	{
		RS485_TX;
		printf("Frequency input incorrect...( 137~1020 )\r\n");
		RS485_RX;
	}
}

void Chg_CR(uint8_t CR)
{
	if(CR<5)
	{
		SX1276LoRaSetOpMode(Stdby_mode);	
		//SX1276LoRaSetErrorCoding(CR);
		
	 unsigned char RECVER_DAT;
	 RECVER_DAT=SX1276ReadBuffer( REG_LR_MODEMCONFIG1);
	 RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG1_CODINGRATE_MASK ) | ( CR << 1 );
	 RECVER_DAT=RECVER_DAT | 1 ;
	 SX1276WriteBuffer( REG_LR_MODEMCONFIG1, RECVER_DAT);
		
		RF_RECEIVE(); 
		RS485_TX;
		printf("CR has changed to %s",&rx_cmd.cmdBuf[3]);
		RS485_RX;
	}
	else 
	{
		RS485_TX;
		printf("CR input incorrect... ( 1~4 )\r\n");
		RS485_RX;
	}
}

void TestIO(void)
{
///////////////////////////////IO拉高////////////////////////////////////
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_SET);	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12,GPIO_PIN_SET);	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,GPIO_PIN_SET);	
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_SET);	
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);	
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_SET);		
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_SET);	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_SET);		
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_SET);		
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12,GPIO_PIN_SET);	
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,GPIO_PIN_SET);	
	
	HAL_Delay(1000);
	
///////////////////////////////IO拉低////////////////////////////////////
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12,GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,GPIO_PIN_RESET);	
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_RESET);	
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);	
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_RESET);		
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_RESET);		
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_RESET);		
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12,GPIO_PIN_RESET);	
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,GPIO_PIN_RESET);
}
//void Chg_BW(uint8_t bw)
//{
//	SX1276LoRaSetOpMode(Stdby_mode);
//  SX1276LoRaSetSignalBandwidth( bw );	
//	RF_RECEIVE(); 
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
