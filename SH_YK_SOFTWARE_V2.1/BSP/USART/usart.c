#include "usart.h"
#include "delay.h"
#include "LED.h"
#include "stdarg.h"	
#include "string.h"	
#include <stdio.h>

FILE* COM1 = (FILE*)0x01;
FILE* COM2 = (FILE*)0x02;
FILE* COM3 = (FILE*)0x03;


#if (defined DEBUG)
//static uint32_t PC_RE_NUM = 0;
#endif

extern unsigned int YK_Rev_LOST_TIME;

#define USART1_TX_BUFF_SIZE  32  //发送缓冲区
#define USART1_RX_BUFF_SIZE  128 //接收缓冲区大于一秒存放的数据量  2*5
#define YK_REV_Size 25
#define YK_Tra_Size 5

#define YK_TxBuff_Size USART1_TX_BUFF_SIZE
uint8_t YK_TX[YK_TxBuff_Size]={0xFF,0x00, 0x00, 0x00, 0xEE};//发送帧缓冲区
#define YK_RxBuff_Size USART1_RX_BUFF_SIZE
uint8_t YK_RX[YK_RxBuff_Size];//接收数据存放缓冲区
uint8_t YK_REV[YK_REV_Size];//存放从主机发送过来的数据帧（在接收中断中解析成功的数据帧）
uint8_t YK_Rx_Flag=1;//产生接收中的标志
uint8_t YK_Tx_Flag=1;//发送成功的标志
uint8_t YK_REV_Flag=1;//解析到一个正确的数据帧的标志


void USART1_Config(u32 Baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
//----------------------------------------------------------------------------------------------------------------------------------------
	//配置无线模块M0端口
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
   //	
   //	重映射的方式一共有三种。分别描述如下：
   //1.GPIO_Remap_SWJ_JTAGDisable： /*!< JTAG-DP Disabled and SW-DP Enabled */  即能用PB3，PB4，PA15做普通IO，PA13&14用于SWD调试
   //2.GPIO_Remap_SWJ_Disable：  /*!< Full SWJ Disabled (JTAG-DP + SW-DP) */  5个引脚全为普通引脚，但不能再用JTAG&SWD仿真器调试，只能用st-link调试 
   //3.GPIO_Remap_SWJ_NoJTRST： /*!< Full SWJ Enabled (JTAG-DP + SW-DP) but without JTRST */PB4可为普通IO口，JTAG&SWD正常使用，但JTAG没有复位
   //如果你用到所有的五个引脚当做普通IO口，那么上述步骤二中的重映射配置应写为GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE); 
   //如果你用PB3，PB4，PA15做普通IO，PA13&14用于SWD调试，则重映射配置应写为GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 同理可配置只用PB4可为普通IO口的情况
   //	
   //	
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//
	//选E62电台

   //#ifdef 	E62  //MO=0
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC  , ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	//选择对应的引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);  //初始化M0端口
  GPIO_ResetBits(GPIOC, GPIO_Pin_11);	 // M0=0
	
	//配置无线模块LOCK端口
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC  , ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	//选择对应的引脚
  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IN_FLOATING;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);  //初始化LOCK端口
	//GPIO_ResetBits(GPIOC, GPIO_Pin_10);	 //LOCK
	
	//配置无线模块AUX端口
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA  , ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	//选择对应的引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化AUX端口
  //GPIO_ResetBits(GPIOA, GPIO_Pin_15);	 // 关闭AUX 

	//配置无线模块WX_EN PA8端口
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA  , ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	//选择对应的引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化AUX端口
   GPIO_SetBits(GPIOA, GPIO_Pin_8);	 // 打开无线电台的电源 
	//-------------------------------------------------------------------------------------------------------
	////////////////////////////////////////////////////////////////////////////////////////////
   /* 第1步：打开GPIO和USART部件的时钟 */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);// 使能 USART1 时钟
   /* 第2步：将USART Tx的GPIO配置为推挽复用模式 */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   /* 第3步：将USART Rx的GPIO配置为浮空输入模式
   由于CPU复位后，GPIO缺省都是浮空输入模式，因此下面这个步骤不是必须的
   但是，我还是建议加上便于阅读，并且防止其它地方修改了这个口线的设置参数
   */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  ////////////////////////////////////////////////////////////////////////////////////////////
    /* 第4步：配置USART参数
    - BaudRate = 115200 baud
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
    */
    USART_InitStructure.USART_BaudRate = Baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    //空闲中断
    USART_ITConfig(USART1, USART_IT_IDLE , ENABLE);
	 //错误中断
    USART_ITConfig(USART1, USART_IT_ERR | USART_IT_ORE | USART_IT_NE | USART_IT_FE ,ENABLE);
   //第5步：使能 USART， 配置完毕//
    USART_Cmd(USART1, ENABLE);  
   //CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
    //如下语句解决第1个字节无法正确发送出去的问题 //
   // USART_ClearFlag(USART1, USART_FLAG_TC); // 清发送外城标志，Transmission Complete flag //
		USART_ClearFlag(USART1, USART_FLAG_TC|USART_FLAG_RXNE|USART_FLAG_IDLE|USART_FLAG_ORE|USART_FLAG_NE|USART_FLAG_FE |USART_FLAG_PE ); 

  ////////////////////////////////////////////////////////////////////////////////////////////
  /* DMA clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//DMA1
  /* DMA1 Channel4 (triggered by USART1 Tx event) Config */
  DMA_DeInit(DMA1_Channel4);  
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)(&USART1->DR);// 0x40013804;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)YK_TX;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = YK_TxBuff_Size;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);
  DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);//传输完成中断
  DMA_ITConfig(DMA1_Channel4, DMA_IT_TE, ENABLE);//传输错误中断
	DMA_ClearITPendingBit(DMA1_IT_GL4 | DMA1_IT_TC4| DMA1_IT_TE4 | DMA1_IT_HT4);//
  //DMA_ClearFlag(DMA1_FLAG_GL4 | DMA1_FLAG_TC4 | DMA1_FLAG_TE4 | DMA1_FLAG_HT4);//
  /* Enable USART1 DMA TX request */
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
  DMA_Cmd(DMA1_Channel4, DISABLE);
	
	  ////////////////////////////////////////////////////////////////////////////////////////////
 /* DMA1 Channel5 (triggered by USART1 Rx event) Config */
  DMA_DeInit(DMA1_Channel5);  
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)(&USART1->DR);// 0x40013804;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)YK_RX;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = YK_RxBuff_Size;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);
  DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);
  DMA_ITConfig(DMA1_Channel5, DMA_IT_TE, ENABLE);
	DMA_ClearITPendingBit(DMA1_IT_GL5 | DMA1_IT_TC5| DMA1_IT_TE5 | DMA1_IT_HT5);//
  //DMA_ClearFlag(DMA1_FLAG_GL5 | DMA1_FLAG_TC5 | DMA1_FLAG_TE5 | DMA1_FLAG_HT5);//
  /* Enable USART1 DMA RX request */ 
  USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
  DMA_Cmd(DMA1_Channel5, ENABLE);
	
	
	////////////////////////////////////////////////////////////////////////////////////////////	
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  
  /* Enable the USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  
  //Enable DMA Channel4 TX Interrupt 
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
  /*Enable DMA Channel5 RX Interrupt */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


//USART1中断服务函数
void USART1_IRQHandler(void)
{
   u16 DATA_LEN;
   u16 i;
	 YK_Rx_Flag=0;//制造发送与接收互斥
   if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)//如果为空闲总线中断
    {
                DATA_LEN=USART1_RX_BUFF_SIZE-DMA_GetCurrDataCounter(DMA1_Channel5); 
              if(DATA_LEN >=YK_REV_Size)//如果收到足够多数据并且产生了空闲中断
                  {   
                      DMA_Cmd(DMA1_Channel5, DISABLE);		////关闭DMA,防止处理其间有数据						
									   if(YK_RX[0]==0xFF && YK_RX[YK_REV_Size-1]==0xEE)
									   {
											 LEDY(2);
									  	 memcpy(YK_REV,YK_RX,YK_REV_Size);//复制数据
											  YK_Rev_LOST_TIME=0;
									  	 YK_REV_Flag=1;//标记数据帧
									   }
									 //重启DMA
                   DMA_Cmd(DMA1_Channel5, DISABLE);
                   DMA_ClearFlag(DMA1_FLAG_GL5 | DMA1_FLAG_TC5 | DMA1_FLAG_TE5 | DMA1_FLAG_HT5);//清标志
                   DMA1_Channel5->CNDTR = USART1_RX_BUFF_SIZE;//重装填
                   DMA_Cmd(DMA1_Channel5, ENABLE);//处理完,重开DMA
									}					
//									 //重启DMA
//                   DMA_Cmd(DMA1_Channel5, DISABLE);
//                   DMA_ClearFlag(DMA1_FLAG_GL5 | DMA1_FLAG_TC5 | DMA1_FLAG_TE5 | DMA1_FLAG_HT5);//清标志
//                   DMA1_Channel5->CNDTR = USART1_RX_BUFF_SIZE;//重装填
//                   DMA_Cmd(DMA1_Channel5, ENABLE);//处理完,重开DMA
                   //读SR后读DR清除Idle----清除串口中断
                   i=USART1->SR;
                   i=USART1->DR;
									 i=i;	
     }

		            //见USART库文件905行
		            //USART_ClearFlag(USART1, USART_FLAG_IDLE|USART_FLAG_ORE|USART_FLAG_NE|USART_FLAG_FE |USART_FLAG_PE );//清除所有错误标志
                USART_ClearITPendingBit(USART1,  USART_IT_IDLE | USART_IT_PE | USART_IT_FE | USART_IT_NE|USART_IT_ERR|USART_IT_ORE);////清除所有错误中断位
		            YK_Rx_Flag=1;//接收中断处理函数执行完毕
}

void USART1_TIMEOUT_Handler(void)	
{
   u16 i;
//	 LED1(0);
//	 LED2(0);
   DMA_Cmd(DMA1_Channel5, DISABLE);//关闭DMA,防止处理其间有数据
	 YK_Rx_Flag=0;//制造发送与接收互斥
   //读SR后读DR清除Idle----清除串口中断
   i=USART1->SR;
   i=USART1->DR;
	 i=i;
   //见USART库文件905行
           USART_ClearFlag(USART1, USART_FLAG_TXE|USART_FLAG_TC|USART_FLAG_RXNE|USART_FLAG_IDLE|USART_FLAG_ORE|USART_FLAG_NE|USART_FLAG_FE |USART_FLAG_PE );//清除所有错误标志
   USART_ClearITPendingBit(USART1, USART_IT_TXE |USART_IT_TC|USART_IT_RXNE|USART_IT_IDLE | USART_IT_PE | USART_IT_FE | USART_IT_NE|USART_IT_ERR|USART_IT_ORE);////清除所有错误中断位
   //重启DMA接收传输通道 
   DMA_ClearFlag(DMA1_FLAG_GL5 | DMA1_FLAG_TC5 | DMA1_FLAG_TE5 | DMA1_FLAG_HT5);//清DMA通道所有标志
   DMA1_Channel5->CNDTR = USART1_RX_BUFF_SIZE;//重装填
   DMA_Cmd(DMA1_Channel5, ENABLE);//处理完,重开DMA  

	//重启DMA接收传输通道 
   YK_Tx_Flag=0;
   DMA_Cmd(DMA1_Channel4, DISABLE);//发送完毕并产生完成传输中断后关闭DMA
	 DMA_ClearITPendingBit(DMA1_IT_GL4 | DMA1_IT_TC4| DMA1_IT_TE4 | DMA1_IT_HT4);
   YK_Rx_Flag=1;//接收中断处理函数执行完毕
}


//DMA1_Channel5串口接收中断服务函数
void DMA1_Channel5_IRQHandler(void)
{
  DMA_ClearITPendingBit(DMA1_IT_GL5|DMA1_FLAG_TC5 | DMA1_IT_TE5 | DMA1_IT_HT5);
  DMA_Cmd(DMA1_Channel5, DISABLE);//关闭DMA,防止处理其间有数据
  DMA1_Channel5->CNDTR = USART1_RX_BUFF_SIZE;//重装填
  DMA_Cmd(DMA1_Channel5, ENABLE);//处理完,重开DMA
}

//DMA1_Channel4串口发送完成中断服务函数
//USART1使用DMA发数据中断服务程序
void DMA1_Channel4_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC4)){YK_Tx_Flag=1;LEDB(2);}//置DMA传输完成 //发送完成中断
  DMA_ClearITPendingBit(DMA1_IT_GL4 | DMA1_IT_TC4| DMA1_IT_TE4 | DMA1_IT_HT4);
  DMA_Cmd(DMA1_Channel4, DISABLE);//发送完毕并产生完成传输中断后关闭DMA
  YK_Tx_Flag=1;//置DMA传输完成
}
//遥控器开启一次数据帧的发送
void YK_CMD_TX(void)
	{
    DMA_Cmd(DMA1_Channel4, DISABLE); 
		DMA_ClearITPendingBit(DMA1_IT_GL4 | DMA1_IT_TC4| DMA1_IT_TE4 | DMA1_IT_HT4);
    DMA1_Channel4->CNDTR = (uint16_t)YK_Tra_Size; // 设置要发送的字节数目个5
    DMA_Cmd(DMA1_Channel4, ENABLE);        //开始DMA发送
	  YK_Tx_Flag=0;	
  }
	
void USART1_DMA_Tx_Data(unsigned char *t,unsigned  char Lenth)
{
	 DMA_Cmd(DMA1_Channel4, DISABLE);  // 关闭DMA通道
	 DMA_ClearITPendingBit(DMA1_IT_GL4 | DMA1_IT_TC4| DMA1_IT_TE4 | DMA1_IT_HT4);//这两节话加上后DMA发送出错导致系统不稳定的情况改善了，
	 memcpy(YK_TX,t,Lenth);
   DMA1_Channel4->CNDTR = (uint16_t)Lenth; // 设置要发送的字节数目个25
   DMA_Cmd(DMA1_Channel4, ENABLE);        //开始DMA发送
	 YK_Tx_Flag=0;
	
}

void WL_EN(unsigned char EN){if(EN)GPIO_SetBits(GPIOA, GPIO_Pin_8);	  else GPIO_ResetBits(GPIOA, GPIO_Pin_8); }
void WL_M0(unsigned char I ){if(I) GPIO_SetBits(GPIOC, GPIO_Pin_11);	else GPIO_ResetBits(GPIOC, GPIO_Pin_11);}
unsigned char WL_LOCK(void ){return  GPIO_ReadInputDataBit(GPIOC ,GPIO_Pin_10  );}
unsigned char WL_AUX(void)  {return  GPIO_ReadInputDataBit(GPIOA ,GPIO_Pin_15  );}
unsigned char WL_Config(unsigned char HEAD,\
	                      unsigned char ID,\
	                      unsigned char FHSS,\
	                      unsigned char BAUD,\
	                      unsigned char W_SPED,\
	                      unsigned char CHAN,\
	                      unsigned char IO_D,\
	                      unsigned char FEC,\
	                      unsigned char P_DB)
	{	
		 WL_EN(1);
		 WL_M0(1);
		 delay_ms(500);
      unsigned char Cmd[6]={0XC0,0X01,0X0A,0X1A,0X0A,0X44};
			//unsigned char Back_Para[3]={0XC1,0XC1,0XC1};
      //[0] HEAD 固定 0xC0 或 0xC2，表示此帧数据为控制命令 必须为 0xC0 或 C2
      //C0：所设置的参数会掉电保存。
      //C2：所设置的参数不会掉电保存。
      switch(HEAD)
      	{
      	  case 0xc0:{Cmd[0]=0xc0; break;}
      	  case 0xc2:{Cmd[0]=0xc2; break;}
      	  default : {Cmd[0]=0xc0; break;}
         }
      	
      //[1] ID 跳频序列 ID
      //（默认为 01H）
      //跳频 ID 确定跳频序列，双方必须一样。
      	 Cmd[1]=ID;
      	 
      //[2] FHSS nums
      //跳频信道数量（默认为 0AH）
      //决定跳频信道的数量，双方必须一样。跳频信道数量越多，模块通信的抗干扰性能
      //力越强，但通信双方的同步时间也更长；跳频信道数量越少，模块抗干扰能力越弱，但同步时间会更短。
      	 Cmd[2]=FHSS;
      	 
      //[3] SPED 速率参数，包括串口速率和空中速率
      //7，6： 串口校验位
      //00：8N1（默认）
      //01：8O1
      //10：8E1
      //11：8N1（等同 00）
      //-------------------------------------------------
      //5，4，3 TTL 串口速率（bps）
      //000：串口波特率为 1200
      //001：串口波特率为 2400
      //010：串口波特率为 4800
      //011：串口波特率为 9600（默认）
      //100：串口波特率为 19200
      //101：串口波特率为 38400
      //110：串口波特率为 57600
      //111：串口波特率为 115200
      //-------------------------------------------------
      //2, 保留，写 0
      //-------------------------------------------------
      //1，0 无线空中速率（bps）
      //00：空中速率为 16K
      //01：空中速率为 32K
      //10：空中速率为 64K（默认）
      //11：空中速率为 128K
      //通信双方串口模式可以不同
      //-------------------------------------
      //通信双方波特率可以不同串口波特率和无
      //线传输参数无关，不影响无线收发特性。
      //-------------------------------------
      //-------------------------------------
      //空中速率越低，距离越远，抗干扰性能越
      //强，发送时间越长。通信双方空中无线传
      //输速率必须相同。 
      //波特率
			Cmd[3]=0;
      switch(BAUD)
      	{
      	  case 96:  {Cmd[3]=(Cmd[3]&0xc7)|0x18;break;}
      	  case 192: {Cmd[3]=(Cmd[3]&0xc7)|0x20;break;}
      		case 115: {Cmd[3]=(Cmd[3]&0xc7)|0x38;break;}
      	  default : {Cmd[3]=(Cmd[3]&0xc7)|0x18; break;}
         }
      	
      //空速
      	 switch(W_SPED)
      	{
      	  case 16:  {Cmd[3]=(Cmd[3]&0xfc)|0x00;break;}
      	  case 32:  {Cmd[3]=(Cmd[3]&0xfc)|0x01;break;}
      		case 64:  {Cmd[3]=(Cmd[3]&0xfc)|0x02;break;}
      		case 128: {Cmd[3]=(Cmd[3]&0xfc)|0x03;break;}
      	  default :{Cmd[3]=(Cmd[3]&0xfc)|0x02; break;}
         }
      	Cmd[3]=Cmd[3]&0X3B;//清除高两位设置成8N1
				 
      //4 CHAN 通信频率（425M + CHAN * 0.5M）
      //（默认为 0AH）
      //00H-C8H，对应 425 - 450MHz
      //当跳频数量为 1 时，模块不进行跳频操
      //作，工作频率固定为此位设置的频率，若
      //跳频信道数量大于 1，则此位将决定整体工
      //作 频 带 ， 即 （ 425MHz + CHAN *
      //0.5MHz ） 到 （ 425MHz + CHAN *
      //0.5MHz +
      //FHSS_nums*0.5MHz）
      Cmd[4]=CHAN;
       
       //				 5 OPTION 7， 保留，写 0
       //-------------------------------------------------
       //6 IO 驱动方式（默认 1）
       //1：TXD、 AUX 推挽输出，RXD 上拉输入
       //0：TXD、 AUX 开路输出，RXD 开路输入
       //-------------------------------------
       //该位用于使能模块内部上拉电阻。漏极开
       //路方式电平适应能力更强，但是某些情况
       //下，可能需要外部上拉电阻E62-T100S2 用户手册 v1.22 成都亿佰特电子科技有限公司
       //样品网址： cdebyte.taobao.com ------------------
       //5，4 , 3 保留，写 0
       //-------------------------------------------------
       //2， FEC 开关
       //0：关闭 FEC
       //1：打开 FEC（默认）
       //-------------------------------------------------
       //1, 0 发射功率（大约值）
       //00： 20dBm（默认）
       //01： 17dBm
       //10： 14dBm
       //11： 11dBm
       //---------------------------------------
       //建议写 0
       //---------------------------------------
       //关闭 FEC 后，数据实际传输速率提升，但抗干扰能力减弱，距离稍近，请根据实际应用选择。收发双方必须相同配置。
       //外部电源必须提供 200mA 以上电流输出能力。并保证电源纹波小于 100mV。不推荐使用较小功率发送，其电源利用效率不高。
      switch(IO_D)
      	{
      	  case 0:  {Cmd[5]=(Cmd[5]&0xbf)|0X00;break;}
      	  case 1:  {Cmd[5]=(Cmd[5]&0xbf)|0X40;break;}
      	  default :{Cmd[5]=(Cmd[5]&0xbf)|0X40; break;}
         }
				
		switch(FEC)
      	{
      	  case 0:  {Cmd[5]=(Cmd[5]&0xfb)|0X00;break;}
      	  case 1:  {Cmd[5]=(Cmd[5]&0xfb)|0X04;break;}
      	  default :{Cmd[5]=(Cmd[5]&0xfb)|0X04; break;}
         }                     
				                       
		 switch(P_DB)              
      	{                      
      	  case 20:{Cmd[5]=(Cmd[5]&0xfc)|0X00;break;}
      	  case 17:{Cmd[5]=(Cmd[5]&0xfc)|0X01;break;}
      		case 14:{Cmd[5]=(Cmd[5]&0xfc)|0X02;break;}
					case 11:{Cmd[5]=(Cmd[5]&0xfc)|0X03;break;}
      	  default:{Cmd[5]=(Cmd[5]&0xfc)|0X00; break;}
         }
				//清除接收区
				 YK_RX[0]=0;YK_RX[1]=0;YK_RX[2]=0;YK_RX[3]=0;YK_RX[4]=0;YK_RX[5]=0;
				 DMA1_Channel5_IRQHandler();
				 delay_ms(100);
	       USART1_DMA_Tx_Data(Cmd,6);//发送设置数据
				 delay_ms(300);
//				 USART1_DMA_Tx_Data(Back_Para,3);
//				 delay_ms(200);
				 if(YK_RX[0]==Cmd[0] && YK_RX[1]==Cmd[1] && YK_RX[2]==Cmd[2] && YK_RX[3]==Cmd[3] && YK_RX[4]==Cmd[4] && YK_RX[5]==Cmd[5])return 1;
					 else return 0;
	}

	unsigned char WL_Get_Config(unsigned char *Par){
    //C1+C1+C1在休眠模式下（M0=1），向模块串口发出命令（HEX 格式）：C1 C1 C1
    //模块会返回当前的配置参数，比如：C0 01 0A 1A 0A 44
		unsigned char WL_Ckeck_Cmd[3]={0xc1,0xc1,0xc1};
		WL_EN(1);
		WL_M0(1);
		delay_ms(400);
		YK_RX[0]=0;YK_RX[1]=0;YK_RX[2]=0;YK_RX[3]=0;YK_RX[4]=0;YK_RX[5]=0;
		DMA1_Channel5_IRQHandler();
		delay_ms(100);
		USART1_DMA_Tx_Data(WL_Ckeck_Cmd,3);//发送设置参数查询
		delay_ms(300);
		if(YK_RX[0]==0xC0){
		Par[0]=YK_RX[0];
		Par[1]=YK_RX[1];
		Par[2]=YK_RX[2];
		Par[3]=YK_RX[3];
		Par[4]=YK_RX[4];
		Par[5]=YK_RX[5];
		return 1;
		}
		else {return 0;}
	}
//------------------------------------------------------------------------------------------------------------------------------
#define USART2_TX_BUFF_SIZE  7//发送缓冲区
#define USART2_RX_BUFF_SIZE 64 //接收缓冲区大于一秒存放的数据量  16*8
#define PC_TxBuff_Size USART2_TX_BUFF_SIZE
uint8_t PC_TX[PC_TxBuff_Size]={0x7E,0x05,0x41,0x00,0x01,0x45,0xef};//发送帧缓冲区
#define PC_RxBuff_Size 8
uint8_t PC_RX[PC_RxBuff_Size]={0xff,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xee};//接收帧数据缓冲区
uint8_t PC_CMD[PC_RxBuff_Size]={0xff,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xee};//存放从PC接收并且解析到的命令数据帧
uint8_t PC_Rx_Flag=1;
uint8_t PC_Tx_Flag=1;
uint8_t PC_CMD_Flag=1;

void USART2_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
/////////////////////////////////////////////////////////////////////////
   //#VO_CT
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC  , ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	//选择对应的引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);  
  GPIO_SetBits(GPIOC, GPIO_Pin_9);	//默认开启
	//GPIO_ResetBits(GPIOC, GPIO_Pin_9);	
	
	//VO_BUSY
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC  , ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;	//选择对应的引脚
  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);  
	//GPIO_ResetBits(GPIOC, GPIO_Pin_10);	
	/////////////////////////////////////////////////////////////////////////////////////	
	      /*第1步：打开GPIO和USART2部件的时钟 */
        //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
        /*第2步：将USART2 Tx的GPIO配置为推挽复用模式 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        /*第3步：将USART2 Rx的GPIO配置为浮空输入模式
                 由于CPU复位后，GPIO缺省都是浮空输入模式，因此下面这个步骤不是必须的
                 但是，我还是建议加上便于阅读，并且防止其它地方修改了这个口线的设置参数
        */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        //第3步已经做了，因此这步可以不做
        //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        //
        GPIO_Init(GPIOA, &GPIO_InitStructure);
				
/////////////////////////////////////////////////////////////////////////////////////	
	      USART_InitStructure.USART_BaudRate = 9600;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_No;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
        USART_Init(USART2, &USART_InitStructure);
				//开启空闲,帧错,噪声,校验错中断 
        USART_ITConfig(USART2, USART_IT_IDLE , ENABLE);
				//错误中断
        USART_ITConfig(USART2, USART_IT_ERR | USART_IT_ORE | USART_IT_NE | USART_IT_FE ,ENABLE);
        USART_Cmd(USART2, ENABLE);
        // CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
        //如下语句解决第1个字节无法正确发送出去的问题 //
        //USART_ClearFlag(USART2, USART_FLAG_TC); //清发送外城标志，Transmission Complete flag //
				USART_ClearFlag(USART2, USART_FLAG_TC|USART_FLAG_RXNE|USART_FLAG_IDLE|USART_FLAG_ORE|USART_FLAG_NE|USART_FLAG_FE |USART_FLAG_PE ); 
					
	/////////////////////////////////////////////////////////////////////////////////////					
       	//DMA1 Channel7 (triggered by USART2 Tx event) Config */
       DMA_DeInit(DMA1_Channel7);  
       DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);
       DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)PC_TX;
       DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
       DMA_InitStructure.DMA_BufferSize =USART2_TX_BUFF_SIZE;
       DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
       DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
       DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
       DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
       DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
       DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
       DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
       DMA_Init(DMA1_Channel7, &DMA_InitStructure);
       DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);
       DMA_ITConfig(DMA1_Channel7, DMA_IT_TE, ENABLE);
			 DMA_ClearITPendingBit(DMA1_IT_GL7 | DMA1_IT_TC7| DMA1_IT_TE7 | DMA1_IT_HT7);
      // DMA_ClearFlag(DMA1_FLAG_GL7| DMA1_FLAG_TC7 | DMA1_FLAG_TE7 | DMA1_FLAG_HT7);
       /* Enable USART1 DMA TX request */
       USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
       DMA_Cmd(DMA1_Channel7, DISABLE);			
								
	/////////////////////////////////////////////////////////////////////////////////////
       /* DMA1 Channel6 (triggered by USART2 Rx event) Config */
       DMA_DeInit(DMA1_Channel6);  
       DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);
       DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)PC_RX;
       DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
       DMA_InitStructure.DMA_BufferSize = USART2_RX_BUFF_SIZE;
       DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
       DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
       DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
       DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
       DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
       DMA_InitStructure.DMA_Priority = DMA_Priority_High;
       DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
       DMA_Init(DMA1_Channel6, &DMA_InitStructure);
       DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);
       DMA_ITConfig(DMA1_Channel6, DMA_IT_TE, ENABLE);
			 DMA_ClearITPendingBit(DMA1_IT_GL6 | DMA1_IT_TC6| DMA1_IT_TE6 | DMA1_IT_HT6);
       //DMA_ClearFlag(DMA1_FLAG_GL6 | DMA1_FLAG_TC6 | DMA1_FLAG_TE6 | DMA1_FLAG_HT6);
       //Enable USART2 DMA RX request //
       USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
       DMA_Cmd(DMA1_Channel6, ENABLE);
			 					
	/////////////////////////////////////////////////////////////////////////////////////
  //Configure one bit for preemption priority //
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);  
  // Enable the USART2 Interrup--接收空闲中断
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
  //Enable DMA Channel7 Interrupt --TX-DMA发送完成中断
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  //Enable DMA Channel6 Interrupt --RX--DMA接收完成中断
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
} 
void USART2_IRQHandler(void)
{
   u16 DATA_LEN;
   u16 i;
	 PC_Rx_Flag=0;//标记数据接收操作正在处理，制造发送与接收操作互斥
	 //DMA_Cmd(DMA1_Channel6, DISABLE);//关闭DMA,防止处理其间有数据
   if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)//如果为空闲总线中断
    { 
			     // LED4(2);
             DATA_LEN=USART2_RX_BUFF_SIZE-DMA_GetCurrDataCounter(DMA1_Channel6); 
             if(DATA_LEN >=PC_RxBuff_Size)//如果收到数据并且产生了空闲中断
             { 
               DMA_Cmd(DMA1_Channel6, DISABLE);//关闭DMA,防止处理其间有数据										
				    if(PC_RX[0]==0xFF && PC_RX[7]==0xEE)
				    {
					    memcpy(PC_CMD,PC_RX,PC_RxBuff_Size);//复制数据
					    PC_CMD_Flag=1;//标记数据帧
					   // LED4(2);
				    }
						//DMA_Cmd(DMA1_Channel6, DISABLE);//关闭DMA,防止处理其间有数据
			          DMA_ClearFlag(DMA1_FLAG_GL6 | DMA1_FLAG_TC6 | DMA1_FLAG_TE6 | DMA1_FLAG_HT6);//清标志
                 DMA1_Channel6->CNDTR = USART2_RX_BUFF_SIZE;//重装填
                 DMA_Cmd(DMA1_Channel6, ENABLE);//处理完,重开DMA
            }
			     else{ }
			     //读SR后读DR清除Idle
            i=USART2->SR;
            i=USART2->DR;
		        i=i;
     }
		   //见USART库文件905行
		   USART_ClearFlag(USART2, USART_FLAG_IDLE|USART_FLAG_ORE|USART_FLAG_NE|USART_FLAG_FE |USART_FLAG_PE );//清除所有错误标志
       USART_ClearITPendingBit(USART2, USART_IT_IDLE | USART_IT_PE | USART_IT_FE | USART_IT_NE|USART_IT_ERR|USART_IT_ORE);////清除所有错误中断位
		   
		   //重启DMA
//		   DMA_ClearFlag(DMA1_FLAG_GL6 | DMA1_FLAG_TC6 | DMA1_FLAG_TE6 | DMA1_FLAG_HT6);//清标志
//       DMA1_Channel6->CNDTR = USART2_RX_BUFF_SIZE;//重装填
//       DMA_Cmd(DMA1_Channel6, ENABLE);//处理完,重开DMA
		   PC_Rx_Flag=1;//接收中断处理函数执行完毕
}     

//DMA1_Channel5串口接收中断服务函数
void DMA1_Channel6_IRQHandler(void)
{
  DMA_ClearITPendingBit(DMA1_IT_GL6 | DMA1_IT_TC6| DMA1_IT_TE6 | DMA1_IT_HT6);
 // DMA_ClearFlag(DMA1_FLAG_GL6 | DMA1_FLAG_TC6| DMA1_FLAG_TE6 | DMA1_FLAG_HT6);
  DMA_Cmd(DMA1_Channel6, DISABLE);//关闭DMA,防止处理其间有数据
  DMA1_Channel6->CNDTR = USART2_RX_BUFF_SIZE;//重装填
  DMA_Cmd(DMA1_Channel6, ENABLE);//处理完,重开DMA
}

//DMA1_Channel4串口发送中断服务函数
//USART2使用DMA发数据中断服务程序
void DMA1_Channel7_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC7)){PC_Tx_Flag=1;}//发送完成中断LED3(2);
  DMA_ClearITPendingBit(DMA1_IT_GL7 | DMA1_IT_TC7| DMA1_IT_TE7 | DMA1_IT_HT7);
  //DMA_ClearFlag(DMA1_FLAG_GL7 | DMA1_FLAG_TC7 | DMA1_FLAG_TE7 | DMA1_FLAG_HT7);
  DMA_Cmd(DMA1_Channel7, DISABLE);//发送完毕并产生完成传输中断后关闭DMA
 // PC_Tx_Flag=1;//置DMA传输完成
}

void USART2_DMA_Tx_Data(u8 COUNT)
{
	 PC_Tx_Flag=0;
	 DMA_ClearITPendingBit(DMA1_IT_GL7 | DMA1_IT_TC7| DMA1_IT_TE7 | DMA1_IT_HT7);//这两节话加上后DMA发送出错导致系统不稳定的情况改善了，
  // DMA_ClearFlag(DMA1_FLAG_GL7 | DMA1_FLAG_TC7 | DMA1_FLAG_TE7 | DMA1_FLAG_HT7);//防止上一次发送出错导致后边接着出错
	 DMA_Cmd(DMA1_Channel7, DISABLE);  // 关闭DMA通道
   DMA1_Channel7->CNDTR = (uint16_t)COUNT; // 设置要发送的字节数目个25
	// while(PC_Rx_Flag==0){};//等待接收中断处理函数执行完毕//处理发送与接收互斥---PC机可以全双工收发，不需要做这个发送与接收互斥处理
   DMA_Cmd(DMA1_Channel7, ENABLE);        //开始DMA发送
}


u8 VO_BUSY(void){return  (GPIO_ReadInputDataBit(GPIOC ,GPIO_Pin_13  ));}
//===============================USART3_DMA_端口==================================================================================================================================
//=======================================================================================================
#define USART3_TX_BUFF_SIZE  64 //发送缓冲区
#define USART3_RX_BUFF_SIZE  128 //冲区大于一秒存放的数据量  16*8
#define EX_TxBuff_Size USART3_TX_BUFF_SIZE
uint8_t EX_TX[EX_TxBuff_Size];//发送帧缓冲区
#define EX_RxBuff_Size USART3_RX_BUFF_SIZE
uint8_t EX_RX[EX_RxBuff_Size];//接收帧数据缓冲区
//uint8_t EX_MSG[512];//存放从EX接收并且解析到的命令数据帧
int EX_Rx_Count=0;
uint8_t EX_Rx_Flag=0;
uint8_t EX_Tx_Flag=1;

 void  USART3_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
//--------------------------------------------------------------------------------
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		// USART3 使用IO端口配置    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);      
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//浮空输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);   //初始化GPIOB  
	//USART3 工作模式配置 
	USART_InitStructure.USART_BaudRate = 115200;	//波特率设置：115200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//数据位数设置：8位
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 	//停止位设置：1位
	USART_InitStructure.USART_Parity = USART_Parity_No ;  //是否奇偶校验：无
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制模式设置：没有使能
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//接收与发送都使能
	USART_Init(USART3, &USART_InitStructure);  //初始化USART3
   
	//开启空闲,帧错,噪声,校验错中断 
     USART_ITConfig(USART3, USART_IT_IDLE , ENABLE);//可以用来判断一帧数据接收完毕
	   USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//使用接收中断
	//错误中断
     USART_ITConfig(USART3, USART_IT_ERR | USART_IT_ORE | USART_IT_NE | USART_IT_FE ,ENABLE);
     USART_Cmd(USART3, ENABLE);
     // CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
     //如下语句解决第1个字节无法正确发送出去的问题 //
     //USART_ClearFlag(USART3, USART_FLAG_TC); //清发送外城标志，Transmission Complete flag //
			USART_ClearFlag(USART3, USART_FLAG_TC|USART_FLAG_RXNE|USART_FLAG_TXE|USART_FLAG_IDLE|USART_FLAG_ORE|USART_FLAG_NE|USART_FLAG_FE |USART_FLAG_PE ); 
					
	/////////////////////////////////////////////////////////////////////////////////////					
       	//DMA1 Channel2 (triggered by USART3 Tx event) Config */
       DMA_DeInit(DMA1_Channel2);  
       DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
       DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)EX_TX;
       DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
       DMA_InitStructure.DMA_BufferSize =USART3_TX_BUFF_SIZE;
       DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
       DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
       DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
       DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
       DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
       DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
       DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
       DMA_Init(DMA1_Channel2, &DMA_InitStructure);
       DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
       DMA_ITConfig(DMA1_Channel2, DMA_IT_TE, ENABLE);
			 DMA_ClearITPendingBit(DMA1_IT_GL2 | DMA1_IT_TC2| DMA1_IT_TE2 | DMA1_IT_HT2);
       /* Enable USART1 DMA TX request */
       USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
       DMA_Cmd(DMA1_Channel7, DISABLE);			
								
	/////////////////////////////////////////////////////////////////////////////////////
       /* DMA1 Channel3 (triggered by USART3 Rx event) Config */
       DMA_DeInit(DMA1_Channel3);  
//       DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
//       DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)EX_RX;
//       DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//       DMA_InitStructure.DMA_BufferSize = USART3_RX_BUFF_SIZE;
//       DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//       DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//       DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//       DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//       DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//       DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//       DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//       DMA_Init(DMA1_Channel3, &DMA_InitStructure);
//       DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
//       DMA_ITConfig(DMA1_Channel3, DMA_IT_TE, ENABLE);
//			 DMA_ClearITPendingBit(DMA1_IT_GL3 | DMA1_IT_TC3| DMA1_IT_TE3 | DMA1_IT_HT3);
//       // Enable USART2 DMA RX request //
//       USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
//       DMA_Cmd(DMA1_Channel3, ENABLE);
			 					
	/////////////////////////////////////////////////////////////////////////////////////
  //Configure one bit for preemption priority //
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
  // Enable the USART3 Interrup--接收空闲中断
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
  //Enable DMA1 Channel2 Interrupt --TX-DMA发送完成中断
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  //Enable DMA1 Channel3 Interrupt --RX--DMA接收完成中断
//  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
}

void USART3_DMA_Tx_Data(void )
{
	 DMA_Cmd(DMA1_Channel2, DISABLE);  // 关闭DMA通道
   DMA1_Channel2->CNDTR = USART3_TX_BUFF_SIZE; // 设置要发送的字节数目
   DMA_Cmd(DMA1_Channel2, ENABLE);        //开始DMA发送
}

//UART3_DMA TX CH2发送完成中断
void DMA1_Channel2_IRQHandler(void)
{
  DMA_ClearITPendingBit(DMA1_IT_GL2 | DMA1_IT_TC2| DMA1_IT_TE2 | DMA1_IT_HT2);
  DMA_Cmd(DMA1_Channel2, DISABLE);//发送完毕并产生完成传输中断后关闭DMA
}

void USART3_IRQHandler(void)
{
   u16 i;
	 EX_Rx_Flag=0;//标记数据接收操作正在处理，制造发送与接收操作互斥
   if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//如果为空闲总线中断
    { 
			if(EX_Rx_Count<=126)EX_Rx_Count++;
			else EX_Rx_Count=0;
			EX_RX[EX_Rx_Count]=USART_ReceiveData(USART3);
			USART_ClearFlag(USART3,USART_FLAG_RXNE);
     }	
   if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)//如果为空闲总线中断
    { 
			 EX_Rx_Flag=1;
			 //读SR后读DR清除Idle
       i=USART3->SR;
       i=USART3->DR;
		   i=i;
     }
		   //见USART库文件905行
		   USART_ClearFlag(USART3, USART_FLAG_IDLE|USART_FLAG_ORE|USART_FLAG_NE|USART_FLAG_FE |USART_FLAG_PE );//清除所有错误标志
       USART_ClearITPendingBit(USART3, USART_IT_IDLE | USART_IT_PE | USART_IT_FE | USART_IT_NE|USART_IT_ERR|USART_IT_ORE);////清除所有错误中断位
}     


//======================================================================================================
//===============================================================================================================================================
int fputc(int ch, FILE *f)
{
// 将Printf内容发往串口 //
	if (f== COM1 )
		{ 
     USART_SendData(USART1, (unsigned char) ch);
     while (!(USART1->SR & USART_FLAG_TXE));
     return (ch);
		}

	if(f== COM2)
		{ 
     USART_SendData(USART2, (unsigned char) ch);
     while (!(USART2->SR & USART_FLAG_TXE));
     return (ch);
		}
   
	 	if (f== COM3 )
		{ 
     USART_SendData(USART3, (unsigned char) ch);
     while (!(USART3->SR & USART_FLAG_TXE));
     return (ch);
		}
		 return  (ch);
   }

//================================================================================================

/*
////------------------------------------------串口2回传测试----------------------------------------------------------------------
void USART2_Initialise( u32 bound )
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    
    // Enable the USART2 Pins Software Remapping //
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 
    
    
    // Configure USART2 Rx (PA.03) as input floating //
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // Configure USART2 Tx (PA.02) as alternate function push-pull //
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // Enable the USART2 Interrupt //
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);    
    
    USART_InitStructure.USART_BaudRate = bound;                
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;    
    USART_InitStructure.USART_Parity = USART_Parity_No;        
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
    
    USART_Init(USART2, &USART_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    //USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    //Enable USART2 //
    USART_Cmd(USART2, ENABLE);
}

void USART2_IRQHandler(void)  
{  
     if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET)  
      {       
             // USART_SendData(USART2, USART_ReceiveData(USART2));             
      }
      
}
*/
