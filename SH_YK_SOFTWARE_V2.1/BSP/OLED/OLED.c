 #include "OLED.h"
 #include "delay.h"
 #include "ASCII.h"
 #include "GB1616.h"	//16*16汉字字模
 #include "LED.h"
// #include "MKY.H"

#ifdef OLED_HARDWARE
//#define SPI1_DR_Addr ( (u32)0x4001300C )
//#define SPI2_DR_Addr ( (u32)0x4000380C )

#define DIS_BUF_SIZE 4608 //96*48

unsigned char  SPI1_DMA_TX_BUSY=0;
//#define SPILCD_W 96
//#define SPILCD_H 96


void Delayms(unsigned short time)
{
  delay_ms(time);
}

/*************************************************************************************************
虚拟显存
******************************************************************************************************/
static u8   VDis_buff[96][48]={0};

/*******************************************************************************
* Function Name  : SPI1_DMA_Configuration
* Description    : 配置SPI1_RX的DMAX通道X，SPI1_TX的DMA1通道3
*******************************************************************************/
void Lcd_SPI1_DMA_Configuration( void )
{
  DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA| RCC_APB2Periph_AFIO,ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	 //LCD_CS 'LCD_CS--A4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//LCD_RS 'LCD_RS--PC4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//LCD_BLCTR 'LCD_BLCTR--PC5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, GPIO_Pin_5);
	
	//MISO--PA6未使用
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;    
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP;// GPIO_Mode_IPU; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_Init(GPIOA, &GPIO_InitStructure); 	 

	//配置SPI1管脚
	//SPI1  FULL DUPLEX MASTER?
	//PA7--SPI1_MOSI---主机输出
	//PA6--SPI1_MISO---主机输入（这里没有用到这个引脚）//当成普通IO CS引脚用
	//PA5--SPI1_SCK ---主机时钟输出
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);//使能SPI1时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//SPI1配置选项
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 ,ENABLE); //使能SPI1时钟
	SPI_InitStructure.SPI_Direction =SPI_Direction_1Line_Tx;//SPI_Direction_2Lines_FullDuplex;////
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
	//DMA_Cmd(DMA1_Channel3,ENABLE);//使能SPI1 DMA发送通道
	//SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);//使能SPI1 DMA发送功能
	SPI_Cmd(SPI1, ENABLE); //使能SPI1
	
	/*------------------配置SPI1_TX_DMA1通道Channel3---------------------*/
  DMA_DeInit(DMA1_Channel3);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//使能DMA1时钟
  DMA_InitStructure.DMA_PeripheralBaseAddr =(u32)&SPI1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)VDis_buff;//显存区
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = DIS_BUF_SIZE; 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel3, &DMA_InitStructure);//对DMA通道进行初始化
	//DMA_Cmd(DMA1_Channel3,ENABLE);//使能SPI3 DMA发送通道
  //设置中断
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0); 
	/*Enable DMA1 Channel3 TX Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);//使能发送完成中断功能
  DMA_ITConfig(DMA1_Channel3, DMA_IT_TE, ENABLE);//使能发送出错中断功能
	DMA_ClearITPendingBit(DMA1_IT_GL3 | DMA1_IT_TC3| DMA1_IT_TE3 | DMA1_IT_HT3);//
	
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);//使能SPI1 DMA发送功能
	DMA_Cmd(DMA1_Channel3, DISABLE);//先关闭DMA
	SPI_Cmd(SPI1, ENABLE); //使能SPI1
}

//DMA1_Channel3发送完成中断服务函数
void DMA1_Channel3_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC3)){
		//DMA是属于减计数，发送读取完成之后计数器的值会自动减为0，所以你要发送（以发送为例子）新的数据，就必须重新配置其数据的数目或者长度。
		//DMA的TC中断产生后，并不代表数据全部传送出去了
		while(DMA_GetCurrDataCounter(DMA1_Channel3));
		//while(SPI_I2S_GetITStatus(SPI1,SPI_I2S_FLAG_BSY));
	  //DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, DISABLE);
	  LEDR(2);	  
	  LCD_CS_SET;
	  SPI1_DMA_TX_BUSY=0;
		DMA_Cmd(DMA1_Channel3, DISABLE);//关闭DMA,防止处理其间有数据
		DMA_ClearITPendingBit(DMA1_IT_GL3|DMA1_FLAG_TC3 | DMA1_IT_TE3 | DMA1_IT_HT3);
	}
	else
	  DMA_ClearITPendingBit(DMA1_IT_TE3 | DMA1_IT_HT3);

}

//开启一次DMA传送
void Lcd_SPI1_DMA_TX( void )
{
	//确保上一次DMA传输已经完成
	if(SPI1_DMA_TX_BUSY==0)
		{
			LCD_CS_CLR;
      LCD_RS_SET;
			SPI1_DMA_TX_BUSY=1;
			DMA_ClearITPendingBit(DMA1_IT_GL3|DMA1_FLAG_TC3 | DMA1_IT_TE3 | DMA1_IT_HT3);//预防措施
			//DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);			
      DMA1_Channel3->CNDTR =(uint16_t)DIS_BUF_SIZE;       //传输数量设置为buffersize个
	    DMA_Cmd(DMA1_Channel3, ENABLE);  
  }
}

/****************************************************************************
* 名    称：u8   SPI_WriteByte(SPI_TypeDef* SPIx,u8   Byte)
* 功    能：STM32_硬件SPI读写一个字节数据底层函数
* 入口参数：SPIx,Byte
* 出口参数：返回总线收到的数据
* 说    明：STM32_硬件SPI读写一个字节数据底层函数
****************************************************************************/
u8   SPI_WriteByte(SPI_TypeDef* SPIx,u8   Byte)
{
	//while((SPIx->SR&SPI_I2S_FLAG_TXE)==RESET);		//等待发送区空	  
	SPIx->DR=Byte;	 	//发送一个byte   
	//while((SPIx->SR&SPI_I2S_FLAG_RXNE)==RESET);//等待接收完一个byte  
	while((SPIx->SR&SPI_I2S_FLAG_TXE)==RESET);		//等待发送区空	  
	return SPIx->DR;          	     //返回收到的数据			
} 

//使用DMA一次刷整个屏函数
void VLcd_refresh(void)
{
	//发送坐标，设置显存区域
	Lcd_Write_Command(0x15);//SET COLUMN ADDR 
	Lcd_Write_Command(0x10); 
	Lcd_Write_Command(63); 
	Lcd_Write_Command(0x75);//SET ROW ADDR 
	Lcd_Write_Command(0x00); 
	Lcd_Write_Command(95); 	
	
   Lcd_SPI1_DMA_TX( );//开启DMA刷新显存数据
   //LCD_CS_SET;---这一句写进中断

}

//不使用DMA，直接发送全部字节刷整个屏幕
void SPI_OLED_Refresh(void)
{
	unsigned short x,y;   
	Lcd_Write_Command(0x15);//SET COLUMN ADDR 
	Lcd_Write_Command(0x10); 
	Lcd_Write_Command(63); 
	Lcd_Write_Command(0x75);//SET ROW ADDR 
	Lcd_Write_Command(0x00); 
	Lcd_Write_Command(95); 	
	for(y=0;y<96;y++)
		for(x=0;x<48;x++)
		        Lcd_Write_Data(VDis_buff[y][x]);//显示所填充的颜色. 
} 
//在虚拟显存中画点
void VLcd_DrawPoint(int16_t  x,int16_t  y,int32_t   Data)
{
	u8 X,Y,Color;
	X=(int8_t)((x&0x00FF)/2);
	Y=(int8_t)(y&0x00FF);
  Color=(int8_t)(Data&0X0000000F);//截取低四位作为灰度等级
	 if(x%2)VDis_buff[Y][X]=(VDis_buff[Y][X]&0xF0)|Color;
	 else   VDis_buff[Y][X]=(VDis_buff[Y][X]&0x0f)|(Color<<4);
}

//在虚拟显存中读点
u8  VLcd_ReadPoint(int16_t  x,int16_t  y)
{
	u8 X,Y;
	X=(int8_t)((x&0X00FF)/2);
	Y=(int8_t)(y&0x00FF);
 //截取低四位作为灰度等级
	 if(x%2)return (VDis_buff[Y][X]&0XF0)>>4;
	 else return(VDis_buff[Y][X]&0X0F);
//	return 0;
}

/****************************************************************************
* 名    称：Lcd_WriteIndex(u8   Index)
* 功    能：向液晶屏写一个8位指令
* 入口参数：Index   寄存器地址
* 出口参数：无
* 说    明：调用前需先选中控制器，内部函数
****************************************************************************/
void Lcd_Write_Command(u8   Cmd)
{
   LCD_CS_CLR;
   LCD_RS_CLR; 
   SPI_WriteByte(SPI1,Cmd);
   LCD_CS_SET;
}
/****************************************************************************
* 名    称：Lcd_WriteData(u8   Data)
* 功    能：向液晶屏写一个8位数据
* 入口参数：dat     寄存器数据
* 出口参数：无
* 说    明：向控制器指定地址写入数据，内部函数
****************************************************************************/
void Lcd_Write_Data(u8   Data)
{
   LCD_CS_CLR;
   LCD_RS_SET; 
   SPI_WriteByte(SPI1,Data);
   LCD_CS_SET;
}




void ILI9325_CMO24_Initial(void)
{
   	Delayms(25);

		Lcd_Write_Command(0x15);//SET COLUMN ADDR 
		Lcd_Write_Command(0x10); 
		Lcd_Write_Command(0x3f); 
		Lcd_Write_Command(0x75);//SET ROW ADDR 
		Lcd_Write_Command(0x00); 
		Lcd_Write_Command(0x5f); 

		Lcd_Write_Command(0xA0);//SET RE-MAP 
		Lcd_Write_Command(0x51); 
		Lcd_Write_Command(0xA1);//SET DISPLAY START LINE 
		Lcd_Write_Command(0x00); 
		Lcd_Write_Command(0xA2);//SET DISPLAY OFFSET 
		Lcd_Write_Command(0x60); 
		Lcd_Write_Command(0xA4);//SET DISPLAY MODE 
		Lcd_Write_Command(0xA8);//SET MUX RADIO 
		Lcd_Write_Command(0x5F); 
		Lcd_Write_Command(0xB2);//SET FRAME FREQUENCY 
		Lcd_Write_Command(0x23); 
		Lcd_Write_Command(0xB3);//SET FRONT CLOCK DIVIDER & OSCILLATOR FREQUENCY 
		Lcd_Write_Command(0xF0); 

		Lcd_Write_Command(0x81);//SET CONTRAST CURRENT 
		Lcd_Write_Command(0x7F);// 
		Lcd_Write_Command(0xBC);    //first pre_charge voltage 
		Lcd_Write_Command(0x1F); 
		Delayms(10);
		Lcd_Write_Command(0x82);   //second pre_charge speed 
		Lcd_Write_Command(0xFE); 
		Delayms(10);
		Lcd_Write_Command(0xB1);   //first pre_charge phase length 
		Lcd_Write_Command(0x21); 

		Lcd_Write_Command(0xBB);//SET SECONDE PRE_CHARGE PERIOD 
		Lcd_Write_Command(0x0F); 

		Lcd_Write_Command(0xbe);//SET VCOMH 
		Lcd_Write_Command(0x1F); 

		Lcd_Write_Command(0xB8);        //SET GS 
		Lcd_Write_Command(0x04); //GS1 
		Lcd_Write_Command(0x06); //GS2 
		Lcd_Write_Command(0x08); //GS3         
		Lcd_Write_Command(0x0A); //GS4 
		Lcd_Write_Command(0x0C); //GS5  
		Lcd_Write_Command(0x0E); //GS6       
		Lcd_Write_Command(0x10); //GS7 
		Lcd_Write_Command(0x12); //GS8 
		Lcd_Write_Command(0x14); //GS9   
		Lcd_Write_Command(0x16); //GS10 
		Lcd_Write_Command(0x18); //GS11 
		Lcd_Write_Command(0x1A); //GS12 
		Lcd_Write_Command(0x1C); //GS13 
		Lcd_Write_Command(0x1E); //GS14 
		Lcd_Write_Command(0x20); //GS15 
		Delayms(10);
		 
		Lcd_Write_Command(0xAF);         //DSPLAY ON
}


//函数名：SPILCD_Clear
//参  数：Color 颜色      
void SPILCD_Clear(unsigned short Color)
{
	unsigned short x,y;   
	Lcd_Write_Command(0x15);//SET COLUMN ADDR 
	Lcd_Write_Command(0x10); 
	Lcd_Write_Command(63); 
	Lcd_Write_Command(0x75);//SET ROW ADDR 
	Lcd_Write_Command(0x00); 
	Lcd_Write_Command(95); 	
  //LCD_CS_CLR;
	for(x=0;x<96;x++)
		for(y=0;y<48;y++)
	   VDis_buff[x][y]=Color;
		  //Lcd_Write_Data(Color);//显示所填充的颜色. 
	// LCD_CS_SET;
} 



void OLED_TEST(void)
{
	Lcd_Write_Command(0x15);//SET COLUMN ADDR 
	Lcd_Write_Command(0x10); 
	Lcd_Write_Command(0x17); 
	Lcd_Write_Command(0x75);//SET ROW ADDR 
	Lcd_Write_Command(0x00); 
	Lcd_Write_Command(0x00); 

	  Lcd_Write_Data(0x01);
	  Lcd_Write_Data(0x23);
	  Lcd_Write_Data(0x45);
	  Lcd_Write_Data(0x67);
	  Lcd_Write_Data(0x89);
	  Lcd_Write_Data(0xAB);
	  Lcd_Write_Data(0xCD);
	  Lcd_Write_Data(0xEF);
	
	Lcd_Write_Command(0x15);//SET COLUMN ADDR 
	Lcd_Write_Command(0x10); 
	Lcd_Write_Command(0x17); 
	Lcd_Write_Command(0x75);//SET ROW ADDR 
	Lcd_Write_Command(0x07); 
	Lcd_Write_Command(0x08);
	
		Lcd_Write_Data(0x01);
	  Lcd_Write_Data(0x23);
	  Lcd_Write_Data(0x45);
	  Lcd_Write_Data(0x67);
	  Lcd_Write_Data(0x89);
	  Lcd_Write_Data(0xAB);
	  Lcd_Write_Data(0xCD);
	  Lcd_Write_Data(0xEF);
		
		Lcd_Write_Data(0xff);
	  Lcd_Write_Data(0xff);
	  Lcd_Write_Data(0xff);
	  Lcd_Write_Data(0xff);
	  Lcd_Write_Data(0xff);
	  Lcd_Write_Data(0xff);
	  Lcd_Write_Data(0xff);
	  Lcd_Write_Data(0xff);
	
	

	
	while(1);
}

//函数名：SPILCD_ShowChar
//参  数：
//(x,y): 
//num:要显示的字符:" "--->"~"
//size:字体大小 8/16
//mode:叠加方式(1)还是非叠加方式(0)
void SPILCD_ShowChar(unsigned char x,unsigned char y,unsigned char num)
{       
//	unsigned char temp;
//	unsigned int pos,t,i;  
//	unsigned char size; 
//	size=16;	//找到字体大小
//  WriteComm(0x15);//SET COLUMN ADDR 
//	WriteComm(0x10+x); 
//	WriteComm(0x10+x+3); 
//	WriteComm(0x75);//SET ROW ADDR 
//	WriteComm(0x00+y); 
//	WriteComm(0x00+y+15); 
//	lcd_RS(1);
//	SPI_CS(0);  
//	num=num-' ';//得到偏移后的值
//	i=num*16;

//		for(pos=0;pos<size;pos++)
//		{

//			temp=nAsciiDot[i+pos];	//调通调用艺术字体
//			for(t=0;t<8;t++)
//		   {                 
//		      if(temp&0x80)
//						LCD_WriteoneSPI(0xff);
//					else 
//						LCD_WriteoneSPI(0x00);
//		      temp<<=1; 
//		    }
//		}	

	  unsigned char i,j,X,Y,M;
	  unsigned int POS;
		Y=y;
		POS=(num-' ')*16;
		for(i=0;i<16;i++){
			X=x;
			M=nAsciiDot[POS+i];	
			for(j=0;j<8;j++)
			{
				if(M&0x80)VLcd_DrawPoint(X,Y,0X0F);
				else VLcd_DrawPoint(X,Y,0X00);
				X++;
				M=M<<1;
			}
			 Y++;
		}
}  
void PutGB1616(unsigned char x, unsigned char  y, unsigned char c[2])
{
//	unsigned int i,j,k;
//	unsigned short m;
//	WriteComm(0x15);//SET COLUMN ADDR 
//	WriteComm(0x10+x); 
//	WriteComm(0x10+x+7); 
//	WriteComm(0x75);//SET ROW ADDR 
//	WriteComm(0x00+y); 
//	WriteComm(0x00+y+15); 	
//	lcd_RS(1);
// SPI_CS(0);	
//	for (k=0;k<64;k++) { //64标示自建汉字库中的个数，循环查询内码
//	  if ((codeGB_16[k].Index[0]==c[0])&&(codeGB_16[k].Index[1]==c[1]))
//			{ 
//    	for(i=0;i<32;i++) 
//			{
//				m=codeGB_16[k].Msk[i];
//				for(j=0;j<8;j++) 
//				{		
//					if((m&0x80)==0x80) {
//						// VLcd_DrawPoint();
//						LCD_WriteoneSPI(0xff); 
//						}   
//					else {
//						LCD_WriteoneSPI(0x00);
//						}
//					m=m<<1;
//				} 
//				if(i%2){y++;x=x-8;}
//				else x=x+8;
//		  }
//		}  
//	  }	
	
		
	unsigned char i,j,k,X,Y,M;
	for (k=0;k<64;k++)  //64标示自建汉字库中的个数，循环查询内码
	  if ((codeGB_16[k].Index[0]==c[0])&&(codeGB_16[k].Index[1]==c[1]))
		for(i=0;i<32;i+=2){
			Y=y+i/2;
			X=x;
			
			M=codeGB_16[k].Msk[i];
			for(j=0;j<8;j++){
				if(M&0x80)VLcd_DrawPoint(X,Y,0X0F);
				else VLcd_DrawPoint(X,Y,0X00);
				X++;
				M=M<<1;}
			
		M=codeGB_16[k].Msk[i+1];
		for(j=0;j<8;j++){
				if(M&0x80)VLcd_DrawPoint(X,Y,0X0F);
				else VLcd_DrawPoint(X,Y,0X00);
			  X++;
			  M=M<<1;}	
		}
		
	}

///////////////////////////////////////////
void LCD_PutString(unsigned char x, unsigned char y, unsigned char *s) 
{
	unsigned char l=0;
	while(*s) 
		{
			if( *s < 0x80) 
				{
					SPILCD_ShowChar(x+l,y,*s);
					s++;l+=8;
				}
			else
				{
					PutGB1616(x+l,y,(unsigned char*)s);
					s+=2;l=l+16;
				}
		}
}


//
void SHOW_BMP(int16_t  x,int16_t  y,int16_t  L,int16_t  H, const unsigned char *s)
{
  int16_t  i,j,k,M;
	for(j=0;j<H;j++)
	 for(i=0;i<L/8;i++)
	{
		 M=s[j*(L/8)+i];
		 for(k=0;k<8;k++)
		 {
		 	if(M&0X80)VLcd_DrawPoint(x+i*8+k,y+j,0x0000000f);
	    else VLcd_DrawPoint(x+i*8+k,y+j,0x00000000);
		  M=M<<1;
		 }
	}
}

void SHOW_4BIT_BMP(int16_t  x,int16_t  y,int16_t  L,int16_t  H, const unsigned char *s)
{
  int16_t  i,j,M;
	for(j=0;j<H;j++)
		for(i=0;i<L/2;i++)
	   {
	   		M=s[j*(L/2)+i];
        // VLcd_DrawPoint(x+i*2,y+j,M&0X0F);
        // VLcd_DrawPoint(x+i*2+1,y+j,(M>>4)&0X0F);}
	   	
	   		VLcd_DrawPoint(x+i*2+1,y+j,M&0X0F);
	   		VLcd_DrawPoint(x+i*2,y+j,(M>>4)&0X0F);
	   }
}


void InterBresenhamline (int x0,int y0,int x1, int y1,int color) 
{ 
    int x, y, dx, dy,e,i; 

    dx = x1-x0;dy = y1- y0;e=-dx; 
    x=x0; y=y0; 
    for (i=0; i<dx; i++){
       VLcd_DrawPoint(x, y, color); 
        x++; e=e+2*dy; 
        if (e >= 0) { y++; e=e-2*dx;} 
    } 
}

//-----------画线。参数：起始坐标，终点坐标，颜色--------  
void draw_line(int x1,int y1,int x2,int y2,int color)  
{  
    int dx,dy,e;  
    dx=x2-x1;   
    dy=y2-y1;  
    if(dx>=0)  
    {  
        if(dy >= 0) // dy>=0  
        {  
            if(dx>=dy) // 1/8 octant  
            {  
                e=dy-dx/2;  
                while(x1<=x2)  
                {  
                    VLcd_DrawPoint(x1,y1,color);  
                    if(e>0){y1+=1;e-=dx;}     
                    x1+=1;  
                    e+=dy;  
                }  
            }  
            else        // 2/8 octant  
            {  
                e=dx-dy/2;  
                while(y1<=y2)  
                {  
                    VLcd_DrawPoint(x1,y1,color);  
                    if(e>0){x1+=1;e-=dy;}     
                    y1+=1;  
                    e+=dx;  
                }  
            }  
        }  
        else           // dy<0  
        {  
            dy=-dy;   // dy=abs(dy)  
            if(dx>=dy) // 8/8 octant  
            {  
                e=dy-dx/2;  
                while(x1<=x2)  
                {  
                    VLcd_DrawPoint(x1,y1,color);  
                    if(e>0){y1-=1;e-=dx;}     
                    x1+=1;  
                    e+=dy;  
                }  
            }  
            else        // 7/8 octant  
            {  
                e=dx-dy/2;  
                while(y1>=y2)  
                {  
                    VLcd_DrawPoint(x1,y1,color);  
                    if(e>0){x1+=1;e-=dy;}     
                    y1-=1;  
                    e+=dx;  
                }  
            }  
        }     
    }  
    else //dx<0  
    {  
        dx=-dx;     //dx=abs(dx)  
        if(dy >= 0) // dy>=0  
        {  
            if(dx>=dy) // 4/8 octant  
            {  
                e=dy-dx/2;  
                while(x1>=x2)  
                {  
                    VLcd_DrawPoint(x1,y1,color);  
                    if(e>0){y1+=1;e-=dx;}     
                    x1-=1;  
                    e+=dy;  
                }  
            }  
            else        // 3/8 octant  
            {  
                e=dx-dy/2;  
                while(y1<=y2)  
                {  
                    VLcd_DrawPoint(x1,y1,color);  
                    if(e>0){x1-=1;e-=dy;}     
                    y1+=1;  
                    e+=dx;  
                }  
            }  
        }  
        else           // dy<0  
        {  
            dy=-dy;   // dy=abs(dy)  
            if(dx>=dy) // 5/8 octant  
            {  
                e=dy-dx/2;  
                while(x1>=x2)  
                {  
                    VLcd_DrawPoint(x1,y1,color);  
                    if(e>0){y1-=1;e-=dx;}     
                    x1-=1;  
                    e+=dy;  
                }  
            }  
            else        // 6/8 octant  
            {  
                e=dx-dy/2;  
                while(y1>=y2)  
                {  
                    VLcd_DrawPoint(x1,y1,color);  
                    if(e>0){x1-=1;e-=dy;}     
                    y1-=1;  
                    e+=dx;  
                }  
            }  
        }     
    }  
}  
#endif



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef OLED_SOFTWARE
#define SPILCD_W 96
#define SPILCD_H 96


void OLED_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE);
													//          CS          SCK          MISO         SDA    
  GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
	//RS   
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//GPIO_SetBits(GPIOA, GPIO_Pin_2);
}
void Delayms(unsigned short time)
{
	unsigned short i,j;
	for(i=0;i<time;i++)
		for(j=0;j<2600;j++)	;
}


//写一个字节
void LCD_WriteByteSPI(unsigned char byte)
{
	  unsigned char buf;
    unsigned char i;
    for(i=0;i<8;i++) 
    {
        buf=(byte>>(7-i))&0x1;
        SPI_SDA(buf);
				SPI_DCLK(0);
        SPI_DCLK(1);
    }	
}
//写4bit,半个字节
void LCD_WriteoneSPI(unsigned char byte)
{
	  unsigned char buf;
    unsigned char i;
	
    for(i=0;i<4;i++) 
    {
        buf=(byte>>(3-i))&0x1;
        SPI_SDA(buf);
				SPI_DCLK(0);
        SPI_DCLK(1);
    }	

}
//写一个字节命令
void WriteComm(unsigned char dat)
{
	SPI_CS(0);
	lcd_RS(0);
	LCD_WriteByteSPI(dat);	//upper eight bits
	lcd_RS(1);
	SPI_CS(1);
}
//写一个字节数据
void LCD_WriteData(unsigned short dat)
{
	  SPI_CS(0);
		lcd_RS(1);
    LCD_WriteByteSPI(dat);
}

void ILI9325_CMO24_Initial(void)
{
	  SPI_CS(0);
	  Delayms(25);

		WriteComm(0x15);//SET COLUMN ADDR 
		WriteComm(0x10); 
		WriteComm(0x3f); 
		WriteComm(0x75);//SET ROW ADDR 
		WriteComm(0x00); 
		WriteComm(0x5f); 

		WriteComm(0xA0);//SET RE-MAP 
		WriteComm(0x51); 
		WriteComm(0xA1);//SET DISPLAY START LINE 
		WriteComm(0x00); 
		WriteComm(0xA2);//SET DISPLAY OFFSET 
		WriteComm(0x60); 
		WriteComm(0xA4);//SET DISPLAY MODE 
		WriteComm(0xA8);//SET MUX RADIO 
		WriteComm(0x5F); 
		WriteComm(0xB2);//SET FRAME FREQUENCY 
		WriteComm(0x23); 
		WriteComm(0xB3);//SET FRONT CLOCK DIVIDER & OSCILLATOR FREQUENCY 
		WriteComm(0xF0); 

		WriteComm(0x81);//SET CONTRAST CURRENT 
		WriteComm(0x7F);// 
		WriteComm(0xBC);    //first pre_charge voltage 
		WriteComm(0x1F); 
		Delayms(10);
		WriteComm(0x82);   //second pre_charge speed 
		WriteComm(0xFE); 
		Delayms(10);
		WriteComm(0xB1);   //first pre_charge phase length 
		WriteComm(0x21); 

		WriteComm(0xBB);//SET SECONDE PRE_CHARGE PERIOD 
		WriteComm(0x0F); 

		WriteComm(0xbe);//SET VCOMH 
		WriteComm(0x1F); 

		WriteComm(0xB8);        //SET GS 
		WriteComm(0x04); //GS1 
		WriteComm(0x06); //GS2 
		WriteComm(0x08); //GS3         
		WriteComm(0x0A); //GS4 
		WriteComm(0x0C); //GS5  
		WriteComm(0x0E); //GS6       
		WriteComm(0x10); //GS7 
		WriteComm(0x12); //GS8 
		WriteComm(0x14); //GS9   
		WriteComm(0x16); //GS10 
		WriteComm(0x18); //GS11 
		WriteComm(0x1A); //GS12 
		WriteComm(0x1C); //GS13 
		WriteComm(0x1E); //GS14 
		WriteComm(0x20); //GS15 
		Delayms(10);
		 
		WriteComm(0xAF);         //DSPLAY ON
}

//函数名：SPILCD_Clear
//参  数：Color 颜色      
void SPILCD_Clear(unsigned short Color)
{
  unsigned short x,y;   	
  WriteComm(0x15);//SET COLUMN ADDR 
	WriteComm(0x10); 
	WriteComm(63); 
	WriteComm(0x75);//SET ROW ADDR 
	WriteComm(0x00); 
	WriteComm(95); 
  SPI_CS(0);	
	for(x=0;x<96;x++)
		for(y=0;y<48;y++)
	{
	  lcd_RS(1);
    LCD_WriteByteSPI(Color);
	}
	SPI_CS(1);
} 

//------------------------------------------------------------------------------------------------------------

//函数名：SPILCD_ShowChar
//参  数：
//(x,y): 
//num:要显示的字符:" "--->"~"
//size:字体大小 8/16
//mode:叠加方式(1)还是非叠加方式(0)
void SPILCD_ShowChar(unsigned char x,unsigned char y,unsigned char num)
{       
//	unsigned char temp;
//	unsigned int pos,t,i;  
//	unsigned char size; 
//	size=16;	//找到字体大小
//  WriteComm(0x15);//SET COLUMN ADDR 
//	WriteComm(0x10+x); 
//	WriteComm(0x10+x+3); 
//	WriteComm(0x75);//SET ROW ADDR 
//	WriteComm(0x00+y); 
//	WriteComm(0x00+y+15); 
//	lcd_RS(1);
//	SPI_CS(0);  
//	num=num-' ';//得到偏移后的值
//	i=num*16;

//		for(pos=0;pos<size;pos++)
//		{

//			temp=nAsciiDot[i+pos];	//调通调用艺术字体
//			for(t=0;t<8;t++)
//		   {                 
//		      if(temp&0x80)
//						LCD_WriteoneSPI(0xff);
//					else 
//						LCD_WriteoneSPI(0x00);
//		      temp<<=1; 
//		    }
//		}	

	  unsigned char i,j,X,Y,M;
	  unsigned int POS;
		Y=y;
		POS=(num-' ')*16;
		for(i=0;i<16;i++){
			X=x;
			M=nAsciiDot[POS+i];	
			for(j=0;j<8;j++)
			{
				if(M&0x80)VLcd_DrawPoint(X,Y,0X0F);
				else VLcd_DrawPoint(X,Y,0X00);
				X++;
				M=M<<1;
			}
			 Y++;
		}
}  
void PutGB1616(unsigned char x, unsigned char  y, unsigned char c[2])
{
//	unsigned int i,j,k;
//	unsigned short m;
//	WriteComm(0x15);//SET COLUMN ADDR 
//	WriteComm(0x10+x); 
//	WriteComm(0x10+x+7); 
//	WriteComm(0x75);//SET ROW ADDR 
//	WriteComm(0x00+y); 
//	WriteComm(0x00+y+15); 	
//	lcd_RS(1);
// SPI_CS(0);	
//	for (k=0;k<64;k++) { //64标示自建汉字库中的个数，循环查询内码
//	  if ((codeGB_16[k].Index[0]==c[0])&&(codeGB_16[k].Index[1]==c[1]))
//			{ 
//    	for(i=0;i<32;i++) 
//			{
//				m=codeGB_16[k].Msk[i];
//				for(j=0;j<8;j++) 
//				{		
//					if((m&0x80)==0x80) {
//						// VLcd_DrawPoint();
//						LCD_WriteoneSPI(0xff); 
//						}   
//					else {
//						LCD_WriteoneSPI(0x00);
//						}
//					m=m<<1;
//				} 
//				if(i%2){y++;x=x-8;}
//				else x=x+8;
//		  }
//		}  
//	  }	
	
		
	unsigned char i,j,k,X,Y,M;
	for (k=0;k<64;k++)  //64标示自建汉字库中的个数，循环查询内码
	  if ((codeGB_16[k].Index[0]==c[0])&&(codeGB_16[k].Index[1]==c[1]))
		for(i=0;i<32;i+=2){
			Y=y+i/2;
			X=x;
			
			M=codeGB_16[k].Msk[i];
			for(j=0;j<8;j++){
				if(M&0x80)VLcd_DrawPoint(X,Y,0X0F);
				else VLcd_DrawPoint(X,Y,0X00);
				X++;
				M=M<<1;}
			
		M=codeGB_16[k].Msk[i+1];
		for(j=0;j<8;j++){
				if(M&0x80)VLcd_DrawPoint(X,Y,0X0F);
				else VLcd_DrawPoint(X,Y,0X00);
			  X++;
			  M=M<<1;}	
		}
		
	}

///////////////////////////////////////////
void LCD_PutString(unsigned char x, unsigned char y, unsigned char *s) 
{
	unsigned char l=0;
	while(*s) 
		{
			if( *s < 0x80) 
				{
					SPILCD_ShowChar(x+l,y,*s);
					s++;l+=8;
				}
			else
				{
					PutGB1616(x+l,y,(unsigned char*)s);
					s+=2;l=l+16;
				}
		}
}

/*********************************************************
虚拟显存
**********************************************************/
static u8   VDis_buff[96][48]={0};

//刷屏函数
void VLcd_refresh(void)
{
	 unsigned  short int x ,y;
	 WriteComm(0x15);//SET COLUMN ADDR 
	 WriteComm(0x10); 
	 WriteComm(63); 
	 WriteComm(0x75);//SET ROW ADDR 
	 WriteComm(0x00); 
	 WriteComm(95); 	
   SPI_CS(0);	
	 lcd_RS(1);
	 for(y=0;y<96;y++)
	 for(x=0;x<48;x++)
   LCD_WriteByteSPI(VDis_buff[y][x]);
	 SPI_CS(1);
}
//在虚拟显存中画点
void VLcd_DrawPoint(int16_t  x,int16_t  y,int32_t   Data)
{
	u8 X,Y,Color;
	X=(int8_t)((x&0x00FF)/2);
	Y=(int8_t)(y&0x00FF);
  Color=(int8_t)(Data&0X0000000F);//截取低四位作为灰度等级
	 if(x%2)VDis_buff[Y][X]=(VDis_buff[Y][X]&0xF0)|Color;
	 else   VDis_buff[Y][X]=(VDis_buff[Y][X]&0x0f)|(Color<<4);
}

//在虚拟显存中读点
u8  VLcd_ReadPoint(int16_t  x,int16_t  y)
{
	u8 X,Y;
	X=(int8_t)((x&0X00FF)/2);
	Y=(int8_t)(y&0x00FF);
 //截取低四位作为灰度等级
	 if(x%2)return (VDis_buff[Y][X]&0XF0)>>4;
	 else return(VDis_buff[Y][X]&0X0F);
//	return 0;
}

void OLED_TEST(void)
{
	WriteComm(0x15);//SET COLUMN ADDR 
	WriteComm(0x10); 
	WriteComm(0x11); 
	WriteComm(0x75);//SET ROW ADDR 
	WriteComm(0x00); 
	WriteComm(0x00); 
	lcd_RS(1);
	SPI_CS(0); 
//  LCD_WriteoneSPI(0x00);
//  LCD_WriteoneSPI(0x01);
//  LCD_WriteoneSPI(0x02);
//	LCD_WriteoneSPI(0x03);
//	LCD_WriteoneSPI(0x04);
//  LCD_WriteoneSPI(0xF5);
//  LCD_WriteoneSPI(0xF6);
//	LCD_WriteoneSPI(0xF7);
//	LCD_WriteoneSPI(0xF8);
//	LCD_WriteoneSPI(0x09);
//  LCD_WriteoneSPI(0x0A);
//	LCD_WriteoneSPI(0x0B);
//	LCD_WriteoneSPI(0x0C);
//  LCD_WriteoneSPI(0xFD);
//  LCD_WriteoneSPI(0xFE);
//	LCD_WriteoneSPI(0xFF);
//

	LCD_WriteByteSPI(0x01);
	LCD_WriteByteSPI(0x23);
//	LCD_WriteByteSPI(0x45);
//	LCD_WriteByteSPI(0x67);
//	LCD_WriteByteSPI(0x89);
//	LCD_WriteByteSPI(0xAB);
//	LCD_WriteByteSPI(0xCD);
//	LCD_WriteByteSPI(0xEF);
//	LCD_WriteByteSPI(0x08);
//	LCD_WriteByteSPI(0x09);
//	LCD_WriteByteSPI(0x0A);
//	LCD_WriteByteSPI(0x0B);
//	LCD_WriteByteSPI(0x0C);
//	LCD_WriteByteSPI(0x0D);
//	LCD_WriteByteSPI(0x0E);
//	LCD_WriteByteSPI(0x0F);
	
	
	while(1);
}
//
void SHOW_BMP(int16_t  x,int16_t  y,int16_t  L,int16_t  H, const  unsigned char *s){
  int16_t  i,j,k,M;
	for(j=0;j<H;j++)
		for(i=0;i<L/8;i++){
			M=s[j*(L/8)+i];
			for(k=0;k<8;k++){
			if(M&0X01)VLcd_DrawPoint(x+i*8+k,y+j,0x0000000f);
	     else VLcd_DrawPoint(x+i*8+k,y+j,0x00000000);
				M=M>>1;}
		}
}
#endif
