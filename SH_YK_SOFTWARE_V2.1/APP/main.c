#include <stdio.h>
#include "delay.h"
#include "sys.h"
#include "adc.h"
#include "OLED.h"
#include "LED.h"
#include "usart.h"
#include "BUTTON.h"
#include "multi_timer.h"
#include "multi_button.h"
#include "TIMER.H"
//#include "smarttimer.h"
#include "ugui.h"
#include "math.h"
#include "rtc.h"
#include "iwdg.h"
#include "stmflash.h"
#include "MKY.H"

void SYS_RUN_LED_TICK(void );
void PC_SEND_TASK(void);
void YK_SEND_TASK(void);
void OLED_Refresh(void);
void VOIC_Play(int16_t Index);
void VOIC_AMP_Set(unsigned char VOL);
void VOIC_EQ_Set(unsigned char EQ);
void YK_MAKE_Fra(unsigned char CMD);
void SYS_START_Logo(void);
void SYS_Set(void);
unsigned char  SYS_Get_Par(unsigned char * Par);
void BAT_Show(void);
void RF_Show(void);
void USB_Show(void);
void RTC_Show(void);
void WL_Show(void);
void TOPBAR_Show(void);
//显示界面
void  Frame_Show(void);
void  FACE_Show(void);
void MABIAO_Show(unsigned short int x,unsigned short int y,unsigned short int Count);
void Nav_Show( unsigned short int x,unsigned short int y,unsigned short int r,unsigned short int Theta);
void PW_LR_Show( unsigned short int lr);
void HL_Show( unsigned short int hl);
void YK_REV_LOST_TIME_TICK(void);
void SYS_Enter_Sleep(void);

	//创建15个按键
	struct Button BUT1,BUT2,BUT3,BUT4,BUT5,BUT6,BUT7,BUT8,BUT9,BUT10,BUT11,BUT12,BUT13,BUT14,BUT15;
//创建按键回调函数
void BUT1_PRESS_DOWN_Handler (void* btn);
void BUT2_PRESS_DOWN_Handler (void* btn);
void BUT3_PRESS_DOWN_Handler (void* btn);
void BUT4_PRESS_DOWN_Handler (void* btn);
void BUT5_PRESS_DOWN_Handler (void* btn);
void BUT6_PRESS_DOWN_Handler (void* btn);
void BUT7_PRESS_DOWN_Handler (void* btn);
void BUT8_PRESS_DOWN_Handler (void* btn);
void BUT9_PRESS_DOWN_Handler (void* btn);
void BUT10_PRESS_DOWN_Handler(void* btn);
void BUT11_PRESS_DOWN_Handler(void* btn);
void BUT12_PRESS_DOWN_Handler(void* btn);
void BUT13_PRESS_DOWN_Handler(void* btn);
void BUT14_PRESS_DOWN_Handler(void* btn);
void BUT15_PRESS_DOWN_Handler(void* btn);

u32  Get_ChipID(void);

extern uint8_t PC_TX[];
extern uint8_t YK_TX[];
u16 adcx;
float temp,i;
unsigned int	Index=0;
extern uint16_t ADC_ConvertedValue[];
char TP_S[12]={'B','A','T',':','2','.','3','6','V'};
extern unsigned char  SPI1_DMA_TX_BUSY;

char Temp[10]="ABCDEFG012";
//unsigned char WL_Def[6]={0XC0,0X01,0X0A,0X1A,0X0A,0X44};
//unsigned char WL_Def[6]={0XC0,0X07,0X0A,0X3B,0X0A,0X40};
struct Timer Multi_Timer1,Multi_Timer2,Multi_Timer3,Multi_Timer4,Multi_Timer5,Multi_Timer6,Multi_Timer7,Multi_Timer8;
//=============GUI========================
UG_GUI gui ; // Global GUIstructure
#define MAX_OBJECTS 10
unsigned int Dis_Tick=0;
	
//主机的状态结构体
typedef struct 
{
	unsigned short int M_HL;//高低档
	unsigned short int M_NAV;//航向角
	unsigned short int M_PW;//喷雾开关
	unsigned short int M_Oil;//油量
  unsigned short int M_Med;//药量
  unsigned short int M_MOTO;//运动状态
} MB_STA;
MB_STA MB_Sta;//遥控接收数据帧后解析到这个结构体

//系统参数设置结构体
typedef struct 
{
	unsigned short int W_HEAD;//调频ID
	unsigned short int W_ID;//调频ID
	unsigned short int W_FHSS;//跳频数
	unsigned short int W_CHAN;//起始频段偏移数
  unsigned short int W_SPEED;//空速
  unsigned short int W_FEC;  //前向纠错
  unsigned short int W_BAUD;	//波特率
	unsigned short int W_IOD;	 //IO输出方式
	unsigned short int W_PDB;	 //发射功率
  unsigned short int V_AMP;	 //音量
	unsigned short int V_EQ;	 //音音效
} SYS_PAR;	
SYS_PAR SYS_Info;//解析后的系统参数
unsigned char SYS_Par[10]={0XC0,0X0D,0X0B,0X3B,0X07,0X44,25,1,0XFF,0XFF};//存储到flash的系统参数
SYS_PAR  SYS_Analysis_Par(unsigned char * Par);
unsigned char  SYS_Set_Par(unsigned char * Par);

unsigned int YK_Rev_LOST_TIME=0xFFFFFFFF;//遥控接收断开时间
unsigned int YK_NO_BT_PRESS_TIME=0xFFFFFFFF;//遥控连续没有按键的时间

unsigned char BT_VAL[16]={0};//存储当前按键值

 int main(void)
 { 
	  delay_init();	    	 //延时函数初始化	  
    ADC1_Init();	  		//ADC初始化	   
    LED_GPIO_Config()	;	 
	  USB_DET_Init();
	  USART1_Config(9600);
    USART2_Config();
    USART3_Config();
	  BUTTON_Init();
	  RTC_Init();
	  LEDR(1); LEDG(1); LEDB(1); LEDY(1); LEDW(1);
	 
	 #ifdef OLED_HARDWARE
   	      Lcd_SPI1_DMA_Configuration();
	 	#endif 

	 #ifdef OLED_SOFTWARE
	        OLED_GPIO_Config();
	 #endif 
	 
	  ILI9325_CMO24_Initial();	
	  SPILCD_Clear(0x00);
	  UG_Init(&gui,(void(*)(UG_S16,UG_S16,UG_COLOR))VLcd_DrawPoint,96 ,96) ;
    UG_SelectGUI (&gui);
		UG_SetForecolor( 15 );//前景色
    UG_SetBackcolor( 0 ); //背景色
		UG_FontSelect(&FONT_8X8 );
		delay_ms(100);
    SYS_Set();//系统设置界面
    SYS_START_Logo();//系统启动界面

	  //读取硬件现在的参数
    WL_Get_Config(SYS_Par);//直接从无线电台获取参数
    Index=STMFLASH_ReadHalfWord(0X0801F3F6);//从FLASH读取音效和音量参数
    SYS_Par[6]=(unsigned char)((0x00ff) &  Index);   //V_AMP;
    SYS_Par[7]=(unsigned char)((0x00ff) & (Index>>8));//V_EQ;
    SYS_Info=SYS_Analysis_Par(SYS_Par);//解析参数
   //显示电台参数
   //sprintf(TP_S,"%03d",SYS_Info.W_ID);  UG_FontSelect(&FONT_5X8);UG_PutString(75, 12,TP_S);
   //sprintf(TP_S,"%02d",SYS_Info.W_FHSS);UG_FontSelect(&FONT_5X8);UG_PutString(80, 21,TP_S);
   //sprintf(TP_S,"%02d",SYS_Info.W_CHAN);UG_FontSelect(&FONT_5X8);UG_PutString(80, 29,TP_S);

	// WL_M0(1);delay_ms(300);
	// USART1_DMA_Tx_Data(WL_Def,6); delay_ms(300);
	 
    //-----------HEAD, ID,  FHSS, BAUD, W_SPED,  CHAN, IO_D,  FEC,  P_DB
    //if( WL_Config(0XC0, 0x07,0x0A, 115,  128,     0X0A, 0X01,  0X00, 20))[ID:07,FHSS:10,CH:10,FCC:0]
	  //if( WL_Config(0XC0, 0x0D,0x0B, 115,  128,     0X07, 0X01,  0X01, 20))//[ID:13,FHSS:11,CH:07,FCC:1][C0 0D 0B 3B 07 44]
		//{delay_ms(100); LEDR(2);delay_ms(100); LEDR(2);delay_ms(100); LEDR(2);delay_ms(100); LEDR(2);delay_ms(100); LEDR(2);delay_ms(100); LEDR(1);}
		 WL_M0(0);	delay_ms(100);//开启透传
		 YK_TX[0]=0xFF;YK_TX[1]=0x00;YK_TX[2]=0x00;YK_TX[3]=0x00;YK_TX[4]=0xEE;//恢复发送数据区数据区
	   USART1_Config(115200);delay_ms(100);//设置传输波特率
			 
		//Test_Write(0X0801F3F0,WL_Def[1]<<8|WL_Def[0]);
		//Test_Write(0X0801F3F2,WL_Def[3]<<8|WL_Def[2]);
		//Test_Write(0X0801F3F4,WL_Def[5]<<8|WL_Def[4]);
		//Test_Write(0X0801F3F6,(WL_Def[1]<<8|WL_Def[0])^(WL_Def[3]<<8|WL_Def[2])^(WL_Def[5]<<8|WL_Def[4]));
		//系统运行指示灯LEDG(2);--
	  timer_init(&Multi_Timer1,  SYS_RUN_LED_TICK, 0, 50);  timer_start(&Multi_Timer1);//偏移100ms，500ms loop
	  timer_init(&Multi_Timer2,  YK_SEND_TASK,     8,  5);  timer_start(&Multi_Timer2);//偏移200ms，50ms loop
	  timer_init(&Multi_Timer3,  button_ticks,     21,  5);  timer_start(&Multi_Timer3);//偏移300ms，50ms loop
	  timer_init(&Multi_Timer4,  OLED_Refresh,     29, 10);  timer_start(&Multi_Timer4);//偏移500ms，100ms loop 
		timer_init(&Multi_Timer5,  TOPBAR_Show,      37, 10);  timer_start(&Multi_Timer5);//偏移400ms，500ms loop	
		timer_init(&Multi_Timer6,  YK_REV_LOST_TIME_TICK, 49, 10);timer_start(&Multi_Timer6);//偏移500ms，100ms loop
		timer_init(&Multi_Timer7,  Frame_Show,       57, 10);  timer_start(&Multi_Timer7);//偏移500ms，100ms loop
		
		 //初始化按键，注册按键响应事件，启动按钮
		button_init(&BUT1,  BT_1,  1);button_attach(&BUT1,  PRESS_DOWN, BUT1_PRESS_DOWN_Handler); button_start(&BUT1);
		button_init(&BUT2,  BT_2,  1);button_attach(&BUT2,  PRESS_DOWN, BUT2_PRESS_DOWN_Handler); button_start(&BUT2);
		button_init(&BUT3,  BT_3,  1);button_attach(&BUT3,  PRESS_DOWN, BUT3_PRESS_DOWN_Handler); button_start(&BUT3);
		button_init(&BUT4,  BT_4,  1);button_attach(&BUT4,  PRESS_DOWN, BUT4_PRESS_DOWN_Handler); button_start(&BUT4);
		button_init(&BUT5,  BT_5,  1);button_attach(&BUT5,  PRESS_DOWN, BUT5_PRESS_DOWN_Handler); button_start(&BUT5);
		button_init(&BUT6,  BT_6,  1);button_attach(&BUT6,  PRESS_DOWN, BUT6_PRESS_DOWN_Handler); button_start(&BUT6);
		button_init(&BUT7,  BT_7,  1);button_attach(&BUT7,  PRESS_DOWN, BUT7_PRESS_DOWN_Handler); button_start(&BUT7);
		button_init(&BUT8,  BT_8,  1);button_attach(&BUT8,  PRESS_DOWN, BUT8_PRESS_DOWN_Handler); button_start(&BUT8);
		button_init(&BUT9,  BT_9,  1);button_attach(&BUT9,  PRESS_DOWN, BUT9_PRESS_DOWN_Handler); button_start(&BUT9);
		button_init(&BUT10, BT_10, 1);button_attach(&BUT10, PRESS_DOWN, BUT10_PRESS_DOWN_Handler);button_start(&BUT10);
		button_init(&BUT11, BT_11, 1);button_attach(&BUT11, PRESS_DOWN, BUT11_PRESS_DOWN_Handler);button_start(&BUT11);
		button_init(&BUT12, BT_12, 1);button_attach(&BUT12, PRESS_DOWN, BUT12_PRESS_DOWN_Handler);button_start(&BUT12);
		button_init(&BUT13, BT_13, 1);button_attach(&BUT13, PRESS_DOWN, BUT13_PRESS_DOWN_Handler);button_start(&BUT13);
		button_init(&BUT14, BT_14, 1);button_attach(&BUT14, PRESS_DOWN, BUT14_PRESS_DOWN_Handler);button_start(&BUT14);
		button_init(&BUT15, BT_15, 1);button_attach(&BUT15, PRESS_DOWN, BUT15_PRESS_DOWN_Handler);button_start(&BUT15);
   // VOIC_AMP_Set(25);   delay_ms(100);//预设音量25
   // VOIC_EQ_Set(1);     delay_ms(100);//设置音效POP
		TIM2_Init(999,719); delay_ms(10);//每秒10ms定时中断时基--系统时基----Multi_Timer中断时基
	  IWDG_Init(4,625);   delay_ms(10);  //与分频数为64,重载值为625,溢出时间为1s	
	while(1)
	{
		IWDG_Feed();//主循环喂狗
		timer_loop();
		LEDY(2);
		if(VO_BUSY())LEDW(0);//音频正在播放时，打开白灯
		else LEDW(1);//指示语音播报状态
		if(BT_X()==0)
			  {
		   	  YK_MAKE_Fra(0x00);//没有按键时，清空发送缓冲区值
			     LEDY(1);
		      if(YK_NO_BT_PRESS_TIME>10*60*10){SYS_Enter_Sleep();}//超过10分钟没有按键，则进入休眠
		   }
		else YK_NO_BT_PRESS_TIME=0;
	}											    
}	
 
void SYS_RUN_LED_TICK(void){}//LEDG(2);}//系统运行闪灯
void YK_SEND_TASK(void){YK_CMD_TX();  LEDG(2);}//  通过电台发送数据
//SPI DMA刷屏
void OLED_Refresh(void)
{
    UG_Update();
  //SPI_OLED_Refresh();
    VLcd_refresh();
}

//右转--05====================================
void BUT1_PRESS_DOWN_Handler (void* btn){BT_VAL[1]=~BT_VAL[1];YK_MAKE_Fra(0x05);VOIC_Play(0X0001);UG_FontSelect(&FONT_8X8 );UG_PutString(78,85,"01");LEDY(2);}
//摇摆开关-14开  1D关
void BUT2_PRESS_DOWN_Handler (void* btn){BT_VAL[2]=~BT_VAL[2];if(BT_VAL[2]){YK_MAKE_Fra(0x14);VOIC_Play(20);}else{YK_MAKE_Fra(0x1d);VOIC_Play(21);}UG_FontSelect(&FONT_8X8 );UG_PutString(78,85,"02");LEDY(2);}
//停止--0E
void BUT3_PRESS_DOWN_Handler (void* btn){BT_VAL[3]=~BT_VAL[3];YK_MAKE_Fra(0x0E);VOIC_Play(0X0003);UG_FontSelect(&FONT_8X8 );UG_PutString(78,85,"03");LEDY(2);}
//后退--06
void BUT4_PRESS_DOWN_Handler (void* btn){BT_VAL[4]=~BT_VAL[4];YK_MAKE_Fra(0x06);VOIC_Play(0X0004);UG_FontSelect(&FONT_8X8 );UG_PutString(78,85,"04");LEDY(2);}
//挂高档--09
void BUT5_PRESS_DOWN_Handler (void* btn){BT_VAL[5]=~BT_VAL[5];YK_MAKE_Fra(0x09);VOIC_Play(16);UG_FontSelect(&FONT_8X8 );UG_PutString(78,85,"05");LEDY(2);}
//前进--0C=====================================
void BUT6_PRESS_DOWN_Handler (void* btn){BT_VAL[6]=~BT_VAL[6];YK_MAKE_Fra(0x0C);VOIC_Play(0X0006);UG_FontSelect(&FONT_8X8 );UG_PutString(78,85,"06");LEDY(2);}
//药泵开关--10开 11关
void BUT7_PRESS_DOWN_Handler (void* btn){BT_VAL[7]=~BT_VAL[7];if(BT_VAL[7]){YK_MAKE_Fra(0x10);VOIC_Play(22);}else{YK_MAKE_Fra(0x11);VOIC_Play(23);}UG_FontSelect(&FONT_8X8 );UG_PutString(78,85,"07");LEDY(2);}
//左侧喷雾打开12开 1E关
void BUT8_PRESS_DOWN_Handler (void* btn){BT_VAL[8]=~BT_VAL[8];if(BT_VAL[8]){YK_MAKE_Fra(0x12);VOIC_Play(26);}else{YK_MAKE_Fra(0x1E);VOIC_Play(27);}UG_FontSelect(&FONT_8X8 );UG_PutString(78,85,"08");LEDY(2);}
//油门加--02
void BUT9_PRESS_DOWN_Handler (void* btn){BT_VAL[9]=~BT_VAL[9];YK_MAKE_Fra(0x02);VOIC_Play(18);UG_FontSelect(&FONT_8X8 );UG_PutString(78,85,"09");LEDY(2);}
//油门减小--03
void BUT10_PRESS_DOWN_Handler(void* btn){BT_VAL[10]=~BT_VAL[10];YK_MAKE_Fra(0x03);VOIC_Play(19);UG_FontSelect(&FONT_8X8 );UG_PutString(78,85,"10");LEDY(2);}
//右侧喷雾打开--13打开 1F关闭
void BUT11_PRESS_DOWN_Handler(void* btn){BT_VAL[11]=~BT_VAL[11];if(BT_VAL[11]){YK_MAKE_Fra(0x13);VOIC_Play(28);}else{YK_MAKE_Fra(0x1F);VOIC_Play(29);}UG_FontSelect(&FONT_8X8 );UG_PutString(78,85,"11");LEDY(2);}
//左转--0B
void BUT12_PRESS_DOWN_Handler(void* btn){BT_VAL[12]=~BT_VAL[12];YK_MAKE_Fra(0x0B);VOIC_Play(0x000C);UG_FontSelect(&FONT_8X8 );UG_PutString(78,85,"12");LEDY(2);}
//挂低档--08
void BUT13_PRESS_DOWN_Handler(void* btn){BT_VAL[13]=~BT_VAL[13];YK_MAKE_Fra(0x08);VOIC_Play(17);UG_FontSelect(&FONT_8X8 );UG_PutString(78,85,"13");LEDY(2);}
//F2-发动机点火熄火--04开 05关
void BUT14_PRESS_DOWN_Handler(void* btn){BT_VAL[14]=~BT_VAL[14];if(BT_VAL[14]){YK_MAKE_Fra(0X04);VOIC_Play(185);}else{YK_MAKE_Fra(0X0A);VOIC_Play(120);}UG_FontSelect(&FONT_8X8 );UG_PutString(78,85,"14");LEDY(2);}
//喷雾风机启停--19开 1A关
void BUT15_PRESS_DOWN_Handler(void* btn){BT_VAL[15]=~BT_VAL[15];if(BT_VAL[15]){YK_MAKE_Fra(0X19);VOIC_Play(24);}else{YK_MAKE_Fra(0X1A);VOIC_Play(25);}UG_FontSelect(&FONT_8X8 );UG_PutString(78,85,"15");LEDY(2);}

//播放指定曲目
void VOIC_Play(int16_t Index)
	{
		 int16_t Pdex;
		 if(SYS_Info.V_AMP%2==0)Pdex=Index;else Pdex=100+Index*5;//当音量为奇数时无聊的神奇功能
     PC_TX[0]=0x7e;
     PC_TX[1]=0x05;
     PC_TX[2]=0x41;
     PC_TX[3]=(unsigned char)((Pdex>>8)&0x00FF);
     PC_TX[4]=(unsigned char)(Pdex&0x00FF); 
     PC_TX[5]=PC_TX[1]^PC_TX[2]^PC_TX[3]^PC_TX[4];
     PC_TX[6]=0xEF;
     USART2_DMA_Tx_Data(7);
 }
	//音量设置
 void VOIC_AMP_Set(unsigned char VOL)
	{
		if(VOL>=30)VOL=30;
     PC_TX[0]=0x7e;
     PC_TX[1]=0x04;
     PC_TX[2]=0x31;
     PC_TX[3]=VOL;
     PC_TX[4]=PC_TX[1]^PC_TX[2]^PC_TX[3];
     PC_TX[5]=0xEF;
     USART2_DMA_Tx_Data(6);
 }
	
	//音效设置，0-5,NO,POP,ROCK,JAZZ,CLASSIC,BASS
  void VOIC_EQ_Set(unsigned char EQ)
	{
		if(EQ>=5)EQ=0;
     PC_TX[0]=0x7e;
     PC_TX[1]=0x04;
     PC_TX[2]=0x32;
     PC_TX[3]=EQ;
     PC_TX[4]=PC_TX[1]^PC_TX[2]^PC_TX[3];
     PC_TX[5]=0xEF;
     USART2_DMA_Tx_Data(6);
 }
 //生成遥控发送数据帧
 void YK_MAKE_Fra(unsigned char CMD)
	{
    YK_TX[0]=0xFF;
    YK_TX[1]=(unsigned char)CMD;
    YK_TX[2]=0X00;
    YK_TX[3]=0x00;
	  //(unsigned char)(YK_TX[1]+YK_TX[2]);
	  //YK_TX[3]=(unsigned char)SYS_Info.W_ID;
    YK_TX[4]=0xEE; 
 }

//此处进入设置界面
void SYS_Set(void)
{
		unsigned short int index=0,Tmp;
    unsigned short int C_Index=0;
		unsigned char Par[6]={0};//显示屏上显示的各项参数，条目从下往上数1-NULLL,2-OL,3-CH,4-FHSS,5-ID,
		unsigned char Par_Store[10]={0};//存储到单片机FLASH的参数
//  unsigned char   Tmp;
		//float A,B;
		char Dis[3]={0};//输入数字显示缓冲区
		SPILCD_Clear(0x00);
		sprintf(Temp,"SN:%08X",Get_ChipID());//获取芯片ID的最后八位-十六进制
		while(BT_9() && BT_10())//按住两个按键不放+
			{
		       index++;
			     sprintf(TP_S,"%03d%c",index*2,'%');
			     UG_FontSelect(&FONT_8X8); UG_PutString(35, 60,TP_S);
					 UG_FontSelect(&FONT_8X14);UG_PutString(25, 20,"KEEP");
				   UG_PutString(15, 40,"PRESSED!");
			     UG_FillRoundFrame( 5, 82, ((unsigned short int )(index*1.7)+5), 92, 4, 15 );
					 UG_Update();
		       VLcd_refresh();//硬件刷屏
					 delay_ms(100);

				if(index==50)
					{
						if(VO_BUSY()==0x00)VOIC_Play(36);
				     while(BT_9() | BT_10());//等待松开两个按键
						 if(BT_5() && (VO_BUSY()==0x00))VOIC_Play(186);//播放喜羊羊
						 else if(BT_13() && (VO_BUSY()==0x00))VOIC_Play(187);//植物大战僵尸-grasswalk
						 else if(BT_6() && (VO_BUSY()==0x00))VOIC_Play(188);//植物大战僵尸--哈哈
						 SPILCD_Clear(0x00);
             //SYS_Get_Par(Par_Store);//获取系统flash存数参数
						      //读取硬件现在的参数
						      WL_Get_Config(Par_Store);//直接从无线电台获取参数
						      Tmp=STMFLASH_ReadHalfWord(0X0801F3F6);//从FLASH读取音效和音量参数
                  Par_Store[6]=(unsigned char)((0x00ff) &  Tmp);   //V_AMP;
                  Par_Store[7]=(unsigned char)((0x00ff) & (Tmp>>8));//V_EQ;
						      SYS_Info=SYS_Analysis_Par(Par_Store);
						      Par[5]=SYS_Info.W_ID;
						      Par[4]=SYS_Info.W_FHSS;
						      Par[3]=SYS_Info.W_CHAN;
					    	//板子初次上电可能出现错误参数
						      if(SYS_Info.V_AMP>=30) Par[2]=30;
						      else if(SYS_Info.V_AMP<=0)Par[2]=0;
						      else  Par[2]=SYS_Info.V_AMP;
					   while(1){
						           UG_Update();
		                   VLcd_refresh();//硬件刷屏
		                   delay_ms(200);
							 			   UG_DrawRoundFrame( 0, 0, 95, 95, 8, 0x00000000F );
							         //==菜单==
							         //6===NULL=============
							         //5===ID===============
							         //4===FHSS=============
							         //3===CH===============
							         //2===OL===============
							         //1===UPDATA===========
							         //0===NULL=============
							         UG_FontSelect(&FONT_8X8); UG_PutString(4 ,  2 ,Temp);//C_Index==6
							         UG_FontSelect(&FONT_8X14);UG_PutString(4 ,  12,"W_ID:");sprintf(Dis,"%03d",Par[5]);UG_PutString(55, 12,Dis);//C_Index==5
							         UG_FontSelect(&FONT_8X14);UG_PutString(4 ,  29,"FHSS:");sprintf(Dis,"%02d",Par[4]);UG_PutString(60, 29,Dis);//C_Index==4
							 	       UG_FontSelect(&FONT_8X14);UG_PutString(4 ,  46,"W_CH:");sprintf(Dis,"%02d",Par[3]);UG_PutString(60, 46,Dis);//C_Index==3
							         UG_FontSelect(&FONT_8X14);UG_PutString(4 ,  63,"V_OL:");sprintf(Dis,"%02d",Par[2]);UG_PutString(60, 63,Dis);//C_Index==2
							         UG_FontSelect(&FONT_8X14);UG_PutString(18, 80,"UPDATE!");//C_Index==1
							 
						           if(BT_4()){if(VO_BUSY()==0x00)VOIC_Play(32);if(C_Index<=1)C_Index=5;else C_Index--;}//下一行
						           if(BT_6()){if(VO_BUSY()==0x00)VOIC_Play(32);if(C_Index==5)C_Index=1;else C_Index++;}//上一行
											 
											 //无线电台参数设置约束
											// A=425+0.5*Par[3];   起始频率425+CH*0.5
											// B=425+0.5*Par[3]+0.5*Par[4];  起始频率、终止频率425+CH*0.5+FHSS*0.5<450
						           if(BT_1()){//右加_++参数约束
												           if(VO_BUSY()==0x00)VOIC_Play(32);
												 					 if(C_Index==1|| C_Index==5){if(Par[C_Index]<255)Par[C_Index]++;}
											             if(C_Index==2){if(Par[C_Index]<30) Par[C_Index]++;}
												           if(((C_Index==3)||(C_Index==4)) && ((Par[3]+Par[4])<50))Par[C_Index]++;
											           }
						           if( BT_12()){//左减--参数约束
												           if(VO_BUSY()==0x00) VOIC_Play(32);
												 					  if(C_Index==1){if(Par[C_Index]>0)Par[C_Index]--;}
																	  if(C_Index==5){if(Par[C_Index]>1)Par[C_Index]--;}
											              if(C_Index==2){if(Par[C_Index]>0) Par[C_Index]--;}
												            if((C_Index==3) && (Par[3]>1))Par[C_Index]--;
																	  if((C_Index==4) && (Par[4]>1))Par[C_Index]--;
											             }
											 //当前选项画框
						           switch(C_Index)
						           {
						           	case 0:
						           		{
						           	     UG_DrawRoundFrame( 2, 10, 94, 27, 4, 0x000000000 );
						           			 UG_DrawRoundFrame( 2, 27, 94, 43, 4, 0x000000000 );
						           			 UG_DrawRoundFrame( 2, 43, 94, 61, 4, 0x000000000 );
						           			 UG_DrawRoundFrame( 2, 61, 94, 78, 4, 0x000000000 );
						           			 UG_DrawRoundFrame( 2, 78, 94, 94, 4, 0x000000000 );
						           	     break;
						           		}
												case 1:
						           		{
						           	     UG_DrawRoundFrame( 2, 10, 94, 27, 4, 0x000000000 );
						           			 UG_DrawRoundFrame( 2, 27, 94, 43, 4, 0x000000000 );
						           			 UG_DrawRoundFrame( 2, 43, 94, 61, 4, 0x000000000 );
						           			 UG_DrawRoundFrame( 2, 61, 94, 78, 4, 0x000000000 );
						           			 UG_DrawRoundFrame( 2, 78, 94, 94, 4, 0x00000000F );
						           	     break;
						           		}
												case 2:
						           		{
						           	     UG_DrawRoundFrame( 2, 10, 94, 27, 4, 0x000000000 );
						           			 UG_DrawRoundFrame( 2, 27, 94, 43, 4, 0x000000000 );
						           			 UG_DrawRoundFrame( 2, 43, 94, 61, 4, 0x000000000 );
						           			 UG_DrawRoundFrame( 2, 78, 94, 94, 4, 0x000000000 );
														 UG_DrawRoundFrame( 2, 61, 94, 78, 4, 0x00000000F );
						           	     break;
						           		}
												case 3:
						           		{
						           	     UG_DrawRoundFrame( 2, 10, 94, 27, 4, 0x000000000 );
						           			 UG_DrawRoundFrame( 2, 27, 94, 43, 4, 0x000000000 );
						           			 UG_DrawRoundFrame( 2, 61, 94, 78, 4, 0x000000000 );
						           			 UG_DrawRoundFrame( 2, 78, 94, 94, 4, 0x000000000 );
														 UG_DrawRoundFrame( 2, 43, 94, 61, 4, 0x00000000F );
						           	     break;
						           		}	
												case 4:
						           		{
						           	     UG_DrawRoundFrame( 2, 10, 94, 27, 4, 0x000000000 );
						           			 UG_DrawRoundFrame( 2, 43, 94, 61, 4, 0x000000000 );
						           			 UG_DrawRoundFrame( 2, 61, 94, 78, 4, 0x000000000 );
						           			 UG_DrawRoundFrame( 2, 78, 94, 94, 4, 0x000000000 );
														 UG_DrawRoundFrame( 2, 27, 94, 43, 4, 0x00000000F );
						           	     break;
						           		}
												case 5:
						           		{
														 UG_DrawRoundFrame( 2, 10, 94, 27, 4, 0x00000000F );
						           			 UG_DrawRoundFrame( 2, 27, 94, 43, 4, 0x000000000 );
						           			 UG_DrawRoundFrame( 2, 43, 94, 61, 4, 0x000000000 );
						           			 UG_DrawRoundFrame( 2, 61, 94, 78, 4, 0x000000000 );
						           			 UG_DrawRoundFrame( 2, 78, 94, 94, 4, 0x000000000 );
						           	     break;
						           		}															
											}
						//此时选中了UPDATA，且按了设置键
								if( BT_3()){
									         if(VO_BUSY()==0x00)VOIC_Play(32);
													  if(C_Index==1)
														 {
															 //复制参数
															 Par_Store[1]=Par[5];
															 Par_Store[2]=Par[4];
															 Par_Store[4]=Par[3];
															 Par_Store[6]=Par[2];
															 Par_Store[7]=1;
															 //在三秒时间内完成设置工作
														   for(index=1;index<=30;index++)
														     {
														       UG_Update();
		                               VLcd_refresh();//硬件刷屏
														       UG_FillRoundFrame(2, 79, ((unsigned short int )(index*3.19)), 93, 4, 10 );
														       if(index==8)WL_Config(0XC0, Par[5],Par[4], 115,  128,    Par[3], 0X01,  0X01, 20);//设置无线电台
														       if(index==16)VOIC_AMP_Set(Par[2]);//设置音量
														       if(index==24)VOIC_EQ_Set(1);//设置音效POP
														       if(index==28)SYS_Set_Par(Par_Store);//存储系统参数到芯片flash
														       delay_ms(100);
														     }
																 if(VO_BUSY()==0x00)VOIC_Play(33);
																 delay_ms(100);
												        break;//退出设置
												       }
												}
						}
					}
				
				}
 }
//显示系统启动界面
void SYS_START_Logo(void)
	{
		unsigned int Index=0;
		SPILCD_Clear(0x00);
		while(1)
		{
			 UG_Update();
		   VLcd_refresh();//硬件刷屏
			 delay_ms(80);
			 UG_DrawRoundFrame( 0, 0, 95, 95, 8, 0x00000000F );
			 sprintf(TP_S,"%02d:%02d:%02d",calendar.hour,calendar.min,calendar.sec);
		   UG_FontSelect(&FONT_5X8);UG_PutString(3, 3,TP_S); 
       SHOW_BMP(16,20,64,47,gImage_MKY);
			 //SHOW_BMP(86,12,8,8,gImage_WIFI);
			 //SHOW_BMP(86,20,8,8,gImage_GPS);
			 //SHOW_BMP(2,2,8,8,gImage_BLE);
			 Index++;
		   UG_DrawRoundFrame( 75, 2, 89, 9, 3, 0x00000000F );//电池框
		   UG_DrawRoundFrame( 77, 4, 78, 7, 1, 0x00000000F );
		   UG_DrawRoundFrame( 80, 4, 81, 7, 1, 0x00000000F );
		   UG_DrawRoundFrame( 83, 4, 84, 7, 1, 0x00000000F );
		   UG_DrawRoundFrame( 86, 4, 87, 7, 1, 0x00000000F );
		   UG_DrawRoundFrame( 90, 5, 91, 6, 1, 0x00000000F );//电池头部
			switch((Index/5)%7){
				case 0:{
				    UG_FillCircle( 10, 85, 2, 0 );
						UG_FillCircle( 25, 85, 2, 0 );
						UG_FillCircle( 40, 85, 2, 0 );
						UG_FillCircle( 55, 85, 2, 0 );
						UG_FillCircle( 70, 85, 2, 0 );
						UG_FillCircle( 85, 85, 2, 0 );
						break;}                        
				case 1:{
				    UG_FillCircle( 10, 85, 2, 15 );
						UG_FillCircle( 25, 85, 2, 0 );
						UG_FillCircle( 40, 85, 2, 0 );
						UG_FillCircle( 55, 85, 2, 0 );
						UG_FillCircle( 70, 85, 2, 0 );
						UG_FillCircle( 85, 85, 2, 0 );
						break;}                        
				case 2:{
				    UG_FillCircle( 10, 85, 2, 15 );
						UG_FillCircle( 25, 85, 2, 15 );
						UG_FillCircle( 40, 85, 2, 0 );
						UG_FillCircle( 55, 85, 2, 0 );
						UG_FillCircle( 70, 85, 2, 0 );
						UG_FillCircle( 85, 85, 2, 0 );
						break;}                        
				case 3:{
				    UG_FillCircle( 10, 85, 2, 15 );
						UG_FillCircle( 25, 85, 2, 15 );
						UG_FillCircle( 40, 85, 2, 15 );
						UG_FillCircle( 55, 85, 2, 0 );
						UG_FillCircle( 70, 85, 2, 0 );
						UG_FillCircle( 85, 85, 2, 0 );
						break;}                                                
				case 4:{
					  if(VO_BUSY()==0x00)VOIC_Play(33);//启动音
				    UG_FillCircle( 10, 85, 2, 15 );
						UG_FillCircle( 25, 85, 2, 15 );
						UG_FillCircle( 40, 85, 2, 15 );
						UG_FillCircle( 55, 85, 2, 15 );
						UG_FillCircle( 70, 85, 2, 0 );
						UG_FillCircle( 85, 85, 2, 0 );
						break;}	
				case 5:{
				    UG_FillCircle( 10, 85, 2, 15 );
						UG_FillCircle( 25, 85, 2, 15 );
						UG_FillCircle( 40, 85, 2, 15 );
						UG_FillCircle( 55, 85, 2, 15 );
						UG_FillCircle( 70, 85, 2, 15 );
						UG_FillCircle( 85, 85, 2, 0 );
						break;}	
				case 6:{
				    UG_FillCircle( 10, 85, 2, 15 );
						UG_FillCircle( 25, 85, 2, 15 );
						UG_FillCircle( 40, 85, 2, 15 );
						UG_FillCircle( 55, 85, 2, 15 );
						UG_FillCircle( 70, 85, 2, 15 );
						UG_FillCircle( 85, 85, 2, 15 );
						break;}					
			}
			if((Index/5)>=7){SPILCD_Clear(0);break;}
		}	
}

// 读取系统参数并且检验参数
unsigned char  SYS_Get_Par(unsigned char * Par){
     unsigned char Tmp;
     //读取flash存储的参数表
     Tmp=STMFLASH_ReadHalfWord(0X0801F3F0);
     Par[0]=(unsigned char)((0x00ff) &  Tmp);
     Par[1]=(unsigned char)((0x00ff) & (Tmp>>8));
     Tmp=STMFLASH_ReadHalfWord(0X0801F3F2);
     Par[2]=(unsigned char)((0x00ff) &  Tmp);
     Par[3]=(unsigned char)((0x00ff) & (Tmp>>8));
     Tmp=STMFLASH_ReadHalfWord(0X0801F3F4);
     Par[4]=(unsigned char)((0x00ff) &  Tmp);
     Par[5]=(unsigned char)((0x00ff) & (Tmp>>8));
     Tmp=STMFLASH_ReadHalfWord(0X0801F3F6);
     Par[6]=(unsigned char)((0x00ff) &  Tmp);
     Par[7]=(unsigned char)((0x00ff) & (Tmp>>8));
     Tmp=STMFLASH_ReadHalfWord(0X0801F3F8);
     Par[8]=(unsigned char)((0x00ff) &  Tmp);
     Par[9]=(unsigned char)((0x00ff) & (Tmp>>8));
     // 校验一下
			if(((Par[1]<<8|Par[0])^\
				  (Par[3]<<8|Par[2])^\
			    (Par[5]<<8|Par[4])^\
			    (Par[7]<<8|Par[6]))\
			  ==(Par[9]<<8|Par[8]))\
			return 1;
			else return 0;  
}

// 把参数存储到单片机FLASH区域
unsigned char  SYS_Set_Par(unsigned char * Par){
  u16 Par_Tmp[5];
	//转换成16bit结构
	Par_Tmp[0]=(Par[1]<<8)|Par[0];
	Par_Tmp[1]=(Par[3]<<8)|Par[2];
	Par_Tmp[2]=(Par[5]<<8)|Par[4];
	Par_Tmp[3]=(Par[7]<<8)|Par[6];
	Par_Tmp[4]=(Par[1]<<8|Par[0])^(Par[3]<<8|Par[2])^(Par[5]<<8|Par[4])^(Par[7]<<8|Par[6]);
  STMFLASH_Write(0X0801F3F0,Par_Tmp,5);//写入FLASH
	//STMFLASH_Write_NoCheck(0X0801F3F0,Par_Tmp,5);
	return 1;
}
//解析存储的系统参数到结构体
SYS_PAR  SYS_Analysis_Par(unsigned char * Par){
  SYS_PAR Sys_Par_Tmp;
	Sys_Par_Tmp.W_HEAD =Par[0];//C0：所设置的参数会掉电保存,C2：所设置的参数不会掉电保存。
	Sys_Par_Tmp.W_ID   =Par[1];//调频ID
	Sys_Par_Tmp.W_FHSS =Par[2];//跳频数425MHz + CHAN*0.5MHz + FHSS*0.5MHz
	switch(Par[3]&0x03){//空速
	case 0x00:{Sys_Par_Tmp.W_SPEED=16; break;}
	case 0x01:{Sys_Par_Tmp.W_SPEED=32; break;}
	case 0x02:{Sys_Par_Tmp.W_SPEED=64; break;}
	case 0x03:{Sys_Par_Tmp.W_SPEED=128;break;}}
	
	switch((Par[3]>>3)&0x07){//波特率
	case 0x00:{Sys_Par_Tmp.W_BAUD=12; break;}
	case 0x01:{Sys_Par_Tmp.W_BAUD=24; break;}
	case 0x02:{Sys_Par_Tmp.W_BAUD=48; break;}
	case 0x03:{Sys_Par_Tmp.W_BAUD=96; break;}
	case 0x04:{Sys_Par_Tmp.W_BAUD=192;break;}
	case 0x05:{Sys_Par_Tmp.W_BAUD=38; break;}
	case 0x06:{Sys_Par_Tmp.W_BAUD=57; break;}
	case 0x07:{Sys_Par_Tmp.W_BAUD=115;break;}}
	
	Sys_Par_Tmp.W_CHAN =Par[4]; //起始频段偏移数:425MHz + CHAN *0.5MHz
	
	switch(Par[5]&0x40){//IO 驱动方式（默认1）
	case 0x00:{Sys_Par_Tmp.W_IOD=0; break;}
	case 0x40:{Sys_Par_Tmp.W_IOD=1; break;}}
	
	switch(Par[5]&0x04){//FEC
	case 0x00:{Sys_Par_Tmp.W_FEC=0; break;}
	case 0x04:{Sys_Par_Tmp.W_FEC=1; break;}}

	switch(Par[5]&0x03){//发射功率
	case 0x00:{Sys_Par_Tmp.W_PDB=20; break;}
	case 0x01:{Sys_Par_Tmp.W_PDB=17; break;}
	case 0x02:{Sys_Par_Tmp.W_PDB=14; break;}
	case 0x03:{Sys_Par_Tmp.W_PDB=11;break;}}
	
  Sys_Par_Tmp.V_AMP = Par[6];	 //音量
	Sys_Par_Tmp.V_EQ  = Par[7];	  //音音效
	
	return Sys_Par_Tmp;
}
//显示电量
void BAT_Show(void)
{
  float BAT_V;
	BAT_V=(float)(Get_Adc_Average(ADC_Channel_1,10))*(3.3*2.0/4096.0);
  if(BAT_V>=4.0){
		              UG_DrawRoundFrame( 75, 2, 89, 9, 3, 0x00000000F );//电池框
		              UG_DrawRoundFrame( 77, 4, 78, 7, 1, 0x00000000F );
		              UG_DrawRoundFrame( 80, 4, 81, 7, 1, 0x00000000F );
		              UG_DrawRoundFrame( 83, 4, 84, 7, 1, 0x00000000F );
		              UG_DrawRoundFrame( 86, 4, 87, 7, 1, 0x00000000F );
		              UG_DrawRoundFrame( 90, 5, 91, 6, 1, 0x00000000F );//电池头部
		              return;}
	
  else if(BAT_V>=3.8){
			                UG_DrawRoundFrame( 75, 2, 89, 9, 3, 0x00000000F );//电池框
		                  UG_DrawRoundFrame( 77, 4, 78, 7, 1, 0x00000000F );
		                  UG_DrawRoundFrame( 80, 4, 81, 7, 1, 0x00000000F );
		                  UG_DrawRoundFrame( 83, 4, 84, 7, 1, 0x00000000F );
		                  UG_DrawRoundFrame( 86, 4, 87, 7, 1, 0x000000000 );
		                  UG_DrawRoundFrame( 90, 5, 91, 6, 1, 0x00000000F );//电池头部
                      return; }
  
  else if(BAT_V>=3.7){
		                 	UG_DrawRoundFrame( 75, 2, 89, 9, 3, 0x00000000F );//电池框
		                  UG_DrawRoundFrame( 77, 4, 78, 7, 1, 0x00000000F );
		                  UG_DrawRoundFrame( 80, 4, 81, 7, 1, 0x00000000F );
		                  UG_DrawRoundFrame( 83, 4, 84, 7, 1, 0x000000000 );
		                  UG_DrawRoundFrame( 86, 4, 87, 7, 1, 0x000000000 );
		                  UG_DrawRoundFrame( 90, 5, 91, 6, 1, 0x00000000F );//电池头部
		                  return;}
	
  else if(BAT_V>=3.6){
			                 UG_DrawRoundFrame( 75, 2, 89, 9, 3, 0x00000000F );//电池框
		                   UG_DrawRoundFrame( 77, 4, 78, 7, 1, 0x00000000F );
		                   UG_DrawRoundFrame( 80, 4, 81, 7, 1, 0x000000000 );
		                   UG_DrawRoundFrame( 83, 4, 84, 7, 1, 0x000000000 );
		                   UG_DrawRoundFrame( 86, 4, 87, 7, 1, 0x000000000 );
		                   UG_DrawRoundFrame( 90, 5, 91, 6, 1, 0x00000000F );//电池头部
		                   return;}
  
  else if(BAT_V>=3.5){
		                  UG_DrawRoundFrame( 75, 2, 89, 9, 3, 0x00000000F );//电池框
		                  UG_DrawRoundFrame( 77, 4, 78, 7, 1, 0x000000000 );
		                  UG_DrawRoundFrame( 80, 4, 81, 7, 1, 0x000000000 );
		                  UG_DrawRoundFrame( 83, 4, 84, 7, 1, 0x000000000 );
		                  UG_DrawRoundFrame( 86, 4, 87, 7, 1, 0x000000000 );
		                  UG_DrawRoundFrame( 90, 5, 91, 6, 1, 0x00000000F );//电池头部
		                  return;}
	
	             else {
		                  UG_DrawRoundFrame( 75, 2, 89, 9, 3, 0x000000008 );//电池框
		                  UG_DrawRoundFrame( 77, 4, 78, 7, 1, 0x000000000 );
		                  UG_DrawRoundFrame( 80, 4, 81, 7, 1, 0x000000000 );
		                  UG_DrawRoundFrame( 83, 4, 84, 7, 1, 0x000000000 );
		                  UG_DrawRoundFrame( 86, 4, 87, 7, 1, 0x000000000 );
		                  UG_DrawRoundFrame( 90, 5, 91, 6, 1, 0x00000000F );//电池头部
								      return;}
}
//显示信号图标
void RF_Show(void){
	unsigned short int X=43,Y=2;
	if(YK_Rev_LOST_TIME<=12){
			             UG_FontSelect(&FONT_8X8);UG_PutString(X+6, Y," "); 
			             UG_FillFrame( X,   Y+6, X+1, Y+7, 0x00000000F );
		               UG_FillFrame( X+3, Y+4, X+4, Y+7, 0x00000000F );
		               UG_FillFrame( X+6, Y+2, X+7, Y+7, 0x00000000F );
		               UG_FillFrame( X+9, Y,   X+10, Y+7, 0x00000000F );}
    else   {
				           UG_FillFrame( X,   Y+6, X+1, Y+7, 0x00000000F );
		               UG_FillFrame( X+3, Y+4, X+4, Y+7, 0x00000000F );
		               UG_FillFrame( X+6, Y+2, X+7, Y+7, 0x000000000 );
		               UG_FillFrame( X+9, Y,   X+10,Y+7, 0x000000000 );
	                 UG_FontSelect(&FONT_8X8);UG_PutString(X+6, Y,"X"); }
  }
//显示USB图标
void USB_Show(void){
		unsigned short int X=58,Y=2;
	  char Tmp[10]={0};
	if(USB_DET()){
						 SHOW_4BIT_BMP(X,Y,16,8, gImage_4bit_USB);}
	else{
          UG_FillFrame( X, Y, X+15, Y+7, 0x000000000 );//先清除这块区域
		      sprintf(Tmp,"%4.2f",(float)(Get_Adc_Average(ADC_Channel_1,10)*3.3*2.0/4096.0));
		      UG_FontSelect(&FONT_4X6 );
		      UG_PutChar(Tmp[0], X-1, Y+1,    15,0);
	        UG_PutChar(Tmp[1], X+3, Y+1,  15,0);
		      UG_PutChar(Tmp[2], X+7, Y+1,  15,0);
		      UG_PutChar(Tmp[3], X+11,Y+1, 15,0);
	
	}
}

//显示无线电台参数图标
void WL_Show(void){
	  char Temp[7];
	  unsigned short int X=1,Y=0;
		UG_DrawRoundFrame( X+2, Y+2,X+39,Y+9,3,0x00000000F );
		sprintf(Temp,"%03d%02d%02d",SYS_Info.W_ID,SYS_Info.W_FHSS,SYS_Info.W_CHAN);
		UG_FontSelect(&FONT_5X8);
	  UG_PutChar(Temp[0], X+3, Y+3, 0,15);
	  UG_PutChar(Temp[1], X+8, Y+3, 0,15);
	  UG_PutChar(Temp[2], X+13,Y+3, 0,15);
	  UG_PutChar(Temp[3], X+18,Y+3, 0,15);
	  UG_PutChar(Temp[4], X+23,Y+3, 0,15);
	  UG_PutChar(Temp[5], X+28,Y+3, 0,15);
	  UG_PutChar(Temp[6], X+33,Y+3, 0,15);
	  UG_DrawLine( X+2, Y+10, X+39, Y+10, 0 );
	  UG_DrawLine( X+38, Y+3, X+38, Y+8, 15 );
}

//显示实时时钟图标
void RTC_Show(void){
	char Temp[9];
	sprintf(Temp,"%02d:%02d:%02d",calendar.hour,calendar.min,calendar.sec);
  UG_FontSelect(&FONT_5X8);UG_PutString(20,86,Temp); 
}

//框架显示
void  FACE_Show(void){
//		char Temp[10];
		UG_DrawRoundFrame( 0, 0, 95, 95, 8, 0x00000000F );//外框
		/*
	  UG_DrawCircle(48, 32, 19, 15 );
	  UG_FillCircle( 48, 32, 2, 7);//画圆心
		draw_line( (unsigned short int)(47+21*cos(3.1415926/4)), (unsigned short int)(31+21*sin(3.1415926/4)),(unsigned short int)(49-20*cos(3.1415926/4)), (unsigned short int)(33-20*sin(3.1415926/4)), 15 );

		LCD_PutString(17,14,"农机系统"); 
	
		sprintf(Temp,"%s%04.1f%s","TMP:",(float)Get_Temp()/10.0,"C");
		UG_FontSelect(&FONT_8X14 );
		UG_PutString(3, 34,Temp);
		
		if(USB_DET())sprintf(Temp,"%s%04.2f%s","USB:",(float)(Get_Adc_Average(ADC_Channel_1,10)*3.3*2.0/4096.0),"V");
		else sprintf(Temp,"%s%04.2f%s","BAT:",(float)(Get_Adc_Average(ADC_Channel_1,10)*3.3*2.0/4096.0),"V");
		UG_FontSelect(&FONT_8X14 );
		UG_PutString(3, 50,Temp);
	*/
	}

	//码表模型
void MABIAO_Show(unsigned short int x,unsigned short int y,unsigned short int Count){
	 unsigned short int X=x,Y=y,R=18,r=1,Theta=0;
	unsigned short int A=1,B=0;
	unsigned short int COUNT=Count;
	unsigned short int Temp,i;
  char Dis[5];
	
		UG_DrawLine( X, Y, X-R, Y, 10 );//画左横梁
	  UG_DrawLine( X, Y, X+R-1, Y, 4 );//画右横梁
	  UG_FillCircle( X, Y, 2, 15);//画圆心
		//画弧线
	Temp=(unsigned short int)(1.8*COUNT);
	  for (i=0;i<=180;i=i+3){
		A=X-R*cos(i/180.0*3.1415926);
		B=Y-R*sin(i/180.0*3.1415926);
		if(i<=Temp)UG_DrawPixel(A,B,(unsigned short int)(i*15.0/Temp));
		else  UG_DrawPixel(A,B,4);}
		
		//画指针
		A=X-R*cos(COUNT*0.031415926);
		B=Y-R*sin(COUNT*0.031415926);
		draw_line( X, Y,A, B, 15 );
  //画点点
		Temp=Temp/30;
  for (i=0;i<=Temp;i++)
	{
		Theta=i*30;
		A=X-R*cos(Theta/180.0*3.1415926);
		B=Y-R*sin(Theta/180.0*3.1415926);
	  UG_DrawCircle(A, B, r+1, 15 );
    UG_FillCircle( A, B, r, (unsigned short int)(5.0+10.0*(((float)i)/((float)Temp))));
		//显示数字
		sprintf(Dis,"%2d%%",COUNT);
		UG_FontSelect(&FONT_8X8 );
		UG_PutString(X-R+6, Y+5,Dis);
	}
}
//导航圆形的显示
void Nav_Show( unsigned short int x,unsigned short int y,unsigned short int r,unsigned short int Theta){
	  unsigned short int X=x,Y=y,R=r;
	  double THETA=(Theta)/180.0*3.1415926;
	  double A=R*cos(THETA),B=R*sin(THETA);
	  UG_FillCircle( X, Y, R-1, 0);
    UG_DrawCircle(X, Y, R, 15 );
	  UG_DrawCircle(X, Y, R+1, 15 );
	  UG_FillCircle( X, Y, 1, 12);//画圆心
		draw_line( (unsigned short int)(X+A), (unsigned short int)(Y-B),(unsigned short int)(X-A), (unsigned short int)(Y+B), 15 );
    A=X+(R-3)*cos(THETA),B=Y-(R-3)*sin(THETA);
    UG_FillCircle((unsigned short int)(A), (unsigned short int)(B), 2, 15);
}

//喷雾图形的显示
void PW_LR_Show( unsigned short int lr){
	unsigned short int i,Color;
	//左侧喷雾图标
	if(lr&0xf0)Color=15;else Color=0;
	for(i=0;i<9;i++)draw_line(3,16+i*4,10,24+i*2,Color);
	//右侧喷雾图标
	if(lr&0x0f)Color=15;else Color=0;
	for(i=0;i<9;i++)draw_line(93,16+i*4,86,24+i*2,Color);
}

//高低档位信息显示
void HL_Show( unsigned short int hl){
	 unsigned short int HL=hl&0x03;
	 unsigned short int X=7,Y=7;
  //UG_DrawRoundFrame( X, Y,X+8,Y+8,3,0x00000000F );
	//依据Nav的圆心坐标
		UG_FontSelect(&FONT_8X14);
    switch(HL){
    	case 0:{UG_PutChar('X', 48+X+18, 32-Y, 15,0);break;}
    	case 1:{UG_PutChar('H', 48+X+18, 32-Y, 15,0);break;}
    	case 2:{UG_PutChar('L', 48+X+18, 32-Y, 15,0);break;}}
		  UG_PutChar('O', 48-X-18-6, 32-Y, 15,0);
}

//顶部状态条显示界面
void TOPBAR_Show(void){
  	if(SPI1_DMA_TX_BUSY==0){
    // FACE_Show();
     WL_Show ( );
     RF_Show ( );
     USB_Show( );
     BAT_Show( );
     RTC_Show();
   // Nav_Show(48,32,20,45);
   // MABIAO_Show(25,70,75);
   // MABIAO_Show(72,70,85);
  	}
}

//显示界面
void  Frame_Show(void){
		if(SPI1_DMA_TX_BUSY==0){
       FACE_Show();
       Nav_Show(48,32,18,(((Dis_Tick++)*6)%360));
       MABIAO_Show(25,70,75);
       MABIAO_Show(72,70,85);
			 PW_LR_Show(0xff);
			 HL_Show(0);
		}
}

//计数器
void YK_REV_LOST_TIME_TICK(void){
  if((YK_Rev_LOST_TIME++)>=0xFFFFFFFE)YK_Rev_LOST_TIME=0XFFFFFFFE;
	if((YK_NO_BT_PRESS_TIME++)>=0xFFFFFFFE)YK_NO_BT_PRESS_TIME=0XFFFFFFFE;
	//在串口接收中断里边会清零此变量
}
//进入休眠省电模式
void SYS_Enter_Sleep(void){}

//获取芯片ID	
u32  Get_ChipID(void)
{
   u32 ChipUniqueID[3];
    //地址从小到大,先放低字节，再放高字节：小端模式
     ChipUniqueID[2] = *(__IO u32*)(0X1FFFF7E8);  // 低字节
     ChipUniqueID[1] = *(__IO u32 *)(0X1FFFF7EC); // 
     ChipUniqueID[0] = *(__IO u32 *)(0X1FFFF7F0); // 高字节
	   return ChipUniqueID[2];
}
