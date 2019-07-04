#ifndef __OLED_H
#define __OLED_H	
#include "sys.h"

#define OLED_HARDWARE
//#define OLED_SOFTWARE

#ifdef OLED_HARDWARE
//Òº¾§¿ØÖÆ¿ÚÖÃ1²Ù×÷Óï¾äºê¶¨Òå
#define	LCD_CS_SET    GPIOA->BSRR=GPIO_Pin_4  
#define	LCD_RS_SET  	GPIOC->BSRR=GPIO_Pin_4   
#define	LCD_SDA_SET  	GPIOA->BSRR=GPIO_Pin_7    
#define	LCD_SCL_SET  	GPIOA->BSRR=GPIO_Pin_5    
//#define	LCD_RST_SET GPIOB->BSRR=GPIO_Pin_9  
#define	LCD_BLCTR_SET  	GPIOC->BSRR=GPIO_Pin_5

//Òº¾§¿ØÖÆ¿ÚÖÃ0²Ù×÷Óï¾äºê¶¨Òå
#define	LCD_CS_CLR  	GPIOA->BRR=GPIO_Pin_4  
#define	LCD_RS_CLR  	GPIOC->BRR=GPIO_Pin_4   
#define	LCD_SDA_CLR  	GPIOA->BRR=GPIO_Pin_7   
#define	LCD_SCL_CLR  	GPIOA->BRR=GPIO_Pin_5 
//#define	LCD_RST_CLR GPIOB->BRR=GPIO_Pin_9    
#define	LCD_BLCTR_CLR  	GPIOC->BRR=GPIO_Pin_5 
						
void ILI9325_CMO24_Initial(void);
void Lcd_SPI1_DMA_Configuration( void );
void DMA1_Channel3IRQHandler(void);
void Lcd_SPI1_DMA_TX( void );

void VLcd_DrawPoint(int16_t  x,int16_t  y,int32_t   Data);
u8  VLcd_ReadPoint(int16_t  x,int16_t  y);

u8   SPI_WriteByte(SPI_TypeDef* SPIx,u8   Byte);
void Lcd_Write_Command(u8   Cmd);
void Lcd_Write_Data(u8   Data);
void VLcd_refresh(void);
void ILI9325_CMO24_Initial(void);
void SPILCD_Clear(unsigned short Color);
void SPILCD_ShowChar(unsigned char x,unsigned char y,unsigned char num);
void PutGB1616(unsigned char x, unsigned char  y, unsigned char c[2]);
void LCD_PutString(unsigned char x, unsigned char y, unsigned char *s);

void OLED_TEST(void);
void SPI_OLED_Refresh(void);
void SHOW_BMP(int16_t  x,int16_t  y,int16_t  L,int16_t  H,const  unsigned char *s);
void SHOW_4BIT_BMP(int16_t  x,int16_t  y,int16_t  L,int16_t  H, const unsigned char *s);

void InterBresenhamline (int x0,int y0,int x1, int y1,int color) ;
void draw_line(int x1,int y1,int x2,int y2,int color) ;

#endif 
//////////////////////////////////////////////////////////////////////////////////////////
#ifdef OLED_SOFTWARE
#define SPI_CS(a)	\
						if (a)	\
						GPIOA->BSRR = GPIO_Pin_4;	\
						else		\
						GPIOA->BRR = GPIO_Pin_4;
#define SPI_DCLK(a)	\
						if (a)	\
						GPIOA->BSRR = GPIO_Pin_5;	\
						else		\
						GPIOA->BRR = GPIO_Pin_5;
#define SPI_SDA(a)	\
						if (a)	\
						GPIOA->BSRR = GPIO_Pin_7;	\
						else		\
						GPIOA->BRR = GPIO_Pin_7;
#define lcd_RS(a)	\
						if (a)	\
						GPIOC->BSRR = GPIO_Pin_4;	\
						else		\
						GPIOC->BRR = GPIO_Pin_4;
						
void OLED_GPIO_Config(void);
void ILI9325_CMO24_Initial(void);
void Delayms(unsigned short time);
void LCD_WriteRegIndex(unsigned char Index);
void LCD_WriteData(unsigned short dat);
void SPILCD_Clear(unsigned short Color);
void SPILCD_Clear_Fast(unsigned char single_Color);
void SPILCD_Fill(unsigned short xsta,unsigned short ysta,unsigned short xend,unsigned short yend,unsigned short color);
void SPILCD_ShowChar(unsigned char x,unsigned char y,unsigned char num);
void LCD_PutString(unsigned char x, unsigned char y, unsigned char *s);
void LCD_Fill_Pic(u16 x, u16 y,u16 pic_H, u16 pic_V, const unsigned char* pic);


void VLcd_DrawPoint(int16_t  x,int16_t  y,int32_t   Data);
u8  VLcd_ReadPoint(int16_t  x,int16_t  y);
void VLcd_refresh(void);

void OLED_TEST(void);
						
void SHOW_BMP(int16_t  x,int16_t  y,int16_t  L,int16_t  H,const  unsigned char *s);
#endif 
#endif 
