#ifndef __USART_H
#define	__USART_H
#include "sys.h" 


void USART1_Config(u32 Baud);
//void USART1_DMA_Tx_Data(void );
void USART1_DMA_Tx_Data(unsigned char *t,unsigned  char Lenth);
void USART1_TIMEOUT_Handler(void);
void YK_CMD_TX(void);

void USART2_Config(void);
void USART2_DMA_Tx_Data(u8 COUNT);

u8 VO_BUSY(void);
 
void  USART3_Config(void);
void  USART3_DMA_Tx_Data(void );

void WL_EN(unsigned char EN);
void WL_M0(unsigned char I );
unsigned char WL_LOCK(void );
unsigned char WL_AUX(void);
unsigned char WL_Config(unsigned char HEAD,\
	                      unsigned char ID,\
	                      unsigned char FHSS,\
	                      unsigned char BAUD,\
	                      unsigned char W_SPED,\
	                      unsigned char CHAN,\
	                      unsigned char IO_D,\
	                      unsigned char FEC,\
	                      unsigned char P_DB);
	unsigned char WL_Get_Config(unsigned char *Par);
	
////int fputc(int ch, FILE *f);	



#endif 
