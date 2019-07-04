 #include "adc.h"
 #include "delay.h"
	   
		   
#define ADC1_DR_Address    ((u32)0x4001244C)
__IO uint16_t ADC_ConvertedValue[2]={0,0}; 

/*配置采样通道端口 使能GPIO时钟	  设置ADC采样PA0端口信号*/
 void ADC1_GPIO_Config(void)
{ 
	GPIO_InitTypeDef GPIO_InitStructure;    
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		    //GPIO设置为模拟输入
  GPIO_Init(GPIOB, &GPIO_InitStructure); 	
}


/*配置ADC1的工作模式为MDA模式  */
 void ADC1_Mode_Config(void)
{
  DMA_InitTypeDef DMA_InitStructure;
  ADC_InitTypeDef ADC_InitStructure;	
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //使能MDA1时钟
	/* DMA channel1 configuration */
  DMA_DeInit(DMA1_Channel1);  //指定DMA通道
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;//设置DMA外设地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;	//设置DMA内存地址，ADC转换结果直接放入该地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //外设为设置为数据传输的来源
  DMA_InitStructure.DMA_BufferSize =2;	//DMA缓冲区设置为2；
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc =DMA_MemoryInc_Enable;  //内存地址固定
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Low ;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  /* Enable DMA channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);  //使能DMA通道

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //分频因子6时钟为72M/6=12MHz
  //ADC_DeInit(ADC1);  //将外设 ADC1 的全部寄存器重设为缺省值
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);	//使能ADC1时钟
     
  /* ADC1 configuration */
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; //使用独立模式，扫描模式
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //无需外接触发器
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //使用数据右对齐
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 2;  // 只有2个转换通道-顺序进行规则转换的ADC通道的数目
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel9 configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9,   1, ADC_SampleTime_55Cycles5);//通道1采样周期55.5个时钟周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16,  2, ADC_SampleTime_55Cycles5);//通道1采样周期55.5个时钟周期
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);	 //使能ADC的DMA
  ADC_TempSensorVrefintCmd(ENABLE); //开启内部温度传感器
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE); //使能ADC1

  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
     
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);  //开始转换
}

/*初始化ADC1 */
void ADC1_Init(void)
{
	ADC1_GPIO_Config();
	ADC1_Mode_Config();
}

u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=ADC_ConvertedValue[0];
		//delay_ms(5);
	}
	return temp_val/times;
} 	 


int Get_Temp(void)
{				 
	u16 temp_val=0;
	u8 t;
	float temperate;   
	for(t=0;t<20;t++)//读20次,取平均值
	{
		temp_val+=ADC_ConvertedValue[1];
		//delay_ms(1);
	}
	temp_val/=20;
	temperate=(float)temp_val*(3.3/4096);//得到温度传感器的电压值
	temperate=(1.43-temperate)/0.0043+25;//计算出当前温度值	 
	temperate*=10;//扩大十倍,使用小数点后一位
	return (int)temperate;	 

}
























