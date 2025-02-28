#include "ele_ccd.h"
#define TSL_SI    PBout(1)   //SI  
#define TSL_CLK   PAout(1)   //CLK 
/***********************************************
��˾����Ȥ�Ƽ�����ݸ�����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V1.0
�޸�ʱ�䣺2023-01-04

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update��2023-01-04

All rights reserved
***********************************************/
/**************************************************************************
�������ܣ���ʱ
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void Dly_us(void)
{
   int ii;    
   for(ii=0;ii<10;ii++);      
}
/**************************************************************************
�������ܣ�CCD���ݲɼ�
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void RD_TSL(void) 
{
	u8 i=0,tslp=0;
  TSL_CLK=0;
  TSL_SI=0; 
  Dly_us();
      
  TSL_SI=1; 
  Dly_us();
	
  TSL_CLK=0;
  Dly_us();
      
  TSL_CLK=1;
  Dly_us();
	
  TSL_SI=0;
  Dly_us(); 
  for(i=0;i<128;i++)
  { 
    TSL_CLK=0; 
    Dly_us();  //�����ع�ʱ��
    ADV[tslp]=(Get_Adc(2))>>4;
    ++tslp;
    TSL_CLK=1;
     Dly_us();
	}
}

/**************************************************************************
Function: Get_Voltage
Input   : none
Output  : none
�������ܣ���ȡADC��ֵ
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 
//��ȡADC��ֵ����Ҫ���ڳ��͵�ѡ��
u16 Get_Adc(u8 ch)   
{
	u16 result;
	ADC_ChannelConfTypeDef sConfig;//ͨ����ʼ��
	sConfig.Channel=ch;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;		//��������239.5����
	sConfig.Rank = 1;
	HAL_ADC_ConfigChannel(&hadc1,&sConfig);		
	
	HAL_ADC_Start(&hadc1);								//����ת��
	HAL_ADC_PollForConversion(&hadc1,30);				//�ȴ�ת������
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
	{
		result=HAL_ADC_GetValue(&hadc1);	//�������һ��ADC1�������ת�����
	}
	return result;
}

///**************************************************************************
//�������ܣ�CCD���ݲɼ�
//��ڲ�������
//����  ֵ����
//��    �ߣ�ƽ��С��֮��
//**************************************************************************/
//void CCD(void)   
//{  
//	int i,j;
//	usart1_send(0xff);			
//   for(i=0; i<100;i++) 
//  { 
//    RD_TSL();           
//   	j=0;
//	for(j=0;j<128;j++)
//	{
//      if(ADV[j] ==0XFF)  ADV[j]--;    
//     usart1_send(ADV[j]);
//    }   
//  }
//}
/**************************************************************************
Function: CCD_ADC_Mode_Config
Input   : none
Output  : none
�������ܣ���ʼ��CCDѲ��ADC
��ڲ���: ��
����  ֵ����
**************************************************************************/	 	
void CCD_ADC_Mode_Config(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADCEx_Calibration_Start(&hadc1); //����adcУ׼

}

/**************************************************************************
Function: CCD_GPIO_Config
Input   : none
Output  : none
�������ܣ���ʼ��CCDѲ��GPIO
��ڲ���: ��
����  ֵ����
**************************************************************************/	 	
void CCD_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	// ��CCD�˿�ʱ��
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_AFIO_CLK_ENABLE();
//	__HAL_AFIO_REMAP_SWJ_NOJTAG();      //�ر�jtag
//	DBGMCU->CR  &= ~((uint32_t)1<<5);  	//�ر��첽���٣�����PB3��һֱ����0 
	
	// CLK,SI����Ϊ���	
	GPIO_InitStruct.Pin = GPIO_PIN_1;				//PA1
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);	


	GPIO_InitStruct.Pin = GPIO_PIN_1;				//PB1
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);		
	
}


/**************************************************************************
Function: CCD_Init
Input   : none
Output  : none
�������ܣ���ʼ��CCDѲ��
��ڲ���: ��
����  ֵ����
**************************************************************************/	 	
void CCD_Init(void)
{
	CCD_GPIO_Config();
//	CCD_ADC_Mode_Config();

}

/**************************************************************************
�������ܣ���Ŵ�����������ʼ��
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void  ELE_Init(void)
{    
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  
   sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADCEx_Calibration_Start(&hadc1); //����adcУ׼
}		


