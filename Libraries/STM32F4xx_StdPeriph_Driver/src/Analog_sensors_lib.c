/******************** (C) COPYRIGHT 2013 ROV TUNISIA ********************
* File Name          : Analog_sensors_lib.c
* Author             : Rami HADJ TAIEB 
* Version            : V1.0
* Date               : 15/11/20
* Description        : Converting electrical analog quantities to digital 
****************************************************************************/

/* this code needs standard functions used by STM32F4xx.
 */
 
 /* Includes ------------------------------------------------------------------*/
 #include "stm32f4xx.h"
 #include "Analog_sensors_lib.h"

 /* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : AnalogSensors_Config
* Description    : Configure analog input for ADC3 channel 10,11,12 and 13 
* Input          : Pointer on the ADC converted data: ADC3ConvertedValue.(Empty)
* Output         : None.
* Return         : None.
*******************************************************************************/
void AnalogSensors_Config(uint32_t *ADC3ConvertedValue)
{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  /* Enable ADC3, DMA2 and GPIO clocks ****************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC3 , ENABLE);

  /* DMA2 Stream0 channel0 configuration **************************************/
  DMA_InitStructure.DMA_Channel = DMA_Channel_2;     // DMA channel 2
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;        // peripheral register address 
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ADC3ConvertedValue;              // Storage address
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;					   // Direction : from peripheral to memory
  DMA_InitStructure.DMA_BufferSize = 4;										   // Buffer size 4 x (4bytes)
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;             // No peripheral address incrementation 
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					   // Memory address incrementation
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;      // Peripheral data size ( 32bits)
  DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Word;		   // Memory data size ( 32bits)
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;							   // Circular mode (Return to column 0 after 4 incrementation)
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;						   // Set DMA as High priority
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         			   // DMA_FIFOMode_Enable
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;			   // 
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;				   // Amount of data that we can't interrupt it writing on memory 
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		   // Amount of data that we can't interrupt it Sampling from the peripheral
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);

  /* Configure ADC3 Channel10,11,12 and 13 pin PC0,PC1,PC2 and PC3 as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC3 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 4;
  ADC_Init(ADC3, &ADC_InitStructure);

  /* ADC3 regular channel12 configuration *************************************/
  ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 1, ADC_SampleTime_28Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 2, ADC_SampleTime_28Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 3, ADC_SampleTime_28Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, ADC_SampleTime_28Cycles);
	
 /* Enable DMA request after last transfer*/
  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC3, ENABLE);
  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);
  ADC_SoftwareStartConv(ADC3);
}
/*******************************************************************************
* Function Name  : AnalogSensors_Update
* Description    : Update analog input for ADC3 channel 10,11,12 and 13 
* Input          : Pointer on the ADC converted data: ADC3ConvertedValue.
* Output         : Pointer on physical data in volt: ADC3ConvertedVoltage .
* Return         : None.
*******************************************************************************/
void AnalogSensors_Update( uint32_t *ADC3ConvertedValue, uint32_t *ADC3ConvertedVoltage){
uint8_t i;
  for (i=0; i<4 ; i++)
  {       
      ADC3ConvertedVoltage[i] = ADC3ConvertedValue[i] *3300/0xFFF;
  }
}


  

