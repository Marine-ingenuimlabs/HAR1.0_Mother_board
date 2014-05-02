/******************** (C) COPYRIGHT 2013 ROV TUNISIA ********************
* File Name          : Timing_lib.c
* Author             : Rami HADJ TAIEB 
* Version            : V1.0
* Date               : 15/11/20
* Description        : Specific configuration of all single channel timers (10,11,13,14)
****************************************************************************/
 
/* this code needs standard functions used by STM32F4xx.
 */

/* Includes ------------------------------------------------------------------*/
#include "Timing_lib.h"
#include "stm32f4xx.h"

/*******************************************************************************
* Function Name  : TIM_Init
* Description    : Initialisation timers 10 at 10Hz, 11 at 50Hz, 12 at 100Hz and 13 at 1KHz
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void TIM_Init() {
  uint16_t PrescalerValue;
  NVIC_InitTypeDef NVIC_InitStructure; 
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  
  /*---------------------------- TIM10 configuration -------------------*/ 
  /* TIM10 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);
  
  /* Enable the TIM10 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&NVIC_InitStructure);
  
  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock ) / 10000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  
  TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM10, PrescalerValue, TIM_PSCReloadMode_Immediate);

  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse =  1000 ;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM10, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Disable);

  /* TIM Interrupts enable */
  TIM_ITConfig(TIM10, TIM_IT_CC1, ENABLE);

  /* TIM10 enable counter */
  TIM_Cmd(TIM10, ENABLE);
  
  
/*---------------------------- TIM11 configuration -------------------*/ 
  /* TIM11 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);

  /* Enable the TIM11 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_TIM11_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  

  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock ) / 10000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM11, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM11, PrescalerValue, TIM_PSCReloadMode_Immediate);

  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 200 ;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM11, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM11, TIM_OCPreload_Disable);

  /* TIM Interrupts enable */
  TIM_ITConfig(TIM11, TIM_IT_CC1, ENABLE);

  /* TIM11 enable counter */
  TIM_Cmd(TIM11, ENABLE);
  
  
 /*---------------------------- TIM13 configuration -------------------*/ 
  /* TIM13 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);

  /* Enable the TIM13 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  


  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 10000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM13, PrescalerValue, TIM_PSCReloadMode_Immediate);

  /* Output Compare Timing Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse =100;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM13, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM13, TIM_OCPreload_Disable);

  /* TIM Interrupts enable */
  TIM_ITConfig(TIM13, TIM_IT_CC1, ENABLE);

  /* TIM13 enable counter */
  TIM_Cmd(TIM13, ENABLE);
  
    
/*---------------------------- TIM14 configuration -------------------*/  
  /* TIM14 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

  /* Enable the TIM14 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM8_TRG_COM_TIM14_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 10000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM14, PrescalerValue, TIM_PSCReloadMode_Immediate);

  /* Output Compare Timing Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 10 ;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM14, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Disable);

  /* TIM Interrupts enable */
  TIM_ITConfig(TIM14, TIM_IT_CC1, ENABLE);

  /* TIM14 enable counter */
  TIM_Cmd(TIM14, ENABLE);
}