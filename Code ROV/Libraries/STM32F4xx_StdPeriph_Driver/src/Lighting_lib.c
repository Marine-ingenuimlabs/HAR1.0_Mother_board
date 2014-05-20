/******************** (C) COPYRIGHT 2013 ROV TUNISIA ********************
* File Name          : Lighting_lib.c
* Author             : Rami HADJ TAIEB 
* Version            : V1.0
* Date               : 31/10/20
* Description        : Lighting control library using 2 PWM -STM32 Communication
****************************************************************************/

/* this code needs standard functions used by STM32F4xx.
 */
   
/* includes -----------------------------------------------------------*/
#include "stm32f4xx.h"
#include "Lighting_lib.h"
#include "structure.h"
   
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

 /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file*/

/*******************************************************************************
* Function Name  : Lighting_init
* Description    : Initialisation of PWM for the lighting 
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Lighting_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

 
  /* GPIOC, GPIOB and GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB  , ENABLE);
 
  /* GPIOC Configuration: TIM3 CH3 (PB0) and TIM3 CH4 (PB1)   */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  
  /* Connect TIM3 pins to AF2 */  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
  
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  uint16_t PrescalerValue;
  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2)/40000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 25-1;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
 
}

/*******************************************************************************
* Function Name  : Lighting_update
* Description    : Changing the lighting strength
* Input          : Right and left leds new values.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Lighting_update(light_led* light)
{
  TIM_OCInitTypeDef  TIM_OCInitStructure;  
  
  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = light->right.integer16;

  TIM_OC3Init(TIM3, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = light->left.integer16;

  TIM_OC4Init(TIM4, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
 
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  

  /* TIM3 enable counter */
 
  TIM_Cmd(TIM3, ENABLE);
  }

/* End of file ---------------------------------------------------------------*/