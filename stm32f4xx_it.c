/**
  ******************************************************************************
  * @file    TIM_PWM_Output/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main Interrupt Service Routines.
  *         This file provides template for all exceptions handler and
  *         peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "communication_lib.h"

/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup TIM_PWM_Output
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Extern variables ---------------------------------------------------------*/
extern void volatile (*ptr[16])(ROV_Struct*,CanRxMsg); // Functions Callback Table
extern ROV_Struct ROV;
/* Private variables ---------------------------------------------------------*/
uint8_t sle7 = 0;
uint16_t capture = 0;
union_float f;
char* s;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
//delay function
void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}
/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */


void SysTick_Handler(void)
{
  
   

}
void UART4_IRQHandler(void)
{
	if(USART_GetITStatus(UART4, USART_IT_RXNE) == SET)
	{
           if((ROV.State&&0x02) == 0x02) //Check if Communication With AIOP is enabled
           {
            AIOP_ReceiveChar((USART_ReceiveData(UART4) & 0xFF),(&(ROV.AIO)));   
            USART_ClearITPendingBit(UART4, USART_IT_RXNE);
           }
	}
}

void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
           /*get the caracter readed  from the RS485 bus*/
          USART_ReceiveData(USART2) ; //
          USART_ClearITPendingBit(USART2, USART_IT_RXNE);  
	}
        if(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == SET)
        {
         // RS485_Send();
          USART_ClearITPendingBit(USART2, USART_FLAG_TXE); 
        }
}
void TIM1_UP_TIM10_IRQHandler(void)
{

  if (TIM_GetITStatus(TIM10, TIM_IT_CC1) != RESET)
  {     if(((ROV.State)&&0x01) == 0x01) //check if streaming is allowed
  {
         ROV_Stream_VAR(ROV);
  }
        
    TIM_ClearITPendingBit(TIM10, TIM_IT_CC1);
    capture = TIM_GetCapture1(TIM10);
    TIM_SetCompare1(TIM10, capture + 1000);
  }
}
void TIM1_TRG_COM_TIM11_IRQHandler(void) //at a frequency of 50hz
{   
  if (TIM_GetITStatus(TIM11, TIM_IT_CC1) != RESET)
  {
    

    TIM_ClearITPendingBit(TIM11, TIM_IT_CC1);
    capture = TIM_GetCapture1(TIM11);
    TIM_SetCompare1(TIM11, capture + 200);
  }
}
void TIM8_UP_TIM13_IRQHandler(void)
{
  
  
  if (TIM_GetITStatus(TIM13, TIM_IT_CC1) != RESET)
  {
    
    TIM_ClearITPendingBit(TIM13, TIM_IT_CC1);
    capture = TIM_GetCapture1(TIM13);
    TIM_SetCompare1(TIM13, capture + 100);
  }
}
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{

  if (TIM_GetITStatus(TIM14, TIM_IT_CC1) != RESET)
  {
     //char dig[15]; 
   //sprintf(dig,"%d",sle7);
   //USART_puts(USART1, dig);
   //USART_puts(USART1,"\n");
   //sle7=0;
    TIM_ClearITPendingBit(TIM14, TIM_IT_CC1);
    capture = TIM_GetCapture1(TIM14);
    TIM_SetCompare1(TIM14, capture + 10);
  }
}
/*******************************************************************************
* Function Name  : CAN1_RX1_IRQHandler
* Description    : This function handles CAN RX1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void CAN1_RX0_IRQHandler(void)
{  //CanTxMsg TxMessage;
   CanRxMsg RxMessage;
   
   if(CAN_GetITStatus(CAN1, CAN_IT_FMP0)!= RESET){ // A message is pending in the FIFO 0
 
   CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
   if(sle7==0)
   {
   Toggle_Var_Stream_State(&ROV,RxMessage);
   sle7 = 1;
   }
   
   /*TxMessage.StdId   = 0x7eF;
   TxMessage.ExtId   = 0x01;
   TxMessage.RTR     = CAN_RTR_DATA;
   TxMessage.IDE     = CAN_ID_STD;
   TxMessage.DLC     = 3;
   TxMessage.Data[0] = (RxMessage.StdId >> 8) & 0xFF;
   TxMessage.Data[1] =  RxMessage.StdId       & 0xFF;
   TxMessage.Data[2] =  RxMessage.DLC;

   CAN_Transmit(CAN1, &TxMessage);*/
   
   //-------- THIS IS JUST FOR DEBUG -----------------------------------//
     f.integer[0] = ROV.Motion.Roll.integer[0];
     f.integer[1] = ROV.Motion.Roll.integer[1];
     f.integer[2] = ROV.Motion.Roll.integer[2];
     f.integer[3] = ROV.Motion.Roll.integer[3];
     s = conv_f2c(f.floating_number);
     USART_puts(USART1, s);
     Delay(0xFFFFF);
    USART_puts(USART1, "\n");
    Delay(0xFFFFF);
    //-----------------------------------------------------------------//
   CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0); }
}

/**
  * @brief  This function handles TIM10 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM10_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM10, TIM_IT_CC1) != RESET)
  {

    TIM_ClearITPendingBit(TIM10, TIM_IT_CC1);
  }
}

/**
  * @brief  This function handles TIM1 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM11_IRQHandler(void)
{
 

 
  
  if (TIM_GetITStatus(TIM11, TIM_IT_CC1) != RESET)
  {

    TIM_ClearITPendingBit(TIM11, TIM_IT_CC1);


  }
}

/**
  * @brief  This function handles TIM10 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM13_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM13, TIM_IT_CC1) != RESET)
  {

    TIM_ClearITPendingBit(TIM13, TIM_IT_CC1);


  }
}

/**
  * @brief  This function handles TIM10 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM14_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM14, TIM_IT_CC1) != RESET)
  {

    TIM_ClearITPendingBit(TIM14, TIM_IT_CC1);


  }
}
/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
