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
extern void volatile (*function_pointer[16])(ROV_Struct*,CanRxMsg); // Functions Callback Table
extern ROV_Struct ROV;
extern __IO uint8_t UserButtonPressed;
/* Private variables ---------------------------------------------------------*/
uint16_t capture = 0;
union_float s;
CanRxMsg Rx;
CanTxMsg CTx;
uint8_t sle7 = 0;
union_thruster entier16;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
char *str;
//delay function
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
  
  //Sensor_DataUpdate_50Hz(&ROV.measurement_unit_sensors,&ROV.aio.buffers.frame_50Hz,&ROV.rov_state);
 // Sensor_DataUpdate_10Hz(&ROV.measurement_unit_sensors,&ROV.aio.buffers.frame_10Hz,&ROV.rov_state);
  
}
/*******************************************************************************
* Function Name  : TIM1_UP_TIM10_IRQHandler
* Description    : This function handles Timer 10 interrupt 
                   request which represent the 10 hz routine .
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART4_IRQHandler(void)
{
  if(USART_GetITStatus(UART4, USART_IT_RXNE) == SET)
  {
    if(ROV.rov_state.is_aiop_communication_allowed == 1)
    {
      
    AIOP_ReceiveChar((USART_ReceiveData(UART4) & 0xFF),(&(ROV.aio)));   
    
    
    }/* if communication with AIOP is enabled */
    USART_ClearITPendingBit(UART4, USART_IT_RXNE);
  }/* if USART4 interrupt has occurred */
}
/*******************************************************************************
* Function Name  : TIM1_UP_TIM10_IRQHandler
* Description    : This function handles Timer 10 interrupt 
                   request which represent the 10 hz routine .
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
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
/*******************************************************************************
* Function Name  : TIM1_UP_TIM10_IRQHandler
* Description    : This function handles Timer 10 interrupt 
                   request which represent the 10 hz routine .
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_UP_TIM10_IRQHandler(void)
{

  if (TIM_GetITStatus(TIM10, TIM_IT_CC1) != RESET)
  {
    if(((ROV.rov_state.is_streaming_enabled) == 1))
    { 
        ROV_Stream_VAR(ROV);
    } /*if streaming is enabled */
    Lighting_update(&ROV.light);
    //PelcoD_Send(&ROV.pelcod);// Pelco-d functionnality is ignored in this version
    TIM_ClearITPendingBit(TIM10, TIM_IT_CC1);
    capture = TIM_GetCapture1(TIM10);
    TIM_SetCompare1(TIM10, capture + 1000);
  }
}

/*******************************************************************************
* Function Name  : TIM1_TRG_COM_TIM11_IRQHandler
* Description    : This function handles Timer 11 interrupt 
                   request which represent the 50 hz routine .
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_TRG_COM_TIM11_IRQHandler(void) //at a frequency of 50hz
{   
  if (TIM_GetITStatus(TIM11, TIM_IT_CC1) != RESET)
  {
    //arm_mat_mult_f32(ROV.thruster_matrix
    /* here goes the place of the control algorithm */
    THRUSTER_update(ROV.propulsion);
      
    TIM_ClearITPendingBit(TIM11, TIM_IT_CC1);
    capture = TIM_GetCapture1(TIM11);
    TIM_SetCompare1(TIM11, capture + 200);
  }
}
/*******************************************************************************
* Function Name  : TIM8_UP_TIM13_IRQHandler
* Description    : This function handles Timer 13 interrupt 
                   request which represent the 100 hz routine .
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM8_UP_TIM13_IRQHandler(void)
{
  
  if (TIM_GetITStatus(TIM13, TIM_IT_CC1) != RESET)
  {
    
    TIM_ClearITPendingBit(TIM13, TIM_IT_CC1);
    capture = TIM_GetCapture1(TIM13);
    TIM_SetCompare1(TIM13, capture + 100);
  }
}
/*******************************************************************************
* Function Name  : TIM8_TRG_COM_TIM14_IRQHandler
* Description    : This function handles Timer 14 interrupt 
                   request which represent the 1 Khz routine .
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{

  if (TIM_GetITStatus(TIM14, TIM_IT_CC1) != RESET)
  {
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
{ 
  
   
   
   if(CAN_GetITStatus(CAN1, CAN_IT_FMP0)!= RESET){ // A message is pending in the FIFO 0
 
   CAN_Receive(CAN1, CAN_FIFO0, &Rx);
   
   
   if( ROV.propulsion[3].speed_command.integer16 == 0)
   {
     ROV.propulsion[3].speed_command.integer16 = 19999;
   }
   else
   {
     ROV.propulsion[3].speed_command.integer16 = 0;
   }
   /*ROV.propulsion[0].speed_command.integer16 = 0;
   ROV.propulsion[1].speed_command.integer16 = 0;
   ROV.propulsion[2].speed_command.integer16 = 0;
   ROV.propulsion[3].speed_command.integer16 = 0;
   */
   
   /*if(sle7<1){
     function_pointer[Rx.StdId](&ROV,Rx);
     sle7++;
   }
   
   s[0] = Rx.Data[0];
   s[1] = Rx.Data[1];
   s[2] = Rx.Data[2];
   s[3] = Rx.Data[3];
      */
   //ROV_Toggle_AIOP_Communication(&ROV,RxMessage);
   
   
   if(Rx.RTR == CAN_RTR_Data)
   {
     
     function_pointer[Rx.StdId](&ROV,Rx);
    
   }
   else
   {
     ROV_Req_Val(&ROV,Rx);
   }
   
     
   //-------- THIS IS JUST FOR DEBUG -----------------------------------//
     //f.integer[0] = ROV.Motion.Roll.integer[0];
     //f.integer[1] = ROV.Motion.Roll.integer[1];
     //f.integer[2] = ROV.Motion.Roll.integer[2];
     //f.integer[3] = ROV.Motion.Roll.integer[3];
     //s = conv_f2c(f.floating_number);
     //USART_puts(USART1, s);
     //Delay(0xFFFFF);
    //USART_puts(USART1,ROV.measurement_unit_sensors.AHRS.Euler_Angle.x_value.integer);
    //Delay(0xFFFFF);
  //  if(Rx.DLC<30){    
    // sprintf(str,"%d",Rx.Data);
   //}
  // else if(Rx.DLC<70){
     //sprintf(str,"%d",Rx.Data);
   //}
   //else{
 /*  s.integer[0] = Rx.Data[0];
   s.integer[1] = Rx.Data[1];
   s.integer[2] = Rx.Data[2];
   s.integer[3] = Rx.Data[3];
   str = conv_f2c(s.floating_number);
   //}
 USART_puts(USART1,str);
 USART_puts(USART1,"\n");
 */
   /*
 if((Rx.DLC%2)==0)
 {
   for(int x=0;x<(Rx.DLC);x=x+2)
   {
   entier16.integer8[0] = Rx.Data[x];
   entier16.integer8[1] = Rx.Data[x+1];
   USART_SendData(USART1,entier16.integer16);
   }}
   else
   {
   for(int x=0;x<(Rx.DLC-1);x=x+2)
   {
   entier16.integer8[0] = Rx.Data[x];
   entier16.integer8[1] = Rx.Data[x+1];
   USART_SendData(USART1,entier16.integer16);
   }
   entier16.integer8[0] = Rx.Data[Rx.DLC-1];
   entier16.integer8[1] = 0x00;
   USART_SendData(USART1,entier16.integer16);
   }*/
 /*
   GPIO_ToggleBits(GPIOD,12);
     
     char strx[20];
 // USART_puts( USART1, "JOYSTICK: ");
     for(int i=0;i<8;i++)
     {
   sprintf(strx,"%d  ",Rx.Data[i]);
    USART_puts( USART1, strx);
//   USART_puts( USART1,  " ");
     }
     USART_puts( USART1,  "\n");*/
 //  sprintf(strx,"X:%d Y:%d Rz:%d Slider:%d Pov:%d Buttons:%d %d %d %d %d %d \n",ROV_joy.X,ROV_joy.Y,ROV_joy.Rz,ROV_joy.slider, ROV_joy.pov,ROV_joy.button[0],ROV_joy.button[1],ROV_joy.button[2],ROV_joy.button[3],ROV_joy.button[4],ROV_joy.button[5]);

   
    //-----------------------------------------------------------------//
   CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0); }
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
/*
void EXTI0_IRQHandler(void)
{
  UserButtonPressed = 0x01;
  CTx.DLC = 4;
  CTx.Data[0] = 0x9A ;
  CTx.Data[1] = 0x99 ;
  CTx.Data[2] = 0xA9 ;
  CTx.Data[3] = 0x40;
  //CanTx.Data[4] = 0x40;

  CTx.IDE = CAN_Id_Extended;
  CTx.ExtId = 0x50;*/
  //CAN_Transmit(CAN1, &CTx);
  /* Clear the EXTI line pending bit */
  /*EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
}*/
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
