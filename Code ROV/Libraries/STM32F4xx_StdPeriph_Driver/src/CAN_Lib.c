/******************** (C) COPYRIGHT 2013 ROV TUNISIA ********************
* File Name          : CAN_Lib.c
* Author             : Rami HADJ TAIEB 
* Version            : V1.0
* Date               : 28/10/20
* Description        : Use of the CAN communication bus
****************************************************************************/

/* this code needs standard functions used by STM32F4xx.
 */
   
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "CAN_Lib.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
CanRxMsg RxMessage;
CanTxMsg TxMessage;
uint8_t CAN_Error_Code;
#define CAN_125KB  //CAN Communication Speed
uint8_t message_length,TransmitMailbox, CAN_Error_Code, counter=0;
//uint8_t copy_cnt = 0;
/*******************************************************************************
* Function Name  : CAN_Configuration
* Description    : Initialisation of the CAN bus communication 
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void CAN_init(void)
{  
  CAN_InitTypeDef CAN_InitStructure;
  CAN_FilterInitTypeDef CAN_FilterInitStructure;
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* CAN GPIOs configuration **************************************************/

  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
  /* Connect CAN pins to AF */
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1); 
  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2);
  
  /* Configure CAN RX and TX pins for CAN1*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  /* Configure CAN RX and TX pins for CAN2 (used for debug)*/
  /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  */
  NVIC_InitTypeDef  NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn; //Use CAN2 in case of just sending informations from CAN1 to CAN2 on the same board
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 

  /* CAN configuration ********************************************************/  
  /* Enable CAN clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
  
  /* CAN register init */
  CAN_DeInit(CAN1);
  CAN_DeInit(CAN2);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;//CAN_Mode_Normal;//CAN_Mode_LoopBack;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    
  //CAN Baudrate = 125 KBps// (APB1 clocked at 42 Mhz = CANclock) 
  CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_6tq;
  CAN_InitStructure.CAN_Prescaler = 21;
  CAN_Init(CAN1, &CAN_InitStructure); 
  CAN_Init(CAN2, &CAN_InitStructure); 

  /* CAN Baudrate = 1 MBps (CAN clocked at 30 MHz) */
/*CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
  CAN_InitStructure.CAN_Prescaler = 2;
  CAN_Init(CANx, &CAN_InitStructure);
*/
  /* CAN filter init */
  CAN_FilterInitStructure.CAN_FilterNumber = 0; //0 if we are using CAN1 for reception, 18 if we are using CAN2 for reception
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
  
  /* Transmit Structure preparation */
  TxMessage.StdId = 0x321;
  TxMessage.ExtId = 0x01;
  TxMessage.RTR = CAN_RTR_DATA;
  TxMessage.IDE = CAN_ID_STD;
  TxMessage.DLC = 1;
  
  /* Enable FIFO 0 message pending Interrupt */
  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);  //Specify which CAN will the data be sent to
}

/*******************************************************************************
* Function Name  : CAN_send
* Description    : Sending default CAN message 
* Input          : Message and message ID
* Output         : None.
* Return         : Error code.
*******************************************************************************/
uint8_t CAN_send(uint8_t volatile *message, uint8_t messageid)
  { CanTxMsg CAN_TxMessage;
    
   if(messageid<30){
     message_length = 1;
   }
   else if(messageid<70){
     message_length = 2;
   }
   else{
     message_length = 4;
   }

    if (message_length > CAN_MAX_DATASIZE)
    {
      return 1;// Error: Message size exceeds the datafield
    }
    CAN_TxMessage.StdId   = 0x00;
    CAN_TxMessage.ExtId   = messageid;
    CAN_TxMessage.RTR     = CAN_RTR_DATA;
    CAN_TxMessage.IDE     = CAN_ID_EXT;
    CAN_TxMessage.DLC     = message_length;
    for(int cnt=0;cnt<message_length;cnt++)
    {
      CAN_TxMessage.Data[cnt] = message[cnt];
    }
    //memcpy(CAN_TxMessage.Data , message ,message_length ) ;  
    TransmitMailbox=CAN_Transmit(CAN1, &CAN_TxMessage);
        
  	while((CAN_TransmitStatus(CAN1, TransmitMailbox) != CANTXOK) && (counter != 0xFF))
  	{counter++;
         
         //Save in the error code attached to RTC value
 	 }
        if (counter == 0xFF)
          CAN_Error_Code= CAN_GetLastErrorCode (CAN1);
        
        return CAN_Error_Code;
  }



/* End of file ---------------------------------------------------------------*/