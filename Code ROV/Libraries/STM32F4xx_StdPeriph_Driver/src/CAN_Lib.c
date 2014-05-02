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
  RCC_AHB1PeriphClockCmd(CAN_GPIO_CLK, ENABLE);

  /* Connect CAN pins to AF9 */
  GPIO_PinAFConfig(CAN_GPIO_PORT, CAN_RX_SOURCE, CAN_AF_PORT);
  GPIO_PinAFConfig(CAN_GPIO_PORT, CAN_TX_SOURCE, CAN_AF_PORT); 
  
  /* Configure CAN RX and TX pins */
  GPIO_InitStructure.GPIO_Pin = CAN_RX_PIN | CAN_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(CAN_GPIO_PORT, &GPIO_InitStructure);
  
  NVIC_InitTypeDef  NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 

  /* CAN configuration ********************************************************/  
  /* Enable CAN clock */
  RCC_APB1PeriphClockCmd(CAN_CLK, ENABLE);
  
  /* CAN register init */
  CAN_DeInit(CANx);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack; //CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    
  //CAN Baudrate = 125 KBps// (APB1 clocked at 42 Mhz = CANclock) 
  CAN_InitStructure.CAN_BS1 = CAN_BS1_8tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_6tq;
  CAN_InitStructure.CAN_Prescaler = 24;
  CAN_Init(CAN1, &CAN_InitStructure); 

  /* CAN Baudrate = 1 MBps (CAN clocked at 30 MHz) */
/*CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
  CAN_InitStructure.CAN_Prescaler = 2;
  CAN_Init(CANx, &CAN_InitStructure);
*/
  /* CAN filter init */
  CAN_FilterInitStructure.CAN_FilterNumber = 0;
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
  CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);  
}

/*******************************************************************************
* Function Name  : CAN_MessageTransmit
* Description    : Sending default CAN message 
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
  void CAN_MessageTransmit(void){
  // Transmit message initialized
        uint8_t  TransmitMailbox;   
  	TxMessage.StdId=0x11;
 	TxMessage.RTR=CAN_RTR_DATA;
  	TxMessage.IDE=CAN_ID_STD;
  	TxMessage.DLC=2;
  	TxMessage.Data[0]=0xCA;
  	TxMessage.Data[1]=0xFE;
	
  	TransmitMailbox=CAN_Transmit(CAN1, &TxMessage);
        CAN_Error_Code= CAN_GetLastErrorCode (CAN1);
  	uint8_t i = 0;
  	while((CAN_TransmitStatus(CAN1, TransmitMailbox) != CANTXOK) && (i != 0xFF))
  	{
   	 i++;
 	 }
  }   
  
/*******************************************************************************
* Function Name  : CAN_send
* Description    : Sending default CAN message 
* Input          : Message and message ID
* Output         : None.
* Return         : Error code.
*******************************************************************************/
uint8_t CAN_send(uint8_t *message, uint8_t messageid)
  { CanTxMsg CAN_TxMessage;
    
    uint8_t message_length ,TransmitMailbox, CAN_Error_Code, counter=0;  
    message_length=strlen((char*)message);
    if (message_length > CAN_MAX_DATASIZE)
    {
      return 1;// Error: Message size exceeds the datafield
    }
    CAN_TxMessage.StdId   = 0x00;
    CAN_TxMessage.ExtId   = messageid;
    CAN_TxMessage.RTR     = CAN_RTR_DATA;
    CAN_TxMessage.IDE     = CAN_ID_EXT;
    CAN_TxMessage.DLC     = message_length;
    memcpy(CAN_TxMessage.Data , message ,message_length ) ;  
    //CAN_Transmit(CAN1, &CAN_TxMessage);
    TransmitMailbox=CAN_Transmit(CAN1, &CAN_TxMessage);
        
  	while((CAN_TransmitStatus(CAN1, TransmitMailbox) != CANTXOK) && (counter != 0xFF))
  	{counter++;
         
         //Save in the error code attached to RTC value
 	 }
        if (counter == 0xFF)
          CAN_Error_Code= CAN_GetLastErrorCode (CAN1);
        
        return CAN_Error_Code;
  }

