/******************** (C) COPYRIGHT 2013 ROV TUNISIA ********************
* File Name          : Camera_control.c
* Author             : Rami HADJ TAIEB 
* Version            : V1.1
* Date               : 12/12/2013
* Description        : Controling Pan/Tilit Camera using Pelco-D protocol 
                       throught RS485 Communication
****************************************************************************/
/*Pelco D pan/tilt camera control using differential twisted shielded wire paire and MAX485 which is a RS-485 transceiver IC transforming TX/RX signal of USART

  
            MAX485
              ____
     nc --- 1|    |8 --- +5
     +5 --- 2|    |7 --- RS-485 wire(-)to cam
     +5 --- 3|    |6 --- RS-485 wire(+)to cam
     pin8 - 4|____|5 --- GND
*/
/* Pelco-D consists of 7 hexadecimal bytes as motionnned in this table:

 ---------------------------------------------------------------
| Byte 1 | Byte 2 | Byte 3  | Byte 4 | Byte 5 | Byte 6 | Byte 7 |
 ---------------------------------------------------------------
|  Sync	 | Camera  |Command |Command | Data   |	Data   |Checksum|
|        | Address |   1    |   2    |   1    |    2   |        |
 ---------------------------------------------------------------
Byte 1 -> Sync:  the synchronization byte, fixed to FF
Byte 2 -> Camera Address: logical address of the camera being controlled
Byte 5 -> Data 1: pan speed, range from 00 (stop) to 3F (high speed) and FF for "turbo" speed 
Byte 6 -> Data 2: tilt speed, range from 00 (stop) to 3F (maximum speed)
Byte 7 -> Checksum:  sum of bytes (excluding the synchronization byte), then modulo 100 (Decimal code: 256)

Command 1&2 are composed of 7 bits each one, which refers to:
 -----------------------------------------------------------------------------------
|           | Bit 7   |   Bit 6 |  Bit 5  |  Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0  |
 -----------------------------------------------------------------------------------
|Command 1  |  Sens   |Reserved |Reserved | Auto / |Camera | Iris  |Iris   | Focus  |
|           |         |         |         | Manual |On/Off | Close |Open   |  Near  |
|           |         |         |         |  Scan  |	   |	   |	   |	    |
 -----------------------------------------------------------------------------------
|Command 2  | Focus   |  Zoom   |  Zoom   |  Tilt  |  Tilt | Pan   | Pan   | Fixed  |
|           |  Far    |  Wide	|  Tele	  |  Down  |   Up  | Left  | Right | to 0   |
 -----------------------------------------------------------------------------------
There is some extra codes that can be usefull in our application as:
 ---------------------------------------------------
|                 |Byte 3 |Byte 4|Byte 5|Byte 6     |        
 ---------------------------------------------------
|  Go to Preset   |  00   |  07  |  00  | 01 to FF  |
 ---------------------------------------------------
|  Set Zoom Speed |  00   |  25  |  00	| 00 to 33  |
 ---------------------------------------------------
|  Set Focus Speed|  00   |  27  |  00  | 00 to 33  |
 ---------------------------------------------------
|   Alarm Ack     |  00   |  19  |  00  | 00 to 33  |
 ---------------------------------------------------

 --> Pelco-D cameras use No parity, 8 Data bits and 1 Stop bit, baud rate depends on your camera setting*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "Camera_Control.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define SYNC 0XFF
#define CAMERA_ADR 0X01
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t rString = 0x00;
uint8_t PelcoD_Counter;
uint8_t Cmd1, Cmd2;
uint8_t checksum;

/* Global variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
/*******************************************************************************
* Function Name  : PelcoD_Send
* Description    : Sending all the commands to control the cam in PelcoD
* Input          : bSend  true: set RS485 in Sending mode, else in Receiving mode 
*                  - 
* Output         : None
* Return         : None
*******************************************************************************/
void PelcoD_Send(PelcoD_Message* PelcoD){
  
    PelcoD_Counter=0;
    
    
    
    RS485Cmd(false,true); 
    Cmd1 = 0;
    Cmd1 += PelcoD->sense*0X80;
    Cmd1 += 0*0X40;
    Cmd1 += 0*0X20;
    Cmd1 += PelcoD->toggle_automan*0X10;
    Cmd1 += PelcoD->toggle_onoff*0X08;
    Cmd1 += PelcoD->iris_close*0X04;
    Cmd1 += PelcoD->iris_open*0X02;
    Cmd1 += PelcoD->focus_near*0X01;
    
    Cmd2 = 0;
    Cmd2 += PelcoD->focus_far*0X80;
    Cmd2 += PelcoD->zoom_wide*0X40;
    Cmd2 += PelcoD->zoom_tele*0X20;
    Cmd2 += PelcoD->tilt_down*0X10;
    Cmd2 += PelcoD->tilt_up*0X08;
    Cmd2 += PelcoD->pan_left*0X04;
    Cmd2 += PelcoD->pan_right*0X02;
    
    checksum = (CAMERA_ADR + Cmd1 + Cmd2 + PelcoD->pan_speed + PelcoD->tilt_speed) % 256;
    
    USART_SendData(USART2,SYNC);
   
}

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : SendMode
* Description    : 
* Input          : bSend  true: set GPIO_Pin_8 in Sending mode, else reset GPIO_Pin_8 in Receiving mode 
*                  - 
* Output         : None
* Return         : None
*******************************************************************************/
void SendMode(bool bSend)
{
  if(bSend)
  {
    GPIO_SetBits(GPIOC, GPIO_Pin_8);
  }
  else
  {
    GPIO_ResetBits(GPIOC, GPIO_Pin_8);  
  }
}

/*******************************************************************************
* Function Name  : RS485_Configuration
* Description    : Init USART2 using PA2, PA3 pins
*                  9600, 8, 1, none flow control
*                  DE/nRE using PA4
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RS485_init()
{ 
  
  /* USART6 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
 
  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  USART_DeInit(USART2);
    
  GPIO_InitTypeDef GPIO_InitStructure;
  /* Config DE/nRE */
  /* Configure (PA.04) as Output push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
 
  /* Connect USART pins to AF */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

  
  /*-------------------------- USART2 Configuration ----------------------------*/
    USART_InitTypeDef USART_InitStructure;
 
  /* USART2 configuration ------------------------------------------------------*/
  /* USART2 configured as follow:
        - BaudRate = 57600 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 57600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
 
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
  USART_Init(USART2, &USART_InitStructure);

   /* Enable the USART2 Interrupt vector*/
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the USART Transmoit interrupt: this interrupt is generated when the 
   USART1 transmit data register is empty */  
   //USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

  /* Enable the USART Receive interrupt: this interrupt is generated when the 
   USART1 receive data register is not empty */
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  
  /* Enable the USART2 */
  USART_Cmd(USART2, ENABLE);

  /* Send RS485 in Rece Mode */
  //SendMode(false);
  SendMode(true);
}

/*******************************************************************************
* Function Name  : RS485Cmd
* Description    : Control RS485 Tx and Rx
* Input          : bRxEnable  
*                  bTxEnable 
* Output         : None
* Return         : None
*******************************************************************************/
void RS485Cmd(bool bRxEnable, bool bTxEnable)
{
  if(bRxEnable)
  {
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
    {
    }
    
    SendMode(false); 
    USART_Cmd(USART2, ENABLE);
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  }
  else if(bTxEnable)
  {
    SendMode(true); 
    USART_Cmd(USART2, ENABLE);
    
    /* Force to Generate a TXE interrupt */
    /* USART2 transmit data register is empty */  
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
  }
  else
  {
    SendMode(false); 
    USART_Cmd(USART2, DISABLE);
  }
}

/*******************************************************************************
* Function Name  : PelcoD_Send
* Description    : Sending a command to control the cam in PelcoD
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RS485_Send(PelcoD_Message* PelcoD)
{ 
   PelcoD_Counter++;
  switch (PelcoD_Counter){
  case 0:
    USART_SendData(USART2,CAMERA_ADR);
      break;
  case 1:
    USART_SendData(USART2,Cmd1);
      break;
  case 2:
    USART_SendData(USART2,Cmd2);
      break;
  case 3:
    USART_SendData(USART2,PelcoD->pan_speed);
      break;
  case 4:
    USART_SendData(USART2,PelcoD->tilt_speed);
      break;
  case 5:
    USART_SendData(USART2,PelcoD->pan_speed);
    
      break;
  case 6: 
    USART_SendData(USART2,checksum);
    RS485Cmd(true,false);
    
      break;
    
  }
}


/* End of file ---------------------------------------------------------------*/