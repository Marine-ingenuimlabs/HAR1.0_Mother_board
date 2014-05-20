/******************** (C) COPYRIGHT 2013 ROV TUNISIA ********************
* File Name          : Timing_lib.c
* Author             : Rami HADJ TAIEB 
* Version            : V1.0
* Date of validation : 02/05/2014
* Description        : Communiacation with the measurement unit
****************************************************************************/
 
/* this code needs standard functions used by STM32F4xx.
 */
#include "stm32f4xx.h"
#include "measurment_unit.h"

uint8_t AIOP_Received_Char;
uint8_t Buffer[32];
//union_type union_message;
uint8_t UART_Counter=0;
uint8_t CRC_Count=0;

uint16_t crc_stm32=0XFFFF;//store serial data bytes
bool Flag_50HZ= false;
bool Flag_10HZ= false;

uint16_t crc_tab[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
  0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
  0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
  0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
  0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
  0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
  0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
  0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
  0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
  0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
  0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
  0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
  0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
  0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
  0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
  0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
  0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
  0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
  0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
  0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
  0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
  0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/*******************************************************************************
* Function Name  : AIOP_init
* Description    : Initialisation of communication with AIOP via USART
* Input          : AIOP structure
* Output         : None.
* Return         : None.
*******************************************************************************/
void MeasurmentUnit_init(__IO measurment_unit *aio){
  
  
  GPIO_InitTypeDef GPIO_InitStructure;

  NVIC_InitTypeDef NVIC_InitStructure;
  /* --------------------------- System Clocks Configuration -----------------*/
  /* UART4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
 
  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
 
  /* Connect USART pins to AF */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);
 
  /*-------------------------- UART4 Configuration ----------------------------*/
    USART_InitTypeDef USART_InitStructure;
 
  /* UARTx configuration ------------------------------------------------------*/
  /* UARTx configured as follow:
        - BaudRate = 38400 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmission mode enabled
  */
  USART_InitStructure.USART_BaudRate = 38400;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
 
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
  USART_Init(UART4, &USART_InitStructure);
 
  /*-------------------------- NVIC Configuration ----------------------------*/

  /* Enable UART4 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* Enable the USART2 Receive interrupt */
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
  USART_Cmd(UART4, ENABLE);

  aio->communication.State = Message_Waitingfor_Header;
} 

/*******************************************************************************
* Function Name  : AIOP_ReceiveChar
* Description    : Receive a message from AIOP via USART
* Input          : Received char, AIOP structure
* Output         : None.
* Return         : None.
*******************************************************************************/
void AIOP_ReceiveChar(uint8_t AIOP_car_recu, __IO measurment_unit *aio)
{ AIOP_Received_Char=AIOP_car_recu;
  //printf("  %d ,",AIOP_Received_Char); // displaying the received chatacter in the teminal I/O window

  switch(aio->communication.State)
  {
  case Message_Waitingfor_Header: //waiting for a new message    
    

        if (AIOP_Received_Char ==  HEADER )  
           {
             aio->communication.State= Message_waitingfor_ID;
             crc_stm32 = 0xFFFF;
             UART_Counter=0;
             aio->communication.Trame_length=0;
             crc_stm32= crc16(crc_stm32, HEADER);
             Flag_50HZ= false;
             Flag_10HZ= false;
           }/* if header received */
   break;
      
  case Message_waitingfor_ID: // waiting for identifier for the new message 
  
          
         if (AIOP_Received_Char == ID_10HZ && Flag_50HZ== false)
         {  
            if(aio->buffers.frame_10Hz.state == DATA_PROCESSING)
            { 
              aio->communication.State= Message_Waitingfor_Header;
              break;
            } /* if the buffer is being used already */
            aio->buffers.frame_10Hz.state = DATA_PROCESSING;
            Flag_10HZ= true;
            aio->communication.State = Message_Waitingfor_Data;
            aio->communication.Type= 0x0B;
            aio->communication.Trame_length = 24; 
            crc_stm32= crc16(crc_stm32, AIOP_Received_Char);
         }/* if a 10Hz trame is received */
         else if (AIOP_Received_Char == ID_50HZ && Flag_10HZ== false)
         {   
            if(aio->buffers.frame_50Hz.state == DATA_PROCESSING)
            { 
              aio->communication.State= Message_Waitingfor_Header;
              break;
            }/* if the buffer is being used already */
             aio->buffers.frame_50Hz.state = DATA_PROCESSING;
             Flag_50HZ= true;
             aio->communication.State = Message_Waitingfor_Data;
             aio->communication.Type = 0x0A;
             aio->communication.Trame_length = 36;
             crc_stm32= crc16(crc_stm32, AIOP_Received_Char);
         }/* if a 50Hz trame is received */
         else {
            aio->communication.State = Message_Waitingfor_Header;
         }/* if waiting for a new header */
  break;   
  
  case Message_Waitingfor_Data :// reception of the message
          
    if (UART_Counter<aio->communication.Trame_length)
          {crc_stm32= crc16(crc_stm32, AIOP_Received_Char);
            
            if (aio->communication.Type == MSG_50HZ)
              {   
                  switch (UART_Counter/4)
                  {
                  case 0 :
                   aio->buffers.frame_50Hz.Accelx.integer[UART_Counter%4]=AIOP_Received_Char; 
                   break;
                   
                  case 1:
                   aio->buffers.frame_50Hz.Accely.integer[UART_Counter%4]=AIOP_Received_Char;                  
                   break;
                   
                  case 2:
                   aio->buffers.frame_50Hz.Accelz.integer[UART_Counter%4]=AIOP_Received_Char;  
                  break;
                  
                  case 3:
                  aio->buffers.frame_50Hz.Gyrox.integer[UART_Counter%4]=AIOP_Received_Char; 
                  break;
                  
                  case 4:
                   aio->buffers.frame_50Hz.Gyroy.integer[UART_Counter%4]=AIOP_Received_Char; 
                 break;
                  
                  case 5:
                   aio->buffers.frame_50Hz.Gyroz.integer[UART_Counter%4]=AIOP_Received_Char; 
                 break;
                     
                  case 6:
                   aio->buffers.frame_50Hz.Pitch.integer[UART_Counter%4]=AIOP_Received_Char; 
                 break;
                 
                  case 7:
                   aio->buffers.frame_50Hz.Roll.integer[UART_Counter%4]=AIOP_Received_Char; 
                 break;
                 
                  case 8:
                   aio->buffers.frame_50Hz.Yaw.integer[UART_Counter%4]=AIOP_Received_Char;
                   if (UART_Counter%4==3)
                    {
                       aio->communication.State =Message_Ready_for_Checking;
                    }
                 break;
       
                  default: 
                break;
                  }
              
               UART_Counter++;  
              }/* if 50Hz trame received */

           else if (aio->communication.Type == MSG_10HZ)
              {     
                    switch (UART_Counter/4)
                    {
                    case 0:
                     aio->buffers.frame_10Hz.Current.integer[UART_Counter%4]=AIOP_Received_Char; 
                     break;
                    case 1:
                     aio->buffers.frame_10Hz.Voltage.integer[UART_Counter%4]=AIOP_Received_Char; 
                     break;
                    case 2:
                     aio->buffers.frame_10Hz.Onboard_Temprature.integer[UART_Counter%4]=AIOP_Received_Char; 
                     break;
                    case 3:
                     aio->buffers.frame_10Hz.Water_Temprature.integer[UART_Counter%4]=AIOP_Received_Char;
                     break;
                    case 4:
                     aio->buffers.frame_10Hz.Pressure.integer[UART_Counter%4]=AIOP_Received_Char; 
                     break;
                    case 5:
                     aio->buffers.frame_10Hz.Empty.integer[UART_Counter%4]=AIOP_Received_Char; 
                     if ((UART_Counter%4)==3)
                     {
                       aio->communication.State =Message_Ready_for_Checking;
                     }
                     break;
                     default: 
                     break;
                    }
              
            UART_Counter++; 
             
             }/* if 10Hz trame received */ 
          }  //end if counter
    break;  
          
  case Message_Ready_for_Checking:
    if (aio->communication.Type == MSG_10HZ){
         aio->buffers.frame_10Hz.Crc.integer8[(UART_Counter+1)%2]=AIOP_Received_Char; 
            UART_Counter++;
            if(UART_Counter==aio->communication.Trame_length+2){
              aio->communication.State = Message_Waitingfor_Header;
              if(aio->buffers.frame_10Hz.Crc.integer16==crc_stm32)
                { 
                  aio->buffers.frame_10Hz.state = DATA_READY;
                }/*if data is valid */
              else
                {
                  aio->buffers.frame_10Hz.state = IDLE;
                }/* if data is not valid */
            }
    }/*if a 10Hz message is received */
    else if (aio->communication.Type == MSG_50HZ){
            aio->buffers.frame_50Hz.Crc.integer8[(UART_Counter+1)%2]=AIOP_Received_Char; 
            UART_Counter++;
            if(UART_Counter==aio->communication.Trame_length+2)
            {
              aio->communication.State = Message_Waitingfor_Header;  
              if(aio->buffers.frame_50Hz.Crc.integer16==crc_stm32)
                {
                  aio->buffers.frame_50Hz.state = DATA_READY;
                }/*if data is valid */
              else
                {
                  aio->buffers.frame_50Hz.state = IDLE;
                }/* if data is not valid */
            }
    }/*if a 10Hz message is received */
     
   break;

  default:break;
  } //end of switch

  

}       

/*******************************************************************************
* Function Name  : crc16
* Description    : CRC16 Calculation
* Input          : Old CRC Val, new char 
* Output         : None.
* Return         : New CRC value.
*******************************************************************************/
uint16_t crc16(uint16_t crcval, uint8_t newchar)
  { uint16_t newcrc;
    newcrc = (crcval >> 8) ^ crc_tab[(crcval ^ newchar) & 0x00ff];
    CRC_Count++;
    return newcrc;
  }


/* End of file ---------------------------------------------------------------*/