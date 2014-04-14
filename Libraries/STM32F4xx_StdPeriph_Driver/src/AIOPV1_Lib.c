#include "stm32f4xx.h"
#include "AIOPV1.h"

#define Header          0XFE
#define ID_50Hz         0XCA
#define ID_10Hz         0XCB

uint8_t AIOP_Received_Char;
uint8_t Buffer[32];
//union_type union_message;
uint8_t UART_Counter=0;
uint8_t CRC_Count=0;
uint16_t crctest =0XFFFF;
uint16_t crc_stm32=0XFFFF;//store serial data bytes
bool Flag_50HZ= false;
bool Flag_10HZ= false;

//CRC 16 
uint8_t crc_test[38]={ 
  254 ,  202  , 102 ,  6 ,  122 ,  68 ,  205 ,  12 ,  122 ,  68 ,
  51 ,  19 ,  122 ,  68 ,  154 ,  25 ,  122 , 68 , 0 , 32 , 122 ,  
  68 ,  102 ,  38 ,  122 ,  68 , 205 , 44 , 122 ,  68 , 51 , 51 , 
  122 ,  68 ,  154 ,  57 ,  122 ,  68  } ;

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
void AIOP_init(__IO AllInOne* AIO){
  
  for (uint8_t i = 0; i < 38; i++)
     crctest = crc16(crctest,crc_test[i] );
  
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
        - BaudRate = 57600 baud
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
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* Enable the USART2 Receive interrupt */
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
  USART_Cmd(UART4, ENABLE);
  AIO->communication.State = Message_Waitingfor_Header;
} 

/*******************************************************************************
* Function Name  : AIOP_ReceiveChar
* Description    : Receive a message from AIOP via USART
* Input          : Received char, AIOP structure
* Output         : None.
* Return         : None.
*******************************************************************************/
void AIOP_ReceiveChar(uint8_t AIOP_car_recu, __IO AllInOne* AIO)
{ AIOP_Received_Char=AIOP_car_recu;
  //printf("  %d ,",AIOP_Received_Char); // displaying the received chatacter in the teminal I/O window
  switch(AIO->communication.State)
  {
  case Message_Waitingfor_Header: //waiting for a new message    
    

        if (AIOP_Received_Char ==  0xFE )  
           {
             AIO->communication.State= Message_waitingfor_ID;
             crc_stm32 = 0xFFFF;
             UART_Counter=0;
             AIO->communication.Trame_length=0;
             crc_stm32= crc16(crc_stm32, 0xFE);
             Flag_50HZ= false;
             Flag_10HZ= false;
           }
   break;
      
  case Message_waitingfor_ID: // waiting for identifier for the new message 
  
          
         if (AIOP_Received_Char == 0xCB && Flag_50HZ== false)
         {  
            Flag_10HZ= true;
            AIO->buffers.Analog_MeasurementDATA_READY=0x02;//waiting for the end of the message 
            AIO->communication.State = Message_Waitingfor_Data;
            AIO->communication.Type= 0x0B;
            AIO->communication.Trame_length = 24; 
            crc_stm32= crc16(crc_stm32, AIOP_Received_Char);
         }else if (AIOP_Received_Char == 0xCA && Flag_10HZ== false)
         { 
             Flag_50HZ= true;
             AIO->buffers.Inertial_MeasurementDATA_READY=0x02;// waiting for the end of the reception 
             AIO->communication.State = Message_Waitingfor_Data;
             AIO->communication.Type = 0x0A;
             AIO->communication.Trame_length = 36;
             crc_stm32= crc16(crc_stm32, AIOP_Received_Char);
         } else {
            AIO->communication.State = Message_Waitingfor_Header;
         }
  break;   
  
  case Message_Waitingfor_Data :// reception of the message
          
    if (UART_Counter<AIO->communication.Trame_length)
          {crc_stm32= crc16(crc_stm32, AIOP_Received_Char);
            
            if (AIO->communication.Type == 0x0A)
              {   
                  switch (UART_Counter/4)
                  {
                  case 0 :
                   AIO->buffers.Current_50HZtrame.Accelx.integer[UART_Counter%4]=AIOP_Received_Char; 
                   break;
                   
                  case 1:
                   AIO->buffers.Current_50HZtrame.Accely.integer[UART_Counter%4]=AIOP_Received_Char;                  
                   break;
                   
                  case 2:
                   AIO->buffers.Current_50HZtrame.Accelz.integer[UART_Counter%4]=AIOP_Received_Char;  
                  break;
                  
                  case 3:
                  AIO->buffers.Current_50HZtrame.Gyrox.integer[UART_Counter%4]=AIOP_Received_Char; 
                  break;
                  
                  case 4:
                   AIO->buffers.Current_50HZtrame.Gyroy.integer[UART_Counter%4]=AIOP_Received_Char; 
                 break;
                  
                  case 5:
                   AIO->buffers.Current_50HZtrame.Gyroz.integer[UART_Counter%4]=AIOP_Received_Char; 
                 break;
                     
                  case 6:
                   AIO->buffers.Current_50HZtrame.Pitch.integer[UART_Counter%4]=AIOP_Received_Char; 
                 break;
                 
                  case 7:
                   AIO->buffers.Current_50HZtrame.Roll.integer[UART_Counter%4]=AIOP_Received_Char; 
                 break;
                 
                  case 8:
                   AIO->buffers.Current_50HZtrame.Yaw.integer[UART_Counter%4]=AIOP_Received_Char;
                   if (UART_Counter%4==3)
                    {
                       AIO->communication.State =Message_Ready_for_Checking;
                    }
                 break;
       
                  default: 
                break;
                  }
              
               UART_Counter++;  
              }

           else if (AIO->communication.Type == 0x0B)
              {     
                    switch (UART_Counter/4)
                    {
                    case 0:
                     AIO->buffers.Current_10HZtrame.Current.integer[UART_Counter%4]=AIOP_Received_Char; 
                     break;
                    case 1:
                     AIO->buffers.Current_10HZtrame.Voltage.integer[UART_Counter%4]=AIOP_Received_Char; 
                     break;
                    case 2:
                     AIO->buffers.Current_10HZtrame.Onboard_Temprature.integer[UART_Counter%4]=AIOP_Received_Char; 
                     break;
                    case 3:
                     AIO->buffers.Current_10HZtrame.Water_Temprature.integer[UART_Counter%4]=AIOP_Received_Char;
                     break;
                    case 4:
                     AIO->buffers.Current_10HZtrame.Pressure.integer[UART_Counter%4]=AIOP_Received_Char; 
                     break;
                    case 5:
                     AIO->buffers.Current_10HZtrame.Empty.integer[UART_Counter%4]=AIOP_Received_Char; 
                     if ((UART_Counter%4)==3)
                     {
                       AIO->communication.State =Message_Ready_for_Checking;
                     }
                     break;
                     default: 
                     break;
                    }
              
            UART_Counter++; 
             
             } 
          }  //end if counter
    break;  
          
  case Message_Ready_for_Checking:
    if (AIO->communication.Type == MSG_10Hz){
         AIO->buffers.Current_10HZtrame.Crc.integer8[UART_Counter%2]=AIOP_Received_Char; 
            UART_Counter++;
            if(UART_Counter==AIO->communication.Trame_length+2){
              AIO->communication.State = Message_Waitingfor_Header;
              if(AIO->buffers.Current_10HZtrame.Crc.integer16==crc_stm32)
                { 
                  AIO->buffers.Analog_MeasurementDATA_READY=VALID_DATA; //OK
                }
              else
                {
                  AIO->buffers.Analog_MeasurementDATA_READY=INVALID_DATA;//NOK
                }
            }
    }
    else if (AIO->communication.Type == MSG_50Hz){
            AIO->buffers.Current_50HZtrame.Crc.integer8[UART_Counter%2]=AIOP_Received_Char; 
            UART_Counter++;
            if(UART_Counter==AIO->communication.Trame_length+2)
            {
              AIO->communication.State = Message_Waitingfor_Header;  
              if(AIO->buffers.Current_50HZtrame.Crc.integer16==crc_stm32)
                {
                  AIO->buffers.Inertial_MeasurementDATA_READY=VALID_DATA; //OK
                }
              else
                {
                  AIO->buffers.Inertial_MeasurementDATA_READY=INVALID_DATA;//NOK
                }
            }
    }
     
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
