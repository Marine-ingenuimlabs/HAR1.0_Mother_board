/******************** (C) COPYRIGHT 2013 ROV TUNISIA ********************
* File Name          : AIOPV1.h
* Author             : Rami HADJ TAIEB 
* Version            : V1.0
* Date               : 15/11/20
* Description        : This file contains the header of the AIOP communication handler
****************************************************************************/


#include <stdbool.h>
#include <stdio.h>

#define Header          0XFE
#define ID_50Hz         0XCA
#define ID_10Hz         0XCB

//@ ref Communication state for UART connected to AIOP

# define Message_Waitingfor_Header 0X01
# define Message_waitingfor_ID 0X02
# define Message_Waitingfor_Data 0X03
# define Message_Ready_for_Checking 0x04

//@ ref Communication message type for UART connected to AIOP

#define MSG_50Hz         0x0A
#define MSG_10Hz         0X0B

//@ ref  message state  comming from AIOP
#define VALID_DATA 0x00
#define INVALID_DATA 0x01
#define PROCESSING_DATA 0x02

typedef union _Data
{
  float floating_number;
  uint8_t integer[4];
}union_Data;

typedef union _Data1
{
  uint16_t integer16;
  uint8_t integer8[2];
}union_int16_to_int8;


typedef struct
{ union_Data Current; 
  union_Data Voltage;
  union_Data Onboard_Temprature;
  union_Data Water_Temprature;
  union_Data Pressure;
  union_Data Empty;
  union_int16_to_int8 Crc;
}
AIOP_10HZMessage;
typedef struct
{ 
  union_Data Accelx;
  union_Data Accely;
  union_Data Accelz;
  union_Data Gyrox;
  union_Data Gyroy;
  union_Data Gyroz;
  union_Data Pitch;
  union_Data Roll;
  union_Data Yaw;
  union_int16_to_int8 Crc;
}
AIOP_50HZMessage;

typedef struct
{
  //uint8_t CAN_TX_Buffer[8];
  //uint8_t CAN_RX_Buffer[8];
  //PelcoD_Message PelcoD;
  bool Analog_MeasurementDATA_READY;
  bool Inertial_MeasurementDATA_READY;
  AIOP_10HZMessage Current_10HZtrame, Last_10HZTrame, Last_Saved_10HZTrame;
  AIOP_50HZMessage Current_50HZtrame, Last_50HZTrame, Last_Saved_50HZTrame;
}AIO_buff;

typedef struct
{
  uint8_t Type;
  uint8_t Status;
  uint8_t State;// specify the state of the communication 
  uint8_t Trame_length;
    
}aio_comm;

typedef struct
{
  aio_comm communication;
  AIO_buff buffers;
}AllInOne;

/* Prototype of the function AIOP_init that initialises the communication with AIOP*/
void AIOP_init(__IO AllInOne* AIO);

/* Prototype of the function to receive a char from the AIOP */
void AIOP_ReceiveChar(uint8_t AIOP_car_recu,__IO AllInOne* AIO);

/* Prototype of the cyclic redundancy check function */
uint16_t crc16(uint16_t crcval, uint8_t newchar);

