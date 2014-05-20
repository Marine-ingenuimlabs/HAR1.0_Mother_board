/******************** (C) COPYRIGHT 2013 ROV TUNISIA ********************
* File Name          : AIOPV1.h
* Author             : Rami HADJ TAIEB 
* Version            : V1.0
* Date               : 15/11/20
* Description        : This file contains the header of the AIOP communication handler
****************************************************************************/


#include <stdbool.h>
#include <stdio.h>

#define HEADER          0XFE
#define ID_50HZ         0XCA
#define ID_10HZ         0XCB

//@ ref Communication state for UART connected to AIOP

# define Message_Waitingfor_Header 0X01
# define Message_waitingfor_ID 0X02
# define Message_Waitingfor_Data 0X03
# define Message_Ready_for_Checking 0x04

//@ ref Communication message type for UART connected to AIOP

#define MSG_50HZ         0x0A
#define MSG_10HZ         0X0B

//@ ref  message state  comming from AIOP
#define VALID_DATA 0x00
#define INVALID_DATA 0x01
#define PROCESSING_DATA 0x02

//@ ref  frame state comming from AIOP
#define IDLE 0x00
#define DATA_PROCESSING 0x01
#define DATA_READY 0x02


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
  uint8_t state;
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
  uint8_t state;
}
AIOP_50HZMessage;

typedef struct
{
  AIOP_10HZMessage frame_10Hz;
  AIOP_50HZMessage frame_50Hz;
}AIO_buff;

typedef struct
{
  uint8_t Type;
  uint8_t State;// specify the state of the communication 
  uint8_t Trame_length; //current frame length
    
}aio_comm;

typedef struct
{ 
  aio_comm communication;
  AIO_buff buffers;
}measurment_unit;

/* Prototype of the function AIOP_init that initialises the communication with AIOP*/
void MeasurmentUnit_init(__IO measurment_unit *aio);

/* Prototype of the function to receive a char from the AIOP */
void AIOP_ReceiveChar(uint8_t AIOP_car_recu,__IO measurment_unit *aio);

/* Prototype of the cyclic redundancy check function */
uint16_t crc16(uint16_t crcval, uint8_t newchar);

