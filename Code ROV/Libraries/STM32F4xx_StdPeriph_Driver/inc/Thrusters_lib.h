/******************** (C) COPYRIGHT 2013 ROV TUNISIA ********************
* File Name          : Thrusters_lib.h
* Author             : Rami HADJ TAIEB 
* Version            : V1.0
* Date               : 15/11/20
* Description        : This file contains all the functions prototypes for thrusters control library
****************************************************************************/

// propulsion system data strusture 
typedef union _Data2
{
  uint16_t integer16;
  uint8_t integer8[2];
}union_thruster;

typedef struct 
{
  uint16_t status;
  union_thruster volatile speed_command;
  union_thruster speed_feedback;
  uint8_t efficiency;
  
}thruster;

/* Exported functions --------------------------------------------------------*/
void THRUSTER_update(thruster *propulsion);
void THRUSTER_init(thruster *propulsion);

