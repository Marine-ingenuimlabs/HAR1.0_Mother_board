/******************** (C) COPYRIGHT 2013 ROV TUNISIA ********************
* File Name          : Lighting_lib.c
* Author             : Rami HADJ TAIEB 
* Version            : V1.0
* Date               : 09/04/2014
* Description        : ROV main functions library
****************************************************************************/

/* includes -----------------------------------------------------------*/
#include "stm32f4xx.h"
#include "Thrusters_lib.h"
#include "Lighting_lib.h"
#include "Analog_sensors_lib.h"
#include "CAN_Lib.h"
#include "measurment_unit.h"
#include "Camera_Control.h"
#include "Timing_lib.h"
#include "pid_lib.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>


/* Private typedef -----------------------------------------------------------*/

typedef struct
{
  uint8_t is_variable_in_use : 1;
  uint8_t is_aiop_communication_allowed : 1;
  uint8_t is_streaming_enabled : 1;
  uint8_t is_computer_connected : 1;
}state_of_rov;

typedef union _5Data
{
  float floating_number;
  uint8_t integer[4];
}union_float;

typedef union _Data4
{
  uint16_t integer16;
  uint8_t integer8[2];
}union_16to8;





// 3D sensor structure
typedef struct 
{
  union_float x_value,y_value,z_value;
  union_float x_offset,y_offset,z_offset;
}Sensor_3D_with_offset;
  
  typedef struct 
{
  union_float x_value,y_value,z_value;
}Sensor_3D_with_no_offset;

// IMU structure
typedef struct 
{
  Sensor_3D_with_no_offset Accel;
  Sensor_3D_with_no_offset Gyro;
  Sensor_3D_with_offset Mag;
  Sensor_3D_with_no_offset Euler_Angle;
}IMU;



//sensors structure
typedef struct  
{
  union_float Current; 
  union_float Voltage;
  union_float Onboard_Temprature;
  union_float Water_Temprature;
  union_float Pressure;
  IMU AHRS;
}Sensors;

typedef struct 
{ 
  // X rotation and translation      
  union_float Roll;
  union_float Surge;
  // Y rotation and translation      
  union_float Pitch;
  union_float Sway;
  // Z rotation and translation      
  union_float Yaw;
  union_float Heave; 
}DOF;


/* universal communication structure */ 
typedef struct
{
  union_16to8 x_axis; //valeur axe des x
  union_16to8 y_axis; //valeur axe des y
  union_16to8 rz_rotation; //valeur de la rotation autour de Z
  union_16to8 throttle_1; // valeur throttle 1
  union_16to8 throttle_2; //valeur throttle 2
  union_16to8 buttons; // value of the other buttons  
  union_16to8 pov;
}Joystick;

typedef struct
{
  uint8_t volatile *pointer;
  uint8_t volatile State;
}VariableID;

//ROV main structure
typedef struct 
{
  light_led light;
  measurment_unit volatile aio;
  thruster propulsion[6];
  Sensors measurement_unit_sensors;
  PelcoD_Message pelcod;
  pid_controller pid[6];
  Joystick joyst;
  VariableID volatile identifiers_table[256];
  state_of_rov rov_state;
  
}ROV_Struct;

/* Initialising all the variables tables for stream and the functions callback table */
void ROV_VAR_Init(ROV_Struct* ROV);


/* ROV Initialisation ------------------------------------------------------- */
void ROV_Init(ROV_Struct* ROV);
void ROV_Routine(ROV_Struct* ROV);
void Sensor_DataUpdate_10Hz(Sensors *destination,AIOP_10HZMessage volatile *source,state_of_rov *state);
void Sensor_DataUpdate_50Hz(Sensors *destination,AIOP_50HZMessage volatile *source,state_of_rov *state);
void update_pelcod_values(ROV_Struct* ROV,CanRxMsg CAN_Msg);
/*
void update_thrusters_values(ROV_Struct* ROV,CanRxMsg CAN_Msg);
void update_lighting_values(ROV_Struct* ROV,CanRxMsg CAN_Msg);
void update_joystick_values(ROV_Struct* ROV,CanRxMsg CAN_Msg);
*/
void communication_init(ROV_Struct* ROV,CanRxMsg RxMessage);



//Les fonctions suivantes sont utilisées pour le débogage
char* conv_f2c(float f); // convert from float to char
void USART_puts(USART_TypeDef* USARTx,__IO char *s); //print via serial port
void Delay(__IO uint32_t nCount);