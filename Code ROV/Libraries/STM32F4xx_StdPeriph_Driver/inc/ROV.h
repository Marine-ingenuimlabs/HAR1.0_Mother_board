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
#include "AIOPV1.h"
#include "Camera_Control.h"
#include "Timing_lib.h"
#include "pid_lib.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>


/* Private typedef -----------------------------------------------------------*/
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


//sensor structure
typedef struct
{
  uint8_t Status;
  uint8_t filter;
}Sensor;


typedef struct
{
  uint8_t* StrPtr;
  uint8_t State;
}Stream;


// 3D sensor structure
typedef struct 
{
  Sensor inertial;
  float value[3];
  float offesets[3];
  uint8_t trust; 
  
}Sensor_3D;

// IMU structure
typedef struct 
{
  Sensor_3D Accel;
  Sensor_3D Gyro;
  Sensor_3D Mag;
  
}IMU;

// analog sensor structure
typedef struct 
{ Sensor analog;
  float value;
}Sensor_analog;

//sensors structure
typedef struct  
{
  Sensor_analog Current; 
  Sensor_analog Voltage;
  Sensor_analog Onboard_Temprature;
  Sensor_analog Water_Temprature;
  Sensor_analog Pressure;
  IMU AHRS;
  uint8_t TrameReady;
  uint8_t EndofTrame;
  
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


typedef struct //this structure will hold all the motion variables( accelerometer, gyroscope, and depth ( from pressure sensor ) )
{
  union_float Depth;
  union_float Accelx;
  union_float Accely;
  union_float Accelz;
  union_float Gyrox;
  union_float Gyroy;
  union_float Gyroz;
  union_float Pitch;
  union_float Roll;
  union_float Yaw;

}MotionVar;



/* universal communication structure */ 
typedef struct
{
  union_16to8 x_axis; //valeur axe des x
  union_16to8 y_axis; //valeur axe des y
  union_16to8 rz_rotation; //valeur de la rotation autour de Z
  union_16to8 th_1; // valeur throttle 1
  union_16to8 th_2; //valeur throttle 2
  union_16to8 buttons; // value of the other buttons
    
}Joystick;

typedef struct
{
  uint8_t Type;
  uint8_t Status;
  uint8_t State;// specify the state of the communication 
  uint8_t Trame_length;
    
}Communication;
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

//ROV main structure
typedef struct 
{
  light_led light;
  
  //DOF DOF_Input;
  //DOF DOF_Last_Input;
  //DOF DOF_Output;
  //DOF DOF_Last_Output;
  uint8_t volatile State; //1rst bit:Stream State - 2nd bit: AIOP State
  MotionVar Motion;
  AllInOne volatile AIO;
  thruster Propulsion[6];
  Communication CAN;
  Communication RS485;
  Sensors Measurement_Unit;
  PelcoD_Message PelcoD;
  pid_controller PID[6];
  Joystick volatile joyst;
  Stream volatile CAN_Tab[256];
 
}ROV_Struct;

/* Initialising all the variables tables for stream and the functions callback table */
void ROV_VAR_Init(ROV_Struct* ROV);


/* ROV Initialisation ------------------------------------------------------- */
void ROV_Init(ROV_Struct* ROV,CanRxMsg RxMessage);  //Initializing the vehicule

void ROV_Routine(ROV_Struct* ROV);

void ROV_DataUpdate(ROV_Struct* ROV);





//Les fonctions suivantes sont utilisées pour le débogage
char* conv_f2c(float f); // convert from float to char
void USART_puts(USART_TypeDef* USARTx,__IO char *s); //print via serial port