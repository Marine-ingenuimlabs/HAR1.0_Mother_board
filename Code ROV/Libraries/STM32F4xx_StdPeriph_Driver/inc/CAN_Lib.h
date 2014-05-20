/******************** (C) COPYRIGHT 2013 ROV TUNISIA ********************
* File Name          : CAN_Lib.h
* Author             : Rami HADJ TAIEB 
* Version            : V1.0
* Date               : 15/11/20
* Description        : This file contains all the functions prototypes for CAN communication library
****************************************************************************/

/* Local includes ------------------------------------------------------------*/
#include "stm32f4xx_can.h"
/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;




 
/* Private define ------------------------------------------------------------*/
  #define GPIO_CAN                   GPIOD
  #define RCC_APB2Periph_GPIO_CAN    RCC_APB2Periph_GPIOD
  #define GPIO_Pin_RX                GPIO_Pin_0
  #define GPIO_Pin_TX                GPIO_Pin_1
  #define GPIO_Remap_CAN             GPIO_Remap1_CAN1 
  #define CAN_Test                   1
  #define CAN_MAX_DATASIZE           8

/** @defgroup CAN_filter_scale 
  * @{
*/
/*This section of constant defining is reserved for CAN variables identifiers 
each amont of data have its unique ID with an extended ID used to specifie either its a request or a response
*/
//Identifiers of command messages
#define CAN_CMD_INIT 0x00
#define CAN_CMD_STREAM 0x01
#define CAN_CMD_AUTO 0x02
#define CAN_CMD_MAG 0x03
#define CAN_CMD_AIOP 0x04
#define CAN_CMD_THSET1234 0x05
#define CAN_CMD_THSET56 0x06
#define CAN_CMD_KP 0x0A
#define CAN_CMD_KD 0x0B
#define CAN_CMD_KI 0x0C

//Identifiers of down streaming messages
#define CAN_JOY_XYRZ 0x82
#define CAN_JOY_TH1TH2 0x83
#define CAN_JOY_BUTTON 0x84

//Identifiers of up streaming messages
#define CAN_DOF_ROLL 0x8C;
#define CAN_DOF_PITCH 0x8D;
#define CAN_DOF_YAW 0x8E;

//Identifiers of request messages
#define CAN_GYROX_GET 0x48
#define CAN_GYROY_GET 0x49
#define CAN_GYROZ_GET 0x4A
#define CAN_ACCX_GET 0x4B
#define CAN_ACCY_GET 0x4C
#define CAN_ACCZ_GET 0x4D

#define CAN_KP_GET 0x4F
#define CAN_KI_GET 0x50
#define CAN_KD_GET 0x51




//  Extended Identifier is used to identify requests and responses
  #define CAN_REQUEST_EXID                              0 // IDE = 0, then the ID is 11 bits.  
  #define CAN_RESPONSE_EXID                             1 // IDE = 1, then the ID is 29 bits.

// ACCEL ID and ACCEL offsets ID
  #define CAN_ACCELX_ID                                 
  #define CAN_ACCELY_ID
  #define CAN_ACCELZ_ID
  #define CAN_ACCELXOFF_ID
  #define CAN_ACCELYOFF_ID
  #define CAN_ACCELZOFF_ID

// Gyroscope ID and Gyroscope offsets ID
  #define CAN_GYROX_ID
  #define CAN_GYROY_ID
  #define CAN_GYROZ_ID
  #define CAN_GYROXOFF_ID
  #define CAN_GYROYOFF_ID
  #define CAN_GYROZOFF_ID

// Magnetometer ID and Magnetometer offsets ID
  #define CAN_MAGX_ID
  #define CAN_MAGY_ID
  #define CAN_MAGZ_ID
  #define CAN_MAGXOFF_ID
  #define CAN_MAGYOFF_ID
  #define CAN_MAGZOFF_ID

// Orientation torques ID 
  #define CAN_YAW_ID
  #define CAN_PITCH_ID
  #define CAN_ROLL_ID

// Temprature ID onboard and offboard
  #define CAN_WATERTEMP_ID
  #define CAN_HOUSINGTEMP_ID

// Depth data ID  
  #define CAN_PRESSURE_ID

// propulsion system 
  #define CAN_FRTHRUSTERS_ID // Data of the front right thruster 
  #define CAN_FLTHRUSTERS_ID // Data of the front left thruster
  #define CAN_RRTHRUSTERS_ID // Data of the rear right thruster 
  #define CAN_RLTHRUSTERS_ID // Data of the rear left thruster 
  #define CAN_URTHRUSTERS_ID // Data of the upper right thruster  
  #define CAN_ULTHRUSTERS_ID // Data of the upper left thruster 

// Control data 
  #define CAN_JoyX_ID //
  #define CAN_JoyY_ID //
  #define CAN_JoyZ_ID //
  #define CAN_JoyButton_ID //
  #define CAN_JOYTHROTR_ID // Right throttle 
  #define CAN_JOYTHROTL_ID // Left throttle
// power data ID  
  #define CAN_POWER_ID

// Real time 
  #define CAN_TIME_ID





// CAMERA data ID
/*
  #define CANx                       CAN1
  #define CAN_CLK                    RCC_APB1Periph_CAN1
  #define CAN_RX_PIN                 GPIO_Pin_0
  #define CAN_TX_PIN                 GPIO_Pin_1
  #define CAN_GPIO_PORT              GPIOD
  #define CAN_GPIO_CLK               RCC_AHB1Periph_GPIOD
  #define CAN_AF_PORT                GPIO_AF_CAN1
  #define CAN_RX_SOURCE              GPIO_PinSource0
  #define CAN_TX_SOURCE              GPIO_PinSource1 

*/

// CAN BAUD RATE SELECTION
#define   _125KB
//#define   _250KB
//#define  _500KB
//#define   _1000KB
/* Private macro -------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/* Initialisation of CAN communication ---------------------------------------*/
void CAN_init(void);

/* Transmission of default message ------------------------------------------ */
void CAN_MessageTransmit (void);

/* Send message via CAN ----------------------------------------------------- */
uint8_t CAN_send(uint8_t volatile *message, uint8_t messageid);


/* End of file ---------------------------------------------------------------*/
