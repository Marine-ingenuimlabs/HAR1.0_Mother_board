/******************** (C) COPYRIGHT 2013 ROV TUNISIA ********************
* File Name          : Camera_Control.h
* Author             : Rami HADJ TAIEB 
* Version            : V1.1
* Date               : 15/11/20
* Description        : This file contains all the functions prototypes for the camera control library
****************************************************************************/
#include "stm32f4xx.h"
#include <stdbool.h>
typedef struct
{
  uint8_t sense;
  uint8_t toggle_automan;
  uint8_t toggle_onoff;
  uint8_t iris_close;
  uint8_t iris_open;
  uint8_t focus_near;
  uint8_t focus_far;
  uint8_t zoom_wide;
  uint8_t zoom_tele;
  uint8_t tilt_down;
  uint8_t tilt_up;
  uint8_t pan_left;
  uint8_t pan_right;
  uint8_t pan_speed;
  uint8_t tilt_speed;
}PelcoD_Message;

/* Exported functions --------------------------------------------------------*/
void PelcoD_Send(PelcoD_Message* PelcoD);
void SendMode(bool bSend);
void RS485_init();
void RS485Cmd(bool bRxEnable, bool bTxEnable);
void RS485_Send(PelcoD_Message* PelcoD);