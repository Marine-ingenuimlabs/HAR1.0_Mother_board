/******************** (C) COPYRIGHT 2013 ROV TUNISIA ********************
* File Name          : Lighting_lib.h
* Author             : Rami HADJ TAIEB 
* Version            : V1.0
* Date               : 15/11/20
* Description        : This file contains all the functions prototypes for light control library
****************************************************************************/

typedef union _Data7
{
  uint16_t integer16;
  uint8_t integer8[2];
}union_light;


typedef struct 
{
  union_light left;
  union_light right;
  
}light_led;

/* Exported functions --------------------------------------------------------*/
void Lighting_update(light_led* light);
void Lighting_init(void);