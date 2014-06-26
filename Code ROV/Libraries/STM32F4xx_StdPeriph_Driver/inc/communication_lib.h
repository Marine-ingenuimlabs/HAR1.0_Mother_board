/*+ -------state in variable identifiers tables-------------+
  +-------------------------+-------------------+-----------+
  | BIT4       BIT3         |        BIT1       |   BIT0    |
  +-----------------------------------------+---------------+
  | Read/Write acces        | is_there_new_val? | Streaming?|
  |                         |                   |           |
  +-+---------------------+---------------------------------+
  | +---------------------+ |  0:NO             | 0:Off     |
  | |1| 0|is_being_written| |  1:YES            | 1:On      |
  | +---------------------+ |                   |           |
  | |0| 1|is_being_read   | |                   |           |
  | +---------------------+ |                   |           |
  | |0| 0|is_free         | |                   |           |
  | +---------------------+ |                   |           |
  +-+---------------------+-+-------------------+-----------+*/

/* includes -----------------------------------------------------------*/
#include "ROV.h"
#include "stm32f4_discovery.h"

/* private defines ---------------------------------------------------*/
//Write/Read acess
#define IS_WROTE_ON(X)  (((X&R_W_MASK)==0x08) ? (1):(0));
#define IS_READ_FROM(X)  (((X&R_W_MASK)==0x04) ? (1):(0));
#define IS_FREE(X)  (((X&R_W_MASK)==0x00) ? (1):(0));

//New value
#define IS_NEW_VAL(X)  (((X&NEW_OLD_MASK)==0x02) ? (1):(0));

//Streaming ON/OFF
#define IS_STREAMING(X)  (((X&STREAMING_MASK)==0x01) ? (1):(0));

//Masks for state in variable identifiers tables
#define STREAMING_MASK 0x01
#define NEW_OLD_MASK 0x02
#define R_W_MASK 0x0C

/* Toggle communication with AIOP via USART --------------------------------- */
void ROV_Toggle_AIOP_Communication(ROV_Struct* ROV,CanRxMsg RxMessage);

/* Toggle Stream State ------------------------------------------------------ */
void ROV_Toggle_Streaming_Mode(ROV_Struct* ROV,CanRxMsg RxMessage);

/* Initialization of callback function table -------------------------------- */
void FuncCallbackTable_INIT();

/* Streaming variables to command station via CAN --------------------------- */
void ROV_Stream_VAR(ROV_Struct ROV); 

/* Answering a request from command station --------------------------------- */
void ROV_Req_Val(ROV_Struct* ROV,CanRxMsg RxMessage); //envoie d'une valeur demandée par la station de commande

/* Setting a variable in ROV ------------------------------------------------ */
void ROV_Set_Var(ROV_Struct* ROV,CanRxMsg RxMessage);

/* Toggle one variable stream state ----------------------------------------- */
void Toggle_Var_Stream_State(ROV_Struct* ROV,CanRxMsg RxMessage);


