/* includes -----------------------------------------------------------*/
#include "ROV.h"



/* Streaming variables to command station via CAN --------------------------- */
void ROV_Stream_VAR(ROV_Struct ROV); 

/* Answering a request from command station --------------------------------- */
void ROV_Req_Val(ROV_Struct ROV,CanRxMsg RxMessage); //envoie d'une valeur demandée par la station de commande

/* Toggle Stream State ------------------------------------------------------ */
void ROV_Toggle_Streaming_Mode(ROV_Struct* ROV,CanRxMsg RxMessage);

/* Toggle communication with AIOP via USART --------------------------------- */
void ROV_Toggle_AIOP_Communication(ROV_Struct* ROV,CanRxMsg RxMessage);

/* Setting a variable in ROV ------------------------------------------------ */
void ROV_Set_Var(ROV_Struct* ROV,CanRxMsg CAN_Msg);

/* Toggle one variable stream state ----------------------------------------- */
void Toggle_Var_Stream_State(ROV_Struct* ROV,CanRxMsg RxMessage);