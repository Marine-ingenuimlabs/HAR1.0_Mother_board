
/* includes -----------------------------------------------------------*/
#include "communication_lib.h"


void (*ptr[16])(ROV_Struct*,CanRxMsg); // Functions Callback Table
int i=0;
uint8_t error_can;


/*******************************************************************************
* Function Name  : ROV_Set_Var
* Description    : Set variable in ROV
* Input          : ROV Structure, Received message from CAN
* Output         : None.
* Return         : None.
*******************************************************************************/
void ROV_Set_Var(ROV_Struct* ROV,CanRxMsg CAN_Msg)
{

 for(int i = 0;i<(CAN_Msg.DLC-1);i++)
 {
   ROV->CAN_Tab[CAN_Msg.Data[0]].StrPtr[i] = CAN_Msg.Data[i+1];
 }

}

/*******************************************************************************
* Function Name  : Toggle_Var_Stream_State
* Description    : Allow or forbid one single variable from stream
* Input          : ROV Structure, Received message from CAN
* Output         : None.
* Return         : None.
*******************************************************************************/
void Toggle_Var_Stream_State(ROV_Struct* ROV,CanRxMsg RxMessage)
{
  ROV->CAN_Tab[RxMessage.Data[0]].State = (ROV->CAN_Tab[RxMessage.Data[0]].State)||(RxMessage.Data[1]&0x01);
  ROV->CAN_Tab[RxMessage.Data[0]].State = (ROV->CAN_Tab[RxMessage.Data[0]].State)&&(RxMessage.Data[1]&0x01);
}

/*******************************************************************************
* Function Name  : ROV_Req_Val
* Description    : Send a variable value once
* Input          : ROV Structure, Received message from CAN
* Output         : None.
* Return         : None.
*******************************************************************************/
void ROV_Req_Val(ROV_Struct ROV,CanRxMsg RxMessage)
{
  error_can = CAN_send(ROV.CAN_Tab[RxMessage.ExtId-128].StrPtr,RxMessage.ExtId); //Send Requested Value Referenced in Can_Tab in Rov_Struct
}


/*******************************************************************************
* Function Name  : ROV_Stream_VAR
* Description    : Stream variables via CAN
* Input          : ROV Structure containing all variables to stream
* Output         : None.
* Return         : None.
*******************************************************************************/
void ROV_Stream_VAR(ROV_Struct ROV)
{ 
  for(int o = 0;o<256;o++) //for all the variables
  {
    if(ROV.CAN_Tab[o].State ==1) // Check if the variable is allowed for stream
    {  
      error_can = CAN_send((ROV.CAN_Tab[o].StrPtr),o); //Send
    }
}
}


/*******************************************************************************
* Function Name  : ROV_Toggle_Stream
* Description    : Enable or disable streaming to command station
* Input          : ROV Structure, Received message from CAN
* Output         : None.
* Return         : None.
*******************************************************************************/
void ROV_Toggle_Streaming_Mode(ROV_Struct* ROV,CanRxMsg RxMessage)
{
  (ROV->State) = (ROV->State)||(RxMessage.Data[0]&0x01);
  (ROV->State) = (ROV->State)&&(RxMessage.Data[0]&0x01);
  
}

/*******************************************************************************
* Function Name  : ROV_Toggle_AIOP
* Description    : Enable or disable communication with AIOP via USART
* Input          : ROV Structure, Received message from CAN
* Output         : None.
* Return         : None.
*******************************************************************************/
void ROV_Toggle_AIOP_Communication(ROV_Struct* ROV,CanRxMsg RxMessage)
{
  (ROV->State) = (RxMessage.Data[0]&0x02);
  
}

/*******************************************************************************
* Function Name  : FuncCallbackTable_INIT
* Description    : Initialising functions' pointers table for calling
* Input          : ROV Structure, Received message from CAN
* Output         : None.
* Return         : None.
*******************************************************************************/
void FuncCallbackTable_INIT(ROV_Struct* ROV)
{
  
 ptr[0] = ROV_Set_Var;
 ptr[1] = ROV_Init;
 ptr[2] = ROV_Toggle_Streaming_Mode;
 //ptr[3] = Toggle_AutoControl;
 //ptr[4] = Toggle_Magneto;
 ptr[5] = ROV_Toggle_AIOP_Communication;
 //ptr[6] = Calib_IMU;                  //Still Not Available
 //ptr[7] = Calib_Magneto;              //Still Not Available
 //ptr[8] = Calib_ESC1;                 //Still Not Available
 //ptr[9] = Calib_ESC2;                 //Still Not Available
 //ptr[10] = Calib_ESC3;                //Still Not Available
 //ptr[11] = Calib_ESC4;                //Still Not Available
 //ptr[12] = Calib_ESC5;                //Still Not Available
 //ptr[13] = Calib_ESC6;                //Still Not Available
 ptr[14] = Toggle_Var_Stream_State;
  
  
  
}


