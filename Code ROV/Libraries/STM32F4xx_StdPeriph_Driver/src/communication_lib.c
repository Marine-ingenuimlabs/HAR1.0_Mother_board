


/* includes -----------------------------------------------------------*/
#include "communication_lib.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t error_can;
uint8_t taille;
//int cnt1,cnt2;
void (*function_pointer[16])(ROV_Struct*,CanRxMsg); // Functions Callback Table

/* Global variables ----------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/*******************************************************************************
* Function Name  : ROV_Set_Var
* Description    : Set variable in ROV
* Input          : ROV Structure, Received message from CAN
* Output         : None.
* Return         : None.
*******************************************************************************/
void ROV_Set_Var(ROV_Struct* ROV,CanRxMsg CAN_Msg)
{
  //if(ROV->rov_state.is_variable_in_use==0)
  //{
    ROV->rov_state.is_variable_in_use = 1;
    for(uint8_t cnt=0;cnt<(CAN_Msg.DLC-1);cnt++)
    {
      ROV->identifiers_table[CAN_Msg.Data[0]].pointer[(CAN_Msg.DLC-2)-cnt] = CAN_Msg.Data[cnt+1];
    }
  ROV->rov_state.is_variable_in_use = 0;
  //}
}

/*******************************************************************************
* Function Name  : Toggle_Var_Stream_State
* Description    : Allow or forbid one single variable from stream
* Input          : ROV Structure, Received message from CAN
* Output         : None.
* Return         : None.
*******************************************************************************/
void Toggle_Var_Stream_State(ROV_Struct* ROV,CanRxMsg CAN_Msg)
{
  
    ROV->identifiers_table[CAN_Msg.Data[0]].State &= (CAN_Msg.Data[1]&0x01);
    ROV->identifiers_table[CAN_Msg.Data[0]].State |= (CAN_Msg.Data[1]&0x01);
    
    
}

/*******************************************************************************
* Function Name  : ROV_Req_Val
* Description    : Send a variable value once
* Input          : ROV Structure, Received message from CAN
* Output         : None.
* Return         : None.
*******************************************************************************/
void ROV_Req_Val(ROV_Struct* ROV,CanRxMsg CAN_Msg)
{
  if(ROV->rov_state.is_variable_in_use==0)
  {
    ROV->rov_state.is_variable_in_use = 1;
    error_can = CAN_send(ROV->identifiers_table[CAN_Msg.ExtId-128].pointer,CAN_Msg.ExtId); //Send Requested Value Referenced in identifiers_table in Rov_Struct
    ROV->rov_state.is_variable_in_use = 0;
  }
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
  for( int cnt = 0;cnt<256;cnt++) //for all the variables
  {
    if (ROV.identifiers_table[cnt].State==1)
    {  
      error_can = CAN_send((ROV.identifiers_table[cnt].pointer),cnt); //Send
    } /*if the variable is enabled for stream */
  }
}



/*******************************************************************************
* Function Name  : ROV_Toggle_Stream
* Description    : Enable or disable streaming to command station
* Input          : ROV Structure, Received message from CAN
* Output         : None.
* Return         : None.
*******************************************************************************/
void ROV_Toggle_Streaming_Mode(ROV_Struct* ROV,CanRxMsg CAN_Msg)
{
(ROV->rov_state.is_streaming_enabled) = (CAN_Msg.Data[0]&0x01);
}

/*******************************************************************************
* Function Name  : ROV_Toggle_AIOP
* Description    : Enable or disable communication with AIOP via USART
* Input          : ROV Structure, Received message from CAN
* Output         : None.
* Return         : None.
*******************************************************************************/
void ROV_Toggle_AIOP_Communication(ROV_Struct* ROV,CanRxMsg CAN_Msg)
{
  (ROV->rov_state.is_aiop_communication_allowed) = (CAN_Msg.Data[0]&0x01);
}

/*******************************************************************************
* Function Name  : communication_init
* Description    : set ROV state is_computer_connect
* Input          : ROV Structure, Received message from CAN
* Output         : None.
* Return         : None.
*******************************************************************************/
void communication_init(ROV_Struct* ROV,CanRxMsg RxMessage)
{
  if((RxMessage.Data[0]==0xFA)&&(RxMessage.Data[1]==0xFE))
  {
    ROV->rov_state.is_streaming_enabled = 1;
    ROV->rov_state.is_computer_connected = 1;
  }
}

/*******************************************************************************
* Function Name  : FuncCallbackTable_INIT
* Description    : Initialising functions' pointers table for calling
* Input          : ROV Structure, Received message from CAN
* Output         : None.
* Return         : None.
*******************************************************************************/
void FuncCallbackTable_INIT()
{
 function_pointer[0] = ROV_Set_Var;
 function_pointer[1] = communication_init;
 function_pointer[2] = ROV_Toggle_Streaming_Mode;
 //function_pointer[3] = Toggle_AutoControl;
 //function_pointer[4] = Toggle_Magneto;
 function_pointer[5] = ROV_Toggle_AIOP_Communication;
 //function_pointer[6] = Calib_IMU;                  //Still Not Available
 //function_pointer[7] = Calib_Magneto;              //Still Not Available
 //function_pointer[8] = Stop_all_thrusters;         //Still Not Available
 //function_pointer[9] = Emergency_stop;             //Still Not Available
 function_pointer[10] = update_pelcod_values;     
 //function_pointer[11] = update_joystick_values;    //Still Not Available
 //function_pointer[12] = update_thrusters_values;             
 //function_pointer[13] = update_lighting_values;       
 function_pointer[14] = Toggle_Var_Stream_State;
}


/* End of file -------------------------------------------------------------- */
