


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
void print_joystick(ROV_Struct* ROV);
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
      ROV->CAN_buffer[CAN_Msg.Data[0]].pointer[(CAN_Msg.DLC-2)-cnt] = CAN_Msg.Data[cnt+1];
    }
  ROV->rov_state.is_variable_in_use = 0;
      ROV->rov_state.is_new_data_available = DATA_AVAILABLE;
  /*if (CAN_Msg.Data[0]>=45 && CAN_Msg.Data[0]<=51)
        print_joystick(ROV); *//*
  if (CAN_Msg.Data[0]==48)
  {
     USART_puts(USART1,"THROTTLE_1\t");  
          USART_puts(USART1,conv_f2c(ROV->joyst.throttle_1.integer16));
          USART_SendData(USART1,'\n');
  }
      else if (CAN_Msg.Data[0]==49)
      {
          USART_puts(USART1,"THROTTLE_2\t");  
          USART_puts(USART1,conv_f2c(ROV->joyst.throttle_2.integer16));
          USART_SendData(USART1,'\n');
      }
    
   */
  //}
}

void print_joystick(ROV_Struct* ROV )
{
          /*USART_puts(USART1,"ROLL");  
          USART_puts(USART1,conv_f2c(ROV.measurement_unit_sensors.AHRS.Euler_Angle.x_value.floating_number));
          USART_SendData(USART1,'\r');
          USART_SendData(USART1,'\n');
          
          USART_puts(USART1,"PITCH");  
          USART_puts(USART1,conv_f2c(ROV.measurement_unit_sensors.AHRS.Euler_Angle.y_value.floating_number));
          USART_SendData(USART1,'\r');
          USART_SendData(USART1,'\n');
          
          USART_puts(USART1,"YAW");  
          USART_puts(USART1,conv_f2c(ROV.measurement_unit_sensors.AHRS.Euler_Angle.z_value.floating_number));
          USART_SendData(USART1,'\r');
          USART_SendData(USART1,'\n');*/
 
          USART_puts(USART1,"JOY_X:\t");  
          USART_puts(USART1,conv_f2c(ROV->joyst.x_axis.integer16));
          USART_SendData(USART1,'\t');

          USART_puts(USART1,"JOY_Y\t");  
          USART_puts(USART1,conv_f2c(ROV->joyst.y_axis.integer16));
          USART_SendData(USART1,'\t');
   
          USART_puts(USART1,"JOY_RZ\t");  
          USART_puts(USART1,conv_f2c(ROV->joyst.rz_rotation.integer16));
          USART_SendData(USART1,'\t');
          
          USART_puts(USART1,"POV\t");  
          USART_puts(USART1,conv_f2c(ROV->joyst.pov.integer16));
          USART_SendData(USART1,'\t');
          
          USART_puts(USART1,"THROTTLE_1\t");  
          USART_puts(USART1,conv_f2c(ROV->joyst.throttle_1.integer16));
          USART_SendData(USART1,'\t');
          
          USART_puts(USART1,"THROTTLE_2\t");  
          USART_puts(USART1,conv_f2c(ROV->joyst.throttle_2.integer16));
          USART_SendData(USART1,'\t');
          
          USART_puts(USART1,"button\t");  
          USART_puts(USART1,conv_f2c(ROV->joyst.buttons.integer16));
          USART_SendData(USART1,'\r');
          USART_SendData(USART1,'\n');
 
         /* 
          for(int countt=0;countt<6;countt++)
          {
            
          USART_puts(USART1,"THRUSTERS\t");  
          USART_puts(USART1,conv_f2c(ROV->propulsion[countt].speed_feedback.integer16));
          USART_SendData(USART1,' ');
          USART_SendData(USART1,'\n');
          
            
          }
          USART_SendData(USART1,'\n');
          
          USART_puts(USART1,"button\t");  
          USART_puts(USART1,conv_f2c(ROV->joyst.buttons.integer16));
          USART_SendData(USART1,'\r');
          USART_SendData(USART1,'\n');
          
          USART_puts(USART1,"button\t");  
          USART_puts(USART1,conv_f2c(ROV->joyst.buttons.integer16));
          USART_SendData(USART1,'\r');
          USART_SendData(USART1,'\n');*/
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
  
    ROV->CAN_buffer[CAN_Msg.Data[0]].State &= (CAN_Msg.Data[1]&0x01);
    ROV->CAN_buffer[CAN_Msg.Data[0]].State |= (CAN_Msg.Data[1]&0x01);
    
    
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
 // if(ROV->rov_state.is_variable_in_use==0)
 // {
   // ROV->rov_state.is_variable_in_use = 1;
  if (CAN_Msg.ExtId>128)
  {
    error_can = CAN_send(ROV->CAN_buffer[CAN_Msg.ExtId-128].pointer,CAN_Msg.ExtId); //Send Requested Value Referenced in CAN_buffer in Rov_Struct
  }
    //ROV->rov_state.is_variable_in_use = 0;
  //}
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
    if (ROV.CAN_buffer[cnt].State==1)
    {  
      error_can = CAN_send((ROV.CAN_buffer[cnt].pointer),cnt); //Send
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
* Function Name  : ROV_Reset
* Description    : This function is used to 
* Input          : ROV Structure, Received message from CAN
* Output         : None.
* Return         : None.
*******************************************************************************/
void ROV_Reset(ROV_Struct* ROV,CanRxMsg RxMessage)
{
  if(((RxMessage.Data[0]=='R') || (RxMessage.Data[0]=='r')) && ((RxMessage.Data[1]=='S')||(RxMessage.Data[1]=='s')) && ((RxMessage.Data[2]=='T')||(RxMessage.Data[2]=='t')))
  {
  NVIC_SystemReset();
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
 function_pointer[15] = ROV_Reset;
}


/* End of file -------------------------------------------------------------- */
