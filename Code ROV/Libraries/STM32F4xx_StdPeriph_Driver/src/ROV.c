/******************** (C) COPYRIGHT 2013 ROV TUNISIA ********************
* File Name          : Lighting_lib.c
* Author             : Rami HADJ TAIEB 
* Version            : V1.0
* Date               : 09/04/2014
* Description        : ROV main functions library
****************************************************************************/

/* includes -----------------------------------------------------------*/
#include "ROV.h"

int cnt = 0;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/





/*******************************************************************************
* Function Name  : ROV_VAR_Init
* Description    : Initialization of variable streaming table and function callback table
* Input          : ROV Structure, Received message from CAN
* Output         : None.
* Return         : None.
*******************************************************************************/
void ROV_VAR_Init(ROV_Struct* ROV_var)
{

    /* PelcoD ---------------------------- */
  /*ROV->identifiers_table[0].pointer = &ROV->PelcoD.sense;
  ROV->identifiers_table[1].pointer = &ROV->PelcoD.toggle_automan;
  ROV->identifiers_table[2].pointer = &ROV->PelcoD.toggle_onoff;
  ROV->identifiers_table[3].pointer = &ROV->PelcoD.iris_close;
  ROV->identifiers_table[4].pointer = &ROV->PelcoD.iris_open;
  ROV->identifiers_table[5].pointer = &ROV->PelcoD.focus_near;
  ROV->identifiers_table[6].pointer = &ROV->PelcoD.focus_far;
  ROV->identifiers_table[7].pointer = &ROV->PelcoD.zoom_wide;
  ROV->identifiers_table[8].pointer = &ROV->PelcoD.zoom_tele;
  ROV->identifiers_table[9].pointer = &ROV->PelcoD.tilt_down;
  ROV->identifiers_table[10].pointer = &ROV->PelcoD.tilt_up;
  ROV->identifiers_table[11].pointer = &ROV->PelcoD.pan_left;
  ROV->identifiers_table[12].pointer = &ROV->PelcoD.pan_right;
  ROV->identifiers_table[13].pointer = &ROV->PelcoD.pan_speed;
  ROV->identifiers_table[14].pointer = &ROV->PelcoD.tilt_speed;*/
  /* Cam Angle pen ------------------------ */
  //ROV->identifiers_table[30].pointer = ROV->CamAngle;

  /* Front light of ROV ------------------- */
  ROV_var->identifiers_table[31].pointer = ROV_var->light.right.integer8;
  ROV_var->identifiers_table[32].pointer = ROV_var->light.left.integer8;
  
  /* Thrusters ------------------------- */
  ROV_var->identifiers_table[33].pointer = ROV_var->propulsion[0].speed_command.integer8;
  ROV_var->identifiers_table[34].pointer = ROV_var->propulsion[1].speed_command.integer8;
  ROV_var->identifiers_table[35].pointer = ROV_var->propulsion[2].speed_command.integer8;
  ROV_var->identifiers_table[36].pointer = ROV_var->propulsion[3].speed_command.integer8;
  ROV_var->identifiers_table[37].pointer = ROV_var->propulsion[4].speed_command.integer8;
  ROV_var->identifiers_table[38].pointer = ROV_var->propulsion[5].speed_command.integer8;
  ROV_var->identifiers_table[39].pointer = ROV_var->propulsion[0].speed_feedback.integer8;
  ROV_var->identifiers_table[40].pointer = ROV_var->propulsion[1].speed_feedback.integer8;
  ROV_var->identifiers_table[41].pointer = ROV_var->propulsion[2].speed_feedback.integer8;
  ROV_var->identifiers_table[42].pointer = ROV_var->propulsion[3].speed_feedback.integer8;
  ROV_var->identifiers_table[43].pointer = ROV_var->propulsion[4].speed_feedback.integer8;
  ROV_var->identifiers_table[44].pointer = ROV_var->propulsion[5].speed_feedback.integer8;
  
  /* Joystick ---------------------------*/
  ROV_var->identifiers_table[45].pointer = ROV_var->joyst.x_axis.integer8;
  ROV_var->identifiers_table[46].pointer = ROV_var->joyst.y_axis.integer8;
  ROV_var->identifiers_table[47].pointer = ROV_var->joyst.rz_rotation.integer8;
  ROV_var->identifiers_table[48].pointer = ROV_var->joyst.throttle_1.integer8;
  ROV_var->identifiers_table[49].pointer = ROV_var->joyst.throttle_2.integer8;
  ROV_var->identifiers_table[50].pointer = ROV_var->joyst.pov.integer8;
  ROV_var->identifiers_table[51].pointer = ROV_var->joyst.buttons.integer8;
  
  /* Magnetometer offsets -------------- */
  ROV_var->identifiers_table[52].pointer = ROV_var->measurement_unit_sensors.AHRS.Mag.x_offset.integer;
  ROV_var->identifiers_table[53].pointer = ROV_var->measurement_unit_sensors.AHRS.Mag.y_offset.integer;
  ROV_var->identifiers_table[54].pointer = ROV_var->measurement_unit_sensors.AHRS.Mag.z_offset.integer;
  
  /* Magnetometer value ---------------- */
   ROV_var->identifiers_table[70].pointer = ROV_var->measurement_unit_sensors.AHRS.Mag.x_value.integer;
   ROV_var->identifiers_table[71].pointer = ROV_var->measurement_unit_sensors.AHRS.Mag.y_value.integer;
   ROV_var->identifiers_table[72].pointer = ROV_var->measurement_unit_sensors.AHRS.Mag.z_value.integer;
   
   /* Gyro Variables --------------------- */
  ROV_var->identifiers_table[74].pointer = ROV_var->measurement_unit_sensors.AHRS.Gyro.x_value.integer;
  ROV_var->identifiers_table[75].pointer = ROV_var->measurement_unit_sensors.AHRS.Gyro.y_value.integer;
  ROV_var->identifiers_table[76].pointer = ROV_var->measurement_unit_sensors.AHRS.Gyro.z_value.integer;
  /* Accelerometer variables ------------ */
  ROV_var->identifiers_table[77].pointer = ROV_var->measurement_unit_sensors.AHRS.Accel.x_value.integer;
  ROV_var->identifiers_table[78].pointer = ROV_var->measurement_unit_sensors.AHRS.Accel.y_value.integer;
  ROV_var->identifiers_table[79].pointer = ROV_var->measurement_unit_sensors.AHRS.Accel.z_value.integer;
  /* Euler Angles ---------------------- */
  ROV_var->identifiers_table[80].pointer = ROV_var->measurement_unit_sensors.AHRS.Euler_Angle.x_value.integer;
  ROV_var->identifiers_table[81].pointer = ROV_var->measurement_unit_sensors.AHRS.Euler_Angle.y_value.integer;
  ROV_var->identifiers_table[82].pointer = ROV_var->measurement_unit_sensors.AHRS.Euler_Angle.z_value.integer;
  /* Depth ----------------------------- */
  ROV_var->identifiers_table[83].pointer = ROV_var->measurement_unit_sensors.Pressure.integer;
  
    /* pid Parameters ------------------- */
  ROV_var->identifiers_table[84].pointer = ROV_var->pid[0].Kp.integer;
  ROV_var->identifiers_table[85].pointer = ROV_var->pid[1].Kp.integer;
  ROV_var->identifiers_table[86].pointer = ROV_var->pid[2].Kp.integer;
  ROV_var->identifiers_table[87].pointer = ROV_var->pid[3].Kp.integer;
  ROV_var->identifiers_table[88].pointer = ROV_var->pid[4].Kp.integer;
  ROV_var->identifiers_table[89].pointer = ROV_var->pid[5].Kp.integer;
  ROV_var->identifiers_table[90].pointer = ROV_var->pid[0].Ki.integer;
  ROV_var->identifiers_table[91].pointer = ROV_var->pid[1].Ki.integer;
  ROV_var->identifiers_table[92].pointer = ROV_var->pid[2].Ki.integer;
  ROV_var->identifiers_table[93].pointer = ROV_var->pid[3].Ki.integer;
  ROV_var->identifiers_table[94].pointer = ROV_var->pid[4].Ki.integer;
  ROV_var->identifiers_table[95].pointer = ROV_var->pid[5].Ki.integer;
  ROV_var->identifiers_table[96].pointer = ROV_var->pid[0].Kd.integer;
  ROV_var->identifiers_table[97].pointer = ROV_var->pid[1].Kd.integer;
  ROV_var->identifiers_table[98].pointer = ROV_var->pid[2].Kd.integer;
  ROV_var->identifiers_table[99].pointer = ROV_var->pid[3].Kd.integer;
  ROV_var->identifiers_table[100].pointer = ROV_var->pid[4].Kd.integer;
  ROV_var->identifiers_table[101].pointer = ROV_var->pid[5].Kd.integer;
  
  /* Current and Voltage -------------- */
  ROV_var->identifiers_table[102].pointer = ROV_var->measurement_unit_sensors.Current.integer;
  ROV_var->identifiers_table[103].pointer = ROV_var->measurement_unit_sensors.Voltage.integer;
  
  /* Temperature -------------- */
  ROV_var->identifiers_table[104].pointer = ROV_var->measurement_unit_sensors.Onboard_Temprature.integer;
  ROV_var->identifiers_table[105].pointer = ROV_var->measurement_unit_sensors.Water_Temprature.integer;
  
 for(cnt=0;cnt<105;cnt++) //Set all the variables OFF from stream and initialize them at 0
 {
   ROV_var->identifiers_table[cnt].State = 0;
    if(cnt<30){
     ROV_var->identifiers_table[cnt].pointer[0] = 0;
   }/* if size if one octet */
   else if(cnt<70){
     ROV_var->identifiers_table[cnt].pointer[0] = 0;
     ROV_var->identifiers_table[cnt].pointer[1] = 0;
   }/* if size if two octet */
   else{
     ROV_var->identifiers_table[cnt].pointer[0] = 0;
     ROV_var->identifiers_table[cnt].pointer[1] = 0;
     ROV_var->identifiers_table[cnt].pointer[3] = 0;
     ROV_var->identifiers_table[cnt].pointer[4] = 0;
   }/* if size if four octet */
 }
}




void ROV_Routine(ROV_Struct* ROV)
{   
  
}

/*******************************************************************************
* Function Name  : Sensor_DataUpdate_50Hz
* Description    : Copying the 50Hz buffer data to the sensor structure
* Input          : sensors structure, 50Hz buffer, rov_state
* Output         : None.
* Return         : None.
*******************************************************************************/
void Sensor_DataUpdate_50Hz(Sensors *destination,AIOP_50HZMessage volatile *source,state_of_rov *state) //This function updates the values of variables coming at 50Hz frequency called by timer interrupt
{
  if(source->state == DATA_READY)
  {
    state->is_variable_in_use = 0;
    source->state = DATA_PROCESSING;
    /* Updating Euler Angles ---------------------- */
    destination->AHRS.Euler_Angle.x_value.floating_number = source->Roll.floating_number;
    destination->AHRS.Euler_Angle.y_value.floating_number = source->Pitch.floating_number;
    destination->AHRS.Euler_Angle.z_value.floating_number = source->Yaw.floating_number;
    /* Updating Accelerations --------------------- */
    destination->AHRS.Accel.x_value.floating_number = source->Accelx.floating_number;
    destination->AHRS.Accel.y_value.floating_number = source->Accely.floating_number;
    destination->AHRS.Accel.z_value.floating_number = source->Accelz.floating_number;
    /* Updating angular velocities --------------- */
    destination->AHRS.Gyro.x_value.floating_number = source->Gyrox.floating_number;
    destination->AHRS.Gyro.y_value.floating_number = source->Gyroy.floating_number;
    destination->AHRS.Gyro.z_value.floating_number = source->Gyroz.floating_number;
    source->state = IDLE;
    state->is_variable_in_use = 1;
  } /* if new data is ready to be copied */
}

/*******************************************************************************
* Function Name  : Sensor_DataUpdate_50Hz_DataUpdate_10Hz
* Description    : Copying the 50Hz buffer data to the sensor structure
* Input          : sensors structure, 50Hz buffer, rov_state
* Output         : None.
* Return         : None.
*******************************************************************************/
void Sensor_DataUpdate_10Hz(Sensors *destination,AIOP_10HZMessage volatile *source,state_of_rov *state) //This function updates the values of variables coming at 10Hz frequency called by timer interrupt
{
   if(source->state == DATA_READY)
  {
    state->is_variable_in_use = 0;
    source->state = DATA_PROCESSING;
    /* Updating current and voltage --------------- */
    destination->Current.floating_number = source->Current.floating_number;
    destination->Voltage.floating_number = source->Voltage.floating_number;
    /* Updating temperature --------------- */
    destination->Onboard_Temprature.floating_number = source->Onboard_Temprature.floating_number;
    destination->Water_Temprature.floating_number = source->Water_Temprature.floating_number;
    /* Updating pressure voltage --------------- */
    destination->Pressure.floating_number = source->Pressure.floating_number;
    source->state = IDLE;
    state->is_variable_in_use = 1;
  } /* if new data is ready to be copied */
}

/*******************************************************************************
* Function Name  : update_pelcod_values
* Description    : send pelcoD values to the cam
* Input          : rov structure, received can message
* Output         : None.
* Return         : None.
*******************************************************************************/
void update_pelcod_values(ROV_Struct* ROV,CanRxMsg CAN_Msg){
  
  PelcoD_Send(&(ROV->pelcod));
  
}

/*******************************************************************************
* Function Name  : update_thrusters_values
* Description    : updates thruster values
* Input          : rov structure, received can message
* Output         : None.
* Return         : None.
*******************************************************************************/
/*
void update_thrusters_values(ROV_Struct* ROV,CanRxMsg CAN_Msg){
  
  THRUSTER_update(ROV->propulsion);
  
}*/

/*******************************************************************************
* Function Name  : update_lighting_values
* Description    : updates lighting values
* Input          : rov structure, received can message
* Output         : None.
* Return         : None.
*******************************************************************************/
/*
void update_lighting_values(ROV_Struct* ROV,CanRxMsg CAN_Msg){
  Lighting_update(&ROV->light);
}*/

/*
void update_joystick_values(ROV_Struct* ROV,CanRxMsg CAN_Msg){
}

*/



/*******************************************************************************
* Function Name  : ROV_Init
* Description    : Initialisation of ROV
* Input          : ROV Structure, Received message from CAN
* Output         : None.
* Return         : None.
*******************************************************************************/
void ROV_Init(ROV_Struct* ROV)
{
  MeasurmentUnit_init(&(ROV->aio)); //initialize USART4 for communication with AIOP
  TIM_Init(); 
  CAN_init(); 
  RS485_init(); 
  Lighting_init();
  THRUSTER_init(ROV->propulsion);
  ROV_VAR_Init(ROV); //initialization of CAN table identifiers
  
  
  /* Initialization of the state machine handling AIOP communciation */
  ROV->aio.buffers.frame_10Hz.state = IDLE;
  ROV->aio.buffers.frame_50Hz.state = IDLE;
  
  /* Initialization of ROV states */
  ROV->rov_state.is_aiop_communication_allowed = 1;
  //ROV->rov_state.is_variable_in_use = 1;
  //ROV->rov_state.is_streaming_enabled = 0;
  //ROV->rov_state.is_computer_connected = 0;
  
  ROV->propulsion[0].speed_command.integer16 = 0;
  ROV->propulsion[1].speed_command.integer16 = 0;
  ROV->propulsion[2].speed_command.integer16 = 0;
  ROV->propulsion[3].speed_command.integer16 = 0;
  THRUSTER_update(ROV->propulsion);
  
  }



void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}



//les fonctions suivantes sont des fonctions utilisées pour le débogage
void USART_puts(USART_TypeDef* USARTx,__IO char *s)
{

	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) ); 
		USART_SendData(USARTx, *s);
		*s++;
	}
}
    
//converting data from float to char
char* conv_f2c(float f)
{
  
    static int pint;
    static int pfract;
    static char string[7];
    static int sign;
    
    sign=(int)f;
    pint =(int)f;
    pfract= (int)((f - pint)*1000);
    pint = abs(pint);
    pfract = abs(pfract);
    
    
    
     if (sign < 0)
    {
      sprintf(string,"-%03d.%03d",pint,pfract);
    }  
    else
    {
      sprintf(string,"%03d.%03d",pint,pfract);
    }
    return string;
  
}

/* End of file ---------------------------------------------------------------*/