/******************** (C) COPYRIGHT 2013 ROV TUNISIA ********************
* File Name          : Lighting_lib.c
* Author             : Rami HADJ TAIEB 
* Version            : V1.0
* Date               : 09/04/2014
* Description        : ROV main functions library
****************************************************************************/

/* includes -----------------------------------------------------------*/
#include "ROV.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ID_TABLE_LENGTH 105

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
int cnt = 0;
float joys[8];
uint32_t u[8];
float32_t Thruster_Axis_Projection;
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
   
    /* PelcoD ---------------------------- */ // This set of variables will not be used in this version 
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
    /* Thrusters Angle ------------------------- */
  ROV_var->identifiers_table[15].pointer = &ROV_var->Thruster_Angle; 
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
   ROV_var->identifiers_table[55].pointer = ROV_var->measurement_unit_sensors.AHRS.Mag.x_value.integer;
   ROV_var->identifiers_table[56].pointer = ROV_var->measurement_unit_sensors.AHRS.Mag.y_value.integer;
   ROV_var->identifiers_table[57].pointer = ROV_var->measurement_unit_sensors.AHRS.Mag.z_value.integer;
   
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
  
 for(cnt=0;cnt<ID_TABLE_LENGTH ;cnt++) //Set all the variables OFF from stream and initialize them at 0
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
/*******************************************************************************
* Function Name  : ROV_ControlMatrix_Init
* Description    : Initialize the thrusters configuration matrix 
* Input          : ROV structure
* Output         : None.
* Return         : None.
*******************************************************************************/
  
void ROV_ControlMatrix_Init(ROV_Struct* ROV)
{/*
  float32_t val1,val2,val3,val4,val5;
  float32_t Thruster_Axis_Projection;
  Thruster_Axis_Projection = DIS_THRUSTER_GCENTER * arm_cos_f32(DEGREE_TO_RADUIS(90 - ALPHA- ROV->Thruster_Angle));
  val1 = (1/(4*arm_cos_f32(DEGREE_TO_RADUIS(ROV->Thruster_Angle))));
  val2 = (1/(4*arm_sin_f32 (DEGREE_TO_RADUIS(ROV->Thruster_Angle))));
  val3 = 0.5;
  val4 = (1/(2*GAMMA));
  val5 = (1/(4*Thruster_Axis_Projection*arm_cos_f32(DEGREE_TO_RADUIS(ROV->Thruster_Angle))));
  
  float32_t thruster_matrix_vector[36]={
  val1, val2,0   ,0    ,0, val5,
  val1,-val2,0   ,0    ,0,-val5,
  val1, val2,0   ,0    ,0,-val5,
  val1,-val2,0   ,0    ,0, val5,
  0   ,0    ,val3, val4,0,    0,
  0   ,0    ,val3, val4,0,    0 
  };*/
  //arm_matrix_instance_f32 force_moment_matrix;
  //float32_t force_moment_vector[] = { 1 , 0 , 0 , 0 , 0 , 0};
  //arm_mat_init_f32(&ROV->thruster_matrix, 6, 6,thruster_matrix_vector);
  
  Thruster_Axis_Projection = DIS_THRUSTER_GCENTER * arm_cos_f32(DEGREE_TO_RADUIS(90 - ALPHA- ROV->Thruster_Angle));
  ROV->thruster_matrix_coef[0] = (1/(4*arm_cos_f32(DEGREE_TO_RADUIS(ROV->Thruster_Angle))));
  ROV->thruster_matrix_coef[1] = (1/(4*arm_sin_f32 (DEGREE_TO_RADUIS(ROV->Thruster_Angle))));
  ROV->thruster_matrix_coef[2] = 0.5;
  ROV->thruster_matrix_coef[3] = (1/(2*GAMMA));
  ROV->thruster_matrix_coef[4] = (1/(4*Thruster_Axis_Projection*arm_cos_f32(DEGREE_TO_RADUIS(ROV->Thruster_Angle))));
  
  
  }
/*******************************************************************************
* Function Name  : ROV_Routine
* Description    : Main Routine of the ROV 
* Input          : ROV Structure
* Output         : None.
* Return         : None.
*******************************************************************************/



void ROV_Routine(ROV_Struct *ROV)
{   
  
  
  /*float32_t val1,val2,val3,val4,val5;
  float32_t Thruster_Axis_Projection;
  Thruster_Axis_Projection = DIS_THRUSTER_GCENTER * arm_cos_f32(DEGREE_TO_RADUIS(90 - ALPHA- ROV->Thruster_Angle));
  val1 = (1/(4*arm_cos_f32(DEGREE_TO_RADUIS(ROV->Thruster_Angle))));
  val2 = (1/(4*arm_sin_f32 (DEGREE_TO_RADUIS(ROV->Thruster_Angle))));
  val3 = 0.5;
  val4 = (1/(2*GAMMA));
  val5 = (1/(4*Thruster_Axis_Projection*arm_cos_f32(DEGREE_TO_RADUIS(ROV->Thruster_Angle))));
  
   float32_t thruster_matrix_vector[36]={
  val1, val2,0   ,0    ,0, val5,
  val1,-val2,0   ,0    ,0,-val5,
  val1, val2,0   ,0    ,0,-val5,
  val1,-val2,0   ,0    ,0, val5,
  0   ,0    ,val3, val4,0,    0,
  0   ,0    ,val3, val4,0,    0 
  };
  
  float32_t result_matrix_tmp[6]; 
  float32_t init_rov_matrix[6*6];
  
  arm_matrix_instance_f32 joystick_matrix;
  arm_matrix_instance_f32 thruster_mat;
  arm_matrix_instance_f32 result_matrix;
   
  //uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
  joystick[0] = (float) ROV->joyst.x_axis.integer16;
   joystick[1] = (float)   ROV->joyst.y_axis.integer16;
   joystick[2]  = (float) ROV->joyst.throttle_1.integer16;
   joystick[3] = 0;
   joystick[4] = 0;
   joystick[5] = (float) ROV->joyst.rz_rotation.integer16;
   
   joystick[0] = map(joystick[0],0,65535,-1414,1414);
   joystick[1] = map(joystick[1],0,65535,-1414,1414);
   joystick[2] = map(joystick[2],0,65535,-1000,1000);
   //joystick[3] = map(joystick[3],0,65535,-1000,1000);
   //joystick[4] = map(joystick[4],0,65535,-1000,1000);
   joystick[5] = map(joystick[5],0,65535,-398868,398868);
   

    
   arm_mat_init_f32(&thruster_mat, 6, 6,thruster_matrix_vector); 
   arm_mat_init_f32(&joystick_matrix, 6, 1,joystick); 
   arm_mat_init_f32(&result_matrix, 6, 1,result_matrix_tmp); 
   
  arm_mat_mult_f32(&joystick_matrix, &thruster_mat,&result_matrix);
   */
  
   joys[0] = ROV->joyst.x_axis.integer16;
   joys[1] = ROV->joyst.y_axis.integer16;
   joys[2] = ROV->joyst.throttle_1.integer16;
   joys[3] = 0;
   joys[4] = 0;
   joys[5] = ROV->joyst.rz_rotation.integer16;
   
   //res=(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
   joys[0] = (joys[0] - 0) * (5656 - (2828)) / (65535 - 0) + (2828);
   //joys[0] = (map(joys[0],0,65535,-1414,1414)+1500);
   joys[1] = (joys[1] - 0) * (1414 - (-1414)) / (65535 - 0) + (-1414);
   //joys[2] = (map(joys[2],0,65535,-1000,1000)+1500);
   //joys[3] = map(joys[3],0,65535,-1000,1000);
   //joys[4] = map(joys[4],0,65535,-1000,1000);
   joys[5] = (joys[5] - 0) * (398868 - (-398868))/(65535 - 0) + (-398868);
   
   u[0] = (uint16_t) (((joys[0]*ROV->thruster_matrix_coef[0] + joys[1]*ROV->thruster_matrix_coef[1] + joys[5]*ROV->thruster_matrix_coef[4])));
   u[1] = (uint16_t) (((joys[0]*ROV->thruster_matrix_coef[0] + joys[1]*(-ROV->thruster_matrix_coef[1]) + joys[5]*(-ROV->thruster_matrix_coef[4]))));
   u[2] = (uint16_t) (((joys[0]*ROV->thruster_matrix_coef[0] + joys[1]*(ROV->thruster_matrix_coef[1]) + joys[5]*(-ROV->thruster_matrix_coef[4]))));
   u[3] = (uint16_t) (((joys[0]*ROV->thruster_matrix_coef[0] + joys[1]*(-ROV->thruster_matrix_coef[1]) + joys[5]*(ROV->thruster_matrix_coef[4]))));
   
   for(int cor = 0;cor<6;cor++)
   {
     if(u[cor]<1000)
     {
       u[cor] = 1000;
     }
     if(u[cor]>2000)
     {
       u[cor] = 2000;
     }
   }
   ROV->propulsion[0].speed_feedback.integer16 = u[0];
   ROV->propulsion[1].speed_feedback.integer16 = u[1];
   ROV->propulsion[2].speed_feedback.integer16 = u[2];
   ROV->propulsion[3].speed_feedback.integer16 = u[3];
   ROV->propulsion[4].speed_feedback.integer16 = ((((float)((joys[2])/65535))*1000)+1000);
   ROV->propulsion[5].speed_feedback.integer16 = ((((float)((joys[2])/65535))*1000)+1000);
  
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
   //if( state->is_variable_in_use == 0)
   //{
    state->is_variable_in_use = 1; 
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
    state->is_variable_in_use = 0;
  // }
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
       //if( state->is_variable_in_use == 0)
   //{
    state->is_variable_in_use = 1;
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
    state->is_variable_in_use = 0;
   //}
  } /* if new data is ready to be copied */
}

/*******************************************************************************
* Function Name  : update_pelcod_values
* Description    : send pelcoD values to the cam
* Input          : rov structure, received can message
* Output         : None.
* Return         : None.
*******************************************************************************/
void update_pelcod_values(ROV_Struct* ROV,CanRxMsg CAN_Msg)
{
  PelcoD_Send(&(ROV->pelcod));
}

/*******************************************************************************
* Function Name  : ROV_Init
* Description    : Initialisation of ROV
* Input          : ROV Structure, Received message from CAN
* Output         : None.
* Return         : None.
*******************************************************************************/
void ROV_Init(ROV_Struct* ROV)
{ /* Initialization of ROV states */
  ROV->rov_state.is_aiop_communication_allowed = 1;
  MeasurmentUnit_init(&(ROV->aio)); //initialize USART4 for communication with AIOP
  TIM_Init(); // Timer for interruptions (1,10,50,100 HZ)
  CAN_init(); 
 // RS485_init(); // For PTZ camera control 
  Lighting_init(); // LED PWM init
  THRUSTER_init(ROV->propulsion);
  ROV_VAR_Init(ROV); //initialization of CAN table identifiers
  ROV->Thruster_Angle=DEFAULT_THRUSTERS_ANGLE;
  /* Initialization of the state machine handling AIOP communciation */
  ROV->aio.buffers.frame_10Hz.state = IDLE;
  ROV->aio.buffers.frame_50Hz.state = IDLE;
  
  
}

void ROV_coldStart_Init(ROV_Struct* ROV)
{
    ROV->propulsion[0].speed_command.integer16 = 1500;
    ROV->propulsion[1].speed_command.integer16 = 1500;
    ROV->propulsion[2].speed_command.integer16 = 1500;
    ROV->propulsion[3].speed_command.integer16 = 1500;
    ROV->propulsion[4].speed_command.integer16 = 1500;
    ROV->propulsion[5].speed_command.integer16 = 1500;
    
    THRUSTER_update(ROV->propulsion);
    
    ROV->propulsion[0].speed_feedback.integer16 = 1500;
    ROV->propulsion[1].speed_feedback.integer16 = 1500;
    ROV->propulsion[2].speed_feedback.integer16 = 1500;
    ROV->propulsion[3].speed_feedback.integer16 = 1500;
    ROV->propulsion[4].speed_feedback.integer16 = 1500;
    ROV->propulsion[5].speed_feedback.integer16 = 1500;
    ROV->joyst.buttons.integer16=0;
    ROV->joyst.pov.integer16=0;
    
    
    ROV->rov_state.is_streaming_enabled = 0;
    ROV->rov_state.is_computer_connected = 0;
    
    ROV->measurement_unit_sensors.AHRS.Euler_Angle.x_value.floating_number = 5.3;
    ROV->measurement_unit_sensors.AHRS.Euler_Angle.y_value.floating_number = 9.6;
    ROV->measurement_unit_sensors.AHRS.Euler_Angle.z_value.floating_number = 7.1;
    ROV->measurement_unit_sensors.Pressure.floating_number=28.65;
    ROV->identifiers_table[80].State = 1;
    ROV->identifiers_table[81].State = 1;
    ROV->identifiers_table[82].State = 1;
    ROV->identifiers_table[83].State = 1;
    ROV->identifiers_table[39].State = 1;
    ROV->identifiers_table[40].State = 1;
    ROV->identifiers_table[41].State = 1;
    ROV->identifiers_table[42].State = 1;
    ROV->identifiers_table[43].State = 1;
    ROV->identifiers_table[44].State = 1;
    

    ROV->light.right.integer16 = 50;
}

float map(float x, float in_min, float in_max, float out_min, float out_max)
        {
            float res=(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
            if (res < out_min)
            {
                res = out_min;
            }
            else if (res > out_max)
            {
                res = out_max;
            }
            return res;

        }

void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

/* End of file ---------------------------------------------------------------*/