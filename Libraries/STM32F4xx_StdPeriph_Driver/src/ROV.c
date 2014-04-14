/******************** (C) COPYRIGHT 2013 ROV TUNISIA ********************
* File Name          : Lighting_lib.c
* Author             : Rami HADJ TAIEB 
* Version            : V1.0
* Date               : 09/04/2014
* Description        : ROV main functions library
****************************************************************************/

/* includes -----------------------------------------------------------*/
#include "ROV.h"


int counter;




/*******************************************************************************
* Function Name  : ROV_VAR_Init
* Description    : Initialization of variable streaming table and function callback table
* Input          : ROV Structure, Received message from CAN
* Output         : None.
* Return         : None.
*******************************************************************************/
void ROV_VAR_Init(ROV_Struct* ROV)
{
  /* PelcoD ---------------------------- */
  ROV->CAN_Tab[0].StrPtr = &ROV->PelcoD.sense;
  ROV->CAN_Tab[1].StrPtr = &ROV->PelcoD.toggle_automan;
  ROV->CAN_Tab[2].StrPtr = &ROV->PelcoD.toggle_onoff;
  ROV->CAN_Tab[3].StrPtr = &ROV->PelcoD.iris_close;
  ROV->CAN_Tab[4].StrPtr = &ROV->PelcoD.iris_open;
  ROV->CAN_Tab[5].StrPtr = &ROV->PelcoD.focus_near;
  ROV->CAN_Tab[6].StrPtr = &ROV->PelcoD.focus_far;
  ROV->CAN_Tab[7].StrPtr = &ROV->PelcoD.zoom_wide;
  ROV->CAN_Tab[8].StrPtr = &ROV->PelcoD.zoom_tele;
  ROV->CAN_Tab[9].StrPtr = &ROV->PelcoD.tilt_down;
  ROV->CAN_Tab[10].StrPtr = &ROV->PelcoD.tilt_up;
  ROV->CAN_Tab[11].StrPtr = &ROV->PelcoD.pan_left;
  ROV->CAN_Tab[12].StrPtr = &ROV->PelcoD.pan_right;
  ROV->CAN_Tab[13].StrPtr = &ROV->PelcoD.pan_speed;
  ROV->CAN_Tab[14].StrPtr = &ROV->PelcoD.tilt_speed;
  /* Cam Angle pen ------------------------ */
  //ROV->CAN_Tab[15].StrPtr = ROV->CamAngle;
  /* Input from power supply -------------- */
  ROV->CAN_Tab[16].StrPtr = &ROV->Measurement_Unit.Current.analog.Status;
  ROV->CAN_Tab[17].StrPtr = &ROV->Measurement_Unit.Voltage.analog.Status;
  /* Front light of ROV ------------------- */
  ROV->CAN_Tab[18].StrPtr = ROV->light.right.integer8;
  ROV->CAN_Tab[19].StrPtr = ROV->light.left.integer8;
  /* Magneto Variables ------------------- */
  //ROV->CAN_Tab[20].StrPtr = ROV->Measurement_Unit.
  //ROV->CAN_Tab[21].StrPtr = ROV->Measurement_Unit.
  //ROV->CAN_Tab[22].StrPtr = ROV->Measurement_Unit.
  //ROV->CAN_Tab[23].StrPtr = ROV->Measurement_Unit.
  /* Gyro Variables --------------------- */
  ROV->CAN_Tab[24].StrPtr = ROV->Motion.Gyrox.integer;
  ROV->CAN_Tab[25].StrPtr = ROV->Motion.Gyroy.integer;
  ROV->CAN_Tab[26].StrPtr = ROV->Motion.Gyroz.integer;
  /* Accelerometer variables ------------ */
  ROV->CAN_Tab[27].StrPtr = ROV->Motion.Accelx.integer;
  ROV->CAN_Tab[28].StrPtr = ROV->Motion.Accely.integer;
  ROV->CAN_Tab[29].StrPtr = ROV->Motion.Accelz.integer;
  /* Euler Angles ---------------------- */
  ROV->CAN_Tab[30].StrPtr = ROV->Motion.Pitch.integer;
  ROV->CAN_Tab[31].StrPtr = ROV->Motion.Roll.integer;
  ROV->CAN_Tab[32].StrPtr = ROV->Motion.Yaw.integer;
  /* Depth ----------------------------- */
  ROV->CAN_Tab[33].StrPtr = ROV->Motion.Depth.integer;
  /* PID Parameters ------------------- */
  ROV->CAN_Tab[34].StrPtr = ROV->PID[0].Kp.integer;
  ROV->CAN_Tab[35].StrPtr = ROV->PID[1].Kp.integer;
  ROV->CAN_Tab[36].StrPtr = ROV->PID[2].Kp.integer;
  ROV->CAN_Tab[37].StrPtr = ROV->PID[3].Kp.integer;
  ROV->CAN_Tab[38].StrPtr = ROV->PID[4].Kp.integer;
  ROV->CAN_Tab[39].StrPtr = ROV->PID[5].Kp.integer;
  ROV->CAN_Tab[40].StrPtr = ROV->PID[0].Ki.integer;
  ROV->CAN_Tab[41].StrPtr = ROV->PID[1].Ki.integer;
  ROV->CAN_Tab[42].StrPtr = ROV->PID[2].Ki.integer;
  ROV->CAN_Tab[43].StrPtr = ROV->PID[3].Ki.integer;
  ROV->CAN_Tab[44].StrPtr = ROV->PID[4].Ki.integer;
  ROV->CAN_Tab[45].StrPtr = ROV->PID[5].Ki.integer;
  ROV->CAN_Tab[46].StrPtr = ROV->PID[0].Kd.integer;
  ROV->CAN_Tab[47].StrPtr = ROV->PID[1].Kd.integer;
  ROV->CAN_Tab[48].StrPtr = ROV->PID[2].Kd.integer;
  ROV->CAN_Tab[49].StrPtr = ROV->PID[3].Kd.integer;
  ROV->CAN_Tab[50].StrPtr = ROV->PID[4].Kd.integer;
  ROV->CAN_Tab[51].StrPtr = ROV->PID[5].Kd.integer;
  /* Thrusters ------------------------- */
  ROV->CAN_Tab[52].StrPtr = ROV->Propulsion[0].speed_command.integer8;
  ROV->CAN_Tab[53].StrPtr = ROV->Propulsion[1].speed_command.integer8;
  ROV->CAN_Tab[54].StrPtr = ROV->Propulsion[2].speed_command.integer8;
  ROV->CAN_Tab[55].StrPtr = ROV->Propulsion[3].speed_command.integer8;
  ROV->CAN_Tab[56].StrPtr = ROV->Propulsion[4].speed_command.integer8;
  ROV->CAN_Tab[57].StrPtr = ROV->Propulsion[5].speed_command.integer8;
  ROV->CAN_Tab[58].StrPtr = ROV->Propulsion[0].speed_feedback.integer8;
  ROV->CAN_Tab[59].StrPtr = ROV->Propulsion[1].speed_feedback.integer8;
  ROV->CAN_Tab[60].StrPtr = ROV->Propulsion[2].speed_feedback.integer8;
  ROV->CAN_Tab[61].StrPtr = ROV->Propulsion[3].speed_feedback.integer8;
  ROV->CAN_Tab[62].StrPtr = ROV->Propulsion[4].speed_feedback.integer8;
  ROV->CAN_Tab[63].StrPtr = ROV->Propulsion[5].speed_feedback.integer8;
  
 
 for(counter = 0;counter<256;counter++) //Set all the variables OFF from stream
 {
   ROV->CAN_Tab[counter].State = 0;
 }
}







/*******************************************************************************
* Function Name  : ROV_Init
* Description    : Initialisation of ROV
* Input          : ROV Structure, Received message from CAN
* Output         : None.
* Return         : None.
*******************************************************************************/
void ROV_Init(ROV_Struct* ROV,CanRxMsg RxMessage)
{
 
    AIOP_init(&(ROV->AIO));
    TIM_Init();
    CAN_init();
    RS485_init();
    Lighting_init();
    THRUSTER_init();
  
}




void ROV_Routine(ROV_Struct* ROV)
{   
}

void ROV_DataUpdate(ROV_Struct* ROV) //This function updates the values of variables called by timer interrupt
{
  ROV->Motion.Roll.floating_number = ROV->AIO.buffers.Current_50HZtrame.Roll.floating_number; //update Roll
  ROV->Motion.Pitch.floating_number = ROV->AIO.buffers.Current_50HZtrame.Pitch.floating_number; //update Pitch
  ROV->Motion.Yaw.floating_number = ROV->AIO.buffers.Current_50HZtrame.Yaw.floating_number; //update Yaw
  ROV->Motion.Accelx.floating_number = ROV->AIO.buffers.Current_50HZtrame.Accelx.floating_number; //update accelx
  ROV->Motion.Accely.floating_number = ROV->AIO.buffers.Current_50HZtrame.Accely.floating_number; //update accely
  ROV->Motion.Accelz.floating_number = ROV->AIO.buffers.Current_50HZtrame.Accelz.floating_number; //update accelz
  ROV->Motion.Gyrox.floating_number = ROV->AIO.buffers.Current_50HZtrame.Gyrox.floating_number; //update gyrox
  ROV->Motion.Gyroy.floating_number = ROV->AIO.buffers.Current_50HZtrame.Gyroy.floating_number; //update gyroy
  ROV->Motion.Gyroz.floating_number = ROV->AIO.buffers.Current_50HZtrame.Gyroz.floating_number; //update gyroz
  ROV->Motion.Depth.floating_number = ROV->AIO.buffers.Current_10HZtrame.Pressure.floating_number; //update depth
  
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
