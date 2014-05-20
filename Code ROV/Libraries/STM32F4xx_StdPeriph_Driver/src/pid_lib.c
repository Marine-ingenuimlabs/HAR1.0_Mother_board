#include "pid_lib.h"

#define RC 0

float output;
float variation;
float derivative, integral, proportional;
float a;

/* Computing the command ---------------------------- */
float pid_compute(float error,float dt,pid_controller pid) //dt is the sampling period
{

  //calculate the sum of errors
  pid.sum_error += error;
  
  //calculate the variation of error
  variation = error - pid.last_error;
  
  //compute proportional value
  proportional = error * pid.Kp.floating_number;
  
  //compute integral value
  integral = (pid.sum_error * pid.Ki.floating_number) * dt;
   
  //compute derivative value
  derivative = (variation * pid.Kd.floating_number) / dt;
  
  //applying a low pas filter to the derivative component to filter the high frequency noise that can drive the controller crazy!!!
  a = dt/(dt + RC);
  derivative = a * derivative + (1-a) * pid.last_derivative;
  
  //store the derivative value
  pid.last_derivative = derivative;
  
  //compute the output ( proportional, integral, derivative )
  output = proportional + integral + derivative;
  
  //save last error
  pid.last_error = error;
  
  //return the computed value
  return output;  
}

/* Initialisation of the pid controller --------------- */
void pid_init(pid_controller* pid, float p, float i, float d)
{
  pid->Kp.floating_number = p;
  pid->Ki.floating_number = i;
  pid->Kd.floating_number = d;
  pid->sum_error = 0;
  pid->last_error = 0;
  pid->last_derivative = 0;
}

void pid_kp_set(pid_controller* pid, float p)
{
  pid->Kp.floating_number = p;
}

void pid_kd_set(pid_controller* pid, float d)
{
  pid->Kd.floating_number = d;
}

void pid_ki_set(pid_controller* pid, float i)
{
  pid->Ki.floating_number = i;
}

void pid_set(pid_controller* pid, float p, float i, float d)
{
  pid->Kp.floating_number = p;
  pid->Ki.floating_number = i;
  pid->Kd.floating_number = d;
}

/* End of file ---------------------------------------------------------------*/