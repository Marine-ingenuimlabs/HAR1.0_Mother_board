#include <stdint.h>


typedef union _3Data
{
  float floating_number;
  uint8_t integer[4];
}union_pid;

typedef struct
{
  union_pid Kp;
  union_pid Ki;
  union_pid Kd;
  float last_error;
  float sum_error;
  float last_derivative;
  
}pid_controller;


float pid_compute(float error,float dt, pid_controller pid);

void pid_init(pid_controller* pid, float Kp, float Ki, float Kd);

void pid_kp_set(pid_controller* pid, float p);

void pid_kd_set(pid_controller* pid, float d);

void pid_ki_set(pid_controller* pid, float i);