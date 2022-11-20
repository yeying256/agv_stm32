#ifndef PID_H
#define PID_H
#include "stm32f4xx_hal.h"

enum {LLAST	= 0, LAST = 1, NOW = 2, POSITION_PID, DELTA_PID};

typedef struct PID_T
{
	float p,i,d;
	float set[3],get[3],err[3];
	float pout,iout,dout;
	float pos_out,last_pos_out,delta_u,delta_out,last_delta_out;
	float max_err,deadband;
	uint32_t PID_mode,MaxOutput,IntegralLimit;
	void (*f_param_init)(struct PID_T *PID, uint32_t PID_mode, uint32_t maxOutput, uint32_t integralLimit, float p, float i, float d);
	void (*f_PID_reset)(struct PID_T *PID, float p, float i, float d);
}Pid_t;

void PID_struct_init(Pid_t* PID, uint32_t mode, uint32_t maxout, uint32_t intergral_limit, float kp, float ki, float kd);
float PID_calc(Pid_t* PID, float get, float set);

#endif
