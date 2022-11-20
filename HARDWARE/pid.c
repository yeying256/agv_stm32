#include "pid.h"
#include "math.h"

#define ABS(x) ((x>0)?(x):(-x))




void abs_limit(float *a, float ABS_MAX)
{
	

	if(*a > ABS_MAX)
		*a = ABS_MAX;
	if(*a < -ABS_MAX)
		*a = -ABS_MAX;
}

static void PID_param_init(Pid_t *PID, uint32_t mode, uint32_t maxout, uint32_t intergral_limit, float kp, float ki, float kd)
{
	PID->IntegralLimit = intergral_limit;
	PID->MaxOutput = maxout;
	PID->PID_mode = mode;
	PID->p = kp;
	PID->i = ki;
	PID->d = kd;
}

static void PID_reset(Pid_t	*PID, float kp, float ki, float kd)
{
	PID->p = kp;
	PID->i = ki;
	PID->d = kd;
}

float PID_calc(Pid_t* PID, float get, float set)
{
	PID->get[NOW] = get;
	PID->set[NOW] = set;
	PID->err[NOW] = set - get;
	if (PID->max_err != 0 && ABS(PID->err[NOW]) > PID->max_err)
	return 0;
	if (PID->deadband != 0 && ABS(PID->err[NOW]) < PID->deadband)
	return 0;
	
	if(PID->PID_mode == POSITION_PID)
	{
		PID->pout  = PID->p * PID->err[NOW];
		PID->iout += PID->i * PID->err[NOW];
		PID->dout  = PID->d * (PID->err[NOW] - PID->err[LAST]);
		
		abs_limit(&(PID->iout), PID->IntegralLimit);
		PID->pos_out = PID->pout + PID->iout + PID->dout;
		abs_limit(&(PID->pos_out), PID->MaxOutput);
		PID->last_pos_out = PID->pos_out;
	}
	else if(PID->PID_mode == DELTA_PID)
	{
		PID->pout = PID->p * (PID->err[NOW] - PID->err[LAST]);
		PID->iout = PID->i * PID->err[NOW];
		PID->dout = PID->d * (PID->err[NOW] - 2*PID->err[LAST] + PID->err[LLAST]);
		
		abs_limit(&(PID->iout), PID->IntegralLimit);
		PID->delta_u = PID->pout + PID->iout + PID->dout;
		PID->delta_out = PID->last_delta_out + PID->delta_u;
		abs_limit(&(PID->delta_out), PID->MaxOutput);
		PID->last_delta_out = PID->delta_out;	//update last time
	}
	PID->err[LLAST] = PID->err[LAST];
	PID->err[LAST] = PID->err[NOW];
	PID->get[LLAST] = PID->get[LAST];
	PID->get[LAST] = PID->get[NOW];
	PID->set[LLAST] = PID->set[LAST];
	PID->set[LAST] = PID->set[NOW];
	return PID->PID_mode == POSITION_PID ? PID->pos_out : PID->delta_out;
}

float PID_sp_calc(Pid_t* PID, float get, float set, float gyro)
{
	PID->get[NOW] = get;
	PID->set[NOW] = set;
	PID->err[NOW] = set - get;	//set - measure

	if(PID->PID_mode == POSITION_PID)
	{
			PID->pout = PID->p * PID->err[NOW];
			if(fabs(PID->i) >= 0.001f)
				PID->iout += PID->i * PID->err[NOW];
			else
				PID->iout = 0;
			PID->dout = -PID->d * gyro/100.0f;	
			abs_limit(&(PID->iout), PID->IntegralLimit);
			PID->pos_out = PID->pout + PID->iout + PID->dout;
			abs_limit(&(PID->pos_out), PID->MaxOutput);
			PID->last_pos_out = PID->pos_out;	//update last time 
	}
	else if(PID->PID_mode == DELTA_PID)
	{
//        PID->pout = PID->p * (PID->err[NOW] - PID->err[LAST]);
//        PID->iout = PID->i * PID->err[NOW];
//        PID->dout = PID->d * (PID->err[NOW] - 2*PID->err[LAST] + PID->err[LLAST]);
//        
//        abs_limit(&(PID->iout), PID->IntegralLimit);
//        PID->delta_u = PID->pout + PID->iout + PID->dout;
//        PID->delta_out = PID->last_delta_out + PID->delta_u;
//        abs_limit(&(PID->delta_out), PID->MaxOutput);
//        PID->last_delta_out = PID->delta_out;	//update last time
	}
	PID->err[LLAST] = PID->err[LAST];
	PID->err[LAST] = PID->err[NOW];
	PID->get[LLAST] = PID->get[LAST];
	PID->get[LAST] = PID->get[NOW];
	PID->set[LLAST] = PID->set[LAST];
	PID->set[LAST] = PID->set[NOW];
	return PID->PID_mode==POSITION_PID ? PID->pos_out : PID->delta_out;
}

void PID_struct_init(Pid_t *PID, uint32_t mode, uint32_t maxout, uint32_t intergral_limit, float kp, float ki, float kd)
{
	PID->f_param_init = PID_param_init;
	PID->f_PID_reset = PID_reset;
	PID->f_param_init(PID, mode, maxout, intergral_limit, kp, ki, kd);
}
