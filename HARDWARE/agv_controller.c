

#include "agv_controller.h"

Chassis chassis;

float turn_bise[4] = {0,-0.04,0,0};

void chassis_init(float length,float width)
{
	chassis.Circle_rad=atan(length/width);
	chassis.wheel_R = 0.075;
	chassis.Wheel_w_vector[0].yaw=chassis.Circle_rad;
	chassis.Wheel_w_vector[1].yaw=-chassis.Circle_rad;
	chassis.Wheel_w_vector[2].yaw=chassis.Circle_rad;
	chassis.Wheel_w_vector[3].yaw=-chassis.Circle_rad;
	
	chassis.R_center2wheel = sqrt(pow((length/2),2)+pow((width/2),2));
	for(uint8_t i = 0;i<4;i++)
	{
		PID_struct_init(chassis.pid_wheel_v2e_+i,POSITION_PID,10000,1000,100,0,10);
		PID_struct_init(chassis.pid_wheel_deltap2v+i,POSITION_PID,2000,1000,65,0,0.01);
		
		PID_struct_init(chassis.pid_turn_outer_+i,POSITION_PID,2000,1000,10,0.01,0.01);
	ZL_Motor[i].position_d=0;
	
	}
	
	chassis.V_speed_limit = 7.0;
}

void chassis_control(float x_speed,float y_speed, float w_speed)
{
	//从中断中获取反馈
		
	
	//计算运动学
	float car_V = sqrt(x_speed*x_speed+y_speed*y_speed);
	if(car_V>chassis.V_speed_limit)
	{
		car_V = chassis.V_speed_limit;
	}
	float car_yaw=0;
	car_yaw = atan2(y_speed,x_speed);

//	temp_=car_yaw*Rad_to_deg;
	
	if(fabs(car_yaw)<0.1)
	{
		car_yaw=0;
	}
	if((car_yaw*Rad_to_deg)>90)
	{
		car_yaw -=180/Rad_to_deg;
		car_V = -car_V;
	}
	if((car_yaw*Rad_to_deg)<-90)
	{
		car_yaw+=180/Rad_to_deg;
		car_V = -car_V;
	}
	
	for(int i =0;i<4;i++)
	{
		chassis.Wheel[i].Velocity_vector.yaw = car_yaw;
		chassis.Wheel[i].Velocity_vector.v = car_V;
	}
	
	float temp_w_speed = w_speed*chassis.R_center2wheel;
	chassis.Wheel_w_vector[0].v = temp_w_speed;
	chassis.Wheel_w_vector[1].v = -temp_w_speed;
	chassis.Wheel_w_vector[2].v = -temp_w_speed;
	chassis.Wheel_w_vector[3].v = temp_w_speed;
	
	
	//temp_ = chassis.Wheel_w_vector[0].yaw;
	for(int i =0;i<4;i++)
	{
		chassis.Wheel[i].Velocity_vector = 
		vector_add(&(chassis.Wheel[i].Velocity_vector),
		&chassis.Wheel_w_vector[i]);
	}//向量相加
	
	vector2motorCMD(chassis.Wheel);
	
	float err_all = 0;
	for(int i = 0;i<4;i++)
	{
		if(TT_motor[i].position_d>3.1 || TT_motor[i].position_d<-3.1)
		{
		err_all += (fabs(TT_motor[i].position_d)-fabs(TT_motor[i].position));
		}
		else
		{
		err_all += fabs(TT_motor[i].position_d-TT_motor[i].position);
		
		}
		
	}
	float k_temp = 0;
	
	if(err_all<0.1)
	{
		k_temp = 100*(err_all-0.1)*(err_all-0.1);
	}else
	{
		k_temp =0;
	}
	static float temp[4] ={0,0,0,0};
//	temp_ = k_temp;
	for(uint8_t i=0;i<4;i++)
	{
		
		temp[i]+=k_temp*0.1*chassis.Wheel[i].Velocity_vector.v;
		PID_calc(&chassis.pid_wheel_deltap2v[i],ZL_Motor[i].position,ZL_Motor[i].position+k_temp*0.08*chassis.Wheel[i].Velocity_vector.v);
		chassis.Wheel[i].motor_wheel = PID_calc(&chassis.pid_wheel_v2e_[i],
																										ZL_Motor[i].vel,
																										chassis.pid_wheel_deltap2v[i].pos_out);
//		chassis.Wheel[i].motor_wheel = k_temp * PID_calc(&chassis.pid_wheel_v2e_[i],
//																										ZL_Motor[i].vel,
//																										chassis.Wheel[i].Velocity_vector.v);
		
		chassis.Wheel[i].motor_turn = chassis.Wheel[i].Velocity_vector.yaw;
		
		TT_motor[i].position_d = chassis.Wheel[i].motor_turn;
		ZL_Motor[i].tor_d = chassis.Wheel[i].motor_wheel;
	}
	
//	temp_ = TT_motor[1].position_d;
	//发送电机命令
	
	for(int i=0;i<4;i++)
	{
		TTset_TT_motor_pos(i);
	}
	ZLset_ZL_motor_tor();
}


Velocity_vector_type vector_add(Velocity_vector_type* a, Velocity_vector_type* b)
{
	Velocity_vector_type temp;
	float x = a->v *cos(a->yaw)+b->v * cos(b->yaw);
	float y = a->v *sin(a->yaw)+b->v * sin(b->yaw);
	temp.v = sqrt(x*x+y*y);
	temp.yaw = atan2(y,x);
	
	return temp;
}

void vector2motorCMD(wheel_control_type *Wheel)
{
	Wheel[0].Velocity_vector.v = -Wheel[0].Velocity_vector.v/chassis.wheel_R;
	
	Wheel[1].Velocity_vector.v = Wheel[1].Velocity_vector.v/chassis.wheel_R;
	Wheel[2].Velocity_vector.v = Wheel[2].Velocity_vector.v/chassis.wheel_R;
	Wheel[3].Velocity_vector.v = -Wheel[3].Velocity_vector.v/chassis.wheel_R;
	
	for(uint8_t i=0;i<4;i++)
	{
		Wheel[i].Velocity_vector.yaw = -Wheel[i].Velocity_vector.yaw +turn_bise[i] ;
	}
}


