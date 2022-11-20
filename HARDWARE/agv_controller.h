#ifndef AGV_CONTROLLER_H
#define AGV_CONTROLLER_H

#include "stm32f4xx_hal.h"
#include "can.h"
#include <math.h>
#include "DJI_Motor.h"

#define PI 3.14159265359
#define Rad_to_deg (180/PI)

typedef struct{
	float v;//m/s
	float yaw;//rad/s
}Velocity_vector_type;

typedef struct{
	int32_t motor_wheel;
	float motor_turn;
	Velocity_vector_type Velocity_vector;
	Velocity_vector_type Velocity_vector_last;
}wheel_control_type;

typedef struct{
float Circle_rad;//正方形就是45度的弧度制
	Velocity_vector_type Wheel_w_vector[4];
	wheel_control_type Wheel[4];
	Pid_t pid_wheel_v2e_[4];
	Pid_t pid_wheel_deltap2v[4];
	Pid_t pid_turn_outer_[4];
	float wheel_R;
	float R_center2wheel;
	float V_speed_limit;
	


}Chassis;





extern Chassis chassis;

void chassis_init(float length,float width);
void chassis_control(float x_speed,float y_speed, float w_speed);
Velocity_vector_type vector_add(Velocity_vector_type* a, Velocity_vector_type* b);
void vector2motorCMD(wheel_control_type *Wheel);










#endif