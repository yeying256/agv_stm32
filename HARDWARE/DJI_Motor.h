#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H
#include "pid.h"
#include "can.h"
#include "stm32f4xx_hal.h"


#define TT_motorEcode 3309568


extern int motor_init_flag;




typedef struct
{
	int16_t	 	speed_rpm;
  int16_t  	given_current;
  uint8_t  	hall;					//温度
	uint16_t 	angle;				//abs angle range:[0,8191]
	uint16_t 	last_angle; 	//abs angle range:[0,8191]
	uint16_t	offset_angle;	//转子的偏移角度
	int32_t		round_cnt;		//圈数
	int32_t		total_angle;	//当前的总角度
	uint32_t  msg_cnt;			//计数的
}DJI_Motor_t;





typedef struct
{
	float	 speed_rpm;
  int16_t  	current;			//mA

	int32_t 	ecode;				//abs angle range:[0,4096]
	int32_t 	last_ecode; 	//abs angle range:[0,4096]
	int32_t  msg_cnt;			//计数的
	float vel;//当前速度
	float position;
	float torque;
	float tor_d;
	float position_d;
	float vel_d;
	int32_t int_vel;
	
}ZL_Motor_t;




typedef struct
{
	float speed_now;
	float position;
	float vel;
	float torque;
	float tor_d;
	float position_d;
	int32_t ecode;
	int32_t int_vel;
}TT_Motor_t;



extern DJI_Motor_t DJI_Motor[2];
extern Pid_t Motor_Speed_Loop[2];
extern Pid_t Motor_Position_Loop[2];

extern Pid_t ZL_Motor_pid_Speed[4];
extern Pid_t ZL_Motor_pid_Position[4];


extern ZL_Motor_t ZL_Motor[4];
extern TT_Motor_t TT_motor[4];

void CAN_Filter_Init(void);		//CAN滤波器的初始化函数
void Get_Motor_Measure(DJI_Motor_t *ptr,uint8_t* RxMessage);//获得电机的原始参数
void Get_Motor_Offset(DJI_Motor_t *ptr,uint8_t* RxMessage);//计算转自角度偏移量
void get_total_angle(DJI_Motor_t *p);//计算总角度
void Set_One_Current(int16_t C,uint16_t ID);
void Set_Four_Current(int16_t C1, int16_t C2, int16_t C3, int16_t C4,uint16_t ID);
void One_Motor_Speed_Loop_Putout(Pid_t *p,uint16_t ID);
void Four_Motor_Speed_Loop_Putout(Pid_t *p1,Pid_t *p2,Pid_t *p3,Pid_t *p4,uint16_t ID);


void ZL_Motor_Init(void);
void Set_ZL_Current(int left_mA,int right_mA,uint16_t ID);



void TTecode2pos(const int32_t* ecode,double* pos);
void TTset_position_now(int32_t ecode,TT_Motor_t* TT_Motor_ptr);
void TTset_vel_now(int32_t vel,TT_Motor_t* TT_Motor_ptr);//转向电机反馈
void TTset_TT_motor_pos(int motor_id);//
int32_t TTpos2ecode(TT_Motor_t* TT_Motor_ptr);




void ZLset_position_now(int32_t ecode,ZL_Motor_t* ZL_Motor_ptr);
void ZLset_vel_now(int32_t vel,ZL_Motor_t* ZL_Motor_ptr);

void ZLset_ZL_motor_tor();//两个电机共用一个id，故直接在程序中写

void motor_init();



#endif
