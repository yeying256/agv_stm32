#include "DJI_Motor.h"
#include "pid.h"


uint16_t cantestflag=0;

Pid_t Motor_Speed_Loop[2]={0};
Pid_t Motor_Position_Loop[2]={0};

DJI_Motor_t DJI_Motor[2];
ZL_Motor_t ZL_Motor[4];
TT_Motor_t TT_motor[4];



Pid_t ZL_Motor_pid_Speed[4];
Pid_t ZL_Motor_pid_Position[4];
int motor_init_flag = 0;

extern CAN_HandleTypeDef hcan1;

void CAN_Filter_Init(void)
{
	CAN_FilterTypeDef CAN_Filter;
	CAN_Filter.FilterActivation = ENABLE;
	CAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_Filter.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_Filter.FilterIdHigh = 0x0000;
	CAN_Filter.FilterIdLow = 0x0000;
	CAN_Filter.FilterMaskIdHigh = 0x0000;
	CAN_Filter.FilterMaskIdLow = 0x0000;
	CAN_Filter.FilterBank = 0;
	CAN_Filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	HAL_CAN_ConfigFilter(&hcan1,&CAN_Filter);
	//上面的不用管他，配置筛选器和掩码，上面的那些是全部通过
	
	//初始化 没有使能不能用，cubemx生成的代码没有使用使能函数，需要自己加
	HAL_CAN_Start(&hcan1);										//给CAN使能
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);//使能fifo接收数据
}


uint8_t tmp_motor = 0;
//接收中断，跟外部中断差不多
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//所有CAN的接收中断hcan
{
	cantestflag++;
	if(hcan->Instance==CAN1)
	{
		uint8_t CAN_RXMessage[8];
		
		CAN_RxHeaderTypeDef CAN_RxHeaderStructure;
		
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&CAN_RxHeaderStructure,CAN_RXMessage);
		cantestflag++;
		int ecode_tmp,vel_tmp = 0;
		switch(CAN_RxHeaderStructure.StdId)
		{
//			case 0x381://判断标识符canID电机电流
//			cantestflag++;
//			//左电机电流
//			ZL_Motor[0].current=((int16_t)CAN_RXMessage[1]<<8 |  CAN_RXMessage[0])*100;//单位0.1A
//			ZL_Motor[1].current=((int16_t)CAN_RXMessage[3]<<8 |	CAN_RXMessage[2])*100;//单位0.1A
//			break;
//			case 0x281://判断电机速度
//			ZL_Motor[0].speed_rpm=((int32_t)CAN_RXMessage[0] | CAN_RXMessage[1]<<8 | CAN_RXMessage[2]<<16 | CAN_RXMessage[3]<<24)/10.0;
//			ZL_Motor[1].speed_rpm=((int32_t)CAN_RXMessage[4]	| CAN_RXMessage[5]<<8 | CAN_RXMessage[6]<<16 | CAN_RXMessage[7]<<24)/10.0;
//			break;
			
				case 0x181:	//turn1
					TT_motor[0].ecode = (((int32_t)CAN_RXMessage[3])<<24) | (((int32_t)CAN_RXMessage[2])<<16) | (((int32_t)CAN_RXMessage[1])<<8) | ((int32_t)CAN_RXMessage[0]);
					TT_motor[0].int_vel = (int32_t)(CAN_RXMessage[7]<<24) | (int32_t)(CAN_RXMessage[6]<<16) | (int32_t)(CAN_RXMessage[5]<<8) | (int32_t)(CAN_RXMessage[4]);
				TTset_position_now(TT_motor[0].ecode,TT_motor);
				TTset_vel_now(TT_motor[0].int_vel,TT_motor);
				tmp_motor = CAN_RXMessage[3];
				break;
				case 0x182:	//turn2
					TT_motor[1].ecode = (int32_t)(CAN_RXMessage[3]<<24) | (int32_t)(CAN_RXMessage[2]<<16) | (int32_t)(CAN_RXMessage[1]<<8) |(int32_t)(CAN_RXMessage[0]);
					TT_motor[1].int_vel = (int32_t)(CAN_RXMessage[7]<<24) | (int32_t)(CAN_RXMessage[6]<<16) | (int32_t)(CAN_RXMessage[5]<<8) | (int32_t)(CAN_RXMessage[4]);
				TTset_position_now(TT_motor[1].ecode,TT_motor+1);
				TTset_vel_now(TT_motor[1].int_vel,TT_motor+1);
				break;
				case 0x183:	//turn3
					TT_motor[2].ecode = (int32_t)(CAN_RXMessage[3]<<24) | (int32_t)(CAN_RXMessage[2]<<16) | (int32_t)(CAN_RXMessage[1]<<8) |(int32_t)(CAN_RXMessage[0]);
					TT_motor[2].int_vel = (int32_t)(CAN_RXMessage[7]<<24) | (int32_t)(CAN_RXMessage[6]<<16) | (int32_t)(CAN_RXMessage[5]<<8) | (int32_t)(CAN_RXMessage[4]);
				TTset_position_now(TT_motor[2].ecode,TT_motor+2);
				TTset_vel_now(TT_motor[2].int_vel,TT_motor+2);
				break;
				case 0x184:	//turn4
					TT_motor[3].ecode = (int32_t)(CAN_RXMessage[3]<<24) | (int32_t)(CAN_RXMessage[2]<<16) | (int32_t)(CAN_RXMessage[1]<<8) |(int32_t)(CAN_RXMessage[0]);
					TT_motor[3].int_vel = (int32_t)(CAN_RXMessage[7]<<24) | (int32_t)(CAN_RXMessage[6]<<16) | (int32_t)(CAN_RXMessage[5]<<8) | (int32_t)(CAN_RXMessage[4]);
				TTset_position_now(TT_motor[3].ecode,TT_motor+3);
				TTset_vel_now(TT_motor[3].int_vel,TT_motor+3);
				break;
				case 0x185: //wheel	1 2
					ZL_Motor[1].ecode= (((int32_t)(CAN_RXMessage[3]))<<24 )| (((int32_t)CAN_RXMessage[2])<<16) | (((int32_t)CAN_RXMessage[1])<<8) | ((int32_t)(CAN_RXMessage[0]));
					ZL_Motor[0].ecode = (((int32_t)(CAN_RXMessage[7]))<<24 )| (((int32_t)CAN_RXMessage[6])<<16) | (((int32_t)CAN_RXMessage[5])<<8) | ((int32_t)(CAN_RXMessage[4]));
				
					ZLset_position_now(ZL_Motor[0].ecode,ZL_Motor);
					ZLset_position_now(ZL_Motor[1].ecode,ZL_Motor+1);
					
				break;
				case 0x285://wheel 1 2
					ZL_Motor[1].int_vel=(int32_t)(CAN_RXMessage[3]<<24)|(int32_t)(CAN_RXMessage[2]<<16)|(int32_t)(CAN_RXMessage[1]<<8)|(int32_t)(CAN_RXMessage[0]);
					ZL_Motor[0].int_vel=(int32_t)(CAN_RXMessage[7]<<24)|(int32_t)(CAN_RXMessage[6]<<16)|(int32_t)(CAN_RXMessage[5]<<8)|(int32_t)(CAN_RXMessage[4]);
					ZLset_vel_now(ZL_Motor[0].int_vel,ZL_Motor);
					ZLset_vel_now(ZL_Motor[1].int_vel,ZL_Motor+1);
				break;
				case 0x186://wheel 3 4
					ZL_Motor[2].ecode= (int32_t)(CAN_RXMessage[3]<<24) | (int32_t)(CAN_RXMessage[2]<<16) | (int32_t)(CAN_RXMessage[1]<<8) |(int32_t)(CAN_RXMessage[0]);
					ZL_Motor[3].ecode = (int32_t)(CAN_RXMessage[7]<<24) | (int32_t)(CAN_RXMessage[6]<<16) | (int32_t)(CAN_RXMessage[5]<<8) | (int32_t)(CAN_RXMessage[4]);
					ZLset_position_now(ZL_Motor[2].ecode,ZL_Motor+2);
					ZLset_position_now(ZL_Motor[3].ecode,ZL_Motor+3);
				break;
				case 0x286://wheel 3 4
					ZL_Motor[2].int_vel=(int32_t)(CAN_RXMessage[3]<<24)|(int32_t)(CAN_RXMessage[2]<<16)|(int32_t)(CAN_RXMessage[1]<<8)|(int32_t)(CAN_RXMessage[0]);
					ZL_Motor[3].int_vel=(int32_t)(CAN_RXMessage[7]<<24)|(int32_t)(CAN_RXMessage[6]<<16)|(int32_t)(CAN_RXMessage[5]<<8)|(int32_t)(CAN_RXMessage[4]);
					ZLset_vel_now(ZL_Motor[2].int_vel,ZL_Motor+2);
					ZLset_vel_now(ZL_Motor[3].int_vel,ZL_Motor+3);
				break;
				default:
				break;

//			case 0x202:
//			{
//				static uint8_t i;
//				i = CAN_RxHeaderStructure.StdId - 1;
//				DJI_Motor[i].msg_cnt++ <= 50	?	Get_Motor_Offset(&DJI_Motor[i],CAN_RXMessage) : Get_Motor_Measure(&DJI_Motor[i],CAN_RXMessage);
//			}
		}
	}
}

void Get_Motor_Measure(DJI_Motor_t *ptr,uint8_t* RxMessage)
{
	ptr->last_angle = ptr->angle;
	//0是高8位       
	ptr->angle = (uint16_t)(RxMessage[0]<<8 | RxMessage[1]);//角度无符号
	ptr->speed_rpm  = (int16_t)(RxMessage[2]<<8 | RxMessage[3]);//有符号，也可以移位
	ptr->given_current = (int16_t)(RxMessage[4]<<8 | RxMessage[5]);
	ptr->hall = RxMessage[6];
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

void Get_Motor_Offset(DJI_Motor_t *ptr,uint8_t* RxMessage)	//设置偏移量
{
	ptr->angle = (uint16_t)(RxMessage[0]<<8 | RxMessage[1]);	
	ptr->offset_angle = ptr->angle;
}

#define ABS(x)	( (x>0) ? (x) : (-x) )
//*@bref 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
void get_total_angle(DJI_Motor_t *p)
{
	int res1, res2, delta;
	if(p->angle < p->last_angle)
		{
			//可能的情况
		  res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
		  res2 = p->angle - p->last_angle;				//反转	delta=-
	  }else
		{
			//angle > last
		  res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
		  res2 = p->angle - p->last_angle;				//正转	delta +
	  }
		//不管正反转，肯定是转的角度小的那个是真的
	  if(ABS(res1)<ABS(res2))
			delta = res1;
	  else
		  delta = res2;
		
		p->total_angle += delta;
		p->last_angle = p->angle;
}

void Set_One_Current(int16_t C,uint16_t ID)
{
	uint8_t Data[2]={0};
	uint32_t pTxMailbox=0;
  CAN_TxHeaderTypeDef TxHeader;
  TxHeader.StdId = ID;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 0x02;
  Data[0] = C >> 8;
  Data[1] = C;
	HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
}

void Set_Four_Current(int16_t C1, int16_t C2, int16_t C3, int16_t C4,uint16_t ID)
{
	uint8_t Data[8]={0};
	uint32_t pTxMailbox=0;
  CAN_TxHeaderTypeDef TxHeader;
  TxHeader.StdId = ID;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 0x08;
  Data[0] = C1 >> 8;
  Data[1] = C1;
  Data[2] = C2 >> 8;
  Data[3] = C2;
  Data[4] = C3 >> 8;
  Data[5] = C3;
  Data[6] = C4 >> 8;
  Data[7] = C4;
	HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
}

void ZL_Motor_Init(void)
{
	uint8_t Data[8]={0};
	uint32_t pTxMailbox=0;
	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId=0x601;
	TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 0x08;
	//2B 40 60 00 00 00 00 00
	//2B 40 60 00 06 00 00 00
	//2B 40 60 00 07 00 00 00
	//2B 40 60 00 0F 00 00 00
	Data[0]=0x2B;
	Data[1]=0x40;
	Data[2]=0x60;
	Data[3]=0x00;
	Data[4]=0x00;
	Data[5]=0x00;
	Data[6]=0x00;
	Data[7]=0x00;
	HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
	HAL_Delay(2);
	Data[4]=0x06;
	HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
	HAL_Delay(2);
	Data[4]=0x07;
	HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
	HAL_Delay(2);
	Data[4]=0x0F;
	HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
	//电机初始化
	HAL_Delay(2);

//	设置异步模式
//2B 0F 20 00 00 00 00 00
	Data[0]=0x2B;
	Data[1]=0x0F;
	Data[2]=0x20;
	Data[3]=0x00;
	Data[4]=0x00;
	Data[5]=0x00;
	Data[6]=0x00;
	Data[7]=0x00;
	HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
	HAL_Delay(2);
//设置为电流模式
//2F 60 60 00 04 00 00 00
	Data[0]=0x2F;
	Data[1]=0x60;
	Data[2]=0x60;
	Data[3]=0x00;
	Data[4]=0x04;
	Data[5]=0x00;
	Data[6]=0x00;
	Data[7]=0x00;
	HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
	HAL_Delay(2);
	//电机设置为发送反馈
	uint8_t Data1[2];
	TxHeader.StdId=0x000;
	Data1[0]=0x01;
	Data1[1]=0x01;
	TxHeader.DLC=0x02;
	HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data1,&pTxMailbox);
//	cantestflag++;
	//PID_struct_init(Pid_t *PID, uint32_t mode, uint32_t maxout,
//	uint32_t intergral_limit, float kp, float ki, float kd)

	for(int i =0;i<4;i++)
	{
		
		//void PID_struct_init(Pid_t *PID, uint32_t mode, uint32_t maxout, uint32_t intergral_limit, float kp, float ki, float kd)
	PID_struct_init(&ZL_Motor_pid_Position[i],DELTA_PID,3000,300,40,0.002,0);
	PID_struct_init(&ZL_Motor_pid_Speed[i],POSITION_PID,3000,300,1000,0.002,0);
	}
	
	
	motor_init_flag = 1;
}

void motor_init()
{
	uint8_t Data[8]={0};
	uint32_t pTxMailbox=0;
  CAN_TxHeaderTypeDef TxHeader;
  TxHeader.StdId = 0;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 0x02;
	
	Data[0]=0x10;
	Data[1]=0x10;
	HAL_Delay(5000);
	for(int i=0;i<4;i++)
	{
		TxHeader.StdId = i+1;
		HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
		HAL_Delay(300);
	}//转向电机启动
	

	
	
	
	Data[0]=0x2f;
	Data[1]=0x60;
	Data[2]=0x60;
	Data[3]=0x00;
	Data[4]=0x01;
	Data[5]=0x00;
	Data[6]=0x00;
	Data[7]=0x00;
	TxHeader.DLC = 0x08;//位置操作模式转向电机
	for(int i=0;i<4;i++)
	{
		TxHeader.StdId = 0x601+i;
		HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
		HAL_Delay(300);
	}
	
	Data[0]=0x23;
	Data[1]=0x7a;
	Data[2]=0x60;
	Data[3]=0x00;
	Data[4]=0x00;
	Data[5]=0x00;
	Data[6]=0x00;
	Data[7]=0x00;//位置初始化
	for(int i=0;i<4;i++)
	{
		TxHeader.StdId = 0x601+i;
		HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
		HAL_Delay(300);
	}
	
	Data[0]=0x23;
	Data[1]=0x81;
	Data[2]=0x60;
	Data[3]=0x00;
	Data[4]=0x20;
	Data[5]=0x4e;
	Data[6]=0x00;
	Data[7]=0x00;//速度20000转
	for(int i=0;i<4;i++)
	{
		TxHeader.StdId = 0x601+i;
		HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
		HAL_Delay(300);
	}
	
	Data[0]=0x2b;
	Data[1]=0x40;
	Data[2]=0x60;
	Data[3]=0x00;
	Data[4]=0x80;
	Data[5]=0x00;
	Data[6]=0x00;
	Data[7]=0x00;//清楚报警
	for(int i=0;i<4;i++)
	{
		TxHeader.StdId = 0x601+i;
		HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
		HAL_Delay(300);
	}
	
	Data[0]=0x2b;
	Data[1]=0x40;
	Data[2]=0x60;
	Data[3]=0x00;
	Data[4]=0x06;
	Data[5]=0x00;
	Data[6]=0x00;
	Data[7]=0x00;//伺服准备
	for(int i=0;i<4;i++)
	{
		TxHeader.StdId = 0x601+i;
		HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
		HAL_Delay(300);
	}
	
	Data[0]=0x2b;
	Data[1]=0x40;
	Data[2]=0x60;
	Data[3]=0x00;
	Data[4]=0x07;
	Data[5]=0x00;
	Data[6]=0x00;
	Data[7]=0x00;//等待伺服使能
	for(int i=0;i<4;i++)
	{
		TxHeader.StdId = 0x601+i;
		HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
		HAL_Delay(300);
	}
	
	
	Data[0]=0x2b;
	Data[1]=0x40;
	Data[2]=0x60;
	Data[3]=0x00;
	Data[4]=0x2f;
	Data[5]=0x00;
	Data[6]=0x00;
	Data[7]=0x00;//立即更新命令1
	for(int i=0;i<4;i++)
	{
		TxHeader.StdId = 0x601+i;
		HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
		HAL_Delay(300);
	}
	
	
	Data[0]=0x2b;
	Data[1]=0x40;
	Data[2]=0x60;
	Data[3]=0x00;
	Data[4]=0x3f;
	Data[5]=0x00;
	Data[6]=0x00;
	Data[7]=0x00;//立即更新命令2
	for(int i=0;i<4;i++)
	{
		TxHeader.StdId = 0x601+i;
		HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
		HAL_Delay(300);
	}
	
	
	
	
	
	
	///////////////////行走电机齿梳化程序
	
	Data[0]=0x2b;
	Data[1]=0x40;
	Data[2]=0x60;
	Data[3]=0x00;
	Data[4]=0x00;
	Data[5]=0x00;
	Data[6]=0x00;
	Data[7]=0x00;//初始化
	for(int i=0;i<2;i++)
	{
		TxHeader.StdId = 0x605+i;
		HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
		HAL_Delay(300);
	}
	
	Data[0]=0x2b;
	Data[1]=0x40;
	Data[2]=0x60;
	Data[3]=0x00;
	Data[4]=0x06;
	Data[5]=0x00;
	Data[6]=0x00;
	Data[7]=0x00;//初始化
	for(int i=0;i<2;i++)
	{
		TxHeader.StdId = 0x605+i;
		HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
		HAL_Delay(300);
	}
	
	
	Data[0]=0x2b;
	Data[1]=0x40;
	Data[2]=0x60;
	Data[3]=0x00;
	Data[4]=0x07;
	Data[5]=0x00;
	Data[6]=0x00;
	Data[7]=0x00;//初始化
	for(int i=0;i<2;i++)
	{
		TxHeader.StdId = 0x605+i;
		HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
		HAL_Delay(300);
	}
	
	Data[0]=0x2b;
	Data[1]=0x40;
	Data[2]=0x60;
	Data[3]=0x00;
	Data[4]=0x0f;
	Data[5]=0x00;
	Data[6]=0x00;
	Data[7]=0x00;//初始化
	for(int i=0;i<2;i++)
	{
		TxHeader.StdId = 0x605+i;
		HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
		HAL_Delay(300);
	}
	
	
	Data[0]=0x2b;
	Data[1]=0x0f;
	Data[2]=0x20;
	Data[3]=0x00;
	Data[4]=0x00;
	Data[5]=0x00;
	Data[6]=0x00;
	Data[7]=0x00;//异步模式
	for(int i=0;i<2;i++)
	{
		TxHeader.StdId = 0x605+i;
		HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
		HAL_Delay(300);
	}
	
	Data[0]=0x2f;
	Data[1]=0x60;
	Data[2]=0x60;
	Data[3]=0x00;
	Data[4]=0x04;
	Data[5]=0x00;
	Data[6]=0x00;
	Data[7]=0x00;//电流模式
	for(int i=0;i<2;i++)
	{
		TxHeader.StdId = 0x605+i;
		HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
		HAL_Delay(300);
	}
	
	Data[0]=0x2b;
	Data[1]=0x30;
	Data[2]=0x20;
	Data[3]=0x08;
	Data[4]=0x00;
	Data[5]=0x00;
	Data[6]=0x00;
	Data[7]=0x00;//右电机解锁抱闸
	for(int i=0;i<2;i++)
	{
		TxHeader.StdId = 0x605+i;
		HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
		HAL_Delay(300);
	}
	
	
	Data[0]=0x2b;
	Data[1]=0x30;
	Data[2]=0x20;
	Data[3]=0x07;
	Data[4]=0x00;
	Data[5]=0x00;
	Data[6]=0x00;
	Data[7]=0x00;//左电机解锁抱闸
	for(int i=0;i<2;i++)
	{
		TxHeader.StdId = 0x605+i;
		HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
		HAL_Delay(300);
	}
	
	Data[0]=0x01;
	Data[1]=0x00;
	TxHeader.DLC = 0x02;
	TxHeader.StdId = 0;
	HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
	//所有电机使能
	HAL_Delay(300);
	
	
	motor_init_flag = 1;
	
	
	
	

}

void Set_ZL_Current(int left_mA,int right_mA,uint16_t ID)
{
	//2B 71 60 01 E8 03 00 00
	uint8_t Data[8]={0};
	uint32_t pTxMailbox=0;
  CAN_TxHeaderTypeDef TxHeader;
  TxHeader.StdId = ID;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 0x08;
	Data[0]=0x2B;
	Data[1]=0x71;
	Data[2]=0x60;
	Data[3]=0x01;
	Data[4]=left_mA;
	Data[5]=left_mA >> 8;
	
	HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
	
	Data[3]=0x02;
	Data[4]=right_mA;
	Data[5]=right_mA >> 8;
	
	HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
	
}

void TTset_position_now(int32_t ecode,TT_Motor_t* TT_Motor_ptr)
{
	float tmp = TT_motorEcode;
	TT_Motor_ptr->position=((float)ecode/tmp)*2.0*3.1415926;
}


void TTset_vel_now(int32_t vel,TT_Motor_t* TT_Motor_ptr)
{
	TT_Motor_ptr->vel=(float)vel/0.1*2*3.1415926/60;//转/min-》rad/s
}

int _tmp2=0;
float tmp3_=0;
void TTset_TT_motor_pos(int motor_id)//这个id
{
	uint8_t Data[8];
	int32_t ecode = TTpos2ecode(TT_motor+motor_id);//电机是0123
	if(motor_id==1)
	{
		_tmp2 = ecode;
		tmp3_ = (TT_motor+1)->position_d;
	}
	uint32_t pTxMailbox=0;
  CAN_TxHeaderTypeDef TxHeader;
  TxHeader.StdId = motor_id+0x601;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 0x08;
	Data[0]=0x23;
	Data[1]=0x7a;
	Data[2]=0x60;
	Data[3]=0x00;
	Data[4] = ecode;
	Data[5] = ecode>>8;
	Data[6] = ecode>>16;
	Data[7] = ecode>>24;
//	Data[7]=*((uint8_t*)(&ecode));
//	Data[6]=*((uint8_t*)(&ecode)+1);
//	Data[5]=*((uint8_t*)(&ecode)+2);
//	Data[4]=*((uint8_t*)(&ecode)+3);

	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)!=3);
	HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
	
	
	Data[0]=0x2b;
	Data[1]=0x40;
	Data[2]=0x60;
	Data[3]=0x00;
	Data[4]=0x2f;
	Data[5]=0x00;
	Data[6]=0x00;
	Data[7]=0x00;//立即更新命令1
	TxHeader.StdId = 0x601+motor_id;
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0);
	HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
	
	
	Data[0]=0x2b;
	Data[1]=0x40;
	Data[2]=0x60;
	Data[3]=0x00;
	Data[4]=0x3f;
	Data[5]=0x00;
	Data[6]=0x00;
	Data[7]=0x00;//立即更新命令2
	TxHeader.StdId = 0x601+motor_id;
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0);
	HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);
}



int32_t TTpos2ecode(TT_Motor_t* TT_Motor_ptr)
{
	return ((int32_t)(TT_Motor_ptr->position_d/(2*3.1415926)*TT_motorEcode));
}






void ZLset_position_now(int32_t ecode,ZL_Motor_t* ZL_Motor_ptr)
{
	ZL_Motor_ptr->position = ((float)ecode)/4096*2*3.1415926;
}
void ZLset_vel_now(int32_t vel,ZL_Motor_t* ZL_Motor_ptr)
{
	ZL_Motor_ptr->vel = (float)vel*0.1*2*3.1415926;
}

int16_t cmd_tor1=0;
int data_tor1 = 0;
void ZLset_ZL_motor_tor()
{
	uint8_t Data[8];
	uint32_t pTxMailbox=0;
  CAN_TxHeaderTypeDef TxHeader;
  TxHeader.StdId = 0x605;//
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 0x08;
	
	int16_t tor_tmp = (int16_t)ZL_Motor[0].tor_d;
	cmd_tor1=tor_tmp;
	Data[0] = 0x2b;
	Data[1] = 0x71;
	Data[2] = 0x60;
	Data[3] = 0x02;//右侧电机 1号
//	Data[4] = *((uint8_t*)(&tor_tmp)+0);
//	Data[5] = *((uint8_t*)(&tor_tmp)+1);
//	Data[5] = *((uint8_t*)(&tor_tmp)+1);
//	Data[4] = *((uint8_t*)(&tor_tmp)+0);
//	Data[7] = tor_tmp>>24;
//	Data[6] = tor_tmp>>16;
	Data[5] = tor_tmp>>8;
	Data[4] = tor_tmp;
	
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0);
	HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);//1号电机 右侧

	tor_tmp = (int16_t)ZL_Motor[1].tor_d;
	Data[3] = 0x01;//左侧电机 2号
//	Data[7] = tor_tmp>>24;
//	Data[6] = tor_tmp>>16;
	Data[5] = tor_tmp>>8;
	Data[4] = tor_tmp;
//	Data[4] = *((uint8_t*)(&tor_tmp)+0);
//	Data[5] = *((uint8_t*)(&tor_tmp)+1);

	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0);
	HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);//2号电机 左侧

	
	
	tor_tmp = (int16_t)ZL_Motor[2].tor_d;
	TxHeader.StdId = 0x606;
	Data[3] = 0x01;//左侧电机 3号
//	Data[7] = tor_tmp>>24;
//	Data[6] = tor_tmp>>16;
	Data[5] = tor_tmp>>8;
	Data[4] = tor_tmp;

//	Data[4] = *((uint8_t*)(&tor_tmp)+0);
//	Data[5] = *((uint8_t*)(&tor_tmp)+1);
	
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0);
	HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);//3号电机 左侧

	
	tor_tmp = (int16_t)ZL_Motor[3].tor_d;
	Data[3] = 0x02;//右侧电机 4号
//	Data[7] = tor_tmp>>24;
//	Data[6] = tor_tmp>>16;
	Data[5] = tor_tmp>>8;
	Data[4] = tor_tmp;

//	Data[4] = *((uint8_t*)(&tor_tmp)+0);
//	Data[5] = *((uint8_t*)(&tor_tmp)+1);
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0);
	HAL_CAN_AddTxMessage(&hcan1,&TxHeader,Data,&pTxMailbox);//4号电机 右侧

}


