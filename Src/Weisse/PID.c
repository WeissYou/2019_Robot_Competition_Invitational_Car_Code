/*****************************************************************************
  * 文件名称: PID.c
  * 作    者: 尤伟宏
  * 编写日期: 二月 2019
  * 功    能: PID算法
  ****************************************************************************
  * 说    明: 
  * 
  *****************************************************************************/
	
/* 包含头文件 ----------------------------------------------------------------*/
#include "PID.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
#define  P_DATA      0.725f                              // P参数
#define  I_DATA      0.10f                              // I参数
#define  D_DATA      0.10f                              // D参数
/* 私有变量 ------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* PID结构体指针 */
PID_TypeDef  sPID;
PID_TypeDef  *ptr =  &sPID;
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/*************************************
  * 函数功能: PID参数初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  ************************************/
void IncPIDInit(void) 
{
	uint8_t i;
	for(i = 0; i < 4; i ++)
	{
	  ptr->SetPoint[i] = 499;   //设定目标Desired Value
    ptr->LastError[i] = 0;            //Error[-1]
    ptr->PrevError[i] = 0;            //Error[-2]
    ptr->Proportion[i] = P_DATA;      //比例常数 Proportional Const
    ptr->Integral[i] = I_DATA;        //积分常数  Integral Const
    ptr->Derivative[i] = D_DATA;      //微分常数 Derivative Const
	}
}

/*************************************
  * 函数功能: PID计算
  * 输入参数: uint8_t Motor_Number, int16_t NextPoint
  * 返 回 值: ptr->Proportion[Motor_Number] * iError //比例项
  + ptr->Integral[Motor_Number] * ptr->SumError[Motor_Number] //积分项
  + ptr->Derivative[Motor_Number] * dError  //微分项
  * 说    明: 无
  ************************************/
int32_t LocPIDCalc(uint8_t Motor_Number, int16_t NextPoint)
{
  int iError,dError;
  iError = ptr->SetPoint[Motor_Number] - NextPoint; //偏差
  if((iError<3 )&& (iError>-3))
    iError = 0;
  ptr->SumError[Motor_Number] += iError; //积分
  dError = iError - ptr->LastError[Motor_Number]; //微分
  ptr->LastError[Motor_Number] = iError;
  return(ptr->Proportion[Motor_Number] * iError //比例项
  + ptr->Integral[Motor_Number] * ptr->SumError[Motor_Number] //积分项
  + ptr->Derivative[Motor_Number] * dError); //微分项
}

/*************************************
  * 函数功能: PID函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: PID执行过程封装
  ************************************/
void PID_Function(void)
{
	uint8_t i;
	int16_t Temp[4];  //待判断方向的PWM_Duty

	/* 转存码盘计数 */
	Encoder_CaptureNumber[0] = __HAL_TIM_GET_COUNTER(&htim2) + Encoder_OverflowCount[0]*65536;
	Encoder_CaptureNumber[1] = __HAL_TIM_GET_COUNTER(&htim3) + Encoder_OverflowCount[1]*65536;
	Encoder_CaptureNumber[2] = __HAL_TIM_GET_COUNTER(&htim4) + Encoder_OverflowCount[2]*65536;
	Encoder_CaptureNumber[3] = __HAL_TIM_GET_COUNTER(&htim5) + Encoder_OverflowCount[3]*65536;

	/* 转存PWM数据并进行PID计算 */
	for(i = 0; i < 4; i ++)
	{
		Temp[i] = PWM_Duty[i];
		PWM_Duty[i] = LocPIDCalc(i, Encoder_CaptureNumber[i]);
	}
	
	/* 判断电机方向并赋予新PWM数据 */
	for(i = 0; i < 4; i ++)
	{
		if(Temp[i] > 499)
		{
			PWM_Duty[i] += 499;
		}
		else if(Temp[i] < 499)
		{
			PWM_Duty[i] -= 499;
		}
		else /* if(Temp[i] = 499) */
		{
			PWM_Duty[i] = 499;
		}
	}
	
	/* PWM数据区间限定 */
	for(i = 0; i < 4; i ++)
	{
		if(PWM_Duty[i]>999)PWM_Duty[i]=999;  
		if(PWM_Duty[i]<0)PWM_Duty[i]=0;
	}
	
	/* 向串口发送实时数据 */
	printf("电机1: 编码器计数 %d 高电平时长 %d\n",Encoder_CaptureNumber[0],PWM_Duty[0]);
	printf("电机2: 编码器计数 %d 高电平时长 %d\n",Encoder_CaptureNumber[1],PWM_Duty[1]);
	printf("电机3: 编码器计数 %d 高电平时长 %d\n",Encoder_CaptureNumber[2],PWM_Duty[2]);
	printf("电机4: 编码器计数 %d 高电平时长 %d\n",Encoder_CaptureNumber[3],PWM_Duty[3]);
	 
	/* 清空编码器计数 */
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	__HAL_TIM_SET_COUNTER(&htim5, 0);
	
	/* 注入新的PWM数据到各电机 */
	for(i = 0; i < 4; i ++)
	{
		DCMotor_Contrl(i, PWM_Duty[i]);
	}

}
/************************************************ 文件结束 ************************************************/
