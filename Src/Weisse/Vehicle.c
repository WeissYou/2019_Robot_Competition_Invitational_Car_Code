/*****************************************************************************
  * 文件名称: Vehicle.c
  * 作    者: 尤伟宏
  * 编写日期: 三月 2019
  * 功    能: 载具控制
  ****************************************************************************
  * 说    明:
  * 
  *****************************************************************************/
	
/* 包含头文件 ----------------------------------------------------------------*/
#include "Vehicle.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/*************************************
  * 函数功能: 直行
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  ************************************/
void Straight(void)
{
//		  DCMotor_Contrl(Motor_1, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_2, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_3, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_4, FORWARD_100PERCENT_PWM);
  ptr->SetPoint[0] = _100PERCENT_COUNT;
  ptr->SetPoint[1] = _100PERCENT_COUNT;
  ptr->SetPoint[2] = _100PERCENT_COUNT;
  ptr->SetPoint[3] = _100PERCENT_COUNT;
}

/*************************************
  * 函数功能: 小调
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  ************************************/
void Small(uint8_t dir)
{
  if(dir == Right)
  {
//		  DCMotor_Contrl(Motor_1, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_2, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_3, FORWARD_80PERCENT_PWM);
//		  DCMotor_Contrl(Motor_4, FORWARD_80PERCENT_PWM);
    ptr->SetPoint[0] = _100PERCENT_COUNT;
    ptr->SetPoint[1] = _100PERCENT_COUNT;
    ptr->SetPoint[2] = _80PERCENT_COUNT;
    ptr->SetPoint[3] = _80PERCENT_COUNT;
  }
  else /* if(dir == Left) */
  {
//		  DCMotor_Contrl(Motor_1, FORWARD_80PERCENT_PWM);
//		  DCMotor_Contrl(Motor_2, FORWARD_80PERCENT_PWM);
//		  DCMotor_Contrl(Motor_3, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_4, FORWARD_100PERCENT_PWM);
    ptr->SetPoint[0] = _80PERCENT_COUNT;
    ptr->SetPoint[1] = _80PERCENT_COUNT;
    ptr->SetPoint[2] = _100PERCENT_COUNT;
    ptr->SetPoint[3] = _100PERCENT_COUNT;
  }
}

/*************************************
  * 函数功能: 中调
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  ************************************/
void Middle(uint8_t dir)
{
  if(dir == Right)
  {
//		  DCMotor_Contrl(Motor_1, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_2, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_3, FORWARD_50PERCENT_PWM);
//		  DCMotor_Contrl(Motor_4, FORWARD_50PERCENT_PWM);
    ptr->SetPoint[0] = _100PERCENT_COUNT;
    ptr->SetPoint[1] = _100PERCENT_COUNT;
    ptr->SetPoint[2] = _50PERCENT_COUNT;
    ptr->SetPoint[3] = _50PERCENT_COUNT;
  }
  else /* if(dir == Left) */
  {
//		  DCMotor_Contrl(Motor_1, FORWARD_50PERCENT_PWM);
//		  DCMotor_Contrl(Motor_2, FORWARD_50PERCENT_PWM);
//		  DCMotor_Contrl(Motor_3, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_4, FORWARD_100PERCENT_PWM);
    ptr->SetPoint[0] = _50PERCENT_COUNT;
    ptr->SetPoint[1] = _50PERCENT_COUNT;
    ptr->SetPoint[2] = _100PERCENT_COUNT;
    ptr->SetPoint[3] = _100PERCENT_COUNT;
  }
}

/*************************************
  * 函数功能: 大调
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  ************************************/
void Big(uint8_t dir)
{
  if(dir == Right)
  {
//		  DCMotor_Contrl(Motor_1, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_2, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_3, FORWARD_30PERCENT_PWM);
//		  DCMotor_Contrl(Motor_4, FORWARD_30PERCENT_PWM);
    ptr->SetPoint[0] = _100PERCENT_COUNT;
    ptr->SetPoint[1] = _100PERCENT_COUNT;
    ptr->SetPoint[2] = _30PERCENT_COUNT;
    ptr->SetPoint[3] = _30PERCENT_COUNT;
  }
  else /* if(dir == Left) */
  {
//		  DCMotor_Contrl(Motor_1, FORWARD_30PERCENT_PWM);
//		  DCMotor_Contrl(Motor_2, FORWARD_30PERCENT_PWM);
//		  DCMotor_Contrl(Motor_3, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_4, FORWARD_100PERCENT_PWM);
    ptr->SetPoint[0] = _30PERCENT_COUNT;
    ptr->SetPoint[1] = _30PERCENT_COUNT;
    ptr->SetPoint[2] = _100PERCENT_COUNT;
    ptr->SetPoint[3] = _100PERCENT_COUNT;
  }
}

/*************************************
  * 函数功能: 转向
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  ************************************/
void Turn(uint8_t dir)
{
  if(dir == Right)
  {
//		  DCMotor_Contrl(Motor_1, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_2, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_3, BACKWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_4, BACKWARD_100PERCENT_PWM);
    ptr->SetPoint[0] = _100PERCENT_COUNT;
    ptr->SetPoint[1] = _100PERCENT_COUNT;
    ptr->SetPoint[2] = _100PERCENT_COUNT;
    ptr->SetPoint[3] = _100PERCENT_COUNT;
  }
  else /* if(dir == Left) */
  {
//		  DCMotor_Contrl(Motor_1, BACKWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_2, BACKWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_3, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_4, FORWARD_100PERCENT_PWM);
    ptr->SetPoint[0] = _100PERCENT_COUNT;
    ptr->SetPoint[1] = _100PERCENT_COUNT;
    ptr->SetPoint[2] = _100PERCENT_COUNT;
    ptr->SetPoint[3] = _100PERCENT_COUNT;
  }
}

/*************************************
  * 函数功能: 停车
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  ************************************/
void Vehicle_Stop(void)
{
//		  DCMotor_Contrl(Motor_1, STOP_PWM);
//		  DCMotor_Contrl(Motor_2, STOP_PWM);
//		  DCMotor_Contrl(Motor_3, STOP_PWM);
//		  DCMotor_Contrl(Motor_4, STOP_PWM);
    ptr->SetPoint[0] = NO_COUNT;
    ptr->SetPoint[1] = NO_COUNT;
    ptr->SetPoint[2] = NO_COUNT;
    ptr->SetPoint[3] = NO_COUNT;
}

/************************************************ 文件结束 ************************************************/
