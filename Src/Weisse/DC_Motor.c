/*****************************************************************************
  * 文件名称: DC_Motor.c
  * 作    者: 尤伟宏
  * 编写日期: 一月 2019
  * 功    能: 直流电机相关
  ****************************************************************************
  * 说    明:
  * 
  *****************************************************************************/
	
/* 包含头文件 ----------------------------------------------------------------*/
#include "DC_Motor.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
 __IO int16_t PWM_Duty[4] = {STOP_PWM};
// 占空比：PWM_Duty/L298N_TIM_PERIOD*100%
// 占空比为：50%时电机不转
// 占空比不为：50%时电机旋转，与50%的绝对差越大旋转速度也高
// 旋转方向不仅与程序有关，也与电机接线有关，需要具体分析
// 简单的方法是：如果控制方向与要求相反，调换两根PWM控制线接法

/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 车轮电机控制
  * 输入参数: number：电机编号，支持三个电机驱动
  *             参数：1：对应高级定时器通道1和互补通道1
  *                   2：停机
  *           speed：电机速度调节
  *             参数：0 - 999 ：数值与499差值越大，速度越快
  * 返 回 值: 无
  * 说    明：无
  */
void DCMotor_Contrl(uint8_t motor_number, uint16_t speed)
{
  switch(motor_number)
  {
    case Motor_1:
      HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);   
    case Motor_2:
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);   
    case Motor_3:
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
      HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, speed);   
    case Motor_4:
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
      HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, speed);   
  }
}
/************************************************ 文件结束 ************************************************/
