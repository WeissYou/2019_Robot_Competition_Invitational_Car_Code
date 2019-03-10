/*****************************************************************************
  * 文件名称: Carrier.c
  * 作    者: 尤伟宏
  * 编写日期: 三月 2019
  * 功    能: 装配机构控制
  ****************************************************************************
  * 说    明:
  * 
  *****************************************************************************/
	
/* 包含头文件 ----------------------------------------------------------------*/
#include "Carrier.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
uint8_t Arm_Status = 0;
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/*************************************
  * 函数功能: 机械臂调零
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  ************************************/
void Carrier_Init(void)
{
  Arm_Control(Vertical);  //机械臂垂直于地
  Paw_Control(Open);  //卡爪张开
  if(HAL_GPIO_ReadPin(Limited_Switch_1_GPIO_Port, Limited_Switch_1_Pin) == GPIO_PIN_RESET)  //若限位开关1已被碰触
  {
    Stepper_Stop(Stepper_1);  //步进电机1停止
  }
  else  /* if(HAL_GPIO_ReadPin(Limited_Switch_1_GPIO_Port, Limited_Switch_1_Pin) == GPIO_PIN_SET)  //若限位开关1未被碰触 */
  {
    Stepper_Start(Stepper_1, Backward);  //步进电机1向后运动
  }
  if(HAL_GPIO_ReadPin(Limited_Switch_3_GPIO_Port, Limited_Switch_3_Pin) == GPIO_PIN_RESET)  //若限位开关2已被碰触
  {
    Stepper_Stop(Stepper_2);  //步进电机2停止
  }
  else  /* if(HAL_GPIO_ReadPin(Limited_Switch_3_GPIO_Port, Limited_Switch_3_Pin) == GPIO_PIN_SET)  //若限位开关2未被碰触 */
  {
    Stepper_Start(Stepper_2, Backward);  //步进电机2向后运动
  }
}

/*************************************
  * 函数功能: 抓取一个目标物
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  ************************************/
void Get_a_Target(void)
{
  Arm_Control(Horizontal);  //机械臂垂直于地
  if(Arm_Status == In_Motion)
  {
    if(HAL_GPIO_ReadPin(Limited_Switch_1_GPIO_Port, Limited_Switch_1_Pin) == GPIO_PIN_RESET)  //若限位开关1已被碰触
    {
      Stepper_Stop(Stepper_1);  //步进电机1停止
    }
    else  /* if(HAL_GPIO_ReadPin(Limited_Switch_1_GPIO_Port, Limited_Switch_1_Pin) == GPIO_PIN_SET)  //若限位开关1未被碰触 */
    {
      Stepper_Start(Stepper_1, Backward);  //步进电机1向后运动
    }
    if(HAL_GPIO_ReadPin(Limited_Switch_3_GPIO_Port, Limited_Switch_3_Pin) == GPIO_PIN_RESET)  //若限位开关2已被碰触
    {
      Stepper_Stop(Stepper_2);  //步进电机2停止
    }
    else  /* if(HAL_GPIO_ReadPin(Limited_Switch_3_GPIO_Port, Limited_Switch_3_Pin) == GPIO_PIN_SET)  //若限位开关2未被碰触 */
    {
      Stepper_Start(Stepper_2, Backward);  //步进电机2向后运动
    }
  }
  else /* if(Arm_Status == Destination) */
  {
    Paw_Control(Close);  //卡爪闭合
  }
}	

/*************************************
  * 函数功能: 装配一个目标物
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  ************************************/
void Put_a_Target(void)
{
  if(HAL_GPIO_ReadPin(Limited_Switch_1_GPIO_Port, Limited_Switch_1_Pin) == GPIO_PIN_RESET)  //若限位开关1已被碰触
  {
    Stepper_Stop(Stepper_1);  //步进电机1停止
  }
  else  /* if(HAL_GPIO_ReadPin(Limited_Switch_1_GPIO_Port, Limited_Switch_1_Pin) == GPIO_PIN_SET)  //若限位开关1未被碰触 */
  {
    Stepper_Start(Stepper_1, Backward);  //步进电机1向后运动
  }
  if(HAL_GPIO_ReadPin(Limited_Switch_3_GPIO_Port, Limited_Switch_3_Pin) == GPIO_PIN_RESET)  //若限位开关2已被碰触
  {
    Stepper_Stop(Stepper_2);  //步进电机2停止
  }
  else  /* if(HAL_GPIO_ReadPin(Limited_Switch_3_GPIO_Port, Limited_Switch_3_Pin) == GPIO_PIN_SET)  //若限位开关2未被碰触 */
  {
    Stepper_Start(Stepper_2, Backward);  //步进电机2向后运动
  }
}

/************************************************ 文件结束 ************************************************/
