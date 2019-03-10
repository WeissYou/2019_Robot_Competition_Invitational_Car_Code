/*****************************************************************************
  * 文件名称: Stepper.c
  * 作    者: 尤伟宏
  * 编写日期: 一月 2019
  * 功    能: 步进电机控制
  ****************************************************************************
  * 说    明:
  * 
  *****************************************************************************/
	
/* 包含头文件 ----------------------------------------------------------------*/
#include "Stepper.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/*************************************
  * 函数功能: 步进电机初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 步进电机使能
  ************************************/
void Stepper_Init(void)
{
  HAL_GPIO_WritePin(Stepper_EN_1_GPIO_Port, Stepper_EN_1_Pin, GPIO_PIN_RESET);  //步进电机1失能
  HAL_GPIO_WritePin(Stepper_EN_2_GPIO_Port, Stepper_EN_2_Pin, GPIO_PIN_RESET);  //步进电机2失能
  HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);  //脉冲产生
  HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);   //脉冲产生
}

/*************************************
  * 函数功能: 步进电机启动
  * 输入参数: uint8_t stepper_number, uint8_t dir
  * 返 回 值: 无
  * 说    明: 无
  ************************************/
void Stepper_Start(uint8_t stepper_number, uint8_t dir)
{
  if(stepper_number == Stepper_1)
  {
    if(dir == Front)
    {
      HAL_GPIO_WritePin(Stepper_DIR_1_GPIO_Port, Stepper_DIR_1_Pin, GPIO_PIN_SET);  //步进电机1前进
    }
    else /* if(dir == Back) */
    {
      HAL_GPIO_WritePin(Stepper_DIR_1_GPIO_Port, Stepper_DIR_1_Pin, GPIO_PIN_RESET);  //步进电机1后退
    }
    HAL_GPIO_WritePin(Stepper_EN_1_GPIO_Port, Stepper_EN_1_Pin, GPIO_PIN_SET);  //步进电机1使能
  }
  else /* if(stepper_number == Stepper_2) */
  {
    if(dir == Front)
    {
      HAL_GPIO_WritePin(Stepper_DIR_2_GPIO_Port, Stepper_DIR_2_Pin, GPIO_PIN_SET);  //步进电机2前进
    }
    else /* if(dir == Back) */
    {
      HAL_GPIO_WritePin(Stepper_DIR_2_GPIO_Port, Stepper_DIR_2_Pin, GPIO_PIN_RESET);  //步进电机2后退
    }
    HAL_GPIO_WritePin(Stepper_EN_2_GPIO_Port, Stepper_EN_2_Pin, GPIO_PIN_SET);  //步进电机2使能
  }
}

/*************************************
  * 函数功能: 步进电机停止
  * 输入参数: uint8_t stepper_number
  * 返 回 值: 无
  * 说    明: 无
  ************************************/
void Stepper_Stop(uint8_t stepper_number)
{
  if(stepper_number == Stepper_1)
  {
    HAL_GPIO_WritePin(Stepper_EN_1_GPIO_Port, Stepper_EN_1_Pin, GPIO_PIN_RESET);  //步进电机1失能

  }
  else /* if(stepper_number == Stepper_2) */
  {
    HAL_GPIO_WritePin(Stepper_EN_2_GPIO_Port, Stepper_EN_2_Pin, GPIO_PIN_RESET);  //步进电机2失能
  }
}

/************************************************ 文件结束 ************************************************/
