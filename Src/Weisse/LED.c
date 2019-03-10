/*****************************************************************************
  * 文件名称: LED.c
  * 作    者: 尤伟宏
  * 编写日期: 一月 2019
  * 功    能: LED指示灯相关
  ****************************************************************************
  * 说    明: 共阴极解法，高电平有效
  * 
  *****************************************************************************/
	
/* 包含头文件 ----------------------------------------------------------------*/
#include "LED.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/*************************************
  * 函数功能: LED初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 默认不发光
  ************************************/
void LED_Init(void)
{
        LED_1_OFF; 
	LED_2_OFF;
	LED_3_OFF;
	LED_4_OFF;
	LED_5_OFF;
	LED_6_OFF;
	LED_7_OFF;
}//End of void LED_Init(void)

/*************************************
  * 函数功能: LED发光
  * 输入参数: uint8_t track_position
  * 返 回 值: 无
  * 说    明: 配合轨迹识别算法使用
  ************************************/
void LED_Blink(uint8_t calculated_track)
{
	switch(calculated_track)
	{
		case 0:  LED_1_ON;
		case 1:  LED_2_ON;
		case 2:  LED_3_ON;
		case 3:  LED_4_ON;
		case 4:  LED_5_ON;
		case 5:  LED_6_ON;
		case 6:  LED_7_ON;
	}
}//End of void LED_Blink(uint8_t Calculated_Track)

/*************************************
  * 函数功能: LED发光
  * 输入参数: uint16_t Objective_Track
  * 返 回 值: 无
  * 说    明: 配合一般二值化使用
  ************************************/
void LED_blink(uint8_t objective_track)
{
    if (objective_track&(0x01 << 0))
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

    if (objective_track&(0x01 << 1))
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

    if (objective_track&(0x01 << 2))
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

    if (objective_track&(0x01 << 3))
      HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);

    if (objective_track&(0x01 << 4))
      HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);

    if (objective_track&(0x01 << 5))
      HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);

    if (objective_track&(0x01 << 6))
      HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET);
}//End of void LED_blink(uint8_t Objective_Track)

/************************************************ 文件结束 ************************************************/
