/*****************************************************************************
  * 文件名称: Encoder.c
  * 作    者: 尤伟宏
  * 编写日期: 一月 2019
  * 功    能: 
  ****************************************************************************
  * 说    明: 
  * 
  *****************************************************************************/
	
/* 包含头文件 ----------------------------------------------------------------*/
#include "Encoder.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
int16_t Encoder_OverflowCount[4] = {0};  //电机编码器计数
__IO int16_t Encoder_CaptureNumber[4] = {0};  // 输入捕获数
uint16_t time_count = 0;
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/*************************************
  * 函数功能: 编码器初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  ************************************/
void Encoder_Init(void)
{
	/* 编码器启动 */
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);  
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);  
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);  
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);  
}

/*************************************
  * 函数功能: 电机测速
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  ************************************/
void Motor_Speed_Measurement(void)
{
	float Objective_Speed[4] = {0};  //实际速度
  Encoder_CaptureNumber[0] = __HAL_TIM_GET_COUNTER(&htim2) + Encoder_OverflowCount[0]*65536;
  Encoder_CaptureNumber[1] = __HAL_TIM_GET_COUNTER(&htim3) + Encoder_OverflowCount[1]*65536;
  Encoder_CaptureNumber[2] = __HAL_TIM_GET_COUNTER(&htim4) + Encoder_OverflowCount[2]*65536;
	Encoder_CaptureNumber[3] = __HAL_TIM_GET_COUNTER(&htim5) + Encoder_OverflowCount[3]*65536;
	printf("电机1码盘计数: %d \n",Encoder_CaptureNumber[0]);
	Objective_Speed[0] = (float)(1200*Encoder_CaptureNumber[0])/PPR;
  printf("电机1实际转速: %0.2f rpm \n",Objective_Speed[0]);
	printf("\n");
	printf("电机2码盘计数: %d \n",Encoder_CaptureNumber[0]);
	Objective_Speed[1] = (float)(1200*Encoder_CaptureNumber[0])/PPR;
  printf("电机2实际转速: %0.2f rpm \n",Objective_Speed[1]);
	printf("\n");
	printf("电机3码盘计数: %d \n",Encoder_CaptureNumber[0]);
	Objective_Speed[2] = (float)(1200*Encoder_CaptureNumber[0])/PPR;
  printf("电机3实际转速: %0.2f rpm \n",Objective_Speed[2]);
	printf("\n");
	printf("电机4码盘计数: %d \n",Encoder_CaptureNumber[0]);
	Objective_Speed[3] = (float)(1200*Encoder_CaptureNumber[0])/PPR;
  printf("电机4实际转速: %0.2f rpm \n",Objective_Speed[3]);
	printf("\n");
  __HAL_TIM_SET_COUNTER(&htim2, 0);
}

/************************************************ 文件结束 ************************************************/
