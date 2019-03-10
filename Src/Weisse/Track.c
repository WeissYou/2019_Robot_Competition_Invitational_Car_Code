/*****************************************************************************
  * 文件名称: Track.c
  * 作    者: 尤伟宏
  * 编写日期: 一月 2019
  * 功    能: 循迹相关
  ****************************************************************************
  * 说    明:
  * 
  *****************************************************************************/
	
/* 包含头文件 ----------------------------------------------------------------*/
#include "Track.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
uint8_t Threshold_1;  //前四路阈值
uint8_t Threshold_2;  //后三路阈值
uint8_t Situation[7] = {0};
uint32_t Last_Objective_Track;
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/*************************************
  * 函数功能: 基础循迹逻辑
  * 输入参数: uint8_t track_recognition
  * 返 回 值: 无
  * 说    明: 无
  ************************************/
void Basic_Track(uint8_t track_recognition)
{
	LED_Blink(track_recognition);  //LED发光
	switch(track_recognition)
	{
		case 0:Big(Left);    break;
		case 1:Middle(Left); break;
		case 2:Small(Left);  break;
    case 3:Straight();   break;
		case 4:Small(Right); break;
		case 5:Middle(Right);break;
		case 6:Big(Right);   break;
	}
}//End of Basic_Track(void)

/*************************************
  * 函数功能: 自动阈值算法
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 为应对灰度传感器自身缺陷，在此将运算作两次进行
  ************************************/
void Automatic_Threshold(void)
{
	 uint8_t i;
	 uint8_t value1_max, value1_min;  //前四路灰度值
	 uint8_t value2_max, value2_min;  //后三路灰度值
	 value1_max = ADC_ConvertedValueLocal[0];  //动态阈值算法，读取最大和最小值
   value2_max = ADC_ConvertedValueLocal[4];  
   for(i = 0; i < 4; i ++)   
   {
     if(value1_max <= ADC_ConvertedValueLocal[i])
		 {
       value1_max = ADC_ConvertedValueLocal[i];
		 }
   }
	 for(i = 4; i < 7; i ++)
	 {
		 if(value2_max <= ADC_ConvertedValueLocal[i])
		 {
       value2_max = ADC_ConvertedValueLocal[i];
		 }
	 }
	 value1_min = ADC_ConvertedValueLocal[0];  //最小值
	 value2_min = ADC_ConvertedValueLocal[4];  
   for(i = 0; i < 7; i ++) 
   {
     if(value1_min >= ADC_ConvertedValueLocal[i])
		 {
       value1_min = ADC_ConvertedValueLocal[i];
		 }
   }
	 for(i = 4; i < 7; i ++)
	 {
		 if(value2_min <= ADC_ConvertedValueLocal[i])
		 {
       value2_min = ADC_ConvertedValueLocal[i];
		 }
	 }
   Threshold_1 = ((value1_max + value1_min) / 2) - 5;	  //计算出本次中线提取的阈值
	 Threshold_2 = ((value2_min) / 2);
}//End of void Automatic_Threshold(void)

/*************************************
  * 函数功能: 简单二值化算法
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  ************************************/
uint8_t Basic_Binary_Code(void)
{
	uint8_t i;
	static uint8_t Objective_Track;  //实际上的黑线位置、上次实际上的黑线位置

  for(i = 0; i < 4; i ++)
	{
	  if(ADC_ConvertedValueLocal[i] <= Threshold_1)
		{
			Objective_Track |= 0x01 << i;
			Situation[i] = 1;
		}
		else /* if(ADC_ConvertedValueLocal[i] < Threshold) */
		{
			Objective_Track &= ~(0x01 << i);
			Situation[i] = 0;
		}
	}
  for(i = 4; i < 7; i ++)
	{
	  if(ADC_ConvertedValueLocal[i] <= Threshold_2)
		{
			Objective_Track |= 0x01 << i;
			Situation[i] = 1;
		}
		else /* if(ADC_ConvertedValueLocal[i] < Threshold) */
		{
			Objective_Track &= ~(0x01 << i);
			Situation[i] = 0;
		}
	}
	Last_Objective_Track = Situation[0] + 10 * Situation[1] + 100 * Situation[2] + 1000 * Situation[3] + 10000 * Situation[4] + 100000 * Situation[5] + 1000000 * Situation[6];
	//Last_Objective_Track = Objective_Track;  //保存上一次的轨迹
	return Objective_Track;
}
uint8_t Track_(void)
{
	int8_t i, left, right, track_;
	left = right = track_ = 0;
	for(i = 0; i < 7; i ++)
	{
		if(Situation[i] == 1)
		{
			left = i;
			break;
		}
	}
	for(i = 6; i > -1; i --)
	{
		if(Situation[i] == 1)
		{
			right = i;
			break;
		}
	}
	track_ = (left + right) / 2;
	return track_;
}

/*************************************
  * 函数功能: 轨迹识别算法
  * 输入参数: 无
  * 返 回 值: uint8_t Calculated_Track
  * 说    明: 无
  ************************************/
uint8_t Track_Recognition(void)
{
	 uint8_t left, right, Calculated_Track;  //计算得到的黑线位置
//         uint8_t Last_Calculated_Track;  //上次计算得到的黑线位置
	 uint8_t Temp_Track_1, Temp_Track_2;  //计算用的临时黑线值
	 left = right = 0;
	 int8_t i;
	 for(i = 0; i < 4; i++)   //从左侧寻找轨迹
	 {
		 if(ADC_ConvertedValueLocal[i] <= Threshold_1 /*|| ADC_ConvertedValueLocal[i + 1] <= Threshold_1*/)
		 {
		   left = i;
			 break;	
		 }
		 else 
		 {
			 left = 3;
//			 break;
		 }
	 }
	 for(i = 3; i > -1; i--)  //从右侧寻找轨迹
   {
		 if(ADC_ConvertedValueLocal[i] <= Threshold_1 /*|| ADC_ConvertedValueLocal[i + 1] <= Threshold_1*/)
		 {
		   right = i;
		   break;	
		 }
		 else
		 {
			 right = 3;
//			 break;
		 }
   }
	 Temp_Track_1 = (right + left) / 2;  //计算中线位置
//	 Temp_Track_1 = left;  //计算中线位置
//	 left = right = 0;
	 for(i = 4; i < 7; i++)   //从左侧寻找轨迹
	 {
		 if(ADC_ConvertedValueLocal[i] <= Threshold_2 /*|| ADC_ConvertedValueLocal[i + 1] <= Threshold_2*/)
		 {
		   left = i;
			 break;	
		 }
		 else
		 {
			 left = 3;
//			 break;
		 }
	 }
	 for(i = 6; i > 3; i--)  //从右侧寻找轨迹
   {
		 if(ADC_ConvertedValueLocal[i] <= Threshold_2 /*|| ADC_ConvertedValueLocal[i + 1] <= Threshold_2*/)
		 {
		   right = i;
		   break;	
		 }
		 else
		 {
			 right = 3;
//			 break;
		 }
   }
	 Temp_Track_2 = (right + left) / 2;  //计算中线位置
//	 Temp_Track_2 = right;  //计算中线位置
//	 if(abs(Calculated_Track - Last_Calculated_Track) > 4)   //计算中线的偏差，如果太大
//	 {
//	   Calculated_Track = Last_Calculated_Track;    //则取上一次的值
//	 }
	 Calculated_Track = (Temp_Track_1 + Temp_Track_2) / 2;
//	 Last_Calculated_Track = Calculated_Track;  //保存上一次的轨迹
	 return Calculated_Track;
}//End of Track_Recognition(void)
/************************************************ 文件结束 ************************************************/
