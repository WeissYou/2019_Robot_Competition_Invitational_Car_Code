#ifndef _Task_h_
#define _Task_h_

/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"
#include "LED.h"
#include "Encoder.h"
#include "Track.h"
#include "Buzzer.h"
#include "Vehicle.h"
#include "Carrier.h"
/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
  IN_START_AREA = 0,  //在出发区内
	OUT_OF_START_AREA,  //驶离出发区
	
	GET_FIRST_TARGET_FIRST_CROSS,  //获取第一个目标物的第一个路口
	GET_FIRST_TARGET,   //获取第一个目标物
	PUT_FIRST_TARGET_FIRST_CROSS,  //放置第一个目标物的第一路口
	PUT_FIRST_TARGET_SECOND_CROSS,  //放置第一个目标物的第二路口
	PUT_FIRST_TARGET_THIRD_CROSS,  //放置第一个目标物的第三路口
	PUT_FIRST_TARGET,  //放置第一个目标物
	
	GET_SECOND_TARGET_FIRST_CROSS,  //获取第二个目标物的第一个路口
	GET_SECOND_TARGET_SECOND_CROSS,  //获取第二个目标物的第二个路口
	GET_SECOND_TARGET_THIRD_CROSS,  //获取第二个目标物的第三个路口
	GET_SECOND_TARGET_FOURTH_CROSS,  //获取第二个目标物的第四个路口
	GET_SECOND_TARGET_FIFTH_CROSS,  //获取第二个目标物的第五个路口
	GET_SECOND_TARGET,   //获取第二个目标物
	PUT_SECOND_TARGET_FIRST_CROSS,  //放置第二目标物的第一路口
	PUT_SECOND_TARGET_SECOND_CROSS,  //放置第二目标物的第二路口
	PUT_SECOND_TARGET_THIRD_CROSS,  //放置第二目标物的第三路口
	PUT_SECOND_TARGET_FOURTH_CROSS,  //放置第二目标物的第四路口
	PUT_SECOND_TARGET_FIFTH_CROSS,  //放置第二目标物的第五路口
	PUT_SECOND_TARGET,  //放置第二个目标物
	
	GET_THIRD_TARGET_FIRST_CROSS,  //获取第三个目标物的第一个路口
	GET_THIRD_TARGET_SECOND_CROSS,  //获取第三个目标物的第二个路口
	GET_THIRD_TARGET_THIRD_CROSS,  //获取第三个目标物的第三个路口
	GET_THIRD_TARGET_FOURTH_CROSS,  //获取第三个目标物的第四个路口
	GET_THIRD_TARGET_FIFTH_CROSS,  //获取第三个目标物的第五路口
	GET_THIRD_TARGET,   //获取第三个目标物
	PUT_THIRD_TARGET_FIRST_CROSS,  //放置第三个目标物的第一路口
	PUT_THIRD_TARGET,  //放置第三个目标物
	
	GET_FOURTH_TARGET_FIRST_CROSS,  //获取第四个目标物的第一个路口
	GET_FOURTH_TARGET_SECOND_CROSS,  //获取第四个目标物的第二个路口
  GET_FOURTH_TARGET,   //获取第四个目标物
	PUT_FOURTH_TARGET_FIRST_CROSS,  //放置第四个目标物的第一路口
	PUT_FOURTH_TARGET_SECOND_CROSS,  //放置第四个目标物的第二路口
	PUT_FOURTH_TARGET_THIRD_CROSS,  //放置第四个目标物的第三路口
  PUT_FOURTH_TARGET,  //放置第四个目标物
	
	GET_FIFTH_TARGET_FIRST_CROSS,  //获取第五个目标物的第一个路口
	GET_FIFTH_TARGET_SECOND_CROSS,  //获取第五个目标物的第二个路口
	GET_FIFTH_TARGET_THIRD_CROSS,  //获取第五个目标物的第二个路口
  GET_FIFTH_TARGET,   //获取第五个目标物
	PUT_FIFTH_TARGET_FIRST_CROSS,  //放置第五个目标物的第一路口
	PUT_FIFTH_TARGET_SECOND_CROSS,  //放置第五个目标物的第二路口
	PUT_FIFTH_TARGET_THIRD_CROSS,  //放置第五个目标物的第三路口
	GET_FIFTH_TARGET_FOURTH_CROSS,  //获取第五个目标物的第四个路口
	GET_FIFTH_TARGET_FIFTH_CROSS,  //获取第五个目标物的第五路口
  PUT_FIFTH_TARGET,  //放置第五个目标物

}TASK;// 任务
/* 宏定义 --------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
extern uint8_t Turn_Flag;  //转向标志位
/* 函数声明 ------------------------------------------------------------------*/
void Task_Timer_Handle(void);
void Task_Delay_Start(unsigned int time);
#endif /* _Task_h_ */

/************************************************ 文件结束 ************************************************/
