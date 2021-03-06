#ifndef _Stepper_h_
#define _Stepper_h_

/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"
/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
  Stepper_1 = 0,  //步进电机1
  Stepper_2,  //步进电机2
  Front,  //前进
  Back,  //后退
  In_Motion,  //在运动中
  At_Destination  //到达目的地
}Stepper_command;  //步进电机控制相关指令选项
/* 宏定义 --------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
void Stepper_Init(void);
void Stepper_Start(uint8_t stepper_number, uint8_t dir);
void Stepper_Stop(uint8_t stepper_number);
#endif /* _Stepper_h_ */

/************************************************ 文件结束 ************************************************/
