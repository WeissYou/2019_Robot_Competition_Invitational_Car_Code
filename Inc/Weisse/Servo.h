#ifndef _Servo_h_
#define _Servo_h_

/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"
#include "Task.h"
/* 类型定义 ------------------------------------------------------------------*/
//舵机控制指令
#define Horizontal 100  //机械臂置水平
#define Vertical   250  //机械臂置竖直
#define Open       250  //爪子张开
#define Close      100  //爪子闭合
/* 宏定义 --------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
void Servo_Init(void);
void Arm_Control(uint8_t action);
void Paw_Control(uint8_t action);
void Motor_PWM_SetDutyRatio(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t value);

#endif /* _Servo_h_ */

/************************************************ 文件结束 ************************************************/
