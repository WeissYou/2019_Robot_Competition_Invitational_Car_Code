#ifndef _Vehicle_h_
#define _Vehicle_h_

/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"
#include "DC_Motor.h"
#include "Encoder.h"
#include "PID.h"
#include "Task.h"
/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
  Left = 0,
  Right,
  Forward,
  Backward
}Vehicle_Command;  //载具控制指令
/* 宏定义 --------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
void Straight(void);
void Small(uint8_t dir);
void Middle(uint8_t dir);
void Big(uint8_t dir);
void Turn(uint8_t dir);
void Vehicle_Stop(void);

#endif /* _Vehicle_h_ */
/************************************************ 文件结束 ************************************************/

