#ifndef _Track_h_
#define _Track_h_

/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"
#include "stdlib.h"
#include "Vehicle.h"
/* 类型定义 ------------------------------------------------------------------*/
extern uint8_t Threshold_1;  //前四路阈值
extern uint8_t Threshold_2;  //后三路阈值
extern uint8_t Situation[7];
extern uint32_t Last_Objective_Track;
/* 宏定义 --------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
void Basic_Track(uint8_t track_recognition);
void Automatic_Threshold(void);
uint8_t Track_Recognition(void);
uint8_t Basic_Binary_Code(void);
uint8_t Track_(void);
#endif /* _Track_h_ */

/************************************************ 文件结束 ************************************************/
