#ifndef _Algorithm_h_
#define _Algorithm_h_

/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"
#include "DC_Motor.h"
#include "Encoder.h"
#include "Track.h"
/* 类型定义 ------------------------------------------------------------------*/
typedef struct
{
	__IO int      SetPoint[4];                         //设定目标 Desired Value
	__IO long     SumError[4];                                 //误差累计
	__IO double   Proportion[4];                               //比例常数 Proportional Const
	__IO double   Integral[4];                                 //积分常数 Integral Const
	__IO double   Derivative[4];                               //微分常数 Derivative Const
	__IO int      LastError[4];                                //Error[-1]
	__IO int      PrevError[4];                                //Error[-2]
}PID_TypeDef;
/* 宏定义 --------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* PID结构体指针 */
extern PID_TypeDef *ptr;
/* 函数声明 ------------------------------------------------------------------*/
void IncPIDInit(void);
int32_t LocPIDCalc(uint8_t Motor_Number, int16_t NextPoint);
void PID_Function(void);

#endif /* _Algorithm_h_ */

/************************************************ 文件结束 ************************************************/
