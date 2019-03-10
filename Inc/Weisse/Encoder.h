#ifndef _ENCODER_H_
#define _ENCODER_H_

/* 包含头文件 ----------------------------------------------------------------*/
#include "main.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
#define ENCODER_LINES 11  //编码器线数
#define SPEEDRATIO  17   // 电机减速比
#define PPR         (SPEEDRATIO * ENCODER_LINES*4) // Pulse/r 每圈可捕获的脉冲数
/* 50毫秒内各占空比对应的编码器计数 */
#define _100PERCENT_COUNT 500
#define  _90PERCENT_COUNT 129
#define  _80PERCENT_COUNT 114
#define  _70PERCENT_COUNT 100
#define  _60PERCENT_COUNT  86
#define  _50PERCENT_COUNT  72
#define  _40PERCENT_COUNT  57
#define  _30PERCENT_COUNT  43
#define  _20PERCENT_COUNT  29
#define  _10PERCENT_COUNT  14
#define          NO_COUNT   0
/* 扩展变量 ------------------------------------------------------------------*/
extern int16_t Encoder_OverflowCount[4];  //电机1编码器溢出次数
extern __IO int16_t Encoder_CaptureNumber[4];     //电机1编码器输入捕获数

/* 函数声明 ------------------------------------------------------------------*/
void Encoder_Init(void);
void Motor_Speed_Measurement(void);
#endif /* _ENCODER_H_ */

/************************************************ 文件结束 ************************************************/
