#ifndef _ENCODER_H_
#define _ENCODER_H_

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "main.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/
#define ENCODER_LINES 11  //����������
#define SPEEDRATIO  17   // ������ٱ�
#define PPR         (SPEEDRATIO * ENCODER_LINES*4) // Pulse/r ÿȦ�ɲ����������
/* 50�����ڸ�ռ�ձȶ�Ӧ�ı��������� */
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
/* ��չ���� ------------------------------------------------------------------*/
extern int16_t Encoder_OverflowCount[4];  //���1�������������
extern __IO int16_t Encoder_CaptureNumber[4];     //���1���������벶����

/* �������� ------------------------------------------------------------------*/
void Encoder_Init(void);
void Motor_Speed_Measurement(void);
#endif /* _ENCODER_H_ */

/************************************************ �ļ����� ************************************************/
