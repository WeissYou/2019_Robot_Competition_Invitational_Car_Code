#ifndef _Algorithm_h_
#define _Algorithm_h_

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "main.h"
#include "DC_Motor.h"
#include "Encoder.h"
#include "Track.h"
/* ���Ͷ��� ------------------------------------------------------------------*/
typedef struct
{
	__IO int      SetPoint[4];                         //�趨Ŀ�� Desired Value
	__IO long     SumError[4];                                 //����ۼ�
	__IO double   Proportion[4];                               //�������� Proportional Const
	__IO double   Integral[4];                                 //���ֳ��� Integral Const
	__IO double   Derivative[4];                               //΢�ֳ��� Derivative Const
	__IO int      LastError[4];                                //Error[-1]
	__IO int      PrevError[4];                                //Error[-2]
}PID_TypeDef;
/* �궨�� --------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
/* PID�ṹ��ָ�� */
extern PID_TypeDef *ptr;
/* �������� ------------------------------------------------------------------*/
void IncPIDInit(void);
int32_t LocPIDCalc(uint8_t Motor_Number, int16_t NextPoint);
void PID_Function(void);

#endif /* _Algorithm_h_ */

/************************************************ �ļ����� ************************************************/
