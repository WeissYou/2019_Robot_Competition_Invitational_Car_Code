#ifndef _Track_h_
#define _Track_h_

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "main.h"
#include "stdlib.h"
#include "Vehicle.h"
/* ���Ͷ��� ------------------------------------------------------------------*/
extern uint8_t Threshold_1;  //ǰ��·��ֵ
extern uint8_t Threshold_2;  //����·��ֵ
extern uint8_t Situation[7];
extern uint32_t Last_Objective_Track;
/* �궨�� --------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
/* �������� ------------------------------------------------------------------*/
void Basic_Track(uint8_t track_recognition);
void Automatic_Threshold(void);
uint8_t Track_Recognition(void);
uint8_t Basic_Binary_Code(void);
uint8_t Track_(void);
#endif /* _Track_h_ */

/************************************************ �ļ����� ************************************************/
