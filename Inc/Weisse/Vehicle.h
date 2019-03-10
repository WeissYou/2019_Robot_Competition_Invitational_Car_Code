#ifndef _Vehicle_h_
#define _Vehicle_h_

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "main.h"
#include "DC_Motor.h"
#include "Encoder.h"
#include "PID.h"
#include "Task.h"
/* ���Ͷ��� ------------------------------------------------------------------*/
typedef enum
{
  Left = 0,
  Right,
  Forward,
  Backward
}Vehicle_Command;  //�ؾ߿���ָ��
/* �궨�� --------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
/* �������� ------------------------------------------------------------------*/
void Straight(void);
void Small(uint8_t dir);
void Middle(uint8_t dir);
void Big(uint8_t dir);
void Turn(uint8_t dir);
void Vehicle_Stop(void);

#endif /* _Vehicle_h_ */
/************************************************ �ļ����� ************************************************/

