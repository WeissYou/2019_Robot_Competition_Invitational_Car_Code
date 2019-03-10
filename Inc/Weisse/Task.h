#ifndef _Task_h_
#define _Task_h_

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "main.h"
#include "LED.h"
#include "Encoder.h"
#include "Track.h"
#include "Buzzer.h"
#include "Vehicle.h"
#include "Carrier.h"
/* ���Ͷ��� ------------------------------------------------------------------*/
typedef enum
{
  IN_START_AREA = 0,  //�ڳ�������
	OUT_OF_START_AREA,  //ʻ�������
	
	GET_FIRST_TARGET_FIRST_CROSS,  //��ȡ��һ��Ŀ����ĵ�һ��·��
	GET_FIRST_TARGET,   //��ȡ��һ��Ŀ����
	PUT_FIRST_TARGET_FIRST_CROSS,  //���õ�һ��Ŀ����ĵ�һ·��
	PUT_FIRST_TARGET_SECOND_CROSS,  //���õ�һ��Ŀ����ĵڶ�·��
	PUT_FIRST_TARGET_THIRD_CROSS,  //���õ�һ��Ŀ����ĵ���·��
	PUT_FIRST_TARGET,  //���õ�һ��Ŀ����
	
	GET_SECOND_TARGET_FIRST_CROSS,  //��ȡ�ڶ���Ŀ����ĵ�һ��·��
	GET_SECOND_TARGET_SECOND_CROSS,  //��ȡ�ڶ���Ŀ����ĵڶ���·��
	GET_SECOND_TARGET_THIRD_CROSS,  //��ȡ�ڶ���Ŀ����ĵ�����·��
	GET_SECOND_TARGET_FOURTH_CROSS,  //��ȡ�ڶ���Ŀ����ĵ��ĸ�·��
	GET_SECOND_TARGET_FIFTH_CROSS,  //��ȡ�ڶ���Ŀ����ĵ����·��
	GET_SECOND_TARGET,   //��ȡ�ڶ���Ŀ����
	PUT_SECOND_TARGET_FIRST_CROSS,  //���õڶ�Ŀ����ĵ�һ·��
	PUT_SECOND_TARGET_SECOND_CROSS,  //���õڶ�Ŀ����ĵڶ�·��
	PUT_SECOND_TARGET_THIRD_CROSS,  //���õڶ�Ŀ����ĵ���·��
	PUT_SECOND_TARGET_FOURTH_CROSS,  //���õڶ�Ŀ����ĵ���·��
	PUT_SECOND_TARGET_FIFTH_CROSS,  //���õڶ�Ŀ����ĵ���·��
	PUT_SECOND_TARGET,  //���õڶ���Ŀ����
	
	GET_THIRD_TARGET_FIRST_CROSS,  //��ȡ������Ŀ����ĵ�һ��·��
	GET_THIRD_TARGET_SECOND_CROSS,  //��ȡ������Ŀ����ĵڶ���·��
	GET_THIRD_TARGET_THIRD_CROSS,  //��ȡ������Ŀ����ĵ�����·��
	GET_THIRD_TARGET_FOURTH_CROSS,  //��ȡ������Ŀ����ĵ��ĸ�·��
	GET_THIRD_TARGET_FIFTH_CROSS,  //��ȡ������Ŀ����ĵ���·��
	GET_THIRD_TARGET,   //��ȡ������Ŀ����
	PUT_THIRD_TARGET_FIRST_CROSS,  //���õ�����Ŀ����ĵ�һ·��
	PUT_THIRD_TARGET,  //���õ�����Ŀ����
	
	GET_FOURTH_TARGET_FIRST_CROSS,  //��ȡ���ĸ�Ŀ����ĵ�һ��·��
	GET_FOURTH_TARGET_SECOND_CROSS,  //��ȡ���ĸ�Ŀ����ĵڶ���·��
  GET_FOURTH_TARGET,   //��ȡ���ĸ�Ŀ����
	PUT_FOURTH_TARGET_FIRST_CROSS,  //���õ��ĸ�Ŀ����ĵ�һ·��
	PUT_FOURTH_TARGET_SECOND_CROSS,  //���õ��ĸ�Ŀ����ĵڶ�·��
	PUT_FOURTH_TARGET_THIRD_CROSS,  //���õ��ĸ�Ŀ����ĵ���·��
  PUT_FOURTH_TARGET,  //���õ��ĸ�Ŀ����
	
	GET_FIFTH_TARGET_FIRST_CROSS,  //��ȡ�����Ŀ����ĵ�һ��·��
	GET_FIFTH_TARGET_SECOND_CROSS,  //��ȡ�����Ŀ����ĵڶ���·��
	GET_FIFTH_TARGET_THIRD_CROSS,  //��ȡ�����Ŀ����ĵڶ���·��
  GET_FIFTH_TARGET,   //��ȡ�����Ŀ����
	PUT_FIFTH_TARGET_FIRST_CROSS,  //���õ����Ŀ����ĵ�һ·��
	PUT_FIFTH_TARGET_SECOND_CROSS,  //���õ����Ŀ����ĵڶ�·��
	PUT_FIFTH_TARGET_THIRD_CROSS,  //���õ����Ŀ����ĵ���·��
	GET_FIFTH_TARGET_FOURTH_CROSS,  //��ȡ�����Ŀ����ĵ��ĸ�·��
	GET_FIFTH_TARGET_FIFTH_CROSS,  //��ȡ�����Ŀ����ĵ���·��
  PUT_FIFTH_TARGET,  //���õ����Ŀ����

}TASK;// ����
/* �궨�� --------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
extern uint8_t Turn_Flag;  //ת���־λ
/* �������� ------------------------------------------------------------------*/
void Task_Timer_Handle(void);
void Task_Delay_Start(unsigned int time);
#endif /* _Task_h_ */

/************************************************ �ļ����� ************************************************/
