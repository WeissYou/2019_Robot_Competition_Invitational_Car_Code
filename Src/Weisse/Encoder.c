/*****************************************************************************
  * �ļ�����: Encoder.c
  * ��    ��: ��ΰ��
  * ��д����: һ�� 2019
  * ��    ��: 
  ****************************************************************************
  * ˵    ��: 
  * 
  *****************************************************************************/
	
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "Encoder.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
int16_t Encoder_OverflowCount[4] = {0};  //�������������
__IO int16_t Encoder_CaptureNumber[4] = {0};  // ���벶����
uint16_t time_count = 0;
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/*************************************
  * ��������: ��������ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  ************************************/
void Encoder_Init(void)
{
	/* ���������� */
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);  
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);  
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);  
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);  
}

/*************************************
  * ��������: �������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  ************************************/
void Motor_Speed_Measurement(void)
{
	float Objective_Speed[4] = {0};  //ʵ���ٶ�
  Encoder_CaptureNumber[0] = __HAL_TIM_GET_COUNTER(&htim2) + Encoder_OverflowCount[0]*65536;
  Encoder_CaptureNumber[1] = __HAL_TIM_GET_COUNTER(&htim3) + Encoder_OverflowCount[1]*65536;
  Encoder_CaptureNumber[2] = __HAL_TIM_GET_COUNTER(&htim4) + Encoder_OverflowCount[2]*65536;
	Encoder_CaptureNumber[3] = __HAL_TIM_GET_COUNTER(&htim5) + Encoder_OverflowCount[3]*65536;
	printf("���1���̼���: %d \n",Encoder_CaptureNumber[0]);
	Objective_Speed[0] = (float)(1200*Encoder_CaptureNumber[0])/PPR;
  printf("���1ʵ��ת��: %0.2f rpm \n",Objective_Speed[0]);
	printf("\n");
	printf("���2���̼���: %d \n",Encoder_CaptureNumber[0]);
	Objective_Speed[1] = (float)(1200*Encoder_CaptureNumber[0])/PPR;
  printf("���2ʵ��ת��: %0.2f rpm \n",Objective_Speed[1]);
	printf("\n");
	printf("���3���̼���: %d \n",Encoder_CaptureNumber[0]);
	Objective_Speed[2] = (float)(1200*Encoder_CaptureNumber[0])/PPR;
  printf("���3ʵ��ת��: %0.2f rpm \n",Objective_Speed[2]);
	printf("\n");
	printf("���4���̼���: %d \n",Encoder_CaptureNumber[0]);
	Objective_Speed[3] = (float)(1200*Encoder_CaptureNumber[0])/PPR;
  printf("���4ʵ��ת��: %0.2f rpm \n",Objective_Speed[3]);
	printf("\n");
  __HAL_TIM_SET_COUNTER(&htim2, 0);
}

/************************************************ �ļ����� ************************************************/
