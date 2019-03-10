/*****************************************************************************
  * �ļ�����: Vehicle.c
  * ��    ��: ��ΰ��
  * ��д����: ���� 2019
  * ��    ��: �ؾ߿���
  ****************************************************************************
  * ˵    ��:
  * 
  *****************************************************************************/
	
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "Vehicle.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/*************************************
  * ��������: ֱ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  ************************************/
void Straight(void)
{
//		  DCMotor_Contrl(Motor_1, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_2, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_3, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_4, FORWARD_100PERCENT_PWM);
  ptr->SetPoint[0] = _100PERCENT_COUNT;
  ptr->SetPoint[1] = _100PERCENT_COUNT;
  ptr->SetPoint[2] = _100PERCENT_COUNT;
  ptr->SetPoint[3] = _100PERCENT_COUNT;
}

/*************************************
  * ��������: С��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  ************************************/
void Small(uint8_t dir)
{
  if(dir == Right)
  {
//		  DCMotor_Contrl(Motor_1, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_2, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_3, FORWARD_80PERCENT_PWM);
//		  DCMotor_Contrl(Motor_4, FORWARD_80PERCENT_PWM);
    ptr->SetPoint[0] = _100PERCENT_COUNT;
    ptr->SetPoint[1] = _100PERCENT_COUNT;
    ptr->SetPoint[2] = _80PERCENT_COUNT;
    ptr->SetPoint[3] = _80PERCENT_COUNT;
  }
  else /* if(dir == Left) */
  {
//		  DCMotor_Contrl(Motor_1, FORWARD_80PERCENT_PWM);
//		  DCMotor_Contrl(Motor_2, FORWARD_80PERCENT_PWM);
//		  DCMotor_Contrl(Motor_3, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_4, FORWARD_100PERCENT_PWM);
    ptr->SetPoint[0] = _80PERCENT_COUNT;
    ptr->SetPoint[1] = _80PERCENT_COUNT;
    ptr->SetPoint[2] = _100PERCENT_COUNT;
    ptr->SetPoint[3] = _100PERCENT_COUNT;
  }
}

/*************************************
  * ��������: �е�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  ************************************/
void Middle(uint8_t dir)
{
  if(dir == Right)
  {
//		  DCMotor_Contrl(Motor_1, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_2, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_3, FORWARD_50PERCENT_PWM);
//		  DCMotor_Contrl(Motor_4, FORWARD_50PERCENT_PWM);
    ptr->SetPoint[0] = _100PERCENT_COUNT;
    ptr->SetPoint[1] = _100PERCENT_COUNT;
    ptr->SetPoint[2] = _50PERCENT_COUNT;
    ptr->SetPoint[3] = _50PERCENT_COUNT;
  }
  else /* if(dir == Left) */
  {
//		  DCMotor_Contrl(Motor_1, FORWARD_50PERCENT_PWM);
//		  DCMotor_Contrl(Motor_2, FORWARD_50PERCENT_PWM);
//		  DCMotor_Contrl(Motor_3, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_4, FORWARD_100PERCENT_PWM);
    ptr->SetPoint[0] = _50PERCENT_COUNT;
    ptr->SetPoint[1] = _50PERCENT_COUNT;
    ptr->SetPoint[2] = _100PERCENT_COUNT;
    ptr->SetPoint[3] = _100PERCENT_COUNT;
  }
}

/*************************************
  * ��������: ���
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  ************************************/
void Big(uint8_t dir)
{
  if(dir == Right)
  {
//		  DCMotor_Contrl(Motor_1, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_2, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_3, FORWARD_30PERCENT_PWM);
//		  DCMotor_Contrl(Motor_4, FORWARD_30PERCENT_PWM);
    ptr->SetPoint[0] = _100PERCENT_COUNT;
    ptr->SetPoint[1] = _100PERCENT_COUNT;
    ptr->SetPoint[2] = _30PERCENT_COUNT;
    ptr->SetPoint[3] = _30PERCENT_COUNT;
  }
  else /* if(dir == Left) */
  {
//		  DCMotor_Contrl(Motor_1, FORWARD_30PERCENT_PWM);
//		  DCMotor_Contrl(Motor_2, FORWARD_30PERCENT_PWM);
//		  DCMotor_Contrl(Motor_3, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_4, FORWARD_100PERCENT_PWM);
    ptr->SetPoint[0] = _30PERCENT_COUNT;
    ptr->SetPoint[1] = _30PERCENT_COUNT;
    ptr->SetPoint[2] = _100PERCENT_COUNT;
    ptr->SetPoint[3] = _100PERCENT_COUNT;
  }
}

/*************************************
  * ��������: ת��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  ************************************/
void Turn(uint8_t dir)
{
  if(dir == Right)
  {
//		  DCMotor_Contrl(Motor_1, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_2, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_3, BACKWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_4, BACKWARD_100PERCENT_PWM);
    ptr->SetPoint[0] = _100PERCENT_COUNT;
    ptr->SetPoint[1] = _100PERCENT_COUNT;
    ptr->SetPoint[2] = _100PERCENT_COUNT;
    ptr->SetPoint[3] = _100PERCENT_COUNT;
  }
  else /* if(dir == Left) */
  {
//		  DCMotor_Contrl(Motor_1, BACKWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_2, BACKWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_3, FORWARD_100PERCENT_PWM);
//		  DCMotor_Contrl(Motor_4, FORWARD_100PERCENT_PWM);
    ptr->SetPoint[0] = _100PERCENT_COUNT;
    ptr->SetPoint[1] = _100PERCENT_COUNT;
    ptr->SetPoint[2] = _100PERCENT_COUNT;
    ptr->SetPoint[3] = _100PERCENT_COUNT;
  }
}

/*************************************
  * ��������: ͣ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  ************************************/
void Vehicle_Stop(void)
{
//		  DCMotor_Contrl(Motor_1, STOP_PWM);
//		  DCMotor_Contrl(Motor_2, STOP_PWM);
//		  DCMotor_Contrl(Motor_3, STOP_PWM);
//		  DCMotor_Contrl(Motor_4, STOP_PWM);
    ptr->SetPoint[0] = NO_COUNT;
    ptr->SetPoint[1] = NO_COUNT;
    ptr->SetPoint[2] = NO_COUNT;
    ptr->SetPoint[3] = NO_COUNT;
}

/************************************************ �ļ����� ************************************************/
