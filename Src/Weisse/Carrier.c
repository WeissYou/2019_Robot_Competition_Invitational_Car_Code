/*****************************************************************************
  * �ļ�����: Carrier.c
  * ��    ��: ��ΰ��
  * ��д����: ���� 2019
  * ��    ��: װ���������
  ****************************************************************************
  * ˵    ��:
  * 
  *****************************************************************************/
	
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "Carrier.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
uint8_t Arm_Status = 0;
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/*************************************
  * ��������: ��е�۵���
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  ************************************/
void Carrier_Init(void)
{
  Arm_Control(Vertical);  //��е�۴�ֱ�ڵ�
  Paw_Control(Open);  //��צ�ſ�
  if(HAL_GPIO_ReadPin(Limited_Switch_1_GPIO_Port, Limited_Switch_1_Pin) == GPIO_PIN_RESET)  //����λ����1�ѱ�����
  {
    Stepper_Stop(Stepper_1);  //�������1ֹͣ
  }
  else  /* if(HAL_GPIO_ReadPin(Limited_Switch_1_GPIO_Port, Limited_Switch_1_Pin) == GPIO_PIN_SET)  //����λ����1δ������ */
  {
    Stepper_Start(Stepper_1, Backward);  //�������1����˶�
  }
  if(HAL_GPIO_ReadPin(Limited_Switch_3_GPIO_Port, Limited_Switch_3_Pin) == GPIO_PIN_RESET)  //����λ����2�ѱ�����
  {
    Stepper_Stop(Stepper_2);  //�������2ֹͣ
  }
  else  /* if(HAL_GPIO_ReadPin(Limited_Switch_3_GPIO_Port, Limited_Switch_3_Pin) == GPIO_PIN_SET)  //����λ����2δ������ */
  {
    Stepper_Start(Stepper_2, Backward);  //�������2����˶�
  }
}

/*************************************
  * ��������: ץȡһ��Ŀ����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  ************************************/
void Get_a_Target(void)
{
  Arm_Control(Horizontal);  //��е�۴�ֱ�ڵ�
  if(Arm_Status == In_Motion)
  {
    if(HAL_GPIO_ReadPin(Limited_Switch_1_GPIO_Port, Limited_Switch_1_Pin) == GPIO_PIN_RESET)  //����λ����1�ѱ�����
    {
      Stepper_Stop(Stepper_1);  //�������1ֹͣ
    }
    else  /* if(HAL_GPIO_ReadPin(Limited_Switch_1_GPIO_Port, Limited_Switch_1_Pin) == GPIO_PIN_SET)  //����λ����1δ������ */
    {
      Stepper_Start(Stepper_1, Backward);  //�������1����˶�
    }
    if(HAL_GPIO_ReadPin(Limited_Switch_3_GPIO_Port, Limited_Switch_3_Pin) == GPIO_PIN_RESET)  //����λ����2�ѱ�����
    {
      Stepper_Stop(Stepper_2);  //�������2ֹͣ
    }
    else  /* if(HAL_GPIO_ReadPin(Limited_Switch_3_GPIO_Port, Limited_Switch_3_Pin) == GPIO_PIN_SET)  //����λ����2δ������ */
    {
      Stepper_Start(Stepper_2, Backward);  //�������2����˶�
    }
  }
  else /* if(Arm_Status == Destination) */
  {
    Paw_Control(Close);  //��צ�պ�
  }
}	

/*************************************
  * ��������: װ��һ��Ŀ����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  ************************************/
void Put_a_Target(void)
{
  if(HAL_GPIO_ReadPin(Limited_Switch_1_GPIO_Port, Limited_Switch_1_Pin) == GPIO_PIN_RESET)  //����λ����1�ѱ�����
  {
    Stepper_Stop(Stepper_1);  //�������1ֹͣ
  }
  else  /* if(HAL_GPIO_ReadPin(Limited_Switch_1_GPIO_Port, Limited_Switch_1_Pin) == GPIO_PIN_SET)  //����λ����1δ������ */
  {
    Stepper_Start(Stepper_1, Backward);  //�������1����˶�
  }
  if(HAL_GPIO_ReadPin(Limited_Switch_3_GPIO_Port, Limited_Switch_3_Pin) == GPIO_PIN_RESET)  //����λ����2�ѱ�����
  {
    Stepper_Stop(Stepper_2);  //�������2ֹͣ
  }
  else  /* if(HAL_GPIO_ReadPin(Limited_Switch_3_GPIO_Port, Limited_Switch_3_Pin) == GPIO_PIN_SET)  //����λ����2δ������ */
  {
    Stepper_Start(Stepper_2, Backward);  //�������2����˶�
  }
}

/************************************************ �ļ����� ************************************************/
