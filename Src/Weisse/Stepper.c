/*****************************************************************************
  * �ļ�����: Stepper.c
  * ��    ��: ��ΰ��
  * ��д����: һ�� 2019
  * ��    ��: �����������
  ****************************************************************************
  * ˵    ��:
  * 
  *****************************************************************************/
	
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "Stepper.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/*************************************
  * ��������: ���������ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: �������ʹ��
  ************************************/
void Stepper_Init(void)
{
  HAL_GPIO_WritePin(Stepper_EN_1_GPIO_Port, Stepper_EN_1_Pin, GPIO_PIN_RESET);  //�������1ʧ��
  HAL_GPIO_WritePin(Stepper_EN_2_GPIO_Port, Stepper_EN_2_Pin, GPIO_PIN_RESET);  //�������2ʧ��
  HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);  //�������
  HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);   //�������
}

/*************************************
  * ��������: �����������
  * �������: uint8_t stepper_number, uint8_t dir
  * �� �� ֵ: ��
  * ˵    ��: ��
  ************************************/
void Stepper_Start(uint8_t stepper_number, uint8_t dir)
{
  if(stepper_number == Stepper_1)
  {
    if(dir == Front)
    {
      HAL_GPIO_WritePin(Stepper_DIR_1_GPIO_Port, Stepper_DIR_1_Pin, GPIO_PIN_SET);  //�������1ǰ��
    }
    else /* if(dir == Back) */
    {
      HAL_GPIO_WritePin(Stepper_DIR_1_GPIO_Port, Stepper_DIR_1_Pin, GPIO_PIN_RESET);  //�������1����
    }
    HAL_GPIO_WritePin(Stepper_EN_1_GPIO_Port, Stepper_EN_1_Pin, GPIO_PIN_SET);  //�������1ʹ��
  }
  else /* if(stepper_number == Stepper_2) */
  {
    if(dir == Front)
    {
      HAL_GPIO_WritePin(Stepper_DIR_2_GPIO_Port, Stepper_DIR_2_Pin, GPIO_PIN_SET);  //�������2ǰ��
    }
    else /* if(dir == Back) */
    {
      HAL_GPIO_WritePin(Stepper_DIR_2_GPIO_Port, Stepper_DIR_2_Pin, GPIO_PIN_RESET);  //�������2����
    }
    HAL_GPIO_WritePin(Stepper_EN_2_GPIO_Port, Stepper_EN_2_Pin, GPIO_PIN_SET);  //�������2ʹ��
  }
}

/*************************************
  * ��������: �������ֹͣ
  * �������: uint8_t stepper_number
  * �� �� ֵ: ��
  * ˵    ��: ��
  ************************************/
void Stepper_Stop(uint8_t stepper_number)
{
  if(stepper_number == Stepper_1)
  {
    HAL_GPIO_WritePin(Stepper_EN_1_GPIO_Port, Stepper_EN_1_Pin, GPIO_PIN_RESET);  //�������1ʧ��

  }
  else /* if(stepper_number == Stepper_2) */
  {
    HAL_GPIO_WritePin(Stepper_EN_2_GPIO_Port, Stepper_EN_2_Pin, GPIO_PIN_RESET);  //�������2ʧ��
  }
}

/************************************************ �ļ����� ************************************************/
