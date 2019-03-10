/*****************************************************************************
  * �ļ�����: LED.c
  * ��    ��: ��ΰ��
  * ��д����: һ�� 2019
  * ��    ��: LEDָʾ�����
  ****************************************************************************
  * ˵    ��: �������ⷨ���ߵ�ƽ��Ч
  * 
  *****************************************************************************/
	
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "LED.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/*************************************
  * ��������: LED��ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: Ĭ�ϲ�����
  ************************************/
void LED_Init(void)
{
        LED_1_OFF; 
	LED_2_OFF;
	LED_3_OFF;
	LED_4_OFF;
	LED_5_OFF;
	LED_6_OFF;
	LED_7_OFF;
}//End of void LED_Init(void)

/*************************************
  * ��������: LED����
  * �������: uint8_t track_position
  * �� �� ֵ: ��
  * ˵    ��: ��Ϲ켣ʶ���㷨ʹ��
  ************************************/
void LED_Blink(uint8_t calculated_track)
{
	switch(calculated_track)
	{
		case 0:  LED_1_ON;
		case 1:  LED_2_ON;
		case 2:  LED_3_ON;
		case 3:  LED_4_ON;
		case 4:  LED_5_ON;
		case 5:  LED_6_ON;
		case 6:  LED_7_ON;
	}
}//End of void LED_Blink(uint8_t Calculated_Track)

/*************************************
  * ��������: LED����
  * �������: uint16_t Objective_Track
  * �� �� ֵ: ��
  * ˵    ��: ���һ���ֵ��ʹ��
  ************************************/
void LED_blink(uint8_t objective_track)
{
    if (objective_track&(0x01 << 0))
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

    if (objective_track&(0x01 << 1))
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

    if (objective_track&(0x01 << 2))
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

    if (objective_track&(0x01 << 3))
      HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);

    if (objective_track&(0x01 << 4))
      HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);

    if (objective_track&(0x01 << 5))
      HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);

    if (objective_track&(0x01 << 6))
      HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET);
}//End of void LED_blink(uint8_t Objective_Track)

/************************************************ �ļ����� ************************************************/
