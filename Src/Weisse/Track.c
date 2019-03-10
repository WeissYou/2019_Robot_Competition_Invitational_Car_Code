/*****************************************************************************
  * �ļ�����: Track.c
  * ��    ��: ��ΰ��
  * ��д����: һ�� 2019
  * ��    ��: ѭ�����
  ****************************************************************************
  * ˵    ��:
  * 
  *****************************************************************************/
	
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "Track.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
uint8_t Threshold_1;  //ǰ��·��ֵ
uint8_t Threshold_2;  //����·��ֵ
uint8_t Situation[7] = {0};
uint32_t Last_Objective_Track;
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/*************************************
  * ��������: ����ѭ���߼�
  * �������: uint8_t track_recognition
  * �� �� ֵ: ��
  * ˵    ��: ��
  ************************************/
void Basic_Track(uint8_t track_recognition)
{
	LED_Blink(track_recognition);  //LED����
	switch(track_recognition)
	{
		case 0:Big(Left);    break;
		case 1:Middle(Left); break;
		case 2:Small(Left);  break;
    case 3:Straight();   break;
		case 4:Small(Right); break;
		case 5:Middle(Right);break;
		case 6:Big(Right);   break;
	}
}//End of Basic_Track(void)

/*************************************
  * ��������: �Զ���ֵ�㷨
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ΪӦ�ԻҶȴ���������ȱ�ݣ��ڴ˽����������ν���
  ************************************/
void Automatic_Threshold(void)
{
	 uint8_t i;
	 uint8_t value1_max, value1_min;  //ǰ��·�Ҷ�ֵ
	 uint8_t value2_max, value2_min;  //����·�Ҷ�ֵ
	 value1_max = ADC_ConvertedValueLocal[0];  //��̬��ֵ�㷨����ȡ������Сֵ
   value2_max = ADC_ConvertedValueLocal[4];  
   for(i = 0; i < 4; i ++)   
   {
     if(value1_max <= ADC_ConvertedValueLocal[i])
		 {
       value1_max = ADC_ConvertedValueLocal[i];
		 }
   }
	 for(i = 4; i < 7; i ++)
	 {
		 if(value2_max <= ADC_ConvertedValueLocal[i])
		 {
       value2_max = ADC_ConvertedValueLocal[i];
		 }
	 }
	 value1_min = ADC_ConvertedValueLocal[0];  //��Сֵ
	 value2_min = ADC_ConvertedValueLocal[4];  
   for(i = 0; i < 7; i ++) 
   {
     if(value1_min >= ADC_ConvertedValueLocal[i])
		 {
       value1_min = ADC_ConvertedValueLocal[i];
		 }
   }
	 for(i = 4; i < 7; i ++)
	 {
		 if(value2_min <= ADC_ConvertedValueLocal[i])
		 {
       value2_min = ADC_ConvertedValueLocal[i];
		 }
	 }
   Threshold_1 = ((value1_max + value1_min) / 2) - 5;	  //���������������ȡ����ֵ
	 Threshold_2 = ((value2_min) / 2);
}//End of void Automatic_Threshold(void)

/*************************************
  * ��������: �򵥶�ֵ���㷨
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  ************************************/
uint8_t Basic_Binary_Code(void)
{
	uint8_t i;
	static uint8_t Objective_Track;  //ʵ���ϵĺ���λ�á��ϴ�ʵ���ϵĺ���λ��

  for(i = 0; i < 4; i ++)
	{
	  if(ADC_ConvertedValueLocal[i] <= Threshold_1)
		{
			Objective_Track |= 0x01 << i;
			Situation[i] = 1;
		}
		else /* if(ADC_ConvertedValueLocal[i] < Threshold) */
		{
			Objective_Track &= ~(0x01 << i);
			Situation[i] = 0;
		}
	}
  for(i = 4; i < 7; i ++)
	{
	  if(ADC_ConvertedValueLocal[i] <= Threshold_2)
		{
			Objective_Track |= 0x01 << i;
			Situation[i] = 1;
		}
		else /* if(ADC_ConvertedValueLocal[i] < Threshold) */
		{
			Objective_Track &= ~(0x01 << i);
			Situation[i] = 0;
		}
	}
	Last_Objective_Track = Situation[0] + 10 * Situation[1] + 100 * Situation[2] + 1000 * Situation[3] + 10000 * Situation[4] + 100000 * Situation[5] + 1000000 * Situation[6];
	//Last_Objective_Track = Objective_Track;  //������һ�εĹ켣
	return Objective_Track;
}
uint8_t Track_(void)
{
	int8_t i, left, right, track_;
	left = right = track_ = 0;
	for(i = 0; i < 7; i ++)
	{
		if(Situation[i] == 1)
		{
			left = i;
			break;
		}
	}
	for(i = 6; i > -1; i --)
	{
		if(Situation[i] == 1)
		{
			right = i;
			break;
		}
	}
	track_ = (left + right) / 2;
	return track_;
}

/*************************************
  * ��������: �켣ʶ���㷨
  * �������: ��
  * �� �� ֵ: uint8_t Calculated_Track
  * ˵    ��: ��
  ************************************/
uint8_t Track_Recognition(void)
{
	 uint8_t left, right, Calculated_Track;  //����õ��ĺ���λ��
//         uint8_t Last_Calculated_Track;  //�ϴμ���õ��ĺ���λ��
	 uint8_t Temp_Track_1, Temp_Track_2;  //�����õ���ʱ����ֵ
	 left = right = 0;
	 int8_t i;
	 for(i = 0; i < 4; i++)   //�����Ѱ�ҹ켣
	 {
		 if(ADC_ConvertedValueLocal[i] <= Threshold_1 /*|| ADC_ConvertedValueLocal[i + 1] <= Threshold_1*/)
		 {
		   left = i;
			 break;	
		 }
		 else 
		 {
			 left = 3;
//			 break;
		 }
	 }
	 for(i = 3; i > -1; i--)  //���Ҳ�Ѱ�ҹ켣
   {
		 if(ADC_ConvertedValueLocal[i] <= Threshold_1 /*|| ADC_ConvertedValueLocal[i + 1] <= Threshold_1*/)
		 {
		   right = i;
		   break;	
		 }
		 else
		 {
			 right = 3;
//			 break;
		 }
   }
	 Temp_Track_1 = (right + left) / 2;  //��������λ��
//	 Temp_Track_1 = left;  //��������λ��
//	 left = right = 0;
	 for(i = 4; i < 7; i++)   //�����Ѱ�ҹ켣
	 {
		 if(ADC_ConvertedValueLocal[i] <= Threshold_2 /*|| ADC_ConvertedValueLocal[i + 1] <= Threshold_2*/)
		 {
		   left = i;
			 break;	
		 }
		 else
		 {
			 left = 3;
//			 break;
		 }
	 }
	 for(i = 6; i > 3; i--)  //���Ҳ�Ѱ�ҹ켣
   {
		 if(ADC_ConvertedValueLocal[i] <= Threshold_2 /*|| ADC_ConvertedValueLocal[i + 1] <= Threshold_2*/)
		 {
		   right = i;
		   break;	
		 }
		 else
		 {
			 right = 3;
//			 break;
		 }
   }
	 Temp_Track_2 = (right + left) / 2;  //��������λ��
//	 Temp_Track_2 = right;  //��������λ��
//	 if(abs(Calculated_Track - Last_Calculated_Track) > 4)   //�������ߵ�ƫ����̫��
//	 {
//	   Calculated_Track = Last_Calculated_Track;    //��ȡ��һ�ε�ֵ
//	 }
	 Calculated_Track = (Temp_Track_1 + Temp_Track_2) / 2;
//	 Last_Calculated_Track = Calculated_Track;  //������һ�εĹ켣
	 return Calculated_Track;
}//End of Track_Recognition(void)
/************************************************ �ļ����� ************************************************/
