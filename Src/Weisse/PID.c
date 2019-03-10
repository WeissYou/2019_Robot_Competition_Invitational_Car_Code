/*****************************************************************************
  * �ļ�����: PID.c
  * ��    ��: ��ΰ��
  * ��д����: ���� 2019
  * ��    ��: PID�㷨
  ****************************************************************************
  * ˵    ��: 
  * 
  *****************************************************************************/
	
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "PID.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
#define  P_DATA      0.725f                              // P����
#define  I_DATA      0.10f                              // I����
#define  D_DATA      0.10f                              // D����
/* ˽�б��� ------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
/* PID�ṹ��ָ�� */
PID_TypeDef  sPID;
PID_TypeDef  *ptr =  &sPID;
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/*************************************
  * ��������: PID������ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  ************************************/
void IncPIDInit(void) 
{
	uint8_t i;
	for(i = 0; i < 4; i ++)
	{
	  ptr->SetPoint[i] = 499;   //�趨Ŀ��Desired Value
    ptr->LastError[i] = 0;            //Error[-1]
    ptr->PrevError[i] = 0;            //Error[-2]
    ptr->Proportion[i] = P_DATA;      //�������� Proportional Const
    ptr->Integral[i] = I_DATA;        //���ֳ���  Integral Const
    ptr->Derivative[i] = D_DATA;      //΢�ֳ��� Derivative Const
	}
}

/*************************************
  * ��������: PID����
  * �������: uint8_t Motor_Number, int16_t NextPoint
  * �� �� ֵ: ptr->Proportion[Motor_Number] * iError //������
  + ptr->Integral[Motor_Number] * ptr->SumError[Motor_Number] //������
  + ptr->Derivative[Motor_Number] * dError  //΢����
  * ˵    ��: ��
  ************************************/
int32_t LocPIDCalc(uint8_t Motor_Number, int16_t NextPoint)
{
  int iError,dError;
  iError = ptr->SetPoint[Motor_Number] - NextPoint; //ƫ��
  if((iError<3 )&& (iError>-3))
    iError = 0;
  ptr->SumError[Motor_Number] += iError; //����
  dError = iError - ptr->LastError[Motor_Number]; //΢��
  ptr->LastError[Motor_Number] = iError;
  return(ptr->Proportion[Motor_Number] * iError //������
  + ptr->Integral[Motor_Number] * ptr->SumError[Motor_Number] //������
  + ptr->Derivative[Motor_Number] * dError); //΢����
}

/*************************************
  * ��������: PID����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: PIDִ�й��̷�װ
  ************************************/
void PID_Function(void)
{
	uint8_t i;
	int16_t Temp[4];  //���жϷ����PWM_Duty

	/* ת�����̼��� */
	Encoder_CaptureNumber[0] = __HAL_TIM_GET_COUNTER(&htim2) + Encoder_OverflowCount[0]*65536;
	Encoder_CaptureNumber[1] = __HAL_TIM_GET_COUNTER(&htim3) + Encoder_OverflowCount[1]*65536;
	Encoder_CaptureNumber[2] = __HAL_TIM_GET_COUNTER(&htim4) + Encoder_OverflowCount[2]*65536;
	Encoder_CaptureNumber[3] = __HAL_TIM_GET_COUNTER(&htim5) + Encoder_OverflowCount[3]*65536;

	/* ת��PWM���ݲ�����PID���� */
	for(i = 0; i < 4; i ++)
	{
		Temp[i] = PWM_Duty[i];
		PWM_Duty[i] = LocPIDCalc(i, Encoder_CaptureNumber[i]);
	}
	
	/* �жϵ�����򲢸�����PWM���� */
	for(i = 0; i < 4; i ++)
	{
		if(Temp[i] > 499)
		{
			PWM_Duty[i] += 499;
		}
		else if(Temp[i] < 499)
		{
			PWM_Duty[i] -= 499;
		}
		else /* if(Temp[i] = 499) */
		{
			PWM_Duty[i] = 499;
		}
	}
	
	/* PWM���������޶� */
	for(i = 0; i < 4; i ++)
	{
		if(PWM_Duty[i]>999)PWM_Duty[i]=999;  
		if(PWM_Duty[i]<0)PWM_Duty[i]=0;
	}
	
	/* �򴮿ڷ���ʵʱ���� */
	printf("���1: ���������� %d �ߵ�ƽʱ�� %d\n",Encoder_CaptureNumber[0],PWM_Duty[0]);
	printf("���2: ���������� %d �ߵ�ƽʱ�� %d\n",Encoder_CaptureNumber[1],PWM_Duty[1]);
	printf("���3: ���������� %d �ߵ�ƽʱ�� %d\n",Encoder_CaptureNumber[2],PWM_Duty[2]);
	printf("���4: ���������� %d �ߵ�ƽʱ�� %d\n",Encoder_CaptureNumber[3],PWM_Duty[3]);
	 
	/* ��ձ��������� */
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	__HAL_TIM_SET_COUNTER(&htim5, 0);
	
	/* ע���µ�PWM���ݵ������ */
	for(i = 0; i < 4; i ++)
	{
		DCMotor_Contrl(i, PWM_Duty[i]);
	}

}
/************************************************ �ļ����� ************************************************/
