/*****************************************************************************
  * �ļ�����: Task.c
  * ��    ��: ��ΰ��
  * ��д����: һ�� 2019
  * ��    ��: �������
  ****************************************************************************
  * ˵    ��:
  * 
  *****************************************************************************/
	
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "Task.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
#define START_AREA_GREY 200  //��ʼ���Ҷ�ֵ
/* ˽�б��� ------------------------------------------------------------------*/
volatile uint8_t Task = IN_START_AREA;  // ����״̬
volatile uint16_t Task_Delay_Time_Cnt = 0;  // �����е���������ʱʱ�����
/* ��չ���� ------------------------------------------------------------------*/
uint8_t Turn_Count = 0;  //ת�����
uint8_t Turn_Flag = 0;  //ת���־λ
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/*************************************
  * ��������: ��ʱ���ص�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  ************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2))
	{
    Encoder_OverflowCount[0]--;       //���¼������
	}
  else
	{
    Encoder_OverflowCount[0]++;       //���ϼ������
	}
  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
	{
    Encoder_OverflowCount[1]--;       //���¼������
	}
  else
	{
    Encoder_OverflowCount[1]++;       //���ϼ������
	}
  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4))
	{
    Encoder_OverflowCount[2]--;       //���¼������
	}
  else
	{
    Encoder_OverflowCount[2]++;       //���ϼ������
	}
  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5))
	{
    Encoder_OverflowCount[3]--;       //���¼������
	}
  else
	{
    Encoder_OverflowCount[3]++;       //���ϼ������
	}
  if(htim->Instance == TIM7) 
  {
		uint8_t i = 0;
		uint8_t track = 0;		
		Get_ADC_Data();  //��ȡ�Ҷȴ�������ֵ
		for(i = 0; i < 7; i ++)
		{
			printf("��%d���������Ĳ���ֵ��%d \n",(i + 1), ADC_ConvertedValueLocal[i]);
		}
    printf("\n");
		Automatic_Threshold();  //�Զ���ֵ�㷨
		track = Basic_Binary_Code();
		printf("ǰ��·��ֵ��%d ; \n����·��ֵ�� %d \n ",Threshold_1, Threshold_2);
		printf("\n");
		printf("��·��ֵ�������%d \n",Last_Objective_Track);
		printf("\n");
		track = Track_();
		printf("����λ�ã���%d·������ \n",(track + 1));
//		printf("\n");
    //Task_Timer_Handle();  //��������
    //Buzzer_Timer_Handle();  //������������   
	  //Motor_Speed_Measurement();  //�������
		//PID_Function();  //PIDִ�к���

  }
}

/*******************************************************************************
*                           ������@2018-11-24
* Function Name  :  Task_Delay_Start
* Description    :  ������ʱ��ʼ
* Input          :  None
* Output         :  None
* Return         :  None
*******************************************************************************/
void Task_Delay_Start(unsigned int time)
{
    // ������ʱʱ��
    Task_Delay_Time_Cnt = time;
}// End of void Task_Delay_Start(unsigned int time)

/*******************************************************************************
*                           ������@2018-11-24
* Function Name  :  Task_Delay_Is_Delay
* Description    :  ������ʱ�Ƿ�����ʱ��
* Input          :  None
* Output         :  None
* Return         :  None
*******************************************************************************/
unsigned char Task_Delay_Is_Delay(void)
{
    if(Task_Delay_Time_Cnt>0)return 1;
    return 0;
}// End of unsigned char Task_Delay_Is_Delay(void)

/*******************************************************************************
*                           ������@2018-11-24
* Function Name  :  Task_Timer_Handle
* Description    :  ����ʱ��������
* Input          :  None
* Output         :  None
* Return         :  None
*******************************************************************************/
void Task_Timer_Handle(void)
{
  // ���������ʱʱ���������ֵ,��������ʱʱ�����--
  if(Task_Delay_Time_Cnt>0)Task_Delay_Time_Cnt--;
	switch(Task)
	{
		/* �ڳ����� */
		case IN_START_AREA:  
		{
			LED_blink(Basic_Binary_Code());  //����
			if(Threshold_1 >= START_AREA_GREY)  //����ֵΪ����������ֵ   ///////////////////�˴�����ʱ�Ķ�//////////////////////////
			{
				Straight();  //С��ǰ��
			}
			else  //����
			{
        Task = OUT_OF_START_AREA;  //�л�����״̬����ʻ���������
			}
			break;
		}
		
		/* ʻ������� */
		case OUT_OF_START_AREA:  
		{
			if(HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_SET)  //���Ҳഫ����û�м�⵽����
			{
				Basic_Track(Track_Recognition());  //ִ�л���ѭ���߼�
			}
			else  //����
			{
				Turn_Flag = 1;  //����ת���־λ
			  Task_Delay_Start(10);  //��ʱ500ms��ʮ����ʱ�����ڣ�
				Task = GET_FIRST_TARGET_FIRST_CROSS;  //�л�����״̬������ȡ��һ��Ŀ����ĵ�һ��·�ڡ�
			}
			break;
		}
		
		/* ��ȡ��һ��Ŀ����ĵ�һ��·�� */
		case GET_FIRST_TARGET_FIRST_CROSS:  
		{
			if(Turn_Flag)  //����Ҫת��
			{
				Turn(Right);  //��ת
			}
			if(Task_Delay_Is_Delay() == 0)  //����ʱ����
			{
			  /* �����洫����������һ��û�м�⵽���� */
			  if((HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_SET) || (HAL_GPIO_ReadPin(Left_Sensor_GPIO_Port, Left_Sensor_Pin) == GPIO_PIN_SET))
			  {
				  /* ������� */
				  /* ���Ҳഫ������⵽���ߵ����û�� */
				  if((HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_RESET) && (HAL_GPIO_ReadPin(Left_Sensor_GPIO_Port, Left_Sensor_Pin) == GPIO_PIN_SET))
          {
				    Turn(Right);  //���ҵ���
				  }
				  /* ����ഫ������⵽���ߵ��Ҳ�û�� */
				  else if((HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_SET) && (HAL_GPIO_ReadPin(Left_Sensor_GPIO_Port, Left_Sensor_Pin) == GPIO_PIN_RESET))
          {
				    Turn(Left);  //�������
				  }
			  }
			  else  //�����ഫ��������⵽����
			  {
				  Motion_Stop();  //ͣ��
				  Arm_Status = IN_GETTING_TASK;  //��е�۽��롰ץȡ��״̬
				  Task = GET_FIRST_TARGET;  //�л�����״̬����ץȡ��һ��Ŀ���
			  }
			}
			break;
		}
		
		/* ��ȡ��һ��Ŀ���� */
		case GET_FIRST_TARGET:  
    {
			if(Arm_Status == NO_TASK)  //����е�����������
			{
				if(Turn_Flag)  //����Ҫת��
				{
				  Turn(Left);  //��ת
					if(Task_Delay_Is_Delay() == 0)  //����ʱ����
				  {
			      /* ���Ҳഫ������⵽���� */
			      if(HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_RESET)
			      {
				      Motion_Stop();  //ͣ��
				      Turn_Flag = 0;  //���ת���־
			      }
          }
				}
        else
				{
					if(HAL_GPIO_ReadPin(Left_Sensor_GPIO_Port, Left_Sensor_Pin) == GPIO_PIN_SET)  //����ഫ����û�м�⵽����
					{
					  Basic_Track(Track_Recognition());  //ִ�л���ѭ���߼�
					}
					else
					{
						Motion_Stop();  //ͣ��
				    Turn_Flag = 1;  //����ת���־λ
			      Task_Delay_Start(4);  //��ʱ200ms���ĸ���ʱ�����ڣ�
				    Task = PUT_FIRST_TARGET_FIRST_CROSS;  //�л�����״̬�������õ�һ��Ŀ����ĵ�һ·�ڡ�
					}
				}
      }
			break;
    }
		
		/* ���õ�һ��Ŀ����ĵ�һ��·�� */
		case PUT_FIRST_TARGET_FIRST_CROSS:
		{
		  if(Turn_Flag)  //����Ҫת��
		  {
			  Turn(Left);  //��ת
				if(Task_Delay_Is_Delay() == 0)  //����ʱ����
				{
			    /* ����ഫ������⵽���� */
			    if(HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_RESET)
			    {
				    Motion_Stop();  //ͣ��
				    Turn_Flag = 0;  //���ת���־
			    }
        }
		  }
      else
			{
			  if(HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_SET)  //���Ҳഫ����û�м�⵽����
				{
					Basic_Track(Track_Recognition());  //ִ�л���ѭ���߼�
			  }
				else
				{
				  Motion_Stop();  //ͣ��
				  Turn_Flag = 1;  //����ת���־λ
			    Task_Delay_Start(2);  //��ʱ100ms��������ʱ�����ڣ�
				  Task = PUT_FIRST_TARGET_SECOND_CROSS;  //�л�����״̬�������õ�һ��Ŀ����ĵڶ�·�ڡ�
				}
			}
		}
		
		/* ���õ�һ��Ŀ����ĵڶ���·�� */
		case PUT_FIRST_TARGET_SECOND_CROSS:
		{
		  if(Turn_Flag)  //����Ҫת��
		  {
			  Turn(Left);  //��ת
				if(Task_Delay_Is_Delay() == 0)  //����ʱ����
				{
			    /* ����ഫ������⵽���� */
			    if(HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_RESET)
			    {
				    Motion_Stop();  //ͣ��
				    Turn_Flag = 0;  //���ת���־
			    }
        }
		  }
      else
			{
			  if(HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_SET)  //���Ҳഫ����û�м�⵽����
				{
					Basic_Track(Track_Recognition());  //ִ�л���ѭ���߼�
			  }
				else
				{
				  Motion_Stop();  //ͣ��
				  Turn_Flag = 1;  //����ת���־λ
			    Task_Delay_Start(10);  //��ʱ500ms��ʮ����ʱ�����ڣ�
				  Task = PUT_FIRST_TARGET_THIRD_CROSS;  //�л�����״̬�������õ�һ��Ŀ����ĵ���·�ڡ�
				}
			}
		}
		
		/* ���õ�һ��Ŀ����ĵ�����·�� */
		case PUT_FIRST_TARGET_THIRD_CROSS:
		{
			if(Turn_Flag)  //����Ҫת��
			{
				Turn(Right);  //��ת
			}
			if(Task_Delay_Is_Delay() == 0)  //����ʱ����
			{
			  /* �����洫����������һ��û�м�⵽���� */
			  if((HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_SET) || (HAL_GPIO_ReadPin(Left_Sensor_GPIO_Port, Left_Sensor_Pin) == GPIO_PIN_SET))
			  {
				  /* ������� */
				  /* ���Ҳഫ������⵽���ߵ����û�� */
				  if((HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_RESET) && (HAL_GPIO_ReadPin(Left_Sensor_GPIO_Port, Left_Sensor_Pin) == GPIO_PIN_SET))
          {
				    Turn(Right);  //���ҵ���
				  }
				  /* ����ഫ������⵽���ߵ��Ҳ�û�� */
				  else if((HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_SET) && (HAL_GPIO_ReadPin(Left_Sensor_GPIO_Port, Left_Sensor_Pin) == GPIO_PIN_RESET))
          {
				    Turn(Left);  //�������
				  }
			  }
			  else  //�����ഫ��������⵽����
			  {
				  Motion_Stop();  //ͣ��
				  Arm_Status = IN_PUTTING_TASK;  //��е�۽��롰���á�״̬
				  Task = PUT_FIRST_TARGET;  //�л�����״̬�������õ�һ��Ŀ���
			  }
			}
			break;
		}
		
		/* ���õ�һ����־�� */
		case PUT_FIRST_TARGET:
		{
      if(Arm_Status == NO_TASK)  //����е�����������
			{
				if(Turn_Flag)  //����Ҫת��
				{
				  Turn(Left);  //��ת
					if(Task_Delay_Is_Delay() == 0)  //����ʱ����
				  {
			      /* ���Ҳഫ������⵽���� */
			      if(HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_RESET)
			      {
				      Motion_Stop();  //ͣ��
				      Turn_Flag = 0;  //���ת���־
			      }
          }
				}
        else
				{
					if(HAL_GPIO_ReadPin(Left_Sensor_GPIO_Port, Left_Sensor_Pin) == GPIO_PIN_SET)  //����ഫ����û�м�⵽����
					{
					  Basic_Track(Track_Recognition());  //ִ�л���ѭ���߼�
					}
					else
					{
						Motion_Stop();  //ͣ��
				    Turn_Flag = 1;  //����ת���־λ
			      Task_Delay_Start(4);  //��ʱ200ms���ĸ���ʱ�����ڣ�
				    Task = GET_SECOND_TARGET_FIRST_CROSS;  //�л�����״̬������ȡ�ڶ���Ŀ����ĵ�һ·�ڡ�
					}
				}
      }
			break;
		}
		case GET_SECOND_TARGET_FIRST_CROSS:
		{
			
		}
  }
}// End of void Task_Timer_Handle(void)
/************************************************ �ļ����� ************************************************/
