/*****************************************************************************
  * 文件名称: Task.c
  * 作    者: 尤伟宏
  * 编写日期: 一月 2019
  * 功    能: 任务管理
  ****************************************************************************
  * 说    明:
  * 
  *****************************************************************************/
	
/* 包含头文件 ----------------------------------------------------------------*/
#include "Task.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
#define START_AREA_GREY 200  //开始区灰度值
/* 私有变量 ------------------------------------------------------------------*/
volatile uint8_t Task = IN_START_AREA;  // 任务状态
volatile uint16_t Task_Delay_Time_Cnt = 0;  // 任务中的无阻塞延时时间计数
/* 扩展变量 ------------------------------------------------------------------*/
uint8_t Turn_Count = 0;  //转向计数
uint8_t Turn_Flag = 0;  //转向标志位
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/*************************************
  * 函数功能: 定时器回调函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  ************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2))
	{
    Encoder_OverflowCount[0]--;       //向下计数溢出
	}
  else
	{
    Encoder_OverflowCount[0]++;       //向上计数溢出
	}
  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
	{
    Encoder_OverflowCount[1]--;       //向下计数溢出
	}
  else
	{
    Encoder_OverflowCount[1]++;       //向上计数溢出
	}
  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4))
	{
    Encoder_OverflowCount[2]--;       //向下计数溢出
	}
  else
	{
    Encoder_OverflowCount[2]++;       //向上计数溢出
	}
  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5))
	{
    Encoder_OverflowCount[3]--;       //向下计数溢出
	}
  else
	{
    Encoder_OverflowCount[3]++;       //向上计数溢出
	}
  if(htim->Instance == TIM7) 
  {
		uint8_t i = 0;
		uint8_t track = 0;		
		Get_ADC_Data();  //获取灰度传感器数值
		for(i = 0; i < 7; i ++)
		{
			printf("第%d个传感器的采样值：%d \n",(i + 1), ADC_ConvertedValueLocal[i]);
		}
    printf("\n");
		Automatic_Threshold();  //自动阈值算法
		track = Basic_Binary_Code();
		printf("前四路阈值：%d ; \n后三路阈值： %d \n ",Threshold_1, Threshold_2);
		printf("\n");
		printf("各路二值化情况：%d \n",Last_Objective_Track);
		printf("\n");
		track = Track_();
		printf("黑线位置：第%d路传感器 \n",(track + 1));
//		printf("\n");
    //Task_Timer_Handle();  //任务处理函数
    //Buzzer_Timer_Handle();  //蜂鸣器处理函数   
	  //Motor_Speed_Measurement();  //电机测速
		//PID_Function();  //PID执行函数

  }
}

/*******************************************************************************
*                           陈苏阳@2018-11-24
* Function Name  :  Task_Delay_Start
* Description    :  任务延时开始
* Input          :  None
* Output         :  None
* Return         :  None
*******************************************************************************/
void Task_Delay_Start(unsigned int time)
{
    // 设置延时时间
    Task_Delay_Time_Cnt = time;
}// End of void Task_Delay_Start(unsigned int time)

/*******************************************************************************
*                           陈苏阳@2018-11-24
* Function Name  :  Task_Delay_Is_Delay
* Description    :  任务延时是否处于延时中
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
*                           陈苏阳@2018-11-24
* Function Name  :  Task_Timer_Handle
* Description    :  任务定时器处理函数
* Input          :  None
* Output         :  None
* Return         :  None
*******************************************************************************/
void Task_Timer_Handle(void)
{
  // 如果任务延时时间计数还有值,则任务延时时间计数--
  if(Task_Delay_Time_Cnt>0)Task_Delay_Time_Cnt--;
	switch(Task)
	{
		/* 在出发区 */
		case IN_START_AREA:  
		{
			LED_blink(Basic_Binary_Code());  //发光
			if(Threshold_1 >= START_AREA_GREY)  //若阈值为出发区的阈值   ///////////////////此处有临时改动//////////////////////////
			{
				Straight();  //小车前进
			}
			else  //否则
			{
        Task = OUT_OF_START_AREA;  //切换任务状态至“驶离出发区”
			}
			break;
		}
		
		/* 驶离出发区 */
		case OUT_OF_START_AREA:  
		{
			if(HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_SET)  //若右侧传感器没有检测到黑线
			{
				Basic_Track(Track_Recognition());  //执行基础循迹逻辑
			}
			else  //否则
			{
				Turn_Flag = 1;  //放置转向标志位
			  Task_Delay_Start(10);  //延时500ms（十个定时器周期）
				Task = GET_FIRST_TARGET_FIRST_CROSS;  //切换任务状态至“获取第一个目标物的第一个路口”
			}
			break;
		}
		
		/* 获取第一个目标物的第一个路口 */
		case GET_FIRST_TARGET_FIRST_CROSS:  
		{
			if(Turn_Flag)  //若需要转向
			{
				Turn(Right);  //右转
			}
			if(Task_Delay_Is_Delay() == 0)  //若延时结束
			{
			  /* 若侧面传感器至少有一个没有检测到黑线 */
			  if((HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_SET) || (HAL_GPIO_ReadPin(Left_Sensor_GPIO_Port, Left_Sensor_Pin) == GPIO_PIN_SET))
			  {
				  /* 区分情况 */
				  /* 若右侧传感器检测到黑线但左侧没有 */
				  if((HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_RESET) && (HAL_GPIO_ReadPin(Left_Sensor_GPIO_Port, Left_Sensor_Pin) == GPIO_PIN_SET))
          {
				    Turn(Right);  //向右调节
				  }
				  /* 若左侧传感器检测到黑线但右侧没有 */
				  else if((HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_SET) && (HAL_GPIO_ReadPin(Left_Sensor_GPIO_Port, Left_Sensor_Pin) == GPIO_PIN_RESET))
          {
				    Turn(Left);  //向左调节
				  }
			  }
			  else  //若两侧传感器均检测到黑线
			  {
				  Motion_Stop();  //停车
				  Arm_Status = IN_GETTING_TASK;  //机械臂进入“抓取“状态
				  Task = GET_FIRST_TARGET;  //切换任务状态至“抓取第一个目标物”
			  }
			}
			break;
		}
		
		/* 获取第一个目标物 */
		case GET_FIRST_TARGET:  
    {
			if(Arm_Status == NO_TASK)  //若机械臂已完成任务
			{
				if(Turn_Flag)  //若需要转向
				{
				  Turn(Left);  //左转
					if(Task_Delay_Is_Delay() == 0)  //若延时结束
				  {
			      /* 若右侧传感器检测到黑线 */
			      if(HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_RESET)
			      {
				      Motion_Stop();  //停车
				      Turn_Flag = 0;  //清除转向标志
			      }
          }
				}
        else
				{
					if(HAL_GPIO_ReadPin(Left_Sensor_GPIO_Port, Left_Sensor_Pin) == GPIO_PIN_SET)  //若左侧传感器没有检测到黑线
					{
					  Basic_Track(Track_Recognition());  //执行基础循迹逻辑
					}
					else
					{
						Motion_Stop();  //停车
				    Turn_Flag = 1;  //放置转向标志位
			      Task_Delay_Start(4);  //延时200ms（四个定时器周期）
				    Task = PUT_FIRST_TARGET_FIRST_CROSS;  //切换任务状态至“放置第一个目标物的第一路口”
					}
				}
      }
			break;
    }
		
		/* 放置第一个目标物的第一个路口 */
		case PUT_FIRST_TARGET_FIRST_CROSS:
		{
		  if(Turn_Flag)  //若需要转向
		  {
			  Turn(Left);  //左转
				if(Task_Delay_Is_Delay() == 0)  //若延时结束
				{
			    /* 若左侧传感器检测到黑线 */
			    if(HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_RESET)
			    {
				    Motion_Stop();  //停车
				    Turn_Flag = 0;  //清除转向标志
			    }
        }
		  }
      else
			{
			  if(HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_SET)  //若右侧传感器没有检测到黑线
				{
					Basic_Track(Track_Recognition());  //执行基础循迹逻辑
			  }
				else
				{
				  Motion_Stop();  //停车
				  Turn_Flag = 1;  //放置转向标志位
			    Task_Delay_Start(2);  //延时100ms（两个定时器周期）
				  Task = PUT_FIRST_TARGET_SECOND_CROSS;  //切换任务状态至“放置第一个目标物的第二路口”
				}
			}
		}
		
		/* 放置第一个目标物的第二个路口 */
		case PUT_FIRST_TARGET_SECOND_CROSS:
		{
		  if(Turn_Flag)  //若需要转向
		  {
			  Turn(Left);  //左转
				if(Task_Delay_Is_Delay() == 0)  //若延时结束
				{
			    /* 若左侧传感器检测到黑线 */
			    if(HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_RESET)
			    {
				    Motion_Stop();  //停车
				    Turn_Flag = 0;  //清除转向标志
			    }
        }
		  }
      else
			{
			  if(HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_SET)  //若右侧传感器没有检测到黑线
				{
					Basic_Track(Track_Recognition());  //执行基础循迹逻辑
			  }
				else
				{
				  Motion_Stop();  //停车
				  Turn_Flag = 1;  //放置转向标志位
			    Task_Delay_Start(10);  //延时500ms（十个定时器周期）
				  Task = PUT_FIRST_TARGET_THIRD_CROSS;  //切换任务状态至“放置第一个目标物的第三路口”
				}
			}
		}
		
		/* 放置第一个目标物的第三个路口 */
		case PUT_FIRST_TARGET_THIRD_CROSS:
		{
			if(Turn_Flag)  //若需要转向
			{
				Turn(Right);  //右转
			}
			if(Task_Delay_Is_Delay() == 0)  //若延时结束
			{
			  /* 若侧面传感器至少有一个没有检测到黑线 */
			  if((HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_SET) || (HAL_GPIO_ReadPin(Left_Sensor_GPIO_Port, Left_Sensor_Pin) == GPIO_PIN_SET))
			  {
				  /* 区分情况 */
				  /* 若右侧传感器检测到黑线但左侧没有 */
				  if((HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_RESET) && (HAL_GPIO_ReadPin(Left_Sensor_GPIO_Port, Left_Sensor_Pin) == GPIO_PIN_SET))
          {
				    Turn(Right);  //向右调节
				  }
				  /* 若左侧传感器检测到黑线但右侧没有 */
				  else if((HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_SET) && (HAL_GPIO_ReadPin(Left_Sensor_GPIO_Port, Left_Sensor_Pin) == GPIO_PIN_RESET))
          {
				    Turn(Left);  //向左调节
				  }
			  }
			  else  //若两侧传感器均检测到黑线
			  {
				  Motion_Stop();  //停车
				  Arm_Status = IN_PUTTING_TASK;  //机械臂进入“放置“状态
				  Task = PUT_FIRST_TARGET;  //切换任务状态至“放置第一个目标物”
			  }
			}
			break;
		}
		
		/* 放置第一个标志物 */
		case PUT_FIRST_TARGET:
		{
      if(Arm_Status == NO_TASK)  //若机械臂已完成任务
			{
				if(Turn_Flag)  //若需要转向
				{
				  Turn(Left);  //左转
					if(Task_Delay_Is_Delay() == 0)  //若延时结束
				  {
			      /* 若右侧传感器检测到黑线 */
			      if(HAL_GPIO_ReadPin(Right_Sensor_GPIO_Port, Right_Sensor_Pin) == GPIO_PIN_RESET)
			      {
				      Motion_Stop();  //停车
				      Turn_Flag = 0;  //清除转向标志
			      }
          }
				}
        else
				{
					if(HAL_GPIO_ReadPin(Left_Sensor_GPIO_Port, Left_Sensor_Pin) == GPIO_PIN_SET)  //若左侧传感器没有检测到黑线
					{
					  Basic_Track(Track_Recognition());  //执行基础循迹逻辑
					}
					else
					{
						Motion_Stop();  //停车
				    Turn_Flag = 1;  //放置转向标志位
			      Task_Delay_Start(4);  //延时200ms（四个定时器周期）
				    Task = GET_SECOND_TARGET_FIRST_CROSS;  //切换任务状态至“获取第二个目标物的第一路口”
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
/************************************************ 文件结束 ************************************************/
