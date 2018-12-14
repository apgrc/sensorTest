/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "usbd_cdc_if.h"
#include "stm32f1xx_hal.h"
#include "dataHelp.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern volatile SerialInput command;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern char stringOut[40];
extern volatile int32_t encoder;
extern volatile int32_t sample;
extern volatile int32_t sampleOld;

extern volatile int16_t pwmOutput;
extern volatile SerialInput command;

extern uint32_t startTime;
extern uint32_t sampleTime;
extern Data dataOut;

/* USER CODE END Variables */
osThreadId motorResultsHandle;
osThreadId pwmControlHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartLogTask(void const * argument);
void StartPWMTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of motorResults */
  osThreadDef(motorResults, StartLogTask, osPriorityNormal, 0, 64);
  motorResultsHandle = osThreadCreate(osThread(motorResults), NULL);

  /* definition and creation of pwmControl */
  osThreadDef(pwmControl, StartPWMTask, osPriorityIdle, 0, 64);
  pwmControlHandle = osThreadCreate(osThread(pwmControl), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartLogTask */
/**
  * @brief  Function implementing the motorResults thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartLogTask */
void StartLogTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN StartLogTask */
	uint32_t notifyValue;
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS( 100 );
	xLastWakeTime = xTaskGetTickCount();
	/* Infinite loop */
	for(;;)
	{
		xTaskNotifyWait(pdFALSE, Sensor_Start, &notifyValue, portMAX_DELAY);


		if ((notifyValue & Sensor_Start) == Sensor_Start) {
			notifyValue = 0;
			xLastWakeTime = xTaskGetTickCount();
			while (1) {
				vTaskDelayUntil( &xLastWakeTime, xPeriod );
				HAL_GPIO_WritePin(SENSOR_TRIGGER_GPIO_Port,SENSOR_TRIGGER_Pin,GPIO_PIN_SET);
				HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_4);
				xTaskNotifyWait(pdFALSE, Sensor_Stop, &notifyValue, 1);
				if ((notifyValue & Sensor_Stop) == Sensor_Stop)
					break;
			}
		}
	}
  /* USER CODE END StartLogTask */
}

/* USER CODE BEGIN Header_StartPWMTask */
/**
* @brief Function implementing the pwmControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPWMTask */
void StartPWMTask(void const * argument)
{
  /* USER CODE BEGIN StartPWMTask */
	/* Infinite loop */
	for (;;) {
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		SerialInput output = command;

		if (output.PWM_INPUT1 >= 0) {
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, MIN(output.PWM_INPUT1, maxPeriod));
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
		} else {
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,
					MIN(abs(output.PWM_INPUT1), maxPeriod));
		}

		if (output.PWM_INPUT2 >= 0) {
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MIN(output.PWM_INPUT2, maxPeriod));
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
		} else {
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,
					MIN(abs(output.PWM_INPUT2), maxPeriod));
		}
	}
  /* USER CODE END StartPWMTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
