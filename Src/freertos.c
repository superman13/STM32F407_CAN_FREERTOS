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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "main.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId MainHandle;
osThreadId ComHandle;
osThreadId DiagHandle;
osThreadId HOSTHandle;
osThreadId SensorMngHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartMain(void const * argument);
void StartCom(void const * argument);
void StartDiag(void const * argument);
void StartHOST(void const * argument);
void StartSensorMng(void const * argument);

extern void MX_USB_HOST_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

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
  /* definition and creation of Main */
  osThreadDef(Main, StartMain, osPriorityNormal, 0, 1024);
  MainHandle = osThreadCreate(osThread(Main), NULL);

  /* definition and creation of Com */
  osThreadDef(Com, StartCom, osPriorityNormal, 0, 1024);
  ComHandle = osThreadCreate(osThread(Com), NULL);

  /* definition and creation of Diag */
  osThreadDef(Diag, StartDiag, osPriorityNormal, 0, 1024);
  DiagHandle = osThreadCreate(osThread(Diag), NULL);

  /* definition and creation of HOST */
  osThreadDef(HOST, StartHOST, osPriorityNormal, 0, 1024);
  HOSTHandle = osThreadCreate(osThread(HOST), NULL);

  /* definition and creation of SensorMng */
  osThreadDef(SensorMng, StartSensorMng, osPriorityNormal, 0, 1024);
  SensorMngHandle = osThreadCreate(osThread(SensorMng), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartMain function */
void StartMain(void const * argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();

  /* USER CODE BEGIN StartMain */
	HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
    osDelay(100);
	HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
	osDelay(100);
  }
  /* USER CODE END StartMain */
}

/* StartCom function */
void StartCom(void const * argument)
{
  /* USER CODE BEGIN StartCom */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCom */
}

/* StartDiag function */
void StartDiag(void const * argument)
{
  /* USER CODE BEGIN StartDiag */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDiag */
}

/* StartHOST function */
void StartHOST(void const * argument)
{
  /* USER CODE BEGIN StartHOST */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartHOST */
}

/* StartSensorMng function */
void StartSensorMng(void const * argument)
{
  /* USER CODE BEGIN StartSensorMng */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSensorMng */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
