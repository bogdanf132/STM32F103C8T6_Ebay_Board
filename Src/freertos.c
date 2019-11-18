/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId OS_Idle_TaskHandle;
osThreadId OS_COM_TaskHandle;
osThreadId OS_IO_TaskHandle;
osThreadId OS_NVM_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Idle_Task(void const * argument);
void COM_Task(void const * argument);
void IO_Task(void const * argument);
void NVM_Task(void const * argument);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of OS_Idle_Task */
  osThreadDef(OS_Idle_Task, Idle_Task, osPriorityIdle, 0, 128);
  OS_Idle_TaskHandle = osThreadCreate(osThread(OS_Idle_Task), NULL);

  /* definition and creation of OS_COM_Task */
  osThreadDef(OS_COM_Task, COM_Task, osPriorityNormal, 0, 512);
  OS_COM_TaskHandle = osThreadCreate(osThread(OS_COM_Task), NULL);

  /* definition and creation of OS_IO_Task */
  osThreadDef(OS_IO_Task, IO_Task, osPriorityNormal, 0, 128);
  OS_IO_TaskHandle = osThreadCreate(osThread(OS_IO_Task), NULL);

  /* definition and creation of OS_NVM_Task */
  osThreadDef(OS_NVM_Task, NVM_Task, osPriorityIdle, 0, 128);
  OS_NVM_TaskHandle = osThreadCreate(osThread(OS_NVM_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Idle_Task */
/**
* @brief Function implementing the OS_Idle_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Idle_Task */
void Idle_Task(void const * argument)
{
  /* USER CODE BEGIN Idle_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Idle_Task */
}

/* USER CODE BEGIN Header_COM_Task */
/**
* @brief Function implementing the OS_COM_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_COM_Task */
void COM_Task(void const * argument)
{
  /* USER CODE BEGIN COM_Task */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	  HAL_Delay(10);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	  HAL_Delay(1000);
	  //HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_0);
	  //HAL_Delay(500);
    osDelay(1);
  }
  /* USER CODE END COM_Task */
}

/* USER CODE BEGIN Header_IO_Task */
/**
* @brief Function implementing the OS_IO_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IO_Task */
void IO_Task(void const * argument)
{
  /* USER CODE BEGIN IO_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IO_Task */
}

/* USER CODE BEGIN Header_NVM_Task */
/**
* @brief Function implementing the OS_NVM_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NVM_Task */
void NVM_Task(void const * argument)
{
  /* USER CODE BEGIN NVM_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END NVM_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
