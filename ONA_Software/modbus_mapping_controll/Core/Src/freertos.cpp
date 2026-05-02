/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "modbus_rtu.h"
#include <stdio.h>
#include <string.h>
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  /* ── إنشاء الـ RTU master ── */
  static ModBus::RTU rtu(&huart2, DE_RE_GPIO_Port, DE_RE_Pin, 200u);
  rtu.setInterFrameDelay(2u);   /* 19200 baud → 2 ms */

  /* ── Helper lambda للطباعة على FTDI232 (USART3) ── */
  auto dbg = [](const char* msg) {
	HAL_UART_Transmit(&huart4,
					  (const uint8_t*)msg,
					  (uint16_t)strlen(msg),
					  100u);
  };

  char line[160];

  dbg("\r\n=== STM32F405 Modbus Master ===\r\n");
  dbg("Slave: ESP32  ID=1  Baud=19200\r\n");
  dbg("================================\r\n\r\n");

  for (;;)
  {
	/* ── FC04: اقرأ Input Registers (sensor data) ── */
	ModBus::Result r4 = rtu.readInput(1u, 0x0000u, 6u);

	if (r4.error == ModBus::Error::OK)
	{
	  float temp  = r4.registers[0] / 10.0f;
	  float press = r4.registers[1] / 10.0f;
	  float flow  = r4.registers[2] / 100.0f;
	  uint16_t st = r4.registers[3];
	  uint32_t up = ((uint32_t)r4.registers[4] << 16u) | r4.registers[5];

	  snprintf(line, sizeof(line),
		"[FC04] Temp=%5.1fC  Press=%7.1fhPa  Flow=%5.2fL/m"
		"  Status=0x%04X  Up=%lus\r\n",
		(double)temp, (double)press, (double)flow, st,
		(unsigned long)up);
	  dbg(line);

	  if (st & 0x0001u) dbg("  ! TEMP ALARM\r\n");
	  if (st & 0x0002u) dbg("  ! PRESS ALARM\r\n");
	  if (st & 0x0004u) dbg("  ! FLOW ALARM\r\n");
	}
	else
	{
	  snprintf(line, sizeof(line),
		"[FC04] ERR=%u\r\n", (unsigned)r4.error);
	  dbg(line);
	}

	/* ── FC03: اقرأ Holding Registers (setpoints) ── */
	ModBus::Result r3 = rtu.readHolding(1u, 0x0000u, 5u);

	if (r3.error == ModBus::Error::OK)
	{
	  const char* mode =
		r3.registers[4] == 0u ? "SINE"  :
		r3.registers[4] == 1u ? "FIXED" :
		r3.registers[4] == 2u ? "RAMP"  : "NOISE";

	  snprintf(line, sizeof(line),
		"[FC03] TempSP=%.1fC  PressAlm=%.1fhPa"
		"  FlowAlm=%.2fL/m  Mode=%s\r\n",
		(double)(r3.registers[1] / 10.0f),
		(double)(r3.registers[2] / 10.0f),
		(double)(r3.registers[3] / 100.0f),
		mode);
	  dbg(line);
	}
	else
	{
	  snprintf(line, sizeof(line),
		"[FC03] ERR=%u\r\n", (unsigned)r3.error);
	  dbg(line);
	}

	dbg("------------------------------------------\r\n");
	osDelay(1000u);
  }
  /* USER CODE END StartDefaultTask */
}
/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

