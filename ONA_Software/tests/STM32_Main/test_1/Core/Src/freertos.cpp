/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.cpp
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
#include "modbus_crc.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SLAVE_ID           5
#define ESP_RX_BUFFER_SIZE 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern UART_HandleTypeDef huart2;
extern ModbusMaster modbus;
extern uint8_t  esp_rx_buffer[];
extern float    g_tds, g_temp, g_flow, g_pressure, g_diff;
extern volatile bool  g_pump_status;
extern volatile float g_pump_speed;
extern volatile float g_target_pres;

extern void flush_modbus_uart(void);
extern void parse_esp_command(const char* line);
/* USER CODE END Variables */
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TelemetryTask */
osThreadId_t TelemetryTaskHandle;
const osThreadAttr_t TelemetryTask_attributes = {
  .name = "TelemetryTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for xQueueControl */
osMessageQueueId_t xQueueControlHandle;
const osMessageQueueAttr_t xQueueControl_attributes = {
  .name = "xQueueControl"
};
/* Definitions for xMutexTelemetry */
osMutexId_t xMutexTelemetryHandle;
const osMutexAttr_t xMutexTelemetry_attributes = {
  .name = "xMutexTelemetry"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartControlTask(void *argument);
void StartTelemetryTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of xMutexTelemetry */
  xMutexTelemetryHandle = osMutexNew(&xMutexTelemetry_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of xQueueControl */
  xQueueControlHandle = osMessageQueueNew (1, sizeof(uint32_t), &xQueueControl_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* creation of TelemetryTask */
  TelemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL, &TelemetryTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartControlTask */
/**
  * @brief  Function implementing the ControlTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
  uint32_t msg;
  for(;;)
  {
    if (osMessageQueueGet(xQueueControlHandle, &msg, NULL, osWaitForever) == osOK)
    {
      char cmd_line[ESP_RX_BUFFER_SIZE];
      memcpy(cmd_line, (const char*)esp_rx_buffer, ESP_RX_BUFFER_SIZE);

      parse_esp_command(cmd_line);

      bool     current_status    = g_pump_status;
      uint16_t current_speed_reg = (uint16_t)(g_pump_speed * 10.0f);

      flush_modbus_uart();
      modbus.writeSingleCoil(SLAVE_ID, 0, current_status);
      vTaskDelay(pdMS_TO_TICKS(5));
      flush_modbus_uart();
      modbus.writeSingleRegister(SLAVE_ID, 0, current_speed_reg);
      flush_modbus_uart();
    }
  }
  /* USER CODE END StartControlTask */
}

void StartTelemetryTask(void *argument)
{
  /* USER CODE BEGIN StartTelemetryTask */
	  vTaskDelay(pdMS_TO_TICKS(2000));

	  TickType_t xLastWakeTime = xTaskGetTickCount();
	  const TickType_t xPeriod = pdMS_TO_TICKS(1000);

  for(;;)
  {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);

    float tds, temp, flow, pressure, diff;

    flush_modbus_uart();
    HAL_Delay(10);
    if (modbus.readInputRegisters(SLAVE_ID, 0, 6))
    {
      uint8_t* rxData = modbus.getRxData();

      (void)(((uint16_t)rxData[3] << 8) | rxData[4]);
      uint16_t raw_pressure = ((uint16_t)rxData[5]  << 8) | rxData[6];
      uint16_t raw_flow     = ((uint16_t)rxData[7]  << 8) | rxData[8];
      uint16_t raw_tds      = ((uint16_t)rxData[9]  << 8) | rxData[10];
      uint16_t raw_temp     = ((uint16_t)rxData[11] << 8) | rxData[12];
      uint16_t raw_diff     = ((uint16_t)rxData[13] << 8) | rxData[14];

      tds      = (float)raw_tds      / 10.0f;
      temp     = (float)raw_temp     / 100.0f;
      flow     = (float)raw_flow     / 100.0f;
      pressure = (float)raw_pressure / 100.0f;
      diff     = (float)raw_diff     / 100.0f;

      if (osMutexAcquire(xMutexTelemetryHandle, 10) == osOK)
      {
        g_tds      = tds;
        g_temp     = temp;
        g_flow     = flow;
        g_pressure = pressure;
        g_diff     = diff;
        osMutexRelease(xMutexTelemetryHandle);
      }
    }
    else
    {
      flush_modbus_uart();
      if (osMutexAcquire(xMutexTelemetryHandle, 10) == osOK)
      {
        tds      = g_tds;
        temp     = g_temp;
        flow     = g_flow;
        pressure = g_pressure;
        diff     = g_diff;
        osMutexRelease(xMutexTelemetryHandle);
      }
    }

    char tx_frame[128];
    int tx_len = snprintf(tx_frame, sizeof(tx_frame),
        "TDS:%.2f,TEMP:%.2f,FLOW:%.2f,PRES:%.2f,DIFF:%.2f\n",
        tds, temp, flow, pressure, diff);

    char* tx_ptr    = tx_frame;
    int   tx_actual = tx_len;
    if (tx_len > 1 && tx_ptr[0] == '\0') {
        tx_ptr++;
        tx_actual--;
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)tx_ptr, (uint16_t)tx_actual, 100);
  }
  /* USER CODE END StartTelemetryTask */
}
/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

