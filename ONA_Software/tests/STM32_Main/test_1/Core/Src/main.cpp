/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "modbus_crc.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
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

/* USER CODE BEGIN PV */
extern osMessageQueueId_t xQueueControlHandle;
ModbusMaster modbus(&huart1, DE_GPIO_Port, DE_Pin);

uint8_t  esp_rx_byte;
uint8_t  esp_rx_buffer[ESP_RX_BUFFER_SIZE];
uint8_t  esp_rx_index = 0;

/* Shared telemetry — protected by xMutexTelemetry */
float g_tds      = 0.0f;
float g_temp     = 0.0f;
float g_flow     = 0.0f;
float g_pressure = 0.0f;
float g_diff     = 0.0f;

/* Shared control state — written from ControlTask only */
volatile bool  g_pump_status = false;
volatile float g_pump_speed  = 0.0f;
volatile float g_target_pres = 3.5f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void start_esp_bridge_rx(void);
void flush_modbus_uart(void);
void parse_esp_command(const char* line);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void start_esp_bridge_rx(void) {
    __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_PE);
    HAL_UART_Receive_IT(&huart2, &esp_rx_byte, 1);
}

void flush_modbus_uart(void) {
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_PE);
    __HAL_UART_FLUSH_DRREGISTER(&huart1);
    volatile uint32_t tmpreg = huart1.Instance->SR;
    tmpreg = huart1.Instance->DR;
    (void)tmpreg;
}

void parse_esp_command(const char* line) {
    if (strstr(line, "CMD:PUMP_ON") != NULL) {
        g_pump_status = true;
    } else if (strstr(line, "CMD:PUMP_OFF") != NULL) {
        g_pump_status = false;
        g_pump_speed  = 0.0f;
        return;
    } else {
        return;
    }
    const char* speed_ptr = strstr(line, "SPEED:");
    if (speed_ptr != NULL) g_pump_speed = strtof(speed_ptr + 6, NULL);
    const char* pres_ptr = strstr(line, "PRES_SP:");
    if (pres_ptr != NULL) g_target_pres = strtof(pres_ptr + 8, NULL);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (esp_rx_index == 0 && esp_rx_byte != 'C') {
            HAL_UART_Receive_IT(&huart2, &esp_rx_byte, 1);
            return;
        }
        if (esp_rx_byte == '\n' || esp_rx_byte == '\r') {
            if (esp_rx_index > 0) {
                esp_rx_buffer[esp_rx_index] = '\0';
                /* بدل new_control_flag — نبعت على Queue من ISR */
                uint32_t msg = 1;
                osMessageQueuePut(xQueueControlHandle, &msg, 0, 0);
            }
            esp_rx_index = 0;
        } else {
            if (esp_rx_index < (ESP_RX_BUFFER_SIZE - 1))
                esp_rx_buffer[esp_rx_index++] = esp_rx_byte;
            else
                esp_rx_index = 0;
        }
        __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_ORE | UART_FLAG_NE);
        HAL_UART_Receive_IT(&huart2, &esp_rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        volatile uint32_t tmpreg = huart->Instance->SR;
        tmpreg = huart->Instance->DR;
        (void)tmpreg;
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_PE);
        esp_rx_index = 0;
        HAL_UART_Receive_IT(huart, &esp_rx_byte, 1);
    }
    if (huart->Instance == USART1)
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_PE);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  flush_modbus_uart();
  HAL_Delay(500);
  start_esp_bridge_rx();
  __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_TC);
  HAL_Delay(100);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
