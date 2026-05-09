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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static float g_tds           = 290.0f;
static float g_temperature   = 22.5f;
static float g_flow          = 41.0f;
static float g_pressure      = 3.8f;
static float g_diff_pressure = 0.42f;

static uint8_t  g_pump_on     = 0;
static float    g_speed_hz    = 0.0f;
static float    g_target_pres = 3.5f;

#define RX_BUF_SIZE 128
static uint8_t  g_rx_byte;
static char     g_rx_line[RX_BUF_SIZE];
static uint16_t g_rx_line_pos = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static float frand_range(float lo, float hi);
static float clampf(float v, float lo, float hi);
static void  update_sensors(void);
static void  send_frame(void);
static void  process_rx_line(const char *line);
static void  start_rx(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t g_seed = 12345;
static uint32_t lcg_next(void) {
    g_seed = g_seed * 1664525u + 1013904223u;
    return g_seed;
}

static float frand_range(float lo, float hi) {
    float t = (float)(lcg_next() & 0xFFFF) / 65535.0f;
    return lo + t * (hi - lo);
}

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static void update_sensors(void) {
    g_tds           = clampf(g_tds           + frand_range(-5.0f,   5.0f),  100.0f, 900.0f);
    g_temperature   = clampf(g_temperature   + frand_range(-0.3f,   0.3f),    5.0f,  45.0f);
    g_flow          = clampf(g_flow          + frand_range(-2.0f,   2.0f),    5.0f, 120.0f);
    g_pressure      = clampf(g_pressure      + frand_range(-0.15f,  0.15f),   0.5f,   8.0f);
    g_diff_pressure = clampf(g_diff_pressure + frand_range(-0.03f,  0.03f),  0.05f,   1.5f);
    g_seed ^= HAL_GetTick();
}

static void send_frame(void) {
    char buf[128];
    int len = snprintf(buf, sizeof(buf),
        "TDS:%.2f,TEMP:%.2f,FLOW:%.2f,PRES:%.2f,DIFF:%.2f\r\n",
        g_tds, g_temperature, g_flow, g_pressure, g_diff_pressure);

    HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100);
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 100);
}

static void process_rx_line(const char *line) {
    char dbg[160];
    int dlen = snprintf(dbg, sizeof(dbg), "[RX] %s\r\n", line);
    HAL_UART_Transmit(&huart1, (uint8_t *)dbg, (uint16_t)dlen, 50);

    if (strncmp(line, "CMD:PUMP_ON", 11) == 0) {
        g_pump_on = 1;

        const char *sp = strstr(line, "SPEED:");
        if (sp) g_speed_hz = strtof(sp + 6, NULL);

        const char *pp = strstr(line, "PRES_SP:");
        if (pp) g_target_pres = strtof(pp + 8, NULL);

        char info[80];
        int ilen = snprintf(info, sizeof(info),
            "[CTRL] PUMP ON  speed=%.1f Hz  pres_sp=%.2f bar\r\n",
            g_speed_hz, g_target_pres);
        HAL_UART_Transmit(&huart1, (uint8_t *)info, (uint16_t)ilen, 50);

    } else if (strncmp(line, "CMD:PUMP_OFF", 12) == 0) {
        g_pump_on  = 0;
        g_speed_hz = 0.0f;

        const char *msg = "[CTRL] PUMP OFF\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 50);
    }
}

static void start_rx(void) {
    HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance != USART2) return;

    uint8_t byte = g_rx_byte;

    if (byte == '\n') {
        if (g_rx_line_pos > 0) {
            if (g_rx_line[g_rx_line_pos - 1] == '\r') {
                g_rx_line_pos--;
            }
            g_rx_line[g_rx_line_pos] = '\0';
            process_rx_line(g_rx_line);
            g_rx_line_pos = 0;
        }
    } else if (byte != '\r') {
        if (g_rx_line_pos < RX_BUF_SIZE - 1) {
            g_rx_line[g_rx_line_pos++] = (char)byte;
        } else {
            g_rx_line_pos = 0;
        }
    }

    HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1);
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
  char clk_buf[64];
  snprintf(clk_buf, sizeof(clk_buf),
      "\r\nSYSCLK=%lu PCLK1=%lu\r\n",
      HAL_RCC_GetSysClockFreq(),
      HAL_RCC_GetPCLK1Freq());
  HAL_UART_Transmit(&huart1, (uint8_t*)clk_buf, strlen(clk_buf), 100);

  const char *banner = "=== Takamul STM32 Fake Sensor Node ===\r\n"
                       "USART2 -> ESP32 @ 115200\r\n"
                       "Sending frame every 3 seconds\r\n\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *)banner, strlen(banner), 200);

  start_rx();

  uint32_t last_send = 0;
  uint32_t last_ping = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    uint32_t now = HAL_GetTick();

	    if ((now - last_send) >= 3000) {
	        last_send = now;
	        update_sensors();
	        send_frame();
	    }

	    if ((now - last_ping) >= 1000) {
	        last_ping = now;
	        const char *ping = "PING\r\n";
	        HAL_UART_Transmit(&huart1, (uint8_t*)ping, 6, 50);
	    }
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
