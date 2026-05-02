/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Modbus RTU Master — test task
  *
  * UART assignment:
  *   huart2  — USART2 (PA2/PA3) → MAX485 → Modbus bus
  *   huart4  — UART4            → FTDI232 → PC debug terminal (printf)
  *
  * Pin mapping:
  *   PA2  USART2 TX  → MAX485 DI
  *   PA3  USART2 RX  → MAX485 RO
  *   PA4  GPIO  DE/RE → MAX485 DE+RE
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "modbus_rtu.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define SLAVE_ID        1u
#define MODBUS_TIMEOUT  50u
/* USER CODE END PD */

/* USER CODE BEGIN Variables */
#define MB_DE_PORT   GPIOA
#define MB_DE_PIN    GPIO_PIN_4

/* USER CODE END Variables */

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name       = "modbusTask",
  .stack_size = 1024u * 4u,
  .priority   = (osPriority_t)osPriorityNormal,
};

void StartDefaultTask(void *argument);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN FunctionPrototypes */
static void log_result(const char* label, const ModBus::Result& r);
/* USER CODE END FunctionPrototypes */

/* ═══════════════════════════════════════════════════════════════════════════
 * printf → UART4 (FTDI) redirect
 *
 * _write is the newlib syscall called by printf/fwrite.
 * Redirecting it to huart4 sends all printf output to the FTDI terminal.
 * extern "C" is required so the C++ compiler emits a C-mangled symbol.
 * ═══════════════════════════════════════════════════════════════════════════*/
extern "C" int _write(int fd, char* ptr, int len)
{
    (void)fd;
    HAL_UART_Transmit(&huart1,
                      reinterpret_cast<uint8_t*>(ptr),
                      static_cast<uint16_t>(len),
                      100u);
    return len;
}
static void dbg(const char* msg)
{
    HAL_UART_Transmit(&huart1,
                      reinterpret_cast<const uint8_t*>(msg),
                      static_cast<uint16_t>(strlen(msg)),
                      100u);
}


/* ── FreeRTOS init ──────────────────────────────────────────────────────── */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
                                  &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * StartDefaultTask — Modbus RTU master test loop
 *
 * Sequence every ~2 s:
 *   1. FC03 — read holding regs [0..2]
 *   2. FC04 — read input  regs [0..1]
 *   3. FC06 — write holding reg [1]   (value cycles each iteration)
 *   4. FC03 — read back  reg  [1]     (verify write)
 *   5. FC16 — write holding regs [3..5]
 *   6. FC03 — read back  regs [3..5]  (verify multi-write)
 * ═══════════════════════════════════════════════════════════════════════════*/
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  static ModBus::RTU rtu(&huart2, DE_RE_GPIO_Port, DE_RE_Pin, 200u);
  rtu.setInterFrameDelay(2u);

  auto dbg = [](const char* msg) {
    HAL_UART_Transmit(&huart1,
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
    /* ════════════════════════════════════════════
     * FC04 — Read Input Registers
     * addr=0x0000  qty=6
     * [0] Temp      °C  × 10
     * [1] Pressure  hPa × 10
     * [2] Flow      L/m × 100
     * [3] Status    bitfield
     * [4] Uptime HI
     * [5] Uptime LO
     * ════════════════════════════════════════════*/
    ModBus::Result r4 = rtu.readInput(1u, 0x0000u, 6u);

    if (r4.error == ModBus::Error::OK)
    {
      uint16_t raw_temp  = r4.registers[0];
      uint16_t raw_press = r4.registers[1];
      uint16_t raw_flow  = r4.registers[2];
      uint16_t st        = r4.registers[3];
      uint32_t up        = ((uint32_t)r4.registers[4] << 16u)
                         |  (uint32_t)r4.registers[5];

      int temp_int  = raw_temp  / 10;
      int temp_dec  = raw_temp  % 10;
      int press_int = raw_press / 10;
      int press_dec = raw_press % 10;
      int flow_int  = raw_flow  / 100;
      int flow_dec  = raw_flow  % 100;

      snprintf(line, sizeof(line),
        "[FC04] Temp=%d.%d C  Press=%d.%d hPa  Flow=%d.%02d L/m"
        "  Status=0x%04X  Up=%lus\r\n",
        temp_int, temp_dec,
        press_int, press_dec,
        flow_int, flow_dec,
        (unsigned)st,
        (unsigned long)up);
      dbg(line);

      if (st & 0x0001u) dbg("  ! TEMP ALARM\r\n");
      if (st & 0x0002u) dbg("  ! PRESS ALARM\r\n");
      if (st & 0x0004u) dbg("  ! FLOW ALARM\r\n");
    }
    else
    {
      snprintf(line, sizeof(line),
        "[FC04] ERROR: %u\r\n", (unsigned)r4.error);
      dbg(line);
    }

    /* ════════════════════════════════════════════
     * FC03 — Read Holding Registers
     * addr=0x0000  qty=5
     * [0] Slave ID
     * [1] Temp setpoint  °C  × 10
     * [2] Press alarm    hPa × 10
     * [3] Flow alarm     L/m × 100
     * [4] Sim mode       0=SINE 1=FIXED 2=RAMP 3=NOISE
     * ════════════════════════════════════════════*/
    ModBus::Result r3 = rtu.readHolding(1u, 0x0000u, 5u);

    if (r3.error == ModBus::Error::OK)
    {
      int tsp_int  = r3.registers[1] / 10;
      int tsp_dec  = r3.registers[1] % 10;
      int palm_int = r3.registers[2] / 10;
      int palm_dec = r3.registers[2] % 10;
      int falm_int = r3.registers[3] / 100;
      int falm_dec = r3.registers[3] % 100;

      const char* mode =
        r3.registers[4] == 0u ? "SINE"  :
        r3.registers[4] == 1u ? "FIXED" :
        r3.registers[4] == 2u ? "RAMP"  : "NOISE";

      snprintf(line, sizeof(line),
        "[FC03] ID=%u  TempSP=%d.%d C  PressAlm=%d.%d hPa"
        "  FlowAlm=%d.%02d L/m  Mode=%s\r\n",
        (unsigned)r3.registers[0],
        tsp_int, tsp_dec,
        palm_int, palm_dec,
        falm_int, falm_dec,
        mode);
      dbg(line);
    }
    else
    {
      snprintf(line, sizeof(line),
        "[FC03] ERROR: %u\r\n", (unsigned)r3.error);
      dbg(line);
    }

    dbg("------------------------------------------\r\n");
    osDelay(1000u);
  }

  /* USER CODE END StartDefaultTask */
}
/* ═══════════════════════════════════════════════════════════════════════════
 * log_result
 * ═══════════════════════════════════════════════════════════════════════════*/
/* USER CODE BEGIN Application */
static void log_result(const char* label, const ModBus::Result& r)
{
    if (r.error == ModBus::Error::OK)
    {
        printf("[OK]  %-20s  cnt=%u", label, r.count);
        for (uint8_t i = 0u; i < r.count; ++i)
            printf("  [%u]=0x%04X", i, r.registers[i]);
        printf("\r\n");
    }
    else
    {
        const char* err_str = "UNKNOWN";
        switch (r.error)
        {
            case ModBus::Error::TIMEOUT:        err_str = "TIMEOUT";        break;
            case ModBus::Error::CRC_MISMATCH:   err_str = "CRC_MISMATCH";   break;
            case ModBus::Error::WRONG_SLAVE:    err_str = "WRONG_SLAVE";    break;
            case ModBus::Error::EXCEPTION:      err_str = "EXCEPTION";      break;
            case ModBus::Error::TX_FAIL:        err_str = "TX_FAIL";        break;
            case ModBus::Error::RX_OVERFLOW:    err_str = "RX_OVERFLOW";    break;
            case ModBus::Error::INVALID_ARG:    err_str = "INVALID_ARG";    break;
            case ModBus::Error::PROTOCOL_ERROR: err_str = "PROTOCOL_ERROR"; break;
            default: break;
        }
        if (r.error == ModBus::Error::EXCEPTION)
            printf("[ERR] %-20s  %s  ex=0x%02X\r\n",
                   label, err_str, r.exception_code);
        else
            printf("[ERR] %-20s  %s\r\n", label, err_str);
    }
}
/* USER CODE END Application */
