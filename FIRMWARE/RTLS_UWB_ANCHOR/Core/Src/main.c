/* USER CODE BEGIN Header */
/**ANCHOR ANCHOR ANCHOR ANCHOR ANCHOR ANCHOR
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "decawave/dw_api.h"
#include "log/log.h"
#include "mac/mac.h"
#include "led/led.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Payload of Anchor send to Tag
typedef struct
{
  uint8_t pollRx[5];
  uint8_t answerTx[5];
  uint8_t finalRx[5];
} __attribute__((packed)) reportPayload_t;

// timestamp
typedef union timestamp_u
{
  uint8_t raw[5];
  uint64_t full;
  struct
  {
    uint32_t low32;
    uint8_t high8;
  } __attribute__((packed));
  struct
  {
    uint8_t low8;
    uint32_t high32;
  } __attribute__((packed));
} timestamp_t;

// typedef struct
// {
//   uint8_t typemess;
//   uint8_t seq;
//   // uint8_t anchorAddr[2];
// } txHandle_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// The four packets for ranging
#define POLL 0x01 // Poll is initiated by the tag
#define ANSWER 0x02
#define FINAL 0x03
#define REPORT 0x04 // Report contains all measurement from the anchor

#define TYPE 0 // POLL, ANWSER, FINAL, REPORT
#define SEQ 1  // sequence

#define RX_TIMEOUT 1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Timestamps for ranging
static dwTimestamp_t poll_rx;
static dwTimestamp_t answer_tx;
static dwTimestamp_t final_rx;

#define ANTENNA_DELAY 16475

packet_t rxPacket = {0};
packet_t txPacket = {0};
// txHandle_t txHandle = {0};

uint8_t curr_seq = 1;
int curr_anchor = 0;

dwDeviceTypes_t device = {
    .extendedFrameLength = FRAME_LENGTH_NORMAL,
    .pacSize = PAC_SIZE_8,
    .pulseFrequency = TX_PULSE_FREQ_64MHZ,
    .dataRate = TRX_RATE_6800KBPS,
    .preambleLength = TX_PREAMBLE_LEN_128,
    .preambleCode = PREAMBLE_CODE_64MHZ_9,
    .channel = CHANNEL_5,
    .smartPower = true,
    .frameCheck = true,
    .permanentReceive = false,
    .deviceMode = IDLE_MODE,
    .forceTxPower = false,
};

uint8_t tagBaseAddr[2] = {0x20, 0x01};
uint8_t anchorAddress[8] = {0x89, 0x01};

volatile bool sentAck = false;
volatile bool recievedAck = false;
volatile bool initAck = true;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void dwInteruptHandler(void)
{
  dwReadSystemEventStatusRegister(&device);
  if (dwIsTransmitDone(&device))
  {
    dwClearTransmitStatus(&device);
    sentAck = true;
  }
  if (dwIsReceiveDone(&device))
  {
    dwClearReceiveStatus(&device);
    recievedAck = true;
  }
  if (dwIsReceiveTimestampAvailable(&device))
  {
    dwClearReceiveTimestampAvailableStatus(&device);
  }
  if (dwIsReceiveFailed(&device))
  {
    dwClearReceiveStatus(&device);
    dwRxSoftReset(&device);
  }
  if (dwIsReceiveTimeout(&device))
  {
    dwClearReceiveStatus(&device);
    dwRxSoftReset(&device);
  }
}

void clearAllFlag(void)
{
  dwClearTransmitStatus(&device);
  dwClearReceiveStatus(&device);
}

void log_data(char *string)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)string, strlen(string), 1000);
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  log_data("[ANCHOR START]\r\n");
  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
  MAC80215_PACKET_INIT(rxPacket, MAC802154_TYPE_DATA);
  // init DW1000
  dwInit(&device);
  if (dwConfigure(&device) == DW_ERROR_OK)
  {
    dwEnableAllLeds(&device);
  }
  else
  {
    log_data("[Configure failed]");
    while (1)
      ;
  }
  dwNewConfiguration(&device);
  dwSetDefaults(&device);
  dwCommitConfiguration(&device);
  clearAllFlag();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    dwInteruptHandler();
    // init receive range
    if (initAck)
    {
      initAck = false;
      memset(&txPacket, 0, sizeof(txPacket));
      memset(&rxPacket, 0, sizeof(rxPacket));
      dwNewReceive(&device);
      dwSetDefaults(&device);
      dwStartReceive(&device);
    }

    if (recievedAck)
    {
      recievedAck = false;
      log_data("RxCallback\r\n");
      dwTimestamp_t arival;
      // check data length
      int datalength = dwGetDataLength(&device);
      if (datalength == 0)
      {
        return 0;
      }
      memset(&rxPacket, 0, sizeof(rxPacket));

      // get data from tag and out in rxPacket
      dwGetData(&device, (uint8_t *)&rxPacket, datalength);
      // check address of anchor due to tag send
      // correct return 0
      if (memcmp(rxPacket.destAddress, anchorAddress, 2))
      {
        // wrong address and repeat receive
        dwNewReceive(&device);
        dwSetDefaults(&device);
        dwStartReceive(&device);
        return 0;
      }
      else
      {
        // oke address
        switch (rxPacket.payload[TYPE])
        {
        case POLL:
          log_data("POLL\r\n");
          if (rxPacket.payload[SEQ] != 1) // 1
          {
            log_data("wrong sequence number\r\n");
            return 0;
          }
          dwGetReceiveTimestamp(&device, &arival);
          arival.timeFull -= ANTENNA_DELAY;
          poll_rx = arival;

          txPacket.payload[TYPE] = ANSWER;
          txPacket.payload[SEQ] = 2; // rxPacket.payload[SEQ] + 1; // 2
          memcpy(txPacket.destAddress, tagBaseAddr, 2);
          memcpy(txPacket.sourceAddress, anchorAddress, 2);
          dwNewTransmit(&device);
          dwSetDefaults(&device);
          dwSetData(&device, (uint8_t *)&txPacket, MAC802154_HEADER_LENGTH + 2);
          dwStartTransmit(&device);
          break;

        case FINAL:
          log_data("FINAL\r\n");
          if (rxPacket.payload[SEQ] != 3) // 3
          {
            log_data("wrong sequence number\r\n");
            return 0;
          }

          dwGetReceiveTimestamp(&device, &arival);
          arival.timeFull -= ANTENNA_DELAY;
          final_rx = arival;

          memset(&txPacket, 0, sizeof(txPacket));

          reportPayload_t *reportmess = (reportPayload_t *)(txPacket.payload + 2);

          txPacket.payload[TYPE] = REPORT;
          txPacket.payload[SEQ] = 4; // rxPacket.payload[SEQ] + 1; // 4
          memcpy(txPacket.destAddress, tagBaseAddr, 2);
          memcpy(txPacket.sourceAddress, anchorAddress, 2);
          memcpy(&reportmess->pollRx, &poll_rx, 5);
          memcpy(&reportmess->answerTx, &answer_tx, 5);
          memcpy(&reportmess->finalRx, &final_rx, 5);
          dwNewTransmit(&device);
          dwSetDefaults(&device);
          dwSetData(&device, (uint8_t *)&txPacket, MAC802154_HEADER_LENGTH + 2 + sizeof(reportPayload_t));
          dwStartTransmit(&device);
          break;
        }
      }
    }

    if (sentAck)
    {
      sentAck = false;
      dwTimestamp_t departure;
      dwGetTransmitTimestamp(&device, &departure);
      departure.timeFull += ANTENNA_DELAY;
      log_data("TxCallback\r\n");
      switch (txPacket.payload[TYPE])
      {
      case ANSWER:
        log_data("ANSWER\r\n");
        answer_tx = departure;
        dwNewReceive(&device);
        dwSetDefaults(&device);
        dwStartReceive(&device);
        break;
      case REPORT:
        log_data("REPORT\r\n");
        //curr_seq = 1;
        initAck = true;
        break;
      }
      //       dwSetReceiveWaitTimeout(&device, RX_TIMEOUT);
      // dwWriteSystemConfigurationRegister(&device);
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
