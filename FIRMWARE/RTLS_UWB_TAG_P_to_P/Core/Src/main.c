/* USER CODE BEGIN Header */
/**TAG TAG TAG TAG TAG TAG TAG TAG TAG TAG TAG TAG TAG TAG
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
#include <stdlib.h>
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

typedef struct
{
  uint8_t typemess;
  uint8_t seq;
  // uint8_t anchorAddr[2];
} txHandle_t;

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
static dwTimestamp_t poll_tx;
static dwTimestamp_t poll_rx;
static dwTimestamp_t answer_tx;
static dwTimestamp_t answer_rx;
static dwTimestamp_t final_tx;
static dwTimestamp_t final_rx;

static const double C = 299702547;//299792458.0;        // Speed of light
static const double tsfreq = 499.2e6 * 128; // Timestamp counter frequency

// #define ANTENNA_OFFSET 154.6                                         // In meter
// #define ANTENNA_DELAY (ANTENNA_OFFSET * 499.2e6 * 128) / 299792458.0 // In radio tick
#define ANTENNA_DELAY 16450//16475

packet_t rxPacket = {0};
packet_t txPacket = {0};
txHandle_t txHandle = {0};

uint8_t curr_seq = 1;
int curr_anchor = 0;
long int seq_mess = 1;


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
uint8_t anchorAddress[2] = {0x89, 0x01};

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
  log_data("[TAG START]\r\n");
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
    log_data("[Configure failed]\r\n");
    while (1)
      ;
  }
  dwNewConfiguration(&device);
  dwSetDefaults(&device);
  dwCommitConfiguration(&device);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //dwInteruptHandler();
    // init ranging
    if (initAck)
    {
      initAck = false;
      char buf[10] = {0};
      sprintf(buf, "####SEQ: %ld\r\n", seq_mess);
      // log_data(buf);
      // memset(&txHandle, 0, sizeof(txHandle));
      // txHandle.typemess = POLL;
      // txHandle.seq = ++curr_seq; // poll message has seq = 1; answer is 2;..
      memset(&txPacket, 0, sizeof(txPacket));
      memset(&rxPacket, 0, sizeof(rxPacket));
      txPacket.payload[TYPE] = POLL;
      txPacket.payload[SEQ] = 1;
      memcpy(txPacket.destAddress, anchorAddress, 2);
      memcpy(txPacket.sourceAddress, tagBaseAddr, 2);
      // memcpy(txPacket.payload, &txHandle, sizeof(txHandle));
      //  transmision
      dwNewTransmit(&device);
      dwSetDefaults(&device);
      dwSetData(&device, (uint8_t *)&txPacket, MAC802154_HEADER_LENGTH + 2); // 2 is length of payload
      dwStartTransmit(&device);
      do
      {
        dwReadSystemEventStatusRegister(&device);
      } while (!(device.sysstatus[0] & (1 << TXFRS_BIT))); // if check TXRFS bit is 1, transmist OKE
      dwInteruptHandler();
    }
    if (sentAck)
    {
      // clear flag
      sentAck = false;
      dwTimestamp_t departure;
      dwGetTransmitTimestamp(&device, &departure);
      departure.timeFull += ANTENNA_DELAY;
      // log_data("TxCallBack\r\n");
      switch (txPacket.payload[TYPE])
      {
      case POLL:
        poll_tx = departure;
        //log_data("POLL\r\n");
        dwNewReceive(&device);
        dwSetDefaults(&device);
        dwStartReceive(&device);
        uint32_t time = HAL_GetTick();
        do
        {
          if (HAL_GetTick() - time > 20)
          {
            // log_data("TIMEOUT POLL AGAIN\r\n");
            memset(&txPacket, 0, sizeof(txPacket));
            txPacket.payload[TYPE] = POLL;
            txPacket.payload[SEQ] = 1;
            memcpy(txPacket.destAddress, anchorAddress, 2);
            memcpy(txPacket.sourceAddress, tagBaseAddr, 2);
            //  transmision
            dwNewTransmit(&device);
            dwSetDefaults(&device);
            dwSetData(&device, (uint8_t *)&txPacket, MAC802154_HEADER_LENGTH + 2); // 2 is length of payload
            dwStartTransmit(&device);
            do
            {
              dwReadSystemEventStatusRegister(&device);
            } while (!(device.sysstatus[0] & (1 << TXFRS_BIT)));
            dwClearTransmitStatus(&device);

            dwGetTransmitTimestamp(&device, &departure);
            departure.timeFull += ANTENNA_DELAY;
            poll_tx = departure;

            // log_data("TXOKE POLL AGAIN\r\n");
            //receive ANWSER
            dwNewReceive(&device);
            dwSetDefaults(&device);
            dwStartReceive(&device);

            time = HAL_GetTick();
            // break;
          }
          dwReadSystemEventStatusRegister(&device);
        } while (!((device.sysstatus[1] & (((1 << RXDFR_BIT) | (1 << RXFCG_BIT)) >> 8)) || (device.sysstatus[2] & ((1 << RXRFTO_BIT) >> 16))));
        dwInteruptHandler();
        break;
      case FINAL:
        final_tx = departure;
        //log_data("FINAL\r\n");
        dwNewReceive(&device);
        dwSetDefaults(&device);
        dwStartReceive(&device);
        uint32_t time1 = HAL_GetTick();
        do
        {
          if (HAL_GetTick() - time1 > 100)
          {
            // log_data("TIMEOUT FINAL\r\n");
            initAck = true;
            break;
          }
          dwReadSystemEventStatusRegister(&device);
        } while ((HAL_GetTick() - time1 < 100)||(!((device.sysstatus[1] & (((1 << RXDFR_BIT) | (1 << RXFCG_BIT)) >> 8)) || (device.sysstatus[2] & ((1 << RXRFTO_BIT) >> 16)))));
        
        dwInteruptHandler();
        break;
      }
    }

    if (recievedAck)
    {
      recievedAck = false;
      // log_data("RxCallback\r\n");
      dwTimestamp_t arival;
      // check data length
      int dataLenght = dwGetDataLength(&device);
      if (dataLenght == 0)
      {
        return 0;
      }
      memset(&rxPacket, 0, sizeof(rxPacket));
      // get data from anchor and put in rxPacket
      dwGetData(&device, (uint8_t *)&rxPacket, dataLenght);
      // check address of tag due to anchor send
      // correct return 0
      if (memcmp(rxPacket.destAddress, tagBaseAddr, 2))
      {
        // wrong address and repeat receive
        log_data("error address\r\n");
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
        case ANSWER:
          // log_data("ANSWER\r\n");
          if (rxPacket.payload[SEQ] != 2) // 2
          {
            log_data("wrong sequence number\r\n");
            return 0;
          }
          dwGetReceiveTimestamp(&device, &arival);
          arival.timeFull -= ANTENNA_DELAY;
          answer_rx = arival;
          // clear txpacket
          memset(&txPacket, 0, sizeof(txPacket));
          txPacket.payload[TYPE] = FINAL;
          txPacket.payload[SEQ] = 3; // 3
          memcpy(txPacket.destAddress, anchorAddress, 2);
          memcpy(txPacket.sourceAddress, tagBaseAddr, 2);
          dwNewTransmit(&device);
          dwSetDefaults(&device);
          dwSetData(&device, (uint8_t *)&txPacket, MAC802154_HEADER_LENGTH + 2); // 2 is length of payload
          dwStartTransmit(&device);
          do
          {
            dwReadSystemEventStatusRegister(&device);
          } while (!(device.sysstatus[0] & (1 << TXFRS_BIT)));
          dwInteruptHandler();
          break;
        case REPORT:
          // log_data("REPORT\r\n");
          {
          // reportpayload to receive timestamp of anchor send to tag to calculate
          // able truy cap cac phan tu trong payload thong qua con tro
          reportPayload_t *reportmess = (reportPayload_t *)(rxPacket.payload + 2);
          double tround1, treply1, tround2, treply2, tprop_ctn, tprop, distance;
          if (rxPacket.payload[SEQ] != 4) // 4
          {
            log_data("wrong sequence number\r\n");
            return 0;
          }
          memcpy(&poll_rx, &reportmess->pollRx, 5);
          memcpy(&answer_tx, &reportmess->answerTx, 5);
          memcpy(&final_rx, &reportmess->finalRx, 5);

          tround1 = answer_rx.timeLow32 - poll_tx.timeLow32;
          treply1 = answer_tx.timeLow32 - poll_rx.timeLow32;
          tround2 = final_rx.timeLow32 - answer_tx.timeLow32;
          treply2 = final_tx.timeLow32 - answer_rx.timeLow32;

          // printf("tround1: %f   treply2: %f\r\r\n", tround1, treply2);
          // printf("tround2: %f   treply1: %f\r\r\n", tround2, treply1);

          // tprop_ctn is value of resigter timer
          tprop_ctn = ((tround1 * tround2) - (treply1 * treply2)) / (tround1 + tround2 + treply1 + treply2);
          // printf("TProp (ctn): %d\r\r\n", (unsigned int)tprop_ctn);

          // tprop is value unit sencond
          tprop = tprop_ctn / tsfreq;
          distance = C * tprop;

          // printf("distance of anchor %d is: %5d(mm)", (int)rxPacket.sourceAddress, (unsigned int)distance * 1000);
          char buf1[50], buf2[30], buf3[30], buf4[30];

          // sprintf(buf3, "time of flight: %.2f(ns)\r\n", tprop * 1000000000000);
          // log_data(buf3);
          if(distance>0)
          {
            sprintf(buf1, "%.1f\r\n", distance * 1000);
            log_data(buf1);
          }

          // dwGetReceiveTimestamp(&device, &arival);
          // arival.timeFull -= ANTENNA_DELAY;
          // double totaltime;
          // totaltime = (arival.timeLow32 - poll_tx.timeLow32) / tsfreq;
          // sprintf(buf2, "total time: %.2f(ms)\r\n", totaltime*1000);
          // log_data(buf2);

          // curr_seq = 1;
          initAck = true;
          seq_mess++;
          break;
          }
        }
      }
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
