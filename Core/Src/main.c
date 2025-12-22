/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SX1278.h"
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>




SX1278_hw_t SX1278_hw;
SX1278_t SX1278;
uint8_t receive_packet[60] = {0};


SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

osThreadId TransmitTaskHandle;
osThreadId ReceiveTaskHandle;

osSemaphoreId dataReadyHandle;
/* USER CODE BEGIN PV */


void uart_printf(const char *format,...)
{
	char packet[128];
	va_list args;
	va_start(args, format);
	int len = vsnprintf(packet, sizeof(packet), format, args);
	va_end(args);

	if (len > 0) {
	    if (len > sizeof(packet)) len = sizeof(packet);
	    HAL_UART_Transmit(&huart1, (uint8_t*)packet, len, 500);
	}
}

void send_to_esp(uint8_t *receive_packet, uint32_t size)
{
	    HAL_UART_Transmit(&huart3,receive_packet, size, 500);
}
int transmit_mode(uint8_t* packet,uint32_t size)
{
    HAL_Delay(100);

    int ret = SX1278_LoRaEntryTx(&SX1278, size, 2000);
    uart_printf("[TX] Entry: %d\n", ret);
    if (!ret) return 0;

    uart_printf("[TX] TX HEX: ");
    for(int i=0;i<size;i++)
        uart_printf("%02X ", packet[i]);
    uart_printf("\n");

    ret = SX1278_LoRaTxPacket(&SX1278, packet, size, 2000);

    uart_printf("[TX] Transmission: %d\n", ret);


    return 1;
}

int receive_mode(uint8_t* packet,uint32_t size)
{
    int ret;
    ret = SX1278_LoRaEntryRx(&SX1278, size, 2000);
    uart_printf("[RX] enter receive mode: %d\n", ret);
    if (!ret) return 0;

    HAL_Delay(500);



    ret = SX1278_LoRaRxPacket(&SX1278);
    uart_printf("[RX] Received: %d bytes\n", ret);

    if (ret > 0)
    {
        SX1278_read(&SX1278, packet, ret);
        uart_printf("[RX] Content HEX: ");
        for(int i=0;i<ret;i++)
            uart_printf("%02X ", packet[i]);
        uart_printf("\n");
    }


    return ret;   // <-- IMPORTANT: return number of bytes!
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
void TransmitTaskInit(void const * argument);
void ReceiveTaskInit(void const * argument);

uint8_t connected_dev[5] = {0x23,0x45,0x58,0x00,0x54};
uint32_t timeout_check[5] = {0};
uint8_t transmit_packet[7] = {0};

uint32_t gateway_handle(uint8_t *rx, uint32_t size)
{

    if (size < 2)        // every packet must have at least cmd + dest + src
        return 0;       // invalid, ignore

    uint8_t cmd = rx[0];
    uint8_t src = rx[1];

    switch (cmd)
    {
    /* ============================================================
       0x01 — CONNECT REQUEST
       Packet: [0x01][0xFF][dev_id]
       ============================================================*/
    case 0x01:
    {
        // add device into list if needed
        for (int i = 0; i < 5; i++)
        {
            if (connected_dev[i] == src)
            {
                uart_printf("Device %02X already connected\n", src);


                timeout_check[i] = HAL_GetTick();
                break;
            }
            else if (connected_dev[i] == 0)
            {
                connected_dev[i] = src;

                timeout_check[i] = HAL_GetTick();
                uart_printf("Added dev %02X\n", src);
                break;
            }
        }
        return 6;  // = 7 bytes
    }

    /* ============================================================
       0x02 — SENSOR DATA FROM NODE
       Packet: [0x02][0xFF][dev_id][data...]
       ============================================================*/
    case 0x02:
    {

        for (int i = 0;i < 5;i++)
        {
        	if (connected_dev[i] == src)
        	{

        		timeout_check[i] = HAL_GetTick();
        		uint8_t buff[9];
        		for (int j = 0; j < 9;j++)
        		{
        			buff[j] = rx[j+1];
        		}
        		send_to_esp(buff, 9);  // relay packet to ESP
        		break;
        	}
        }

        return 10;             // forward as-is
    }
    case 0x20:
        {

            for (int i = 0;i < 5;i++)
            {
            	if (connected_dev[i] == src)
            	{

            		timeout_check[i] = HAL_GetTick();
            		uint8_t buff[9];
            		for (int j = 0; j < 9;j++)
            		{
            			buff[j] = rx[j+2];
            		}
            		send_to_esp(buff, 9);  // relay packet to ESP
            		break;
            	}
            }

            return 11;             // forward as-is
        }

    default:
        return 0;  // unknown command
    }
}

void gateway_timeout_check(uint32_t timeout_ms)
{
     uint32_t now = HAL_GetTick();

    for (int i = 0; i < 5; i++)
    {
        if (connected_dev[i] != 0 )
        {
            if ((now - timeout_check[i]) > timeout_ms)
            {
                uart_printf("Device %02X OFFLINE\n", connected_dev[i]);

                timeout_check[i] = 0;

                connected_dev[i] = 0x00;   // optional: remove
            }
        }
    }
}

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
  MX_USART3_UART_Init();

  SX1278_hw.dio0.port =GPIOA;
     	SX1278_hw.dio0.pin = GPIO_PIN_2;
     	SX1278_hw.nss.port = GPIOA;
     	SX1278_hw.nss.pin = GPIO_PIN_4;
     	SX1278_hw.reset.port = GPIOA;
     	SX1278_hw.reset.pin = GPIO_PIN_3;
     	SX1278_hw.spi = &hspi1;

     	SX1278.hw = &SX1278_hw;

     	uart_printf("Configuring LoRa module\r\n");
     	SX1278_init(&SX1278, 434000000, SX1278_POWER_20DBM, SX1278_LORA_SF_10,
     	    	SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_7, SX1278_LORA_CRC_EN, 15);
     	uart_printf("Done configuring LoRaModule\r\n");
     	SX1278_LoRaEntryTx(&SX1278, 16, 2000);

     	osSemaphoreDef(dataReady);
     	  dataReadyHandle = osSemaphoreCreate(osSemaphore(dataReady), 1);
  /* definition and creation of TransmitTask */
  osThreadDef(TransmitTask, TransmitTaskInit, 0, 0, 220);
  TransmitTaskHandle = osThreadCreate(osThread(TransmitTask), NULL);

  /* definition and creation of ReceiveTask */
  osThreadDef(ReceiveTask, ReceiveTaskInit, -1, 0, 380);
  ReceiveTaskHandle = osThreadCreate(osThread(ReceiveTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RESET_PIN_Pin|NSS_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO_PIN_Pin */
  GPIO_InitStruct.Pin = DIO_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RESET_PIN_Pin NSS_PIN_Pin */
  GPIO_InitStruct.Pin = RESET_PIN_Pin|NSS_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_TransmitTaskInit */
/**
  * @brief  Function implementing the TransmitTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_TransmitTaskInit */
void TransmitTaskInit(void const * argument)
{
    while (1)
    {
        // Wait for packet ready
        if (osSemaphoreWait(dataReadyHandle, osWaitForever) != osOK)
            continue;

        transmit_packet[0] = 0x01;
        for (int i=1;i<6;i++)
        {
        	transmit_packet[i] = connected_dev[i-1];
        }

        int ret = transmit_mode(transmit_packet, 6);


        // Allow ReceiveTask to run next
        osSemaphoreRelease(dataReadyHandle);
        osDelay(500);
    }
}



void ReceiveTaskInit(void const * argument)
{
    int ret;

    while (1)
    {
        if (osSemaphoreWait(dataReadyHandle, osWaitForever) != osOK)
            continue;

        ret = receive_mode(receive_packet, sizeof(receive_packet));
        uart_printf("[RX] receive: %d\n", ret);

        if (ret > 0)
        {
            gateway_handle(receive_packet, ret);

        }
        else
        {
            uart_printf("[RX] receive failed\n");
        }
        gateway_timeout_check(600000);
        osSemaphoreRelease(dataReadyHandle);
        osDelay(200);
    }
}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
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

#ifdef  USE_FULL_ASSERT
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

