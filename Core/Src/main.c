
#include "main.h"

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdbool.h>

#include "SX1278.h"

#define MAX_TX_PACKET_SIZE   60  // adjust as needed
#define MAX_RX_PACKET_SIZE   60
#define SENSOR_PACKET_LEN   8    // sensor fields you copy into TX (3..10 -> 8 bytes)
#define CONNECTED_NODE_COUNT 5

#define dev_id 0xFF //gateway

#define GATEWAY_ADV_CMD 0x22
#define SENSOR_PACKET_CMD 0x02
#define FORWARD_PACKET_CMD 0x20
#define CONNECT_GATEWAY_CMD 0X01
#define CONNECT_NODE_CMD 0X10
#define NODE_ADV_CMD 0X11
#define GATEWAY_BEACON_CMD 0X33
#define NODE_BEACON (dev_id | 0x33)

uint8_t frame = 0;
uint8_t connected_dev[5] = {0x25,0x12,0x00,0x45,0x32};
uint32_t timeout_check[5] = {0};
uint8_t transmit_packet[7] = {0};
uint8_t receive_packet[60] = {0};
uint8_t frame_count[5] = {0};
#define TDMA_SLOT_TIME 2000 //2s
#define TDMA_GUARD_TIME 200
#define TDMA_TX_TIME    1600


SX1278_hw_t SX1278_hw;
SX1278_t SX1278;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

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


uint8_t tx_mode_start(uint32_t max_size, uint32_t timeout)
{
    uint8_t ret = SX1278_LoRaEntryTx(&SX1278, max_size, timeout);
    uart_printf("[TX] Entry TX: %d\n", ret);
    return ret;
}

uint8_t tx_mode_send(uint8_t *packet, uint32_t size, uint32_t timeout)
{
    uint8_t ret;

    uart_printf("[TX] TX HEX: ");
    for (uint32_t i = 0; i < size; i++)
        uart_printf("%02X ", packet[i]);
    uart_printf("\n");

    ret = SX1278_LoRaTxPacket(&SX1278, packet, size, timeout);
    uart_printf("[TX] Transmit: %d\n", ret);

    return ret;
}


uint8_t rx_mode_start(uint32_t max_size, uint32_t timeout)
{
    uint8_t ret = SX1278_LoRaEntryRx(&SX1278, max_size, timeout);
    uart_printf("[RX] Entry RX: %d\n", ret);
    return ret;
}

uint8_t rx_mode_standby(uint8_t *packet)
{
    uint8_t ret;
    HAL_Delay(200);
    ret = SX1278_LoRaRxPacket(&SX1278);
    if (ret > 0)
    {
        SX1278_read(&SX1278, packet, ret);

        uart_printf("[RX] Content: ");
        for (uint8_t i = 0; i < ret; i++)
            uart_printf("%02X ", packet[i]);
        uart_printf("\n");

        return ret;
    }

    return 0;
}




uint8_t packet_build(uint8_t cmd, uint8_t *transmit_packet)
{
	switch (cmd)
	{
	case GATEWAY_ADV_CMD:
		transmit_packet[0] = GATEWAY_ADV_CMD;
		transmit_packet[1] = 0xFF;
		for (uint8_t i = 2; i <7; i++)
			transmit_packet[i] = connected_dev[i-2];
	return 7;
	case GATEWAY_BEACON_CMD:
			transmit_packet[0] = GATEWAY_BEACON_CMD;
			transmit_packet[1] = 0xFF;
			transmit_packet[2] = frame++;
			for (uint8_t i = 3; i <8; i++)
				transmit_packet[i] = connected_dev[i-3];
		return 8;
	}
return 0;
}

void packet_process(uint8_t *receive_packet, uint8_t size)
{
	    uint8_t cmd = receive_packet[0];
	    uint8_t src = receive_packet[1];
	    uint8_t node_check = 0;
	    uint8_t slot_check = 0;
	    uint8_t frame_check = 0;
	    uint8_t buffer[10] = {0};
	    switch (cmd)
	    {

	    case CONNECT_GATEWAY_CMD:
	    {
	        // add device into list if needed
	        for (int i = 0; i < 5; i++)
	        {
	            if (connected_dev[i] == src)
	            {
	                uart_printf("Device %02X already connected\n", src);

	                break;
	            }
	            else if (connected_dev[i] == 0)
	            {
	                connected_dev[i] = src;


	                uart_printf("Added dev %02X\n", src);
	                break;
	            }
	        }
	       break;
	    }
	    case SENSOR_PACKET_CMD:
	    	node_check = receive_packet[2];
	    	slot_check = receive_packet[3];
	    	frame_check = receive_packet[4];
	    	frame_count[slot_check] = frame_check;
	    	buffer[0] = node_check;
	    	for (uint8_t i = 1; i < 9; i++)
	    		buffer[i] = receive_packet[i+4];
	    	send_to_esp(buffer, 9);
	    	break;
	    default:
	       break;  // unknown command
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


bool interval_check(uint32_t *target, uint32_t timeout)
{
    uint32_t current = HAL_GetTick();

    if (current - *target >= timeout)
    {
        *target = current;
        return true;
    }
    return false;
}

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();


  SystemClock_Config();


  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */


   SX1278_hw.dio0.port =GPIOA;
   SX1278_hw.dio0.pin = GPIO_PIN_2;
   SX1278_hw.nss.port = GPIOA;
   SX1278_hw.nss.pin = GPIO_PIN_4;
   SX1278_hw.reset.port = GPIOA;
   SX1278_hw.reset.pin = GPIO_PIN_3;
   SX1278_hw.spi = &hspi1;

    SX1278.hw = &SX1278_hw;

    uart_printf("Configuring LoRa module\r\n");
    SX1278_init(&SX1278, 433000000, SX1278_POWER_20DBM, SX1278_LORA_SF_7,
    SX1278_LORA_BW_250KHZ, SX1278_LORA_CR_4_5, SX1278_LORA_CRC_EN, 15);
    uart_printf("Done configuring LoRaModule\r\n");
    SX1278_LoRaEntryRx(&SX1278, 16, 2000);
    uint32_t last_adv = 0;
    uint32_t last_beacon = 0;
    while (1)
    {
    	uint8_t rx_size = rx_mode_standby(receive_packet);
    	if (rx_size > 0)
    	{
    		packet_process(receive_packet, rx_size);
    	}

    	if (interval_check(&last_adv, 5000) )
    	{
    		uint8_t tx_size = packet_build(GATEWAY_ADV_CMD, transmit_packet);
    		tx_mode_start(tx_size, 1000);
    		tx_mode_send(transmit_packet, tx_size, 1000);
    		rx_mode_start(10, 700);
    	}
    	if (interval_check(&last_beacon, 12000))
    	    	{
    	    		uint8_t tx_size = packet_build(GATEWAY_BEACON_CMD, transmit_packet);
    	    		tx_mode_start(tx_size, 1000);
    	    		tx_mode_send(transmit_packet, tx_size, 1000);
    	    		rx_mode_start(10, 700);
    	    	}
    	HAL_Delay(200);
    }
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

