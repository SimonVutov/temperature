/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// GPIO Pin for DHT22 data
#define DHT22_GPIO_PORT GPIOA
#define DHT22_GPIO_PIN GPIO_PIN_1

/* Timing definitions for DHT22 communication */
#define DHT22_START_SIGNAL_DELAY 20   // Delay in ms (20ms for start signal)
#define DHT22_RESPONSE_TIMEOUT 100   // Timeout in us
#define DHT22_BIT_TIMEOUT 50         // Timeout for a single bit in us
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char uart_buf[100]; // UART transmission buffer
uint8_t uart_buf_len;
float temperature;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
int DHT22_ReadTemperature(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, float *temperature);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  /* Initialize the HAL Library */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* Main Loop */
  while (1) {
    // Attempt to read temperature from DHT22
    if (DHT22_ReadTemperature(DHT22_GPIO_PORT, DHT22_GPIO_PIN, &temperature) == 0) {
      // Successfully read temperature
      uart_buf_len = sprintf(uart_buf, "Temperature: %.2f°C\r\n", temperature);
      HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, HAL_MAX_DELAY);
    } else {
      // Failed to read temperature
      uart_buf_len = sprintf(uart_buf, "Failed to read temperature\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, HAL_MAX_DELAY);
    }

    // Wait 2 seconds before the next reading
    HAL_Delay(2000);
  }
}

/**
  * @brief Reads temperature from the DHT22 sensor using HAL GPIO.
  * @param GPIOx GPIO Port (e.g., GPIOA)
  * @param GPIO_Pin GPIO Pin (e.g., GPIO_PIN_1)
  * @param temperature Pointer to store the temperature value
  * @retval 0 if successful, 1 if there is an error
  */
int DHT22_ReadTemperature(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, float *temperature) {
  uint8_t data[5] = {0}; // Buffer to store 40 bits (5 bytes) of DHT22 data
  uint32_t time_us;

  // Step 1: Send Start Signal
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); // Pull data pin low
  HAL_Delay(DHT22_START_SIGNAL_DELAY);               // Wait for at least 20ms
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);  // Pull data pin high
  HAL_Delay(1);                                      // Wait 1ms

  // Switch GPIO to input mode
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);

  // Step 2: Wait for DHT22 Response
  time_us = 0;
  while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET) {
    time_us++;
    if (time_us > DHT22_RESPONSE_TIMEOUT) {
      return 1; // Timeout waiting for DHT22 response
    }
  }

  // Step 3: Read 40 bits (5 bytes) of data
  for (int i = 0; i < 40; i++) {
    // Wait for the start of the bit (low signal)
    while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET) {
      time_us++;
      if (time_us > DHT22_BIT_TIMEOUT) {
        return 1; // Timeout waiting for bit start
      }
    }

    // Measure the duration of the high signal
    time_us = 0;
    while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET) {
      time_us++;
      if (time_us > DHT22_BIT_TIMEOUT) {
        return 1; // Timeout waiting for bit signal
      }
    }

    // If the high signal is longer than a threshold, it's a '1'
    data[i / 8] <<= 1; // Shift previous bits
    if (time_us > 40) { // Adjust the threshold as needed
      data[i / 8] |= 1;
    }
  }

  // Step 4: Verify Checksum
  if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
    return 1; // Checksum error
  }

  // Step 5: Convert Data to Temperature
  *temperature = ((data[2] << 8) | (data[3] & 0x7F)) / 10.0;
  if (data[3] & 0x80) {
    *temperature *= -1; // Negative temperature
  }

  return 0; // Success
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /* Initializes the RCC Oscillators */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /* Initializes the CPU, AHB, and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /* Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void) {
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief  Error Handler
  * @retval None
  */
void Error_Handler(void) {
  while (1) {
    // Stay in an infinite loop for debugging purposes
  }
}