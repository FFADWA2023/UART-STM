/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
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
COM_InitTypeDef BspCOMInit;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t received_data;                // Single-byte buffer for receiving data
uint8_t receive_buffer[100];          // Buffer to store incoming RFID data
uint8_t transfer_complete = 0;        // Flag to indicate the end of message
uint8_t receive_index = 0;            // Index for the receive_buffer array
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void log_message(const char* message); // Function prototype for logging
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
    /* Initialize the Hardware Abstraction Layer */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize GPIO and UART */
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    // Start UART receive in interrupt mode for a single byte
    HAL_UART_Receive_IT(&huart2, &received_data, 1);

    // Log initialization message
    log_message("UART Initialized and Ready to Receive\n");

    while (1)
    {
        if (transfer_complete)
        {
            // Transmit the full message received via UART to PuTTY
            HAL_UART_Transmit(&huart2, receive_buffer, receive_index, 10);

            // Log the received message
            receive_buffer[receive_index] = '\0'; // Null-terminate the string for logging
            log_message("Received Message: ");
            log_message((const char*)receive_buffer); // Log the received buffer
            log_message("\n");

            // Reset the index and transfer complete flag
            receive_index = 0;
            transfer_complete = 0;
        }
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

    /** Configure the main internal regulator output voltage */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN = 85;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200; // Set the baud rate to 115200
    huart2.Init.WordLength = UART_WORDLENGTH_8B; // Set word length to 8 bits
    huart2.Init.StopBits = UART_STOPBITS_1; // Set stop bits to 1
    huart2.Init.Parity = UART_PARITY_NONE; // No parity
    huart2.Init.Mode = UART_MODE_TX_RX; // Enable TX and RX
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE; // No hardware flow control
    huart2.Init.OverSampling = UART_OVERSAMPLING_16; // Over sampling by 16
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE; // Disable one bit sampling
    huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1; // Set prescaler to divide by 1
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; // No advanced features

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler(); // Handle initialization error
    }

    if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler(); // Handle FIFO threshold setting error
    }

    if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler(); // Handle FIFO threshold setting error
    }

    if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
    {
        Error_Handler(); // Handle disabling FIFO mode error
    }
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
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

    /* Configure GPIO pin : PA5 */
    GPIO_InitStruct.Pin = GPIO_PIN_5; // Pin for LED or other output
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Set pin mode to push-pull output
    GPIO_InitStruct.Pull = GPIO_NOPULL; // No pull-up or pull-down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Low speed
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // Initialize GPIO
}

/**
  * @brief UART RX Complete Callback
  * @param huart: UART handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // Log received data for debugging
        char debug_message[50];
        sprintf(debug_message, "Received Byte: %c\n", received_data);
        log_message(debug_message);

        if (received_data == '\n')  // Check for end of message
        {
            transfer_complete = 1;  // Set flag to indicate message is ready
        }
        else
        {
            // Store the received byte in the buffer
            if (receive_index < sizeof(receive_buffer) - 1) // Avoid overflow
            {
                receive_buffer[receive_index++] = received_data; // Store data
            }
            else
            {
                log_message("Buffer Overflow, resetting...\n"); // Log overflow
                receive_index = 0;  // Reset index if buffer overflows
            }
        }

        // Re-enable UART receive interrupt for the next byte
        HAL_UART_Receive_IT(&huart2, &received_data, 1);
    }
}

/**
  * @brief  Log message to UART for debugging
  * @param message: pointer to the string message
  * @retval None
  */
void log_message(const char* message)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 10); // Send message via UART
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    // User can add his own implementation to report the HAL error return state
    while(1)
    {
        // Error handling code
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
