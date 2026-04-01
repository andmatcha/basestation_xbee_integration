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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "stddef.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UPLINK_TX_UART_HANDLE huart6
#define ROVER_DMA_RX_BUFFER_SIZE 256U
#define ARM_DMA_RX_BUFFER_SIZE 256U
#define ROVER_LINE_MAX_LEN 96U
#define UPLINK_TX_QUEUE_DEPTH 16U
#define UPLINK_TX_FRAME_MAX_LEN 128U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DEBUG_LOG(fmt, ...) printf("[DBG] " fmt "\r\n", ##__VA_ARGS__)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim13;
DMA_HandleTypeDef hdma_tim3_ch3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */
typedef struct __attribute__((packed)) {
    char     header[2];   // "AC"
    uint8_t  seq;         // シーケンス番号        [ 1 byte ]
    uint8_t  flags;       // フラグ (bit4-5: mode) [ 1 byte ]
    uint16_t current[7];  // 電流値 x7             [14 bytes]
    uint16_t angle[3];    // 角度   x3             [ 6 bytes]
    int16_t  vel[3];      // 速度   x3             [ 6 bytes]
    uint8_t control_byte; // 1 byte
    int16_t base_rel_mm_j0; // 2 bytes
    uint16_t auto_flags; // 2 bytes
    uint16_t fault_code; // 2 bytes
    uint16_t crc16; // 2 bytes
} PacketAC_v6;            // 合計 39 bytes

typedef struct {
    uint8_t data[UPLINK_TX_FRAME_MAX_LEN];
    uint16_t len;
} UplinkTxFrame;

typedef enum {
    ARM_SYNC_WAIT_A = 0,
    ARM_SYNC_WAIT_C,
    ARM_SYNC_COLLECT_PAYLOAD,
} ArmRxState;

static uint8_t rover_dma_rx_buffer[ROVER_DMA_RX_BUFFER_SIZE];
static uint16_t rover_dma_last_pos = 0;
static char rover_line_buffer[ROVER_LINE_MAX_LEN];
static uint16_t rover_line_index = 0;
static bool rover_line_overflow = false;
static uint32_t rover_dma_chunk_count = 0U;
static uint32_t rover_raw_byte_count = 0U;

static uint8_t arm_dma_rx_buffer[ARM_DMA_RX_BUFFER_SIZE];
static uint16_t arm_dma_last_pos = 0;
static uint8_t arm_packet_buffer[sizeof(PacketAC_v6)];
static uint16_t arm_packet_index = 0;
static ArmRxState arm_rx_state = ARM_SYNC_WAIT_A;
static uint32_t arm_dma_chunk_count = 0U;
static uint32_t arm_raw_byte_count = 0U;

static UplinkTxFrame uplink_tx_queue[UPLINK_TX_QUEUE_DEPTH];
static volatile uint16_t uplink_tx_head = 0;
static volatile uint16_t uplink_tx_tail = 0;
static volatile uint16_t uplink_tx_count = 0;
static volatile bool uplink_tx_busy = false;
static uint8_t uplink_tx_dma_buffer[UPLINK_TX_FRAME_MAX_LEN];
static uint32_t rover_valid_line_count = 0U;
static uint32_t rover_invalid_line_count = 0U;
static uint32_t arm_valid_packet_count = 0U;
static uint32_t arm_invalid_packet_count = 0U;
static uint32_t uplink_enqueue_drop_count = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef startCircularReception(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size);
static void restartCircularReception(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size);
static void pollRoverDmaRx(void);
static void pollArmDmaRx(void);
static void consumeRoverByte(uint8_t byte);
static void consumeArmByte(uint8_t byte);
static bool validateRoverLine(const char *line);
static bool validateArmPacket(const uint8_t *raw_packet);
static uint16_t crc16_ccitt_false(const uint8_t* data, size_t len);
static bool enqueueUplinkFrame(const uint8_t *data, uint16_t len);
static void pumpUplinkTx(void);
static const char *uartName(const UART_HandleTypeDef *huart);
static void logArmPacketSummary(const uint8_t *raw_packet);
static void logRxChunk(const char *label,
                       const uint8_t *buffer,
                       uint16_t start_pos,
                       uint16_t end_pos,
                       uint16_t buffer_size,
                       uint32_t *chunk_count,
                       uint32_t *byte_count);
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

  /* USER CODE BEGIN 1 */
  printf("[BOOT] main entered\r\n");
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
  MX_DMA_Init();
  MX_TIM13_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  DEBUG_LOG("Peripherals initialized, starting DMA reception");
  if (startCircularReception(&huart1, rover_dma_rx_buffer, sizeof(rover_dma_rx_buffer)) != HAL_OK) {
    Error_Handler();
  }

  if (startCircularReception(&huart2, arm_dma_rx_buffer, sizeof(arm_dma_rx_buffer)) != HAL_OK) {
    Error_Handler();
  }
  DEBUG_LOG("Initialization complete, entering main loop");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    pollRoverDmaRx();
    pollArmDmaRx();
    pumpUplinkTx();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 104;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 83;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 999;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 57600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : Push_Switch_Pin */
  GPIO_InitStruct.Pin = Push_Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Push_Switch_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static const char *uartName(const UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        return "USART1";
    }

    if (huart == &huart2) {
        return "USART2";
    }

    if (huart == &huart3) {
        return "USART3";
    }

    if (huart == &huart4) {
        return "UART4";
    }

    if (huart == &huart6) {
        return "USART6";
    }

    return "UNKNOWN";
}

static void logArmPacketSummary(const uint8_t *raw_packet)
{
    PacketAC_v6 packet;

    memcpy(&packet, raw_packet, sizeof(packet));
    DEBUG_LOG("Arm packet ok #%lu: seq=%u flags=0x%02X ctrl=0x%02X fault=0x%04X crc=0x%04X",
              (unsigned long)arm_valid_packet_count,
              packet.seq,
              packet.flags,
              packet.control_byte,
              packet.fault_code,
              packet.crc16);
}

static void logRxChunk(const char *label,
                       const uint8_t *buffer,
                       uint16_t start_pos,
                       uint16_t end_pos,
                       uint16_t buffer_size,
                       uint32_t *chunk_count,
                       uint32_t *byte_count)
{
    uint16_t chunk_size;
    uint16_t last_pos;
    uint8_t first_byte;
    uint8_t last_byte;

    if (start_pos == end_pos) {
        return;
    }

    if (end_pos > start_pos) {
        chunk_size = (uint16_t)(end_pos - start_pos);
    } else {
        chunk_size = (uint16_t)((buffer_size - start_pos) + end_pos);
    }

    first_byte = buffer[start_pos];
    last_pos = (end_pos == 0U) ? (uint16_t)(buffer_size - 1U) : (uint16_t)(end_pos - 1U);
    last_byte = buffer[last_pos];

    (*chunk_count)++;
    *byte_count += chunk_size;

    if (*chunk_count <= 8U || ((*chunk_count % 32U) == 0U)) {
        DEBUG_LOG("%s raw chunk #%lu: bytes=%u total=%lu first=0x%02X last=0x%02X pos=%u->%u",
                  label,
                  (unsigned long)*chunk_count,
                  (unsigned int)chunk_size,
                  (unsigned long)*byte_count,
                  first_byte,
                  last_byte,
                  (unsigned int)start_pos,
                  (unsigned int)end_pos);
    }
}

static HAL_StatusTypeDef startCircularReception(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size)
{
    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(huart, buffer, size);

    if (status == HAL_OK) {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_TC);
        DEBUG_LOG("DMA RX started on %s, buffer=%u bytes", uartName(huart), (unsigned int)size);
    } else {
        DEBUG_LOG("DMA RX start failed on %s, status=%d", uartName(huart), (int)status);
    }

    return status;
}

static void restartCircularReception(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size)
{
    HAL_StatusTypeDef stop_status = HAL_UART_DMAStop(huart);

    DEBUG_LOG("Restarting DMA RX on %s", uartName(huart));
    if (stop_status != HAL_OK) {
        DEBUG_LOG("HAL_UART_DMAStop returned %d on %s", (int)stop_status, uartName(huart));
    }

    if (startCircularReception(huart, buffer, size) != HAL_OK) {
        Error_Handler();
    }
}

static void pollRoverDmaRx(void)
{
    uint16_t start_pos = rover_dma_last_pos;
    uint16_t dma_pos = (uint16_t)(sizeof(rover_dma_rx_buffer) - __HAL_DMA_GET_COUNTER(huart1.hdmarx));

    logRxChunk("USART1", rover_dma_rx_buffer, start_pos, dma_pos, sizeof(rover_dma_rx_buffer),
               &rover_dma_chunk_count, &rover_raw_byte_count);

    while (rover_dma_last_pos != dma_pos) {
        consumeRoverByte(rover_dma_rx_buffer[rover_dma_last_pos]);
        rover_dma_last_pos++;
        if (rover_dma_last_pos >= sizeof(rover_dma_rx_buffer)) {
            rover_dma_last_pos = 0;
        }
    }
}

static void pollArmDmaRx(void)
{
    uint16_t start_pos = arm_dma_last_pos;
    uint16_t dma_pos = (uint16_t)(sizeof(arm_dma_rx_buffer) - __HAL_DMA_GET_COUNTER(huart2.hdmarx));

    logRxChunk("USART2", arm_dma_rx_buffer, start_pos, dma_pos, sizeof(arm_dma_rx_buffer),
               &arm_dma_chunk_count, &arm_raw_byte_count);

    while (arm_dma_last_pos != dma_pos) {
        consumeArmByte(arm_dma_rx_buffer[arm_dma_last_pos]);
        arm_dma_last_pos++;
        if (arm_dma_last_pos >= sizeof(arm_dma_rx_buffer)) {
            arm_dma_last_pos = 0;
        }
    }
}

static void consumeRoverByte(uint8_t byte)
{
    uint8_t tx_frame[ROVER_LINE_MAX_LEN + 2U];

    if (byte == '\r') {
        return;
    }

    if (byte == '\n') {
        if (rover_line_overflow) {
            DEBUG_LOG("Rover line dropped after overflow");
            rover_line_index = 0;
            rover_line_overflow = false;
            return;
        }

        if (rover_line_index == 0U) {
            return;
        }

        rover_line_buffer[rover_line_index] = '\0';
        if (validateRoverLine(rover_line_buffer)) {
            rover_valid_line_count++;
            DEBUG_LOG("Rover line ok #%lu: %s", (unsigned long)rover_valid_line_count, rover_line_buffer);
            memcpy(tx_frame, rover_line_buffer, rover_line_index);
            tx_frame[rover_line_index] = '\r';
            tx_frame[rover_line_index + 1U] = '\n';
            (void)enqueueUplinkFrame(tx_frame, (uint16_t)(rover_line_index + 2U));
        } else {
            rover_invalid_line_count++;
            DEBUG_LOG("Rover line rejected #%lu: %s", (unsigned long)rover_invalid_line_count, rover_line_buffer);
        }
        rover_line_index = 0;
        return;
    }

    if (rover_line_overflow) {
        return;
    }

    if (rover_line_index >= (ROVER_LINE_MAX_LEN - 1U)) {
        rover_line_index = 0;
        rover_line_overflow = true;
        DEBUG_LOG("Rover line overflow, dropping until newline");
        return;
    }

    rover_line_buffer[rover_line_index++] = (char)byte;
}

static void consumeArmByte(uint8_t byte)
{
    DEBUG_LOG("USART2 byte=0x%02X", byte);
    switch (arm_rx_state) {
    case ARM_SYNC_WAIT_A:
        if (byte == 'A') {
            arm_packet_buffer[0] = byte;
            arm_packet_index = 1U;
            arm_rx_state = ARM_SYNC_WAIT_C;
        }
        break;

    case ARM_SYNC_WAIT_C:
        if (byte == 'C') {
            arm_packet_buffer[1] = byte;
            arm_packet_index = 2U;
            arm_rx_state = ARM_SYNC_COLLECT_PAYLOAD;
        } else if (byte == 'A') {
            arm_packet_buffer[0] = byte;
            arm_packet_index = 1U;
        } else {
            arm_packet_index = 0U;
            arm_rx_state = ARM_SYNC_WAIT_A;
        }
        break;

    case ARM_SYNC_COLLECT_PAYLOAD:
        arm_packet_buffer[arm_packet_index++] = byte;
        if (arm_packet_index >= sizeof(PacketAC_v6)) {
            if (validateArmPacket(arm_packet_buffer)) {
                arm_valid_packet_count++;
                logArmPacketSummary(arm_packet_buffer);
                (void)enqueueUplinkFrame(arm_packet_buffer, sizeof(PacketAC_v6));
            }
            arm_packet_index = 0U;
            arm_rx_state = ARM_SYNC_WAIT_A;
        }
        break;

    default:
        arm_packet_index = 0U;
        arm_rx_state = ARM_SYNC_WAIT_A;
        break;
    }
}

static bool validateRoverLine(const char *line)
{
    const char *p = line;
    unsigned long can_id = 0UL;

    if (p[0] != '0' || (p[1] != 'x' && p[1] != 'X')) {
        return false;
    }

    p += 2;
    if (*p == '\0') {
        return false;
    }

    while (*p != '\0' && *p != ',') {
        char c = *p;
        if (c >= '0' && c <= '9') {
            can_id = (can_id * 16UL) + (unsigned long)(c - '0');
        } else if (c >= 'A' && c <= 'F') {
            can_id = (can_id * 16UL) + (unsigned long)(c - 'A' + 10);
        } else if (c >= 'a' && c <= 'f') {
            can_id = (can_id * 16UL) + (unsigned long)(c - 'a' + 10);
        } else {
            return false;
        }
        p++;
    }

    if (*p != ',' || can_id > 0x7FFUL) {
        return false;
    }

    p++;
    if (*p == '-' || *p == '+') {
        p++;
    }
    if (*p == '\0') {
        return false;
    }

    while (*p != '\0') {
        if (*p < '0' || *p > '9') {
            return false;
        }
        p++;
    }

    return true;
}

static bool validateArmPacket(const uint8_t *raw_packet)
{
    PacketAC_v6 packet;
    uint16_t crc_calc;

    memcpy(&packet, raw_packet, sizeof(packet));
    if (packet.header[0] != 'A' || packet.header[1] != 'C') {
        arm_invalid_packet_count++;
        DEBUG_LOG("Arm packet header mismatch #%lu: %02X %02X",
                  (unsigned long)arm_invalid_packet_count,
                  raw_packet[0],
                  raw_packet[1]);
        return false;
    }

    crc_calc = crc16_ccitt_false(raw_packet, offsetof(PacketAC_v6, crc16));
    if (crc_calc != packet.crc16) {
        arm_invalid_packet_count++;
        DEBUG_LOG("Arm packet CRC mismatch #%lu: seq=%u calc=0x%04X recv=0x%04X",
                  (unsigned long)arm_invalid_packet_count,
                  packet.seq,
                  crc_calc,
                  packet.crc16);
        return false;
    }

    return true;
}

static uint16_t crc16_ccitt_false(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFFU;

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static bool enqueueUplinkFrame(const uint8_t *data, uint16_t len)
{
    uint32_t primask;
    uint16_t tail;
    uint16_t head_snapshot;
    uint16_t tail_snapshot;

    if (len == 0U || len > UPLINK_TX_FRAME_MAX_LEN) {
        DEBUG_LOG("Uplink enqueue rejected: invalid len=%u", (unsigned int)len);
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (uplink_tx_count >= UPLINK_TX_QUEUE_DEPTH) {
        uplink_enqueue_drop_count++;
        head_snapshot = uplink_tx_head;
        tail_snapshot = uplink_tx_tail;
        __set_PRIMASK(primask);
        DEBUG_LOG("Uplink enqueue rejected #%lu: queue full len=%u head=%u tail=%u",
                  (unsigned long)uplink_enqueue_drop_count,
                  (unsigned int)len,
                  (unsigned int)head_snapshot,
                  (unsigned int)tail_snapshot);
        return false;
    }

    tail = uplink_tx_tail;
    memcpy(uplink_tx_queue[tail].data, data, len);
    uplink_tx_queue[tail].len = len;
    uplink_tx_tail = (uint16_t)((tail + 1U) % UPLINK_TX_QUEUE_DEPTH);
    uplink_tx_count++;
    __set_PRIMASK(primask);
    return true;
}

static void pumpUplinkTx(void)
{
    uint32_t primask;
    uint16_t len;
    uint16_t queued_count;

    primask = __get_PRIMASK();
    __disable_irq();
    if (uplink_tx_busy || uplink_tx_count == 0U) {
        __set_PRIMASK(primask);
        return;
    }

    len = uplink_tx_queue[uplink_tx_head].len;
    queued_count = uplink_tx_count;
    memcpy(uplink_tx_dma_buffer, uplink_tx_queue[uplink_tx_head].data, len);
    uplink_tx_busy = true;
    __set_PRIMASK(primask);

    DEBUG_LOG("Uplink TX start: len=%u queued=%u first=0x%02X second=0x%02X",
              (unsigned int)len,
              (unsigned int)queued_count,
              uplink_tx_dma_buffer[0],
              (len > 1U) ? uplink_tx_dma_buffer[1] : 0U);

    if (HAL_UART_Transmit_DMA(&UPLINK_TX_UART_HANDLE, uplink_tx_dma_buffer, len) != HAL_OK) {
        primask = __get_PRIMASK();
        __disable_irq();
        uplink_tx_busy = false;
        __set_PRIMASK(primask);
        DEBUG_LOG("Uplink TX DMA start failed");
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t primask;

    if (huart != &UPLINK_TX_UART_HANDLE) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (uplink_tx_count > 0U) {
        uplink_tx_head = (uint16_t)((uplink_tx_head + 1U) % UPLINK_TX_QUEUE_DEPTH);
        uplink_tx_count--;
    }
    uplink_tx_busy = false;
    __set_PRIMASK(primask);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    DEBUG_LOG("UART error on %s: code=0x%08lX",
              uartName(huart),
              (unsigned long)HAL_UART_GetError(huart));

    if (huart == &huart1) {
        rover_dma_last_pos = 0U;
        rover_line_index = 0U;
        rover_line_overflow = false;
        restartCircularReception(&huart1, rover_dma_rx_buffer, sizeof(rover_dma_rx_buffer));
        return;
    }

    if (huart == &huart2) {
        arm_dma_last_pos = 0U;
        arm_packet_index = 0U;
        arm_rx_state = ARM_SYNC_WAIT_A;
        restartCircularReception(&huart2, arm_dma_rx_buffer, sizeof(arm_dma_rx_buffer));
        return;
    }

    if (huart == &UPLINK_TX_UART_HANDLE) {
        (void)HAL_UART_DMAStop(huart);
        uplink_tx_busy = false;
    }
}

void send_xbee_int(uint16_t id, int value)
{
    char msg_uart[32];
    int len = snprintf(msg_uart, sizeof(msg_uart), "0x%03X,%d\r\n", id, value);

    if (len > 0) {
        DEBUG_LOG("send_xbee_int: id=0x%03X value=%d", id, value);
        (void)enqueueUplinkFrame((const uint8_t*)msg_uart, (uint16_t)len);
        pumpUplinkTx();
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  printf("[ERR] Error_Handler invoked\r\n");
  NVIC_SystemReset();
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
