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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
#include "hw.h"
#include "SCH1.h"
#include "minmea.h"
#include "usbd_cdc_if.h"
#include <string.h>
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
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
volatile uint8_t  dry_flag = 0;
volatile uint32_t cyccnt_hi = 0;
volatile uint32_t cyccnt_prev = 0;
volatile uint64_t pps_posix_us = 0;
volatile uint64_t pps_cycles = 0;          // local cycle count at the last accepted PPS
volatile uint32_t pps_cycles_per_sec = 0;  // measured CPU clock, set to nominal in main
volatile uint64_t pending_posix_us = 0;
volatile uint64_t dry_timestamp_us = 0;
volatile uint8_t  dry_gps_status = 0;

// GPS sync status, sent with every sample (see README.md)
#define GPS_TIME_VALID  0x01  // locked to PPS at least once: timestamps are Unix time
#define GPS_PPS_OK      0x02  // last PPS accepted < 1.1 s ago; otherwise holding over
#define GPS_NMEA_OK     0x04  // valid RMC time received < 1 s ago
#define GPS_RATE_CAL    0x08  // CPU clock rate measured against PPS at least once
volatile uint8_t  gps_flags = 0;  // GPS_TIME_VALID and GPS_RATE_CAL, set by the PPS handler
volatile uint32_t pps_holdoff_count_ms = 0;
volatile uint32_t pps_nmea_age_ms = 0;
static uint8_t nmea_rx_byte = 0;
static char    nmea_buf[100];
static uint8_t nmea_len = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// CRC-8, polynomial 0x07, init 0. Table-driven: the bitwise version took ~60 us per packet.
static uint8_t crc8_table[256];

static void crc8_init(void)
{
    for (int i = 0; i < 256; i++) {
        uint8_t crc = (uint8_t)i;
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
        crc8_table[i] = crc;
    }
}

static uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++)
        crc = crc8_table[crc ^ data[i]];
    return crc;
}

// Extends the 32-bit DWT cycle counter to 64 bits. Every caller checks for the
// wrap itself, so a DRY/PPS interrupt that preempts SysTick right after the
// counter wraps still gets the right high word. Must be called at least once
// per wrap period (~59 s at 72 MHz); SysTick does that.
uint64_t get_local_cycles(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t cnt = DWT->CYCCNT;
    if (cnt < cyccnt_prev)
        cyccnt_hi++;
    cyccnt_prev = cnt;
    uint64_t cycles = ((uint64_t)cyccnt_hi << 32) | cnt;
    __set_PRIMASK(primask);
    return cycles;
}

// Converts local cycles since the last PPS to GPS time, using the CPU clock
// rate measured between PPS pulses instead of the nominal 72 MHz. Split into
// whole seconds and remainder so it cannot overflow however long PPS is lost.
static uint64_t local_to_posix_us(uint64_t cycles)
{
    uint64_t elapsed = cycles - pps_cycles;
    uint32_t cps     = pps_cycles_per_sec;
    uint64_t secs    = elapsed / cps;
    uint64_t rem     = elapsed % cps;
    return pps_posix_us + secs * 1000000ULL + rem * 1000000ULL / cps;
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
  // Before any EXTI is enabled: the DRY handler divides by this.
  pps_cycles_per_sec = SystemCoreClock;

  // The board has a fixed D+ pull-up, so hold D+ low briefly to make the host
  // re-enumerate after a reset or flash without replugging the cable.
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef usb_dp = {
    .Pin   = GPIO_PIN_12,
    .Mode  = GPIO_MODE_OUTPUT_PP,
    .Speed = GPIO_SPEED_FREQ_LOW,
  };
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_GPIO_Init(GPIOA, &usb_dp);
  HAL_Delay(10);
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  // CYCCNT survives a system reset and SysTick has already sampled it, so reset
  // the extension state together with it or the next read looks like a wrap.
  __disable_irq();
  DWT->CYCCNT = 0;
  cyccnt_prev = 0;
  cyccnt_hi   = 0;
  __enable_irq();
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
  HAL_UART_Receive_IT(&huart3, &nmea_rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  hw_init();

  SCH1_filter Filter = {
    .Rate12 = FILTER_RATE,
    .Acc12  = FILTER_ACC12,
    .Acc3   = FILTER_ACC3,
  };
  SCH1_sensitivity Sensitivity = {
    .Rate1 = SENSITIVITY_RATE1,
    .Rate2 = SENSITIVITY_RATE2,
    .Acc1  = SENSITIVITY_ACC1,
    .Acc2  = SENSITIVITY_ACC2,
    .Acc3  = SENSITIVITY_ACC3,
  };
  SCH1_decimation Decimation = {
    .Rate2 = DECIMATION_RATE,
    .Acc2  = DECIMATION_ACC,
  };

  int init_status;
  do {
    init_status = SCH1_init(Filter, Sensitivity, Decimation, true);
    if (init_status != SCH1_OK) {
      HAL_UART_Transmit(&huart1, (uint8_t*)"SCH1 NOK\r\n", 10, 500);
    }

  } while (init_status != SCH1_OK);

  HAL_UART_Transmit(&huart1, (uint8_t*)"SCH1 OK\r\n", 9, 500);

  typedef struct __attribute__((packed)) {
    uint8_t  sync;
    uint64_t timestamp_us;
    int32_t  rate[3];
    int32_t  acc[3];
    int32_t  temp;
    uint8_t  gps_status;  // GPS_* flags
    uint8_t  crc;
  } imu_pkt_t;

  static imu_pkt_t pkt[2];
  // USB batches packets: one CDC transfer per packet caps out around 1500 packets/s,
  // so packets collect in one buffer while the other is being sent.
  enum { USB_BATCH_PKTS = 26 };   // 26 * 39 B = 1014 B per transfer
  static imu_pkt_t usb_buf[2][USB_BATCH_PKTS];
  uint8_t  usb_fill = 0;
  uint16_t usb_count = 0;
  uint8_t pkt_idx = 0;
  SCH1_raw_data SCH1_data;
  crc8_init();

  while (1)
  {
    while (!dry_flag);
    // Take the timestamp together with the flag: the next DRY can arrive while
    // the SPI read below is running and would overwrite dry_timestamp_us.
    __disable_irq();
    dry_flag = 0;
    uint64_t timestamp_us = dry_timestamp_us;
    uint8_t  gps_status   = dry_gps_status;
    __enable_irq();

    SCH1_getData(&SCH1_data);
    if (!SCH1_data.frame_error) {
      uint8_t next = pkt_idx ^ 1;
      pkt[next].sync         = 0xAA;
      pkt[next].timestamp_us = timestamp_us;
      pkt[next].gps_status   = gps_status;
      pkt[next].rate[0] = SCH1_data.Rate2_raw[AXIS_X];
      pkt[next].rate[1] = SCH1_data.Rate2_raw[AXIS_Y];
      pkt[next].rate[2] = SCH1_data.Rate2_raw[AXIS_Z];
      pkt[next].acc[0]  = SCH1_data.Acc2_raw[AXIS_X];
      pkt[next].acc[1]  = SCH1_data.Acc2_raw[AXIS_Y];
      pkt[next].acc[2]  = SCH1_data.Acc2_raw[AXIS_Z];
      pkt[next].temp    = SCH1_data.Temp_raw;
      static uint16_t t = 0;
      if (t++ % 100 == 0) {
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
      }

      pkt[next].crc = crc8((uint8_t*)&pkt[next], offsetof(imu_pkt_t, crc));

      if (huart1.gState == HAL_UART_STATE_READY) {
        pkt_idx = next;
        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&pkt[pkt_idx], sizeof(imu_pkt_t));
      }

      // A full batch means the host is not reading; drop rather than block the sensor loop
      if (usb_count < USB_BATCH_PKTS)
        usb_buf[usb_fill][usb_count++] = pkt[next];
      if (usb_count > 0 && CDC_IsTxReady_FS()) {
        CDC_Transmit_FS((uint8_t*)usb_buf[usb_fill], usb_count * sizeof(imu_pkt_t));
        usb_fill ^= 1;
        usb_count = 0;
      }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  huart1.Init.BaudRate = 460800;
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
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_RX;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DBG_Pin|EXTERNSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DBG_Pin EXTERNSN_Pin */
  GPIO_InitStruct.Pin = DBG_Pin|EXTERNSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DRY_Pin */
  GPIO_InitStruct.Pin = DRY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PPS_IN_Pin */
  GPIO_InitStruct.Pin = PPS_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PPS_IN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == DRY_Pin) {


    dry_timestamp_us = local_to_posix_us(get_local_cycles());
    uint8_t status = gps_flags;
    if ((status & GPS_TIME_VALID) && pps_holdoff_count_ms < 1100)
      status |= GPS_PPS_OK;
    if (pps_nmea_age_ms < 1000)
      status |= GPS_NMEA_OK;
    dry_gps_status = status;
    dry_flag = 1;
    return;
  }
  // accept pps only:
  //  - if PPS_IN_Pin is high
  //  - there is more than 900 ms from last pps
  //  - nmea message is not older than 1000 ms
  if (GPIO_Pin == PPS_IN_Pin && pps_holdoff_count_ms > 900 && pps_nmea_age_ms < 999) {
    uint64_t cycles = get_local_cycles();
    // The newest RMC is from the second before this pulse, but at NMEA rates
    // above 1 Hz it can be stamped .2/.4/.6/.8, so round down before adding 1 s.
    uint64_t posix_us = (pending_posix_us / 1000000ULL + 1ULL) * 1000000ULL;

    // Measure the CPU clock against GPS when the previous pulse was exactly 1 s
    // earlier. Reject anything over 200 ppm off nominal (missed or false pulse)
    // and smooth by 1/8 to average out PPS and interrupt latency jitter.
    if (posix_us - pps_posix_us == 1000000ULL) {
      uint64_t measured = cycles - pps_cycles;
      uint32_t nominal  = SystemCoreClock;
      uint32_t limit    = nominal / 5000U;
      if (measured > nominal - limit && measured < nominal + limit) {
        int32_t error = (int32_t)((uint32_t)measured - pps_cycles_per_sec);
        pps_cycles_per_sec += error / 8;
        gps_flags |= GPS_RATE_CAL;
      }
    }
    pps_cycles   = cycles;
    pps_posix_us = posix_us;
    pps_holdoff_count_ms = 0;
    gps_flags |= GPS_TIME_VALID;

  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3) {
    char c = (char)nmea_rx_byte;
    if (c == '$') nmea_len = 0;
    if (nmea_len < sizeof(nmea_buf) - 1)
      nmea_buf[nmea_len++] = c;
    if (c == '\n' && nmea_len > 6) {
      nmea_buf[nmea_len] = '\0';
      if (minmea_sentence_id(nmea_buf) == MINMEA_SENTENCE_RMC) {
        struct minmea_sentence_rmc rmc;
        if (minmea_parse_rmc(&rmc, nmea_buf) && rmc.valid) {
          struct timeval tv;
          if (minmea_gettimeofday(&tv, &rmc.date, &rmc.time) == 0) {
            // PPS preempts this handler, so update both together or it could
            // see a fresh age with a stale or half-written 64-bit time.
            __disable_irq();
            pending_posix_us = (uint64_t)tv.tv_sec * 1000000ULL + tv.tv_usec;
            pps_nmea_age_ms  = 0;
            __enable_irq();
          }
        }
      }
    }
    HAL_UART_Receive_IT(&huart3, &nmea_rx_byte, 1);
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
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
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
