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
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define R 30//mm  //radious of wheel
#define PPR 2000 //pulses per revolution
#define CONV M_PI / 180.0f // 度/秒 を ラジアン/秒 に変換する時に掛ける
#define SPI_BUFFER_SIZE 13 // アドレス(1) + データ(12)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// 送信バッファ（最初の1バイト目に読み出し先アドレス 0x22 | 0x80 を入れておく）
uint8_t spi_tx_buf[SPI_BUFFER_SIZE] = {0xA2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 

// 受信バッファ（IMUごとに独立して用意する）
uint8_t imu1_rx_buf[SPI_BUFFER_SIZE];
uint8_t imu2_rx_buf[SPI_BUFFER_SIZE];
uint8_t imu3_rx_buf[SPI_BUFFER_SIZE];

// ステートマシン用の状態変数
// 0: 待機中, 1: IMU1読出中, 2: IMU2読出中, 3: IMU3読出中, 4: データ準備完了
volatile uint8_t imu_read_state = 0;

volatile uint8_t imu_data_ready = 0; // 割り込みフラグ

typedef struct {
    int16_t gx, gy, gz; //gyro (x,y,z)
    int16_t ax, ay, az; //accel(x,y,z)
} IMUData;

volatile IMUData imu[3];
uint8_t whoami, whoami2, whoami3;
float gyro_x, gyro_y, gyro_z;
float accel_x, accel_y, accel_z;
double gyro_x_bias = 0.0f;
double gyro_y_bias = 0.0f;
double gyro_z_bias = 0.0f;
const float G_sensitivity = 0.070f;
const float A_sensitivity = 0.488f;
const float GYRO_Z_SCALE = 0.9965f;
float cos_30, sin_30;
float beta = 0.1f; //Madgwickフィルタのゲイン
float roll, pitch, yaw;
float dt = 0.001f;
bool isSettingBias = 1;
float def_az = 0.0f;
float a_Ex = 0.0f;
float a_Ey = 0.0f;
float a_Ez = 0.0f;
float accelCVR; //mgをm/s^2に変換する時に掛ける
float vz = 0.0f; //z軸方向の速度
float z = 0.0f; //デバッグ用高さ
GPIO_TypeDef* const IMU_CS_PORTS[3] = {IMU1_CS_GPIO_Port, IMU2_CS_GPIO_Port, IMU3_CS_GPIO_Port};
const uint16_t IMU_CS_PINS[3]       = {IMU1_CS_Pin, IMU2_CS_Pin, IMU3_CS_Pin};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void LSM6_Write(uint8_t reg, uint8_t data, int port);
void LSM6_ReadMulti(uint8_t reg, uint8_t* pData, uint16_t size, int port);
uint8_t LSM6_Read(uint8_t reg, int port);
void INIT_IMU(int port);
float invSqrt(float x);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void getEulerAngles();
void resetBias();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; //クォータニオン (w, x, y ,z)

int _write(int file,char *ptr,int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 10);
  return len;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (&htim6 == htim) { // 1000Hz
    imu_data_ready = 1;
    
    // 現在待機中(0)か、前回の処理が終わってデータ準備完了(4)なら次を開始
    /*if (imu_read_state == 0 || imu_read_state == 4) {
      imu_read_state = 1; // 状態を「IMU1読出中」に変更
      
      // IMU1のCSをLOWにして通信開始
      HAL_GPIO_WritePin(IMU1_CS_GPIO_Port, IMU1_CS_Pin, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive_DMA(&hspi2, spi_tx_buf, imu1_rx_buf, SPI_BUFFER_SIZE);
    } else {
      // ここに入る場合は、1ms以内に前回計算が終わっていない（処理落ち）
      //printf("DMA START ERROR! State: %d\r\n", hspi2.State);
      // エラーカウントを増やすなどの処理を入れるとデバッグに役立ちます
    }*/
  }
}



void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI2) {
        // ここにブレークポイントを貼っておけば、エラー時に止まる
        printf("SPI ERROR! Code: %lu\r\n", hspi->ErrorCode);
        // エラー内容：hspi->ErrorCode を確認
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  int loop_count = 0;
  setbuf(stdout, NULL);

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
  MX_FDCAN1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(2000);

  INIT_IMU(1);
  INIT_IMU(2);
  INIT_IMU(3);

  cos_30 = sqrt(3)/2;
  sin_30 = 0.50f;

  resetBias();
  //printf("bias resetted\r\n");
  //__HAL_SPI_CLEAR_OVRFLAG(&hspi2);

  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (imu_data_ready){
      imu_data_ready = 0;
      loop_count++;
      uint8_t buffer[12];
      for (int i=0; i<3; i++){
        LSM6_ReadMulti(0x22, buffer, 12, i+1); // IMU:一括読み出し
        imu[i].gx = ((int16_t)(buffer[1] << 8 | buffer[0]));
        imu[i].gy = ((int16_t)(buffer[3] << 8 | buffer[2]));
        imu[i].gz = ((int16_t)(buffer[5] << 8 | buffer[4]));
        imu[i].ax = ((int16_t)(buffer[7] << 8 | buffer[6]));
        imu[i].ay = ((int16_t)(buffer[9] << 8 | buffer[8]));
        imu[i].az = ((int16_t)(buffer[11] << 8 | buffer[10]));
      }

      gyro_x = (float)(-imu[1].gy + imu[0].gy*sin_30 - imu[0].gx*cos_30 + imu[2].gx*cos_30 + imu[2].gy*sin_30)*G_sensitivity/3.0f - gyro_x_bias;
      gyro_y = (float)( imu[1].gx - imu[0].gy*cos_30 - imu[0].gx*sin_30 - imu[2].gx*sin_30 + imu[2].gy*cos_30)*G_sensitivity/3.0f - gyro_y_bias;
      gyro_z = (float)( imu[0].gz + imu[1].gz + imu[2].gz )*G_sensitivity/3.0f - gyro_z_bias;
      gyro_z = gyro_z * GYRO_Z_SCALE;

      accel_x = (float)(-imu[1].ay + imu[0].ay*sin_30 - imu[0].ax*cos_30 + imu[2].ax*cos_30 + imu[2].ay*sin_30)*A_sensitivity/3.0f;
      accel_y = (float)( imu[1].ax - imu[0].ay*cos_30 - imu[0].ax*sin_30 - imu[2].ax*sin_30 + imu[2].ay*cos_30)*A_sensitivity/3.0f;
      accel_z = (float)( imu[0].az + imu[1].az + imu[2].az )*A_sensitivity/3.0f;

      if (fabs(gyro_x) < 0.5) gyro_x = 0.0;
      if (fabs(gyro_y) < 0.5) gyro_y = 0.0;
      if (fabs(gyro_z) < 0.5) gyro_z = 0.0;

      MadgwickAHRSupdateIMU(gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, dt);
      getEulerAngles(); //デバッグ用
      //compensateGravity(accel_x, accel_y, accel_z);//重力補正
    }

    /*if (imu_read_state == 4) {
      // ----------------------------------------------------
      // ① データのパース（rx_bufの[1]～[12]から復元）
      // ----------------------------------------------------
      imu[0].gx = ((int16_t)(imu1_rx_buf[2] << 8 | imu1_rx_buf[1]));
      imu[0].gy = ((int16_t)(imu1_rx_buf[4] << 8 | imu1_rx_buf[3]));
      imu[0].gz = ((int16_t)(imu1_rx_buf[6] << 8 | imu1_rx_buf[5]));
      imu[0].ax = ((int16_t)(imu1_rx_buf[8] << 8 | imu1_rx_buf[7]));
      imu[0].ay = ((int16_t)(imu1_rx_buf[10] << 8 | imu1_rx_buf[9]));
      imu[0].az = ((int16_t)(imu1_rx_buf[12] << 8 | imu1_rx_buf[11]));

      imu[1].gx = ((int16_t)(imu2_rx_buf[2] << 8 | imu2_rx_buf[1]));
      imu[1].gy = ((int16_t)(imu2_rx_buf[4] << 8 | imu2_rx_buf[3]));
      imu[1].gz = ((int16_t)(imu2_rx_buf[6] << 8 | imu2_rx_buf[5]));
      imu[1].ax = ((int16_t)(imu2_rx_buf[8] << 8 | imu2_rx_buf[7]));
      imu[1].ay = ((int16_t)(imu2_rx_buf[10] << 8 | imu2_rx_buf[9]));
      imu[1].az = ((int16_t)(imu2_rx_buf[12] << 8 | imu2_rx_buf[11]));

      imu[2].gx = ((int16_t)(imu3_rx_buf[2] << 8 | imu3_rx_buf[1]));
      imu[2].gy = ((int16_t)(imu3_rx_buf[4] << 8 | imu3_rx_buf[3]));
      imu[2].gz = ((int16_t)(imu3_rx_buf[6] << 8 | imu3_rx_buf[5]));
      imu[2].ax = ((int16_t)(imu3_rx_buf[8] << 8 | imu3_rx_buf[7]));
      imu[2].ay = ((int16_t)(imu3_rx_buf[10] << 8 | imu3_rx_buf[9]));
      imu[2].az = ((int16_t)(imu3_rx_buf[12] << 8 | imu3_rx_buf[11]));
      // ... (中略：IMU1〜3のジャイロと加速度を取り出す) ...

      // ----------------------------------------------------
      // ② 座標変換と合成
      // ----------------------------------------------------
      gyro_x = (float)(-imu[1].gy + imu[0].gy*sin_30 - imu[0].gx*cos_30 + imu[2].gx*cos_30 + imu[2].gy*sin_30)*G_sensitivity/3.0f - gyro_x_bias;
      gyro_y = (float)( imu[1].gx - imu[0].gy*cos_30 - imu[0].gx*sin_30 - imu[2].gx*sin_30 + imu[2].gy*cos_30)*G_sensitivity/3.0f - gyro_y_bias;
      gyro_z = (float)( imu[0].gz + imu[1].gz + imu[2].gz )*G_sensitivity/3.0f - gyro_z_bias;
      gyro_z = gyro_z * GYRO_Z_SCALE;

      accel_x = (float)(-imu[1].ay + imu[0].ay*sin_30 - imu[0].ax*cos_30 + imu[2].ax*cos_30 + imu[2].ay*sin_30)*A_sensitivity/3.0f;
      accel_y = (float)( imu[1].ax - imu[0].ay*cos_30 - imu[0].ax*sin_30 - imu[2].ax*sin_30 + imu[2].ay*cos_30)*A_sensitivity/3.0f;
      accel_z = (float)( imu[0].az + imu[1].az + imu[2].az )*A_sensitivity/3.0f;

      if (fabs(gyro_x) < 0.5) gyro_x = 0.0;
      if (fabs(gyro_y) < 0.5) gyro_y = 0.0;
      if (fabs(gyro_z) < 0.5) gyro_z = 0.0;

      // ----------------------------------------------------
      // ③ フィルタ演算（重い処理）
      // ----------------------------------------------------
      MadgwickAHRSupdateIMU(gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, dt);
      getEulerAngles();

      // ----------------------------------------------------
      // ④ 状態をリセットし、次の1msのタイマーを待つ
      // ----------------------------------------------------
      loop_count++;
      imu_read_state = 0; 
    }*/
    //whoami = LSM6_Read(0x0F,1);
    //whoami2 = LSM6_Read(0x0F,2);
    //whoami3 = LSM6_Read(0x0F,3);
    //printf("serial\n");
    //printf("1:0x%02X,2:0x%02X,3:0x%02X\r\n",whoami,whoami2,whoami3);
    if (loop_count == 100){
      loop_count = 0;
      printf("%.4f,%.4f,%.4f,%.4f\r\n", q[0], q[1], q[2], -q[3]);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 2;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 15;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 79;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IMU3_CS_Pin|IMU2_CS_Pin|IMU1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : IMU3_CS_Pin IMU2_CS_Pin IMU1_CS_Pin */
  GPIO_InitStruct.Pin = IMU3_CS_Pin|IMU2_CS_Pin|IMU1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void LSM6_Write(uint8_t reg, uint8_t data, int port)
{
    GPIO_TypeDef* PORT = IMU_CS_PORTS[port - 1];
    uint16_t PIN = IMU_CS_PINS[port - 1];
    uint8_t tx[2] = {reg & 0x7F, data};

    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_SET);
}

uint8_t LSM6_Read(uint8_t reg, int port) { //1つのIMUから単独で読み出す場合
    GPIO_TypeDef* PORT = IMU_CS_PORTS[port - 1];
    uint16_t PIN = IMU_CS_PINS[port - 1];
    uint8_t tx = reg | 0x80;  // Read（MSB=1）
    uint8_t rx = 0;

    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_RESET); // CS LOW
    HAL_SPI_Transmit(&hspi2, &tx, 1, HAL_MAX_DELAY); // アドレス送信
    HAL_SPI_Receive(&hspi2, &rx, 1, HAL_MAX_DELAY); // データ受信
    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_SET); // CS HIGH

    return rx;
}

void LSM6_ReadMulti(uint8_t reg, uint8_t* pData, uint16_t size, int port) {
    GPIO_TypeDef* PORT = IMU_CS_PORTS[port - 1];
    uint16_t PIN = IMU_CS_PINS[port - 1];
    uint8_t addr = reg | 0x80;

    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_RESET);
    
    // まずレジスタアドレスを送る
    HAL_SPI_Transmit(&hspi2, &addr, 1, HAL_MAX_DELAY);
    // その後、必要なサイズ分だけ受信する
    HAL_SPI_Receive(&hspi2, pData, size, HAL_MAX_DELAY);
    
    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_SET);
}

void INIT_IMU(int port){
  /*センサの初期化*/
  LSM6_Write(0x12, 0x44,port); // CTRL3: reboot,BDU有効化,アドレス自動インクリメント有効化
  /*ジャイロの初期化*/
  LSM6_Write(0x15, 0x04,port); // CTRL6: FS=±2000dps
  LSM6_Write(0x11, 0x06,port); // CTRL2: ODR=120Hz
  /*加速度の初期化*/
  LSM6_Write(0x17, 0x03,port); // CTRL8: FS=±16g
  LSM6_Write(0x10, 0x06,port); // CTRL1: ODR=120Hz
}

void resetBias(){
  // 起動時に1000回計測して平均をとる
  isSettingBias = 1;
  gyro_x_bias = 0.0f;
  gyro_y_bias = 0.0f;
  gyro_z_bias = 0.0f;
  for(int i=0; i<1000; i++) {
    uint8_t buffer6[12];
    for (int j=0; j<3; j++) {
      LSM6_ReadMulti(0x22, buffer6, 12, j+1); // IMU:一括読み出し
      imu[j].gx = ((int16_t)(buffer6[1] << 8 | buffer6[0]));
      imu[j].gy = ((int16_t)(buffer6[3] << 8 | buffer6[2]));
      imu[j].gz = ((int16_t)(buffer6[5] << 8 | buffer6[4]));
      imu[j].ax = ((int16_t)(buffer6[7] << 8 | buffer6[6]));
      imu[j].ay = ((int16_t)(buffer6[9] << 8 | buffer6[8]));
      imu[j].az = ((int16_t)(buffer6[11]<< 8 | buffer6[10]));
    }
    gyro_x_bias += (float)(-imu[1].gy + imu[0].gy*sin_30 - imu[0].gx*cos_30 + imu[2].gx*cos_30 + imu[2].gy*sin_30);
    gyro_y_bias += (float)( imu[1].gx - imu[0].gy*cos_30 - imu[0].gx*sin_30 - imu[2].gx*sin_30 + imu[2].gy*cos_30);
    gyro_z_bias += (float)( imu[0].gz + imu[1].gz + imu[2].gz );
    def_az += (float)(imu[0].az + imu[1].az +imu[2].az);

    HAL_Delay(1);
  }
  gyro_x_bias = gyro_x_bias *G_sensitivity * 0.001f / 3.0f;
  gyro_y_bias = gyro_y_bias *G_sensitivity * 0.001f / 3.0f;
  gyro_z_bias = gyro_z_bias *G_sensitivity * 0.001f / 3.0f;
  def_az = def_az * A_sensitivity *0.001f / 3.0f;
  accelCVR = 9.80665f/def_az;
  isSettingBias = 0;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI2) {
    switch (imu_read_state) {
      case 1: // IMU1の受信が完了した
        HAL_GPIO_WritePin(IMU1_CS_GPIO_Port, IMU1_CS_Pin, GPIO_PIN_SET); // IMU1終了
        
        imu_read_state = 2; // 次はIMU2
        HAL_GPIO_WritePin(IMU2_CS_GPIO_Port, IMU2_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive_DMA(&hspi2, spi_tx_buf, imu2_rx_buf, SPI_BUFFER_SIZE);
        break;

      case 2: // IMU2の受信が完了した
        HAL_GPIO_WritePin(IMU2_CS_GPIO_Port, IMU2_CS_Pin, GPIO_PIN_SET); // IMU2終了
        
        imu_read_state = 3; // 次はIMU3
        HAL_GPIO_WritePin(IMU3_CS_GPIO_Port, IMU3_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive_DMA(&hspi2, spi_tx_buf, imu3_rx_buf, SPI_BUFFER_SIZE);
        break;

      case 3: // IMU3の受信が完了した
        HAL_GPIO_WritePin(IMU3_CS_GPIO_Port, IMU3_CS_Pin, GPIO_PIN_SET); // IMU3終了
        
        // 全てのデータが揃ったので、メインループに計算を許可する
        imu_read_state = 4; 
        break;
          
      default:
        break;
    }
  }
}

float invSqrt(float x){
  return 1.0f / sqrtf(x);
}

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt){
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // 度/秒 を ラジアン/秒 に変換
  gx *= CONV;
  gy *= CONV;
  gz *= CONV;

  // ジャイロによるクォータニオンの変化率
  qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
  qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
  qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
  qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);

  // 加速度が有効な場合のみ補正計算を行う
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    // 勾配降下法の計算
    _2q0 = 2.0f * q[0]; _2q1 = 2.0f * q[1]; _2q2 = 2.0f * q[2]; _2q3 = 2.0f * q[3];
    _4q0 = 4.0f * q[0]; _4q1 = 4.0f * q[1]; _4q2 = 4.0f * q[2];
    _8q1 = 8.0f * q[1]; _8q2 = 8.0f * q[2];
    q0q0 = q[0] * q[0]; q1q1 = q[1] * q[1]; q2q2 = q[2] * q[2]; q3q3 = q[3] * q[3];

    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q[3] - _2q1 * ax + 4.0f * q2q2 * q[3] - _2q2 * ay;

    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

    qDot1 -= beta * s0; qDot2 -= beta * s1; qDot3 -= beta * s2; qDot4 -= beta * s3;
  }

  // 積分してクォータニオンを更新
  q[0] += qDot1 * dt; q[1] += qDot2 * dt; q[2] += qDot3 * dt; q[3] += qDot4 * dt;
  recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] *= recipNorm; q[1] *= recipNorm; q[2] *= recipNorm; q[3] *= recipNorm;
}

void getEulerAngles(){
  roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])) * 57.29578f;
  pitch = asinf(2.0f * (q[0] * q[2] - q[3] * q[1])) * 57.29578f;
  yaw   = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) * 57.29578f;
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
