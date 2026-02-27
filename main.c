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
#include <math.h>
#include <stdbool.h>
#include "icm_20948.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define MAHONY_KP  2.0f
//#define MAHONY_KI  0.005f
#define DEG_TO_RAD 0.01745329251f
#define RAD_TO_DEG 57.2957795f
#define PI 3.141592653589793f
//#define I_LIMIT    0.1f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
axises gyro;
axises accel;
axises mag;

float Ax, Ay, Az;
float Gx, Gy, Gz;
float Mx, My, Mz;

float mx_offset = 0.0f;
float my_offset = 0.0f;
float mz_offset = 0.0f;

float mx_scale = 1.0f;
float my_scale = 1.0f;
float mz_scale = 1.0f;

float roll;
float pitch;
float yaw;

double dt = 0.02;

//kalman parameters
double q_angle = 0.001; // noise accel
double q_bias = 0.003; // noise gyro bias
double r_measure = 0.03; // measurement noise
double angle = 0;
double bias = 0;
double rate = 0;
double p[2][2] = {{0,0},{0,0}}; // error covariance matrix

unsigned long lastTime;

//float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
//float mah_ix = 0.0f, mah_iy = 0.0f, mah_iz = 0.0f;

static CAN_TxHeaderTypeDef TxHeader;
static uint32_t TxMailbox;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

void read_accel(void);
void read_gyro(void);
void read_mag(void);

void calibrate_mag(void);

double kalman_filter(double angle, double gyro_rate, double accel_angle);

void calculate_angle(void);

void send_data(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void read_accel(void)
{
	icm20948_accel_read_g(&accel);

	Ax = accel.x;
	Ay = accel.y;
	Az = accel.z;
}

void read_gyro(void)
{
	icm20948_gyro_read_dps(&gyro);

	Gx = gyro.x;
	Gy = gyro.y;
	Gz = gyro.z;
}

void read_mag(void)
{
	ak09916_mag_read_uT(&mag);

	Mx = (mag.x - mx_offset) * mx_scale;
	My = (mag.y - my_offset) * my_scale;
	Mz = (mag.z - mz_offset) * mz_scale;
}

void calibrate_mag(void)
{
    float mx_min = 9999.0f, mx_max = -9999.0f;
    float my_min = 9999.0f, my_max = -9999.0f;
    float mz_min = 9999.0f, mz_max = -9999.0f;

    for (int i = 0; i < 500; i++) {

    	ak09916_mag_read_uT(&mag);
    	float Mx_raw = mag.x;
    	float My_raw = mag.y;
    	float Mz_raw = mag.z;

        if (Mx_raw < mx_min) mx_min = Mx_raw;
        if (Mx_raw > mx_max) mx_max = Mx_raw;
        if (My_raw < my_min) my_min = My_raw;
        if (My_raw > my_max) my_max = My_raw;
        if (Mz_raw < mz_min) mz_min = Mz_raw;
        if (Mz_raw > mz_max) mz_max = Mz_raw;

        HAL_Delay(10);

    }

    mx_offset = (mx_max + mx_min) / 2.0f;
    my_offset = (my_max + my_min) / 2.0f;
    mz_offset = (mz_max + mz_min) / 2.0f;

    float delta_x = mx_max - mx_min;
    float delta_y = my_max - my_min;
    float delta_z = mz_max - mz_min;

    if (delta_x < 0.1f) delta_x = 1.0f;
    if (delta_y < 0.1f) delta_y = 1.0f;
    if (delta_z < 0.1f) delta_z = 1.0f;

    float avg_delta = (delta_x + delta_y + delta_z)/3.0f;

    mx_scale = avg_delta / delta_x;
    my_scale = avg_delta / delta_y;
    mz_scale = avg_delta / delta_z;

}

double kalman_filter(double angle, double gyro_rate, double accel_angle)
{
	//predict
	rate = 	gyro_rate - bias;
	angle += dt * rate;

	p[0][0] += dt * (dt * p[1][1] - p[0][1] - p[1][0] + q_angle);
	p[0][1] -= dt * p[1][1];
	p[1][0] -= dt * p[1][1];
	p[1][1] += q_bias * dt;

	//update
	double s = p[0][0] + r_measure; //estimasi error
	double k[2];
	k[0] = p[0][0] / s;
	k[1] = p[1][0] / s;

	double y = accel_angle - angle; //angle difference
	angle += k[0] * y;
	bias += k[1] * y;

	double p00_temp = p[0][0];
	double p01_temp = p[0][1];

	p[0][0] -= k[0] * p00_temp;
	p[0][1] -= k[0] * p01_temp;
	p[1][0] -= k[1] * p00_temp;
	p[1][1] -= k[1] * p01_temp;

	return angle;
}

void calculate_angle(void)
{
	pitch = kalman_filter(pitch, Gx, atan2(-Ax,sqrt(Ay * Ay + Az * Az)) * RAD_TO_DEG);
	roll = kalman_filter(roll, Gy, atan2(Ay, Az) * RAD_TO_DEG);
	yaw = atan2(My, Mx);

	float declinationAngle = 0; // cari di web
	yaw += declinationAngle;

	//normalize yaw to 0-360 degree
	if (yaw < 0) yaw += 2 * PI;
	if (yaw > 2 * PI) yaw -= 2 * PI;

	yaw = yaw * RAD_TO_DEG;
}

void send_data(void)
{
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();

    if (now - last_tick < 10) {
        return;
    }

    last_tick = now;

    calculate_angle();

    int16_t ax_send = (int16_t)(Ax * 1000.0f);
    int16_t ay_send = (int16_t)(Ay * 1000.0f);
    int16_t az_send = (int16_t)(Az * 1000.0f);

    int16_t gx_send = (int16_t)(Gx * 10.0f);
    int16_t gy_send = (int16_t)(Gy * 10.0f);
    int16_t gz_send = (int16_t)(Gz * 10.0f);

    int16_t mx_send = (int16_t)(Mx * 10.0f);
    int16_t my_send = (int16_t)(My * 10.0f);
    int16_t mz_send = (int16_t)(Mz * 10.0f);

    int16_t roll_send = (int16_t)(roll * 100.0f);
    int16_t pitch_send = (int16_t)(pitch * 100.0f);
    int16_t yaw_send = (int16_t)(yaw * 100.0f);

    uint8_t accel_data[6];
    uint8_t gyro_data[6];
    uint8_t mag_data[6];
    uint8_t angle_data[6];

	 accel_data[0] = (uint8_t)(ax_send >> 8);
	 accel_data[1] = (uint8_t)(ax_send & 0xFF);
	 accel_data[2] = (uint8_t)(ay_send >> 8);
	 accel_data[3] = (uint8_t)(ay_send & 0xFF);
	 accel_data[4] = (uint8_t)(az_send >> 8);
	 accel_data[5] = (uint8_t)(az_send & 0xFF);

	 TxHeader.StdId = 1;
	 TxHeader.DLC = 6;
	 TxHeader.IDE = CAN_ID_STD;
     TxHeader.RTR = CAN_RTR_DATA;
     TxHeader.TransmitGlobalTime = DISABLE;

     if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan)>0){
    	 HAL_CAN_AddTxMessage(&hcan, &TxHeader, accel_data, &TxMailbox);
     }

	 gyro_data[0] = (uint8_t)(gx_send >> 8);
	 gyro_data[1] = (uint8_t)(gx_send & 0xFF);
	 gyro_data[2] = (uint8_t)(gy_send >> 8);
	 gyro_data[3] = (uint8_t)(gy_send & 0xFF);
	 gyro_data[4] = (uint8_t)(gz_send >> 8);
	 gyro_data[5] = (uint8_t)(gz_send & 0xFF);

	 TxHeader.StdId = 2;
	 TxHeader.DLC = 6;
	 TxHeader.IDE = CAN_ID_STD;
     TxHeader.RTR = CAN_RTR_DATA;
     TxHeader.TransmitGlobalTime = DISABLE;

     if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan)>0){
    	 HAL_CAN_AddTxMessage(&hcan, &TxHeader, gyro_data, &TxMailbox);
     }

	 mag_data[0] = (uint8_t)(mx_send >> 8);
	 mag_data[1] = (uint8_t)(mx_send & 0xFF);
	 mag_data[2] = (uint8_t)(my_send >> 8);
	 mag_data[3] = (uint8_t)(my_send & 0xFF);
	 mag_data[4] = (uint8_t)(mz_send >> 8);
	 mag_data[5] = (uint8_t)(mz_send & 0xFF);

	 TxHeader.StdId = 3;
	 TxHeader.DLC = 6;
	 TxHeader.IDE = CAN_ID_STD;
     TxHeader.RTR = CAN_RTR_DATA;
     TxHeader.TransmitGlobalTime = DISABLE;

     if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan)>0){
    	 HAL_CAN_AddTxMessage(&hcan, &TxHeader, mag_data, &TxMailbox);
     }

	 angle_data[0] = (uint8_t)(pitch_send >> 8);
	 angle_data[1] = (uint8_t)(pitch_send & 0xFF);
	 angle_data[2] = (uint8_t)(roll_send >> 8);
	 angle_data[3] = (uint8_t)(roll_send & 0xFF);
	 angle_data[4] = (uint8_t)(yaw_send >> 8);
	 angle_data[5] = (uint8_t)(yaw_send & 0xFF);

	 TxHeader.StdId = 4;
	 TxHeader.DLC = 6;
	 TxHeader.IDE = CAN_ID_STD;
     TxHeader.RTR = CAN_RTR_DATA;
     TxHeader.TransmitGlobalTime = DISABLE;

     if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan)>0){
    	 HAL_CAN_AddTxMessage(&hcan, &TxHeader, angle_data, &TxMailbox);
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
  MX_CAN_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  CAN_FilterTypeDef canFilter;
  canFilter.FilterBank = 0;
  canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilter.FilterIdHigh = 0x0000;
  canFilter.FilterIdLow = 0x0000;
  canFilter.FilterMaskIdHigh = 0x0000;
  canFilter.FilterMaskIdLow = 0x0000;
  canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
  canFilter.FilterActivation = ENABLE;

  HAL_CAN_ConfigFilter(&hcan, &canFilter);

  HAL_CAN_Start(&hcan);

  icm20948_init();
  ak09916_init();

  HAL_Delay(100);

  icm20948_gyro_full_scale_select(_250dps);

  icm20948_accel_full_scale_select(_2g);

  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
  HAL_Delay(100);

  icm20948_accel_calibration();

  icm20948_gyro_calibration();

  calibrate_mag();

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	send_data();
	HAL_Delay(1);


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
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin|OEPIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : INT1_Pin */
  GPIO_InitStruct.Pin = INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_Pin OEPIN_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|OEPIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
