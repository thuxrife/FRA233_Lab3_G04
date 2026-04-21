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
#include "dma.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Ultrasonic.h"
#include "kalman_filter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#pragma pack(push, 1)
typedef struct {
	uint16_t header;       // 0xABCD (2 bytes)
	float raw_pos;         // Measured z_k (4 bytes)
	float kf_zero_pos;     // Zero-order x[0] (4 bytes)
	float kf_first_pos;    // First-order x[0] (4 bytes)
	float kf_first_vel;    // First-order x[1] (4 bytes)
	float kf_msd_pos;      // MSD x[0] (4 bytes)
	float kf_msd_vel;      // MSD x[1] (4 bytes)
	uint8_t footer;        // 0x7F (1 byte)
} TelemetryFrame_t;        // Total: 27 bytes
#pragma pack(pop)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Ultrasonic_HandleTypeDef hus1;
Kalman_HandleTypeDef hkf_zero, hkf_first, hkf_msd;

const float M_KG = 0.339f;
//const float M_KG = 1e-9f;
const float K_NM = 29.32750224f;
const float C_NSM = 0.00002f;
//const float F_GRAVITY = 0.0f;
//const float F_GRAVITY = 0.5f * 9.80665f; // Newtons
const float F_GRAVITY = 0.339f * 9.80665f; // Newtons

TelemetryFrame_t txFrame = { .header = 0xABCD, .footer = 0x7F };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_DMA_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_LPUART1_UART_Init();
	/* USER CODE BEGIN 2 */
	// 1. Initialize Library Handles
	Ultrasonic_Init(&hus1, &htim3, TIM_CHANNEL_1, &htim1, TIM_CHANNEL_1);
	Ultrasonic_StartAutonomous(&hus1);

	// 2. Initialize all 3 Kalman Filter Models
	// 2. Initialize all 3 Kalman Filter Models (Unit: Meters)
	// In main.c, around line 121
	float p_start = 0.1134f; // Matches calculated static deflection (Mg/K)
	float dt = 0.001f;     // 1000Hz sampling

	// In main.c, around lines 124-126
	float R = 9e-6f; // Standard for 3mm sensor resolution
	float Q_p = 1e-6f; // Increase 100x to follow sensor faster
	float Q_v = 1e-5f; // Increase 100x to allow velocity changes

	// Adjusted R to 9e-6f to match 0.003m sensor resolution
	Kalman_InitZeroOrder(&hkf_zero, p_start, Q_p, R);
	Kalman_InitFirstOrder(&hkf_first, p_start, dt, Q_p, Q_v, R);
	Kalman_InitMSD(&hkf_msd, p_start, dt, M_KG, C_NSM, K_NM, Q_p, Q_v, R);

	// 3. Start 1000Hz Timer Interrupt (Crucial)
	if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM4) {
		// 1. SI Unit Conversion (mm to m)
		float z = hus1.distance_mm * 0.001f;

		// 2. State Propagation and Update
		// Note: Kalman_Update returns x[0], but internal state x[1] is also updated
		float z_zero = Kalman_Update(&hkf_zero, 0.0f, z);
		float z_first = Kalman_Update(&hkf_first, 0.0f, z);
		float z_msd = Kalman_Update(&hkf_msd, F_GRAVITY, z);

		// 3. Mapping Variables to Telemetry Frame
		txFrame.raw_pos = z;
		txFrame.kf_zero_pos = z_zero;
		txFrame.kf_first_pos = z_first;
		txFrame.kf_first_vel = hkf_first.x[1]; // Velocity State (m/s)
		txFrame.kf_msd_pos = z_msd;
		txFrame.kf_msd_vel = hkf_msd.x[1];   // Velocity State (m/s)

		// 4. Non-Blocking Transmission
		HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*) &txFrame, sizeof(txFrame));
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	// Delegate Input Capture to the library handler [cite: 1006]
	if (htim->Instance == TIM1) {
		Ultrasonic_CaptureCallback(&hus1, htim);
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
