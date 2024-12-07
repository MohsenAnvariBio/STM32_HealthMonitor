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
#include "system.h"
#include <string.h>
#include <stdio.h>
#include "../../lvgl/lvgl.h"
#include "../../lv_conf.h"
#include "tft.h"
#include "touchpad.h"
#include "pulse_oximeter.h"
#include "ui.h"
#include "start.h"
#include "signal_processing.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOVING_AVG_L 12       // Size of the moving average buffer
#define DATA_LENGTH  1000     // Length of data buffer
#define INVALID_VALUE 0xFFFFFFFF // Sentinel value for invalid SpO2 data
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
volatile uint8_t pulseOximiterIntFlag = 0;
extern uint8_t startFinish;
float prevInput_ir = 0.0f, prevOutput_ir = 0.0f;
float prevInput_red = 0.0f, prevOutput_red = 0.0f;
float buffer_ir[MOVING_AVG_L] = {0};
float buffer_red[MOVING_AVG_L] = {0};
float bufferPeakDet_ir[DATA_LENGTH] = {0};
float bufferPeakDet_red[DATA_LENGTH] = {0};
uint32_t R[DATA_LENGTH] = {0};
float SpO2 = 0, ratio = 0;
uint32_t buffHR = 0;
int i = 0, j = 0, filled = 0, filled2 = 0;
uint32_t R_count = 0;
const float alpha = 0.8f; // Smoothing factor (0 < alpha <= 1)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
static void processPulseOximeterData(void);
static void handleHighPassFilter(float irRaw, float redRaw);
static void processMovingAverage(float irSignal, float redSignal);
static void handlePeakDetection(void);
static void resetBuffers(void);
static void shiftBuffers(void);
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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  lv_init();
  tft_init();
  lv_disp_set_rotation(lv_disp_get_default(), LV_DISP_ROT_270);
  touchpad_init();
  create_splash_screen();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  FIFO_LED_DATA fifoLedData;
  pulseOximeter_resetRegisters();
  pulseOximeter_initFifo();
  pulseOximeter_setSampleRate(_800SPS);
  pulseOximeter_setPulseWidth(_411_US);
  pulseOximeter_setLedCurrent(RED_LED, 50);
  pulseOximeter_setLedCurrent(IR_LED, 5);
  pulseOximeter_resetFifo();
  pulseOximeter_setMeasurementMode(SPO2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if (PUSLE_OXIMETER_INTERRUPT && startFinish)
	{
		if (pulseOximiterIntFlag)
		{
			pulseOximiterIntFlag = 0;
			processPulseOximeterData();
		}
	}

    lv_timer_handler();
    HAL_Delay(1);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_9)
	{
		pulseOximiterIntFlag = 1;
	}
}


/* Function definitions */

/**
 * @brief Process pulse oximeter data by reading FIFO and applying filtering.
 */
static void processPulseOximeterData(void)
{
    FIFO_LED_DATA fifoLedData = pulseOximeter_readFifo();

    float irRaw = (float)fifoLedData.irLedRaw;
    float redRaw = (float)fifoLedData.redLedRaw;

    handleHighPassFilter(irRaw, redRaw);
}

/**
 * @brief Apply high-pass filtering on raw signals and proceed with further processing.
 * @param irRaw Raw IR signal.
 * @param redRaw Raw red signal.
 */
static void handleHighPassFilter(float irRaw, float redRaw)
{
    float irFiltered = highPassFilter(irRaw, &prevInput_ir, &prevOutput_ir, 0.95f);
    float redFiltered = highPassFilter(redRaw, &prevInput_red, &prevOutput_red, 0.95f);

    if (is_moving_average_enabled())
    {
        processMovingAverage(irFiltered, redFiltered);
    }
    else
    {
        update_chart_with_gain(irFiltered);
    }

    pulseOximeter_clearInterrupt();
}

/**
 * @brief Process signals with moving average and manage peak detection.
 * @param irSignal Filtered IR signal.
 * @param redSignal Filtered red signal.
 */
static void processMovingAverage(float irSignal, float redSignal)
{
    if (i < MOVING_AVG_L)
    {
        buffer_ir[i] = irSignal;
        buffer_red[i] = redSignal;
        i++;

        if (i == MOVING_AVG_L)
        {
            filled = 1;
        }
    }
    else if (filled)
    {
        float ma_ir = mean(buffer_ir, MOVING_AVG_L);
        float ma_red = mean(buffer_red, MOVING_AVG_L);
        update_chart_with_gain(ma_ir);

        if (j < DATA_LENGTH)
        {
            bufferPeakDet_ir[j] = -ma_ir / 40;
            bufferPeakDet_red[j] = -ma_red / 40;
            j++;

            if (j == DATA_LENGTH)
            {
                filled2 = 1;
            }
        }
        else if (filled2)
        {
            handlePeakDetection();
            resetBuffers();
        }

        shiftBuffers();
        buffer_ir[MOVING_AVG_L - 1] = irSignal;
        buffer_red[MOVING_AVG_L - 1] = redSignal;
    }
}

/**
 * @brief Handle peak detection and calculate SpO2 and heart rate.
 */
static void handlePeakDetection(void)
{
    if (isFingerDetected(bufferPeakDet_ir, DATA_LENGTH))
    {
        update_heartimg(1);
        update_temp(pulseOximeter_readTemperature());

        findPeaks(bufferPeakDet_ir, DATA_LENGTH, R, &R_count);
        calculate_SpO2(bufferPeakDet_red, bufferPeakDet_ir, DATA_LENGTH, &SpO2, &ratio);
        SpO2 = (SpO2 > 100.0f) ? 100.0f : (SpO2 < 0.0f ? 0.0f : SpO2);
        update_SPO2((uint32_t)SpO2);

        buffHR = (buffHR == 0)
                     ? heartRate(R, R_count)
                     : alpha * heartRate(R, R_count) + (1.0f - alpha) * buffHR;
        update_HR(buffHR);
    }
    else
    {
        update_heartimg(0);
        update_SPO2(INVALID_VALUE);
        update_HR(INVALID_VALUE);
        update_temp(INVALID_VALUE);
    }
}

/**
 * @brief Reset buffers and counters for the next cycle of data processing.
 */
static void resetBuffers(void)
{
    filled2 = 0;
    j = 0;
    R_count = 0;
    SpO2 = 0.0f;

    memset(bufferPeakDet_ir, 0, sizeof(bufferPeakDet_ir));
    memset(bufferPeakDet_red, 0, sizeof(bufferPeakDet_red));
    memset(R, 0, sizeof(R));
}

/**
 * @brief Shift the elements of the moving average buffers to make room for new data.
 */
static void shiftBuffers(void)
{
    for (int k = 0; k < MOVING_AVG_L - 1; k++)
    {
        buffer_ir[k] = buffer_ir[k + 1];
        buffer_red[k] = buffer_red[k + 1];
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
