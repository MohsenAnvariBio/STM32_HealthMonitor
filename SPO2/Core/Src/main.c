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
#include "../../lvgl/lvgl.h"
#include "../../lv_conf.h"
#include "tft.h"
#include "touchpad.h"
#include "pulse_oximeter.h"
#include "system.h"
#include "string.h"
#include <stdio.h>
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
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
volatile uint8_t pulseOximiterIntFlag = 0;
#define FILTER_LENGTH 5       // Length of the high-pass filter buffer
#define MOVING_AVG_LENGTH 2    // Length of the moving average buffer
#define ALPHA 0.95f // High-pass filter coefficient
#define M 10 // Size of the buffer
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void Display_SpO2_Update(uint32_t spo2);
void Display_SpO2_Init(void);
uint32_t calculate_spo2(uint32_t red_data, uint32_t ir_data);
float highPassFilter(float input, float *prevInput, float *prevOutput, float alpha);
void highPassFilterWithBuffer(float input, float *inputBuffer, float *outputBuffer, float *filteredSample, float alpha) ;
float mean(const float *buffer, uint16_t size);
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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  lv_init();
  tft_init();
  lv_disp_set_rotation(lv_disp_get_default(), LV_DISP_ROT_270);
  touchpad_init();
  //lv_example_label_1();
//  Display_SpO2_Init();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */


	lv_obj_t * label_S;
	lv_obj_t * label_t;
	lv_obj_t * chart;

	//Title
	label_t = lv_label_create(lv_scr_act());
	lv_label_set_long_mode(label_t, LV_LABEL_LONG_WRAP);     /*Break the long lines*/
	lv_label_set_recolor(label_t, true);                      /*Enable re-coloring by commands in the text*/
	lv_label_set_text(label_t, "#0000ff SPO2 Measurement#");
	lv_obj_align(label_t,LV_ALIGN_TOP_MID,0, 10);

	/*Create a chart*/

	chart = lv_chart_create(lv_scr_act());
	lv_obj_set_size(chart, 310, 200);
	lv_obj_align_to(chart, label_t, LV_ALIGN_TOP_MID,0,20);
	lv_obj_align(chart, LV_ALIGN_CENTER, 0, -25);
	lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_CIRCULAR);
	lv_chart_set_type(chart, LV_CHART_TYPE_LINE);   /*Show lines and points too*/
	lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -200, 200);
	uint16_t cnt = 1000;
	lv_chart_set_point_count(chart, cnt);
	lv_chart_series_t * ser2 = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_SECONDARY_Y);
	lv_chart_series_t * ser1 = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_SECONDARY_Y);


	// Create the label once during initialization
	label_S = lv_label_create(lv_scr_act());
	lv_label_set_long_mode(label_S, LV_LABEL_LONG_WRAP);
	lv_label_set_recolor(label_S, true);
	lv_obj_align_to(label_S, chart, LV_ALIGN_TOP_LEFT,0,100);

	/* Create a style for the label */
	static lv_style_t style_label_border;
	lv_style_init(&style_label_border);
	lv_style_set_border_width(&style_label_border, 2);               // Border width
	lv_style_set_border_color(&style_label_border, lv_color_black()); // Border color
	lv_style_set_border_opa(&style_label_border, LV_OPA_50);         // Border opacity
	lv_style_set_pad_all(&style_label_border, 5);                   // Add padding inside the border

	/* Apply the style to the label */
	lv_obj_add_style(label_S, &style_label_border, 0);





	FIFO_LED_DATA fifoLedData;
	pulseOximeter_resetRegisters();
	pulseOximeter_initFifo();
	pulseOximeter_setSampleRate(_800SPS);
	pulseOximeter_setPulseWidth(_411_US);
	pulseOximeter_setLedCurrent(RED_LED, 50);
	pulseOximeter_setLedCurrent(IR_LED, 5);
	pulseOximeter_resetFifo();
	// Set the Measurement Mode
	// Measurement Modes:
	// HEART_RATE - only Red Led active
	// SPO2 - Both IR & Red Led active
	// MULTI_LED - Both led's active (timing can be configured; see DataSheet)
	pulseOximeter_setMeasurementMode(SPO2);

	// High-pass filter state variables
	float prevInput = 0.0f, prevOutput = 0.0f;

	// Circular buffer for storing filtered values
	float buffer[M] = {0};
	int i = 0;
	int filled = 0; // Tracks if the buffer is fully filled

  while (1)
  {
//
	  if( PUSLE_OXIMETER_INTERRUPT == 1 )
	  {
		  if( pulseOximiterIntFlag )
		  {
			  pulseOximiterIntFlag = 0;
			  fifoLedData = pulseOximeter_readFifo();
			  float ppg_signal = (float)fifoLedData.irLedRaw;

			  float ppg_signal_rdc = highPassFilter(ppg_signal, &prevInput, &prevOutput, 0.95f);
			  lv_chart_set_next_value(chart, ser1, (-ppg_signal_rdc /(40)) +20);

			  // Fill or update the buffer
			  if (i < M) {
				  buffer[i] = ppg_signal_rdc; // Initial filling of the buffer
				  i++;
				  if (i == M) {
					  filled = 1; // Mark buffer as filled
				  }
			  } else if (filled) {
				  // Calculate mean of the buffer
				  float output = mean(buffer, M);
//			              printf("%.2f\n", k, output);
				  lv_chart_set_next_value(chart, ser2, (-output /(40)) +60);

				  // Shift buffer elements to the left
				  for (int j = 0; j < M - 1; j++) {
					  buffer[j] = buffer[j + 1];
				  }

				  // Add the new sample to the buffer
				  buffer[M - 1] = ppg_signal_rdc;
			  }

			  pulseOximeter_clearInterrupt();
		  }

	  }
//	  else{
////
//		  // Read FIFO LED Data
//		  fifoLedData = pulseOximeter_readFifo();
//
//		  // Get BPM/SpO2 readings
//		  pulseOximeter = pulseOximeter_update(fifoLedData);
//
//		  pulseOximeter_clearInterrupt();
//		  pulseOximeter_resetFifo();
//
//		  // Small delay
//		  HAL_Delay(1);
//	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_Delay(1);
	  lv_timer_handler();
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

//float highPassFilter(float input, float *prevInput, float *prevOutput, float alpha) {
//    // Apply the high-pass filter formula
//    float output = alpha * (*prevOutput + input - *prevInput);
//    *prevInput = input;     // Update previous input
//    *prevOutput = output;   // Update previous output
//    return output;
//}
// High-pass filter function
float highPassFilter(float input, float *prevInput, float *prevOutput, float alpha) {
    float inputF = input; // No need to cast, input is already float
    float output = alpha * (*prevOutput + inputF - *prevInput);
    *prevInput = inputF;
    *prevOutput = output;
    return output;
}

// Function to calculate the mean of an array
float mean(const float *buffer, uint16_t size) {
    float sum = 0.0f;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return sum / size;
}


void highPassFilterWithBuffer(float input, float *inputBuffer, float *outputBuffer, float *filteredSample, float alpha) {
    static int index = 0; // Circular buffer index

    // Add new input to the buffer (circular behavior)
    inputBuffer[index] = input;

    // Compute the high-pass filter output
    int prevIndex = (index - 1 + FILTER_LENGTH) % FILTER_LENGTH; // Previous index in circular buffer
    outputBuffer[index] = alpha * (outputBuffer[prevIndex] + inputBuffer[index] - inputBuffer[prevIndex]);

    // Save the current filtered sample for visualization
    *filteredSample = outputBuffer[index];

    // Advance the circular buffer index
    index = (index + 1) % FILTER_LENGTH;
}
void highPassFilterWithBufferAndMovingAvg(float input, float *inputBuffer, float *outputBuffer,
                                          float *filteredSample, float alpha,
                                          float *movingAvgBuffer, float *smoothedSample) {
    static int index = 0;                // Circular buffer index for high-pass filter
    static int movingAvgIndex = 0;       // Circular buffer index for moving average
    static float movingAvgSum = 0.0f;    // Sum of moving average buffer for efficiency

    // Add new input to the high-pass filter input buffer (circular behavior)
    inputBuffer[index] = input;

    // Compute the high-pass filter output
    int prevIndex = (index - 1 + FILTER_LENGTH) % FILTER_LENGTH; // Previous index in circular buffer
    outputBuffer[index] = alpha * (outputBuffer[prevIndex] + inputBuffer[index] - inputBuffer[prevIndex]);

//    printf("High-Pass Filter Output: %f\n", outputBuffer[index]);


    // Store the current high-pass filtered sample
    *filteredSample = outputBuffer[index];

    // Add the filtered output to the moving average buffer
    movingAvgSum -= movingAvgBuffer[movingAvgIndex]; // Remove the oldest value from the sum
    movingAvgBuffer[movingAvgIndex] = outputBuffer[index]; // Add the new value to the buffer
    movingAvgSum += movingAvgBuffer[movingAvgIndex]; // Update the sum with the new value

    // Compute the moving average
    *smoothedSample = movingAvgSum / MOVING_AVG_LENGTH;

    // Update the indices for both buffers
    index = (index + 1) % FILTER_LENGTH;
    movingAvgIndex = (movingAvgIndex + 1) % MOVING_AVG_LENGTH;
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
