/*
 *  FILE          : main.c
 *  PROJECT       : Assignment3
 *  PROGRAMMER    : Manu Murali Student
 *  FIRST VERSION : 2017-07-06
 *  DESCRIPTION   :
 *    Temperature sensor MCP7000A is used to measure the temperature and
 *    display the temperature with one decimal point in the top right corner
 *    of the LCD. It also illuminate RGB LED with the specified color for
 *    the given temperature ranges.
 *    -15 C  to 5.0 C   blue
 *    5.1 C to 15.0 C   yellow
 *    15.1 C to 25 C    orange
 *    Above 21.5 C 	   red
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "time.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HD44780_Init(void);
void HD44780_PutChar(unsigned char c);
void HD44780_PutStr(char *str);
void HD44780_GotoXY(unsigned char x, unsigned char y);
void HD44780_ClrScr(void);
void HD44780_GotoXY(unsigned char x, unsigned char y);
void redLedON();
void yellowLedON();
void orangeLedON();
void blueLedON();
void displayTemperatureInLCD(float);
void startTimer();
float measureTemperature();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

//
// FUNCTION      : redLedON()
// DESCRIPTION   :
//	This function will illuminate red LED
// PARAMETERS    :
//	Nothing
// RETURNS       :
//	Nothing
//
void redLedON()
{
	// blue led OFF
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
	// red led ON
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 60000);
	// green led OFF
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 000);
}

//
// FUNCTION      : orangeLedON()
// DESCRIPTION   :
//	This function will illuminate orange LED
// PARAMETERS    :
//	Nothing
// RETURNS       :
//	Nothing
//
void orangeLedON()
{
	// blue led OFF
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
	// adjusting voltage of red led
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 50000);
	// adjusting voltage of green led
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2500);
}

//
// FUNCTION      : yellowLedON()
// DESCRIPTION   :
//	This function will illuminate yellow LED
// PARAMETERS    :
//	Nothing
// RETURNS       :
//	Nothing
//
void yellowLedON()
{
	// blue led OFF
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
	// adjusting voltage of red led
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 30000);
	// adjusting voltage of green led
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 6000);
}

//
// FUNCTION      : blueLedON()
// DESCRIPTION   :
//	This function will illuminate blue LED
// PARAMETERS    :
//	Nothing
// RETURNS       :
//	Nothing
//
void blueLedON()
{
	// red led OFF
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 000);
	// green led OFF
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 000);
	// blue led ON
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
}

//
// FUNCTION      : displayTemperatureInLCD()
// DESCRIPTION   :
//	This function will display the temperature in the top right corner of LCD
// PARAMETERS    :
//	float		 : Temperature measured using the sensor
// RETURNS       :
//	Nothing
//
void displayTemperatureInLCD(float temperature)
{
	// initializing a default value for stringToDisplay
	char stringToDisplay[16] = "0";
	// converting temperature to string format
	sprintf(stringToDisplay, "% 2.1f\337C", temperature);
	// StringSize: store the size of the string to be displayed on LCD
	// default value chosen as 0
	uint8_t StringSize = 0;
	// calculating the size of the string
	while(stringToDisplay[StringSize] != '\0')
	{
		StringSize++;
	}
	// bringing the cursor to the top right corner of the LCD
	HD44780_GotoXY(16-StringSize, 0);
	// display the temperature in top right corner of LCD
	HD44780_PutStr(stringToDisplay);

}

//
// FUNCTION      : startTimer()
// DESCRIPTION   :
//	This function will start the timer1 channel-4 and channel-1
// PARAMETERS    :
//	Nothing
// RETURNS       :
//	Nothing
//
void startTimer()
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

//
// FUNCTION      : measureTemperature()
// DESCRIPTION   :
//	This function will measure the temperature using the value received
//  from ADC
// PARAMETERS    :
//	Nothing
// RETURNS       :
//	float		 : temperature, the measured temperature
//
float measureTemperature()
{
	// variable declaration and initialization
	uint16_t ADCValue = 0;
	float VoltageValue = 0.0;
	float temperature = 0.0;
	// read value of ADC which corresponds to the temperature sensor output
	ADCValue = HAL_ADC_GetValue(&hadc1);
	// computes the voltage value corresponding to the received ADC value
	VoltageValue = (ADCValue *3.3) / 4095;
	// computes the temperature using voltage
	temperature = (VoltageValue - 0.5) /0.010;
	return temperature;
}
/* USER CODE END 0 */

int main(void)
{
	/* USER CODE BEGIN 1 */
	// default value chosen as 0.0
	float temperature = 0.0;

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_USART2_UART_Init();
	MX_TIM1_Init();

	/* USER CODE BEGIN 2 */
	HD44780_Init();
	startTimer();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// calibrating ADC
		HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);

		if(HAL_ADC_Start(&hadc1) != HAL_OK)
		{
			printf("\r\nError in ADC Start \r\n");
		}
		if(HAL_ADC_PollForConversion(&hadc1, 10))
		{
			printf("\r\nError in ADC Timeout \r\n");
		}
		else
		{
			// read the ADC value and measures the temperature
			temperature = measureTemperature();
		}
		// display the measured temperature value in LCD
		displayTemperatureInLCD(temperature);
		// checking the range at which the measured temperature lies and
		// illuminate the required color in LED based on the temperature
		// measured using the sensor
		if(temperature > 25.0)
		{
			redLedON();
		}
		else if(temperature > 15.0)
		{
			orangeLedON();
		}
		else if(temperature > 5.0)
		{
			yellowLedON();
		}
		else
		{
			blueLedON();
		}
	}
	/* USER CODE END 3 */
}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 16;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
	PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
	PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
	PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
	PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}

	/**Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

	ADC_ChannelConfTypeDef sConfig;

	/**Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.NbrOfDiscConversion = 1;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 60000;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim1);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_BLUE_Pin */
	GPIO_InitStruct.Pin = LED_BLUE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_BLUE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD3_Pin */
	GPIO_InitStruct.Pin = LD3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
	/* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
