/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *  This component has been extended to form a firmware for the STM32 series
 *  microcontrollers that converts serial MIDI to Hybrid Music 4000 signals.
 *  The completed firmware, and new code, is released under GPL 3.0.
 *
 *  USER CODE segments
 *  Copyright (C) 2019  Daniel Jameson
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
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

#define MIDI_MASK 0b11110000
#define MIDI_ON 0b10010000
#define MIDI_OFF 0b10000000
#define MIDI_CONT 0b10110000
#define MIDI_PRES 0b11010000
#define MIDI_PROG 0b11000000
#define MIDI_IGNORE 255
#define MIDI_PED 64



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

uint8_t keyboardMatrix[8];
uint8_t midiByte=0;
uint8_t midiData[2];
uint8_t m4kZone;
uint8_t m4kNote;
uint8_t rawZone;
uint8_t midiStatus;

HAL_StatusTypeDef receiveStatus;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

inline void Note_On(uint8_t);
inline void Note_Off(uint8_t);

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

	/* all keys high to start with */
	for (uint8_t i=0; i<8; i++) {
		keyboardMatrix[i]=255;
	}

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
	MX_SPI3_Init();
	MX_UART4_Init();
	/* USER CODE BEGIN 2 */
	/* enable SPI */
	__HAL_SPI_ENABLE(&hspi3);

	/* set initial midi-status to ignore */
	midiStatus=MIDI_IGNORE;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */


	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/**
		 * Zones 1-7 have notes, zone 8 has the control pedal
		 * middle C is 1st note of zone 4 (int 127, midi 60)
		 */

		/* get a byte from the UART */
		receiveStatus = HAL_UART_Receive(&huart4, &midiByte, 1, 100);

		if (receiveStatus == HAL_OK) {
			/* is it a control byte? */
			if (midiByte>127) {
				/* Set a new status */
				switch(midiByte & MIDI_MASK) {
				case MIDI_ON :
					midiStatus = MIDI_ON;
					break;
				case MIDI_OFF :
					midiStatus = MIDI_OFF;
					break;
				case MIDI_CONT :
					midiStatus = MIDI_CONT;
					break;
					/* These two controls only have one data byte following */
				case (MIDI_PROG || MIDI_PRES) :
										midiStatus = MIDI_PROG;
				break;
				/* If it's something else */
				default:
					midiStatus = MIDI_IGNORE;
				}

				switch (midiStatus) {
				case MIDI_PROG :
					HAL_UART_Receive(&huart4, midiData, 1, 100);
					midiData[1]=0;
					break;
				case MIDI_IGNORE :
					HAL_UART_Receive(&huart4, midiData, 2, 100);
					break;
				case MIDI_ON :
					HAL_UART_Receive(&huart4, midiData, 2, 100);
					if (midiData[1]!=0) {
						Note_On(midiData[0]);
					} else {
						Note_Off(midiData[0]);
					}
					break;
				case MIDI_OFF :
					HAL_UART_Receive(&huart4, midiData, 2, 100);
					Note_Off(midiData[0]);
					break;
				case MIDI_CONT :
					HAL_UART_Receive(&huart4, midiData, 2, 100);
					if (midiData[0]==MIDI_PED) {
						if (midiData[0]<64) {
							/**
							 * Turn it on by NOT ANDing
							 */
							keyboardMatrix[0] &= ~(1 << 6);
						} else {
							/**
							 * Turn it off by ORing
							 */
							keyboardMatrix[0] |= (1 << 6);
						}
					}
					break;

				}
			} else {
				/* We're in a run-on condition, we have a byte already */
				switch (midiStatus) {
				case MIDI_PROG :
					break;
				case MIDI_IGNORE :
					HAL_UART_Receive(&huart4, &midiByte, 1, 100);
					break;
				case MIDI_ON :
					midiData[0]=midiByte;
					HAL_UART_Receive(&huart4, &midiByte, 1, 100);
					if (midiByte!=0) {
						Note_On(midiData[0]);
					} else {
						Note_Off(midiData[0]);
					}
					break;
				case MIDI_OFF :
					Note_Off(midiByte);
					HAL_UART_Receive(&huart4, &midiByte, 1, 100);
					break;
				case MIDI_CONT :
					if (midiByte==MIDI_PED) {
						if (midiByte<64) {
							/**
							 * Turn it on by NOT ANDing
							 */
							keyboardMatrix[0] &= ~(1 << 6);
						} else {
							/**
							 * Turn it off by ORing
							 */
							keyboardMatrix[0] |= (1 << 6);
						}
					}
					HAL_UART_Receive(&huart4, &midiByte, 1, 100);
					break;
				}
			}

		}

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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
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
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void)
{

	/* USER CODE BEGIN SPI3_Init 0 */

	/* USER CODE END SPI3_Init 0 */

	/* USER CODE BEGIN SPI3_Init 1 */

	/* USER CODE END SPI3_Init 1 */
	/* SPI3 parameter configuration*/
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_SLAVE;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
	hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI3_Init 2 */

	/* Set interrupt on buffer not empty */
	hspi3.Instance->CR2 |= SPI_CR2_RXNEIE;

	/* USER CODE END SPI3_Init 2 */

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
	huart4.Init.BaudRate = 31250;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_RX;
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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, PB0_Pin|PB1_Pin|PB2_Pin|PB3_Pin
			|PB4_Pin|PB5_Pin|PB6_Pin|PB7_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA2 PA3 */
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0_Pin PB1_Pin PB2_Pin PB3_Pin
                           PB4_Pin PB5_Pin PB6_Pin PB7_Pin */
	GPIO_InitStruct.Pin = PB0_Pin|PB1_Pin|PB2_Pin|PB3_Pin
			|PB4_Pin|PB5_Pin|PB6_Pin|PB7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* We're only allowing midi notes between 36 and 84 otherwise
 * we'll overshoot the keyboard matrix array...
 */

/**
 * @brief Set Keyboard Key
 * @param uint_8_t MIDInote
 * @retval None
 */

inline void Note_On(uint8_t note)
{
	if ((note>35) && (note<85)) {
		rawZone=((note-36)/8);
		m4kZone=7-rawZone;
		m4kNote=note-36-(8*rawZone);
		keyboardMatrix[m4kZone] &= ~(1 << m4kNote);
	}
}

/**
 * @brief Set Keyboard Key
 * @param uint_8_t MIDInote
 * @retval None
 */

inline void Note_Off(uint8_t note)
{
	if ((note>35) && (note<85)) {
		rawZone=((note-36)/8);
		m4kZone=7-rawZone;
		m4kNote=note-36-(8*rawZone);
		keyboardMatrix[m4kZone] |= (1 << m4kNote);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
