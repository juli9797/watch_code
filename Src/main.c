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
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

struct LED_CONFIG {
	int led_count;
	int brightness_steps;
} led_config = { .led_count = LED_COUNT, .brightness_steps = 10 };

int counter = 0;

//LEDs and shit
uint8_t brightness[LED_COUNT];
uint8_t brightnessB[LED_COUNT];

uint8_t * ptr_buffer = &brightness[0];
uint8_t * ptr_draw = &brightnessB[0];

uint8_t brt_values[6] = { 0b00000, 0b00100, 0b01010, 0b10101, 0b11011, 0b11111 };

uint8_t blink[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t blink_stat[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

static uint8_t offsetLED_mapping[4] = { 14, 15, 13, 12 };
//INPUT
int button_state = 0; // 0 = none, 1 = short 2 = long
uint32_t button_time = 0;
const uint32_t button_time_thr = 700;
//Timekeeping
uint8_t hours = 0;
uint8_t minutes = 7;
uint8_t seconds = 0;
int en_timekeeping = 1;
//Timeout
int timeout_state = 0;
static uint32_t timeout[3] = { 0, 10000, 30000 };
int timeout_ref = 0;

//State machine
enum {
	time, menue, program, timeset
};
int state = time;

//Animation
int animation_counter;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
//LEDs
static void flip_Framebuffer();

static void set_Time(uint32_t);

static void animation();
static void clear_wakeup();
static void reset_timeout();
static void second_Expired();
static void standby();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t get_Minute_led() {
	return minutes / 5;
}
uint8_t get_Minute_Offset_led() {
	int val = minutes % 5;
	if (val == 0) {
		return 17;
	}
	return offsetLED_mapping[val - 1];
}
uint8_t get_Hour_led() {
	return hours;
}

void clear_blink() {
	for (int i = 0; i < LED_COUNT; i++) {
		ptr_buffer[i] = 0;
		blink[i] = 0;
		blink_stat[i] = 0;
	}
}

void animation() {
	animation_counter++;
	if (animation_counter >= 1000) {
		animation_counter = 0;
	}
	//blink
	for (int i = 0; i < LED_COUNT; i++) {
		if (blink[i] != 0) {
			if (animation_counter % blink[i] == 0) {
				blink_stat[i] = !blink_stat[i];

			}

			ptr_buffer[i] = blink_stat[i] * 5;
		}
	}

}

void tick() {
	int ticks = HAL_GetTick();
	//Gets called by SysTick_Handler
	if (button_time != 0 && ticks - button_time >= button_time_thr) {
		// Thats a long button press my friends
		button_state = 2;
		button_time = 0;
	}

	//Timeout
	if (timeout[timeout_state] != 0
			&& ticks - timeout_ref > timeout[timeout_state]) {
		standby();
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	reset_timeout();
	if (HAL_GPIO_ReadPin(SWITCH_GPIO_Port, SWITCH_Pin)) {
		//Button pressed
		button_state = 0;
		button_time = HAL_GetTick();

	} else {
		//Button released
		if (HAL_GetTick() - button_time < button_time_thr && button_time != 0) {

			button_state = 1;

		}
		button_time = 0;

	}

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		//Deactivate Matrix
		HAL_GPIO_WritePin(K0_GPIO_Port, K0_Pin, 0);
		HAL_GPIO_WritePin(K1_GPIO_Port, K1_Pin, 0);
		HAL_GPIO_WritePin(EN_A_GPIO_Port, EN_A_Pin, 1);

		//Decide which LED to illuminate
		// 00100 01010 10101 11011 11111
		int led = counter % LED_COUNT;
		int brt_step = (counter / LED_COUNT);
		int curr_brt = ptr_draw[led];
		int en = (brt_values[curr_brt] >> brt_step) & 0x1;
		//Control matrix
		if (en) {

			int temp = led % 8;
			int A1 = temp & 0b1;
			int A2 = (temp & 0b10) >> 1;
			int A3 = (temp & 0b100) >> 2;
			int K0 = 1 - led / 8;
			int K1 = led / 8;

			HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, A1);
			HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, A2);
			HAL_GPIO_WritePin(A3_GPIO_Port, A3_Pin, A3);
			HAL_GPIO_WritePin(K0_GPIO_Port, K0_Pin, K0);
			HAL_GPIO_WritePin(K1_GPIO_Port, K1_Pin, K1);
			HAL_GPIO_WritePin(EN_A_GPIO_Port, EN_A_Pin, 0);

		} else {
			/*HAL_GPIO_WritePin(EN_A_GPIO_Port, EN_A_Pin, 1);
			 HAL_GPIO_WritePin(K0_GPIO_Port, K0_Pin, 0);
			 HAL_GPIO_WritePin(K1_GPIO_Port, K1_Pin, 0);*/
		}

		//counter
		counter++;
		if (counter >= 16 * 6) {
			counter = 0;
		}

	} else if (htim->Instance == TIM2) {
		//animation();
		second_Expired();
	}
}
static void flip_Framebuffer() {
	//DO SOME SWAP MAGIC (TM)
	uint8_t * ptr_temp = ptr_buffer;
	ptr_buffer = ptr_draw;
	ptr_draw = ptr_temp;
	/*
	 ptr_draw = (void*) ((uintptr_t) ptr_draw ^ (uintptr_t) ptr_buffer);
	 ptr_buffer = (void*) ((uintptr_t) ptr_draw ^ (uintptr_t) ptr_buffer);
	 ptr_draw = (void*) ((uintptr_t) ptr_draw ^ (uintptr_t) ptr_buffer);
	 */

}
static void draw_time(int blink_min, int blink_h, int blink_ofst) {
	uint8_t min = get_Minute_led();
	uint8_t h = get_Hour_led();
	uint8_t ofst = get_Minute_Offset_led();

	for (int i = 0; i < 16; i++) {
		if (i == min) {
			blink[i] = blink_min;
		} else if (i == h) {
			blink[i] = blink_h;
		} else if (i == ofst) {
			blink[i] = blink_ofst;
		} else {
			blink[i] = 0;
		}

		ptr_buffer[i] = (min == i) || (h == i) || (ofst == i) ? 5 : 0;

		/*if (i == min) {
		 blink[i] = 10;
		 }
		 else{
		 blink[i] = 0;
		 }
		 if (blink[i] == 0) {
		 ptr_buffer[i] = (h == i) || (ofst == i) ? 5 : 0;
		 }*/

	}

}
void second_Expired() {
	if (en_timekeeping) {
		seconds++;
		if (seconds >= 60) {
			seconds = 0;
			minutes++;
			if (minutes >= 60) {
				minutes = 0;
				hours++;
				if (hours >= 12) {
					hours = 0;
				}
			}
		}
	}
}
void standby() {
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	//PWR->CR->PDDS 1
	PWR->CR |= PWR_CR_PDDS;

	clear_wakeup();
	//WKP PIN
	PWR->CSR |= PWR_CSR_EWUP;
	//Sleepdeep

	HAL_GPIO_WritePin(K0_GPIO_Port, K0_Pin, 0);
	HAL_GPIO_WritePin(K1_GPIO_Port, K1_Pin, 0);
	HAL_GPIO_WritePin(EN_A_GPIO_Port, EN_A_Pin, 1);

	__WFI();
}
void reset_timeout() {
	timeout_ref = HAL_GetTick();
}
void clear_wakeup() {
	//Clear Standby Flag
	PWR->CR |= PWR_CR_CSBF;
	//Clear Wakeup Event Flag
	PWR->CR |= PWR_CR_CWUF;
}
void wait_RTC_write() {
	while (!(RTC->CRL & RTC_CRL_RTOFF)) {
		asm("NOP");
	}
}
void RTC_init() {
	//Enable RTC & backup domain clocks
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	RCC->APB1ENR |= RCC_APB1ENR_BKPEN;
	//unlock backup domain registers
	PWR->CR |= PWR_CR_DBP;
	//activate LSE clock
	RCC->BDCR |= RCC_BDCR_LSEON;
	// check if LSE is running

	/*
	 * SOME LOOPY SHIT NECESSARY
	 */
	while (1) {
		if (RCC->BDCR & RCC_BDCR_LSERDY) {
			//COOL THIS IS UP AND RUNNING!!!!!!
			break;
		} else {
			//SHIT THE QUARTZ IS NOT FEELING IT TODAY:(
		}
	}
	//select LSE as RTC CLOCK
	RCC->BDCR |= RCC_BDCR_RTCSEL_LSE;
	//enable RTC
	RCC->BDCR |= RCC_BDCR_RTCEN;

	wait_RTC_write();
	//Set the prescaler (VALUE STOLEN FROM DATASHEET)
	RTC->PRLL |= 0x7FFF;

}

void set_Time(uint32_t t) {
	seconds = t % 60;
	minutes = (t / 60) % 60;
	hours = (t / 60 / 12) % 12;
}
uint32_t generate_Timestamp() {
	return hours * 60 * 60 + minutes * 60 + seconds;
}
uint32_t read_RTC() {
	return (8 << (RTC->CNTH)) | RTC->CNTL;
}
uint32_t write_RTC(uint32_t t) {
	//see 18.3.4 in reference manual
	uint16_t H = 16 >> t;
	uint16_t L = t & 0xFFFF;

	//enter RTC config mode
	wait_RTC_write();
	RTC->CRL |= RTC_CRL_CNF;
	//Write count bits
	wait_RTC_write();
	RTC->CNTH = H;
	wait_RTC_write();
	RTC->CNTL = L;
	//leave RTC config mode
	wait_RTC_write();
	RTC->CRL &= ~RTC_CRL_CNF;

	//wait until all write operations finished
	wait_RTC_write();

}

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
	MX_I2C2_Init();
	MX_USART1_UART_Init();
	MX_TIM3_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	//INIT RTC
	//RTC_init();
	set_Time(read_RTC());
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t starttime = HAL_GetTick();
	uint32_t now;
	int programm_nr = 0;
//timeset vars
	int timeset_state = 0;
	while (1) {
		switch (state) {
		case time:
			//short standby timeout
			timeout_state = 0;
			//DRAW
			draw_time(10, 0, 0);
			/*for (int i = 0; i<16; i++){
			 ptr_buffer[i] = (5>i) ? 5 : 0;
			 }*/
			if (button_state == 1) {
				button_state = 0;
				clear_blink();
				state = menue;
				programm_nr = 0;
			} else if (button_state == 2) {
				button_state = 0;
				clear_blink();
				state = timeset;
				programm_nr = 0;
			}

			break;
		case menue:
			//long standby timeout
			timeout_state = 2;
			for (int i = 0; i < 16; i++) {
				ptr_buffer[i] = (offsetLED_mapping[programm_nr] == i) ? 5 : 0;

			}
			if (button_state == 2) {
				button_state = 0;
				if (programm_nr < 4) {
					state = program;
				}
			} else if (button_state == 1) {
				button_state = 0;
				programm_nr++;
				if (programm_nr == 4) {
					programm_nr = 0;
					state = time;
				}

			}
			break;
		case program:
			//long standby timeout
			timeout_state = 2;
			state = time;
			break;
		case timeset:
			//long standby timeout
			timeout_state = 2;
			//disable timekeeping
			en_timekeeping = 0;
			if (button_state == 1) {

				button_state = 0;
				clear_blink();
				switch (timeset_state) {
				case 0:
					//H
					hours++;
					if (hours > 11) {
						hours = 0;
					}
					break;
				case 1:
					//MIN
					minutes += 5;
					minutes = minutes - minutes % 5;
					if (minutes > 60) {
						minutes -= 60;
					}
					break;
				case 2:
					//MIN OFFSET
					minutes += 1;
					if (minutes > 59) {
						minutes = 0;
					}
					break;
				}

			}

			switch (timeset_state) {
			case 0:

				draw_time(10, 3, 0);
				break;
			case 1:

				draw_time(3, 0, 0);
				break;
			case 2:

				draw_time(10, 0, 3);
				break;
			default:
				break;
			}
			if (button_state == 2) {

				button_state = 0;
				timeset_state++;
				if (timeset_state == 3) {
					//save everything first
					//write_RTC(generate_Timestamp());

					//enable timekeeping again
					en_timekeeping = 1;

					state = time;
					timeset_state = 0;
					seconds = 0;
				}

				clear_blink();

			}
			break;
		default:
			break;
		}
		animation();

		//Gameloop moves
		now = HAL_GetTick();
		int period = 50;
		if (now - starttime < period) {
			HAL_Delay(period - (now - starttime));
		}
		starttime = now;
		flip_Framebuffer();

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

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

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
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 7999;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
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
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */
//100-1
//10-1
	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 99;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 9;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, EN_A_Pin | A1_Pin | A2_Pin | A3_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, BUZ_Pin | K1_Pin | K0_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : SWITCH_Pin */
	GPIO_InitStruct.Pin = SWITCH_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SWITCH_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : EN_A_Pin A1_Pin A2_Pin A3_Pin */
	GPIO_InitStruct.Pin = EN_A_Pin | A1_Pin | A2_Pin | A3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : BAT_STAT_Pin */
	GPIO_InitStruct.Pin = BAT_STAT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BAT_STAT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : BUZ_Pin K1_Pin K0_Pin */
	GPIO_InitStruct.Pin = BUZ_Pin | K1_Pin | K0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_SET);
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
