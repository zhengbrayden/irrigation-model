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
#include <string.h>
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static void ADC_Select_CH(int CH);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ADC_Select_CH(int CH) {
    ADC_ChannelConfTypeDef sConfig = {0};

switch(CH)
{
case 0:
sConfig.Channel = ADC_CHANNEL_0;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 1:
sConfig.Channel = ADC_CHANNEL_1;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 2:
sConfig.Channel = ADC_CHANNEL_2;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 3:
sConfig.Channel = ADC_CHANNEL_3;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 4:
sConfig.Channel = ADC_CHANNEL_4;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 5:
sConfig.Channel = ADC_CHANNEL_5;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 6:
sConfig.Channel = ADC_CHANNEL_6;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 7:
sConfig.Channel = ADC_CHANNEL_7;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 8:
sConfig.Channel = ADC_CHANNEL_8;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 9:
sConfig.Channel = ADC_CHANNEL_9;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 10:
sConfig.Channel = ADC_CHANNEL_10;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 11:
sConfig.Channel = ADC_CHANNEL_11;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 12:
sConfig.Channel = ADC_CHANNEL_12;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 13:
sConfig.Channel = ADC_CHANNEL_13;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 14:
sConfig.Channel = ADC_CHANNEL_14;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
case 15:
sConfig.Channel = ADC_CHANNEL_15;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
break;
}
}

volatile uint8_t hcsr04_Rx_flag = 0;
volatile uint8_t first_edge = 0;
volatile uint16_t time_edge1 = 0;
volatile uint16_t time_edge2 = 0;
volatile uint8_t forcer = 0;
int setup_mode = 1;
uint8_t rcv_intpt_flag = 0;
uint8_t byte;

int connections[4] = {0};
int pwms[4] = {0};
int starts[4] = {0};
int stops[4] = {0};

int button_pushed = 0;
int rpm_tick_count = 0;

volatile uint8_t clock_hours = 0;
volatile uint8_t clock_mins = 0;
volatile uint16_t rpm_mins = 0;
volatile uint8_t clock_secs = 0;
volatile int depth = 1;

volatile int wall_clock_hr_update_flag = 0;
volatile int read_depth_flag = 0;

int active_zone = 0;

int manual_control = 0;

uint8_t ADC_CH9;

void change_pipe(void) {
	uint8_t txd_msg_buffer[64];
	for (int i=0; i<4; i++) {
		if (starts[i] == clock_hours){
			active_zone = i;
		    return;
		}
		else if (stops[i] == clock_hours) {
			HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
			active_zone = -1;
		}
	}
}

void emergency_mode() {
		uint8_t txd_msg_buffer[64];
		sprintf((char*)txd_msg_buffer, "\r\nRESERVOIR EMPTY!");
		HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		while (1) {
			HAL_GPIO_WritePin(GPIOC, GRN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, BLU_Pin, GPIO_PIN_SET);
			HAL_Delay(500);
			HAL_GPIO_WritePin(GPIOC, GRN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, BLU_Pin, GPIO_PIN_RESET);
			HAL_Delay(500);
		}
	}


uint16_t time_diff = 0;
uint16_t distance = 0;



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t txd_msg_buffer[64] = {'h'};

	void set_timer(uint8_t DIGIT_A, uint8_t DIGIT_B) {
				uint8_t DIGITA_VAL = 0x0F & DIGIT_A; //mask off higher4 bits
				 int Abit0 = (DIGITA_VAL ) & 1;  	// extract Abit0 of the 4-bit value
				 int Abit1 = (DIGITA_VAL >> 1) & 1;  // extract Abit1 of the 4-bit value
				 int Abit2 = (DIGITA_VAL >> 2) & 1;  // extract Abit2 of the 4-bit value
				 int Abit3 = (DIGITA_VAL >> 3) & 1;  // extract Abit3 of the 4-bit value

				 uint8_t DIGITB_VAL = 0x0F & DIGIT_B; //mask off higher4 bits
				 int Bbit0 = (DIGITB_VAL ) & 1;  	// extract Bbit0 of the 4-bit value
				 int Bbit1 = (DIGITB_VAL >> 1) & 1;  // extract Bbit1 of the 4-bit value
				 int Bbit2 = (DIGITB_VAL >> 2) & 1;  // extract Bbit2 of the 4-bit value
				 int Bbit3 = (DIGITB_VAL >> 3) & 1;  // extract Bbit3 of the 4-bit value

				 if (Abit0 == (0))
				 {
					 HAL_GPIO_WritePin(GPIOC, A0_Pin, GPIO_PIN_RESET);
				 }
				 else
				 {
					 HAL_GPIO_WritePin(GPIOC, A0_Pin, GPIO_PIN_SET);

				 }
				 if (Abit1 == (0))
				 {
					 HAL_GPIO_WritePin(GPIOC, A1_Pin, GPIO_PIN_RESET);
				 }
				 else
				 {
					 HAL_GPIO_WritePin(GPIOC, A1_Pin, GPIO_PIN_SET);

				 }
				 if (Abit2 == (0))
				 {
					 HAL_GPIO_WritePin(GPIOC, A2_Pin, GPIO_PIN_RESET);
				 }
				 else
				 {
					 HAL_GPIO_WritePin(GPIOC, A2_Pin, GPIO_PIN_SET);

				 }
				 if (Abit3 == (0))
				 {
					 HAL_GPIO_WritePin(GPIOC, A3_Pin, GPIO_PIN_RESET);
				 }
				 else
				 {
					 HAL_GPIO_WritePin(GPIOC, A3_Pin, GPIO_PIN_SET);

				 }


				 if (Bbit0 == (0))
				 {
					 HAL_GPIO_WritePin(GPIOB, B0_Pin, GPIO_PIN_RESET);
				 }
				 else
				 {
					 HAL_GPIO_WritePin(GPIOC, B0_Pin, GPIO_PIN_SET);

				 }
				 if (Bbit1 == (0))
				 {
					 HAL_GPIO_WritePin(GPIOC, B1_Pin, GPIO_PIN_RESET);
				 }
				 else
				 {
					 HAL_GPIO_WritePin(GPIOC, B1_Pin, GPIO_PIN_SET);

				 }
				 if (Bbit2 == (0))
				 {
					 HAL_GPIO_WritePin(GPIOC, B2_Pin, GPIO_PIN_RESET);
				 }
				 else
				 {
					 HAL_GPIO_WritePin(GPIOC, B2_Pin, GPIO_PIN_SET);

				 }
				 if (Bbit3 == (0))
				 {
					 HAL_GPIO_WritePin(GPIOC, B3_Pin, GPIO_PIN_RESET);
				 }
				 else
				 {
					 HAL_GPIO_WritePin(GPIOC, B3_Pin, GPIO_PIN_SET);

				 }

		}

	void HCSR04_TRIG_PULSE(void) {
				HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_SET);
				for (int j=0; j!=15; j++) {};
				HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);
			}

	void read_depth(void) {
			hcsr04_Rx_flag = 0;
			 first_edge = 0;
			 time_edge1 = 0;
			 time_edge2 = 0;
			 time_diff = 0;
			 distance = 0;

			 HCSR04_TRIG_PULSE();

			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

			 while(hcsr04_Rx_flag == 0){

			 };

			 time_diff = time_edge2 - time_edge1;


			 depth = 100 - time_diff/58 * 5;

			 if (depth < 0) {
				 emergency_mode();
			 }

			 set_timer((int)(depth/10), depth%10);
		}

	void reset_outputs() {
		TIM1->CCR2 = 0;
		TIM1->CCR1 = 0;
		HAL_GPIO_WritePin(GPIOC, GRN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, BLU_Pin, GPIO_PIN_RESET);
	}

	//HAL_GPIO_WritePin(GPIOC, B1_Pin|B3_Pin|A0_Pin|B2_Pin
//    |GPIO_PIN_4|RED_Pin|GRN_Pin|BLU_Pin
//    |A3_Pin|A2_Pin|A1_Pin, GPIO_PIN_RESET);


	void set_motor(uint8_t percent, int forward) {
		rpm_mins = 1;
		rpm_tick_count = 0;

		if (percent == 0) manual_control = 1;
		else manual_control = 0;

		if (manual_control) {
			//manual control, set to adc value
			ADC_Select_CH(1);
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 1000);
			ADC_CH9 = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);
			//turn adc into percent by multipling by 100 and dividing by 255
			percent = ADC_CH9 * 100 / 255;
		}

		if (forward) {
			TIM1->CCR2 = 0;
			TIM1->CCR1 = percent ? (uint32_t)(2000*percent/100) - 1 : 0;
		} else {
			TIM1->CCR1 = 0;
			TIM1->CCR2 = percent ? (uint32_t)(2000*percent/100) - 1 : 0;
		}
	}

	void set_servo(int pipe) {
		switch(pipe){
			case 0:
				TIM4->CCR1 = 500;
				break;
			case 1:
				TIM4->CCR1 = 1167;
				break;
			case 2:
				TIM4->CCR1 = 1833;
				break;
			case 3:
				TIM4->CCR1 = 2500;
				break;

		}
	}

	void set_led(int pipe) {
		switch(pipe) {
		 	 case 0:
		 		 HAL_GPIO_WritePin(GPIOC, GRN_Pin, GPIO_PIN_RESET);
		 		 HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_SET);
		 		 HAL_GPIO_WritePin(GPIOC, BLU_Pin, GPIO_PIN_SET);
		 		 break;
		 	 case 1:
		 		 HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_SET);
		 		 HAL_GPIO_WritePin(GPIOC, GRN_Pin, GPIO_PIN_RESET);
		 		 HAL_GPIO_WritePin(GPIOC, BLU_Pin, GPIO_PIN_RESET);
		 		 break;
		 	 case 2:
		 		HAL_GPIO_WritePin(GPIOC, GRN_Pin, GPIO_PIN_SET);
		 		HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_RESET);
		 		HAL_GPIO_WritePin(GPIOC, GRN_Pin, GPIO_PIN_SET);
		 		break;
		 	 case 3:
		 		HAL_GPIO_WritePin(GPIOC, GRN_Pin, GPIO_PIN_RESET);
		 		HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_RESET);
		 		HAL_GPIO_WritePin(GPIOC, BLU_Pin, GPIO_PIN_SET);

		}
	}

	void print_diagnostics(void) {
//HOURS | MINS | ZONE | MOTOR SPEED | MOTOR RPM | RESERVOIR DEPTH

		sprintf((char*)txd_msg_buffer, "\r\n%d, %d, %d, %d, %d, %d", clock_hours,
				clock_mins, active_zone, active_zone != -1 ? (pwms[active_zone] ?
				pwms[0] : ADC_CH9) : 0,
				rpm_tick_count*12/rpm_mins, depth);
		HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Init(&htim3);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Base_Init(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  TIM1->CCR1 = 0;
  clock_hours = 0;
  clock_mins = 0;
  clock_secs = 0;

  HAL_TIM_Base_Start(&htim4);
  TIM4->PSC = 16-1;
  TIM4->ARR = 20000-1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  sprintf((char*)txd_msg_buffer, "\r\nFIRST PIPELINE CHOICE FOR CONNECTION: ");
  while (1)
  {
	  if (setup_mode) {
		  rcv_intpt_flag = 00;
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  connections[0] = (byte-'0');
		  rcv_intpt_flag = 00;


		  sprintf((char*)txd_msg_buffer, "\r\nFIRST PIPELINE CHOICE FOR MOTOR PWM: ");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  rcv_intpt_flag = 00;
		  pwms[0] += (byte-'0')*10;
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  pwms[0] += (byte-'0');
		  rcv_intpt_flag = 00;

		  sprintf((char*)txd_msg_buffer, "\r\nSECOND ZONE CHOICE FOR CONNECTION: ");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  connections[1] = (byte-'0');
		  rcv_intpt_flag = 00;

		  sprintf((char*)txd_msg_buffer, "\r\nSECOND PIPELINE CHOICE FOR MOTOR PWM: ");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  rcv_intpt_flag = 00;
		  pwms[1] += (byte-'0')*10;
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  pwms[1] += (byte-'0');
		  rcv_intpt_flag = 00;

		  sprintf((char*)txd_msg_buffer, "\r\nTHIRD ZONE CHOICE FOR CONNECTION: ");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  connections[2] = (byte-'0');
		  rcv_intpt_flag = 00;

		  sprintf((char*)txd_msg_buffer, "\r\nTHIRD PIPELINE CHOICE FOR MOTOR PWM: ");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  rcv_intpt_flag = 00;
		  pwms[2] += (byte-'0')*10;
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  pwms[2] += (byte-'0');
		  rcv_intpt_flag = 00;

		  sprintf((char*)txd_msg_buffer, "\r\nFOURTH ZONE CHOICE FOR CONNECTION: ");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  connections[3] = (byte-'0');
		  rcv_intpt_flag = 00;

		  sprintf((char*)txd_msg_buffer, "\r\nFOURTH PIPELINE CHOICE FOR MOTOR PWM: ");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  rcv_intpt_flag = 00;
		  pwms[3] += (byte-'0')*10;
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  pwms[3] += (byte-'0');
		  rcv_intpt_flag = 00;



		  // setup part b:
		  sprintf((char*)txd_msg_buffer, "\r\nINLET WALL CLOCK START TIME: ");
		  rcv_intpt_flag = 00;
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  rcv_intpt_flag = 00;
		  starts[0] += (byte-'0')*10;
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  starts[0] += (byte-'0');
		  rcv_intpt_flag = 00;

		  sprintf((char*)txd_msg_buffer, "\r\nINLET WALL CLOCK STOP TIME: ");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  rcv_intpt_flag = 00;
		  stops[0] += (byte-'0')*10;
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  stops[0] += (byte-'0');
		  rcv_intpt_flag = 00;

		  sprintf((char*)txd_msg_buffer, "\r\nFIRST ZONE WALL CLOCK START TIME: ");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  rcv_intpt_flag = 00;
		  starts[1] += (byte-'0')*10;
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  starts[1] += (byte-'0');
		  rcv_intpt_flag = 00;

		  sprintf((char*)txd_msg_buffer, "\r\nFIRST ZONE WALL CLOCK STOP TIME: ");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  rcv_intpt_flag = 00;
		  stops[1] += (byte-'0')*10;
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  stops[1] += (byte-'0');
		  rcv_intpt_flag = 00;

		  sprintf((char*)txd_msg_buffer, "\r\nSECOND ZONE WALL CLOCK START TIME: ");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  rcv_intpt_flag = 00;
		  starts[2] += (byte-'0')*10;
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  starts[2] += (byte-'0');
		  rcv_intpt_flag = 00;

		  sprintf((char*)txd_msg_buffer, "\r\nSECOND ZONE WALL CLOCK STOP TIME: ");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  rcv_intpt_flag = 00;
		  stops[2] += (byte-'0')*10;
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  stops[2] += (byte-'0');
		  rcv_intpt_flag = 00;

		  sprintf((char*)txd_msg_buffer, "\r\nTHIRD ZONE WALL CLOCK START TIME: ");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  rcv_intpt_flag = 00;
		  starts[3] += (byte-'0')*10;
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  starts[3] += (byte-'0');
		  rcv_intpt_flag = 00;

		  sprintf((char*)txd_msg_buffer, "\r\nTHIRD ZONE WALL CLOCK STOP TIME: ");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  rcv_intpt_flag = 00;
		  stops[3] += (byte-'0')*10;
		  HAL_UART_Receive_IT(&huart6, &byte, 1);
		  while(rcv_intpt_flag == 00) {};
		  stops[3] += (byte-'0');
		  rcv_intpt_flag = 00;

		  setup_mode = 0;

		  sprintf((char*)txd_msg_buffer, "\r\nSETUP MODE COMPLETE!");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

		  sprintf((char*)txd_msg_buffer, "\r\nPARAMETERS: ");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

		  sprintf((char*)txd_msg_buffer, "\r\nCONNECTION 1: %d", connections[0]);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

		  sprintf((char*)txd_msg_buffer, "\r\nPWM 1: %d", pwms[0]);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

		  sprintf((char*)txd_msg_buffer, "\r\nCONNECTION 2: %d", connections[1]);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

		  sprintf((char*)txd_msg_buffer, "\r\nPWM 2: %d", pwms[1]);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

		  sprintf((char*)txd_msg_buffer, "\r\nCONNECTION 3: %d", connections[2]);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

		  sprintf((char*)txd_msg_buffer, "\r\nPWM 3: %d", pwms[2]);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

		  sprintf((char*)txd_msg_buffer, "\r\nCONNECTION 4: %d", connections[3]);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

		  sprintf((char*)txd_msg_buffer, "\r\nPWM 4: %d", pwms[3]);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

		  sprintf((char*)txd_msg_buffer, "\r\nSTART INLET: %d", starts[0]);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

		  sprintf((char*)txd_msg_buffer, "\r\nSTOP INLET: %d", stops[0]);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

		  sprintf((char*)txd_msg_buffer, "\r\nSTART ZONE 1: %d", starts[1]);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

		  sprintf((char*)txd_msg_buffer, "\r\nSTOP ZONE 1 %d", stops[1]);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

		  sprintf((char*)txd_msg_buffer, "\r\nSTART ZONE 2: %d", starts[2]);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

		  sprintf((char*)txd_msg_buffer, "\r\nSTOP ZONE 2: %d", stops[2]);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

		  sprintf((char*)txd_msg_buffer, "\r\nSTART ZONE 3: %d", starts[3]);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

		  sprintf((char*)txd_msg_buffer, "\r\nSTOP ZONE 3: %d", stops[3]);
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

		  sprintf((char*)txd_msg_buffer, "\r\nPRESS BLUE PUSHBUTTON TO CONTINUE");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);


		  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0); // Set priority for EXTI lines 10-15
		  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

		  while (!button_pushed) {
			  HAL_GPIO_WritePin(GPIOC, GRN_Pin, GPIO_PIN_SET);

			  HAL_Delay(500);
			  HAL_GPIO_WritePin(GPIOC, GRN_Pin, GPIO_PIN_RESET);
			  HAL_Delay(500);

		  }

		  change_pipe();

		  HAL_TIM_Base_Start_IT(&htim5);
		  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

		  reset_outputs();

		  sprintf((char*)txd_msg_buffer, "\r\nHOURS | MINS | ZONE | MOTOR SPEED | MOTOR RPM | RESERVOIR DEPTH");
		  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
		  read_depth();
		  print_diagnostics();

	  }
	  else {

		  //ok we have all of our settings. Now we must start the inlet
		 set_led(connections[active_zone]);
		 set_servo(connections[active_zone]);
		 set_motor(pwms[active_zone], active_zone != 0);
		 //keep checking for hour change
		 if (active_zone != -1) {
			 set_led(connections[active_zone]);
			 set_servo(connections[active_zone]);
			 set_motor(pwms[active_zone], active_zone!=0);
		 } else {
			 reset_outputs();
		 }
		 read_depth();
		 while (!wall_clock_hr_update_flag) {}

		 change_pipe();
		 print_diagnostics();

		 wall_clock_hr_update_flag = 0;
		}

	  if (clock_hours >= 24) {
		  reset_outputs();
		  while (1) {}
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1200-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65536-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 53-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 60000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  huart6.Init.BaudRate = 9600;
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
  HAL_GPIO_WritePin(GPIOC, B1_Pin|B3_Pin|A0_Pin|B2_Pin
                          |GPIO_PIN_4|RED_Pin|GRN_Pin|BLU_Pin
                          |A3_Pin|A2_Pin|A1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, HCSR04_TRIG_Pin|GPIO_PIN_4|B0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BLUE_BUTTON_Pin */
  GPIO_InitStruct.Pin = BLUE_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLUE_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B1_Pin B3_Pin A0_Pin B2_Pin
                           PC4 RED_Pin GRN_Pin BLU_Pin
                           A3_Pin A2_Pin A1_Pin */
  GPIO_InitStruct.Pin = B1_Pin|B3_Pin|A0_Pin|B2_Pin
                          |GPIO_PIN_4|RED_Pin|GRN_Pin|BLU_Pin
                          |A3_Pin|A2_Pin|A1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RPM_Pin */
  GPIO_InitStruct.Pin = RPM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RPM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HCSR04_TRIG_Pin PB4 B0_Pin */
  GPIO_InitStruct.Pin = HCSR04_TRIG_Pin|GPIO_PIN_4|B0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		if (htim->Channel == 2)
		{
			if (first_edge == 0)
			{
				time_edge1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
				first_edge = 1;
			}

	else
			{
				time_edge2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
				__HAL_TIM_SET_COUNTER(htim, 0);
				hcsr04_Rx_flag = 1;
			}
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART6) {

		HAL_UART_Transmit(&huart6, &byte, 1, 100);
		rcv_intpt_flag = 1;
	}
}

void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(BLUE_BUTTON_Pin); // Call the HAL interrupt handler
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BLUE_BUTTON_Pin) // Check if it is the BLUE_BUTTON
    {
        // Handle button press
        button_pushed = 1;
    }
    if (GPIO_Pin == RPM_Pin) {
    	if (button_pushed) rpm_tick_count += 1;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM5) {
		clock_mins += 1;
		rpm_mins += 1;
		clock_secs = 0;
		if ((clock_mins == 60 )) {
				clock_hours += 1;
				clock_mins = 0;
				wall_clock_hr_update_flag = 1;
		}
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
