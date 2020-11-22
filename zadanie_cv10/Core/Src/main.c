/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
uint8_t currentDutyCycle = 50;
uint8_t targetDutyCycle = 50;
uint8_t programMode = AUTO_MODE;
uint8_t DutyCycleTrend = INCREASING_TREND;
char manualString[] = "manual";
char autoString[] = "auto";
char pwmString[] = "PWM";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void proccesDmaData(char *paket,uint16_t length);
uint8_t extractDataString(char *paket, uint16_t *length);
void executeInstruction(char *instruction,uint16_t length);
uint8_t compareString(char *string1,char *string2,uint16_t dataLength);
void executePWMinstruction(char *instruction,uint16_t length);
uint16_t string2int(char *string, uint8_t length);
uint16_t nonNegativePower(uint16_t base, uint8_t exponent);

void updateCurrentDutyCycle(void);
void setCurrentDutyCycle(uint8_t new_currentDutyCycle);
void autoModeUpdateTargetDutyCycle(void);
void setTargetDutyCycleAndTrend(uint8_t desiredDutyCycle);
uint8_t determineCurrentDutyCycle(void);
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

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  USART2_RegisterCallback(proccesDmaData);
  TIM2_RegisterCallback(determineCurrentDutyCycle);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_Init1msTick(8000000);
  LL_SetSystemCoreClock(8000000);
}

/* USER CODE BEGIN 4 */
/*
 * Implementation of function processing data received via USART.
 */
void proccesDmaData(char *paket,uint16_t length){
	if(extractDataString(paket, &length) != 0){
		return;
	}
	else{
		executeInstruction(paket,length);
	}
}

uint8_t extractDataString(char *paket, uint16_t *length_ptr){
	uint16_t length = *length_ptr;
	if(paket[length-1] == '\r'){
		length -= 1;
	}
	if(length <= 2){
		return -1;
	}
	if((paket[0] == '$') && (paket[length-1] == '$')){
		for(int i = 0; i < (length - 2); i++){
			paket[i] = paket[i+1];
		}
		paket[length-2] = '\0';
		*length_ptr = length - 2; //length without '\0'
		return 0;
	}
	else return -1;
}

void executeInstruction(char *instruction,uint16_t length){
	if(programMode == MANUAL_MODE){
		executePWMinstruction(instruction, length);
	}

	if(compareString(instruction, manualString, length) == 0){
		programMode = MANUAL_MODE;
	}

	if(compareString(instruction, autoString, length) == 0){
			programMode = AUTO_MODE;
	}
}

uint8_t compareString(char *string1,char *string2,uint16_t dataLength){
	for(int i = 0; i < dataLength; i++){
		if(string1[i] != string2[i]){
			return -1;
		}
	}
	return 0;
}

void executePWMinstruction(char *instruction,uint16_t length){
	/* PWMinstructions PWMxx consists of 2 parts:
	 * 1) pwmString : PWM
	 * 2) dutyCycleStr : xx
	 */
	static int dutyCycleStrLen = 2;
	int pwmInstructionSize = sizeof(pwmString) -1 + dutyCycleStrLen; // 2 chars are numbers representing Duty cycle, 1 char is '\0' in pwmString

	if(length != pwmInstructionSize) return;

	if(compareString(instruction, pwmString, length - dutyCycleStrLen) != 0) return;

	for(int i = (sizeof(pwmString) -1); i < pwmInstructionSize; i++){
		if((instruction[i] < '0') || (instruction[i] > '9')){
				return;
		}
	}

	setTargetDutyCycleAndTrend( string2int(instruction + sizeof(pwmString) - 1, dutyCycleStrLen));
}

uint16_t string2int(char *string, uint8_t length){
	uint16_t value = 0;
	for(int i = 0; i < length; i++){
		value += (string[i] - '0') * nonNegativePower(10,length-i-1);
	}
	return value;
}

uint16_t nonNegativePower(uint16_t base, uint8_t exponent){
	int value = 1;

	if(exponent == 0){
		return 1;
	}

	for(int i = 1; i <= exponent; i++){
		value *= base;
	}
	return value;
}

void updateCurrentDutyCycle(void){
	if(targetDutyCycle > currentDutyCycle){
		setCurrentDutyCycle(currentDutyCycle + UPTREND_STEP);
	}
	else if(targetDutyCycle < currentDutyCycle){
		setCurrentDutyCycle(currentDutyCycle - DOWNTREND_STEP);
	}
}

void setCurrentDutyCycle(uint8_t new_currentDutyCycle){
	if(new_currentDutyCycle >= MAX_DUTY_CYCLE){
		currentDutyCycle = MAX_DUTY_CYCLE;
	}
	else if(new_currentDutyCycle <= MIN_DUTY_CYCLE){
		currentDutyCycle = MIN_DUTY_CYCLE;
	}
	else{
		currentDutyCycle = new_currentDutyCycle;
	}
}

void autoModeUpdateTargetDutyCycle(void){
	if(DutyCycleTrend == INCREASING_TREND){
		setTargetDutyCycleAndTrend(targetDutyCycle + UPTREND_STEP);
	}
	else{
		setTargetDutyCycleAndTrend(targetDutyCycle - DOWNTREND_STEP);
	}
}

void setTargetDutyCycleAndTrend(uint8_t desiredDutyCycle){
	if(desiredDutyCycle >= MAX_DUTY_CYCLE){
			targetDutyCycle = MAX_DUTY_CYCLE;
			DutyCycleTrend = DECREASING_TREND;
	}
	else if(desiredDutyCycle <= MIN_DUTY_CYCLE){
			targetDutyCycle = MIN_DUTY_CYCLE;
			DutyCycleTrend = INCREASING_TREND;
	}
	else{
		targetDutyCycle = desiredDutyCycle;
	}
}

uint8_t determineCurrentDutyCycle(void){
	if(programMode == AUTO_MODE){
		autoModeUpdateTargetDutyCycle();
	}
	updateCurrentDutyCycle();
	return currentDutyCycle;
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
