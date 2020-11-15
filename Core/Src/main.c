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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// ZMIENNE DO MySysTick()

uint8_t ms_set=0;
extern uint16_t arg;

//BUFOROWE ZMIENNE

#define USART_TXBUF_LEN 500 //długość nadawczego bufora
#define USART_RXBUF_LEN 500 //długość odbiorczego bufora
uint8_t USART_TxBuf[USART_TXBUF_LEN]; //bufor nadawczy
uint8_t USART_RxBuf[USART_RXBUF_LEN]; //bufor odbiorczy

__IO int USART_TX_Empty = 0;
__IO int USART_TX_Busy = 0;
__IO int USART_RX_Empty = 0;
__IO int USART_RX_Busy = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void MySysTick(int arg){

	static uint16_t ms=0;
	ms++;
	if(ms>arg)
	{
		ms=0;
		ms_set=1;
	}

}

uint8_t USART_RX_IsEmpty(){
	if(USART_RX_Busy == USART_RX_Empty)
	{
		return 0;
	}else{
		return 1;
	}
}//Funkcja sprawdzajacy czy bufor odbiorczy jest pusty

uint8_t USART_GC(){
	uint8_t tmp;
	if(USART_RX_Empty!=USART_RX_Busy)
	{
		tmp = USART_RxBuf[USART_RX_Busy];
		USART_RX_Busy++;
		if(USART_RX_Busy >= USART_RXBUF_LEN) USART_RX_Busy =0;
		return tmp;
	}else
	{
		return 0;
	}
} //Funkcja zwracająca znak

uint8_t USART_GD(char *buf){
	static uint8_t bf[128];
	static uint8_t index=0;
	int i;
	uint8_t len_com;
	while(USART_RX_IsEmpty())
	{
		bf[index] = USART_GC();
		if((bf[index] == 59)) //dodać znak końca stringa \0
		{
			for(i=0 ; i<=index ; i++)
			{
				buf[i]=bf[i];
			}
			len_com=index;
			index=0;
			return len_com;
		}else
		{
			index++;
			if(index>=128) index=0;
		}
	}
	return 0;
}//Funkcja odbierająca dane

void USART_send(char* format,...){
	char tmp_s[512];
	int i;
	__IO int index;
	va_list arglist;
	va_start(arglist,format);
	vsprintf(tmp_s,format,arglist);
	va_end(arglist);
	index=USART_TX_Empty;
	for(i=0;i<strlen(tmp_s);i++){
		USART_TxBuf[index] = tmp_s[i];
		index++;
		if(index >= USART_TXBUF_LEN) index=0;
	}

	__disable_irq();
	if((USART_TX_Empty == USART_TX_Busy) && (__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TXE)==SET)){ //2 parametr sprawdza bufor nadajnika
		USART_TX_Empty = index;
		uint8_t tmp=USART_TxBuf[USART_TX_Busy];
		USART_TX_Busy++;
		if(USART_TX_Busy >= USART_TXBUF_LEN) USART_TX_Busy = 0;
		HAL_UART_Transmit_IT(&huart2,&tmp,1);
	}else{
		USART_TX_Empty = index;
	}
	__enable_irq();
}//Funkcja wysyłająca dane

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart2){
		if(USART_TX_Empty != USART_TX_Busy){
			uint8_t tmp= USART_TxBuf[USART_TX_Busy];
			USART_TX_Busy++;
			if(USART_TX_Busy >= USART_TXBUF_LEN) USART_TX_Busy =0;
			HAL_UART_Transmit_IT(&huart2, &tmp , 1);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart2){
		USART_RX_Empty++;
		if(USART_RX_Empty >= USART_RXBUF_LEN) USART_RX_Empty=0;
		HAL_UART_Receive_IT(&huart2, &USART_RxBuf[USART_RX_Empty], 1);
	}
}


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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Receive_IT(&huart2, &USART_RxBuf[0], 1);


  int len=0;
  char bx[128];
  char komenda[128];
  int i,y=0;

  while (1)
  {
	  len = USART_GD(bx);
	  arg = 500;
	  if(ms_set)
	  {
		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		  ms_set=0;
	  }
	  if(len>0){
		  USART_send("Odebrana zostala wiadomosc %d \r\n",len);

		  for(i=0 ; i<128 ; i++){
			  if(bx[i] == 58)
			  {
				  if((bx[i+4] == 'S') && (bx[i+5]=='T') && (bx[i+6]=='M')){
					  i =+ 7;
					  USART_send("Poczatek komendy \r \n");
					  USART_send("Wczytywanie Komendy : %c \r \n",bx[i]);
					  do{
						  if(ms_set){
		  	  	  	  	  	  komenda[y]= bx[i];
		  	  	  	  	  	  bx[i-7]=0;
							  ms_set=0;
		  	  	  	  	  	  y++;
						  }
					  }while(bx[i+1]==';');
					  USART_send("Koniec komendy takiej i owakiej \r \n");
					  break;
				  }
			  }

		  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SW_BLUE_Pin */
  GPIO_InitStruct.Pin = SW_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_BLUE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
