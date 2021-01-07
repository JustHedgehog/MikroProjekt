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
#include <math.h>
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
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// ZMIENNE DO MySysTick()

uint8_t ms_set=0;
extern uint16_t arg;

//BUFOROWE ZMIENNE

#define USART_TXBUF_LEN 2000 //długość nadawczego bufora
#define USART_RXBUF_LEN 2000 //długość odbiorczego bufora
uint8_t USART_TxBuf[USART_TXBUF_LEN]; //bufor nadawczy
uint8_t USART_RxBuf[USART_RXBUF_LEN]; //bufor odbiorczy

__IO int USART_TX_Empty = 0;
__IO int USART_TX_Busy = 0;
__IO int USART_RX_Empty = 0;
__IO int USART_RX_Busy = 0;

//ZMIENNE DO GENERACJI SINUSA

uint16_t fs = 10000;
const double PI = 3.14;
int f = 1;
int tablica_wartosci[10000];

//ZMIENNE DO CZUJNIKA TEMPERATURY

uint32_t temp;

//ZMIENNE DO RAMKI

#define  FRSTART 0x3A
#define FREND 0x3B
#define FRCOD 0x5C
#define FRCODS 0x61
#define FRCODE 0x62
int fr_busy = 0;
char receiver_name[3];
char sender_name[3];
char command[256];
char frame[263];
int frame_read=0;

//zmienne do testów

int k=0;

// zmienne do DMA

uint16_t dma_buff[4096];
uint32_t i=0;
uint32_t suma_dma;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
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
	static uint8_t bf[500];
	static uint8_t index=0;
	int i;
	uint8_t len_com;
	while(USART_RX_IsEmpty())
	{
		bf[index] = USART_GC();
		if (frame_read == 1 && bf[index] == FRSTART) {
			bf[0] = bf[index];
			index = 0;
		}
		if (frame_read == 0) {
			if (bf[index] == FRSTART) {
				frame_read = 1;
			}
		}
		if (frame_read == 1) {
			if ((bf[index] == FREND)) {
				for (i = 0; i <= index; i++) {
					buf[i] = bf[i];
				}
				len_com = index;
				index = 0;
				frame_read = 0;
				return len_com;
			} else {
				index++;
				if (index >= 500) {
					index = 0;
				}
			}
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


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1){
	int i;
	suma_dma=0;
	for(i= 2048 ; i<4096 ; i++){
		suma_dma += dma_buff[i];
	}
	temp = (((suma_dma/2048)*5)/4095)*100;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc1){
	int i;
	suma_dma=0;
	for(i= 0 ; i<2048 ; i++){
		suma_dma += dma_buff[i];
	}
	temp = (((suma_dma/2048)*5)/4095)*100;
}


void generacja_sinusa(int  *tablica_wartosci ) {

	int n,f=1; // n = ilość próbek na jeden okres sygnału
	double faza_sygnalu;
	uint16_t i = 0;
	n = fs / f; //obliczenie ilości próbek w momencie zmiany f

	while (i < n) {
		faza_sygnalu = ((i * 2 * PI * f) / fs);
		*(tablica_wartosci+i) = 2048 + (sin(faza_sygnalu) * 2048);
		i = i + 1;
	}
	i = 0;
}// Funkcja generująca sinusa


void clean_frame(char * tab ,int len){

	for(int i = 0 ; i<=len ; i++)
	{
		*(tab+i) = '\0';
	}
}


void clean_after_all(int len){

	clean_frame(frame, len);
	clean_frame(command, (len - 6));
	clean_frame(sender_name, 3);
	clean_frame(receiver_name, 3);

}

int tmp_to_hz(uint32_t temp){
	int value,hz=200;
	value = -55+temp;
	for(int i =0 ; i<value;i++){
		hz=+9;
	}
	return hz;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Receive_IT(&huart2, &USART_RxBuf[0], 1);
  HAL_ADC_Start_DMA(&hadc1, dma_buff , 4096); // Start ADC z DMA

  int len=0;
  char bx[500];
  clean_frame(bx,499);
  generacja_sinusa(tablica_wartosci);
  clean_frame(frame, len);
	while (1) {

		len = USART_GD(bx);
		int y = 0, i = 0;
		if (len > 0) {
			while (i <= len) {
				if (len < 7) {
					break;
				}
				char singlefrchar = bx[i];
				switch (singlefrchar) {
					case FRCOD: {
						if (bx[i + 1] == FRCODS) {
							frame[y] = FRSTART;
							i++;
						} else if (bx[i + 1] == FRCODE) {
							frame[y] = FREND;
							i++;
						} else if (bx[i + 1] == FRCOD) {
							frame[y] = FRCOD;
							i++;
						} else {
							i = len;
							clean_frame(bx,len);
						}
						break;
					}
					case FREND: {
	//					funkcja wykonująca komende tutaj prolly
						memcpy(sender_name, &frame[1], 3);
						memcpy(receiver_name, &frame[4], 3);
						memcpy(command, &frame[7], (y - 6));
	//					wykonywanie komendy
						if (strcmp("STM", receiver_name) == 0
								&& strcmp("STM", sender_name) != 0) {
							if (command[0] == 0) {
								USART_send(":STM%sFREMPTY;\r\n", sender_name);
								clean_after_all(y);
							} else {
								if (strcmp("temp", command) == 0) {
									USART_send(":STM%stemp,%i;\r\n", sender_name,
											temp);
									clean_after_all(y);
								}else if(strcmp("sin",command) == 0){
									int hz = tmp_to_hz(temp);
									USART_send(":STM%ssin",sender_name);
									for(int i=0; i<10000 ; i+=hz){
										USART_send(",%i",tablica_wartosci[i]); //do przemyslenia
									}
									USART_send(";");
								}else {
									USART_send(":STM%s%s;\r\n", sender_name,
											command);
									clean_after_all(y);
								}
							}
						} else {
							clean_after_all(y);
						}
						break;
					}
					default: {
						frame[y] = bx[i];
					}
				}
				y++;
				i++;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
