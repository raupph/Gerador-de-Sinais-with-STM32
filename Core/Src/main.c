/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include <string.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define POT_BUFFER 64

#define SCALE_ADC (3.3/4095)

#define TOUT 100
#define TAM_MSG 100

#define TAM_MEDIDAS 64

#define N_AMOSTRAS 64

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// MAQUINA DE ESTADOS
enum {
	DESLIGADO = 0, SENOIDE, QUADRADA, TRIANGULAR
};

uint8_t estado = DESLIGADO;

// DETEC BORDA
enum {
	AGORA = 0, ANTES
};

uint8_t comando = 0;
uint8_t bt1[2] = { 0, 0 };

uint8_t bsubida;
uint8_t bdescida;

char msg[TAM_MSG];

volatile uint16_t medidas[TAM_MEDIDAS][2];
int media[2];

#define NAMOSTRAS 64

/*
///////CRIAR UMA MATRIZ PARA ONDAS DIFERENTES ////////////////// UMA TABELONA
const uint16_t senoide[NAMOSTRAS] = {

		2047,2247, 2446, 2641,
		2830, 3011, 3184, 3345,
		3494, 3629, 3749, 3852,
		3938, 4005, 4054, 4084,
		4093, 4084, 4054, 4005,
		3938, 3852, 3749, 3629,
		3494, 3345, 3184, 3011,
		2830, 2641, 2446, 2247,
		2047, 1846, 1647, 1452,
		1263, 1082, 909, 748,
		599, 464, 344, 241,
		155, 88, 39, 9,
		0, 9, 39, 88,
		155, 241, 344, 464,
		599, 748, 909, 1082,
		1263, 1452, 1647, 1846


};

const uint16_t triangular[NAMOSTRAS] = {

        0, 127, 255, 383, 511, 639, 767, 895, 1023, 1151, 1279, 1407,
        1535, 1663, 1791, 1919, 2046, 2174, 2302, 2430, 2558, 2686,
        2814, 2942, 3070, 3198, 3326, 3454, 3582, 3710, 3838, 3965,
        4093, 4221, 3842, 3714, 3586, 3458, 3330, 3202, 3074, 2946,
        2818, 2690, 2562, 2434, 2306, 2179, 2051, 1923, 1795, 1667,
        1539, 1411, 1283, 1155, 1027, 899, 771, 643, 515, 387, 260, 132


};


const uint16_t quadrada[NAMOSTRAS] = {

        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
        4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
        4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
        4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095

};

uint16_t sinal_quadrada[NAMOSTRAS];

uint16_t sinal_triangular[NAMOSTRAS];

uint16_t sinal_senoide[NAMOSTRAS];
*/

typedef enum {SIN=0,TRI,SQR,SAW,NONE} sinal_t;

uint32_t pino[4]={GPIO_PIN_2,GPIO_PIN_3,GPIO_PIN_4,GPIO_PIN_4};

const uint16_t modSinal[4][NAMOSTRAS] = {
										{2047,2247, 2446, 2641,
										2830, 3011, 3184, 3345,
										3494, 3629, 3749, 3852,
										3938, 4005, 4054, 4084,
										4093, 4084, 4054, 4005,
										3938, 3852, 3749, 3629,
										3494, 3345, 3184, 3011,
										2830, 2641, 2446, 2247,
										2047, 1846, 1647, 1452,
										1263, 1082, 909, 748,
										599, 464, 344, 241,
										155, 88, 39, 9,
										0, 9, 39, 88,
										155, 241, 344, 464,
										599, 748, 909, 1082,
										1263, 1452, 1647, 1846},
									   {0, 127, 255, 383, 511, 639, 767, 895, 1023, 1151, 1279, 1407,
										1535, 1663, 1791, 1919, 2046, 2174, 2302, 2430, 2558, 2686,
										2814, 2942, 3070, 3198, 3326, 3454, 3582, 3710, 3838, 3965,
										4093, 4221, 3842, 3714, 3586, 3458, 3330, 3202, 3074, 2946,
										2818, 2690, 2562, 2434, 2306, 2179, 2051, 1923, 1795, 1667,
										1539, 1411, 1283, 1155, 1027, 899, 771, 643, 515, 387, 260, 132
									   	 },
									   {0, 0, 0, 0, 0, 0, 0, 0,
										0, 0, 0, 0, 0, 0, 0, 0,
										0, 0, 0, 0, 0, 0, 0, 0,
										0, 0, 0, 0, 0, 0, 0, 0,
										4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
										4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
										4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
										4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095
									   },
										 {0, 127, 255, 383, 511, 639, 767, 895, 1023, 1151, 1279, 1407,
										1535, 1663, 1791, 1919, 2046, 2174, 2302, 2430, 2558, 2686,
										2814, 2942, 3070, 3198, 3326, 3454, 3582, 3710, 3838, 3965,
										4093, 4221,	0, 0, 0, 0, 0, 0, 0, 0,0,0,
										0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0}
};

const uint16_t zero[2]={1,1};

uint16_t sinal[NAMOSTRAS];

sinal_t sinAtual;

uint16_t escala=0,nova_escala=0;

float escala_amplitude;

uint16_t valor_pot2 = 0;

float fator_divisorio = 10/4096;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM10) {
////////////////////ENVIANDO PELA SERIAL///////////////////////////

		sprintf(msg, "\rMedia1 = %4i | Media2 = %4i", media[0], media[1]);
		HAL_UART_Transmit_DMA(&huart2, msg, strlen(msg));
		msg[0] = '\0';

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);

//////////////////DEBOUNCE E DETECCAO DE BORDA///////////////////

		bt1[AGORA] = HAL_GPIO_ReadPin(BOT1_GPIO_Port, BOT1_Pin);

		if (bt1[AGORA] == 1 && bt1[ANTES] == 0) {

			bsubida = 1;
			bdescida = 0;
			comando++;

		}

		if (bt1[AGORA] == 0 && bt1[ANTES] == 1) {

			bdescida = 1;
			bsubida = 0;

		}

		bt1[ANTES] = bt1[AGORA];  /////// DETC BOT 1

		if (comando >=5 )
			comando = 0;

	} //////////////FIM DEBOUNCE E DETEC.BORDA

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	int i;
	media[0] = media[1] = 0;

	for (i = 0; i < TAM_MEDIDAS; i++) {

		media[0] += medidas[i][0]; // escreve os valores de uma das medidas na coluna 0 e na linha referente a i. ate ler toda a matriz
		media[1] += medidas[i][1]; // escreve os valores da segunda medida na coluna 2

	}

	for (i = 0; i < 2; i++){

		media[i] /= TAM_MEDIDAS;

	}
	if(media[1] < 50)
		media[1] = 50;
	if(media[1] > 4095)
		media[1] = 4095;

	__HAL_TIM_SET_AUTORELOAD(&htim6, media[0]);

	if (media[1]>escala)
		nova_escala=media[1]-escala;
	else
		nova_escala=escala-media[1];

	if(nova_escala > 4 )
		{
		nova_escala=media[1];
		}
	else
		nova_escala=escala;
}

void escala_sinal(void)
{
	escala_amplitude =((float)escala/4095.0);

	for(int i= 0; i < NAMOSTRAS; i++){
			sinal[i] = (uint16_t)(((float)modSinal[sinAtual][i]) * escala_amplitude)  ; //FATOR DE ESCALA;
		}

}

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
  MX_TIM10_Init();
  MX_TIM3_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  	 sinAtual=SIN;
 // 	HAL_DMA_Start (&hdma_memtomem_dma2_stream1,(uint32_t *)modSinal[sinAtual],(uint32_t *) sinal,NAMOSTRAS); // transferencia de memoria pra memoria

	HAL_TIM_Base_Start_IT(&htim10);
	HAL_TIM_Base_Start(&htim6);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)medidas, TAM_MEDIDAS * 2); // (qual adc eh, onde armazena, tamanho)


//	__HAL_TIM_SET_AUTORELOAD(&htim6, media[0]);


	//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, media[1]);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		// Escala sinal quando necessario
		if(escala!=nova_escala)
		{
		escala=nova_escala;
		escala_sinal();
		}

// Gerencia saida
		switch (comando) {

		case 0:

			HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
			//HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
			HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1,zero,2,DAC_ALIGN_12B_R);
			HAL_GPIO_WritePin(GPIOC, (GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4), GPIO_PIN_RESET);
			/*
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
			*/
			sinAtual=NONE;
			break;
			//sprintf(msg, "\r ESTADO = DESLIGADO");
		default :
			if((sinAtual+1)!=comando)
			{
				sinAtual=comando-1;
				HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
			  	escala_sinal();
			  	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1,sinal,NAMOSTRAS,DAC_ALIGN_12B_R);
			  	HAL_GPIO_WritePin(GPIOC, (GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4), GPIO_PIN_RESET);
			  	/*
			  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
			  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
			  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
			  	 */
			  	HAL_GPIO_WritePin(GPIOC, pino[sinAtual], GPIO_PIN_SET);

				}
    /* USER CODE END WHILE */
		}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
	__disable_irq();
	while (1) {
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
