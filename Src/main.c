/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2019 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include <stm32f103xb.h>
#include <stm32f1xx.h>
#include <stm32f1xx_hal_adc.h>
#include <stm32f1xx_hal_adc_ex.h>
#include <stm32f1xx_hal_cortex.h>
#include <stm32f1xx_hal_def.h>
#include <stm32f1xx_hal_flash.h>
#include <stm32f1xx_hal_gpio.h>
#include <stm32f1xx_hal_rcc.h>
#include <stm32f1xx_hal_rcc_ex.h>
#include <stm32f1xx_hal_uart.h>
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mx_prat_05_funcoes.h"   // header do arqv das funcoes do programa
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum state_machine {
	CLOCK, LDR_LOCAL, LDR_NOT_LOCAL
} e_state_machine;

typedef struct { // struct usado para guardar as variaveis de horas, minutos e segundos do relogio
	int hours;
	int minutes;
	int seconds;
} aData;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DT_VARRE  5             // inc varredura a cada 5 ms (~200 Hz)
#define DIGITO_APAGADO 0x10    // kte valor p/ apagar um d�ｿｽgito no display
#define SEGUNDO 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void set_modo_oper(int);             // seta modo_oper (no stm32f1xx_it.c)
int get_modo_oper(void);             // obt�ｿｽm modo_oper (stm32f1xx_it.c)
void reset_pin_GPIOs(void);         // reset pinos da SPI
void serializar(int ser_data);       // prot fn serializa dados p/ 74HC595
int16_t conv_7_seg(int NumHex);      // prot fn conv valor --> 7-seg

void verifyTime(aData *myTime) { // Se souber colocar em outro arquivo .c ajudaria
	if (myTime->hours >= 24) {
		myTime->hours = 0;
	}
	if (myTime->minutes >= 60) {
		myTime->hours++;
		myTime->minutes = 0;
	}
	if (myTime->seconds >= 60) {
		myTime->minutes++;
		myTime->seconds = 0;
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t val_adc = 0;                 // var global: valor lido no ADC
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	// vars e flags de controle do programa no superloop...
	int milADC = 0,                    // ini decimo de seg
			cenADC = 0,                      // ini unidade de seg
			dezADC = 0,                      // ini dezena de seg
			uniADC = 0;                      // ini unidade de minuto
	int dezHours = 0, uniHours = 0, dezMinutes = 0, uniMinutes = 0, dezSeconds =
			0, uniSeconds = 0;

	int16_t val7seg = 0x00FF,          // inicia 7-seg com 0xF (tudo apagado)
			serial_data = 0x01FF;           // dado a serializar (dig | val7seg)

	uint32_t miliVolt = 0x0,           // val adc convertido p/ miliVolts
			tIN_varre = 0,                  // registra tempo �ｿｽltima varredura
			tIN_relogio = 0;

	aData actualTime = { 0, 0, 0 };
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
	MX_ADC1_Init();
	MX_USART1_UART_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	/* USER CODE BEGIN 2 */
	// inicializa a SPI (pinos 6,9,10 da GPIOB)
	reset_pin_GPIOs();
	// var de estado que controla a varredura (qual display �ｿｽ mostrado)
	static enum {
		DIG_UNI, DIG_DEC, DIG_CENS, DIG_MILS
	} sttVARRE = DIG_UNI;
	e_state_machine stateMachine = CLOCK;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if ((HAL_GetTick() - tIN_relogio) > SEGUNDO) {
			tIN_relogio = HAL_GetTick();
			actualTime.seconds++;
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_15);
			verifyTime(&actualTime);
			uniHours = actualTime.hours % 10;
			dezHours = actualTime.hours / 10;
			uniMinutes = actualTime.minutes % 10;
			dezMinutes = actualTime.minutes / 10;
			uniSeconds = actualTime.seconds % 10;
			dezSeconds = actualTime.seconds / 10;

		}

		switch (stateMachine) {
		case CLOCK:
			if ((HAL_GetTick() - tIN_varre) > DT_VARRE) // se ++0,1s atualiza o display
			{
				tIN_varre = HAL_GetTick();  // salva tIN p/ prox tempo varredura
				switch (sttVARRE)
				// teste e escolha de qual DIG vai varrer
				{
				case DIG_MILS: {
					sttVARRE = DIG_CENS;           // ajusta p/ prox digito
					serial_data = 0x0008;          // display #1
					val7seg = conv_7_seg(uniMinutes);
					break;
				}
				case DIG_CENS: {
					sttVARRE = DIG_DEC;            // ajusta p/ prox digito
					serial_data = 0x00004;         // display #2
					val7seg = conv_7_seg(dezMinutes);
					break;
				}
				case DIG_DEC: {
					sttVARRE = DIG_UNI;            // ajusta p/ prox digito
					serial_data = 0x0002;          // display #3
					val7seg = conv_7_seg(uniHours);
					val7seg &= 0x7FFF;            // liga o ponto decimal
					break;
				}
				case DIG_UNI: {
					sttVARRE = DIG_MILS;           // ajusta p/ prox digito
					serial_data = 0x0001;          // display #3
					val7seg = conv_7_seg(dezHours);
					break;
				}
				}  // fim case
				tIN_varre = HAL_GetTick(); // tmp atual em que fez essa varredura
				serial_data |= val7seg;    // OR com val7seg = dado a serializar
				serializar(serial_data); // serializa dado p/74HC595 (shift reg)
			}
			break;
		case LDR_LOCAL:
			// tarefa #1: se (modo_oper=1) faz uma convers�ｿｽo ADC
			if (get_modo_oper() == 1) {
				// dispara por software uma convers�ｿｽo ADC
				set_modo_oper(0);                // muda modo_oper p/ 0
				HAL_ADC_Start_IT(&hadc1);  // dispara ADC p/ convers�ｿｽo por IRQ
			}

			//tarefa #2: depois do IRQ ADC, converte para mVs (decimal, p/ 7-seg)
			if (get_modo_oper() == 2)      // entra qdo valor val_adc atualizado
					{
				// converter o valor lido em decimais p/ display
				miliVolt = val_adc * 3300 / 4095;
				uniADC = miliVolt / 1000;
				dezADC = (miliVolt - (uniADC * 1000)) / 100;
				cenADC = (miliVolt - (uniADC * 1000) - (dezADC * 100)) / 10;
				milADC = miliVolt - (uniADC * 1000) - (dezADC * 100)
						- (cenADC * 10);
				set_modo_oper(0);                // zera var modo_oper
			}

			// tarefa #3: qdo milis() > DELAY_VARRE ms, desde a �ｿｽltima mudan�ｿｽa
			if ((HAL_GetTick() - tIN_varre) > DT_VARRE) // se ++0,1s atualiza o display
			{
				tIN_varre = HAL_GetTick();  // salva tIN p/ prox tempo varredura
				switch (sttVARRE)
				// teste e escolha de qual DIG vai varrer
				{
				case DIG_MILS: {
					sttVARRE = DIG_CENS;           // ajusta p/ prox digito
					serial_data = 0x0008;          // display #1
					val7seg = conv_7_seg(milADC);
					break;
				}
				case DIG_CENS: {
					sttVARRE = DIG_DEC;            // ajusta p/ prox digito
					serial_data = 0x00004;         // display #2
					if (cenADC > 0 || dezADC > 0 || uniADC > 0) {
						val7seg = conv_7_seg(cenADC);
					} else {
						val7seg = conv_7_seg(DIGITO_APAGADO);
					}
					break;
				}
				case DIG_DEC: {
					sttVARRE = DIG_UNI;            // ajusta p/ prox digito
					serial_data = 0x0002;          // display #3
					if (dezADC > 0 || uniADC > 0) {
						val7seg = conv_7_seg(dezADC);
					} else {
						val7seg = conv_7_seg(DIGITO_APAGADO);
					}
					break;
				}
				case DIG_UNI: {
					sttVARRE = DIG_MILS;           // ajusta p/ prox digito
					serial_data = 0x0001;          // display #3
					if (uniADC > 0) {
						val7seg = conv_7_seg(uniADC);
						val7seg &= 0x7FFF;            // liga o ponto decimal
					} else {
						val7seg = conv_7_seg(DIGITO_APAGADO);
					}
					break;
				}
				}  // fim case
				tIN_varre = HAL_GetTick(); // tmp atual em que fez essa varredura
				serial_data |= val7seg;    // OR com val7seg = dado a serializar
				serializar(serial_data); // serializa dado p/74HC595 (shift reg)
			}  // -- fim da tarefa #3 - varredura do display

			break;
		case LDR_NOT_LOCAL:
			break;
		}

	}    // -- fim do loop infinito

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
	/* EXTI1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	/* EXTI3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	/* EXTI2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/**Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

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
	huart1.Init.BaudRate = 9600;
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
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			GPIO_PIN_0 | GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14
					| GPIO_PIN_15 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9,
			GPIO_PIN_SET);

	/*Configure GPIO pins : PA1 PA2 PA3 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB10 PB12 PB13
	 PB14 PB15 PB5 PB6
	 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_13
			| GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// fn que atende ao callback da ISR do conversor ADC1
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc->Instance == ADC1) {
		val_adc = HAL_ADC_GetValue(&hadc1);  // capta valor adc
		set_modo_oper(2);
		;                  // alterou valor lido
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
