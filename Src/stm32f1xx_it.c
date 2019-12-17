/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f1xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
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
#include "main.h"
#include "stm32f1xx_it.h"
#include "stateMachine.h"
#include "e_adc_state.h"
#include "clock.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DT_DEBOUNCING 250              // tempo delay para debouncing
#define MAX_MODO_OPER 2                // kte p/ testar val max modo_oper
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile eAdcState adcState = ADC_STATE_SHOOT_ADC_CONVERSION;       // VAR modo_oper LOCAL
volatile uint32_t tIN_IRQ1 = 0;        // tempo entrada na �ｿｽltima IRQ6
volatile uint32_t tIN_IRQ2 = 0;        // tempo entrada na �ｿｽltima IRQ6
volatile uint32_t tIN_IRQ3 = 0;        // tempo entrada na �ｿｽltima IRQ6
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

eMachineState machineState = MACHINE_STATE_CLOCK;
eResetClock resetClock = RESET_CLOCK;
eSetClockSelect setClockSelect = SET_HOURS;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**aumenta numero do relogio
 * @brief This function handles EXTI line1 interrupt.
 */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
	if ((HAL_GetTick() - tIN_IRQ1) > DT_DEBOUNCING)
		  {
		    tIN_IRQ1 = HAL_GetTick();                // tIN (ms) da �ｿｽltima IRQ1
		    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == 0)
		    {
		    	switch (setClockSelect){
		    		case SET_HOURS:
		    			incrementTempTimeHours();
		    		break;
		    		case SET_MINUTES:
		    			incrementTempTimeMinutes();
		    		break;
		    	}
		    	verifyTempTime();
		    }
		  }
  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/** entra no modo de edicao de horas
 *
 */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
  if ((HAL_GetTick() - tIN_IRQ2) > DT_DEBOUNCING)
  {
    tIN_IRQ2 = HAL_GetTick();                // tIN (ms) da �ｿｽltima IRQ1
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET)
    {
      ++setClockSelect;
      if (setClockSelect > START_CLOCK)
        setSetClockSelect(SET_HOURS);
    }
    if (setClockSelect == START_CLOCK){
      setResetClock(NOT_RESET_CLOCK);
    }
  }
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/** Altera modo de operacao e caso seja precionado por 3s zera relogio
 *
 */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
  if ((HAL_GetTick() - tIN_IRQ3) > DT_DEBOUNCING)
  {
    tIN_IRQ3 = HAL_GetTick();                // tIN (ms) da �ｿｽltima IRQ1
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0)
    {
      ++machineState;
      if (machineState == MACHINE_STATE_LDR_LOCAL) {
        setAdcState(ADC_STATE_SHOOT_ADC_CONVERSION);
      }
      if (machineState >= MACHINE_STATE_MAX_STATE) {
        machineState = MACHINE_STATE_CLOCK;
      }
    }
  }

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
 * @brief This function handles ADC1 and ADC2 global interrupts.
 */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
 * @brief This function handles USART1 global interrupt.
 */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/* USER CODE BEGIN 1 */
// fn que permite setar o valor da var modo_oper


void setAdcState(eAdcState state)
{
  // OBS: sessao critica, desabilitamos todas as IRQs p/ atualizar var
  __disable_irq();                     // desabilita IRQs
  if (state > ADC_STATE_MAX_STATE)                // se x maior MAX permitido
  {
    adcState = ADC_STATE_MAX_STATE - 1;           // set apenas com max
  }
  else if (state < ADC_STATE_IDLE)                      // se x menor que 0
  {
    adcState = ADC_STATE_IDLE;                       // set com 0
  }
  else                                // valor no intervalo 0-MAX
  {
    adcState = state;                       // modifica adcState
  }
  __enable_irq();                      // volta habilitar IRQs
}

// fn que qpenas retorna o valor da var adcState
eAdcState getAdcState(void)
{
  static int x;                        // var local recebe modo_oper
  // OBS: se�ｿｽ�ｿｽo cr�ｿｽtica, desabilitamos todas as IRQs p/ atualizar var
  __disable_irq();                     // desabilita IRQs
  x = adcState;                       // faz x = adcState
  __enable_irq();                      // volta habilitar IRQs
  return x;                            // retorna x (=adcState)
}

void setMachineState(eMachineState state)
{
  // OBS: sessao critica, desabilitamos todas as IRQs p/ atualizar var
  __disable_irq();                     // desabilita IRQs

  if (state > MACHINE_STATE_LDR_NOT_LOCAL)                // se x maior MAX permitido
  {
    machineState = MACHINE_STATE_LDR_NOT_LOCAL;           // set apenas com max
  }
  else if (state < MACHINE_STATE_CLOCK)                      // se x menor que 0
  {
	  machineState = MACHINE_STATE_CLOCK;                       // set com 0
  }
  else                                // valor no intervalo 0-MAX
  {
	  machineState = state;                       // modifica adcState
  }
  __enable_irq();                      // volta habilitar IRQs
}

// fn que qpenas retorna o valor da var adcState
eMachineState getMachineState(void)
{
  static int x;                        // var local recebe modo_oper
  // OBS: se�ｿｽ�ｿｽo cr�ｿｽtica, desabilitamos todas as IRQs p/ atualizar var
  __disable_irq();                     // desabilita IRQs
  x = machineState;                       // faz x = adcState
  __enable_irq();                      // volta habilitar IRQs
  return x;                            // retorna x (=modo_oper)
}

void setResetClock(eResetClock rClock)
{
  // OBS: sessao critica, desabilitamos todas as IRQs p/ atualizar var
  __disable_irq();                     // desabilita IRQs

  if (rClock > NOT_RESET_CLOCK)                // se x maior MAX permitido
  {
    resetClock = RESET_CLOCK;           // set apenas com max
  }
  else if (rClock < RESET_CLOCK)                      // se x menor que 0
  {
	  resetClock = NOT_RESET_CLOCK;                       // set com 0
  }
  else                                // valor no intervalo 0-MAX
  {
	  resetClock = rClock;                       // modifica adcState
  }
  __enable_irq();                      // volta habilitar IRQs
}

eResetClock getResetClock(void)
{
  static int x;                        // var local recebe modo_oper
  // OBS: se�ｿｽ�ｿｽo cr�ｿｽtica, desabilitamos todas as IRQs p/ atualizar var
  __disable_irq();                     // desabilita IRQs
  x = resetClock;                       // faz x = adcState
  __enable_irq();                      // volta habilitar IRQs
  return x;                            // retorna x (=modo_oper)
}

void setSetClockSelect(eSetClockSelect clockSelect)
{
  // OBS: sessao critica, desabilitamos todas as IRQs p/ atualizar var
  __disable_irq();                     // desabilita IRQs

  if (clockSelect > SET_MINUTES)                // se x maior MAX permitido
  {
    setClockSelect = SET_HOURS;           // set apenas com max
  }
  else if (clockSelect < SET_HOURS)                      // se x menor que 0
  {
	  setClockSelect = SET_MINUTES;                       // set com 0
  }
  else                                // valor no intervalo 0-MAX
  {
	  setClockSelect = clockSelect;                       // modifica adcState
  }
  __enable_irq();                      // volta habilitar IRQs
}

// fn que qpenas retorna o valor da var adcState
eResetClock getSetClockSelect(void)
{
  static int x;                        // var local recebe modo_oper
  // OBS: se�ｿｽ�ｿｽo cr�ｿｽtica, desabilitamos todas as IRQs p/ atualizar var
  __disable_irq();                     // desabilita IRQs
  x = setClockSelect;                       // faz x = adcState
  __enable_irq();                      // volta habilitar IRQs
  return x;                            // retorna x (=modo_oper)
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
