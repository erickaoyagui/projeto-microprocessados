/*
 * st_adc_state.h
 *
 *  Created on: 12 de dez de 2019
 *      Author: victor
 */

#ifndef E_ADC_STATE_H_
#define E_ADC_STATE_H_

typedef enum e_adc_state {
  ADC_STATE_IDLE,
  ADC_STATE_SHOOT_ADC_CONVERSION,
  ADC_STATE_CONVERT_TO_MILI_VOLTS,
  ADC_STATE_MAX_STATE
} eAdcState;

#endif /* E_ADC_STATE_H_ */
