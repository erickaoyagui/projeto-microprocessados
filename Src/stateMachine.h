/*
 * stateMachine.h
 *
 *  Created on: 7 de dez de 2019
 *      Author: erick
 */

#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

typedef enum state_machine {
	MACHINE_STATE_CLOCK,
	MACHINE_STATE_LDR_LOCAL,
	MACHINE_STATE_LDR_NOT_LOCAL,
	MACHINE_STATE_MAX_STATE
} eMachineState;

typedef enum set_clock_select {
	CLOCK_EDIT_MODE_RUN,
	CLOCK_EDIT_MODE_HOUR,
	CLOCK_EDIT_MODE_MINUTES,
	CLOCK_EDIT_MODE_MAX_VALUE
} eClockEditMode;

#endif /* STATEMACHINE_H_ */
