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

typedef enum reset_clock {
	RESET_CLOCK,
	NOT_RESET_CLOCK
} eResetClock;

typedef enum set_clock_select {
	SET_HOURS,
	SET_MINUTES
} eSetClockSelect;

#endif /* STATEMACHINE_H_ */
