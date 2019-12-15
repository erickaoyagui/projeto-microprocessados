/*
 * clock.c
 *
 *  Created on: 14 de dez de 2019
 *      Author: erick
 */
#include "clock.h"

void verifyTime(aData *myTime) {
	if (myTime->hours >= 24) {
		myTime->hours = 0;
	}
	if (myTime->minutes >= 60) {
		myTime->hours++;
		myTime->minutes = 0;
	}
	if (myTime->seconds >= 60) {
		//setAdcState(ADC_STATE_SHOOT_ADC_CONVERSION);
		myTime->minutes++;
		myTime->seconds = 0;
	}
}

void incrementSeconds(aData *myTime) {
	myTime->seconds++;
	verifyTime(myTime);
}

int getUniHours(aData *myTime){
	return myTime->hours % 10;
}


int getDezHours(aData *myTime){
	return myTime->hours / 10;
}


int getUniMinutes(aData *myTime){
	return myTime->minutes % 10;
}

int getDezMinutes(aData *myTime){
	return myTime->minutes / 10;
}
