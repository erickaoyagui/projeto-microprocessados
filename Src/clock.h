/*
 * clock.h
 *
 *  Created on: 14 de dez de 2019
 *      Author: erick
 */

#ifndef CLOCK_H_
#define CLOCK_H_

typedef struct { // struct usado para guardar as variaveis de horas, minutos e segundos do relogio
	int hours;
	int minutes;
	int seconds;
} aData;

void verifyTime(aData *myTime);

void incrementSeconds(aData *myTime);

int getUniHours(aData *myTime);

int getDezHours(aData *myTime);

int getUniMinutes(aData *myTime);

int getDezMinutes(aData *myTime);

void setHours (aData *myTime, int hours);

void setMinutes (aData *myTime, int minutes);

void setTempTimeToMyTime(aData *myTime);

void resetTempTime();

void incrementTempTimeMinutes();

void incrementTempTimeHours();

int getTempTimeMinutes();

int getTempTimeHours();

int getTempTimeUniHours();

int getTempTimeDezHours();

int getTempTimeUniMinutes();

int getTempTimeDezMinutes();

#endif /* CLOCK_H_ */
