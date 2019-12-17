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
} stHour;

void verifyTime(stHour *myTime);

void incrementSeconds();

int getUniHours(stHour *myTime);

int getDezHours(stHour *myTime);

int getUniMinutes(stHour *myTime);

int getDezMinutes(stHour *myTime);

int getHours (stHour *myTime);

int getMinutes (stHour *myTime);

void setHours (stHour *myTime, int hours);

void setMinutes (stHour *myTime, int minutes);

void setTempTimeToMyTime(stHour *myTime);

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
