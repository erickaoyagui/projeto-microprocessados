/*
 * clock.c
 *
 *  Created on: 14 de dez de 2019
 *      Author: erick
 */
#include "clock.h"

stHour tempTime = { 0, 0, 0 };

void verifyTime(stHour *myTime) {
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

void verifyTempTime(){
	if (tempTime.hours >= 24) {
		tempTime.hours = 0;
	}
	if (tempTime.minutes >= 60) {
		tempTime.minutes = 0;
	}
}

void incrementSeconds(stHour *myTime) {
	myTime->seconds++;
	verifyTime(myTime);
}

int getUniHours(stHour *myTime){
	return myTime->hours % 10;
}


int getDezHours(stHour *myTime){
	return myTime->hours / 10;
}


int getUniMinutes(stHour *myTime){
	return myTime->minutes % 10;
}

int getDezMinutes(stHour *myTime){
	return myTime->minutes / 10;
}

void setHours (stHour *myTime, int hours){
	myTime->hours = hours;
}

void setMinutes (stHour *myTime, int minutes){
	myTime->minutes = minutes;
}

int getHours (stHour *myTime){
	return myTime->hours;
}

int getMinutes (stHour *myTime){
	return myTime->minutes;
}

void setTempTimeToMyTime(stHour *myTime){
	myTime->hours = tempTime.hours;
	myTime->minutes = tempTime.minutes;
	myTime->seconds = 0;
}

void resetTempTime(){
	tempTime.hours = 0;
	tempTime.minutes = 0;
}

void incrementTempTimeMinutes(){
	tempTime.minutes++;
}

void incrementTempTimeHours(){
	tempTime.hours++;
}

int getTempTimeMinutes(){
	return tempTime.minutes;
}

int getTempTimeHours(){
	return tempTime.hours;
}

int getTempTimeUniHours(){
	return tempTime.hours % 10;
}

int getTempTimeDezHours(){
	return tempTime.hours / 10;
}

int getTempTimeUniMinutes(){
	return tempTime.minutes % 10;
}

int getTempTimeDezMinutes(){
	return tempTime.minutes / 10;
}
