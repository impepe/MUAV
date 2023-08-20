/******************************************
file name:	gpio.h
description:	header file of gpio.c
created by:	pepe
date:		2023/7/19
******************************************/
#ifndef __GPIO_H__
#define __GPIO_H__

//counter for hardware PWM at 10kHz
void PWM_Count();

//create output soft PWM at 100Hz
void myPwmWrite(int dutyCycle1, int dutyCycle2, int dutyCycle3, int dutyCycle4);

//initialize output PWM pins
void myPwmInit();

void softPWMwrite(float m1, float m2, float m3, float m4);

//initialize PWM used as timer
void setPWM_timer();

#endif
