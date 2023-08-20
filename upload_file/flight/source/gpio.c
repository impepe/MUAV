/******************************
file name:	gpio.c
discribe:	source file for raspberry pi gpio control
created by:	pepe
******************************/
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include "../include/constants.h"
#include "../include/gpio.h"


static int PWM_counter = 0;
int dutyCycle1,dutyCycle2,dutyCycle3,dutyCycle4;
void myPwmWrite()
{
    if(PWM_counter<dutyCycle1)  digitalWrite(PWM_pin_1,HIGH);
    else                        digitalWrite(PWM_pin_1,LOW);
    if(PWM_counter<dutyCycle2)  digitalWrite(PWM_pin_2,HIGH);
    else                        digitalWrite(PWM_pin_2,LOW);
    if(PWM_counter<dutyCycle3)  digitalWrite(PWM_pin_3,HIGH);
    else                        digitalWrite(PWM_pin_3,LOW);
    if(PWM_counter<dutyCycle4)  digitalWrite(PWM_pin_4,HIGH);
    else                        digitalWrite(PWM_pin_4,LOW);
}

int PWM_Timer = 0;
static void PWM_ISR()
{
    if(PWM_counter<99)  PWM_counter++;
	else				
	{
		if(PWM_Timer == DISABLE)	PWM_Timer = ENABLE;
		else						PWM_Timer = DISABLE;
		PWM_counter=0;
	}
	myPwmWrite();
}

void myPwmInit()
{
	pullUpDnControl(PWM_pin_1,PUD_UP);
	pullUpDnControl(PWM_pin_2,PUD_UP);
	pullUpDnControl(PWM_pin_3,PUD_UP);
	pullUpDnControl(PWM_pin_4,PUD_UP);

	pinMode(PWM_pin_1,OUTPUT);
 	pinMode(PWM_pin_2,OUTPUT);
 	pinMode(PWM_pin_3,OUTPUT);
 	pinMode(PWM_pin_4,OUTPUT);

    wiringPiISR(PWM_TIM_PIN, INT_EDGE_FALLING, &PWM_ISR);

    pinMode(PWM_TIM_PIN,PWM_OUTPUT);   //create hardware PWM at 10kHz
    pwmSetMode(PWM_MODE_MS);
	pwmSetClock(DIVISOR);
	pwmSetRange(PWM_RANGE);
	pwmWrite(PWM_TIM_PIN,PWM_RANGE/2);
}
