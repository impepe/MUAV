/******************************************
file name:	filter.c
description:	defination of different filters
created by:	pepe
date:		2023/7/2
******************************************/

#include "../include/filter.h"

//first order low pass filter

float lowPass_filter(float newData, float oldData)
{
	return K1*newData+(1-K1)*oldData;
}

//window average filter
float win_average(float newData,float *sum, float (*datalist)[WIN_LEN])
{
	static int i=0;
	
	*sum += newData;
	*sum -= (*datalist)[i];
	(*datalist)[i] = newData;
	if (i<WIN_LEN-1)	i++;
	else			i=0;
	
	return *sum/WIN_LEN;
}

//second order Butterworth filter
float Butterworth_filter()
{
	return 0.;
}
