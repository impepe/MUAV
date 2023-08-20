/******************************************
file name:	filter.h
description:	header file of filter.c
created by:	pepe
date:		2023/7/2
******************************************/
#ifndef __FILTER_H__
#define __FILTER_H__

#define K1	0.65	//parameter of 1st order low pass filter. K1 = T/(T+(1/2*pi*fc)), fc = 30 Hz
#define WIN_LEN	8	//parameter of window average filter

// first order low pass filter
float lowPass_filter(float newData, float oldData);

//window average filter
float win_average(float newData, float *sum, float (*datalist)[WIN_LEN]);

//second order Butterworth filter
float Butterworth_filter();


#endif
