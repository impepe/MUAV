/******************************************
file name:	funcs.h
description:	header file of funcs.h
created by:	pepe
date:		2023/7/29
******************************************/
#ifndef __FUNCS_H__
#define __FUNCS_H__

#define TTY_PATH            "/dev/tty"
#define STTY_US             "stty raw -echo -F "
#define STTY_DEF            "stty -raw echo -F "

#define kb_w    119
#define kb_a    97
#define kb_s    115
#define kb_d    100
#define kb_ent  13
#define kb_5    53
#define kb_1    49
#define kb_2    50
#define kb_3    51

#define kb_P1 0
#define kb_I1 1
#define kb_D1 2
#define kb_P2 3
#define kb_I2 4
#define kb_D2 5
#define kb_BS 6

//initialize PID controllers
void pid_init();

//save three data in a txt file
void save_data(float data1,float data2,float data3);

//limit the duty cycle of drive PWM signals
static void output_limit(struct UAV *UAV);

//distribute controllers' output to each motor
//x-type:
//m4 m2
//m1 m3
void power_distribute(struct UAV *UAV);

//check keyboard input
static int scanKeyboard();

//change parameters due to keyboard input
static void paraChange(struct refData *kbData, int paraSelect, float num);

//get reference value from keyboard
void ref_kb(struct kbData *kbData);

void show_state(struct UAV *UAV, struct kbData *kbData);

#endif
