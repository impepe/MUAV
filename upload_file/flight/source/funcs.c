/******************************************
file name:	funcs.c
description:	some common functions
created by:	pepe
date:		2023/7/29
******************************************/

#include <stdio.h>
#include <stdlib.h>
#include "../include/funcs.h"
#include "../include/controller.h"
#include "../include/constants.h"

extern struct PID angVx_pid;
extern struct PID angVy_pid;
extern struct PID angVz_pid;

extern struct PID angx_pid;
extern struct PID angy_pid;
extern struct PID angz_pid;

extern struct PID vlx_pid;
extern struct PID vly_pid;
extern struct PID vlz_pid;

extern struct PID posx_pid;
extern struct PID posy_pid;
extern struct PID posz_pid;

//initialize PID controllers
void pid_init()
{
	//para: struct,kp,ki,kd,output limit
	pid_set(&angx_pid,ang_Kp,ang_Ki,ang_Kd,ang_limt);	    //limit: +-5
	pid_set(&angy_pid,ang_Kp,ang_Ki,ang_Kd,ang_limt);
	pid_set(&angz_pid,angz_Kd,angz_Ki,angz_Kd,ang_limt);
	pid_clean(&angx_pid,&angy_pid,&angz_pid);

	pid_set(&angVx_pid,angV_Kp,angV_Ki,angV_Kd,angV_limt);	//no limitation
	pid_set(&angVy_pid,angV_Kp,angV_Ki,angV_Kd,angV_limt);
	pid_set(&angVz_pid,angVz_Kd,angVz_Ki,angVz_Kd,angV_limt);
    pid_clean(&angx_pid,&angy_pid,&angz_pid);

	pid_set(&vlx_pid,0.,0.,0.,0.5);	//about 30 degrees
	pid_set(&vly_pid,0.,0.,0.,0.5);
	pid_set(&vlz_pid,0.,0.,0.,0.5);
    pid_clean(&vlx_pid,&vly_pid,&vlz_pid);

	pid_set(&posx_pid,0.,0.,0.,0.);	//ESO drive PWM duty cycle range: 10%-20%
	pid_set(&posy_pid,0.,0.,0.,0.);
	pid_set(&posz_pid,0.,0.,0.,0.);
    pid_clean(&posx_pid,&posy_pid,&posz_pid);
}

//save three data in a txt file
void save_data(float data1,float data2,float data3)
{
    static int i=0;

    static float list1[DATA_LEN];
    static float list2[DATA_LEN];
    static float list3[DATA_LEN];

	list1[i] = data1;
	list2[i] = data2;
	list3[i] = data3;
	
	i++;
	if (i>=DATA_LEN)
	{
		FILE *file;
		file = fopen("data.txt","w");
		for (int j=0;j<i;j++)
		{
			fprintf(file,"%f	",list1[j]);
			fprintf(file,"%f	",list2[j]);
			fprintf(file,"%f\n",list3[j]);
		}
		fclose(file);
		printf("\ndata saved\n");
		exit(0);
	}
}


//limit the duty cycle of drive PWM signals
static void output_limit(struct UAV *UAV)
{
	if(UAV->drive.m1 > 20)		UAV->drive.m1 = 20;
	else if (UAV->drive.m1<10)	UAV->drive.m1 = 10;
	else						{}
	if(UAV->drive.m2 > 20)		UAV->drive.m2 = 20;
	else if (UAV->drive.m2<10)	UAV->drive.m2 = 10;
	else						{}
	if(UAV->drive.m3 > 20)		UAV->drive.m3 = 20;
	else if (UAV->drive.m3<10)	UAV->drive.m3 = 10;
	else						{}
	if(UAV->drive.m4 > 20)		UAV->drive.m4 = 20;
	else if (UAV->drive.m4<10)	UAV->drive.m4 = 10;
	else						{}
}


//distribute controllers' output to each motor
//x-type:
//m4 m2
//m1 m3
void power_distribute(struct UAV *UAV)
{
	UAV->drive.m1 = UAV->drive.base + UAV->drive.y - UAV->drive.x + UAV->drive.z;
   	UAV->drive.m2 = UAV->drive.base - UAV->drive.y + UAV->drive.x + UAV->drive.z;
    UAV->drive.m3 = UAV->drive.base + UAV->drive.y + UAV->drive.x - UAV->drive.z;
    UAV->drive.m4 = UAV->drive.base - UAV->drive.y - UAV->drive.x - UAV->drive.z;

	output_limit(UAV);
}

//check keyboard input
static int scanKeyboard()
{
    fd_set rfds;
    int ch = 0;

    FD_ZERO(&rfds);
    FD_SET(0, &rfds);

	struct timeval tv;
	tv.tv_sec  = 0;
	tv.tv_usec = 0;

    if (select(1, &rfds, NULL, NULL, &tv) > 0)  ch = getchar();

    return ch;
}

#define pidDebug1    0.001
#define pidDebug2    0.0001
#define baseDebug    1

//change parameters due to keyboard input
static void paraChange(struct kbData *kbData, int paraSelect, int sign)
{
    switch (paraSelect)
    {
        case kb_P1:
            kbData->P1 += pidDebug1*sign;
            break;
        case kb_I1:
            kbData->I1 += pidDebug1*sign;
            break;
        case kb_D1:
            kbData->D1 += pidDebug1*sign;
            break;
        case kb_P2:
            kbData->P2 += pidDebug2*sign;
            break;
        case kb_I2:
            kbData->I2 += pidDebug2*sign;
            break;
        case kb_D2:
            kbData->D2 += pidDebug2*sign;
            break;
        case kb_BS:
            kbData->base += baseDebug*sign;
            break;

        default:    break;
    }
}

//get reference value from keyboard
static int paraSelect = 0;
void ref_kb(struct kbData *kbData)
{
    int ch = scanKeyboard();

    switch (ch)
    {
        case kb_w:
            if(kbData->roll>-10)	kbData->roll -= 10;
			else					{}
            break;
        case kb_s:
            if(kbData->roll<10)		kbData->roll += 10;
			else					{}
            break;
        case kb_a:
            if (kbData->pitch>-10)	kbData->pitch -= 10;
			else					{}
            break;
        case kb_d:
			if(kbData->pitch<10)	kbData->pitch += 10;
			else					{}
            break;
        case kb_1:
            if(paraSelect>kb_P1)  paraSelect -= 1;
            else                  paraSelect = kb_BS;
            break;
        case kb_3:
            if(paraSelect<kb_BS)  paraSelect += 1;
            else                  paraSelect = kb_P1;
            break;
        case kb_5:
            paraChange(kbData, paraSelect, 1);
            break;
        case kb_2:
            paraChange(kbData, paraSelect, -1);
            break;
        case kb_ent:
            if(kbData->motor == DISABLE)   kbData->motor = ENABLE;
            else                           kbData->motor = DISABLE;
            break;

        default:	break;
    }

    //pid_set(&angx_pid,kbData->P1,kbData->I1,kbData->D1,1000);
	pid_set(&angz_pid,kbData->P1,kbData->I1,kbData->D1,1000);

	//pid_set(&angVx_pid,kbData->P2,kbData->I2,kbData->D2,5);
	pid_set(&angVz_pid,kbData->P2,kbData->I2,kbData->D2,5);

}

void show_state(struct UAV *UAV, struct kbData *kbData)
{
   	/*
	if(UAV->MOTOR_ENA)
    {
        printf("fbkroll: %f		fbkpitch: %f\n\r",UAV->fbk.roll, UAV->fbk.pitch);
        printf("refroll: %f		refpitch: %f\n\r",UAV->ref.roll, UAV->ref.pitch);
    }
	*/
    //else
    //{
        switch(paraSelect)
        {
		case kb_P1:
			printf("P1: %f	motor: %d\r",kbData->P1,kbData->motor);
			break;
		case kb_I1:
			printf("I1: %f	motor: %d\r",kbData->I1,kbData->motor);
			break;
		case kb_D1:
			printf("D1: %f	motor: %d\r",kbData->D1,kbData->motor);
			break;
		case kb_P2:
			printf("P2: %f	motor: %d\r",kbData->P2,kbData->motor);
			break;
		case kb_I2:
			printf("I2: %f	motor: %d\r",kbData->I2,kbData->motor);
			break;
		case kb_D2:
			printf("D2: %f	motor: %d\r",kbData->D2,kbData->motor);
			break;
		default:
			printf("bs: %f	motor: %d\r",kbData->base,kbData->motor);
        }
    //}

}









