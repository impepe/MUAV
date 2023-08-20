/********************************
file name:	controller.h
discribe:	header file of controllers
created by:	pepe
********************************/

#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "constants.h"

#define POSITION_LOOP	1
#define SPEED_LOOP	2
#define ANGLE_LOOP	3
#define ANGLE_V_LOOP	4
#define ANGLE_ACC_LOOP	5


//structure of PID
struct PID
{
	float Kp,Ki,Kd;
	float err_old;
	float err_sum;
	float limt;
};


//clear data in PID controllers
void pid_clean(struct PID *PID1, struct PID *PID2, struct PID *PID3);

void pid_allClean();

//initialize pid data and parameters
void pid_set(struct PID *PID,float Kp, float Ki, float Kd, float limt);

//calculation of pid
float pid_cal(float ref, float fbk, struct PID *PID);

//pid data distribution
void pid_controller(struct UAV *UAV);

//top function which decides the controller type
void controller(struct UAV *UAV);


#endif










