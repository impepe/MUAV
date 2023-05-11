/********************************
file name:	controller.h
discribe:	head file of controllers
created by:	pepe
********************************/

#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_ 

#include "constants.h"

#define POSITION_LOOP	1
#define SPEED_LOOP		2
#define ANGLE_LOOP		3
#define ANGLE_V_LOOP	4
#define ANGLE_ACC_LOOP	5


//data of a PID controller
struct pid_para
{
	float Kp;
	float Ki;
	float Kd;
	float err_sum;
	float err_old;
	float limit;
};

//PID controllers for each loop
struct PID
{
	struct pid_para pos_loop_x;
	struct pid_para pos_loop_y;
	struct pid_para pos_loop_h;

	struct pid_para speed_loop_x;
	struct pid_para speed_loop_y;
	struct pid_para speed_loop_h;

	struct pid_para angle_loop_x;
	struct pid_para angle_loop_y;
	struct pid_para angle_loop_z;

	struct pid_para angV_loop_x;
	struct pid_para angV_loop_y;
	struct pid_para angV_loop_z;
};

//initialize pid data and parameters
void pid_init(struct PID *PID);

//calculation of pid
float pid_cal(float ref, float fbk, struct pid_para *PID);

//pid data distribution
void pid_controller(struct UAV *UAV, struct PID *PID);

//top function which decides the controller type
void controller(struct UAV *UAV, void *ptr);


#endif










