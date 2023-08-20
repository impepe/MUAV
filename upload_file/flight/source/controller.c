/*****************************
file name:	controller.c
discribe:	the source file of all digital controllers
			including PID, ADRC, full-state feedback, slide mode
created by: pepe
*****************************/

#include <stdio.h>
#include <string.h>
#include "../include/controller.h"

struct PID angVx_pid;
struct PID angVy_pid;
struct PID angVz_pid;

struct PID angx_pid;
struct PID angy_pid;
struct PID angz_pid;

struct PID vlx_pid;
struct PID vly_pid;
struct PID vlz_pid;

struct PID posx_pid;
struct PID posy_pid;
struct PID posz_pid;


//clear data in PID controllers
void pid_clean(struct PID *PID1, struct PID *PID2, struct PID *PID3)
{
	PID1->err_sum = 0.;
	PID1->err_old = 0.;
	PID2->err_sum = 0.;
	PID2->err_old = 0.;
	PID3->err_sum = 0.;
	PID3->err_old = 0.;
}


void pid_allClean()
{
    pid_clean(&angVx_pid,&angVy_pid,&angVz_pid);
    pid_clean(&angx_pid,&angy_pid,&angz_pid);
    pid_clean(&vlx_pid,&vly_pid,&vlz_pid);
    pid_clean(&posx_pid,&posy_pid,&posz_pid);
}


//set PID parameters
void pid_set(struct PID *PID, float Kp, float Ki, float Kd, float limt)
{
	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;
	PID->limt = limt;
}

//pid calculation
float pid_cal(float ref, float fbk, struct PID *PID)
{
	float err = ref - fbk;
	float rslt = (PID->Kp)*err + (PID->Ki)*(PID->err_sum) + (PID->Kd)*(err - PID->err_old)/Ts;

	if (rslt > PID->limt)		rslt = PID->limt;
	else if (rslt < -PID->limt)	rslt = -PID->limt;
	else 				PID->err_sum += err*Ts;
	PID->err_old = err;

	return rslt;
}

//distribute data according to different loops
void pid_controller(struct UAV *UAV)
{
	float ref,fbk;	//reference and feedback

	switch (UAV->control_state)
	{
		case POSITION_LOOP:			//position loop
			ref = UAV->ref.pos_x;
			fbk = UAV->fbk.pos_x;
			UAV->ref.speed_x = pid_cal(ref, fbk, &posx_pid);

			ref = UAV->ref.pos_y;
			fbk = UAV->fbk.pos_y;
			UAV->ref.speed_y = pid_cal(ref, fbk, &posy_pid);

			ref = UAV->ref.pos_h;
			fbk = UAV->fbk.pos_h;
			UAV->ref.speed_h = pid_cal(ref, fbk, &posz_pid);
			break;

		case SPEED_LOOP:			//velocity loop
			ref = UAV->ref.speed_x;
			fbk = UAV->fbk.speed_x;
			UAV->ref.roll = pid_cal(ref, fbk, &vlx_pid);

			ref = UAV->ref.speed_y;
			fbk = UAV->fbk.speed_y;
			UAV->ref.pitch = pid_cal(ref, fbk, &vly_pid);

			ref = UAV->ref.speed_h;
			fbk = UAV->fbk.speed_h;
			UAV->drive.base = pid_cal(ref, fbk, &vlz_pid);
			break;

		case ANGLE_LOOP:			//angle loop
			ref = UAV->ref.roll;
			fbk = UAV->fbk.roll;
			UAV->ref.angV_x = pid_cal(ref, fbk, &angx_pid);

			ref = UAV->ref.pitch;
			fbk = UAV->fbk.pitch;
			UAV->ref.angV_y = pid_cal(ref, fbk, &angy_pid);

			ref = UAV->ref.yaw;
			fbk = UAV->fbk.yaw;
			UAV->ref.angV_z = pid_cal(ref, fbk, &angz_pid);
			break;

		case ANGLE_V_LOOP:			//angular speed loop
			ref = UAV->ref.angV_x;
			fbk = UAV->fbk.angV_x;
			UAV->drive.x = pid_cal(ref, fbk, &angVx_pid);

			ref = UAV->ref.angV_y;
			fbk = UAV->fbk.angV_y;
			UAV->drive.y = pid_cal(ref, fbk, &angVy_pid);

			ref = UAV->ref.angV_z;
			fbk = UAV->fbk.angV_z;
			UAV->drive.z = pid_cal(ref, fbk, &angVz_pid);
			break;

		default:
			printf("pid controller state error\n");
			UAV->MOTOR_ENA = DISABLE;	//shut down motors
			break;
	}
}


void controller(struct UAV *UAV)
{
	if (!strcmp(UAV->controller_type,"PID"))
	{
		pid_controller(UAV);
	}

	else
	{
		printf("no such a controller!\n");
		UAV->MOTOR_ENA = DISABLE;
	}
}











































