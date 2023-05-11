/*****************************
file name:	controller.c
discribe:	the source file of all digital controllers
			including PID, ADRC, full-state feedback, slide mode
created by: pepe
*****************************/

#include <stddef.h>		//define NULL
#include <stdio.h>
#include "../include/controller.h"

//pid initialization
void pid_init(struct PID *PID)
{
	if (PID == NULL)
	{
		// position loop
		PID->pos_loop_x.Kp = 1.;
		PID->pos_loop_x.Ki = 0.;
		PID->pos_loop_x.Kd = 0.;

		PID->pos_loop_y.Kp = 1.;
		PID->pos_loop_y.Ki = 0.;
		PID->pos_loop_y.Kd = 0.;

		PID->pos_loop_h.Kp = 2.;
		PID->pos_loop_h.Ki = 0.;
		PID->pos_loop_h.Kd = 0.;

		//speed loop
		PID->speed_loop_x.Kp = 4.2;
		PID->speed_loop_x.Ki = 0.;
		PID->speed_loop_x.Kd = 0.;

		PID->speed_loop_y.Kp = 4.2;
		PID->speed_loop_y.Ki = 0.;
		PID->speed_loop_y.Kd = 0.;

		PID->speed_loop_h.Kp = 10.7;
		PID->speed_loop_h.Ki = 12.;
		PID->speed_loop_h.Kd = 0.;

		//angle loop
		PID->angle_loop_x.Kp = 13.8;
		PID->angle_loop_x.Ki = 0.;
		PID->angle_loop_x.Kd = 0.;
		
		PID->angle_loop_y.Kp = 13.8;
		PID->angle_loop_y.Ki = 0.;
		PID->angle_loop_y.Kd = 0.;

		PID->angle_loop_z.Kp = 5.;
		PID->angle_loop_z.Ki = 0.;
		PID->angle_loop_z.Kd = 0.;

		//angle speed loop
		PID->angV_loop_x.Kp = 0.16;
		PID->angV_loop_x.Ki = 0.01;
		PID->angV_loop_x.Kd = 0.;

		PID->angV_loop_y.Kp = 0.16;
		PID->angV_loop_y.Kp = 0.01;
		PID->angV_loop_y.Kp = 0.;
		
		PID->angV_loop_z.Kp = 20.;
		PID->angV_loop_z.Ki = 0.;
		PID->angV_loop_z.Kd = 0.;

		//data of integrator and differentiator
		PID->pos_loop_x.err_sum = 0.;
		PID->pos_loop_x.err_old = 0.;
		PID->pos_loop_y.err_sum = 0.;
		PID->pos_loop_y.err_old = 0.;
		PID->pos_loop_h.err_sum = 0.;
		PID->pos_loop_h.err_old = 0.;

		PID->speed_loop_x.err_sum = 0.;
		PID->speed_loop_x.err_old = 0.;
		PID->speed_loop_y.err_sum = 0.;
		PID->speed_loop_y.err_old = 0.;
		PID->speed_loop_h.err_sum = 0.;
		PID->speed_loop_h.err_old = 0.;

		PID->angle_loop_x.err_sum = 0.;
		PID->angle_loop_x.err_old = 0.;
		PID->angle_loop_y.err_sum = 0.;
		PID->angle_loop_y.err_old = 0.;
		PID->angle_loop_z.err_sum = 0.;
		PID->angle_loop_z.err_old = 0.;

		PID->angV_loop_x.err_sum = 0.;
		PID->angV_loop_x.err_old = 0.;
		PID->angV_loop_y.err_sum = 0.;
		PID->angV_loop_y.err_old = 0.;
		PID->angV_loop_z.err_sum = 0.;
		PID->angV_loop_z.err_sum = 0.;

		//output limitation
		PID->pos_loop_x.limit = 1.;
		PID->pos_loop_y.limit = 1.;
		PID->pos_loop_h.limit = 5.;

		PID->speed_loop_x.limit = 1146.;	//20(degrees)*180/pi = 1146(rads)
		PID->speed_loop_y.limit = 1146.;
		PID->speed_loop_h.limit = 1.;

		PID->angle_loop_x.limit = -1.;		//no limitation
		PID->angle_loop_y.limit = -1.;
		PID->angle_loop_z.limit = -1.;
		
		PID->angV_loop_x.limit = 10.;	//ESC PWM drive limitation. width: 1ms-2ms.
		PID->angV_loop_y.limit = 1.;
		PID->angV_loop_z.limit = 0.;
	}
}

//pid calculation
float pid_cal(float ref, float fbk, struct pid_para *PID)
{	
	float err = ref - fbk;
	PID->err_sum += err;
	float rslt = (PID->Kp)*err + (PID->Ki)*(PID->err_sum)*Ts + (PID->Kd)*(PID->err_old)*(1/Ts);
	PID->err_old = err;

	return rslt;
}

//distribute data according to different loops
void pid_controller(struct UAV *UAV, struct PID *PID)
{
	float ref,fbk;	//reference and feedback

	if (UAV->control_state == POSITION_LOOP)
	{
		ref = UAV->ref.pos_x;
		fbk = UAV->fbk.pos_x;
		UAV->ref.speed_x = pid_cal(ref, fbk, &PID->pos_loop_x);
		
		ref = UAV->ref.pos_y;
		fbk = UAV->fbk.pos_y;
		UAV->ref.speed_y = pid_cal(ref, fbk, &PID->pos_loop_y);

		ref = UAV->ref.pos_h;
		fbk = UAV->fbk.pos_h;
		UAV->ref.speed_h = pid_cal(ref, fbk, &PID->pos_loop_h);
	}

	else if (UAV->control_state == SPEED_LOOP)
	{
		ref = UAV->ref.speed_x;
		fbk = UAV->fbk.speed_x;
		UAV->ref.ang_x = pid_cal(ref, fbk, &PID->speed_loop_x);

		ref = UAV->ref.speed_y;
		fbk = UAV->fbk.speed_y;
		UAV->ref.ang_y = pid_cal(ref, fbk, &PID->speed_loop_y);

		ref = UAV->ref.speed_h;
		fbk = UAV->fbk.speed_h;
		UAV->output.base = pid_cal(ref, fbk, &PID->speed_loop_h);
	}

	else if (UAV->control_state == ANGLE_LOOP)
	{
		ref = UAV->ref.ang_x;
		fbk = UAV->fbk.ang_x;
		UAV->ref.angV_x = pid_cal(ref, fbk, &PID->angle_loop_x);

		ref = UAV->ref.ang_y;
		fbk = UAV->fbk.ang_y;
		UAV->ref.angV_y = pid_cal(ref, fbk, &PID->angle_loop_y);

		ref = UAV->ref.ang_z;
		fbk = UAV->fbk.ang_z;
		UAV->ref.angV_z = pid_cal(ref, fbk, &PID->angle_loop_z);
	}

	else if (UAV->control_state == ANGLE_V_LOOP)
	{
		ref = UAV->ref.angV_x;
		fbk = UAV->fbk.angV_x;
		UAV->output.x = pid_cal(ref, fbk, &PID->angV_loop_x);

		ref = UAV->ref.angV_y;
		fbk = UAV->fbk.angV_y;
		UAV->output.y = pid_cal(ref, fbk, &PID->angV_loop_y);

		ref = UAV->ref.angV_z;
		fbk = UAV->fbk.angV_z;
		UAV->output.z = pid_cal(ref, fbk, &PID->angV_loop_z);
	}

	else
	{
		printf("pid controller state error\n");
		UAV->MOTOR_ENA = DISABLE;	
	}
}


void controller(struct UAV *UAV, void *ptr)
{
	if (UAV->controller_type == "PID")
	{
		struct PID *PID = (struct PID*)ptr;
		pid_controller(UAV,PID);
	}

	else 
	{
		printf("no such a controller!\n");
		UAV->MOTOR_ENA = DISABLE;
	}
}











































