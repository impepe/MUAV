/*********************************
file name: 	pos_info.c
description:	source file of pos_info.h
		get velocity and position information from accelerator,estimator and camera.
created by: 	pepe
date:		06/07/2023
*********************************/

#include <stdio.h>
#include <math.h>
#include "../include/pos_info.h"

//get velocity from estimator
void get_vlEst(struct UAV_state *state, float (*output)[3])
{
	static float estVx = 0.;	//estimated velocity
	static float estVy = 0.;
	static float estVz = 0.;
	
	static float estPx = 0.;
	static float estPy = 0.;
	static float estPz = 0.;

	estPx += estVx*Ts;
	estPy += estVy*Ts;
	estPz += estVz*Ts;
	
	float K = -10;
	estVx = K*(estVx - state->pos_x);
	estVy = K*(estVy - state->pos_y);
	estVz = K*(estVz - state->pos_h);

	(*output)[0] = estVx;
	(*output)[1] = estVy;
	(*output)[2] = estVz;

}

//get velocity from camera. read shared memory with Python
void get_vlCam(float (*output)[3])
{
	
}

//final output of velocity
void get_vl(struct UAV_state *state)
{
	float vl_fromEst[3] = {0.};
	float vl_fromCam[3] = {0.};

	get_vlEst(state, &vl_fromEst);
	get_vlCam(&vl_fromCam);
}

void getPos_fromAcc(struct UAV_state *state, float (*pos_fromAcc)[3])
{
	float sax = 0.;
	float say = 0.;
	float saz = 0.;
	
	static float Vx_fromAcc = 0.;
	static float Vy_fromAcc = 0.;
	static float Vz_fromAcc = 0.;

	float roll  = state->roll*3.14/180;
	float pitch = state->pitch*3.14/180;
	float yaw   = 0.;

	float wax = state->acc_x;
	float way = state->acc_y;
	float waz = state->acc_z;
	
	sax += cos(pitch)*cos(yaw)*wax;
	sax += (sin(roll)*sin(pitch)*cos(yaw)+cos(roll)*sin(yaw))*way;
	sax += (-cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw))*waz;

	say += -cos(pitch)*sin(yaw)*wax;
	say += (-sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw))*way;
	say += (cos(roll)*sin(pitch)*sin(yaw)+sin(roll)*cos(yaw))*waz;

	saz += sin(pitch)*wax;
	saz += -sin(roll)*cos(pitch)*way;
	saz += cos(roll)*cos(pitch)*waz;
	saz -= g;

	Vx_fromAcc += sax*Ts;
	Vy_fromAcc += say*Ts;
	Vz_fromAcc += saz*Ts;
	
	printf("acc_x: %f	acc_y: %f	acc_z: %f\n",wax,way,waz);
	printf("Vx: %f	Vy: %f	Vz: %f\n",Vx_fromAcc,Vy_fromAcc,Vz_fromAcc);

	(*pos_fromAcc)[0] += Vx_fromAcc*Ts;
	(*pos_fromAcc)[1] += Vy_fromAcc*Ts;
	(*pos_fromAcc)[2] += Vz_fromAcc*Ts;
}


void get_pos(struct UAV_state *state)
{

}
