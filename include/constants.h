/********************************
file name:	constant.h
discribe:	head file including working frequency, some constants, and UAV states
created by:	pepe
********************************/

#ifndef _CONSTANTS_H
#define _CONSTANTS_H

#define Ts			0.01	//period of controller, f=100Hz
#define g			9.8		//define of gravity

#define ENABLE	1
#define DISABLE	0

//states of UAV
struct UAV_state
{
	//angular acceleration
	float angAcc_x;
	float angAcc_y;
	float angAcc_z;

	//angluar speed
	float angV_x;
	float angV_y;
	float angV_z;

	//angle
	float ang_x;
	float ang_y;
	float ang_z;

	//acceleration
	float acc_x;
	float acc_y;
	float acc_z;

	//speed
	float speed_x;
	float speed_y;
	float speed_h;	//H = High

	//position
	float pos_x;
	float pos_y;
	float pos_h;
	
};

struct ESO_drive
{
	float x;
	float y;
	float z;
	/*******************
	structure: X-type
	m1   m2
	   \/
	   /\
	m3   m4
	*******************/
	float base;
	float m1;
	float m2;
	float m3;
	float m4;
};

//UAV object
struct UAV
{
	struct UAV_state ref;	//reference value
	struct UAV_state fbk;	//feedback value
	
	struct ESO_drive output;	//drive signal to electronic speed controller(ESO)

	_Bool MOTOR_ENA;	//enable signal of motor,IMU and nrf24l01
	_Bool NRF_ENA;
	_Bool IMU_ENA;
	
	short control_state;	//this signal indicates which control loop is processing
	char controller_type[10];	//this signal decides the type of controller

};


#endif
