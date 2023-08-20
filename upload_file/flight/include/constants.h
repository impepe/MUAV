/********************************
file name:	constants.h
discribe:	head file including working frequency, some constants, and UAV states
created by:	pepe
********************************/
#include <stdint.h>

#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

#define Ts	0.01	//period of controller, f=100Hz
#define g	9.8		//define of gravity
#define pi	3.1415926

#define PWM_pin_1	6	//soft pwm pin
#define PWM_pin_2	13
#define PWM_pin_3	19
#define PWM_pin_4	26

#define PWM_TIM_PIN	18
#define DIVISOR		32		//f=19.2M/(32*60)=10kHz
#define PWM_RANGE	60

#define ENABLE	1
#define DISABLE	0

#define DATA_LEN    2048    //saved data


//parameters for PID controllers
#define ang_Kp   	4.5
#define ang_Ki   	0.0038
#define ang_Kd   	0.
#define angz_Kp   	3
#define angz_Ki   	0.
#define angz_Kd   	0.
#define ang_limt    1000.

#define angV_Kp  	0.015
#define angV_Ki  	0.
#define angV_Kd  	0.
#define angVz_Kp   	0.01
#define angVz_Ki   	0.
#define angVz_Kd   	0.
#define angV_limt   5.

#define init_base	13


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
	float roll;
	float pitch;
	float yaw;

	//acceleration
	float acc_x;
	float acc_y;
	float acc_z;

	//speed
	float speed_x;
	float speed_y;
	float speed_h;	//H = Height

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
	m3   m2
	   \/
	   /\
	m1   m4
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

	struct ESO_drive drive;	//drive signal to electronic speed controller(ESO)

	int16_t MOTOR_ENA;	//enable motor and camera

	uint8_t control_state;	//this signal indicates which control loop is processing
	char controller_type[10];	//this signal decides the type of controller

};

struct rf24Data
{
	int16_t P1;
	int16_t I1;
	int16_t D1;

	int16_t P2;
	int16_t I2;
	int16_t D2;

	int16_t base;
	int16_t motor;

	int16_t roll;
	int16_t pitch;
};

struct kbData
{
	float P1;
	float I1;
	float D1;

	float P2;
	float I2;
	float D2;

	float base;
	int   motor;

	float roll;
	float pitch;
};


#endif
