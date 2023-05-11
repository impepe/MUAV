/*********************************************
file name:	flight_control.c
discribe:	main source file
created by:	pepe
*********************************************/

#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h>
#include "include/constants.h"
#include "include/controller.h"
#include "include/my_bmi088.h"

struct UAV my_UAV;
struct bmi08_dev bmi08dev;
struct PID pid;

int8_t all_init()
{	
	
	if (wiringPiSetup() == -1)
	{
		printf("wiringPi set up failed!\n");
		return -1;
	}

	if (bmi08_i2c_init(&bmi08dev) == -1)	
	{
		printf("bmi088 i2c init failed!\n");
		return -1;
	}
	
	pid_init(&pid);
}

int main(void)
{	
	if (all_init() == -1)
	{
		printf("init failed\n");
		return 0;
	}
	
	my_UAV.IMU_ENA = ENABLE;

// get feedback data here
	while (1)
	{	
		if (my_UAV.IMU_ENA)
		{
			read_bmi088(&bmi08dev, &my_UAV.fbk);

			printf("acce: %f  %f  %f\n",my_UAV.fbk.acc_x,my_UAV.fbk.acc_y,my_UAV.fbk.acc_z);
			printf("gyro: %f  %f  %f\n\n",my_UAV.fbk.angV_x,my_UAV.fbk.angV_y,my_UAV.fbk.angV_z);
			usleep(10000);
		}
	}
/*
	//position control loop
	my_UAV.control_type = "PID";
	my_UAV.control_state = POSITION_LOOP;
	controller(&my_UAV);

	//speed control loop
	my_UAV.control_state = SPEED_LOOP;
	controller(&my_UAV);
	
	//transform speed to angle
	my_UAV.ref.ang_X = (1/g)*(sin(my_UAV.fbk.ang_Z)*my_UAV.ref.acc_X - cos(my_UAV.fbk.ang_Z)*my_UAV.ref.acc_Y);
	my_UAV.ref.ang_Y = (1/g)*(sin(my_UAV.fbk.ang_Z)*my_UAV.ref.acc_Y + cos(my_UAV.fbk.ang_Z)*my_UAV.ref.acc_X);

	//angle control loop
	my_UAV.control_state = ANGLE_LOOP;
	controller(&my_UAV);

	//angle acceleration control loop
	my_UAV.control_state = ANGLE_ACC_LOOP;
	controller(&my_UAV);
*/	
	return 0;
}










