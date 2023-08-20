/*********************************************
file name:	flight_control.c
discribe:	main source file
created by:	pepe
*********************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "include/constants.h"
#include "include/controller.h"
#include "include/my_bmi088.h"
#include "include/pos_info.h"
#include "include/gpio.h"
#include "include/nrf2.h"
#include "include/funcs.h"

struct UAV my_UAV;
struct bmi08_dev bmi08dev;

struct rf24Data  rf24Data;
struct kbData    kbData;

int Timer = DISABLE;


int all_init()
{
    system(STTY_US TTY_PATH);

	if (wiringPiSetup() == -1 || wiringPiSetupGpio()==-1)
	{
		printf("wiringPi set up failed!\n");
		return -1;
	}

	if (bmi08_i2c_init(&bmi08dev) == -1)
	{
		printf("bmi088 i2c init failed!\n");
		return -1;
	}

//	rf24Init();

    myPwmInit();

    pid_init();

    kbData.P1 = angz_Kp;
    kbData.I1 = angz_Ki;
    kbData.D1 = 0.;
    kbData.P2 = angVz_Kp;
    kbData.I2 = 0.;
    kbData.D2 = 0.;
    kbData.base  = init_base;
    kbData.motor = DISABLE;
	
	my_UAV.MOTOR_ENA = DISABLE;
	return 1;

}


void get_fbkInfo()
{

	read_bmi088(&bmi08dev, &my_UAV.fbk);    //also get feedback angular speed here
	get_angle(&my_UAV.fbk);					//get angle from IMU
	//get_vl(&my_UAV.fbk);
	//get_pos(&my_UAV.fbk);
}


void get_refInfo()
{
//	my_UAV.ref.roll  = 0;
//	my_UAV.ref.pitch = 0;

/*
	if(readrf24(&rf24Data) == -1)
	{
		printf("nrf disconnected!\n");
		my_UAV.MOTOR_ENA = DISABLE;
	}
	//delay(20);
	//sendrf24(&my_UAV);

	my_UAV.ref.roll  = (float)rf24Data.roll;
	my_UAV.ref.pitch = (float)rf24Data.pitch;

	float Kp1 =((float)rf24Data.P1)/1000;
	float Ki1 =((float)rf24Data.I1)/1000;
	float Kd1 =((float)rf24Data.D1)/1000;
	float Kp2 =((float)rf24Data.P2)/1000;
	float Ki2 =((float)rf24Data.I2)/1000;
	float Kd2 =((float)rf24Data.D2)/1000;

	pid_set(&angx_pid,Kp1,Ki1,Kd1,1000);
	pid_set(&angy_pid,Kp1,Ki1,Kd1,1000);

	pid_set(&angVx_pid,Kp2,Ki2,Kd2,5);
	pid_set(&angVy_pid,Kp2,Ki2,Kd2,5);

	my_UAV.drive.base =(float)rf24Data.base/10;

	my_UAV.MOTOR_ENA = rf24Data.motor;
*/
    ref_kb(&kbData);
    my_UAV.ref.roll   = kbData.roll;
    my_UAV.ref.pitch  = kbData.pitch;
    my_UAV.MOTOR_ENA  = kbData.motor;
    my_UAV.drive.base = kbData.base;
}

void PWM_ISR()
{
	Timer = ENABLE;
}

void PwmOutput(float m1,float m2, float m3, float m4)
{
	extern int dutyCycle1;
	extern int dutyCycle2;
	extern int dutyCycle3;
	extern int dutyCycle4;

	dutyCycle1 = (int)m1;
	dutyCycle2 = (int)m2;
	dutyCycle3 = (int)m3;
	dutyCycle4 = (int)m4;
}

int main(void)
{
	if (all_init() == -1)
	{
		printf("init failed\n");
		return -1;
	}
	extern int PWM_Timer;

	sleep(1);
//loop
	while (1)
	{
		if (PWM_Timer)
		{
		    get_refInfo();
            get_fbkInfo();
            show_state(&my_UAV, &kbData);
			if (my_UAV.MOTOR_ENA)
			{
				//angle control loop
				strcpy(my_UAV.controller_type,"PID");
				my_UAV.control_state = ANGLE_LOOP;
				controller(&my_UAV);

				//angle velocity control loop
				my_UAV.control_state = ANGLE_V_LOOP;
				controller(&my_UAV);

				//printf("x: %f	y: %f	z: %f\n\r",my_UAV.drive.x,my_UAV.drive.y,my_UAV.drive.z);
				//distribute control signal to each ESC
				power_distribute(&my_UAV);
				PwmOutput(my_UAV.drive.m1, my_UAV.drive.m2, my_UAV.drive.m3, my_UAV.drive.m4);
				//PwmOutput(10, 10, 10, 10);
			}
			else
			{
				//stop controllers and motors
				pid_allClean();
				PwmOutput(10, 10, 10, 10);
			}
			//float data1 = my_UAV.ref.pitch;
			//float data2 = my_UAV.fbk.pitch;
			//save_data(data1, data2, 0);
			PWM_Timer = DISABLE;
		}
	}

	return 0;
}










