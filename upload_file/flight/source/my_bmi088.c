/******************************
file name:	my_bmi088.c
discribe:	source file for bmi088 initialization
created by:	pepe
******************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <math.h>
#include "../include/constants.h"
#include "../include/filter.h"
#include "../include/funcs.h"
#include "../include/my_bmi088.h"


#define GRAVITY (9.80665f)
#define IMU_dataErr (isnan(wa_roll))||(isnan(wa_pitch))

struct bmi08_sensor_data bmi08_accel;
struct bmi08_sensor_data bmi08_gyro;

int8_t bmi08_i2c_init(struct bmi08_dev *dev)
{
	int8_t rslt = -1;

	rslt = bmi08_interface_init(dev, BMI08_I2C_INTF, BMI088_VARIANT);
	bmi08_error_codes_print_result("bmi08_interface_init",rslt);

	rslt = bmi08xa_init(dev);
	bmi08_error_codes_print_result("bmi08xa_init",rslt);

	rslt = bmi08g_init(dev);
	bmi08_error_codes_print_result("bmi08g_init",rslt);

	dev->accel_cfg.odr	= BMI08_ACCEL_ODR_1600_HZ;
	dev->accel_cfg.range	= BMI088_ACCEL_RANGE_24G;

	dev->accel_cfg.bw    = BMI08_ACCEL_BW_NORMAL;
    dev->accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;

	rslt = bmi08a_set_power_mode(dev);
	printf("bmi08a_set_power_mode: %d\n", rslt);

	rslt = bmi08xa_set_meas_conf(dev);
	bmi08_error_codes_print_result("bmi08xa_set_meas_conf",rslt);

	dev->gyro_cfg.odr       = BMI08_GYRO_BW_230_ODR_2000_HZ;
    dev->gyro_cfg.range     = BMI08_GYRO_RANGE_250_DPS;
    dev->gyro_cfg.bw        = BMI08_GYRO_BW_230_ODR_2000_HZ;
    dev->gyro_cfg.power     = BMI08_GYRO_PM_NORMAL;

	rslt = bmi08g_set_power_mode(dev);
	printf("bmi08g_set_power_mode: %d\n", rslt);

	rslt = bmi08g_set_meas_conf(dev);
	printf("bmi08g_set_meas_conf: %d\n", rslt);

	printf("bmiinit rslt: %d\n",rslt);
	return rslt;
}

float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width)
{
	double power = 2;

	float half_scale = (float)(pow((double)power,(double)bit_width) / 2.0f);

	return ((GRAVITY*val*g_range)/half_scale);
}


float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
	double power = 2;

	float half_scale = (float)(pow((double)power, (double)bit_width) / 2.0f);

	return (dps/half_scale)*val;
}


int8_t read_bmi088(struct bmi08_dev *dev, struct UAV_state *state)
{
	int8_t rslt = -1;

	rslt = bmi08a_get_data(&bmi08_accel, dev);
	bmi08_error_codes_print_result("bmi08a_get_data",rslt);

	state->acc_x = lsb_to_mps2(bmi08_accel.x,24,16);
	state->acc_y = lsb_to_mps2(bmi08_accel.y,24,16);
	state->acc_z = lsb_to_mps2(bmi08_accel.z,24,16);

	rslt = bmi08g_get_data(&bmi08_gyro, dev);
	bmi08_error_codes_print_result("bmi08g_get_data",rslt);

	state->angV_y = lsb_to_dps(bmi08_gyro.x,(float)250,16);
	state->angV_x = lsb_to_dps(bmi08_gyro.y,(float)250,16);
	state->angV_z = lsb_to_dps(bmi08_gyro.z,(float)250,16);

	return rslt;
}


//elminate the error of gyroscope at steady state
static void wbError_correct(struct UAV_state *state)
{
	if (fabs(state->angV_x)<1.1)	state->angV_x = 0;
	else				state->angV_x -= 0.1;
	if (fabs(state->angV_y)<1.1)	state->angV_y = 0;
	else 				state->angV_y -= 0.04;
	if (fabs(state->angV_z)<1.1)	state->angV_z = 0;
	else				state->angV_z -= -0.015;
}


//get angle by combining data of gyroscope and acceleration
void get_angle(struct UAV_state *state)
{
	//get gyroscope angle by integrating
	static float wb_roll  = 0.;
	static float wb_pitch = 0.;
	static float wb_yaw   = 0.;

	wbError_correct(state);

	wb_roll  -= state->angV_x*Ts;
	wb_pitch -= state->angV_y*Ts;
	wb_yaw   += state->angV_z*Ts;

	//get angle by resolving acceleration
	static float accX_old = 0.;
	static float accY_old = 0.;
	static float accZ_old = 0.;

	state->acc_x = lowPass_filter(state->acc_x,accX_old);	//1st orderlow pass filter
	state->acc_y = lowPass_filter(state->acc_y,accY_old);
	state->acc_z = lowPass_filter(state->acc_z,accZ_old);

	accX_old = state->acc_x;
	accY_old = state->acc_y;
	accZ_old = state->acc_z;

	float wa_roll  = atanf(state->acc_x/sqrt(state->acc_y*state->acc_y+state->acc_z*state->acc_z));		//resolve
	float wa_pitch = atanf(-state->acc_y/sqrt(state->acc_x*state->acc_x+state->acc_z*state->acc_z));

	static float win_filt1[WIN_LEN] = {0.};		//window average filter
	static float win_filt2[WIN_LEN] = {0.};
	static float sum1 = 0.;
	static float sum2 = 0.;
	wa_roll  = win_average(wa_roll,&sum1,&win_filt1);
	wa_pitch = win_average(wa_pitch,&sum2,&win_filt2);

	wa_roll  = wa_roll*180/pi;
	wa_pitch = wa_pitch*180/pi;

	//first order compensation filter
	float K = 0.7;

	if (IMU_dataErr) {}
	else
	{
		state->roll  = K*(state->roll - state->angV_x*Ts) + (1-K)*wa_roll;
		state->pitch = K*(state->pitch - state->angV_y*Ts) + (1-K)*wa_pitch;
	}
	
	//printf("roll: %f	pitch: %f\n",state->roll,state->pitch);	

	state->yaw = wb_yaw;


}


