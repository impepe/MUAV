/******************************
file name:	my_bmi088.c
discribe:	source file for bmi088 initialization
created by:	pepe
******************************/

#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <math.h>
#include "../include/my_bmi088.h"

#define GRAVITY (9.80665f)

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

	dev->accel_cfg.odr		= BMI08_ACCEL_ODR_1600_HZ;
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

	state->angV_x = lsb_to_dps(bmi08_gyro.x,(float)250,16);
	state->angV_y = lsb_to_dps(bmi08_gyro.x,(float)250,16);
	state->angV_z = lsb_to_dps(bmi08_gyro.x,(float)250,16);
	
	return rslt;
}





















