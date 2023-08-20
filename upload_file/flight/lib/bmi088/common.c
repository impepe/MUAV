/**
 * Copyright (C) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#include "../../include/common.h"

/******************************************************************************/
/*!                       Macro definitions                                   */

#define BMI08_READ_WRITE_LEN  UINT8_C(46)

/*! BMI085 shuttle id */
#define BMI085_SHUTTLE_ID     UINT16_C(0x46)

/*! BMI088 shuttle id */
#define BMI088_SHUTTLE_ID     UINT16_C(0x66)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection for accel */
uint8_t acc_dev_add;

/*! Variable that holds the I2C device address or SPI chip selection for gyro */
uint8_t gyro_dev_add;

int dev_acc;
int dev_gyro;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function
 */



BMI08_INTF_RET_TYPE bmi08_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    	uint8_t dev_addr = *(uint8_t*)intf_ptr;
	int fd;

	if (dev_addr == 0x19)	fd = dev_acc;
	if (dev_addr == 0x69)	fd = dev_gyro;
	
//	printf("reg addr: %d	dev_addr: %d	len: %d\n",reg_addr,dev_addr,len);
	int i;
	for (i=0;i<len;i++)
	{
		int tmp = wiringPiI2CReadReg8(fd,reg_addr+i);
		reg_data[i] = (uint8_t)tmp;
	}

    return 0;
}

/*!
 * I2C write function
 */
BMI08_INTF_RET_TYPE bmi08_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
	int fd;

	if (dev_addr == 0x19)	fd = dev_acc;
	if (dev_addr == 0x69)	fd = dev_gyro;

	int i;
	for (i=0;i<len;i++)
	{
		int tmp = wiringPiI2CWriteReg8(fd,reg_addr+i,reg_data[i]);
	}

    return 0;
}

/*!
 * Delay function map to COINES platform
 */
void bmi08_delay_us(uint32_t period, void *intf_ptr)
{
	usleep(period*10);
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *  Also to initialize coines platform
 */
int8_t bmi08_interface_init(struct bmi08_dev *bmi08, uint8_t intf, uint8_t variant)
{
    int8_t rslt = BMI08_OK;

    if (bmi08 != NULL)
    {
        /* Bus configuration : I2C */
        if (intf == BMI08_I2C_INTF)
        {
        	printf("I2C Interface \n");

            	/* To initialize the user I2C function */
            	acc_dev_add = BMI08_ACCEL_I2C_ADDR_SECONDARY;
            	gyro_dev_add = BMI08_GYRO_I2C_ADDR_SECONDARY;
            	bmi08->intf = BMI08_I2C_INTF;
            	bmi08->read = bmi08_i2c_read;
            	bmi08->write = bmi08_i2c_write;

	    	dev_acc  = wiringPiI2CSetup(0x19);
	    	dev_gyro = wiringPiI2CSetup(0x69);
			
	    	if (dev_acc == -1)	printf("0x19 not found\n");
		if (dev_gyro == -1)	printf("0x69 not found\n");
		
	}


        /* Selection of bmi085 or bmi088 sensor variant */
        bmi08->variant = variant;

        /* Assign accel device address to accel interface pointer */
        bmi08->intf_ptr_accel = &acc_dev_add;

        /* Assign gyro device address to gyro interface pointer */
        bmi08->intf_ptr_gyro = &gyro_dev_add;

        /* Configure delay in microseconds */
        bmi08->delay_us = bmi08_delay_us;

        /* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
        bmi08->read_write_len = BMI08_READ_WRITE_LEN;
    }
    else
    {
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;

}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmi08_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMI08_OK)
    {
        printf("%s\t", api_name);
        if (rslt == BMI08_E_NULL_PTR)
        {
            printf("Error [%d] : Null pointer\r\n", rslt);
        }
        else if (rslt == BMI08_E_COM_FAIL)
        {
            printf("Error [%d] : Communication failure\r\n", rslt);
        }
        else if (rslt == BMI08_E_DEV_NOT_FOUND)
        {
            printf("Error [%d] : Device not found\r\n", rslt);
        }
        else if (rslt == BMI08_E_OUT_OF_RANGE)
        {
            printf("Error [%d] : Out of Range\r\n", rslt);
        }
        else if (rslt == BMI08_E_INVALID_INPUT)
        {
            printf("Error [%d] : Invalid input\r\n", rslt);
        }
        else if (rslt == BMI08_E_CONFIG_STREAM_ERROR)
        {
            printf("Error [%d] : Config stream error\r\n", rslt);
        }
        else if (rslt == BMI08_E_RD_WR_LENGTH_INVALID)
        {
            printf("Error [%d] : Invalid Read write length\r\n", rslt);
        }
        else if (rslt == BMI08_E_INVALID_CONFIG)
        {
            printf("Error [%d] : Invalid config\r\n", rslt);
        }
        else if (rslt == BMI08_E_FEATURE_NOT_SUPPORTED)
        {
            printf("Error [%d] : Feature not supported\r\n", rslt);
        }
        else if (rslt == BMI08_W_FIFO_EMPTY)
        {
            printf("Warning [%d] : FIFO empty\r\n", rslt);
        }
        else
        {
            printf("Error [%d] : Unknown error code\r\n", rslt);
        }
   	 exit(1);
    }
}

