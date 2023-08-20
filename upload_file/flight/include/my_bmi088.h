#ifndef _MY_BMI088_H
#define _MY_BMI088_H

#ifdef __cplusplus
extern "C"{
#endif

#include "bmi08x.h"
#include "common.h"

int8_t bmi08_i2c_init(struct bmi08_dev *dev);

float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width);

float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

int8_t read_bmi088(struct bmi08_dev *dev, struct UAV_state *state);

void get_angle(struct UAV_state *state);

#ifdef __cplusplus
}
#endif

#endif
