/***************************
file name: 	pos_info.h
description:	header file of pos_info.c
created by:	pepe
date:		06/07/2023
***************************/
#ifndef __POS_INFO_H__
#define __POS_INFO_H__

#include "constants.h"

void get_vlEst(struct UAV_state *state, float (*output)[3]);

void get_vlCam(float (*output)[3]);

void get_vl(struct UAV_state *state);

void getV_fromAcc(struct UAV_state *state, float (*pos_fromAcc)[3]);

void get_pos(struct UAV_state *state);


#endif
