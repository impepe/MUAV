
#ifndef _CONTROLLERS_H
#define _CONTROLLERS_H

// 参数依次为参考值，反馈值，KP，KI，限幅输出，积分量
float PI(float, float, float, float, float, float *);

float logControl(float ref, float fbk, float K1, float K2, float Ki, float *error_sum);

#endif