#include "controllers.h"
#include <math.h>

float PI(float ref, float fbk, float Kp, float Ki, float limt, float *error_sum)
{
    float error = ref - fbk;
    *error_sum += error;
    
    float output = Kp*error + Ki**error_sum;
    
//     return output;
    
    if (limt == 0)  return output;
    else
    {   
        if (output > limt)          return  limt;
        else if (output < -limt)    return  -limt;
        else                        return  output;   
    }
    
}

float logControl(float ref, float fbk, float K1, float K2, float Ki, float *error_sum)
{
    float error = ref - fbk;
    *error_sum += error;
    
    int sign = 0;
    if (error>0)    sign = 1;
    else            sign = -1;
    
    float output = K1*log(fabs(K2*error)+1)*sign + Ki**error_sum;
    return output;
}



