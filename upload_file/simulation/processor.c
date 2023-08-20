/*====================================
File name: 		processor.c
Description: 	s-function declarations of UAV
Created by: 	Peiyan
Date created: 	2023/3/3
Date modified: 	2023/3/3
====================================*/

#define S_FUNCTION_NAME  processor
#define S_FUNCTION_LEVEL 2

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"
#include <math.h>
#include "parameters.h"
#include "controllers.h"

float posX_errsum, posY_errsum, posZ_errsum;
float vlx_errsum, vly_errsum, vlz_errsum;
float roll_errsum, pitch_errsum, yaw_errsum;
float wbx_errsum, wby_errsum, wbz_errsum;
float waccx_errsum, waccy_errsum, waccz_errsum;


static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 19);
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 4);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}


static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, Ts);
    ssSetOffsetTime(S, 0, 0.0);

}



#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
  static void mdlInitializeConditions(SimStruct *S)
  {
  }


#define MDL_START  /* Change to #undef to remove function */
  static void mdlStart(SimStruct *S)
  {
      posX_errsum   = 0.;
      posY_errsum   = 0.; 
      posZ_errsum   = 0.;
      vlx_errsum    = 0.;
      vly_errsum    = 0.;
      vlz_errsum    = 0.;
      roll_errsum    = 0.; 
      pitch_errsum  = 0.;
      yaw_errsum    = 0.;
      wbx_errsum    = 0.;
      wby_errsum    = 0.;
      wbz_errsum    = 0.;
      waccx_errsum  = 0.;
      waccy_errsum  = 0.;
      waccz_errsum  = 0.;
  }

static void mdlOutputs(SimStruct *S, int_T tid)
{
    double            *y      = ssGetOutputPortRealSignal(S,0);
    double            *x      = ssGetRealDiscStates(S);
    InputRealPtrsType uPtrs   = ssGetInputPortRealSignalPtrs(S,0);
    
    float pos_x,pos_y,pos_z;
    float vl_x,vl_y,vl_z;
    float roll,pitch,yaw;
    float wbx,wby,wbz;
    float posX_ref,posY_ref,posZ_ref;
    float yaw_ref;
    float waccx, waccy, waccz;
    
    float roll_ref,pitch_ref;
    
    pos_x =      (float)    (*uPtrs[0]);
    pos_y =      (float)    (*uPtrs[1]);
    pos_z =      (float)    (*uPtrs[2]);
    vl_x  =      (float)    (*uPtrs[3]);
    vl_y  =      (float)    (*uPtrs[4]);
    vl_z  =      (float)    (*uPtrs[5]);
    roll  =      (float)    (*uPtrs[6]);
    pitch =      (float)    (*uPtrs[7]);
    yaw   =      (float)    (*uPtrs[8]);
    wbx   =      (float)    (*uPtrs[9]);
    wby   =      (float)    (*uPtrs[10]);
    wbz   =      (float)    (*uPtrs[11]);
    waccx =      (float)    (*uPtrs[12]);
    waccy =      (float)    (*uPtrs[13]);
    waccz =      (float)    (*uPtrs[14]);
    posX_ref =  (float)  (*uPtrs[15]);
    posY_ref =  (float)  (*uPtrs[16]);
    posZ_ref =  (float)  (*uPtrs[17]);
    yaw_ref  =  (float)  (*uPtrs[18]);
    
    //位置环
    float vlx_ref,vly_ref,vlz_ref;
//     
//     vlx_ref = PI(posX_ref, pos_x, ps_Kp, ps_Ki, ps_limt, &posX_errsum);
//     vly_ref = PI(posY_ref, pos_y, ps_Kp, ps_Ki, ps_limt, &posY_errsum);
    vlz_ref = PI(posZ_ref, pos_z, psZ_Kp, psZ_Ki, psZ_limt, &posZ_errsum);
//     
    //速度环
    float ax_ref,ay_ref,base;
    
//     ax_ref  = PI(vlx_ref, vl_x, vl_Kp, vl_Ki, vl_limt, &vlx_errsum);
//     ay_ref  = PI(vly_ref, vl_y, vl_Kp, vl_Ki, vl_limt, &vly_errsum);
    base    = PI(vlz_ref, vl_z, vlz_Kp, vlz_Ki, vlz_limt, &vlz_errsum);
    
    //速度到角度的转换
//     float roll_ref,pitch_ref;
//     
//     roll_ref  = (-1/g)*(sin(yaw)*ax_ref - cos(yaw)*ay_ref);
//     pitch_ref = (-1/g)*(cos(yaw)*ax_ref + sin(yaw)*ay_ref);
    
    //角度环
     float wbx_ref,wby_ref,wbz_ref;
     
      roll_ref  = 30;
      pitch_ref = 0;
      yaw_ref   = 0;
    
//     wbx_ref = logControl(roll_ref, roll, 5, 2, 0, &roll_errsum); 
//     wby_ref = PI(pitch_ref, pitch, angle_Kp, angle_Ki, angle_limt, &pitch_errsum); 
//     wbz_ref = PI(yaw_ref, yaw, yaw_Kp, yaw_Ki, yaw_limt, &yaw_errsum);
    
//      wbx_ref = PI(roll_ref, roll, angle_Kp, angle_Ki, angle_limt, &roll_errsum); 
//      wby_ref = PI(pitch_ref, pitch, angle_Kp, angle_Ki, angle_limt, &pitch_errsum); 
//      wbz_ref = PI(yaw_ref, yaw, yaw_Kp, yaw_Ki, yaw_limt, &yaw_errsum);
//      
//      wbx_ref = 10;
//      wby_ref = 0;
//      wbz_ref = 0;
    
    //角速度环
     float waccx_ref, waccy_ref, yaw_pwm;
     
     waccx_ref  = PI(wbx_ref, wbx, wb_Kp, wb_Ki, wb_limt, &wbx_errsum);
     waccy_ref  = PI(wby_ref, wby, wb_Kp, wb_Ki, wb_limt, &wby_errsum);
     yaw_pwm    = PI(wbz_ref, wbz, wbz_Kp, wbz_Ki, wb_limt, &wbz_errsum);
    
    //角加速度环
    float roll_pwm,pitch_pwm;
    
    waccx_ref = 20;
    waccy_ref = 0;
    
    roll_pwm  = PI(waccx_ref, waccx, wacc_Kp, wacc_Ki, wacc_limt, &waccx_errsum);
    pitch_pwm = PI(waccy_ref, waccy, wacc_Kp, wacc_Ki, wacc_limt, &waccy_errsum);
    
    //yaw_pwm = 0.;
//     base = 0.60899 - base;
//     
//     if (base > 1)       base = 1.;
//     else if (base < 0)  base = 0.;
//     else                base = base;
    base = 0.60899;
    
    //动力分配
    float m1,m2,m3,m4;
    
    int sign_roll  = 0;
    int sign_pitch = 0;
    if (roll_pwm>0)     sign_roll=1;
    else                sign_roll=-1;
    if (pitch_pwm>0)    sign_pitch=1;
    else                sign_pitch=-1;
    
    roll_pwm = fabs(roll_pwm);
    pitch_pwm= fabs(pitch_pwm);
    
    m1 = base - sqrt(roll_pwm/2)*sign_roll + sqrt(pitch_pwm/2)*sign_pitch + yaw_pwm;
    m2 = base + sqrt(roll_pwm/2)*sign_roll - sqrt(pitch_pwm/2)*sign_pitch + yaw_pwm;
    m3 = base + sqrt(roll_pwm/2)*sign_roll + sqrt(pitch_pwm/2)*sign_pitch - yaw_pwm;
    m4 = base - sqrt(roll_pwm/2)*sign_roll - sqrt(pitch_pwm/2)*sign_pitch - yaw_pwm;
    
    //输出
    y[0]   =    m1;
    y[1]   =    m2;
    y[2]   =    m3;
    y[3]   =    m4;

}

#define MDL_UPDATE  /* Change to #undef to remove function */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
        real_T            *x       = ssGetRealDiscStates(S);
        InputRealPtrsType uPtrs    = ssGetInputPortRealSignalPtrs(S,0);
  }

static void mdlTerminate(SimStruct *S)
{
     UNUSED_ARG(S); /* unused input argument */
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
