/*====================================
File name: 		angle_estimator.c
Description: 	s-function declarations of UAV
Created by: 	Peiyan
Date created: 	2023/6/18
Date modified: 	2023/6/18
====================================*/

#define S_FUNCTION_NAME  angle_estimator
#define S_FUNCTION_LEVEL 2

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"
#include <math.h>
#include "parameters.h"

float wacc_x,wacc_y,wacc_z;
float ang_wbx,ang_wby,ang_wbz;
float ang_wax,ang_way;
float angEst_x,angEst_y,angEst_z;
float wax_old,way_old,waz_old;
float win_filt1[8],win_filt2[8]; 

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
    ssSetInputPortWidth(S, 0, 6);
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 12);

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
      wacc_x = 0.;
      wacc_y = 0.;
      wacc_z = 0.;
      ang_wbx = 0.;
      ang_wby = 0.;
      ang_wbz = 0.;
      ang_wax = 0.;
      ang_way = 0.;
      angEst_x = 0.;
      angEst_y = 0.;
      angEst_z = 0.;
      wax_old = 0.;
      way_old = 0.;
      waz_old = 0.;
      for (int i=0;i<8;i++)
      {
        win_filt1[i] = 0.;
        win_filt2[i] = 0.;
      }
  }


#define MDL_START  /* Change to #undef to remove function */
  static void mdlStart(SimStruct *S)
  {

  }

static void mdlOutputs(SimStruct *S, int_T tid)
{
    double            *y      = ssGetOutputPortRealSignal(S,0);
    double            *x      = ssGetRealDiscStates(S);
    InputRealPtrsType uPtrs   = ssGetInputPortRealSignalPtrs(S,0);
    
    float wa_x,wa_y,wa_z;   //angular acceleration
    float wb_x,wb_y,wb_z;   //angular speed

    // input
    wb_x =     (float)    (*uPtrs[0]);
    wb_y =     (float)    (*uPtrs[1]);
    wb_z =     (float)    (*uPtrs[2]);
    wa_x  =    (float)    (*uPtrs[3]);
    wa_y  =    (float)    (*uPtrs[4]);
    wa_z  =    (float)    (*uPtrs[5]);
    
    //angular speed integrator
    ang_wbx += wb_x*Ts;
    ang_wby += wb_y*Ts;
    ang_wbz += wb_z*Ts;
    
    //1st order filter of accelerator
    float K1 = 0.6;
    wa_x = K1*wa_x + (1-K1)*wax_old;
    wa_y = K1*wa_y + (1-K1)*way_old;
    wa_z = K1*wa_z + (1-K1)*waz_old;
    
    wax_old = wa_x;
    way_old = wa_y;
    waz_old = wa_z;
    
    //acceleration angle resolving
    ang_wax = atanf(wa_y/(sqrt(wa_x*wa_x+wa_z*wa_z)));
    ang_way = atanf(-wa_x/(sqrt(wa_y*wa_y+wa_z*wa_z)));
    
    ang_wax = ang_wax*180/pi;
    ang_way = ang_way*180/pi;
    
    //window average filter
    static int i = 0;
    static float win_sum1 = 0.;
    static float win_sum2 = 0.;
    
    win_sum1 -= win_filt1[i];
    win_sum2 -= win_filt2[i];
    win_filt1[i] = ang_wax;
    win_filt2[i] = ang_way;
    win_sum1 += win_filt1[i];
    win_sum2 += win_filt2[i];
    
    if (i<7)    i++;
    else        i = 0;
    
    ang_wax = win_sum1/8;
    ang_way = win_sum2/8;

    //1st order complementary filter
    float K2 = 0.7;
    float K3 = log(0.8*sqrt(wb_x*wb_x+wb_y*wb_y)+1);
    if (K3 > 1) K3 = 1;
    
    float gain = K2;
    
    angEst_x = gain*(angEst_x+wb_x*Ts) + (1-gain)*ang_wax;
    angEst_y = gain*(angEst_y+wb_y*Ts) + (1-gain)*ang_way;
    angEst_z = ang_wbz;
    
    //angular acceleration estimator
    static float wbx_est = 0.;
    static float wby_est = 0.;
    static float wbz_est = 0.;
    
    wbx_est += wacc_x*Ts;
    wby_est += wacc_y*Ts;
    wbz_est += wacc_z*Ts;
    
    wb_x = wb_x*180/pi;
    wb_y = wb_y*180/pi;
    wb_z = wb_z*180/pi;
    
    float K4 = 150;
    wacc_x = K4*(wb_x - wbx_est);
    wacc_y = K4*(wb_y - wby_est);
    wacc_z = K4*(wb_z - wbz_est);
    
    //output
    y[0]    =   ang_wbx;
    y[1]    =   ang_wby;
    y[2]    =   ang_wbz;
    y[3]    =   ang_wax;
    y[4]    =   ang_way;
    y[5]    =   K3;
    y[6]    =   angEst_x;
    y[7]    =   angEst_y;
    y[8]    =   angEst_z;
    y[9]    =   wacc_x;
    y[10]   =   wacc_y;
    y[11]   =   wacc_z;
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
