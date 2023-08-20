#ifndef _PARAMETERS_H
#define _PARAMETERS_H

#define Ts      1./100

#define pi  3.1415926
#define g   9.8

//位置控制器参数
#define ps_Kp       0.5
#define ps_Ki       0
#define ps_limt     0

#define psZ_Kp      1.5
#define psZ_Ki      0.05*Ts
#define psZ_limt    5


//速度控制器参数
#define vl_Kp       1
#define vl_Ki       0.005*Ts
#define vl_limt     30*180/pi   //角度限制

#define vlz_Kp      0.05
#define vlz_Ki      0.*Ts
#define vlz_limt    1

// 姿态控制器参数
#define angle_Kp    1.5
#define angle_Ki    0.001*Ts
#define angle_limt  0    //不做限制

#define yaw_Kp      2.5
#define yaw_Ki      0.0*Ts
#define yaw_limt    0    //不做限制

// 角速度控制器参数
#define wb_Kp       4
#define wb_Ki       0.01*Ts
#define wb_limt     0     //不做限制

#define wbz_Kp      10
#define wbz_Ki      0     //不做限制

// 角加速度控制器参数
#define wacc_Kp     0.0000002
#define wacc_Ki     0.00001*Ts
#define wacc_limt   1   //油门限制  

#define waccz_Kp    1
#define waccz_Ki    0*Ts

#endif