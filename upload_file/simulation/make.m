%参数清单
clear
path('./icon/',path);

%动力单元参数
PowerUnit_CR = 1148;%常量参数，表示从油门到电机转速的线性关系斜率
PowerUnit_wb = -141.4;%转速常数，为从油门到电机转速线性关系中的常数项
PowerUnit_Tm = 0.02;%电机动态响应时间常数

%控制效率单元参数
ConEfficiency_drone_R = 0.225;%机身半径，单位米
ConEfficiency_drone_cT = 1.105e-05;%螺旋桨拉力系数
ConEfficiency_drone_cM = 1.779e-07;%螺旋桨力矩系数

%位置动力学参数
PosDyna_Mass = 1.4;%多旋翼总质量，单位Kg
PosDyna_GravityAcc = 9.8;%重力加速度

%姿态动力学
AttDyna_Ixx = 0.0211;%机身x轴转动惯量
AttDyna_Iyy = 0.0219;%机身y轴转动惯量
AttDyna_Izz = 0.0366;%机身z轴转动惯量
AttDyna_JRP = 0.0001287;%电机+螺旋桨的转动惯量

mex processor.c controllers.c 
mex angle_estimator.c