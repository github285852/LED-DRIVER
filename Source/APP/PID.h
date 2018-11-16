/*
 * PID.h
 *
 *  Created on: 2016-5-16
 *      Author: Administrator
 */

#ifndef PID_H_
#define PID_H_

/*------------------------------------------
 				PID结构体
------------------------------------------*/
typedef struct PIDTypdDef
{
	float  SetPoint; 	//  设定目标 Desired Value
	double  SumError;		//	误差累计

	float  Proportion;      //  比例常数 Proportional Const
	float  Integral;        //  积分常数 Integral Const
	float  Derivative;      //  微分常数 Derivative Const

	float LastError;     //  Error[-1]
	float PrevError;     //  Error[-2]
}PID;


float PID_PosLocCalc(struct PIDTypdDef *pid,float NextPoint,float SE_limit,float T);
float PID_IncLocCalc(struct PIDTypdDef *pid,float NextPoint,float T);
void PID_SetKd(struct PIDTypdDef *pid,float dKdd);
void PID_SetKi(struct PIDTypdDef *pid,float dKii);
void PID_SetKp(struct PIDTypdDef *pid,float dKpp);
void PID_SetPoint(struct PIDTypdDef *pid,float setpoint);
void PID_Init(struct PIDTypdDef *pid);

#endif /* PID_H_ */
