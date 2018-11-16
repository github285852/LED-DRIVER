/*
 * PID.h
 *
 *  Created on: 2016-5-16
 *      Author: Administrator
 */

#ifndef PID_H_
#define PID_H_

/*------------------------------------------
 				PID�ṹ��
------------------------------------------*/
typedef struct PIDTypdDef
{
	float  SetPoint; 	//  �趨Ŀ�� Desired Value
	double  SumError;		//	����ۼ�

	float  Proportion;      //  �������� Proportional Const
	float  Integral;        //  ���ֳ��� Integral Const
	float  Derivative;      //  ΢�ֳ��� Derivative Const

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
