/*
 * PID.c
 *
 *  Created on: 2016-5-16
 *      Author: Administrator
 */
#include "PID.h"
/*------------------------------------------
 ��������:��ʼ��PID�ṹ�����
 ����˵��:
------------------------------------------*/
void PID_Init(struct PIDTypdDef *pid)
{

	pid->LastError  = 0;			//Error[-1]
	pid->PrevError  = 0;			//Error[-2]
	pid->Proportion = 0;			//�������� Proportional Const
	pid->Integral   = 0;			//���ֳ��� Integral Const
	pid->Derivative = 0;			//΢�ֳ��� Derivative Const
	pid->SetPoint   = 0;
	pid->SumError   = 0;
}
/*------------------------------------------
 ��������:����PID����ֵ
 ����˵��:
------------------------------------------*/
void PID_SetPoint(struct PIDTypdDef *pid,float setpoint)
{
	pid->SetPoint = setpoint;
}

/*------------------------------------------
 ��������:����PID����ϵ��
 ����˵��:������
------------------------------------------*/
void PID_SetKp(struct PIDTypdDef *pid,float dKpp)
{
	pid->Proportion = dKpp;
}
/*------------------------------------------
 ��������:����PID����ϵ��
 ����˵��:������
------------------------------------------*/
void PID_SetKi(struct PIDTypdDef *pid,float dKii)
{
	pid->Integral = dKii;
}
/*------------------------------------------
 ��������:����PID΢��ϵ��
 ����˵��:������
------------------------------------------*/
void PID_SetKd(struct PIDTypdDef *pid,float dKdd)
{
	pid->Derivative = dKdd;
}
/*------------------------------------------
 ��������: ����ʽPID����
 ����˵��:	ʹ��ǰ���ú�����ֵ
����˵����	PID�ṹ�壬����ֵ
------------------------------------------*/
float PID_IncLocCalc(struct PIDTypdDef *pid,float NextPoint,float T)
{
    register float  iError;

	iError = pid->SetPoint - NextPoint;        // ƫ��
  pid->PrevError = pid->LastError;
	pid->LastError = iError;

	return(float)(  pid->Proportion   * iError           	// ������
          		    + pid->Integral   * pid->LastError		// ������
          		    + pid->Derivative * pid->PrevError);
}



/*------------------------------------------
 ��������: λ��ʽPID����
 ����˵��:	ʹ��ǰ���ú�����ֵ
����˵����	PID�ṹ�壬����ֵ
------------------------------------------*/
float PID_PosLocCalc(struct PIDTypdDef *pid,float NextPoint,float SE_limit,float T)
{
    register float  iError,dError;

	iError = pid->SetPoint - NextPoint;        // ƫ��
	pid->SumError += iError;				    // ����
	if(pid->SumError > SE_limit)					//�����޷�2300
		pid->SumError = SE_limit;
	else if(pid->SumError < -SE_limit)
		pid->SumError = -SE_limit;
	dError = iError - pid->LastError; 			// ��ǰ΢��
	pid->LastError = iError;

	return(float)(  pid->Proportion   * iError           	// ������
          		    + pid->Integral   * pid->SumError 		// ������
          		    + pid->Derivative * dError);
}




