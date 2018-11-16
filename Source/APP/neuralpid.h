#ifndef __NEURALPID_H
#define __NEURALPID_H




typedef struct
{
	unsigned char M;//选择模型
	float MK;//模型输出的比例
	float xiteP;//P的学习率
	float xiteI;//I的学习率
	float xiteD;//D的学习率
	float wkp,wki,wkd; //PID，三者参数
	float op,oi,od;//PID三者输出量
	float error1,error2,error;
	float exp;//期望值
	float out;//输出
}NeuralPID;	

float SignleNeuralAdaptivPID(NeuralPID *npid,float _true);
void NeuralPID_SetExp(NeuralPID *npid,float exp);

#endif




