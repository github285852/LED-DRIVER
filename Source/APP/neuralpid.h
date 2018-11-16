#ifndef __NEURALPID_H
#define __NEURALPID_H




typedef struct
{
	unsigned char M;//ѡ��ģ��
	float MK;//ģ������ı���
	float xiteP;//P��ѧϰ��
	float xiteI;//I��ѧϰ��
	float xiteD;//D��ѧϰ��
	float wkp,wki,wkd; //PID�����߲���
	float op,oi,od;//PID���������
	float error1,error2,error;
	float exp;//����ֵ
	float out;//���
}NeuralPID;	

float SignleNeuralAdaptivPID(NeuralPID *npid,float _true);
void NeuralPID_SetExp(NeuralPID *npid,float exp);

#endif




