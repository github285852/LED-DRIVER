#include "neuralpid.h"
#include "mymath.h"

void NeuralPID_SetExp(NeuralPID *npid,float exp)
{
	npid->exp = exp;
}

float SignleNeuralAdaptivPID(NeuralPID *npid,float _true)
{
	float norm,w11,w22,w33;
	
	npid->error = npid->exp - _true;
	//反馈
	switch(npid->M)
	{
		case 0:
			npid->wkp +=  npid->xiteP*npid->out*npid->op;
			npid->wki += npid->xiteI*npid->out*npid->oi;
			npid->wkd += npid->xiteD*npid->out*npid->od;
		break;
		case 1:
			npid->wkp += npid->xiteP*npid->out*npid->error;
			npid->wki += npid->xiteI*npid->out*npid->error;
			npid->wkd += npid->xiteD*npid->out*npid->error;
		break;
		case 2:
			npid->wkp += npid->xiteP*npid->out*npid->error*npid->op;
			npid->wki += npid->xiteI*npid->out*npid->error*npid->oi;
		  npid->wkd += npid->xiteD*npid->out*npid->error*npid->od;
		break;
		case 3:
			npid->wkp += npid->xiteP*npid->out*npid->error*(2*npid->error - npid->error1);
			npid->wki += npid->xiteI*npid->out*npid->error*(2*npid->error - npid->error1);
		  npid->wkd += npid->xiteD*npid->out*npid->error*(2*npid->error - npid->error1);//学习算法
		break;

		default :break;

	}

	//增量式PID
	npid->op = npid->error - npid->error1;
	npid->oi = npid->error;
	npid->od = npid->error - 2*npid->error1 + npid->error2;

	norm = ABS(npid->wkp)+ABS(npid->wki)+ABS(npid->wkd); 
	//归一化，利于收敛
	w11 = npid->wkp/norm;

	w22 = npid->wki/norm;

	w33 = npid->wkd/norm;

	//神经网络PID 计算
	npid->out += npid->MK * (w11*npid->op+w22*npid->oi+w33*npid->od);
	
	npid->error2 = npid->error1;
	npid->error1 = npid->error;

	return npid->out;
}


