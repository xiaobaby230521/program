#include "My_Digital_Control_LL.h"

void SOGI_transfrom(SOGI_TypeDef *Pamer, float in)
{
	Pamer->ui0 = in;
	Pamer->ua0 = Pamer->b0*(Pamer->ui0-Pamer->ui2) + Pamer->a1*Pamer->ua1 + Pamer->a2*Pamer->ua2;
	Pamer->ua2 = Pamer->ua1;
	Pamer->ua1 = Pamer->ua0;

	Pamer->ub0 = Pamer->qb0*Pamer->ui0 + Pamer->qb1*Pamer->ui1 + Pamer->qb2*Pamer->ui2 + Pamer->a1*Pamer->ub1 + Pamer->a2*Pamer->ub2;
	Pamer->ub2 = Pamer->ub1;
	Pamer->ub1 = Pamer->ub0;

	Pamer->ui2 = Pamer->ui1;
	Pamer->ui1 = Pamer->ui0;
	
	/*
	alpha = Pamer->ua0;
	beta = Pamer->ub0;
	*/
}

void SOGI_Parameter_Init(SOGI_TypeDef *Pamer, float k, float Ts, float w)
{
	
	Pamer->x = 2*k*w*Ts;
	Pamer->y = w*w*Ts*Ts;
	Pamer->b0 = Pamer->x/(Pamer->x+Pamer->y+4);
	Pamer->b2 = -Pamer->b0;
	Pamer->a1 = (8-2*Pamer->y)/(Pamer->x+Pamer->y+4);
	Pamer->a2 = (Pamer->x-Pamer->y-4)/(Pamer->x+Pamer->y+4);
	Pamer->qb0 = k*Pamer->y/(Pamer->x+Pamer->y+4);
	Pamer->qb1 = 2*Pamer->qb0;
	Pamer->qb2 = Pamer->qb0;
}

void Vg_GetVpp_Init(GETVPP_TypeDef *_Pamer, uint16_t _X)
{
	_Pamer->cin[1] = _Pamer->cin[0];
	_Pamer->cin[0] = _X;
	if(_Pamer->cin[0]>_Pamer->cin[1])
	{
		if(_Pamer->stage == 1)
		{
			_Pamer->vmn[_Pamer->j_Vpp] = _Pamer->vmn_tmp;
			_Pamer->j_Vpp++;
		}
		_Pamer->vmp_tmp = _Pamer->cin[0];
		_Pamer->stage = 2;
	}

	else if(_Pamer->cin[0]<_Pamer->cin[1])
	{
		if(_Pamer->stage == 2)
		{
			_Pamer->vmp[_Pamer->i_Vpp] = _Pamer->vmp_tmp;
			_Pamer->i_Vpp++;
		}
		_Pamer->vmn_tmp = _Pamer->cin[0];
		_Pamer->stage = 1;
	}

	if(_Pamer->i_Vpp >= 4 && _Pamer->j_Vpp >= 4)
	{
		uint16_t tmp1;
		uint16_t tmp2;
		
		int8_t ac1;
		ac1 = _Pamer->vmp[0] - _Pamer->vmp[3];	
		tmp1 = (_Pamer->vmp[0] + _Pamer->vmp[1] + _Pamer->vmp[2] + _Pamer->vmp[3])>>2;
		tmp2 = (_Pamer->vmn[0] + _Pamer->vmn[1] + _Pamer->vmn[2] + _Pamer->vmn[3])>>2;
		
		_Pamer->i_Vpp = 0;
		_Pamer->j_Vpp = 0;
		_Pamer->Vg_Vpp = (tmp1-tmp2)>>1;
		if(_Pamer->Vg_Vpp<=10)
			_Pamer->Vg_Vpp = 10;
		_Pamer->DC_Offest = (tmp1+tmp2)>>1;
		
		if(ac1 <=80 && ac1 >=-80 && _Pamer->Vg_Vpp >=350)
		{
			_Pamer->EN_SOGI = 1;
		}
		else
			_Pamer->EN_SOGI = 0;
		
		
	}
}
//传入归一化叫角度wt进行DQ变换
void alpha_bata_to_DQ(float alpha, float beta, float wt, float DQ[2])
{
	int16_t Q15_wt = wt*0x8000;
	DQ[0] = (float) (alpha*arm_sin_q15(Q15_wt) - beta*arm_cos_q15(Q15_wt))/0x8000;
	DQ[1] = (float) (alpha*arm_cos_q15(Q15_wt) + beta*arm_sin_q15(Q15_wt))/0x8000;
}


void f32_PI_Init(PI_TypeDef *PI_Pamer ,float Ts, float Kp, float Ki, int16_t TH, int16_t TL)
{
	PI_Pamer->Ts = Ts;
	PI_Pamer->Kp = Kp;
	PI_Pamer->Ki = Ki;
	PI_Pamer->filter_B0 = Ts*Ki/2 + Kp;
	PI_Pamer->filter_B1 = Ts*Ki/2 - Kp;
	PI_Pamer->TH = TH;
	PI_Pamer->TL = TL;
}

float f32_PI_Calculate(PI_TypeDef *PI_Pamer ,float REF, float Sample)
{
	
	PI_Pamer->x1 = PI_Pamer->x0;
	PI_Pamer->x0 = (REF - Sample);
	
	float tmp_co;
	//tmp_co = PI_Pamer->y1 + PI_Pamer->Kp*(PI_Pamer->x0-PI_Pamer->x1) + PI_Pamer->Ts*PI_Pamer->Ki*(PI_Pamer->x0+PI_Pamer->x1)/2;
	tmp_co = PI_Pamer->y1 + PI_Pamer->filter_B0*PI_Pamer->x0 + PI_Pamer->filter_B1*PI_Pamer->x1;
	
	if(tmp_co >= PI_Pamer->TH)
		PI_Pamer->y0 = PI_Pamer->TH;
	else if(tmp_co <= PI_Pamer->TL)
		PI_Pamer->y0 = PI_Pamer->TL;
	else
		PI_Pamer->y0 = tmp_co;
	
	PI_Pamer->y1 = PI_Pamer->y0;
	return PI_Pamer->y0;
}

void f32_Integral_Init(Integral_TypeDef *I_pamer, float _Ts, float Intergral_MAX)
{
	I_pamer->Ts = _Ts;
	I_pamer->Ki = _Ts/2;
	I_pamer->Integral_MAX = Intergral_MAX;
}

float f32_Integral_Calculate(Integral_TypeDef *I_pamer, float _X)
{
	float I_tmp;
	I_pamer->x1 = I_pamer->x0;	
	I_pamer->x0 = _X;
	I_tmp = I_pamer->Ki*(I_pamer->x0 + I_pamer->x1) + I_pamer->y1;
	I_pamer->y0 = MyFmod(I_tmp, I_pamer->Integral_MAX);
	I_pamer->y1 = I_pamer->y0;
	return I_pamer->y0;
}

void f32_Type2_Init(Type2_TypeDef *Type2_Pamer, float _A_filter[2], float _B_filter[3], float _K, float _TH, float _TL)
{
	Type2_Pamer->filter_A1 = _A_filter[0];
	Type2_Pamer->filter_A2 = _A_filter[1];
	Type2_Pamer->filter_B0 = _B_filter[0];
	Type2_Pamer->filter_B1 = _B_filter[1];
	Type2_Pamer->filter_B2 = _B_filter[2];
	Type2_Pamer->k = _K;
	Type2_Pamer->TH = _TH;
	Type2_Pamer->TL = _TL;
}

void f32_Type3_Init(Type3_TypeDef *Type3_Pamer, float _A_filter[3], float _B_filter[4], float _K, float _TH, float _TL)
{
	Type3_Pamer->filter_A1 = _A_filter[0];
	Type3_Pamer->filter_A2 = _A_filter[1];
	Type3_Pamer->filter_A3 = _A_filter[2];
	
	Type3_Pamer->filter_B0 = _B_filter[0];
	Type3_Pamer->filter_B1 = _B_filter[1];
	Type3_Pamer->filter_B2 = _B_filter[2];
	Type3_Pamer->filter_B3 = _B_filter[3];
	Type3_Pamer->k = _K;
	Type3_Pamer->TH = _TH;
	Type3_Pamer->TL = _TL;
}

float f32_Type2_Calculate(Type2_TypeDef *Type2_Pamer, float _REF, float _Sample)
{
	Type2_Pamer->x[2] = Type2_Pamer->x[1];
	Type2_Pamer->x[1] = Type2_Pamer->x[0];
	Type2_Pamer->x[0] = (_REF - _Sample)*Type2_Pamer->k;
	float tmp_co = Type2_Pamer->filter_A1 * Type2_Pamer->y[1] + Type2_Pamer->filter_A2 * Type2_Pamer->y[2]
		+(Type2_Pamer->filter_B0 * Type2_Pamer->x[0] + Type2_Pamer->filter_B1 * Type2_Pamer->x[1] + Type2_Pamer->filter_B2 * Type2_Pamer->x[2]);
	
	if (tmp_co >= Type2_Pamer->TH)
		Type2_Pamer->y[0] = Type2_Pamer->TH;
	else if(tmp_co <= Type2_Pamer->TL)
		Type2_Pamer->y[0] = Type2_Pamer->TL;
	else
		Type2_Pamer->y[0] = tmp_co;
	
	Type2_Pamer->y[2] = Type2_Pamer->y[1];
	Type2_Pamer->y[1] = Type2_Pamer->y[0];
	return Type2_Pamer->y[0];
}


float f32_Type3_Calculate(Type3_TypeDef *Type3_Pamer, float _REF, float _Sample)
{
	Type3_Pamer->x[3] = Type3_Pamer->x[2];
	Type3_Pamer->x[2] = Type3_Pamer->x[1];
	Type3_Pamer->x[1] = Type3_Pamer->x[0];
	Type3_Pamer->x[0] = (_REF - _Sample)*Type3_Pamer->k;
	float tmp_co = Type3_Pamer->filter_A1 * Type3_Pamer->y[1] + Type3_Pamer->filter_A2 * Type3_Pamer->y[2] + Type3_Pamer->filter_A3 * Type3_Pamer->y[3]
		+(Type3_Pamer->filter_B0 * Type3_Pamer->x[0] + Type3_Pamer->filter_B1 * Type3_Pamer->x[1] + Type3_Pamer->filter_B2 * Type3_Pamer->x[2] + + Type3_Pamer->filter_B3 * Type3_Pamer->x[3]);
	
	if (tmp_co >= Type3_Pamer->TH)
		Type3_Pamer->y[0] = Type3_Pamer->TH;
	else if(tmp_co <= Type3_Pamer->TL)
		Type3_Pamer->y[0] = Type3_Pamer->TL;
	else
		Type3_Pamer->y[0] = tmp_co;
	Type3_Pamer->y[3] = Type3_Pamer->y[2];
	Type3_Pamer->y[2] = Type3_Pamer->y[1];
	Type3_Pamer->y[1] = Type3_Pamer->y[0];
	return Type3_Pamer->y[0];
}
//单极性调制输入范围为-1到1，输出为占空比
uint32_t single_polarity_Duty_Gen(float input, uint32_t Counter_MAX, int8_t State)
{
	uint32_t D_Tmp;
	if(State == 1)
	{
		D_Tmp = input*Counter_MAX;
	}
	else if(State == -1)
	{
		D_Tmp = (input + 1.0f)*Counter_MAX;
	}
	
	if(D_Tmp<=2000)
	{
		D_Tmp = 2000;
	}
	else if(D_Tmp>=Counter_MAX - 2000)
	{
		D_Tmp = Counter_MAX - 2000;
	}
	return D_Tmp;
}
//单极性PFC调制，输入范围为-1到1，输出为占空比，State = 1代表交流正半轴，为-1代表交流负半轴
uint32_t single_polarity_PFC_Duty_Gen(float input, uint32_t Counter_MAX, uint32_t D_MAX, uint32_t D_MIN, int8_t State)
{
	uint32_t D_Tmp;
	if(State == 1)
	{
		D_Tmp = (1 + input)*Counter_MAX;
	}
	else if(State == -1)
	{
		D_Tmp = (1 - input)*Counter_MAX;
	}
	
	if(D_Tmp<=D_MIN)
	{
		D_Tmp = D_MIN;
	}
	else if(D_Tmp>=D_MAX)
	{
		D_Tmp = D_MAX;
	}
	return D_Tmp;
}
//单极性逆变调制，输入范围为-1到1，输出为占空比，State = 1代表交流正半轴，为-1代表交流负半轴
uint32_t single_polarity_Inv_Duty_Gen(float input, uint32_t Counter_MAX, uint32_t D_MAX, uint32_t D_MIN, int8_t State)
{
	uint32_t D_Tmp;
	if(State == 1)
	{
		D_Tmp = input*Counter_MAX;
	}
	else if(State == -1)
	{
		D_Tmp = (-input)*Counter_MAX;
	}
	
	if(D_Tmp<=D_MIN)
	{
		D_Tmp = D_MIN;
	}
	else if(D_Tmp>=D_MAX)
	{
		D_Tmp = D_MAX;
	}
	return D_Tmp;
}

//双极性调制输入范围为-1到1，输出为占空比
uint32_t bipolar_Polarity_Duty_Gen(float input, int32_t Counter_MAX, int32_t D_MAX, int32_t D_Min)
{
	float D_tmp = (input + 1.f)*(Counter_MAX>>1);
	if(D_tmp >= D_MAX)
	{
		D_tmp = D_MAX;
	}
	else if (D_tmp <= D_Min)
	{
		D_tmp = D_Min;
	}
	return D_tmp;
}

//ABC到DQ变换，输入wt的范围为归一化的[0~1]

void ABC_to_Dq(float DQ[2], float ABC[3], float _wt)
{
	float f32_Cordic_input[3];
	int32_t Cordic_input[3];
	int32_t Cordic_output[6];
	
	f32_Cordic_input[0] = _wt;
	f32_Cordic_input[1] = MyFmod(_wt + 0.666667f, 1.0f);
	f32_Cordic_input[2] = MyFmod(_wt + 0.333333f, 1.0f);
	
	value_to_cordic31(f32_Cordic_input,Cordic_input, 3);
	My_Cordic_Calculate(Cordic_input, Cordic_output, 3);
	DQ[0] = (float)0.6667f*(Cordic_output[1]*ABC[0] + Cordic_output[3]*ABC[1] + Cordic_output[5]*ABC[2])/0x80000000;
	DQ[1] = (float)0.6667f*(Cordic_output[0]*ABC[0] + Cordic_output[2]*ABC[1] + Cordic_output[4]*ABC[2])/0x80000000;
	
}

//DQ到ABC变换，输入wt的范围为归一化的[0~1]

void DQ_to_ABC(float DQ[2], float ABC[3], float _wt)
{
	float f32_Cordic_input[3];
	int32_t Cordic_input[3];
	int32_t Cordic_output[6];
	
	f32_Cordic_input[0] = _wt;
	f32_Cordic_input[1] = MyFmod(_wt + 0.666667f, 1.0f);
	f32_Cordic_input[2] = MyFmod(_wt + 0.333333f, 1.0f);
	
	value_to_cordic31(f32_Cordic_input,Cordic_input, 3);
	My_Cordic_Calculate(Cordic_input, Cordic_output, 3);
	
	ABC[0] = (float)(Cordic_output[1]*DQ[0] + Cordic_output[0]*DQ[1])/0x80000000;
	ABC[1] = (float)(Cordic_output[3]*DQ[0] + Cordic_output[2]*DQ[1])/0x80000000;
	ABC[2] = (float)(Cordic_output[5]*DQ[0] + Cordic_output[4]*DQ[1])/0x80000000;
	
}


//DQ控制的三相变换器中，三角函数是固定的，这里进行计算，方便后续处理
void Three_Phase_Convent_Trigonometric_Calculate(float Trigonometric[6], float _wt)
{
	float f32_Cordic_input[3];
	int32_t Cordic_input[3];
	int32_t Cordic_output[6];
	
	f32_Cordic_input[0] = _wt;
	f32_Cordic_input[1] = MyFmod(_wt + 0.666667f, 1.0f);
	f32_Cordic_input[2] = MyFmod(_wt + 0.333333f, 1.0f);
	value_to_cordic31(f32_Cordic_input,Cordic_input, 3);
	My_Cordic_Calculate(Cordic_input, Cordic_output, 3);
	cordic31_to_value(Cordic_output, Trigonometric, 6);
}

//ABC到DQ的变换，三角函数已经计算过
void ABC_to_Dq_Simple(float _DQ[2], float _ABC[3], float _Trigonometric[6])
{
	_DQ[0] = (_ABC[0]*_Trigonometric[1] + _ABC[1]*_Trigonometric[3]+ _ABC[2]*_Trigonometric[5])*0.66667f;
	_DQ[1] = (_ABC[0]*_Trigonometric[0] + _ABC[1]*_Trigonometric[2]+ _ABC[2]*_Trigonometric[4])*0.66667f;
}

//DQ到ABC的变换，三角函数已经计算过
void DQ_TO_ABC_Simple(float _DQ[2], float _ABC[3], float _Trigonometric[6])
{
	_ABC[0] = (_DQ[0]*_Trigonometric[1] + _DQ[1]*_Trigonometric[0]);
	_ABC[1] = (_DQ[0]*_Trigonometric[3] + _DQ[1]*_Trigonometric[2]);
	_ABC[2] = (_DQ[0]*_Trigonometric[5] + _DQ[1]*_Trigonometric[4]);
}

float Three_Phase_PLL(PI_TypeDef *PLL_PI_Pamer, float UQ, float fc)
{
	float tmp_A1, wt;
	tmp_A1 = f32_PI_Calculate(PLL_PI_Pamer,0, UQ) + 6.2832f*fc;
    wt = MyFmod(wt + 50*2e-5f, 1.f);
	return wt;
}

//单相DQ锁相环
///////////////////////////////////////////
float Single_Phase_PLL_ARMMATH(GridPLL_TypeDef* PLL_Pamer, float Cin)
{
	SOGI_transfrom(&(PLL_Pamer->SOGI_Pamer), Cin);
	int16_t Q15_Phase = (int16_t)(PLL_Pamer->PLL_Phase*0x8000);
	PLL_Pamer->PLL_DQ[1] = (float) (PLL_Pamer->SOGI_Pamer.ua0*arm_cos_q15(Q15_Phase) + PLL_Pamer->SOGI_Pamer.ub0*arm_sin_q15(Q15_Phase))/0x8000;
	PLL_Pamer->PLL_Wt = f32_PI_Calculate(&(PLL_Pamer->PI_Pamer), 0, PLL_Pamer->PLL_DQ[1]) + 50.f;
	PLL_Pamer->PLL_Phase = MyFmod((PLL_Pamer->PLL_Phase + PLL_Pamer->PLL_Wt*PLL_Pamer->Sample_Time), 1);
	if(PLL_Pamer->PLL_DQ[1]>=-0.05f && PLL_Pamer->PLL_DQ[1]<=0.05f)
	{
		if(PLL_Pamer->PLL_Counter >= PLL_Pamer->PLL_CounterMax)
			PLL_Pamer->PLL_FLAG = 1;
		else
		{
			if(PLL_Pamer->PLL_FLAG == 1)
			{
				PLL_Pamer->PLL_FLAG = 0;
				PLL_Pamer->PLL_Counter = 0;
			}
			PLL_Pamer->PLL_Counter++;
		}
	}
	return PLL_Pamer->PLL_Phase;
}

float Single_Phase_PLL_Cordic(GridPLL_TypeDef* PLL_Pamer, float Cin)
{
	float tmp_Phase;
	int32_t Cordic_output[2];
	SOGI_transfrom(&(PLL_Pamer->SOGI_Pamer), Cin);
	if(PLL_Pamer->PLL_Phase > 0.5f)								//将角度从[0, 1.0]转换到[-0.5, 0.5]
		tmp_Phase = PLL_Pamer->PLL_Phase - 1.0f;
	else
		tmp_Phase = PLL_Pamer->PLL_Phase;
	
	int32_t Cordic_input = (int32_t)(tmp_Phase*0x100000000);
	CORDIC->WDATA = Cordic_input;
	Cordic_output[0] = CORDIC->RDATA;
	Cordic_output[1] = CORDIC->RDATA;	
	PLL_Pamer->PLL_DQ[0] = (float)(PLL_Pamer->SOGI_Pamer.ua0*Cordic_output[0] - PLL_Pamer->SOGI_Pamer.ub0*Cordic_output[1])/0x80000000;
	PLL_Pamer->PLL_DQ[1] = (float)(PLL_Pamer->SOGI_Pamer.ua0*Cordic_output[1] + PLL_Pamer->SOGI_Pamer.ub0*Cordic_output[0])/0x80000000;
	PLL_Pamer->PLL_Wt = f32_PI_Calculate(&(PLL_Pamer->PI_Pamer), 0, PLL_Pamer->PLL_DQ[1]) + 50.f;
	PLL_Pamer->PLL_Phase = MyFmod((PLL_Pamer->PLL_Phase + PLL_Pamer->PLL_Wt*PLL_Pamer->Sample_Time), 1);
	if(PLL_Pamer->PLL_DQ[1]>=-0.05f && PLL_Pamer->PLL_DQ[1]<=0.05f && PLL_Pamer->PLL_DQ[0]>=0.5f)
	{
		if(PLL_Pamer->PLL_Counter >= PLL_Pamer->PLL_CounterMax)
			PLL_Pamer->PLL_FLAG = 1;
		else
		{
			PLL_Pamer->PLL_FLAG = 0;
			PLL_Pamer->PLL_Counter++;
		}
	}
	else
	{
		PLL_Pamer->PLL_FLAG = 0;
		PLL_Pamer->PLL_Counter = 0;
	}
	return PLL_Pamer->PLL_Phase;
}

void Single_Phase_PLL_Init(GridPLL_TypeDef* PLL_Pamer, float Frequency, float Sample_Time, uint16_t PLL_CounterMax)
{
	PLL_Pamer->Sample_Time = Sample_Time;
	SOGI_Parameter_Init(&(PLL_Pamer->SOGI_Pamer), 0.4f, Sample_Time, Frequency*6.283185f);
	f32_PI_Init(&(PLL_Pamer->PI_Pamer), Sample_Time, -10, -800, 50 ,-50);
	PLL_Pamer->PLL_CounterMax = PLL_CounterMax;
}
///////////////////////////////////////////

float f32_SinWave_Gen(uint16_t fc, float *wt, float Ts)
{
	*wt = MyFmod(*wt + Ts*fc, 1.0f);
	return arm_sin_q15((*wt)*32768)/32768.f;
}

void AC_Stage_Discern(AC_Discern_TypeDef *AC_Discern_Pamer, float Vac_PLL, float PLL_UQ, _Bool Other_Criteria)
{
	float derta_V = AC_Discern_Pamer->derta_VAC;
	if(AC_Discern_Pamer->AC_State==0 && Vac_PLL>=derta_V)								//状态切换为交流输入正半周
	{
		AC_Discern_Pamer->AC_State = 1;
	}
	else if(AC_Discern_Pamer->AC_State == 1 && (Vac_PLL<derta_V && Vac_PLL>-derta_V))	//状态切换为交流输入过零
	{
		AC_Discern_Pamer->AC_State = 2;
	}
	else if(AC_Discern_Pamer->AC_State == 2 && Vac_PLL<=-derta_V*2)						//状态切换为交流输入负半周
	{
		AC_Discern_Pamer->AC_State = 3;
		if(PLL_UQ >=-0.05f && PLL_UQ<=0.05f)
		{
			if(AC_Discern_Pamer->PLL_Discern_Counter >= AC_Discern_Pamer->PLL_Discern_Counter_MAX)
			{
				if(Other_Criteria == 1)													//在其他条件都满足的情况下，在交流过零点开始启动
					AC_Discern_Pamer->Covernt_EN = 1;
				else
					AC_Discern_Pamer->Covernt_EN = 0;
			}
			else
			{
				AC_Discern_Pamer->Covernt_EN = 0;
				AC_Discern_Pamer->PLL_Discern_Counter++;
			}
		}
		else
		{
			AC_Discern_Pamer->Covernt_EN = 0;
			AC_Discern_Pamer->PLL_Discern_Counter = 0;
		}
		
	}
	else if(AC_Discern_Pamer->AC_State == 3 && (Vac_PLL>-derta_V && Vac_PLL<derta_V))	//状态切换为交流输入过零
	{
		AC_Discern_Pamer->AC_State = 4;
	}
	else if(AC_Discern_Pamer->AC_State == 4 && Vac_PLL>=derta_V*2)						//状态切换为交流输入正半周
	{
		AC_Discern_Pamer->AC_State = 1;
	}
}

float MPPT_P_Q_Calculate(MPPT_TypeDef *MPPT_Pamer, float V_in, float I_in)
{
	MPPT_Pamer->I_in[0] = I_in;
	MPPT_Pamer->V_in[0] = V_in;
	MPPT_Pamer->P_in[0] = (float)I_in*V_in;
	if(MPPT_Pamer->P_in[0]>MPPT_Pamer->P_in[1])
	{
		if(MPPT_Pamer->V_in[0]>MPPT_Pamer->V_in[1])
			MPPT_Pamer->D_OUT= MPPT_Pamer->D_OUT - MPPT_Pamer->deta;
		else
			MPPT_Pamer->D_OUT= MPPT_Pamer->D_OUT + MPPT_Pamer->deta;
	}
	else
	{
		if(MPPT_Pamer->V_in[0]<MPPT_Pamer->V_in[1])
			MPPT_Pamer->D_OUT= MPPT_Pamer->D_OUT - MPPT_Pamer->deta;
		else
			MPPT_Pamer->D_OUT= MPPT_Pamer->D_OUT + MPPT_Pamer->deta;
	}
	if(MPPT_Pamer->D_OUT >= MPPT_Pamer->D_MAX)
	{
		MPPT_Pamer->D_OUT = MPPT_Pamer->D_MAX;
	}
	else if(MPPT_Pamer->D_OUT <= MPPT_Pamer->D_MIN)
		MPPT_Pamer->D_OUT = MPPT_Pamer->D_MIN;

	MPPT_Pamer->I_in[1] = MPPT_Pamer->I_in[0];
	MPPT_Pamer->V_in[1] = MPPT_Pamer->V_in[0];
	MPPT_Pamer->P_in[1] = MPPT_Pamer->P_in[0];
	return MPPT_Pamer->D_OUT;
}

float Trip_Filter_Calculate(Type2_TypeDef *Type2_Pamer,float _Sample)
{
	Type2_Pamer->x[2] = Type2_Pamer->x[1];
	Type2_Pamer->x[1] = Type2_Pamer->x[0];
	Type2_Pamer->x[0] = _Sample;
	Type2_Pamer->y[0] = Type2_Pamer->filter_A1 * Type2_Pamer->y[1] + Type2_Pamer->filter_A2 * Type2_Pamer->y[2]
		+(Type2_Pamer->filter_B0 * Type2_Pamer->x[0] + Type2_Pamer->filter_B1 * Type2_Pamer->x[1] + Type2_Pamer->filter_B2 * Type2_Pamer->x[2]);
	
	Type2_Pamer->y[2] = Type2_Pamer->y[1];
	Type2_Pamer->y[1] = Type2_Pamer->y[0];
	return Type2_Pamer->y[0];
}

void f32_LPF_Init(LPF_TypeDef *LPF_Pamer, float wc, uint8_t Order, float Ts)
{
	LPF_Pamer->Order = Order;
	float tmp_fp = wc*Ts;
	if(Order == 1)
	{
		float tmp_c = Ts*wc + 2;
		LPF_Pamer->Afilter[1] = -(tmp_fp - 2)/tmp_c;
		
		LPF_Pamer->Bfilter[0] = tmp_fp/tmp_c;
		LPF_Pamer->Bfilter[1] = LPF_Pamer->Bfilter[0];
	}
	else if(Order == 2)
	{
		float tmp_c  = (tmp_fp*tmp_fp) + 2.8284f*wc*Ts + 4;
		LPF_Pamer->Afilter[1] = -(2*tmp_fp*tmp_fp -8)/tmp_c;
		LPF_Pamer->Afilter[2] = -(tmp_fp*tmp_fp - 2.8284f*tmp_fp + 4)/tmp_c;
		
		LPF_Pamer->Bfilter[0] = (tmp_fp*tmp_fp)/tmp_c;
		LPF_Pamer->Bfilter[1] = 2*LPF_Pamer->Bfilter[0];
		LPF_Pamer->Bfilter[2] = LPF_Pamer->Bfilter[0];
	}
}

float f32_LPF_Calculate(LPF_TypeDef *LPF_Pamer, float _Sample)
{
	if(LPF_Pamer->Order == 1)
	{
		LPF_Pamer->x[1] = LPF_Pamer->x[0];
		LPF_Pamer->x[0] = _Sample;
		LPF_Pamer->y[0] = LPF_Pamer->Afilter[1]*LPF_Pamer->y[1] 
			+ LPF_Pamer->Bfilter[0]*LPF_Pamer->x[0] + LPF_Pamer->Bfilter[1]*LPF_Pamer->x[1];
		
		LPF_Pamer->y[1] = LPF_Pamer->y[0];
		return LPF_Pamer->y[0];
	}
	else if(LPF_Pamer->Order == 2)
	{
		LPF_Pamer->x[2] = LPF_Pamer->x[1];
		LPF_Pamer->x[1] = LPF_Pamer->x[0];
		LPF_Pamer->x[0] = _Sample;
		LPF_Pamer->y[0] = LPF_Pamer->Afilter[1] * LPF_Pamer->y[1] + LPF_Pamer->Afilter[2] * LPF_Pamer->y[2]
			+(LPF_Pamer->Bfilter[0] * LPF_Pamer->x[0] + LPF_Pamer->Bfilter[1] * LPF_Pamer->x[1] + LPF_Pamer->Bfilter[2] * LPF_Pamer->x[2]);
		
		LPF_Pamer->y[2] = LPF_Pamer->y[1];
		LPF_Pamer->y[1] = LPF_Pamer->y[0];
		return LPF_Pamer->y[0];
	}
	else
		return _Sample;
}

void Totem_polePFC_StageControl(Totem_PolePFC_TypeDef *Totem_PolePFC_Pamer, float VAC_PLL, _Bool Other_Criteria)
{
	if(!Other_Criteria)
		Totem_PolePFC_Pamer->Covernt_EN = 0;
	//状态切换为交流输入正半周
	if(Totem_PolePFC_Pamer->PFC_Stage==0 && VAC_PLL>=Totem_PolePFC_Pamer->deta_V)														
	{
		Totem_PolePFC_Pamer->PFC_Stage = 1;
	}
	//状态切换为交流输入向下过零
	else if(Totem_PolePFC_Pamer->PFC_Stage == 1 && (VAC_PLL < Totem_PolePFC_Pamer->deta_V && VAC_PLL > -Totem_PolePFC_Pamer->deta_V))	
	{
		if(Other_Criteria)
		{
			Totem_PolePFC_Pamer->Covernt_EN = 1;
		}
		Totem_PolePFC_Pamer->PFC_Stage = 2;
	}
	//状态切换为交流输入负半周开始软启动
	else if(Totem_PolePFC_Pamer->PFC_Stage == 2 && VAC_PLL<= -Totem_PolePFC_Pamer->deta_V)												
	{
		Totem_PolePFC_Pamer->PFC_Stage = 3;
	}
	//负半周软起动中
	else if(Totem_PolePFC_Pamer->PFC_Stage == 3)	
	{
		if(Totem_PolePFC_Pamer->PFC_SFST_Counter>=10)
		{
			Totem_PolePFC_Pamer->PFC_Stage = 4;
			Totem_PolePFC_Pamer->PFC_SFST_Counter = 0;
			Totem_PolePFC_Pamer->PFC_SFST_Value = 0;
		}
		else
		{
			Totem_PolePFC_Pamer->PFC_SFST_Counter++;
			Totem_PolePFC_Pamer->PFC_SFST_Value = Totem_PolePFC_Pamer->PFC_SFST_Value + Totem_PolePFC_Pamer->PFC_SFST_Step;
		}
	}
	//状态切换为交流输入向上过零
	else if(Totem_PolePFC_Pamer->PFC_Stage == 4 && (VAC_PLL > -Totem_PolePFC_Pamer->deta_V && VAC_PLL < Totem_PolePFC_Pamer->deta_V))	
	{
		Totem_PolePFC_Pamer->PFC_Stage = 5;
	}
	//状态切换为交流输入正半周开始软起动
	else if(Totem_PolePFC_Pamer->PFC_Stage == 5 && VAC_PLL >= Totem_PolePFC_Pamer->deta_V)		
	{
		Totem_PolePFC_Pamer->PFC_Stage = 6;
	}	
	//正半周软起动中
	else if(Totem_PolePFC_Pamer->PFC_Stage == 6)	
	{
		if(Totem_PolePFC_Pamer->PFC_SFST_Counter>=10)
		{
			Totem_PolePFC_Pamer->PFC_Stage = 1;
			Totem_PolePFC_Pamer->PFC_SFST_Counter = 0;
			Totem_PolePFC_Pamer->PFC_SFST_Value = 0;
		}
		else
		{	
			Totem_PolePFC_Pamer->PFC_SFST_Counter++;
			Totem_PolePFC_Pamer->PFC_SFST_Value = Totem_PolePFC_Pamer->PFC_SFST_Value + Totem_PolePFC_Pamer->PFC_SFST_Step;
		}
	}
}

void single_inverter_control(siginv_TypeDef *siginv_Pamer, float VAC_PLL, _Bool Other_Criteria)
{
	if(!Other_Criteria)
	{
		siginv_Pamer->Covernt_EN = 0;
		siginv_Pamer->LSS_Charge_Counter = 0;
		siginv_Pamer->HSS_FLAG = 0;
	}
	
	if(siginv_Pamer->siginv_Stage==0 && VAC_PLL >= siginv_Pamer->deta_V)			//状态切换为交流输入正半周
	{
		siginv_Pamer->siginv_Stage = 1;
	}
	else if(siginv_Pamer->siginv_Stage == 1 && (VAC_PLL < siginv_Pamer->deta_V && VAC_PLL > -siginv_Pamer->deta_V))	//状态切换为交流输入向下过零
	{
		if(Other_Criteria)
		{
			siginv_Pamer->Covernt_EN = 1;
			if(siginv_Pamer->LSS_Charge_Counter>=10)
			{
				siginv_Pamer->HSS_FLAG = 1;
			}
			else
			{
				siginv_Pamer->LSS_Charge_Counter++;
			}
		}
		siginv_Pamer->siginv_Stage = 2;
	}
	else if(siginv_Pamer->siginv_Stage == 2 && VAC_PLL<=-siginv_Pamer->deta_V)	//交流负半轴
	{
		siginv_Pamer->siginv_Stage = 3;
	}
	else if(siginv_Pamer->siginv_Stage == 3)	
	{
		if(siginv_Pamer->siginv_SFST_Counter>=10)
		{
			siginv_Pamer->siginv_Stage = 4;
			siginv_Pamer->siginv_SFST_Counter = 0;
		}
		else
		{
			siginv_Pamer->siginv_SFST_Counter++;
		}
	}
	else if(siginv_Pamer->siginv_Stage == 4 && (VAC_PLL > -siginv_Pamer->deta_V && VAC_PLL < siginv_Pamer->deta_V))	//状态切换为交流输入向上过零
	{
		siginv_Pamer->siginv_Stage = 5;
	}
	else if(siginv_Pamer->siginv_Stage == 5 && VAC_PLL >= siginv_Pamer->deta_V)	//交流正半轴
	{
		siginv_Pamer->siginv_Stage = 6;
	}
	else if(siginv_Pamer->siginv_Stage == 6)	
	{
		if(siginv_Pamer->siginv_SFST_Counter>=10)
		{
			siginv_Pamer->siginv_Stage = 1;
			siginv_Pamer->siginv_SFST_Counter = 0;
		}
		else
		{
			siginv_Pamer->siginv_SFST_Counter++;
		}
	}
}
