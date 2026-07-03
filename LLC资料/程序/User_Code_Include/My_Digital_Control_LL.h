#ifndef __My_Digital_Control_LL_H
#define __My_Digital_Control_LL_H
#endif

#include "arm_math.h"
#include "My_cordic_LL.h"
#define CCMRAM __attribute__((section("ccmram"))) 
typedef struct
{
	float x, y;
	float a1, a2;
	float b0, b2;
	float qb0, qb1, qb2;
	float ui0, ui1, ui2;
	float ua0, ua1, ua2;
	float ub0, ub1, ub2;
} SOGI_TypeDef;


typedef struct
{
	uint16_t vmp_tmp, vmn_tmp;
	uint16_t vmp[6], vmn[6];
	uint8_t i_Vpp,j_Vpp;
	uint16_t cin[2];
	uint8_t stage;
	uint16_t Vg_Vpp;
	uint16_t DC_Offest;
	_Bool EN_SOGI;

} GETVPP_TypeDef;

typedef struct
{
	float x0, x1;
	float Kp, Ki ,Ts;
	float y0, y1;
	float filter_B0, filter_B1;
	int16_t TH;
	int16_t TL;
} PI_TypeDef;

typedef struct
{
	float x0, x1;
	float Ts;
	float Ki;
	float y0, y1;
	float Integral_MAX;
} Integral_TypeDef;

typedef struct
{
	float x[3];
	float y[3];
	float filter_B0 , filter_B1, filter_B2, filter_A1, filter_A2;
	float k;
	float TH,TL;
} Type2_TypeDef;

typedef struct
{
	float x[4];
	float y[4];
	float filter_B0 , filter_B1, filter_B2 , filter_B3 ,  filter_A1, filter_A2, filter_A3;
	float k;
	float TH,TL;
} Type3_TypeDef;
typedef struct
{
	float derta_VAC;
	uint8_t AC_State;
	uint8_t Covernt_EN;
	uint32_t PLL_Discern_Counter;
	uint32_t PLL_Discern_Counter_MAX;
	uint8_t Covern_FLAG;
} AC_Discern_TypeDef;

typedef struct
{
	float deta;
	float I_in[2];
	float V_in[2]; 
	float P_in[2];
	float D_MAX;
	float D_MIN;
	float D_OUT;
} MPPT_TypeDef;

typedef struct
{
	float PLL_Wt;
	float PLL_Phase;
	float Sample_Time; 
	float PLL_Cout;
	float PLL_DQ[2];
	_Bool PLL_FLAG;
	uint16_t PLL_Counter, PLL_CounterMax;
	SOGI_TypeDef SOGI_Pamer;
	PI_TypeDef PI_Pamer;
} GridPLL_TypeDef;

typedef struct
{
	uint32_t PFC_SFST_Value;
	uint16_t PFC_SFST_Counter;
	uint16_t PFC_Stage;
	uint16_t PFC_SFST_Step;
	uint8_t Covernt_EN;
	float deta_V;
} Totem_PolePFC_TypeDef;

typedef struct
{
	uint8_t Covernt_EN;
	uint8_t LSS_Charge_Counter;
	_Bool HSS_FLAG;
	uint8_t siginv_Stage;
	uint8_t siginv_SFST_Counter;
	float deta_V;
} siginv_TypeDef;

typedef struct
{
	uint8_t Order;
	float Ts;
	float Afilter[3];
	float Bfilter[3];
	float x[3];
	float y[3];
} LPF_TypeDef;

float MyFmod(float _X, float _Y);
inline float MyFmod(float _X, float _Y)
{
    return _X - (int16_t)(_X / _Y) * _Y;
}
void SOGI_Parameter_Init(SOGI_TypeDef *Pamer, float k, float Ts, float w);//SOGI变换初始化

void Vg_GetVpp_Init(GETVPP_TypeDef *VPP_Pamer, uint16_t _X);//自动峰值检测和自动直流偏执检测初始化

void SOGI_transfrom(SOGI_TypeDef *Pamer, float in);//SOGI变换

void alpha_bata_to_DQ(float alpha, float beta, float wt, float DQ[2]);//alpha_beta轴DQ变换

void f32_PI_Init(PI_TypeDef *PI_Pamer ,float _Ts, float _Kp, float _Ki, int16_t _TH, int16_t _TL);//浮点PI参数初始化

float f32_PI_Calculate(PI_TypeDef *PI_Pamer ,float REF, float Sample);//浮点PI运算

void f32_Integral_Init(Integral_TypeDef *I_pamer, float _Ts, float Intergral_MAX);//浮点积分初始化

float f32_Integral_Calculate(Integral_TypeDef *I_pamer, float _X);//浮点积分运算

void f32_Type2_Init(Type2_TypeDef *Type2_Pamer, float _A_filter[2], float _B_filter[3], float _K, float _TH, float _TL);//浮点TypeⅡ型补偿器初始化

void f32_Type3_Init(Type3_TypeDef *Type3_Pamer, float _A_filter[3], float _B_filter[4], float _K, float _TH, float _TL);//浮点TypeⅢ型补偿器初始化

float f32_Type2_Calculate(Type2_TypeDef *Type2_Pamer, float _REF, float _Sample);//浮点TypeⅡ型补偿器计算 expend 0.52us

float f32_Type3_Calculate(Type3_TypeDef *Type3_Pamer, float _REF, float _Sample);//浮点TypeⅢ型补偿器计算

uint32_t single_polarity_Duty_Gen(float input, uint32_t Counter_MAX, int8_t State);//单极性调制

uint32_t single_polarity_PFC_Duty_Gen(float input, uint32_t Counter_MAX, uint32_t D_MAX, uint32_t D_MIN, int8_t State);//单极性PFC调制

uint32_t single_polarity_Inv_Duty_Gen(float input, uint32_t Counter_MAX, uint32_t D_MAX, uint32_t D_MIN, int8_t State);//单极性逆变调制

uint32_t bipolar_Polarity_Duty_Gen(float input, int32_t Counter_MAX, int32_t D_MAX, int32_t D_Min);//双极性调制

void ABC_to_Dq(float DQ[2], float ABC[3], float _wt);//三相ABC到DQ的变化

void DQ_to_ABC(float DQ[2], float ABC[3], float _wt);

void Three_Phase_Convent_Trigonometric_Calculate(float Trigonometric[6], float _wt);//DQ控制的三相变换器中，三角函数的计算

void ABC_to_Dq_Simple(float _DQ[2], float _ABC[3], float _Trigonometric[6]);//ABC到DQ的变换，三角函数已经计算过

void DQ_TO_ABC_Simple(float _DQ[2], float _ABC[3], float _Trigonometric[6]);//DQ到ABC的变换，三角函数已经计算过

float Single_Phase_PLL_Cordic(GridPLL_TypeDef* PLL_Pamer, float Cin);//使用Cordic实现的单相锁相环，expend 2.37us

float Single_Phase_PLL_ARMMATH(GridPLL_TypeDef* PLL_Pamer, float Cin);//使用ARM_MATH库实现的单相锁相环，expend 3.02us

void Single_Phase_PLL_Init(GridPLL_TypeDef* PLL_Pamer, float Frequency, float Sample_Time, uint16_t PLL_CounterMax);//锁相环初始化

float f32_SinWave_Gen(uint16_t fc, float *wt, float Ts);

void AC_Stage_Discern(AC_Discern_TypeDef *AC_Discern_Pamer, float Vac_PLL, float PLL_UQ, _Bool Other_Criteria);//交流电压状态检测和过零点启动

float MPPT_P_Q_Calculate(MPPT_TypeDef *MPPT_Pamer, float V_in, float I_in);

float Trip_Filter_Calculate(Type2_TypeDef *Type2_Pamer,float _Sample);//陷波器计算 expend 0.52us

void Totem_polePFC_StageControl(Totem_PolePFC_TypeDef *Totem_PolePFC_Pamer, float VAC_PLL, _Bool Other_Criteria);//图腾柱PFC状态控制

void single_inverter_control(siginv_TypeDef *siginv_Pamer, float VAC_PLL, _Bool Other_Criteria);//单极性逆变控制

float f32_LPF_Calculate(LPF_TypeDef *LPF_Pamer, float _Sample);//数字低通滤波器计算

void f32_LPF_Init(LPF_TypeDef *LPF_Pamer, float wc, uint8_t Order, float Ts);//数字低通滤波器初始化
