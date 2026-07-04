#include "Interrupt.h"
#include "hrtim.h"
#include "My_Digital_Control_LL.h"

#define Vo_Sample_Scale 0.00805f    //ADC转换系数
#define Vin_Sample_Scale 0.023f     //ADC转换系数
extern uint16_t ADC1_samples[2];    //ADC存储数组
extern Type2_TypeDef LLC_CVPamer;   //电压环结构体名称
uint32_t LLC_Counter;
void HRTIM1_TIMFREP_IRQ(void)       //定时器F中断   用于环路控制
{
	static uint16_t LLC_SofStart_Counter = 2000;     //软起动初始TD值
	static _Bool LLC_SofStart_Flag;
	static _Bool LLC_OVP_FLAG;
	
	float LLC_Vo_Sample = (float)(ADC1_samples[0])*Vo_Sample_Scale;      //adc采样转换
	float Vin =(float)(ADC1_samples[1])*Vin_Sample_Scale;
	
		if(LLC_Vo_Sample>=16.f||Vin<=39.f)  //输出过压   输入欠压封波
	{
		LL_HRTIM_DisableOutput(HRTIM1, LL_HRTIM_OUTPUT_TA1|LL_HRTIM_OUTPUT_TA2);   //封波
		LLC_OVP_FLAG = 1;
	}
	else if(LLC_Vo_Sample<=10.f && LLC_OVP_FLAG)   //封波后使能软起标志位
	{
		LLC_OVP_FLAG = 0;
		LLC_SofStart_Flag = 0;
	}                                               //当你看到这里的时候   就已经执行完一个控制周期了
	float LLC_CVLP_OUT = f32_Type2_Calculate(&LLC_CVPamer, 12.f, LLC_Vo_Sample);   //电压环计算
	
	uint32_t LLC_CVLP_Counter = (uint32_t)(LLC_CVLP_OUT*45000.f);    //环路输出转TD比较值

	LLC_Counter = LLC_CVLP_Counter;
	if(!LLC_SofStart_Flag)    //软起动
	{
		LL_HRTIM_EnableOutput(HRTIM1, LL_HRTIM_OUTPUT_TA1|LL_HRTIM_OUTPUT_TA2);    //开启pwm
		if(LLC_SofStart_Counter <= LLC_CVLP_Counter)    //环路切入判定
		{
			LLC_SofStart_Counter+=2;                    //慢慢给他加  软起动
			LLC_Counter = LLC_SofStart_Counter;
		}
		else
		{
			LLC_SofStart_Flag = 1;
			LLC_SofStart_Counter = 2000;
		}
	}
	HRTIM1->sTimerxRegs[4].CMP4xR = LLC_Counter;    //给TD定时器赋值  用于控制TD时间
                                                   //当你看到这里的时候   就已经执行完一个控制周期了

}

