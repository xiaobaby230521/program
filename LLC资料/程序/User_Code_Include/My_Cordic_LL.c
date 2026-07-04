#include "My_cordic_LL.h"

//输入的角度为归一化角度[0~1]<-->[0~2*pi]
void value_to_cordic31(float *value, int32_t *cordic31, uint8_t Length)
{
	for(uint8_t i = 0; i<Length; i++)
	{
		if(value[i] > 0.5f)								//将角度从[0, 1.0]转换到[-0.5, 0.5]
			value[i] -= 1.0f;
		
		cordic31[i] = (int32_t)((value[i])*0x100000000);
	}
}

void cordic31_to_value(int32_t *cordic31, float *res, uint8_t Length)
{
	for(uint8_t i = 0; i< Length; i++)
	{
		res[i] = (float)cordic31[i]/0x80000000;
	}
}

void My_Cordic_Calculate(int32_t *Input_Data, int32_t *Output_Data, uint8_t Length)
{
	uint8_t j = 0;
	for(uint8_t i = 0; i<Length; i++)
	{
		CORDIC->WDATA = Input_Data[i];
		Output_Data[j] = CORDIC->RDATA;
		j++;
		Output_Data[j] = CORDIC->RDATA;
		j++;
	}
}
