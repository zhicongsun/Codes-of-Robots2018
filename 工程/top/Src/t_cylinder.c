#include "t_cylinder.h"
#include "gpio.h"

void Cylinder_ctr(bool cyflag)
{	
	if(cyflag)
	{	
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_SET);	//气缸1伸出	
		HAL_Delay(CYTIME);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);	//气缸2伸出	
		HAL_Delay(CYTIME);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_RESET);	//气缸1回缩
		HAL_Delay(CYTIME);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);	//气缸3伸出

		HAL_Delay(1000);

		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_RESET);	//气缸3回缩
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_SET);	//气缸1伸出	
		HAL_Delay(CYTIME);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);	//气缸2回缩	
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_RESET);	//气缸1回缩
	}
}

