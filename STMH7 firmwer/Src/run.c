#include "main.h"
#include "stm32h7xx_hal.h"
	int fpwm=0;
	int dir=0;
void run_stepper(int stepper_name,int delta)
{
	dir=0;
	fpwm=0;
	if(delta<=0)
	{
		delta=-delta;
		dir=1; // dem xuong
	}
	if(delta>0)
	{
		fpwm=(400000000)/(delta*100*12);
	}else fpwm = 65534;
	
  if (stepper_name==1)
  {

		if(dir==0)
		{
			TIM8->CR1&= ~(1 << 4);
			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		if(dir==1)
		{
			TIM8->CR1|=(1<<4);
			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET);
		}
    TIM1->PSC=fpwm;
  }
	if (stepper_name==2)
	{
		if(dir==0)
		{
			TIM4->CR1&=~(1<<4);
			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12,GPIO_PIN_RESET);
		}
		if(dir==1)
		{
			TIM4->CR1|=(1<<4);
			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12,GPIO_PIN_SET);
		}
		TIM2->PSC=fpwm;
	}
	if (stepper_name==3)
	{
		if(dir==0)
		{
			TIM5->CR1&=~(1<<4);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
		}
		if(dir==1)
		{
			TIM5->CR1|=(1<<4);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
		}
		TIM3->PSC=fpwm;
	}
		
}
