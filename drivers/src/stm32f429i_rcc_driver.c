 /********************************************************************************************************************************
 * Module: RCC
 *
 * File Name: stm32f429i_rcc_driver.h
 *
 * Description: header file for RCC module driver
 *
 * Author: AhmadShabrawy
 ********************************************************************************************************************************/

#include "../Inc/stm32f429i_rcc_driver.h"

/*********************************************************************************************************************************
 *                    								 Function Definitions    						                                 *
 ********************************************************************************************************************************/


uint32 RCC_getPclkval(RCC_Bus a_Bus )
{
	uint32 pclk1, pclk2, hclk, sysclk;
	uint16 ahb_prescalerValues[] = {2, 4, 8, 16, 64, 128, 256, 512};
	uint8 apb1_prescalerValues[] = {2, 4, 8, 16};
	uint8 apb2_prescalerValues[] = {2, 4, 8, 16};
	uint8 clksrc, temp, ahbPres, apb1Pres, apb2Pres;
	clksrc = (RCC->CFGR>>2)&0x3;/*SW bits*/
	switch(clksrc)
	{
		case 0:
			sysclk= 16000000; /*Internal OSC*/
			break;
		case 1:
			sysclk= 8000000; /*External OSC*/
			break;
		case 2:
			sysclk= 938; /*PLL not configured yet*/
			break;
	}
	/*Find AHB prescaler*/
	temp =  (RCC->CFGR>>4)&0xF;/*HPRE bits*/
	if(temp < 8)
	{
		ahbPres = 1; /*Sys clock is not divided*/
	}
	else if((temp >= 8) && (temp <= 15))
	{
		ahbPres = ahb_prescalerValues[temp - 8];
	}
	else
	{
		/*No action required*/
	}
	/*Find APB1 Prescaler*/
	temp =  (RCC->CFGR>>10)&0x7;/*PPRE1 bits*/
	if(temp < 4)
	{
		apb1Pres = 1; /*Hclk is not dived*/
	}
	if((temp >= 4) && (temp <= 7))
	{
		apb1Pres = apb1_prescalerValues[temp - 4];
	}
	else
	{
		/*No action required*/
	}
	/*Find APB2 Prescaler*/
	temp =  (RCC->CFGR>>13)&0x7;/*PPRE1 bits*/
	if(temp < 4)
	{
		apb2Pres = 1; /*Hclk is not dived*/
	}
	if((temp >= 4) && (temp <= 7))
	{
		apb2Pres = apb2_prescalerValues[temp - 4];
	}
	else
	{
		/*No action required*/
	}

	if(a_Bus == RCC_AHB_BUS){
		return sysclk/(ahbPres);/*AHB bus clk*/

	}
	else if(a_Bus == RCC_APB1_BUS)
	{
		return sysclk/(ahbPres*apb1Pres);/*APB1 bus clk*/

	}
	else if (a_Bus == RCC_APB2_BUS)
	{
		return sysclk/(ahbPres*apb2Pres);/*APB2 bus clk*/

	}
	else
	{
		/*No action required*/
	}



}
