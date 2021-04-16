/*
* The Clear BSD License
* Copyright (c) 2019 Adesto Technologies Corporation.
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or without modification,
* are permitted (subject to the limitations in the disclaimer below) provided
*  that the following conditions are met:
*
* o Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
*
* o Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
*
* o Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdlib.h>
#include "stm32h7xx_hal.h"
#include "exip_defs.h"

 #define SYS_CLOCK_FREQ 280000000
 #define PLL_INPUT_FREQ 2000000

///////////////////////////////////////////////////////////////////////////////
// Function: SystemClock_Config
// This function initializes the clock tree including the system clock and other
// clocks. In particular it sets up a dedicated clock source for the flash
// interface (OctoSPI block).
///////////////////////////////////////////////////////////////////////////////
void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphCLKInit;

	// Supply configuration update enable
	HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

	/* The voltage scaling allows optimizing the power consumption when the device is
	clocked below the maximum system frequency, to update the voltage scaling value
	regarding system frequency refer to product datasheet.
	*/
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	// Enable HSE Oscillator and activate PLL with HSE as source
	// Set system clock frequency to 280MHz
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
	RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

	RCC_OscInitStruct.PLL.PLLM = HSE_VALUE / PLL_INPUT_FREQ;;
	RCC_OscInitStruct.PLL.PLLN = (SYS_CLOCK_FREQ * 2) / PLL_INPUT_FREQ;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLQ = 2;

	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	// Select PLL as system clock source and configure  bus clocks dividers
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 | \
	RCC_CLOCKTYPE_PCLK2  | RCC_CLOCKTYPE_D3PCLK1);

	RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6);

	// Configure PLL2 to provide an independent clock source to OctoSPI.
	// Note: pll2_r_ck is one of the available clock sources for OctoSPI
	// so we'll use it.
	PeriphCLKInit.PeriphClockSelection = RCC_PERIPHCLK_OSPI;
	PeriphCLKInit.OspiClockSelection = RCC_OSPICLKSOURCE_PLL2;
	// set divider to get PLL input frequency
	PeriphCLKInit.PLL2.PLL2M = HSE_VALUE / PLL_INPUT_FREQ;
	// Calculate multiplier for generating the OctoSPI/flash clock source.
	// Large multipliers seem to work better. So here we will generate the desired
	// frequency times 2. Below we'll divide it back by 2 for pll2_r_ck and that
	// will give us the desired source clock frequency.
	PeriphCLKInit.PLL2.PLL2N = (EXIP_CLK_SRC_FREQ * 2) / PLL_INPUT_FREQ;
	PeriphCLKInit.PLL2.PLL2P = 2;
	PeriphCLKInit.PLL2.PLL2R = 2;
	PeriphCLKInit.PLL2.PLL2Q = 2;
	PeriphCLKInit.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
	PeriphCLKInit.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
	HAL_RCCEx_PeriphCLKConfig(&PeriphCLKInit);

	__HAL_RCC_CSI_ENABLE() ;

	__HAL_RCC_SYSCFG_CLK_ENABLE() ;
}
