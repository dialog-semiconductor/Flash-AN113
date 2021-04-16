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

#include "stm32h7xx_hal.h"
#include "exip_defs.h"


// Enable instruction and data caches
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

// This configures higher I/O speeds when VDD < 2.5V
static void Enable_IO_Speed_Optimization(void)
{
	// Unlock option bytes
	HAL_FLASH_OB_Unlock();
	// Set VDDIO_HSLV VDDMMC_HSLV option bits to enable I/O speed optimization
	MODIFY_REG(FLASH->OPTSR_PRG, FLASH_OPTSR_VDDMMC_HSLV | FLASH_OPTSR_IO_HSLV, FLASH_OPTSR_VDDMMC_HSLV | FLASH_OPTSR_IO_HSLV);
	// Launch option byte modification to apply option bit change
	HAL_FLASH_OB_Launch();
	// Now that the option bits allow it, set HSLVx bits in SYSCFG_CCCSR register
	// to apply the I/O speed optimization
	HAL_SYSCFG_EnableIOSpeedOptimize();
	// Lock option bytes
	HAL_FLASH_OB_Lock();
}

///////////////////////////////////////////////////////////////////////////////
// Function: main
// The main function initializes the system and in particular the flash interface
// for optimal code execute-in-place (XiP) from external flash. After this is done
// it is possible to jump to code located on external flash.
///////////////////////////////////////////////////////////////////////////////
void main(void)
{
	/* Enable the CPU Cache */
	CPU_CACHE_Enable();

	// STM32H7xx HAL library initialization:
	HAL_Init();

	// Configure the system clock (and other clocks)
	SystemClock_Config();

	// Enable high I/O speed at low voltage
	Enable_IO_Speed_Optimization();

	// Initialize flash driver
	flash_init();

	//
	flash_set_dummy_cycles_and_wrap_size(EXIP_CONFIGURABLE_DUMMY_CYCLES, CACHELINE_SIZE);

	// Switch to octal-DDR 
	flash_enter_octal_mode(true);

	// Switch OctoSPI to memory mapped mode
	flash_set_memory_mapped_mode();

	// At this point XiP enabled and it's OK to jump to code in external flash
	// 
}
