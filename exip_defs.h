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

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_ospi.h"

// Serial clock frequency to be used for flash interface
// Note: 110MHz was tested successfully on an optimal hardware design.
// On a Nucleo board with flash shield board flash clock may have to limited
// to ~90MHz due to long traces.
#define EXIP_CLK_FREQ 110000000
#define EXIP_CLK_SRC_FREQ (2 * EXIP_CLK_FREQ)
// Dummy cycles count for read commands where configurable
#define EXIP_CONFIGURABLE_DUMMY_CYCLES 14

#define USE_WRAP
// Cache line size (in bytes)
#define CACHELINE_SIZE 32

// Status Register busy bit
#define BUSY_BIT_MASK 0x1

// EcoXiP commands
#define	EXIP_CMD_WRITE_STATUS_REG_BYTE1	0x01	// Write Status Register Byte 1 	
#define	EXIP_CMD_PAGE_PROG				0x02	// Byte/Page Program (1 - 256 Bytes) 	
#define	EXIP_CMD_READARRAY_SPI			0x03	// Read Array (1-1-1) 	
#define	EXIP_CMD_READ_STATUS_REG_BYTE1	0x05	// Read Status Register Byte 1 	
#define	EXIP_CMD_WRITE_ENABLE			0x06	// Write Enable 	
#define	EXIP_CMD_READARRAY				0x0B	// Read Array 	
#define	EXIP_CMD_BURST_READ_WRAP		0x0C	// Burst Read with Wrap 	
#define	EXIP_CMD_BLOCK_ERASE_4K			0x20	// Block Erase (4 Kbytes) 	
#define	EXIP_CMD_ACTVE_STATUS_INT		0x25	// Active Status Interrupt 	
#define	EXIP_CMD_WRITE_STATUS_REG_BYTE2	0x31	// Write Status Register Byte 2 	
#define	EXIP_CMD_UNPROTECT_SECTOR		0x39	// Unprotect Sector 	
#define	EXIP_CMD_READ_STAT_CTRL_REGS	0x65	// Read Status/Control Registers 
#define	EXIP_CMD_WRITE_STAT_CTRL_REGS	0x71	// Write Status/Control Registers 	
#define	EXIP_CMD_READ_MANUF_DEV_ID		0x9F	// Read Manufacturer and Device ID




//This is the  pin selection for the NUCLEO-H7A3ZI board
/* Definition for OSPI Pins */
/* OSPI_CS */
#define OSPI_CS_PIN                       GPIO_PIN_6
#define OSPI_CS_GPIO_PORT                 GPIOG
#define OSPI_CS_PIN_AF                    GPIO_AF10_OCTOSPIM_P1
/* OSPI_CLK */
#define OSPI_CLK_PIN                      GPIO_PIN_2
#define OSPI_CLK_GPIO_PORT                GPIOB
#define OSPI_CLK_PIN_AF                   GPIO_AF9_OCTOSPIM_P1
/* OSPI_D0 */
#define OSPI_D0_PIN                       GPIO_PIN_11
#define OSPI_D0_GPIO_PORT                 GPIOD
#define OSPI_D0_PIN_AF                    GPIO_AF9_OCTOSPIM_P1
/* OSPI_D1 */
#define OSPI_D1_PIN                       GPIO_PIN_12
#define OSPI_D1_GPIO_PORT                 GPIOD
#define OSPI_D1_PIN_AF                    GPIO_AF9_OCTOSPIM_P1
/* OSPI_D2 */
#define OSPI_D2_PIN                       GPIO_PIN_2
#define OSPI_D2_GPIO_PORT                 GPIOE
#define OSPI_D2_PIN_AF                    GPIO_AF9_OCTOSPIM_P1
/* OSPI_D3 */
#define OSPI_D3_PIN                       GPIO_PIN_13
#define OSPI_D3_GPIO_PORT                 GPIOD
#define OSPI_D3_PIN_AF                    GPIO_AF9_OCTOSPIM_P1
/* OSPI_D4 */
#define OSPI_D4_PIN                       GPIO_PIN_4
#define OSPI_D4_GPIO_PORT                 GPIOD
#define OSPI_D4_PIN_AF                    GPIO_AF10_OCTOSPIM_P1
/* OSPI_D5 */
#define OSPI_D5_PIN                       GPIO_PIN_5
#define OSPI_D5_GPIO_PORT                 GPIOD
#define OSPI_D5_PIN_AF                    GPIO_AF10_OCTOSPIM_P1
/* OSPI_D6 */
#define OSPI_D6_PIN                       GPIO_PIN_6
#define OSPI_D6_GPIO_PORT                 GPIOD
#define OSPI_D6_PIN_AF                    GPIO_AF10_OCTOSPIM_P1
/* OSPI_D7 */
#define OSPI_D7_PIN                       GPIO_PIN_7
#define OSPI_D7_GPIO_PORT                 GPIOD
#define OSPI_D7_PIN_AF                    GPIO_AF10_OCTOSPIM_P1
/* OSPI_DQS */
#define OSPI_DQS_PIN                       GPIO_PIN_1
#define OSPI_DQS_GPIO_PORT                 GPIOA
#define OSPI_DQS_PIN_AF                    GPIO_AF11_OCTOSPIM_P1


/* Definition for OSPI clock resources */
#define OSPI_CLK_ENABLE()           __HAL_RCC_OSPI1_CLK_ENABLE()
#define OSPI_CLK_DISABLE()          __HAL_RCC_OSPI1_CLK_DISABLE()
#define OSPIM_CLK_ENABLE()          __HAL_RCC_OCTOSPIM_CLK_ENABLE()
#define OSPIM_CLK_DISABLE()         __HAL_RCC_OCTOSPIM_CLK_DISABLE()
#define OSPI_CS_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOG_CLK_ENABLE()
#define OSPI_CLK_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define OSPI_D0_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOD_CLK_ENABLE()
#define OSPI_D1_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOD_CLK_ENABLE()
#define OSPI_D2_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOE_CLK_ENABLE()
#define OSPI_D3_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOD_CLK_ENABLE()
#define OSPI_D4_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOD_CLK_ENABLE()
#define OSPI_D5_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOD_CLK_ENABLE()
#define OSPI_D6_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOD_CLK_ENABLE()
#define OSPI_D7_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOD_CLK_ENABLE()
#define OSPI_DQS_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()

#define OSPI_FORCE_RESET()          __HAL_RCC_OSPI1_FORCE_RESET()
#define OSPI_RELEASE_RESET()        __HAL_RCC_OSPI1_RELEASE_RESET()


// Misc defs
#define EXT_FLASH_PAGE_SIZE  256
#define EXT_FLASH_SECTORE_SIZE 4096
#define EXT_FLASH_SIZE 0x1000000
#define EXT_FLASH_SIZE_LOG2 24 //log2 of flash size - necessary for one of the OSPI fields

// Types


// Globals

extern UART_HandleTypeDef UartHandle;

extern void flash_init(void);
extern void flash_set_memory_mapped_mode();
extern void flash_set_dummy_cycles(uint32_t cycles);
extern void flash_set_drive_strength(uint32_t impedance_level);
extern void flash_write_enable(void);
extern uint8_t flash_read_reg1(void);
extern void flash_write_reg1(uint8_t val);
extern void flash_write_reg2(uint8_t val);
extern void flash_write_status_control_register(uint8_t reg_addr, uint8_t val);
extern void flash_unprotect_sector(uint32_t addr);
extern void flash_block_erase_4k(uint32_t addr);
extern void flash_page_prog(uint32_t addr, uint8_t * data_buf, uint32_t data_size);
extern void flash_readarray_spi(uint32_t addr, uint8_t * data_buf, uint32_t data_size);
extern void flash_readarray_octal(uint32_t addr, uint8_t * data_buf, uint32_t data_size);
extern void flash_enter_octal_mode(bool ddr);
extern void flash_read_jedec_id(uint32_t nbytes, uint8_t * jid_p);
extern void flash_set_dummy_cycles_and_wrap_size(uint32_t cycles, uint32_t wrap_size);
extern void flash_set_drive_strength(uint32_t impedance);
extern void flash_set_memory_mapped_mode(void);

extern void SystemClock_Config(void);
