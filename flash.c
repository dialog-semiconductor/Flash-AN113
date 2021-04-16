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
#include <stdio.h>
#include "exip_defs.h"

#define NUM_OSPI_PINS 11

OSPI_HandleTypeDef OSPIHandle;
OSPI_RegularCmdTypeDef flash_cmd;

typedef struct
{
	GPIO_TypeDef * port;
	uint32_t pin;
	uint32_t alt_func;
	uint32_t pull;
} pin_assignment_t;


// OctoSPI pin table
pin_assignment_t ospi_pin_assignment[NUM_OSPI_PINS] = 
{
	{.port = OSPI_CS_GPIO_PORT, .pin = OSPI_CS_PIN, .alt_func = OSPI_CS_PIN_AF, .pull = GPIO_PULLUP},
	{.port = OSPI_CLK_GPIO_PORT, .pin = OSPI_CLK_PIN, .alt_func = OSPI_CLK_PIN_AF, .pull = GPIO_NOPULL},
	{.port = OSPI_D0_GPIO_PORT, .pin = OSPI_D0_PIN, .alt_func = OSPI_D0_PIN_AF, .pull = GPIO_NOPULL},
	{.port = OSPI_D1_GPIO_PORT, .pin = OSPI_D1_PIN, .alt_func = OSPI_D1_PIN_AF, .pull = GPIO_NOPULL},
	{.port = OSPI_D2_GPIO_PORT, .pin = OSPI_D2_PIN, .alt_func = OSPI_D2_PIN_AF, .pull = GPIO_NOPULL},
	{.port = OSPI_D3_GPIO_PORT, .pin = OSPI_D3_PIN, .alt_func = OSPI_D3_PIN_AF, .pull = GPIO_NOPULL},
	{.port = OSPI_D4_GPIO_PORT, .pin = OSPI_D4_PIN, .alt_func = OSPI_D4_PIN_AF, .pull = GPIO_NOPULL},
	{.port = OSPI_D5_GPIO_PORT, .pin = OSPI_D5_PIN, .alt_func = OSPI_D5_PIN_AF, .pull = GPIO_NOPULL},
	{.port = OSPI_D6_GPIO_PORT, .pin = OSPI_D6_PIN, .alt_func = OSPI_D6_PIN_AF, .pull = GPIO_NOPULL},
	{.port = OSPI_D7_GPIO_PORT, .pin = OSPI_D7_PIN, .alt_func = OSPI_D7_PIN_AF, .pull = GPIO_NOPULL},
	{.port = OSPI_DQS_GPIO_PORT, .pin = OSPI_DQS_PIN, .alt_func = OSPI_DQS_PIN_AF, .pull = GPIO_NOPULL},
};

///////////////////////////////////////////////////////////////////////////////
// Function: flash_config_ospi_pins
// This function assigns the pins to the OctoSPI host controller by setting the
// pins alternate function in the pin mux. It also configures the pin attributes.
// Arguments:
// None
///////////////////////////////////////////////////////////////////////////////
static void flash_config_ospi_pins(void)
{
	uint32_t i;
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

	for(i = 0; i < NUM_OSPI_PINS; i++)
	{
		GPIO_InitStruct.Pin       = ospi_pin_assignment[i].pin;
		GPIO_InitStruct.Pull      = ospi_pin_assignment[i].pull;
		GPIO_InitStruct.Alternate = ospi_pin_assignment[i].alt_func;
		HAL_GPIO_Init(ospi_pin_assignment[i].port, &GPIO_InitStruct);
	}
}

///////////////////////////////////////////////////////////////////////////////
// Function: flash_jreset
// This function performs a flash JEDEC reset on the flash device.
// A JEDEC reset is a sequence transmitted by a host MCU to a flash device
// resulting in a reset event on the flash device. The sequence is transmitted
// by the host over two of the SPI interface signals. It is implemented here
// using GPIO operations.
// Arguments:
// None
///////////////////////////////////////////////////////////////////////////////
static void flash_jreset(void)
{
	uint32_t i;
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin       = OSPI_CS_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(OSPI_CS_GPIO_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.Pin       = OSPI_CLK_PIN;
	HAL_GPIO_Init(OSPI_CLK_GPIO_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.Pin       = OSPI_D0_PIN;
	HAL_GPIO_Init(OSPI_D0_GPIO_PORT, &GPIO_InitStruct);

	// Perform a reset sequence:
	// CS goes low 4 times with alternating values of SOUT
	// SCK is drive low or high and must stay in one state
	HAL_GPIO_WritePin(OSPI_CLK_GPIO_PORT, OSPI_CLK_PIN, GPIO_PIN_RESET); // set SCK low
	for(i = 0; i < 4; i++)
	{
		// drive CS low
		HAL_GPIO_WritePin(OSPI_CS_GPIO_PORT, OSPI_CS_PIN, GPIO_PIN_RESET);
		// drive SI low or high: alternate its state every iteration
		HAL_GPIO_WritePin(OSPI_D0_GPIO_PORT, OSPI_D0_PIN, (i&1)?GPIO_PIN_SET:GPIO_PIN_RESET);
		// drive CS high; SI state will be captured on the CS rising edge
		HAL_GPIO_WritePin(OSPI_CS_GPIO_PORT, OSPI_CS_PIN, GPIO_PIN_SET);
	}

	//temporary patch: primitive delay loop after reset to let flash settle down
	volatile int j;
	for(j = 0; j < 1000000; j++);
}


///////////////////////////////////////////////////////////////////////////////
// Function: flash_read_jedec_id
// Send read JEDEC ID command to flash device
// Arguments:
// nbytes: number of JEDEC ID bytes to read from flash device
// jid_p: pointer to JEDEC ID buffer to which the device's response will be copied
///////////////////////////////////////////////////////////////////////////////
void flash_read_jedec_id(uint32_t nbytes, uint8_t * jid_p)
{
	flash_cmd.Instruction = EXIP_CMD_READ_MANUF_DEV_ID;
	flash_cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;
	flash_cmd.NbData = nbytes;
	flash_cmd.DataMode = HAL_OSPI_DATA_1_LINE;
	flash_cmd.DummyCycles = 0;
	HAL_OSPI_Command(&OSPIHandle, &flash_cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
	HAL_OSPI_Receive(&OSPIHandle, jid_p, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
}

///////////////////////////////////////////////////////////////////////////////
// Function: flash_write_enable
// Send write-enable command to flash device
// Arguments:
// None
///////////////////////////////////////////////////////////////////////////////
void flash_write_enable(void)
{
	flash_cmd.Instruction = EXIP_CMD_WRITE_ENABLE;
	flash_cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;
	flash_cmd.DataMode = HAL_OSPI_DATA_NONE;
	flash_cmd.DummyCycles = 0;
	HAL_OSPI_Command(&OSPIHandle, &flash_cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
}

///////////////////////////////////////////////////////////////////////////////
// Function: flash_read_reg1
// Send read register 1 command to flash device
// Arguments:
// None
// Returns:
// the device's response (register 1 value)
///////////////////////////////////////////////////////////////////////////////
uint8_t flash_read_reg1(void)
{
	uint8_t rx_buf[4];

	flash_cmd.Instruction = EXIP_CMD_READ_STATUS_REG_BYTE1;
	flash_cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;
	flash_cmd.NbData = 1;
	flash_cmd.DataMode = HAL_OSPI_DATA_1_LINE;
	flash_cmd.DummyCycles = 0;
	HAL_OSPI_Command(&OSPIHandle, &flash_cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
	HAL_OSPI_Receive(&OSPIHandle, rx_buf, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
	return (rx_buf[0]);
}

///////////////////////////////////////////////////////////////////////////////
// Function: flash_write_reg1
// Send write register 1 command to flash device
// Arguments:
// val: the value to be written to register 1
///////////////////////////////////////////////////////////////////////////////
void flash_write_reg1(uint8_t val)
{
	uint8_t tx_buf = val;

	flash_cmd.Instruction = EXIP_CMD_WRITE_STATUS_REG_BYTE1;
	flash_cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;
	flash_cmd.DataMode = HAL_OSPI_DATA_1_LINE;
	flash_cmd.NbData = 1;
	flash_cmd.DummyCycles = 0;
	HAL_OSPI_Command(&OSPIHandle, &flash_cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
	HAL_OSPI_Transmit(&OSPIHandle, &tx_buf, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
}

///////////////////////////////////////////////////////////////////////////////
// Function: flash_write_reg2
// Send write register 2 command to flash device
// Arguments:
// val: the value to be written to register 2
///////////////////////////////////////////////////////////////////////////////
void flash_write_reg2(uint8_t val)
{
	uint8_t tx_buf = val;

	flash_cmd.Instruction = EXIP_CMD_WRITE_STATUS_REG_BYTE2;
	flash_cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;
	flash_cmd.DataMode = HAL_OSPI_DATA_1_LINE;
	flash_cmd.NbData = 1;
	flash_cmd.DummyCycles = 0;
	HAL_OSPI_Command(&OSPIHandle, &flash_cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
	HAL_OSPI_Transmit(&OSPIHandle, &tx_buf, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
}

///////////////////////////////////////////////////////////////////////////////
// Function: flash_write_status_control_register
// Send write register status/control register command to flash device. Using
// this command it is possible to write to any of the registers.
// Arguments:
// reg_addr: status/control register number/address
// val: the value to be written to the register
///////////////////////////////////////////////////////////////////////////////
void flash_write_status_control_register(uint8_t reg_addr, uint8_t val)
{
	uint8_t tx_buf = val;

	flash_cmd.Instruction = EXIP_CMD_WRITE_STAT_CTRL_REGS;
	flash_cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
	flash_cmd.AddressSize = HAL_OSPI_ADDRESS_8_BITS;
	flash_cmd.Address = reg_addr;
	flash_cmd.DataMode = HAL_OSPI_DATA_1_LINE;
	flash_cmd.NbData = 1;
	flash_cmd.DummyCycles = 0;
	HAL_OSPI_Command(&OSPIHandle, &flash_cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
	HAL_OSPI_Transmit(&OSPIHandle, &tx_buf, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
}

///////////////////////////////////////////////////////////////////////////////
// Function: flash_unprotect_sector
// Send unprotect sector command to flash device
// Arguments:
// addr: any address within the sector to be unprotected
///////////////////////////////////////////////////////////////////////////////
void flash_unprotect_sector(uint32_t addr)
{
	flash_cmd.Instruction = EXIP_CMD_UNPROTECT_SECTOR;
	flash_cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
	flash_cmd.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
	flash_cmd.Address = addr;
	flash_cmd.DataMode = HAL_OSPI_DATA_NONE;;
	flash_cmd.DummyCycles = 0;
	HAL_OSPI_Command(&OSPIHandle, &flash_cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
}

///////////////////////////////////////////////////////////////////////////////
// Function: flash_block_erase_4k
// Send block erase 4K command to flash device
// Arguments:
// addr: the address of the 4K block to be erased
///////////////////////////////////////////////////////////////////////////////
void flash_block_erase_4k(uint32_t addr)
{
	flash_cmd.Instruction = EXIP_CMD_BLOCK_ERASE_4K;
	flash_cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
	flash_cmd.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
	flash_cmd.Address = addr;
	flash_cmd.DataMode = HAL_OSPI_DATA_NONE;;
	flash_cmd.DummyCycles = 0;
	HAL_OSPI_Command(&OSPIHandle, &flash_cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
}

///////////////////////////////////////////////////////////////////////////////
// Function: flash_page_prog
// Send page program command to flash device
// Arguments:
// addr: program operation start address in flash memory
// data_buf: pointer to a buffer holding the data to be written
// data_size: number of bytes to be programmed
///////////////////////////////////////////////////////////////////////////////
void flash_page_prog(uint32_t addr, uint8_t * data_buf, uint32_t data_size)
{

	flash_cmd.Instruction = EXIP_CMD_PAGE_PROG;
	flash_cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
	flash_cmd.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
	flash_cmd.Address = addr;
	flash_cmd.DataMode = HAL_OSPI_DATA_1_LINE;
	flash_cmd.NbData = data_size;
	flash_cmd.DummyCycles = 0;
	HAL_OSPI_Command(&OSPIHandle, &flash_cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
	HAL_OSPI_Transmit(&OSPIHandle, data_buf, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
}

///////////////////////////////////////////////////////////////////////////////
// Function: flash_readarray_spi
// Send a read array command in single SPI mode to flash device
// Arguments:
// addr: read operation start address in flash memory
// data_buf: pointer to a buffer to which the read data will be copied
// data_size: number of bytes to be read
///////////////////////////////////////////////////////////////////////////////
void flash_readarray_spi(uint32_t addr, uint8_t * data_buf, uint32_t data_size)
{
	flash_cmd.Instruction = EXIP_CMD_READARRAY;
	flash_cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
	flash_cmd.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
	flash_cmd.Address = addr;
	flash_cmd.DataMode = HAL_OSPI_DATA_1_LINE;
	flash_cmd.NbData = data_size;
	flash_cmd.DummyCycles = 8;
	HAL_OSPI_Command(&OSPIHandle, &flash_cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
	HAL_OSPI_Receive(&OSPIHandle, data_buf, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
}

///////////////////////////////////////////////////////////////////////////////
// Function: flash_readarray_octal
// Send a read array command in octal SPI mode to flash device
// Arguments:
// addr: read operation start address in flash memory
// data_buf: pointer to a buffer to which the read data will be copied
// data_size: number of bytes to be read
///////////////////////////////////////////////////////////////////////////////
void flash_readarray_octal(uint32_t addr, uint8_t * data_buf, uint32_t data_size)
{
	flash_cmd.Instruction = EXIP_CMD_READARRAY;
	flash_cmd.AddressMode = HAL_OSPI_ADDRESS_8_LINES;
	flash_cmd.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
	flash_cmd.Address = addr;
	flash_cmd.DataMode = HAL_OSPI_DATA_8_LINES;
	flash_cmd.NbData = data_size;
	flash_cmd.DummyCycles = EXIP_CONFIGURABLE_DUMMY_CYCLES;
	HAL_OSPI_Command(&OSPIHandle, &flash_cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
	HAL_OSPI_Receive(&OSPIHandle, data_buf, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
}

///////////////////////////////////////////////////////////////////////////////
// Function: flash_set_dummy_cycles_and_wrap_size
// Configure number of dummy cycles (for octal-SPI reads) and wrap size (if
// applicable) in flash device
// Arguments:
// cycles: number of dummy cycles to be configured (range 8-22, even  values only)
// wrap_size: size of wrap window for wrapped reads (8, 16, 32 or 64)
///////////////////////////////////////////////////////////////////////////////
void flash_set_dummy_cycles_and_wrap_size(uint32_t cycles, uint32_t wrap_size)
{
	uint8_t bit_val_dummy = 0, bit_val_wrap = 0;

	// Check for validity of dummy cycle argument
	if(cycles < 8 || cycles > 22 || (cycles & 0x1))
		return;

	bit_val_dummy = (cycles - 8) >> 1;

	switch(wrap_size)
	{
		// Note: we only set wrap-and-continue
		case 8:
			bit_val_wrap = 4;
			break;
		case 16:
			bit_val_wrap = 5;
			break;
		case 32:
			bit_val_wrap = 6;
			break;
		case 64:
			bit_val_wrap = 7;
			break;
		default:
			return;
			break;
	}

	// send write enable
	flash_write_enable();

	// write calculated values to bitfield in register 3
	flash_write_status_control_register(3, (0x10 | bit_val_dummy | (bit_val_wrap << 5)));
}

///////////////////////////////////////////////////////////////////////////////
// Function: flash_set_drive_strength
// Configure output drive strength in flash device
// Arguments:
// impedance: impedance in Ohms (allowed values: 33, 40, 50, 66, 100)
///////////////////////////////////////////////////////////////////////////////
void flash_set_drive_strength(uint32_t impedance)
{
	uint8_t bit_val;

	switch(impedance)
	{
		case 50:
			bit_val = 0;
			break;
		case 33:
			bit_val = 1;
			break;
		case 66:
			bit_val = 2;
			break;
		case 100:
			bit_val = 3;
			break;
		case 40:
			bit_val = 4;
			break;
		default:
			bit_val = 0;
			break;
	}

	// send write enable
	flash_write_enable();

	// write calculated value to regiser 0x81 (note: this is a non-volatile regitser
	flash_write_status_control_register(0x81, (bit_val));
}

///////////////////////////////////////////////////////////////////////////////
// Function: flash_enter_octal_mode
// Switch the device to octal-SPI mode. This includes:
// 1. Writing to flasg register 2 to change the mode
// 2. Preparing  fields in the flash command structure for subsequent triggering
// of command in octal-SDR or octal-DDR modes
// Arguments:
// ddr: boolean indicating if double data rate (DDR) should be used; if true
// DDR is to be used, if false SDR will be used.
///////////////////////////////////////////////////////////////////////////////
void flash_enter_octal_mode(bool ddr)
{
	uint8_t reg_val;

	// send write enable 
	flash_write_enable();

	// Write to status/control register 2 to switch to octal-SDR or octal-DDR
	reg_val = ddr ? 0x88 : 0x08;
	flash_write_reg2(reg_val);

	// Change opcode address and data formats to 8 lines. 
	flash_cmd.InstructionMode = HAL_OSPI_INSTRUCTION_8_LINES;
	flash_cmd.AddressMode = HAL_OSPI_ADDRESS_8_LINES;
	flash_cmd.DataMode = HAL_OSPI_DATA_8_LINES;

	// Handle DDR vs. SDR in command structure. Note: the opcode is always sent
	// in SDR mode (another way to see it: in DDR we must send the opcode
	// on both edges of the clock. Hence we will opocde to SDR format.
	if(ddr)
	{
		// For DDR we enable use of data strobe signal
		flash_cmd.DQSMode = HAL_OSPI_DQS_ENABLE;
		// Enable DDR format for address
		flash_cmd.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_ENABLE;
		// Enable DDR format for data
		flash_cmd.DataDtrMode = HAL_OSPI_DATA_DTR_ENABLE;
		// For DDR ST recommends enabling the delay hold quarter cycle
		OSPIHandle.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
	}
	else
	{
		// For SDR we do not use data strobe signal
		flash_cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
		// Disable DDR format for address
		flash_cmd.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
		// Disable DDR format for address
		flash_cmd.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
		// For SDR delay hold quarter cycle is disabled
		OSPIHandle.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
	}

	// Since we changed options which are handled by HAL_OSPI_Init we must call
	// it again
	HAL_OSPI_Init(&OSPIHandle);
}


///////////////////////////////////////////////////////////////////////////////
// Function: flash_init
// Initialize the OctoSPI host controller as well as the flash device
// Arguments:
// None
///////////////////////////////////////////////////////////////////////////////
void flash_init(void)
{
	// Enable clock for the OctoSPI memory interface clock */
	OSPI_CLK_ENABLE();

	// Reset the OctoSPI memory interface
	OSPI_FORCE_RESET();
	OSPI_RELEASE_RESET();

	// Enable clock for all OctoSPI pins
	OSPI_CS_GPIO_CLK_ENABLE();
	OSPI_CLK_GPIO_CLK_ENABLE();
	OSPI_D0_GPIO_CLK_ENABLE();
	OSPI_D1_GPIO_CLK_ENABLE();
	OSPI_D2_GPIO_CLK_ENABLE();
	OSPI_D3_GPIO_CLK_ENABLE();
	OSPI_D4_GPIO_CLK_ENABLE();
	OSPI_D5_GPIO_CLK_ENABLE();
	OSPI_D6_GPIO_CLK_ENABLE();
	OSPI_D7_GPIO_CLK_ENABLE();
	OSPI_DQS_GPIO_CLK_ENABLE();

	// Reset flash
	flash_jreset();

	// Assign flash interface pins to OctoSPI
	flash_config_ospi_pins();

	// Fill OctosSPI iniialization structure
	OSPIHandle.Instance = OCTOSPI1;
	OSPIHandle.Init.ClockPrescaler        = EXIP_CLK_SRC_FREQ/EXIP_CLK_FREQ;
	OSPIHandle.Init.FifoThreshold         = 4;
	OSPIHandle.Init.MemoryType            = HAL_OSPI_MEMTYPE_APMEMORY;
	OSPIHandle.Init.SampleShifting        = HAL_OSPI_SAMPLE_SHIFTING_NONE;
	OSPIHandle.Init.DeviceSize             = EXT_FLASH_SIZE_LOG2;
	OSPIHandle.Init.ChipSelectHighTime    = 3;
	OSPIHandle.Init.FreeRunningClock      = HAL_OSPI_FREERUNCLK_DISABLE;
	OSPIHandle.Init.ClockMode             = HAL_OSPI_CLOCK_MODE_0;
#ifdef USE_WRAP
	OSPIHandle.Init.WrapSize              = HAL_OSPI_WRAP_32_BYTES;
#else
	OSPIHandle.Init.WrapSize              = HAL_OSPI_WRAP_NOT_SUPPORTED;
#endif
	OSPIHandle.Init.DualQuad             = HAL_OSPI_DUALQUAD_DISABLE;
	OSPIHandle.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
	OSPIHandle.Init.DelayBlockBypass      = HAL_OSPI_DELAY_BLOCK_BYPASSED;
	OSPIHandle.Init.ChipSelectBoundary    = 0;

	HAL_OSPI_Init(&OSPIHandle);

	// Fill command structure with defualt values - these can change later
	flash_cmd.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
	flash_cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
	flash_cmd.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
	flash_cmd.AlternateBytes     = HAL_OSPI_ALTERNATE_BYTES_NONE;
	flash_cmd.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
	flash_cmd.Address            = 0;
	flash_cmd.DQSMode            = HAL_OSPI_DQS_DISABLE;
	flash_cmd.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;
}

///////////////////////////////////////////////////////////////////////////////
// Function: flash_set_memory_mapped_mode
// Set OctoSPI mode to memory mapped; setup read and write commands for this mode
// To ensure high-peformance execute-in-place (XiP) we configure octal-DDR reads
// Arguments:
// None
///////////////////////////////////////////////////////////////////////////////
void flash_set_memory_mapped_mode(void)
{
	OSPI_MemoryMappedTypeDef mem_map_cfg;

	// Fill some default values for all commands involved
	flash_cmd.InstructionMode = HAL_OSPI_INSTRUCTION_8_LINES;
	flash_cmd.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
	flash_cmd.AddressMode = HAL_OSPI_ADDRESS_8_LINES;
	flash_cmd.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_ENABLE;
	flash_cmd.Address = 0; // actual read address will be set by read request
	flash_cmd.DataMode = HAL_OSPI_DATA_8_LINES;
	flash_cmd.DataDtrMode = HAL_OSPI_DATA_DTR_ENABLE;
	flash_cmd.NbData = 1; // actual read size will be set by read request

	// Looks like we have to set up a write command for memory mapped even
	// though we will never use it
	flash_cmd.OperationType = HAL_OSPI_OPTYPE_WRITE_CFG;
	flash_cmd.Instruction = EXIP_CMD_PAGE_PROG;
	flash_cmd.DummyCycles   = 0;
	HAL_OSPI_Command(&OSPIHandle, &flash_cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
	
	// Next we'll set up a read commands. Opetionally two read commands: one for
	// wrapped read and one for normal read.
	flash_cmd.DummyCycles = EXIP_CONFIGURABLE_DUMMY_CYCLES;
#ifdef USE_WRAP
	flash_cmd.OperationType      = HAL_OSPI_OPTYPE_WRAP_CFG;
	flash_cmd.Instruction        = EXIP_CMD_BURST_READ_WRAP;
	HAL_OSPI_Command(&OSPIHandle, &flash_cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
#endif
	flash_cmd.OperationType = HAL_OSPI_OPTYPE_READ_CFG;
	flash_cmd.Instruction = EXIP_CMD_READARRAY;
	HAL_OSPI_Command(&OSPIHandle, &flash_cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);

	// Set up some memory mapped mode parameters and switch to 
	// memory mapped mode
	mem_map_cfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_ENABLE;
	mem_map_cfg.TimeOutPeriod     = 32;
  
	HAL_OSPI_MemoryMapped(&OSPIHandle, &mem_map_cfg);
}
