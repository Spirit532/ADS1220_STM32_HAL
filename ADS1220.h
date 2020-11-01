/*
 * Single-header pure C ADS1220 driver for STM32 with HAL.
 * You probably want to add error checking, since there's none
 *
 * Chip select twiddling is not implemented - pull to ground(1 device only) or implement your own
 *
 * Bits taken from various examples, mostly GH@ Protocentral/Protocentral_ADS1220
 * License: MIT(http://opensource.org/licenses/MIT)
 */

#ifndef INC_ADS1220_H_
#define INC_ADS1220_H_

#include "main.h" // Your project's main should include HAL

//Commands
#define ADS1220_RESET 0x06 // Good idea to reset on power-up
#define ADS1220_START 0x08 // Both single-shot and continuous conversion must be started via 0x08
#define ADS1220_WREG  0x40
#define ADS1220_RREG  0x20

//Registers
#define ADS1220_CONFIG_REG0_ADDRESS 0x00
#define ADS1220_CONFIG_REG1_ADDRESS 0x01
#define ADS1220_CONFIG_REG2_ADDRESS 0x02
#define ADS1220_CONFIG_REG3_ADDRESS 0x03

//Masks
#define ADS1220_REG_CONFIG1_DR_MASK       0xE0
#define ADS1220_REG_CONFIG0_PGA_GAIN_MASK 0x0E
#define ADS1220_REG_CONFIG0_MUX_MASK      0xF0

//Sample rate
#define ADS1220_DR_20SPS   0x00
#define ADS1220_DR_45SPS   0x20
#define ADS1220_DR_90SPS   0x40
#define ADS1220_DR_175SPS  0x60
#define ADS1220_DR_330SPS  0x80
#define ADS1220_DR_600SPS  0xA0
#define ADS1220_DR_1000SPS 0xC0

//PGA gain settings
#define ADS1220_PGA_GAIN_1   0x00
#define ADS1220_PGA_GAIN_2   0x02
#define ADS1220_PGA_GAIN_4   0x04
#define ADS1220_PGA_GAIN_8   0x06
#define ADS1220_PGA_GAIN_16  0x08
#define ADS1220_PGA_GAIN_32  0x0A
#define ADS1220_PGA_GAIN_64  0x0C
#define ADS1220_PGA_GAIN_128 0x0E

//Input mux
#define ADS1220_MUX_AIN0_AIN1 0x00
#define ADS1220_MUX_AIN0_AIN2 0x10
#define ADS1220_MUX_AIN0_AIN3 0x20
#define ADS1220_MUX_AIN1_AIN2 0x30
#define ADS1220_MUX_AIN1_AIN3 0x40
#define ADS1220_MUX_AIN2_AIN3 0x50
#define ADS1220_MUX_AIN1_AIN0 0x60
#define ADS1220_MUX_AIN3_AIN2 0x70
#define ADS1220_MUX_AIN0_AVSS 0x80
#define ADS1220_MUX_AIN1_AVSS 0x90
#define ADS1220_MUX_AIN2_AVSS 0xA0
#define ADS1220_MUX_AIN3_AVSS 0xB0

#define _BV(bit) (1<<(bit))

struct ADS1220_regs_s
{
	uint8_t cfg_reg0; // = 0x00;   //AINP=AIN0, AINN=AIN1, gain=1, PGA is enabled
	uint8_t cfg_reg1; // = 0x04;   //20SPS, Mode=Normal, Conversion=Continuous, Temp mode disabled, Current source disabled
	uint8_t cfg_reg2; // = 0x10;   //Internal 2.048V VREF, 50/60Hz filter, transistors open, IDAC off
	uint8_t cfg_reg3; // = 0x00;   //IDAC1&2 are disabled, only DRDY signals conversion completion
} ADS1220_default_regs =
{ 0x00, 0x04, 0x10, 0x00 };

typedef struct ADS1220_regs_s ADS1220_regs; // Default init

void ADS1220_writeRegister(SPI_HandleTypeDef *hspi, uint8_t address, uint8_t value)
{
	uint8_t arr[2] =
	{ ADS1220_WREG | (address << 2), value };

	HAL_SPI_Transmit(hspi, arr, 2, 100);
}

uint8_t ADS1220_readRegister(SPI_HandleTypeDef *hspi, uint8_t address)
{
	uint8_t data[2] =
	{ 0, 0 };

	uint8_t txd[2] =
	{ (ADS1220_RREG | (address << 2)), 0xFF };

	HAL_SPI_TransmitReceive(hspi, txd, data, 2, 1000); // When doing bidirectional, transmit a dummy byte(0xFF), 2 in total, received register is in [1]
	return data[1];
}

void ADS1220_reset(SPI_HandleTypeDef *hspi)
{
	const uint8_t cmd = ADS1220_RESET;
	HAL_SPI_Transmit(hspi, (uint8_t*) &cmd, 1, 100);
}

uint8_t ADS1220_init(SPI_HandleTypeDef *hspi, ADS1220_regs *r)
{
	ADS1220_reset(hspi);
	HAL_Delay(100);

	ADS1220_writeRegister(hspi, ADS1220_CONFIG_REG0_ADDRESS, r->cfg_reg0);
	ADS1220_writeRegister(hspi, ADS1220_CONFIG_REG1_ADDRESS, r->cfg_reg1);
	ADS1220_writeRegister(hspi, ADS1220_CONFIG_REG2_ADDRESS, r->cfg_reg2);
	ADS1220_writeRegister(hspi, ADS1220_CONFIG_REG3_ADDRESS, r->cfg_reg3);

	HAL_Delay(10);

	uint8_t CR0 = ADS1220_readRegister(hspi, ADS1220_CONFIG_REG0_ADDRESS);
	uint8_t CR1 = ADS1220_readRegister(hspi, ADS1220_CONFIG_REG1_ADDRESS);
	uint8_t CR2 = ADS1220_readRegister(hspi, ADS1220_CONFIG_REG2_ADDRESS);
	uint8_t CR3 = ADS1220_readRegister(hspi, ADS1220_CONFIG_REG3_ADDRESS);

	return (CR0 == r->cfg_reg0 && CR1 == r->cfg_reg1 && CR2 == r->cfg_reg2 && CR3 == r->cfg_reg3);
}

void ADS1220_start_conversion(SPI_HandleTypeDef *hspi)
{
	const uint8_t cmd = ADS1220_START;
	HAL_SPI_Transmit(hspi, (uint8_t*) &cmd, 1, 100);
}

void ADS1220_enable_PGA(SPI_HandleTypeDef *hspi, ADS1220_regs *r)
{
	r->cfg_reg0 &= ~_BV(0);
	ADS1220_writeRegister(hspi, ADS1220_CONFIG_REG0_ADDRESS, r->cfg_reg0);
}

void ADS1220_disable_PGA(SPI_HandleTypeDef *hspi, ADS1220_regs *r)
{
	r->cfg_reg0 |= _BV(0);
	ADS1220_writeRegister(hspi, ADS1220_CONFIG_REG0_ADDRESS, r->cfg_reg0);
}

void ADS1220_set_conv_mode_continuous(SPI_HandleTypeDef *hspi, ADS1220_regs *r)
{
	r->cfg_reg1 |= _BV(2);
	ADS1220_writeRegister(hspi, ADS1220_CONFIG_REG1_ADDRESS, r->cfg_reg1);
}

void ADS1220_set_conv_mode_single_shot(SPI_HandleTypeDef *hspi, ADS1220_regs *r)
{
	r->cfg_reg1 &= ~_BV(2);
	ADS1220_writeRegister(hspi, ADS1220_CONFIG_REG1_ADDRESS, r->cfg_reg1);
}

void ADS1220_set_data_rate(SPI_HandleTypeDef *hspi, int datarate, ADS1220_regs *r)
{
	r->cfg_reg1 &= ~ADS1220_REG_CONFIG1_DR_MASK;
	r->cfg_reg1 |= datarate;
	ADS1220_writeRegister(hspi, ADS1220_CONFIG_REG1_ADDRESS, r->cfg_reg1);
}

void ADS1220_select_mux_config(SPI_HandleTypeDef *hspi, int channels_conf, ADS1220_regs *r)
{
	r->cfg_reg0 &= ~ADS1220_REG_CONFIG0_MUX_MASK;
	r->cfg_reg0 |= channels_conf;
	ADS1220_writeRegister(hspi, ADS1220_CONFIG_REG0_ADDRESS, r->cfg_reg0);
}

void ADS1220_set_pga_gain(SPI_HandleTypeDef *hspi, int pgagain, ADS1220_regs *r)
{
	r->cfg_reg0 &= ~ADS1220_REG_CONFIG0_PGA_GAIN_MASK;
	r->cfg_reg0 |= pgagain;
	ADS1220_writeRegister(hspi, ADS1220_CONFIG_REG0_ADDRESS, r->cfg_reg0);
}

uint8_t* ADS1220_get_config(SPI_HandleTypeDef *hspi, ADS1220_regs *r)
{
	static uint8_t cfgbuf[4];

	r->cfg_reg0 = ADS1220_readRegister(hspi, ADS1220_CONFIG_REG0_ADDRESS);
	r->cfg_reg1 = ADS1220_readRegister(hspi, ADS1220_CONFIG_REG1_ADDRESS);
	r->cfg_reg2 = ADS1220_readRegister(hspi, ADS1220_CONFIG_REG2_ADDRESS);
	r->cfg_reg3 = ADS1220_readRegister(hspi, ADS1220_CONFIG_REG3_ADDRESS);

	cfgbuf[0] = r->cfg_reg0;
	cfgbuf[1] = r->cfg_reg1;
	cfgbuf[2] = r->cfg_reg2;
	cfgbuf[3] = r->cfg_reg3;

	return cfgbuf;
}

int32_t ADS1220_read_blocking(SPI_HandleTypeDef *hspi, GPIO_TypeDef *DRDY_PORT, uint16_t DRDY_PIN, uint16_t timeout) // Timeout should be at least as long as sampletime+some clock cycles, obviously
{
	uint8_t SPIbuf[3];
	int32_t result32 = 0;
	long int bit24;

	uint8_t time = 0;

	while (HAL_GPIO_ReadPin(DRDY_PORT, DRDY_PIN) == GPIO_PIN_SET)
	{
		HAL_Delay(1); // This is a bit hacky
		time++;
		if (time >= timeout)
			return 0;
	}

	HAL_SPI_Receive(hspi, SPIbuf, 3, 100);

	bit24 = SPIbuf[0];
	bit24 = (bit24 << 8) | SPIbuf[1];
	bit24 = (bit24 << 8) | SPIbuf[2]; //Converting 3 bytes to a 24 bit int

	bit24 = (bit24 << 8);
	result32 = (bit24 >> 8); //Converting 24 bit two's complement to 32 bit two's complement

	return result32;
}

int32_t ADS1220_read_singleshot(SPI_HandleTypeDef *hspi, GPIO_TypeDef *DRDY_PORT, uint16_t DRDY_PIN, uint16_t timeout)
{
	ADS1220_start_conversion(hspi);

	return ADS1220_read_blocking(hspi, DRDY_PORT, DRDY_PIN, timeout);
}

int32_t ADS1220_read_singleshot_channel(SPI_HandleTypeDef *hspi, uint8_t channel_num, ADS1220_regs *r, GPIO_TypeDef *DRDY_PORT, uint16_t DRDY_PIN, uint16_t timeout)
{
	ADS1220_select_mux_config(hspi, channel_num, r);

	ADS1220_start_conversion(hspi);

	return ADS1220_read_blocking(hspi, DRDY_PORT, DRDY_PIN, timeout);
}

#endif /* INC_ADS1220_H_ */
