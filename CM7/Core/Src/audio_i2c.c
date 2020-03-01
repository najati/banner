
#include "audio_i2c.h"

static I2C_HandleTypeDef hI2cAudioHandler = {0};

void AUDIO_IO_Init(void) {
}

void AUDIO_IO_DeInit(void) { }

void AUDIO_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value) {
	uint16_t tmp = Value;

	Value = ((uint16_t)(tmp >> 8) & 0x00FF);
	Value |= ((uint16_t)(tmp << 8)& 0xFF00);

	I2Cx_WriteMultiple(&hI2cAudioHandler, Addr, Reg, I2C_MEMADD_SIZE_16BIT,(uint8_t*)&Value, 2);
}

uint16_t AUDIO_IO_Read(uint8_t Addr, uint16_t Reg) {
	uint16_t read_value = 0, tmp = 0;

	I2Cx_ReadMultiple(&hI2cAudioHandler, Addr, Reg, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&read_value, 2);

	tmp = ((uint16_t)(read_value >> 8) & 0x00FF);
	tmp |= ((uint16_t)(read_value << 8)& 0xFF00);
	read_value = tmp;

	return read_value;
}

void AUDIO_IO_Delay(uint32_t Delay) {
	HAL_Delay(Delay);
}


