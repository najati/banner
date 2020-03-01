
#include "stm32h7xx_hal.h"

#include "audio_i2c.h"

extern I2C_HandleTypeDef hi2c4;

void AUDIO_IO_Init(void) {
}

void AUDIO_IO_DeInit(void) { }

void AUDIO_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value) {
	uint16_t tmp = Value;

	Value = ((uint16_t)(tmp >> 8) & 0x00FF);
	Value |= ((uint16_t)(tmp << 8)& 0xFF00);

	HAL_I2C_Mem_Write(&hi2c4, Addr, Reg, I2C_MEMADD_SIZE_16BIT,(uint8_t*)&Value, 2, 1000);
}

uint16_t AUDIO_IO_Read(uint8_t Addr, uint16_t Reg) {
	uint16_t read_value = 0, tmp = 0;

	HAL_I2C_Mem_Read(&hi2c4, Addr, Reg, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&read_value, 2, 1000);

	tmp = ((uint16_t)(read_value >> 8) & 0x00FF);
	tmp |= ((uint16_t)(read_value << 8)& 0xFF00);
	read_value = tmp;

	return read_value;
}

void AUDIO_IO_Delay(uint32_t Delay) {
	HAL_Delay(Delay);
}


