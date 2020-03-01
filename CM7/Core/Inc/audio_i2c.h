
#ifndef INC_AUDIO_I2C_H_
#define INC_AUDIO_I2C_H_

#include <stdint.h>

#define AUDIO_I2C_ADDRESS                ((uint16_t)0x34)

/* AUDIO I2C functions */
void            AUDIO_IO_Init(void);
void            AUDIO_IO_DeInit(void);
void            AUDIO_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value);
uint16_t        AUDIO_IO_Read(uint8_t Addr, uint16_t Reg);
void            AUDIO_IO_Delay(uint32_t Delay);

#endif
