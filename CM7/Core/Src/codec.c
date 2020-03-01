
#include "playback_parameters.h"

#include "stm32h7xx_hal.h"

#include "wm8994.h"


#define AUDIO_I2C_ADDRESS                0x34U

#define VOLUME_OUT_CONVERT(Volume)    (((Volume) > 100)? 63:((uint8_t)(((Volume) * 63) / 100)))
#define VOLUME_IN_CONVERT(Volume)     (((Volume) >= 100)? 239:((uint8_t)(((Volume) * 239) / 100)))

int32_t ReturnZero(void) { return 0; }

// TODO
I2C_HandleTypeDef* hi2c;

int32_t NAJ_I2C4_WriteReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length) {
	return HAL_I2C_Mem_Write(hi2c, DevAddr, Reg, I2C_MEMADD_SIZE_16BIT, pData, Length, 1000);
}

int32_t NAJ_I2C4_ReadReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length) {
	return HAL_I2C_Mem_Read(hi2c, DevAddr, Reg, I2C_MEMADD_SIZE_16BIT, pData, Length, 1000);
}

/**
 * @brief  Delay function
 * @retval Tick value
 */
int32_t NAJ_GetTick(void)
{
	return (int32_t) HAL_GetTick();
}

int32_t InitCodec(PlaybackParameters* AudioInit, I2C_HandleTypeDef* hbus_i2c) {
	hi2c = hbus_i2c;

	WM8994_IO_t IOCtx;
	WM8994_Object_t WM8994Obj;
	uint32_t id;

	/* Configure the audio driver */
	IOCtx.Address     = AUDIO_I2C_ADDRESS;
	IOCtx.Init        = ReturnZero;
	IOCtx.DeInit      = ReturnZero;
	IOCtx.ReadReg     = NAJ_I2C4_ReadReg16;
	IOCtx.WriteReg    = NAJ_I2C4_WriteReg16;
	IOCtx.GetTick     = NAJ_GetTick;

	if(WM8994_RegisterBusIO(&WM8994Obj, &IOCtx) != WM8994_OK) {
		return -1;
	}

	/* Reset the codec */
	if(WM8994_Reset(&WM8994Obj) != WM8994_OK) {
		return -1;
	}

	if(WM8994_ReadID(&WM8994Obj, &id) != WM8994_OK) {
		return -1;
	}

	if(id != WM8994_ID) {
		return -1;
	}

	WM8994_Drv_t *Audio_Drv = &WM8994_Driver;

	void *Audio_CompObj = &WM8994Obj;

	WM8994_Init_t codec_init;
	codec_init.Resolution = WM8994_RESOLUTION_16b;
	codec_init.Frequency = AudioInit->SampleRate;
	codec_init.InputDevice = WM8994_IN_LINE1;
	codec_init.OutputDevice = WM8994_OUT_HEADPHONE;

	/* Convert volume before sending to the codec */
	codec_init.Volume       = VOLUME_OUT_CONVERT(AudioInit->Volume);

	/* Initialize the codec internal registers */
	return Audio_Drv->Init(Audio_CompObj, &codec_init);
}
