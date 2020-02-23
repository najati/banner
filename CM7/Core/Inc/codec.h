
#ifndef __CODEC__
#define __CODEC__

#include "playback_parameters.h"

int32_t InitCodec(PlaybackParameters* AudioInit, I2C_HandleTypeDef* hbus_i2c);

#endif
