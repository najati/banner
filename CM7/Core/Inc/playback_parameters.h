
#ifndef __PLAYBACK_PARAMETERS__
#define __PLAYBACK_PARAMETERS__

#include <stdint.h>

#define AUDIO_FREQUENCY_192K     192000U
#define AUDIO_FREQUENCY_96K       96000U
#define AUDIO_FREQUENCY_48K       48000U

#define AUDIO_RESOLUTION_16B                16U
#define AUDIO_RESOLUTION_32B                32U

#define AUDIO_FREQUENCY     	    AUDIO_FREQUENCY_48K
#define AUDIO_RESOLUTION         AUDIO_RESOLUTION_16B

typedef struct
{
  uint32_t                    SampleRate;
  uint32_t                    BitsPerSample;
  uint32_t                    ChannelsNbr;
  uint32_t                    Volume;
} PlaybackParameters;

#endif
