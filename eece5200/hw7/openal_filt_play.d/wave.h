// WAVE VARS
typedef struct
{
        ALubyte         riff[4];                // 'RIFF'
        ALsizei         riffSize;
        ALubyte         wave[4];                // 'WAVE'
        ALubyte         fmt[4];                 // 'fmt '
        ALuint          fmtSize;
        ALushort        Format;
        ALushort        Channels;
        ALuint          SamplesPerSec;
        ALuint          BytesPerSec;
        ALushort        BlockAlign;
        ALushort        BitsPerSample;
        ALubyte         data[4];                // 'data'
        ALuint          dataSize;
} WAVE_Struct;


