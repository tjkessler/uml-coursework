#include <stdlib.h>
#include <stdio.h>
#include <AL/al.h>
#include "wave.h"

FILE *Openwavefile( const char *filename, WAVE_Struct *wave, int size)
{
 FILE *fp;
 extern ALuint DataSize;
 extern ALuint Format;

 fp = fopen( filename,"rb");

 if (fp==NULL)
  {
        printf ("can not open wavefile %s\n",filename);
        return NULL;
  }
  if (fread(wave,1,size,fp) !=(unsigned)size)
  {
        printf(" can not read file header\n");
        fclose(fp);
        return NULL;
  }
//==========
printf("\tWAVEFILE header:%s\n",filename);
printf("\twave->riff= %cif%c\n",wave->riff[0],wave->riff[3]);
printf("\twave->wave= %cav%c\n",wave->wave[0],wave->wave[3]);
printf("\twave->fmt= %cm%c\n",wave->fmt[0],wave->fmt[2]);

printf("\twave->Channels= %d\n",wave->Channels);
printf("\twave->SamplesPerSec= %d\n",wave->SamplesPerSec);
printf("\twave->BytesPerSec = %d\n",wave->BytesPerSec );
printf("\twave->BitsPerSample = %d\n",wave->BitsPerSample );
printf("\twave->dataSize= %d\n",wave->dataSize);
//===========
// data needed is GLOBAL
// DataSize, Format,N2,N
        DataSize=wave->dataSize;
 if (wave->BitsPerSample ==16)
        {
          if(wave->Channels == 1)
            Format = AL_FORMAT_MONO16;
          else
            Format = AL_FORMAT_STEREO16;
        }
        else
        {
          printf (" GET INFO WAVEFILE not 16 bit\n");
         return NULL;
        }
  return fp;
}

