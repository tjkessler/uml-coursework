#include <stdio.h>
#include <stdlib.h>


//#include <AL/alut.h>
#include <AL/al.h>
#include <AL/alc.h>



#include "buffer.h"
#include "wave.h"
void Prebuffer()
{
extern void sfastfilt(int, float *, float *, float *);
extern float H[2][BSIZE];
extern float X[2][BSIZE],SAVE[2][BSIZE];
extern WAVE_Struct wave;
extern  ALbyte DATA[BSIZE];
extern  ALuint Buffers[NUMBUFFERS];
extern  ALuint DataSize;
extern FILE *fp;
extern int N;
extern ALuint Format;

short *m;
int i,j,k;

    if ( DataSize==wave.dataSize) //none read yet
     {
       for(k=0; k <NUMBUFFERS;k++)
        {
        fread(DATA,1 , BSIZE,fp);
        DataSize= DataSize -BSIZE;
        //=FILTER======================
	// m assumes 16 bits per sample !!!
          m = (short *)DATA;
         for (j=0;j<wave.Channels;j++)
         {
         for (i=0;i< N;i++) X[j][i]  = m[wave.Channels*i+j];        
         sfastfilt(N,(float *)&H[j][0], (float*)&X[j][0],(float *)&SAVE[j][0] );
         for (i=0;i< N;i++) m[wave.Channels*i+j] =(short)X[j][i];
         }
        //=FILTER END======================
        alBufferData(Buffers[k],Format,DATA,BSIZE,wave.SamplesPerSec);
        }
     }

}

