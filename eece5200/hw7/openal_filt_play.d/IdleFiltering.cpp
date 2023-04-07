//#include <AL/alut.h>

#include <AL/al.h>
#include <AL/alc.h>


#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include "buffer.h"
#include "wave.h"

void IdleFiltering(void)
{
//start
extern void sfastfilt(int, float *, float *, float*);
extern float H[2][BSIZE],X[2][BSIZE],SAVE[2][BSIZE];
int i,j,k;
extern ALbyte          DATA   [BSIZE];
extern ALuint          Buffers[NUMBUFFERS];
extern ALuint          BufferID;
extern ALuint          DataSize;
extern ALuint          DataToRead;
extern ALint           processed;
extern ALboolean	bFinished;
extern ALuint		Format;
extern ALint		State;
extern ALuint		Source;
extern ALuint 		buffersreturned;
extern ALuint  		buffersinqueue;
extern ALboolean 	bFinishedPlaying;
extern int N;
extern FILE *fp;
extern WAVE_Struct wave;

short *m;

alGetSourcei(Source,AL_SOURCE_STATE,&State);
 if ( !bFinishedPlaying && (State == AL_PLAYING) )
 {
  alGetSourcei(Source,AL_BUFFERS_PROCESSED,&processed); //status
  if (processed >0)
    {
     buffersreturned += processed;
                while(processed) //BEGIN WHILE 1
                {
                 alSourceUnqueueBuffers(Source,1,&BufferID);
                  if(!bFinished)
                    {
                     DataToRead = (DataSize >BSIZE)?BSIZE:DataSize;
                     if(DataToRead == DataSize) bFinished= AL_TRUE;
                     fread(DATA,1,DataToRead,fp);
                     DataSize -= DataToRead;
                     if (bFinished ==AL_TRUE)
                                memset(DATA+DataToRead,0,BSIZE-DataToRead);
//=FILTER===================================
// m assume data is 16 bits!
                     m = (short *)DATA;
                     for (j=0;j<wave.Channels;j++)
                     {
                      for (i=0;i< N;i++) X[j][i]= m[wave.Channels*i+j];  
                      sfastfilt(N,&H[j][0], &X[j][0],&SAVE[j][0] );
                      for (i=0;i< N;i++) m[wave.Channels*i+j] =(short)X[j][i];
                     }
//=FILTER END===================================
                   alBufferData(BufferID,Format,DATA,BSIZE,wave.SamplesPerSec);
                   alSourceQueueBuffers(Source,1,&BufferID);
                   processed--;
                    }
                  else
                    {
                     buffersinqueue--;
                     processed--;
                     if(buffersinqueue == 0 )
                        {
                         bFinishedPlaying = true;
                         break;
                        }
                     }
                } //WHILE 1 END
        } //ENDIF
  } //ENDIF


//end
}
