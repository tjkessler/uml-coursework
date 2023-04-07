//number of bytes in buffer
//#define BSIZE	(4096*2)  
//number of short int per channel N = BIZE/channels/2
//#define NUMBUFFERS	(8)
#include "buffer.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ncurses.h> //non blocking kbd io
#include <math.h>

// openal
#include <AL/al.h>
#include <AL/alc.h>

// opengl
#include <GLUT/glut.h>
int screen_width, screen_height, icenterx, icentery;
int x, y;
void display(void);
void processMouse(int button, int state, int x, int y);
void processMouseMotion(int x, int y);
void processKeyboard(unsigned char key, int x, int y);
void init(void);
void idle();

static GLfloat light_ambient[]  = { 0.0, 0.0, 0.0, 1.0 };
static GLfloat light_diffuse[]  = { 1.0, 1.0, 1.0, 1.0 };
static GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
static GLfloat light_position[] = { 0.0, 1.0, 1.0, 0.0 };

static GLfloat mat_ambient[]    = { 0.7, 0.7, 0.7, 1.0 };
static GLfloat mat_diffuse[]    = { 0.8, 0.8, 0.8, 1.0 };
static GLfloat mat_specular[]   = { 1.0, 1.0, 1.0, 1.0 };
static GLfloat high_shininess[] = { 100.0 };

//wave file vars
#include "wave.h"
WAVE_Struct	wave;
FILE		*fp;

FILE * Openwavefile( const char *, WAVE_Struct *, int );
void Prebuffer(void);
void IdleFiltering (void);
void srealfft(int, float *);


//filter
float H[2][BSIZE],X[2][BSIZE],SAVE[2][BSIZE],TIME[BSIZE],bufy[BSIZE];

// process vars
const char      file[] = {"music.wav"};
ALbyte          DATA   [BSIZE];
ALuint          Buffers[NUMBUFFERS];
ALuint          BufferID;
ALuint          DataSize;
ALuint          DataToRead;
ALint           processed;
ALboolean       bFinished = AL_FALSE;
ALuint          Format;
ALint           State;
ALuint 		Source;

ALuint	buffersreturned=0;
ALuint	buffersinqueue=NUMBUFFERS;
ALboolean bFinishedPlaying;
int N;

//++++++++++++++++++++++++++++++++++++++++++

ALfloat SourcePos[] = { 0.0, 0.0, 0.0 };
ALfloat SourceVel[] = { 0.0, 0.0, 0.0 };
ALfloat ListenerPos[] = { 0.0, 0.0, 0.0 };
ALfloat ListenerVel[] = { 0.0, 0.0, 0.0 };
ALfloat ListenerOri[] = { 0.0, 0.0, -1.0,  0.0, 1.0, 0.0 };




int main(int argc, char *argv[])
{
	extern FILE *fp;
	extern ALuint  Buffers[NUMBUFFERS];
	extern float H[2][BSIZE],X[2][BSIZE],SAVE[2][BSIZE];
	extern int N;
	int i,j;

	// START: OPEN DEFAULT AUDIO DEVICE
 	ALCdevice *device;
 	ALCcontext *ctx;
 	const ALCchar  *name;

  	device = alcOpenDevice(NULL);
   	if(!device) { printf("cannot open default device\n");exit(0);}

  	ctx = alcCreateContext(device,NULL);
    if(ctx == NULL || alcMakeContextCurrent(ctx) == ALC_FALSE)
    {
		if(ctx != NULL) alcDestroyContext(ctx);
        alcCloseDevice(device);
        fprintf(stderr, "Could not set a context!\n");
		exit(0);
	}

    name = NULL;
    if(alcIsExtensionPresent(device, "ALC_ENUMERATE_ALL_EXT"))
        name = alcGetString(device, ALC_ALL_DEVICES_SPECIFIER);
    if(!name || alcGetError(device) != AL_NO_ERROR)
        name = alcGetString(device, ALC_DEVICE_SPECIFIER);
    printf("Opened  \"%s\"\n", name);
	// END: OPEN DEFAULT AUDIO DEVICE

	// START: GET DATA BUFFERS
	alGetError(); //reset error
	alGenBuffers(NUMBUFFERS,Buffers);
	if(alGetError() != AL_NO_ERROR) printf(" Error alGenBuffers \n");
	// END: GET DATA BUFFERS

	// open wavefile assumed stereo 16 bit per sample
	fp =Openwavefile( (const char *)&file,&wave, sizeof(WAVE_Struct));

	// N = number of (wave.BitsPerSample/8) Byte samples per channel
	// Min size needed for signal and filter samples for overlap-save filtering is 2*N

	N = (BSIZE/wave.Channels)/(wave.BitsPerSample/8);

	// Begin: INITIALIZE DSP FILTERING
	for(j=0;j<wave.Channels;j++)
	{
		for(i=0;i<BSIZE;i++)
		{
			H[j][i]=0;
			X[j][i]=0;
			SAVE[j][i]=0;
		}
	}
	for (i = 0; i < BSIZE; i++)
	{
		TIME[i] = 0.5 * (i * 2.0 / (N / 2) - 1);
		bufy[i] = 0;
	}
	// impluse function of the filter
	for(j=0;j<wave.Channels;j++)
		H[j][0]=1;

	// DFT of H
	srealfft(N,&H[0][0]);
	srealfft(N,&H[1][0]);

	// Prebuffer and filter prebuffer NBUFFERS of data
	Prebuffer();
	// End: INITIALIZE DSP FILTERING

	// Generate 1 source
	alGetError();
	alGenSources(1,&Source);
	if(alGetError() != AL_NO_ERROR) printf(" location bad 2\n");

	// Set source features
	alSourcef (Source, AL_GAIN,     1.0);
	alSourcef (Source, AL_PITCH,    1.0);
	alSourcefv(Source, AL_POSITION, SourcePos);
	alSourcefv(Source, AL_VELOCITY, SourceVel);

	// Set listener position, velocity, orientation
	alListenerfv( AL_POSITION,    ListenerPos);
	alListenerfv( AL_VELOCITY,    ListenerVel);
	alListenerfv( AL_ORIENTATION, ListenerOri);

	// queue buffers "Source"
	alSourceQueueBuffers(Source,NUMBUFFERS,Buffers);
	// Do not loop buffers
	alSourcei(Source,AL_LOOPING,AL_FALSE);

	// IMPORTANT TO IdleFiltering();
	// Initialize buffer status and state
	buffersreturned=0;
	bFinishedPlaying = AL_FALSE;
	buffersinqueue = NUMBUFFERS;

	printf("p) Play \n");
	printf("h) Hold \n");
	printf("s) Stop \n");
	printf("q) Quit \n");

	screen_width = 800;
	screen_height = 800;
	icenterx = (screen_width) / 2;
	icentery = (screen_height) / 2;

	IdleFiltering();

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(screen_width,screen_height);
	glutInitWindowPosition(0,0);
	glutCreateWindow("Big Iron");

	glutDisplayFunc(display);
	glutKeyboardFunc(processKeyboard);
	glutMouseFunc(processMouse);
	glutMotionFunc(processMouseMotion);
	init();
	glutIdleFunc(idle);
    glutMainLoop();

	alSourceStop(Source);
	// CLOSE file
	fclose(fp);
	// CLOSE AL
	//retrive context and device
	ctx = alcGetCurrentContext();
	device = alcGetContextsDevice(ctx);
	//close context and device	
	alcMakeContextCurrent(NULL);
	alcDestroyContext(ctx);
	alcCloseDevice(device);

	// close ncurses window
	endwin();

	return 0;
}

void processMouse(int button, int state, int x, int y)
{
	return;
}

void processMouseMotion(int x, int y)
{
	return;
}

void init(void)
{
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glHint(GL_LINE_SMOOTH_HINT,GL_DONT_CARE);
	glClearColor(1.0,1.0,1.0,1.0);
	gluOrtho2D(-1.0,1.0,-1.0,1.0);
	return;
}

void idle()
{
	IdleFiltering();
	glutPostRedisplay();
}

void display(void)
{
	extern int screen_width, screen_height, icenterx, icentery;
	extern float X[2][BSIZE], TIME[BSIZE];
	extern int N, N2;
	int i, j, M;
	float SCALE, trigger_level, trigger_slope, level, slope;
	int index;
	float bufy[BSIZE];

	SCALE = 24000;
	index = 0;
	trigger_level = 0.0100 * 4;
	trigger_slope = 0.0005 * 2;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	for (i = 0; i < (N - 1); i++)
	{
		level = X[0][i] / SCALE;
		slope = (X[0][i + 1] - X[0][i]) / SCALE;
		if ((level >= trigger_level) && (slope >= trigger_slope)) index = i;
		if (index != 0) break;
	}

	if (index != 0)
	{
		glLineWidth(4);
		glColor3f(0, 0, 0);
		glBegin(GL_LINE_STRIP);
		M = 1;
		for (i = 0; i < N; i = i + M)
		{
			for (j = 0; j < M; j++) bufy[i + j] = X[0][i] / SCALE;
		}
		for (i = 0; i < N / 2; i++)
		{
			if (i + index < N) glVertex2f(TIME[i], bufy[i + index] * exp(-10 * TIME[i] * TIME[i]));
		}
		glEnd();
	}

	glLineWidth(1);
	glColor3f(0, 0, 1);
	glBegin(GL_LINES);
	glVertex2f(-icenterx, 0);
	glVertex2f(icenterx, 0);
	glEnd();

	glLineWidth(1);
	glColor3f(0, 1, 0);
	glBegin(GL_LINES);
	glVertex2f(0, -icentery);
	glVertex2f(0, icentery);
	glEnd();

	glFlush();
	glutSwapBuffers();
}

void processKeyboard(unsigned char key, int x,int y)
{
	switch(key)
	{
		case 'p': {
			alGetSourcei(Source, AL_SOURCE_STATE, &State);
			if (State != AL_PLAYING) alSourcePlay(Source);
			printf("Playing audio\n");
			break;
		}
		case 'h': {
			printf("Hold audio\n");
			alSourcePause(Source);
			break;
		}
		case 's': {
			printf("Stop audio\n");
			alSourceStop(Source);
		}
		case 'q': {
			exit(0);
			break;
		}
		default:
			break;
	}
}
