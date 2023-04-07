#include <stdlib.h>
#include <math.h>
//---------------------------
#ifdef LINUX
#include <GL/freeglut.h>
#endif
#ifdef MAC
#include <GLUT/glut.h>
#endif

//glut global
int screen_width, screen_height;
int x,y;
void init(void);
void display(void);
void ProcessKeyboard   (unsigned char key, int x,int y);
void ProcessMouse      (int button,int state, int x, int y);
void ProcessMouseMotion(int x, int y);
void ProcessIdle(void);
int NPTS_IN_X;
//------------------

// Stuff Travis added
float _OFFSET_X = 0.0;
float _SCALAR_Y = 1.0;

int main(int argc, char* argv[])
{
	extern int screen_height,screen_width;
	extern int NPTS_IN_X;

	NPTS_IN_X=100;
	screen_width = 500; // in pixels
	screen_height= 250; // in pixles

	glutInit(&argc,argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(screen_width,screen_height);
	glutInitWindowPosition(0,0);
	glutCreateWindow("Travis Kessler's Homework 6");

	glutDisplayFunc (display);
	glutKeyboardFunc(ProcessKeyboard);
	glutMouseFunc   (ProcessMouse);
	glutMotionFunc  (ProcessMouseMotion);
	glutIdleFunc	(ProcessIdle);
	init();
	glutMainLoop();

	return 0;
}

//-----------
void init(void)
{
	glClearColor(1.0,1.0,1.0,1.0); //background  color white
	gluOrtho2D(-1.0,1.0,-1.0,1.0);
}
//--------------------------

void display(void)
{
	extern int NPTS_IN_X;
	float xx,yy;
	int i;


	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	// define x-axis
	glLineWidth(2);
	glColor3f(0.0,0.0,1.0); //blue
	glBegin(GL_LINES);
	glVertex2f(-.75f,0.0f);
	glVertex2f( .75f,0.0f);
	glEnd();
	// define y-axis
	glLineWidth(2);
	glColor3f(0.0,0.0,1.0); //blue
	glBegin(GL_LINES);
	glVertex2f(0.0f,-.75f);
	glVertex2f(0.0f, .75f);
	glEnd();


// define function values
	glLineWidth(3);
	glColor3f(1.0,0.0,0.0); //red
	glBegin(GL_LINE_STRIP);
	for(i=0;i<NPTS_IN_X;i++) {
		xx = -1 + i*2.0/NPTS_IN_X;
		yy = sin (3.14159*xx) * _SCALAR_Y;
		xx = xx + _OFFSET_X;
		glVertex2f(xx,yy);
	}
	glEnd();

//draw
glFlush();	
glutSwapBuffers();

}
//----
void ProcessKeyboard   (unsigned char key, int x,int y)
{
	float _offset, xx, yy;
	switch(key)
	{
		case 27: //ESCAPE
			exit(0);
			break;
		case '0':
			break;
		case 'c':
			break;
		case 'r':
			_OFFSET_X += 0.1;
			break;
		case 'l':
			_OFFSET_X += -0.1;
			break;
		case 'u':
			_SCALAR_Y += 0.1;
			break;
		case 'd':
			_SCALAR_Y += -0.1;
			break;
		default:
			break;
	}
	glutPostRedisplay();
}
//----
void ProcessMouse        (int button,int state, int x, int y)
{
	if(state== GLUT_DOWN){
		if(button== GLUT_LEFT_BUTTON){
			exit(0);
			return;
		}
		else if(button== GLUT_MIDDLE_BUTTON){
			return;
		}
		else if(button== GLUT_RIGHT_BUTTON){
			return;
		}
	} 

	else{
		return;
	}

}
//----
void ProcessMouseMotion  (int x, int y)
{

}
//----
void ProcessIdle(void)
{

}
//----
