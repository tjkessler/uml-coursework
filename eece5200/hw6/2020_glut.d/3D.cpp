#define N 20 
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifdef LINUX
#include <GL/freeglut.h>
#endif
#ifdef MAC 
#include <GLUT/glut.h>
#endif


//
float points[N][N][3];
float roty;

//glut global
int screen_width, screen_height;
int x,y;
void init(void);
void display(void);
void ProcessKeyboard   (unsigned char key, int x,int y);
void ProcessMouse      (int button,int state, int x, int y);
void ProcessMouseMotion(int x, int y);
void idle(void);
//------------------

// Stuff Travis added
float _OFFSET_X = 0.0;
float _SCALAR_Y = 1.0;

int main(int argc, char** argv)
{
	extern int screen_height,screen_width;

	screen_width = 300;
	screen_height= 300;
	roty=0;

	glutInit(&argc,argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(screen_width,screen_height);
	glutInitWindowPosition(0,0);
	glutCreateWindow("Travis Kessler's Homework 6");

	glutDisplayFunc (display);
	glutKeyboardFunc(ProcessKeyboard);
	glutMouseFunc   (ProcessMouse);
	glutMotionFunc  (ProcessMouseMotion);
	glutIdleFunc	(idle);
	init();
	glutMainLoop();

	return 0;
}

//-----------
void init(void)
{
	glClearColor(.0,.0,.0,.0); //bg black
	glClearDepth(1.0);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//
	//gluPerspective(60,1,1,1000);
	gluPerspective(45,1,1,9);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(		//setup camera
		2.0,3.0,3.0,	//eye position
		0.0,0.0,0.0,	//look at position	
		0.0,1.0,0.0);	//up direction

	glEnable(GL_DEPTH_TEST);//enable hidden line removal

	glEnable(GL_LIGHTING);	//enable lighting
	glEnable(GL_LIGHT0);	//enable 

	float lpos[ ]={0,2,0,0};
	glLightfv(GL_LIGHT0,GL_AMBIENT,lpos);

	glShadeModel(GL_SMOOTH); //smooth shading

	glPolygonMode(GL_BACK, GL_FILL);
	glPolygonMode(GL_FRONT,GL_LINE);

}
//--------------------------
void display(void)
{
	extern float points[N][N][3];
	extern float roty;

	float xx,yy;
	int i,j;

	for (i=0;i<N;i++){
		for (j=0;j<N;j++){
			points[i][j][0]= -1 + i*2.0/(N-1); //x
			points[i][j][1]= -1 + j*2.0/(N-1); //z
			points[i][j][2]= sin(3.14*points[i][j][0]) * _SCALAR_Y; //y
			points[i][j][0] = points[i][j][0] + _OFFSET_X;
		}
	}

	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);	// clear the depth buffer


	glPushMatrix();		//save current camera transformtion
   	glRotatef(roty,0.0,1.0,0.0); // rotate roty degrees
	// define x-axis
	glLineWidth(2);
	glColor3f(1.0,0.0,0.0); //red
	glBegin(GL_LINES);
	 glVertex3f(-1.0,0.0,0.0);
	 glVertex3f( 1.0,0.0,0.0);
	glEnd();
	// define y-axis
	glLineWidth(2);
	glColor3f(0.0,1.0,0.0); //green
	glBegin(GL_LINES);
	 glVertex3f(0.0, -1.0,0.0);
	 glVertex3f(0.0,  1.0,0.0);
	glEnd();
	// define z-axis
	glLineWidth(2);
	glColor3f(1.0,1.0,0.0); //green
	glBegin(GL_LINES);
	 glVertex3f(0.0, 0.0,-1.0);
	 glVertex3f(0.0, 0.0, 1.0);
	glEnd();

	// define function values
	glLineWidth(1);
	glColor3f(1.0,1.0,1.0);
	glBegin(GL_QUADS);
	  for(i=0;i<(N-1);i++){
	  for(j=0;j<(N-1);j++){
	glVertex3f(points[i][j][0],points[i][j][2],points[i][j][1]);
	glVertex3f(points[i][j+1][0],points[i][j+1][2],points[i][j+1][1]);
	glVertex3f(points[i+1][j+1][0],points[i+1][j+1][2],points[i+1][j+1][1]);
	glVertex3f(points[i+1][j][0],points[i+1][j][2],points[i+1][j][1]);
	}}	
	glEnd();

	glPopMatrix();

	//draw
	glFlush();	
	glutSwapBuffers();

}
//----
void ProcessKeyboard   (unsigned char key, int x,int y)
{
	extern float roty;

	switch(key)
		{
		case 27: //ESCAPE
			exit(0);
			break;
		case '0':
			break;
		case 'c':
			break;
		case 'p':
			{
			roty = (roty+10);
			display();
			printf("%f\n",roty);
			}
			break;
		case 'm':
			{
			roty = (roty-10);
			display();
			printf("%f\n",roty);
			}
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
void idle(void)
{

}
//----
