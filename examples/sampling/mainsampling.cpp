// mainsampling.cpp  a simple code that shows a graphical comparation among three sample strategies..
//


#include <mrcore.h>
#include <GL/glut.h>
#include <iostream>


using namespace mr;
using namespace std;

void OnDraw(void);
void OnTimer(int value); //esta funcion sera llamada cuando transcurra una temporizacion
void OnKeyboardDown(unsigned char key, int x, int y); //cuando se pulse una tecla	
void OnMouseMove(int x,int y);
void OnMouseClick(int button,int state, int x,int y);

GLScene scene;
World world;



Sampler *samplerU,*samplerR, *samplerG,*sampler;
BoundingBox box,box1,box2;
vector<Sample> sampleSet;




int iteracion=0;
//***************variables globales para organizar un poco el sistema



int main(int argc, char* argv[])
{
	mrcoreInit();
	//GL Initialization stuff
	glutInit(&argc, argv);
	glutInitWindowSize(800,600);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutCreateWindow("SAMPLING STRATEGIES");
	glutDisplayFunc(OnDraw);
	glutMotionFunc(OnMouseMove);
	glutMouseFunc(OnMouseClick);
	glutKeyboardFunc(OnKeyboardDown);

	scene.init();



samplerR=new RandomSampler(Vector3D(0,0,0), Vector3D(10,10,10));
samplerU=new UniformSampler(Vector3D(12,0,0), Vector3D(10,10,10));
samplerG=new GaussianSampler(Vector3D(24,0,0), Vector3D(10,10,10));

box=BoundingBox(Vector3D(0,0,0), Vector3D(10,10,10));
box1=BoundingBox(Vector3D(12,0,0), Vector3D(22,10,10));
box2=BoundingBox(Vector3D(24,0,0), Vector3D(34,10,10));

//comparative 2D sampling 
/*
samplerR=new RandomSampler(Vector2D(0,0), Vector2D(10,10));
samplerU=new UniformSampler(Vector2D(12,0), Vector2D(10,10));
samplerG=new GaussianSampler(Vector2D(24,0), Vector2D(10,10));

box=BoundingBox(Vector3D(0,0,0), Vector3D(10,10,0.1));
box1=BoundingBox(Vector3D(12,0,0), Vector3D(22,10,0.10));
box2=BoundingBox(Vector3D(24,0,0), Vector3D(34,10,0.10));
*/

	glutMainLoop();
	return 0;
}

void OnDraw(void)
{  
glDisable(GL_LIGHTING);
GLTools::BackgroundColor(BLACK);
scene.setShowGrid(false);
scene.setShowFrame(false);
scene.BackgroundColor(0.9, 0.9, 1);
scene.Draw();

char mensaje[200];
sprintf_s(mensaje,200,"Number of Samples: %d\n",sampleSet.size()/3);
glColor3f(0,0,0);
GLTools::Color(BLACK);
glTranslated(15, 5, 14);
GLTools::Print(mensaje,1.5);
glTranslated(-15, -5, -14);
	cout<<mensaje;
GLTools::DrawGrid(50,2,BLUE);
//local drawing
glTranslated(-17, 0, 0);
	glPointSize(3);
	glColor3f(0.2,0,0);
	glBegin(GL_POINTS);
		for(unsigned int i=0;i<sampleSet.size();i++)
		{
			if(sampleSet[i].size()>2)
				glVertex3f(sampleSet[i][0],sampleSet[i][1],sampleSet[i][2]);
			else if(sampleSet[i].size()==2)
				glVertex3f(sampleSet[i][0],sampleSet[i][1],0.001);

		}
	glEnd();
	box.drawGL();
	box1.drawGL();
	box2.drawGL();

	glutSwapBuffers();
}


void OnKeyboardDown(unsigned char key, int x, int y)
{
	sampleSet.push_back(samplerR->getNextSample());
	sampleSet.push_back(samplerU->getNextSample());
	sampleSet.push_back(samplerG->getNextSample());
	glutPostRedisplay();	
}
void OnMouseClick(int b,int state, int x,int y)
{
	bool down=(state==GLUT_DOWN);
	int button;
	if(b==GLUT_LEFT_BUTTON)	button=MOUSE_LEFT_BUTTON;
	if(b==GLUT_RIGHT_BUTTON)button=MOUSE_RIGHT_BUTTON;
	int specialKey = glutGetModifiers();
	bool ctrlKey= (specialKey & GLUT_ACTIVE_CTRL)?true:false ;
	bool sKey= specialKey&GLUT_ACTIVE_SHIFT ;
	
	scene.MouseButton(x,y,b,down,sKey,ctrlKey);
	glutPostRedisplay();
}
void OnMouseMove(int x,int y)
{
	scene.MouseMove(x,y);
	glutPostRedisplay();
}

