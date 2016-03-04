// testSCARA.cpp :
//

#include "mrcore.h"
//#include "robotsim.h"
#include "puma560sim.h"
//#include "adeptonesim.h"
#include "tool.h"

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

WheeledBaseSim *myrobot;
AdeptOneSim *scara;
Puma560Sim *puma;
Tool *worktool;

int num=0;

int main(int argc, char* argv[])
{
	//GL Initialization stuff
	glutInit(&argc, argv);
	glutInitWindowSize(800,600);
	glutInitWindowPosition(400,100);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutCreateWindow("TestSCARA");

/***
Se cambia el color de fondo de la escena
***/
	glClearColor(0.5,0.5,0.5,1);

	glutDisplayFunc(OnDraw);
	glutMotionFunc(OnMouseMove);
	glutMouseFunc(OnMouseClick);
	glutKeyboardFunc(OnKeyboardDown);
	glutTimerFunc(100,OnTimer,0);
	scene.init();

	Face suelo(Transformation3D(0,0,0),0,-10,10,10);
	FaceSetPart *building=new FaceSetPart; 
	building->addFace(suelo);

	world+=building;

//Robot Pioneer
/*	myrobot=new Pioneer3ATSim;
	myrobot->setRelativePosition(Vector3D(2,8,0));
	myrobot->setName("Neo");
	world+=myrobot;*/

//Scara
	scara=new AdeptOneSim();
	//scara->setRelativePosition(Vector3D(0,0,0.3));//Con NEO
	//scara->setRelativePosition(Vector3D(0,0,0));
	scara->setRelativePosition(Vector3D(5,5,0));//....ABS
	//scara->setRelativeOrientation(Z_AXIS,PI/2);
	scara->setName("SCARA");
	//scara->LinkTo(myrobot);
	world+=scara;

//Puma560
	puma=new Puma560Sim();
	puma->setRelativePosition(Vector3D(5,-5,0));
	world+=puma;

//Herramienta de trabajo
	worktool=new Tool;
	worktool->LinkTo(puma->getTcp());
	world+=worktool;

//Add object to scene
	scene.addObject(&world);
	
	glutMainLoop();
	world.destroyContent();
	return 0;
}

void OnDraw(void)
{ 
	scene.Draw();
	glutSwapBuffers();
}
void OnTimer(int value)
{
//	myrobot->simulate(0.1);
	puma->setFlash();

	glutTimerFunc(100,OnTimer,0); //set again the timer
	glutPostRedisplay();
}
void OnKeyboardDown(unsigned char key, int x, int y)
{
	if(key==27)exit(0);

	if(key=='0')num=0;
	if(key=='1')num=1;
	if(key=='2')num=2;
	if(key=='3')num=3;

	double val;
	scara->getJointValue(num,val);
	if(key=='+')val+=0.05;
	if(key=='-')val-=0.05;
	scara->setJointValue(num,val);

	if(key=='i')
	{
		vector<double> q;
		q.push_back(0.261749);
		q.push_back(-0.436332);
		q.push_back(-0.25);
		q.push_back(-1.57079);
	
		if(scara->moveTo(q))
			cout<<"OK moveTo(q)"<<endl;
		else
			cout<<"No ok moveTo(q)"<<endl;
	}

	if(key=='m')
	{
		Transformation3D t;
		t.position=Vector3D(0.780,0.045,0.720);
		t.orientation.setRPY(0,0,-1.57079);
		if(scara->moveTo(t))
			cout<<"OK moveTo(t3d)"<<endl;
		else
			cout<<"NO ok moveTo(t3d)"<<endl;
	}

	if(key=='j')
	{
		Transformation3D t;
		t.position=Vector3D(5.780,5.045,0.720);
		t.orientation.setRPY(0,0,-1.57079);
		if(scara->moveToAbs(t))
			cout<<"OK moveToAbs(t3d)"<<endl;
		else
			cout<<"NO ok moveToAbs(t3d)"<<endl;
	}

	if(key=='a')
	{
		Vector3D pos=Vector3D(0.780,0.045,0.720);
		double yaw=0.4957;
		vector<double> q;
	/*	if(scara->ADEPTONEinverseKinematics(yaw,pos,q,0x80))
		{
			cout<<"OK SCARAinerseKinematics()"<<endl;
			cout<<q[0]<<"\t"<<q[1]<<"\t"<<q[2]<<"\t"<<q[3]<<endl;
		}
		else
			cout<<"NO ok SCARAinerseKinematics()"<<endl;*/
	}
	
	if(key=='z')
	{
		vector<double> q;
		Transformation3D t;
		t.position=Vector3D(5.780,5.045,0.720);
		t.orientation.setRPY(0,0,0.4957);

		if(scara->inverseKinematicsAbs(t,q,0x81))
		{
			cout<<"OK inverseKinematicsABS()"<<endl;
			cout<<q[0]<<"\t"<<q[1]<<"\t"<<q[2]<<"\t"<<q[3]<<endl;
		}
		else
			cout<<"NO ok inverseKinematicsABS()"<<endl;
	}
//relative direct and inverse kinematics cheq

	if(key=='8'){
		vector<double> q;
		Transformation3D t=scara->getTcpAbsLocation();
		t.position.x+=0.01;
		scara->moveToAbs(t);
	}
	if(key=='9'){
		vector<double> q;
		Transformation3D t=scara->getTcpAbsLocation();
		t.position.x-=0.01;
		scara->moveToAbs(t);
	}
	if(key=='6'){
		vector<double> q;
		Transformation3D t=scara->getTcpLocation();
		t.position.y+=0.01;
		scara->moveTo(t);
	}
	if(key=='7'){
		vector<double> q;
		Transformation3D t=scara->getTcpLocation();
		t.position.y-=0.01;
		scara->moveTo(t);
	}
	if(key=='4'){
		vector<double> q;
		Transformation3D t=scara->getTcpLocation();
		t.position.z+=0.01;
		scara->moveTo(t);
	}
	if(key=='5'){
		vector<double> q;
		Transformation3D t=scara->getTcpLocation();
		t.position.z-=0.01;
		scara->moveTo(t);
	}
	if(key=='s')
	{
		vector<double> q;
		Transformation3D t;
		t.position=Vector3D(0.780,0.045,0.720);
		t.orientation.setRPY(0,0,0.4957);

		if(scara->inverseKinematics(t,q,0x81))
		{
			cout<<"OK inverseKinematics()"<<endl;
			cout<<q[0]<<"\t"<<q[1]<<"\t"<<q[2]<<"\t"<<q[3]<<endl;
		}
		else
			cout<<"NO ok inverseKinematics()"<<endl;
	}

	if(key=='n')
	{
		vector<double> q;
		q.push_back(0.261749);
		q.push_back(-0.436332);
		q.push_back(-0.25);
		q.push_back(-1.57079);
		Transformation3D t;
		unsigned char c;
		if(scara->forwardKinematics(q,t))
		{
			cout<<"OK forwardKinematics()"<<endl;
			cout<<t.position<<endl;
			cout<<c<<endl;
		}
		else
			cout<<"NO ok forwardKinematics()"<<endl;
	}

	static double speed=0;
	static double rotspeed=0;
	if(key=='t')speed+=0.1;
	if(key=='b')speed-=0.1;
	if(key=='h')rotspeed-=0.2;
	if(key=='f')rotspeed+=0.2;
//	myrobot->move(speed,rotspeed);

	glutPostRedisplay();

}
void OnMouseClick(int b,int state, int x,int y)
{
	bool down=(state==GLUT_DOWN);
	int button;
	if(b==GLUT_LEFT_BUTTON)
		button=MOUSE_LEFT_BUTTON;
	if(b==GLUT_RIGHT_BUTTON)
		button=MOUSE_RIGHT_BUTTON;
		
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