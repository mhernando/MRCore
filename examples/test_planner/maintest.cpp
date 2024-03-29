// PruebaMRcore.cpp : Defines the entry point for the console application.
//


#include <mrcore.h>

#include <freeglut.h>

#include <iostream>


using namespace mr;
using namespace std;
void createEnvironment(); //edificio
void createEnvironment2(); //experimentto rampa
void createEnvironment3(); //experimento variable
void loadEnvironment(int i);
void setStartGoal(int i);

void OnDraw(void);
void OnTimer(int value); //esta funcion sera llamada cuando transcurra una temporizacion
void OnKeyboardDown(unsigned char key, int x, int y); //cuando se pulse una tecla	
void OnMouseMove(int x,int y);
void OnMouseClick(int button,int state, int x,int y);
//MeshPart *createMan(void);
MeshPart *createWoodBench(void);
//MeshPart *createPionerAT(void);
//The scene global object
GLScene scene;
World world;

WheeledBaseSim *myrobot;

Sampler *samplerU,*samplerR, *samplerG,*sampler;
BoundingBox box,box1,box2;
vector<Sample> sampleSet;
PathPlanner *planner;
RobotState *auxrs=0;
RobotPath solution;

PositionableEntity *objeto1=0,*objeto2=0;

ofstream *file;
//int automatic <0 inactivo,  >=0 fase automatica
int automatic=-2;


//bool animation... solucion cutre para ver un poco como evoluciona
bool ANIMATION=false;
int iteracion=0;
//***************variables globales para organizar un poco el sistema
struct seleccion
{
	int samplerSel;
	int environmentSel;
	int numPlansSel;
	int startGoalSel;
	int times[2002];
	int numNodes[2002];
	
	char fileName[100];
}selection;



int main(int argc, char* argv[])
{
	//GL Initialization stuff
	glutInit(&argc, argv);
	glutInitWindowSize(800,600);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutCreateWindow("GL");
	glutDisplayFunc(OnDraw);
	glutMotionFunc(OnMouseMove);
	glutMouseFunc(OnMouseClick);
	glutKeyboardFunc(OnKeyboardDown);
	glutTimerFunc(100,OnTimer,0);
	scene.init();

	selection.environmentSel=0;
	selection.numPlansSel=100;
	selection.samplerSel=0;
	selection.startGoalSel=0;
	strcpy(selection.fileName,"outputData.txt");

//Intializing test environment Faces included in a FacePart

	samplerR=new RandomSampler(&world);
	samplerG=new GaussianSampler(&world);
	samplerU=new UniformSampler(&world);	
	
	loadEnvironment(3);

	planner=new RDTplanner;


	sampler=samplerR;

	setStartGoal(1);




//comparative 3D sampling 

/*samplerR=new RandomSampler(Vector3D(0,0,0), Vector3D(10,10,10));
samplerU=new UniformSampler(Vector3D(12,0,0), Vector3D(10,10,10));
samplerG=new GaussianSampler(Vector3D(24,0,0), Vector3D(10,10,10));

box=BoundingBox(Vector3D(0,0,0), Vector3D(10,10,10));
box1=BoundingBox(Vector3D(12,0,0), Vector3D(22,10,10));
box2=BoundingBox(Vector3D(24,0,0), Vector3D(34,10,10));

*/
//comparative 2D sampling 
/*
samplerR=new RandomSampler(Vector2D(0,0), Vector2D(10,10));
samplerU=new UniformSampler(Vector2D(12,0), Vector2D(10,10));
samplerG=new GaussianSampler(Vector2D(24,0), Vector2D(10,10));

box=BoundingBox(Vector3D(0,0,0), Vector3D(10,10,0.1));
box1=BoundingBox(Vector3D(12,0,0), Vector3D(22,10,0.10));
box2=BoundingBox(Vector3D(24,0,0), Vector3D(34,10,0.10));
*/
//3d sampling

	scene.addObject(&world);	
	
	glutMainLoop();
	world.destroyContent();
	return 0;
}

void OnDraw(void)
{  
/*scene.setShowGrid(false);
scene.setShowFrame(false);
GLTools::BackgroundColor(WHITE);
*/


	scene.Draw();
	if(planner)planner->drawGL();
	if(auxrs)auxrs->drawGL();
	solution.drawGL();

/*	char mensaje[200];
	sprintf_s(mensaje,200,"Number of Samples: %d\n",sampleSet.size()/3);
	glColor3f(0,0,0);
	GLTools::Print(mensaje,1,1,0);
	cout<<mensaje;
GLTools::DrawGrid(50,2,BLUE);
//local drawing
	glDisable(GL_LIGHTING);
	glPointSize(2);
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

*/
	glutSwapBuffers();
}
void variaEntorno()
{
	if(objeto1==0)return;
	if(objeto2==0)return;
//ENTORNO VARIABLE EN BASE A OBJETO1 y OBJETO2
		double z=-1.0+(rand()*1.0/RAND_MAX);
		double rot1=PI+0.5*PI*(rand()*1.0/RAND_MAX);
		double rot2=-PI+0.5*PI*(rand()*1.0/RAND_MAX);
		double y1=1.5+(rand()*0.5/RAND_MAX);
		double y2=-1.5-(rand()*0.5/RAND_MAX);
		double x1=2*((rand()*1.0/RAND_MAX)-0.5);
		double x2=2*((rand()*1.0/RAND_MAX)-0.5);
		objeto1->setRelativePosition(Vector3D(x1,y1,z));
		objeto2->setRelativePosition(Vector3D(x2,y2,z));
		objeto1->setRelativeOrientation(Z_AXIS,rot1);
		objeto2->setRelativeOrientation(Z_AXIS,rot2);

}
void computeMedDesv(int *v,int num, double &med, double &desv)
{
int medio=0,stddev=0,i;
for(i=0;i<num;i++)medio+=v[i];
med=((double)medio)/num;

for(i=0;i<num;i++)stddev+=((med-v[i])*(med-v[i]));
desv=sqrt(((double)stddev)/num);
}

void OnTimer(int value)
{
RDTplanner *paux=(RDTplanner *)planner;
	if(ANIMATION){// a mitad de una planificacion manual
		cout<<"trying nodes: "<<100*(iteracion++)<<endl;
		if(planner->computePlan(100)){
			solution.path=(planner->getPlan())->path;
			ANIMATION=false;	
			glutPostRedisplay();
			return;
		}
		
		glutTimerFunc(100,OnTimer,0); //set again the timer
		glutPostRedisplay();
		return;
	}
	if(automatic>=0)
	{

		//primero verifica que es alcanzable
		setStartGoal(selection.startGoalSel);
		if(planner->isValid()==false){
			automatic=-1;
			cout<<std::endl<<"ERROR: invalid Start or Goal states"<<std::endl;
			glutTimerFunc(100,OnTimer,0); //set again the timer
			return;
		}

		if(automatic==0){paux->setSampler(samplerU);selection.samplerSel=3;}
		else if(automatic<=selection.numPlansSel){paux->setSampler(samplerR);selection.samplerSel=1;}
		else {paux->setSampler(samplerG);selection.samplerSel=2;}

		solution.path.clear();
		
		MRTime tempo;
		tempo.tic();
		//ejecucion intento de 30000 nodos como maximo
		bool solved=planner->computePlan(30000);
		long t=tempo.toc();
		if((automatic==0)&&(solved==false)){
			cout<<std::endl<<"ERROR: NO ALCANZABLE EN 30000 nodos"<<std::endl;
			automatic=-1;
			glutTimerFunc(100,OnTimer,0); //set again the timer
			return;
		}

		if(solved)solution.path=(planner->getPlan())->path;

		//save data:

		if(automatic==0){
			file=new ofstream(selection.fileName);
			*file<<"ENVIRONMENT:"<<selection.environmentSel<<std::endl;
			*file<<"START & GOAL:"<<selection.startGoalSel<<std::endl;
			*file<<"SOLVED\tID\tTIME\tNODES"<<std::endl;
		}
		cout<<solved<<'\t'<<selection.samplerSel<<'\t'<<t<<'\t'<<((RDTplanner *)planner)->getNumNodes()<<std::endl;
		(*file)<<solved<<'\t'<<selection.samplerSel<<'\t'<<t<<'\t'<<((RDTplanner *)planner)->getNumNodes()<<std::endl;
		selection.numNodes[automatic]=((RDTplanner *)planner)->getNumNodes();
		selection.times[automatic]=(double)t;
		automatic++;
		

		//condicion de finalizaci�n
		if(automatic>2*(selection.numPlansSel)){
			*file<<"____________________\tT.med \t T.desv \t Nodes.med \t Nodes.desv"<<std::endl;
			*file<<"UNIFORM SAMPLING: "<<selection.times[0]<<"\t  0.0\t"<<selection.numNodes[0]<<"\t 0.0"<<std::endl;
			double med,stddesv;
			computeMedDesv((selection.times)+1,selection.numPlansSel,med,stddesv);
			*file<<"RANDOM SAMPLING: "<<med<<"\t"<<stddesv;
			computeMedDesv((selection.numNodes)+1,selection.numPlansSel,med,stddesv);
			*file<<"\t"<<med<<"\t"<<stddesv<<std::endl;;
			computeMedDesv((selection.times)+(selection.numPlansSel+1),selection.numPlansSel,med,stddesv);
			*file<<"GAUSSIAN SAMPLING: "<<med<<"\t"<<stddesv;
			computeMedDesv((selection.numNodes)+(selection.numPlansSel+1),selection.numPlansSel,med,stddesv);
			*file<<"\t"<<med<<"\t"<<stddesv;
			automatic=-1;
			

			//compute statistics
			file->close();
			delete file;
			}


	}
	else
	{
	myrobot->simulate(0.1);
	}
	glutTimerFunc(100,OnTimer,0); //set again the timer
	glutPostRedisplay();	
}
void OnKeyboardDown(unsigned char key, int x, int y)
{
	static double angle=0;

	static int i=0;

	static double speed=0;
	static double rotspeed=0;
	static double altura=0.1;


	static int menu=0;

	if(key=='m')
	{
		cout<<"\nMENU"<<endl;
		cout<<"1- Samplig mode"<<endl;
		cout<<"2- Environment"<<endl;
		cout<<"3- Goal-Start Positions"<<endl;
		cout<<"4- Automatic planning"<<endl;
		cout<<"0-Exit menu"<<endl;
	
		char menu;
		cin>>key;
		if(key=='1'){
			cout<<"1-Random Sampling"<<endl;
			cout<<"2-Gaussian Sampling"<<endl;
			cout<<"3-Uniformed sampling"<<endl;
			cin>>menu;
			switch(menu){
				case '1': sampler=samplerR;selection.samplerSel=1;break;
				case '2': sampler=samplerG;selection.samplerSel=2;break;
				case '3': sampler=samplerU;selection.samplerSel=3;break;
			}
			return;
		}else if(key=='2'){
			cout<<"1-Rooms"<<endl;
			cout<<"2-Building"<<endl;
			cout<<"3-Random environment"<<endl;
			cin>>menu;
			switch(menu){
				case '1': selection.environmentSel=1;break;
				case '2': selection.environmentSel=2;break;
				case '3': selection.environmentSel=3;break;
				default: selection.environmentSel=1;
			}
			loadEnvironment(selection.environmentSel);
			setStartGoal(selection.startGoalSel);
			
			return;
		}else if(key=='3'){
			cout<<"1-Rooms 1"<<endl;
			cout<<"2-Rooms 2"<<endl;
			cout<<"3-Building"<<endl;
			cout<<"4-Random environment"<<endl;
			cin>>menu;
			switch(menu){
				case '1': selection.startGoalSel=1;break;
				case '2': selection.startGoalSel=2;break;
				case '3': selection.startGoalSel=3;break;
				case '4': selection.startGoalSel=4;break;
				default: selection.startGoalSel=1;
			}
			setStartGoal(selection.startGoalSel);
			return;
		}else if(key=='4'){
			cout<<"Introduzca el numero de experimentos: ";
			cin>>selection.numPlansSel;
			if(selection.numPlansSel<0)selection.numPlansSel=1;
			if(selection.numPlansSel>1000)selection.numPlansSel=1000;
			return;
		}else return;
		
	}





	if(key=='A')
	{
		automatic = 0;
		//ejecucion autom�tica de test con medidas de tiempo, etc
		return;
	}
	if(automatic>=0)return; //mientras hay ejecucion autom�tica... no se puede hacer nada
	
	if(key=='v') variaEntorno();
	if(key=='i')srand( (unsigned)time( NULL ) );
	if(key=='t')speed+=0.1;
	if(key=='b')speed-=0.1;
	if(key=='h')rotspeed-=0.2;
	if(key=='f')rotspeed+=0.2;
	if(key=='+') {

		if(planner){
				//replanifico
				setStartGoal(selection.startGoalSel);
				ANIMATION=true;
				iteracion=0;
				glutTimerFunc(100,OnTimer,0); //set again the timer
		
			
		}
		/*WBState gen(myrobot,&world);
		delete auxrs;
		auxrs=gen.createStateFromSample(sampler->getNextSample());
		cout<<i++<<std::endl;*/
	//	sampleSet.push_back(samplerR->getNextSample());
	//	sampleSet.push_back(samplerU->getNextSample());
	//	sampleSet.push_back(samplerG->getNextSample());
	}

	if(key=='F')
	{
		cout<<"FILTRADO: "<<solution.path.size()<<"-->";
		solution.filter();
		cout<<solution.path.size()<<endl;
	}
	if(key=='u') sampler=samplerU;
	
	if(key=='g') sampler=samplerG;

	if(key=='r') sampler=samplerR;



	myrobot->move(speed,rotspeed);
	

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
void setStartGoal(int i)
{
if((i<1)||(i>4))return;

WBState gen(myrobot,&world);
WBState *start, *goal;
switch(i){
	case 1:
		start=gen.createStateFromPoint3D(2.0,-8,0);
		goal=gen.createStateFromPoint3D(8.0,1.5,2);
		break;
	case 2:
		start=gen.createStateFromPoint3D(2.0,-8,0);
		goal=gen.createStateFromPoint3D(2.0,1.5,0);
		break;
	case 3:
		start=gen.createStateFromPoint3D(1.0,-2.5,0);
		goal=gen.createStateFromPoint3D(2.0,1.5,4);
		break;
	case 4:
		start=gen.createStateFromPoint3D(0,2.5,2);
		goal=gen.createStateFromPoint3D(0,-2.5,0);
		break;

}
planner->setStartAndGoalStates(start, goal);
if(dynamic_cast<SBPathPlanner *>(planner))
	(dynamic_cast<SBPathPlanner *>(planner))->setSampler(sampler);
solution.path.clear();
delete start;
delete goal;
}
void loadEnvironment(int i)
{
	if((i<1)||(i>3))return;
	world.destroyContent();
	objeto1=objeto2=0;
  //borro el que halla
   if(i==1)createEnvironment();
   else if(i==2) createEnvironment2();
   else if(i==3) createEnvironment3();

   //probando el robot
	myrobot=new Pioneer3ATSim;
	myrobot->setRelativePosition(Vector3D(2.0,8,0));
	world+=myrobot;

samplerR->setWorld(&world);
samplerG->setWorld(&world);
samplerU->setWorld(&world);

}
void createEnvironment2()
{
	//Intializing test environment Faces included in a FacePart
	Face suelo(Transformation3D(0,0,0),0,-4,7,4);


	FaceSetPart *building=new FaceSetPart; 
	building->addFace(suelo);

	



//nuevo entorno
	Face sueloroom(Transformation3D(0,1,0),0,0,6,3);
	Face paredfront(Transformation3D(6,1,0,Y_AXIS,-PI/2),0,0,2,3);
	Face paredback(Transformation3D(0,1,0,Y_AXIS,-PI/2),0,0,2,3);
	Face paredcerca(Transformation3D(0,4,0,X_AXIS,PI/2),0,0,6,2);
	Face paredpuerta;

	paredpuerta.setBase(Transformation3D(0,1,0,X_AXIS,PI/2));
	paredpuerta.addVertex(0,0);
	paredpuerta.addVertex(0,2);
	paredpuerta.addVertex(6,2);
	paredpuerta.addVertex(6,0);
	paredpuerta.addVertex(5.60,0);
	paredpuerta.addVertex(5.60,1.75);
	paredpuerta.addVertex(4.40,1.75);
	paredpuerta.addVertex(4.40,0);
	

	FaceSetPart *room=new FaceSetPart;
	room->addFace(sueloroom);
	room->addFace(paredfront);
	room->addFace(paredback);
	room->addFace(paredcerca);
	room->addFace(paredpuerta);

	FaceSetPart *roomi=new FaceSetPart;
	Vector3D vo1(0,1,0),vo2(0,4,0);
	paredcerca.setOrigin(vo1);
	paredpuerta.setOrigin(vo2);
	roomi->addFace(sueloroom);
	roomi->addFace(paredfront);
	roomi->addFace(paredback);
	roomi->addFace(paredcerca);
	roomi->addFace(paredpuerta);
	roomi->setRelativePosition(Vector3D(0,-5,0));

	ComposedEntity *ramp=new ComposedEntity;
	Face rampPoligon;
	PrismaticPart *ramp_=new PrismaticPart;
	rampPoligon.addVertex(0,-0.2);
	rampPoligon.addVertex(0,0);
	rampPoligon.addVertex(3,1);
	rampPoligon.addVertex(3,0.8);
	ramp_->setHeight(1);
	ramp_->setPolygonalBase(rampPoligon);
	ramp->addObject(ramp_);
	Face plat_(Transformation3D(0,0,0),0,-0.2,1,0);
	PrismaticPart *plat=new PrismaticPart;
	plat->setHeight(2);
	plat->setPolygonalBase(plat_);
	plat->setRelativePosition(Vector3D(3,1,0));
	ramp->addObject(plat);

	Face plat2_(Transformation3D(0,0,0),0,-0.2,2,0);
	PrismaticPart *plat2=new PrismaticPart;
	plat2->setHeight(2);
	plat2->setPolygonalBase(plat2_);
	plat2->setRelativePosition(Vector3D(-2,0,0));
	ramp->addObject(plat2);

	PrismaticPart *ramp2_=new PrismaticPart(*ramp_);
	ramp2_->setRelativeT3D(Transformation3D(3,1,2,0,PI,0));
	ramp->addObject(ramp2_);
	ramp->setRelativeOrientation(PI/2,0,PI);
	ramp->setRelativePosition(Vector3D(4,-1,0));

	const int num=4;
	int i;
	ComposedEntity *stages[num];
	stages[0]=new ComposedEntity;
	stages[0]->addObject(room);
	stages[0]->addObject(roomi);
	stages[0]->addObject(ramp);

	for(i=1;i<num;i++){
		stages[i]=new ComposedEntity(*(stages[0]));
		stages[i]->setRelativePosition(Vector3D(0,0,2*i));
	}
	
for(i=0;i<num;i++)
	world+=stages[i];







	

	world+=building;

}
void createEnvironment()
{
	/*Face rampPoligon;
	PrismaticPart ramp;
	rampPoligon.addVertex(0,-0.2);
	rampPoligon.addVertex(0,0);
	rampPoligon.addVertex(3,1);
	rampPoligon.addVertex(3,2.8);
	ramp.setHeight(1);
	ramp.setPolygonalBase(rampPoligon);*/


	//Intializing test environment Faces included in a FacePart
	Face suelo(Transformation3D(0,0,0),0,-10,10,10);
	Face tablon_fino1(Transformation3D(8,3,2,X_AXIS,-0.53),0,0,0.2,3.95);
	Face tablon_fino2(Transformation3D(8.5,3,2,X_AXIS,-0.53),0,0,0.2,3.95);
	Face tablon_grueso(Transformation3D(2,3,2,X_AXIS,-0.53),0,0,1.4,3.95);
	Face plataforma(Transformation3D(2,0,2),0,0,8,3);
	Face paredfondo1(Transformation3D(0,0,0,Y_AXIS,PI/2),-4,-10,0,10);
	Face paredfondo2;

	paredfondo2.setBase(Transformation3D(0,0,0,X_AXIS,-PI/2));
	paredfondo2.addVertex(0,-4);
	paredfondo2.addVertex(10,-4);
	paredfondo2.addVertex(10,0);
	paredfondo2.addVertex(6,0);
	paredfondo2.addVertex(6,-1.5);
	paredfondo2.addVertex(4,-1.5);
	paredfondo2.addVertex(4,0);
	paredfondo2.addVertex(0,0);

	FaceSetPart *building=new FaceSetPart; 
	building->addFace(suelo);
	building->addFace(tablon_fino1);
	building->addFace(tablon_fino2);
	building->addFace(tablon_grueso);
	building->addFace(plataforma);
	building->addFace(paredfondo1);
	building->addFace(paredfondo2);
	
	world+=building;

}
//entorno con objetos variables de posicion
//para poder comparar
void createEnvironment3()
{
	/*Face rampPoligon;
	PrismaticPart ramp;
	rampPoligon.addVertex(0,-0.2);
	rampPoligon.addVertex(0,0);
	rampPoligon.addVertex(3,1);
	rampPoligon.addVertex(3,2.8);
	ramp.setHeight(1);
	ramp.setPolygonalBase(rampPoligon);*/


	//Intializing test environment Faces included in a FacePart
	Face suelo(Transformation3D(0,0,0),-7,-7,7,7);
FaceSetPart *ssuelo=new FaceSetPart();
ssuelo->addFace(suelo);
	Face tablon_grueso(Transformation3D(0,1.5,2,X_AXIS,-0.53),0,0,1.4,3.95);
	Face plataforma(Transformation3D(0,0,2),-2.5,-1.5,2.5,1.5);
	FaceSetPart *pieza1=new FaceSetPart; 

	pieza1->addFace(tablon_grueso);
	pieza1->addFace(plataforma);

	FaceSetPart *pieza2=new FaceSetPart(*pieza1);
	pieza2->setRelativePosition(Vector3D(0,1.5,0));
	pieza1->setRelativePosition(Vector3D(0,-1.5,0));
	pieza1->setRelativeOrientation(Z_AXIS,PI);
	world+=ssuelo;
	world+=pieza1;
	world+=pieza2;

	objeto1=pieza1;
	objeto2=pieza2;

}
//Automatic code created by MeshPart, in order to code in mrcode
//a mesh so it is posible to create it easily by code
//It is recomended to change the name of the created generic function
//just call: MeshPart *aux=thisfunction(); in your code

MeshPart *createRamp(void)
{
const int num=662;
const double v[][3]={
{0.490631,0.361148,0.0971143},{0.488619,0.357511,0.100092},{0.490176,0.360326,0.0966224},
{0.488619,0.357511,0.100092},{0.490631,0.361148,0.0971143},{0.495047,0.36913,0.101891},
{0.503751,0.38486,0.101891},{0.495047,0.36913,0.101891},{0.502837,0.383208,0.0976362},
{0.495047,0.36913,0.101891},{0.503751,0.38486,0.101891},{0.498732,0.37579,0.12126},
{0.451677,0.290744,0.0776394},{0.425985,0.24431,0.0669358},{0.440287,0.270159,0.0669358},
{0.425985,0.24431,0.0669358},{0.451677,0.290744,0.0776394},{0.41528,0.224964,0.0669358},
{0.41528,0.224964,0.0669358},{0.451677,0.290744,0.0776394},{0.413481,0.221713,0.0669358},
{0.413481,0.221713,0.0669358},{0.451677,0.290744,0.0776394},{0.411682,0.218461,0.0669358},
{0.411682,0.218461,0.0669358},{0.451677,0.290744,0.0776394},{0.412007,0.219049,0.0985213},
{0.412007,0.219049,0.0985213},{0.451677,0.290744,0.0776394},{0.465316,0.315395,0.0904569},
{0.412007,0.219049,0.0985213},{0.465316,0.315395,0.0904569},{0.488619,0.357511,0.100092},
{0.412007,0.219049,0.0985213},{0.488619,0.357511,0.100092},{0.407782,0.211411,0.130779},
{0.407782,0.211411,0.130779},{0.488619,0.357511,0.100092},{0.495047,0.36913,0.101891},
{0.407782,0.211411,0.130779},{0.495047,0.36913,0.101891},{0.498732,0.37579,0.12126},
{0.407782,0.211411,0.130779},{0.498732,0.37579,0.12126},{0.49209,0.363785,0.135207},
{0.407782,0.211411,0.130779},{0.49209,0.363785,0.135207},{0.418508,0.230798,0.149596},
{0.418508,0.230798,0.149596},{0.49209,0.363785,0.135207},{0.491882,0.36341,0.142502},
{0.418508,0.230798,0.149596},{0.491882,0.36341,0.142502},{0.495777,0.370449,0.154007},
{0.418508,0.230798,0.149596},{0.495777,0.370449,0.154007},{0.416883,0.227861,0.222847},
{0.416883,0.227861,0.222847},{0.495777,0.370449,0.154007},{0.490845,0.361534,0.179294},
{0.416883,0.227861,0.222847},{0.490845,0.361534,0.179294},{0.495756,0.370411,0.18983},
{0.416883,0.227861,0.222847},{0.495756,0.370411,0.18983},{0.502288,0.382217,0.237101},
{0.416883,0.227861,0.222847},{0.502288,0.382217,0.237101},{0.425383,0.243224,0.399935},
{0.425383,0.243224,0.399935},{0.502288,0.382217,0.237101},{0.507388,0.391434,0.349951},
{0.425383,0.243224,0.399935},{0.507388,0.391434,0.349951},{0.503472,0.384356,0.369825},
{0.425383,0.243224,0.399935},{0.503472,0.384356,0.369825},{0.505964,0.38886,0.422821},
{0.425383,0.243224,0.399935},{0.505964,0.38886,0.422821},{0.430663,0.252766,0.441329},
{0.430663,0.252766,0.441329},{0.505964,0.38886,0.422821},{0.514487,0.404265,0.437748},
{0.430663,0.252766,0.441329},{0.514487,0.404265,0.437748},{0.522593,0.418915,0.584883},
{0.430663,0.252766,0.441329},{0.522593,0.418915,0.584883},{0.42742,0.246904,0.491319},
{0.42742,0.246904,0.491319},{0.522593,0.418915,0.584883},{0.428599,0.249035,0.636411},
{0.428599,0.249035,0.636411},{0.522593,0.418915,0.584883},{0.533542,0.438703,0.721986},
{0.428599,0.249035,0.636411},{0.533542,0.438703,0.721986},{0.416373,0.226939,0.756906},
{0.428599,0.249035,0.636411},{0.416373,0.226939,0.756906},{0.414387,0.22335,0.75759},
{0.536595,0.444221,0.849498},{0.416373,0.226939,0.756906},{0.533542,0.438703,0.721986},
{0.416373,0.226939,0.756906},{0.536595,0.444221,0.849498},{0.418111,0.23008,0.759301},
{0.418111,0.23008,0.759301},{0.536595,0.444221,0.849498},{0.419021,0.231725,0.770935},
{0.419021,0.231725,0.770935},{0.536595,0.444221,0.849498},{0.433391,0.257696,0.805787},
{0.419021,0.231725,0.770935},{0.433391,0.257696,0.805787},{0.416357,0.226911,0.801436},
{0.416357,0.226911,0.801436},{0.433391,0.257696,0.805787},{0.431023,0.253417,0.805379},
{0.433391,0.257696,0.805787},{0.536595,0.444221,0.849498},{0.435232,0.261024,0.808506},
{0.435232,0.261024,0.808506},{0.536595,0.444221,0.849498},{0.435495,0.261499,0.811225},
{0.435495,0.261499,0.811225},{0.536595,0.444221,0.849498},{0.432338,0.255794,0.823463},
{0.432338,0.255794,0.823463},{0.536595,0.444221,0.849498},{0.430629,0.252704,0.836788},
{0.430629,0.252704,0.836788},{0.536595,0.444221,0.849498},{0.426946,0.246047,0.855551},
{0.426946,0.246047,0.855551},{0.536595,0.444221,0.849498},{0.534473,0.440385,0.902547},
{0.426946,0.246047,0.855551},{0.534473,0.440385,0.902547},{0.425324,0.243116,0.869808},
{0.425324,0.243116,0.869808},{0.534473,0.440385,0.902547},{0.522984,0.419621,0.900047},
{0.425324,0.243116,0.869808},{0.522984,0.419621,0.900047},{0.418617,0.230995,0.897276},
{0.522984,0.419621,0.900047},{0.41343,0.221619,0.906694},{0.418617,0.230995,0.897276},
{0.41343,0.221619,0.906694},{0.522984,0.419621,0.900047},{0.467872,0.320016,0.941123},
{0.41343,0.221619,0.906694},{0.467872,0.320016,0.941123},{0.409306,0.214166,0.920764},
{0.409306,0.214166,0.920764},{0.467872,0.320016,0.941123},{0.446968,0.282234,0.943266},
{0.416357,0.226911,0.801436},{0.426025,0.244383,0.812041},{0.416357,0.226911,0.839235},
{0.426025,0.244383,0.812041},{0.416357,0.226911,0.801436},{0.428656,0.249138,0.807554},
{0.428656,0.249138,0.807554},{0.416357,0.226911,0.801436},{0.431023,0.253417,0.805379},
{0.416357,0.226911,0.839235},{0.426025,0.244383,0.812041},{0.421882,0.236895,0.82523},
{0.416357,0.226911,0.839235},{0.421882,0.236895,0.82523},{0.418725,0.23119,0.843042},
{0.416357,0.226911,0.839235},{0.418725,0.23119,0.843042},{0.418121,0.230097,0.843816},
{0.511976,0.399726,1.43735},{0.518928,0.41229,1.40596},{0.521431,0.416815,1.40297},
{0.518928,0.41229,1.40596},{0.511976,0.399726,1.43735},{0.507311,0.391295,1.43829},
{0.507311,0.391295,1.43829},{0.511976,0.399726,1.43735},{0.515178,0.405512,1.46571},
{0.451817,0.290998,0.024232},{0.439165,0.268132,0.0177566},{0.457139,0.300617,0.00827749},
{0.439165,0.268132,0.0177566},{0.451817,0.290998,0.024232},{0.418997,0.231681,0.0248036},
{0.418997,0.231681,0.0248036},{0.451817,0.290998,0.024232},{0.443072,0.275193,0.0367642},
{0.418997,0.231681,0.0248036},{0.443072,0.275193,0.0367642},{0.413062,0.220955,0.0334404},
{0.413062,0.220955,0.0334404},{0.443072,0.275193,0.0367642},{0.412644,0.2202,0.0380369},
{0.412644,0.2202,0.0380369},{0.443072,0.275193,0.0367642},{0.423316,0.239487,0.0534973},
{0.412644,0.2202,0.0380369},{0.423316,0.239487,0.0534973},{0.41267,0.220246,0.0512004},
{0.41267,0.220246,0.0512004},{0.423316,0.239487,0.0534973},{0.413481,0.221713,0.0669358},
{0.413481,0.221713,0.0669358},{0.423316,0.239487,0.0534973},{0.41528,0.224964,0.0669358},
{0.522687,0.419084,1.50822},{0.515408,0.405929,1.50281},{0.519218,0.412815,1.50281},
{0.515408,0.405929,1.50281},{0.522687,0.419084,1.50822},{0.510745,0.397501,1.5061},
{0.510745,0.397501,1.5061},{0.522687,0.419084,1.50822},{0.506047,0.389011,1.50881},
{0.506047,0.389011,1.50881},{0.522687,0.419084,1.50822},{0.523711,0.420934,1.51504},
{0.536445,0.443949,1.57092},{0.532051,0.436008,1.56893},{0.532702,0.437184,1.56883},
{0.532051,0.436008,1.56893},{0.536445,0.443949,1.57092},{0.529716,0.431789,1.56928},
{0.538355,0.447401,1.64132},{0.536015,0.443173,1.63488},{0.537052,0.445046,1.62969},
{0.536015,0.443173,1.6424},{0.534326,0.44012,1.64025},{0.536015,0.443173,1.63488},
{0.52354,0.420626,1.651},{0.512734,0.401096,1.64121},{0.521071,0.416164,1.64025},
{0.512734,0.401096,1.64121},{0.52354,0.420626,1.651},{0.518212,0.410997,1.65852},
{0.518212,0.410997,1.65852},{0.52354,0.420626,1.651},{0.52367,0.420861,1.6647},
{0.518212,0.410997,1.65852},{0.52367,0.420861,1.6647},{0.522371,0.418513,1.67061},
{0.513402,0.402304,1.64829},{0.510646,0.397323,1.64173},{0.512734,0.401096,1.64121},
{0.536159,0.443433,1.62766},{0.502044,0.381775,1.6255},{0.533916,0.439378,1.62258},
{0.502044,0.381775,1.6255},{0.536159,0.443433,1.62766},{0.510646,0.397323,1.64173},
{0.502044,0.381775,1.6255},{0.510646,0.397323,1.64173},{0.508558,0.393549,1.64294},
{0.510646,0.397323,1.64173},{0.536159,0.443433,1.62766},{0.537052,0.445046,1.62969},
{0.510646,0.397323,1.64173},{0.537052,0.445046,1.62969},{0.536015,0.443173,1.63488},
{0.510646,0.397323,1.64173},{0.536015,0.443173,1.63488},{0.512734,0.401096,1.64121},
{0.512734,0.401096,1.64121},{0.536015,0.443173,1.63488},{0.521071,0.416164,1.64025},
{0.521071,0.416164,1.64025},{0.536015,0.443173,1.63488},{0.534326,0.44012,1.64025},
{0.508558,0.393549,1.64294},{0.510646,0.397323,1.64173},{0.511565,0.398983,1.65745},
{0.504466,0.386152,1.6369},{0.501459,0.380718,1.6312},{0.502044,0.381775,1.6255},
{0.501459,0.380718,1.6312},{0.504466,0.386152,1.6369},{0.502712,0.382982,1.63465},
{0.502712,0.382982,1.63465},{0.504466,0.386152,1.6369},{0.505134,0.38736,1.64484},
{0.538569,0.447789,1.58305},{0.481687,0.344984,1.58192},{0.538468,0.447606,1.5751},
{0.481687,0.344984,1.58192},{0.538569,0.447789,1.58305},{0.489927,0.359877,1.60482},
{0.489927,0.359877,1.60482},{0.538569,0.447789,1.58305},{0.533005,0.437733,1.61505},
{0.489927,0.359877,1.60482},{0.533005,0.437733,1.61505},{0.496448,0.371661,1.62239},
{0.496448,0.371661,1.62239},{0.533005,0.437733,1.61505},{0.533916,0.439378,1.62258},
{0.496448,0.371661,1.62239},{0.533916,0.439378,1.62258},{0.502044,0.381775,1.6255},
{0.496448,0.371661,1.62239},{0.502044,0.381775,1.6255},{0.501459,0.380718,1.6312},
{0.496448,0.371661,1.62239},{0.501459,0.380718,1.6312},{0.502879,0.383284,1.64208},
{0.496171,0.371161,1.44028},{0.486602,0.353867,1.43744},{0.493384,0.366123,1.42968},
{0.486602,0.353867,1.43744},{0.496171,0.371161,1.44028},{0.486103,0.352965,1.462},
{0.486103,0.352965,1.462},{0.496171,0.371161,1.44028},{0.499165,0.376572,1.48304},
{0.486103,0.352965,1.462},{0.499165,0.376572,1.48304},{0.480281,0.342443,1.48343},
{0.480281,0.342443,1.48343},{0.499165,0.376572,1.48304},{0.501856,0.381436,1.49312},
{0.480281,0.342443,1.48343},{0.501856,0.381436,1.49312},{0.446216,0.280875,1.53611},
{0.446216,0.280875,1.53611},{0.501856,0.381436,1.49312},{0.503187,0.38384,1.4981},
{0.446216,0.280875,1.53611},{0.503187,0.38384,1.4981},{0.506047,0.389011,1.50881},
{0.446216,0.280875,1.53611},{0.506047,0.389011,1.50881},{0.523711,0.420934,1.51504},
{0.446216,0.280875,1.53611},{0.523711,0.420934,1.51504},{0.523312,0.420215,1.52985},
{0.446216,0.280875,1.53611},{0.523312,0.420215,1.52985},{0.525928,0.424942,1.53446},
{0.446216,0.280875,1.53611},{0.525928,0.424942,1.53446},{0.527901,0.428508,1.54244},
{0.446216,0.280875,1.53611},{0.527901,0.428508,1.54244},{0.449754,0.28727,1.54168},
{0.449754,0.28727,1.54168},{0.527901,0.428508,1.54244},{0.457559,0.301375,1.54384},
{0.449754,0.28727,1.54168},{0.457559,0.301375,1.54384},{0.450706,0.28899,1.54601},
{0.481687,0.344984,1.58192},{0.481416,0.344494,1.59141},{0.477928,0.33819,1.58593},
{0.481416,0.344494,1.59141},{0.481687,0.344984,1.58192},{0.484854,0.350708,1.59832},
{0.525928,0.424942,1.54638},{0.457559,0.301375,1.54384},{0.527901,0.428508,1.54244},
{0.457559,0.301375,1.54384},{0.525928,0.424942,1.54638},{0.46089,0.307396,1.54561},
{0.46089,0.307396,1.54561},{0.525928,0.424942,1.54638},{0.458035,0.302236,1.55014},
{0.458035,0.302236,1.55014},{0.525928,0.424942,1.54638},{0.528658,0.429876,1.54784},
{0.458035,0.302236,1.55014},{0.528658,0.429876,1.54784},{0.530756,0.433667,1.55393},
{0.458035,0.302236,1.55014},{0.530756,0.433667,1.55393},{0.459938,0.305676,1.55644},
{0.459938,0.305676,1.55644},{0.530756,0.433667,1.55393},{0.530234,0.432724,1.55643},
{0.459938,0.305676,1.55644},{0.530234,0.432724,1.55643},{0.529716,0.431789,1.56375},
{0.459938,0.305676,1.55644},{0.529716,0.431789,1.56375},{0.481687,0.344984,1.58192},
{0.459938,0.305676,1.55644},{0.481687,0.344984,1.58192},{0.463541,0.312187,1.59644},
{0.481687,0.344984,1.58192},{0.529716,0.431789,1.56375},{0.529716,0.431789,1.56928},
{0.481687,0.344984,1.58192},{0.529716,0.431789,1.56928},{0.536445,0.443949,1.57092},
{0.481687,0.344984,1.58192},{0.536445,0.443949,1.57092},{0.538468,0.447606,1.5751},
{0.463541,0.312187,1.59644},{0.481687,0.344984,1.58192},{0.477928,0.33819,1.58593},
{0.463541,0.312187,1.59644},{0.477928,0.33819,1.58593},{0.475964,0.334639,1.5974},
{0.475964,0.334639,1.5974},{0.477928,0.33819,1.58593},{0.480532,0.342896,1.60776},
{0.475964,0.334639,1.5974},{0.462069,0.309527,1.60266},{0.463541,0.312187,1.59644},
{0.462069,0.309527,1.60266},{0.475964,0.334639,1.5974},{0.476496,0.335601,1.61288},
{0.462069,0.309527,1.60266},{0.476496,0.335601,1.61288},{0.460085,0.305941,1.61728},
{0.460085,0.305941,1.61728},{0.476496,0.335601,1.61288},{0.472447,0.328284,1.6156},
{0.460085,0.305941,1.61728},{0.472447,0.328284,1.6156},{0.463346,0.311836,1.61884},
{0.463346,0.311836,1.61884},{0.472447,0.328284,1.6156},{0.469413,0.322801,1.62228},
{0.469413,0.322801,1.62228},{0.472447,0.328284,1.6156},{0.471958,0.327399,1.62471},
{0.533806,0.439181,1.65368},{0.521071,0.416164,1.64025},{0.534326,0.44012,1.64025},
{0.521071,0.416164,1.64025},{0.533806,0.439181,1.65368},{0.528998,0.430491,1.64885},
{0.521071,0.416164,1.64025},{0.528998,0.430491,1.64885},{0.525489,0.424149,1.6467},
{0.528998,0.430491,1.64885},{0.533806,0.439181,1.65368},{0.531077,0.434249,1.65449},
{0.531077,0.434249,1.65449},{0.533806,0.439181,1.65368},{0.531597,0.435188,1.6647},
{0.528478,0.429551,1.65906},{0.525489,0.424149,1.6467},{0.528998,0.430491,1.64885},
{0.469413,0.322801,1.62228},{0.466576,0.317672,1.62916},{0.463346,0.311836,1.61884},
{0.466576,0.317672,1.62916},{0.469413,0.322801,1.62228},{0.470294,0.324393,1.63138},
{0.463346,0.311836,1.61884},{0.458101,0.302355,1.6319},{0.460085,0.305941,1.61728},
{0.458101,0.302355,1.6319},{0.463346,0.311836,1.61884},{0.460215,0.306176,1.62734},
{0.460215,0.306176,1.62734},{0.463346,0.311836,1.61884},{0.462466,0.310244,1.62693},
{0.458101,0.302355,1.6319},{0.460215,0.306176,1.62734},{0.464521,0.313958,1.64717},
{0.458101,0.302355,1.6319},{0.464521,0.313958,1.64717},{0.458869,0.303743,1.64011},
{0.458869,0.303743,1.64011},{0.464521,0.313958,1.64717},{0.461685,0.308833,1.64659},
{0.474893,0.332705,1.61783},{0.472447,0.328284,1.6156},{0.476496,0.335601,1.61288},
{0.48284,0.347068,1.59964},{0.477928,0.33819,1.58593},{0.481416,0.344494,1.59141},
{0.447755,0.283658,1.54581},{0.446216,0.280875,1.53611},{0.449754,0.28727,1.54168},
{0.474418,0.331845,1.44387},{0.442592,0.274326,1.4875},{0.467383,0.319131,1.41928},
{0.442592,0.274326,1.4875},{0.474418,0.331845,1.44387},{0.475532,0.333859,1.45138},
{0.442592,0.274326,1.4875},{0.475532,0.333859,1.45138},{0.480281,0.342443,1.48343},
{0.442592,0.274326,1.4875},{0.480281,0.342443,1.48343},{0.446216,0.280875,1.53611},
{0.442592,0.274326,1.4875},{0.446216,0.280875,1.53611},{0.436221,0.26281,1.50898},
{0.442592,0.274326,1.4875},{0.436221,0.26281,1.50898},{0.434357,0.259442,1.50457},
{0.436221,0.26281,1.50898},{0.446216,0.280875,1.53611},{0.441146,0.271712,1.53664},
{0.441146,0.271712,1.53664},{0.446216,0.280875,1.53611},{0.443774,0.276461,1.5375},
{0.495704,0.370317,1.42488},{0.467383,0.319131,1.41928},{0.498805,0.375922,1.41847},
{0.467383,0.319131,1.41928},{0.495704,0.370317,1.42488},{0.474418,0.331845,1.44387},
{0.474418,0.331845,1.44387},{0.495704,0.370317,1.42488},{0.493384,0.366123,1.42968},
{0.474418,0.331845,1.44387},{0.493384,0.366123,1.42968},{0.486602,0.353867,1.43744},
{0.474418,0.331845,1.44387},{0.486602,0.353867,1.43744},{0.475532,0.333859,1.45138},
{0.534473,0.440385,0.902547},{0.467872,0.320016,0.941123},{0.522984,0.419621,0.900047},
{0.467872,0.320016,0.941123},{0.534473,0.440385,0.902547},{0.536295,0.443679,0.902944},
{0.467872,0.320016,0.941123},{0.536295,0.443679,0.902944},{0.530042,0.432377,0.929398},
{0.467872,0.320016,0.941123},{0.530042,0.432377,0.929398},{0.527131,0.427116,1.03773},
{0.467872,0.320016,0.941123},{0.527131,0.427116,1.03773},{0.446968,0.282234,0.943266},
{0.446968,0.282234,0.943266},{0.527131,0.427116,1.03773},{0.406545,0.209176,0.95403},
{0.406545,0.209176,0.95403},{0.527131,0.427116,1.03773},{0.404182,0.204907,1.00979},
{0.404182,0.204907,1.00979},{0.527131,0.427116,1.03773},{0.405811,0.207849,1.05372},
{0.405811,0.207849,1.05372},{0.527131,0.427116,1.03773},{0.527644,0.428044,1.06527},
{0.405811,0.207849,1.05372},{0.527644,0.428044,1.06527},{0.404695,0.205832,1.09006},
{0.404695,0.205832,1.09006},{0.527644,0.428044,1.06527},{0.518717,0.411909,1.2237},
{0.404695,0.205832,1.09006},{0.518717,0.411909,1.2237},{0.401347,0.199782,1.12899},
{0.401347,0.199782,1.12899},{0.518717,0.411909,1.2237},{0.401873,0.200733,1.16399},
{0.401873,0.200733,1.16399},{0.518717,0.411909,1.2237},{0.406895,0.209809,1.1842},
{0.406895,0.209809,1.1842},{0.518717,0.411909,1.2237},{0.410927,0.217097,1.20651},
{0.410927,0.217097,1.20651},{0.518717,0.411909,1.2237},{0.394587,0.187565,1.22453},
{0.394587,0.187565,1.22453},{0.518717,0.411909,1.2237},{0.515484,0.406066,1.28107},
{0.394587,0.187565,1.22453},{0.515484,0.406066,1.28107},{0.377341,0.156396,1.23332},
{0.377341,0.156396,1.23332},{0.515484,0.406066,1.28107},{0.362135,0.128912,1.26888},
{0.362135,0.128912,1.26888},{0.515484,0.406066,1.28107},{0.363745,0.131822,1.37946},
{0.363745,0.131822,1.37946},{0.515484,0.406066,1.28107},{0.509495,0.395242,1.35142},
{0.363745,0.131822,1.37946},{0.509495,0.395242,1.35142},{0.498805,0.375922,1.41847},
{0.363745,0.131822,1.37946},{0.498805,0.375922,1.41847},{0.369111,0.141521,1.40868},
{0.369111,0.141521,1.40868},{0.498805,0.375922,1.41847},{0.467383,0.319131,1.41928},
{0.369111,0.141521,1.40868},{0.467383,0.319131,1.41928},{0.381159,0.163295,1.44028},
{0.381159,0.163295,1.44028},{0.467383,0.319131,1.41928},{0.442592,0.274326,1.4875},
{0.381159,0.163295,1.44028},{0.442592,0.274326,1.4875},{0.398788,0.195156,1.46674},
{0.398788,0.195156,1.46674},{0.442592,0.274326,1.4875},{0.434357,0.259442,1.50457},
{0.36389,0.132084,1.2269},{0.362135,0.128912,1.26888},{0.361935,0.128551,1.22239},
{0.362135,0.128912,1.26888},{0.36389,0.132084,1.2269},{0.377341,0.156396,1.23332},
{0.406545,0.209176,0.95403},{0.409306,0.214166,0.920764},{0.446968,0.282234,0.943266},
{0.450706,0.28899,1.54601},{0.447755,0.283658,1.54581},{0.449754,0.28727,1.54168},
{0.46089,0.307396,1.54561},{0.450706,0.28899,1.54601},{0.457559,0.301375,1.54384},
{0.450706,0.28899,1.54601},{0.46089,0.307396,1.54561},{0.458035,0.302236,1.55014},
{0.489927,0.359877,1.60482},{0.484854,0.350708,1.59832},{0.481687,0.344984,1.58192},
{0.484854,0.350708,1.59832},{0.489927,0.359877,1.60482},{0.48284,0.347068,1.59964},
{0.48284,0.347068,1.59964},{0.480532,0.342896,1.60776},{0.477928,0.33819,1.58593},
{0.480532,0.342896,1.60776},{0.48284,0.347068,1.59964},{0.489927,0.359877,1.60482},
{0.480532,0.342896,1.60776},{0.489927,0.359877,1.60482},{0.496448,0.371661,1.62239},
{0.480532,0.342896,1.60776},{0.496448,0.371661,1.62239},{0.476496,0.335601,1.61288},
{0.476496,0.335601,1.61288},{0.496448,0.371661,1.62239},{0.474893,0.332705,1.61783},
{0.474893,0.332705,1.61783},{0.471958,0.327399,1.62471},{0.472447,0.328284,1.6156},
{0.471958,0.327399,1.62471},{0.474893,0.332705,1.61783},{0.496448,0.371661,1.62239},
{0.471958,0.327399,1.62471},{0.496448,0.371661,1.62239},{0.502879,0.383284,1.64208},
{0.471958,0.327399,1.62471},{0.502879,0.383284,1.64208},{0.470294,0.324393,1.63138},
{0.502712,0.382982,1.63465},{0.502879,0.383284,1.64208},{0.501459,0.380718,1.6312},
{0.502879,0.383284,1.64208},{0.502712,0.382982,1.63465},{0.505134,0.38736,1.64484},
{0.538355,0.447401,1.64132},{0.536015,0.443173,1.6424},{0.536015,0.443173,1.63488},
{0.536015,0.443173,1.6424},{0.538355,0.447401,1.64132},{0.539997,0.450369,1.64585},
{0.540694,0.451629,1.64777},{0.539997,0.450369,1.64585},{0.541168,0.452485,1.64261},
{0.508558,0.393549,1.64294},{0.504466,0.386152,1.6369},{0.502044,0.381775,1.6255},
{0.504466,0.386152,1.6369},{0.508558,0.393549,1.64294},{0.505134,0.38736,1.64484},
{0.505134,0.38736,1.64484},{0.508558,0.393549,1.64294},{0.511565,0.398983,1.65745},
{0.505134,0.38736,1.64484},{0.511565,0.398983,1.65745},{0.464521,0.313958,1.64717},
{0.518212,0.410997,1.65852},{0.513402,0.402304,1.64829},{0.512734,0.401096,1.64121},
{0.513402,0.402304,1.64829},{0.518212,0.410997,1.65852},{0.511565,0.398983,1.65745},
{0.531077,0.434249,1.65449},{0.528478,0.429551,1.65906},{0.528998,0.430491,1.64885},
{0.528478,0.429551,1.65906},{0.531077,0.434249,1.65449},{0.531597,0.435188,1.6647},
{0.542383,0.454682,1.66497},{0.541712,0.453469,1.65814},{0.542613,0.455097,1.65619},
{0.541712,0.453469,1.65814},{0.542383,0.454682,1.66497},{0.531597,0.435188,1.6647},
{0.531597,0.435188,1.6647},{0.542383,0.454682,1.66497},{0.52367,0.420861,1.6647},
{0.52367,0.420861,1.6647},{0.542383,0.454682,1.66497},{0.522371,0.418513,1.67061},
{0.522371,0.418513,1.67061},{0.542383,0.454682,1.66497},{0.540293,0.450904,1.69208},
{0.447755,0.283658,1.54581},{0.443774,0.276461,1.5375},{0.446216,0.280875,1.53611},
{0.443774,0.276461,1.5375},{0.447755,0.283658,1.54581},{0.441703,0.27272,1.56774},
{0.441703,0.27272,1.56774},{0.447755,0.283658,1.54581},{0.450706,0.28899,1.54601},
{0.441703,0.27272,1.56774},{0.450706,0.28899,1.54601},{0.458035,0.302236,1.55014},
{0.441703,0.27272,1.56774},{0.458035,0.302236,1.55014},{0.459938,0.305676,1.55644},
{0.441703,0.27272,1.56774},{0.459938,0.305676,1.55644},{0.463541,0.312187,1.59644},
{0.441703,0.27272,1.56774},{0.463541,0.312187,1.59644},{0.43521,0.260983,1.59565},
{0.43521,0.260983,1.59565},{0.463541,0.312187,1.59644},{0.433745,0.258335,1.66228},
{0.433745,0.258335,1.66228},{0.463541,0.312187,1.59644},{0.462069,0.309527,1.60266},
{0.433745,0.258335,1.66228},{0.462069,0.309527,1.60266},{0.460085,0.305941,1.61728},
{0.433745,0.258335,1.66228},{0.460085,0.305941,1.61728},{0.458101,0.302355,1.6319},
{0.433745,0.258335,1.66228},{0.458101,0.302355,1.6319},{0.458869,0.303743,1.64011},
{0.433745,0.258335,1.66228},{0.458869,0.303743,1.64011},{0.461685,0.308833,1.64659},
{0.433745,0.258335,1.66228},{0.461685,0.308833,1.64659},{0.464521,0.313958,1.64717},
{0.433745,0.258335,1.66228},{0.464521,0.313958,1.64717},{0.511565,0.398983,1.65745},
{0.433745,0.258335,1.66228},{0.511565,0.398983,1.65745},{0.518212,0.410997,1.65852},
{0.433745,0.258335,1.66228},{0.518212,0.410997,1.65852},{0.522371,0.418513,1.67061},
{0.433745,0.258335,1.66228},{0.522371,0.418513,1.67061},{0.441253,0.271905,1.70734},
{0.441253,0.271905,1.70734},{0.522371,0.418513,1.67061},{0.540293,0.450904,1.69208},
{0.441253,0.271905,1.70734},{0.540293,0.450904,1.69208},{0.533647,0.438893,1.71755},
{0.441253,0.271905,1.70734},{0.533647,0.438893,1.71755},{0.44467,0.278082,1.71489},
{0.44467,0.278082,1.71489},{0.533647,0.438893,1.71755},{0.457001,0.300367,1.74217},
{0.457001,0.300367,1.74217},{0.533647,0.438893,1.71755},{0.522787,0.419264,1.73699},
{0.457001,0.300367,1.74217},{0.522787,0.419264,1.73699},{0.50836,0.393191,1.74972},
{0.457001,0.300367,1.74217},{0.50836,0.393191,1.74972},{0.471101,0.325851,1.75617},
{0.471101,0.325851,1.75617},{0.50836,0.393191,1.74972},{0.489233,0.358621,1.75877},
{0.525489,0.424149,1.6467},{0.52354,0.420626,1.651},{0.521071,0.416164,1.64025},
{0.52354,0.420626,1.651},{0.525489,0.424149,1.6467},{0.528478,0.429551,1.65906},
{0.52354,0.420626,1.651},{0.528478,0.429551,1.65906},{0.52367,0.420861,1.6647},
{0.52367,0.420861,1.6647},{0.528478,0.429551,1.65906},{0.531597,0.435188,1.6647},
{0.536015,0.443173,1.6424},{0.533806,0.439181,1.65368},{0.534326,0.44012,1.64025},
{0.533806,0.439181,1.65368},{0.536015,0.443173,1.6424},{0.539997,0.450369,1.64585},
{0.533806,0.439181,1.65368},{0.539997,0.450369,1.64585},{0.540694,0.451629,1.64777},
{0.533806,0.439181,1.65368},{0.540694,0.451629,1.64777},{0.541712,0.453469,1.65814},
{0.533806,0.439181,1.65368},{0.541712,0.453469,1.65814},{0.531597,0.435188,1.6647},
{0.511565,0.398983,1.65745},{0.510646,0.397323,1.64173},{0.513402,0.402304,1.64829},
{0.466576,0.317672,1.62916},{0.462466,0.310244,1.62693},{0.463346,0.311836,1.61884},
{0.462466,0.310244,1.62693},{0.466576,0.317672,1.62916},{0.460215,0.306176,1.62734},
{0.460215,0.306176,1.62734},{0.466576,0.317672,1.62916},{0.464521,0.313958,1.64717},
{0.464521,0.313958,1.64717},{0.466576,0.317672,1.62916},{0.470294,0.324393,1.63138},
{0.464521,0.313958,1.64717},{0.470294,0.324393,1.63138},{0.502879,0.383284,1.64208},
{0.464521,0.313958,1.64717},{0.502879,0.383284,1.64208},{0.505134,0.38736,1.64484},
{0.470294,0.324393,1.63138},{0.469413,0.322801,1.62228},{0.471958,0.327399,1.62471},
{0.476496,0.335601,1.61288},{0.475964,0.334639,1.5974},{0.480532,0.342896,1.60776},
{0.48284,0.347068,1.59964},{0.481416,0.344494,1.59141},{0.484854,0.350708,1.59832},
{0.43521,0.260983,1.59565},{0.440703,0.270912,1.55363},{0.441703,0.27272,1.56774},
{0.486103,0.352965,1.462},{0.475532,0.333859,1.45138},{0.486602,0.353867,1.43744},
{0.475532,0.333859,1.45138},{0.486103,0.352965,1.462},{0.480281,0.342443,1.48343},
{0.498253,0.374923,0.0516863},{0.483369,0.348024,0.0460633},{0.499549,0.377266,0.0444885},
{0.483369,0.348024,0.0460633},{0.498253,0.374923,0.0516863},{0.495156,0.369326,0.0529478},
{0.451677,0.290744,0.0776394},{0.440287,0.270159,0.0669358},{0.468832,0.32175,0.0520928},
{0.497719,0.373959,0.0414405},{0.465863,0.316385,0.038916},{0.469199,0.322413,0.0342559},
{0.465863,0.316385,0.038916},{0.497719,0.373959,0.0414405},{0.483369,0.348024,0.0460633},
{0.465863,0.316385,0.038916},{0.483369,0.348024,0.0460633},{0.425985,0.24431,0.0669358},
{0.483369,0.348024,0.0460633},{0.497719,0.373959,0.0414405},{0.499549,0.377266,0.0444885},
{0.425985,0.24431,0.0669358},{0.483369,0.348024,0.0460633},{0.468832,0.32175,0.0520928},
{0.425985,0.24431,0.0669358},{0.468832,0.32175,0.0520928},{0.440287,0.270159,0.0669358},
{0.552559,0.473072,0.77809},{0.548814,0.466304,0.779605},{0.550442,0.469247,0.776575},
{0.548814,0.466304,0.779605},{0.552559,0.473072,0.77809},{0.557117,0.481312,0.800306},
{0.548814,0.466304,0.779605},{0.557117,0.481312,0.800306},{0.547837,0.464539,0.788188},
{0.547837,0.464539,0.788188},{0.557117,0.481312,0.800306},{0.54857,0.465863,0.796435},
{0.54857,0.465863,0.796435},{0.557117,0.481312,0.800306},{0.550012,0.46847,0.809291},
{0.550012,0.46847,0.809291},{0.557117,0.481312,0.800306},{0.558539,0.483881,0.818932},
{0.550012,0.46847,0.809291},{0.558539,0.483881,0.818932},{0.550418,0.469204,0.822937},
{0.567772,0.500568,0.76076},{0.562647,0.491306,0.760458},{0.56499,0.49554,0.758339},
{0.562647,0.491306,0.760458},{0.567772,0.500568,0.76076},{0.561769,0.489719,0.766058},
{0.561769,0.489719,0.766058},{0.567772,0.500568,0.76076},{0.57092,0.506258,0.769993},
{0.561769,0.489719,0.766058},{0.57092,0.506258,0.769993},{0.562135,0.49038,0.772717},
{0.562135,0.49038,0.772717},{0.57092,0.506258,0.769993},{0.57431,0.512385,0.785601},
{0.562135,0.49038,0.772717},{0.57431,0.512385,0.785601},{0.562601,0.491222,0.775575},
{0.562601,0.491222,0.775575},{0.57431,0.512385,0.785601},{0.554668,0.476884,0.779982},
{0.554668,0.476884,0.779982},{0.57431,0.512385,0.785601},{0.556571,0.480325,0.787702},
{0.560744,0.487866,0.76969},{0.55562,0.478605,0.766815},{0.558109,0.483103,0.765906},
{0.55562,0.478605,0.766815},{0.560744,0.487866,0.76969},{0.554155,0.475958,0.772566},
{0.554155,0.475958,0.772566},{0.560744,0.487866,0.76969},{0.562601,0.491222,0.775575},
{0.554155,0.475958,0.772566},{0.562601,0.491222,0.775575},{0.554668,0.476884,0.779982},
{0.580302,0.523215,0.773212},{0.574865,0.513388,0.7739},{0.577972,0.519003,0.77},
{0.574865,0.513388,0.7739},{0.580302,0.523215,0.773212},{0.58463,0.531037,0.815426},
{0.574865,0.513388,0.7739},{0.58463,0.531037,0.815426},{0.57431,0.512385,0.785601},
{0.57431,0.512385,0.785601},{0.58463,0.531037,0.815426},{0.556571,0.480325,0.787702},
{0.556571,0.480325,0.787702},{0.58463,0.531037,0.815426},{0.558841,0.484426,0.800869},
{0.558841,0.484426,0.800869},{0.58463,0.531037,0.815426},{0.558539,0.483881,0.812957},
{0.558539,0.483881,0.812957},{0.58463,0.531037,0.815426},{0.558539,0.483881,0.818932},
{0.558539,0.483881,0.818932},{0.58463,0.531037,0.815426},{0.580302,0.523215,0.856492},
{0.558539,0.483881,0.818932},{0.580302,0.523215,0.856492},{0.550418,0.469204,0.822937},
{0.550418,0.469204,0.822937},{0.580302,0.523215,0.856492},{0.543615,0.456908,0.844561},
{0.543615,0.456908,0.844561},{0.580302,0.523215,0.856492},{0.540162,0.450668,0.865765},
{0.540162,0.450668,0.865765},{0.580302,0.523215,0.856492},{0.57334,0.510631,0.879365},
{0.540162,0.450668,0.865765},{0.57334,0.510631,0.879365},{0.538118,0.446973,0.903341},
{0.538118,0.446973,0.903341},{0.57334,0.510631,0.879365},{0.547874,0.464606,0.904351},
{0.547874,0.464606,0.904351},{0.57334,0.510631,0.879365},{0.563119,0.492158,0.906402},
{0.547874,0.464606,0.904351},{0.563119,0.492158,0.906402},{0.544377,0.458285,0.922245},
{0.544377,0.458285,0.922245},{0.563119,0.492158,0.906402},{0.545318,0.459986,1.0069},
{0.544377,0.458285,0.922245},{0.545318,0.459986,1.0069},{0.536343,0.443765,1.08648},
{0.536343,0.443765,1.08648},{0.545318,0.459986,1.0069},{0.536285,0.443661,1.09643},
{0.545961,0.461149,0.905048},{0.538118,0.446973,0.903341},{0.547874,0.464606,0.904351},
{0.540169,0.45068,0.0173143},{0.527003,0.426885,0.0177134},{0.523059,0.419756,0.0103483},
{0.527003,0.426885,0.0177134},{0.540169,0.45068,0.0173143},{0.548257,0.465297,0.0261399},
{0.527003,0.426885,0.0177134},{0.548257,0.465297,0.0261399},{0.527243,0.427318,0.0197571},
{0.527243,0.427318,0.0197571},{0.548257,0.465297,0.0261399},{0.528374,0.429362,0.0294042},
{0.528374,0.429362,0.0294042},{0.548257,0.465297,0.0261399},{0.547975,0.464789,0.0381686},
{0.528374,0.429362,0.0294042},{0.547975,0.464789,0.0381686},{0.525118,0.423478,0.0400322},
{0.525118,0.423478,0.0400322},{0.547975,0.464789,0.0381686},{0.541031,0.452238,0.0467051},
{0.525118,0.423478,0.0400322},{0.541031,0.452238,0.0467051},{0.526574,0.426109,0.0424557},
{0.526574,0.426109,0.0424557},{0.541031,0.452238,0.0467051},{0.529494,0.431387,0.0447778},
{0.529494,0.431387,0.0447778},{0.541031,0.452238,0.0467051},{0.528034,0.428748,0.0484933},
{0.528034,0.428748,0.0484933},{0.541031,0.452238,0.0467051},{0.530333,0.432904,0.0488392},
{0.528034,0.428748,0.0484933},{0.530333,0.432904,0.0488392},{0.523492,0.420539,0.0518415},
{0.523492,0.420539,0.0518415},{0.530333,0.432904,0.0488392},{0.506029,0.388977,0.0761948},
{0.523492,0.420539,0.0518415},{0.506029,0.388977,0.0761948},{0.512634,0.400914,0.0567657},
{0.512634,0.400914,0.0567657},{0.506029,0.388977,0.0761948},{0.505428,0.387892,0.0626447},
{0.505428,0.387892,0.0626447},{0.506029,0.388977,0.0761948},{0.490176,0.360326,0.0966224},
{0.490176,0.360326,0.0966224},{0.506029,0.388977,0.0761948},{0.496783,0.372267,0.0911341},
{0.490176,0.360326,0.0966224},{0.496783,0.372267,0.0911341},{0.490631,0.361148,0.0971143},
{0.490631,0.361148,0.0971143},{0.496783,0.372267,0.0911341},{0.495047,0.36913,0.101891},
{0.521978,0.417804,0.0432462},{0.525118,0.423478,0.0400322},{0.526574,0.426109,0.0424557},
{0.526013,0.425096,0.00266449},{0.521691,0.417284,0.00779389},{0.507612,0.391838,0.00167739},
{0.521691,0.417284,0.00779389},{0.526013,0.425096,0.00266449},{0.541226,0.452591,0.0111122},
{0.521691,0.417284,0.00779389},{0.541226,0.452591,0.0111122},{0.523059,0.419756,0.0103483},
{0.523059,0.419756,0.0103483},{0.541226,0.452591,0.0111122},{0.540169,0.45068,0.0173143},
{0.540169,0.45068,0.0173143},{0.541226,0.452591,0.0111122},{0.548257,0.465297,0.0261399},
{0.544377,0.458285,0.922245},{0.545961,0.461149,0.905048},{0.547874,0.464606,0.904351},
{0.538118,0.446973,0.903341},{0.530042,0.432377,0.929398},{0.536295,0.443679,0.902944},
{0.530042,0.432377,0.929398},{0.538118,0.446973,0.903341},{0.545961,0.461149,0.905048},
{0.530042,0.432377,0.929398},{0.545961,0.461149,0.905048},{0.544377,0.458285,0.922245},
{0.530042,0.432377,0.929398},{0.544377,0.458285,0.922245},{0.536343,0.443765,1.08648},
{0.530042,0.432377,0.929398},{0.536343,0.443765,1.08648},{0.527131,0.427116,1.03773},
{0.527131,0.427116,1.03773},{0.536343,0.443765,1.08648},{0.527644,0.428044,1.06527},
{0.527644,0.428044,1.06527},{0.536343,0.443765,1.08648},{0.518717,0.411909,1.2237},
{0.518717,0.411909,1.2237},{0.536343,0.443765,1.08648},{0.536285,0.443661,1.09643},
{0.518717,0.411909,1.2237},{0.536285,0.443661,1.09643},{0.535524,0.442285,1.22808},
{0.518717,0.411909,1.2237},{0.535524,0.442285,1.22808},{0.515484,0.406066,1.28107},
{0.515484,0.406066,1.28107},{0.535524,0.442285,1.22808},{0.535992,0.443131,1.30184},
{0.515484,0.406066,1.28107},{0.535992,0.443131,1.30184},{0.509495,0.395242,1.35142},
{0.509495,0.395242,1.35142},{0.535992,0.443131,1.30184},{0.533733,0.439048,1.33135},
{0.509495,0.395242,1.35142},{0.533733,0.439048,1.33135},{0.524271,0.421947,1.38675},
{0.509495,0.395242,1.35142},{0.524271,0.421947,1.38675},{0.498805,0.375922,1.41847},
{0.498805,0.375922,1.41847},{0.524271,0.421947,1.38675},{0.518928,0.41229,1.40596},
{0.498805,0.375922,1.41847},{0.518928,0.41229,1.40596},{0.507311,0.391295,1.43829},
{0.498805,0.375922,1.41847},{0.507311,0.391295,1.43829},{0.495704,0.370317,1.42488},
{0.521691,0.417284,0.00779389},{0.464959,0.31475,0.0041534},{0.507612,0.391838,0.00167739},
{0.464959,0.31475,0.0041534},{0.521691,0.417284,0.00779389},{0.480666,0.343139,0.00628454},
{0.480666,0.343139,0.00628454},{0.521691,0.417284,0.00779389},{0.493552,0.366428,0.00803289},
{0.493552,0.366428,0.00803289},{0.521691,0.417284,0.00779389},{0.523059,0.419756,0.0103483},
{0.493552,0.366428,0.00803289},{0.523059,0.419756,0.0103483},{0.517019,0.40884,0.017347},
{0.517019,0.40884,0.017347},{0.523059,0.419756,0.0103483},{0.527003,0.426885,0.0177134},
{0.517019,0.40884,0.017347},{0.527003,0.426885,0.0177134},{0.527243,0.427318,0.0197571},
{0.464211,0.313399,0},{0.503751,0.38486,-1.77965e-047},{0.507612,0.391838,0.00167739},
{0.464211,0.313399,0},{0.507612,0.391838,0.00167739},{0.464959,0.31475,0.0041534},
{0.464211,0.313399,0},{0.464959,0.31475,0.0041534},{0.446057,0.280588,0.0074893},
{0.446057,0.280588,0.0074893},{0.464959,0.31475,0.0041534},{0.457139,0.300617,0.00827749},
{0.446057,0.280588,0.0074893},{0.457139,0.300617,0.00827749},{0.422101,0.237292,0.00766383},
{0.422101,0.237292,0.00766383},{0.457139,0.300617,0.00827749},{0.41458,0.223699,0.016727},
{0.41458,0.223699,0.016727},{0.457139,0.300617,0.00827749},{0.439165,0.268132,0.0177566},
{0.41458,0.223699,0.016727},{0.439165,0.268132,0.0177566},{0.418997,0.231681,0.0248036},
{0.41458,0.223699,0.016727},{0.418997,0.231681,0.0248036},{0.413062,0.220955,0.0334404},
{0.507311,0.391295,1.43829},{0.493384,0.366123,1.42968},{0.495704,0.370317,1.42488},
{0.493384,0.366123,1.42968},{0.507311,0.391295,1.43829},{0.496171,0.371161,1.44028},
{0.496171,0.371161,1.44028},{0.507311,0.391295,1.43829},{0.515178,0.405512,1.46571},
{0.496171,0.371161,1.44028},{0.515178,0.405512,1.46571},{0.499165,0.376572,1.48304},
{0.499165,0.376572,1.48304},{0.515178,0.405512,1.46571},{0.501856,0.381436,1.49312},
{0.501856,0.381436,1.49312},{0.515178,0.405512,1.46571},{0.503187,0.38384,1.4981},
{0.411739,0.218564,0.755537},{0.406608,0.209291,0.750747},{0.410084,0.215573,0.750405},
{0.406608,0.209291,0.750747},{0.411739,0.218564,0.755537},{0.404374,0.205253,0.75605},
{0.404374,0.205253,0.75605},{0.411739,0.218564,0.755537},{0.41207,0.219162,0.762039},
{0.404374,0.205253,0.75605},{0.41207,0.219162,0.762039},{0.403269,0.203255,0.761062},
{0.418111,0.23008,0.759301},{0.414387,0.22335,0.75759},{0.416373,0.226939,0.756906},
{0.414387,0.22335,0.75759},{0.418111,0.23008,0.759301},{0.41207,0.219162,0.762039},
{0.41207,0.219162,0.762039},{0.418111,0.23008,0.759301},{0.419021,0.231725,0.770935},
{0.433391,0.257696,0.805787},{0.428656,0.249138,0.807554},{0.431023,0.253417,0.805379},
{0.428656,0.249138,0.807554},{0.433391,0.257696,0.805787},{0.435232,0.261024,0.808506},
{0.428656,0.249138,0.807554},{0.435232,0.261024,0.808506},{0.426025,0.244383,0.812041},
{0.426025,0.244383,0.812041},{0.435232,0.261024,0.808506},{0.435495,0.261499,0.811225},
{0.426025,0.244383,0.812041},{0.435495,0.261499,0.811225},{0.432338,0.255794,0.823463},
{0.426025,0.244383,0.812041},{0.432338,0.255794,0.823463},{0.421882,0.236895,0.82523},
{0.421882,0.236895,0.82523},{0.432338,0.255794,0.823463},{0.430629,0.252704,0.836788},
{0.421882,0.236895,0.82523},{0.430629,0.252704,0.836788},{0.418725,0.23119,0.843042},
{0.418725,0.23119,0.843042},{0.430629,0.252704,0.836788},{0.426946,0.246047,0.855551},
{0.418725,0.23119,0.843042},{0.426946,0.246047,0.855551},{0.418121,0.230097,0.843816},
{0.403269,0.203255,0.761062},{0.398168,0.194036,0.760328},{0.402471,0.201813,0.758275},
{0.398168,0.194036,0.760328},{0.403269,0.203255,0.761062},{0.392042,0.182964,0.782855},
{0.392042,0.182964,0.782855},{0.403269,0.203255,0.761062},{0.41207,0.219162,0.762039},
{0.392042,0.182964,0.782855},{0.41207,0.219162,0.762039},{0.419021,0.231725,0.770935},
{0.392042,0.182964,0.782855},{0.419021,0.231725,0.770935},{0.416357,0.226911,0.801436},
{0.392042,0.182964,0.782855},{0.416357,0.226911,0.801436},{0.383769,0.168012,0.795059},
{0.383769,0.168012,0.795059},{0.416357,0.226911,0.801436},{0.380446,0.162007,0.81124},
{0.380446,0.162007,0.81124},{0.416357,0.226911,0.801436},{0.416357,0.226911,0.839235},
{0.380446,0.162007,0.81124},{0.416357,0.226911,0.839235},{0.379613,0.160501,0.874101},
{0.379613,0.160501,0.874101},{0.416357,0.226911,0.839235},{0.418121,0.230097,0.843816},
{0.379613,0.160501,0.874101},{0.418121,0.230097,0.843816},{0.426946,0.246047,0.855551},
{0.379613,0.160501,0.874101},{0.426946,0.246047,0.855551},{0.425324,0.243116,0.869808},
{0.379613,0.160501,0.874101},{0.425324,0.243116,0.869808},{0.418617,0.230995,0.897276},
{0.379613,0.160501,0.874101},{0.418617,0.230995,0.897276},{0.382121,0.165034,0.908264},
{0.382121,0.165034,0.908264},{0.418617,0.230995,0.897276},{0.41343,0.221619,0.906694},
{0.382121,0.165034,0.908264},{0.41343,0.221619,0.906694},{0.409306,0.214166,0.920764},
{0.382121,0.165034,0.908264},{0.409306,0.214166,0.920764},{0.378059,0.157693,0.926142},
{0.378059,0.157693,0.926142},{0.409306,0.214166,0.920764},{0.406545,0.209176,0.95403},
{0.378059,0.157693,0.926142},{0.406545,0.209176,0.95403},{0.376749,0.155325,0.952688},
{0.376749,0.155325,0.952688},{0.406545,0.209176,0.95403},{0.367446,0.138512,1.0474},
{0.367446,0.138512,1.0474},{0.406545,0.209176,0.95403},{0.404182,0.204907,1.00979},
{0.367446,0.138512,1.0474},{0.404182,0.204907,1.00979},{0.405811,0.207849,1.05372},
{0.367446,0.138512,1.0474},{0.405811,0.207849,1.05372},{0.363896,0.132095,1.13882},
{0.363896,0.132095,1.13882},{0.405811,0.207849,1.05372},{0.404695,0.205832,1.09006},
{0.363896,0.132095,1.13882},{0.404695,0.205832,1.09006},{0.401347,0.199782,1.12899},
{0.363896,0.132095,1.13882},{0.401347,0.199782,1.12899},{0.401873,0.200733,1.16399},
{0.363896,0.132095,1.13882},{0.401873,0.200733,1.16399},{0.364894,0.133899,1.16194},
{0.364894,0.133899,1.16194},{0.401873,0.200733,1.16399},{0.36389,0.132084,1.2269},
{0.36389,0.132084,1.2269},{0.401873,0.200733,1.16399},{0.406895,0.209809,1.1842},
{0.36389,0.132084,1.2269},{0.406895,0.209809,1.1842},{0.410927,0.217097,1.20651},
{0.36389,0.132084,1.2269},{0.410927,0.217097,1.20651},{0.394587,0.187565,1.22453},
{0.36389,0.132084,1.2269},{0.394587,0.187565,1.22453},{0.377341,0.156396,1.23332},
{0.392042,0.182964,0.782855},{0.389065,0.177584,0.782398},{0.391051,0.181173,0.781029},
{0.389065,0.177584,0.782398},{0.392042,0.182964,0.782855},{0.383769,0.168012,0.795059},
{0.493552,0.366428,0.00803289},{0.472534,0.328441,0.0295958},{0.480666,0.343139,0.00628454},
{0.472534,0.328441,0.0295958},{0.493552,0.366428,0.00803289},{0.517019,0.40884,0.017347},
{0.472534,0.328441,0.0295958},{0.517019,0.40884,0.017347},{0.527243,0.427318,0.0197571},
{0.472534,0.328441,0.0295958},{0.527243,0.427318,0.0197571},{0.528374,0.429362,0.0294042},
{0.472534,0.328441,0.0295958},{0.528374,0.429362,0.0294042},{0.525118,0.423478,0.0400322},
{0.472534,0.328441,0.0295958},{0.525118,0.423478,0.0400322},{0.469199,0.322413,0.0342559},
{0.469199,0.322413,0.0342559},{0.525118,0.423478,0.0400322},{0.497719,0.373959,0.0414405},
{0.497719,0.373959,0.0414405},{0.525118,0.423478,0.0400322},{0.521978,0.417804,0.0432462},
{0.497719,0.373959,0.0414405},{0.521978,0.417804,0.0432462},{0.499549,0.377266,0.0444885},
{0.499549,0.377266,0.0444885},{0.521978,0.417804,0.0432462},{0.51211,0.399968,0.0533484},
{0.499549,0.377266,0.0444885},{0.51211,0.399968,0.0533484},{0.498253,0.374923,0.0516863},
{0.498253,0.374923,0.0516863},{0.51211,0.399968,0.0533484},{0.495156,0.369326,0.0529478},
{0.495156,0.369326,0.0529478},{0.468832,0.32175,0.0520928},{0.483369,0.348024,0.0460633},
{0.468832,0.32175,0.0520928},{0.495156,0.369326,0.0529478},{0.451677,0.290744,0.0776394},
{0.451677,0.290744,0.0776394},{0.495156,0.369326,0.0529478},{0.51211,0.399968,0.0533484},
{0.451677,0.290744,0.0776394},{0.51211,0.399968,0.0533484},{0.507469,0.39158,0.0580999},
{0.451677,0.290744,0.0776394},{0.507469,0.39158,0.0580999},{0.505428,0.387892,0.0626447},
{0.451677,0.290744,0.0776394},{0.505428,0.387892,0.0626447},{0.490176,0.360326,0.0966224},
{0.451677,0.290744,0.0776394},{0.490176,0.360326,0.0966224},{0.465316,0.315395,0.0904569},
{0.465316,0.315395,0.0904569},{0.490176,0.360326,0.0966224},{0.488619,0.357511,0.100092},
{0.480666,0.343139,0.00628454},{0.457139,0.300617,0.00827749},{0.464959,0.31475,0.0041534},
{0.457139,0.300617,0.00827749},{0.480666,0.343139,0.00628454},{0.472534,0.328441,0.0295958},
{0.457139,0.300617,0.00827749},{0.472534,0.328441,0.0295958},{0.451817,0.290998,0.024232},
{0.451817,0.290998,0.024232},{0.472534,0.328441,0.0295958},{0.443072,0.275193,0.0367642},
{0.443072,0.275193,0.0367642},{0.472534,0.328441,0.0295958},{0.469199,0.322413,0.0342559},
{0.443072,0.275193,0.0367642},{0.469199,0.322413,0.0342559},{0.465863,0.316385,0.038916},
{0.443072,0.275193,0.0367642},{0.465863,0.316385,0.038916},{0.423316,0.239487,0.0534973},
{0.423316,0.239487,0.0534973},{0.465863,0.316385,0.038916},{0.425985,0.24431,0.0669358},
{0.423316,0.239487,0.0534973},{0.425985,0.24431,0.0669358},{0.41528,0.224964,0.0669358},
{0.529494,0.431387,0.0447778},{0.521978,0.417804,0.0432462},{0.526574,0.426109,0.0424557},
{0.521978,0.417804,0.0432462},{0.529494,0.431387,0.0447778},{0.51211,0.399968,0.0533484},
{0.51211,0.399968,0.0533484},{0.529494,0.431387,0.0447778},{0.528034,0.428748,0.0484933},
{0.51211,0.399968,0.0533484},{0.528034,0.428748,0.0484933},{0.523492,0.420539,0.0518415},
{0.51211,0.399968,0.0533484},{0.523492,0.420539,0.0518415},{0.512634,0.400914,0.0567657},
{0.51211,0.399968,0.0533484},{0.512634,0.400914,0.0567657},{0.507469,0.39158,0.0580999},
{0.507469,0.39158,0.0580999},{0.512634,0.400914,0.0567657},{0.505428,0.387892,0.0626447},
{6.08984,15.246,0},{0.000185323,-0.000313449,0},{0.000185323,15.246,0},
{0.000185323,-0.000313449,0},{6.08984,15.246,0},{3.04184,-0.000313449,0},
{3.04184,-0.000313449,0},{6.08984,15.246,0},{6.01489,-0.000313449,-2.58041e-031},
{6.01489,-0.000313449,-2.58041e-031},{6.08984,15.246,0},{6.08984,-0.000313449,0},
{0.000185323,12.6933,1.8288},{0.000185323,-0.000313449,3.3922},{0.000185323,-0.000313449,3.6576},
{0.000185323,-0.000313449,3.3922},{0.000185323,12.6933,1.8288},{0.000185323,-0.000313449,0},
{0.000185323,-0.000313449,0},{0.000185323,12.6933,1.8288},{0.000185323,15.246,0},
{0.000185323,15.246,0},{0.000185323,12.6933,1.8288},{0.000185323,15.1863,1.8288},
{0.000185323,15.246,0},{0.000185323,15.1863,1.8288},{0.000185323,15.246,1.8288},
{0.000185323,15.246,1.8288},{6.08984,15.246,0},{0.000185323,15.246,0},
{6.08984,15.246,0},{0.000185323,15.246,1.8288},{6.08984,15.246,1.8288},
{2.97125,12.6933,1.8288},{0.000185323,15.1863,1.8288},{0.000185323,12.6933,1.8288},
{0.000185323,15.1863,1.8288},{2.97125,12.6933,1.8288},{0.0155184,15.1863,1.8288},
{0.0155184,15.1863,1.8288},{2.97125,12.6933,1.8288},{0.0155184,15.246,1.8288},
{0.0155184,15.246,1.8288},{2.97125,12.6933,1.8288},{3.04309,15.246,1.8288},
{3.04309,15.246,1.8288},{2.97125,12.6933,1.8288},{3.04184,12.6933,1.8288},
{3.04309,15.246,1.8288},{3.04184,12.6933,1.8288},{6.01489,12.6933,1.8288},
{3.04309,15.246,1.8288},{6.01489,12.6933,1.8288},{6.08984,15.246,1.8288},
{6.08984,15.246,1.8288},{6.01489,12.6933,1.8288},{6.08984,12.6933,1.8288},
{3.04184,12.6489,1.8224},{3.04184,-0.000313449,3.3922},{3.04184,-0.000313449,0},
{3.04184,-0.000313449,3.3922},{3.04184,12.6489,1.8224},{3.04184,-0.000313449,3.6576},
{3.04184,-0.000313449,3.6576},{3.04184,12.6489,1.8224},{3.04184,0.0441143,3.69609},
{3.04184,0.0441143,3.69609},{3.04184,12.6489,1.8224},{3.04184,12.6489,1.88009},
{3.04184,0.0441143,3.69609},{3.04184,-0.000313449,3.69609},{3.04184,-0.000313449,3.6576},
{6.01489,-0.000313449,-2.58041e-031},{3.04184,12.6489,1.8224},{3.04184,-0.000313449,0},
{3.04184,12.6489,1.8224},{6.01489,-0.000313449,-2.58041e-031},{3.04184,12.6933,1.8288},
{3.04184,12.6933,1.8288},{6.01489,-0.000313449,-2.58041e-031},{6.01489,12.6933,1.8288},
{0.000185323,15.246,2.73339},{0.0155184,15.246,1.8288},{0.000185323,15.246,1.8288},
{0.0155184,15.246,1.8288},{0.000185323,15.246,2.73339},{0.0155184,15.246,2.16692},
{0.0155184,15.246,2.16692},{0.000185323,15.246,2.73339},{0.0161388,15.246,2.18287},
{0.0155184,15.246,2.16692},{0.0161388,15.246,2.18287},{6.08984,15.246,2.16692},
{0.0161388,15.246,2.18287},{0.000185323,15.246,2.73339},{0.0161388,15.246,2.71744},
{0.0161388,15.246,2.71744},{0.000185323,15.246,2.73339},{5.99893,15.246,2.71744},
{6.08984,15.246,2.16692},{5.99893,15.246,2.18287},{6.08984,15.246,2.73339},
{5.99893,15.246,2.18287},{6.08984,15.246,2.16692},{0.0161388,15.246,2.18287},
{6.08984,15.246,2.73339},{5.99893,15.246,2.18287},{5.99893,15.246,2.71744},
{6.08984,15.246,2.73339},{5.99893,15.246,2.71744},{0.000185323,15.246,2.73339},
{6.01489,15.1863,2.16692},{5.99893,15.1863,2.18287},{0.0155184,15.1863,2.16692},
{5.99893,15.1863,2.18287},{6.01489,15.1863,2.16692},{6.01489,15.1863,2.73339},
{5.99893,15.1863,2.18287},{6.01489,15.1863,2.73339},{5.99893,15.1863,2.71744},
{5.99893,15.1863,2.71744},{6.01489,15.1863,2.73339},{0.0161388,15.1863,2.71744},
{0.0155184,15.1863,2.16692},{0.000185323,15.1863,1.8288},{0.0155184,15.1863,1.8288},
{0.000185323,15.1863,1.8288},{0.0155184,15.1863,2.16692},{0.000185323,15.1863,2.73339},
{0.000185323,15.1863,2.73339},{0.0155184,15.1863,2.16692},{0.0161388,15.1863,2.18287},
{0.0161388,15.1863,2.18287},{0.0155184,15.1863,2.16692},{5.99893,15.1863,2.18287},
{0.000185323,15.1863,2.73339},{0.0161388,15.1863,2.18287},{0.0161388,15.1863,2.71744},
{0.000185323,15.1863,2.73339},{0.0161388,15.1863,2.71744},{6.01489,15.1863,2.73339},
{6.08984,15.246,2.16692},{0.0155184,15.1863,2.16692},{0.0155184,15.246,2.16692},
{0.0155184,15.1863,2.16692},{6.08984,15.246,2.16692},{6.01489,15.1863,2.16692},
{6.01489,15.1863,2.16692},{6.08984,15.246,2.16692},{6.01489,15.0402,2.16692},
{6.01489,15.0402,2.16692},{6.08984,15.246,2.16692},{6.08984,15.0402,2.16692},
{6.08984,12.6933,1.8288},{6.01489,15.0402,2.16692},{6.01489,12.6933,1.8288},
{6.01489,15.0402,2.16692},{6.08984,12.6933,1.8288},{6.08984,15.0402,2.16692},
{6.01489,-0.000313449,0.904595},{6.01489,0.0333246,0.875455},{6.01489,-0.000313449,-2.58041e-031},
{6.01489,0.0333246,0.875455},{6.01489,-0.000313449,0.904595},{6.01489,12.6933,2.73339},
{6.01489,0.0333246,0.875455},{6.01489,12.6933,2.73339},{6.01489,12.6597,2.69456},
{6.01489,12.6597,2.69456},{6.01489,12.6933,2.73339},{6.01489,12.6597,1.85794},
{6.01489,-0.000313449,-2.58041e-031},{6.01489,0.0333246,0.0388317},{6.01489,12.6933,1.8288},
{6.01489,0.0333246,0.0388317},{6.01489,-0.000313449,-2.58041e-031},{6.01489,0.0333246,0.875455},
{6.01489,12.6933,1.8288},{6.01489,0.0333246,0.0388317},{6.01489,12.6597,1.85794},
{6.01489,12.6933,1.8288},{6.01489,12.6597,1.85794},{6.01489,12.6933,2.73339},
{6.01489,12.6933,1.8288},{6.01489,12.6933,2.73339},{6.01489,12.7268,2.69996},
{6.01489,12.7268,2.69996},{6.01489,12.6933,2.73339},{6.01489,15.1863,2.73339},
{6.01489,12.7268,2.69996},{6.01489,15.1863,2.73339},{6.01489,15.1528,2.69996},
{6.01489,15.1528,2.69996},{6.01489,15.1863,2.73339},{6.01489,15.1528,2.20036},
{6.01489,12.6933,1.8288},{6.01489,12.7268,1.8674},{6.01489,15.0402,2.16692},
{6.01489,12.7268,1.8674},{6.01489,12.6933,1.8288},{6.01489,12.7268,2.69996},
{6.01489,15.0402,2.16692},{6.01489,12.7268,1.8674},{6.01489,15.0378,2.20036},
{6.01489,15.0402,2.16692},{6.01489,15.0378,2.20036},{6.01489,15.1528,2.20036},
{6.01489,15.0402,2.16692},{6.01489,15.1528,2.20036},{6.01489,15.1863,2.16692},
{6.01489,15.1863,2.16692},{6.01489,15.1528,2.20036},{6.01489,15.1863,2.73339},
{6.08984,-0.000313449,0},{6.01489,-0.000313449,0.904595},{6.01489,-0.000313449,-2.58041e-031},
{6.01489,-0.000313449,0.904595},{6.08984,-0.000313449,0},{6.08984,-0.000313449,0.0388317},
{6.01489,-0.000313449,0.904595},{6.08984,-0.000313449,0.0388317},{6.08984,-0.000313449,0.904595},
{6.08984,12.6933,2.73339},{6.01489,15.1863,2.73339},{6.01489,12.6933,2.73339},
{6.01489,15.1863,2.73339},{0.000185323,15.246,2.73339},{0.000185323,15.1863,2.73339},
{0.000185323,15.246,2.73339},{6.01489,15.1863,2.73339},{6.08984,15.246,2.73339},
{6.08984,15.246,2.73339},{6.01489,15.1863,2.73339},{6.08984,12.6933,2.73339},
{6.08984,-0.000313449,0.904595},{6.01489,12.6933,2.73339},{6.01489,-0.000313449,0.904595},
{6.01489,12.6933,2.73339},{6.08984,-0.000313449,0.904595},{6.08984,12.6933,2.73339},
{6.08984,-0.000313449,0},{6.08984,0.0333246,0.0388317},{6.08984,-0.000313449,0.0388317},
{6.08984,0.0333246,0.0388317},{6.08984,-0.000313449,0},{6.08984,15.246,0},
{6.08984,0.0333246,0.0388317},{6.08984,15.246,0},{6.08984,12.6933,1.8288},
{6.08984,0.0333246,0.0388317},{6.08984,12.6933,1.8288},{6.08984,12.6597,1.85794},
{6.08984,12.6933,1.8288},{6.08984,15.246,0},{6.08984,15.246,1.8288},
{6.08984,0.0333246,0.0388317},{6.01489,12.6597,1.85794},{6.01489,0.0333246,0.0388317},
{6.01489,12.6597,1.85794},{6.08984,0.0333246,0.0388317},{6.08984,12.6597,1.85794},
{6.01489,0.0333246,0.0388317},{6.08984,0.0333246,0.875455},{6.08984,0.0333246,0.0388317},
{6.08984,0.0333246,0.875455},{6.01489,0.0333246,0.0388317},{6.01489,0.0333246,0.875455},
{6.08984,12.6597,2.69456},{6.01489,0.0333246,0.875455},{6.01489,12.6597,2.69456},
{6.01489,0.0333246,0.875455},{6.08984,12.6597,2.69456},{6.08984,0.0333246,0.875455},
{6.08984,12.6597,2.69456},{6.01489,12.6597,1.85794},{6.08984,12.6597,1.85794},
{6.01489,12.6597,1.85794},{6.08984,12.6597,2.69456},{6.01489,12.6597,2.69456},
{6.08984,15.1528,2.69996},{6.01489,15.1528,2.20036},{6.08984,15.1528,2.20036},
{6.01489,15.1528,2.20036},{6.08984,15.1528,2.69996},{6.01489,15.1528,2.69996},
{6.08984,15.0378,2.20036},{6.01489,15.1528,2.20036},{6.01489,15.0378,2.20036},
{6.01489,15.1528,2.20036},{6.08984,15.0378,2.20036},{6.08984,15.1528,2.20036},
{6.08984,12.7268,1.8674},{6.01489,15.0378,2.20036},{6.01489,12.7268,1.8674},
{6.01489,15.0378,2.20036},{6.08984,12.7268,1.8674},{6.08984,15.0378,2.20036},
{6.01489,12.7268,2.69996},{6.08984,12.7268,1.8674},{6.01489,12.7268,1.8674},
{6.08984,12.7268,1.8674},{6.01489,12.7268,2.69996},{6.08984,12.7268,2.69996},
{6.08984,15.1528,2.69996},{6.01489,12.7268,2.69996},{6.01489,15.1528,2.69996},
{6.01489,12.7268,2.69996},{6.08984,15.1528,2.69996},{6.08984,12.7268,2.69996},
{0.000185323,15.246,2.73339},{0.000185323,15.1863,1.8288},{0.000185323,15.1863,2.73339},
{0.000185323,15.1863,1.8288},{0.000185323,15.246,2.73339},{0.000185323,15.246,1.8288},
{0.0155184,15.246,1.8288},{0.0155184,15.1863,2.16692},{0.0155184,15.1863,1.8288},
{0.0155184,15.1863,2.16692},{0.0155184,15.246,1.8288},{0.0155184,15.246,2.16692},
{5.99893,15.1863,2.18287},{0.0161388,15.246,2.18287},{0.0161388,15.1863,2.18287},
{0.0161388,15.246,2.18287},{5.99893,15.1863,2.18287},{5.99893,15.246,2.18287},
{5.99893,15.246,2.71744},{5.99893,15.1863,2.18287},{5.99893,15.1863,2.71744},
{5.99893,15.1863,2.18287},{5.99893,15.246,2.71744},{5.99893,15.246,2.18287},
{5.99893,15.246,2.71744},{0.0161388,15.1863,2.71744},{0.0161388,15.246,2.71744},
{0.0161388,15.1863,2.71744},{5.99893,15.246,2.71744},{5.99893,15.1863,2.71744},
{0.0161388,15.246,2.18287},{0.0161388,15.1863,2.71744},{0.0161388,15.1863,2.18287},
{0.0161388,15.1863,2.71744},{0.0161388,15.246,2.18287},{0.0161388,15.246,2.71744},
{2.97125,-0.000313449,3.6576},{0.000185323,12.6933,1.8288},{0.000185323,-0.000313449,3.6576},
{0.000185323,12.6933,1.8288},{2.97125,-0.000313449,3.6576},{2.97125,12.6933,1.8288},
{2.97125,12.6933,2.72415},{3.04184,12.6933,1.8288},{2.97125,12.6933,1.8288},
{3.04184,12.6933,1.8288},{2.97125,12.6933,2.72415},{3.04184,12.6933,2.72415},
{3.04184,-0.000313449,3.69609},{2.97125,-0.000313449,3.6576},{3.04184,-0.000313449,3.6576},
{2.97125,-0.000313449,3.6576},{3.04184,-0.000313449,3.69609},{2.97125,-0.000313449,4.55295},
{2.97125,-0.000313449,4.55295},{3.04184,-0.000313449,3.69609},{3.04184,-0.000313449,4.55295},
{2.97125,-0.000313449,4.55295},{2.97125,0.0441143,4.50166},{2.97125,-0.000313449,3.6576},
{2.97125,0.0441143,4.50166},{2.97125,-0.000313449,4.55295},{2.97125,12.6933,2.72415},
{2.97125,0.0441143,4.50166},{2.97125,12.6933,2.72415},{2.97125,12.6489,2.68566},
{2.97125,12.6489,2.68566},{2.97125,12.6933,2.72415},{2.97125,12.6489,1.88009},
{2.97125,-0.000313449,3.6576},{2.97125,0.0441143,3.69609},{2.97125,12.6933,1.8288},
{2.97125,0.0441143,3.69609},{2.97125,-0.000313449,3.6576},{2.97125,0.0441143,4.50166},
{2.97125,12.6933,1.8288},{2.97125,0.0441143,3.69609},{2.97125,12.6489,1.88009},
{2.97125,12.6933,1.8288},{2.97125,12.6489,1.88009},{2.97125,12.6933,2.72415},
{3.04184,-0.000313449,4.55295},{2.97125,12.6933,2.72415},{2.97125,-0.000313449,4.55295},
{2.97125,12.6933,2.72415},{3.04184,-0.000313449,4.55295},{3.04184,12.6933,2.72415},
{3.04184,12.6489,2.68566},{2.97125,0.0441143,4.50166},{2.97125,12.6489,2.68566},
{2.97125,0.0441143,4.50166},{3.04184,12.6489,2.68566},{3.04184,0.0441143,4.50166},
{3.04184,12.6489,2.68566},{2.97125,12.6489,1.88009},{3.04184,12.6489,1.88009},
{2.97125,12.6489,1.88009},{3.04184,12.6489,2.68566},{2.97125,12.6489,2.68566},
{3.04184,0.0441143,3.69609},{2.97125,12.6489,1.88009},{2.97125,0.0441143,3.69609},
{2.97125,12.6489,1.88009},{3.04184,0.0441143,3.69609},{3.04184,12.6489,1.88009},
{2.97125,0.0441143,4.50166},{3.04184,0.0441143,3.69609},{2.97125,0.0441143,3.69609},
{3.04184,0.0441143,3.69609},{2.97125,0.0441143,4.50166},{3.04184,0.0441143,4.50166},
{3.04184,-0.000313449,3.6576},{0.000185323,-0.000313449,3.3922},{3.04184,-0.000313449,3.3922},
{0.000185323,-0.000313449,3.3922},{3.04184,-0.000313449,3.6576},{0.000185323,-0.000313449,3.6576},
{3.04184,-0.000313449,3.3922},{0.000185323,-0.000313449,0},{3.04184,-0.000313449,0},
{0.000185323,-0.000313449,0},{3.04184,-0.000313449,3.3922},{0.000185323,-0.000313449,3.3922},
{6.08984,0.0333246,0.0388317},{6.08984,12.6597,2.69456},{6.08984,0.0333246,0.875455},
{6.08984,12.6597,2.69456},{6.08984,0.0333246,0.0388317},{6.08984,12.6597,1.85794},
{6.08984,15.0378,2.20036},{6.08984,12.7268,2.69996},{6.08984,12.7268,1.8674},
{6.08984,12.7268,2.69996},{6.08984,15.0378,2.20036},{6.08984,15.1528,2.69996},
{6.08984,15.1528,2.69996},{6.08984,15.0378,2.20036},{6.08984,15.1528,2.20036},
{0.0161388,15.246,2.71744},{5.99893,15.246,2.18287},{0.0161388,15.246,2.18287},
{5.99893,15.246,2.18287},{0.0161388,15.246,2.71744},{5.99893,15.246,2.71744},
{3.04184,0.0441143,3.69609},{3.04184,12.6489,2.68566},{3.04184,0.0441143,4.50166},
{3.04184,12.6489,2.68566},{3.04184,0.0441143,3.69609},{3.04184,12.6489,1.88009},
{3.04184,0.0441143,3.69609},{3.04184,0.0441143,4.50166},{3.04184,-0.000313449,3.69609},
{3.04184,12.6933,1.8288},{3.04184,12.6489,1.88009},{3.04184,12.6489,1.8224},
{3.04184,12.6489,1.88009},{3.04184,12.6933,1.8288},{3.04184,12.6489,2.68566},
{3.04184,-0.000313449,4.55295},{3.04184,0.0441143,4.50166},{3.04184,12.6933,2.72415},
{3.04184,0.0441143,4.50166},{3.04184,-0.000313449,4.55295},{3.04184,-0.000313449,3.69609},
{3.04184,12.6933,2.72415},{3.04184,0.0441143,4.50166},{3.04184,12.6489,2.68566},
{3.04184,12.6933,2.72415},{3.04184,12.6489,2.68566},{3.04184,12.6933,1.8288},
{6.08984,-0.000313449,0.904595},{6.08984,0.0333246,0.875455},{6.08984,12.6933,2.73339},
{6.08984,0.0333246,0.875455},{6.08984,-0.000313449,0.904595},{6.08984,-0.000313449,0.0388317},
{6.08984,12.6933,2.73339},{6.08984,0.0333246,0.875455},{6.08984,12.6597,2.69456},
{6.08984,12.6933,2.73339},{6.08984,12.6597,2.69456},{6.08984,12.6933,1.8288},
{6.08984,12.6933,2.73339},{6.08984,12.6933,1.8288},{6.08984,12.7268,1.8674},
{6.08984,12.7268,1.8674},{6.08984,12.6933,1.8288},{6.08984,15.0402,2.16692},
{6.08984,12.7268,1.8674},{6.08984,15.0402,2.16692},{6.08984,15.0378,2.20036},
{6.08984,15.0378,2.20036},{6.08984,15.0402,2.16692},{6.08984,15.1528,2.20036},
{6.08984,15.1528,2.20036},{6.08984,15.0402,2.16692},{6.08984,15.246,2.16692},
{6.08984,15.1528,2.20036},{6.08984,15.246,2.16692},{6.08984,15.1528,2.69996},
{6.08984,12.6933,2.73339},{6.08984,12.7268,2.69996},{6.08984,15.246,2.73339},
{6.08984,12.7268,2.69996},{6.08984,12.6933,2.73339},{6.08984,12.7268,1.8674},
{6.08984,15.246,2.73339},{6.08984,12.7268,2.69996},{6.08984,15.1528,2.69996},
{6.08984,15.246,2.73339},{6.08984,15.1528,2.69996},{6.08984,15.246,2.16692},
{6.08984,0.0333246,0.0388317},{6.08984,0.0333246,0.875455},{6.08984,-0.000313449,0.0388317},
{6.08984,12.6933,1.8288},{6.08984,12.6597,2.69456},{6.08984,12.6597,1.85794}};
MeshPart *aux=new MeshPart;
for(int i=0;i<num;i++){
	aux->addTriangle(Vector3D(v[i*3][0],v[i*3][1],v[i*3][2]),Vector3D(v[i*3+1][0],v[i*3+1][1],v[i*3+1][2]),
	                 Vector3D(v[i*3+2][0],v[i*3+2][1],v[i*3+2][2]));
}
return aux;
}
//Automatic code created by MeshPart, in order to code in mrcode
//a mesh so it is posible to create it easily by code
//It is recomended to change the name of the created generic function
//just call: MeshPart *aux=thisfunction(); in your code

MeshPart *createWoodBench(void)
{
const int num=432;
const double v[][3]={
{1.62909,0.555356,1.18439e-016},{1.54691,0.702132,0.938257},{1.54691,0.555356,5.63993e-018},
{1.54691,0.702132,0.938257},{1.62909,0.555356,1.18439e-016},{1.62909,0.702132,0.938257},
{1.54691,0.555356,5.63993e-018},{1.54691,0.789964,0.924517},{1.54691,0.645338,2.06138e-016},
{1.54691,0.789964,0.924517},{1.54691,0.555356,5.63993e-018},{1.54691,0.787515,0.947396},
{1.54691,0.787515,0.947396},{1.54691,0.555356,5.63993e-018},{1.54691,0.773956,0.965985},
{1.54691,0.773956,0.965985},{1.54691,0.555356,5.63993e-018},{1.54691,0.752918,0.975303},
{1.54691,0.752918,0.975303},{1.54691,0.555356,5.63993e-018},{1.54691,0.73004,0.972855},
{1.54691,0.73004,0.972855},{1.54691,0.555356,5.63993e-018},{1.54691,0.702132,0.938257},
{1.54691,0.73004,0.972855},{1.54691,0.702132,0.938257},{1.54691,0.711451,0.959295},
{1.54691,0.787515,0.947396},{1.62909,0.789964,0.924517},{1.54691,0.789964,0.924517},
{1.62909,0.789964,0.924517},{1.54691,0.787515,0.947396},{1.62909,0.787515,0.947396},
{1.62909,0.787515,0.947396},{1.54691,0.773956,0.965985},{1.62909,0.773956,0.965985},
{1.54691,0.773956,0.965985},{1.62909,0.787515,0.947396},{1.54691,0.787515,0.947396},
{1.62909,0.773956,0.965985},{1.54691,0.752918,0.975303},{1.62909,0.752918,0.975303},
{1.54691,0.752918,0.975303},{1.62909,0.773956,0.965985},{1.54691,0.773956,0.965985},
{1.62909,0.752918,0.975303},{1.54691,0.73004,0.972855},{1.62909,0.73004,0.972855},
{1.54691,0.73004,0.972855},{1.62909,0.752918,0.975303},{1.54691,0.752918,0.975303},
{1.62909,0.711451,0.959295},{1.54691,0.73004,0.972855},{1.54691,0.711451,0.959295},
{1.54691,0.73004,0.972855},{1.62909,0.711451,0.959295},{1.62909,0.73004,0.972855},
{1.62909,0.702132,0.938257},{1.54691,0.711451,0.959295},{1.54691,0.702132,0.938257},
{1.54691,0.711451,0.959295},{1.62909,0.702132,0.938257},{1.62909,0.711451,0.959295},
{1.54691,0.789964,0.924517},{1.62909,0.645338,1.65585e-016},{1.54691,0.645338,2.06138e-016},
{1.62909,0.645338,1.65585e-016},{1.54691,0.789964,0.924517},{1.62909,0.789964,0.924517},
{1.62909,0.789964,0.924517},{1.62909,0.555356,1.18439e-016},{1.62909,0.645338,1.65585e-016},
{1.62909,0.555356,1.18439e-016},{1.62909,0.789964,0.924517},{1.62909,0.787515,0.947396},
{1.62909,0.555356,1.18439e-016},{1.62909,0.787515,0.947396},{1.62909,0.773956,0.965985},
{1.62909,0.555356,1.18439e-016},{1.62909,0.773956,0.965985},{1.62909,0.752918,0.975303},
{1.62909,0.555356,1.18439e-016},{1.62909,0.752918,0.975303},{1.62909,0.73004,0.972855},
{1.62909,0.555356,1.18439e-016},{1.62909,0.73004,0.972855},{1.62909,0.702132,0.938257},
{1.62909,0.702132,0.938257},{1.62909,0.73004,0.972855},{1.62909,0.711451,0.959295},
{1.54691,0.555356,5.63993e-018},{1.62909,0.645338,1.65585e-016},{1.62909,0.555356,1.18439e-016},
{1.62909,0.645338,1.65585e-016},{1.54691,0.555356,5.63993e-018},{1.54691,0.645338,2.06138e-016},
{1.62909,0.644423,0.569354},{1.54691,0.171576,0.569354},{1.54691,0.644423,0.569354},
{1.54691,0.171576,0.569354},{1.62909,0.644423,0.569354},{1.62909,0.171576,0.569354},
{1.54691,0.171576,9.36009e-015},{1.62909,0.171576,0.569354},{1.62909,0.171576,9.00142e-015},
{1.62909,0.171576,0.569354},{1.54691,0.171576,9.36009e-015},{1.54691,0.171576,0.569354},
{1.62909,0.0953757,9.04152e-015},{1.62909,0.171576,0.569354},{1.62909,0.0826757,0.569354},
{1.62909,0.171576,0.569354},{1.62909,0.0953757,9.04152e-015},{1.62909,0.171576,9.00142e-015},
{1.54691,0.171576,9.36009e-015},{1.62909,0.0953757,9.04152e-015},{1.54691,0.0953757,9.06135e-015},
{1.62909,0.0953757,9.04152e-015},{1.54691,0.171576,9.36009e-015},{1.62909,0.171576,9.00142e-015},
{1.54691,0.171576,0.569354},{1.54691,0.0953757,9.06135e-015},{1.54691,0.0826757,0.569354},
{1.54691,0.0953757,9.06135e-015},{1.54691,0.171576,0.569354},{1.54691,0.171576,9.36009e-015},
{1.54691,0.04445,0.658254},{1.62909,0.0826131,0.658254},{1.54691,0.0826131,0.658254},
{1.62909,0.0826131,0.658254},{1.54691,0.04445,0.658254},{1.62909,0.04445,0.658254},
{1.62909,0.04445,0.569354},{1.54691,0.022225,0.575309},{1.54691,0.04445,0.569354},
{1.54691,0.022225,0.575309},{1.62909,0.04445,0.569354},{1.62909,0.022225,0.575309},
{1.62909,0.00595517,0.636029},{1.54691,0,0.613804},{1.62909,1.48894e-015,0.613804},
{1.54691,0,0.613804},{1.62909,0.00595517,0.636029},{1.54691,0.00595517,0.636029},
{1.62909,1.48894e-015,0.613804},{1.54691,0.00595517,0.591579},{1.62909,0.00595517,0.591579},
{1.54691,0.00595517,0.591579},{1.62909,1.48894e-015,0.613804},{1.54691,0,0.613804},
{1.62909,0.0826757,0.569354},{1.54691,0.04445,0.569354},{1.54691,0.0826757,0.569354},
{1.54691,0.04445,0.569354},{1.62909,0.0826757,0.569354},{1.62909,0.04445,0.569354},
{1.62909,0.00595517,0.636029},{1.54691,0.022225,0.652298},{1.54691,0.00595517,0.636029},
{1.54691,0.022225,0.652298},{1.62909,0.00595517,0.636029},{1.62909,0.022225,0.652298},
{1.54691,0.00595517,0.636029},{1.54691,0.00595517,0.591579},{1.54691,0,0.613804},
{1.54691,0.00595517,0.591579},{1.54691,0.00595517,0.636029},{1.54691,0.022225,0.652298},
{1.54691,0.00595517,0.591579},{1.54691,0.022225,0.652298},{1.54691,0.022225,0.575309},
{1.54691,0.022225,0.575309},{1.54691,0.022225,0.652298},{1.54691,0.04445,0.569354},
{1.54691,0.04445,0.569354},{1.54691,0.022225,0.652298},{1.54691,0.04445,0.658254},
{1.54691,0.04445,0.569354},{1.54691,0.04445,0.658254},{1.54691,0.0826131,0.658254},
{1.54691,0.04445,0.569354},{1.54691,0.0826131,0.658254},{1.54691,0.0826757,0.569354},
{1.54691,0.0826757,0.569354},{1.54691,0.0826131,0.658254},{1.54691,0.409579,0.653001},
{1.54691,0.0826757,0.569354},{1.54691,0.409579,0.653001},{1.54691,0.171576,0.569354},
{1.54691,0.171576,0.569354},{1.54691,0.409579,0.653001},{1.54691,0.644423,0.569354},
{1.54691,0.644423,0.569354},{1.54691,0.409579,0.653001},{1.54691,0.65833,0.658254},
{1.54691,0.022225,0.652298},{1.62909,0.04445,0.658254},{1.54691,0.04445,0.658254},
{1.62909,0.04445,0.658254},{1.54691,0.022225,0.652298},{1.62909,0.022225,0.652298},
{1.62909,0.00595517,0.591579},{1.54691,0.022225,0.575309},{1.62909,0.022225,0.575309},
{1.54691,0.022225,0.575309},{1.62909,0.00595517,0.591579},{1.54691,0.00595517,0.591579},
{1.54691,0.0826131,0.658254},{1.62909,0.409579,0.653001},{1.54691,0.409579,0.653001},
{1.62909,0.409579,0.653001},{1.54691,0.0826131,0.658254},{1.62909,0.0826131,0.658254},
{1.54691,0.409579,0.653001},{1.62909,0.65833,0.658254},{1.54691,0.65833,0.658254},
{1.62909,0.65833,0.658254},{1.54691,0.409579,0.653001},{1.62909,0.409579,0.653001},
{1.62909,0.0826757,0.569354},{1.54691,0.0953757,9.06135e-015},{1.62909,0.0953757,9.04152e-015},
{1.54691,0.0953757,9.06135e-015},{1.62909,0.0826757,0.569354},{1.54691,0.0826757,0.569354},
{1.62909,0.00595517,0.591579},{1.62909,0.00595517,0.636029},{1.62909,1.48894e-015,0.613804},
{1.62909,0.00595517,0.636029},{1.62909,0.00595517,0.591579},{1.62909,0.022225,0.652298},
{1.62909,0.022225,0.652298},{1.62909,0.00595517,0.591579},{1.62909,0.022225,0.575309},
{1.62909,0.022225,0.652298},{1.62909,0.022225,0.575309},{1.62909,0.04445,0.569354},
{1.62909,0.022225,0.652298},{1.62909,0.04445,0.569354},{1.62909,0.04445,0.658254},
{1.62909,0.04445,0.658254},{1.62909,0.04445,0.569354},{1.62909,0.0826131,0.658254},
{1.62909,0.0826131,0.658254},{1.62909,0.04445,0.569354},{1.62909,0.0826757,0.569354},
{1.62909,0.0826131,0.658254},{1.62909,0.0826757,0.569354},{1.62909,0.409579,0.653001},
{1.62909,0.409579,0.653001},{1.62909,0.0826757,0.569354},{1.62909,0.171576,0.569354},
{1.62909,0.409579,0.653001},{1.62909,0.171576,0.569354},{1.62909,0.644423,0.569354},
{1.62909,0.409579,0.653001},{1.62909,0.644423,0.569354},{1.62909,0.65833,0.658254},
{1.6038,0.171576,0.304908},{1.55955,0.171576,0.400158},{1.55955,0.171576,0.304908},
{1.55955,0.171576,0.400158},{1.6038,0.171576,0.304908},{1.6038,0.171576,0.400158},
{1.55955,0.617955,0.400158},{1.55955,0.171576,0.304908},{1.55955,0.171576,0.400158},
{1.55955,0.171576,0.304908},{1.55955,0.617955,0.400158},{1.55955,0.603055,0.304908},
{1.6038,0.603055,0.304908},{1.6038,0.171576,0.400158},{1.6038,0.171576,0.304908},
{1.6038,0.171576,0.400158},{1.6038,0.603055,0.304908},{1.6038,0.617955,0.400158},
{1.55955,0.603055,0.304908},{1.6038,0.171576,0.304908},{1.55955,0.171576,0.304908},
{1.6038,0.171576,0.304908},{1.55955,0.603055,0.304908},{1.6038,0.603055,0.304908},
{1.6038,0.171576,0.400158},{1.55955,0.617955,0.400158},{1.55955,0.171576,0.400158},
{1.55955,0.617955,0.400158},{1.6038,0.171576,0.400158},{1.6038,0.617955,0.400158},
{0.0917091,0.555356,1.18439e-016},{0.009525,0.702132,0.938257},{0.009525,0.555356,5.63993e-018},
{0.009525,0.702132,0.938257},{0.0917091,0.555356,1.18439e-016},{0.0917091,0.702132,0.938257},
{0.009525,0.555356,5.63993e-018},{0.009525,0.789964,0.924517},{0.009525,0.645338,2.06138e-016},
{0.009525,0.789964,0.924517},{0.009525,0.555356,5.63993e-018},{0.009525,0.787515,0.947396},
{0.009525,0.787515,0.947396},{0.009525,0.555356,5.63993e-018},{0.009525,0.773956,0.965985},
{0.009525,0.773956,0.965985},{0.009525,0.555356,5.63993e-018},{0.009525,0.752918,0.975303},
{0.009525,0.752918,0.975303},{0.009525,0.555356,5.63993e-018},{0.009525,0.73004,0.972855},
{0.009525,0.73004,0.972855},{0.009525,0.555356,5.63993e-018},{0.009525,0.702132,0.938257},
{0.009525,0.73004,0.972855},{0.009525,0.702132,0.938257},{0.009525,0.711451,0.959295},
{0.009525,0.787515,0.947396},{0.0917091,0.789964,0.924517},{0.009525,0.789964,0.924517},
{0.0917091,0.789964,0.924517},{0.009525,0.787515,0.947396},{0.0917091,0.787515,0.947396},
{0.0917091,0.787515,0.947396},{0.009525,0.773956,0.965985},{0.0917091,0.773956,0.965985},
{0.009525,0.773956,0.965985},{0.0917091,0.787515,0.947396},{0.009525,0.787515,0.947396},
{0.0917091,0.773956,0.965985},{0.009525,0.752918,0.975303},{0.0917091,0.752918,0.975303},
{0.009525,0.752918,0.975303},{0.0917091,0.773956,0.965985},{0.009525,0.773956,0.965985},
{0.0917091,0.752918,0.975303},{0.009525,0.73004,0.972855},{0.0917091,0.73004,0.972855},
{0.009525,0.73004,0.972855},{0.0917091,0.752918,0.975303},{0.009525,0.752918,0.975303},
{0.0917091,0.711451,0.959295},{0.009525,0.73004,0.972855},{0.009525,0.711451,0.959295},
{0.009525,0.73004,0.972855},{0.0917091,0.711451,0.959295},{0.0917091,0.73004,0.972855},
{0.0917091,0.702132,0.938257},{0.009525,0.711451,0.959295},{0.009525,0.702132,0.938257},
{0.009525,0.711451,0.959295},{0.0917091,0.702132,0.938257},{0.0917091,0.711451,0.959295},
{0.009525,0.789964,0.924517},{0.0917091,0.645338,1.65585e-016},{0.009525,0.645338,2.06138e-016},
{0.0917091,0.645338,1.65585e-016},{0.009525,0.789964,0.924517},{0.0917091,0.789964,0.924517},
{0.0917091,0.789964,0.924517},{0.0917091,0.555356,1.18439e-016},{0.0917091,0.645338,1.65585e-016},
{0.0917091,0.555356,1.18439e-016},{0.0917091,0.789964,0.924517},{0.0917091,0.787515,0.947396},
{0.0917091,0.555356,1.18439e-016},{0.0917091,0.787515,0.947396},{0.0917091,0.773956,0.965985},
{0.0917091,0.555356,1.18439e-016},{0.0917091,0.773956,0.965985},{0.0917091,0.752918,0.975303},
{0.0917091,0.555356,1.18439e-016},{0.0917091,0.752918,0.975303},{0.0917091,0.73004,0.972855},
{0.0917091,0.555356,1.18439e-016},{0.0917091,0.73004,0.972855},{0.0917091,0.702132,0.938257},
{0.0917091,0.702132,0.938257},{0.0917091,0.73004,0.972855},{0.0917091,0.711451,0.959295},
{0.009525,0.555356,5.63993e-018},{0.0917091,0.645338,1.65585e-016},{0.0917091,0.555356,1.18439e-016},
{0.0917091,0.645338,1.65585e-016},{0.009525,0.555356,5.63993e-018},{0.009525,0.645338,2.06138e-016},
{0.0917091,0.644423,0.569354},{0.009525,0.171576,0.569354},{0.009525,0.644423,0.569354},
{0.009525,0.171576,0.569354},{0.0917091,0.644423,0.569354},{0.0917091,0.171576,0.569354},
{0.009525,0.171576,9.36009e-015},{0.0917091,0.171576,0.569354},{0.0917091,0.171576,9.00142e-015},
{0.0917091,0.171576,0.569354},{0.009525,0.171576,9.36009e-015},{0.009525,0.171576,0.569354},
{0.0917091,0.0953757,9.04152e-015},{0.0917091,0.171576,0.569354},{0.0917091,0.0826757,0.569354},
{0.0917091,0.171576,0.569354},{0.0917091,0.0953757,9.04152e-015},{0.0917091,0.171576,9.00142e-015},
{0.009525,0.171576,9.36009e-015},{0.0917091,0.0953757,9.04152e-015},{0.009525,0.0953757,9.06135e-015},
{0.0917091,0.0953757,9.04152e-015},{0.009525,0.171576,9.36009e-015},{0.0917091,0.171576,9.00142e-015},
{0.009525,0.171576,0.569354},{0.009525,0.0953757,9.06135e-015},{0.009525,0.0826757,0.569354},
{0.009525,0.0953757,9.06135e-015},{0.009525,0.171576,0.569354},{0.009525,0.171576,9.36009e-015},
{0.009525,0.04445,0.658254},{0.0917091,0.0826131,0.658254},{0.009525,0.0826131,0.658254},
{0.0917091,0.0826131,0.658254},{0.009525,0.04445,0.658254},{0.0917091,0.04445,0.658254},
{0.0917091,0.04445,0.569354},{0.009525,0.022225,0.575309},{0.009525,0.04445,0.569354},
{0.009525,0.022225,0.575309},{0.0917091,0.04445,0.569354},{0.0917091,0.022225,0.575309},
{0.0917091,0.00595517,0.636029},{0.009525,0,0.613804},{0.0917091,1.48894e-015,0.613804},
{0.009525,0,0.613804},{0.0917091,0.00595517,0.636029},{0.009525,0.00595517,0.636029},
{0.0917091,1.48894e-015,0.613804},{0.009525,0.00595517,0.591579},{0.0917091,0.00595517,0.591579},
{0.009525,0.00595517,0.591579},{0.0917091,1.48894e-015,0.613804},{0.009525,0,0.613804},
{0.0917091,0.0826757,0.569354},{0.009525,0.04445,0.569354},{0.009525,0.0826757,0.569354},
{0.009525,0.04445,0.569354},{0.0917091,0.0826757,0.569354},{0.0917091,0.04445,0.569354},
{0.0917091,0.00595517,0.636029},{0.009525,0.022225,0.652298},{0.009525,0.00595517,0.636029},
{0.009525,0.022225,0.652298},{0.0917091,0.00595517,0.636029},{0.0917091,0.022225,0.652298},
{0.009525,0.00595517,0.636029},{0.009525,0.00595517,0.591579},{0.009525,0,0.613804},
{0.009525,0.00595517,0.591579},{0.009525,0.00595517,0.636029},{0.009525,0.022225,0.652298},
{0.009525,0.00595517,0.591579},{0.009525,0.022225,0.652298},{0.009525,0.022225,0.575309},
{0.009525,0.022225,0.575309},{0.009525,0.022225,0.652298},{0.009525,0.04445,0.569354},
{0.009525,0.04445,0.569354},{0.009525,0.022225,0.652298},{0.009525,0.04445,0.658254},
{0.009525,0.04445,0.569354},{0.009525,0.04445,0.658254},{0.009525,0.0826131,0.658254},
{0.009525,0.04445,0.569354},{0.009525,0.0826131,0.658254},{0.009525,0.0826757,0.569354},
{0.009525,0.0826757,0.569354},{0.009525,0.0826131,0.658254},{0.009525,0.409579,0.653001},
{0.009525,0.0826757,0.569354},{0.009525,0.409579,0.653001},{0.009525,0.171576,0.569354},
{0.009525,0.171576,0.569354},{0.009525,0.409579,0.653001},{0.009525,0.644423,0.569354},
{0.009525,0.644423,0.569354},{0.009525,0.409579,0.653001},{0.009525,0.65833,0.658254},
{0.009525,0.022225,0.652298},{0.0917091,0.04445,0.658254},{0.009525,0.04445,0.658254},
{0.0917091,0.04445,0.658254},{0.009525,0.022225,0.652298},{0.0917091,0.022225,0.652298},
{0.0917091,0.00595517,0.591579},{0.009525,0.022225,0.575309},{0.0917091,0.022225,0.575309},
{0.009525,0.022225,0.575309},{0.0917091,0.00595517,0.591579},{0.009525,0.00595517,0.591579},
{0.009525,0.0826131,0.658254},{0.0917091,0.409579,0.653001},{0.009525,0.409579,0.653001},
{0.0917091,0.409579,0.653001},{0.009525,0.0826131,0.658254},{0.0917091,0.0826131,0.658254},
{0.009525,0.409579,0.653001},{0.0917091,0.65833,0.658254},{0.009525,0.65833,0.658254},
{0.0917091,0.65833,0.658254},{0.009525,0.409579,0.653001},{0.0917091,0.409579,0.653001},
{0.0917091,0.0826757,0.569354},{0.009525,0.0953757,9.06135e-015},{0.0917091,0.0953757,9.04152e-015},
{0.009525,0.0953757,9.06135e-015},{0.0917091,0.0826757,0.569354},{0.009525,0.0826757,0.569354},
{0.0917091,0.00595517,0.591579},{0.0917091,0.00595517,0.636029},{0.0917091,1.48894e-015,0.613804},
{0.0917091,0.00595517,0.636029},{0.0917091,0.00595517,0.591579},{0.0917091,0.022225,0.652298},
{0.0917091,0.022225,0.652298},{0.0917091,0.00595517,0.591579},{0.0917091,0.022225,0.575309},
{0.0917091,0.022225,0.652298},{0.0917091,0.022225,0.575309},{0.0917091,0.04445,0.569354},
{0.0917091,0.022225,0.652298},{0.0917091,0.04445,0.569354},{0.0917091,0.04445,0.658254},
{0.0917091,0.04445,0.658254},{0.0917091,0.04445,0.569354},{0.0917091,0.0826131,0.658254},
{0.0917091,0.0826131,0.658254},{0.0917091,0.04445,0.569354},{0.0917091,0.0826757,0.569354},
{0.0917091,0.0826131,0.658254},{0.0917091,0.0826757,0.569354},{0.0917091,0.409579,0.653001},
{0.0917091,0.409579,0.653001},{0.0917091,0.0826757,0.569354},{0.0917091,0.171576,0.569354},
{0.0917091,0.409579,0.653001},{0.0917091,0.171576,0.569354},{0.0917091,0.644423,0.569354},
{0.0917091,0.409579,0.653001},{0.0917091,0.644423,0.569354},{0.0917091,0.65833,0.658254},
{0.0664217,0.171576,0.304908},{0.0221687,0.171576,0.400158},{0.0221687,0.171576,0.304908},
{0.0221687,0.171576,0.400158},{0.0664217,0.171576,0.304908},{0.0664217,0.171576,0.400158},
{0.0221687,0.617955,0.400158},{0.0221687,0.171576,0.304908},{0.0221687,0.171576,0.400158},
{0.0221687,0.171576,0.304908},{0.0221687,0.617955,0.400158},{0.0221687,0.603055,0.304908},
{0.0664217,0.603055,0.304908},{0.0664217,0.171576,0.400158},{0.0664217,0.171576,0.304908},
{0.0664217,0.171576,0.400158},{0.0664217,0.603055,0.304908},{0.0664217,0.617955,0.400158},
{0.0221687,0.603055,0.304908},{0.0664217,0.171576,0.304908},{0.0221687,0.171576,0.304908},
{0.0664217,0.171576,0.304908},{0.0221687,0.603055,0.304908},{0.0664217,0.603055,0.304908},
{0.0664217,0.171576,0.400158},{0.0221687,0.617955,0.400158},{0.0221687,0.171576,0.400158},
{0.0221687,0.617955,0.400158},{0.0664217,0.171576,0.400158},{0.0664217,0.617955,0.400158},
{0,0.507006,0.446132},{0,0.600085,0.407051},{0,0.509948,0.400162},
{0,0.600085,0.407051},{0,0.507006,0.446132},{0,0.597143,0.453021},
{0,0.597143,0.453021},{1.63835,0.600085,0.407051},{0,0.600085,0.407051},
{1.63835,0.600085,0.407051},{0,0.597143,0.453021},{1.63835,0.597143,0.453021},
{1.63835,0.507006,0.446132},{0,0.509948,0.400162},{1.63835,0.509948,0.400162},
{0,0.509948,0.400162},{1.63835,0.507006,0.446132},{0,0.507006,0.446132},
{1.63835,0.600085,0.407051},{1.63835,0.507006,0.446132},{1.63835,0.509948,0.400162},
{1.63835,0.507006,0.446132},{1.63835,0.600085,0.407051},{1.63835,0.597143,0.453021},
{1.63835,0.600085,0.407051},{0,0.509948,0.400162},{0,0.600085,0.407051},
{0,0.509948,0.400162},{1.63835,0.600085,0.407051},{1.63835,0.509948,0.400162},
{1.63835,0.507006,0.446132},{0,0.597143,0.453021},{0,0.507006,0.446132},
{0,0.597143,0.453021},{1.63835,0.507006,0.446132},{1.63835,0.597143,0.453021},
{0,0.404601,0.446132},{0,0.494959,0.40005},{0,0.404601,0.40005},
{0,0.494959,0.40005},{0,0.404601,0.446132},{0,0.494959,0.446132},
{0,0.494959,0.446132},{1.63835,0.494959,0.40005},{0,0.494959,0.40005},
{1.63835,0.494959,0.40005},{0,0.494959,0.446132},{1.63835,0.494959,0.446132},
{1.63835,0.404601,0.446132},{0,0.404601,0.40005},{1.63835,0.404601,0.40005},
{0,0.404601,0.40005},{1.63835,0.404601,0.446132},{0,0.404601,0.446132},
{1.63835,0.494959,0.40005},{1.63835,0.404601,0.446132},{1.63835,0.404601,0.40005},
{1.63835,0.404601,0.446132},{1.63835,0.494959,0.40005},{1.63835,0.494959,0.446132},
{1.63835,0.494959,0.40005},{0,0.404601,0.40005},{0,0.494959,0.40005},
{0,0.404601,0.40005},{1.63835,0.494959,0.40005},{1.63835,0.404601,0.40005},
{1.63835,0.404601,0.446132},{0,0.494959,0.446132},{0,0.404601,0.446132},
{0,0.494959,0.446132},{1.63835,0.404601,0.446132},{1.63835,0.494959,0.446132},
{0,0.199791,0.459909},{0,0.286987,0.407051},{0,0.19685,0.413939},
{0,0.286987,0.407051},{0,0.199791,0.459909},{0,0.289929,0.453021},
{0,0.289929,0.453021},{1.63835,0.286987,0.407051},{0,0.286987,0.407051},
{1.63835,0.286987,0.407051},{0,0.289929,0.453021},{1.63835,0.289929,0.453021},
{1.63835,0.199791,0.459909},{0,0.19685,0.413939},{1.63835,0.19685,0.413939},
{0,0.19685,0.413939},{1.63835,0.199791,0.459909},{0,0.199791,0.459909},
{1.63835,0.286987,0.407051},{1.63835,0.199791,0.459909},{1.63835,0.19685,0.413939},
{1.63835,0.199791,0.459909},{1.63835,0.286987,0.407051},{1.63835,0.289929,0.453021},
{1.63835,0.286987,0.407051},{0,0.19685,0.413939},{0,0.286987,0.407051},
{0,0.19685,0.413939},{1.63835,0.286987,0.407051},{1.63835,0.19685,0.413939},
{1.63835,0.199791,0.459909},{0,0.289929,0.453021},{0,0.199791,0.459909},
{0,0.289929,0.453021},{1.63835,0.199791,0.459909},{1.63835,0.289929,0.453021},
{0,0.302196,0.453021},{0,0.389392,0.400162},{0,0.299255,0.407051},
{0,0.389392,0.400162},{0,0.302196,0.453021},{0,0.392334,0.446132},
{0,0.392334,0.446132},{1.63835,0.389392,0.400162},{0,0.389392,0.400162},
{1.63835,0.389392,0.400162},{0,0.392334,0.446132},{1.63835,0.392334,0.446132},
{1.63835,0.302196,0.453021},{0,0.299255,0.407051},{1.63835,0.299255,0.407051},
{0,0.299255,0.407051},{1.63835,0.302196,0.453021},{0,0.302196,0.453021},
{1.63835,0.389392,0.400162},{1.63835,0.302196,0.453021},{1.63835,0.299255,0.407051},
{1.63835,0.302196,0.453021},{1.63835,0.389392,0.400162},{1.63835,0.392334,0.446132},
{1.63835,0.389392,0.400162},{0,0.299255,0.407051},{0,0.389392,0.400162},
{0,0.299255,0.407051},{1.63835,0.389392,0.400162},{1.63835,0.299255,0.407051},
{1.63835,0.302196,0.453021},{0,0.392334,0.446132},{0,0.302196,0.453021},
{0,0.392334,0.446132},{1.63835,0.302196,0.453021},{1.63835,0.392334,0.446132},
{0.1651,0.688803,0.508},{0.1651,0.716211,0.889},{0.1651,0.6731,0.51145},
{0.1651,0.716211,0.889},{0.1651,0.688803,0.508},{0.1651,0.731915,0.88555},
{0.127,0.716211,0.889},{0.1651,0.731915,0.88555},{0.127,0.731915,0.88555},
{0.1651,0.731915,0.88555},{0.127,0.716211,0.889},{0.1651,0.716211,0.889},
{0.127,0.716211,0.889},{0.127,0.688803,0.508},{0.127,0.6731,0.51145},
{0.127,0.688803,0.508},{0.127,0.716211,0.889},{0.127,0.731915,0.88555},
{0.1651,0.688803,0.508},{0.127,0.6731,0.51145},{0.127,0.688803,0.508},
{0.127,0.6731,0.51145},{0.1651,0.688803,0.508},{0.1651,0.6731,0.51145},
{0.127,0.731915,0.88555},{0.1651,0.688803,0.508},{0.127,0.688803,0.508},
{0.1651,0.688803,0.508},{0.127,0.731915,0.88555},{0.1651,0.731915,0.88555},
{0.1651,0.716211,0.889},{0.127,0.6731,0.51145},{0.1651,0.6731,0.51145},
{0.127,0.6731,0.51145},{0.1651,0.716211,0.889},{0.127,0.716211,0.889},
{0.09144,0.632692,0.420868},{0.09144,0.689535,0.510036},{0.09144,0.675131,0.413751},
{0.09144,0.689535,0.510036},{0.09144,0.632692,0.420868},{0.09144,0.647095,0.517152},
{0.09144,0.647095,0.517152},{1.54691,0.689535,0.510036},{0.09144,0.689535,0.510036},
{1.54691,0.689535,0.510036},{0.09144,0.647095,0.517152},{1.54691,0.647095,0.517152},
{1.54691,0.632692,0.420868},{0.09144,0.675131,0.413751},{1.54691,0.675131,0.413751},
{0.09144,0.675131,0.413751},{1.54691,0.632692,0.420868},{0.09144,0.632692,0.420868},
{1.54691,0.689535,0.510036},{1.54691,0.632692,0.420868},{1.54691,0.675131,0.413751},
{1.54691,0.632692,0.420868},{1.54691,0.689535,0.510036},{1.54691,0.647095,0.517152},
{1.54691,0.689535,0.510036},{0.09144,0.675131,0.413751},{0.09144,0.689535,0.510036},
{0.09144,0.675131,0.413751},{1.54691,0.689535,0.510036},{1.54691,0.675131,0.413751},
{1.54691,0.632692,0.420868},{0.09144,0.647095,0.517152},{0.09144,0.632692,0.420868},
{0.09144,0.647095,0.517152},{1.54691,0.632692,0.420868},{1.54691,0.647095,0.517152},
{0.09144,0.697028,0.850937},{0.09144,0.753871,0.940105},{0.09144,0.739468,0.843821},
{0.09144,0.753871,0.940105},{0.09144,0.697028,0.850937},{0.09144,0.711432,0.947221},
{0.09144,0.711432,0.947221},{1.54691,0.753871,0.940105},{0.09144,0.753871,0.940105},
{1.54691,0.753871,0.940105},{0.09144,0.711432,0.947221},{1.54691,0.711432,0.947221},
{1.54691,0.697028,0.850937},{0.09144,0.739468,0.843821},{1.54691,0.739468,0.843821},
{0.09144,0.739468,0.843821},{1.54691,0.697028,0.850937},{0.09144,0.697028,0.850937},
{1.54691,0.753871,0.940105},{1.54691,0.697028,0.850937},{1.54691,0.739468,0.843821},
{1.54691,0.697028,0.850937},{1.54691,0.753871,0.940105},{1.54691,0.711432,0.947221},
{1.54691,0.753871,0.940105},{0.09144,0.739468,0.843821},{0.09144,0.753871,0.940105},
{0.09144,0.739468,0.843821},{1.54691,0.753871,0.940105},{1.54691,0.739468,0.843821},
{1.54691,0.697028,0.850937},{0.09144,0.711432,0.947221},{0.09144,0.697028,0.850937},
{0.09144,0.711432,0.947221},{1.54691,0.697028,0.850937},{1.54691,0.711432,0.947221},
{0.09144,0.0946866,0.31195},{0.09144,0.137655,0.409434},{0.09144,0.137655,0.31195},
{0.09144,0.137655,0.409434},{0.09144,0.0946866,0.31195},{0.09144,0.0946866,0.409434},
{0.09144,0.0946866,0.409434},{1.54691,0.137655,0.409434},{0.09144,0.137655,0.409434},
{1.54691,0.137655,0.409434},{0.09144,0.0946866,0.409434},{1.54691,0.0946866,0.409434},
{1.54691,0.0946866,0.31195},{0.09144,0.137655,0.31195},{1.54691,0.137655,0.31195},
{0.09144,0.137655,0.31195},{1.54691,0.0946866,0.31195},{0.09144,0.0946866,0.31195},
{1.54691,0.137655,0.409434},{1.54691,0.0946866,0.31195},{1.54691,0.137655,0.31195},
{1.54691,0.0946866,0.31195},{1.54691,0.137655,0.409434},{1.54691,0.0946866,0.409434},
{1.54691,0.137655,0.409434},{0.09144,0.137655,0.31195},{0.09144,0.137655,0.409434},
{0.09144,0.137655,0.31195},{1.54691,0.137655,0.409434},{1.54691,0.137655,0.31195},
{1.54691,0.0946866,0.31195},{0.09144,0.0946866,0.409434},{0.09144,0.0946866,0.31195},
{0.09144,0.0946866,0.409434},{1.54691,0.0946866,0.31195},{1.54691,0.0946866,0.409434},
{0.09144,0.60372,0.31195},{0.09144,0.646689,0.409434},{0.09144,0.646689,0.31195},
{0.09144,0.646689,0.409434},{0.09144,0.60372,0.31195},{0.09144,0.60372,0.409434},
{0.09144,0.60372,0.409434},{1.54691,0.646689,0.409434},{0.09144,0.646689,0.409434},
{1.54691,0.646689,0.409434},{0.09144,0.60372,0.409434},{1.54691,0.60372,0.409434},
{1.54691,0.60372,0.31195},{0.09144,0.646689,0.31195},{1.54691,0.646689,0.31195},
{0.09144,0.646689,0.31195},{1.54691,0.60372,0.31195},{0.09144,0.60372,0.31195},
{1.54691,0.646689,0.409434},{1.54691,0.60372,0.31195},{1.54691,0.646689,0.31195},
{1.54691,0.60372,0.31195},{1.54691,0.646689,0.409434},{1.54691,0.60372,0.409434},
{1.54691,0.646689,0.409434},{0.09144,0.646689,0.31195},{0.09144,0.646689,0.409434},
{0.09144,0.646689,0.31195},{1.54691,0.646689,0.409434},{1.54691,0.646689,0.31195},
{1.54691,0.60372,0.31195},{0.09144,0.60372,0.409434},{0.09144,0.60372,0.31195},
{0.09144,0.60372,0.409434},{1.54691,0.60372,0.31195},{1.54691,0.60372,0.409434},
{0.09144,0.0916892,0.454816},{0.09144,0.186537,0.416234},{0.09144,0.0946866,0.409434},
{0.09144,0.186537,0.416234},{0.09144,0.0916892,0.454816},{0.09144,0.18354,0.461616},
{0.09144,0.18354,0.461616},{1.54691,0.186537,0.416234},{0.09144,0.186537,0.416234},
{1.54691,0.186537,0.416234},{0.09144,0.18354,0.461616},{1.54691,0.18354,0.461616},
{1.54691,0.0916892,0.454816},{0.09144,0.0946866,0.409434},{1.54691,0.0946866,0.409434},
{0.09144,0.0946866,0.409434},{1.54691,0.0916892,0.454816},{0.09144,0.0916892,0.454816},
{1.54691,0.186537,0.416234},{1.54691,0.0916892,0.454816},{1.54691,0.0946866,0.409434},
{1.54691,0.0916892,0.454816},{1.54691,0.186537,0.416234},{1.54691,0.18354,0.461616},
{1.54691,0.186537,0.416234},{0.09144,0.0946866,0.409434},{0.09144,0.186537,0.416234},
{0.09144,0.0946866,0.409434},{1.54691,0.186537,0.416234},{1.54691,0.0946866,0.409434},
{1.54691,0.0916892,0.454816},{0.09144,0.18354,0.461616},{0.09144,0.0916892,0.454816},
{0.09144,0.18354,0.461616},{1.54691,0.0916892,0.454816},{1.54691,0.18354,0.461616},
{0.2921,0.689345,0.508},{0.2921,0.717698,0.889},{0.2921,0.6731,0.51145},
{0.2921,0.717698,0.889},{0.2921,0.689345,0.508},{0.2921,0.733943,0.88555},
{0.254,0.717698,0.889},{0.2921,0.733943,0.88555},{0.254,0.733943,0.88555},
{0.2921,0.733943,0.88555},{0.254,0.717698,0.889},{0.2921,0.717698,0.889},
{0.254,0.717698,0.889},{0.254,0.689345,0.508},{0.254,0.6731,0.51145},
{0.254,0.689345,0.508},{0.254,0.717698,0.889},{0.254,0.733943,0.88555},
{0.2921,0.689345,0.508},{0.254,0.6731,0.51145},{0.254,0.689345,0.508},
{0.254,0.6731,0.51145},{0.2921,0.689345,0.508},{0.2921,0.6731,0.51145},
{0.254,0.733943,0.88555},{0.2921,0.689345,0.508},{0.254,0.689345,0.508},
{0.2921,0.689345,0.508},{0.254,0.733943,0.88555},{0.2921,0.733943,0.88555},
{0.2921,0.717698,0.889},{0.254,0.6731,0.51145},{0.2921,0.6731,0.51145},
{0.254,0.6731,0.51145},{0.2921,0.717698,0.889},{0.254,0.717698,0.889},
{0.4191,0.689345,0.508},{0.4191,0.717698,0.889},{0.4191,0.6731,0.51145},
{0.4191,0.717698,0.889},{0.4191,0.689345,0.508},{0.4191,0.733943,0.88555},
{0.381,0.717698,0.889},{0.4191,0.733943,0.88555},{0.381,0.733943,0.88555},
{0.4191,0.733943,0.88555},{0.381,0.717698,0.889},{0.4191,0.717698,0.889},
{0.381,0.717698,0.889},{0.381,0.689345,0.508},{0.381,0.6731,0.51145},
{0.381,0.689345,0.508},{0.381,0.717698,0.889},{0.381,0.733943,0.88555},
{0.4191,0.689345,0.508},{0.381,0.6731,0.51145},{0.381,0.689345,0.508},
{0.381,0.6731,0.51145},{0.4191,0.689345,0.508},{0.4191,0.6731,0.51145},
{0.381,0.733943,0.88555},{0.4191,0.689345,0.508},{0.381,0.689345,0.508},
{0.4191,0.689345,0.508},{0.381,0.733943,0.88555},{0.4191,0.733943,0.88555},
{0.4191,0.717698,0.889},{0.381,0.6731,0.51145},{0.4191,0.6731,0.51145},
{0.381,0.6731,0.51145},{0.4191,0.717698,0.889},{0.381,0.717698,0.889},
{0.5461,0.689345,0.508},{0.5461,0.717698,0.889},{0.5461,0.6731,0.51145},
{0.5461,0.717698,0.889},{0.5461,0.689345,0.508},{0.5461,0.733943,0.88555},
{0.508,0.717698,0.889},{0.5461,0.733943,0.88555},{0.508,0.733943,0.88555},
{0.5461,0.733943,0.88555},{0.508,0.717698,0.889},{0.5461,0.717698,0.889},
{0.508,0.717698,0.889},{0.508,0.689345,0.508},{0.508,0.6731,0.51145},
{0.508,0.689345,0.508},{0.508,0.717698,0.889},{0.508,0.733943,0.88555},
{0.5461,0.689345,0.508},{0.508,0.6731,0.51145},{0.508,0.689345,0.508},
{0.508,0.6731,0.51145},{0.5461,0.689345,0.508},{0.5461,0.6731,0.51145},
{0.508,0.733943,0.88555},{0.5461,0.689345,0.508},{0.508,0.689345,0.508},
{0.5461,0.689345,0.508},{0.508,0.733943,0.88555},{0.5461,0.733943,0.88555},
{0.5461,0.717698,0.889},{0.508,0.6731,0.51145},{0.5461,0.6731,0.51145},
{0.508,0.6731,0.51145},{0.5461,0.717698,0.889},{0.508,0.717698,0.889},
{0.6731,0.689345,0.508},{0.6731,0.717698,0.889},{0.6731,0.6731,0.51145},
{0.6731,0.717698,0.889},{0.6731,0.689345,0.508},{0.6731,0.733943,0.88555},
{0.635,0.717698,0.889},{0.6731,0.733943,0.88555},{0.635,0.733943,0.88555},
{0.6731,0.733943,0.88555},{0.635,0.717698,0.889},{0.6731,0.717698,0.889},
{0.635,0.717698,0.889},{0.635,0.689345,0.508},{0.635,0.6731,0.51145},
{0.635,0.689345,0.508},{0.635,0.717698,0.889},{0.635,0.733943,0.88555},
{0.6731,0.689345,0.508},{0.635,0.6731,0.51145},{0.635,0.689345,0.508},
{0.635,0.6731,0.51145},{0.6731,0.689345,0.508},{0.6731,0.6731,0.51145},
{0.635,0.733943,0.88555},{0.6731,0.689345,0.508},{0.635,0.689345,0.508},
{0.6731,0.689345,0.508},{0.635,0.733943,0.88555},{0.6731,0.733943,0.88555},
{0.6731,0.717698,0.889},{0.635,0.6731,0.51145},{0.6731,0.6731,0.51145},
{0.635,0.6731,0.51145},{0.6731,0.717698,0.889},{0.635,0.717698,0.889},
{0.8001,0.689345,0.508},{0.8001,0.717698,0.889},{0.8001,0.6731,0.51145},
{0.8001,0.717698,0.889},{0.8001,0.689345,0.508},{0.8001,0.733943,0.88555},
{0.762,0.717698,0.889},{0.8001,0.733943,0.88555},{0.762,0.733943,0.88555},
{0.8001,0.733943,0.88555},{0.762,0.717698,0.889},{0.8001,0.717698,0.889},
{0.762,0.717698,0.889},{0.762,0.689345,0.508},{0.762,0.6731,0.51145},
{0.762,0.689345,0.508},{0.762,0.717698,0.889},{0.762,0.733943,0.88555},
{0.8001,0.689345,0.508},{0.762,0.6731,0.51145},{0.762,0.689345,0.508},
{0.762,0.6731,0.51145},{0.8001,0.689345,0.508},{0.8001,0.6731,0.51145},
{0.762,0.733943,0.88555},{0.8001,0.689345,0.508},{0.762,0.689345,0.508},
{0.8001,0.689345,0.508},{0.762,0.733943,0.88555},{0.8001,0.733943,0.88555},
{0.8001,0.717698,0.889},{0.762,0.6731,0.51145},{0.8001,0.6731,0.51145},
{0.762,0.6731,0.51145},{0.8001,0.717698,0.889},{0.762,0.717698,0.889},
{0.9271,0.688803,0.508},{0.9271,0.716211,0.889},{0.9271,0.6731,0.51145},
{0.9271,0.716211,0.889},{0.9271,0.688803,0.508},{0.9271,0.731915,0.88555},
{0.889,0.716211,0.889},{0.9271,0.731915,0.88555},{0.889,0.731915,0.88555},
{0.9271,0.731915,0.88555},{0.889,0.716211,0.889},{0.9271,0.716211,0.889},
{0.889,0.716211,0.889},{0.889,0.688803,0.508},{0.889,0.6731,0.51145},
{0.889,0.688803,0.508},{0.889,0.716211,0.889},{0.889,0.731915,0.88555},
{0.9271,0.688803,0.508},{0.889,0.6731,0.51145},{0.889,0.688803,0.508},
{0.889,0.6731,0.51145},{0.9271,0.688803,0.508},{0.9271,0.6731,0.51145},
{0.889,0.731915,0.88555},{0.9271,0.688803,0.508},{0.889,0.688803,0.508},
{0.9271,0.688803,0.508},{0.889,0.731915,0.88555},{0.9271,0.731915,0.88555},
{0.9271,0.716211,0.889},{0.889,0.6731,0.51145},{0.9271,0.6731,0.51145},
{0.889,0.6731,0.51145},{0.9271,0.716211,0.889},{0.889,0.716211,0.889},
{1.0541,0.688803,0.508},{1.0541,0.716211,0.889},{1.0541,0.6731,0.51145},
{1.0541,0.716211,0.889},{1.0541,0.688803,0.508},{1.0541,0.731915,0.88555},
{1.016,0.716211,0.889},{1.0541,0.731915,0.88555},{1.016,0.731915,0.88555},
{1.0541,0.731915,0.88555},{1.016,0.716211,0.889},{1.0541,0.716211,0.889},
{1.016,0.716211,0.889},{1.016,0.688803,0.508},{1.016,0.6731,0.51145},
{1.016,0.688803,0.508},{1.016,0.716211,0.889},{1.016,0.731915,0.88555},
{1.0541,0.688803,0.508},{1.016,0.6731,0.51145},{1.016,0.688803,0.508},
{1.016,0.6731,0.51145},{1.0541,0.688803,0.508},{1.0541,0.6731,0.51145},
{1.016,0.731915,0.88555},{1.0541,0.688803,0.508},{1.016,0.688803,0.508},
{1.0541,0.688803,0.508},{1.016,0.731915,0.88555},{1.0541,0.731915,0.88555},
{1.0541,0.716211,0.889},{1.016,0.6731,0.51145},{1.0541,0.6731,0.51145},
{1.016,0.6731,0.51145},{1.0541,0.716211,0.889},{1.016,0.716211,0.889},
{1.1811,0.688803,0.508},{1.1811,0.716211,0.889},{1.1811,0.6731,0.51145},
{1.1811,0.716211,0.889},{1.1811,0.688803,0.508},{1.1811,0.731915,0.88555},
{1.143,0.716211,0.889},{1.1811,0.731915,0.88555},{1.143,0.731915,0.88555},
{1.1811,0.731915,0.88555},{1.143,0.716211,0.889},{1.1811,0.716211,0.889},
{1.143,0.716211,0.889},{1.143,0.688803,0.508},{1.143,0.6731,0.51145},
{1.143,0.688803,0.508},{1.143,0.716211,0.889},{1.143,0.731915,0.88555},
{1.1811,0.688803,0.508},{1.143,0.6731,0.51145},{1.143,0.688803,0.508},
{1.143,0.6731,0.51145},{1.1811,0.688803,0.508},{1.1811,0.6731,0.51145},
{1.143,0.731915,0.88555},{1.1811,0.688803,0.508},{1.143,0.688803,0.508},
{1.1811,0.688803,0.508},{1.143,0.731915,0.88555},{1.1811,0.731915,0.88555},
{1.1811,0.716211,0.889},{1.143,0.6731,0.51145},{1.1811,0.6731,0.51145},
{1.143,0.6731,0.51145},{1.1811,0.716211,0.889},{1.143,0.716211,0.889},
{1.3081,0.688803,0.508},{1.3081,0.716211,0.889},{1.3081,0.6731,0.51145},
{1.3081,0.716211,0.889},{1.3081,0.688803,0.508},{1.3081,0.731915,0.88555},
{1.27,0.716211,0.889},{1.3081,0.731915,0.88555},{1.27,0.731915,0.88555},
{1.3081,0.731915,0.88555},{1.27,0.716211,0.889},{1.3081,0.716211,0.889},
{1.27,0.716211,0.889},{1.27,0.688803,0.508},{1.27,0.6731,0.51145},
{1.27,0.688803,0.508},{1.27,0.716211,0.889},{1.27,0.731915,0.88555},
{1.3081,0.688803,0.508},{1.27,0.6731,0.51145},{1.27,0.688803,0.508},
{1.27,0.6731,0.51145},{1.3081,0.688803,0.508},{1.3081,0.6731,0.51145},
{1.27,0.731915,0.88555},{1.3081,0.688803,0.508},{1.27,0.688803,0.508},
{1.3081,0.688803,0.508},{1.27,0.731915,0.88555},{1.3081,0.731915,0.88555},
{1.3081,0.716211,0.889},{1.27,0.6731,0.51145},{1.3081,0.6731,0.51145},
{1.27,0.6731,0.51145},{1.3081,0.716211,0.889},{1.27,0.716211,0.889},
{1.4351,0.688803,0.508},{1.4351,0.716211,0.889},{1.4351,0.6731,0.51145},
{1.4351,0.716211,0.889},{1.4351,0.688803,0.508},{1.4351,0.731915,0.88555},
{1.397,0.716211,0.889},{1.4351,0.731915,0.88555},{1.397,0.731915,0.88555},
{1.4351,0.731915,0.88555},{1.397,0.716211,0.889},{1.4351,0.716211,0.889},
{1.397,0.716211,0.889},{1.397,0.688803,0.508},{1.397,0.6731,0.51145},
{1.397,0.688803,0.508},{1.397,0.716211,0.889},{1.397,0.731915,0.88555},
{1.4351,0.688803,0.508},{1.397,0.6731,0.51145},{1.397,0.688803,0.508},
{1.397,0.6731,0.51145},{1.4351,0.688803,0.508},{1.4351,0.6731,0.51145},
{1.397,0.731915,0.88555},{1.4351,0.688803,0.508},{1.397,0.688803,0.508},
{1.4351,0.688803,0.508},{1.397,0.731915,0.88555},{1.4351,0.731915,0.88555},
{1.4351,0.716211,0.889},{1.397,0.6731,0.51145},{1.4351,0.6731,0.51145},
{1.397,0.6731,0.51145},{1.4351,0.716211,0.889},{1.397,0.716211,0.889}};
MeshPart *aux=new MeshPart;
for(int i=0;i<num;i++){
	aux->addTriangle(Vector3D(v[i*3][0],v[i*3][1],v[i*3][2]),Vector3D(v[i*3+1][0],v[i*3+1][1],v[i*3+1][2]),
	                 Vector3D(v[i*3+2][0],v[i*3+2][1],v[i*3+2][2]));
}
return aux;
}

