/**********************************************************************
 *
 * This code is part of the MRcore project
 * Author:  Rodrigo Azofra Barrio & Miguel Hernando Gutierrez
 * 
 *
 * MRcore is licenced under the Common Creative License,
 * Attribution-NonCommercial-ShareAlike 3.0
 *
 * You are free:
 *   - to Share - to copy, distribute and transmit the work
 *   - to Remix - to adapt the work
 *
 * Under the following conditions:
 *   - Attribution. You must attribute the work in the manner specified
 *     by the author or licensor (but not in any way that suggests that
 *     they endorse you or your use of the work).
 *   - Noncommercial. You may not use this work for commercial purposes.
 *   - Share Alike. If you alter, transform, or build upon this work,
 *     you may distribute the resulting work only under the same or
 *     similar license to this one.
 *
 * Any of the above conditions can be waived if you get permission
 * from the copyright holder.  Nothing in this license impairs or
 * restricts the author's moral rights.
 *
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  
 **********************************************************************/

#include "puma560sim.h"
#include <iostream>
/*
#include "gl/gltools.h"

#include "base/logger.h"
#include "../../world/world.h"
#include "../../world/cylindricalpart.h"
#include "../../world/tcp.h"
*/

namespace mr
{
IMPLEMENT_MR_OBJECT(Puma560Sim)

Puma560Sim::Puma560Sim()
{
	name="Puma 560";
	move=false;
	i=0;
	//Units in meters
	q[0]=0;
	q[1]=0;
	q[2]=0;
	q[3]=0;
	q[4]=0;
	q[5]=0;

//Base
	cyl[0]=new CylindricalPart(0.52,0.08);
	cyl[0]->setColor(0.6,0.4,0.1);

	cyl[1]=new CylindricalPart(0.04,0.17);
	cyl[1]->setColor(0.6,0.4,0.1);

	cyl[2]=new CylindricalPart(0.15,0.05);
	cyl[2]->setRelativePosition(Vector3D(0,-0.12,0));
	cyl[2]->setColor(0.6,0.4,0.1);
	
	gof[0]=new SimpleJoint(8*PI/9 , -8*PI/9);
	gof[0]->setValue(q[0]);
	gof[0]->setRelativePosition(Vector3D(0,0,0.66));
	gof[0]->setRelativeOrientation(Z_AXIS,PI/2);
	gof[0]->LinkTo(cyl[0]);

//Arm 1
	cyl[3]=new CylindricalPart(0.14,0.1);
	cyl[3]->setColor(0.9,0.9,0.9);
	cyl[3]->setRelativePosition(Vector3D(0,0,-0.14));
	cyl[3]->LinkTo(gof[0]);

	cyl[4]=new CylindricalPart(0.17,0.1);
	cyl[4]->setColor(0.9,0.9,0.9);
	cyl[4]->setRelativeOrientation(0,-PI/2,PI);
	cyl[4]->LinkTo(gof[0]);

	pris[0]=new PrismaticPart;
	vector<Vector2D> list;
	list.push_back(Vector2D(0,0.1));
	list.push_back(Vector2D(0.1,0.1));
	list.push_back(Vector2D(0.1,-0.1));
	list.push_back(Vector2D(0,-0.1));
	pris[0]->setPolygonalBase(list);
	list.clear();
	pris[0]->setColor(0.9,0.9,0.9);
	pris[0]->setHeight(0.25);
	pris[0]->setRelativePosition(Vector3D(0,0,-0.14));
	pris[0]->setRelativeOrientation(Z_AXIS,PI);
	pris[0]->LinkTo(gof[0]);

	cyl[5]=new CylindricalPart(0.03,0.035);
	cyl[5]->setColor(0.9,0.9,0.9);
	cyl[5]->setRelativePosition(Vector3D(0.05,0,0.25));
	cyl[5]->LinkTo(pris[0]);

	cyl[6]=new CylindricalPart(0.04,0.03);
	cyl[6]->setColor(1,1,0);
	cyl[6]->setRelativePosition(Vector3D(0,0,0.03 - 0.014));
	cyl[6]->LinkTo(cyl[5]);

	cyl[7]=new CylindricalPart(0.025,0.12);//GRIS finito
	cyl[7]->setColor(0.7,0.7,0.7);
	cyl[7]->setRelativePosition(Vector3D(0,0,0.17));
	cyl[7]->LinkTo(cyl[4]);

	gof[1]=new SimpleJoint(7*PI/36 , -43*PI/36);//(25*PI/36 , -25*PI/36);
	gof[1]->setValue(q[1]);
	gof[1]->setRelativePosition(Vector3D(0.15,0,0));//Desplazamiento d2
	gof[1]->setRelativeOrientation(-PI/2,0,-PI/2);
	gof[1]->LinkTo(gof[0]);

//Arm 2
	list.push_back(Vector2D(0.15,0));
	list.push_back(Vector2D(0.07,0.432));
	list.push_back(Vector2D(0.06142221,0.45502801));
	list.push_back(Vector2D(0.04552583,0.47376762));
	list.push_back(Vector2D(0.02420495,0.48598596));
	list.push_back(Vector2D(0,0.49022719));
	list.push_back(Vector2D(-0.02420495,0.48598596));
	list.push_back(Vector2D(-0.04552583,0.47376762));
	list.push_back(Vector2D(-0.06142221,0.45502801));
	list.push_back(Vector2D(-0.07,0.432));
	list.push_back(Vector2D(-0.15,0));
	list.push_back(Vector2D(-0.15,-0.12));
	list.push_back(Vector2D(-0.10479903,-0.14998657));
	list.push_back(Vector2D(-0.05387139,-0.16866003));
	list.push_back(Vector2D(0,-0.175));
	list.push_back(Vector2D(0.05387139,-0.16866003));
	list.push_back(Vector2D(0.10479903,-0.14998657));
	list.push_back(Vector2D(0.15,-0.12));

	pris[1]=new PrismaticPart();
	pris[1]->setPolygonalBase(list);
	list.clear();
	pris[1]->setColor(1,1,1);
	pris[1]->setHeight(0.1);
	pris[1]->setRelativePosition(Vector3D(0,0,0.045));
	pris[1]->setRelativeOrientation(Z_AXIS,-PI/2);
	pris[1]->LinkTo(gof[1]);

	gof[2]=new SimpleJoint(5*PI/4,-PI/4);
	gof[2]->setValue(q[2]);
	gof[2]->setRelativePosition(Vector3D(0.432,0,0));
	gof[2]->setRelativeOrientation(Z_AXIS,-PI/2);
	gof[2]->LinkTo(gof[1]);

//Arm 3
	cyl[8]=new CylindricalPart(0.15,0.04);
	cyl[8]->setColor(0.7,0.7,0.7);
	cyl[8]->setRelativePosition(Vector3D(0,0,0.005));
	cyl[8]->LinkTo(gof[2]);

	list.push_back(Vector2D(0.08159934,-0.03039292));
	list.push_back(Vector2D(0.045,0.36));
	list.push_back(Vector2D(-0.045,0.36));
	list.push_back(Vector2D(-0.08159934,-0.03039292));
	list.push_back(Vector2D(-0.08057515,-0.05303013));
	list.push_back(Vector2D(-0.07339124,-0.07452161));
	list.push_back(Vector2D(-0.06059678,-0.0932244));
	list.push_back(Vector2D(-0.04316989,-0.10770873));
	list.push_back(Vector2D(-0.02244279,-0.11686731));
	list.push_back(Vector2D(0,-0.12));
	list.push_back(Vector2D(0.02244279,-0.11686731));
	list.push_back(Vector2D(0.04316989,-0.10770873));
	list.push_back(Vector2D(0.06059678,-0.0932244));
	list.push_back(Vector2D(0.07339124,-0.07452161));
	list.push_back(Vector2D(0.08057515,-0.05303013));

	pris[2]=new PrismaticPart();
	pris[2]->setPolygonalBase(list);
	list.clear();
	pris[2]->setColor(0.8,0.8,0.8);
	pris[2]->setHeight(0.09);
	pris[2]->setRelativeOrientation(Z_AXIS,-PI/2);
	pris[2]->setRelativePosition(Vector3D(0,-0.02,-0.045));
	pris[2]->LinkTo(gof[2]);

	gof[3]=new SimpleJoint(7*PI/9 , -7*PI/9);
	gof[3]->setRelativePosition(Vector3D(0.36,-0.02,0));
	gof[3]->setRelativeOrientation(PI,-PI/2,0);
	gof[3]->LinkTo(gof[2]);

//Arm 4
	list.push_back(Vector2D(0.045,-0.045));
	list.push_back(Vector2D(0.045,0.045));
	list.push_back(Vector2D(-0.045,0.045));
	list.push_back(Vector2D(-0.045,-0.045));

	pris[3]=new PrismaticPart();
	pris[3]->setPolygonalBase(list);
	list.clear();
	pris[3]->setColor(0.39,0.39,0.39);
	pris[3]->setHeight(0.027);
	pris[3]->LinkTo(gof[3]);

	cyl[9]=new CylindricalPart(0.035,0.045);
	cyl[9]->setColor(0.39,0.39,0.39);
	cyl[9]->setRelativePosition(Vector3D(0.01,0,0.072));
	cyl[9]->setRelativeOrientation(PI/2,0,PI/2);
	cyl[9]->LinkTo(pris[3]);

	cyl[10]=new CylindricalPart(0.035,0.045);
	cyl[10]->setColor(0.39,0.39,0.39);
	cyl[10]->setRelativePosition(Vector3D(-0.01,0,0.072));
	cyl[10]->setRelativeOrientation(-PI/2,0,PI/2);
	cyl[10]->LinkTo(pris[3]);

	cyl[11]=new CylindricalPart(0.1,0.02);
	cyl[11]->setColor(0.69,0.69,0.69);
	cyl[11]->setRelativePosition(Vector3D(-0.05,0,0.072));
	cyl[11]->setRelativeOrientation(PI/2,0,PI/2);
	cyl[11]->LinkTo(pris[3]);

	gof[4]=new SimpleJoint(5*PI/9,-5*PI/9);
	gof[4]->setRelativePosition(Vector3D(0,0,0.072));
	gof[4]->setRelativeOrientation(PI,-PI/2,0);
	gof[4]->LinkTo(gof[3]);

//Arm 5
	cyl[12]=new CylindricalPart(0.02,0.045);
	cyl[12]->setColor(0.55,0.55,0.55);
	cyl[12]->setRelativePosition(Vector3D(0,0,-0.01));
	cyl[12]->LinkTo(gof[4]);

	cyl[13]=new CylindricalPart(0.01,0.022);
	cyl[13]->setColor(0.55,0.55,0.55);
	cyl[13]->setRelativePosition(Vector3D(0.045,0,0));
	cyl[13]->setRelativeOrientation(0,PI/2,0);
	cyl[13]->LinkTo(gof[4]);

	gof[5]=new SimpleJoint(133*PI/90,-133*PI/90);
	gof[5]->setRelativePosition(Vector3D(0.055,0,0));
	gof[5]->setRelativeOrientation(PI/2,0,PI/2);
	gof[5]->LinkTo(gof[4]);

//Arm 6
	cyl[14]=new CylindricalPart(0.01,0.025);
	cyl[14]->setColor(0.75,0.75,0.75);
	cyl[14]->LinkTo(gof[5]);

//Tcp
	tcp=new Tcp();
	tcp->setRelativePosition(Vector3D(0,0,0.01));
	tcp->LinkTo(gof[5]);

	(*this)+=cyl[0];
	(*this)+=cyl[1];
	(*this)+=cyl[2];
	
	list.clear();
}


void Puma560Sim::drawGL()
{
	ComposedEntity::drawGL();
	tcp->setDrawReferenceSystem(true);
	return;
}

bool Puma560Sim::forwardKinematics(vector<double> _q, Transformation3D& t)
{
/***
Devuelve por referencia una T3D relativa al robot
***/
	if(_q.size()!=6)return false;

//Comprobar rangos de coordenadas
	if((_q[0]<(-8*PI/9))||((8*PI/9)<_q[0]))return false;
	if((_q[1]<(-25*PI/36))||((25*PI/36)<_q[1]))return false;
	if((_q[2]<(-3*PI/4))||((3*PI/4)<_q[2]))return false;
	if((_q[3]<(-7*PI/9))||((7*PI/9)<_q[3]))return false;
	if((_q[4]<(-5*PI/9))||((5*PI/9)<_q[4]))return false;
	if((_q[5]<(-133*PI/45))||((133*PI/45)<_q[5]))return false;

//Analisis de la configuracion

//1º Almaceno los valores actuales de q
	double q_aux[6];
	q_aux[0]=gof[0]->getValue();
	q_aux[1]=gof[1]->getValue();
	q_aux[2]=gof[2]->getValue();
	q_aux[3]=gof[3]->getValue();
	q_aux[4]=gof[4]->getValue();
	q_aux[5]=gof[5]->getValue();

//2º Cambio las coordenadas de las articulaciones a los nuevos valores
	gof[0]->setValue(_q[0]);
	gof[1]->setValue(_q[1]);
	gof[2]->setValue(_q[2]);
	gof[3]->setValue(_q[3]);
	gof[4]->setValue(_q[4]);
	gof[5]->setValue(_q[5]);

//3º Calculo la matriz de transformacion relativa
	Transformation3D _t=tcp->getAbsoluteT3D();
	Transformation3D aux=cyl[0]->getAbsoluteT3D();
	t=(aux.inverted())*_t;

//4º Recoloco el robot en las q iniciales
	gof[0]->setValue(q_aux[0]);
	gof[1]->setValue(q_aux[1]);
	gof[2]->setValue(q_aux[2]);
	gof[3]->setValue(q_aux[3]);
	gof[4]->setValue(q_aux[4]);
	gof[5]->setValue(q_aux[5]);

	return true;
}

bool Puma560Sim::forwardKinematicsAbs(vector<double> _q, Transformation3D& t)
{
/***
Devuelve por referencia una T3D absoluta del robot
***/
	if(_q.size()!=6)return false;
//Comprobar rangos de coordenadas
	if((_q[0]<(-8*PI/9))||((8*PI/9)<_q[0]))return false;
	if((_q[1]<(-25*PI/36))||((25*PI/36)<_q[1]))return false;
	if((_q[2]<(-3*PI/4))||((3*PI/4)<_q[2]))return false;
	if((_q[3]<(-7*PI/9))||((7*PI/9)<_q[3]))return false;
	if((_q[4]<(-5*PI/9))||((5*PI/9)<_q[4]))return false;
	if((_q[5]<(-133*PI/45))||((133*PI/45)<_q[5]))return false;

//Analisis de la configuracion

	Transformation3D t_aux;
	bool flag=forwardKinematics(_q,t_aux);
	t=t_aux*cyl[0]->getAbsoluteT3D();//----------------------------------------------------------------------------------------------
	//Puede ser asi o la multiplicacion al reves

	return flag;
}

bool Puma560Sim::inverseKinematics(Transformation3D t, vector<double> &_q, unsigned char conf)
{
	//Faltaria comprobar la configuracion--------------------------------------------------------------------------------------------------------

	return PUMA560inverseKinematics(t,_q,conf);
}

bool Puma560Sim::inverseKinematicsAbs(Transformation3D t, vector<double> &_q, unsigned char conf)
{
/***
Funcion generica
Recibe una T3D absoluta
Le manda una T3D relativa al metodo relativo inverso y devuelve el vector de 
coordenadas correspondiente
***/
	Transformation3D _t=cyl[0]->getAbsoluteT3D();
	Transformation3D t_aux=(_t.inverted())*t;
	return inverseKinematics(t_aux,_q,conf);
}

bool Puma560Sim::PUMA560inverseKinematics(Transformation3D t, vector<double> &_q, unsigned char conf)
{
/***
Funcion especifica
Recive una T3D relativa y devuelve el vector de coordenadas
***/
	return true;
}

bool Puma560Sim::moveTo(vector<double> _q)
{
	if(_q.size()!=6)return false;

//Compruebo rangos
	if((_q[0]<(-8*PI/9))||((8*PI/9)<_q[0]))return false;
	if((_q[1]<(-25*PI/36))||((25*PI/36)<_q[1]))return false;
	if((_q[2]<(-3*PI/4))||((3*PI/4)<_q[2]))return false;
	if((_q[3]<(-7*PI/9))||((7*PI/9)<_q[3]))return false;
	if((_q[4]<(-5*PI/9))||((5*PI/9)<_q[4]))return false;
	if((_q[5]<(-133*PI/45))||((133*PI/45)<_q[5]))return false;

//FALTA LA CONFIGURACION DE LOS CODOS--------------------------------------------------------------------------------------------------------

	for(int i=0;i<6;i++)
		gof[i]->setValue(_q[i]);

	return true;
}




void Puma560Sim::getConfiguration(float& _shoulder, float& _elbow)
{
	double q1=gof[1]->getValue();
	double q2=gof[2]->getValue();
	double alfa=PI/2 - atan(0.36/0.0203);

	if(q2<alfa)
		_elbow=1;//Codo arriba
	else if(q2>alfa)
		_elbow=-1;
	else
		_elbow=0;
}

void Puma560Sim::simulate(double delta_t)
{
}

void Puma560Sim::setFlash()
{
	if(i==4)
	{
		cyl[6]->setColor(1,1,0);
		i=0;
	}
	else
	{
		i+=1;
		cyl[6]->setColor(0.5,0.5,0);
	}
}

};//Namespace mr
