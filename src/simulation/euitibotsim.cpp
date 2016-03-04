/**********************************************************************
 *
 * This code is part of the MRcore project
 * Author:  Cristina Gajate & Miguel Hernando Gutierrez
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

#include "euitibotsim.h"
#include <iostream>


namespace mr
{
IMPLEMENT_MR_OBJECT(EUITIbotSim)
void EUITIbotSim::writeToStream(Stream& stream)
 {SolidEntity::writeToStream(stream);}
void EUITIbotSim::readFromStream(Stream& stream)
 {SolidEntity::readFromStream(stream);}
void EUITIbotSim::writeToXML(XMLElement* parent)
{
	SolidEntity::writeToXML(parent);
}
void EUITIbotSim::readFromXML(XMLElement* parent)
{
	SolidEntity::readFromXML(parent);
}

char* EUITIbotSim::CreateXMLText()
{
	XMLElement* elem=new XMLElement(0,"EUITIbotSim");
	writeToXML(elem);
	return elem->CreateXMLText();
}

void EUITIbotSim::loadFromXMLText(char *XmlText)
{
	XML x;
	readFromXML(x.Paste(XmlText));
}

EUITIbotSim::EUITIbotSim()
{
	name="EUITIbot";
	//Units in meters and rad.

	for(int j=0;j<7;j++)q_init.push_back(0);

	vector<Vector2D> list;
	Actuator* actuator;

////////////////////////////////////////////////////////////////////////////
//////////////////////////   BASE = LINK[0]       //////////////////////////
////////////////////////////////////////////////////////////////////////////

//Base = link[0]
	ComposedEntity *link=new ComposedEntity;
	link->setName("Articulación 1");
	link->setRelativePosition(Vector3D(0,0,0.05));
	link->setRelativeOrientation(0,0,0);

	//CILINDRO(Cilindro)
	CylindricalPart *auxCyl=new CylindricalPart(0.05,0.0425);//Altura y radio
	auxCyl->setColor(0.6,0,0);
	auxCyl->setRelativePosition(Vector3D(0,0,0));
	(*link)+=auxCyl;

	links.push_back(link);

	//Joint[0]
	SimpleJoint *auxJoint=new SimpleJoint(PI,-PI,true,0,Z_AXIS,false);
	auxJoint->LinkTo(links[0]);
	auxJoint->setValue(q_init[0]);
	joints.push_back(auxJoint);

	actuator=new Actuator();
	actuator->linkTo(joints[0]);
	actuators.push_back(actuator);


////////////////////////////////////////////////////////////////////////////
//////////////////////////ARTICULACION 2 = LINK[1]//////////////////////////
////////////////////////////////////////////////////////////////////////////
	link=new ComposedEntity;
	link->setName("Articulación 2");
	link->setRelativePosition(Vector3D(0,0,0.05));
	link->setRelativeOrientation(0,0,0);

	// CILINDRO
	auxCyl=new CylindricalPart(0.006,0.0425);//Altura y radio
	auxCyl->setColor(1,1,0);
	(*link)+=auxCyl;

	// PRISMA IRREGULAR DERECHO
	PrismaticPart *auxPrism=new PrismaticPart;
	list.push_back(Vector2D(-0.0155,0));
	list.push_back(Vector2D(0.0155,0));
	list.push_back(Vector2D(0.0155,0.05));
	list.push_back(Vector2D(0.0125,0.055));
	list.push_back(Vector2D(-0.0125,0.055));
	list.push_back(Vector2D(-0.0155,0.05));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(1,1,0);
	auxPrism->setHeight(0.01);
	auxPrism->setRelativePosition(Vector3D(0,0.04,0.006));
	auxPrism->setRelativeOrientation(PI/2, 0, 0);		
	(*link)+=auxPrism;

	// PRISMA IRREGULAR IZQUIERDO
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(-0.0155,0));
	list.push_back(Vector2D(0.0155,0));
	list.push_back(Vector2D(0.0155,0.05));
	list.push_back(Vector2D(0.0125,0.055));
	list.push_back(Vector2D(-0.0125,0.055));
	list.push_back(Vector2D(-0.0155,0.05));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(1,1,0);
	auxPrism->setHeight(0.01);
	auxPrism->setRelativePosition(Vector3D(0,-0.03,0.006));
	auxPrism->setRelativeOrientation(PI/2, 0 , 0);		
	(*link)+=auxPrism;

	// SERVO3(Servo3)
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(-0.0095,-0.02));
	list.push_back(Vector2D(0.0095,-0.02));
	list.push_back(Vector2D(0.0095,0.02));
	list.push_back(Vector2D(-0.0095,0.02));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(0,0,0);
	auxPrism->setHeight(0.035);
	auxPrism->setRelativePosition(Vector3D(0,0.05,0.02));
	auxPrism->setRelativeOrientation(Z_AXIS,-PI);
	(*link)+=auxPrism;	

	link->LinkTo(joints[0]);
	links.push_back(link);

	//Joint[1]
	auxJoint=new SimpleJoint(PI,0,true,0,Z_AXIS,false);
	auxJoint->setRelativePosition(Vector3D(0.00,0,0.0955));
	auxJoint->setRelativeOrientation(PI/2,0,0);
	auxJoint->LinkTo(joints[0]);
	auxJoint->setValue(q_init[1]);
	joints.push_back(auxJoint);

	actuator=new Actuator();
	actuator->linkTo(auxJoint);
	actuators.push_back(actuator);

////////////////////////////////////////////////////////////////////////////
//////////////////////////ARTICULACION 2 = LINK[1]//////////////////////////
////////////////////////////////////////////////////////////////////////////

	//NUEVO GRUPO DE PIEZAS
	link=new ComposedEntity;
	link->setName("Articulación 3");
	link->setRelativePosition(Vector3D(0,0,0.025));
	link->setRelativeOrientation(-PI,0,0);


	// CILINDRO IZQUIERDO ABAJO(Cil_Ab_Izq)
	auxCyl=new CylindricalPart(0.003,0.0175);//Altura y radio
	auxCyl->setColor(0.6,0,0);
	auxCyl->setRelativePosition(Vector3D(0,0,0));
	(*link)+=auxCyl;

	// PRISMA IRREGULAR IZQUIERDA(Pris_Izq)
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(-0.013,0));
	list.push_back(Vector2D(0.013,0));
	list.push_back(Vector2D(0.013,0.11));
	list.push_back(Vector2D(-0.013,0.11));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(0.6,0,0);
	auxPrism->setHeight(0.003);
	auxPrism->setRelativePosition(Vector3D(0,0,0));
	auxPrism->setRelativeOrientation(Z_AXIS,-PI/2);
	(*link)+=auxPrism;

	// CILINDRO IZQUIERDO ARRIBA(Cil_Ar_Izq)
	auxCyl=new CylindricalPart(0.003,0.0175);//Altura y radio
	auxCyl->setColor(0.6,0,0);
	auxCyl->setRelativePosition(Vector3D(0.11,0,0));
	(*link)+=auxCyl;


	// PRISMA IRREGULAR REFUERZO IZQUIERDA(Pris_Ref_Izq)
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(-0.04,0));
	list.push_back(Vector2D(0.04,0));
	list.push_back(Vector2D(0.0315,0.004));
	list.push_back(Vector2D(-0.0315,0.004));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(0.6,0,0);
	auxPrism->setHeight(0.026);
	auxPrism->setRelativePosition(Vector3D(0.055,-0.013,0));
	auxPrism->setRelativeOrientation(X_AXIS,-PI/2);
	(*link)+=auxPrism;

	// PRISMA IRREGULAR CENTRO ABAJO(Pris_Cen_Ab)
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(0,0));
	list.push_back(Vector2D(0.049,0));
	list.push_back(Vector2D(0.049,0.008));
	list.push_back(Vector2D(0.04,0.008));
	list.push_back(Vector2D(0.04,0.027));
	list.push_back(Vector2D(0.033,0.027));
	list.push_back(Vector2D(0.033,0.008));
	list.push_back(Vector2D(0.016,0.008));
	list.push_back(Vector2D(0.016,0.027));
	list.push_back(Vector2D(0.009,0.027));
	list.push_back(Vector2D(0.009,0.008));
	list.push_back(Vector2D(0,0.008));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(0.6,0,0);
	auxPrism->setHeight(0.02);
	auxPrism->setRelativePosition(Vector3D(0.038,-0.01,0.003));
	auxPrism->setRelativeOrientation(0,-PI/2,-PI/2);
	
	(*link)+=auxPrism;

	// PRISMA IRREGULAR CENTRO ARRIBA(Pris_Cen_Ar)
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(0,0));
	list.push_back(Vector2D(0.049,0));
	list.push_back(Vector2D(0.049,0.008));
	list.push_back(Vector2D(0,0.008));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(0.6,0,0);
	auxPrism->setHeight(0.02);
	auxPrism->setRelativePosition(Vector3D(0.064,-0.01,0.003));
	auxPrism->setRelativeOrientation(0,-PI/2,-PI/2);

	(*link)+=auxPrism;

	// CILINDRO DERECHO ABAJO(Cil_Ab_Der)
	auxCyl=new CylindricalPart(0.003,0.0175);//Altura y radio
	auxCyl->setColor(0.6,0,0);
	auxCyl->setRelativePosition(Vector3D(0,0,0.052));
	(*link)+=auxCyl;

	// PRISMA IRREGULAR DERECHA(Pris_Der)
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(-0.013,0));
	list.push_back(Vector2D(0.013,0));
	list.push_back(Vector2D(0.013,0.11));
	list.push_back(Vector2D(-0.013,0.11));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(0.6,0,0);
	auxPrism->setHeight(0.003);
	auxPrism->setRelativePosition(Vector3D(0,0,0.052));
	auxPrism->setRelativeOrientation(Z_AXIS,-PI/2);
	(*link)+=auxPrism;
	

	// CILINDRO DERECHO ARRIBA(Cil_Ar_Der)
	auxCyl=new CylindricalPart(0.003,0.0175);//Altura y radio
	auxCyl->setColor(0.6,0,0);
	auxCyl->setRelativePosition(Vector3D(0.11,0,0.052));
	(*link)+=auxCyl;

	// PRISMA IRREGULAR REFUERZO DERECHA(Pris_Ref_Der)
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(-0.04,0));
	list.push_back(Vector2D(0.04,0));
	list.push_back(Vector2D(0.0315,0.004));
	list.push_back(Vector2D(-0.0315,0.004));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(0.6,0,0);
	auxPrism->setHeight(0.026);
	auxPrism->setRelativePosition(Vector3D(0.055,0.013,0.055));
	auxPrism->setRelativeOrientation(X_AXIS,PI/2);
	(*link)+=auxPrism;

	//CILINDRO SERVO PEQUEÑO(Servo_Peq)
	auxCyl=new CylindricalPart(0.007,0.010);//Altura y radio
	auxCyl->setColor(0,0,0);
	auxCyl->setRelativePosition(Vector3D(0,0,0.056));
	(*link)+=auxCyl;

	//CILINDRO SERVO GRANDE(Servo_Gran)
	auxCyl=new CylindricalPart(0.003,0.0175);//Altura y radio
	auxCyl->setColor(0,0,0);
	auxCyl->setRelativePosition(Vector3D(0,0,0.05));
	(*link)+=auxCyl;

	// AÑADIMOS AL VECTOR LINKS
	link->LinkTo(joints[1]);
	links.push_back(link);

//Joint[2]
	auxJoint=new SimpleJoint(0,-PI,true,0,Z_AXIS,false);
	auxJoint->setRelativePosition(Vector3D(0.11,0,0));
	auxJoint->setRelativeOrientation(2*PI,0,0);
	auxJoint->LinkTo(joints[1]);
	auxJoint->setValue(q_init[2]);
	joints.push_back(auxJoint);

	actuator=new Actuator();
	actuator->linkTo(auxJoint);
	actuators.push_back(actuator);


////////////////////////////////////////////////////////////////////////////
//////////////////////////ARTICULACION 3 = LINK[2]//////////////////////////
////////////////////////////////////////////////////////////////////////////


	//NUEVO GRUPO DE PIEZAS
	link=new ComposedEntity;
	link->setName("Articulación 4");
	link->setRelativePosition(Vector3D(0,0,-0.02));//Desplazamiento a2
	link->setRelativeOrientation(0,0,PI/2);

	// PRISMA IRREGULAR CENTRO ARRIBA(Pris_Cen_Ar)
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(-0.016,0));
	list.push_back(Vector2D(0.016,0));
	list.push_back(Vector2D(0.016,0.0065));
	list.push_back(Vector2D(-0.016,0.0065));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(1,1,0);
	auxPrism->setHeight(0.02);
	auxPrism->setRelativeOrientation(0,PI/2, -PI/2);
	auxPrism->setRelativePosition(Vector3D(0.056,0.01,0.022));
	(*link)+=auxPrism;

	//CILINDRO(Cilindro_Rojo)
	auxCyl=new CylindricalPart(0.003,0.0175);//Altura y radio
	auxCyl->setColor(1,1,0);
	auxCyl->setRelativePosition(Vector3D(0,0,0.038));
	(*link)+=auxCyl;

	// PRISMA IRREGULAR DERECHA(Pris_Der)
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(-0.0125,0));
	list.push_back(Vector2D(0.0125,0));
	list.push_back(Vector2D(0.0125,0.097));
	list.push_back(Vector2D(-0.0125,0.097));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(1,1,0);
	auxPrism->setHeight(0.003);
	auxPrism->setRelativeOrientation(Z_AXIS,-PI/2);
	auxPrism->setRelativePosition(Vector3D(-0.001,0,0.038));
	(*link)+=auxPrism;

	// PRISMA IRREGULAR IZQUIERDA(Pris_Izq)
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(0.0155,-0.018));
	list.push_back(Vector2D(0.0155,0.032));
	list.push_back(Vector2D(0.0125,0.038));
	list.push_back(Vector2D(0.0125,0.096));
	list.push_back(Vector2D(-0.0125,0.096));
	list.push_back(Vector2D(-0.0125,0.038));
	list.push_back(Vector2D(-0.0155,0.032));
	list.push_back(Vector2D(-0.0155,-0.018));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(1,1,0);
	auxPrism->setHeight(0.006);
	auxPrism->setRelativeOrientation(Z_AXIS,-PI/2);
	auxPrism->setRelativePosition(Vector3D(0,0,0));
	(*link)+=auxPrism;

	// PRISMA IRREGULAR CENTRO ABAJO(Pris_Cen_Ab)
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(-0.016,0));
	list.push_back(Vector2D(0.016,0));
	list.push_back(Vector2D(0.016,0.0065));
	list.push_back(Vector2D(0.01,0.0065));
	list.push_back(Vector2D(0.01,0.018));
	list.push_back(Vector2D(0.0045,0.018));
	list.push_back(Vector2D(0.0045,0.0065));
	list.push_back(Vector2D(-0.0045,0.0065));
	list.push_back(Vector2D(-0.0045,0.018));
	list.push_back(Vector2D(-0.01,0.018));
	list.push_back(Vector2D(-0.01,0.0065));
	list.push_back(Vector2D(-0.016,0.0065));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(1,1,0);
	auxPrism->setHeight(0.02);
	auxPrism->setRelativeOrientation(0,PI/2,-PI/2);
	
	auxPrism->setRelativePosition(Vector3D(0.038,0.01,0.022));
	(*link)+=auxPrism;

	// PRISMA IRREGULAR REFUERZO DERECHA(Pris_Ref_Der)
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(0,0));
	list.push_back(Vector2D(0.004,0.0045));
	list.push_back(Vector2D(0.004,0.081));
	list.push_back(Vector2D(0,0.081));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(1,1,0);
	auxPrism->setHeight(0.02);
	auxPrism->setRelativeOrientation(0,-PI/2,-PI/2);

	auxPrism->setRelativePosition(Vector3D(0.015,-0.013,0.041));
	(*link)+=auxPrism;

	// PRISMA IRREGULAR ARRIBA(Pris_Ar)
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(-0.016,0));
	list.push_back(Vector2D(-0.0115,0));
	list.push_back(Vector2D(-0.0115,0.005));
	list.push_back(Vector2D(0.0115,0.005));
	list.push_back(Vector2D(0.0155,0));
	list.push_back(Vector2D(0.016,0));
	list.push_back(Vector2D(0.016,0.011));
	list.push_back(Vector2D(-0.016,0.011));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(1,1,0);
	auxPrism->setHeight(0.025);
	auxPrism->setRelativeOrientation(-PI/2, -PI/2,0);
	
	auxPrism->setRelativePosition(Vector3D(0.085,-0.013,0.022));
	(*link)+=auxPrism;

	//CILINDRO SERVO GRANDE(Servo_Gran)
	auxCyl=new CylindricalPart(0.003,0.015);//Altura y radio
	auxCyl->setColor(0,0,0);
	auxCyl->setRelativePosition(Vector3D(0,0,-0.008));
	(*link)+=auxCyl;

	//CILINDRO SERVO PEQUEÑO(Servo_Peq)
	auxCyl=new CylindricalPart(0.006,0.015);//Altura y radio
	auxCyl->setColor(0,0,0);
	auxCyl->setRelativePosition(Vector3D(0,0,-0.006));
	(*link)+=auxCyl;

	// SERVO4(Servo4)
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(-0.0095,-0.02));
	list.push_back(Vector2D(0.0095,-0.02));
	list.push_back(Vector2D(0.0095,0.02));
	list.push_back(Vector2D(-0.0095,0.02));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(0,0,0);
	auxPrism->setHeight(0.035);
	auxPrism->setRelativePosition(Vector3D(0.007,0,0));
	auxPrism->setRelativeOrientation(Z_AXIS,-PI/2);
	(*link)+=auxPrism;

	// SERVO5(Servo5)
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(-0.0055,-0.011));
	list.push_back(Vector2D(0.0055,-0.011));
	list.push_back(Vector2D(0.0055,0.011));
	list.push_back(Vector2D(-0.0055,0.011));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(0,0,0);
	auxPrism->setHeight(0.025);
	auxPrism->setRelativePosition(Vector3D(0.067,0,0.023));
	auxPrism->setRelativeOrientation(Y_AXIS,PI/2);
	(*link)+=auxPrism;

	//CILINDRO REFUERZO SERVO(Cil_Ref)
	auxCyl=new CylindricalPart(0.006,0.008);//Altura y radio
	auxCyl->setColor(1,1,0);
	auxCyl->setRelativePosition(Vector3D(0.09,-0.015,0.022));
	auxCyl->setRelativeOrientation(Y_AXIS,PI/2);
	(*link)+=auxCyl;

	// AÑADIMOS AL VECTOR LINKS
	link->LinkTo(joints[2]);
	links.push_back(link);

	auxJoint=new SimpleJoint(0,-PI,true,0,Z_AXIS,false);
	auxJoint->setRelativePosition(Vector3D(0,0,0));//Desplazamiento a2
	auxJoint->setRelativeOrientation(-PI/2,0,0);
	auxJoint->LinkTo(joints[2]);
	auxJoint->setValue(q_init[3]);
	joints.push_back(auxJoint);

	actuator=new Actuator();
	actuator->linkTo(auxJoint);
	actuators.push_back(actuator);
			
////////////////////////////////////////////////////////////////////////////
//////////////////////////ARTICULACION 4 = LINK[3]//////////////////////////
////////////////////////////////////////////////////////////////////////////

	//NUEVO GRUPO DE PIEZAS
	link=new ComposedEntity;
	link->setName("Articulación 5");
	link->setRelativePosition(Vector3D(0,-0.002,0.1));
	link->setRelativeOrientation(0, 0 , 0);

	// PRISMA IRREGULAR(Prisma)
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(-0.0125,0));
	list.push_back(Vector2D(0.02,0));
	list.push_back(Vector2D(0.026,0.003));
	list.push_back(Vector2D(0.029,0.009));
	list.push_back(Vector2D(0.029,0.023));
	list.push_back(Vector2D(0.026,0.0275));
	list.push_back(Vector2D(0.023,0.0275));
	list.push_back(Vector2D(0.023,0.011));
	list.push_back(Vector2D(0.02,0.011));
	list.push_back(Vector2D(0.02,0.006));
	list.push_back(Vector2D(0.009,0.006));
	list.push_back(Vector2D(0.009,0.003));
	list.push_back(Vector2D(-0.009,0.003));
	list.push_back(Vector2D(-0.009,0.006));
	list.push_back(Vector2D(-0.0125,0.006));
	list.push_back(Vector2D(-0.0125,0.011));
	list.push_back(Vector2D(-0.0155,0.011));
	list.push_back(Vector2D(-0.0155,0.013));
	list.push_back(Vector2D(-0.0185,0.013));
	list.push_back(Vector2D(-0.021,0.009));
	list.push_back(Vector2D(-0.0185,0.003));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(0.6,0,0);
	auxPrism->setHeight(0.025);
	auxPrism->setRelativeOrientation(PI/2, 0 , -PI);		
	auxPrism->setRelativePosition(Vector3D(0,-0.01,0));
	(*link)+=auxPrism;

	// PRISMA IRREGULAR IZQUIERDA(Pris_Izq)
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(-0.0045,0));
	list.push_back(Vector2D(0.0045,0));
	list.push_back(Vector2D(0.0045,0.0125));
	list.push_back(Vector2D(-0.0045,0.0125));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(0.6,0,0);
	auxPrism->setHeight(0.003);
	auxPrism->setRelativeOrientation(Y_AXIS,PI/2);
	auxPrism->setRelativePosition(Vector3D(0.016,-0.004,0.016));
	(*link)+=auxPrism;

	//CILINDRO IZQUIERDA(Cil_Izq)
	auxCyl=new CylindricalPart(0.003,0.0065);
	auxCyl->setColor(0.6,0,0);
	auxCyl->setRelativePosition(Vector3D(0.016,0.005,0.02));
	auxCyl->setRelativeOrientation(0,PI/2,0);
	(*link)+=auxCyl;

	//CILINDRO DERECHA(Cil_Der)
	auxCyl=new CylindricalPart(0.003,0.0125);
	auxCyl->setColor(0.6,0,0);
	auxCyl->setRelativePosition(Vector3D(-0.026,0.002,0.027));
	auxCyl->setRelativeOrientation(0,PI/2,0);
	(*link)+=auxCyl;

	//CILINDRO SERVO ARRIBA(Servo_Ar)
	auxCyl=new CylindricalPart(0.009,0.005);//Altura y radio
	auxCyl->setColor(0,0,0);
	auxCyl->setRelativePosition(Vector3D(-0.025,0.002,0.027));
	auxCyl->setRelativeOrientation(Y_AXIS,PI/2);
	(*link)+=auxCyl;

	//CILINDRO SERVO ABAJO(Servo_Ab)
	auxCyl=new CylindricalPart(0.005,0.005);//Altura y radio
	auxCyl->setColor(0,0,0);
	auxCyl->setRelativePosition(Vector3D(0,0,-0.004));
	(*link)+=auxCyl;

	// SERVO6(Servo6)
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(-0.0055,-0.011));
	list.push_back(Vector2D(0.0055,-0.011));
	list.push_back(Vector2D(0.0055,0.011));
	list.push_back(Vector2D(-0.0055,0.011));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(0,0,0);
	auxPrism->setHeight(0.025);
	auxPrism->setRelativePosition(Vector3D(-0.024,0.003,0.022));
	auxPrism->setRelativeOrientation(-PI/2,0,PI/2);

	(*link)+=auxPrism;

	link->LinkTo(joints[3]);
	links.push_back(link);

	auxJoint=new SimpleJoint(0 , -PI,true,0,Z_AXIS,0);
	auxJoint->setRelativePosition(Vector3D(0,0,0.128));
	auxJoint->setRelativeOrientation(-PI/2, 0 , -PI/2);
	auxJoint->LinkTo(joints[3]);
	auxJoint->setValue(q_init[4]);
	joints.push_back(auxJoint);

	actuator=new Actuator();
	actuator->linkTo(auxJoint);
	actuators.push_back(actuator);


////////////////////////////////////////////////////////////////////////////
//////////////////////////ARTICULACION 5 = LINK[4]//////////////////////////
////////////////////////////////////////////////////////////////////////////
		
	//NUEVO GRUPO DE PIEZAS
	link=new ComposedEntity;
	link->setName("Articulación 6");
	link->setRelativePosition(Vector3D(0,0,-0.02));
	link->setRelativeOrientation(0,0,PI/2);

	// PRISMA IRREGULAR(Prisma)
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(-0.017,0));
	list.push_back(Vector2D(0.017,0));
	list.push_back(Vector2D(0.021,0.002));
	list.push_back(Vector2D(0.023,0.006));
	list.push_back(Vector2D(0.023,0.01));
	list.push_back(Vector2D(0.019,0.019));
	list.push_back(Vector2D(0.017,0.019));
	list.push_back(Vector2D(0.017,0.01));
	list.push_back(Vector2D(0.014,0.01));
	list.push_back(Vector2D(0.014,0.006));
	list.push_back(Vector2D(-0.014,0.006));
	list.push_back(Vector2D(-0.014,0.01));
	list.push_back(Vector2D(-0.017,0.01));
	list.push_back(Vector2D(-0.017,0.019));
	list.push_back(Vector2D(-0.019,0.019));
	list.push_back(Vector2D(-0.023,0.01));
	list.push_back(Vector2D(-0.023,0.006));
	list.push_back(Vector2D(-0.021,0.002));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(1,1,0);
	auxPrism->setHeight(0.025);
	auxPrism->setRelativeOrientation(PI/2, PI/2, PI/2);
	
	auxPrism->setRelativePosition(Vector3D(-0.012,-0.019,0.019));
	(*link)+=auxPrism;

	// CILINDRO DERECHA(Cil_Der)
	auxCyl=new CylindricalPart(0.002,0.012);
	auxCyl->setColor(1,1,0);
	(*link)+=auxCyl;

	//CILINDRO IZQUIERDA(Cil_Izq)
	auxCyl=new CylindricalPart(0.002,0.012);
	auxCyl->setColor(1,1,0); 
	auxCyl->setRelativePosition(Vector3D(0,0,0.036));
	(*link)+=auxCyl;


	//CILINDRO REFUERZO(Cil_Ref)
	auxCyl=new CylindricalPart(0.006,0.008);
	auxCyl->setColor(1,1,0);
	auxCyl->setRelativePosition(Vector3D(0,-0.019,0.020));
	auxCyl->setRelativeOrientation(0,PI/2,-PI/2);
	(*link)+=auxCyl;

	// SERVO7(Servo7)
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(-0.0055,-0.011));
	list.push_back(Vector2D(0.0055,-0.011));
	list.push_back(Vector2D(0.0055,0.011));
	list.push_back(Vector2D(-0.0055,0.011));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(0,0,0);
	auxPrism->setHeight(0.025);
	auxPrism->setRelativePosition(Vector3D(0.015,-0.005,0.019));
	auxPrism->setRelativeOrientation(Y_AXIS,-PI/2);
	(*link)+=auxPrism;

	// AÑADIMOS AL VECTOR LINKS
	link->LinkTo(joints[4]);
	links.push_back(link);

	auxJoint=new SimpleJoint(0,-PI,true,0,Z_AXIS,false);
	auxJoint->setRelativePosition(Vector3D(0,0,0));
	auxJoint->setRelativeOrientation(-PI/2,0,-PI/2);
	auxJoint->LinkTo(joints[4]);
	auxJoint->setValue(q_init[5]);
	joints.push_back(auxJoint);

	actuator=new Actuator();
	actuator->linkTo(auxJoint);
	actuators.push_back(actuator);

////////////////////////////////////////////////////////////////////////////
//////////////////////////PINZA//////////////////////////
////////////////////////////////////////////////////////////////////////////

	//NUEVO GRUPO DE PIEZAS
	link=new ComposedEntity;
	link->setName("PINZA");
	link->setRelativePosition(Vector3D(0,0,0.025));
	link->setRelativeOrientation(PI/2, 0 , 0);

	// PRISMA IRREGULAR CENTRAL
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(0.024,0));
	list.push_back(Vector2D(0.028,0.004));
	list.push_back(Vector2D(0.028,0.024));
	list.push_back(Vector2D(0.012,0.036));
	list.push_back(Vector2D(-0.012,0.036));
	list.push_back(Vector2D(-0.028,0.024));
	list.push_back(Vector2D(-0.028,0.004));
	list.push_back(Vector2D(-0.024,0));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(0.6,0,0);
	auxPrism->setHeight(0.005);
	auxPrism->setRelativeOrientation(0, 0 , 0);		
	auxPrism->setRelativePosition(Vector3D(0,0,0));
	(*link)+=auxPrism;

	// PRISMA IRREGULAR ABAJO
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(-0.01,-0.01));
	list.push_back(Vector2D(0.01,-0.01));
	list.push_back(Vector2D(0.02,0));
	list.push_back(Vector2D(0.02,0.005));
	list.push_back(Vector2D(0.01,0.015));
	list.push_back(Vector2D(-0.01,0.015));
	list.push_back(Vector2D(-0.02,0.005));
	list.push_back(Vector2D(-0.02,0));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(0.6,0,0);
	auxPrism->setHeight(0.005);
	auxPrism->setRelativeOrientation(-PI/2, 0 , 0);		
	auxPrism->setRelativePosition(Vector3D(0,0,0.005));
	(*link)+=auxPrism;

	// PRISMA IRREGULAR PINZA
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(0.006,0));
	list.push_back(Vector2D(0,0));
	list.push_back(Vector2D(0,0.03));
	list.push_back(Vector2D(0.006,0.06));
	list.push_back(Vector2D(0.006,0.09));
	list.push_back(Vector2D(0.012,0.09));
	list.push_back(Vector2D(0.012,0.06));
	list.push_back(Vector2D(0.006,0.03));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(1,1,0);
	auxPrism->setHeight(0.005);	
	auxPrism->setRelativePosition(Vector3D(-0.015,0.01,-0.005));
	(*link)+=auxPrism;

	// PRISMA IRREGULAR PINZA
	auxPrism=new PrismaticPart;
	list.push_back(Vector2D(-0.006,0));
	list.push_back(Vector2D(-0.006,0.03));
	list.push_back(Vector2D(-0.012,0.06));
	list.push_back(Vector2D(-0.012,0.09));
	list.push_back(Vector2D(-0.006,0.09));
	list.push_back(Vector2D(-0.006,0.06));
	list.push_back(Vector2D(0,0.03));
	list.push_back(Vector2D(0,0));
	auxPrism->setPolygonalBase(list);
	list.clear();
	auxPrism->setColor(1,1,0);
	auxPrism->setHeight(0.005);	
	auxPrism->setRelativePosition(Vector3D(0.015,0.01,-0.005));
	(*link)+=auxPrism;

	link->LinkTo(joints[5]);
	links.push_back(link);

	//Tcp
	tcp=new Tcp();
	tcp->setName("Tcp");
	tcp->setRelativePosition(Vector3D(0,0,0.108));
	tcp->LinkTo(joints[5]);
	tcp->setDrawReferenceSystem(true);

	//getConfigurationOf(q_init,conf);

	actuators[0]->setSimulationParameters(PI/12);//	15º/seg
	actuators[1]->setSimulationParameters(23*PI/36);//	115º/seg
	actuators[2]->setSimulationParameters(23*PI/36);//	115º/seg
	actuators[3]->setSimulationParameters(14*PI/9);//	280º/seg
	actuators[4]->setSimulationParameters(5*PI/3);//	300º/seg
	actuators[5]->setSimulationParameters(5*PI/3);//	300º/seg

	(*this)+=links[0];
}

bool EUITIbotSim::getCoordinatesOf(vector<double> &_q)
{
	double v;
	_q.clear();
	for(int i=0;i<6;i++)
	{
		getJointValue(i,v);
		_q.push_back(v);
	}
	return true;
}

bool EUITIbotSim::inverseKinematics(Transformation3D t06, vector<double> &_q, unsigned char _conf)
{
	if(_conf == 0x00)
		_conf=getCurrentConfiguration();

	return EUITIbotInverseKinematics(t06,_q,_conf);
}

bool EUITIbotSim::EUITIbotInverseKinematics(Transformation3D t06, vector<double> &_q, unsigned char _conf)
{
//*****************************************
	vector<double> q_act;
	unsigned char c_act;
	getCoordinatesOf(q_act);//Save current coordiantes
	getConfigurationOf(q_act,c_act);
//*****************************************

	double L1=0.09550;
	double L2=0.108;
	double L3=0.128;
	double L6=0.103;

	if(_conf == 0x00)
		_conf=getCurrentConfiguration();

	double _s, _e;
	configuration(_conf,_s,_e);

	Vector3D c = t06.position - t06.getVectorW()*(L6);

//THETA 1
	double theta1 = atan2(c.y,c.x);
	if (theta1<0)theta1 = 2*PI + theta1;

//THETA 2
	double theta2;
	double m = sqrt(c.x*c.x + c.y*c.y + c.z*c.z);
	double r = sqrt(c.x*c.x + c.y*c.y + (c.z-L1)*(c.z-L1));
	double alpha = acos((L1*L1 + r*r - m*m )/2*r*L1);
	double beta = atan((L2*L2 + r*r - L3*L3 )/2*r*L2);

	if (r + L2 == m)theta2 = PI/2 - beta;
	else if (L2 + L3 == r)theta2 = alpha - PI/2;
	else if (L1 + L2 + L3 == m)theta2 = PI/2;
	else {theta2 = alpha - beta - PI/2;}

//THETA 3
	double theta3;
	double gamma = acos((L3*L3 + L2*L2 - r*r )/2*L2*L3);

	if(L2 + L3 == r)theta3 = PI/2;
	else {theta3 = 3*PI/2 - gamma;}

//DESACOPLO CINEMATICO
	OrientationMatrix m06,m03,m36;
	Matrix3x3 &aux03 = m03;
	m06=t06.orientation;

//Meto las 3 primeras coordenadas finales y corregidas
	setJointValue(0,theta1);
	setJointValue(1,theta2);
	setJointValue(2,theta3);
	Transformation3D auxm = joints[3]->getAbsoluteT3D();

	//Devuelvo la posicion previa
	for(int i=0;i<6;i++)
		setJointValue(i,q_act[i]);

	aux03=auxm.orientation;	
	m36=(m03.inverted())*m06;

//THETA 4
	double theta4 = asin(m36[2][3]/m36[3][3]);
//THETA 5
	double theta5 = acos(m36[3][3]);
//THETA 6
	double theta6 = atan(m36[3][2]/-m36[3][1]);

//CONFIGURACIONES
	_q.push_back(theta1);
	_q.push_back(theta2);
	_q.push_back(theta3);
	_q.push_back(theta4);
	_q.push_back(theta5);
	_q.push_back(theta6);
	for(int i=0;i<6;i++){if(fabs(_q[i])<EPS){_q[i]=0;}}

	return true;
}

bool EUITIbotSim::getConfigurationOf(const vector<double> &_q, unsigned char &_conf)
{
//Compruebo tamaño y rangos de coordenadas
	if(!checkJointValues(_q))return false;
	double _s,_e;

//HOMBRO
	if(_q[1]>PI/2)//Hombro arriba
		_s=1.0;
	else//Hombro abajo
		_s=-1.0;

//CODO
	if (_s == -1){
		if (_q[2] < PI/2) _e = -1;
		else{ _e = 1;}
	}else{
		if (_q[2] < PI/2) _e = 1;
		else{ _e = -1;} 
	}

	return configuration(_s,_e,_conf);
}


bool EUITIbotSim::configuration(unsigned char _conf, double& _s, double& _e)
{
	_s=_e=-1.0;
	if(_conf == INIT_EUITI_BOT)return true;
	if(_conf & SHOULDERRIGHT)_s=1.0;
	if(_conf & ELBOWDOWN)_e=1.0;
	return true;
}

bool EUITIbotSim::configuration(double _s, double _e, unsigned char &_conf)
{
	_conf = INIT_EUITI_BOT;
	if( _s == -1.0)_conf = (_conf|SHOULDERRIGHT);
	if( _e == -1.0)_conf = (_conf|ELBOWDOWN);
	return true;
}

void EUITIbotSim::simulate(double delta_t)
{
	RobotSim::simulate(delta_t);
}

};//Namespace mr
