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

#include "tool.h"
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
IMPLEMENT_MR_OBJECT(Tool)

Tool::Tool(void)
{
	name="Work_tool";

	PrismaticPart *base=new PrismaticPart();
	base->setName("base_pinza");
	vector<Vector2D> list;
	list.push_back(Vector2D( 0.017, 0.017));
	list.push_back(Vector2D(-0.017, 0.017));
	list.push_back(Vector2D(-0.017,-0.017));
	list.push_back(Vector2D( 0.017,-0.017));
	base->setPolygonalBase(list);
	base->setHeight(0.02);
	base->setColor(1,0.5,0);
	list.clear();

	PrismaticPart *soporte=new PrismaticPart();
	soporte->setName("soporte");
	list.push_back(Vector2D( 0.017, 0.027));
	list.push_back(Vector2D(-0.017, 0.027));
	list.push_back(Vector2D(-0.017,-0.027));
	list.push_back(Vector2D( 0.017,-0.027));
	soporte->setPolygonalBase(list);
	list.clear();
	soporte->setColor(1,0.5,0);//Naranja
	soporte->setRelativePosition(Vector3D(0,0,0.02));
	soporte->setHeight(0.014);
	soporte->LinkTo(base);

	PrismaticPart *guia=new PrismaticPart();
	guia->setName("guia");
	list.push_back(Vector2D( 0.01, 0.027));
	list.push_back(Vector2D(-0.01, 0.027));
	list.push_back(Vector2D(-0.01,-0.027));
	list.push_back(Vector2D( 0.01,-0.027));
	guia->setPolygonalBase(list);
	list.clear();
	guia->setColor(0.2,0.2,0.2);
	guia->setRelativePosition(Vector3D(0,0,0.034));
	guia->setHeight(0.02);
	guia->LinkTo(base);

	joint[0]=new SimpleJoint(0.0125,0.005,true,0,Y_AXIS,true);
	joint[0]->setName("Articulacion prismatica 0");
	joint[0]->setRelativePosition(Vector3D(0,0,0.054));
	joint[0]->LinkTo(base);

	joint[1]=new SimpleJoint(0.0125,0.005,false,0,Y_AXIS,true);
	joint[1]->setName("Articulacion prismatica 1");
	joint[1]->setRelativePosition(Vector3D(0,0,0.054));
	joint[1]->LinkTo(base);


	PrismaticPart *pinza0=new PrismaticPart();
	pinza0->setName("Pinza 0");
	list.push_back(Vector2D(0,0));
	list.push_back(Vector2D(0.03,0));
	list.push_back(Vector2D(0.03,0.018));
	list.push_back(Vector2D(0.015,0.05));
	list.push_back(Vector2D(0,0.05));
	list.push_back(Vector2D(0,0.02));
	list.push_back(Vector2D(0.015,0.02));
	list.push_back(Vector2D(0.015,0.015));
	list.push_back(Vector2D(0,0.015));
	pinza0->setPolygonalBase(list);
	pinza0->setColor(1,1,1);
	pinza0->setHeight(0.04);
	pinza0->setRelativePosition(Vector3D(-0.02,0.005,0));
	pinza0->setRelativeOrientation(PI/2,0,PI/2);
	pinza0->LinkTo(joint[0]);

	PrismaticPart *pinza1=new PrismaticPart();
	pinza1->setName("Pinza 1");
	pinza1->setPolygonalBase(list);
	list.clear();
	pinza1->setColor(1,1,1);
	pinza1->setHeight(0.04);
	pinza1->setRelativePosition(Vector3D(0.02,-0.005,0));
	pinza1->setRelativeOrientation(PI/2,0,-PI/2);
	pinza1->LinkTo(joint[1]);

	tcp=new Tcp();
	tcp->setRelativePosition(Vector3D(0,0,0.085));
	tcp->LinkTo(base);
	(*this)+=base;
}

void Tool::drawGL()
{
	ComposedEntity::drawGL();
	//tcp->setDrawReferenceSystem(true);
}

int Tool::GetPos(double& val)
{
/***
No se si esto es correcto, ya que se usa la misma variable para las dos pinzas,
pero ya que se tienen que mover al mismo tiempo y estar siempre simetricas, puede
servir.
***/
	val=(double)joint[0]->getValue();
	val=(double)joint[1]->getValue();
	return 1;
}

void Tool::setMove(double val)
{
	joint[0]->setValue(val);
	joint[1]->setValue(val);
}

};//Namespace mr