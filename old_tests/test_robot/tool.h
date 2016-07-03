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

#ifndef __TOOL_H_
#define __TOOL_H_

/*
#include "hw/powercube70.h"
#include "base/globject.h"
#include "../../world/composedentity.h"
#include "../../world/simplejoint.h"
*/

#include "mrcore.h"

namespace mr
{

class Tool : public ComposedEntity
{
	DECLARE_MR_OBJECT(Tool)

public:
	//Serializers
	virtual void writeToStream(Stream& stream){}
	virtual void readFromStream(Stream& stream){}

	//Constructor
	Tool(void);

	//expose the saving to log file service
	bool saveDataTo(DataLogOut* log, string name)
	{
		return saveDataTo(log,name);
	}

	virtual void drawGL();
	int GetPos(double& val);
	void setMove(double val);


protected:

	SimpleJoint *joint[2];
	Tcp *tcp;//Estara situado en la parte media de las pinzas del util


};

};//end namespace mr

#endif