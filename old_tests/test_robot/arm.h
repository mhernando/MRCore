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

#ifndef __ARM_H_
#define __ARM_H_

#include "mrcore.h"

namespace mr
{

class Arm : public ComposedEntity , public DataSource
{
	DECLARE_MR_OBJECT(Arm)

public:
	//Serializers
	virtual void writeToStream(Stream& stream){}
	virtual void readFromStream(Stream& stream){}

	//Constructor
	Arm(void);
	Arm(double _long, Vector3D pos);

	//expose the saving to log file service
	bool saveDataTo(DataLogOut* log, string name)
	{
		return saveDataTo(log,name);
	}

	virtual int Home(){return 1;}
	virtual int SetAccel(double acc_pan, double acc_tilt){return 1;}
	virtual int SetSpeed(double vel_pan, double vel_tilt){return 1;}
	virtual int Move(double pos_pan, double pos_tilt){return 1;}

protected:

	Tcp *tcp;
	SimpleJoint *joint;
	double longitud;

private:
	CylindricalPart *arm;

};

};//end namespace mr

#endif