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

#ifndef __PUMA560_SIM_H_
#define __PUMA560_SIM_H_


#include "mrcore.h"
#include "robotsim.h"

#define ELBOW_UP	0x80
#define ELBOW_DOWN	0x81

#define SHOULDER_LEFT	0x80
#define SHOULDER_RIGHT	0x82

namespace mr
{

class Puma560Sim : public RobotSim
{
	DECLARE_MR_OBJECT(Puma560Sim)

public:
	//Serializers
	virtual void writeToStream(Stream& stream){}
	virtual void readFromStream(Stream& stream){}

	//Constructor
	Puma560Sim(void);

	//expose the saving to log file service
	bool saveDataTo(DataLogOut* log, string name)
	{
		return saveDataTo(log,name);
	}

	virtual int Home(){return 1;}
	virtual int SetAccel(double acc_pan, double acc_tilt){return 1;}
	virtual int SetSpeed(double vel_pan, double vel_tilt){return 1;}
	virtual int Move(double pos_pan, double pos_tilt){return 1;}

	void setFlash();



//Forward PUMA 560 kinematics rel and abs
	bool forwardKinematics(vector<double> _q, Transformation3D& t);
	bool forwardKinematicsAbs(vector<double> _q, Transformation3D& t);

//Inverse PUMA 560 kinematics rel and abs
	bool inverseKinematics(Transformation3D t, vector<double> &_q, unsigned char conf=NULL);
	bool inverseKinematicsAbs(Transformation3D t, vector<double> &_q, unsigned char conf=NULL);

//Specific inverse kinematics Puma 560
	bool PUMA560inverseKinematics(Transformation3D t, vector<double> &_q, unsigned char conf=NULL);

//Movements methods
	bool moveTo(vector<double> _q);

//Return robot configuration
	void getConfiguration(float& _shoulder, float& _elbow);

	virtual void simulate(double delta_t);//time interval in seconds
	virtual void drawGL();

protected:
	SimpleJoint *gof[6];
/***
Rangos sacados de las hojas de caracteristicas:
	gof[0]	->	320º	->[8*PI/9 , -8*PI/9]
	gof[1]	->	250º	->[25*PI/36 , -25*PI/36]
	gof[2]	->	270º	->[3*PI/4 , -3*PI/4]
	gof[3]	->	280º	->[7*PI/9 , -7*PI/9]
	gof[4]	->	200º	->[5*PI/9,-5*PI/9]
	gof[5]	->	532º	->[133*PI/45,-133*PI/45]
***/
	double q[6];
	Tcp *tcp;

	double speed;

	float elbow;
	float shoulder;

	bool move;//Si el Puma se esta moviendo, este valor es verdadero

private:
	CylindricalPart *cyl[15];
	PrismaticPart *pris[4];

	//Aux variables
	int i;
};

};//end namespace mr

#endif





