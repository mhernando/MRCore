/**********************************************************************
 *
 * This code is part of the MRcore project
 * Author:  Rodrigo Azofra Barrio & Miguel Hernando Gutierrez &
 *			Francisco Ramirez de Anton Montoro
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

#ifndef __ROBOT_SIM_H_
#define __ROBOT_SIM_H_

#include "../composedentity.h"
#include "../simplejoint.h"
#include "../cylindricalpart.h"
#include "../tcp.h"
#include "../actuator.h"

namespace mr
{
/*!
    \class RobotSim
    A generic class for robot manipulators. Includes a series of interfaces that allows to
	share common methods and operations. When a new Robot is defined, the series of links and
	joints have to be included into the protected vectors. The same have to be done with the tcp.
	Check the AdeptOne class as example.
	The Inverse and Direct Kinematics is defined between the poses of the links[0] and the tcp.
	Usually the relative position of the links[0] is the same than the RoboSim... but is a posible degree
	of freedom, if mathematically is easiest to proceed in a diferent way.

	DESIGN INFO:
	
	*/

enum PathType //Linear path or synchronous path
		{
			SYNC_JOINT,
			LINEAR,
		};

enum OrientationInterpolator //types of orientation interpolators
		{
			SLERP,
			NIELSON_SHIEH,
		};

class RobotSim : public ComposedEntity 
{
	DECLARE_MR_OBJECT(RobotSim)


public:
	//Serializers
	virtual void writeToStream(Stream& stream){}
	virtual void readFromStream(Stream& stream){}
	virtual void writeToXML(XMLElement* parent);
	virtual void readFromXML(XMLElement* parent);
	virtual char* CreateXMLText();
	virtual void loadFromXMLText(char* XmlText);
	//Implemented to save a defined robotic class as robotSim
	void writeToXMLasRobotSim(XMLElement* parent);

//Constructor
	RobotSim(void):tcp(0),interpolator_position(TVP),controlFrequency(100),
		path_type(LINEAR), interpolator_orientation(SLERP){}

//Set and get i-joint value
	virtual bool setJointValue(int i,double val) {if(i<(int)joints.size())joints[i]->setValue(val);else return false; return true;}
	virtual bool getJointValue(int i,double& val){if(i<(int)joints.size())val=joints[i]->getValue();else return false; return true;}

//limits checking: generic
	bool checkJointValues(const vector<double> & _q) const; 
//retrieves the robot configuration. If are invalid qs returns false. Have to be redefined for each robot
	virtual bool getConfigurationOf(const vector<double> & _q,unsigned char &conf){conf = 0x80;return true;} 
	unsigned char getCurrentConfiguration();
	unsigned char getCurrentConfiguration(vector<double> _q);
	Transformation3D getTcpAbsLocation();
	Transformation3D getTcpLocation();
//Forward and inverse kinematics Relative. The inverse kinematics must be defined for each new class of robot 
	virtual bool forwardKinematics(const vector<double> &_q, Transformation3D& t);
//Inverse kinematics
	virtual bool inverseKinematics(Transformation3D t06, vector<double> &_q, unsigned char conf=NULL){return true;}

//Forward and inverse kinematics Absolute: generic methods. 
	virtual bool forwardKinematicsAbs(vector<double> _q, Transformation3D& t);
	virtual bool inverseKinematicsAbs(Transformation3D t, vector<double> &_q, unsigned char conf=0x00);

//Movements methods: generic
	virtual bool moveTo(const vector<double> &_q);
	virtual bool moveTo(Transformation3D t3d, unsigned char conf=0x0);
	virtual bool moveToAbs(Transformation3D t3d, unsigned char conf=0x0);

	//Movements methods: generic but not safe
	bool moveTo(double *_q);
//Simulation of time 
	virtual void simulate(double delta_t);//time interval in seconds
//data interface
	int getNumJoints(){return (int)joints.size();}
	bool getJointLimits(int i, double &max, double &min){
		if((i<0)||(i>=getNumJoints()))return false; 
		joints[i]->getMaxMin(max,min);
		return true;
	}
//returns a vector with a copy of the pointers included in joints
	vector<SimpleJoint *> getJoints(){return joints;}

//returns the address of the i-th joint 
	SimpleJoint *getJoint(int i){
		if((i<0)||(i>=getNumJoints()))return 0; 
		return joints[i];
	}
//Specific Collision checking
	bool checkRobotColision();

//Kinematic simulation methods
	bool checkRobotIsMoving(){
		for(int i=0;i<(int)actuators.size();i++){
			if(actuators[i]->isMoving())
				return true;
		}
		return false;
	}

	void computeTargetTime();//general function to compute the target time depending on interpolator
	void computeTargetTimePolinomial();//CPT and SPLINE
	void computeTargetTimeTVP();//TVP

	void setControlFrequency (float _freq){controlFrequency = _freq;}
	float getControlFrequency () {return controlFrequency;}

//Load T3D relative and absolute
	virtual bool computeTrajectoryTo(Transformation3D t);
	virtual bool computeTrajectoryToAbs(Transformation3D t);

//Load values to move all the robot's joint
	virtual void computeTrajectoryTo(vector<double> _q);

//Selection type of interpolator and movement
	virtual bool setPositionInterpolator (PositionInterpolator _type){
		if (checkRobotIsMoving()) return false;
		for (int i=0;i<(int)actuators.size();i++){
			actuators[i]->setPositionInterpolator(_type);
		}
		interpolator_position=_type;
		return true;
	}	
	
	virtual PositionInterpolator getPositionInterpolator (){
		return interpolator_position;
	}
	
	virtual bool setOrientationInterpolator (OrientationInterpolator _type){
		if (checkRobotIsMoving()) return false;
		interpolator_orientation = _type;
		return true;
	}

	virtual bool setPathType (PathType _type){
		if (checkRobotIsMoving()) return false;
		path_type=_type;
		return true;
	}	

//Spline trajetory (interpolation points)
	 /* 
	 Thomas Algorithm for Tridiagonal Matrix
	
			| b1 c1	0   .	0	|
			| a2 b2 c2	.	.   |
		M = | 0	 a3  .	.	0   |
			| .	 .  .	.   cn-1|
			| 0	 .	0   an	bn  |
	*/
	vector<double> TDMA (vector<double> a, vector<double> b, vector<double> c,vector<double> d, int nIterations);


//Methods to draw graphs of position, velocity and acceleration of all joints
	vector<double> getDataGraphPosition ();
	vector<double> getDataGraphVelocity ();
	vector<double> getDataGraphAcceleration();


protected:
	
//Methods to linear path movement (position and orientation)
	bool computeLinearPath (Transformation3D td3d);
	bool computeLinearPathAbs (Transformation3D td3d);
	void updateTargetAndTagetTime(int index);
	Quaternion computeOrientationSLERP(Quaternion qa, Quaternion qb, double t);

//via point between two targets
	bool computeViaPoint(Transformation3D td3_a,Transformation3D td3_b,Transformation3D td3_c);

//redundant information to easily access the kinematic chain and joint controllers
	vector<SolidEntity *> links;
	vector<SimpleJoint *> joints;
	vector<Actuator*> actuators;
	Tcp *tcp;
	
//cinematic simulation atributes

	double time; //time consumed since the beginning of the trajectory
	double targetTime; //time to achieve the trajectory

	vector<double> q_init;// initial joint values
	vector<double> q_target;
	vector<double> next_q_target;

	PositionInterpolator interpolator_position; //TVP, CPT or SPLINE
	OrientationInterpolator interpolator_orientation; //SLERP, NIELSON_SHIEH
	PathType path_type;
	float controlFrequency; //Hz 

//Specific linear path trajectory
	vector<Transformation3D> all_space_points;//sequence of linear path intermediate points (T3D)
	vector<vector<double>> all_q_values;//sequence of linear path intermediate joints value
	int index_pos;

//detect vía points
	bool via_point_flag;//changing target between trajectory segments
	vector<Transformation3D> all_space_via_points;

};

};//end namespace mr

#endif