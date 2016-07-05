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

#include "robotsim.h"
#include "../world.h"
#include "../referencesystem.h"
#include <iostream>

namespace mr
{
IMPLEMENT_MR_OBJECT(RobotSim)

void RobotSim::writeToXML(XMLElement *parent)
{
	ComposedEntity::writeToXML(parent);
}

void RobotSim::writeToXMLasRobotSim(XMLElement* parent)
{
	ComposedEntity::writeToXML(parent);
}

void RobotSim::readFromXML(XMLElement* parent)
{
	ComposedEntity::readFromXML(parent);
	int num=parent->GetChildrenNum();
	int objSize=objects.size();
	XMLElement** pObj=parent->GetChildren();
	XMLfile file(parent);
	for(int i=0;i<objSize;i++)
	{
		if(dynamic_cast<Tcp *> (objects[i])) tcp=dynamic_cast<Tcp *> (objects[i]);
	}
	for(int i=0;i<objSize;i++)
	{
		if(dynamic_cast<Actuator *> (objects[i])) 
		{
			Actuator* act=dynamic_cast<Actuator *> (objects[i]);
			actuators.push_back(act);
		}
	}
	for(int i=0;i<objSize;i++)
	{
		if(dynamic_cast<SimpleJoint *> (objects[i])) 
		{
			SimpleJoint* joint=dynamic_cast<SimpleJoint *> (objects[i]);
			joints.push_back(joint);
		}
	}
	for(int i=0;i<objSize;i++)
	{
		if((!dynamic_cast<Actuator *> (objects[i]))&&(!dynamic_cast<SimpleJoint *> (objects[i]))&&(!dynamic_cast<Tcp *> (objects[i])))
		{
			SolidEntity* link=dynamic_cast<SolidEntity *> (objects[i]);
			links.push_back(link);
		}
	}
}

char* RobotSim::CreateXMLText()
{
	XMLElement* elem=new XMLElement(0,"RobotSim");
	writeToXMLasRobotSim(elem);
	return elem->CreateXMLText();
}

void RobotSim::loadFromXMLText(char* XmlText)
{
	XML x;
	readFromXML(x.Paste(XmlText));
}


bool RobotSim::checkRobotColision()
{
//checks the collision of robot and the rest of the objects
	bool prev=isIntersectable();
	this->setIntersectable(false);
	World *w=getWorld();
	int i;
	for(i=0;i<(int)(objects.size());i++)
	{	
		SolidEntity *aux=dynamic_cast<SolidEntity *>(objects[i]);
		//en cuanto hay colision devuelvo true
		if(aux){
			if(aux!=this->links[0])
			if(w->checkCollisionWith(*aux))return true;
		}
	}
//checks the collision of grasped objects with the world
	if(tcp!=0){
		ReferenceSystem *tcpr=tcp->getReferenciableLocation();
		int n=tcpr->getNumberOfDependents();

		for(int j=0;j<n;j++)
		{
			ReferenceSystem* raux=tcpr->getDependent(j);
			SolidEntity *aux=dynamic_cast<SolidEntity *>(raux->getOwner());
			if(aux){
				if(w->checkCollisionWith(*aux))return true;
			}
		}

	}

//checks the collision of robot links between them
	setIntersectable(prev);
return false;
}
bool RobotSim::checkJointValues(const vector<double> & _q) const
{
//check dimension
	if(_q.size()!=joints.size())return false;
	for(int i=0;i<(int)joints.size();i++){
		if(joints[i]->checkValue(_q[i])==false)return false;
	}
	return true;
}
bool RobotSim::forwardKinematicsAbs(vector<double> _q, Transformation3D& t)
{
	int i;
	if(tcp==0)return false;
	if(links.size()==0)return false;

	if(!checkJointValues(_q))return false; //limits checking
	
	vector<double> qold;
	//save current joint values and update the joint values
	for(i=0;i<(int)joints.size();i++){
		qold.push_back(joints[i]->getValue());
		joints[i]->setValue(_q[i]);
	}

	// Compute the relative transformation between the tcp and the robot base
	t=tcp->getAbsoluteT3D();

	// Move the robot to the qold values
	for(i=0;i<(int)joints.size();i++)joints[i]->setValue(qold[i]);
	return true;
}
bool RobotSim::inverseKinematicsAbs(Transformation3D t, vector<double> &_q, unsigned char conf)
{
	Transformation3D t_base=links[0]->getAbsoluteT3D();
	Transformation3D aux=t_base.inverted();
	Transformation3D t_p=(aux)*(t);
	return inverseKinematics(t_p,_q,conf);
	
}
bool  RobotSim::forwardKinematics(const vector<double> & _q, Transformation3D& t)//conf puntero salida
{

	// Compute the relative transformation between the tcp and the robot base
	Transformation3D _t;
	forwardKinematicsAbs(_q,_t);
	Transformation3D aux=links[0]->getAbsoluteT3D();
	t=(aux.inverted())*_t;

	return true;
}
unsigned char RobotSim::getCurrentConfiguration()
{
	vector<double> q;
	unsigned char conf;
	//save current joint values and update the joint values
	for(int i=0;i<(int)joints.size();i++)q.push_back(joints[i]->getValue());
	getConfigurationOf(q,conf);
	return conf;
	
}

unsigned char RobotSim::getCurrentConfiguration(vector<double> _q) //used by linear path
{
	unsigned char conf;
	//we already know joints' values to next target
	getConfigurationOf(_q,conf);
	return conf;
}

Transformation3D RobotSim::getTcpAbsLocation()
{
	return tcp->getAbsoluteT3D();
}
Transformation3D RobotSim::getTcpLocation()
{
	Transformation3D _t=tcp->getAbsoluteT3D();
	Transformation3D aux=links[0]->getAbsoluteT3D();
	return (aux.inverted())*_t;
}


void RobotSim::simulate(double delta_t)
{
	if((int)q_target.size()==(int)joints.size())
	{

		double error=0;
		for(int i=0;i<(int)joints.size();i++)
			error+=fabs(q_target[i]-joints[i]->getValue());//*

		if( (time+delta_t) >= targetTime)//*
		{
			for(int i=0;i<(int)joints.size();i++)
				actuators[i]->setTarget(q_target[i]);

			q_target.clear();
			time=0.0;
			targetTime=0.0;
			
			via_point_flag=false; //end of vía point

			if (path_type==LINEAR)
			{
				if ((index_pos+1) != (int)all_q_values.size())
				{
					index_pos++;
					updateTargetAndTagetTime(index_pos);
				}
				else
				{
					all_q_values.clear();
					all_space_points.clear();
					return;
				}
			}				
		}

		else
		{

			/****************************************************************
				IF INTERPOLATOR SELECTED IS CUBIC POLINOMIAL TRAJECTORY TYPE 
			*****************************************************************/
			if (interpolator_position==CPT)
				for(int i=0;i<(int)actuators.size();i++){
					actuators[i]->computeTargetInterpolatorPolinomial(time);}

			/*****************************************************************
				IF INTERPOLATOR SELECTED IS TRAPEZOIDAL VELOCITY PROFILE TYPE 
			 *****************************************************************/
			else if (interpolator_position==TVP)
				for (int i=0;i<(int)actuators.size();i++)
					actuators[i]->computeTargetInterpolatorTVP(time);

			/*************************************************************
				IF INTERPOLATOR SELECTED IS SPLINE TRAJECTORY TYPE 
			**************************************************************/
			else if (interpolator_position==SPLINE)
				for(int i=0;i<(int)actuators.size();i++){
					actuators[i]->computeTargetInterpolatorPolinomial(time);}

			time+=delta_t;
		}
	}

	ComposedEntity::simulate(delta_t);
}



void RobotSim::computeTrajectoryTo(vector<double> q)
{
	q_target=q;//Target loaded
	double error=0;

	for(int i=0;i<(int)joints.size();i++)
		error+=fabs(q_target[i] - joints[i]->getValue());//*

	if(fabs(error) < EPS)
	{
		q_target.clear();
		return;
	}
	else
	{
		time=0.0;
		computeTargetTime();
	}
	return;
}


void RobotSim::updateTargetAndTagetTime(int index)
{ 
	q_target = all_q_values[index];
	computeTargetTime();
}


/* Methods to calculate the target time depending on the interpolator chosen */

void RobotSim::computeTargetTime()
{
	/*
		Following options are depending on type of trajectory and each one 
		calculates the target time for the actuators which control the movement
	*/

	if (joints.size()!=actuators.size())return;
	if (q_target.size()!=joints.size())return;

	/*************************************************************
		IF INTERPOLATOR SELECTED IS CUBIC POLINOMIAL  TYPE 
	**************************************************************/
	if (interpolator_position==CPT)
		computeTargetTimePolinomial();

	/******************************************************************
		IF INTERPOLATOR SELECTED IS TRAPEZOIDAL VELOCITY PROFILE TYPE 
	*******************************************************************/
	else if (interpolator_position==TVP)
		computeTargetTimeTVP();

	/******************************************************************
		IF INTERPOLATOR SELECTED IS SPLINE TYPE
	*******************************************************************/

	else if (interpolator_position==SPLINE)
		computeTargetTimePolinomial();

}

void RobotSim::computeTargetTimePolinomial()
{
	/*
		Here we are calculating the max time which one of the 
		actuators last in reaching the target
	*/

	double lowestSpeed=100.0;
	double longestPath=0.0;
	vector<double> pathJoint;

	for(int i=0;i<(int)actuators.size();i++)
	{
		pathJoint.push_back(q_target[i] - joints[i]->getValue());//Path each coordinate: target minus current coordinates

		if(actuators[i]->getMaxSpeed() <= lowestSpeed)
			lowestSpeed = actuators[i]->getMaxSpeed();
		if(fabs(pathJoint[i])>= longestPath)
			longestPath =fabs(pathJoint[i]);//*
	}
	targetTime = longestPath / lowestSpeed;

	//if interpolator is CPT
	if (interpolator_position == CPT)
	{
		for(int i=0;i<(int)actuators.size();i++)
		{
			actuators[i]->computeCubicPolinomialCoeficients(pathJoint[i],targetTime);
		}
		return;
	}
	//if interpolator is SPLINE
	else if (interpolator_position == SPLINE)
	{
		double Tk=0.1,stretch=0.1;
		int nIterations=(int)(targetTime/Tk);
		vector<double> a,b,c,d;
		for (int i=0;i<nIterations+2;i++)
		{
			a.push_back(Tk);
			b.push_back(4*Tk);//2*(Tk+Tk)
			c.push_back(Tk);
			d.push_back(6*stretch);//(3/(Tk^2))*(2*Tk^2*(qk+1-qk))
		}
		vector<double> veloc= TDMA(a,b,c,d,nIterations);
		
		for (int i=1;i<(int)actuators.size();i++)
		{
			actuators[i]->computeVelocIntermediates(veloc);
			actuators[i]->computeCubicPolinomialCoeficients(stretch,Tk);
		}
		return;
	}

}

void RobotSim::computeTargetTimeTVP()
{		
		
	/* Check which is the maximum time to get the target by all the joints.
	For this we obtain the max values of the speed and acceleration of the joints and check
	with their own path to travel  */

	double auxTargetTime=0.0,aux_time_acceleration=0.0;
	double max_time_acceleration=0.0,maxTargetTime=0.0;
	double maxSpeed=0.0,maxAcceleration=0.0;
	double condition=0.0;
	double TVP_acceleration_time=0.00;
	vector<double> path;
	int index=0;

	for(int i=0;i<(int)actuators.size();i++)
	{	
		maxSpeed=actuators[i]->getMaxSpeed();
		maxAcceleration=actuators[i]->getMaxAcceleration();

		condition=(maxSpeed*maxSpeed)/maxAcceleration;

		path.push_back(fabs(joints[i]->getValue()-q_target[i]));

		if (condition>path[i])//Bang Bang movement
		{
			auxTargetTime=2*sqrt(path[i]/maxAcceleration);//targetTime
			aux_time_acceleration=0.5*auxTargetTime;//acceleration time 
		}
		else // speed and acceleration with maximus values
		{		
			auxTargetTime=path[i]/maxSpeed + maxSpeed/maxAcceleration;//targetTime
			aux_time_acceleration=maxSpeed/maxAcceleration;// acceleration time
		}

		if (maxTargetTime<auxTargetTime)//choose the max values
		{
			index=i;
			maxTargetTime=auxTargetTime;
			max_time_acceleration=aux_time_acceleration;
		}

	}

	targetTime=maxTargetTime;
	TVP_acceleration_time=max_time_acceleration;

	if(max_time_acceleration==(0.5*maxTargetTime))
		actuators[index]->setPositionInterpolatorTVP("BangBang");
	else 
	{
		actuators[index]->setSpeed(actuators[index]->getMaxSpeed());
		actuators[index]->setAcceleration(actuators[index]->getMaxAcceleration());
		actuators[index]->setPositionInterpolatorTVP("MaximumSpeedAcceleration");
	}

	/*
		Now we are going to adjust the motion of all joints in TVP trajectory
	*/

	//we adjust the speed and acceleration to get all the joints are finishing the movement at the same time
	if(maxTargetTime<=0 && max_time_acceleration<=0)return;
	double speed=0,accel=0,maxSp=0,maxAccel=0;

	for(int i=0;i<(int)actuators.size();i++)
	{
		if(i==index)continue;
		maxSp=actuators[i]->getMaxSpeed();
		maxAccel=actuators[i]->getMaxAcceleration();

		//movement with speed and acceleration values adjust to timeTarget global
		speed=path[i]/(maxTargetTime-max_time_acceleration);
		accel=speed/max_time_acceleration;

		if (speed>maxSp || accel>maxAccel)//if true, try to change the movement to Bang Bang
		{
			accel=path[i]/(max_time_acceleration*max_time_acceleration);
			speed=accel*max_time_acceleration;

			if (speed>maxSp || accel>maxAccel)
				return; //we have to recalculate the  acceleration time to this actuator
			actuators[i]->setAcceleration(accel);
			actuators[i]->setSpeed(speed);
			actuators[i]->setPositionInterpolatorTVP("BangBang");
		}
		else
		{
			actuators[i]->setSpeed(speed);
			actuators[i]->setAcceleration(accel);
			actuators[i]->setPositionInterpolatorTVP("MaximumSpeedAcceleration");
		}
		
	}

	//Now set all the necessary attributes to simulate with this interpolator		
	int signMovement=1;
	for (int i=0;i<(int)actuators.size();i++)
	{
		if ((q_target[i]-joints[i]->getValue())<0)//compare the initial joint value
			signMovement=-1;//if we go to target in the negative cuadrant
		else 
			signMovement=1;
		actuators[i]->loadAttributesTVP(q_target[i],signMovement,  TVP_acceleration_time, targetTime);
	}

}


/*
	Method to calculate the linear path for any space point
*/

bool RobotSim::computeLinearPathAbs (Transformation3D td3d)
{	
	Transformation3D t_base=links[0]->getAbsoluteT3D();
	Transformation3D aux=t_base.inverted();
	Transformation3D t_p=(aux)*(td3d);

	return computeLinearPath(t_p);
} 


bool RobotSim::computeLinearPath (Transformation3D td3d)
{	
	/*
		Calculating the vector of intermediate positions
	*/
	Vector3D posIni(getTcpLocation().position);
	Vector3D posEnd(td3d.position);
	all_space_points.clear();
	all_q_values.clear();
	
	Vector3D direct_vec = posEnd - posIni; //direction vector
	double total_path = direct_vec.module(); //distance L = total path
	
	if (total_path<EPS)
		return false;
	
	double speed_to_achieve = 5.00; //velocity which will be imposed to all actuators
	double target_max_time = total_path / speed_to_achieve;
	
	double divisions = target_max_time * controlFrequency;
	int div_pos = (int)divisions;
	if (div_pos == 0)div_pos=1;
	direct_vec=direct_vec/div_pos;

	/*
		Calculating the intermediate orientations
	*/
	OrientationMatrix orientIni = td3d.orientation;//get final orientation
	OrientationMatrix orientEnd = getTcpLocation().orientation;//get current orientation
	//compute matrix Init->Final
	//OrientationMatrix orientInit_End = orientIni.transposed()*orientEnd;

	Quaternion q1;//quaternion initial
	Quaternion q2;//quaternion final and intermediate
	Quaternion q_inter;//interpolated quaternion
	
	orientIni.getQuaternion(q1);
	orientEnd.getQuaternion(q2);

	double t=0.00;
	double r=0.00,p=0.00,y=0.00;

	for (int i=0;i<div_pos;i++)
	{
		
		//position
		posIni.x+=direct_vec.x;
		posIni.y+=direct_vec.y;
		posIni.z+=direct_vec.z;

		//orientation
		t = i/(double)div_pos;//t -> [0.00,1.00]
		q_inter = computeOrientationSLERP(q1,q2,t);
		OrientationMatrix m = q_inter.getOrientationMatrix();
		m.getRPY(r,p,y);

		Transformation3D td3_final(posIni.x, posIni.y, posIni.z, r, p, y);
		all_space_points.push_back(td3_final);
		
	}// we already have all the intermediate positions and orientations (X,Y,Z,ROLL,PITCH,YAW)
	return true;
}

 /*
	Method to interpolate orientation in a robot movement.
	It uses the SLERP method which calculates intermediate quaternions.

	SLERP(qa,qb,u) = qm = (qa*sin((1-t)*theta)+qb*sin(t*theta))/sin(theta)
		qm = interpolated quaternion
		qa = quaternion a (first quaternion to be interpolated between)
		qb = quaternion b (second quaternion to be interpolated between)
		t = a scalar between 0.0 (at qa) and 1.0 (at qb)
		theta is half the angle between qa and qb
 */

Quaternion RobotSim::computeOrientationSLERP(Quaternion qa, Quaternion qb, double t)
{
	if (t>1.00)
		t = 1.00;

	Quaternion q, qm;
	
	//compute matrix Init->Final
	//OrientationMatrix orientInit_End = orientIni.transposed()*orientEnd;
	//orientInit_End.getQuaternion(q);
	
	qa = qa.inverse();
	q = qa * qb;//q wich connects qa with qb -> q = qa^-1*qb

	// Calculate angle between them.
	double cosHalfTheta = q.scal;
	// if qa=qb or qa=-qb then theta = 0 and we can return qa
	if (abs(cosHalfTheta) >= 1.0){
		return qa;//qm = qa
	}

	// Calculate temporary values.
	double halfTheta = acos(cosHalfTheta);
	double sinHalfTheta = sqrt(1.00 - cosHalfTheta*cosHalfTheta);
	// if theta = 180 degrees then result is not fully defined
	// we could rotate around any axis normal to qa or qb
	if (fabs(sinHalfTheta) < EPS){ // fabs is floating point absolute
		qm = qa*0.5 + qb*0.5;
		return qm;
	}
	double ratioA = sin((1 - t) * halfTheta) / sinHalfTheta;
	double ratioB = sin(t * halfTheta) / sinHalfTheta; 
	//calculate Quaternion
	qm = qa*ratioA + qb*ratioB;

	return qm;

	//qm = qa*(sin((1-t)*theta)/sin(theta)) + qb*(sin(t*theta)/sin(theta));
}



/*
	Method to calculate the via point in a trajectory with two different targets
*/
bool RobotSim::computeViaPoint(Transformation3D td3_a_prim,Transformation3D td3_b,Transformation3D td3_c_prim)
{
	/*
	When robot have gone over the 90% of total distance 
	if there is a new target to go, it'll start to change 
	its direction to the other target --> Via Point

	Via point ---> (A'-->C')
			 
			B
			/\
		   /  \
		  A'---C'
		 /      \
		/        \
	   /		  \
	  A		       \				
					C

	Math to calculate trajectory in via point
	**********************************************************
	* p(t) = A' + v1*Kab*t +t^2/(2*(Tc'-Ta'))*(v2*Kbc-v1*Kab)*
	**********************************************************
	
	*/

	
	/*
		Calculating the vector of intermediate positions
	*/

	Vector3D pos_a_prim(td3_a_prim.position);
	Vector3D pos_b(td3_b.position);
	Vector3D pos_c_prim(td3_c_prim.position);

	//all_space_points.clear();
	//all_q_values.clear();

	//direction vectors
	Vector3D Kab = pos_b - pos_a_prim; 
	Vector3D Kbc = pos_c_prim - pos_b;
	Vector3D pos;

	//unit vectors
	Kab = Kab.normalize();
	Kbc = Kbc.normalize();

	//lenght
	double d1 = Kab.module();
	double d2 = Kbc.module();

	if (d1<=EPS || d2<=EPS)
		return false;

	double veloc_max=5.00, acel_max = 1.00;//rad/s and rad/s^2

	//we supposed v1 = v2 = vmax
	double delta_time = (veloc_max/acel_max)*((Kbc-Kab).module());

	double divisions = delta_time * controlFrequency;
	int div_pos = (int)divisions;
	if (div_pos == 0)div_pos=1;

	/*
		Calculating the intermediate orientations
	*/
	OrientationMatrix orientIni = td3_a_prim.orientation;//get final orientation
	OrientationMatrix orientEnd = td3_c_prim.orientation;//get current orientation

	Quaternion q1;//quaternion initial
	Quaternion q2;//quaternion final and intermediate
	Quaternion q_inter;//interpolated quaternion
	
	orientIni.getQuaternion(q1);
	orientEnd.getQuaternion(q2);

	double t=0.00;
	double r=0.00,p=0.00,y=0.00;

	for (int i=0;i<div_pos;i++)
	{
		t = i/(double)div_pos;//t -> [0.00,1.00]	

		//position
		pos = pos_a_prim + Kab*(veloc_max*t) + (Kbc*veloc_max-Kab*veloc_max)*(((t*t)/delta_time)*0.5);

		//orientation
		q_inter = computeOrientationSLERP(q1,q2,t);
		OrientationMatrix m = q_inter.getOrientationMatrix();
		m.getRPY(r,p,y);

		Transformation3D td3_final(pos.x, pos.y, pos.z, r, p, y);
		all_space_via_points.push_back(td3_final);
		
	}// we already have all the intermediate positions and orientations (X,Y,Z,ROLL,PITCH,YAW)
	return true;
}


/***************************************************
	Different methods to move the joints or robot
****************************************************/


bool  RobotSim::moveTo(Transformation3D t3d, unsigned char conf)
{
/*
the T3D is a relative transformation to the base (link[0])
*/

	if(conf==0x0)conf=getCurrentConfiguration();

	vector<double> q;
	if(inverseKinematics(t3d,q,conf))
	{
		for(int i=0;i<(int)joints.size();i++)
			joints[i]->setValue(q[i]);
		return true;
	}
	else return false;

}

bool  RobotSim::moveToAbs(Transformation3D t3d, unsigned char conf)
{

	Transformation3D t_base=links[0]->getAbsoluteT3D();
	Transformation3D aux=t_base.inverted();
	Transformation3D t_p=(aux)*(t3d);

	return moveTo(t_p,conf);
}

bool RobotSim::moveTo(double *_q)
{
	int num=(int)joints.size();
	vector<double> q;
	for(int i=0;i<num;i++)q.push_back(_q[i]);
	return moveTo(q);
}

bool RobotSim::moveTo(const vector<double> & _q)
{

	if(_q.size()!=joints.size())return false;
	if(!checkJointValues(_q))return false; //limits checking
	for(int i=0;i<(int)joints.size();i++)joints[i]->setValue(_q[i]);
	//computeTrajectoryTo(_q);
	return true;
}

/*
	Copmpute absolute and relative coordinates
*/

bool RobotSim::computeTrajectoryToAbs(Transformation3D t)
{
	unsigned char conf=getCurrentConfiguration();
	vector<double> q;
	
	if (path_type==LINEAR)
	{
		computeLinearPathAbs(t);//convert to relative space points
		int size = (int)all_space_points.size();
		if (!size)
			return false;
		for (int i=0;i<size;i++)
		{
			q.clear();
			if(!inverseKinematics(all_space_points[i],q,conf))
				return false;
			all_q_values.push_back(q);
			conf=getCurrentConfiguration(q);
		}
		q = all_q_values[0];
		index_pos = 0;
	}
	else if (path_type==SYNC_JOINT)
		if(!inverseKinematicsAbs(t,q))
			return false;

	computeTrajectoryTo(q);
	return true;
}

bool RobotSim::computeTrajectoryTo(Transformation3D t)
{
	unsigned char conf=getCurrentConfiguration();
	vector<double> q;
	
	if (path_type==LINEAR)
	{
		computeLinearPath (t);
		int size = (int)all_space_points.size();
		if (!size)
			return false;
		for (int i=0;i<size;i++)
		{
			q.clear();
			if(!inverseKinematics(all_space_points[i],q,conf))
				return false;
			all_q_values.push_back(q);
			conf=getCurrentConfiguration(q);
		}
		q = all_q_values[0];
		index_pos = 0;
	}
	else if (path_type==SYNC_JOINT)
		if(!inverseKinematics(t,q,conf))
			return false;

	computeTrajectoryTo(q);
	return true;
}


/*
	Simplified method to resolve the tridiagonal matrix of velocities in TVP trajectory
*/

vector<double> RobotSim::TDMA (vector<double> a, vector<double> b, vector<double> c,vector<double> d, int nIterations)
{
	/*It's the Thomas algorithm to resolve tridiagonal matrix. It's useful 
	to calculate the intermediate values of the velocity in the points to 
	interpolate (vk).

	c'1=c1/b1
	c'i= ci/(bi-c'[i-1]*ai)		i=2...n-1

	d'1=d1/b1
	d'i=(di-d'[i-1]*ai)/(bi-c'[i-1]*ai)	i=2...n

	xn=d'n
	xi=d'i-c'i*x[i+1]	i=n-1....1
	*/

	double denom;
	vector<double> gamma,betta,x;
	gamma.push_back(0);
	betta.push_back(0);
	
	int i=0;

	//a=2*(Tk+Tk)=4*Tk; b=Tk; c=Tk;
	//dk = (3/(Tk*Tk))*((Tk*Tk*(path))+(Tk*Tk*(path)));
	//dk = (3*(2*path))
	
	//dk = 6*path; //more simplificate

	for (i=1;i<=nIterations;i++)
	{
		denom=a[i]*gamma[i-1]+b[i];
		if(i!=nIterations)
			gamma.push_back((-c[i])/denom);
		betta.push_back((d[i]-(a[i]*betta[i-1]))/denom);
	}
	
	//calculate the solutions
	x.push_back(betta[nIterations]);
	int index=0;

	for (i=nIterations-1;i>=1;i--)
	{
		x.push_back(gamma[i]*x[index++]+betta[i]);
	}

	//reordenate the vector  to return all the velocities in order
	vector<double> veloc;

	veloc.push_back(0.0);//veloc init

	for(i=0;i<(int)x.size();i++)
	{
		veloc.push_back(x[i]);
	}
	veloc.push_back(0.0);//veloc final

	return veloc;

}



};//Namespace mr