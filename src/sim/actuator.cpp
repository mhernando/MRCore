/**********************************************************************
 *
 * This code is part of the MRcore project
 * Author:  Francisco Ramirez de Anton Montoro
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


#include "actuator.h"

#include <math.h>

namespace mr{
IMPLEMENT_MR_OBJECT(Actuator)

Actuator::Actuator(double _speed,double _maxSpeed,double _acceleration, double _maxAcceleration)
{
	name="Actuator";
	s_Joint=0;
	maxSpeed=_maxSpeed;
	speed=_speed;
	target=0;
	targetActive=false;
	acceleration=_acceleration;
	maxAcceleration=_maxAcceleration;

	a0=a1=a2=a3=0.0;	
	frequency=1000;

	PositionInterpolatorTVP="MaximumSpeedAcceleration";

	index_veloc_intermediates=0;
	interpolator_position=TVP;
}


Actuator::~Actuator(void)
{

}

void  Actuator::setSimulationParameters(double _maxSpeed,  double _maxAcceleration)
{
	if(_maxSpeed<0)
		return;
	maxSpeed=_maxSpeed; 
	if(_maxAcceleration<=0)
		return;
	maxAcceleration=_maxAcceleration;
}


double Actuator::setSpeed(double sp)
{
	if(sp<0)return speed; 
	speed=sp>maxSpeed?maxSpeed:sp;
	return speed;
}

double Actuator::setAcceleration(double ac)
{
	if(ac<0)return acceleration; 
	acceleration=ac>maxAcceleration?maxAcceleration:ac;
	return acceleration;
}

bool Actuator::setTarget(double val)
{

	double valMax,valMin;
	s_Joint->getMaxMin(valMax,valMin);

	if((val<valMin)||(val>valMax))
		return false;
	target=val;
	targetActive=true;
	return true;
}

//bool Actuator::setTargetIntermediate(double _time)
//{
//
//	if (s_Joint)
//	{	
//		double _targetInterm = a0 + a1*_time + a2*_time*_time + a3*_time*_time*_time;
//		double valMax,valMin;
//		s_Joint->getMaxMin(valMax,valMin);
//		if((_targetInterm<valMin)||(_targetInterm>valMax))
//		{
//			targetActive=false;
//			return false;
//		}
//		targetIntermediate=_targetInterm;
//		targetActive=true;
//		return true;
//	}
//	else
//		return false;
//}



void Actuator::simulate(double delta_t)
{
		
	if(targetActive==false)
		return;
	
	double value=s_Joint->getValue();
	double d=target-value;	
	if(interpolator_position==CPT)
	{	
		double inc=delta_t*speed;
		if(d<0)inc=((-inc<d)?d:(-inc));
		else inc=(inc>d?d:inc);
		if(d<EPS){
			targetActive=false;
			s_Joint->setValue(target);
		}
		else s_Joint->setValue(value+inc);
	}

	else if(interpolator_position==TVP || interpolator_position==SPLINE)
	{
		if(fabs(d)<EPS)
			targetActive=false;
		else
			s_Joint->setValue(target);
	}

}


void Actuator::linkTo (PositionableEntity *p)
{
	SimpleJoint* simpJoint = dynamic_cast <SimpleJoint*>(p);
	
	if (simpJoint)
	{
		s_Joint=simpJoint;
		PositionableEntity::LinkTo(p);
	}
}


/******************************************************************************
	SPECIFIC METHODS CUBIC POLINOMIAL AND SPLINE INTERPOLATORS
*******************************************************************************/
void Actuator::computeCubicPolinomialCoeficients(double path_joint,double targetTime)
{
	if (interpolator_position==CPT)
	{
		a0=s_Joint->getValue();//Current coordiantes
		a1=0.0;
		a2=( 3*(path_joint)/(targetTime*targetTime));
		a3=(-2*(path_joint)/(targetTime*targetTime*targetTime));
	}
	else if (interpolator_position==SPLINE)
	{
		double stretch=path_joint;
		double Tk=targetTime;

		if(index_veloc_intermediates>=(int)velocInter.size())return;
		a0=s_Joint->getValue();//Current coordiantes
		a1=velocInter[index_veloc_intermediates];
		a2=( 1/Tk)*((3*(stretch)/(Tk))-2*velocInter[index_veloc_intermediates]-velocInter[index_veloc_intermediates+1]);
		a3=(1/(Tk*Tk))*(((-2*(stretch))/Tk)+velocInter[index_veloc_intermediates]+velocInter[index_veloc_intermediates+1]);

		index_veloc_intermediates++;
	}
}
void Actuator::computeTargetInterpolatorPolinomial(double _time)
{
	double val=0.00,sp=0.00;

	val=a0 + a1*_time + a2*square(_time) + a3*square(_time)*_time;

	if (interpolator_position == CPT)
	{
		sp=a1+ 2*a2*_time + 3*a3*square(_time);
		setSpeed(sp);
	}

	setTarget(val);
}

/******************************************************************
	SPECIFIC METHODS INTERPOLATOR TRAPEZOIDAL VELOCITY PROFILE
*******************************************************************/

void Actuator::loadAttributesTVP(double _q_target, int _signMovement, double _TVP_acceleration_time, double targetTime)
{
	signMovement = _signMovement;
	q_init = s_Joint->getValue();//initial value
	q_target = _q_target;	
	initial_time = 0.00;
	target_time = targetTime;
	signMovement = signMovement;
	TVP_acceleration_time = _TVP_acceleration_time;
}

void Actuator::computeTargetInterpolatorTVP(double _time)
{
	double val=0.00;

    if(getPositionInterpolatorTVP()=="MaximumSpeedAcceleration")
    {
        //Acceleration phase
        if (_time<(initial_time+TVP_acceleration_time) && _time>=initial_time)
			val=q_init+signMovement*((getAcceleration()*0.5)*square(_time-initial_time));

        //Velocity constant phase
        if (_time>=(initial_time+TVP_acceleration_time) && _time<=(target_time-TVP_acceleration_time))
			val=q_init+signMovement*(getSpeed()*(_time-initial_time-TVP_acceleration_time*0.5));

        //Deceleration phase
        if (_time>(target_time-TVP_acceleration_time) && _time<=target_time)
			val=q_target-signMovement*((getAcceleration()*0.5)*square(target_time-_time));
    }
    else if(getPositionInterpolatorTVP()=="BangBang")
    {
        //Acceleration phase
        if (_time<(target_time*0.5) && _time>=initial_time)
			val=q_init+signMovement*((getMaxAcceleration()*0.5)*square(_time-initial_time));

        //Deceleration phase
        if (_time>=(target_time*0.5) && _time<=target_time)
			val=q_target-signMovement*((getMaxAcceleration()*0.5)*square(target_time-_time));
    }

    setTarget(val);

}

bool Actuator::setPositionInterpolatorTVP(string _type)
{
	PositionInterpolatorTVP = string();

	if(_type=="BangBang" || _type=="MaximumSpeedAcceleration")
	{
		PositionInterpolatorTVP=_type;
		return true;
	}

	return false;
}

/******************************************************************
	METHOD TO SET VELOC. INTERMEDIATES IN SPLINE INTERPOLATOR
*******************************************************************/
void Actuator::computeVelocIntermediates (vector<double> veloc)
{
	double auxsp;
	for(int i=0;i<(int)veloc.size();i++)
	{
		if(veloc[i]<0)
			auxsp=0.0;
		else
			auxsp=veloc[i]>maxSpeed?maxSpeed:veloc[i];
		velocInter.push_back(auxsp);
	}
}
}//mr
