/**********************************************************************
 *
 * This code is part of the MRcore project
 * Author:  Miguel Hernando Gutierrez
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

#include "genericsensor.h"
#include "world.h"
namespace mr{

GenericSensor::GenericSensor(void)
{
	period=0;
}
void GenericSensor::setFrecuency(double hertzs)
{
	if(hertzs==0)period=0;
	else{
		period=1/hertzs;
	}
}

GenericSensor::~GenericSensor(void)
{
}

void GenericSensor::simulate(double t)
{
	//check if last update was done a period time previous to period
	//if no world is attached, always update
	ComposedEntity::simulate(t);
	World *w=getWorld();
	if(w){
		TimeSim ct=w->getCurrentTime();
		if((double)(ct-lastUpdate)<period)return;
		
	}
	m.Lock();
	updateSensorData();
	m.Unlock();
	if(w)lastUpdate=w->getCurrentTime();
}
//retrieving the specific sensor data should do something like
//	m.Lock();
//	d=data;
//	m.Unlock();
}