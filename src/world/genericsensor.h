/**********************************************************************
 *
 * This code is part of the MRcore project
 * Author:  Rodrigo Azofra Barrio &Miguel Hernando Gutierrez
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

#pragma once
#include "composedentity.h"
#include "timesim.h"
#include "../system/mutex.h"
using namespace std;

namespace mr
{
/*!
    \class GenericSensor
    GenericSensor	->	An abstract class for sensors (second generation). It is responsible of the correct update
	of the sensor, and should include frecuency info. Although many sensors do not include a solid representation
	due to the ComposedEntity heritage this is directly allowed.
	The Sensor has as "linkedTo" the component required for its sensing. i.e.: a joint encoder should be linked to the
	sensed joint. A set of movile platform encoders should be linked to the wheeledBase...and so on. 
*/
class GenericSensor :public ComposedEntity
{
	TimeSim lastUpdate;
	double period;
protected: 
	Mutex m;
	//Construction/destruction
	GenericSensor(void);
	virtual ~GenericSensor(void);
	//methods
	void setFrecuency(double hertzs=0);
	virtual void updateSensorData()=0; //it is executed only if frec condition is satisficed
public:
	void simulate(double t);
};

}//mr