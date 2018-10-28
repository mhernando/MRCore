/**********************************************************************
 *
 * This code is part of the MRcore projec
 * Author:  -----------anyone
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



#include "../math/transformation2d.h"
#include "../math/transformation3d.h"


using namespace std;

namespace mr{

	/** The WBodometry is a simple utility class that emulates a real odometry:
	The s **/
class WheeledBaseSim;
class WBodometry
{
	Transformation2D data;
	Transformation3D lastpose;//last pose
	Transformation3D basepose; //odometry origin (x,y,yaw are measured on this base)

public:
	WBodometry();
	virtual ~WBodometry (){}
	void reset(WheeledBaseSim *wb=0);
	void set(double x, double y, double theta);
	void set(Transformation2D _data); 
	void update(WheeledBaseSim *wb);
	void setNoiseProperties(double v_noise, double w_noise);
	Transformation2D get() { return data; }
	//Serializers
protected:
	double v_noise; //gaussian noise advance
	double w_noise; //gaussian noise turn
};
 


}




