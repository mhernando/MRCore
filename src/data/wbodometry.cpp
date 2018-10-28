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

#include "wbodometry.h"
#include "math/mrmath.h"
#include "math/gaussian.h"
#include "../sim/rovers/wheeledbasesim.h"




namespace mr{

//	Transformation2D data;
//	Transformation3D lastpose;//last pose




WBodometry::WBodometry()
{
	v_noise = 0;
	w_noise = 0;
}
/**/
void WBodometry::set(Transformation2D _data)
{
	set(_data.x, _data.y, _data.angle());
}
void WBodometry::set(double x, double y, double theta)
{
	data.theta=theta;
	data.x =x;
	data.y =y;
}

void WBodometry::update(WheeledBaseSim *wb)
{
	Transformation3D pose = wb->getAbsoluteT3D();
	Transformation3D inc = lastpose.inverted()*pose;
	double r, p, incy;
	inc.orientation.getRPY(r, p, incy);
	incy *= sampleGaussian(1, w_noise);
	Vector3D incpos = lastpose.orientation*(inc.position*(sampleGaussian(1, v_noise)));
	//obtengo ahora este incremento en base a la base de la odometría
	Vector3D inc_odom = basepose.orientation.transposed()*incpos;
	data.theta += incy;
	data.x += inc_odom.x;
	data.y += inc_odom.y;
	lastpose = pose;
}
void WBodometry::setNoiseProperties(double v_noise, double w_noise)
{
	this->v_noise = v_noise;
	this->w_noise = w_noise;
}

void WBodometry::reset(WheeledBaseSim *wb )
{
	Transformation3D pose = wb->getAbsoluteT3D();
	lastpose = basepose = pose;
	set(0, 0, 0);
}





}//end namespace
