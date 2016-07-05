/**********************************************************************
 *
 * This code is part of the MRcore project
 * Author:  Cristina Gajate & Miguel Hernando Gutierrez
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

#ifndef __EUITI_BOT_SIM_H_
#define __EUITI_BOT_SIM_H_


#include "robotsim.h"


#define	INIT_EUITI_BOT			0x80	//	0x1000.0000
#define	SHOULDERRIGHT	0x01	//	0x0000.0001
#define ELBOWDOWN		0x02	//	0x0000.0010

namespace mr
{

class EUITIbotSim : public RobotSim
{
	DECLARE_MR_OBJECT(EUITIbotSim)

public:
	//Serializers
	virtual void writeToStream(Stream& stream);
	virtual void readFromStream(Stream& stream);
	virtual void writeToXML(XMLElement* parent);
	virtual void readFromXML(XMLElement* parent);
	virtual void loadFromXMLText(char* XmlText);
	virtual char* CreateXMLText();
	//Constructor
	EUITIbotSim(void);

//Inverse  kinematics rel and abs
	virtual bool inverseKinematics(Transformation3D t06, vector<double> &_q, unsigned char _conf=NULL);

//Return robot configuration
	virtual bool getConfigurationOf(const vector<double> &_q, unsigned char &_conf);
	bool configuration(unsigned char _conf, double& _s, double& _e);
	bool configuration(double _s, double _e,unsigned char &_conf);

//simulate specific
	void simulate(double delta_t);

//Aux. methods
	void setFlash();
	bool getCoordinatesOf(vector<double> &_q);// delete?
	
protected:
	bool EUITIbotInverseKinematics(Transformation3D t06, vector<double> &_q, unsigned char _conf=0x00);

};

};//end namespace mr

#endif