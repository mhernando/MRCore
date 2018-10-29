/**********************************************************************
 *
 * This code is part of the MRcore projec
 * Author:  the MRCore group
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

#include "../PositionableEntity.h"
#include <vector>
using namespace std;
namespace mr
{
class WorldInfo: public PositionableEntity
{
	DECLARE_MR_OBJECT(WorldInfo)

public:
	enum INFO_TYPE {REG_NUMBER, NAME};
	struct Winfo {
		union {
			unsigned int reg_number;
			char name[200];
		};
		INFO_TYPE type;
	};
	WorldInfo();
	//Serializers
	virtual void writeToStream(Stream& stream);
	virtual void readFromStream(Stream& stream);
	virtual void writeToXML(XMLElement* parent);
	virtual void readFromXML(XMLElement* parent);
	virtual char* CreateXMLText(){return 0;};
	virtual void loadFromXMLText(char* XmlText){};

	virtual void simulate(double delta_t);
	virtual void drawGL();

protected:
	vector<Winfo> data;
	bool affectWorld; //flag for modifying the world only one time
	void updateWorld(); //functiona executed only one time, but with the world fully created
};

}; //end namespace mr

