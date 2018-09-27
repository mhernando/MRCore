#pragma once
#include "../../hw/lasersensor.h"
#include "../composedentity.h"
#include "../world.h"
#include "../../system/mutex.h"
#include "../../base/xmlaux.h"
#include <vector>


using namespace std;

namespace mr
{
	/*!
	\class LandMark
	
	*/

	
	class LandMark : public PositionableEntity
	{
		DECLARE_MR_OBJECT(LandMark)
	public:
		/**Text output**/
		friend ostream& operator<<(ostream& os, const LandMark& p);
	protected:
		int mark_id;
	public:
		//attributes

		//constructors destructors
		LandMark(int mark_id=0);
		virtual ~LandMark();
		//methods
		//serialization
		virtual void writeToStream(Stream& stream);
		virtual void readFromStream(Stream& stream);
		virtual void writeToXML(XMLElement* parent);
		virtual void readFromXML(XMLElement* parent);
		virtual char* CreateXMLText();
		virtual void loadFromXMLText(char* XmlText);
		//set sensor properties
		void setMarkId(int mark);
		void drawGL();
	};

}