#include "LandMark.h"


namespace mr
{
	IMPLEMENT_MR_OBJECT(LandMark)

	LandMark::LandMark(int mark_id)
	{
		this->mark_id = mark_id;
		lm_size = 0.4;
	}

	LandMark::~LandMark()
	{
	}

	void LandMark::writeToStream(Stream& stream)
	{
		PositionableEntity::writeToStream(stream);
		//data.writeToStream(stream);
		stream << mark_id;

	}
	void LandMark::readFromStream(Stream& stream)
	{
		PositionableEntity::readFromStream(stream);
		//data.readFromStream(stream);
		stream >> mark_id;
	}
	void LandMark::writeToXML(XMLElement* parent)
	{
		XMLVariable* _mark_id = new XMLVariable("mark_id", XMLAux::string_Convert<int>(mark_id).c_str());
		parent->AddVariable(_mark_id);
		PositionableEntity::writeToXML(parent);

	}

	void LandMark::readFromXML(XMLElement* parent)
	{
		if (parent->FindVariableZ("mark_id"))
			mark_id = parent->FindVariableZ("mark_id")->GetValueInt();
		PositionableEntity::readFromXML(parent);
	}

	char* LandMark::CreateXMLText()
	{
		XMLElement* elem = new XMLElement(0, "LandMark");
		writeToXML(elem);
		return elem->CreateXMLText();
	}

	void LandMark::loadFromXMLText(char *XmlText)
	{
		XML x;
		readFromXML(x.Paste(XmlText));
	}

	ostream& operator<<(ostream& os, const LandMark& p)
	{
		//os<<p.x<<" "<<p.y<<" "<<p.z;
		return os;
	}


	//set sensor properties
	void LandMark::setMarkId(int mark)
	{
		mark_id = mark;
	}

	void LandMark::drawGL()
	{
		//Draw representation
		PositionableEntity::drawGL();
		//DrawData
		char ids[20];
		sprintf(ids, "LM:%d", mark_id);
		glPushMatrix();
		location.getAbsoluteT3D().transformGL();
		glColor3f(0, 1, 0);
		GLTools::Print(ids, 0.2);
		glPopMatrix();


	}

}