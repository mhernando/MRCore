#include "LandMark.h"


namespace mr
{
	IMPLEMENT_MR_OBJECT(LandMark)

	LandMark::LandMark(int mark_id)
	{
		this->mark_id = mark_id;
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
		glPushMatrix();
		location.getAbsoluteT3D().transformGL();
		material.loadMaterial();
		//de momento pinto todos los haces como lineas suaves recortadas por el rango
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		double r, g, b;
		material.getColor(r,g,b);
		float intensity = 1 - 0.99*(measure - minRange) / (maxRange - minRange);
		glColor4f(r, g, b, intensity);
		glLineWidth(intensity);
		glBegin(GL_LINES);
		for (int i = 0; i < vectorBeam.size(); i++) {
			Vector3D v = vectorBeam[i];
			glVertex3f(minRange*v.x, minRange*v.y, minRange*v.z);
			glVertex3f(measure*v.x, measure*v.y, measure*v.z);
		}
		glEnd();
		glDisable(GL_BLEND);
		glPopMatrix();


	}

}