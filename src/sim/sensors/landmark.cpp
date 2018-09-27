#include "LandMark.h"


namespace mr
{
	IMPLEMENT_MR_OBJECT(LandMark)

	LandMark::LandMark()
	{
		setDrawReferenceSystem(); //by default the refence system is drawn
		setColor(1, 1, 0);
		sensorActivated = true; //by default, when simulated... the values are computed
		measure = 3;
		setSensorProperties(DEG2RAD*10, 3, 0.05, 0);
		beamNeedToBeUpdated = true;
		//el cristal se pinta pero no se usa en la detección
		PrismaticPart *piezo = new PrismaticPart();
		piezo->setColor(0.9, 0.9, 0.9);
		piezo->setRegularPolygonBase(0.025, 6);
		piezo->setHeight(0.005);
		piezo->setRelativeOrientation(0,M_PI/2,0);
		//sensor non detectable by raytracing
		piezo->setIntersectable(false);
		(*this) += piezo;
	}


	LandMark::~LandMark()
	{
	}


	void LandMark::writeToStream(Stream& stream)
	{
		PositionableEntity::writeToStream(stream);
		//data.writeToStream(stream);
		stream << conusAngle << maxRange << minRange << gaussianError;

	}
	void LandMark::readFromStream(Stream& stream)
	{
		PositionableEntity::readFromStream(stream);
		//data.readFromStream(stream);
		stream >> conusAngle >> maxRange >> minRange >> gaussianError;
		setSensorProperties(conusAngle, maxRange, minRange, gaussianError);
	}
	void LandMark::writeToXML(XMLElement* parent)
	{

		//	XMLAux aux;
		//	XMLElement* lasersensor=new XMLElement(parent,"laserSensorSim");
		XMLVariable* _conusAngle = new XMLVariable("conusAngle", XMLAux::string_Convert<double>(conusAngle).c_str());
		XMLVariable* _maxRange = new XMLVariable("maxRange", XMLAux::string_Convert<double>(maxRange).c_str());
		XMLVariable* _minRange = new XMLVariable("minRange", XMLAux::string_Convert<double>(minRange).c_str());
		XMLVariable* _error = new XMLVariable("error", XMLAux::string_Convert<double>(gaussianError).c_str());


		parent->AddVariable(_conusAngle);
		parent->AddVariable(_maxRange);
		parent->AddVariable(_minRange);
		parent->AddVariable(_error);


		PositionableEntity::writeToXML(parent);

	}

	void LandMark::readFromXML(XMLElement* parent)
	{
		double _conusAngle = conusAngle, _maxRange = maxRange, _minRange = minRange, _error = gaussianError;

		if (parent->FindVariableZ("conusAngle"))
		{
			_conusAngle = XMLAux::GetValueDouble(parent->FindVariableZ("conusAngle"));
		}
		if (parent->FindVariableZ("maxRange"))
		{
			_maxRange = XMLAux::GetValueDouble(parent->FindVariableZ("maxRange"));
		}

		if (parent->FindVariableZ("minRange"))
		{
			_minRange = XMLAux::GetValueDouble(parent->FindVariableZ("minRange"));
		}
		if (parent->FindVariableZ("error"))
		{
			_error = XMLAux::GetValueDouble(parent->FindVariableZ("error"));
		}

		setSensorProperties(_conusAngle, _maxRange, _minRange, _error);

		PositionableEntity::readFromXML(parent);

	}

	char* LandMark::CreateXMLText()
	{
		XMLElement* elem = new XMLElement(0, "UltrasonicSensor");
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
	void LandMark::setSensorProperties(double _conusangle, double _maxrange, double _minrange, double _gaussian_error)
	{

		conusAngle = _conusangle;
		maxRange = _maxrange;
		minRange = _minrange;
		gaussianError = _gaussian_error;

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