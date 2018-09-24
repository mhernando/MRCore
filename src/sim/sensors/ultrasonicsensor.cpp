#include "ultrasonicsensor.h"


namespace mr
{
	IMPLEMENT_MR_OBJECT(UltrasonicSensor)

		UltrasonicSensor::UltrasonicSensor()
	{
		setDrawReferenceSystem(); //by default the refence system is drawn
		setColor(1, 1, 0);
		sensorActivated = true; //by default, when simulated... the values are computed
		setSensorProperties(M_PI/12, 8, 0.5, 0);
	}


	UltrasonicSensor::~UltrasonicSensor()
	{
	}


	void UltrasonicSensor::writeToStream(Stream& stream)
	{
		SolidEntity::writeToStream(stream);
		//data.writeToStream(stream);
		stream << conusAngle << maxRange << minRange << gaussianError;

	}
	void UltrasonicSensor::readFromStream(Stream& stream)
	{
		SolidEntity::readFromStream(stream);
		//data.readFromStream(stream);
		stream >> conusAngle >> maxRange >> minRange >> gaussianError;
		setSensorProperties(conusAngle, maxRange, minRange, gaussianError);
	}
	void UltrasonicSensor::writeToXML(XMLElement* parent)
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


		SolidEntity::writeToXML(parent);

	}

	void UltrasonicSensor::readFromXML(XMLElement* parent)
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

		SolidEntity::readFromXML(parent);

	}

	char* UltrasonicSensor::CreateXMLText()
	{
		XMLElement* elem = new XMLElement(0, "UltrasonicSensor");
		writeToXML(elem);
		return elem->CreateXMLText();
	}

	void UltrasonicSensor::loadFromXMLText(char *XmlText)
	{
		XML x;
		readFromXML(x.Paste(XmlText));
	}

	ostream& operator<<(ostream& os, const UltrasonicSensor& p)
	{
		//os<<p.x<<" "<<p.y<<" "<<p.z;
		return os;
	}


	void UltrasonicSensor::locationUpdated()
	{
		PositionableEntity::locationUpdated();
		//the laser beam is only updated under request
		beamNeedToBeUpdated = true;
	}

	//set sensor properties
	void UltrasonicSensor::setSensorProperties(double _conusangle, double _maxrange, double _minrange, double _gaussian_error)
	{

		conusAngle = _conusangle;
		maxRange = _maxrange;
		minRange = _minrange;
		gaussianError = _gaussian_error;


		//CONO con el eje de rotación en x una recta cada 5 º de latitud y cada 5ª de altitud son unos 15º de apertura
		vectorBeam.clear();
		absoluteVectorBeam.clear();
		vectorBeam.push_back(Vector3D(1, 0, 0));
		for (double psi = conusAngle; psi >= 6 * M_PI / 180; psi -= 5 * M_PI / 180) {
			double cpsi = cos(psi), spsi = sin(psi);
			for (double alpha = 0; alpha < 2 * M_PI; alpha += M_PI / 8)
				vectorBeam.push_back(Vector3D(cpsi, spsi*cos(alpha), spsi*sin(alpha)));
		}
		absoluteVectorBeam.resize(vectorBeam.size());
		sensorBox = BoundingBox(
			Vector3D(cos(conusAngle)*minRange, -sin(conusAngle)*maxRange, -sin(conusAngle)*maxRange),
			Vector3D(maxRange, sin(conusAngle)*maxRange, sin(conusAngle)*maxRange)
		);
		beamNeedToBeUpdated = true;

		//BoundingBox sensorbox; debe ser recalculado para un nuevo conjunto de haces de laser

	}
	void UltrasonicSensor::updateBeam() {
		if (!beamNeedToBeUpdated)return;
		Transformation3D T = getAbsoluteT3D();
		for (int i = 0; i < vectorBeam.size(); i++)absoluteVectorBeam[i] = (T.orientation)*(vectorBeam[i]);
		beamNeedToBeUpdated = false;
	}
	//ESTA VERSION ES AL MENOS UN 200% MAS RAPIDA QUE LA RAW
	//PARA ULTRASONIDOS HABRIA QUE GENERAR UN BOUNDING BOX DEL CONO


	void UltrasonicSensor::updateSensorData(World *w, float dt)
	{

		if (w == 0)w = getWorld();
		if (w == 0)return;
		if (sensorActivated == false)return;
		if (beamNeedToBeUpdated)updateBeam();
		Transformation3D T = getAbsoluteT3D();
		//plane filter
		vector<SolidEntity *> list;
		absoluteSensorBox = T * sensorBox;
		w->getPrimitivesCollidingBB(absoluteSensorBox, list);
		//Ahora queda recorrer la lista y verificar
		Vector3D pos = T.position;
		int num = (int)list.size();
		measure = maxRange;
		for (int i = 0; i < absoluteVectorBeam.size(); i++) {
			double dist;
			for (int j = 0; j < num; j++)
			{
				if (list[j]->rayIntersection(pos, absoluteVectorBeam[i], dist)) {
					if ((dist < minRange) || (dist > maxRange))continue;
					//de momento tomo el valor mínimo.. pero aquí se podría hacer más con ruido y tal
					if (dist < measure)dist = measure;
				}
			}

		}
	}
	//VERSION BASICA
	/*
	void LaserSensorSim::updateSensorData(World *w,float dt)
	{

	if(w==0)w=getWorld();
	if(w==0)return;
	if(beamNeedToBeUpdated)updateBeam();
	Vector3D pos=getAbsoluteT3D().position;
	double daux;
	for(int i=0;i<numSteps;i++){
	if(w->rayIntersection(pos,absoluteVectorBeam[i],daux)==false)daux=maxRange;
	data.setRange(i,daux);
	}
	}
	*/

	void UltrasonicSensor::drawGL()
	{
		//Draw axis
		PositionableEntity::drawGL();
		//DrawData
		glPushMatrix();
		location.getAbsoluteT3D().transformGL();
		material.loadMaterial();
		//de momento pinto todos los haces como lineas suaves recortadas por el rango
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glColor4f(0, 0, 1, 0.5);
		glBegin(GL_LINES);
		for (int i = 0; i < vectorBeam.size(); i++) {
			Vector3D v = vectorBeam[i];
			glVertex3f(minRange*v.x, minRange*v.y, minRange*v.z);
			glVertex3f(maxRange*v.x, maxRange*v.y, maxRange*v.z);
		}
		glEnd();
		glDisable(GL_BLEND);
		glPopMatrix();
		//test
		Transformation3D T = getAbsoluteT3D();
		absoluteSensorBox = T * sensorBox;
		absoluteSensorBox.drawGL();

	}

}