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
	\class UltrasonicSensor
	
	*/

	
	class UltrasonicSensor : public ComposedEntity
	{
		DECLARE_MR_OBJECT(UltrasonicSensor)
	public:




		/**Text output**/
		friend ostream& operator<<(ostream& os, const UltrasonicSensor& p);
	protected:

		//beam of segments relative to the sensor base
		vector<Vector3D> vectorBeam;
		//beam of segments in absolute coordinates
		vector<Vector3D> absoluteVectorBeam;
		//bool flag that indicates that the absolute segments have to be updated if needed
		bool beamNeedToBeUpdated;

		//box referred to the intrinsic ref system that bounds the sensor volume
		BoundingBox sensorBox;
		BoundingBox absoluteSensorBox;
		

		//virtual methods executed each time the sensor is moved
		void locationUpdated();
		//ultrasonic sensor properties
		double conusAngle; //minimum -PI/2
		double maxRange; //max range in m : 10
		double minRange;  //min range =0
		double gaussianError; // %error gaussian distribution
		void updateBeam();
		Mutex m;
		bool sensorActivated; //true if when simulated... is updated
		double measure;
	public:
		//attributes

		//constructors destructors
		UltrasonicSensor();
		virtual ~UltrasonicSensor();

		//methods
		//serialization
		virtual void writeToStream(Stream& stream);
		virtual void readFromStream(Stream& stream);
		virtual void writeToXML(XMLElement* parent);
		virtual void readFromXML(XMLElement* parent);
		virtual char* CreateXMLText();
		virtual void loadFromXMLText(char* XmlText);
		//set sensor properties
		void setSensorProperties(double _conusangle, double _maxrange, double _minrange, double _gaussian_error);
		void updateSensorData(World *w = 0, float dt = 0);
		void drawGL();
		void setDrawGLMode(int m);
		void setActive(bool val = true) { sensorActivated = val; }
		void simulate(double t)	{updateSensorData();}
		//laserSensor Methods
		virtual double getDistance() {return measure;}

	};

}