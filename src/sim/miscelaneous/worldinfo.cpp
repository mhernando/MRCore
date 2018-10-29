#include "WorldInfo.h"
#include "../world.h"
#include "../sim/rovers/wheeledbasesim.h"
#include "../sim/sensors/lasersensorsim.h"

namespace mr
{
IMPLEMENT_MR_OBJECT(WorldInfo)

WorldInfo::WorldInfo()
{
	affectWorld = true;

}
void WorldInfo::writeToStream(Stream& stream)
{
	PositionableEntity::writeToStream(stream);
	//specific data
}
void WorldInfo::readFromStream(Stream& stream)
{
	PositionableEntity::readFromStream(stream);
	//specific data
}
void WorldInfo::writeToXML(XMLElement* parent)
{
//	XMLVariable* _mark_id = new XMLVariable("mark_id", XMLAux::string_Convert<int>(mark_id).c_str());
//	parent->AddVariable(_mark_id);
	PositionableEntity::writeToXML(parent);
}

void WorldInfo::readFromXML(XMLElement* parent)
{
	if (parent->FindVariableZ("name"))
	{
		Winfo winfo;
		(parent->FindVariableZ("name"))->GetValue(winfo.name);
		winfo.type = INFO_TYPE::NAME;
		data.push_back(winfo);
	}
	int num = parent->GetChildrenNum();
	if (num){
		XMLElement** infos = parent->GetChildren();
		for (int i = 0; i<num; i++)
		{
			char buff[100];
			infos[i]->GetElementName(buff);
			if( strcmp(buff, "RegisterNumber") == 0) {
				if (infos[i]->FindVariableZ("number")) {
					int nmat = infos[i]->FindVariableZ("number")->GetValueInt();
					Winfo winfo;
					winfo.type = INFO_TYPE::REG_NUMBER;
					winfo.reg_number = nmat;
					data.push_back(winfo);
				}		
			}

		}
	}
}
void WorldInfo::drawGL()
{
	//it si possible to make the drawing independent of the geometric model
	//so we overwrite the composedentity drawing function with this one
	//Draw representation
	if (affectWorld)updateWorld();
	PositionableEntity::drawGL();
	//DrawData

	glDisable(GL_LIGHTING);

	glMatrixMode(GL_TEXTURE);
	glPushMatrix();
	glLoadIdentity();

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	GLint m_viewport[4];
	glGetIntegerv(GL_VIEWPORT, m_viewport);
	gluOrtho2D(0, m_viewport[2], 0, m_viewport[3]);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glColor3f(1, 1, 0);
	int num = (int)data.size();
	glTranslated(5, 20 * (num-0.5), 0);
	int rnum = 0;
	for (int i = 0; i < num; i++) {
		char line[255];
		switch (data[i].type) {
			case INFO_TYPE::REG_NUMBER:
				sprintf(line, "R.Number[%d]: %d", ++rnum, data[i].reg_number);
				GLTools::Print(line, 15);
			break;
			case INFO_TYPE::NAME:
				GLTools::Print(data[i].name, 15);
			break;
		}
		glTranslated(0, -20, 0);
	}


	glMatrixMode(GL_TEXTURE);
	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();



	return;
}

void WorldInfo::simulate(double delta_t)
{
	if (affectWorld)updateWorld();
//	cout<<"simulating: "<<speed<<" "<<rotSpeed<<" "<<delta_t<<endl;


}
//TIPICAL VALUES
//-	Varianza longitudinal del láser sick detectando balizas reflectantes: 1.355x10-4 m^2
//Varianza angular del láser sick detectando balizas reflectantes : 3.08x10 - 4 rad ^ 2
//Varianza longitudinal del láser sick detectando objetos : 9.0x10 - 6 m ^ 2
//Varianza en avance longitudinal de la odometría : 1.347x10 - 3 m ^ 2
//Varianza en avance angular de la odometría : 3.68x10 - 4 rad ^ 2
#define PRIME_FACTOR(X, P) (1.5-((X)%(P))/(1.0*(P))) 
void WorldInfo::updateWorld()
{
	
	affectWorld = false;
	//code to be executed only one time

	//compute a number between 0-1000 with the register numbers of the students
	long num = 1;
	
	for (int i=0;i<data.size();i++)
		if (data[i].type == INFO_TYPE::REG_NUMBER) {
			num += 6983*(data[i].reg_number);
		}
	//PRIME FACTOR IS A VALUE FROM 0.5 to 1.5
	double odom_v = 0.002*PRIME_FACTOR(num,31); //incremental = percentual 
	double odom_w = 0.0004*PRIME_FACTOR(num, 37) ; //incremental
	double laser_d = 0.00001*PRIME_FACTOR(num, 41); //absolute
	double laser_lm_d = 0.0002*PRIME_FACTOR(num, 47) ; //absolute
	double laser_lm_ang = 0.0003*PRIME_FACTOR(num, 43) ; //absolute

	vector<WheeledBaseSim *> listwb;
	getWorld()->getObjectsOftype(listwb);
	for (int i = 0; i < listwb.size(); i++) 
		(listwb[i]->getWBodometry()).setNoiseProperties(odom_v, odom_w);
	vector<LaserSensorSim *> listls;
	for (int i = 0; i < listls.size(); i++)
		listls[i]->setLaserVariances(laser_d, laser_lm_d, laser_lm_ang);

}


}; //Namespace mr
