/**********************************************************************
 *
 * This code is part of the MRcore project
 * Authors:  Rodrigo Azofra Barrio & Miguel Hernando Gutierrez
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

#include "composedentity.h"
#include "tcp.h"
#include <math.h>

namespace mr{
IMPLEMENT_MR_OBJECT(ComposedEntity)

void ComposedEntity::writeToStream(Stream& stream)
{
	SolidEntity::writeToStream(stream);
	EntitySet::writeToStream(stream);

}
void ComposedEntity::readFromStream(Stream& stream)
{
	SolidEntity::readFromStream(stream);
	EntitySet::readFromStream(stream);

}

void ComposedEntity::writeToXML(XMLElement* parent)
{
	int i,num=(int)objects.size();
	int internalRelations=0;
	int tcpRelations=0;

	XMLfile file(parent);


   for(i=0;i<num;i++)
   {
		file<<objects[i];  //save all the objects in XML_FILE
		file<<i; //put one ID in every xml objects
	}
	vector<XMLElement*> pObj=file.getXMLElementsObjects();


	 //compruebo si su parent es otro objeto de la colección
    for(i=0;i<num;i++)
	{
		 if(objects[i]->getLinkedTo())
		 {
			 if(this==objects[i]->getLinkedTo()->getOwner())
			 {
					int a=getIndexOf(objects[i]->getLinkedTo());
					
					string _name=XMLAux::GetValueCadena(pObj[a]->FindVariableZ("name",true));
					XMLVariable* linkedTo;
					bool repeatedOrEmptyName=false;
					//linked one object with another one (i,a)
					if (_name.empty())
						repeatedOrEmptyName=true;

					else
					{
						for (int j=0;j<num;j++)
						{
							if (_name==XMLAux::GetValueCadena(pObj[j]->FindVariableZ("name")) && a!=j)
								repeatedOrEmptyName=true;		
								
						}
					}
										
					if (repeatedOrEmptyName)
						linkedTo= new XMLVariable ("linkTo",XMLAux::setLinkTo("noName",a).c_str());
						 
					else							
						linkedTo= new XMLVariable ("linkTo",XMLAux::setLinkTo(_name,a).c_str());
					

					pObj[i]->AddVariable(linkedTo);
						
			   }
			}
		}

		for(i=0;i<num;i++){
		if(objects[i]->getLinkedTo())
		{
				Tcp *check=dynamic_cast<Tcp *>(objects[i]->getLinkedTo());
				if(check){
					 ComposedEntity *aux=dynamic_cast<ComposedEntity *>(check->getOwner());
					 if(aux){ 
						 if(this==aux->getOwner()){

							int a=getIndexOf(aux);
							int b=aux->getTcpIndex(check);
							

							string _name=XMLAux::GetValueCadena(pObj[a]->FindVariableZ("name",true));
							string _nameTcp=string();
							XMLVariable* linkedToTcp;
							bool repeatedName=false;
							if(_name.empty())
							{
								stringstream str;
								str<<"Composed Entity"<<a;
								_name=str.str();
								pObj[a]->FindVariableZ("name")->SetValue(_name.c_str());
							}
							else 
							{
								for (int j=0;j<num;j++)
								{
									if (_name==XMLAux::GetValueCadena(pObj[j]->FindVariableZ("name")) && a!=j)
										repeatedName=true;		
										
								}
							}
							if (XMLAux::GetNameElement(pObj[a])=="ComposedEntity")
							{
								_nameTcp=XMLAux::GetValueCadena(pObj[a]->FindElementZ("Tcp")->FindVariableZ("name",true));
								if(_nameTcp.empty())
								{
									stringstream str;
									str<<"tcp"<<b;
									_nameTcp=str.str();
									pObj[a]->FindElementZ("Tcp")->FindVariableZ("name")->SetValue(_nameTcp.c_str());
								}
								if (repeatedName)
									linkedToTcp= new XMLVariable ("linkToTcp",XMLAux::setLinkToTcp("noName",_nameTcp,b,a).c_str());
								else
									linkedToTcp= new XMLVariable ("linkToTcp",XMLAux::setLinkToTcp(_name,_nameTcp,b,a).c_str());
							}
							else
							{
								if(repeatedName)
									linkedToTcp= new XMLVariable ("linkToTcp",XMLAux::setLinkToTcpDefault("noName",b,a).c_str());
								else
									linkedToTcp= new XMLVariable ("linkToTcp",XMLAux::setLinkToTcpDefault(_name,b,a).c_str());

							}
							pObj[i]->AddVariable(linkedToTcp);
						 }
					 }
				 }
		}
	}
	SolidEntity::writeToXML(file.getRoot());
}

void ComposedEntity::readFromXML(XMLElement* parent)
{
	SolidEntity::readFromXML(parent);

	int i,num=parent->GetChildrenNum();
	int numElem=0;
	XMLElement** pObj=parent->GetChildren();
	vector<XMLElement*> auxpObj;
	XMLfile file(parent);
	objects.clear();
	for(int i=0;i<num;i++)
	{
		if(dynamic_cast<PositionableEntity *>(file.read(pObj[i])))
		{
			PositionableEntity * aux=dynamic_cast<PositionableEntity *>(file.read(pObj[i]));
			addObject(aux);
			numElem++;
			auxpObj.push_back(pObj[i]);
		}
	}
	for(i=0;i<numElem;i++)
	{
		if(auxpObj[i]->FindVariableZ("linkTo"))
		{
			XMLVariable* link=auxpObj[i]->FindVariableZ("linkTo");
			string type=XMLAux::getTypeConectionLink(link,false);
			if (type=="conectionId")
			{
				int id_linked=link->GetValueInt();
				for (int j=0;j<numElem;j++)
				{
					if (auxpObj[j]->FindVariableZ("id"))
					{
						if (id_linked==auxpObj[j]->FindVariableZ("id")->GetValueInt())
							objects[i]->LinkTo(objects[j]);
					}
				}
			}
			else if(type=="conectionNames")
			{
				string nameLink=XMLAux::GetNameLinkTo(link);
				for (int j=0;j<numElem;j++)
				{
					if (nameLink==XMLAux::GetValueCadena(auxpObj[j]->FindVariableZ("name")))
							objects[i]->LinkTo(objects[j]);
				}

			}
		}
	}

	for(i=0;i<numElem;i++)
	{
		if(auxpObj[i]->FindVariableZ("linkToTcp"))
		{
			XMLVariable* linkTcp=auxpObj[i]->FindVariableZ("linkToTcp");
			string type=XMLAux::getTypeConectionLink(linkTcp);

			if (type=="conectionId")
			{				
				int ids[2]={0};
				XMLAux::GetValueOwnerAndTcp(linkTcp,ids);
				int id_OwnerTcp=ids[0], id_linkTcp=ids[1];

				if(id_OwnerTcp>=0)
				{
					for (int j=0;j<numElem;j++)
					{
						if (auxpObj[j]->FindVariableZ("id"))
						{
							if (id_OwnerTcp==auxpObj[j]->FindVariableZ("id")->GetValueInt())
							{
								ComposedEntity *aux=dynamic_cast<ComposedEntity *>(objects[j]);
								if(aux)
								{
									if (XMLAux::GetNameElement(auxpObj[j])=="ComposedEntity")
									{
										int numChildCompos=auxpObj[j]->GetChildrenNum();
										if (numChildCompos)
										{
											XMLElement** childsComposed=auxpObj[j]->GetChildren();
											int indexTcp=0;
											for (int z=0;z<numChildCompos;z++)
											{
												if (XMLAux::GetNameElement(childsComposed[z])=="Tcp")
												{
													if (id_linkTcp==childsComposed[z]->FindVariableZ("id",true)->GetValueInt())
														objects[i]->LinkTo(aux->getTcp(indexTcp));
													else if (indexTcp==id_linkTcp)
														objects[i]->LinkTo(aux->getTcp(indexTcp));
															
													indexTcp++;

												}
											}
										}
								}
								else 
									objects[i]->LinkTo(aux->getTcp(id_linkTcp));//it's supposed that user knows id_linkTcp=Tcp's index

							}
						}
					}
				}
			}	
			else if (type=="conectionNames")
			{
				vector<string> names=XMLAux::GetNameOwnerAndTcp(linkTcp);
				int idTcp=XMLAux::getIndTcp(names[1]);
				string nameOnlyTcp=XMLAux::getOnlyNameTcp(names[1]);
				for (int j=0;j<numElem;j++)
				{
					if (names[0]==XMLAux::GetValueCadena(auxpObj[j]->FindVariableZ("name")))
					{
						ComposedEntity *aux=dynamic_cast<ComposedEntity *>(objects[j]);
						if(aux)
						{
							if (XMLAux::GetNameElement(auxpObj[j])=="ComposedEntity")
							{
								int numChildCompos=auxpObj[j]->GetChildrenNum();
								if(numChildCompos)
								{
									XMLElement** childsComposed=auxpObj[j]->GetChildren();
									int indexTcp=0;
									for (int z=0;z<numChildCompos;z++)
									{
										if (XMLAux::GetNameElement(childsComposed[z])=="Tcp")
										{
											if (nameOnlyTcp==XMLAux::GetValueCadena(childsComposed[z]->FindVariableZ("name")) &&
												idTcp==childsComposed[z]->FindVariableZ("id")->GetValueInt())
													objects[i]->LinkTo(aux->getTcp(indexTcp));
												else if (nameOnlyTcp==XMLAux::GetValueCadena(childsComposed[z]->FindVariableZ("name")) &&
														idTcp==indexTcp)
													objects[i]->LinkTo(aux->getTcp(indexTcp));											
												else if(idTcp==childsComposed[z]->FindVariableZ("id")->GetValueInt())
													objects[i]->LinkTo(aux->getTcp(indexTcp));	
												else if (idTcp==indexTcp)
													objects[i]->LinkTo(aux->getTcp(indexTcp));

												indexTcp++;
										}
									}
								}
							}
								else 
									objects[i]->LinkTo(aux->getTcp(idTcp));

							}
						}
					}

				}
			}
		}
	}
}

char* ComposedEntity::CreateXMLText()
{
	XMLElement* elem=new XMLElement(0,"ComposedEntity");
	writeToXML(elem);
	return elem->CreateXMLText();
}

void ComposedEntity::loadFromXMLText(char *XmlText)
{
	XML x;
	readFromXML(x.Paste(XmlText));
}

ComposedEntity::ComposedEntity(void)
{
//prueba
//drawBox=true;

}

ComposedEntity::~ComposedEntity(void)
{
}

bool ComposedEntity::linkToBase(PositionableEntity *p)
{
	if(p==0)return false;
	//if linked... do nothing
	if(p->location.getBase())return false;
	//else directly link
	p->location.linkTo(getReferenciableLocation());
	computeBoundingBox();
return true;
}
void ComposedEntity::drawGL()
{
	EntitySet::drawGL();
	SolidEntity::drawGL();
}
BoundingBox  ComposedEntity::getAbsoluteBoundingBox()
{
	//check if some of the contained solid objects have to update their bounding box
	if(boxNeedToBeUpdated)computeBoundingBox();
	return SolidEntity::getAbsoluteBoundingBox();

}
void ComposedEntity::computeBoundingBox()
{
	BoundingBox nbox;
	for(int i=0;i<(int)(objects.size());i++)
	{
		SolidEntity *aux=dynamic_cast<SolidEntity *>(objects[i]);
		if(aux)nbox.includeBox(aux->getAbsoluteBoundingBox());
	}
	box=nbox/getAbsoluteT3D();
	absoluteBox=nbox;
	boxNeedToBeUpdated=false;
}
int ComposedEntity::getTcpIndex(Tcp *tcp)
{
	int index=0;
	for(int i=0;i<(int)(objects.size());i++)
	{
		Tcp *aux=dynamic_cast<Tcp *>(objects[i]);
		if(aux==tcp)return index;
		if(aux)index++;
	}
	return -1;
}
Tcp *ComposedEntity::getTcp(int num)
{
	int index=0;
	for(int i=0;i<(int)(objects.size());i++)
	{
		Tcp *aux=dynamic_cast<Tcp *>(objects[i]);
		if(aux){if(index==num)return aux; else index++;}
	}
	return 0;
}
//Collision detection
bool ComposedEntity::checkCollisionWith(SolidEntity &solid)
{
	//si no es intersectable return false
	if(this->isIntersectable()==false)return false;

	//chequeo los bounding boxes globales y si no voy elemento a elemento
	if(checkBoundingBoxes(solid)==false)return false;
	for(int i=0;i<(int)(objects.size());i++)
	{	
		SolidEntity *aux=dynamic_cast<SolidEntity *>(objects[i]);
		//en cuanto hay colision devuelvo true
		if(aux){
			if(aux->isIntersectable()) //solo compruebo los intersectables
			if(aux->checkCollisionWith(solid))return true;
		}
	}
	return false;
}
void ComposedEntity::simulate(double delta_t)
{
for(int i=0;i<(int)(objects.size());i++)
	objects[i]->simulate(delta_t);
}
}//mr