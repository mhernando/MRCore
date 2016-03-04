/**********************************************************************
 *
 * This code is part of the MRcore projec
 * Author:  Francisco Ramirez de Anton Montoro
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
#ifndef __MRCORE__XMLFILE__H
#define __MRCORE__XMLFILE__H

#include "xmlaux.h"
#include <vector>

using namespace std;

//using namespace std;



namespace mr
{
class Object;
class XMLElement;
/*!
    \class XMLfile
    \brief open, read, and writes a XMLfile for one object. 
	is useful for worlds and world objects. 
*/
class XMLfile 
{
public:
	//constructor and destructor
		//this class cannot be directly instantiated
	XMLfile(const char *file="xml_file.xml");
	XMLfile (XMLElement *parent);
	~XMLfile();

	Object * load(const char *file="xml_file.xml"); //loads and parses the file
	bool save(Object* pObj,const char *file="xml_file.xml"); //saves the XML file created with the object
	bool save();

	void write (Object* pObj);
	Object* read (XMLElement* obj);
	bool importFromXML ();
	
	XMLfile& operator<<(Object *obj);
	XMLfile& operator<<(int _id);

	
	vector<XMLElement*> getXMLElementsObjects () {return pElem;}
	XMLElement* getRoot(){return root;}

private:
	bool checkOverwrite;
	vector<XMLElement*> pElem;
	XML* xml;
	XMLElement* root;
	XMLElement* imported;


}; //End of class XMLFile

};
#endif  //__MRCORE__XMLfile_H
