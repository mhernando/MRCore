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

#include <string>
#include <fstream>
#include "datasource.h"
#include "datalog.h"
using namespace std;
using namespace mr;


namespace mr
{
DataSource::DataSource()
{
	dataLog=0;
}
bool DataSource::saveDataTo(DataLogOut* log,string name)
{
	dataLog=log;
	return log->registerHW(this,name,false);
}
bool DataSource::loadDataFrom(DataLogIn* log,string name)
{
	dataLog=log;
	return log->registerHW(this,name,true);
}



}; //end namespace mr

