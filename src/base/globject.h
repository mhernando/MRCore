/**********************************************************************
 *
 * This code is part of the MRcore projec
 * Author:  -----------anyone
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
#ifndef __MRCORE__GL_OBJECT__H
#define __MRCORE__GL_OBJECT__H

#include "object.h"

namespace mr
{
/*!
    \class GLObject
    \brief GLObject that supports GL drawing.
*/
class GLObject :public Object
{
public:
	GLObject(){r=g=b=255;}//White color by default
	virtual void drawGL()=0;

	inline void setColor(unsigned char rp,unsigned char gp,unsigned char bp){r=rp;g=gp;b=bp;}
	virtual void writeToStream(Stream& stream){stream<<r<<g<<b;}
	virtual void readFromStream(Stream& stream){stream>>r>>g>>b;}

protected:
	unsigned char r,g,b;
};

};
#endif  //__MRCORE__OBJECT_H
