#pragma once

/**********************************************************************
 *
 * This code is part of the MRcore project
 * Author:  Miguel Hernando Gutierrez
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

#ifndef __MRCORE__TIMESIM__H
#define __MRCORE__TIMESIM__H
#include <iostream>
using namespace std;

namespace mr
{
/*!
    \class timesim
    Simple inline class whose role is to simulate virtual time with good accuracy on millisecons.
	It has been included due to the need to handle the different update frecuencies of sensors.
*/
class TimeSim
{
public:
	int seconds;
	int microseconds;
	friend ostream& operator<<(ostream& os, const TimeSim& t);
	void normalize(){
		//no negative times
		if(microseconds>999999){
			seconds+=microseconds/1000000;
			microseconds%=1000000;
		}
		if(microseconds<0){
			int mcs=microseconds%1000000;
			seconds-=microseconds/1000000;
			if(mcs<0){seconds--; microseconds=1000000+mcs;}
		}
		if(seconds<0)microseconds=seconds=0;
	}
	operator float(){
		return seconds+microseconds*0.000001F;
	}
	operator double(){
		return seconds+microseconds*0.000001;
	}
	TimeSim(int s=0,int mcs=0):seconds(s),microseconds(mcs){normalize();}
	TimeSim(double s){
		seconds=(int)s;
		microseconds=(int)(1000000*(s-seconds));
		normalize();
	}
	void set(int s, int mcs){
		seconds=s;
		microseconds=mcs;
		normalize();
	}
	void operator+=(double sec){
		int s=(int)sec;
		int ms=(int)(1000000*(sec-s*1.0));
		seconds+=s;
		microseconds+=ms;
		normalize();
	}
	TimeSim operator-(const TimeSim & t){
		return TimeSim(seconds-t.seconds, microseconds-t.microseconds);
	}
	TimeSim operator+(const TimeSim & t){
		return TimeSim(seconds+t.seconds, microseconds+t.microseconds);
	}
	bool operator>(const TimeSim& t) const{
		return ((seconds>t.seconds) || (seconds==t.seconds && microseconds>t.microseconds));
	}
	bool operator==(const TimeSim& t) const	{
		return (seconds==t.seconds && microseconds == t.microseconds);
	}
	bool operator<(const TimeSim& t) const{
		return ((seconds<t.seconds) || (seconds==t.seconds && microseconds<t.microseconds));
	}
	void reset(){
		seconds=microseconds=0;
	}

};

inline ostream& operator<<(ostream& os, const TimeSim& t)
{
	os<<t.seconds/60<<":"<<t.seconds%60<<":"<<t.microseconds/1000;
	return os;
}

} //mr
#endif //__MRCORE__TIMESIM__H

	

