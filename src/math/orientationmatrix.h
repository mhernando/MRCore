/**********************************************************************
 *
 * This code is part of the MRcore project
 * Authors:  Miguel Hernando, Diego Rodríguez-Losada, Paloma de la Puente,
 *			 Alberto Valero, Luis Pedraza
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
#ifndef __MRCORE__ORIENTATIONMATRIX_H
#define __MRCORE__ORIENTATIONMATRIX_H

#include <iostream>
#include "mrmath.h"
#include "matrix3x3.h"

namespace mr
{
/*!
    \class OrientationMatrix
    \brief OrientationMatrix - a basic mathematical class for 3x3 matrix.
	It is any 3x3 matrix not only an orientation matrix, therefore, 
	operations like inversion are made through a general formula. Another
	consequence is that the matrix elements are public.
*/
enum Axis {X_AXIS, Y_AXIS, Z_AXIS};
class Quaternion;

class OrientationMatrix:public Matrix3x3{

public:
//constructors
	inline OrientationMatrix(const Matrix3x3 &m):Matrix3x3(m){}
	inline OrientationMatrix():Matrix3x3(1){}
	OrientationMatrix(double roll, double pitch, double yaw);
	OrientationMatrix(Axis axis,double ang);
	OrientationMatrix(Vector3D vu, Vector3D vv);
//operators
	inline const double * operator [](int i){return mat[i];}
//only a rotation matrix could =,*,*= with another rotation matrix 
	inline OrientationMatrix & operator= (const OrientationMatrix &m){
		matrix=m.matrix;
		return *this;
	}
	inline OrientationMatrix operator * (const OrientationMatrix &m) const{
		OrientationMatrix ret(*this);
		ret.Matrix3x3::operator*=(m);
		return ret;
	}
	inline OrientationMatrix & operator *= (const OrientationMatrix &m){
		Matrix3x3::operator *=(m);
		return *this;
	}
	inline Vector3D operator *(const Vector3D &v){return Matrix3x3::operator *(v);}

//methods
	//Roll pitch yaw convention (eq to rotation trx, ry, rz fixed axis)
	OrientationMatrix & setRPY(double roll, double pitch, double yaw);
	void getRPY(double& roll, double& pitch, double& yaw);
	void getAxisAngle (double& _theta, Vector3D& _axis);
	void getQuaternion (Quaternion& _q);

	inline double getDeterminant(){return 1;} 
	inline OrientationMatrix inverted()const{
		return Matrix3x3::transposed();
	}
	inline OrientationMatrix transposed() const{
		return Matrix3x3::transposed();
	}	
	inline Vector3D getVectorU(){
		return Vector3D(mat[0][0],mat[1][0],mat[2][0]);
	}
	inline Vector3D getVectorV(){
		return Vector3D(mat[0][1],mat[1][1],mat[2][1]);
	}
	inline Vector3D getVectorW(){
		return Vector3D(mat[0][2],mat[1][2],mat[2][2]);
	}

	

//friend operators


	
//friend methods

};



//using quaternions to represent rotations
class Quaternion
{
public:
	//attributes
	double angle;//theta angle
	double scal;//const part = cos(p/2)
	Vector3D axis;//axis vector
	Vector3D vec;//vectorial part = v*sin(p/2)
	double q0,q1,q2,q3;//other represenattion q = [q0,q1,q2,q3]

	//methods
	Quaternion(){};
	Quaternion(double _scalar, Vector3D _vector):scal(_scalar), vec(_vector){
		
		angle = acos(_scalar)*2.00;
		axis = _vector/sin(angle/2.00);
	
		q0 = _scalar;
		q1 = _vector.x;
		q2 = _vector.y;
		q3 = _vector.z;
	}

	Quaternion(double _q0, double _q1, double _q2, double _q3){
		
		q0 = _q0;
		q1 = _q1;
		q2 = _q2;
		q3 = _q3;

		scal = q0;
		vec = Vector3D(q1,q2,q3);

		angle = acos(scal)*2.00;
		axis = vec/sin(angle/2.00);
	}

	virtual ~Quaternion(){};

	void setAngleAxis(double _angle, Vector3D _axis){

		angle = _angle;
		axis = _axis;

		scal = cos(_angle/2.00);
		vec = _axis*sin(angle/2.00);
	
		q0 = scal;
		q1 = vec.x;
		q2 = vec.y;
		q3 = vec.z;
	}

	Quaternion & operator= (const Quaternion &q){
		setAngleAxis(q.angle, q.axis);
		return *this;
	}

	Quaternion operator *(const Quaternion& q)const{
		return Quaternion (scal*q.scal - vec*q.vec,
						   vec.cross(q.vec) + q.vec*scal + vec*q.scal);
	}
	
	Quaternion operator *(double f)const{
		return Quaternion (scal*f, vec*f);
	}

	Quaternion operator /(double f)const{
		return Quaternion (scal/f, vec/f);
	}
	Quaternion operator +(const Quaternion& q)const{
		return Quaternion (scal + q.scal, vec + q.vec);
	}

	Quaternion conjugated(){return Quaternion(scal,vec*(-1));}

	Quaternion inverse(){
		double div = square(norm());
		if (div<EPS)return (*this);
		return Quaternion(scal/div, conjugated().vec/div);
	}

	double norm(){
		return sqrt(scal*scal + vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
		//if unit return 1
	}
	
	//Quaternion power (double t){

	//	return Quaternion(cos(angle*t/2.00),axis*sin(angle*t/2.00));
	//}

	OrientationMatrix getOrientationMatrix (){

		Matrix3x3 m( 
			q0*q0+q1*q1-q2*q2-q3*q3, 2*q1*q2 - 2*q0*q3, 2*q1*q3 + 2*q0*q2,
			2*q1*q2 + 2*q0*q3, q0*q0-q1*q1+q2*q2-q3*q3, 2*q2*q3-2*q0*q1,
			2*q1*q3-2*q0*q2, 2*q2*q3+2*q0*q1, q0*q0-q1*q1-q2*q2+q3*q3);

		return OrientationMatrix(m);
	}



};

}
#endif  //__MRCORE__ORIENTATIONMATRIX_H
