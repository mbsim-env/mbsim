/*=============================================================================
        File: matrix.cpp
     Purpose:       
    Revision: $Id: matrix_point.cpp,v 1.2 2002/05/13 21:07:45 philosophil Exp $
  Created by: Philippe Lavoie          (3 Oct, 1996)
 Modified by: 

 Copyright notice:
          Copyright (C) 1996-1998 Philippe Lavoie
 
	  This library is free software; you can redistribute it and/or
	  modify it under the terms of the GNU Library General Public
	  License as published by the Free Software Foundation; either
	  version 2 of the License, or (at your option) any later version.
 
	  This library is distributed in the hope that it will be useful,
	  but WITHOUT ANY WARRANTY; without even the implied warranty of
	  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	  Library General Public License for more details.
 
	  You should have received a copy of the GNU Library General Public
	  License along with this library; if not, write to the Free
	  Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
=============================================================================*/

#include "matrix.cpp"

namespace PLib {

  double
    Matrix<Point3Df>::norm(void) {
    int i,j ;
    double sumX, sumY, sumZ, maxsum;
    int init=0 ;
    Point3Df *ptr ;
    ptr = m-1 ;
    maxsum = -1 ; // shuts up the warning messages 
    for(i=0;i<rows();++i){
      sumX = 0.0 ;
      sumY = 0.0 ;
      sumZ = 0.0 ;
      for ( j = 0; j < cols(); ++j) {
	sumX += (*ptr).x() * (*ptr).x() ;
	sumY += (*ptr).y() * (*ptr).y() ;
	sumZ += (*ptr).z() * (*ptr).z() ;
      }
      if(init)
	maxsum = (maxsum>(sumX+sumY+sumZ)) ? maxsum : (sumX+sumY+sumZ);
      else{
	maxsum = (sumX+sumY+sumZ) ;
	init = 1;
      }
      ++ptr ;
    }
    return sqrt(maxsum);
  }
  
  double
    Matrix<Point3Dd>::norm(void) {
    int i,j ;
    double sumX, sumY, sumZ, maxsum;
    int init=0 ;
    Point3Dd *ptr ;
    ptr = m-1 ;
    maxsum = -1 ; // shuts up the warning messages 
    for(i=0;i<rows();++i){
      sumX = 0.0 ;
      sumY = 0.0 ;
      sumZ = 0.0 ;
      for ( j = 0; j < cols(); ++j) {
	sumX += (*ptr).x() * (*ptr).x() ;
	sumY += (*ptr).y() * (*ptr).y() ;
	sumZ += (*ptr).z() * (*ptr).z() ;
      }
      if(init)
	maxsum = (maxsum>(sumX+sumY+sumZ)) ? maxsum : (sumX+sumY+sumZ);
      else{
	maxsum = (sumX+sumY+sumZ) ;
	init = 1;
      }
      ++ptr ;
    }
    return sqrt(maxsum);
  }
  
  double
    Matrix<Point2Df>::norm(void) {
    int i,j ;
    double sumX, sumY, sumZ, maxsum;
    int init=0 ;
    Point2Df *ptr ;
    ptr = m-1 ;
    maxsum = -1 ; // shuts up the warning messages 
    for(i=0;i<rows();++i){
      sumX = 0.0 ;
      sumY = 0.0 ;
      sumZ = 0.0 ;
      for ( j = 0; j < cols(); ++j) {
	sumX += (*ptr).x() * (*ptr).x() ;
	sumY += (*ptr).y() * (*ptr).y() ;
	sumZ += (*ptr).z() * (*ptr).z() ;
      }
      if(init)
	maxsum = (maxsum>(sumX+sumY+sumZ)) ? maxsum : (sumX+sumY+sumZ);
      else{
	maxsum = (sumX+sumY+sumZ) ;
	init = 1;
      }
      ++ptr ;
    }
    return sqrt(maxsum);
  }

  double
    Matrix<Point2Dd>::norm(void) {
    int i,j ;
    double sumX, sumY, sumZ, maxsum;
    int init=0 ;
    Point2Dd *ptr ;
    ptr = m-1 ;
    maxsum = -1 ; // shuts up the warning messages 
    for(i=0;i<rows();++i){
      sumX = 0.0 ;
      sumY = 0.0 ;
      sumZ = 0.0 ;
      for ( j = 0; j < cols(); ++j) {
	sumX += (*ptr).x() * (*ptr).x() ;
	sumY += (*ptr).y() * (*ptr).y() ;
	sumZ += (*ptr).z() * (*ptr).z() ;
      }
      if(init)
	maxsum = (maxsum>(sumX+sumY+sumZ)) ? maxsum : (sumX+sumY+sumZ);
      else{
	maxsum = (sumX+sumY+sumZ) ;
	init = 1;
      }
      ++ptr ;
    }
    return sqrt(maxsum);
  }
  

#ifdef NO_IMPLICIT_TEMPLATES

  // Point3D instantiation
  
  template class Matrix<Point3Df> ;
  
  template Matrix<Point3Df> operator+(const Matrix<Point3Df>&,const Matrix<Point3Df>&);
  template Matrix<Point3Df> operator-(const Matrix<Point3Df>&,const Matrix<Point3Df>&);
  template Matrix<Point3Df> operator*(const Matrix<Point3Df>&,const Matrix<Point3Df>&);
  template Matrix<Point3Df> operator*(const double,const Matrix<Point3Df>&);
  template int operator==(const Matrix<Point3Df>&,const Matrix<Point3Df>&);
  // template int operator!=(const Matrix<Point3Df>&,const Matrix<Point3Df>&);
  template Matrix<Point3Df> comm(const Matrix<Point3Df>&,const Matrix<Point3Df>&);
  
  
  template class Matrix<Point3Dd> ;
  
  template Matrix<Point3Dd> operator+(const Matrix<Point3Dd>&,const Matrix<Point3Dd>&);
  template Matrix<Point3Dd> operator-(const Matrix<Point3Dd>&,const Matrix<Point3Dd>&);
  template Matrix<Point3Dd> operator*(const Matrix<Point3Dd>&,const Matrix<Point3Dd>&);
  template Matrix<Point3Dd> operator*(const double,const Matrix<Point3Dd>&);
  template int operator==(const Matrix<Point3Dd>&,const Matrix<Point3Dd>&);
  //template int operator!=(const Matrix<Point3Dd>&,const Matrix<Point3Dd>&);
  template Matrix<Point3Dd> comm(const Matrix<Point3Dd>&,const Matrix<Point3Dd>&);
  
  // Point2D instantiation
  
  template class Matrix<Point2Df> ;
  
  template Matrix<Point2Df> operator+(const Matrix<Point2Df>&,const Matrix<Point2Df>&);
  template Matrix<Point2Df> operator-(const Matrix<Point2Df>&,const Matrix<Point2Df>&);
  template Matrix<Point2Df> operator*(const Matrix<Point2Df>&,const Matrix<Point2Df>&);
  template Matrix<Point2Df> operator*(const double,const Matrix<Point2Df>&);
  template int operator==(const Matrix<Point2Df>&,const Matrix<Point2Df>&);
  //template int operator!=(const Matrix<Point2Df>&,const Matrix<Point2Df>&);
  template Matrix<Point2Df> comm(const Matrix<Point2Df>&,const Matrix<Point2Df>&);
  

  template class Matrix<Point2Dd> ;
  
  template Matrix<Point2Dd> operator+(const Matrix<Point2Dd>&,const Matrix<Point2Dd>&);
  template Matrix<Point2Dd> operator-(const Matrix<Point2Dd>&,const Matrix<Point2Dd>&);
  template Matrix<Point2Dd> operator*(const Matrix<Point2Dd>&,const Matrix<Point2Dd>&);
  template Matrix<Point2Dd> operator*(const double,const Matrix<Point2Dd>&);
  template int operator==(const Matrix<Point2Dd>&,const Matrix<Point2Dd>&);
  //template int operator!=(const Matrix<Point2Dd>&,const Matrix<Point2Dd>&);
  template Matrix<Point2Dd> comm(const Matrix<Point2Dd>&,const Matrix<Point2Dd>&);

#endif

}
