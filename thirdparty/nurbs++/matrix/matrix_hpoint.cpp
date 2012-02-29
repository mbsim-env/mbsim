/*=============================================================================
        File: matrix.cpp
     Purpose:       
    Revision: $Id: matrix_hpoint.cpp,v 1.2 2002/05/13 21:07:45 philosophil Exp $
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
    Matrix<HPoint3Df>::norm(void) {
    int i,j ;
    double sumX, sumY, sumZ, sumW, maxsum;
    int init=0 ;
    HPoint3Df *ptr ;
    ptr = m-1 ;
    maxsum = -1 ; // shuts up the warning messages 
    for(i=0;i<rows();++i){
      sumX = 0.0 ;
      sumY = 0.0 ;
      sumZ = 0.0 ;
      sumW = 0.0 ;
      for ( j = 0; j < cols(); ++j) {
	sumX += (*ptr).x() * (*ptr).x() ;
	sumY += (*ptr).y() * (*ptr).y() ;
	sumZ += (*ptr).z() * (*ptr).z() ;
	sumW += (*ptr).w() * (*ptr).w() ;
      }
      if(init)
	maxsum = (maxsum>(sumX+sumY+sumZ+sumW)) ? maxsum : (sumX+sumY+sumZ+sumW);
      else{
	maxsum = (sumX+sumY+sumZ+sumW) ;
	init = 1;
      }
      ++ptr ;
    }
    return sqrt(maxsum);
  }
  
  
  double
    Matrix<HPoint3Dd>::norm(void) {
    int i,j ;
    double sumX, sumY, sumZ, sumW, maxsum;
    int init=0 ;
    HPoint3Dd *ptr ;
    ptr = m-1 ;
    maxsum = -1 ; // shuts up the warning messages 
    for(i=0;i<rows();++i){
      sumX = 0.0 ;
      sumY = 0.0 ;
      sumZ = 0.0 ;
      sumW = 0.0 ;
      for ( j = 0; j < cols(); ++j) {
	sumX += (*ptr).x() * (*ptr).x() ;
	sumY += (*ptr).y() * (*ptr).y() ;
	sumZ += (*ptr).z() * (*ptr).z() ;
	sumW += (*ptr).w() * (*ptr).w() ;
      }
      if(init)
	maxsum = (maxsum>(sumX+sumY+sumZ+sumW)) ? maxsum : (sumX+sumY+sumZ+sumW);
      else{
	maxsum = (sumX+sumY+sumZ+sumW) ;
	init = 1;
      }
      ++ptr ;
    }
    return sqrt(maxsum);
  }
  
  
  double
    Matrix<HPoint2Df>::norm(void) {
    int i,j ;
    double sumX, sumY, sumZ, sumW, maxsum;
    int init=0 ;
    HPoint2Df *ptr ;
    ptr = m-1 ;
    maxsum = -1 ; // shuts up the warning messages 
    for(i=0;i<rows();++i){
      sumX = 0.0 ;
      sumY = 0.0 ;
      sumZ = 0.0 ;
      sumW = 0.0 ;
      for ( j = 0; j < cols(); ++j) {
	sumX += (*ptr).x() * (*ptr).x() ;
	sumY += (*ptr).y() * (*ptr).y() ;
	sumZ += (*ptr).z() * (*ptr).z() ;
	sumW += (*ptr).w() * (*ptr).w() ;
      }
      if(init)
	maxsum = (maxsum>(sumX+sumY+sumZ+sumW)) ? maxsum : (sumX+sumY+sumZ+sumW);
      else{
	maxsum = (sumX+sumY+sumZ+sumW) ;
	init = 1;
      }
      ++ptr ;
    }
    return sqrt(maxsum);
  }
  
  double
    Matrix<HPoint2Dd>::norm(void) {
    int i,j ;
    double sumX, sumY, sumZ, sumW, maxsum;
    int init=0 ;
    HPoint2Dd *ptr ;
    ptr = m-1 ;
    maxsum = -1 ; // shuts up the warning messages 
    for(i=0;i<rows();++i){
      sumX = 0.0 ;
      sumY = 0.0 ;
      sumZ = 0.0 ;
      sumW = 0.0 ;
      for ( j = 0; j < cols(); ++j) {
	sumX += (*ptr).x() * (*ptr).x() ;
	sumY += (*ptr).y() * (*ptr).y() ;
	sumZ += (*ptr).z() * (*ptr).z() ;
	sumW += (*ptr).w() * (*ptr).w() ;
      }
      if(init)
	maxsum = (maxsum>(sumX+sumY+sumZ+sumW)) ? maxsum : (sumX+sumY+sumZ+sumW);
      else{
	maxsum = (sumX+sumY+sumZ+sumW) ;
	init = 1;
      }
      ++ptr ;
    }
    return sqrt(maxsum);
  }
  
  

#ifdef NO_IMPLICIT_TEMPLATES

  // HPoint3D instantiation
  
  template class Matrix<HPoint3Df> ;
  
  template Matrix<HPoint3Df> operator+(const Matrix<HPoint3Df>&,const Matrix<HPoint3Df>&);
  template Matrix<HPoint3Df> operator-(const Matrix<HPoint3Df>&,const Matrix<HPoint3Df>&);
  template Matrix<HPoint3Df> operator*(const Matrix<HPoint3Df>&,const Matrix<HPoint3Df>&);
  template Matrix<HPoint3Df> operator*(const double,const Matrix<HPoint3Df>&);
  template int operator==(const Matrix<HPoint3Df>&,const Matrix<HPoint3Df>&);
  // template int operator!=(const Matrix<HPoint3Df>&,const Matrix<HPoint3Df>&);
  template Matrix<HPoint3Df> comm(const Matrix<HPoint3Df>&,const Matrix<HPoint3Df>&);
  
  
  template class Matrix<HPoint3Dd> ;

  template Matrix<HPoint3Dd> operator+(const Matrix<HPoint3Dd>&,const Matrix<HPoint3Dd>&);
  template Matrix<HPoint3Dd> operator-(const Matrix<HPoint3Dd>&,const Matrix<HPoint3Dd>&);
  template Matrix<HPoint3Dd> operator*(const Matrix<HPoint3Dd>&,const Matrix<HPoint3Dd>&);
  template Matrix<HPoint3Dd> operator*(const double,const Matrix<HPoint3Dd>&);
  template int operator==(const Matrix<HPoint3Dd>&,const Matrix<HPoint3Dd>&);
  //template int operator!=(const Matrix<HPoint3Dd>&,const Matrix<HPoint3Dd>&);
  template Matrix<HPoint3Dd> comm(const Matrix<HPoint3Dd>&,const Matrix<HPoint3Dd>&);
  
  
  // HPoint2D instantiation
  
  template class Matrix<HPoint2Df> ;
  
  template Matrix<HPoint2Df> operator+(const Matrix<HPoint2Df>&,const Matrix<HPoint2Df>&);
  template Matrix<HPoint2Df> operator-(const Matrix<HPoint2Df>&,const Matrix<HPoint2Df>&);
  template Matrix<HPoint2Df> operator*(const Matrix<HPoint2Df>&,const Matrix<HPoint2Df>&);
  template Matrix<HPoint2Df> operator*(const double,const Matrix<HPoint2Df>&);
  template int operator==(const Matrix<HPoint2Df>&,const Matrix<HPoint2Df>&);
  //template int operator!=(const Matrix<HPoint2Df>&,const Matrix<HPoint2Df>&);
  template Matrix<HPoint2Df> comm(const Matrix<HPoint2Df>&,const Matrix<HPoint2Df>&);
  
  
  template class Matrix<HPoint2Dd> ;
  
  template Matrix<HPoint2Dd> operator+(const Matrix<HPoint2Dd>&,const Matrix<HPoint2Dd>&);
  template Matrix<HPoint2Dd> operator-(const Matrix<HPoint2Dd>&,const Matrix<HPoint2Dd>&);
  template Matrix<HPoint2Dd> operator*(const Matrix<HPoint2Dd>&,const Matrix<HPoint2Dd>&);
  template Matrix<HPoint2Dd> operator*(const double,const Matrix<HPoint2Dd>&);
  template int operator==(const Matrix<HPoint2Dd>&,const Matrix<HPoint2Dd>&);
  //template int operator!=(const Matrix<HPoint2Dd>&,const Matrix<HPoint2Dd>&);
  template Matrix<HPoint2Dd> comm(const Matrix<HPoint2Dd>&,const Matrix<HPoint2Dd>&);
  
#endif

}
