/*============================================================================
        File: matrixRT.h
     Purpose: Describes a rotation-translation matrix 
    Revision: $Id: matrixRT.h,v 1.2 2002/05/13 21:07:46 philosophil Exp $
  Created by: Philippe Lavoie          (3 Oct, 1996)
 Modified by: 

 Copyright notice:
          Copyright (C) 1996-1997 Philippe Lavoie
 
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
#ifndef _nurbs_matrixRT_h_
#define _nurbs_matrixRT_h_

#include "nurbs_global.h"
#include "matrix.h"
// Predefining every friend functions
// This is required by latest ISO C++ draft

/*!
 */
namespace PLib {
  template <class T> class MatrixRT ; 

  template <class T> MatrixRT<T> operator*(const MatrixRT<T>&,const MatrixRT<T>&) ;


/*!
  \class MatrixRT matrixRT.h
  \brief a matrix for rotation and translation transformation

  This is a matrix for the rotation, translation and scaling of a point in 
  3D or 4D.

  \author Philippe Lavoie 
  \date 25 July 1997
 */
template <class T>
class MatrixRT : public Matrix<T> {
public:
  MatrixRT(T ax, T ay, T az, T x, T y, T z) ;
  MatrixRT() ;
  MatrixRT(T* p) ;
  MatrixRT(const Matrix<T>& plM ) ;
  
  MatrixRT<T>& rotate(T ax,T ay, T az);
  MatrixRT<T>& rotateXYZ(T ax,T ay, T az);
  MatrixRT<T>& translate(T x, T y, T z) ;
  MatrixRT<T>& scale(T x, T y, T z) ;
  
  //! A rotation with the angles specified in degree
  MatrixRT<T>& rotateDeg(T ax, T ay, T az) { return rotate(T(ax*M_PI/180.0),T(ay*M_PI/180.0),T(az*M_PI/180.0)) ; } 

  //! A rotation in the X,Y and Z order with the angles specified in degree
  MatrixRT<T>& rotateDegXYZ(T ax, T ay, T az) { return rotateXYZ(T(ax*M_PI/180.0),T(ay*M_PI/180.0),T(az*M_PI/180.0)) ; } 

  MatrixRT<T>& operator=(const Matrix<T>& M) ;
  MatrixRT<T>& operator=(const MatrixRT<T>& M) ;

#ifdef HAVE_ISO_FRIEND_DECL
  friend MatrixRT<T> operator* <>(const MatrixRT<T>&,const MatrixRT<T>&) ;
#else
  friend MatrixRT<T> operator* (const MatrixRT<T>&,const MatrixRT<T>&) ;
#endif

protected:
  //! indicate if the data was initialized by this class or not.
  int created ;
};


#ifdef HAVE_TEMPLATE_OF_TEMPLATE
template <int N>
inline HPoint_nD<float,N> operator*(const MatrixRT<double>& M, const HPoint_nD<float,N>& P){
  HPoint_nD<float,N> P2 ;

  P2.x() = float(M(0,0)*(double)P.x() + M(0,1)*(double)P.y() + M(0,2)*(double)P.z() + M(0,3)*(double)P.w()) ;
  P2.y() = float(M(1,0)*(double)P.x() + M(1,1)*(double)P.y() + M(1,2)*(double)P.z() + M(1,3)*(double)P.w()) ;
  P2.z() = float(M(2,0)*(double)P.x() + M(2,1)*(double)P.y() + M(2,2)*(double)P.z() + M(2,3)*(double)P.w()) ;
  P2.w() = float(M(3,0)*(double)P.x() + M(3,1)*(double)P.y() + M(3,2)*(double)P.z() + M(3,3)*(double)P.w()) ;

  return P2 ;
}
#else

inline HPoint_nD<float,2> operator*(const MatrixRT<double>& M, const HPoint_nD<float,2>& P){
  HPoint_nD<float,2> P2 ;

  P2.x() = float(M(0,0)*(double)P.x() + M(0,1)*(double)P.y() + M(0,2)*(double)P.z() + M(0,3)*(double)P.w()) ;
  P2.y() = float(M(1,0)*(double)P.x() + M(1,1)*(double)P.y() + M(1,2)*(double)P.z() + M(1,3)*(double)P.w()) ;
  P2.z() = float(M(2,0)*(double)P.x() + M(2,1)*(double)P.y() + M(2,2)*(double)P.z() + M(2,3)*(double)P.w()) ;
  P2.w() = float(M(3,0)*(double)P.x() + M(3,1)*(double)P.y() + M(3,2)*(double)P.z() + M(3,3)*(double)P.w()) ;

  return P2 ;
}


inline HPoint_nD<float,3> operator*(const MatrixRT<double>& M, const HPoint_nD<float,3>& P){
  HPoint_nD<float,3> P2 ;

  P2.x() = float(M(0,0)*(double)P.x() + M(0,1)*(double)P.y() + M(0,2)*(double)P.z() + M(0,3)*(double)P.w()) ;
  P2.y() = float(M(1,0)*(double)P.x() + M(1,1)*(double)P.y() + M(1,2)*(double)P.z() + M(1,3)*(double)P.w()) ;
  P2.z() = float(M(2,0)*(double)P.x() + M(2,1)*(double)P.y() + M(2,2)*(double)P.z() + M(2,3)*(double)P.w()) ;
  P2.w() = float(M(3,0)*(double)P.x() + M(3,1)*(double)P.y() + M(3,2)*(double)P.z() + M(3,3)*(double)P.w()) ;

  return P2 ;
}
#endif

template <class T, int N> HPoint_nD<T,N> operator*(const MatrixRT<T>&,const HPoint_nD<T,N>&);
template <class T, int N> Point_nD<T,N> operator*(const MatrixRT<T>&,const Point_nD<T,N>&);



} // end namespace 

typedef PLib::MatrixRT<float> MatrixRT_FLOAT ;
typedef PLib::MatrixRT<double> MatrixRT_DOUBLE ;
typedef PLib::MatrixRT<float> MatrixRTf ;
typedef PLib::MatrixRT<double> MatrixRTd ;

#ifdef INCLUDE_TEMPLATE_SOURCE
#include "matrixRT.cpp"
#endif


#endif
