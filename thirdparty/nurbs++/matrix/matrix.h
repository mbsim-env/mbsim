/*=============================================================================
        File: matrix.h
     Purpose:       
    Revision: $Id: matrix.h,v 1.3 2002/05/24 17:08:34 philosophil Exp $
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


#ifndef _Matrix_matrix_h_
#define _Matrix_matrix_h_

#include <iostream>
#include "matrix_global.h"
#include "barray2d.h"
#include "vector.h"


// Predefining every friend functions 
// This is required by latest ISO C++ draft

/*!
 */
namespace PLib {
  template <class T> class Matrix ;

  template <class T> PLib::Matrix<T> operator+(const PLib::Matrix<T>&,const PLib::Matrix<T>&);
  template <class T> PLib::Matrix<T> operator-(const PLib::Matrix<T>&,const PLib::Matrix<T>&);
  template <class T> PLib::Matrix<T> operator*(const PLib::Matrix<T>&,const PLib::Matrix<T>&);
  template <class T> PLib::Matrix<T> operator*(const double,const PLib::Matrix<T>&);
  template <class T> PLib::Matrix<T> operator*(const Complex&,const PLib::Matrix<T>&);
  template <class T> PLib::Vector<T> operator*(const PLib::Matrix<T>&,const PLib::Vector<T>&);
  template <class T> int operator==(const PLib::Matrix<T>&,const PLib::Matrix<T>&);
  template <class T> int operator!=(const PLib::Matrix<T>& a,const PLib::Matrix<T>& b) ;
  
  template <> PLib::Matrix<Complex>  operator*(const double d, const PLib::Matrix<Complex> &a);
  template <> PLib::Matrix<Complex>  operator*(const Complex &d, const PLib::Matrix<Complex> &a);
  

  /*!
    \brief A templated matrix class with basic mathematical operators
    
    This is a matrix class which has basic mathematical operators 
    and some routines for input/output.
    
    \author Philippe Lavoie 
    \date 4 Oct. 1996
  */
  template<class T>
    class Matrix : public Basic2DArray<T>
    {
    public:
      Matrix(const int r,const int c) : Basic2DArray<T>(r,c) {}
      Matrix() : Basic2DArray<T>() {}
      Matrix(const Matrix<T>& M) : Basic2DArray<T>(M) {}
      Matrix(T* p, const int r, const int c) : Basic2DArray<T>(p,r,c) {}
      //~Matrix() {}	
      
      Matrix<T>&	operator=(const Matrix<T>&);	
      T operator=(const T v) 
	{ this->reset((T)0); 
	diag(v);
	return v; }
      void submatrix(int i, int j, Matrix<T>&); 
      void as(int rw, int cl, Matrix<T>&) ;
      Matrix<T> get(int rw, int cl, int nr, int nc) const ;
      
      // Mathematical functions
      Matrix<T>& operator+=(const Matrix<T>&);
      Matrix<T>& operator-=(const Matrix<T>&);
      Matrix<T>& operator+=(double d) ;
      Matrix<T>& operator-=(double d) ;
      Matrix<T>& operator*=(double d) ;
      Matrix<T>& operator/=(double d) ;
      
#ifdef HAVE_ISO_FRIEND_DECL
      friend Matrix<T> operator+ <>(const Matrix<T>&,
				    const Matrix<T>&);
      friend Matrix<T> operator- <>(const Matrix<T>&,
				    const Matrix<T>&);
      friend Matrix<T> operator* <>(const Matrix<T>&,
				    const Matrix<T>&);
      friend Matrix<T> operator* <>(const double,
				    const Matrix<T>&);
      friend Matrix<T> operator* <>(const Complex&,
				    const Matrix<T>&);
      friend Vector<T> operator* <>(const Matrix<T>&,
				    const Vector<T>&);
      friend int operator== <>(const Matrix<T>&,
			       const Matrix<T>&);
      friend int operator!= <>(const Matrix<T>& a,
			       const Matrix<T>& b) ;
      
#else
      friend Matrix<T> operator+ (const Matrix<T>&,
				  const Matrix<T>&);
      friend Matrix<T> operator- (const Matrix<T>&,
				  const Matrix<T>&);
      friend Matrix<T> operator* (const Matrix<T>&,
				  const Matrix<T>&);
      friend Matrix<T> operator* (const double,
				  const Matrix<T>&);
      friend Matrix<T> operator* (const Complex&,
				  const Matrix<T>&);
      friend Vector<T> operator* (const Matrix<T>&,
				  const Vector<T>&);
      friend int operator== (const Matrix<T>&,
			     const Matrix<T>&);
      friend int operator!= (const Matrix<T>& a,
			     const Matrix<T>& b) ;
#endif
      
      Matrix<T> herm() const ;
      Matrix<T> transpose() const ;
      Matrix<T> flop() const ; 
      T trace() const ;
      
      double norm(void) ;
      void diag(const T fv);
      Vector<T> getDiag(); 
      
      void qSort() ;
      
      
      // file i/o functions
      int read(char* filename) ; 
      int read(char* filename, int rows, int cols) ;
      int write(char* filename) ; 
      int writeRaw(char* filename) ; 
      
      // calls to LAPACK functions are defined in this class
      friend class LAPACK ;
    };
    
} // end namespace


template <class T>
PLib::Matrix<T> comm( const PLib::Matrix<T>& a, const PLib::Matrix<T>& b);

template <class T>
inline PLib::Matrix<T> herm( const PLib::Matrix<T>& a) {
  return a.herm() ;
}

template <class T>
inline PLib::Matrix<T> transpose( const PLib::Matrix<T>& a) {
  return a.transpose() ;
}

template <class T>
inline T trace( const PLib::Matrix<T>& a) {
  return a.trace() ;
}

template <class T>
inline int operator!=(const PLib::Matrix<T>& a, const PLib::Matrix<T>& b) {
  return a==b?0:1 ;
}


typedef PLib::Matrix<int> Matrix_INT ;
typedef PLib::Matrix<char> Matrix_BYTE ;
typedef PLib::Matrix<float> Matrix_FLOAT ;
typedef PLib::Matrix<double> Matrix_DOUBLE ;
typedef PLib::Matrix<Complex> Matrix_COMPLEX ;
typedef PLib::Matrix<unsigned char> Matrix_UBYTE ;
typedef PLib::Matrix<PLib::Point3Df> Matrix_Point3Df ;
typedef PLib::Matrix<PLib::HPoint3Df> Matrix_HPoint3Df ;
typedef PLib::Matrix<PLib::Point3Dd> Matrix_Point3Dd ;
typedef PLib::Matrix<PLib::HPoint3Dd> Matrix_HPoint3Dd ;
typedef PLib::Matrix<PLib::Point2Df> Matrix_Point2Df ;
typedef PLib::Matrix<PLib::HPoint2Df> Matrix_HPoint2Df ;
typedef PLib::Matrix<PLib::Point2Dd> Matrix_Point2Dd ;
typedef PLib::Matrix<PLib::HPoint2Dd> Matrix_HPoint2Dd ;

typedef PLib::Matrix<int> PlMatrix_int ;
typedef PLib::Matrix<char> PlMatrix_byte ;
typedef PLib::Matrix<float> PlMatrix_float ;
typedef PLib::Matrix<double> PlMatrix_double ;
typedef PLib::Matrix<Complex> PlMatrix_complex ;
typedef PLib::Matrix<unsigned char> PlMatrix_ubyte ;
typedef PLib::Matrix<PLib::Point3Df> PlMatrix_Point3Df ;
typedef PLib::Matrix<PLib::HPoint3Df> PlMatrix_HPoint3Df ;
typedef PLib::Matrix<PLib::Point3Dd> PlMatrix_Point3Dd ;
typedef PLib::Matrix<PLib::HPoint3Dd> PlMatrix_HPoint3Dd ;
typedef PLib::Matrix<PLib::Point3Df> PlMatrix_Point2Df ;
typedef PLib::Matrix<PLib::HPoint3Df> PlMatrix_HPoint2Df ;
typedef PLib::Matrix<PLib::Point3Dd> PlMatrix_Point2Dd ;
typedef PLib::Matrix<PLib::HPoint3Dd> PlMatrix_HPoint2Dd ;

#ifdef INCLUDE_TEMPLATE_SOURCE
#include "matrix.cpp"
#endif

#endif 
