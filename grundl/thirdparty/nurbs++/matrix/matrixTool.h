/*============================================================================
        File: matrixTool.h
     Purpose: 
    Revision: $Id: matrixTool.h,v 1.2 2002/05/13 21:07:45 philosophil Exp $
  Created by: Philippe Lavoie          (26 January, 1998)
 Modified by: 

 Copyright notice:
          Copyright (C) 1996-1999 Philippe Lavoie
 
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

#ifndef _Matrix_matrixTool_h_
#define _Matrix_matrixTool_h_

#include "matrix_global.h"

namespace PLib {




  template <class T>
  inline int compareT(T*a, T* b){
    if(*(a) > *(b))
      return 1 ;
    if(*(a) < *(b))
      return -1 ;
    return 0 ;
  }

  // These functions are needed for qSort (in vector.h and matrix.h)
  inline int compareInt(const void* a, const void* b){
    return compareT((int*)a,(int*)b) ;
  }

  inline int compareFloat(const void* a, const void* b){
    return compareT((float*)a,(float*)b) ;
  }

  inline int compareDouble(const void* a, const void* b){
    return compareT((float*)a,(float*)b) ;
  }

  template <class T>
  inline T absolute(T a){
    return ( a<T() ) ? -a : a ;
  }

  inline double absolute(double a) { return fabs(a) ; }
  inline float absolute(float a) { return fabs(a) ; }

  template <class T>
  inline T to2power(T a){
    return a*a ;
  }

  template <class T>
  inline T minimum(T a, T b){
    return a<b ? a : b ;
  }

  template <class T>
  inline T maximum(T a, T b){
      return a>b ? a : b ;
  }

  // Use minimumRef or maximumRef if you want to pass by reference
  // You should only use this for special types and not for
  // the base types
  template <class T>
  inline T minimumRef(const T &a, const T &b){
    return a<b ? a : b ;
  }

  template <class T>
  inline T maximumRef(const T &a, const T &b){
      return a>b ? a : b ;
  }


  // Some often used inline functions

  // definition for Complex, HPoint_3D, Point_3D, Vector2 and Coordinate
  // follows

  inline Complex minimum(Complex a, Complex b){
    double r,i ;
    r = minimum(real(a),real(b)) ;
    i = minimum(imag(a),imag(b)) ;
    return Complex(r,i) ;
  }

  inline Complex maximum(Complex a, Complex b){
  double r,i ;
  r = maximum(real(a),real(b)) ;
  i = maximum(imag(a),imag(b)) ;
  return Complex(r,i) ;
  }

  
  inline Complex minimumByRef(const Complex &a, const Complex &b){
    double r,i ;
    r = minimum(real(a),real(b)) ;
    i = minimum(imag(a),imag(b)) ;
    return Complex(r,i) ;
  }


  inline Complex maximumByRef(const Complex &a, const Complex &b){
    double r,i ;
    r = maximum(real(a),real(b)) ;
    i = maximum(imag(a),imag(b)) ;
    return Complex(r,i) ;
  }


  /*!
    \fn void boundTo(T& a, T low, T high)
    \brief bounds a variable to a region

    Bounds a value to a region, the variable $a$ is not changed
    if it's already in the region. Otherwise it is set to one 
    of the boundary value.
    
    \param a  the variable to bound
    \param low  the low bound value
    \param  high  the higest bound value
    \return The variable $a$ is now in the range $[low,high]$.

    \warning The variable low must be smaller then high.
    \author Philippe Lavoie 
    \date 24 January 1997
  */
  template<class T> inline void boundTo(T& a, T low, T high){
    if(a<=low) a = low ;
    if(a>=high) a = high ;
  }

} // end namespace

#endif
