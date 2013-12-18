/*=============================================================================
        File: barray.h
     Purpose:      
    Revision: $Id: barray.h,v 1.3 2002/05/24 17:08:34 philosophil Exp $
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

#ifndef _MATRIX_barray_h_
#define _MATRIX_barray_h_

#include "matrix_global.h"
#include "specialType.h"
#include "list.h"

// Predefining every friend functions
// This is required by the latest ISO C++ draft

/*!
 */
namespace PLib {
  template <class T> class BasicArray ;

  template <class T> int operator!=(const BasicArray<T>&,const BasicArray<T>&);
  template <class T> int operator==(const BasicArray<T>&,const BasicArray<T>&);
  template <class T> istream& operator>>(istream& is, BasicArray<T>& arry);
  template <class T> ostream& operator<<(ostream& os, const BasicArray<T>& arry);

#include "galloc.h"


/*!
  \brief A basic templated array class

  This is a basis array class, the only particularity is that 
  the resize is not destructive. 

  \author Philippe Lavoie 
  \date 4 October 1996
*/
template<class T> class BasicArray
{
public:
  int n() const //!< the perceived size of the array
    { return sze; } 
  BasicArray();
  BasicArray(const int ni);
  BasicArray(const BasicArray<T>& f2);
  BasicArray(T* ap, const int size) ;  
  BasicArray(BasicList<T>& list) ;
  virtual ~BasicArray();
  
  BasicArray<T>& operator=(const BasicArray<T>& f2);
  
  int size() const //!< returns the size of the array
    { return sze; } 
  void resize(const int nsize) 
    { resizeBasicArray(*this,nsize) ; }
  void resize(const BasicArray<T>& A) //!< resize the array with the same dimension of the vector A
    { resize(A.n()); } 
  
  void trim(const int nsize);    
  void clear();
  void untrim()	//!< sets the array to the maximal allocated size
    { sze = rsize; } 
  
  T& push_back(const T i, int end_buffer=10, double end_mult=-1);

  
  virtual void reset(const T val = 0.0);
  T operator=(const T val) //!< set all elements of the vector to val
    { reset(val); return val; } 
  
#ifdef DEBUG_PLIB
  T& operator[](const int i) ; //!< checks that i is in the proper range and returns x[i]
  T  operator[](const int i) const ; //!< checks that i is in the proper range and returns x[i]
#else
  T& operator[](const int i) //!< no range checks are performed
    { return x[i]; } 
  T  operator[](const int i) const  //!< no range checks are performed 
    { return x[i]; } 
#endif
  T* memory() const //!< returns the data pointer 
    { return x ; }

  void width(const int w) //!< set output columns
    { wdth = w; }   

#ifdef HAVE_ISO_FRIEND_DECL
  friend int operator!= <>(const BasicArray<T>&,const BasicArray<T>&);    
  friend int operator== <>(const BasicArray<T>&,const BasicArray<T>&);      
  friend istream& operator>> <>(istream& is, BasicArray<T>& arry);
  friend ostream& operator<< <>(ostream& os, const BasicArray<T>& arry);
#else
  friend int operator!= (const BasicArray<T>&,const BasicArray<T>&);    
  friend int operator== (const BasicArray<T>&,const BasicArray<T>&);      
  friend istream& operator>> (istream& is, BasicArray<T>& arry);
  friend ostream& operator<< (ostream& os, const BasicArray<T>& arry);
#endif

  ostream& print(ostream& os) const ; 

  FRIEND_ARRAY_ALLOCATOR 

  // compatibility for std:vector
  typedef T* iterator ;
  typedef const T* const_iterator ;

  iterator begin() { return (0<sze) ? x : 0 ; }
  const_iterator begin() const { return (0<sze) ? x : 0 ; }

  iterator end() { return (0<sze) ? x+sze : 0; }
  const_iterator end() const { return (0<sze) ? x+sze : 0; }

protected:
  int rsize; //!< the actual size of the array
  int wdth;  //!< the number of output columns
  int destruct ; //!< specifies if the data should be destroyed on exit
  int sze; //!< the known size of the array
  T *x;   //!< the data pointer
};

} // end namespace 

typedef PLib::BasicArray<int> BasicArray_INT ;            
typedef PLib::BasicArray<char> BasicArray_BYTE ;          
typedef PLib::BasicArray<double> BasicArray_DOUBLE ;      
typedef PLib::BasicArray<float> BasicArray_FLOAT ;      
typedef PLib::BasicArray<Complex> BasicArray_COMPLEX ;    
typedef PLib::BasicArray<unsigned char> BasicArray_UBYTE ;
typedef PLib::BasicArray<PLib::HPoint3Df> BasicArray_HPoint3D ;
typedef PLib::BasicArray<PLib::Point3Df> BasicArray_Point3D ;
typedef PLib::BasicArray<PLib::HPoint3Dd> BasicArray_HPoint3Dd ;
typedef PLib::BasicArray<PLib::Point3Dd> BasicArray_Point3Dd ;
typedef PLib::BasicArray<PLib::HPoint2Df> BasicArray_HPoint2D ;
typedef PLib::BasicArray<PLib::Point2Df> BasicArray_Point2D ;
typedef PLib::BasicArray<PLib::HPoint2Dd> BasicArray_HPoint2Dd ;
typedef PLib::BasicArray<PLib::Point2Dd> BasicArray_Point2Dd ;
typedef PLib::BasicArray<void*> BasicArray_VoidPtr ;
typedef PLib::BasicArray<PLib::Coordinate> BasicArray_Coordinate ;

#ifdef INCLUDE_TEMPLATE_SOURCE
#include "barray.cpp"
#include "barray_hpoint.cpp"
#endif



#endif 
