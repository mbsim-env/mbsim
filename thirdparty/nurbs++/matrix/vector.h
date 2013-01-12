/*=============================================================================
        File: vector.h
     Purpose:       
    Revision: $Id: vector.h,v 1.2 2002/05/13 21:07:45 philosophil Exp $
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

#ifndef _Matrix_vector_h_
#define _Matrix_vector_h_

#include <math.h>
#include "barray.h"
#include "specialType.h"

// Predefining every friend functions 
// This is required by latest ISO C++ draft

/*!
 */
namespace PLib {

  template <class T> class Vector ;

  template <class T> Vector<T> operator+(const Vector<T>&, const Vector<T>&);
  template <class T> Vector<T> operator-(const Vector<T>&, const Vector<T>&);

  template <class T> T operator*(const Vector<T>&,const Vector<T>&); 
  template <class T> Vector<T> operator*(const Vector<T>& v, const double d);
  template <class T> Vector<T> operator*(const Vector<T>& v, const Complex d);
		    
  template <class T> Vector<T> operator*(const double d,const Vector<T>& v) ;
  template <class T> Vector<T> operator*(const Complex d,const Vector<T>& v) ;

  template<> Vector<Complex>  operator*(const Vector<Complex>& v, const double d);
  template <> Vector<Complex>  operator*(const Vector<Complex>& v, const Complex d);

  template <class T> int operator==(const Vector<T>&,const Vector<T>&);
  template <class T> int operator!=(const Vector<T>& a,const Vector<T>& b);


  /*!
    \brief A template vector class

    A simple vector class with basic linear algebraic vector 
    operators defined.

    \author Philippe Lavoie 
    \date 4 Oct. 1996
  */
  template<class T> class Vector : public BasicArray<T>
  {
  public:
    int rows() const //!< a reference to the size of the vector
      { return this->sze ;}
    Vector() : BasicArray<T>(1) {} //!< Basic constructor
    Vector(const int r) : BasicArray<T>(r) {}
    Vector(const Vector<T>& v) : BasicArray<T>(v) {}
    Vector(const BasicArray<T>& v) : BasicArray<T>(v)  {}
    Vector(T* ap, const int size) : BasicArray<T>(ap,size) {}
    Vector(BasicList<T>& list) : BasicArray<T>(list) {}
    
    virtual ~Vector() {}
    
    Vector<T>& operator=(const Vector<T>& v);
    Vector<T>& operator=(const BasicArray<T>& b);
    
    Vector<T>& operator+=(const Vector<T>& a);
    Vector<T>& operator-=(const Vector<T>& a);
    
    T operator=(const T d);
    void as(int i, const Vector<T>& b);
    Vector<T> get(int i, int l);
    
    int minIndex() const ;
    T minimum() const { return this->operator[](minIndex()) ; }  // returns the minimal value inside the vector
    
    void qSortStd() ;
    void qSort(int M=7) ; 
    void sortIndex(Vector<int>& index, int M=7) const;
    
    
#ifdef HAVE_ISO_FRIEND_DECL
    friend Vector<T> operator+ <>(const Vector<T> &a, const Vector<T> &b);
    friend Vector<T> operator- <>(const Vector<T> &a, const Vector<T> &b);
    friend T operator* <>(const Vector<T> &a,const Vector<T> &b); 
    
    friend Vector<T> operator* <>(const Vector<T>& v, const double d);
    friend Vector<T> operator* <>(const Vector<T>& v, const Complex d);
    
    friend Vector<T> operator* <>(const double d,const Vector<T>& v) ;
    friend Vector<T> operator* <>(const Complex d,const Vector<T>& v) ;
    
    friend int operator== <>(const Vector<T> &a,const Vector<T> &b);
    friend int operator!= <>(const Vector<T>& a,const Vector<T>& b);
    
#else
    friend Vector<T> operator+(const Vector<T> &a, const Vector<T> &b);
    friend Vector<T> operator-(const Vector<T> &a, const Vector<T> &b);
    friend T operator* (const Vector<T> &a,const Vector<T> &b); 
    
    friend Vector<T> operator*(const Vector<T>& v, const double d);
    friend Vector<T> operator*(const Vector<T>& v, const Complex d);
    			      
    friend Vector<T> operator*(const double d,const Vector<T>& v) ;
    friend Vector<T> operator*(const Complex d,const Vector<T>& v) ;
    
    friend int operator==(const Vector<T> &a,const Vector<T> &b);
    friend int operator!=(const Vector<T>& a,const Vector<T>& b);
    
#endif
    
  };
  
  
  template<class T> inline 
    int operator!=(const Vector<T>& a, const Vector<T>& b) //!< The inequality operator
    {
      return (a==b)?0:1 ; 
    }
  
  template<class T> inline 
    Vector<T>  operator*(const double d, const Vector<T>& v) //!< Multiplies by a double
    {
      return v*d ;
    }
  
  template<class T> inline 
    Vector<T>  operator*(const Complex d, const Vector<T>& v) //!< Multiplies by a Comples 
    {
      return v*d ;
    }
  
} // end namespace


typedef PLib::Vector<int> Vector_INT ;
typedef PLib::Vector<char> Vector_BYTE ;
typedef PLib::Vector<float> Vector_FLOAT ;
typedef PLib::Vector<double> Vector_DOUBLE ;
typedef PLib::Vector<Complex> Vector_COMPLEX ;
typedef PLib::Vector<unsigned char> Vector_UBYTE ;
typedef PLib::Vector<PLib::HPoint3Df> Vector_HPoint3Df;
typedef PLib::Vector<PLib::Point3Df> Vector_Point3Df ;
typedef PLib::Vector<PLib::HPoint3Dd> Vector_HPoint3Dd;
typedef PLib::Vector<PLib::Point3Dd> Vector_Point3Dd ;
typedef PLib::Vector<PLib::HPoint2Df> Vector_HPoint2Df;
typedef PLib::Vector<PLib::Point2Df> Vector_Point2Df ;
typedef PLib::Vector<PLib::HPoint2Dd> Vector_HPoint2Dd;
typedef PLib::Vector<PLib::Point2Dd> Vector_Point2Dd ;

typedef PLib::Vector<int> PlVector_int ;
typedef PLib::Vector<char> PlVector_byte ;
typedef PLib::Vector<float> PlVector_float ;
typedef PLib::Vector<double> PlVector_double ;
typedef PLib::Vector<Complex> PlVector_complex ;
typedef PLib::Vector<unsigned char> PlVector_ubyte ;
typedef PLib::Vector<PLib::HPoint3Df> PlVector_HPoint3Df;
typedef PLib::Vector<PLib::Point3Df> PlVector_Point3Df ;
typedef PLib::Vector<PLib::HPoint3Dd> PlVector_HPoint3Dd;
typedef PLib::Vector<PLib::Point3Dd> PlVector_Point3Dd ;
typedef PLib::Vector<PLib::HPoint2Df> PlVector_HPoint2Df;
typedef PLib::Vector<PLib::Point2Df> PlVector_Point2Df ;
typedef PLib::Vector<PLib::HPoint2Dd> PlVector_HPoint2Dd;
typedef PLib::Vector<PLib::Point2Dd> PlVector_Point2Dd ;


#ifdef INCLUDE_TEMPLATE_SOURCE
#include "vector.cpp"
#endif

#endif 
