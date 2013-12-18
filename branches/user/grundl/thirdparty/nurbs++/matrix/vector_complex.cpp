/*=============================================================================
        File: vector.cpp
     Purpose:
    Revision: $Id: vector_complex.cpp,v 1.2 2002/05/13 21:07:45 philosophil Exp $
  Created by:    Philippe Lavoie          (3 Oct, 1996)
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

#include "vector.cpp"

namespace PLib {

  template<>
    Vector<Complex>  operator*(const Vector<Complex>& v, const double d)
    {
      int i, sz = v.size();
      Vector<Complex> b(v.size());
      
      Complex *aptr,*bptr ;
      aptr = v.x-1 ;
      bptr = b.x-1 ;
      for (i = sz; i > 0; --i){
	*(++bptr) = (Complex)(d * (*(++aptr)) ) ;
      }
      
      return b;
      
    }
  
  template <>
    Vector<Complex>  operator*(const Vector<Complex>& v, 
			       const Complex d)
    {
      int i, sz = v.size();
      Vector<Complex> b = v;
      
      Complex *bptr=b.x ;
      for (i = sz; i > 0; --i)
	*(++bptr) *= d;
      
      return b;
      
    }
  
  template <>
    int Vector<Complex>::minIndex() const {
    Complex min = x[0] ;
    int index = 0 ;
    
    Complex *p ;
    p = x-1 ;
    
    for(int i=1;i<n();i++){
      if(abs(*(++p))<abs(min)){
	min = *p ;
	index = i ;
      }
    }
    return index ;
  }

  template <>
    void Vector<Complex>::qSort(int){
#ifdef USE_EXCEPTION
    throw MatrixErr() ;
#else
    Error error("Vector<Complex>::qSort()");
    error << "Sorry! The complex quick sort operator isn't working.\n" ;
    error.fatal() ;
#endif
  }
  
  template <>
    void Vector<Complex>::sortIndex(Vector<int>&, int) const {
#ifdef USE_EXCEPTION
    throw MatrixErr() ;
#else
    Error error("Vector<Complex>::sortIndex()");
    error << "Sorry! The complex quick sort operator isn't working.\n" ;
    error.fatal() ;
#endif
  }
  
  
#ifdef NO_IMPLICIT_TEMPLATES
  
  template class Vector<Complex> ;
  
  template Vector<Complex> operator+(const Vector<Complex>&, const Vector<Complex>&);
  template Vector<Complex> operator-(const Vector<Complex>&, const Vector<Complex>&);
  template Complex operator*(const Vector<Complex>&,const Vector<Complex>&);
  template int operator==(const Vector<Complex>&,const Vector<Complex>&);
  template int operator!=(const Vector<Complex>&,const Vector<Complex>&);

#endif

}
