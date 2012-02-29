/*=============================================================================
        File: matrix.cpp
     Purpose:       
    Revision: $Id: matrix_complex.cpp,v 1.3 2002/05/24 17:08:34 philosophil Exp $
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

  double Matrix<Complex>::norm(void){
    int i,j ;
    double sumR, sumI, maxsum;
    int init=0 ;
    Complex *ptr ;
    ptr = m-1 ;
    maxsum = -1 ; // shuts up the warning messages
    for(i=0;i<rows();++i){
      sumR = 0.0 ;
      sumI = 0.0 ;
      for ( j = 0; j < cols(); ++j) {
	sumR += real(*ptr)*real(*ptr) ;
	sumI += imag(*ptr)*imag(*ptr) ;
      }
      if(init)
	maxsum = (maxsum>(sumR+sumI)) ? maxsum : (sumR+sumI);
      else{
	maxsum = (sumR+sumI) ;
	init = 1;
	}
      ++ptr ;
    }
    return sqrt(maxsum);
  }
  
  template <>
    Matrix<Complex>  operator*(const double d,
			       const Matrix<Complex> &a)
    {
      int i, size=a.rows()*a.cols() ;
      Matrix<Complex> b(a.rows(),a.cols());
      
      Complex *bptr,*aptr ;
      bptr = b.m - 1 ;
      aptr = a.m - 1 ;
      for (i = size; i > 0 ; --i)
	*(++bptr) = (Complex)(d * (*(++aptr))) ;
      return b;
      
    }
  
  template <>
    Matrix<Complex>  operator*(const Complex &d,
			       const Matrix<Complex> &a)
    {
      int i, size=a.rows()*a.cols() ;
      Matrix<Complex> b(a.rows(),a.cols());
      
      Complex *bptr,*aptr ;
      bptr = b.m - 1 ;
      aptr = a.m - 1 ;
      for (i = size; i > 0 ; --i)
	*(++bptr) = d * *(++aptr) ;
      return b;
    }
  
  
  template <>
    Matrix<Complex> Matrix<Complex>::herm() const
    {
      int i, j, r = cols(), c = rows();
      Matrix<Complex> adj(r,c);
      
      for (i = 0; i < r; ++i)
        for (j = 0; j < c; ++j)
	      adj.elem(i,j) = conj(elem(j,i)) ;
  
      return adj;    
    }
  
  
#ifdef NO_IMPLICIT_TEMPLATES

// Complex instantation

  template class Matrix<Complex> ;
  
  template Matrix<Complex> operator+(const Matrix<Complex>&,const Matrix<Complex>&);
  template Matrix<Complex> operator-(const Matrix<Complex>&,const Matrix<Complex>&);
  template Matrix<Complex> operator*(const Matrix<Complex>&,const Matrix<Complex>&);
  template Vector<Complex> operator*(const Matrix<Complex>&,const Vector<Complex>&);
  template int operator==(const Matrix<Complex>&,const Matrix<Complex>&);
  // template int operator!=(const Matrix<Complex>&,const Matrix<Complex>&);
  template Matrix<Complex> comm(const Matrix<Complex>&,const Matrix<Complex>&);


#endif

}
