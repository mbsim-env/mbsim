/*=============================================================================
        File: color.cpp
     Purpose:
    Revision: $Id: color.cpp,v 1.2 2002/05/13 21:07:45 philosophil Exp $
  Created by: Philippe Lavoie          (26 January, 1999)
 Modified by: Martin Schuerch

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

#ifndef Color_SOURCES
#define Color_SOURCES

#include "color.h"

#include "barray.cpp"
#include "barray2d.cpp"
#include "matrix.cpp"
#include "vector.cpp"

/*!
 */
namespace PLib {

  /*
  Color whiteColor(255,255,255);
  Color redColor(255,0,0) ;
  Color blueColor(0,0,255) ;
  Color greenColor(0,255,0) ;
  Color yellowColor(255,255,0) ;
  Color cyanColor(0,255,255) ;
  Color magentaColor(255,0,255);
  Color gray80Color(204,204,204) ;
  Color gray50Color(127,127,127) ;
  Color blackColor(0,0,0) ;
  */

  double
    Matrix<Color>::norm(void) {
#ifdef USE_EXCEPTION
    throw MatrixErr();
#else
    Error error("Matrix<Color>::norm()") ;
    error << "ERROR: you can't get the norm of a Color matrix\n" ;
    error.fatal() ;
#endif
    return 0 ;
  }

#ifndef USING_VCC
  int Matrix<Color>::read(char* filename,int r, int c) {
    ifstream fin(filename) ;
    if(!fin) {
      resize(1,1) ;
      return 0 ;
    }
    resize(r,c) ;
    Matrix<unsigned char> T(r,c) ;
    unsigned char* p = T[0] ;
    if(!fin.read((char*)p,sizeof(unsigned char)*r*c)) return 0 ;
    int i,j ;
    for(i=0;i<r ;++i)
      for(j=0;j<c ;++j)
	elem(i,j).r = T(i,j) ;
    if(!fin.read((char*)p,sizeof(unsigned char)*r*c)) return 0 ;
    for(i=0;i<r ;i++)
      for(j=0;j<c ;++j)
	elem(i,j).g = T(i,j) ;
    if(!fin.read((char*)p,sizeof(unsigned char)*r*c)) return 0 ;
    for(i=0;i<r ;i++)
      for(j=0;j<c ;++j)
	elem(i,j).b = T(i,j) ;
    return 1 ;
  }
#endif

  int Vector<Color>::minIndex() const {
#ifdef USE_EXCEPTION
    throw MatrixErr() ;
#else
    Error error("Vector<color>::minIndex") ;
    error << "ERROR: you can't get the minIndex of a vector of Colors!\n" ;
    error.fatal() ;
#endif
    return 0 ;
  }
 



#ifdef  NO_IMPLICIT_TEMPLATES

  // from barray.cpp
  template class BasicArray<Color>;
  template void resizeBasicArray<Color>(BasicArray<Color>&,int) ;
  template int operator==(const BasicArray<Color>&,const BasicArray<Color>&);
  template istream& operator>>(istream& is, BasicArray<Color>& ary);
  template ostream& operator<<(ostream& os, const BasicArray<Color>& ary);

  // from barray2d.cpp
  template class Basic2DArray<Color> ;
  template void initBasic2DArray<Color>(Basic2DArray<Color>&,const int,const int) ;
  template void resizeKeepBasic2DArray<Color>(Basic2DArray<Color>&,const int,const int) ;
  template istream& operator>>(istream& is, Basic2DArray<Color>& ary);
  template ostream& operator<<(ostream& os, const Basic2DArray<Color>& ary);

  // Matrix instantiation
  template class Matrix<Color> ;

  template Matrix<Color> operator+(const Matrix<Color>&,const Matrix<Color>&);
  template Matrix<Color> operator-(const Matrix<Color>&,const Matrix<Color>&);
  template Matrix<Color> operator*(const Matrix<Color>&,const Matrix<Color>&);
  template Matrix<Color> operator*(const double,const Matrix<Color>&);
  template int operator==(const Matrix<Color>&,const Matrix<Color>&);
  // template int operator!=(const Matrix<Color>&,const Matrix<Color>&);
  template Matrix<Color> comm(const Matrix<Color>&,const Matrix<Color>&);


 // Vector instantiation

  template class Vector<Color> ;
  
  template Vector<Color> operator+(const Vector<Color>&, const Vector<Color>&);
  template Vector<Color> operator-(const Vector<Color>&, const Vector<Color>&);
  template Color operator*(const Vector<Color>&,const Vector<Color>&);
  template Vector<Color> operator*(const Vector<Color>& v, const double d);
  template Vector<Color> operator*(const Vector<Color>& v, const Complex d);
  template int operator==(const Vector<Color>&,const Vector<Color>&);
  template int operator!=(const Vector<Color>&,const Vector<Color>&);
  template void swap(Color&,Color&) ;

#endif

} // end namespace

#ifdef  NO_IMPLICIT_TEMPLATES

  // add the following for g++-v3 
  //template std::basic_ostream<char, std::char_traits<char> >& std::operator<< <char, std::char_traits<char> >(std::basic_ostream<char, std::char_traits<char> >&, std::_Setw);
#endif

#endif


