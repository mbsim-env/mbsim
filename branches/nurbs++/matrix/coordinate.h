/*============================================================================
        File: coordinate.h
     Purpose: 
    Revision: $Id: coordinate.h,v 1.2 2002/05/13 21:07:45 philosophil Exp $
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
#ifndef _Matrix_coordinate_h_
#define _Matrix_coordinate_h_

#include "matrix_global.h"
#include "matrixTool.h"


namespace PLib {


  /*!
    \class Coordinate coordinate.h matrix/coordinate.h
    \brief A class for a matrix coordinate point
     
    \author Philippe Lavoie 
    \date 4 October 1996
  */
  class Coordinate {
  public:
    int i,j ;
    Coordinate() 
      { i=0;j=0;} 
    Coordinate(int I, int J) 
      { i=I ; j=J ; }
    Coordinate(int a) 
      { i=j=a; }

    const Coordinate& operator=(const Coordinate& c)
      {i=c.i; j=c.j; return *this ;}
    friend int operator==(const Coordinate& a, const Coordinate& b)
      { return ( a.i == b.i && a.j == b.j ); }

    friend float distance2(const Coordinate& a){ return float(a.i*a.i) + float(a.j*a.j) ; }
    friend float distance(const Coordinate& a) { return sqrt(distance(a)) ; }

    friend Coordinate operator-(const Coordinate& a, const Coordinate& b){ Coordinate m ; m.i = a.i-b.i ; m.j = a.j-b.j ; return m ; }
    friend Coordinate operator+(const Coordinate& a, const Coordinate& b){ Coordinate m ; m.i = a.i+b.i ; m.j = a.j+b.j ; return m ; }

    friend ostream& operator<<(ostream& os, const Coordinate& point);
    friend istream& operator>>(istream& os, Coordinate& point);
  };

  inline int operator<(const Coordinate& a, const Coordinate& b){ 
    return (a.i<b.i && a.j<b.j ) ; } // the smaller than operator
  inline int operator>(const Coordinate& a, const Coordinate& b){ 
    return (a.i>b.i && a.j>b.j ) ; } // the greater than operator
  inline int operator<=(const Coordinate& a, const Coordinate& b){ 
    return (a.i<=b.i && a.j<=b.j ) ; } // the smaller or equal operator
  inline int operator>=(const Coordinate& a, const Coordinate& b){ 
    return (a.i>=b.i && a.j>=b.j ) ; } // the greater or equal operator


  /*!
    \fn ostream& operator<<(ostream& os,const Coordinate& c)
    \brief The output operator of a coordinate to an ostream

    The output operator of a coordinate to an ostream

    \param os  the ostream
    \param c  the coordinate to output
    \return  the ostream with the coordinate $c$.

    \author Philippe Lavoie 
    \date 24 January 1997
  */
  inline ostream& operator<<(ostream& os,const Coordinate& c)
  {
    os << c.i << " " << c.j << " " ;
    return os;	
  }


  /*!
     \fn istream& operator>>(istream& os, Coordinate& c)
     \brief the input operator of a coordinate from an istream

     Initialize a coordinate from an istream

     \param os  the input stream
     \param c  the coordinate to initialize
     \return the istream without the coordinate
     \author Philippe Lavoie 
     \date 24 January 1997
  */
  inline istream& operator>>(istream& os, Coordinate& c){
    os >> c.i >> c.j ;
    return os ;
  }


  inline Coordinate minimum(Coordinate a, Coordinate b){
    Coordinate m ;
    m.i = minimum(a.i,b.i) ;
    m.j = minimum(a.j,b.j) ;
    return m ;
  }

  inline Coordinate maximum(Coordinate a, Coordinate b){
    Coordinate m ;
    m.i = maximum(a.i,b.i) ;
    m.j = maximum(a.j,b.j) ;
    return m ;
  }


  template <class T>
  inline Coordinate minimumByRef(const Coordinate &a, const Coordinate &b){
    Coordinate m ;
    m.i = minimum(a.i,b.i) ;
    m.j = minimum(a.j,b.j) ;
    return m ;
  }


  inline Coordinate maximumByRef(const Coordinate &a, const Coordinate &b){
    Coordinate m ;
    m.i = maximum(a.i,b.i) ;
    m.j = maximum(a.j,b.j) ;
    return m ;
  }


} // end namespace

#endif
