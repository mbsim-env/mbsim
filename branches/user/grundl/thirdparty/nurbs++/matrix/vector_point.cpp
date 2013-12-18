/*=============================================================================
        File: vector.cpp
     Purpose:
    Revision: $Id: vector_point.cpp,v 1.2 2002/05/13 21:07:45 philosophil Exp $
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

#ifdef NO_IMPLICIT_TEMPLATES

// Point3D instantation

template class Vector<Point3Df> ;

template Vector<Point3Df> operator+(const Vector<Point3Df>&, const Vector<Point3Df>&);
template Vector<Point3Df> operator-(const Vector<Point3Df>&, const Vector<Point3Df>&);
template Point3Df operator*(const Vector<Point3Df>&,const Vector<Point3Df>&);
template Vector<Point3Df> operator*(const Vector<Point3Df>& v, const double d);
template Vector<Point3Df> operator*(const Vector<Point3Df>& v, const Complex d);
template int operator==(const Vector<Point3Df>&,const Vector<Point3Df>&);
template int operator!=(const Vector<Point3Df>&,const Vector<Point3Df>&);

template class Vector<Point3Dd> ;

template Vector<Point3Dd> operator+(const Vector<Point3Dd>&, const Vector<Point3Dd>&);
template Vector<Point3Dd> operator-(const Vector<Point3Dd>&, const Vector<Point3Dd>&);
template Point3Dd operator*(const Vector<Point3Dd>&,const Vector<Point3Dd>&);
template Vector<Point3Dd> operator*(const Vector<Point3Dd>& v, const double d);
template Vector<Point3Dd> operator*(const Vector<Point3Dd>& v, const Complex d);
template int operator==(const Vector<Point3Dd>&,const Vector<Point3Dd>&);
template int operator!=(const Vector<Point3Dd>&,const Vector<Point3Dd>&);

// Point2D instantation

template class Vector<Point2Df> ;

template Vector<Point2Df> operator+(const Vector<Point2Df>&, const Vector<Point2Df>&);
template Vector<Point2Df> operator-(const Vector<Point2Df>&, const Vector<Point2Df>&);
template Point2Df operator*(const Vector<Point2Df>&,const Vector<Point2Df>&);
template Vector<Point2Df> operator*(const Vector<Point2Df>& v, const double d);
template Vector<Point2Df> operator*(const Vector<Point2Df>& v, const Complex d);
template int operator==(const Vector<Point2Df>&,const Vector<Point2Df>&);
template int operator!=(const Vector<Point2Df>&,const Vector<Point2Df>&);

template class Vector<Point2Dd> ;

template Vector<Point2Dd> operator+(const Vector<Point2Dd>&, const Vector<Point2Dd>&);
template Vector<Point2Dd> operator-(const Vector<Point2Dd>&, const Vector<Point2Dd>&);
template Point2Dd operator*(const Vector<Point2Dd>&,const Vector<Point2Dd>&);
template Vector<Point2Dd> operator*(const Vector<Point2Dd>& v, const double d);
template Vector<Point2Dd> operator*(const Vector<Point2Dd>& v, const Complex d);
template int operator==(const Vector<Point2Dd>&,const Vector<Point2Dd>&);
template int operator!=(const Vector<Point2Dd>&,const Vector<Point2Dd>&);



#endif

}
