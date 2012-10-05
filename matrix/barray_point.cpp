/*=============================================================================
        File: barray_point.cpp
     Purpose:       
    Revision: $Id: barray_point.cpp,v 1.2 2002/05/13 21:07:45 philosophil Exp $
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



#include "barray.cpp"

namespace PLib {

#ifdef NO_IMPLICIT_TEMPLATES

  template class BasicArray<Point3Df> ;
  template void resizeBasicArray<Point3Df>(BasicArray<Point3Df>&,int) ;
  template int operator!=(const BasicArray<Point3Df>&,const BasicArray<Point3Df>&); 
  template int operator==(const BasicArray<Point3Df>&,const BasicArray<Point3Df>&); 
  template istream& operator>>(istream& is, BasicArray<Point3Df>& ary);
  template ostream& operator<<(ostream& os, const BasicArray<Point3Df>& ary);

  template class BasicArray<Point2Df> ;
  template void resizeBasicArray<Point2Df>(BasicArray<Point2Df>&,int) ;
  template int operator!=(const BasicArray<Point2Df>&,const BasicArray<Point2Df>&); 
  template int operator==(const BasicArray<Point2Df>&,const BasicArray<Point2Df>&); 
  template istream& operator>>(istream& is, BasicArray<Point2Df>& ary);
  template ostream& operator<<(ostream& os, const BasicArray<Point2Df>& ary);

  template class BasicArray<Point3Dd> ;
  template void resizeBasicArray<Point3Dd>(BasicArray<Point3Dd>&,int) ;
  template int operator!=(const BasicArray<Point3Dd>&,const BasicArray<Point3Dd>&); 
  template int operator==(const BasicArray<Point3Dd>&,const BasicArray<Point3Dd>&); 
  template istream& operator>>(istream& is, BasicArray<Point3Dd>& ary);
  template ostream& operator<<(ostream& os, const BasicArray<Point3Dd>& ary);

  template class BasicArray<Point2Dd> ;
  template void resizeBasicArray<Point2Dd>(BasicArray<Point2Dd>&,int) ;
  template int operator!=(const BasicArray<Point2Dd>&,const BasicArray<Point2Dd>&); 
  template int operator==(const BasicArray<Point2Dd>&,const BasicArray<Point2Dd>&); 
  template istream& operator>>(istream& is, BasicArray<Point2Dd>& ary);
  template ostream& operator<<(ostream& os, const BasicArray<Point2Dd>& ary);

#endif

}
