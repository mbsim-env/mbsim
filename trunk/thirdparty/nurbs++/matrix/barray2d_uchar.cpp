/*=============================================================================
        File: barray2d.cpp
     Purpose:
    Revision: $Id: barray2d_uchar.cpp,v 1.2 2002/05/13 21:07:45 philosophil Exp $
  Created by: Philippe Lavoie          (3 Oct, 1996)
 Modified by: 

 Copyright notice:
          Copyright (C) 1996-1997 Philippe Lavoie
 
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

#include "barray2d.cpp"

namespace PLib {

ostream& 
Basic2DArray<unsigned char>::print(ostream& os) const
{
  int i, j;
  const int r = rows() ;
  const int c = cols();
  
  if ( by_columns )
    for (j = 0; j < c; j++)
      {
	for (i = 0; i < r; i++) {
	  os <<  setw(width) << (int)elem(i,j) << ' ';
	}
	os << '\n';
      }  
  else
    for (i = 0; i < r; i++)
      {
	for (j = 0; j < c; j++){
	  os <<  setw(width) << (int)elem(i,j) << ' ';
	}
	os << '\n';
      }

  return os;
}



#ifdef NO_IMPLICIT_TEMPLATES

template class Basic2DArray<unsigned char> ;
template void initBasic2DArray<unsigned char>(Basic2DArray<unsigned char>&,const int,const int) ;
template void resizeKeepBasic2DArray<unsigned char>(Basic2DArray<unsigned char>&,const int,const int) ;
template istream& operator>>(istream& is, Basic2DArray<unsigned char>& ary);
template ostream& operator<<(ostream& os, const Basic2DArray<unsigned char>& ary);


#endif

}
