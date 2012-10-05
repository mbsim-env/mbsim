/*=============================================================================
        File: filter.cpp
     Purpose:
    Revision: $Id: filter_.cpp,v 1.2 2002/05/13 21:07:45 philosophil Exp $
  Created by: Philippe Lavoie          (18 February 1999)
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

#include "filter.cpp"

namespace PLib {

  namespace Filter{

#ifdef NO_IMPLICIT_TEMPLATES
    template void median(const Basic2DArray<int>& a, Basic2DArray<int>& b);
    template void median(const Basic2DArray<float>& a, Basic2DArray<float>& b);
    template void median(const Basic2DArray<double>& a, Basic2DArray<double>& b);
    template void median(const Basic2DArray<char>& a, Basic2DArray<char>& b);
    template void median(const Basic2DArray<unsigned char>& a, Basic2DArray<unsigned char>& b);

    template void medianT(const Basic2DArray<int>& a, Basic2DArray<int>& b, int, int);
    template void medianT(const Basic2DArray<float>& a, Basic2DArray<float>& b, float, int);
    template void medianT(const Basic2DArray<double>& a, Basic2DArray<double>& b, double, int);
    template void medianT(const Basic2DArray<char>& a, Basic2DArray<char>& b, char, int);
    template void medianT(const Basic2DArray<unsigned char>& a, Basic2DArray<unsigned char>& b, unsigned char, int);

#endif

  }
}
