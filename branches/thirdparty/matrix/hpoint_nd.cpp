/*=============================================================================
        File: hpoint_nd.cpp
     Purpose:
    Revision: $Id: hpoint_nd.cpp,v 1.2 2002/05/13 21:07:45 philosophil Exp $
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

#ifndef HPOINT_SOURCES
#define HPOINT_SOURCES

#include "hpoint_nd.h"

namespace PLib {


  float HPoint_nD<float,2>::dumbVar ;
  double HPoint_nD<double,2>::dumbVar ;

  /*
#ifdef NO_IMPLICIT_TEMPLATES 

  template class HPoint_nD<float,3> ;
  template class HPoint_nD<double,3> ;
  template class HPoint_nD<float,2> ;
  template class HPoint_nD<double,2> ;

#endif
  */

} // end library

#endif
