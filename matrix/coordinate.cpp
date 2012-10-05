/*=============================================================================
        File: coordinate.cpp
     Purpose:
    Revision: $Id: coordinate.cpp,v 1.2 2002/05/13 21:07:45 philosophil Exp $
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

#include "coordinate.h"

namespace PLib {

#ifdef NO_IMPLICIT_TEMPLATES

  template Coordinate maximum(Coordinate, Coordinate) ;
  template Coordinate maximumRef(const Coordinate&, const Coordinate&) ;
  template Coordinate minimum(Coordinate, Coordinate) ;
  template Coordinate minimumRef(const Coordinate&, const Coordinate&) ;

#endif
} // end namespace
