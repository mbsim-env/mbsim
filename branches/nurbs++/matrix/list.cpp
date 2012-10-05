/*=============================================================================
        File: list.cpp
     Purpose:
    Revision: $Id: list.cpp,v 1.2 2002/05/13 21:07:45 philosophil Exp $
  Created by: Philippe Lavoie          (28 October 1997)
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
#include "list.h"

#ifdef NO_IMPLICIT_TEMPLATES

template class BasicNode<float> ;
template class BasicNode<double> ;
template class BasicNode<int> ;
template class BasicNode<char> ;
template class BasicNode<unsigned char> ;
template class BasicNode<PLib::Point3Df> ;
template class BasicNode<PLib::HPoint3Df> ;
template class BasicNode<PLib::Point3Dd> ;
template class BasicNode<PLib::HPoint3Dd> ;
template class BasicNode<PLib::Point2Df> ;
template class BasicNode<PLib::HPoint2Df> ;
template class BasicNode<PLib::Point2Dd> ;
template class BasicNode<PLib::HPoint2Dd> ;
template class BasicNode<PLib::Coordinate> ;

template class BasicList<float> ;
template class BasicList<double> ;
template class BasicList<int> ;
template class BasicList<char> ;
template class BasicList<unsigned char> ;
template class BasicList<PLib::Point3Df> ;
template class BasicList<PLib::HPoint3Df> ;
template class BasicList<PLib::Point3Dd> ;
template class BasicList<PLib::HPoint3Dd> ;
template class BasicList<PLib::Point2Df> ;
template class BasicList<PLib::HPoint2Df> ;
template class BasicList<PLib::Point2Dd> ;
template class BasicList<PLib::HPoint2Dd> ;
template class BasicList<PLib::Coordinate> ;

#endif

