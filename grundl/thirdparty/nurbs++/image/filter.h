/*=============================================================================
        File: rec_filter.h
     Purpose:       
    Revision: $Id: filter.h,v 1.2 2002/05/13 21:07:45 philosophil Exp $
  Created by: Philippe Lavoie          (18 February 1999)
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


#ifndef _Matrix_Filters_h_
#define _Matrix_Filters_h_
#include "barray2d.h"

/*!
 */
namespace PLib {
  /*!
   */
  namespace Filter {
    //void mean(const Basic2DArray<T>& a, Basic2DArray<T>& b);
    template <class T>
      void median(const Basic2DArray<T>& a, Basic2DArray<T>& b);
    template <class T>
      void medianT(const Basic2DArray<T>& a, Basic2DArray<T>& b, T value, int op);
  }
}

#endif
