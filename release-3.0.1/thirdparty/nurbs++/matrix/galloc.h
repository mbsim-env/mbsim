/*=============================================================================
        File: galloc.h
     Purpose: There is a conflict between VC++6 and normal (i.e. egcs) C++
              compiler. This defines macros necessary to solve that problem.
    Revision: $Id: galloc.h,v 1.2 2002/05/13 21:07:45 philosophil Exp $
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

#ifndef _MATRIX_galloc_h_
#define _MATRIX_galloc_h_

// this file is included inside the PLib namespace

//template <class T, const int D> class HPoint_nD ;

  template <class T> 
    void resizeBasicArray(BasicArray<T>& a, int nsize) ;
  
  template <class T, const int D> 
    void resizeBasicArrayHPoint(BasicArray<HPoint_nD<T,D> >&, int) ;

  // forward declartations of specialisations:
  template<> void resizeBasicArray(BasicArray<HPoint_nD<float,2> >& a, int);
  template<> void resizeBasicArray(BasicArray<HPoint_nD<double,2> >& a, int);
  template<> void resizeBasicArray(BasicArray<HPoint_nD<float,3> >& a, int );
  template<> void resizeBasicArray(BasicArray<HPoint_nD<double,3> >& a, int);
  


#ifdef HAVE_ISO_FRIEND_DECL
 
#define FRIEND_ARRAY_ALLOCATOR \
  friend void resizeBasicArray<>(BasicArray<T>&, int) ; \
  friend void resizeBasicArrayHPoint<>(BasicArray<HPoint_nD<float,2> >&,int); \
  friend void resizeBasicArrayHPoint<>(BasicArray<HPoint_nD<double,2> >&,int);\
  friend void resizeBasicArrayHPoint<>(BasicArray<HPoint_nD<float,3> >&,int); \
  friend void resizeBasicArrayHPoint<>(BasicArray<HPoint_nD<double,3> >&,int); 

#else

  // explicit instantiation
  template void resizeBasicArrayHPoint(BasicArray<HPoint_nD<float,2> >&,int);
  template void resizeBasicArrayHPoint(BasicArray<HPoint_nD<double,2> >&,int);
  template void resizeBasicArrayHPoint(BasicArray<HPoint_nD<float,3> >&,int);
  template void resizeBasicArrayHPoint(BasicArray<HPoint_nD<double,3> >&,int);


#define FRIEND_ARRAY_ALLOCATOR \
  friend void resizeBasicArray(BasicArray<T>&, int) ; \
  friend void resizeBasicArrayHPoint(BasicArray<HPoint_nD<float,2> >&,int); \
  friend void resizeBasicArrayHPoint(BasicArray<HPoint_nD<double,2> >&,int);\
  friend void resizeBasicArrayHPoint(BasicArray<HPoint_nD<float,3> >&,int); \
  friend void resizeBasicArrayHPoint(BasicArray<HPoint_nD<double,3> >&,int); 

#endif


#endif
