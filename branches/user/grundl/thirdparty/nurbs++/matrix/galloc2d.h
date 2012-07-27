/*=============================================================================
        File: galloc.h
     Purpose: There is a conflict between VC++6 and normal (i.e. egcs) C++
              compiler. This defines macros necessary to solve that problem.
    Revision: $Id: galloc2d.h,v 1.2 2002/05/13 21:07:45 philosophil Exp $
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

#ifndef _MATRIX_galloc2d_h_
#define _MATRIX_galloc2d_h_

// This file is included inside the PLib namespace by barray2d.h


//template <class T, const int D> class HPoint_nD ;

  template<class T>
  void initBasic2DArray(Basic2DArray<T>& ary, const int nr, const int nc);

  template<class T>
  void resizeKeepBasic2DArray(Basic2DArray<T>& ary, const int nr, const int nc);

  template <class T, const int D> 
  void initBasic2DArrayHPoint(Basic2DArray<HPoint_nD<T,D> >& a, const int r,const int c) ;

  template <class T, const int D> 
  void resizeKeepBasic2DArrayHPoint(Basic2DArray<HPoint_nD<T,D> >& a, const int r,const int c);

 
  // forward declerations
  template<> void initBasic2DArray(Basic2DArray<HPoint_nD<float,3> >& ary, const int nr, const int nc);	  
  template<> void initBasic2DArray(Basic2DArray<HPoint_nD<double,3> >& ary, const int nr, const int nc);	  
  template<> void initBasic2DArray(Basic2DArray<HPoint_nD<float,2> >& ary, const int nr, const int nc);	  
  template<> void initBasic2DArray(Basic2DArray<HPoint_nD<double,2> >& ary, const int nr, const int nc);


  template<> void resizeKeepBasic2DArray(Basic2DArray<HPoint_nD<float,3> >& ary, const int nr, const int nc);
  template<> void resizeKeepBasic2DArray(Basic2DArray<HPoint_nD<double,3> >& ary, const int nr, const int nc);
  template<> void resizeKeepBasic2DArray(Basic2DArray<HPoint_nD<float,2> >& ary, const int nr, const int nc);
  template<> void resizeKeepBasic2DArray(Basic2DArray<HPoint_nD<double,2> >& ary, const int nr, const int nc);



#ifdef HAVE_ISO_FRIEND_DECL

#define FRIEND_2DARRAY_ALLOCATOR \
  friend void initBasic2DArray<>(Basic2DArray<T>&, const int, const int); \
  friend void resizeKeepBasic2DArray<>(Basic2DArray<T>&, const int, const int); \
  friend void initBasic2DArrayHPoint<>(Basic2DArray<HPoint_nD<float,3> >&,const int,const int); \
  friend void initBasic2DArrayHPoint<>(Basic2DArray<HPoint_nD<double,3> >&, const int r,const int c); \
  friend void initBasic2DArrayHPoint<>(Basic2DArray<HPoint_nD<float,2> >&, const int r,const int c); \
  friend void initBasic2DArrayHPoint<>(Basic2DArray<HPoint_nD<double,2> >&, const int r,const int c); \
  friend void resizeKeepBasic2DArrayHPoint<>(Basic2DArray<HPoint_nD<float,3> >&,const int,const int); \
  friend void resizeKeepBasic2DArrayHPoint<>(Basic2DArray<HPoint_nD<double,3> >&,const int,const int); \
  friend void resizeKeepBasic2DArrayHPoint<>(Basic2DArray<HPoint_nD<float,2> >&, const int,const int); \
  friend void resizeKeepBasic2DArrayHPoint<>(Basic2DArray<HPoint_nD<double,2> >&, const int,const int); \

#else



  template void initBasic2DArrayHPoint(Basic2DArray<HPoint_nD<float,3> >& a, const int r,const int c);
  template void initBasic2DArrayHPoint(Basic2DArray<HPoint_nD<double,3> >& a,const int r,const int c);
  template void initBasic2DArrayHPoint(Basic2DArray<HPoint_nD<float,2> >& a, const int r,const int c);
  template void initBasic2DArrayHPoint(Basic2DArray<HPoint_nD<double,2> >& a,const int r,const int c);
		
  template void resizeKeepBasic2DArrayHPoint(Basic2DArray<HPoint_nD<float,3> >& a, const int r,const int c);
  template void resizeKeepBasic2DArrayHPoint(Basic2DArray<HPoint_nD<double,3> >& a, const int r,const int c);
  template void resizeKeepBasic2DArrayHPoint(Basic2DArray<HPoint_nD<float,2> >& a, const int r,const int c);
  template void resizeKeepBasic2DArrayHPoint(Basic2DArray<HPoint_nD<double,2> >& a, const int r,const int c);



#define FRIEND_2DARRAY_ALLOCATOR \
  friend void initBasic2DArray(Basic2DArray<T>&, const int, const int); \
  friend void resizeKeepBasic2DArray(Basic2DArray<T>&, const int, const int); \
  friend void initBasic2DArrayHPoint(Basic2DArray<HPoint_nD<float,3> >&,const int,const int); \
  friend void initBasic2DArrayHPoint(Basic2DArray<HPoint_nD<double,3> >&, const int r,const int c); \
  friend void initBasic2DArrayHPoint(Basic2DArray<HPoint_nD<float,2> >&, const int r,const int c); \
  friend void initBasic2DArrayHPoint(Basic2DArray<HPoint_nD<double,2> >&, const int r,const int c); \
  friend void resizeKeepBasic2DArrayHPoint(Basic2DArray<HPoint_nD<float,3> >&,const int,const int); \
  friend void resizeKeepBasic2DArrayHPoint(Basic2DArray<HPoint_nD<double,3> >&,const int,const int); \
  friend void resizeKeepBasic2DArrayHPoint(Basic2DArray<HPoint_nD<float,2> >&, const int,const int); \
  friend void resizeKeepBasic2DArrayHPoint(Basic2DArray<HPoint_nD<double,2> >&, const int,const int); \


#endif
  

#endif
