/*=====================================================================
        File: nurbs.cpp
     Purpose:       
    Revision: $Id: tri_spline.h,v 1.2 2002/05/13 21:07:46 philosophil Exp $
      Author: Philippe Lavoie          (3 Oct, 1996)
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
=====================================================================*/
#ifndef _TRIANGULAR_BSPLINE_H
#define _TRIANGULAR_BSPLINE_H

#include "nurbs_global.h"

#include "point_nd.h"
#include "hpoint_nd.h"
#include "barray.h"

#include "nurbsS.h"

/*!
 */
namespace PLib {

  template <class T, int D>
    class TriangularBSpline  {
    private:
      BasicArray<Point_nD<T,D> > cp;
      int deg ;
    public:
      TriangularBSpline(int degree);
      Point_nD<T,D>& b(int i, int j, int);
      Point_nD<T,D> b(int, int ,int) const ; 
      Point_nD<T,D> operator()(T u, T v) const;

    };
  
  template <class T, int D>
    class RTriangularBSpline {
    private:
      BasicArray<HPoint_nD<T,D> > cp;
      int deg ; 
    public:
      RTriangularBSpline(int degree);
      HPoint_nD<T,D>& b(int, int, int);
      HPoint_nD<T,D> b(int, int ,int) const ; 
      HPoint_nD<T,D> operator()(T u, T v) const;

      int writeVRML(const char* filename, const Color& color=whiteColor, int Nu=20, int Nv=20, int Nw=20) const;
      int writeVRML(ostream& fout, const Color& color=whiteColor, int Nu=20, int Nv=20, int Nw=20) const;

      void setDegree(int d) ;
    };

  template < class T, int D> void convert(const NurbsSurface<T,D>& surf, RTriangularBSpline<T,D> &t1, RTriangularBSpline<T,D> &t2) ;

}


#endif
