/*=============================================================================
        File: surface.H
     Purpose:       
    Revision: $Id: surface.h,v 1.2 2002/05/13 21:07:46 philosophil Exp $
  Created by: Philippe Lavoie          (3 Oct, 1996)
 Modified by: 

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
#ifndef _nurbs_surface_h_
#define _nurbs_surface_h_

#include "curve.h"

#include "matrix.h"
#include "color.h"

/*!
 */
namespace PLib {

template <class T, int N>
struct SurfParam{
  T u,v ; 
  SurfParam() : u(0), v(0) {; }
  SurfParam(T a, T b) : u(a), v(b) { ; }
};

template <class T, int N>
struct InterPoint {
  Point_nD<T,N> tangent ;
  Point_nD<T,N> point ; 
  SurfParam<T,N> paramA ;
  SurfParam<T,N> paramB ; 
  InterPoint() : tangent(0), paramA(), paramB() { ; }

  InterPoint<T,N>& operator=(const InterPoint<T,N>& A) 
    { point = A.point ; paramA = A.paramA ; paramB = A.paramB ; tangent = A.tangent ; return *this;}
};

/*!
  \class ParaSurface surface.h
  \brief An abstract parametric surface class

  This is an abstract class used as a basis for NURBS and 
  HNURBS surfaces.

  \author Philippe Lavoie 
  \date 4 Oct. 1996
*/

template <class T, int N>
class ParaSurface{
public:
  ParaSurface() //!< Empty constructor
    {;}

  virtual HPoint_nD<T,N> operator()(T u, T v) const =0; //!< an abstract function
  HPoint_nD<T,N> hpointAt(T u, T v) const  //!< Calls operator()
    { return operator()(u,v); }
  Point_nD<T,N> pointAt(T u, T v) const  //!< Projects the point in the normal space
    { return project(operator()(u,v)) ; }


  virtual void deriveAtH(T u, T v, int d, Matrix< HPoint_nD<T,N> >& skl) const =0; //!< the derivative in homogenous space
  virtual void deriveAt(T u, T v, int d, Matrix< Point_nD<T,N> >& skl) const =0; //!< the derivative in normal space
  
  virtual T minDist2(const Point_nD<T,N>& p, T& guessU, T& guessV, T error=0.001,T s=0.2,int sep=9,int maxIter=10,T um=0.0, T uM=1.0, T vm=0.0, T vM=1.0) const ;
  virtual T minDist2b(const Point_nD<T,N>& p, T& guessU, T& guessV, T error=0.001,T s=0.3,int sep=5,int maxIter=10,T um=0.0, T uM=1.0, T vm=0.0, T vM=1.0) const ;
  virtual T minDist2xy(const Point_nD<T,N>& p, T& guessU, T& guessV, T error=0.01,T dU=0.0001, T s=0.3,int sepU=5, int sepV=5, int maxIter=10,T um=0.0, T uM=1.0, T vm=0.0, T vM=1.0) const ;

  int projectOn(const Point_nD<T,N>& p, T& u, T& v, int maxI=100, const T um=0.0, const T uM=1.0, const T vm=0.0, const T vM=1.0) const ;

  T extremum(int findMin, CoordinateType coord, T minDu=0.0001, int sepU=5, int sepV=5, int maxIter=10, T um=0.0, T uM=1.0, T vm=0.0, T vM=1.0) const ;
  
  int intersectWith(const ParaSurface<T,N> &S, Point_nD<T,N>& p, T& u, T& v, T& s, T& t, int maxI=100, T um=0.0, T uM=1.0, T vm=0.0, T vM=1.0) const ;
  int intersectWith(const ParaSurface<T,N> &S, InterPoint<T,N> &iter, int maxI=100, T um=0.0, T uM=1.0, T vm=0.0, T vM=1.0) const ;

  virtual int writeVRML(ostream &fout,const Color& color,int Nu,int Nv, T u_s, T u_e, T v_s, T v_e) const ;
  virtual int writeVRML(const char* filename,const Color& color,int Nu,int Nv, T u_s, T u_e, T v_s, T v_e) const ;
  virtual int writeVRML(const char* filename, const Color& color=whiteColor, int Nu=20, int Nv=20) const=0 ; //!< an abstract function
  virtual int writeVRML97(ostream &fout,const Color& color,int Nu,int Nv, T u_s, T u_e, T v_s, T v_e) const ;
  virtual int writeVRML97(const char* filename,const Color& color,int Nu,int Nv, T u_s, T u_e, T v_s, T v_e) const ;
  virtual int writeVRML97(const char* filename, const Color& color=whiteColor, int Nu=20, int Nv=20) const=0 ; //!< an abstract function
};


template <class T, int N>
void intersectSurfaces(const PLib::ParaSurface<T,N> &surfA, const PLib::ParaSurface<T,N> &surfB, BasicList<PLib::InterPoint<T,N> > &points, int maxI=100, T um=0.0, T uM=1.0, T vm=0.0, T vM=1.0);

} // end namespace


#ifdef INCLUDE_TEMPLATE_SOURCE
#include "surface.cpp"
#endif


#endif 

