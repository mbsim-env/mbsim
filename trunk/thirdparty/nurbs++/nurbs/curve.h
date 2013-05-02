/*=============================================================================
        File:              curve.h
     Purpose:       
    Revision:       $Id: curve.h,v 1.2 2002/05/13 21:07:45 philosophil Exp $
  Created by:    Philippe Lavoie          (3 Oct, 1996)
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
#ifndef _nurbs_curve_h_
#define _nurbs_curve_h_

#include "nurbs_global.h"

#include "vector.h"

/*!
 */
namespace PLib {
  template <class T, int N> class ParaCurve ;

/*!
 \brief An abstract parametric curve class

  This is an abstract class used as a basis for NURBS and HNURBS curves.
  \author Philippe Lavoie
  \date 4 Oct. 1996
*/
template <class T, int N>
class ParaCurve{
public:
  ParaCurve() //!< Empty constructor
    {;} 

  virtual HPoint_nD<T,N> operator()(T u) const =0; //!< abstract function
  //! Wrapper to the operator() function
  HPoint_nD<T,N> hpointAt(T u) const { return operator()(u); }

  //! Projects the homogenouse point at \a u into normal space
  Point_nD<T,N> pointAt(T u) const { return project(operator()(u)) ; }

  virtual HPoint_nD<T,N> hpointAt(T u, int span) const =0 ; //!< abstract function

  //! Projects the homogenouse point at \a u into normal space
  Point_nD<T,N> pointAt(T u, int span) { return project(hpointAt(u,span)) ; }

  virtual void deriveAtH(T u, int, Vector< HPoint_nD<T,N> >&) const =0; //!< abstract function
  virtual void deriveAt(T u, int, Vector< Point_nD<T,N> >&) const =0; //!< abstract function

  virtual T minKnot() const = 0 ; //!< abstract function
  virtual T maxKnot() const = 0 ; //!< abstract function

  virtual T minDist2(const Point_nD<T,N>& p, T& guess,T error=0.0001,T s=0.2,int sep=9,int maxIter=10,T um=-1, T uM=-1) const ;
  virtual Point_nD<T,N> minDistY(T y, T& guessU, T error=0.0001, T s=-1, int sep=9, int maxIter=10, T um=-1, T uM=-1) const ;
  virtual Point_nD<T,N> minDistX(T y, T& guessU, T error=0.0001, T s=-1, int sep=9, int maxIter=10, T um=-1, T uM=-1) const ;
  virtual Point_nD<T,N> minDistZ(T y, T& guessU, T error=0.0001, T s=-1, int sep=9, int maxIter=10, T um=-1, T uM=-1) const ;

  virtual T extremum(int findMin, CoordinateType coord, T minDu=0.0001, int sep=9, int maxIter=10, T um=-1, T uM=-1) const ;

};

template <class T, int N>
inline void CderH(T u,const ParaCurve<T,N>& c,int d, Vector< HPoint_nD<T,N> >& ders) { c.deriveAtH(u,d,ders);}

template <class T, int N>
inline void Cder(T u,const ParaCurve<T,N>& c,int d, Vector< Point_nD<T,N> >& ders) { c.deriveAt(u,d,ders);}

} // end namespace


#ifdef INCLUDE_TEMPLATE_SOURCE
#include "curve.cpp"
#endif

#endif 

