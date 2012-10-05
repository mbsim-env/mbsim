/*=============================================================================
        File: curve.cpp
     Purpose:       
    Revision: $Id: curve.cpp,v 1.2 2002/05/13 21:07:45 philosophil Exp $
  Created by: Philippe Lavoie          (3 Oct, 1996)
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
#include <curve.h>

/*!
 */
namespace PLib {

/*!
  \brief Find the minimal distance between a point and the curve

  This is an iterative method to find the closest point to a curve. 

  \param p  the minimal distance from that point
  \param guess  a starting value for the parameter \a u, on exit this 
	        will be set to the value
		of the point on the curve closest to \a p.
  \param error  when iterations have an error smaller than this value, 
	        the function exits
  \param s  the size of the search in the parametric space.
  \param sep  the number of points initially looked at to find a 
	      minimal distance
  \param maxiter  the maximal number of iterations
  \param um  the minimal parametric value
  \param uM  the maximal parametric value

  \return The value of the minimal distance between \a p and the curve. 
          The variable guess now holds the parametric value of the curve 
          point closest to \a p. 

  \warning It has not been tested with closed loop curves.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
T ParaCurve<T,N>::minDist2(const Point_nD<T,N>& p, T& guess,T error,T s,int sep,int maxIter, T um, T uM) const {
  if(um<0)
    um = minKnot() ;
  if(uM<0)
    uM = maxKnot() ;
  if(s<0)
    s = uM-um ;
  T d,d1,d2 ;
  Point_nD<T,N> p2 ;
  p2 = pointAt(guess) ;
  d = norm2(p-p2) ;
  d2 = d1 = 0 ;
  int niter = 0 ;
  T u1,u2 ;
  T step ;
  step = s/(T)sep ;
  u1 = guess-s ;
  u2 = guess+s ;
  while(d>error && niter<maxIter) {
    if(u1<um)
      u1=um;
    if(u2>uM)
      u2 = uM ;
    T u = u1 ;
    d2 = d1 ;
    for(;u<u2;u+=step){
      p2 = pointAt(u) ;
      d1 = norm2(p-p2) ;
      if(d1<d){
	d = d1 ;
	guess = u ;
      }
    }
    s /= 2.0 ;
    u1 = guess - s ;
    u2 = guess + s ;
    step = 2.0*s/(T)sep ;
    if(d-d2==0) niter = maxIter ;
    if(step<error) niter = maxIter ;
    niter++;
  }
  return d ;
}


/*!
  \brief Find the closest point on the curve to the \a y coordinate

  This is an iterative method to find the closest point on the 
  curve to the y coordinate.

  \param y the \a y coordinate to be close too.
  \param guess  a starting value for the parameter \a u, on exit this 
	        will be set to the value of the point on the curve 
		closest to \a y.
  \param error  when iterations have an error smaller than this value, 
	        the function exits
  \param s  the size of the search in the parametric space.
  \param sep  the number of points initially looked at to find a minimal
	      distance
  \param maxiter  the maximal number of iterations
  \param um  the minimal parametric value
  \param uM  the maximal parametric value

  \return The value of the minimal distance between \a p and the curve. 
          The variable guess now holds
          the parametric value of the curve point closest to \a p. 

  \warning It has not been tested with closed loop curves.

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T, int N>
Point_nD<T,N> ParaCurve<T,N>::minDistY(T y, T& guessU, T error, T s, int sep, int maxIter, T um, T uM) const {
  if(um<0)
    um = minKnot() ;
  if(uM<0)
    uM = maxKnot() ;
  if(s<0)
    s = uM-um ;
  T d,d1,d2 ;
  Point_nD<T,N> p2,result ;
  p2 = pointAt(guessU) ;
  result = p2 ;
  d = to2power(y-p2.y()) ;
  d2 = d1 = 0 ;
  int niter = 0 ;
  T u1,u2 ;
  T step ;
  step = s/(T)sep ;
  u1 = guessU-s ;
  u2 = guessU+s ;
  while(d>error && niter<maxIter) {
    if(u1<um)
      u1=um;
    if(u2>uM)
      u2 = uM ;
    T u = u1 ;
    d2 = d1 ;
    for(;u<u2;u+=step){
      p2 = pointAt(u) ;
      d1 = to2power(y-p2.y()) ;
      if(d1<d){
	d = d1 ;
	guessU = u ;
	result = p2 ;
      }
    }
    s /= 2.0 ;
    u1 = guessU - s ;
    u2 = guessU + s ;
    step = 2.0*s/(T)sep ;
    if(d-d2==0) niter = maxIter ;
    if(step<error) niter = maxIter ;
    niter++;
  }
  return result ;
}


/*!
  \brief Find the closest point on the curve to the \a x coordinate

  This is an iterative method to find the closest point on the 
  curve to the \a x coordinate.

  \param x  the \a x coordinate to be close too.
  \param guess  a starting value for the parameter \a u, on exit this 
	        will be set to the value of the point on the curve 
		closest to \a x.
  \param error  when iterations have an error smaller than this value, 
	        the function exits
  \param s  the size of the search in the parametric space.
  \param sep  the number of points initially looked at to find a minimal
	      distance
  \param maxiter  the maximal number of iterations
  \param um  the minimal parametric value
  \param  uM  the maximal parametric value

  \return The value of the minimal distance between \a p and the curve. 
          The variable guess now holds
	  the parametric value of the curve point closest to \a p. 

  \warning It has not been tested with closed loop curves.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
Point_nD<T,N> ParaCurve<T,N>::minDistX(T x, T& guessU, T error, T s, int sep, int maxIter, T um, T uM) const {
  if(um<0)
    um = minKnot() ;
  if(uM<0)
    uM = maxKnot() ;
  if(s<0)
    s = uM-um ;
  T d,d1,d2 ;
  Point_nD<T,N> p2,result ;
  p2 = pointAt(guessU) ;
  result = p2 ;
  d = to2power(x-p2.x()) ;
  d2 = d1 = 0 ;
  int niter = 0 ;
  T u1,u2 ;
  T step ;
  step = s/(T)sep ;
  u1 = guessU-s ;
  u2 = guessU+s ;
  while(d>error && niter<maxIter) {
    if(u1<um)
      u1=um;
    if(u2>uM)
      u2 = uM ;
    T u = u1 ;
    d2 = d1 ;
    for(;u<u2;u+=step){
      p2 = pointAt(u) ;
      d1 = to2power(x-p2.x()) ;
      if(d1<d){
	d = d1 ;
	guessU = u ;
	result = p2 ;
      }
    }
    s /= 2.0 ;
    u1 = guessU - s ;
    u2 = guessU + s ;
    step = 2.0*s/(T)sep ;
    if(d-d2==0) niter = maxIter ;
    if(step<error) niter = maxIter ;
    niter++;
  }
  return result ;
}


/*!
  \brief Find the closest point on the curve to the \a x coordinate

  This is an iterative method to find the closest point on the 
  curve to the \a x coordinate.

  \param z  the \a x coordinate to be close too.
  \param guess  a starting value for the parameter \a u, on exit this 
	        will be set to the value of the point on the curve 
		closest to \a x.
  \param error  when iterations have an error smaller than this value, 
	        the function exits
  \param s  the size of the search in the parametric space.
  \param sep  the number of points initially looked at to find a minimal
	      distance
  \param maxiter  the maximal number of iterations
  \param um  the minimal parametric value
  \param uM  the maximal parametric value

  \return The value of the minimal distance between \a p and the curve. 
          The variable guess now holds
	  the parametric value of the curve point closest to \a p. 

  \warning It has not been tested with closed loop curves.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
Point_nD<T,N> ParaCurve<T,N>::minDistZ(T z, T& guessU, T error, T s, int sep, int maxIter, T um, T uM) const {
  if(um<0)
    um = minKnot() ;
  if(uM<0)
    uM = maxKnot() ;
  if(s<0)
    s = uM-um ;
  T d,d1,d2 ;
  Point_nD<T,N> p2,result ;
  p2 = pointAt(guessU) ;
  result = p2 ;
  d = to2power(z-p2.z()) ;
  d2 = d1 = 0 ;
  int niter = 0 ;
  T u1,u2 ;
  T step ;
  step = s/(T)sep ;
  u1 = guessU-s ;
  u2 = guessU+s ;
  while(d>error && niter<maxIter) {
    if(u1<um)
      u1=um;
    if(u2>uM)
      u2 = uM ;
    T u = u1 ;
    d2 = d1 ;
    for(;u<u2;u+=step){
      p2 = pointAt(u) ;
      d1 = to2power(z-p2.z()) ;
      if(d1<d){
	d = d1 ;
	guessU = u ;
	result = p2 ;
      }
    }
    s /= 2.0 ;
    u1 = guessU - s ;
    u2 = guessU + s ;
    step = 2.0*s/(T)sep ;
    if(d-d2==0) niter = maxIter ;
    if(step<error) niter = maxIter ;
    niter++;
  }
  return result ;
}

template <class T, int N>
inline T coordValue(CoordinateType coord, const Point_nD<T,N>& p){
  switch(coord){
  case coordX: return p.x() ; break ;
  case coordY: return p.y() ; break ;
  case coordZ: return p.z() ; break ;
  }
  return 0.0 ; // elliminates warning messages
}

/*!
  \brief Finds the minimal or maximal value on the curve of the x,y or z coordinate.

  Finds the minimal or maximal value on the curve of the x,y or 
  z coordinate.

  \param findMin  a flag indicatinf if we're looking for the minimal
	          value or the maximal value.
  \param coord  Which coordinate to find: x,y or z.
  \param  minDu  The minimal distance between iterations in the parametric
	         space.
  \param sep  the number of points initially looked at to find a minimal
	       distance
  \param maxiter  the maximal number of iterations
  \param um  the minimal parametric value
  \param uM  the maximal parametric value

  \return The minimal value of \a z along the curve

  \warning It has not been tested with closed loop curves.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
T ParaCurve<T,N>::extremum(int findMin, CoordinateType coord, T minDu, int sep, int maxIter, T um, T uM) const {
  if(um<0)
    um = minKnot() ;
  if(uM<0)
    uM = maxKnot() ;

  T c,d,d1,d2,result,guessU ;
  Point_nD<T,N> p2 ;
  p2 = pointAt(um) ;
  c = coordValue(coord,p2) ;
  p2 = pointAt(uM) ;
  if(findMin)
    c = minimum(c,coordValue(coord,p2)) ;
  else
    c = maximum(c,coordValue(coord,p2)) ;
  result = c ;
  d = minDu*10.0 ;
  d2 = d1 = 0 ;
  int niter = 0 ;
  T u1,u2 ;
  T step ;
  T s ;
  s = uM - um ;
  step = s/(T)(sep+1) ;
  u1 = um ;
  u2 = uM ;
  guessU = um ;
  while(d>minDu && niter<maxIter) {
    if(u1<um)
      u1=um;
    if(u2>uM)
      u2 = uM ;
    T u = u1 ;
    d2 = c ;
    d = guessU ;
    for(;u<=u2;u+=step){
      p2 = pointAt(u) ;
      if(findMin){
	d1 = minimum(c,coordValue(coord,p2)) ;
	if(d1<c){
	  c = d1 ;
	  guessU = u ;
	  result = d1 ;
	}
      }
      else{
	d1 = maximum(c,coordValue(coord,p2)) ;
	if(d1>c){
	  c = d1 ;
	  guessU = u ;
	  result = d1 ;
	}
      }
    }
    s /= 2.0 ;
    u1 = guessU - s ;
    u2 = guessU + s ;
    step = 2.0*s/(T)sep ;
    if((c-d2)==0.0) niter = maxIter ;
    if(step<minDu) niter = maxIter ;
    d = absolute(guessU-d) ;
    niter++;
  }
  return result ;
}

} // end namespace
