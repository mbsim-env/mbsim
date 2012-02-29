/*=====================================================================
        File: nurbs_sp.cpp
     Purpose:       
    Revision: $Id: nurbs_sp.cpp,v 1.2 2002/05/13 21:07:46 philosophil Exp $
  Created by: Philippe Lavoie          (7 May, 1998)
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
#include <nurbs_sp.h>

/*!
 */
namespace PLib {

/*!
  \brief Updates the basis value

  Updates the basis value at which a control point has 
  maximal influence. It also finds where the control point
  has maximal influence. 

  \warning The degree of the curve must be of 3 or less.
  \author Philippe Lavoie 
  \date 7 May, 1998
*/
template <class T, int N>
void NurbsCurveSP<T,N>::updateMaxU() {
  if(deg_>3){
#ifdef USE_EXCEPTION
    throw NurbsInputError();
#else
    Error error("NurbsCurveSP") ; 
    error << "This class can't handle a curve of degree 4 or higher.\n" ;
    error.fatal() ;
#endif
  }
  else{
    maxU.resize(P.n()) ;
    maxAt_.resize(P.n()) ;
    for(int i=0;i<P.n();++i){
      if(!maxInfluence(i,U,deg_,maxAt_[i]))
	cerr << "Problem in maxInfluence U!\n" ;
      if(i>0)
	if(maxAt_[i]<maxAt_[i-1]){
#ifdef USE_EXCEPTION
	  throw NurbsError();
#else
	  Error error("Error updating maxU");
	  error << "HUGE ERROR!\n" ;
	  error << "Knot = " << U << endl ;
	  error << " i = " << i << endl ;
	  error << " deg = " << deg_ << endl ;
	  error.fatal() ; 
#endif
	}
      maxU[i] = basisFun(maxAt_[i],i,deg_) ;
    }
    
  }
}

/*!
  \brief Move the surface point only

  Moves only the specified surface point. The other surface
  points normally affected by moving this point are {\em not}
  moved.
  
  The point a is in the 4D homogenous space, but only
  the x,y,z value are used. The weight is not moved by 
  this function.

  \param i  the surface point to move
  \param a  move that surface point by that amount.
  
  \warning The degree of the curve must be of 3 or less.

  \author Philippe Lavoie
  \date 7 June, 1998
*/
template <class T, int N>
void NurbsCurveSP<T,N>::modOnlySurfCPby(int i, const HPoint_nD<T,N>& a){
  Vector<T> u(2*deg_+3) ;
  Vector< Point_nD<T,N> > pts(2*deg_+3) ; 

  int n=0;
  for(int j=i-deg_-1;j<=i+deg_+1;++j){
    if(j<0)
      continue ;
    if(j>=P.n())
      break ; 
    u[n] = maxAt_[j] ;
    if( j == i){
      pts[n].x() = a.x() ; 
      pts[n].y() = a.y() ; 
      pts[n].z() = a.z() ; 
    }
    //else
    //  pts[n] = Point3D(0,0,0) ; pts is alredy set to 0,0,0
    ++n ;
  }  

  u.resize(n) ;
  pts.resize(n) ; 

  movePoint(u,pts) ;
}

} // end namespace
