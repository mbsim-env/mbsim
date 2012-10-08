/*=====================================================================
        File: nurbsS_sp.cpp
     Purpose:       
    Revision: $Id: nurbsS_sp.cpp,v 1.2 2002/05/13 21:07:46 philosophil Exp $
  Created by: Philippe Lavoie          (8 May, 1998)
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
#include <nurbsS_sp.h>
//#include "nan.h"

/*!
 */
namespace PLib {

/*!
  \brief Updates the basis value for the U direction

  Updates the basis value at which a control point has 
  maximal influence. It also finds where the control point
  has maximal influence. 

  \warning The degree in U of the surface must be of 3 or less.

  \author Philippe Lavoie
  \date 8 May, 1998
*/
template <class T, int N>
void NurbsSurfaceSP<T,N>::updateMaxU() {
  if(this->degU>3){
#ifdef USE_EXCEPTION
    throw NurbsInputError();
#else
    Error error("NurbsSurfaceSP<T,N>::updateMaxU()") ;
    error << "This class doesn't support surfaces having a degree of 4 and higher.\n" ;
    error.fatal() ;
#endif
  }
  else{
    maxU.resize(this->P.rows()) ;
    maxAtU_.resize(this->P.rows()) ;
    for(int i=0;i<this->P.rows();++i){
      if(!maxInfluence(i,this->U,this->degU,maxAtU_[i]))
	cerr << "Problem in maxInfluence U!\n" ;
      maxU[i] = nurbsBasisFun(maxAtU_[i],i,this->degU,this->U) ;
    }
    
  }
}

/*!
  \brief Updates the basis value for the V direction

  Updates the basis value at which a control point has 
  maximal influence. It also finds where the control point
  has maximal influence. 

  \warning The degree in V of the surface must be of 3 or less.

  \author Philippe Lavoie
  \date 8 May, 1998
*/
template <class T, int N>
void NurbsSurfaceSP<T,N>::updateMaxV() {
  if(this->degV>3){
#ifdef USE_EXCEPTION
    throw NurbsInputError();
#else
    Error error("NurbsSurfaceSP<T,N>::updateMaxV()") ;
    error << "This class doesn't support surfaces having a degree of 4 and higher.\n" ;
    error.fatal() ;
#endif
  }
  else{
    maxV.resize(this->P.cols()) ;
    maxAtV_.resize(this->P.cols()) ;
    for(int i=0;i<this->P.cols();++i){
      if(!maxInfluence(i,this->V,this->degV,maxAtV_[i]))
	cerr << "Problem in maxInfluence V!\n" ;
      maxV[i] = nurbsBasisFun(maxAtV_[i],i,this->degV,this->V) ;
    }
    
  }
}

/*!
  \brief Generate a parallel surface

  Generates an offset surface from this surface. An offset
  surface is a surface wich has its surface parallel to
  an other one. There is a distance of \a d between the two
  parallel surfaces.
  
  The algorithm used is very naive. It generates a surface
  such that a point \a s_2(u,v) = s(u,v) + d n(u,v) where
  \a s_2(u,v,) is the point on the parallel surface at \a (u,v) ,
  \a s(u,v) is the point on the original surface at \a (u,v),
  \a d is the offset between the two and \a n(u,v) is the
  normal on the surface at the point \a (u,v).

  \param d  the distance between the surface and its offset
            the parallel surface.

  \author Philippe Lavoie
  \date 8 May, 1998
*/
template <class T, int N>
NurbsSurfaceSP<T,N> NurbsSurfaceSP<T,N>::generateParallel(T d) const {
  NurbsSurfaceSP<T,N> p(*this) ;

  Vector< Point_nD<T,N> > offset(this->P.rows()*this->P.cols()) ;
  Vector<T> ur(this->P.rows()*this->P.cols()) ;
  Vector<T> vr(this->P.rows()*this->P.cols()) ;
  Vector_INT Du(this->P.rows()*this->P.cols()) ;
  Vector_INT Dv(this->P.rows()*this->P.cols()) ;

  Du.reset(0) ;
  Dv.reset(0) ;

  int i,j,no ;

  no = 0 ;

  for(i=0;i<this->P.rows();++i)
    for(j=0;j<this->P.cols();++j){
      Point_nD<T,N> norm ;
      norm = this->normal(maxAtU_[i],maxAtV_[j]) ;
      if(norm.x() == T(0) && 
	 norm.y() == T(0) &&
	 norm.z() == T(0)){
	// normal is undefined there...
	// compute an average and find a suitable normal
	const T delta = 0.00001 ;
	// must handle the corner cases
	int ok = 0 ; 
	if(i==0 && j==0){
	  norm = this->normal(maxAtU_[i]+delta,maxAtV_[j]) ;
	  norm += this->normal(maxAtU_[i],maxAtV_[j]+delta) ;
	  norm /= T(2) ;
	  ok = 1 ;
	}
	if(i==this->P.rows()-1 && j==this->P.cols()-1){
	  norm = this->normal(maxAtU_[i]-delta,maxAtV_[j]) ;
	  norm += this->normal(maxAtU_[i],maxAtV_[j]-delta) ;
	  norm /= T(2) ;
	  ok = 1 ;
	}
	if(i==0 && j==this->P.cols()-1){
	  norm = this->normal(maxAtU_[i]-delta,maxAtV_[j]) ;
	  norm += this->normal(maxAtU_[i],maxAtV_[j]+delta) ;
	  norm /= T(2) ;
	  ok = 1 ;
	}
	if(i==this->P.rows()-1 && j==0){
	  norm = this->normal(maxAtU_[i]-delta,maxAtV_[j]) ;
	  norm += this->normal(maxAtU_[i],maxAtV_[j]+delta) ;
	  norm /= T(2) ;
	  ok = 1 ;
	}
	if(!ok){
	  T nt = 1.0 ; 
	  while(norm.x() == T(0) && 
	     norm.y() == T(0) &&
	     norm.z() == T(0)){
	    if( nt*d >(this->U[this->U.n()-1]-this->U[0])){
#ifdef USE_EXCEPTION
	      throw NurbsComputationError();
#else
	      Error error("generateParallel");
	      error << "Can't compute a normal point.\n" ;
	      error.fatal() ;
#endif
	    }
	    T u1,u2,v1,v2 ;
	    if(i==0 || i==this->P.rows()-1){
	      u1 = u2 = maxAtU_[i] ;
	      v1 = maxAtV_[j]+ nt*delta ;
	      v2 = maxAtV_[j]- nt*delta ;
	      if(v1>this->V[this->V.n()-1]) v1 = this->V[this->V.n()-1] ;
	      if(v2<this->V[0]) v2 = this->V[0] ;
	      norm = this->normal(u1,v1);
	      norm += this->normal(u2,v2) ;
	      norm /= 2 ; 
	    }
	    else{
	      u1 = maxAtU_[i]- nt*delta ;
	      u2 = maxAtU_[i]+ nt*delta ;
	      v1 = v2 = maxAtV_[j] ;
	      if(u1 < this->U[0]) u1 = this->U[0] ;
	      if(u2 > this->U[this->U.n()-1]) u2 = this->U[this->U.n()-1] ;

	      T u3,v3 ;
	      u3 = maxAtU_[i] ;
	      if(j==0)
		v3 = maxAtV_[j]+ nt*delta ;
	      else
		v3 = maxAtV_[j]- nt*delta ;

	      if(v3<this->V[0]) v3 = this->V[0] ;
	      if(v3>this->V[this->V.n()-1]) v3 = this->V[this->V.n()-1] ;

	      norm = this->normal(u1,v1);
	      norm += this->normal(u2,v2) ;
	      norm += this->normal(u3,v3) ;
	      norm /= 3 ; 
	    }
	    nt *= 10.0 ; 
	  }
	}
      }
      Point_nD<T,N> unit = norm.unitLength();
      unit *= d ;
      //HPoint_nD<T,N> offset(unit ) ;
      //offset.w() = 0.0 ; 
      //p.modSurfCPby(i,j,offset) ;
      Du[no] = i ;
      Dv[no] = j ;
      offset[no] = unit ;
      ++no ;
    }

  p.movePoint(maxAtU_,maxAtV_,offset,Du,Dv) ;

  return p ;
}

/*!
  \brief Move the surface point only

  Moves only the specified surface point. The other surface
  points normally affected by moving this point are {\em not}
  moved.
  
  The point a is in the 4D homogenous space, but only
  the x,y,z value are used. The weight is not moved by 
  this function.
  
  \param i  the row of the surface point to move
  \param j  the column of the surface point to move
  \param a  move that surface point by that amount.

  \author Philippe Lavoie
  \date 7 June, 1998
*/
template <class T, int N>
void NurbsSurfaceSP<T,N>::modOnlySurfCPby(int i, int j, const HPoint_nD<T,N>& a){

  int sizeU, sizeV ;

  sizeU = 2*this->degU+3 ; 
  if(i-this->degU-1<0) sizeU += i-this->degU-1 ; 
  if(i+this->degU+1>=this->P.rows()) sizeU -= i+this->degU+1-this->P.rows() ;

  sizeV = 2*this->degV+3 ;
  if(j-this->degV-1<0) sizeV += j-this->degV-1 ; 
  if(j+this->degV+1>=this->P.cols()) sizeV -= j+this->degV+1-this->P.cols() ;
  
  Vector<T> u(sizeU) ;
  Vector<T> v(sizeV) ;
  Vector<Point_nD<T,N> > pts(sizeU*sizeV) ; 
  Vector<int> pu(sizeU*sizeV) ;
  Vector<int> pv(sizeU*sizeV) ;

  int n=0;
  int nu = 0 ;
  int nv = 0 ; 
  for(int k=i-this->degU-1;k<=i+this->degU+1;++k){
    if(k<0)
      continue ;
    if(k>=this->P.rows())
      break ; 
    nv = 0 ;
    for(int l=j-this->degV-1;l<=j+this->degV+1;++l){
      if(l<0)
	continue ;
      if(l>=this->P.cols())
	break ; 
      if( k == i && j==l){
	pts[n].x() = a.x() ; 
	pts[n].y() = a.y() ; 
	pts[n].z() = a.z() ; 
      }
      //else
      //pts[n] = Point3D(0,0,0) ;
      pu[n] = nu ; 
      pv[n] = nv ; 
      if(k==i){
	v[nv] = maxAtV_[l] ; // only need to initialise this once
      }
      ++n ;
      ++nv ; 
    }  
    u[nu] = maxAtU_[k] ;
    ++nu ; 
  }

  u.resize(nu) ;
  v.resize(nv) ; 
  pts.resize(n) ;
  pu.resize(n) ; 
  pv.resize(n) ; 

  this->movePoint(u,v,pts,pu,pv) ;
}

} // end namespace
