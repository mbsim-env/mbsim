/*=============================================================================
        File: nurbs_sp.h
     Purpose:       
    Revision: $Id: nurbs_sp.h,v 1.3 2002/05/17 18:24:21 philosophil Exp $
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
=============================================================================*/
#ifndef _nurbs_nurbs_sp_h_
#define _nurbs_nurbs_sp_h_

#include "nurbs.h"

/*!
 */
namespace PLib {

/*!
  \class NurbsCurveSP nurbs_sp.h
  \brief A NURBS curve with surface point

  A Nurbs curve with surface point manipulators. This allows 
  someone to modify the point on a curve for which a ControlPoint
  has maximal influence over it. This might provide a more
  intuitive method to modify a curve. 

  \author Philippe Lavoie
  \date 7 May, 1998
*/
template <class T, int N>
class NurbsCurveSP : public NurbsCurve<T,N>{
public:
  NurbsCurveSP() : NurbsCurve<T,N>() { ; }
  NurbsCurveSP(const NurbsCurve<T,N>& nurb) ;
  NurbsCurveSP(const NurbsCurveSP<T,N>& nurb) ;
  NurbsCurveSP(const Vector< HPoint_nD<T,N> >& P1, const Vector<T> &U1, int degree=3) ;
  NurbsCurveSP(const Vector< Point_nD<T,N> >& P1, const Vector<T> &W, const Vector<T> &U1, int degree=3) ;


  virtual void reset(const Vector< HPoint_nD<T,N> >& P1, const Vector<T> &U1, int degree);
  virtual NurbsCurve<T,N>& operator=(const NurbsCurve<T,N>& a);
  NurbsCurveSP<T,N>& operator=(const NurbsCurveSP<T,N>& a);

  virtual void modKnot(const Vector<T>& knot) ;

  virtual void removeKnot(int r, int s, int num) ;
  virtual void removeKnotsBound(const Vector<T>& ub, Vector<T>& ek, T E);

  virtual void refineKnotVector(const Vector<T>& X) ;
  virtual void mergeKnotVector(const Vector<T> &Um) ;
  virtual void knotInsertion(T u, int r,NurbsCurveSP<T,N>& nc);


  virtual void degreeElevate(int t);

  int read(ifstream &fin) ;

  void modSurfCPby(int i, const HPoint_nD<T,N>& a) 
    { this->P[i] +=  a / maxU[i] ;  }
  void modSurfCP(int i, const HPoint_nD<T,N>& a) 
    { modSurfCPby(i,a-surfP(i)) ;  }

  void modOnlySurfCPby(int i, const HPoint_nD<T,N>& a) ;
  void modOnlySurfCP(int i, const HPoint_nD<T,N>& a)
    { modOnlySurfCPby(i,a-surfP(i)) ; }

  T maxAt(int i) const 
    { return maxAt_[i] ; }

  HPoint_nD<T,N> surfP(int i) const 
    { return this->hpointAt(maxAt_[i]) ; }

  void updateMaxU() ;

  int okMax() 
    { return (maxU.n()<=1)?0:1 ; }

protected:

  Vector<T> maxU ;
  Vector<T> maxAt_ ;
};


template <class T, int N>
inline NurbsCurveSP<T,N>::NurbsCurveSP(const NurbsCurve<T,N>& nurb) : 
  NurbsCurve<T,N>(nurb) 
{
  updateMaxU(); 
}

template <class T, int N>
inline NurbsCurveSP<T,N>::NurbsCurveSP(const NurbsCurveSP<T,N>& nurb) : 
  NurbsCurve<T,N>(nurb) 
{ 
  maxU = nurb.maxU ;  
  maxAt_ = nurb.maxAt_ ; 
}

template <class T, int N>
inline NurbsCurveSP<T,N>::NurbsCurveSP(const Vector< HPoint_nD<T,N> >& P1, const Vector<T> &U1, int degree) : 
  NurbsCurve<T,N>(P1,U1,degree) 
{ 
  updateMaxU(); 
}

template <class T, int N>
inline NurbsCurveSP<T,N>::NurbsCurveSP(const Vector< Point_nD<T,N> >& P1, const Vector<T> &W, const Vector<T> &U1, int degree) : 
  NurbsCurve<T,N>(P1,W,U1,degree) 
{ 
  updateMaxU(); 
}


template <class T, int N>
inline void NurbsCurveSP<T,N>::reset(const Vector< HPoint_nD<T,N> >& P1, const Vector<T> &U1, int degree) { 
  NurbsCurve<T,N>::reset(P1,U1,degree); 
  updateMaxU() ; 
}

template <class T, int N>
inline NurbsCurve<T,N>& NurbsCurveSP<T,N>::operator=(const NurbsCurve<T,N>& a) { 
  NurbsCurve<T,N>::operator=(a); 
  updateMaxU() ; 
  return *this; 
}

template <class T, int N>
inline NurbsCurveSP<T,N>& NurbsCurveSP<T,N>::operator=(const NurbsCurveSP<T,N>& a) { 
  NurbsCurve<T,N>::operator=(a); 
  maxU = a.maxU ; 
  maxAt_ = a.maxAt_ ; 
  return *this; 
}

template <class T, int N>
inline void NurbsCurveSP<T,N>::modKnot(const Vector<T>& knot) { 
  NurbsCurve<T,N>::modKnot(knot) ; 
  updateMaxU() ; 
}

template <class T, int N>
inline void NurbsCurveSP<T,N>::removeKnot(int r, int s, int num) { 
  NurbsCurve<T,N>::removeKnot(r,s,num); 
  updateMaxU() ; 
}

template <class T, int N>
inline void NurbsCurveSP<T,N>::removeKnotsBound(const Vector<T>& ub, Vector<T>& ek, T E) { 
  NurbsCurve<T,N>::removeKnotsBound(ub,ek,E); 
  updateMaxU() ; 
}

template <class T, int N>
inline void NurbsCurveSP<T,N>::refineKnotVector(const Vector<T>& X) {
  NurbsCurve<T,N>::refineKnotVector(X); 
  updateMaxU() ; 
}

template <class T, int N>
inline void NurbsCurveSP<T,N>::mergeKnotVector(const Vector<T> &Um) { 
  NurbsCurve<T,N>::mergeKnotVector(Um); 
  updateMaxU() ; 
}

template <class T, int N>
inline void NurbsCurveSP<T,N>::knotInsertion(T u, int r,NurbsCurveSP<T,N>& nc){ 
  NurbsCurve<T,N>::knotInsertion(u,r,nc) ; 
  nc.updateMaxU() ; 
}

template <class T, int N>
inline int NurbsCurveSP<T,N>::read(ifstream &fin) {
  int r = NurbsCurve<T,N>::read(fin) ;
  updateMaxU() ; 
  return r ;
}

template <class T, int N>
inline void NurbsCurveSP<T,N>::degreeElevate(int t){
  NurbsCurve<T,N>::degreeElevate(t) ;
  updateMaxU() ; 
}


typedef NurbsCurveSP<float,3> NurbsCurveSPf ;
typedef NurbsCurveSP<double,3> NurbsCurveSPd ;
typedef NurbsCurveSP<float,3> NurbsCurveSP_2Df ;
typedef NurbsCurveSP<double,3> NurbsCurveSP_2Dd ;

} // end namespace

typedef PLib::NurbsCurveSP<float,3> PlNurbsCurveSPf ;
typedef PLib::NurbsCurveSP<double,3> PlNurbsCurveSPd ;
typedef PLib::NurbsCurveSP<float,3> PlNurbsCurveSP_2Df ;
typedef PLib::NurbsCurveSP<double,3> PlNurbsCurveSP_2Dd ;



#endif
