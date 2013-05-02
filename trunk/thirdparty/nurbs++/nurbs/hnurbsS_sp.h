/*=============================================================================
        File: hnurbsS_sp.h
     Purpose:       
    Revision: $Id: hnurbsS_sp.h,v 1.2 2002/05/13 21:07:46 philosophil Exp $
  Created by: Philippe Lavoie          (14 May, 1998)
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
#ifndef _nurbs_hnurbsS_sp_h_
#define _nurbs_hnurbsS_sp_h_

#include "hnurbsS.h"

/*!
 */
namespace PLib {

/*!
  \class HNurbsSurfaceSP 
  \brief A NURBS surface with surface point

  A Nurbs surface with surface point manipulators. This allows 
  someone to modify the point on a surface for which a 
  control point has maximal influence over it. This might 
  provide a more intuitive method to modify a surface. 

  \author Philippe Lavoie
  \date 14 May, 1998
*/
template <class T, int N>
class HNurbsSurfaceSP : public HNurbsSurface<T,N>{
public:
  HNurbsSurfaceSP() ; 
  HNurbsSurfaceSP(const NurbsSurface<T,N>& S) ; 
  HNurbsSurfaceSP(const HNurbsSurface<T,N>& S) ;
  HNurbsSurfaceSP(const HNurbsSurfaceSP<T,N>& S) ;
  HNurbsSurfaceSP(HNurbsSurface<T,N>* base) ;
  HNurbsSurfaceSP(HNurbsSurface<T,N>* base, const Vector<T>& xU, const Vector<T>& xV) ;
  HNurbsSurfaceSP(HNurbsSurface<T,N>* base, const HNurbsSurface<T,N> &surf) ;

  virtual void resizeKeep(int Pu, int Pv, int DegU, int DegV) ;
  
  virtual void refineKnots(const Vector<T>& nU, const Vector<T>& nV) ;
  virtual void refineKnotU(const Vector<T>& X);
  virtual void refineKnotV(const Vector<T>& X);
  
  virtual void mergeKnots(const Vector<T>& nU, const Vector<T>& nV) ;
  virtual void mergeKnotU(const Vector<T>& X);
  virtual void mergeKnotV(const Vector<T>& X);

  virtual void updateSurface(int i0=-1, int j0=-1) ;  
  virtual void updateLevels(int updateLevel=-1) ;

  virtual HNurbsSurfaceSP<T,N>* addLevel(int nsplit, int s=1) ;
  virtual HNurbsSurfaceSP<T,N>* addLevel() ;

  virtual void copy(const HNurbsSurface<T,N>& nS) ;

  virtual int read(ifstream &fin) ;
  

  void modSurfCPby(int i, int j, const HPoint_nD<T,N>& a) ;
  void modOnlySurfCPby(int i, int j, const HPoint_nD<T,N>& a) ;

  T maxAtUV(int i, int j) const { return maxAtU_[i]*maxAtV_[j] ; }
  T maxAtU(int i) const { return maxAtU_[i] ; }
  T maxAtV(int i) const { return maxAtV_[i] ; }

  HPoint_nD<T,N> surfP(int i,int j) const 
    { return this->hpointAt(maxAtU_[i],maxAtV_[j]); }

  HPoint_nD<T,N> surfP(int i,int j, int lod) const
    { return this->hpointAt(maxAtU_[i],maxAtV_[j],lod) ; }

  void updateMaxUV() 
    { updateMaxU() ; updateMaxV() ; }
  void updateMaxU() ;
  void updateMaxV() ;

  int okMax() { return (maxU.n()<=1)?0:1 ; }

protected:

  Vector<T> maxU,maxV ;
  Vector<T> maxAtU_,maxAtV_ ;
};

typedef HNurbsSurfaceSP<float,3> HNurbsSurfaceSPf ;
typedef HNurbsSurfaceSP<double,3> HNurbsSurfaceSPd ;

template <class T, int N>
inline HNurbsSurfaceSP<T,N>::HNurbsSurfaceSP() : HNurbsSurface<T,N>() { 
  ;
}

template <class T, int N>
inline HNurbsSurfaceSP<T,N>::HNurbsSurfaceSP(const NurbsSurface<T,N>& nS)  : HNurbsSurface<T,N>(nS) { 
  updateMaxUV() ; 
}

template <class T, int N>
inline HNurbsSurfaceSP<T,N>::HNurbsSurfaceSP(HNurbsSurface<T,N>* base): HNurbsSurface<T,N>(base) {
  updateMaxUV() ; 
}

template <class T, int N>
inline HNurbsSurfaceSP<T,N>::HNurbsSurfaceSP(HNurbsSurface<T,N>* base, const Vector<T>& xU, const Vector<T>& xV) : HNurbsSurface<T,N>(base,xU,xV) {
  updateMaxUV() ;
}

template <class T, int N>
inline HNurbsSurfaceSP<T,N>::HNurbsSurfaceSP(const HNurbsSurface<T,N>& S) : HNurbsSurface<T,N>() {
  copy(S) ;
}

template <class T, int N>
inline HNurbsSurfaceSP<T,N>::HNurbsSurfaceSP(const HNurbsSurfaceSP<T,N>& S) : HNurbsSurface<T,N>() {
  copy(S) ;
}

template <class T, int N>
inline HNurbsSurfaceSP<T,N>::HNurbsSurfaceSP(HNurbsSurface<T,N>* base, const HNurbsSurface<T,N> &surf):
  HNurbsSurface<T,N>(base)
{
  copy(surf) ;
  updateMaxUV() ;
}


template <class T, int N>
inline void 
HNurbsSurfaceSP<T,N>::resizeKeep(int Pu, int Pv, int DegU, int DegV) { 
  HNurbsSurface<T,N>::resizeKeep(Pu,Pv,DegU,DegV) ; 
  updateMaxUV() ; 
}

template <class T, int N>
inline void 
HNurbsSurfaceSP<T,N>::refineKnots(const Vector<T>& nU, const Vector<T>& nV) {
  HNurbsSurface<T,N>::refineKnots(nU,nV) ;
  updateMaxUV() ;
}
  
template <class T, int N>
inline void 
HNurbsSurfaceSP<T,N>::refineKnotU(const Vector<T>& X){
  HNurbsSurface<T,N>::refineKnotU(X) ;
  updateMaxU() ;
}

template <class T, int N>
inline void 
HNurbsSurfaceSP<T,N>::refineKnotV(const Vector<T>& X){
  HNurbsSurface<T,N>::refineKnotV(X) ;
  updateMaxV() ;
}

  
template <class T, int N>
inline void 
HNurbsSurfaceSP<T,N>::mergeKnots(const Vector<T>& nU, const Vector<T>& nV) {
  HNurbsSurface<T,N>::mergeKnots(nU,nV) ;
  updateMaxUV() ;
}


template <class T, int N>
inline void 
HNurbsSurfaceSP<T,N>::mergeKnotU(const Vector<T>& X){
  HNurbsSurface<T,N>::mergeKnotU(X) ;
  updateMaxU() ;
}


template <class T, int N>
inline void 
HNurbsSurfaceSP<T,N>::mergeKnotV(const Vector<T>& X){
  HNurbsSurface<T,N>::mergeKnotV(X) ;
  updateMaxV() ;
}

} // end namespace

typedef PLib::HNurbsSurfaceSP<float,3> PlHNurbsSurfaceSPf ;
typedef PLib::HNurbsSurfaceSP<double,3> PlHNurbsSurfaceSPd ;



#endif
