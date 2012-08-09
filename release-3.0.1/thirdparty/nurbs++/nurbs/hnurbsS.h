/*=============================================================================
        File: hnurbsS.h
     Purpose:       
    Revision: $Id: hnurbsS.h,v 1.2 2002/05/13 21:07:46 philosophil Exp $
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
#ifndef _nurbs_hnurbs_h_
#define _nurbs_hnurbs_h_

#include "nurbsS.h"
#include "vector.h"

/*!
 */
namespace PLib {

/*!
  \class HNurbsSurface hnurbsS.h
  \brief A hierarchichal NURBS surface class

  This class can represent and manipulate a hierarchical NURBS
  surface. The surface is composed of points in homogenous space. It
  can have any degree and have any number of control points.
  
  This does not correspond to the HBsplines given by
  Forsey. However I hope that it will be usefull for
  interactive manipulations of NURBS surfaces.
  
  Other aspects of my implementation are different. They
  will be documented when the class is fully functionnal.
  
  Presently there is only a limited set of functions available for
  this class. And honestly, until I can optimize the space requirement
  of the class I don't think you should build anything critical with
  a HNurbsSurface.

  \author Philippe Lavoie 
  \date 28 September 1997 
*/
template <class T, int N>
class HNurbsSurface: public NurbsSurface<T,N> {
public:
  HNurbsSurface() ; 
  HNurbsSurface(const NurbsSurface<T,N>& S) ; 
  HNurbsSurface(const HNurbsSurface<T,N>& S) ;
  HNurbsSurface(HNurbsSurface<T,N>* base) ;
  HNurbsSurface(HNurbsSurface<T,N>* base, const Vector<T>& xU, const Vector<T>& xV) ;
  HNurbsSurface(HNurbsSurface<T,N>* base, const HNurbsSurface<T,N> &surf) ;
  virtual ~HNurbsSurface() ;

  HNurbsSurface<T,N>* baseLevel() const { return baseLevel_;}
  HNurbsSurface<T,N>* nextLevel() const { return nextLevel_;}
  HNurbsSurface<T,N>* firstLevel() const { return firstLevel_;}
  HNurbsSurface<T,N>* lastLevel() const { return lastLevel_; }

  void splitUV(int nu, int nv, Vector<T> &nU, Vector<T> &nV) ;
  void splitUV(int nu, int su, int nv, int sv, Vector<T> &nU, Vector<T> &nV) ;
  
  virtual HNurbsSurface<T,N>* addLevel(int nsplit) ;
  virtual HNurbsSurface<T,N>* addLevel() ;

  virtual void copy(const HNurbsSurface<T,N>& nS) ;

  int modifies(T u, T v) ;

  
  HPoint_nD<T,N> operator()(T u, T v) const { return hpointAt(u,v,-1) ; }
  HPoint_nD<T,N> hpointAt(T u, T v, int lod=-1) const ;

  void deriveAtH(T u, T v, int, Matrix< HPoint_nD<T,N> >&, int lod=-1) const;
  void deriveAt(T u, T v, int, Matrix< Point_nD<T,N> >&, int lod=-1) const;

  int movePointOffset(T u, T v, const Point_nD<T,N>& delta) ; 

  void scale(const Point_nD<T,N>& s) ; 


  int initBase(int force=0) ;
  virtual void updateSurface(int i0=-1, int j0=-1) ;
  virtual void updateLevels(int updateLevel=-1) ;

  int isoCurveU(T u, NurbsCurve<T,N>& c,int lod=-1) const ;
  int isoCurveV(T v, NurbsCurve<T,N>& c,int lod=-1) const ;

  // I/O functions
  int read(const char* filename);
  int write(const char* filename) const;
  virtual int read(ifstream &fin) ;
  int write(ofstream &fout) const ;

  Matrix< HPoint_nD<T,N> > offset ;
  Vector<T> rU,rV ;

  int level() const { return level_ ;}

  int maxLevel() const ;

  int modifiedN() const { return updateN ; }

  void refineKnots(const Vector<T>& nU, const Vector<T>& nV) ;
  void refineKnotU(const Vector<T>& X) ;
  void refineKnotV(const Vector<T>& X) ;


  void axis(int i, int j, Point_nD<T,N>& xaxis, Point_nD<T,N>& yaxis, Point_nD<T,N>& zaxis)const;

  void setFixedOffsetVector(const Point_nD<T,N> &I, const Point_nD<T,N> &J, const Point_nD<T,N>& K);
  void setVariableOffsetVector();

protected:
  NurbsSurface<T,N> baseSurf ;
  HNurbsSurface<T,N> *baseLevel_,*nextLevel_,*firstLevel_,*lastLevel_ ;

  Matrix< Point_nD<T,N> > ivec,jvec,kvec ;

  int level_ ;
  int updateN,baseUpdateN ;
  int update_ ; // indicates that one of the offset point was modified

  T uS_,uE_,vS_,vE_ ;
  T uD,vD ; 
  int fixedOffset;
};

template <class T, int N>
inline void 
HNurbsSurface<T,N>::axis(int i, int j, Point_nD<T,N>& xaxis, Point_nD<T,N>& yaxis, Point_nD<T,N>& zaxis) const 
{ 
  if(baseLevel_) { 
    xaxis = ivec(i,j) ; 
    yaxis = jvec(i,j) ; 
    zaxis = kvec(i,j) ; 
  } 
  else{
    xaxis = Point_nD<T,N>(T(1),T(0),T(0)) ; 
    yaxis = Point_nD<T,N>(T(0),T(1),T(0)) ; 
    zaxis = Point_nD<T,N>(T(0),T(0),T(1)) ; 
  }
}

typedef HNurbsSurface<float,3> HNurbsSurfacef ;
typedef HNurbsSurface<double,3> HNurbsSurfaced ;

} // end namespace

typedef PLib::HNurbsSurface<float,3> PlHNurbsSurfacef ;
typedef PLib::HNurbsSurface<double,3> PlHNurbsSurfaced ;


#endif // _nurbs_hnurbs_h_
