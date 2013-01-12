/*=============================================================================
        File: nurbsS_sp.h
     Purpose:       
    Revision: $Id: nurbsS_sp.h,v 1.2 2002/05/13 21:07:46 philosophil Exp $
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
=============================================================================*/
#ifndef _nurbs_nurbsS_sp_h_
#define _nurbs_nurbsS_sp_h_

#include "nurbsS.h"

/*!
 */
namespace PLib {

/*!
  \class NurbsSurfaceSP nurbsS_sp.h
  \brief A NURBS surface with surface point

  A Nurbs surface with surface point manipulators. This allows 
  someone to modify the point on a surface for which a 
  control point has maximal influence over it. This might 
  provide a more intuitive method to modify a surface. 


  \author Philippe Lavoie
  \date 8 May, 1998
*/
template <class T, int N>
class NurbsSurfaceSP : public NurbsSurface<T,N>{
public:
  NurbsSurfaceSP(); 
  NurbsSurfaceSP(const NurbsSurface<T,N>& nS); 
  NurbsSurfaceSP(const NurbsSurfaceSP<T,N>& nS); 
  NurbsSurfaceSP(int DegU, int DegV, const Vector<T>& Uk, const Vector<T>& Vk, const Matrix< HPoint_nD<T,N> >& Cp);  
  NurbsSurfaceSP(int DegU, int DegV, Vector<T>& Uk, Vector<T>& Vk, Matrix< Point_nD<T,N> >& Cp, Matrix<T>& W)  ; 


  virtual NurbsSurface<T,N>& operator=(const NurbsSurface<T,N>& a) ;
  virtual NurbsSurfaceSP<T,N>& operator=(const NurbsSurfaceSP<T,N>& a) ;
  
  virtual void resizeKeep(int Pu, int Pv, int DegU, int DegV) ;
  

  virtual void refineKnots(const Vector<T>& nU, const Vector<T>& nV) ;
  virtual void refineKnotU(const Vector<T>& X);
  virtual void refineKnotV(const Vector<T>& X);
  
  virtual void mergeKnots(const Vector<T>& nU, const Vector<T>& nV) ;
  virtual void mergeKnotU(const Vector<T>& X);
  virtual void mergeKnotV(const Vector<T>& X);

  virtual int read(ifstream &fin);

  
  virtual void degreeElevateU(int tU);
  virtual void degreeElevateV(int tV);

  NurbsSurfaceSP<T,N> generateParallel(T d) const ; 


  void modSurfCPby(int i, int j, const HPoint_nD<T,N>& a) //!< Moves a surface point by a value
    { this->P(i,j) +=  a / (maxU[i]*maxV[j]) ;  }
  void modSurfCP(int i, int j, const HPoint_nD<T,N>& a) //!< Moves a surface point to a value
    { modSurfCPby(i,j,a-surfP(i,j)) ;  }

  void modOnlySurfCPby(int i, int j, const HPoint_nD<T,N>& a) ;
  void modOnlySurfCP(int i, int j, const HPoint_nD<T,N>& a) //!< Changes only the the surface point near a control point, the other surface point will not be moved.
    { modOnlySurfCPby(i,j,a-surfP(i,j)) ;  }

  T maxAtUV(int i, int j) const //!< The maximal basis function for the control point i,j
    { return maxAtU_[i]*maxAtV_[j] ; }
  T maxAtU(int i) const //!< Where is the maximal basis function in U for the control points in row i
    { return maxAtU_[i] ; }
  T maxAtV(int i) const //!< Where is the maximal basis function in U for the control points in colum i
    { return maxAtV_[i] ; }

  HPoint_nD<T,N> surfP(int i,int j) const  //!< the surface point for the control point at i,j
    { return this->hpointAt(maxAtU_[i],maxAtV_[j]); }

  void updateMaxUV() //!< Updates both the maxU and maxV values
    { updateMaxU() ; updateMaxV() ; }
  void updateMaxU() ;
  void updateMaxV() ;

  int okMax() //!< Is the maximal value in a valid range ?
    { return (maxU.n()<=1)?0:1 ; }

protected:

  Vector<T> maxU ; //!< The vector of maximal basis function value in U 
  Vector<T> maxV ; //!< The vector of maximal basis function value in V
  Vector<T> maxAtU_ ; //!< The vector identifiying where is the maximal basis function in U
  Vector<T> maxAtV_ ; //!< The vector identifiying where is the maximal basis function in V
};


/*! 
  \fn NurbsSurfaceSP<T,N>::NurbsSurface()
  \brief  Default constructor

  \warning The surface is initialized to invalid values. Use a reset or
           a read function to set them to correct values.
  \author Philippe Lavoie  
  \date 24 January, 1997
*/
template <class T, int N>
inline NurbsSurfaceSP<T,N>::NurbsSurfaceSP() : NurbsSurface<T,N>() { 
  ;
}

/*! 
  \fn NurbsSurfaceSP<T,N>::NurbsSurface(const NurbsSurface<T,N>& s)
  \brief  the copy constructor

  \param s the NurbsSurface<T,N> to copy    

  \author Philippe Lavoie 
  \date 24 January, 1997
*/
template <class T, int N>
inline NurbsSurfaceSP<T,N>::NurbsSurfaceSP(const NurbsSurface<T,N>& nS)  : NurbsSurface<T,N>(nS) { 
  updateMaxUV() ; 
}

/*! 
  \fn NurbsSurfaceSP<T,N>::NurbsSurface(const NurbsSurfaceSP<T,N>& s)
  \brief  the copy constructor

  \param s the NurbsSurface<T,N> to copy    

  \author Philippe Lavoie 
  \date 24 January, 1997
*/
template <class T, int N>
inline NurbsSurfaceSP<T,N>::NurbsSurfaceSP(const NurbsSurfaceSP<T,N>& nS)  : NurbsSurface<T,N>(nS) { 
  maxU = nS.maxU ; 
  maxV = nS.maxV ; 
  maxAtU_ = nS.maxAtU_ ; 
  maxAtV_ = nS.maxAtV_ ; 
}

/*! 
  \fn NurbsSurfaceSP<T,N>::NurbsSurface(int DegU, int DegV, const Vector<T>& Uk, const Vector<T>& Vk, const Matrix< HPoint_nD<T,N> >& Cp)
  \brief constructor with points in homogenous space

  \param DegU  the degree in the $u$ direction
  \param DegV  the degree in the $v$ direction
  \param   Uk  the $u$ knot vector
  \param   Vk  the $v$ knot vector   
  \param   Cp  the matrix of control points in 4D

  \author Philippe Lavoie 
  \date 24 January, 1997
*/
template <class T, int N>
inline NurbsSurfaceSP<T,N>::NurbsSurfaceSP(int DegU, int DegV, const Vector<T>& Uk, const Vector<T>& Vk, const Matrix< HPoint_nD<T,N> >& Cp)  : NurbsSurface<T,N>(DegU,DegV,Uk,Vk,Cp) { 
  updateMaxUV() ; 
}

/*! 
  \fn NurbsSurfaceSP<T,N>::NurbsSurface(int DegU, int DegV, Vector<T>& Uk, Vector<T>& Vk, Matrix< Point_nD<T,N> >& Cp, Matrix<T>& W)
  \brief constructor with points in 3D

  \param DegU  the degree of the surface in the U direction
  \param DegV  the degree of the surface in the V direction
  \param Uk  the U knot vector 
  \param Vk  the V knot vector
  \param Cp  the matrix of 3D control points
  \param W  the weight value for each control points

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
inline NurbsSurfaceSP<T,N>::NurbsSurfaceSP(int DegU, int DegV, Vector<T>& Uk, Vector<T>& Vk, Matrix< Point_nD<T,N> >& Cp, Matrix<T>& W)  : NurbsSurface<T,N>(DegU,DegV,Uk,Vk,Cp,W) { 
  updateMaxUV() ; 
}

/*! 
  \fn virtual NurbsSurface<T,N>& NurbsSurface<T,N>::operator=(const NurbsSurface<T,N>& nS)
  \brief NurbsSurface<T,N> assignment

  \param nS the NURBS surface to copy

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
inline NurbsSurface<T,N>& 
NurbsSurfaceSP<T,N>::operator=(const NurbsSurface<T,N>& a) { 
  NurbsSurface<T,N>::operator=(a) ; 
  updateMaxUV() ; 
  return *this ; 
}

/*! 
  \fn virtual NurbsSurface<T,N>& NurbsSurfaceSP<T,N>::operator=(const NurbsSurface<T,N>& nS)
  \brief NurbsSurface<T,N> assignment
  \param nS the NURBS surface to copy

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
inline  NurbsSurfaceSP<T,N>& 
NurbsSurfaceSP<T,N>::operator=(const NurbsSurfaceSP<T,N>& a) { 
  NurbsSurface<T,N>::operator=(a) ;  
  maxU = a.maxU ; 
  maxV = a.maxV ; 
  maxAtU_ = a.maxAtU_ ; 
  maxAtV_ = a.maxAtV_ ;  
  return *this ; 
}

/*! 
  \fn virtual void NurbsSurfaceSP<T,N>::resizeKeep(int Pu, int Pv, int DegU, int DegV)
  \brief Resize the surface while keeping the old values.

  \param  Pu  the number of control points in the U direction
  \param  Pv  the number of control points in the V direction
  \param  DegU  the degree of the surface in the U direction
  \param  DegV  the degree of the surface in the V direction

  \author Philippe Lavoie          
  \date 24 January, 1997
*/
template <class T, int N>
inline void 
NurbsSurfaceSP<T,N>::resizeKeep(int Pu, int Pv, int DegU, int DegV) { 
  NurbsSurface<T,N>::resizeKeep(Pu,Pv,DegU,DegV) ; 
  updateMaxUV() ; 
}

/*! 
  \fn virtual void NurbsSurfaceSP<T,N>::refineKnots(const Vector<T>& nU, const Vector<T>& nV)
  \brief Refine both knot vectors

  \param nU  the U knot vector to refine from 
  \param nV  the V knot vector to refine from 

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
inline void 
NurbsSurfaceSP<T,N>::refineKnots(const Vector<T>& nU, const Vector<T>& nV) {
  NurbsSurface<T,N>::refineKnots(nU,nV) ;
  updateMaxUV() ;
}
  
/*! 
  \fn virtual void NurbsSurface<T,N>::refineKnotU(const Vector<T>& X)
  \brief  Refines the U knot vector

  \param X  the knot vector to refine from

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
inline void 
NurbsSurfaceSP<T,N>::refineKnotU(const Vector<T>& X){
  NurbsSurface<T,N>::refineKnotU(X) ;
  updateMaxU() ;
}

/*! 
  \fn virtual void NurbsSurfaceSP<T,N>::refineKnotV(const Vector<T>& X)
  \brief Refines the V knot vector

  \param X  the knot vector to refine from 

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
inline void 
NurbsSurfaceSP<T,N>::refineKnotV(const Vector<T>& X){
  NurbsSurface<T,N>::refineKnotV(X) ;
  updateMaxV() ;
}

  
/*
  \fn virtual void NurbsSurfaceSP<T,N>::mergeKnots(const Vector<T>& nU, const Vector<T>& nV)
  \brief Merge a knot vector in the U and V direction
  
  \param nU  the knot to merge with in the U direction
  \param nV  the knot to merge with in the V direction

  \author Philippe Lavoie
*/
template <class T, int N>
inline void 
NurbsSurfaceSP<T,N>::mergeKnots(const Vector<T>& nU, const Vector<T>& nV) {
  NurbsSurface<T,N>::mergeKnots(nU,nV) ;
  updateMaxUV() ;
}


/*
  \fn virtual void NurbsSurfaceSP<T,N>::mergeKnotU(const Vector<T>& X)
  \brief Merge a knot vector in the U direction
  
  \param X  the knot to merge with

  \author Philippe Lavoie
*/
template <class T, int N>
inline void 
NurbsSurfaceSP<T,N>::mergeKnotU(const Vector<T>& X){
  NurbsSurface<T,N>::mergeKnotU(X) ;
  updateMaxU() ;
}


/*
  \fn virtual void NurbsSurfaceSP<T,N>::mergeKnotV(const Vector<T>& X)
  \brief Merge a knot vector in the V direction
  
  \param X  the knot to merge with

  \author Philippe Lavoie
*/
template <class T, int N>
inline void 
NurbsSurfaceSP<T,N>::mergeKnotV(const Vector<T>& X){
  NurbsSurface<T,N>::mergeKnotV(X) ;
  updateMaxV() ;
}

/*
  \fn virtual int NurbsSurfaceSP<T,N>::read(ifstream &fin)
  \brief Reads an input file stream
  
  \param fin the input file stream

  \author Philippe Lavoie
*/
template <class T, int N>
inline int NurbsSurfaceSP<T,N>::read(ifstream &fin) {
  int r = NurbsSurface<T,N>::read(fin) ;
  updateMaxUV() ; 
  return r ; 
}

/*
  \fn virtual void NurbsSurfaceSP<T,N>::degreeElevateU(int tU)
  \brief Degree Elevate in the U direction
  
  \param tU the number of degree to elevate the surface by

  \author Philippe Lavoie
*/
template <class T, int N>
inline void NurbsSurfaceSP<T,N>::degreeElevateU(int tU) {
  NurbsSurface<T,N>::degreeElevateU(tU);
  updateMaxU();
}

/*
  \fn virtual void NurbsSurfaceSP<T,N>::degreeElevateV(int tV) 
  \brief Degree Elevate in the V direction
  
  \param tV the number of degree to elevate the surface by

  \author Philippe Lavoie
*/
template <class T, int N>
inline void NurbsSurfaceSP<T,N>::degreeElevateV(int tV) {
  NurbsSurface<T,N>::degreeElevateV(tV);
  updateMaxV();
}

typedef NurbsSurfaceSP<float,3> NurbsSurfaceSPf ;
typedef NurbsSurfaceSP<double,3> NurbsSurfaceSPd ;

} // end namespace

typedef PLib::NurbsSurfaceSP<float,3> PlNurbsSurfaceSPf ;
typedef PLib::NurbsSurfaceSP<double,3> PlNurbsSurfaceSPd ;


#endif
