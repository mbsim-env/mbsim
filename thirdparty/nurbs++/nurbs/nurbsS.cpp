/*=============================================================================
        File: nurbsS.cpp
     Purpose:       
    Revision: $Id: nurbsS.cpp,v 1.2 2002/05/13 21:07:46 philosophil Exp $
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

#include <string.h>
#include <matrixRT.h>
#include <math.h>
#include <nurbsS.h>
#include "integrate.h"

#ifdef USING_VCC
#include <malloc.h>
#endif

/*!
 */
namespace PLib {

/*! 
  \brief  Default constructor

  \warning The surface is initialized to invalid values. Use a reset or
           a read function to set them to correct values.
  \author Philippe Lavoie  
  \date 24 January, 1997
*/
template <class T, int N>
NurbsSurface<T,N>::NurbsSurface(): 
  ParaSurface<T,N>(), U(1),V(1),P(1,1),degU(0),degV(0)
{
}

/*! 
  \brief  the copy constructor

  \param s the NurbsSurface<T,N> to copy    

  \author Philippe Lavoie 
  \date 24 January, 1997
*/
template <class T, int N>
NurbsSurface<T,N>::NurbsSurface(const NurbsSurface<T,N>& s): 
  ParaSurface<T,N>(), U(s.U), V(s.V), P(s.P), degU(s.degU), degV(s.degV)
{
}


/*! 
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
NurbsSurface<T,N>::NurbsSurface(int DegU, int DegV, const Vector<T>& Uk, const Vector<T>& Vk, const Matrix< HPoint_nD<T,N> >& Cp) : 
  ParaSurface<T,N>(), U(Uk),V(Vk),P(Cp),degU(DegU),degV(DegV)
{
  int bad = 0 ;

  if(U.n() != P.rows()+degU+1){
#ifdef USE_EXCEPTION
    throw NurbsSizeError(P.rows(),U.n(),degU) ;
#else
    Error err("NurbsSurface<T,N> constructor") ;
    err << "The U knot vector and the number of rows of the control points are incompatible\n" ;
    err << "P.rows() = " << P.rows() << ", U.n() = " << U.n() << endl ;
    bad = 1 ;
    err.fatal() ;
#endif
  }
  if(V.n() != P.cols()+degV+1){
#ifdef USE_EXCEPTION
    throw NurbsSizeError(P.cols(),V.n(),degV);
#else
    Error err("NurbsSurface<T,N> constructor") ;    
    err << "The V knot vector and the number of columns of the control points are incompatible\n" ;
    err << "P.cols() = " << P.cols() << ", V.n() = " << V.n() << endl ;
    bad = 1 ;
    err.fatal() ;
#endif
  }
  if(bad)
    exit(-1) ;
}

/*! 
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
NurbsSurface<T,N>::NurbsSurface(int DegU, int DegV, Vector<T>& Uk, Vector<T>& Vk, Matrix< Point_nD<T,N> >& Cp, Matrix<T>& W) : 
  ParaSurface<T,N>(), U(Uk),V(Vk),P(Cp.rows(),Cp.cols()),degU(DegU),degV(DegV)
{
  int bad = 0 ;

  if(U.n() != Cp.rows()+degU+1){
#ifdef USE_EXCEPTION
    throw NurbsSizeError(P.rows(),U.n(),degU);
#else
    Error err("NurbsSurface<T,N> constructor") ;
    err << "The U knot vector and the number of rows of the control points are incompatible\n" ;
    err << "P.rows() = " << P.rows() << ", U.n() = " << U.n() << endl ;
    bad = 1 ;
    err.fatal() ;
#endif
  }
  if(V.n() != Cp.cols()+degV+1){
#ifdef USE_EXCEPTION
    throw NurbsSizeError(P.cols(),V.n(),degV);
#else
    Error err("NurbsSurface<T,N> constructor") ;
    err << "The V knot vector and the number of columns of the control points are incompatible\n" ;
    err << "P.cols() = " << P.cols() << ", V.n() = " << V.n() << endl ;
    bad = 1 ;
    err.fatal() ;
#endif
  }
  if(W.rows() != Cp.rows()){
#ifdef USE_EXCEPTION
    throw NurbsInputError(W.rows(),Cp.rows()) ;
#else
    Error err("NurbsSurface<T,N> constructor") ;
    err << "The dimension of the weights are incompatible with the dimension of the control poitns\n" ;
    err << "W( " << W.rows() << ", " << W.cols() << ") and P( " << P.rows() << ", " << P.cols() << ") \n" ;
    bad = 1 ;
    err.fatal() ;
#endif
  }
  if(W.cols() != Cp.cols()){
#ifdef USE_EXCEPTION
    throw NurbsInputError(W.cols(),Cp.cols()) ;
#else
    Error err("NurbsSurface<T,N> constructor") ;
    err << "The dimension of the weights are incompatible with the dimension of the control poitns\n" ;
    err << "W( " << W.rows() << ", " << W.cols() << ") and P( " << P.rows() << ", " << P.cols() << ") \n" ;
    bad = 1 ;
    err.fatal() ;
#endif
  }

  for(int i=0;i<Cp.rows();i++)
    if(bad)
      break ;
    else{
      for(int j=0;j<Cp.cols();j++)
	if(W(i,j)==0.0){
#ifdef USE_EXCEPTION
	  throw NurbsInputError();
#else
	  Error err("NurbsSurface<T,N> constructor") ;
	  err << "Error initializing a NurbsSurface.\n" ;
	  err << "The weight W(" << i << ", " << j << ") is equal to 0.\n" ;
	  bad = 1 ;
	  err.fatal() ;
	  break ;
#endif
	}
	else {
	  P(i,j) = Cp(i,j) ;
	  P(i,j) *= W(i,j) ;
	}
    }

  if(bad)
    exit(-1) ;
}

/*! 
  \brief NurbsSurface<T,N> assignment
  \param nS the NURBS surface to copy

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
NurbsSurface<T,N>& NurbsSurface<T,N>::operator=(const NurbsSurface<T,N>& nS){
  P = nS.P ;
  U = nS.U ;
  V = nS.V ;
  degU = nS.degU ;
  degV = nS.degV ;
  return *this ;
}

template <class T, int N>
struct AreaData {
  T v;
  T eps;
  T knotUi;
  T knotUii;
  T knotVj;
  T knotVjj;
  const NurbsSurface<T,N>& s ; 
  const BasicArray<T>  w ; 
  AreaData(const NurbsSurface<T,N>& surf,T e,
           const BasicArray<T>& ww): 
    s(surf),v(T(0)),eps(e),w(ww),
    knotUi(T(0)), knotUii(T(1)) {}
};

template <class T, int N> 
struct OpAreaAuxFcn : public ClassPOvoid<T> {
  T operator()(T u, void* data){
    AreaData<T,N>* p = (AreaData<T,N>*)data ;
    return (p->s).areaF(u,p->v) ; 
  }
};

template <class T, int N>
struct OpAreaFcn : public ClassPOvoid<T> {
  T operator()(T v, void* data){
    static Vector<T> w;
    T err;
    OpAreaAuxFcn<T,N>  f;
    AreaData<T,N>* Data = (AreaData<T,N>*)data ;
    Data->v = v ;            
    return intcc2((ClassPOvoid<T>*)&f,Data,
		  Data->knotUi,Data->knotUii,
		  Data->eps,Data->w,err);  
  }
};

/*!
  \brief Computes the area of the surface

  Computes an approximation of the area of the surface
  using a numerical automatic integrator.
  
  That integrator uses a Chebyshev Series Expansion
  to perform its approximation. This is why you can
  change the value \a n which sets the number of 
  elements in the series. 

  The method is simple, integrate between each span.
  This is necessary in case the tangant of a point
  at u_i is undefined. Add the result and return
  this as the approximation.

  \param eps  the accepted relative error
  \param n  the number of element in the Chebyshev series
  
  \return the area of the NURBS surface.

  \author Alejandro Frangi
  \date 20 January 1999
*/
template <class T, int N>
T NurbsSurface<T,N>::area(T eps,int n) const {
  T a = T(0.0) ;
  T err ; 
  static Vector<T> bufFcn(0) ;

  if(bufFcn.n() != n){
    bufFcn.resize(n) ;
    intccini(bufFcn) ;
  }

  AreaData<T,N> data(*this,eps,bufFcn) ; 
  OpAreaFcn<T,N> op;
  

  for(int i=degU;i<P.rows();++i){
    if(U[i] >= U[i+1] || U[i]>=T(1.0))
      continue ;
    data.knotUi  = U[i] ; 
    data.knotUii = U[i+1] ; 
    for(int j=degV;j<P.cols();++j){
      if(V[j] >= V[j+1] || V[j]>=T(1.0))
        continue ;
      data.knotVj  = V[j] ; 
      data.knotVjj = V[j+1] ; 
      a += intcc2((ClassPOvoid<T>*)&op,(void*)&data,
		  data.knotVj,data.knotVjj,
		  eps,bufFcn,err) ;
    }
  }
  return a ; 
}

/*!
  \brief Computes the area of the surface inside [u_s,u_e]

  Computes an approximation of the area of the surface
  using a numerical automatic integrator. The area
  is computed for the range [u_s,u_e]
  
  That integrator uses a Chebyshev Series Expansion
  to perform its approximation. This is why you can
  change the value \a n which sets the number of 
  elements in the series. 

  The method is similar to the one used by area
  excepted that it needs to check for the range.

  \param  us  the starting range 
  \param  ue  the end of the range
  \param  vs  the starting range 
  \param  ve  the end of the range
  \param  n  the number of element in the Chebyshev series
  \param  eps  the accepted relative error

  \return the area of the NURBS surface.

  \warning ue (ve) must be greater than us (vs) and both must be in a valid range.

  \author Alejandro  Frangi
  \date   20 January 1999
*/
template <class T, int N>
T NurbsSurface<T,N>::areaIn(T us, T ue, T vs, T ve, T eps, int n) const {
  T l = T() ;
  T err ; 
  T a ;
  bool bLastU = false;
  bool bLastV = false;
  static Vector<T> bufFcn ;

  if(bufFcn.n() != n){
    bufFcn.resize(n) ;
    intccini(bufFcn) ;
  }
  
  AreaData<T,N> data(*this,eps,bufFcn) ; 
  OpAreaFcn<T,N> op;
  for(int i=degU;i<P.rows();++i){
    if(U[i] >= U[i+1] || U[i] >= T(1))
      continue;
    if(i<findSpanU(us))
      continue ; 
    if(us>=U[i] && ue<=U[i+findMultU(i)]){
      data.knotUi  = us;
      data.knotUii = ue;
      bLastU = true;
      goto Integrate_I;
    }
    if(us>=U[i]){
      data.knotUi  = us;
      data.knotUii = U[i+findMultU(i)];
      bLastU = false;
      goto Integrate_I;
    }
    if(ue<=U[i+1]){
      data.knotUi  = U[i];
      data.knotUii = ue;
      bLastU = true;
      goto Integrate_I;
    }
    data.knotUi  = U[i] ; 
    data.knotUii = U[i+findMultU(i)] ; 
  Integrate_I:
    for(int j=degV;j<P.cols();++j){
      if(V[j] >= V[j+1] || V[j]>=T(1))
        continue ;
      if(j<findSpanV(vs))
        continue ; 
      if(vs>=V[j] && ve<=V[j+findMultV(j)]){
        data.knotVj  = vs;
        data.knotVjj = ve;
        bLastV = true;
        goto Integrate_II;
      }
      if(vs>=V[j]){
        data.knotVj  = vs;
        data.knotVjj = V[j+findMultV(j)];
        bLastV = false;
        goto Integrate_II;
      }
      if(ve<=V[j+1]){
        data.knotVj  = V[j];
        data.knotVjj = ve;
        bLastV = true;
        goto Integrate_II;
      }
      data.knotVj  = V[j] ; 
      data.knotVjj = V[j+findMultV(j)] ; 
    Integrate_II:
      a += intcc2((ClassPOvoid<T>*)&op,(void*)&data,data.knotVj,data.knotVjj,eps,bufFcn,err) ;
      if (bLastU && bLastV)
        return a;
      if (bLastV)
        break;
    }
  }
  return a ; 
}

// the definitions are in f_nurbs.cpp and d_nurbs.cpp


/*!

  \a area needs to integrate a function over an interval
  to determine the area of the NURBS surface. 
  Well, this is the function.

  \param u  the parameter
  \param v  the parameter

  \return the elemental area at (u,v)

  \author Alejandro Frangi
  \date   20 January 1999
*/
template <class T, int N>
  T NurbsSurface<T,N>::areaF(T u, T v) const {
  
  Matrix<Point_nD<T,N> > Skl(2,2) ; 
  deriveAt(u,v,1,Skl);
  T tmp = norm(crossProduct(Skl(1,0),Skl(0,1)));
  return tmp ; 
}
 
/*! 
  \brief Determines if the surface is valid

  Determines if the surface is valid. The routine only verifies 
  if the number of control points in the U and V direction 
  matches the area of the U and V knot vectors.

  \return 1 if the surface is valid, 0 otherwise

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
int NurbsSurface<T,N>::ok() {
  if(P.rows() <= degU)
    return 0 ;
  if(P.cols() <= degV)
    return 0 ;
  if(P.rows() != U.n()+degU+1)
    return 0 ;
  if(P.cols() != V.n()+degV+1)
    return 0 ;
  return 1 ;
}

/*! 
  \brief Resize the surface

  Resize the surface. Proper values must be assigned once this 
  function has been called since the resize operator is 
  destructive.

  \param  Pu  the number of control points in the U direction
  \param  Pv  the number of control points in the V direction
  \param  DegU  the degree of the surface in the U direction
  \param  DegV  the degree of the surface in the V direction

  \author Philippe Lavoie 
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsSurface<T,N>::resize(int Pu, int Pv, int DegU, int DegV){
  P.resize(Pu,Pv) ;
  degU = DegU ;
  degV = DegV ;
  U.resize(Pu+DegU+1) ;
  V.resize(Pv+DegV+1) ;
}

/*! 
  \brief Resize the surface while keeping the old values.

  \param  Pu  the number of control points in the U direction
  \param  Pv  the number of control points in the V direction
  \param  DegU  the degree of the surface in the U direction
  \param  DegV  the degree of the surface in the V direction

  \author Philippe Lavoie          
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsSurface<T,N>::resizeKeep(int Pu, int Pv, int DegU, int DegV){
  P.resizeKeep(Pu,Pv) ;
  degU = DegU ;
  degV = DegV ;
  U.resize(Pu+DegU+1) ;
  V.resize(Pv+DegV+1) ;
}

/*! 
  \brief Generates a NURBS surface from  skinning

  The NURBS surface is generated from skinning. The skinning is performed 
  in the V direction.

  \param   ca  an array of NURBS curves
  \param  degV  the degree to skin in the V direction
  \param  surf  the skinned surface

  \return 0 if an error occurs, 1 otherwise

  \warning The number of curves to skin from must be greater than degV

  \author Philippe Lavoie          
  \date 24 January, 1997
*/
template <class T, int D>
int NurbsSurface<T,D>::skinV(NurbsCurveArray<T,D>& ca, int dV) {
  Vector<T> vk(ca.n()) ;
  //Vector<T> d(ca.n()) ;
  T* d ;
  int i,k,N,K ;

  if(ca.n()<dV){
#ifdef USE_EXCEPTION
    throw NurbsInputError();
#else
    Error err("NurbsSurface<T,D> skinV") ;
    err << "The number of curves are insufficient for the degree of interpolation specified.\n" ;
    err << "n of curves = " << ca.n() << ", degree of interpolation = " << dV << endl ;
    err.warning() ;
    return 0 ;
#endif
  }

  generateCompatibleCurves(ca) ;

  K = ca.n() ;
  N = ca[0].ctrlPnts().n() ;

  resize(N,K,ca[0].degree(),dV) ;

  //d.resize(ca[0].ctrlPnts().n()) ;
  d = new T[ca[0].ctrlPnts().n()] ; 
  for(i=0;i<N;i++){
    d[i] = 0 ;
    for(k=1;k<K;k++){
      d[i] += norm(ca[k].ctrlPnts(i)-ca[k-1].ctrlPnts(i)) ;
    }
  }

  vk[0] = 0 ;
  for(k=1;k<K;k++){
    vk[k] = 0 ;
    for(i=0;i<N;i++)
      vk[k] += norm(ca[k].ctrlPnts(i)-ca[k-1].ctrlPnts(i))/d[i] ;
    vk[k] /= (T)N ;//+ 1.0 ;
    vk[k] += vk[k-1] ;
  }
  vk[vk.n()-1] = 1.0 ;

  for(i=1;i<K-degV;i++){
    V[i+degV] = 0 ;
    for(k=i;k<i+degV;k++)
      V[i+degV] += vk[k] ;
    V[i+degV] /= (T)degV ;
  }
  for(i=0;i<=degV;i++) V[i] = 0.0 ;
  for(i=V.n()-degV-1;i<V.n();i++) V[i] = 1.0 ;


  Vector< HPoint_nD<T,D> > Q(K) ;
  NurbsCurve<T,D> i_curve ;


  for(i=0;i<N;i++){
    for(k=0;k<K;k++)
      Q[k] = ca[k].ctrlPnts(i) ;
    i_curve.globalInterpH(Q,vk,V,degV);
    for(k=0;k<K;k++)
      P(i,k) = i_curve.ctrlPnts(k) ;
  }

  U = ca[0].knot() ;

  delete []d ; 
  return 1 ;
}

/*! 
  \brief Generates a NURBS surface from  skinning

  The NURBS surface is generates from skinning. 
  The skinning is performed in the U direction.

  \param    ca  an array of NURBS curves
  \param   degU  the degree to skin in the Udirection

  \return 0 if an error occurs, 1 otherwise

  \warning The number of curves to skin from must be greater than degU

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int nD>
int NurbsSurface<T,nD>::skinU(NurbsCurveArray<T,nD>& ca, int dU) {
  Vector<T> uk(ca.n());
  //Vector<T> d(ca.n()) ;
  T* d; 
  int i,k,N,K ;

  if(ca.n()<dU){
#ifdef USE_EXCEPTION
    throw NurbsInputError();
#else
    Error err("NurbsSurface<T,N> skinU") ;
    err << "The number of curves are insufficient for the degree of interpolation specified.\n" ;
    err << "n of curves = " << ca.n() << ", degree of interpolation = " << dU << endl ;
    err.warning() ;
    return 0 ;
#endif
  }

  generateCompatibleCurves(ca) ;

  K = ca.n() ;
  N = ca[0].ctrlPnts().n() ;

  resize(K,N,dU,ca[0].degree()) ;

  //d.resize(ca[0].ctrlPnts().n()) ;
  d = new T[ca[0].ctrlPnts().n()] ; 
  for(i=0;i<N;i++){
    d[i] = 0 ;
    for(k=1;k<K;k++){
      d[i] += norm(ca[k].ctrlPnts(i)-ca[k-1].ctrlPnts(i)) ;
    }
  }

  uk[0] = 0 ;
  for(k=1;k<K;k++){
    uk[k] = 0 ;
    for(i=0;i<N;i++)
      uk[k] += norm(ca[k].ctrlPnts(i)-ca[k-1].ctrlPnts(i))/d[i] ;
    uk[k] /= (T)N ;//+ 1.0 ;
    uk[k] += uk[k-1] ;
  }
  uk[uk.n()-1] = 1.0 ;

  for(i=1;i<K-degU;i++){
    U[i+degU] = 0 ;
    for(k=i;k<i+degU;k++)
      U[i+degU] += uk[k] ;
    U[i+degU] /= (T)degU ;
  }
  for(i=0;i<=degU;i++) U[i] = 0.0 ;
  for(i=U.n()-degU-1;i<U.n();i++) U[i] = 1.0 ;


  Vector< HPoint_nD<T,nD> > Q(K) ;
  NurbsCurve<T,nD> i_curve ;


  for(i=0;i<N;i++){
    for(k=0;k<K;k++)
      Q[k] = ca[k].ctrlPnts(i) ;
    i_curve.globalInterpH(Q,uk,U,degU) ;
    for(k=0;k<K;k++)
      P(k,i) = i_curve.ctrlPnts(k) ;
  }

  V = ca[0].knot() ;

  delete []d ; 

  return 1 ;
}

/*! 
  \relates NurbsSurface
  \brief Computes the parameters for global surface interpolation

  Computes the parameters for global surface interpolation. 
  For more information, see A9.3 on p377 on the NURBS book.

  \param Q   the matrix of 3D points
  \param uk  the knot coefficients in the U direction
  \param vl  the knot coefficients in the V direction            

  \return 0 if an error occurs, 1 otherwise

  \warning 

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
int surfMeshParams(const Matrix< Point_nD<T,N> >& Q, Vector<T>& uk, Vector<T>& vl){
  int n,m,k,l,num ;
  double d,total ;
  //Vector<T> cds(Q.rows()) ;
  T* cds = new T[maximum(Q.rows(),Q.cols())] ; // alloca might not have enough space 

  n = Q.rows() ;
  m = Q.cols() ;
  uk.resize(n) ;
  vl.resize(m) ;
  num = m ;
  

  // Compute the uk
  uk.reset(0) ;

  for(l=0;l<m;l++){
    total = 0.0 ;
    for(k=1;k<n;k++){
      cds[k] = norm(Q(k,l)-Q(k-1,l)) ;
      total += cds[k] ;
    }
    if(total==0.0) 
      num-- ;
    else {
      d = 0.0 ;
      for(k=1;k<n;k++){
	d += cds[k] ;
	uk[k] += d/total ;
      }
    }
  }

  if(num==0) {
    delete []cds ; 
    return 0 ;
  }
  for(k=1;k<n-1;k++)
    uk[k] /= num ;
  uk[n-1] = 1.0 ;

  // Compute the vl
  vl.reset(0) ;

  //cds.resize(m) ; // this line removed since the maximum is allocated at the beginning

  num = n ;

  for(k=0;k<n;k++){
    total = 0.0 ;
    for(l=1;l<m;l++){
      cds[l] = norm(Q(k,l)-Q(k,l-1)) ;
      total += cds[l] ;
    }
    if(total==0.0) 
      num-- ;
    else {
      d = 0.0 ;
      for(l=1;l<m;l++){
	d += cds[l] ;
	vl[l] += d/total ;
      }
    }
  }

  delete []cds ; 

  if(num==0) 
    return 0 ;
  for(l=1;l<m-1;l++)
    vl[l] /= num ;
  vl[m-1] = 1.0 ;


  return 1 ;
}

/*! 
  \relates NurbsSurface
  \brief Computes the parameters for global surface interpolation

  Computes the parameters for global surface interpolation. 
  For more information, see A9.3 on p377 on the NURBS book.

  \param Q   the matrix of 3D points
  \param uk  the knot coefficients in the U direction
  \param vl  the knot coefficients in the V direction            

  \return 0 if an error occurs, 1 otherwise

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
int surfMeshParamsH(const Matrix< HPoint_nD<T,N> >& Q, Vector<T>& uk, Vector<T>& vl){
  int n,m,k,l,num ;
  double d,total ;
  //Vector<T> cds(Q.rows()) ;
  T* cds = new T[maximum(Q.rows(),Q.cols())] ;

  n = Q.rows() ;
  m = Q.cols() ;
  uk.resize(n) ;
  vl.resize(m) ;
  num = m ;
  

  // Compute the uk
  uk.reset(0) ;

  for(l=0;l<m;l++){
    total = 0.0 ;
    for(k=1;k<n;k++){
      cds[k] = norm(Q(k,l)-Q(k-1,l)) ;
      total += cds[k] ;
    }
    if(total==0.0) 
      num-- ;
    else {
      d = 0.0 ;
      for(k=1;k<n;k++){
	d += cds[k] ;
	uk[k] += d/total ;
      }
    }
  }

  if(num==0) {
    delete []cds ; 
    return 0 ;
  }
  for(k=1;k<n-1;k++)
    uk[k] /= num ;
  uk[n-1] = 1.0 ;

  // Compute the vl
  vl.reset(0) ;
  //cds.resize(m) ; // taking the maximum so this is not needed

  num = n ;

  for(k=0;k<n;k++){
    total = 0.0 ;
    for(l=1;l<m;l++){
      cds[l] = norm(Q(k,l)-Q(k,l-1)) ;
      total += cds[l] ;
    }
    if(total==0.0) 
      num-- ;
    else {
      d = 0.0 ;
      for(l=1;l<m;l++){
	d += cds[l] ;
	vl[l] += d/total ;
      }
    }
  }

  delete []cds ; 

  if(num==0) 
    return 0 ;
  for(l=1;l<m-1;l++)
    vl[l] /= num ;
  vl[m-1] = 1.0 ;
  return 1 ;
}

/*! 
  \brief Generates a surface using global interpolation

  \param    Q  a matrix of 3D points
  \param pU  the degree of interpolation in the U direction
  \param pV  the degree of interpolation in the V direction

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsSurface<T,N>::globalInterp(const Matrix< Point_nD<T,N> >& Q, int pU, int pV){
  Vector<T> vk,uk ;

  resize(Q.rows(),Q.cols(),pU,pV) ;

  surfMeshParams(Q,uk,vk) ;
  knotAveraging(uk,pU,U) ;
  knotAveraging(vk,pV,V) ;
  

  Vector< HPoint_nD<T,N> > Pts(Q.rows()) ;
  NurbsCurve<T,N> R ;
  
  int i,j ;

  for(j=0;j<Q.cols();j++){
    for(i=0;i<Q.rows();i++)
      Pts[i] = Q(i,j) ;
    R.globalInterpH(Pts,uk,U,pU);
    for(i=0;i<Q.rows();i++)
      P(i,j) = R.ctrlPnts(i) ;
  }

  Pts.resize(Q.cols()) ;
  for(i=0;i<Q.rows();i++){
    for(j=0;j<Q.cols();j++)
      Pts[j] = P(i,j) ;
    R.globalInterpH(Pts,vk,V,pV) ;
    for(j=0;j<Q.cols();j++)
      P(i,j) = R.ctrlPnts(j) ;
  }
}

/*! 
  \brief Generates a surface using global interpolation with homogenous points

  \param  Q  a matrix of 4D points
  \param pU  the degree of interpolation in the U direction
  \param pV  the degree of interpolation in the V direction

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsSurface<T,N>::globalInterpH(const Matrix< HPoint_nD<T,N> >& Q, int pU, int pV){
  Vector<T> vk,uk ;

  resize(Q.rows(),Q.cols(),pU,pV) ;

  surfMeshParamsH(Q,uk,vk) ;
  knotAveraging(uk,pU,U) ;
  knotAveraging(vk,pV,V) ;
  

  Vector< HPoint_nD<T,N> > Pts(Q.rows()) ;
  NurbsCurve<T,N> R ;
  
  int i,j ;

  for(j=0;j<Q.cols();j++){
    for(i=0;i<Q.rows();i++)
      Pts[i] = Q(i,j) ;
    R.globalInterpH(Pts,uk,U,pU);
    for(i=0;i<Q.rows();i++)
      P(i,j) = R.ctrlPnts(i) ;
  }

  Pts.resize(Q.cols()) ;
  for(i=0;i<Q.rows();i++){
    for(j=0;j<Q.cols();j++)
      Pts[j] = P(i,j) ;
    R.globalInterpH(Pts,vk,V,pV) ;
    for(j=0;j<Q.cols();j++)
      P(i,j) = R.ctrlPnts(j) ;
  }
}

/*! 
  \brief generates a surface using global least squares approximation.

  \param   Q  a matrix of 3D points
  \param  pU  the degree of interpolation in the U direction
  \param  pV  the degree of interpolation in the V direction
  \param  nU  the number of points in the U direction
  \param  nV  the number of poitns in the V direction

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsSurface<T,N>::leastSquares(const Matrix< Point_nD<T,N> >& Q, int pU, int pV, int nU, int nV){
  Vector<T> vk,uk ;

  resize(nU,nV,pU,pV) ;

  surfMeshParams(Q,uk,vk) ;

  Vector< HPoint_nD<T,N> > Pts(Q.rows()) ;
  NurbsCurve<T,N> R ;
  
  int i,j ;

  Matrix< HPoint_nD<T,N> > P2 ;

  P2.resize(nU,Q.cols()) ;

  for(j=0;j<Q.cols();j++){
    for(i=0;i<Q.rows();i++)
      Pts[i] = Q(i,j) ;
    R.leastSquaresH(Pts,pU,nU,uk);
    for(i=0;i<P.rows();i++)
      P2(i,j) = R.ctrlPnts(i) ;
    if(j==0)
      U = R.knot() ;
  }

  Pts.resize(Q.cols()) ;
  for(i=0;i<P.rows();i++){
    for(j=0;j<Q.cols();j++)
      Pts[j] = P2(i,j) ;
    R.leastSquaresH(Pts,pV,nV,vk) ;
    for(j=0;j<P.cols();j++)
      P(i,j) = R.ctrlPnts(j) ;
    if(i==0)
      V = R.knot() ;
  }
}

/*! 
  \relates NurbsSurface
  \brief Generates a surface using global interpolation

  
  Generates a NURBS surface using global interpolation. 
  The data points are assumed to be part of grided points 
  in x-y. \e i.e. the original data set as points covering
  the xy plane at regular \a x and \a y intervals with only the \a z
  being a free variable

  \param    Q a matrix of 3D points
  \param  pU  the degree of interpolation in the U direction
  \param pV  the degree of interpolation in the V direction
  \param S  the interpolated surface

  \warning Q(0,0) in x-y should be the smallest corner and 
               Q(Q.rows()-1,Q.cols()-1) the biggest.

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void globalSurfInterpXY(const Matrix< Point_nD<T,N> >& Q, int pU, int pV, NurbsSurface<T,N>& S) {
  Vector<T> uk,vk ;
  T um,uM ;
  T vm,vM ;

  um = Q(0,0).y() ;
  vm = Q(0,0).x() ;
  uM = Q(Q.rows()-1,0).y() ;
  vM = Q(0,Q.cols()-1).x() ;

  uk.resize(Q.rows()) ;
  vk.resize(Q.cols()) ;

  uk[0] = 0.0 ;
  vk[0] = 0.0 ;
  uk[uk.n()-1] = 1.0 ;
  vk[vk.n()-1] = 1.0 ;

  T dU = uM-um ;
  T dV = vM-vm ;

  int i ;
  for(i=1;i<uk.n()-1;++i){
    uk[i] = Q(i,0).y()/dU ;
  }
  
  for(i=1;i<vk.n()-1;++i){
    vk[i] = Q(0,i).x()/dV ;
  }
  
  globalSurfInterpXY(Q,pU,pV,S,uk,vk) ;
}

/*! 
  \relates NurbsSurface
  \brief generates a surface using global interpolation
  
  Generates a NURBS surface using global interpolation. 
  The data points are assumed to be part of grided points 
  in x-y. i.e. the original data set as points covering 
  the xy plane at regular \a x and \a y intervals with only the \a z 
  being a free variable

  \param  Q  a matrix of 3D points
  \param pU  the degree of interpolation in the U direction
  \param pV  the degree of interpolation in the V direction
  \param  S  the interpolated surface

  \warning  Q(0,0) in x-y should be the smallest corner and 
                Q(Q.rows()-1,Q.cols()-1) the biggest.

  \author    Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void globalSurfInterpXY(const Matrix< Point_nD<T,N> >& Q, int pU, int pV, NurbsSurface<T,N>& S, const Vector<T>& uk, const Vector<T>& vk){
  Vector<T> V,U ;
  int i,j ;


  knotAveraging(uk,pU,U) ;
  knotAveraging(vk,pV,V) ;
  
  Vector< HPoint_nD<T,N> > P(Q.rows()) ;
  NurbsCurve<T,N> R ;
  
  S.resize(Q.rows(),Q.cols(),pU,pV) ;
  S.U = U ;
  S.V = V ;

  for(j=0;j<Q.cols();j++){
    for(i=0;i<Q.rows();i++)
      P[i] = Q(i,j) ;
    R.globalInterpH(P,uk,U,pU) ;
    for(i=0;i<Q.rows();i++)
      S.P(i,j) = R.ctrlPnts(i) ;
  }

  P.resize(Q.cols()) ;
  for(i=0;i<Q.rows();i++){
    for(j=0;j<Q.cols();j++)
      P[j] = S.P(i,j) ;
    R.globalInterpH(P,vk,V,pV) ;
    for(j=0;j<Q.cols();j++)
      S.P(i,j) = R.ctrlPnts(j) ;
  }
}


/*! 
  \relates NurbsSurface
  \brief Generates a surface using global approximation

  \param  Q  a matrix of 3D points
  \param pU  the degree of interpolation in the U direction
  \param pV  the degree of interpolation in the V direction
  \param  S  the interpolated surface

  \warning This routine is still in a research phase

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void globalSurfApprox(const Matrix< Point_nD<T,N> >& Q, int pU, int pV, NurbsSurface<T,N>& S, double error){

  NurbsCurveArray<T,N> R ; 
  Vector< Point_nD<T,N> > P ;

  Matrix< HPoint_nD<T,N> > St ;
  Vector<T> Ut,Vt ;

  Vector<T> vk,uk ;

  surfMeshParams(Q,uk,vk) ;
  
  R.resize(Q.cols()) ;
  P.resize(Q.rows()) ;

  int i,j ;

  for(j=0;j<Q.cols();j++){
    for(i=0;i<Q.rows();i++)
      P[i] = Q(i,j) ;
    R[j].globalApproxErrBnd(P,uk,pU,error) ;
  }


  generateCompatibleCurves(R) ;

  Ut.resize(R[0].knot().n()) ;
  Ut = R[0].knot() ;

  St.resize(R[0].ctrlPnts().n(),R.n()) ;

  for(i=0;i<R[0].ctrlPnts().n();i++){
    for(j=0;j<R.n();j++)
      St(i,j) = R[j].ctrlPnts(i) ;
  }

  P.resize(St.cols()) ;
  R.resize(St.rows()) ;
  for(i=0;i<St.rows();i++){
    for(j=0;j<St.cols();j++)
      P[j] = project(St(i,j)) ;
    R[i].globalApproxErrBnd(P,vk,pV,error) ;
  }

  generateCompatibleCurves(R) ;

  Vt.resize(R[0].knot().n()) ;
  Vt = R[0].knot() ;
  
  S.resize(St.rows(),R[0].ctrlPnts().n(),pU,pV) ;
  for(i=0;i<S.ctrlPnts().rows();i++)
    for(j=0;j<S.ctrlPnts().cols();j++){
      S.P(i,j) = R[i].ctrlPnts(j) ;
    }
  S.U = Ut ;
  S.V = Vt ;
}


/*! 
  \brief Degree elevate the surface in the U and V direction

  \param tU  elevate the degree of the surface in the u direction 
	              by this amount.
  \param tV  elevate the degree of the surface in the v direction 
	              by this amount.

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsSurface<T,N>::degreeElevate(int tU, int tV) {
  degreeElevateU(tU) ;
  degreeElevateV(tV) ;
}

/*! 
  \brief Degree elevate the surface in the U direction

  \param t elevate the degree in the u direction by this amount.

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsSurface<T,N>::degreeElevateU(int t) {
  if(t<=0){
    return ;
  }

  NurbsSurface<T,N> S(*this) ;
  
  int i,j,k ;
  int n = S.ctrlPnts().rows()-1;
  int p = S.degU ;
  int m = n+p+1;
  int ph = p+t ;
  int ph2 = ph/2 ;
  Matrix<T> bezalfs(p+t+1,p+1) ; // coefficients for degree elevating the Bezier segment
  Matrix< HPoint_nD<T,N> > bpts(p+1,S.P.cols()) ; // pth-degree Bezier control points of the current segment
  Matrix< HPoint_nD<T,N> > ebpts(p+t+1,S.P.cols()) ; // (p+t)th-degree Bezier control points of the  current segment
  Matrix< HPoint_nD<T,N> > Nextbpts(p-1,S.P.cols()) ; // leftmost control points of the next Bezier segment
  Vector<T> alphas(p-1) ; // knot instertion alphas.
  
  // Compute the Binomial coefficients
  Matrix<T> Bin(ph+1,ph2+1) ;
  binomialCoef(Bin) ;
  
  // Compute Bezier degree elevation coefficients
  T inv,mpi ;
  bezalfs(0,0) = bezalfs(ph,p) = 1.0 ;
  for(i=1;i<=ph2;i++){
    inv= 1.0/Bin(ph,i) ;
    mpi = minimum(p,i) ;
    for(j=maximum(0,i-t); j<=mpi; j++){
      bezalfs(i,j) = inv*Bin(p,j)*Bin(t,i-j) ;
    }
  }
  
  for(i=ph2+1;i<ph ; i++){
    mpi = minimum(p,i) ;
    for(j=maximum(0,i-t); j<=mpi ; j++)
      bezalfs(i,j) = bezalfs(ph-i,p-j) ;
  }
  
  // Allocate more control points than necessary
  resize(S.P.rows()+S.P.rows()*t,S.P.cols(),ph,S.degV) ; 
  
  int colJ ;
  int mh = ph ;
  int kind = ph+1 ;
  T ua = S.U[0] ;
  T ub = 0.0 ;
  int r=-1 ; 
  int oldr ;
  int a = p ;
  int b = p+1 ; 
  int cind = 1 ;
  int rbz,lbz = 1 ; 
  int mul,save,s;
  T alf ;
  int first, last, kj ;
  T den,bet,gam,numer ;
  
  for(i=0;i<S.P.cols();i++)
    P(0,i) = S.P(0,i) ;
  for(i=0; i <= ph ; i++){
    U[i] = ua ;
  }
  
  // Initialize the first Bezier segment
  
  for(i=0;i<=p ;i++) 
    for(j=0;j<S.P.cols();j++)
      bpts(i,j) = S.P(i,j);
  
  while(b<m){ // Big loop thru knot vector
    i=b ;
    while(b<m && S.U[b] >= S.U[b+1]) // for some odd reasons... == doesn't work
      b++ ;
    mul = b-i+1 ; 
    mh += mul+t ;
    ub = S.U[b] ;
    oldr = r ;
    r = p-mul ;
    if(oldr>0)
      lbz = (oldr+2)/2 ;
    else
      lbz = 1 ;
    if(r>0) 
      rbz = ph-(r+1)/2 ;
    else
      rbz = ph ;
    if(r>0){ // Insert knot to get Bezier segment
      numer = ub-ua ;
      for(k=p;k>mul;k--){
	alphas[k-mul-1] = numer/(S.U[a+k]-ua) ;
      }
      for(j=1;j<=r;j++){
	save = r-j ; s = mul+j ;
	for(k=p;k>=s;k--){
	  for(colJ=0;colJ<S.P.cols();colJ++){
	    bpts(k,colJ) = alphas[k-s]*bpts(k,colJ)+(1.0-alphas[k-s])*bpts(k-1,colJ) ;}
	}
	if(Nextbpts.rows()>0)
	  for(colJ=0;colJ<S.P.cols();colJ++){
	    Nextbpts(save,colJ) = bpts(p,colJ) ;}
      }
    }
    
    for(i=lbz;i<=ph;i++){ // Degree elevate Bezier,  only the points lbz,...,ph are used
      for(colJ=0;colJ<S.P.cols();colJ++){
	ebpts(i,colJ) = 0.0 ;}
      mpi = minimum(p,i) ;
      for(j=maximum(0,i-t); j<=mpi ; j++)
	for(colJ=0;colJ<S.P.cols();colJ++){
	  ebpts(i,colJ) += bezalfs(i,j)*bpts(j,colJ) ;}
    }
    
    if(oldr>1){ // Must remove knot u=c.U[a] oldr times
      // if(oldr>2) // Alphas on the right do not change
      //	alfj = (ua-nc.U[kind-1])/(ub-nc.U[kind-1]) ;
      first = kind-2 ; last = kind ;
      den = ub-ua ;
      bet = (ub-U[kind-1])/den ;
      for(int tr=1; tr<oldr; tr++){ // Knot removal loop
	i = first ; j = last ;
	kj = j-kind+1 ;
	while(j-i>tr){ // Loop and compute the new control points for one removal step
	  if(i<cind){
	    alf=(ub-U[i])/(ua-U[i]) ;
	    for(colJ=0;colJ<S.P.cols();colJ++){
	      P(i,colJ) = alf*P(i,colJ) + (1.0-alf)*P(i-1,colJ) ;}
	  }
	  if( j>= lbz){
	    if(j-tr <= kind-ph+oldr){
	      gam = (ub-U[j-tr])/den ;
	      for(colJ=0;colJ<S.P.cols();colJ++){
		ebpts(kj,colJ) = gam*ebpts(kj,colJ) + (1.0-gam)*ebpts(kj+1,colJ) ;}
	    }
	    else{
	      for(colJ=0;colJ<S.P.cols();colJ++){
		ebpts(kj,colJ) = bet*ebpts(kj,colJ)+(1.0-bet)*ebpts(kj+1,colJ) ;}
	    }
	  }
	  ++i ; --j; --kj ;
	}
	--first ; ++last ;
      }
    }
  
    if(a!=p) // load the knot u=c.U[a]
      for(i=0;i<ph-oldr; i++){
	U[kind++] = ua ; 
      }
    
    for(j=lbz; j<=rbz ; j++) { // load control points onto nc
      for(colJ=0;colJ<S.P.cols();colJ++)
	P(cind,colJ) = ebpts(j,colJ) ; 
      ++cind ;
    }
    
    if(b<m){ // Set up for next pass thru loop
      for(colJ=0;colJ<S.P.cols();colJ++){
	for(j=0;j<r;j++)
	  bpts(j,colJ) = Nextbpts(j,colJ) ;
	for(j=r;j<=p;j++)
	  bpts(j,colJ) = S.P(b-p+j,colJ) ;
      }
      a=b ; 
      b++ ;
      ua = ub ;
    }
    else{
      for(i=0;i<=ph;i++)
	U[kind+i] = ub ;
    }
  }
  // Resize to the proper number of control points  
  resizeKeep(mh-ph,S.P.cols(),ph,S.degV) ;
}

/*! 
  \brief Degree elevate the surface in the V direction

  \param t  elevate the degree in the v direction by this amount.

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsSurface<T,N>::degreeElevateV(int t) {
  if(t<=0){
    return ;
  }

  NurbsSurface<T,N> S(*this) ;

  int i,j,k ;
  int n = S.ctrlPnts().cols()-1;
  int p = S.degV ;
  int m = n+p+1;
  int ph = p+t ;
  int ph2 = ph/2 ;
  Matrix<T> bezalfs(p+t+1,p+1) ; // coefficients for degree elevating the Bezier segment
  Matrix< HPoint_nD<T,N> > bpts(p+1,S.P.rows()) ; // pth-degree Bezier control points of the current segment
  Matrix< HPoint_nD<T,N> > ebpts(p+t+1,S.P.rows()) ; // (p+t)th-degree Bezier control points of the  current segment
  Matrix< HPoint_nD<T,N> > Nextbpts(p-1,S.P.rows()) ; // leftmost control points of the next Bezier segment
  //Vector<T> alphas(p-1) ; // knot instertion alphas.
  T* alphas = (T*) alloca((p-1)*sizeof(T)) ;
  
  // Compute the Binomial coefficients
  Matrix<T> Bin(ph+1,ph2+1) ;
  binomialCoef(Bin) ;
  
  // Compute Bezier degree elevation coefficients
  T inv,mpi ;
  bezalfs(0,0) = bezalfs(ph,p) = 1.0 ;
  for(i=1;i<=ph2;i++){
    inv= 1.0/Bin(ph,i) ;
    mpi = minimum(p,i) ;
    for(j=maximum(0,i-t); j<=mpi; j++){
      bezalfs(i,j) = inv*Bin(p,j)*Bin(t,i-j) ;
    }
  }
  
  for(i=ph2+1;i<ph ; i++){
    mpi = minimum(p,i) ;
    for(j=maximum(0,i-t); j<=mpi ; j++)
      bezalfs(i,j) = bezalfs(ph-i,p-j) ;
  }
  
  resize(S.P.rows(),S.P.cols()+S.P.cols()*t,S.degU,ph) ; // Allocate more control points than necessary
  
  int rowJ ;
  int mh = ph ;
  int kind = ph+1 ;
  T va = S.V[0] ;
  T vb = 0.0 ;
  int r=-1 ; 
  int oldr ;
  int a = p ;
  int b = p+1 ; 
  int cind = 1 ;
  int rbz,lbz = 1 ; 
  int mul,save,s;
  T alf ;
  int first, last, kj ;
  T den,bet,gam,numer ;
  
  for(i=0;i<S.P.rows();i++)
    P(i,0) = S.P(i,0) ;
  for(i=0; i <= ph ; i++){
    V[i] = va ;
  }
  
  // Initialize the first Bezier segment
  
  for(i=0;i<=p ;i++) 
    for(j=0;j<S.P.rows();j++)
      bpts(i,j) = S.P(j,i);
  
  while(b<m){ // Big loop thru knot vector
    i=b ;
    while(b<m && S.V[b] >= S.V[b+1]) // for some odd reasons... == doesn't work
      b++ ;
    mul = b-i+1 ; 
    mh += mul+t ;
    vb = S.V[b] ;
    oldr = r ;
    r = p-mul ;
    if(oldr>0)
      lbz = (oldr+2)/2 ;
    else
      lbz = 1 ;
    if(r>0) 
      rbz = ph-(r+1)/2 ;
    else
      rbz = ph ;
    if(r>0){ // Insert knot to get Bezier segment
      numer = vb-va ;
      for(k=p;k>mul;k--){
	alphas[k-mul-1] = numer/(S.V[a+k]-va) ;
      }
      for(j=1;j<=r;j++){
	save = r-j ; s = mul+j ;
	for(k=p;k>=s;k--){
	  for(rowJ=0;rowJ<S.P.rows();rowJ++){
	    bpts(k,rowJ) = alphas[k-s]*bpts(k,rowJ)+(1.0-alphas[k-s])*bpts(k-1,rowJ) ;}
	}
	if(Nextbpts.rows()>0)
	  for(rowJ=0;rowJ<S.P.rows();rowJ++){
	    Nextbpts(save,rowJ) = bpts(p,rowJ) ;}
      }
    }
    
    for(i=lbz;i<=ph;i++){ // Degree elevate Bezier,  only the points lbz,...,ph are used
      for(rowJ=0;rowJ<S.P.rows();rowJ++){
	ebpts(i,rowJ) = 0.0 ;}
      mpi = minimum(p,i) ;
      for(j=maximum(0,i-t); j<=mpi ; j++)
	for(rowJ=0;rowJ<S.P.rows();rowJ++){
	  ebpts(i,rowJ) += bezalfs(i,j)*bpts(j,rowJ) ;}
    }
    
    if(oldr>1){ // Must remove knot V=S.V[a] oldr times
      // if(oldr>2) // Alphas on the right do not change
      //	alfj = (va-nc.U[kind-1])/(vb-V[kind-1]) ;
      first = kind-2 ; last = kind ;
      den = vb-va ;
      bet = (vb-V[kind-1])/den ;
      for(int tr=1; tr<oldr; tr++){ // Knot removal loop
	i = first ; j = last ;
	kj = j-kind+1 ;
	while(j-i>tr){ // Loop and compute the new control points for one removal step
	  if(i<cind){
	    alf=(vb-V[i])/(va-V[i]) ;
	    for(rowJ=0;rowJ<S.P.rows();rowJ++){
	      P(rowJ,i) = alf*P(rowJ,i) + (1.0-alf)*P(rowJ,i-1) ;}
	  }
	  if( j>= lbz){
	    if(j-tr <= kind-ph+oldr){
	      gam = (vb-V[j-tr])/den ;
	      for(rowJ=0;rowJ<S.P.rows();rowJ++){
		ebpts(kj,rowJ) = gam*ebpts(kj,rowJ) + (1.0-gam)*ebpts(kj+1,rowJ) ;}
	    }
	    else{
	      for(rowJ=0;rowJ<S.P.rows();rowJ++){
		ebpts(kj,rowJ) = bet*ebpts(kj,rowJ)+(1.0-bet)*ebpts(kj+1,rowJ) ;}
	    }
	  }
	  ++i ; --j; --kj ;
	}
	--first ; ++last ;
      }
    }
  
    if(a!=p) // load the knot v=S.V[a]
      for(i=0;i<ph-oldr; i++){
	V[kind++] = va ; 
      }
    
    for(j=lbz; j<=rbz ; j++) { // load control points onto nc
      for(rowJ=0;rowJ<S.P.rows();rowJ++)
	P(rowJ,cind) = ebpts(j,rowJ) ; 
      ++cind ;
    }
    
    if(b<m){ // Set up for next pass thru loop
      for(rowJ=0;rowJ<S.P.rows();rowJ++){
	for(j=0;j<r;j++)
	  bpts(j,rowJ) = Nextbpts(j,rowJ) ;
	for(j=r;j<=p;j++)
	  bpts(j,rowJ) = S.P(rowJ,b-p+j) ;
      }
      a=b ; 
      b++ ;
      va = vb ;
    }
    else{
      for(i=0;i<=ph;i++)
	V[kind+i] = vb ;
    }
  }
  // Resize to the proper number of control points  
  resizeKeep(S.P.rows(),mh-ph,S.degU,ph) ; 
}
/*!
  \brief Finds the multiplicity of a knot in the U knot

  \param r the knot to observe

  \return the multiplicity of the knot
  \warning \a r must be a valid U knot index

  \author  Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
int NurbsSurface<T,N>::findMultU(int r) const {
  int s=1 ;
  for(int i=r;i>degU+1;i--)
    if(U[i]<=U[i-1])
      s++ ;
    else
      return s ;
  return s ;
}

/*!
  \brief finds the multiplicity of a knot in the V knot

  \param r  the knot to observe

  \return the multiplicity of the V knot

  \warning \a r must be a valid knot index

  \author  Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
int NurbsSurface<T,N>::findMultV(int r) const {
  int s=1 ;
  for(int i=r;i>degV+1;i--)
    if(V[i]<=V[i-1])
      s++ ;
    else
      return s ;
  return s ;
}




/*! 
  \brief finds the span in the U and V direction

  Finds the span in the U and V direction. The spanU is the index
  \a k for which the parameter \a u is valid in the \a [u_k,u_{k+1}]
  range. The spanV is the index \a k for which the parameter \a v is 
  valid in the \a [v_k,v_{k+1}] range.

  \param u  find the U span for this parametric value 
  \param v  find the V span for this parametric value
  \param spanU  the U span
  \param spanV  the V span

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsSurface<T,N>::findSpan(T u, T v, int& spanU, int& spanV) const{
  spanU = findSpanU(u) ;
  spanV = findSpanV(v) ;
}

/*! 
  \brief finds the span in the U direction

  Finds the span in the U direction. The span is the index
  \a k for which the parameter \a u is valid in the \a [u_k,u_{k+1}]
  range.

  \param u --> find the span for this parametric value

  \return the span for \a u

  \author    Philippe Lavoie
  \date 24 January, 1997

  \modified 20 January, 1999 (Alejandro Frangi)
*/
template <class T, int N>
int NurbsSurface<T,N>::findSpanU(T u) const{
  if(u>=U[P.rows()]) 
    return P.rows()-1 ;
  if(u<=U[degU])
    return degU ;

  //AF
  int low = 0 ;
  int high = P.rows()+1 ; 
  int mid = (low+high)/2 ;

  while(u<U[mid] || u>= U[mid+1]){
    if(u<U[mid])
      high = mid ;
    else
      low = mid ;
    mid = (low+high)/2 ;
  }
  return mid ;  
}

/*! 
  \brief finds the span in the V direction

  Finds the span in the V direction. The span is the index
  \a k for which the parameter \a v is valid in the \a [v_k,v_{k+1}]
  range. 

  \param v  find the span for this parametric value    

  \return the span for \a v

  \author Philippe Lavoie
  \date 24 January, 1997
  \modified 20 January, 1999 (Alejandro Frangi)

*/
template <class T, int N>
int NurbsSurface<T,N>::findSpanV(T v) const{
  if(v>=V[P.cols()]) 
    return P.cols()-1 ;
  if(v<=V[degV])
    return degV ;

  //AF
  int low  = 0 ;
  int high = P.cols()+1 ; 
  int mid = (low+high)/2 ;

  while(v<V[mid] || v>= V[mid+1]){
    if(v<V[mid])
      high = mid ;
    else
      low = mid ;
    mid = (low+high)/2 ;
  }
  return mid ;
}

/*! 
  \brief Find the non-zero basis functions in the U and V direction

  \param   u  the u parametric value
  \param   v  the v parametric value 
  \param spanU  the span of u
  \param spanV  the span of v
  \param    Nu  the vector containing the U non-zero basis functions
  \param    Nv  the vector containing the V non-zero basis functions

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsSurface<T,N>::basisFuns(T u, T v, int spanU, int spanV, Vector<T>& Nu, Vector<T> &Nv) const{
  basisFunsU(u,spanU,Nu) ;
  basisFunsV(v,spanV,Nv) ;
}

/*! 
  \brief Finds the non-zero basis function in the U direction

  \param   u  the u parametric value
  \param span  the span of u
  \param    N  the vector containing the basis functions

  \author    Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int nD>
void NurbsSurface<T,nD>::basisFunsU(T u, int span, Vector<T>& N) const {
  //Vector<T> left(degU+1), right(degU+1) ;
  T* left = (T*) alloca((degU+1)*sizeof(T)) ;
  T* right = (T*) alloca((degU+1)*sizeof(T)) ;
  T temp,saved ;
   

  N.resize(degU+1) ;

  N[0] = 1.0 ;
  for(int j=1; j<= degU ; j++){
    left[j] = u-U[span+1-j] ;
    right[j] = U[span+j]-u ;
    saved = 0.0 ;
    for(int r=0 ; r<j; r++){
      temp = N[r]/(right[r+1]+left[j-r]) ;
      N[r] = saved+right[r+1]*temp ;
      saved = left[j-r]*temp ;
    }
    N[j] = saved ;
  }  

}

/*! 
  \brief Finds the non-zero basis function in the V direction

  \param    v  the v parametric value
  \param span  the span of v 
  \param    N  the vector containing the basis functions

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int nD>
void NurbsSurface<T,nD>::basisFunsV(T v, int span, Vector<T>& N) const {
  //Vector<T> left(degV+1), right(degV+1) ;
  T* left = (T*) alloca((degV+1)*sizeof(T)) ;
  T* right = (T*) alloca((degV+1)*sizeof(T)) ;
  T temp,saved ;
   

  N.resize(degV+1) ;

  N[0] = 1.0 ;
  for(int j=1; j<= degV ; j++){
    left[j] = v-V[span+1-j] ;
    right[j] = V[span+j]-v ;
    saved = 0.0 ;
    for(int r=0 ; r<j; r++){
      temp = N[r]/(right[r+1]+left[j-r]) ;
      N[r] = saved+right[r+1]*temp ;
      saved = left[j-r]*temp ;
    }
    N[j] = saved ;
  }  

}

/*! 
  \brief computes the point and the derivatives of degree 
         \a d and below at \a (u,v)

  Computes the matrix of derivatives at \a u,v . 
  The value of skl(k,l) represents the 
  derivative of the surface \a S(u,v) with respect to 
  \a u \a k times and to $v$ $l$ times.

  \param  u  the u parametric value
  \param  v  the v parametric value
  \param  d  the derivative is computed up to and including to this value
  \param skl  the matrix containing the derivatives

  \author    Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int nD>
void NurbsSurface<T,nD>::deriveAtH(T u, T v, int d, Matrix< HPoint_nD<T,nD> > &skl) const {
  int k,l,du,dv;
  skl.resize(d+1,d+1) ;

  du = minimum(d,degU) ;
  for(k=degU+1;k<=d;++k)
    for(l=0;l<=d-k;++l)
      skl(k,l) = 0.0 ;
  dv=minimum(d,degV) ;
  for(l=degV+1;l<=d;++l)
    for(k=0;k<=d-l;++k)
      skl(k,l) = 0.0 ;
  int uspan = findSpanU(u) ;
  int vspan = findSpanV(v) ;
  Matrix<T> Nu,Nv ;
  nurbsDersBasisFuns(du,u,uspan,degU,U,Nu) ;
  nurbsDersBasisFuns(dv,v,vspan,degV,V,Nv) ;

  Vector< HPoint_nD<T,nD> > temp(degV+1) ;
  int dd,r,s ;
  for(k=0;k<=du;++k){
    for(s=0;s<=degV;++s){
      temp[s] = 0.0 ;
      for(r=0;r<=degU;++r)
	temp[s] += Nu(k,r)*P(uspan-degU+r,vspan-degV+s) ;
    }
    dd = minimum(d-k,dv) ;
    for(l=0;l<=dd;++l){
      skl(k,l) = 0.0 ;
      for(s=0;s<=degV;++s)
	skl(k,l) += Nv(l,s)*temp[s] ;
    }
  }
}

/*! 
  \brief Computes the point and the derivatives of degree 
         \a d and below at \a (u,v)

  Computes the matrix of derivatives at \a u,v . 
  The value of skl(k,l) represents the 
  derivative of the surface \a S(u,v) with respect to 
  \a u, \a k times and to \a v, \a l times.

  \param   u  the u parametric value
  \param   v  the v parametric value
  \param   d  the derivative is computed up to and including to  this value
  \param skl  the matrix containing the derivatives

  \author    Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsSurface<T,N>::deriveAt(T u, T v, int d, Matrix< Point_nD<T,N> > &skl) const {
  int k,l ;
  Matrix< HPoint_nD<T,N> > ders ;
  Point_nD<T,N> pv,pv2 ;
  
  skl.resize(d+1,d+1) ;

  deriveAtH(u,v,d,ders) ;
  
  Matrix<T> Bin(d+1,d+1) ;
  binomialCoef(Bin) ;
  int i,j ; 

  for(k=0;k<=d;++k){
    for(l=0;l<=d-k;++l){
      pv.x() = ders(k,l).x() ;
      pv.y() = ders(k,l).y() ;
      pv.z() = ders(k,l).z() ;
      for(j=1;j<=l;j++)
	pv -= Bin(l,j)*ders(0,j).w()*skl(k,l-j) ;
      for(i=1;i<=k;i++){
	pv -= Bin(k,i)*ders(i,0).w()*skl(k-i,l) ;
	pv2 = 0.0 ;
	for(j=1;j<=l;j++)
	  pv2 += Bin(l,j)*ders(i,j).w()*skl(k-i,l-j) ;
	pv -= Bin(k,i)*pv2 ;
      }
      skl(k,l) = pv/ders(0,0).w() ;
    }
  }
}

/*! 
  \brief Computes the normal of the surface at \a (u,v)

  \param  u  the u parametric value
  \param  v  the v parametric value

  \return the normal at \a (u,v) .

  \author    Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
Point_nD<T,N> NurbsSurface<T,N>::normal(T u, T v) const {
  Matrix< Point_nD<T,N> > ders ;

  deriveAt(u,v,1,ders) ;

  return crossProduct(ders(1,0),ders(0,1)) ;
}

/*! 
  \brief Returns the point on the surface at \a u,v

  Returns the point on the surface at \a u,v

  \param u  the u parametric value
  \param v  the v parametric value

  \return The homogenous point at \a u,v

  \author    Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
HPoint_nD<T,N> NurbsSurface<T,N>::operator()(T u, T v) const{
  int uspan = findSpanU(u) ;
  int vspan = findSpanV(v) ;
  Vector<T> Nu,Nv ;
  Vector< HPoint_nD<T,N> > temp(degV+1)  ;

  basisFuns(u,v,uspan,vspan,Nu,Nv) ;

  int l;
  for(l=0;l<=degV;l++){
    temp[l] =0.0 ;
    for(int k=0;k<=degU;k++){
      temp[l] += Nu[k]*P(uspan-degU+k,vspan-degV+l) ;
    }
  }
  HPoint_nD<T,N> sp(0,0,0,0) ;
  for(l=0;l<=degV;l++){
    sp += Nv[l]*temp[l] ;
  }
  return sp ; 
}

inline
int max3(int a,int b, int c){
  int m = a ;
  if(m <b)
    m = b ;
  if(m <c)
    m = c ;
  return m ;
}

/*! 
  \relates NurbsSurface
  \brief Interpolation of a surface from 2 sets of orthogonal curves
  
  Interpolation of a surface from 2 sets of orthogonal curves. 
  See A10.3 at page 494 on the NURBS book for more details about
  the implementation.

  \param lU  an array of curves in the U direction
  \param lV  an array of curves in the V direction
  \param intersections  a matrix of 3D points specifying where the 
                the curves intersect
  \param gS  the Gordon surface

  \author    Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void gordonSurface(NurbsCurveArray<T,N>& lU, NurbsCurveArray<T,N>& lV, const Matrix< Point_nD<T,N> >& intersections, NurbsSurface<T,N>& gS){
  NurbsSurface<T,N> sU,sV,sI ;
  sU.skinU(lU,3) ;
  sV.skinV(lV,3) ;
  sI.globalInterp(intersections,3,3) ;

  int du = max3(sU.degU,sV.degU,sI.degU) ;
  int dv = max3(sU.degV,sV.degV,sI.degV) ;
  

  NurbsSurface<T,N> SU,SV,SI ;
  degreeElevate(sU,du-sU.degU,dv-sU.degV,SU) ;
  degreeElevate(sV,du-sV.degU,dv-sV.degV,SV) ;
  degreeElevate(sI,du-sI.degU,dv-sI.degV,SI) ;


  Vector<T> U,V ;
  U = knotUnion(SU.knotU(),SV.knotU()) ;
  U = knotUnion(U,SI.knotU()) ;
  V = knotUnion(SU.knotV(),SV.knotV()) ;
  V = knotUnion(V,SI.knotV()) ;

  SU.mergeKnots(U,V) ;
  SV.mergeKnots(U,V) ;
  SI.mergeKnots(U,V) ;
  
  gS = SU ;

  for(int i=0;i<gS.P.rows();i++)
    for(int j=0;j<gS.P.cols();j++){
      gS.P(i,j) += SV.P(i,j) ;
      gS.P(i,j) -= SI.P(i,j) ;
    }
}

template <class T>
inline void insert(T u, Vector<T>& v){
  int i ;
  if(u<v[0] || u>v[v.n()-1])
    return ;
  v.resize(v.n()+1) ;
  i = v.n()-1 ;
  while(v[i-1]>u){
    v[i] = v[i-1] ;
    --i ;
  }
  v[i] = u ;
}


/*! 
  \brief Generates a surface by sweeping a curve along a trajectory
  
  Sweeping consists of creating a surface by moving a curve
  profile through a trajectory. The method uses here consists of 
  using skinning of \a K instances of the curve \a C(u) along 
  \a T(u). The \a K value should be viewed as the minimum number of 
  sections required.
  
  The profile curve \a C(u) should lie on the xz-plane.
  It follows the trajectory curve \a T(u) along its y-axis.
  
  The scaling function is used to modify the shape of the curve
  profile while it's being swept. It can scale in any of the
  4 dimensions to obtain the desired effects on the profile.
  
  See A10.1 on page 476 of the NURBS book for more details 
  about the implementation.
  
  You might have to play with the useAy and invAz variables
  to obtain a satisfactory result for the sweep operation.
  This is either because there is an error in the code or 
  because it is the way it is supposed to work.

  \param   T  the trajectory of the curve 
  \param  C  the curve profile to sweep
  \param  Sv  a scaling function
  \param  K  the minimum number of insertion
  \param  useAy  a 0 indicates that rotation angle around the $y$-axis 
	            is not computed, otherwise it is computed. Results
		    are usually better with a value of 0.
  \param  invAz  a 1 indicates that the computed rotation around the
	               $z$-axis should be inversed, a 0 indicates it
		       stays as it is.
  \return the NURBS surface representing the sweep of $C$ along $T$
  \warning This will not yield a correct value for a closed trajectory 
           curve.

  \author Philippe Lavoie
  \date 25 July, 1997
*/
template <class T, int N>
void NurbsSurface<T,N>::sweep(const NurbsCurve<T,N>& Trj, const NurbsCurve<T,N>& C, const NurbsCurve<T,N>& Sv, int K, int useAy, int invAz) {
  int i,j,k,m,q,ktv,nsect ;
  q = Trj.degree() ;
  ktv = Trj.knot().n() ;
  nsect = K  ;
  
  V.resize(Trj.knot().n()) ;
  V = Trj.knot() ;

  if(ktv <= nsect+q){
    m = nsect+q-ktv+1 ;
    // insert m knots into T(v)
    // locations are not critical so inserting in the middle of the 
    // biggest span is used.
    for(i=0;i<m;++i){
      T md,mt ;
      T mu = -1;
      md = 0 ;
      for(j=1;j<V.n();++j){
	mt = V[j]-V[j-1] ;
	if(mt>md){
	  md = mt ;
	  mu = (V[j]+V[j-1])/2.0 ;
	}
      }
      insert(mu,V) ;
    }
  }
  else{
    if(ktv>nsect){ // must increase the number of instances of C(u)
      nsect = ktv-q-1 ; 
    }
  }

  Vector<T> v; 

  // Compute the parameters by averaging the knots
  v.resize(nsect) ;
  v[0] = Trj.knot(Trj.degree()) ;
  v[nsect-1] = Trj.knot(Trj.knot().n()-Trj.degree()-1) ;
  for(k=1;k<nsect-1;++k){ 
    v[k] = 0.0 ;
    for(i=k+1;i<k+q+1;++i) 
      v[k] += V[i] ;
    v[k] /= q ;
  }

  resize(C.ctrlPnts().n(),nsect,C.degree(),Trj.degree()) ;

  U = C.knot() ;

  // setup Bv ;
  Vector< Point_nD<T,N> > B ;
  NurbsCurve<T,N> Bv ;
  Vector< Point_nD<T,N> > Td ;

  B.resize(v.n()) ;
  Trj.deriveAt(v[0],1,Td) ;

  B[0] = Td[1] ;
  if(Td[1].y() ==0)
    B[0] = crossProduct(Point_nD<T,N>(0,1,0),B[0]) ;
  else
    B[0] = crossProduct(Point_nD<T,N>(1,0,0),B[0]) ;
  B[0] /= norm(B[0]) ;


  for(i=1;i<v.n();++i){
    Trj.deriveAt(v[i],1,Td);
    Point_nD<T,N> Ti(Td[1]) ;
    Ti = Ti/norm(Ti) ;
    Point_nD<T,N> bi ;
    bi = B[i-1]-(B[i-1]*Ti)*Ti ;
    B[i] = bi/norm(bi) ;
  }

  Bv.globalInterp(B,v,minimum(3,B.n()-1)) ;

  Vector< HPoint_nD<T,N> > Q(C.ctrlPnts().n()) ; 
  for(k=0;k<nsect;++k){
    // scale the control points by Sv(v[k])
    for(i=0;i<Q.n();++i){
      HPoint_nD<T,N> sk(Sv(v[k])) ;
      Q[i].x() = sk.x()*C.ctrlPnts(i).x() ;
      Q[i].y() = sk.y()*C.ctrlPnts(i).y() ;
      Q[i].z() = sk.z()*C.ctrlPnts(i).z() ;
      Q[i].w() = sk.w()*C.ctrlPnts(i).w() ;
    }
    // compute o(v[k])
    Point_nD<T,N> o = Trj.pointAt(v[k]) ;
    //T w = T(v[k]).w() ;

    // compute x(v[k])
    Trj.deriveAt(v[k],1,Td) ;

    Point_nD<T,N> x = Td[1]/norm(Td[1]) ;

    // compute z(v[k])
    Point_nD<T,N> z = Bv.pointAt(v[k]) ; 
    z /= norm(z) ;

    // compute y(v[k]) 
    Point_nD<T,N> y = crossProduct(z,x) ;

    /*
    // compute the transform matrix
    double az = M_PI+atan2(y.y(),y.x()) ;
    double ax = -atan2(z.y(),z.z()) ;
    double ay = 0 ;
    if(useAy){
      ay = atan2(x.z(),x.x()) ;
    }
    if(invAz){
      az = -1.0*az ;
    }
    MatrixRT_DOUBLE A(ax,ay,az,o.x(),o.y(),o.z()) ;
    */

    MatrixRT_DOUBLE R ; // M(4,4)
    R(0,0) = y.x();
    R(1,0) = y.y();
    R(2,0) = y.z();
    R(0,1) = x.x();
    R(1,1) = x.y();
    R(2,1) = x.z();
    R(0,2) = z.x();
    R(1,2) = z.y();
    R(2,2) = z.z();
    //R(3,3) = 1.0; 

    MatrixRT_DOUBLE Tx ;
    Tx.translate(o.x(),o.y(),o.z());
    //MatrixRT_DOUBLE R(M) ;
    MatrixRT_DOUBLE A ;
    A = Tx * R ;

    for(i=0;i<Q.n();++i){
      P(i,k) = A*(Q[i]) ;
    }
  }

  // interpolate along V
  Q.resize(P.cols()) ;
  NurbsCurve<T,N> R;
  for(i=0;i<P.rows();++i){
    for(k=0;k<P.cols();++k)
      Q[k] = P(i,k) ;
    R.globalInterpH(Q,v,V,degV) ;
    for(k=0;k<P.cols();++k)
      P(i,k) = R.ctrlPnts(k) ;
  }
  
}

/*! 
  \brief Generates a surface by sweeping a curve along a  trajectory

  Sweeping consists of creating a surface by moving a curve
  through a trajectory. The method uses here consists of using
  skinning of $K$ instances of the curve \a C(u) along \a T(u).
  The \a K value should be viewed as a minimum.

  The cross-sectional curve \a C(u) should lie on the xz-plane.
  It follows the trajectory curve \a T(u) along its y-axis.
  
  You might have to play with the useAy and invAz variables
  to obtain a satisfactory result for the sweep operation.
  This is either because there is an error in the code or 
  because it is the way it is supposed to work.

  \param T  the trajectory of the curve 
  \param C  the curve to sweep
  \param K  the minimum number of insertion
  \param useAy  a 0 indicates that rotation angle around the y-axis 
	            is not computed, otherwise it is computed. 
  \param invAz  a 1 indicates that the computed rotation around the
	            z-axis should be inversed, a 0 indicates it
		    stays as it is.

  \return the NURBS surface representing the sweep of \a C along \a T
  \warning This will not yield a correct value for a closed trajectory 
           curve.

  \author Philippe Lavoie
  \date 25 July, 1997
*/
template <class T, int N>
void NurbsSurface<T,N>::sweep(const NurbsCurve<T,N>& Trj, const NurbsCurve<T,N>& C, int K, int useAy,int invAz) {
  // setup Sv 
  Vector< HPoint_nD<T,N> > p(2) ;
  p[0] = HPoint_nD<T,N>(1,1,1,1) ;
  p[1] = HPoint_nD<T,N>(1,1,1,1) ;
  Vector<T> u(4) ;
  u[0] = u[1] = 0.0 ; u[2] = u[3] = 1.0 ;

  NurbsCurve<T,N> Sv(p,u,1) ;
  
  sweep(Trj,C,Sv,K,useAy,invAz) ;
}

/*!
  \brief Performs geometrical modifications

  Each control points will be modified by a rotation-translation
  matrix.
	      
  \param A  the rotation-translation matrix

  \author Philippe Lavoie
  \date 22 August 1997
*/
template <class T, int N>
void NurbsSurface<T,N>::transform(const MatrixRT<T>& A){
  for(int i=0;i<P.rows();++i)
    for(int j=0;j<P.cols();++j)
      P(i,j) = A*P(i,j) ;
}

/*! 
  \brief Refine both knot vectors

  \param nU  the U knot vector to refine from 
  \param nV  the V knot vector to refine from 

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsSurface<T,N>::refineKnots(const Vector<T>& nU, const Vector<T>& nV){
  refineKnotU(nU) ;
  refineKnotV(nV) ;
}

/*! 
  \brief  Refines the U knot vector

  \param X  the knot vector to refine from

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsSurface<T,N>::refineKnotU(const Vector<T>& X) {
  if(X.n()<=0)
    return ;
  int n = P.rows()-1 ;
  int p = degU;
  int m = n+p+1 ;
  int a,b ;
  int r = X.n()-1 ;
  NurbsSurface<T,N> nS ;
  
  nS = *this ;
  nS.resize(r+1+n+1,P.cols(),degU,degV) ;

  a = findSpanU(X[0]) ;
  b = findSpanU(X[r]) ;
  ++b ;

  int j,col ;
  for(col=0;col<P.cols();col++){
    for(j=0; j<=a-p ; j++)
      nS.P(j,col) = P(j,col);
    for(j = b-1 ; j<=n ; j++)
      nS.P(j+r+1,col) = P(j,col) ;
  }
  for(j=0; j<=a ; j++)
    nS.U[j] = U[j] ;
  for(j=b+p ; j<=m ; j++)
    nS.U[j+r+1] = U[j] ;
  int i = b+p-1 ; 
  int k = b+p+r ;
  for(j=r; j>=0 ; j--){
    while(X[j] <= U[i] && i>a){
      for(col=0;col<P.cols();col++)
	nS.P(k-p-1,col) = P(i-p-1,col) ;
      nS.U[k] = U[i] ;
      --k ;
      --i ;
    }
    for(col=0;col<P.cols();col++)
	nS.P(k-p-1,col) = nS.P(k-p,col) ;
    for(int l=1; l<=p ; l++){
      int ind = k-p+l ;
      T alpha = nS.U[k+l] - X[j] ;
      if(alpha==0.0)
	for(col=0;col<P.cols();col++)
	  nS.P(ind-1,col) = nS.P(ind,col) ;
      else
	alpha /= nS.U[k+l]-U[i-p+l] ;
      for(col=0;col<P.cols();col++)
	nS.P(ind-1,col) = alpha*nS.P(ind-1,col) + (1.0-alpha)*nS.P(ind,col) ;
    }
    nS.U[k] = X[j] ;
    --k ;
  }
  *this = nS ;
}

/*! 
  \brief Refines the V knot vector

  \param X  the knot vector to refine from 

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsSurface<T,N>::refineKnotV(const Vector<T>& X) {
  if(X.n()<=0)
    return ;
  int n = P.cols()-1 ;
  int p = degV;
  int m = n+p+1 ;
  int a,b ;
  int r = X.n()-1 ;
  NurbsSurface<T,N> nS ;
  
  try {
    nS = *this ;
    nS.resize(P.rows(),r+1+n+1,degU,degV) ;
  }
  catch(...){
    cerr << "Out of memory\n" ; 
  }

  a = findSpanV(X[0]) ;
  b = findSpanV(X[r]) ;
  ++b ;

  int j,row ;
  for(row=0;row<P.rows();row++){
    for(j=0; j<=a-p ; j++)
      nS.P(row,j) = P(row,j);
    for(j = b-1 ; j<=n ; j++)
      nS.P(row,j+r+1) = P(row,j) ;
  }
  for(j=0; j<=a ; j++)
    nS.V[j] = V[j] ;
  for(j=b+p ; j<=m ; j++)
    nS.V[j+r+1] = V[j] ;
  int i = b+p-1 ; 
  int k = b+p+r ;
  for(j=r; j>=0 ; j--){
    while(X[j] <= V[i] && i>a){
      for(row=0;row<P.rows();row++)
	nS.P(row,k-p-1) = P(row,i-p-1) ;
      nS.V[k] = V[i] ;
      --k ;
      --i ;
    }
    for(row=0;row<P.rows();row++)
	nS.P(row,k-p-1) = nS.P(row,k-p) ;
    for(int l=1; l<=p ; l++){
      int ind = k-p+l ;
      T alpha = nS.V[k+l] - X[j] ;
      if(alpha==0.0)
	for(row=0;row<P.rows();row++)
	  nS.P(row,ind-1) = nS.P(row,ind) ;
      else
	alpha /= nS.V[k+l]-V[i-p+l] ;
      for(row=0;row<P.rows();row++)
	nS.P(row,ind-1) = alpha*nS.P(row,ind-1) + (1.0-alpha)*nS.P(row,ind) ;
    }
    nS.V[k] = X[j] ;
    --k ;
  }
  *this = nS ;
}


/*! 
  \brief merges a U and V knot vector with the surface knot vectors

  \param  nU  the U knot vector to merge with
  \param  nV  the V knot vector to merge with

  \warning  The nU and nV knot vectors must be compatible with the 
                current vectors

  \author    Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsSurface<T,N>::mergeKnots(const Vector<T>& nU, const Vector<T>& nV) {
  mergeKnotU(nU) ;
  mergeKnotV(nV) ;
}

/*! 
  \brief merges the U knot vector with another one

  \param X  a knot vector

  \warning The knot vector must be compatible with the U knot vector

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsSurface<T,N>::mergeKnotU(const Vector<T>& X){
  int i,ia,ib ;
  // Find the knots to insert
  Vector<T> I(U.n()) ;
  int done = 0 ;
  i = ia = ib = 0 ;
  while(!done) {
    if(X[ib] == U[ia]){
      ++ib ; ++ia ;
    }
    else{
      I[i++] = X[ib] ;
      ib++ ;
    }
    done = (ia>=U.n() || ib >= X.n()) ;
  }
  I.resize(i) ;

  if(I.n()>0)
    refineKnotU(I) ;
}

/*! 
  \brief merges the V knot vector with another one

  \param X  a knot vector

  \warning The knot vector must be compatible with the V knot vector

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsSurface<T,N>::mergeKnotV(const Vector<T>& X){
  int i,ia,ib ;
  // Find the knots to insert
  Vector<T> I(V.n()) ;
  int done = 0 ;
  i = ia = ib = 0 ;
  while(!done) {
    if(X[ib] == V[ia]){
      ++ib ; ++ia ;
    }
    else{
      I[i++] = X[ib] ;
      ib++ ;
    }
    done = (ia>=V.n() || ib >= X.n()) ;
  }
  I.resize(i) ;

  if(I.n()>0)
    refineKnotV(I) ;
}

/*! 
  \brief Read a surface from an input stream

  \param fin  the input file stream

  \return 0 if an error occurs, 1 otherwise

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
int NurbsSurface<T,N>::read(ifstream &fin){
  if(!fin) {
    return 0 ;
  }
  int nu,nv,du,dv;
  char *type ;
  type = new char[3] ;
  if(!fin.read(type,sizeof(char)*3)) { delete []type ; return 0 ;}
  int r1 = strncmp(type,"ns3",3) ;
  int r2 = strncmp(type,"ns4",3) ;
  if(!(r1 || r2)) 
    return 0 ;
  int st ;
  char stc ;
  if(!fin.read((char*)&stc,sizeof(char))) { delete []type ; return 0 ;}
  if(!fin.read((char*)&nu,sizeof(int))) { delete []type ; return 0 ;}
  if(!fin.read((char*)&nv,sizeof(int))) { delete []type ; return 0 ;}
  if(!fin.read((char*)&du,sizeof(int))) { delete []type ; return 0 ;}
  if(!fin.read((char*)&dv,sizeof(int))) { delete []type ; return 0 ;}

  st = stc - '0' ; 
  if(st != sizeof(T)){ // not of the same type size
    delete []type ;
    return 0 ; 
  }

  resize(nu,nv,du,dv) ;
  
  if(!fin.read((char*)U.memory(),sizeof(T)*U.n())) { delete []type ; return 0 ;}
  if(!fin.read((char*)V.memory(),sizeof(T)*V.n())) { delete []type ; return 0 ;}
     

  T *p,*p2 ;
  if(!r1){
    p = new T[3*nu*nv] ;
    if(!fin.read((char*)p,sizeof(T)*3*nu*nv)) { delete []type ; return 0 ;}
    p2 = p ;
    for(int i=0;i<nu;i++)
      for(int j=0;j<nv;j++){
	P(i,j).x() = *(p++) ;
	P(i,j).y() = *(p++) ;
	P(i,j).z() = *(p++) ;
	P(i,j).w() = 1.0 ;
      }
    delete []p2 ;
  }
  else{
    p = new T[4*nu*nv] ;
    if(!fin.read((char*)p,sizeof(T)*4*nu*nv)) { delete []type ; return 0 ;}
    p2 = p ;
    for(int i=0;i<nu;i++)
      for(int j=0;j<nv;j++){
	P(i,j).x() = *(p++) ;
	P(i,j).y() = *(p++) ;
	P(i,j).z() = *(p++) ;
	P(i,j).w() = *(p++) ;
      }
    delete []p2 ;
  }

  delete []type ;
  return 1 ;
}

/*! 
  \brief read a surface from a file

  \param filename  the filename to read from 

  \return 0 if an error occurs, 1 otherwise

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
int NurbsSurface<T,N>::read(const char* filename){
  ifstream fin(filename) ;
  if(!fin) {
    return 0 ;
  }

  return read(fin) ;
}

/*! 
  \brief Write a surface to a file stream

  \param fout  the output filestream to write to.

  \return 1 on success, 0 on failure

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
int NurbsSurface<T,N>::write(ofstream &fout) const {
  if(!fout)
    return 0 ;
  int prows = P.rows();
  int pcols = P.cols();
  char st = '0' + sizeof(T) ;
  if(!fout.write((char*)&"ns4",sizeof(char)*3)) return 0 ;
  if(!fout.write((char*)&st,sizeof(char))) return 0 ; 
  if(!fout.write((char*)&prows,sizeof(int))) return 0 ;
  if(!fout.write((char*)&pcols,sizeof(int))) return 0 ;
  if(!fout.write((char*)&degU,sizeof(int))) return 0 ;
  if(!fout.write((char*)&degV,sizeof(int))) return 0 ;
  if(!fout.write((char*)U.memory(),sizeof(T)*U.n())) return 0 ;
  if(!fout.write((char*)V.memory(),sizeof(T)*V.n())) return 0 ;
  
  T *p,*p2 ;
  p = new T[P.rows()*P.cols()*4] ;
  p2 = p ;
  for(int i=0;i<P.rows();i++) 
    for(int j=0;j<P.cols();j++){
      *p = P(i,j).x() ; p++ ;
      *p = P(i,j).y() ; p++ ;
      *p = P(i,j).z() ; p++ ;
      *p = P(i,j).w() ; p++ ;
    }
  if(!fout.write((char*)p2,sizeof(T)*P.rows()*P.cols()*4)) return 0 ;
  delete []p2 ;
  return 1 ;
}

/*! 
  \brief Write a surface to a file

  \param filename  the filename to write to

  \return 1 on success, 0 on failure

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
int NurbsSurface<T,N>::write(const char* filename) const {
  ofstream fout(filename) ;    
  if(!fout)
    return 0 ;
  return write(fout);
}

/*! 
  \brief Transpose the U and V coordinates of a surface
  
  Transpose the U and V coordinates of a surface. After this
  operation the \a (u,v) points correspond to \a (v,u).

  \return A reference to itself
  \warning This is not completely tested

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
NurbsSurface<T,N>& NurbsSurface<T,N>::transpose(void){
  Vector<T> t(U) ;
  int d ;
  U = V ;
  V = t ;
  d = degU ;
  degU = degV ;
  degV = d ;
  P = ::transpose(P) ;
  return *this ;
}

/*!
  \brief Moves a point on the surface

  This moves the point \a s(u,v) by delta.	      

  \param u  the parameter in the u direction
  \param v  the parameter in the v direction
  \param delta  the displacement of the point at S(u,v)

  \return 1 if the operation is possible, 0 if the problem is ill defined
               \e i.e. there isn't enough information to find a unique 
	       solution (the system is overdetermined) or that the system
	       has non-independant components.

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T, int N>
int NurbsSurface<T,N>::movePoint(T u, T v, const Point_nD<T,N>& delta){
  // setup B
  Matrix_DOUBLE B(1,(degU+1)*(degV+1)) ;
  int i,j,k ;
  
  int spanU,spanV ;

  Vector<T> Ru,Rv ;

  B.reset(0.0) ;
  
  findSpan(u,v,spanU,spanV) ;
  nurbsBasisFuns(u,spanU,degU,U,Ru) ;
  nurbsBasisFuns(v,spanV,degV,V,Rv) ;
  for(j=0;j<=degU;++j){
    for(k=0;k<=degV;++k){
      B(0,j*(degV+1)+k) 
	= (double)Ru[j]*(double)Rv[k] ;
    }
  }
  
  Matrix_DOUBLE A  ;
  Matrix_DOUBLE Bt(::transpose(B)) ;
  Matrix_DOUBLE BBt ;

  BBt = inverse(B*Bt) ;
  A = Bt*BBt ;

  Matrix_DOUBLE dD(1,3) ;

  for(j=0;j<3;++j)
    dD(0,j) = (double)delta.data[j] ;

  Matrix_DOUBLE dP ;

  dP = A*dD ;

  i= 0 ;

  for(j=0;j<=degU;++j){
    for(k=0;k<=degV;++k){
      T w = P(spanU-degU+j,spanV-degV+k).w() ;
      P(spanU-degU+j,spanV-degV+k).x() += dP(i,0)*w ;
      P(spanU-degU+j,spanV-degV+k).y() += dP(i,1)*w ;
      P(spanU-degU+j,spanV-degV+k).z() += dP(i,2)*w ;
      ++i ;
    }
  }

  return 1 ;
}

/*!
  \brief Moves a point with some constraint
  
  This will modify the NURBS surface by respecting a certain 
  number of constraints. \a u_r and \a v_r specifies the parameters
  on which the constraints should be applied. 
  The constraint are defined by \a D_i(u,v) which requires 3
  vectors to fully qualify. $D$ specifies the value
  of the constraint and \a Du and \a Dv are used to specify
  on which parameter the constraint is applied.
  
  ur and vr should be in an increasing order.

  \param ur  the vector of parameters in the u direction
  \param vr  the vector of parameters in the v direction
  \param  D  a vector of the value of \a D_i^(k,l)(u,v)
  \param Du  a vector specifying the index of the value of u for \a D_i
  \param Dv  a vector specifying the index of the value of v for \a D_i

  \return 1 if the operation is possible, 0 if the problem is ill defined
               \e i.e. there isn't enough information to find a unique 
	       solution (the system is overdetermined) or that the system
	       has non-independant components.

  \warning The vectors defining \a D_i(u,v) should all be of the
               same size. 

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T, int N>
int NurbsSurface<T,N>::movePoint(const Vector<T>& ur, const Vector<T>& vr, const Vector< Point_nD<T,N> >& D, const Vector_INT& Du, const Vector_INT& Dv) {
  Vector_INT Dk(Du.n()),Dl(Dv.n()) ;
  BasicArray<Coordinate> fixCP(0) ;
  Dk.reset(0) ;
  Dl.reset(0) ;
  return movePoint(ur,vr,D,Du,Dv,Dk,Dl,fixCP) ;
}

/*!
  \brief Moves a point with some constraint

  This will modify the NURBS surface by respecting a certain 
  number of constraints. \a u_r and \a v_r specifies the parameters
  on which the constraints should be applied. 
  The constraint are defined by \a D_i^{(k,l)}(u,v) which requires
  5 vectors to fully qualify. \a D specifies the value
  of the constraint and \a Du and \a Dv are used to specify
  on which parameter the constraint is applied and 
  \a Dk and \a Dl specify the partial degree of the constraint.
  
  The values in \a D should be ordered in respect with i,k and l.
  ur and vr should be in an increasing order.

  \param ur  the vector of parameters in the u direction
  \param vr  the vector of parameters in the v direction
  \param  D  a vector of the value of \a D_i^(k,l)(u,v)
  \param Du  a vector specifying the index of the value of u for \a D_i
  \param Dv  a vector specifying the index of the value of v for \a D_i
  \param Dk  a vector specifying the value of \a k for \a D_i
  \param Dl  a vector specifying the value of \a l for \a D_i

  \return 1 if the operation is possible, 0 if the problem is ill defined
               \e i.e. there isn't enough information to find a unique 
	       solution (the system is overdetermined) or that the system
	       has non-independant components.

  \warning The vectors defining \a D_i^{(k,l)}(u,v) should all be of the
               same size. 

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T, int N>
int NurbsSurface<T,N>::movePoint(const Vector<T>& ur, const Vector<T>& vr, const Vector< Point_nD<T,N> >& D, const Vector_INT& Du, const Vector_INT& Dv, const Vector_INT& Dk, const Vector_INT& Dl) {
  BasicArray<Coordinate> fixCP(0) ;
  return movePoint(ur,vr,D,Du,Dv,Dk,Dl,fixCP) ;
}



/*!
  \brief Moves a point with some constraint

  This will modify the NURBS surface by respecting a certain 
  number of constraints. \a u_r and \a v_r specifies the parameters
  on which the constraints should be applied. 
  The constraint are defined by $D_i^{(k,l)}(u,v)$ which requires
  5 vectors to fully qualify. \a D specifies the value
  of the constraint and \a Du and \a Dv are used to specify
  on which parameter the constraint is applied and 
  \a Dk and \a Dl specify the partial degree of the constraint.
  
  A second constraint \a fixCP consists of specifying which 
  control points can not be moved by the routine.
  
  The values in D should be ordered in respect with i,k and l.
  ur and vr should be in an increasing order.
  
  \param ur  the vector of parameters in the u direction
  \param vr  the vector of parameters in the v direction
  \param  D  a vector of the value of \a D_i^(k,l)(u,v)
  \param Du  a vector specifying the index of the value of u for \a D_i
  \param Dv  a vector specifying the index of the value of v for \a D_i
  \param Dk  a vector specifying the value of \a k for \a D_i
  \param Dl  a vector specifying the value of \a l for \a D_i
  \param fixCP  a vector specifying which control points can \e not be
	              modified.

  \return 1 if the operation is possible, 0 if the problem is ill defined
               \e i.e. there isn't enough information to find a unique 
	       solution (the system is overdetermined) or that the system
	       has non-independant components.

  \warning The vectors defining $D_i^{(k,l)}(u,v)$ should all be of the
               same size. 

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T, int N>
int NurbsSurface<T,N>::movePoint(const Vector<T>& ur, const Vector<T>& vr, const Vector< Point_nD<T,N> >& D, const Vector_INT& Du, const Vector_INT& Dv, const Vector_INT& Dk, const Vector_INT& Dl, const BasicArray<Coordinate>& fixCP) {
  int i,j,k,n ;

  if(D.n() != Du.n() || D.n() !=Du.n() || D.n() != Dv.n() || D.n() != Dv.n()){
#ifdef USE_EXCEPTION
    throw NurbsInputError();
#else
    Error err("movePoint(ur,D,Dr,Dk,fixCP)");
    err << "The D,Dr,Dk vectors are not of the same size\n" ;
    err << "D.n()= " << D.n() << ", Du.n() = " << Du.n() 
	<< ", Dk.n() = " << Dk.n() << ", Dv.n() = " << Dv.n() 
	<< ", Dl.n() = " << Dl.n() << endl ;
    err.fatal() ;
#endif
  }  
  
  // setup B
  Matrix_DOUBLE B ;
  
  B.resize(D.n(),P.rows()*P.cols()) ;
  
  int spanU,spanV ;

  Matrix<T> Ru,Rv ;

  B.reset(0.0) ;
  
  for(i=0;i<D.rows();++i){
    findSpan(ur[Du[i]],vr[Dv[i]],spanU,spanV) ;
    nurbsDersBasisFuns(Dk[i],ur[Du[i]],spanU,degU,U,Ru) ;
    nurbsDersBasisFuns(Dl[i],vr[Dv[i]],spanV,degV,V,Rv) ;
    for(j=0;j<=degU;++j){
      for(k=0;k<=degV;++k){
	B(i,(spanU-degU+j)*P.cols()+spanV-degV+k) 
	  = (double)Ru(Dk[i],j)*(double)Rv(Dl[i],k) ;
      }
    }
  }
  
  // optimize B
  Vector_INT remove(B.cols()) ;
  BasicArray<Coordinate> map(B.cols()) ;
  remove.reset((int)1.0) ;

  for(j=0;j<B.cols();++j){
    for(i=0;i<B.rows();++i)
      if((B(i,j)*B(i,j))>1e-10){
	remove[j] = 0 ;
	break ;
      }
  }

  for(i=0;i<fixCP.n();++i){
    remove[fixCP[i].i*P.cols()+fixCP[i].j] = 1 ;
  }
  
  n = 0 ;
  for(i=0;i<B.cols();++i){
    if(!remove[i]){
      map[n].i = i/P.cols() ;
      map[n].j = i%P.cols() ;
      ++n ;
    }
  }

  map.resize(n) ;

  Matrix_DOUBLE Bopt(B.rows(),n) ;
  for(j=0;j<n;++j){
    for(i=0;i<B.rows();++i)
      Bopt(i,j) = B(i,map[j].i*P.cols()+map[j].j) ;
  }

  Matrix_DOUBLE A  ;
  Matrix_DOUBLE Bt(::transpose(Bopt)) ;
  Matrix_DOUBLE BBt ;

  BBt = inverse(Bopt*Bt) ;

  A = Bt*BBt ;

  Matrix_DOUBLE dD(D.rows(),3) ;

  for(i=0;i<D.rows();++i){
    const Point_nD<T,N>& d = D[i] ; // this makes the SGI compiler happy
    for(j=0;j<3;++j)
      dD(i,j) = (double)d.data[j] ;
  }

  Matrix_DOUBLE dP ;

  dP = A*dD ;

  for(i=0;i<map.n();++i){
    P(map[i].i,map[i].j).x() += dP(i,0)*P(map[i].i,map[i].j).w() ;
    P(map[i].i,map[i].j).y() += dP(i,1)*P(map[i].i,map[i].j).w() ;
    P(map[i].i,map[i].j).z() += dP(i,2)*P(map[i].i,map[i].j).w() ;
  }

  return 1 ;
}


/*!
  \brief Projects a point on a line
	       
  \param S a point on the line
  \param T the tangent vector of the line
  \param pnt the point to project
  \param p the point projected on the line

  \author Philippe Lavoie
  \date 25 July, 1997
*/
template <class T, int N>
void projectToLine(const Point_nD<T,N>& S, const Point_nD<T,N>& Trj, const Point_nD<T,N>& pnt, Point_nD<T,N>& p) {

  Point_nD<T,N> a = pnt-S ;  
  //p = S+ norm(a)*cos(angle(a,Trj))*Trj.unitLength() ;
  T fraction, denom ;
  denom = norm2(Trj) ; 
  fraction = (denom == 0.0) ? 0.0 : (Trj*a) / denom ; 
  p =  fraction * Trj ; 
  p += S ; 
}

/*!
  \brief Generates a surface of revolution

  Generates a surface of revolution of a profile around an
  arbitrary axis (specified by a starting point S and a 
  tangent T) with a certain angle.
  
  The angle is specified in radians.

  \param profile  the curves to rotate around the axis
  \param S  a point on the axis
  \param T  the tangent vector of the rotation axis
  \param theta  the angle of the revolution (in radians)

  \warning If a point is within a distance of 1e-7 from the axis,
           it will be assumed to be on the axis. The angle theta
	   is only valid in the range [0,2 PI].

  \author Philippe Lavoie
  \date 7 October, 1997
*/
template <class T, int N>
void NurbsSurface<T,N>::makeFromRevolution(const NurbsCurve<T,N>& profile, const Point_nD<T,N>& S, const Point_nD<T,N>& Tvec, double theta){
  double angle,dtheta ;
  int narcs ;
  int i,j ;

  //while(theta>2.0*M_PI)
  //  theta -= 2.0*M_PI ;

  if(theta>2.0*M_PI)
    theta = 2.0*M_PI ;

  if(theta <= 0)
    theta = 0 ;

  if(theta<M_PI/2.0)	
    narcs = 1 ;
  else{
    if(theta<M_PI)
      narcs = 2 ;
    else{
      if(theta<1.5*M_PI)
	narcs = 3 ;
      else
	narcs = 4 ;
    }
  }
  dtheta = theta/(double)narcs ;

  int n = 2*narcs+1 ; // n control points ;
  resize(n,profile.ctrlPnts().n(),2,profile.degree()) ;

  switch(narcs){
  case 1: break ;
  case 2: 
    U[3] = U[4] = 0.5 ;
    break ;
  case 3:
    U[3] = U[4] = 1.0/3.0 ;
    U[5] = U[6] = 2.0/3.0 ;
    break ;
  case 4:
    U[3] = U[4] = 0.25 ;
    U[5] = U[6] = 0.50 ;  
    U[7] = U[8] = 0.75 ;
    break ;    
  }

  j = 3 + 2*(narcs-1) ;  // loading the end knots
  for(i=0;i<3;++j,++i){
    U[i] = 0.0 ;
    U[j] = 1.0 ;
  }

  V = profile.knot() ;

  double wm = cos(dtheta/2.0) ; // dtheta/2.0 is base angle

  Vector_DOUBLE cosines(narcs+1),sines(narcs+1) ;
  angle = 0 ;
  for(i=1;i<=narcs;++i){
    angle = dtheta*(double)i ;
    cosines[i] = cos(angle) ;
    sines[i] = sin(angle) ;
  }

  Point_nD<T,N> P0,T0,P2,T2,P1 ;

  for(j=0;j<P.cols();++j){
    Point_nD<T,N> O ;
    T wj = profile.ctrlPnts(j).w() ;

    projectToLine(S,Tvec,project(profile.ctrlPnts(j)),O) ;
    //projectToLine(S,Tvec,profile.ctrlPnts(j).projectW(),O) ;
    Point_nD<T,N> X,Y ;
    X = project(profile.ctrlPnts(j))-O ;
    //X = profile.ctrlPnts(j).projectW() - O ; 

    double r = norm(X) ;

    if(r < 1e-7){
      for(i=0;i<P.rows();++i){
	P(i,j) = O ;
	P(i,j) *= wj ;
      }
      continue ;
    }

    T b1 = X.norm2() ;
    T b2 = X.norm() ;
    T b3 = sqrt(b1) ;
    T b4 = norm(X) ;
   
    X = X.unitLength() ;
    Y = crossProduct(Tvec,X) ;
    Y = Y.unitLength() ;

    P0 = project(profile.ctrlPnts(j)) ;
    //P0 = profile.ctrlPnts(j).projectW() ;
    P(0,j) = profile.ctrlPnts(j) ;

    T0 = Y ;
    int index = 0 ;

    for(i=1;i<=narcs;++i){
      angle = dtheta*(double)i ;
      P2 = O+ r*cosines[i]*X + r*sines[i]*Y ;  
      //P2 = O+ r*cos(angle)*X + r*sin(angle)*Y ;  
      P(index+2,j) = P2 ;
      P(index+2,j) *= wj ;
      T2 = -sines[i]*X + cosines[i]*Y ;
      //T2 = -sin(angle)*X + cos(angle)*Y ;
      intersectLine(P0,T0,P2,T2,P1) ;
      P(index+1,j) = P1 ;
      P(index+1,j) *= wm*wj ;
      index += 2 ;
      P0 = P2 ;
      T0 = T2 ;
    }
  }
}

/*!
  \brief Generates a surface of revolution

  Generates a surface of revolution of a profile around an
  arbitrary axis (specified by a starting point S and a 
  tangent T). 
  
  \param profile  the curves to rotate around the axis
  \param S  a point on the axis
  \param T  the tangent vector of the rotation axis

  \warning If a point is within a distance of 1e-7 from the axis,
           it will be assumed to be on the axis. 

  \author Philippe Lavoie
  \date 3 May, 1999
*/
template <class T, int N>
void NurbsSurface<T,N>::makeFromRevolution(const NurbsCurve<T,N>& profile, const Point_nD<T,N>& S, const Point_nD<T,N>& Tvec){
  double angle,dtheta ;
  int narcs ;
  int i,j ;

  int n = 9 ; // n control points ;
  resize(n,profile.ctrlPnts().n(),2,profile.degree()) ;

  U[0] = U[1] = U[2] = 0 ;
  U[3] = U[4] = 0.25 ;
  U[5] = U[6] = 0.50 ;  
  U[7] = U[8] = 0.75 ;
  U[9] = U[10] = U[11] = 1 ;

  V = profile.knot() ;


  const T wm = T(0.707106781185) ; // sqrt(2)/2

  for(j=0;j<P.cols();++j){
    Point_nD<T,N> O ;
    T wj = profile.ctrlPnts(j).w() ;

    projectToLine(S,Tvec,project(profile.ctrlPnts(j)),O) ;
    //projectToLine(S,Tvec,profile.ctrlPnts(j).projectW(),O) ;
    Point_nD<T,N> X,Y ;
    X = project(profile.ctrlPnts(j))-O ;
    //X = profile.ctrlPnts(j).projectW() - O ; 

    double r = norm(X) ;

    if(r < 1e-7){
      for(i=0;i<P.rows();++i){
	P(i,j) = O ;
	P(i,j) *= wj ;
      }
      continue ;
    }

    X = X.unitLength() ;
    Y = crossProduct(Tvec,X) ;
    Y = Y.unitLength() ;

    P(0,j) = O + r*X ;
    P(1,j) = O + r*X + r*Y ; 
    P(2,j) = O + r*Y ; 
    P(3,j) = O - r*X + r*Y ; 
    P(4,j) = O - r*X ;
    P(5,j) = O - r*X - r*Y ; 
    P(6,j) = O - r*Y ; 
    P(7,j) = O + r*X - r*Y ; 
    P(8,j) = P(0,j) ; 
    P(0,j) *= wj ;
    P(1,j) *= wj*wm ;
    P(2,j) *= wj ;
    P(3,j) *= wj*wm ;
    P(4,j) *= wj ;
    P(5,j) *= wj*wm ;
    P(6,j) *= wj ;
    P(7,j) *= wj*wm ;
    P(8,j) *= wj ;
  }
}

/*!
  \brief Generates a surface of revolution

  Generates a surface of revolution of a profile around the z axis.
  
  \param profile  the curves to rotate around the z-axis

  \author Philippe Lavoie
  \date 3 May, 1999
*/
template <class T, int N>
void NurbsSurface<T,N>::makeFromRevolution(const NurbsCurve<T,N>& profile){
  int j ; 
  int n = 9 ; // n control points ;
  resize(n,profile.ctrlPnts().n(),2,profile.degree()) ;

  U[0] = U[1] = U[2] = 0 ;
  U[3] = U[4] = 0.25 ;
  U[5] = U[6] = 0.50 ;  
  U[7] = U[8] = 0.75 ;
  U[9] = U[10] = U[11] = 1 ;

  V = profile.knot() ;


  const T wm = T(0.707106781185) ; // sqrt(2)/2

  for(j=0;j<P.cols();++j){
    T wj = profile.ctrlPnts(j).w() ;

    Point_nD<T,N> p = project(profile.ctrlPnts(j)) ;
    T r = T(sqrt(p.x()*p.x()+p.y()*p.y()));

    T wjm = wj*wm ; 
    T rwjm = r*wj*wm ;
    T rwj = r*wj ; 
    p.z() *= wj ; 
    
    P(0,j) = HPoint_nD<T,N>(rwj,0,p.z(),wj) ;          
    P(1,j) = HPoint_nD<T,N>(rwjm,rwjm,p.z()*wm,wjm) ;   
    P(2,j) = HPoint_nD<T,N>(0,rwj,p.z(),wj) ;          
    P(3,j) = HPoint_nD<T,N>(-rwjm,rwjm,p.z()*wm,wjm) ;  
    P(4,j) = HPoint_nD<T,N>(-rwj,0,p.z(),wj) ;         
    P(5,j) = HPoint_nD<T,N>(-rwjm,-rwjm,p.z()*wm,wjm) ; 
    P(6,j) = HPoint_nD<T,N>(0,-rwj,p.z(),wj) ;         
    P(7,j) = HPoint_nD<T,N>(rwjm,-rwjm,p.z()*wm,wjm) ;  
    P(8,j) = HPoint_nD<T,N>(rwj,0,p.z(),wj) ;          
  }
}

/*!
  \brief Generates an iso curve in the U direction

  Generates an iso-parametric curve which goes through the
  parametric value u along the U direction.

  \param u  the U parametric value
  \param c  the iso-parametric curve

  \warning the parametric value $u$ must be in a valid range

  \author Philippe Lavoie
  \date 7 October, 1997
*/
template <class T, int D>
void NurbsSurface<T,D>::isoCurveU(T u, NurbsCurve<T,D>& c) const {
  c.resize(P.cols(),degV) ;

  c.modKnot(V) ;

  if(u>U[U.n()-1])
    u = U[U.n()-1] ;
  if(u<U[0])
    u = U[0] ;



  int span = findSpanU(u) ;
  
  Vector<T> N ;
  basisFunsU(u,span,N) ;

  HPoint_nD<T,D> p ;
  for(int i=0;i<P.cols();++i){
    p = 0 ;
    for(int j=0;j<=degU;++j)
      p += N[j]*P(span-degU+j,i) ;
    c.modCP(i,p) ;
  }
  
}

/*!
  \brief Generates an iso curve in the V direction

  Generates an iso-parametric curve which goes through the
  parametric value v along the V direction.

  \param v  the V parametric value
  \param c  the iso-parametric curve

  \warning the parametric value $v$ must be in a valid range

  \author Philippe Lavoie
  \date 7 October, 1997
*/
template <class T, int D>
void NurbsSurface<T,D>::isoCurveV(T v, NurbsCurve<T,D>& c) const {
  c.resize(P.rows(),degU) ;

  c.modKnot(U) ;

  if(v>V[V.n()-1])
    v = V[V.n()-1] ;
  if(v<V[0])
    v = V[0] ;

  int span = findSpanV(v) ;
  
  Vector<T> N ;
  basisFunsV(v,span,N) ;
  
  HPoint_nD<T,D> p ;
  for(int i=0;i<P.rows();++i){
    p = 0  ;
    for(int j=0;j<=degV;++j)
      p += N[j]*P(i,span-degV+j) ;
    c.modCP(i,p) ;
  }
}

/*!
  \brief Decompose the surface into Bézier patches

  This function decomposes the curve into an array of homogenous Bézier 
  patches.

  \param S  an array of Bézier segments
  \return The number of Bézier strips in the u direction.

  \author Philippe Lavoie
  \date 8 October, 1997
*/
template <class T, int N>
int NurbsSurface<T,N>::decompose(NurbsSurfaceArray<T,N>& S) const {
  int i,m,a,b,nb,mult,j,r,save,s,k,row,col ;
  T numer,alpha ;


  //Vector<T> alphas(degU+1) ;
  T* alphas = (T*) alloca((maximum(degU,degV)+1)*sizeof(T)) ;
  // all the surfaces will have the same knot vector in both the U and V
  // direction
  Vector<T> nU ;
  nU.resize(2*(degU+1)) ;
  for(i=0;i<nU.n()/2;++i)
    nU[i] = 0 ;
  for(i=nU.n()/2;i<nU.n();++i)
    nU[i] = 1 ;
  
  Vector<T> nV ;
  nV.resize(2*(degV+1)) ;
  for(i=0;i<nV.n()/2;++i)
    nV[i] = 0 ;
  for(i=nV.n()/2;i<nV.n();++i)
    nV[i] = 1 ;

  NurbsSurfaceArray<T,N> Su ;
  Su.resize(P.rows()-degU) ; // )*(P.cols()-degV)) ;
  for(i=0;i<Su.n();i++){
    Su[i].resize(degU+1,P.cols(),degU,degV) ;
    Su[i].U = nU ;
    Su[i].V = V ;
  }
  
  m = P.rows()+degU ;
  a = degU ;
  b = degU+1 ;
  nb = 0 ;

  for(i=0;i<=degU;i++)
    for(col=0;col<P.cols();col++)
      Su[nb].P(i,col) = P(i,col) ;
  while(b<m){
    i = b ;
    while(b<m && U[b+1] <= U[b]) b++ ;
    mult = b-i+1 ;
    if(mult<degU){
      numer = U[b]-U[a] ; // the enumerator of the alphas
      for(j=degU;j>mult;j--) // compute and store the alphas
	alphas[j-mult-1] = numer/(U[a+j]-U[a]) ;
      r = degU-mult ; // insert knot r times
      for(j=1;j<=r;j++){
	save=r-j;
	s=mult+j; // this many new points
	for(k=degU;k>=s;k--){
	  alpha = alphas[k-s] ;
	  for(col=0;col<P.cols();++col)
	    Su[nb].P(k,col) = alpha*Su[nb].P(k,col)+(1.0-alpha)*Su[nb].P(k-1,col);
	}
	if(b<m) // control point of next patch
	  for(col=0;col<P.cols();++col)
	    Su[nb+1].P(save,col) = Su[nb].P(degU,col) ;
      }
    }
    ++nb ;
    if(b<m){ // initialize for next segment
      for(i=degU-mult; i<= degU ; ++i)
	for(col=0;col<P.cols();++col)
	  Su[nb].P(i,col) = P(b-degU+i,col) ;
      a=b ;
      ++b ;
    }
  }
  Su.resize(nb) ;
  
  S.resize(Su.n()*(P.cols()-degV)) ;

  for(i=0;i<S.n();i++){
    S[i].resize(degU+1,degV+1,degU,degV) ;
    S[i].U = nU ;
    S[i].V = nV ;
  }
  
  nb = 0 ;

  for(int np=0;np<Su.n();++np){
    for(i=0;i<=degU;i++)
      for(j=0;j<=degV;++j)
	S[nb].P(i,j) = Su[np].P(i,j) ;
    m = P.cols()+degV ;
    a = degV ;
    b = degV+1 ;
    while(b<m){
      i = b ;
      while(b<m && V[b+1] <= V[b]) b++ ;
      mult = b-i+1 ;
      if(mult<degV){
	numer = V[b]-V[a] ; // the enumerator of the alphas
	for(j=degV;j>mult;j--) // compute and store the alphas
	  alphas[j-mult-1] = numer/(V[a+j]-V[a]) ;
	r = degV-mult ; // insert knot r times
	for(j=1;j<=r;j++){
	  save=r-j;
	  s=mult+j; // this many new points
	  for(k=degV;k>=s;k--){
	    alpha = alphas[k-s] ;
	    for(row=0;row<=degU;++row)
	      S[nb].P(row,k) = alpha*S[nb].P(row,k)+(1.0-alpha)*S[nb].P(row,k-1);
	  }
	  if(b<m) // control point of next patch
	    for(row=0;row<=degU;++row)
	      S[nb+1].P(row,save) = S[nb].P(row,degV) ;
	}
      }
      ++nb ;
      if(b<m){ // initialize for next patch
	for(i=degV-mult; i<= degV ; ++i)
	  for(row=0;row<=degU;++row)
	    S[nb].P(row,i) = Su[np].P(row,b-degV+i) ;
	a=b ;
	++b ;
      }
    }
  }

  S.resize(nb) ;

  return Su.n() ;
}

/*!
  \brief Writes a set of povray bicubic patches to the ostream.

  \param patch_type  the type of the patch
  \param flatness  the flatness level
  \param  num_u_steps  the minimum number of triangles in the U 
	                   direction
  \param num_v_steps  the minimum number of triangles in the V 
	                   direction
  \param povray  the output stream

  \return 1 on success, 0 on failure.

  \warning POVRAY only accepts rational spline patches. Thus you can't
               have a value other then 1.0 for the weights of your surface.
	       A warning message will be generated if this is the case.

	       POVRAY only deals with surfaces of degree 3. If the surface 
	       as a degree below 3 either in the U or V direction it will be
	       elevated to be at 3 in both directions. If the surface as a
	       degree higher then 3, then the function aborts.

  \author Philippe Lavoie
  \date 8 October, 1997
*/
template <class T, int N>
int NurbsSurface<T,N>::writePOVRAY(ostream& povray, int patch_type, double flatness, int num_u_steps, int num_v_steps) const {
  if(degU>3 || degV>3){
#ifdef USE_EXCEPTION
    throw NurbsInputError();
#else
    Error err("NurbsSurface<T,N> writePOVRAY") ;
    err << "The degree of the surface is higher than 3!\n"
	<< "A povrary file can not be generated.\n" ;
    err.warning() ;
    return 0;
#endif
  }

  NurbsSurface<T,N> S(*this) ;

  S.degreeElevate(3-degU,3-degV) ;

  NurbsSurfaceArray<T,N> Sa ;
  S.decompose(Sa) ;
  int bad = 0 ; 

  povray << "//\n//Generated for POV-Ray(tm) 3.0 by Phil's NURBS library\n" ;
  povray << "//   http://yukon.genie.uottawa.ca/info/soft/nurbs\n//\n" ;

  for(int i=0;i<Sa.n();++i){
    povray << "bicubic_patch {\n\ttype " << patch_type << endl ;
    povray << "\tflatness " << flatness << endl ;
    povray << "\tu_steps " << num_u_steps << endl ;
    povray << "\tv_steps " << num_v_steps << endl ;
    for(int j=0;j<4;++j){
      for(int k=0;k<4;++k){
	Point_nD<T,N> p = project(Sa[i].ctrlPnts(j,k)) ;
	if(Sa[i].ctrlPnts(j,k).w()>1.0001 || Sa[i].ctrlPnts(j,k).w()<0.9999)
	  bad = 1 ;
	povray << "\t<" << p.x() << ", " << p.y() << ", " << p.z() << ">" ;
	if(j==3 && k==3)
	  povray << "\n}\n\n" ;
	else
	  povray << ",\n " ;
      }
      povray << endl ;
    }
  }
  if(bad){
#ifdef USE_EXCEPTION
    throw NurbsWarning();
#else
    Error err("NurbsSurface<T,N> writePOVRAY") ;
    err << "Warning: at least one of the control point was not rational\n" ;
    err << "The resulting surface will NOT be the same as the one which\n" ;
    err << "generated it.\n" ;
    err.warning() ;
#endif
  }
  return bad ;
}


/*!
  \brief Writes the surface as a mesh of triangles
  
  Writes the surface as a mesh of triangles. You might have
  to change the values for the tolerance to get exactly what
  you're looking for.

  \param povray  the output stream
  \param tolerance  the tolerance when performing the tesselation
  \param color  the color of the object 
  \param diffuse  the diffuse factor 
  \param ambient  the ambient factor 
  \param smooth  flags which indicates if we generate smooth triangles 

  \return 1 on success, 0 on failure

  \warning It doesn't work very well.

  \author Philippe Lavoie
  \date 8 October, 1997
*/
template <class T, int N>
int NurbsSurface<T,N>::writePOVRAY(T tolerance, ostream& povray,const Color& color, int smooth, T ambient, T diffuse) const {
  BasicList<Point_nD<T,N> > points ;
  BasicList<int> connect ;
  BasicList<Point_nD<T,N> > norm ;

  if(smooth)
    tesselate(tolerance,points,connect,&norm) ;
  else
    tesselate(tolerance,points,connect) ;
    
  povray << "mesh {\n" ;
  
  BasicNode<int> *node ;

  BasicNode<Point_nD<T,N> > *nodeP ;
  BasicNode<Point_nD<T,N> > *nodeN ;
  nodeP = points.goToFirst() ;
  nodeN = 0 ;

  if(smooth)
    nodeN = norm.goToFirst() ;

  Vector< Point_nD<T,N> > Pts(points.size()) ;
  Vector< Point_nD<T,N> > Norm(norm.size()) ;
  for(int i=0;i<Pts.n();++i){
    Pts[i] = *nodeP->data ;
    nodeP = points.goToNext() ;
    if(smooth){
      Norm[i] = *nodeN->data ;
      nodeN = norm.goToNext() ;
    }
  }

  node = connect.goToFirst() ;
  while(node){
    if(smooth)
      povray << "\tsmooth_triangle { " ;
    else
      povray << "\ttriangle { " ;
    povray << "< " << Pts[*node->data].x() << ", " << Pts[*node->data].y() << ", " << Pts[*node->data].z() << "> , " ;
    if(smooth)
      povray << "< " << Norm[*node->data].x() << ", " << Norm[*node->data].y() << ", " << Norm[*node->data].z() << "> , " ;
    node = connect.goToNext() ; 
    povray << "< " << Pts[*node->data].x() << ", " << Pts[*node->data].y() << ", " << Pts[*node->data].z() << "> , " ;
    if(smooth)
      povray << "< " << Norm[*node->data].x() << ", " << Norm[*node->data].y() << ", " << Norm[*node->data].z() << "> , " ;
    node = connect.goToNext() ;
    povray << "< " << Pts[*node->data].x() << ", " << Pts[*node->data].y() << ", " << Pts[*node->data].z() << "> " ;
    if(smooth)
      povray << ", < " << Norm[*node->data].x() << ", " << Norm[*node->data].y() << ", " << Norm[*node->data].z() << "> " ;
    node = connect.goToNext() ; // skip the -1 value
    node = connect.goToNext() ;
    povray << "}\n" ;
  }
  
  povray << "\ttexture{ pigment { rgb < " << (double)color.r/255.0 << ", " << (double)color.g/255.0 << ", " << (double)color.b/255.0 << " >}\n" ;
  povray << "\t\tfinish { ambient " << ambient << " diffuse " << diffuse << " }\n\t}\n" ;
  
  povray << "}\n\n" ;
  return povray.good() ;
}

/*!
  \brief Writes a set of povray bicubic patches

  \param patch_type  the type of the patch
  \param flatness  the flatness level
  \param num_u_steps  the minimum number of triangles in the U direction
  \param num_v_steps  the minimum number of triangles in the V direction

  \return an ostream containing the definition of the surface

  \warning POVRAY only accepts rational spline patches. Thus you can't
               have a value other then 1.0 for the weights of your surface.
	       A warning message will be generated if this is the case.

	       POVRAY only deals with surfaces of degree 3. If the surface 
	       as a degree below 3 either in the U or V direction it will be
	       elevated to be at 3 in both directions. If the surface as a
	       degree higher then 3, then the function aborts.

  \author Philippe Lavoie
  \date 8 October, 1997
*/
template <class T, int N>
int NurbsSurface<T,N>::writePOVRAY(const char *filename, const Color& col, const Point_nD<T,N>& cView, const Point_nD<T,N>& up, int patch_type, double flatness, int num_u_steps, int num_v_steps) const {
  ofstream fout(filename) ;
  if(!fout)
    return 0 ;
  
  Point_nD<T,N> view(T(-1.0)*cView) ;

  fout << "//\n//Generated for POV-Ray(tm) 3.0 by Phil's NURBS library\n//\n" ;
  fout << "\n#include \"colors.inc\"\n" ;

  // we want the camera to look at the center of the object
  // and be able to view the whole object
  // we use and angle of 36 to view the object
  // and position the rest according to this.
  Point_nD<T,N> minP, maxP ;
  minP.x() = extremum(1,coordX) ;
  minP.y() = extremum(1,coordY) ;
  minP.z() = extremum(1,coordZ) ;
  maxP.x() = extremum(0,coordX) ;
  maxP.y() = extremum(0,coordY) ;
  maxP.z() = extremum(0,coordZ) ;

  Point_nD<T,N> lookAt  ;
  lookAt.x() = (minP.x()+maxP.x())/2.0 ;
  lookAt.y() = (minP.y()+maxP.y())/2.0 ;
  lookAt.z() = (minP.z()+maxP.z())/2.0 ;

  Point_nD<T,N> camera1, camera2 ;

  Point_nD<T,N> q1 = minP-lookAt ;
  Point_nD<T,N> q2 = maxP-lookAt ;
  T D1 = absolute(dot(q1,view))/norm(view) ;
  T D2 = absolute(dot(q2,view))/norm(view) ;

  T a1 = norm(q1)*cos(angle(view,q1)) ;
  T a2 = norm(q2)*cos(angle(view,q2)) ;

  T b1 = D1/tan(18.0*M_PI/180.0) ;
  T b2 = D2/tan(18.0*M_PI/180.0) ; // this gives the 36 degree angle

  camera1 = lookAt+(a1+b1)*view.unitLength() ;
  camera2 = lookAt+(a2+b2)*view.unitLength() ;

  Point_nD<T,N> right ;

  right = crossProduct(view,up) ; // inversed because pov-ray uses a left-handed system

  fout << "camera {\n\tlocation <" ;
  if(norm2(camera1-lookAt)>norm2(camera2-lookAt))
    fout << camera1.x() << ", " << camera1.y() << ", " << camera1.z() << ">\n" ;
  else
    fout << camera2.x() << ", " << camera2.y() << ", " << camera2.z() << ">\n" ;
  fout << "\tup < " << up.x() << ", " << up.y() << ", " << up.z() << ">\n" ;
  fout << "\tright < " << right.x() << ", " << right.y() << ", " << right.z() << ">\n" ;
  fout << "\tlook_at < " << lookAt.x() << ", " << lookAt.y() << ", " << lookAt.z() << ">\n\tangle 36\n}\n\n" ;

  fout << "union {\n" ;
  writePOVRAY(fout,patch_type,flatness,num_u_steps,num_v_steps) ;
  fout << " texture {\n\tpigment {\n\t\tcolor rgb < " << (double)col.r/255.0 << 
    ", " << (double)col.g/255.0 << ", " << (double)col.b/255.0 << "> \n" <<
    "\t}\n\tfinish { \n\t\tambient .2\n\t\tdiffuse .6\n\t}\n }\n" ;
  fout << "\n}\n" ;

  fout << "light_source { < " ;
  if(norm2(camera1-lookAt)>norm2(camera2-lookAt))
    fout << camera1.x()+view.x() << ", " << camera1.y()+view.y() << ", " << camera1.z()+view.z() << "> color White}\n\n" ;
  else
    fout << camera2.x()+view.x() << ", " << camera2.y()+view.y() << ", " << camera2.z()+view.z() << "> color White}\n\n" ;
  
  return fout.good() ;
}


/*!
  \brief Writes a set of povray bicubic patches

  \param tolerance  the tolerance when performing the tesselation
  \param filename the file to write to
  \param color  the color of the object 
  \param diffuse  the diffuse factor 
  \param ambient  the ambient factor 
  \param smooth  flags which indicates if we generate smooth triangles 

  \return an ostream containing the definition of the surface

  \warning POVRAY only accepts rational spline patches. Thus you can't
               have a value other then 1.0 for the weights of your surface.
	       A warning message will be generated if this is the case.

	       POVRAY only deals with surfaces of degree 3. If the surface 
	       as a degree below 3 either in the U or V direction it will be
	       elevated to be at 3 in both directions. If the surface as a
	       degree higher then 3, then the function aborts.

  \author Philippe Lavoie
  \date 8 October, 1997
*/
template <class T, int N>
int NurbsSurface<T,N>::writePOVRAY(T tolerance, const char *filename, const Color& col, const Point_nD<T,N>& cView, const Point_nD<T,N>& up, int smooth, T ambient, T diffuse) const {
  ofstream fout(filename) ;
  if(!fout)
    return 0 ;
  
  Point_nD<T,N> view(T(-1)*cView) ;

  fout << "//\n//Generated for POV-Ray(tm) 3.0 by Phil's NURBS library\n//\n" ;
  fout << "\n#include \"colors.inc\"\n" ;

  // we want the camera to look at the center of the object
  // and be able to view the whole object
  // we use and angle of 36 to view the object
  // and position the rest according to this.
  Point_nD<T,N> minP, maxP ;
  minP.x() = extremum(1,coordX) ;
  minP.y() = extremum(1,coordY) ;
  minP.z() = extremum(1,coordZ) ;
  maxP.x() = extremum(0,coordX) ;
  maxP.y() = extremum(0,coordY) ;
  maxP.z() = extremum(0,coordZ) ;

  Point_nD<T,N> lookAt  ;
  lookAt.x() = (minP.x()+maxP.x())/2.0 ;
  lookAt.y() = (minP.y()+maxP.y())/2.0 ;
  lookAt.z() = (minP.z()+maxP.z())/2.0 ;

  Point_nD<T,N> camera1, camera2 ;

  Point_nD<T,N> q1 = minP-lookAt ;
  Point_nD<T,N> q2 = maxP-lookAt ;
  T D1 = absolute(dot(q1,view))/norm(view) ;
  T D2 = absolute(dot(q2,view))/norm(view) ;

  T a1 = norm(q1)*cos(angle(view,q1)) ;
  T a2 = norm(q2)*cos(angle(view,q2)) ;

  T b1 = D1/tan(18.0*M_PI/180.0) ;
  T b2 = D2/tan(18.0*M_PI/180.0) ; // this gives the 36 degree angle

  camera1 = lookAt+(a1+b1)*view.unitLength() ;
  camera2 = lookAt+(a2+b2)*view.unitLength() ;

  Point_nD<T,N> right ;

  right = crossProduct(view,up) ; // inversed because pov-ray uses a left-handed system

  fout << "camera {\n\tlocation <" ;
  if(norm2(camera1-lookAt)>norm2(camera2-lookAt))
    fout << camera1.x() << ", " << camera1.y() << ", " << camera1.z() << ">\n" ;
  else
    fout << camera2.x() << ", " << camera2.y() << ", " << camera2.z() << ">\n" ;
  fout << "\tup < " << up.x() << ", " << up.y() << ", " << up.z() << ">\n" ;
  fout << "\tright < " << right.x() << ", " << right.y() << ", " << right.z() << ">\n" ;
  fout << "\tlook_at < " << lookAt.x() << ", " << lookAt.y() << ", " << lookAt.z() << ">\n\tangle 36\n}\n\n" ;

  writePOVRAY(tolerance,fout,col,smooth,ambient,diffuse) ;
  

  fout << "light_source { < " ;
  if(norm2(camera1-lookAt)>norm2(camera2-lookAt))
    fout << camera1.x()+view.x() << ", " << camera1.y()+view.y() << ", " << camera1.z()+view.z() << "> color White}\n\n" ;
  else
    fout << camera2.x()+view.x() << ", " << camera2.y()+view.y() << ", " << camera2.z()+view.z() << "> color White}\n\n" ;
  
  return fout.good() ;
}




/*!
  \brief Writes a NuPatch for render man

  Writes a stream which is compatible with Render Man 
  specifications of a NURBS surface.

  \latexonly
	       \begin{center}
	       The RenderMan \Pisymbol{psy}{226} 
	       Interface Procedures and RIB Protocol are:
	       Copyright 1988, 1989, Pixar.  All rights reserved.
	       RenderMan \Pisymbol{psy}{226} is a registered trademark of 
	       Pixar.
	       \end{center}
  \endlatexonly
  \htmlonly
  <verbatim>
     The RenderMan (R) Interface Procedures and RIB Protocol are:
	       Copyright 1988, 1989, Pixar.  All rights reserved.
	       RenderMan (R) is a registered trademark of Pixar.
  </verbatim>   
  \endhtmlonly

  \param rib the rib ostream

  \return 0 if an error occurs, 1 otherwise

  \author Philippe Lavoie
  \date 8 October, 1997
*/
template <class T, int N>
int NurbsSurface<T,N>::writeRIB(ostream& rib) const {
  rib << "NuPatch " << P.rows() << ' ' << degU+1 << " [ " ; 
  int k;
  // I have to loop since RIB is new line sensitive and 
  // by default a vector adds a new line at the end when using cout
  for(k=0;k<U.n();++k) 
    rib << U[k] << ' ' ;
  rib << " ] " << U[0] << ' ' << U[U.n()-1] << ' ' << P.cols() << ' ' << degV+1 << " [ " ;
  for(k=0;k<V.n();++k)
    rib << V[k] << ' ' ;
  rib << " ] " << V[0] << ' ' << V[V.n()-1] << " \"Pw\" [ " ;
  for(int j=0;j<P.cols();++j)
    for(int i=0;i<P.rows();++i)
      rib << P(i,j) ;
  rib << " ]\n" ;
  return rib.good() ;
}

/*!
  \brief Writes a NuPatch for render man

  Writes a file whith follows the RIB protocol of RenderMan.
  It generates a file which views the whole object.
  The material used for rendering is plastic.

  \latexonly
	       \begin{center}
	       The RenderMan \Pisymbol{psy}{226} 
	       Interface Procedures and RIB Protocol are:
	       Copyright 1988, 1989, Pixar.  All rights reserved.
	       RenderMan \Pisymbol{psy}{226} is a registered trademark of 
	       Pixar.
	       \end{center}
  \endlatexonly
  \htmlonly
  <verbatim>
     The RenderMan (R) Interface Procedures and RIB Protocol are:
	       Copyright 1988, 1989, Pixar.  All rights reserved.
	       RenderMan (R) is a registered trademark of Pixar.
  </verbatim>   
  \endhtmlonly

  \param filename  the file to write to
  \param col  the color of the object
  \param view  the view point

  \return 0 if an error occurs, 1 otherwise

  \author Philippe Lavoie
  \date 8 October, 1997
*/
template <class T, int N>
int NurbsSurface<T,N>::writeRIB(const char* filename, const Color& col, const Point_nD<T,N>& view) const {
  ofstream fout(filename) ;
  if(!fout)
    return 0;
  
  // The following code is based on Listing 8.2 from the RenderMan Companion
  // http://pete.cs.caltech.edu/RMR/Pixar/ch8/listing8_2.c
  int i,j ;

  // aimZ
  MatrixRT<double> Rx ;
  double xzlen, yzlen, yrot, xrot;

  //
  // The initial rotation about the y axis is given by the projection of
  // the direction vector onto the x,z plane: the x and z components
  // of the direction. 
  xzlen = sqrt(view.x()*view.x()+view.z()*view.z()) ;
  if(xzlen == 0)    
    yrot = (view.y() < 0) ? M_PI : 0;
  else
    yrot = acos(view.y()/xzlen);
  
  //
  // The second rotation, about the x axis, is given by the projection on
  // the y,z plane of the y-rotated direction vector: the original y
  // component, and the rotated x,z vector from above. 
  //
  yzlen = sqrt(view.y()*view.y()+xzlen*xzlen);
  xrot = acos(xzlen/yzlen);       /* yzlen should never be 0 */

  // A rotation around y first 
  if (view.y() > 0){
    if(view.x()>0)
      Rx.rotate(xrot,yrot,0.0) ;
    else
      Rx.rotate(xrot,-yrot,0.0) ;
  }
  else{
    if(view.x()>0)
      Rx.rotate(-xrot,yrot,0.0) ;
    else
      Rx.rotate(-xrot,-yrot,0.0) ;
  }

  Point_nD<T,N> minP, maxP ;
  minP.x() = extremum(1,coordX) ;
  minP.y() = extremum(1,coordY) ;
  minP.z() = extremum(1,coordZ) ;
  maxP.x() = extremum(0,coordX) ;
  maxP.y() = extremum(0,coordY) ;
  maxP.z() = extremum(0,coordZ) ;

  Point_nD<T,N> lookAt  ;
  lookAt.x() = (minP.x()+maxP.x())/2.0 ;
  lookAt.y() = (minP.y()+maxP.y())/2.0 ;
  lookAt.z() = (minP.z()+maxP.z())/2.0 ;

  Point_nD<T,N> camera1, camera2 ;

  Point_nD<T,N> q1 = minP-lookAt ;
  Point_nD<T,N> q2 = maxP-lookAt ;
  T D1 = absolute(dot(q1,view))/norm(view) ;
  T D2 = absolute(dot(q2,view))/norm(view) ;

  T a1 = norm(q1)*cos(angle(view,q1)) ;
  T a2 = norm(q2)*cos(angle(view,q2)) ;

  T b1 = D1/tan(18.0*M_PI/180.0) ;
  T b2 = D2/tan(18.0*M_PI/180.0) ; // this gives the 36 degree angle

  camera1 = lookAt+(a1+b1)*view.unitLength() ;
  camera2 = lookAt+(a2+b2)*view.unitLength() ;

  Point_nD<T,N> camera ;

  if(norm(camera1-lookAt)>norm(camera2-lookAt))
    camera = camera1 ;
  else
    camera = camera2 ;

  char front[1024] ;

  char *ext ;
  ext = strstr(filename,".rib") ;
  if(ext){
    for(i=0;i<1024;++i){
      if(&filename[i] == ext)
	break ;
      else
	front[i] = filename[i] ;
      if(!front[i])
	break ;
    }
  }
  else{
    strcpy(front,filename) ;
  }
  

  fout << "##RenderMan RIB-Structure 1.0\n" ;
  fout << "#" << filename << endl;
  fout << "Format 400 400 1\n";
  fout << "Display \"" << front << ".tif\" \"file\" \"rgba\"\n" ;
  fout << "Projection \"perspective\" \"fov\" [36]\n" ;
  fout << "Translate 0 0 " << norm(camera-lookAt) << endl ;
  fout << "Option \"render\" \"prmanspecular\" [1]\n" ;

  fout << "\nWorldBegin\n" ;
  fout << "LightSource \"ambientlight\" 0 \"intensity\" [0.3]\n" ;
  fout << "LightSource \"distantlight\" 1 \"to\" [ " << view.x() << ' ' << view.y() << ' ' << view.z()  << "]\n" ;
  //fout << "LightSource \"spotlight\" 1 \"intensity\" [300] \"from\" [ " ;
  //fout << camera.x()+view.x() << ' ' << camera.y()+view.y() << ' ' << camera.z()+view.z() << " ]\n\n" ;
  fout << "AttributeBegin\nSurface \"plastic\"\n" ;
  fout << "Color [ " << (double)col.r/255.0 << ' ' << (double)col.g/255.0 << ' ' << double(col.b)/255.0 << "]\n" ;

  fout << "Transform [ " ;

  for(j=0;j<4;++j)
    for(i=0;i<4;++i)
      fout << Rx(i,j) << ' ' ;
  fout << "]\n" ;
  fout << "Translate " << -lookAt.x() << ' ' << -lookAt.y() << ' ' << -lookAt.z() << endl ;


  writeRIB(fout) ;

  fout << "AttributeEnd\nWorldEnd\n\n" ;

  return fout.good() ;
}


/*!
  \brief Generates a list of triangles for a surface

  This function is deprecated, please use the NurbsSubSurface class which
  implements everything that this function was suppose to do.

  \param tolerance  the tolerance for the tesselation.
  \param points  the list of points
  \param connect  how the points should be connected

  \author Philippe Lavoie
  \date 8 October, 1997
*/
template <class T, int N>
void NurbsSurface<T,N>::tesselate(T tolerance, BasicList<Point_nD<T,N> > &points, BasicList<int> &connect, BasicList<Point_nD<T,N> > *Norm) const {

#ifdef USE_EXCEPTION
  throw NurbsError() ;
#else
  Error err("NurbsSurface<T,N>::tesselate");
  err << "The tesselate member function is deprecated. Please use\n"
    "the NurbsSubSurface class member functions instead.\n" ; 
  err.fatal();
#endif
}


/*!
  \brief Generates a sphere

  The NURBS surface is now a sphere of radius \a r located
  at \a O.

  \param O  the location of the center of the sphere
  \param r  the radius of the sphere

  \author Philippe Lavoie
  \date 8 May, 1998
*/
template <class T, int N>
void NurbsSurface<T,N>::makeSphere(const Point_nD<T,N>& O, T r) {
  NurbsCurve<T,N> c ;

  const T wm = T(0.707106781185) ;  // sqrt(2)/2

  c.resize(5,2) ; 

  c.modCP(0,HPoint_nD<T,N>(0,0,r,1)) ; 
  c.modCP(1,HPoint_nD<T,N>(-r*wm,0,r*wm,wm)) ; 
  c.modCP(2,HPoint_nD<T,N>(-r,0,0,1)) ; 
  c.modCP(3,HPoint_nD<T,N>(-r*wm,0,-r*wm,wm)) ; 
  c.modCP(4,HPoint_nD<T,N>(0,0,-r,1)) ; 
  
  Vector<T> k(5+2+1) ;
  k[0] = k[1] = k[2] = 0 ;
  k[3] = k[4] = 0.5 ;
  k[5] = k[6] = k[7] = 1 ; 
  
  c.modKnot(k) ; 

  makeFromRevolution(c) ; 
  MatrixRT<T> Tx ;
  Tx.translate(O.x(),O.y(),O.z()) ;
  transform(Tx) ;
}

/*!
  \brief Writes a post-script file representing the curve

  \param filename  the file to write the postscript file to
  \param nu  the number of lines in the U direction
  \param nv  the number of lines in the V direction
  \param camera  where the camera is 
  \param lookAt  where the camera is looking at
  \param plane  where is the projection plane from the camera
  \param cp  a flag indicating if the control points should be 
	     drawn, 0 = no and 1 = yes
  \param magFact  a magnification factor, the 2D point of the control
                  points will be magnified by this value. The size is
		  measured in postscript points. If the magFact is 
		  set to a value smaller or equal to 0, than the 
		  program will try to guess a magnification factor 
		  such that the curve is large enough to fill the 
		  page.
  \param dash  the size of the dash in postscript points . A size 
	smaller or equal to 0 indicates that 
	the line joining the control points is plain.

  \return 0 if an error occurs, 1 otherwise

  \warning If the weights of the curve are not all at 1, the result might 
               not be representative of the true NURBS curve.

  \author Philippe Lavoie
  \date 7 October 1998
*/
template <class T, int N>
int NurbsSurface<T,N>::writePS(const char* filename, int nu, int nv, const Point_nD<T,N>& camera, const Point_nD<T,N>& lookAt, int cp,T magFact,T dash) const {
  NurbsCurveArray<T,N> Ca ;
  if(nu<=0 || nv<=0)
    return 0 ;


  // We need to find the rotation matrix to have z axis along nv
  Point_nD<T,N> np  = lookAt-camera ; 
  np = np.unitLength() ; 
  np *= -1 ;
  
  T rx = atan2(np.z(),np.y()) ;
  T ry = atan2(np.z(),np.x()) ;
  
  MatrixRT<T> Tx(rx,ry,0,camera.x(),camera.y(),camera.z()) ;
  //MatrixRT<T,N> Sc ; Sc.scale(1,1,T(norm(lookAt-camera))/plane) ;
  //MatrixRT<T,N> Tg(Sc*Tx) ; 

  Ca.resize(nu+nv+2) ; 
  int i ; 
  for(i=0;i<=nu;++i){
    T u = U[0]+T(i)*(U[U.n()-1]-U[0])/T(nu) ;
    isoCurveU(u , Ca[i]) ;
    Ca[i].transform(Tx) ; 
  }
  for(;i<=nv+nu+1;++i){
    T v = V[0]+T(i-nu-1)*(V[V.n()-1]-V[0])/T(nv) ;
    isoCurveV(v , Ca[i]) ;
    Ca[i].transform(Tx) ; 
  }


  return Ca.writePS(filename,cp,magFact,dash) ;
}

/*!
  \brief writes a post-script file representing the curve

  Writes the curve in the postscript format to a file, it also
  draws the points defined in \a points with their associated
  vectors if \a vector is used.

  \param filename  the file to write the postscript file to
  \param nu  the number of lines in the U direction
  \param nv  the number of lines in the V direction
  \param camera  where the camera is 
  \param lookAt  where the camera is looking at
  \param plane  where is the projection plane from the camera
  \param points  draws these additional points as empty circles
  \param vectors  specify a vector associated with the points
		   (this can be an empty vector)
  \param cp  a flag indicating if the control points should be 
		         drawn, 0 = no and 1 = yes
  \param magFact  a magnification factor, the 2D point of the control
		  points will be magnified by this value. The size 
		  is measured in postscript points. If the magFact 
		  is set to a value smaller or equal to 0, than the 
		  program will try to guess a magnification factor 
		  such that the curve is large enough to fill the 
		  page.
  \param dash  the size of the dash in postscript points . A size
	smaller or equal to 0 indicates that 
	the line joining the control points is plain.

  \return 0 if an error occurs, 1 otherwise

  \warning If the weights of the curve are not all at 1, the result might 
               not be representative of the true NURBS curve. If vectors is
	       used, then it must be of the same size as points. If a vector
	       element is (0,0,0) it will not be drawn.

  \author Philippe Lavoie
  \date 7 October 1998
*/
template <class T, int N>
int NurbsSurface<T,N>::writePSp(const char*, int nu, int nv, const Point_nD<T,N>& camera, const Point_nD<T,N>& lookAt, const Vector< Point_nD<T,N> >&,const Vector< Point_nD<T,N> >&, int cp,T magFact,T dash) const {
  cerr << "Not implemented. Not sure what is different between this and writePS\n";
  return 0;
}

/*!
  \brief Sends the NURBS Surface to ostream for display

  \return the ostream

  \author Philippe Lavoie
  \date 9 November 1998
*/
template <class T, int N>
ostream& NurbsSurface<T,N>::print(ostream& o) const {
  o << "Degree: " << degU << ' ' << degV << endl;
  o << "U : " << U << endl;
  o << "V: " << V << endl ;
  o << "matrix size: " << P.rows() << ' ' << P.cols() << endl ;
  o << P << endl;
  return o;
}

/*! 
  \brief Computes the parameters for global surface interpolation closed 
         in the u direction

  Computes the parameters for global surface interpolation. 
  For more information, see A9.3 on p377 on the NURBS book.

  \param   Q  the matrix of 3D points (wrapped in the u dir.- rows)
  \param  uk  the knot coefficients in the U direction
  \param  vl  the knot coefficients in the V direction            

  \return 0 if an error occurs, 1 otherwise

  \author Alejandro Frangi
  \date 24 January, 1997
*/
template <class T, int N>
int surfMeshParamsClosedU(const Matrix< Point_nD<T,N> >& Q, Vector<T>& uk, Vector<T>& vl, int degU){

  int n,m,k,l,num ;
  double d,total ;
  Vector<T> cds(Q.rows()) ;

  n = Q.rows() ;
  m = Q.cols() ;
  uk.resize(n) ;
  vl.resize(m) ;
  num = m ;
  
  // Compute the uk
  uk.reset(0) ;

  for(l=0;l<m;l++){
    total = 0.0 ;
    for(k=1;k<=n-degU;k++){
      cds[k] = norm(Q(k,l)-Q(k-1,l)) ;
      total += cds[k] ;
    }
    for(k=n-degU+1; k<n ;k++)
      cds[k] = norm(Q(k,l)-Q(k-1,l)) ;

    if(total==0.0) 
      num-- ;
    else {
      d = 0.0 ;
      for(k=1;k<n;k++){
        d += cds[k] ;
        uk[k] += d/total ;
      }
    }
  }
  if(num==0) 
    return 0 ;
  for(k=1;k<n;k++)
    uk[k] /= num ;

  // Compute the vl
  vl.reset(0) ;
  cds.resize(m) ;

  num = n ;

  for(k=0;k<n;k++){
    total = 0.0 ;
    for(l=1;l<m;l++){
      cds[l] = norm(Q(k,l)-Q(k,l-1)) ;
      total += cds[l] ;
    }
    if(total==0.0) 
      num-- ;
    else {
      d = 0.0 ;
      for(l=1;l<m;l++){
        d += cds[l] ;
        vl[l] += d/total ;
      }
    }
  }
  if(num==0) 
    return 0 ;
  for(l=1;l<m-1;l++)
    vl[l] /= num ;
  vl[m-1] = 1.0 ;


  return 1 ;
}

/*! 
  \biref Computes the parameters for global surface interpolation closed 
         in u dir

  Computes the parameters for global surface interpolation. 
  For more information, see A9.3 on p377 on the NURBS book.

  \param  Q  the matrix of 3D points (wrapped in u dir - rows)
  \param uk  the knot coefficients in the U direction
  \param vl  the knot coefficients in the V direction            

  \return 0 if an error occurs, 1 otherwise

  \author Alejandro Frangi
  \date 24 January, 1997
*/
template <class T, int N>
int surfMeshParamsClosedUH(const Matrix< HPoint_nD<T,N> >& Q, Vector<T>& uk, Vector<T>& vl, int degU){

  int n,m,k,l,num ;
  double d,total ;
  Vector<T> cds(Q.rows()) ;

  n = Q.rows() ;
  m = Q.cols() ;
  uk.resize(n) ;
  vl.resize(m) ;
  num = m ;
  
  // Compute the uk
  uk.reset(0) ;

  for(l=0;l<m;l++){
    total = 0.0 ;
    // Normalization factor
    for(k=1;k<=n-degU;k++){
      cds[k] = norm(Q(k,l)-Q(k-1,l)) ;
      total += cds[k] ;
    }
    for(k=n-degU+1; k<n ;k++)
      cds[k] = norm(Q(k,l)-Q(k-1,l)) ;
    if(total==0.0) 
      num-- ;
    else {
      d = 0.0 ;
      for(k=1;k<n;k++){
        d += cds[k] ;
        uk[k] += d/total ;
      }
    }
  }
  if(num==0) 
    return 0 ;
  for(k=1;k<n;k++)
    uk[k] /= num ;

  // Compute the vl
  vl.reset(0) ;
  cds.resize(m) ;

  num = n ;

  for(k=0;k<n;k++){
    total = 0.0 ;
    for(l=1;l<m;l++){
      cds[l] = norm(Q(k,l)-Q(k,l-1)) ;
      total += cds[l] ;
    }
    if(total==0.0) 
      num-- ;
    else {
      d = 0.0 ;
      for(l=1;l<m;l++){
        d += cds[l] ;
        vl[l] += d/total ;
      }
    }
  }
  if(num==0) 
    return 0 ;
  for(l=1;l<m-1;l++)
    vl[l] /= num ;
  vl[m-1] = 1.0 ;

  return 1 ;
}


/*!
  \brief Generates a closed surface using global interpolation. 

  Generates a NURBS surface using global interpolation. In the u
  direction the curve will be closed and with C(pU-1)
  continuity. Each column in Q indicates the points
  for a closed curve in the u direction. First and
  last point have to be equal.

  \param  Q  a matrix of 3D points (wrapped in u dir. -rows)
  \param pU  the degree of interpolation in the U direction
  \param pV  the degree of interpolation in the V direction

  \author Alejandro Frangi
  \date 30 June, 1998
*/
template <class T, int N>
void NurbsSurface<T,N>::globalInterpClosedU(const Matrix< Point_nD<T,N> >& Q, int pU, int pV){
  Vector<T> vk,uk ;

  resize(Q.rows(),Q.cols(),pU,pV) ;

  surfMeshParamsClosedU(Q,uk,vk,pU) ;
  knotAveragingClosed(uk,pU,U) ;
  knotAveraging(vk,pV,V) ;


  Vector< HPoint_nD<T,N> > Pts(Q.cols()) ;
  NurbsCurve<T,N> R ;
  
  int i,j ;
  for(i=0;i<Q.rows();i++){
    for(j=0;j<Q.cols();j++)
      Pts[j] = Q(i,j) ;
    R.globalInterpH(Pts,vk,V,pV) ;
    for(j=0;j<Q.cols();j++)
      P(i,j) = R.ctrlPnts(j) ;
  }
  
  Pts.resize(Q.rows()) ;
  for(j=0;j<Q.cols();j++){
    for(i=0;i<Q.rows();i++)
      Pts[i] = P(i,j) ;
    
    R.globalInterpClosedH(Pts,uk,U,pU);
    for(i=0;i<Q.rows();i++)
      P(i,j) = R.ctrlPnts(i) ;
  }

}

/*! 
  \brief Generates a surface using global interpolation. 

  Generates a NURBS surface using global interpolation. In the u direction 
  the curve will be closed and with C(pU-1) continuity. Each column in Q
  indicates the points for a closed curve in the u
  direction. First and last point have to be equal.

  \param  Q  a matrix of 4D points (wrapped in u dir. -rows)
  \param pU  the degree of interpolation in the U direction
  \param pV  the degree of interpolation in the V direction

  \author Alejandro Frangi
  \date 30 June, 1998
*/
template <class T, int N>
void NurbsSurface<T,N>::globalInterpClosedUH(const Matrix< HPoint_nD<T,N> >& Q, int pU, int pV){
  Vector<T> vk,uk ;

  resize(Q.rows(),Q.cols(),pU,pV) ;

  surfMeshParamsClosedUH(Q,uk,vk,pU) ;
  knotAveragingClosed(uk,pU,U) ;
  knotAveraging(vk,pV,V) ;
  
  Vector< HPoint_nD<T,N> > Pts(Q.rows()) ;
  NurbsCurve<T,N> R ;
  
  int i,j ;

  for(j=0;j<Q.cols();j++){
    for(i=0;i<Q.rows();i++)
      Pts[i] = Q(i,j) ;
    R.globalInterpH(Pts,uk,U,pU);
    for(i=0;i<Q.rows();i++)
      P(i,j) = R.ctrlPnts(i) ;
  }

  Pts.resize(Q.cols()) ;
  for(i=0;i<Q.rows();i++){
    for(j=0;j<Q.cols();j++)
      Pts[j] = P(i,j) ;
    R.globalInterpClosedH(Pts,vk,V,pV) ;
    for(j=0;j<Q.cols();j++)
      P(i,j) = R.ctrlPnts(j) ;
  }
}


/*! 
  \brief Generates a closed surface using global least squares approximation. 

  Generates a NURBS surface using global least squares
  approximation. This will be closed in the u direction and open
  in the v direction. At u=0("="1) the 
  surface will have C(pU-1) continuity 
  in the u direction.

  \param  Q  a matrix of 3D points (wrapped in u dir. -rows)
  \param pU  the degree of interpolation in the U direction
  \param pV  the degree of interpolation in the V direction
  \param nU  the number of points in the U direction
  \param nV  the number of poitns in the V direction

  \author Alejandro Frangi
  \date 1 August 1998
*/
template <class T, int N>
void NurbsSurface<T,N>::leastSquaresClosedU(const Matrix< Point_nD<T,N> >& Q, int pU, int pV, int nU, int nV){
  Vector<T> vk,uk ;

  resize(nU+pU,nV,pU,pV) ;

  surfMeshParamsClosedU(Q,uk,vk,pU) ;


  Vector< HPoint_nD<T,N> > Pts(Q.rows()) ;
  NurbsCurve<T,N> R ;
  
  int i,j ;

  Matrix< HPoint_nD<T,N> > P2 ;

  P2.resize(nU+pU,Q.cols()) ;

  for(j=0;j<Q.cols();j++){
    for(i=0;i<Q.rows();i++)
      Pts[i] = Q(i,j) ;
    R.leastSquaresClosedH(Pts,pU,nU,uk);
    for(i=0;i<P.rows();i++)
      P2(i,j) = R.ctrlPnts(i) ;
    if(j==0)
      U = R.knot() ;
  }

  Pts.resize(Q.cols()) ;
  for(i=0;i<P.rows();i++){
    for(j=0;j<Q.cols();j++)
      Pts[j] = P2(i,j) ;
    R.leastSquaresH(Pts,pV,nV,vk) ;
    for(j=0;j<P.cols();j++)
      P(i,j) = R.ctrlPnts(j) ;
    if(i==0)
      V = R.knot() ;
  }  

}

/*!
  \brief Write the NURBS surface to a OOGL mesh file

  Writes a OOGL mesh file which represents the surface for the 
  parametric space [fBu,fEu] and [fBv,fEv].
  It does not optimize the number of points required to 
  represent the surface. 

  \param filename  the file name for the output OOGL file
  \param fDu  the parameter step size in u 
  \param fDv  the parameter step size in v
  \param OOGLheader_options  OOGL mesh format header options
               (evrything before the MESH keyword). If you want the
               mesh to be closed in the u/v direction you have to
               give a string containing [u][v] where [] means optional 
               and indicates the direction to be wrapped. See Geomview 
               documentation.
  \param fBu  the initial parameter value in u
  \param fBv  the initial parameter value in v
  \param fEu  the end parameter value in u
  \param fEv  the end parameter value in v
  \param bDrawCP   draws the control points as circles

  \return 1 on success, 0 otherwise

  \warning The parametric surface must be valid
  \author Alejandro Frangi
  \date 19 May, 1998
*/
template <class T, int N>
int NurbsSurface<T,N>::writeOOGL(const char* filename,
				 T fDu,
				 T fDv,
				 T fBu, T fBv,
				 T fEu, T fEv,
				 bool  bDrawCP) const {
  ofstream fout(filename) ;
  if(!fout)
    return 0 ;
  
  // Write file header
  fout << "{ LIST \n";
  fout << "\t{ appearance { shading smooth } \n " ;
  fout << "\tNMESH" << endl;
  T Nu = (fEu-fBu)/fDu + 1;
  T Nv = (fEv-fBv)/fDv + 1;
  fout << "\t"<< Nu << ' ' << Nv << endl;

  // Write mesh vertexes
  Point_nD<T,N> Sp, Np;
  T u,v;
  for (u = fBu; u<fEu+fDu/2; u+=fDu)
    for (v = fBv; v<fEv+fDv/2; v+=fDv){
      Sp = pointAt(u,v);
      Np = normal(u,v);
      Np = (norm(Np)!=0)?Np.unitLength():Point_nD<T,N>(0.0);
      fout << "\t" << Sp << "\t " << Np << endl;
    }
  fout << "\t}" << endl << std::flush;

  // Write the control points 
  if (bDrawCP){
    fout << "\t{ " << endl; 
    fout << "\t  appearance {shading smooth linewidth 5 } " << endl;

    fout << "\t" << "SKEL" << endl; 
    fout <<  P.rows()*P.cols() << ' ' << P.rows()*P.cols() << endl;
    for (int i = 0; i<P.rows(); i++)
      for (int j = 0; j<P.cols(); j++) 
        fout << "\t" << project(P(i,j)) << endl;    
    fout << endl;
    for (int i = 0; i<P.rows()*P.cols(); i++)
      fout << "\t" << "1 " << i << " 0 0 1 0.5 " << endl;
      fout << "\t" << " }" << endl;
  }
  fout << "} " << endl;
  fout << std::flush;

  return 1 ;
}

/*!
  \brief Write the NURBS surface to a mesh file

  This function writes a surface in QUADMESH ascii format to
  interface with Display (Copyright 1993,1994,1995 David MacDonald,
  McConnell Brain Imaging Centre), Montreal Neurological Institute,
  McGill University.

  \param filename  the file name for the output OOGL file
               
  \return 1 on success, 0 otherwise
  
  \warning The parametric surface must be valid

  \author Alejandro Frangi
  \date 19 May, 1998
*/
template <class T, int N>
int NurbsSurface<T,N>::writeDisplayQUADMESH(const char* filename, int iNu,int iNv,const Color& color,T fA, T fO) const 
{

  T fDu = 1/T(iNu);
  T fDv = 1/T(iNv-1);

  ofstream fout(filename) ;
  if(!fout)
    return 0 ;

  // Save the object type
  const char QUADMESH='q'+ ('A' - 'a');
  fout << QUADMESH << ' ';       ;

  // Compute surface properties
  T a, d, s, se, t;

  // Ambient reflectance coefficient
  a  = 0.3;
  // Diffusse reflectance coefficient
  d  = 0.3;
  // Specularity reflectance coeficcient
  s  = 0.4;
  // Specularity reflectance exponent
  se = 10;
  // Opacity
  t  = fO;

  // Save surface properties
  fout << a << ' ' 
       << d << ' ' 
       << s << ' ' 
       << se << ' ' 
       << t  << ' ';

  // Save mesh dimensions  
  fout << iNu << ' ' 
       << iNv << ' ' ;
  
  // Save wrapp status in each direction (v,u)  
  fout << "F T ";

  // New line
  fout << endl ;
  
  // Surface color RGBA (one color for the whole surface)
  T fR= T(color.r)/255;
  T fG= T(color.g)/255;
  T fB= T(color.b)/255;

  /* Colour flag = ONE_COLOUR */
  fout << 0 << ' ' ;
  /* The colour */
  fout << fR << ' '
       << fG << ' '
       << fB << ' '
       << fA << endl;
  
  // New line
  fout << endl ;

  // Now the list of 3D points
  T u,v;
  Point_nD<T,N> Sp;
  for (v = 0; v<1+fDv/2; v+=fDv)
    for (u = 0; u<1-fDu/2; u+=fDu){
      // The change in sign and the swap of y and z coordinates is
      // for conversion to MINC format.
      Sp = -(T)1.0 * pointAt(u,v) ;
      fout << Sp.x() << ' ' << Sp.z() << ' ' << Sp.y() << endl;
    }

  // New line
  fout << endl ;

  // Now the normal vectors
  Point_nD<T,N> Np;
  for (v = 0; v<1+fDv/2; v+=fDv)
    for (u = 0; u<1-fDu/2; u+=fDu){
      Np = normal(u,v);
      Np = (norm(Np)!=0)?Np.unitLength():Point_nD<T,N>(0.0);
      fout << Np.x() << ' ' << Np.z() << ' ' << Np.y() << endl;
    }

  // New line
  fout << endl ;


  return 1;
}

/*!
  \brief Write the NURBS surface to a OOGL mesh file

  Writes a OOGL bezier file which represents the NURBS
  surface decomposed into its Bezier patches.

  \param filename  the file name for the output OOGL file
               
  \return 1 on success, 0 otherwise

  \warning The parametric surface must be valid
  \author Alejandro Frangi
  \date 19 May, 1998
*/
template <class T, int N>
int NurbsSurface<T,N>::writeOOGL(const char* filename) const {
  ofstream fout(filename) ;
  if(!fout)
    return 0 ;

  // Write file header
  int iPointDim = 4;
  fout << "BEZ" << degU << degV << iPointDim << endl;

  // Decompose surface in its Bezier Patches
  NurbsSurfaceArray<T,N> S;
  NurbsSurface<T,N>      surface(*this);
  surface.decompose(S);

  // Write patch vertexes
  for (int iPatch = 0; iPatch < S.n(); iPatch++){
    for(int iu = 0; iu < degU + 1; iu++){
      for(int iv = 0; iv < degV + 1; iv++)
        fout << S[iPatch].ctrlPnts(iu,iv).x() << ' ' 
             << S[iPatch].ctrlPnts(iu,iv).y() << ' '  
             << S[iPatch].ctrlPnts(iu,iv).z() << ' '
             << S[iPatch].ctrlPnts(iu,iv).w() << endl;
    }
    fout << endl;    
  }
  fout << std::flush;

  return 1 ;
}


/*!
  \brief Wraps d points to the end of a point matrix in a given direction

  Qw contains the same points that Q and wraps the end is 
  padded with the first d points from Q

  \param Q  a matrix of 3D points 
  \param d  number of wrapped points
  \param dir  direction 0=rows, 1=cols
  \param Qw  a wrapped matrix of 4D points

  \author    Alejandro Frangi
  \date 14 July, 1998
*/
template <class T, int N>
void wrapPointMatrix(const Matrix< Point_nD<T,N> >& Q, int d, int dir,  Matrix< Point_nD<T,N> >& Qw){
  int i, row, col;

  Qw = Q;

  if (dir==0){
    //    cout << " Wrapping in U dir " << endl << std::flush ;
    Qw.resizeKeep(Q.rows()+d,Q.cols());
    for (col=0; col < Q.cols(); col++)
      for (i=0; i<d; i++)
        Qw(i+Q.rows(),col) = Q(i,col);
  }
  else{
    //    cout << " Wrapping in V dir " << endl << std::flush ;
    Qw.resizeKeep(Q.rows(),Q.cols()+d);
    for (row=0; row < Q.rows(); row++)
      for (i=0; i<d; i++)
        Qw(row,i+Q.cols()) = Q(row,i);
  }
  
} 

/*!
  \brief Compute the derivatives functions at \a u,v of the basis  functions of the NURBS surface
  \relates NurbsCurve, nurbsDersBasisFuns

  \param dU, dV  the degrees of the derivation
  \param u,v  the parametric values
  \param uspan,vspan  the span for the basis functions
  \param Niku  A matrix containing the derivatives of the
basis functions in the u direction.
  \param Njku  A matrix containing the derivatives of the
basis functions in the v direction.
  
  \warning \a dU,dV, \a u,v and \a uspan and vspan must be
in a valid range.
  \author Alejandro Frangi
  \date 15 January 1998
*/
template <class T, int N>
void NurbsSurface<T,N>::dersBasisFuns(T u, T v, int dU, int dV, int uspan, int vspan, Matrix<T> & Niku, Matrix<T> & Njkv ) const {
  // Get derivatives
  nurbsDersBasisFuns(dU,u,uspan,degU,U,Niku) ;
  nurbsDersBasisFuns(dV,v,vspan,degV,V,Njkv) ;
}

/*!
  \brief Generates a torus

  The NURBS surface is now a torus with major radius \a R, minor 
  radius \a r and located at \a O. The torus goes around the z-axis.
  This routine is an adaptation of a routine created by John W. Peterson.

  \param O  the location of the center of the torus
  \param R  the major radius of the torus
  \param r  the minor radius of the torus

  \author Philippe Lavoie
  \date 4 May, 1999
*/
template <class T, int N>
void NurbsSurface<T,N>::makeTorus(const Point_nD<T,N>& O, T R, T r) {
  // These define the shape of a unit torus centered about the origin. 
  T xvalues[] = { 0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0, 1.0, 0.0 };
  T yvalues[] = { 1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0 };
  T zvalues[] = { 0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0 };
  T offsets[] = { -1.0, -1.0, 0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0 };

  // Piecewise Bezier knot vector for a quadratic curve with four segments 
  T knotsMem[] = { 0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75 , 1, 1, 1 };
  Vector<T> knots(knotsMem,12) ;

  resize(9,9,2,2);

  int i, j;

  double r2over2 = sqrt( 2.0 ) / 2.0;
  double weight;

  for (i = 0; i < 9; i++){
    for (j = 0; j < 9; j++) {
      HPoint_nD<T,N> hp ;
      weight = ((j & 1) ? r2over2 : 1.0) * ((i & 1) ? r2over2 : 1.0);
      // Notice how the weights are pre-multiplied with the x, y and z values
      P(i,j).x() = xvalues[j]* (R + offsets[i] * r) * weight;
      P(i,j).y() = yvalues[j]* (R + offsets[i] * r) * weight;
      P(i,j).z() = (zvalues[i] * r) * weight;
      P(i,j).w() = weight;
    }
  }

  // The knot vectors define piecewise Bezier segments 
  // (the same in both U and V).
  
  U = knots ;
  V = knots ; 

  MatrixRT<T> Tx ;
  Tx.translate(O.x(),O.y(),O.z()) ;
  transform(Tx) ;  
}


} // end namespace

