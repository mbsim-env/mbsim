/*=====================================================================
        File: nurbs.cpp
     Purpose:       
    Revision: $Id: nurbs.cpp,v 1.3 2002/05/24 17:27:24 philosophil Exp $
      Author: Philippe Lavoie          (3 Oct, 1996)
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
#include <nurbs.h>
#include <fstream>
#include <string.h>
#include <nurbsS.h>
#include "integrate.h"

#include <malloc.h>

/*!
 */
namespace PLib {

/*!
  \brief default constructor
  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
NurbsCurve<T,N>::NurbsCurve(): P(1),U(1),deg_(0)
{
}

/*!
  \brief A copy constructor.

  \param nurb the NURBS curve to copy

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
NurbsCurve<T,N>::NurbsCurve(const NurbsCurve<T,N>& nurb): 
  ParaCurve<T,N>(), P(nurb.P),U(nurb.U),deg_(nurb.deg_)
{
}

/*!
  \brief Resets a NURBS curve to new values

  \param P1  the new values for the control points
  \param U1  the new values for the knot vector
  \param Degree the new degree of the curve

  \warning The size of P1,U1 and Degree must agree: P.n()+degree+1=U.n()
  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::reset(const Vector< HPoint_nD<T,N> >& P1, const Vector<T> &U1, int Degree) {
  int nSize = P1.n() ;
  int mSize = U1.n() ;
  deg_ = Degree ;
  if(nSize != mSize-deg_-1){
#ifdef USE_EXCEPTION
    throw NurbsSizeError(P1.n(),U1.n(),Degree) ;
#else
    Error err("reset");
    err << "Invalid input size for the control points and the knot vector when reseting a Nurbs Curve.\n";
    err << nSize << " control points  and " << mSize << " knots\n" ;
    err.fatal() ;
#endif
  }
  P.resize(P1.n()) ;
  U.resize(U1.n()) ;
  P = P1 ;
  U = U1 ;
}

/*!
  \brief Constructor with control points in 4D

  \param P1  the control points
  \param U1  the knot vector
  \param Degree  the degree of the curve

  \warning The size of P1,U1 and Degree must agree: P.n()+degree+1=U.n()

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
NurbsCurve<T,N>::NurbsCurve(const Vector< HPoint_nD<T,N> >& P1, const Vector<T> &U1, int Degree): P(P1), U(U1), deg_(Degree) 
{

  if(P.n() != U.n()-deg_-1){
#ifdef USE_EXCEPTION
    throw NurbsSizeError(P.n(),U.n(),deg_) ;
#else
    Error err("NurbsCurve(P1,U1,Degree)");
    err << "Invalid input size for the control points and the knot vector.\n";
    err << P.n() << " control points  and " << U.n() << " knots\n" ;
    err.fatal() ;
#endif
  }
}

/*!
  \brief Constructor with control points in 3D

  \param P1 --> the control point vector
  \param W --> the weight for each control points
  \param U1 --> the knot vector
  \param Degree --> the degree of the curve

  \warning The size of P1,U1 and Degree must agree: P.n()+degree+1=U.n()

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
NurbsCurve<T,N>::NurbsCurve(const Vector< Point_nD<T,N> >& P1, const Vector<T>& W, const Vector<T>& U1, int Degree): P(P1.n()), U(U1), deg_(Degree)
{
  int nSize = P1.n() ;
  int mSize = U1.n() ;

  if(nSize != mSize-deg_-1){
#ifdef USE_EXCEPTION
    throw NurbsSizeError(P.n(),U.n(),deg_) ;
#else
    Error err("NurbsCurve(P1,W,U1,Degree)") ;
    err << "Invalid input size for the control points and the knot vector.\n" ;
    err << nSize << " control points  and " << mSize << " knots\n" ;
    err.fatal() ;
#endif
  }
  if(nSize != W.n()){
#ifdef USE_EXCEPTION
    throw NurbsInputError(nSize,W.n()) ;
#else
    Error err("NurbsCurve(P1,W,U1,Degree)") ;
    err << "Size mismatched between the control points and the weights\n" ;
    err << "ControlPoints size = " << nSize << ", Weight size = " << W.n() << endl ;
    err.fatal() ;
#endif
  }

  for(int i = 0 ;i<nSize;i++){
    const Point_nD<T,N>& pt = P1[i] ; // This makes the SGI compiler happy
    for(int j=0;j<N;j++)
      P[i].data[j] = pt.data[j] * W[i] ;
    P[i].w() = W[i] ;
  }
}

/*!
  \brief The assignment operator for a NURBS curve

  \param curve  the NURBS curve to copy
  \return A reference to itself

  \warning The curve being copied must be valid, otherwise strange 
           results might occur.
  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
NurbsCurve<T,N>& NurbsCurve<T,N>::operator=(const NurbsCurve<T,N>& curve) {
  if(curve.U.n() != curve.P.n()+curve.deg_+1){
#ifdef USE_EXCEPTION
    throw NurbsSizeError(curve.P.n(),curve.U.n(),curve.deg_) ;
#else
    Error err("operator=") ;
    err << "Invalid assignment... the curve being assigned to isn't valid\n" ;
    err.fatal() ;
#endif
  }
  deg_ = curve.deg_ ;
  U = curve.U ;
  P = curve.P ;
  if(U.n()!=P.n()+deg_+1){
#ifdef USE_EXCEPTION
    throw NurbsSizeError(P.n(),U.n(),deg_) ;
#else
    Error err("operator=") ;
    err << "Error in assignment... couldn't assign properly the vectors\n" ;
    err.fatal() ;
#endif
  }
  return *this ;
}


/*!
  \brief draws a NURBS curve on an image

  This will draw very primitively the NURBS curve on an image.
  The drawing assumes the line is only in the xy plane (the z 
  is not used for now). 

  The algorithm finds the points on the curve at a \a step
  parametric intervall between them and join them by a line.
  No fancy stuff.

  \param Img <-- draws the nurbs curve to this Image
  \param color --> the line is drawn in this color
  \param step --> the parametric distance between two computed points.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::drawImg(Image_UBYTE& Img,unsigned char color,T step){
  Point_nD<T,N> a1,a2 ;
  T u_max = U[U.n()-1-deg_] ;
  if(step<=0)
    step = 0.01 ;
  a1 = this->pointAt(U[deg_]) ;
  T u ;
  int i1,j1,i2,j2 ;
  getCoordinates(a1,i1,j1,Img.rows(),Img.cols()) ;
  for(u=U[deg_]+step ; u < u_max+(step/2.0) ; u+=step){ // the <= u_max doesn't work
    a2 = this->pointAt(u) ;
    if(!getCoordinates(a2,i2,j2,Img.rows(),Img.cols()))
      continue ;
    Img.drawLine(i1,j1,i2,j2,color) ;
    i1 = i2 ;
    j1 = j2 ;
  }
  a2 = this->pointAt(U[P.n()]) ;
  if(getCoordinates(a2,i2,j2,Img.rows(),Img.cols()))
     Img.drawLine(i1,j1,i2,j2,color) ;
}

/*!
  \brief Draws a NURBS curve on an image

  This will draw very primitively the NURBS curve on an image.
  The drawing assumes the line is only in the xy plane (the z 
  is not used for now). 
  
  The algorithm finds the points on the curve at a \a step
  parametric intervall between them and join them by a line.
  No fancy stuff.

  \param Img  draws the nurbs curve to this Image
  \param color  the line is drawn in this color
  \param step  the parametric distance between two computed points.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::drawImg(Image_Color& Img,const Color& color,T step){
  Point_nD<T,N> a1,a2 ;
  T u_max = U[U.n()-1-deg_] ;
  if(step<=0)
    step = 0.01 ;
  a1 = this->pointAt(U[deg_]) ;
  int i1,j1,i2,j2 ;
  getCoordinates(a1,i1,j1,Img.rows(),Img.cols()) ;
  T u ;
  for(u=U[deg_]+step ; u < u_max+(step/2.0) ; u+=step){ // the <= u_max doesn't work
    a2 = this->pointAt(u) ;
    if(!getCoordinates(a2,i2,j2,Img.rows(),Img.cols()))
      continue ;
    Img.drawLine(i1,j1,i2,j2,color) ;
    i1 = i2 ;
    j1 = j2 ;
  }
  a2 = this->pointAt(U[P.n()]) ;
  if(getCoordinates(a2,i2,j2,Img.rows(),Img.cols()))
    Img.drawLine(i1,j1,i2,j2,color) ;
}

/*!
  \brief Draws an anti-aliased NURBS curve on an image
  
  This will draw the NURBS by using a circular brush profile. The
  drawing is performed by averaging the intensity of the 
  profile at the pixels.
	      
  \param Img  draws the nurbs curve to this Image
  \param color  the line is drawn in this color
  \param precision  this number influences the number of points used for
	            averaging purposes.
  \param alpha  a flag indicating if the profile is used as an alpha
                chanel. If so, the line doesn't overwrite, it blends
		the line with the image already present in Img.

  \warning This routine is very \e slow; use normal drawing for speed.

  \author Philippe Lavoie 
  \date 25 July 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::drawAaImg(Image_Color& Img, const Color& color, int precision, int alpha){
  NurbsCurve<T,3> profile ;

  profile.makeCircle(Point_nD<T,3>(0,0,0),Point_nD<T,3>(1,0,0),Point_nD<T,3>(0,0,1),1.0,0,M_PI) ;
  drawAaImg(Img,color,profile,precision,alpha) ;
}

/*!
  \brief draws an anti-aliased NURBS curve on an image

  This will draw the NURBS by using a user-defined brush profile.
  The drawing is performed by averaging the intensity of the 
  profile at the pixels.
	      
  \param Img  draws the nurbs curve to this Image
  \param color  the line is drawn in this color
  \param profile  the profile of the NURBS curve to draw
  \param precision  this number influences the number of points used for
	            averaging purposes.
  \param alpha  a flag indicating if the profile is used as an alpha
	        chanel. If so, the line doesn't overwrite, it blends
		the line with the image already present in Img.

  \warning This routine is very \e slow; use normal drawing for speed.

  \author Philippe Lavoie 
  \date 22 August 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::drawAaImg(Image_Color& Img, const Color& color, const NurbsCurve<T,3>& profile, int precision, int alpha){
  Vector< HPoint_nD<T,3> > sPts(2) ;
  sPts[0] = sPts[1] = HPoint_nD<T,3>(1,1,1,1) ;
  Vector<T> sKnot(4) ;
  sKnot[0] = sKnot[1] = 0.0 ;
  sKnot[2] = sKnot[3] = 1.0 ;
  
  NurbsCurve<T,3> scaling(sPts,sKnot,1) ;  

  drawAaImg(Img,color,profile,scaling,precision,alpha) ;
}

/*!
  \brief Draws an anti-aliased NURBS curve on an image

  This will draw the NURBS by using a brush profile. The
  drawing is performed by averaging the intensity of the 
  profile at the pixels. 
  
  This function generates a sweep surface by using the profile
  given in its argument. The sweep is always performed by 
  following the y-axis of the profile. A scaling function
  is also used when sweeping. This is used to vary the shape
  of the profile while it's being swept (see the sweep member
  function of NurbsSurface<T,N> for more details).

  \param Img  draws the nurbs curve to this Image
  \param color  the line is drawn in this color
  \param profile  the profile of the NURBS curve to draw
  \param scaling  the scaling to give the profile while drawing the curve
  \param precision  this number influences the number of points used for
	            averaging purposes.
  \param alpha  a flag indicating if the profile is used as an alpha
	        chanel. If so, the line doesn't overwrite, it blends
		the line with the image already present in Img.

  \warning This routine is very \e slow; use normal drawing for speed 
               or lower the precision factor.

  \author Philippe Lavoie 
  \date 25 July 1997
*/
template <class T, int N>
NurbsSurface<T,3> NurbsCurve<T,N>::drawAaImg(Image_Color& Img, const Color& color, const NurbsCurve<T,3>& profile, const NurbsCurve<T,3>& scaling, int precision, int alpha){
  Matrix<T> addMatrix ;
  Matrix_INT nMatrix ;

  addMatrix.resize(Img.rows(),Img.cols()) ;
  nMatrix.resize(Img.rows(),Img.cols()) ;

  int i,j ;

  T du,dv ;
  // compute a coarse distance for the curve
  Point_nD<T,N> a,b,c ;
  a = pointAt(0.0) ;
  b = pointAt(0.5) ;
  c = pointAt(1.0) ;

  T distance = norm(b-a) + norm(c-b) ;

  dv = distance*T(precision) ;
  dv = (U[U.n()-1]-U[0])/dv ;

  // compute a coarse distance for the trajectory
  Point_nD<T,3> a2,b2,c2 ;
  a2 = profile.pointAt(0.0) ;
  b2 = profile.pointAt(0.5) ;
  c2 = profile.pointAt(1.0) ;
  distance = norm(b2-a2) + norm(c2-b2) ;
  du = distance*T(precision) ;
  du = (profile.knot()[profile.knot().n()-1]-profile.knot()[0])/du ;

  NurbsSurface<T,3> drawCurve ;
  NurbsCurve<T,3> trajectory ; 

  to3D(*this,trajectory) ; 

  drawCurve.sweep(trajectory,profile,scaling,P.n()-1) ;

  T u,v ;

  for(u=U[0];u<U[U.n()-1];u+=du)
    for(v=profile.knot()[0];v<profile.knot()[profile.knot().n()-1];v+=dv){
      Point_nD<T,3> p ;
      p = drawCurve.pointAt(u,v) ;
      if(getCoordinates(p,i,j,Img.rows(),Img.cols())){
	addMatrix(i,j) += p.z() ;
	nMatrix(i,j) += 1 ;
      }
    }

  T maxP = 1.0 ;
  for(i=0;i<Img.rows();++i)
    for(j=0;j<Img.cols();++j){
      addMatrix(i,j) /= T(nMatrix(i,j)) ;
      if(addMatrix(i,j)>maxP)
	maxP = addMatrix(i,j) ;
    }

  for(i=0;i<Img.rows();++i)
    for(j=0;j<Img.cols();++j){
      if(nMatrix(i,j)){
	double mean = double(addMatrix(i,j))/double(maxP) ;
	if(alpha){
	  Img(i,j).r = (unsigned char)(mean*double(color.r)+(1.0-mean)*double(Img(i,j).r)) ;
	  Img(i,j).g = (unsigned char)(mean*double(color.g)+(1.0-mean)*double(Img(i,j).g)) ;
	  Img(i,j).b = (unsigned char)(mean*double(color.b)+(1.0-mean)*double(Img(i,j).b)) ;
	}
	else{
	  Img(i,j) = mean*color ;
	}
      }
    }
  return drawCurve ;
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
void NurbsCurve<T,N>::transform(const MatrixRT<T>& A){
  for(int i=P.n()-1;i>=0;--i)
    P[i] = A*P[i] ;
}

/*!
  \brief Evaluates the curve in 4D at parameter \a u

  \latexonly
  It evaluates the NURBS curve in 4D at the parametric point 
  $u$.  Using the following equation
  \begin{equation}
  C(u) = \sum_{i=0}^n N_{i,p} P_i \hspace{0.5in} a \leq u \leq b
  \end{equation}
  where $P_i$ are the control points and $N_{i,p}$ are the 
  $p$th degree B-spline basis functions.
  \endlatexonly

  For more details on the algorithm, see A4.1 on page 124 of 
  the Nurbs Book.

  \param u  the parametric value at which the curve is evaluated

  \return the 4D point at \a C(u)

  \warning the parametric value must be in a valid range

  \author Philippe Lavoie 
  \date 24 January, 1997
*/
template <class T, int N>
HPoint_nD<T,N> NurbsCurve<T,N>::operator()(T u) const{
  static Vector<T> Nb ;
  int span = findSpan(u) ;

  basisFuns(u,span,Nb) ;
  
  HPoint_nD<T,N> p(0) ;
  for(int i=deg_;i>=0;--i) {
    p += Nb[i] * P[span-deg_+i] ;
  }
  return p ; 
}

/*!
  \brief Evaluates the curve in homogenous space at parameter \a u

  \latexonly
    It evaluates the NURBS curve in 4D at the parametric point 
    $u$.  Using the following equation
    \begin{equation}
    C(u) = \sum_{i=0}^n N_{i,p} P_i \hspace{0.5in} a \leq u \leq b
    \end{equation}
    where $P_i$ are the control points and $N_{i,p}$ are the 
    $p$th degree B-spline basis functions.
  \endlatexonly    

  For more details on the algorithm, see A4.1 on page 124 of 
  the Nurbs Book.

  \param u  the parametric value at which the curve is evaluated
  \param span  the span of u

  \return  the 4D point at \a C(u)
  
  \warning the parametric value must be in a valid range
  \author Philippe Lavoie    
  \date 24 January, 1997
*/
template <class T, int N>
HPoint_nD<T,N> NurbsCurve<T,N>::hpointAt(T u, int span) const{
  static Vector<T> Nb ;

  basisFuns(u,span,Nb) ;
  
  HPoint_nD<T,N> p(0,0,0,0) ;
  for(int i=deg_;i>=0;--i) {
    p += Nb[i] * P[span-deg_+i] ;
  }
  return p ; 
}

/*!
  \brief Computes the derivative of degree \a d of the curve at 
         parameter \a u
   
   For more information on the algorithm used, see A3.2 on p 93 
   of the NurbsBook.
   
   \param u  the parametric value to evaluate at
   \param d  the degree of the derivative

   \return The derivative \a d in norma space at the parameter \a u

   \warning \a u and \a d must be in a valid range.
   \author  Philippe Lavoie
   \date 24 January, 1997
*/
template <class T, int N>
Point_nD<T,N> NurbsCurve<T,N>::derive3D(T u, int d) const {
  Vector< Point_nD<T,N> > ders ;
  deriveAt(u,d,ders) ;
  return ders[d] ;
}

/*!
  \brief Computes the derivative of degree \a of the curve at parameter \a u

  For more information on the algorithm used, see A3.2 on p 93 
  of the NurbsBook.
  
  \param u  the parametric value to evaluate at
  \param d  the degree of the derivative

  \return The derivative \a d in 4D at the parameter \a u

  \warning \a u and \a d must be in a valid range.

  \author  Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
HPoint_nD<T,N> NurbsCurve<T,N>::derive(T u, int d) const {
  Vector< HPoint_nD<T,N> > ders ;
  deriveAtH(u,d,ders) ;
  return ders[d] ;
}

/*!
  \brief Computes the derivative of degree \a d of the 
         curve at parameter \a u in the homonegeous domain

  For more information on the algorithm used, see A3.2 on p 93 
  of the NurbsBook.

  \param u  the parametric value to evaluate at
  \param d  the degree of the derivative
  \param ders a vector containing the derivatives of the curve at \a u.

  \warning \a u and \a d must be in a valid range.

  \author  Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::deriveAtH(T u,int d, Vector< HPoint_nD<T,N> >& ders) const{
  int du = minimum(d,deg_) ;
  int span ;
  Matrix<T> derF(du+1,deg_+1) ;
  ders.resize(d+1) ;

  span = findSpan(u) ;
  dersBasisFuns(du,u,span,derF) ;
  for(int k=du;k>=0;--k){
    ders[k] = 0 ;
    for(int j=deg_;j>=0;--j){
      ders[k] += derF(k,j)*P[span-deg_+j] ;
    }
  }
}

/*!
  \brief Computes the derivative of degree \a d of the  curve at parameter \a u

  For more information on the algorithm used, see A3.2 on p 93 
  of the NurbsBook.

  \param u  the parametric value to evaluate at
  \param d  the degree of the derivative
  \param span  the span of \a u 
  \param ders  a vector containing the derivatives of the curve at \a u.

  \warning \a u and \a d must be in a valid range.
  \author    Philippe Lavoie    
  \date 9 October, 1998
*/
template <class T, int N>
void NurbsCurve<T,N>::deriveAtH(T u, int d, int span, Vector< HPoint_nD<T,N> >& ders) const{
  int du = minimum(d,deg_) ;
  Matrix<T> derF(du+1,deg_+1) ;
  ders.resize(d+1) ;

  dersBasisFuns(du,u,span,derF) ;
  for(int k=du;k>=0;--k){
    ders[k] = 0 ;
    for(int j=deg_;j>=0;--j){
      ders[k] += derF(k,j)*P[span-deg_+j] ;
    }
  }
}


// Setup the binomial coefficients into th matrix Bin
// Bin(i,j) = (i  j)
// The binomical coefficients are defined as follow
//   (n)         n!
//   (k)  =    k!(n-k)!       0<=k<=n
// and the following relationship applies 
// (n+1)     (n)   ( n )
// ( k ) =   (k) + (k-1)
/*!
  \brief Setup a matrix containing binomial coefficients
  
  Setup the binomial coefficients into th matrix Bin
  \htmlonly
               \[ Bin(i,j) = \left( \begin{array}{c}i \\ j\end{array} \right)\]
	       The binomical coefficients are defined as follow
	       \[ \left(\begin{array}{c}   n \\ k \end{array} \right)= \frac{ n!}{k!(n-k)!} \mbox{for $0\leq k \leq n$} \]
	       and the following relationship applies 
	       \[ \left(\begin{array}{c} n+1 \\ k \end{array} \right) = 
	       \left(\begin{array}{c} n \\ k \end{array} \right) +
	       \left(\begin{array}{c} n \\ k-1 \end{array} \right) \]
  \endhtmlonly

  \param Bin  the binomial matrix
  \author Philippe Lavoie  
  \date 24 January, 1997
*/
template <class T>
void binomialCoef(Matrix<T>& Bin){
  int n,k ;
  // Setup the first line
  Bin(0,0) = 1.0 ;
  for(k=Bin.cols()-1;k>0;--k)
    Bin(0,k) = 0.0 ;
  // Setup the other lines
  for(n=0;n<Bin.rows()-1;n++){
    Bin(n+1,0) = 1.0 ;
    for(k=1;k<Bin.cols();k++)
      if(n+1<k)
	Bin(n,k) = 0.0 ;
      else
	Bin(n+1,k) = Bin(n,k) + Bin(n,k-1) ;
  }
}

/*!
  \brief Computes the derivative at the parameter \a u

  \param u  the parameter at which the derivative is computed
  \param d  the degree of derivation
  \param ders  the vector containing the derivatives of the point at \a u.

  \wanring \a u and \a d must be in a valid range.
  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::deriveAt(T u, int d, Vector< Point_nD<T,N> >& ders) const{
  Vector< HPoint_nD<T,N> > dersW ;
  deriveAtH(u,d,dersW) ;
  Point_nD<T,N> v ;
  int k,i ;
  ders.resize(d+1) ;
  
  static Matrix<T> Bin(1,1) ;
  if(Bin.rows() != d+1){
    Bin.resize(d+1,d+1) ;
    binomialCoef(Bin) ;
  }

  // Compute the derivative at the parmeter u

  for(k=0;k<=d;k++){
    v.x() = dersW[k].x() ;
    v.y() = dersW[k].y() ;
    v.z() = dersW[k].z() ;
    for(i=k ;i>0 ;--i){
      v -= (Bin(k,i)*dersW[i].w())*ders[k-i] ;
    }
    ders[k] = v ;
    ders[k] /= dersW[0].w() ;
  }
}

/*!
  \brief Computes the derivative of the curve at the parameter \a u

  \param u  the parameter at which the derivative is computed
  \param d  the degree of derivation
  \param span  the span of \a u.
  \param ders  the vector containing the derivatives of the point at \a u.

  \warning \a u and $d$ must be in a valid range.
  \author Philippe Lavoie     
  \date 9 October 1998
*/
template <class T, int N>
void NurbsCurve<T,N>::deriveAt(T u, int d, int span, Vector< Point_nD<T,N> >& ders) const{
  Vector< HPoint_nD<T,N> > dersW ;
  deriveAtH(u,d,span,dersW) ;
  Point_nD<T,N> v ;
  int k,i ;
  ders.resize(d+1) ;
  
  static Matrix<T> Bin(1,1) ;
  if(Bin.rows() != d+1){
    Bin.resize(d+1,d+1) ;
    binomialCoef(Bin) ;
  }

  // Compute the derivative at the parmeter u

  for(k=0;k<=d;k++){
    v.x() = dersW[k].x() ;
    v.y() = dersW[k].y() ;
    v.z() = dersW[k].z() ;
    for(i=k ;i>0 ;--i){
      v -= (Bin(k,i)*dersW[i].w())*ders[k-i];
    }
    ders[k] = v ;
    ders[k] /= dersW[0].w() ;
  }
}

/*!
  \brief Computes the normal of the curve at \a u from a vector.

  Computes the normal of the curve at \a u from a vector.
  If the curve lies only in the xy-plane, then calling the
  function with the vector v = (0,0,1) (the z$axis) will 
  yield a proper normal for this curve.

  \param u  the parameter at which the normal is computed
  \param v  the vector to compute the normal with
   
  \return the normal vector in 3D.
  \warning \a u must be in a valid range.
  \author Philippe Lavoie    
  \date 2 September, 1997
*/
template <class T, int N>
Point_nD<T,N> NurbsCurve<T,N>::normal(T u, const Point_nD<T,N>& v) const{
  return crossProduct(firstDn(u),v) ;
}

/*!
  \brief Computes the basis function of the curve

  Computes the \a i basis function of degree \a p  of the curve at 
  parameter \a u. 

  \latexonly
  The basis function is noted as $N_{ip}(u)$.

  The B-spline basis function of $p$-degree is defined as
  \begin{eqnarray}
  N_{i,0}(u) & = & \left\{ \begin{array}{ll} 1 & \mbox{if $u_i \leq u < u_{i+1}$} \\ 0 & \mbox{otherwise}\end{array}\right. \nonumber \\
  N_{i,p}(u) & = & \frac{u-u_i}{u_{i+p}-u_i}N_{i,p-1}(u)+\frac{u_{i+p+1}-u}{u_{i+p+1}-u_{i+1}}N_{i+1,p-1}(u) \nonumber
  \end{eqnarray}
  
  where the $u_i$ define the knot vector $U = \{u_0,\ldots,u_m\}$
  as a nondecreasing sequence of real numbers, {\em i.e.}, 
  $u_i \leq u_{i+1}$ for $i=0,\ldots,m-1$. And $m$ is related
  to the number of control points $n$ and the degree of the curve
  $p$ with the relation $m = n + p + 1$. The knot vector has
  the form

  \begin{equation}
  U=\{\underbrace{a,\ldots,a}_{p+1},u_{p+1},\ldots,u_{m-p-1},\underbrace{b,\ldots,b}_{p+1} \} 
  \end{equation}

  \endlatexonly
  \htmlonly
    You can have more information about this function in the LaTeX version.
  \endhtmlonly

  \param u  the parametric variable
  \param i  specifies which basis function to compute
  \param p  the degree to which the basis function is computed

  \return the value of  \a N_{ip}(u)

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int D>
T NurbsCurve<T,D>::basisFun(T u, int i, int p) const{
  T Nip ;
  T saved,Uleft,Uright,temp ;
  
  if(p<1)
    p = deg_ ;

  if((i==0 && u == U[0]) ||
     (i == U.n()-p-2 && u==U[U.n()-1])){
    Nip = 1.0 ;
    return Nip ;
  }
  if(u<U[i] || u>=U[i+p+1]){
    Nip = 0.0 ;
    return Nip;
  }
  T* N = (T*) alloca((p+1)*sizeof(T)) ; // Vector<T> N(p+1) ;
  

  int j ;
  for(j=p;j>=0;--j){
    if(u>=U[i+j] && u<U[i+j+1]) 
      N[j] = 1.0 ;
    else
      N[j] = 0.0 ;
  }
  for(int k=1; k<=p ; k++){
    if(N[0] == 0.0)
      saved = 0.0 ;
    else
      saved = ( (u-U[i])*N[0])/(U[i+k]-U[i]) ;
    for(j=0;j<p-k+1;j++){
      Uleft = U[i+j+1] ;
      Uright = U[i+j+k+1] ;
      if(N[j+1]==0.0){
	N[j] = saved ;
	saved = 0.0 ;
      }
      else {
	temp = N[j+1]/(Uright-Uleft) ;
	N[j] = saved+(Uright-u)*temp ;
	saved = (u-Uleft)*temp ;
      }
    }
  }
  Nip = N[0] ;

  return Nip ;  
}

// it returns the matrix ders, where ders(n+1,deg+1) and the C'(u) = ders(1,span-deg+j) ;

/*!
  \brief Compute the derivatives functions at \a u of the NURBS curve

  For information on the algorithm, see A2.3 on p72 of the NURBS 
  book.

  The result is stored in the ders matrix, where ders is of 
  size \a (n+1,deg+1) and the derivative 
  N'_i(u) = ders(1,i=span-deg+j) where j=0...deg+1.

  \param  n   the degree of the derivation
  \param  u   the parametric value
  \param span  the span for the basis functions
  \param ders  A matrix containing the derivatives of the curve.

  \warning \a n, \a u and \a span must be valid values.
  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::dersBasisFuns(int n,T u, int span, Matrix<T>& ders) const {
  T* left = (T*) alloca(2*(deg_+1)*sizeof(T)) ;
  T* right = &left[deg_+1] ;
  
  Matrix<T> ndu(deg_+1,deg_+1) ;
  T saved,temp ;
  int j,r ;

  ders.resize(n+1,deg_+1) ;

  ndu(0,0) = 1.0 ;
  for(j=1; j<= deg_ ;j++){
    left[j] = u-U[span+1-j] ;
    right[j] = U[span+j]-u ;
    saved = 0.0 ;
    
    for(r=0;r<j ; r++){
      // Lower triangle
      ndu(j,r) = right[r+1]+left[j-r] ;
      temp = ndu(r,j-1)/ndu(j,r) ;
      // Upper triangle
      ndu(r,j) = saved+right[r+1] * temp ;
      saved = left[j-r] * temp ;
    }

    ndu(j,j) = saved ;
  }

  for(j=deg_;j>=0;--j)
    ders(0,j) = ndu(j,deg_) ;

  // Compute the derivatives
  Matrix<T> a(deg_+1,deg_+1) ;
  for(r=0;r<=deg_;r++){
    int s1,s2 ;
    s1 = 0 ; s2 = 1 ; // alternate rows in array a
    a(0,0) = 1.0 ;
    // Compute the kth derivative
    for(int k=1;k<=n;k++){
      T d ;
      int rk,pk,j1,j2 ;
      d = 0.0 ;
      rk = r-k ; pk = deg_-k ;

      if(r>=k){
	a(s2,0) = a(s1,0)/ndu(pk+1,rk) ;
	d = a(s2,0)*ndu(rk,pk) ;
      }

      if(rk>=-1){
	j1 = 1 ;
      }
      else{
	j1 = -rk ;
      }

      if(r-1 <= pk){
	j2 = k-1 ;
      }
      else{
	j2 = deg_-r ;
      }

      for(j=j1;j<=j2;j++){
	a(s2,j) = (a(s1,j)-a(s1,j-1))/ndu(pk+1,rk+j) ;
	d += a(s2,j)*ndu(rk+j,pk) ;
      }
      
      if(r<=pk){
	a(s2,k) = -a(s1,k-1)/ndu(pk+1,r) ;
	d += a(s2,k)*ndu(r,pk) ;
      }
      ders(k,r) = d ;
      j = s1 ; s1 = s2 ; s2 = j ; // Switch rows
    }
  }

  // Multiply through by the correct factors
  r = deg_ ;
  for(int k=1;k<=n;k++){
    for(j=deg_;j>=0;--j)
      ders(k,j) *= r ;
    r *= deg_-k ;
  }

}

// Computes the non-zero basis functions into N of size deg+1
// The following relationship applies   N[i] <= N[span-deg+i]  for i = 0..deg
// A2.1 on p68 of the Nurbs Book
/*!
  \brief computes the non-zero basis functions of the curve

   Computes the non-zero basis functions and puts the result 
   into \a N. \a N has a size of deg+1. To relate \a N to the basis 
   functions, Basis[span -deg +i] = N[i] for i=0...deg.

   \latexonly
   The B-spline basis function of $p$-degree is defined as
   \begin{eqnarray}
   N_{i,0}(u) & = & \left\{ \begin{array}{ll} 1 & \mbox{if $u_i \leq u < u_{i+1}$} \\ 0 & \mbox{otherwise}\end{array}\right. \nonumber \\
   N_{i,p}(u) & = & \frac{u-u_i}{u_{i+p}-u_i}N_{i,p-1}(u)+\frac{u_{i+p+1}-u}{u_{i+p+1}-u_{i+1}}N_{i+1,p-1}(u) \nonumber
   \end{eqnarray}
   
   where the $u_i$ define the knot vector $U = \{u_0,\ldots,u_m\}$
   as a nondecreasing sequence of real numbers, {\em i.e.}, 
   $u_i \leq u_{i+1}$ for $i=0,\ldots,m-1$. And $m$ is related
   to the number of control points $n$ and the degree of the curve
   $p$ with the relation $m = n + p + 1$. The knot vector has
   the form
   
   \begin{equation}
   U=\{\underbrace{a,\ldots,a}_{p+1},u_{p+1},\ldots,u_{m-p-1},\underbrace{b,\ldots,b}_{p+1} \} 
   \end{equation}
   
   The B-spline basis function are non-zero for at most $p+1$ of
   the $N_{i,p}$. The relationship between the non-zero basis
   functions in N and $N_{i,p}$ is as follows, 
   $N_{span - deg + j, p} = N[j]$ for $j=0,\ldots,deg$. Where
   span is the non-zero span of the basis functions. This
   non-zero span for \a u can be found by calling 
   {\tt findSpan(u)}.

   \endlatexonly
   \htmlonly
    You can find more information in the LaTeX version.
   \endhtmlonly

   \param u  the parametric value
   \param i  the non-zero span of the basis functions
   \param N  the non-zero basis functions

   \warning \a u and \a i must be valid values
   \author Philippe Lavoie 
   \date 24 January 1997
*/
template <class T, int D>
void NurbsCurve<T,D>::basisFuns(T u, int i, Vector<T>& N) const{
  T* left = (T*) alloca(2*(deg_+1)*sizeof(T)) ;
  T* right = &left[deg_+1] ;
  
  T temp,saved ;
   
  N.resize(deg_+1) ;

  N[0] = 1.0 ;
  for(int j=1; j<= deg_ ; j++){
    left[j] = u-U[i+1-j] ;
    right[j] = U[i+j]-u ;
    saved = 0.0 ;
    for(int r=0 ; r<j; r++){
      temp = N[r]/(right[r+1]+left[j-r]) ;
      N[r] = saved+right[r+1] * temp ;
      saved = left[j-r] * temp ;
    }
    N[j] = saved ;
  }
  
}

/*!
  \brief Determines the knot span index

  Determines the knot span for which their exists non-zero basis 
  functions. The span is the index \a k for which the parameter 
  \a u is valid in the [u_k,u_{k+1}] range.
  
  \param u  the parametric value
  \return the span index at \a u.
  \warning \a u must be in a valid range
  \author Philippe Lavoie 
  \date 24 January 1997
  \modified 20 January, 1999 (Alejandro Frangi)
*/
template <class T, int N>
int NurbsCurve<T,N>::findSpan(T u) const{
  if(u>=U[P.n()]) 
    return P.n()-1 ;
  if(u<=U[deg_])
    return deg_ ;

  int low  = 0 ;
  int high = P.n()+1 ; 
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
  \brief Finds the knot \a k for which \a u is in the range [u_k,u_{k+1})

  \param u  parametric value

  \return the index \a k

  \warning \a u must be in a valid range.

  \author    Philippe Lavoie      
  \date 24 January, 1997
*/
template <class T, int N>
int NurbsCurve<T,N>::findKnot(T u) const{
  if(u==U[P.n()])
    return P.n() ;
  for(int i=deg_+1; i<P.n() ; i++)
    if(U[i]>u){
      return i-1 ;
    }
  return -1 ;
}


/*!
  \brief Finds the multiplicity of a knot
  \param  r  the knot to observe
  \return the multiplicity of the knot

  \warning \a r must be a valid knot index
  \author  Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
int NurbsCurve<T,N>::findMult(int r) const {
  int s=1 ;
  for(int i=r;i>deg_+1;--i)
    if(U[i]<=U[i-1])
      s++ ;
    else
      return s ;
  return s ;
}

/*!
  \brief Finds the multiplicity of a knot at a parametric value

  Finds the index of the knot at parametric value \a u and 
  returns its multiplicity.

  \param u  the parametric value
  \param r  the knot of interest
  \param s  the multiplicity of this knot

  \warning \a u must be in a valid range.

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::findMultSpan(T u, int& r, int& s) const {
  r = findKnot(u) ;
  if(u==U[r]){
    s = findMult(r) ;
  }
  else
    s = 0 ;
}

/*!
  \brief Resizes a NURBS curve

  Resizes a NURBS curve. The old values are lost and new ones
  have to be created.

  \param n  the new number of control points for the curve
  \param Deg  the new degree for the curve

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::resize(int n, int Deg){
  deg_ = Deg ;
  P.resize(n) ;
  U.resize(n+deg_+1) ;
}

/*!
  \brief A least squares curve approximation

  \latexonly
  This routine solves the following problem: find the NURBS curve
  $C$ satisfying
	       \begin{itemize}
	       \item $Q_0 = C(0)$ and $Q_m = C(1)$
	       \item the remaining $Q_k$ are approximated in the least squares
	           sense, {\em i.e.}
		   \[ \sum_{k=1}^{m-1} | Q_k-C(\bar{u}_k)|^2 \]
		   in a minimum with respect to the $n$ variable $P_i$; the
		   $\bar{u}$ are the parameter values computed with the 
		   chord length method.
	       \end{itemize}

	       The resulting curve will generally not pass through $Q_k$ and
	       $C(\bar{u}_k)$ is not the closest point on $C(u)$ to $Q_k$.
  \endlatexonly
  \htmlonly
     This routines generates a curve that approrimates the points in the 
     least square sense, you can find more details in the LaTeX version.
  \endhtmlonly

  For more details, see section 9.4.1 on page 491 of the NURBS 
  book.
  
  \param Q  the vector of 3D points
  \param degC  the degree of the curve
  \param n  the number of control points in the new curve.

  \warning \a deg must be smaller than Q.n().
  \author Philippe Lavoie   
  \date 24 January, 1997
*/
template <class T, int N>
int NurbsCurve<T,N>::leastSquares(const Vector< Point_nD<T,N> >& Q, int degC, int n){
  Vector<T> ub(Q.n()) ;

  chordLengthParam(Q,ub) ;

  return leastSquares(Q,degC,n,ub) ;
}

/*!
  \brief  A least squares curve approximation

  \latexonly
   This routine solves the following problem: find the NURBS curve
               $C$ satisfying
	       \begin{itemize}
	       \item $Q_0 = C(0)$ and $Q_m = C(1)$
	       \item the remaining $Q_k$ are approximated in the least squares
	           sense, {\em i.e.}
		   \[ \sum_{k=1}^{m-1} | Q_k-C(\bar{u}_k)|^2 \]
		   in a minimum with respect to the $n$ variable $P_i$; the
		   $\bar{u}$ are the precomputed parameter values.
	       \end{itemize}

	       The resulting curve will generally not pass through $Q_k$ and
	       $C(\bar{u}_k)$ is not the closest point on $C(u)$ to $Q_k$.
  \endlatexonly
  \htmlonly
     This routines generates a curve that approrimates the points in the 
     least square sense, you can find more details in the LaTeX version.
  \endhtmlonly

  For more details, see section 9.4.1 on page 491 of the NURBS 
  book.

  \param Q  the vector of 3D points
  \param degC  the degree of the curve
  \param n  the number of control points in the new curve
  \param ub  the knot coefficients

  \warning the variable curve \b must contain a valid knot vector.
  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
int NurbsCurve<T,N>::leastSquares(const Vector< Point_nD<T,N> >& Q, int degC, int n, const Vector<T>& ub){
  int i,j;
  T d,a ;

  if(ub.n() != Q.n()){
#ifdef USE_EXCEPTION
    throw NurbsInputError(ub.n(),Q.n()) ;
#else
    Error err("leastSquares");
    err << "leastSquaresCurve\n" ;
    err << "ub size is different than Q's\n" ;
    err.fatal() ;
#endif
  }

  deg_ = degC ;
  U.resize(n+deg_+1) ;

  // Changing the method to generate a U compare to the one 
  // described by Piegl and Tiller in the NURBS book (eq 9.69)

  U.reset(1.0) ;
  d = (T)(Q.n())/(T)(n) ; 
  for(j=0;j<=deg_;++j)
    U[j] = 0 ;

  for(j=1;j<n-deg_;++j){
    U[deg_+j] = 0.0 ;
    for(int k=j;k<j+deg_;++k){
      i = (int)(k*d) ;
      a = T(k*d)-T(i) ;
      int i2 = (int)((k-1)*d) ;
      U[deg_+j] += a*ub[i2]+(1-a)*ub[i] ;
      }
    U[deg_+j] /= deg_ ;
  }

  return leastSquares(Q, degC, n, ub, U) ;
}

/*!
  \brief A least squares curve approximation

  \latexonly
   This routine solves the following problem: find the NURBS curve
               $C$ satisfying
	       \begin{itemize}
	       \item $Q_0 = C(0)$ and $Q_m = C(1)$
	       \item the remaining $Q_k$ are approximated in the least squares
	           sense, {\em i.e.}
		   \[ \sum_{k=1}^{m-1} | Q_k-C(\bar{u}_k)|^2 \]
		   in a minimum with respect to the $n$ variable $P_i$; the
		   $\bar{u}$ are the precomputed parameter values.
	       \end{itemize}

	       The resulting curve will generally not pass through $Q_k$ and
	       $C(\bar{u}_k)$ is not the closest point on $C(u)$ to $Q_k$.
  \endlatexonly
  \htmlonly
     This routines generates a curve that approrimates the points in the 
     least square sense, you can find more details in the LaTeX version.
  \endhtmlonly

  For more details, see section 9.4.1 on page 491 of the NURBS 
  book.

  \param Q  the vector of 4D points
  \param degC  the degree of the curve
  \param n  the number of control points in the new curve
  \param ub  the knot coefficients

  \warning the variable curve \b must contain a valid knot vector.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
int NurbsCurve<T,N>::leastSquaresH(const Vector< HPoint_nD<T,N> >& Q, int degC, int n, const Vector<T>& ub){
  int i,j;
  T d,a ;

  if(ub.n() != Q.n()){
#ifdef USE_EXCEPTION
    throw NurbsInputError(ub.n(),Q.n()) ;
#else
    Error err("leastSquares");
    err << "leastSquaresCurve\n" ;
    err << "ub size is different than Q's\n" ;
    err.fatal() ;
#endif
  }

  deg_ = degC ;
  U.resize(n+deg_+1) ;

  // Changing the method to generate a U compare to the one 
  // described by Piegl and Tiller in the NURBS book (eq 9.69)

  U.reset(1.0) ;
  d = (T)(Q.n())/(T)(n) ; 
  for(j=0;j<=deg_;++j)
    U[j] = 0 ;

  for(j=1;j<n-deg_;++j){
    U[deg_+j] = 0.0 ;
    for(int k=j;k<j+deg_;++k){
      i = (int)(k*d) ;
      a = T(k*d)-T(i) ;
      int i2 = (int)((k-1)*d) ;
      U[deg_+j] += a*ub[i2]+(1-a)*ub[i] ;
      }
    U[deg_+j] /= deg_ ;
  }

  return leastSquaresH(Q, degC, n, ub, U) ;
}

/*!
  \brief  A least squares curve approximation
  
  \latexonly
   This routine solves the following problem: find the NURBS curve
               $C$ satisfying
	       \begin{itemize}
	       \item $Q_0 = C(0)$ and $Q_m = C(1)$
	       \item the remaining $Q_k$ are approximated in the least squares
	           sense, {\em i.e.}
		   \[ \sum_{k=1}^{m-1} | Q_k-C(\bar{u}_k)|^2 \]
		   in a minimum with respect to the $n$ variable $P_i$; the
		   $\bar{u}$ are the precomputed parameter values.
	       \end{itemize}

	       The resulting curve will generally not pass through $Q_k$ and
	       $C(\bar{u}_k)$ is not the closest point on $C(u)$ to $Q_k$.

  \endlatexonly
  \htmlonly
     This routines generates a curve that approrimates the points in the 
     least square sense, you can find more details in the LaTeX version.
  \endhtmlonly

  For more details, see section 9.4.1 on page 491 of the NURBS 
  book.

  \param Q  the vector of 3D points
  \param degC  the degree of the curve
  \param n  the number of control points in the new curve
  \param ub  the knot coefficients
  \param knot  the knot vector to use for the curve

  \return 1 if succesfull, 0 it the number of points to approximate 
          the curve with is too big compared to the number of points.
  \warning the variable curve \b must contain a valid knot vector.
  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int D>
int NurbsCurve<T,D>::leastSquares(const Vector< Point_nD<T,D> >& Q, int degC, int n, const Vector<T>& ub, const Vector<T>& knot){
  int i,j,span;
  const int& m=Q.n() ;

  if(ub.n() != Q.n()){
#ifdef USE_EXCEPTION
    throw NurbsInputError(ub.n(),Q.n()) ;
#else
    Error err("leastSquares");
    err << "leastSquaresCurve\n" ;
    err << "ub size is different than Q's\n" ;
    err.fatal();
#endif
  }

  if(knot.n() != n+degC+1){
#ifdef USE_EXCEPTION
    throw NurbsSizeError(n,knot.n(),degC) ;
#else
    Error err("leastSquares");
    err << "The knot vector supplied doesn't have the proper size.\n" ;
    err << "It should be n+degC+1 = " << n+degC+1 << " and it is " << knot.n() << endl ;
    err.fatal() ;
#endif
  }

  deg_ = degC ;

  U = knot ;

  P.resize(n) ;

  Vector< Point_nD<T,D> > R(n),rk(m) ; 
  Vector<T> funs(deg_+1) ;
  Matrix_DOUBLE N(m,n) ;
  R[0] = Q[0] ;
  R[n-1] = Q[m-1] ;
  N(0,0) = 1.0 ;
  N(m-1,n-1) = 1.0 ;

  // Set up N 
  N(0,0) = 1.0 ;
  N(m-1,n-1) = 1.0 ;

  //  for(i=1;i<m-1;i++){
  for(i=0;i<m;i++){
     span = findSpan(ub[i]) ;
     basisFuns(ub[i],span,funs);
     for(j=0;j<=deg_;++j){ // BOOO
       //if(span-deg_+j>0)
	 N(i,span-deg_+j) = (double)funs[j] ;
     }
     rk[i] = Q[i]-N(i,0)*Q[0]-N(i,n-1)*Q[m-1] ;  

  }

  // Set up R
  //  for(i=1;i<n-1;i++){
  for(i=0;i<n;i++){
    R[i] = 0.0 ;
    //    for(j=1;j<m-1;j++)
    for(j=0;j<m;j++)
      R[i] += N(j,i)*rk[j] ;
    if(R[i].x()*R[i].x()<1e-10 && 
       R[i].y()*R[i].y()<1e-10 &&
       R[i].z()*R[i].z()<1e-10)
      return 0 ; 
  }

  // Solve      N^T*N*P = R

  // must check for the case where we want a curve of degree 1 having
  // only 2 points.
  if(n-2>0){ 
    Matrix_DOUBLE X(n-2,D),B(n-2,D),Ns(m-2,n-2) ;
    for(i=0;i<B.rows();i++){
      for(j=0;j<D;j++)
	B(i,j) = (double)R[i+1].data[j] ;
    }
    Ns = N.get(1,1,m-2,n-2) ;
    
    solve(transpose(Ns)*Ns,B,X) ;

    for(i=0;i<X.rows();i++){
      for(j=0;j<X.cols();j++)
	P[i+1].data[j] = (T)X(i,j) ;
      P[i+1].w() = 1.0 ;
    }
  }
  P[0] = Q[0] ;
  P[n-1] = Q[m-1] ;
  return 1 ;
}

/*!
  \brief  A least squares curve approximation

  \latexonly
   This routine solves the following problem: find the NURBS curve
               $C$ satisfying
	       \begin{itemize}
	       \item $Q_0 = C(0)$ and $Q_m = C(1)$
	       \item the remaining $Q_k$ are approximated in the least squares
	           sense, {\em i.e.}
		   \[ \sum_{k=1}^{m-1} | Q_k-C(\bar{u}_k)|^2 \]
		   in a minimum with respect to the $n$ variable $P_i$; the
		   $\bar{u}$ are the precomputed parameter values.
	       \end{itemize}

	       The resulting curve will generally not pass through $Q_k$ and
	       $C(\bar{u}_k)$ is not the closest point on $C(u)$ to $Q_k$.

  \endlatexonly
  \htmlonly
     This routines generates a curve that approrimates the points in the 
     least square sense, you can find more details in the LaTeX version.
  \endhtmlonly

  For more details, see section 9.4.1 on page 491 of the NURBS 
  book.

  \param Q  the vector of 4D points
  \param degC  the degree of the curve
  \param n  the number of control points in the new curve
  \param ub  the knot coefficients
  \param knot  the knot vector to use for the curve

  \return 1 if succesfull, 0 it the number of points to approximate 
          the curve with is too big compared to the number of points.

  \warning the variable curve \b must contain a valid knot vector.
  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int D>
int NurbsCurve<T,D>::leastSquaresH(const Vector< HPoint_nD<T,D> >& Q, int degC, int n, const Vector<T>& ub, const Vector<T>& knot){
  int i,j,span,m ;

  m = Q.n() ;

  if(ub.n() != Q.n()){
#ifdef USE_EXCEPTION
    throw NurbsInputError(ub.n(),Q.n()) ;
#else
    Error err("leastSquares");
    err << "leastSquaresCurve\n" ;
    err << "ub size is different than Q's\n" ;
    err.fatal();
#endif
  }

  if(knot.n() != n+degC+1){
#ifdef USE_EXCEPTION
    throw NurbsSizeError(n,knot.n(),degC) ;
#else
    Error err("leastSquares");
    err << "The knot vector supplied doesn't have the proper size.\n" ;
    err << "It should be n+degC+1 = " << n+degC+1 << " and it is " << knot.n() << endl ;
    err.fatal() ;
#endif
  }

  deg_ = degC ;

  U = knot ;

  P.resize(n) ;

  Vector< HPoint_nD<T,D> > R(n),rk(m) ; 
  Vector<T> funs(deg_+1) ;
  Matrix_DOUBLE N(m,n) ;
  R[0] = Q[0] ;
  R[n-1] = Q[m-1] ;
  N(0,0) = 1.0 ;
  N(m-1,n-1) = 1.0 ;

  // Set up N 
  N(0,0) = 1.0 ;
  N(m-1,n-1) = 1.0 ;

  //  for(i=1;i<m-1;i++){
  for(i=0;i<m;i++){
     span = findSpan(ub[i]) ;
     basisFuns(ub[i],span,funs);
     for(j=0;j<=deg_;j++){
       //if(span-deg_+j>0)
	 N(i,span-deg_+j) = (double)funs[j] ;
     }
     rk[i] = Q[i]-N(i,0)*Q[0]-N(i,n-1)*Q[m-1] ;  

  }

  // Set up R
  //  for(i=1;i<n-1;i++){
  for(i=0;i<n;i++){
    R[i] = 0.0 ;
    //    for(j=1;j<m-1;j++)
    for(j=0;j<m;j++)
      R[i] += N(j,i)*rk[j] ;
    if(R[i].x()*R[i].x()<1e-10 && 
       R[i].y()*R[i].y()<1e-10 &&
       R[i].z()*R[i].z()<1e-10)
      return 0 ; 
  }

  // Solve      N^T*N*P = R

  // must check for the case where we want a curve of degree 1 having
  // only 2 points.
  if(n-2>0){ 
    Matrix_DOUBLE X(n-2,D+1),B(n-2,D+1),Ns(m-2,n-2) ;
    for(i=0;i<B.rows();i++){
      for(j=0;j<D+1;j++)
	B(i,j) = (double)R[i+1].data[j] ;
    }
    Ns = N.get(1,1,m-2,n-2) ;
    
    solve(transpose(Ns)*Ns,B,X) ;
    
    for(i=0;i<X.rows();i++){
      for(j=0;j<X.cols();j++)
	P[i+1].data[j] = (T)X(i,j) ;
      P[i+1].w() = 1.0 ;
    }
  }
  P[0] = Q[0] ;
  P[n-1] = Q[m-1] ;
  return 1 ;
}

/*!
  \brief Get the knot removal error bound for an internal knot 

  Get the knot removal error bound for an internal knot r 
  (non-rational). For more information on the algorithm, see 
  A9.8 from the Nurbs book on page 428.


  \param curve  a NURBS curve
  \param r  the index of the internal knot to check
  \param s  the multiplicity of that knot

  \return The maximum distance between the new curve and the old one

  \author    Philippe Lavoie 
  \date 24 January, 1997
*/
template <class T, int N>
T NurbsCurve<T,N>::getRemovalBnd(int r, int s ) const{
  Vector< HPoint_nD<T,N> > temp(U.rows()) ;
  int ord = deg_+1 ;
  int last = r-s ;
  int first = r-deg_ ;
  int off ; 
  int i,j,ii,jj ;
  T alfi,alfj ;
  T u ;

  u = U[r] ;

  off = first-1;
  temp[0] = P[off] ;
  temp[last+1-off] = P[last+1] ;

  i=first ; j=last ;
  ii=1 ; jj=last-off ;

  while(j-i>0){
    alfi = (u-U[i])/(U[i+ord]-U[i]) ;
    alfj = (u-U[j])/(U[j+ord]-U[j]) ;
    temp[ii] = (P[i]-(1.0-alfi)*temp[ii-1])/alfi ; 
    temp[jj] = (P[j]-alfj*temp[jj+1])/(1.0-alfj) ;
    ++i ; ++ii ;
    --j ; --jj ;
  }
  if(j-i<0){
    return distance3D(temp[ii-1],temp[jj+1]) ;
  }
  else{
    alfi=(u-U[i])/(U[i+ord]-U[i]) ;
    return distance3D(P[i],alfi*temp[ii+1]+(1.0-alfi)*temp[ii-1]) ;
  }
  
}

/*!
  \brief Removes an internal knot from a curve.
  This is A5.8 on p185 from the NURB book modified to not check for 
  tolerance before removing the knot.

  \param r  the knot to remove
  \param s  the multiplicity of the knot
  \param num  the number of times to try to remove the knot

  \warning r \b must be an internal knot.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::removeKnot(int  r, int s, int num)
{
  int m = U.n() ;
  int ord = deg_+1 ;
  int fout = (2*r-s-deg_)/2 ;
  int last = r-s ;
  int first = r-deg_ ;
  T alfi, alfj ;
  int i,j,k,ii,jj,off ;
  T u ;

  Vector< HPoint_nD<T,N> > temp( 2*deg_+1 ) ;

  u = U[r] ;

  if(num<1){
#ifdef USE_EXCEPTION
    throw NurbsInputError() ;
#else   
    Error err("removeKnot");
    err << "A knot can only be removed a positive number of times!\n" ;
    err << "num = " << num << endl ;
    err.fatal() ;
#endif
  }

  int t;
  for(t=0;t<num;++t){
    off = first-1 ;
    temp[0] = P[off] ;
    temp[last+1-off] = P[last+1] ;
    i = first; j = last ;
    ii = 1 ; jj = last-off ;
    while(j-i > t){
      alfi = (u-U[i])/(U[i+ord+t]-U[i]) ;
      alfj = (u-U[j-t])/(U[j+ord]-U[j-t]) ;
      temp[ii] = (P[i]-(1.0-alfi)*temp[ii-1])/alfi ;
      temp[jj] = (P[j]-alfj*temp[jj+1])/(1.0-alfj) ;
      ++i ; ++ii ;
      --j ; --jj ;
    }
    i = first ; j = last ;
    while(j-i>t){
      P[i] = temp[i-off] ;
      P[j] = temp[j-off] ;
      ++i; --j ;
    }
    --first ; ++last ;
  }
  if(t==0) {
#ifdef USE_EXCEPTION
    throw NurbsError();
#endif
    cerr << "Major error happening... t==0\n" ;
    return ;
  }

  for(k=r+1; k<m ; ++k)
    U[k-t] = U[k] ;
  j = fout ; i=j ; // Pj thru Pi will be overwritten
  for(k=1; k<t; k++)
    if( (k%2) == 1)
      ++i ;
    else
      --j ;
  for(k=i+1; k<P.n() ; k++) {// Shift
    P[j++] = P[k] ; 
  }

  resize(P.n()-t,deg_) ;

  return ;
}


/*!
  \brief Remove knots from a curve without exceeding an error bound

  For more information about the algorithm, see A9.9 on p429 of the NURB book.

  \param ub  the knot coefficients
  \param ek  the error after removing

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::removeKnotsBound(const Vector<T>& ub,
				    Vector<T>& ek, T E){
  Vector<T> Br(U.n()) ;
  Vector_INT S(U.n()) ;
  Vector_INT Nl(U.n());
  Vector_INT Nr(U.n());
  Vector<T> NewError(ub.n()) ;
  Vector<T> temp(ub.n()) ;
  int i,BrMinI ;
  int r,s,Rstart,Rend,k ;
  T BrMin,u,Infinity=1e20 ;

  if(ek.n() != ub.n()){
#ifdef USE_EXCEPTION
    throw NurbsInputError(ek.n(),ub.n());
#else
    Error err("removeKnotsBound");
    err << "Error in removeKnotsBoundCurve\n" ;
    err << "The size of ub and ek mismatch\n" ;
    err.fatal() ;
#endif
  }

  Br.reset(Infinity) ;
  S.reset(0) ;
  s = 1 ;
  for(i=deg_+1;i<P.n();i++){
    if(U[i]<U[i+1]){
      Br[i] = getRemovalBnd(i,s) ;
      S[i] = s ;
      s = 1 ;
    }
    else {
      Br[i] = Infinity ;
      S[i] = 1 ;
      s++ ;
    }
  }

  
  // This might NOT be ok
  Nl[0] = 0 ;
  Nr.reset(ub.n()-1) ;
  for(i=0;i<ub.n();i++){
    int span = findSpan(ub[i]);
    if(!Nl[span]){
      Nl[span] = i ;
    }
    if(i+1<ub.n())
      Nr[span] = i+1 ;
  }

  // set up N
  while(1) {
    BrMinI = Br.minIndex() ;
    BrMin = Br[BrMinI] ;

    if(BrMin == Infinity)
      break ;

    s = S[BrMinI] ;
    r = BrMinI ;

    // Verify the Error for the affected range
    Rstart = maximum(r-deg_,deg_+1) ;
    Rend = minimum(r+deg_-S[r+deg_]+1,P.n()-1) ;
    Rstart = Nl[Rstart] ;
    Rend = Nr[Rend] ;

    int removable ;
    removable = 1 ;
    for(i=Rstart;i<=Rend ; i++){
      // Using eqn 9.81, 9.83 and A2.4 to compute NewError[i]
      T a ;
      s = S[r] ;
      if((deg_+s)%2){
	u = ub[i] ;
	k = (deg_+s+1)/2 ;
	a = U[r]-U[r-k+1] ;
	a /= U[r-k+deg_+2]-U[r-k+1] ;
	NewError[i] = (1.0-a)*Br[r] * basisFun(u,r-k+1);
      }
      else{
	u = ub[i] ;
	k = (deg_+s)/2 ;
	NewError[i] = Br[r] * basisFun(u,r-k);
      }
      temp[i] = NewError[i] + ek[i];
      if(temp[i]>E){
	removable = 0 ;
	Br[r] = Infinity ;
	break ;
      }
    }
    if(removable){
      // Remove the knot
      removeKnot(r,S[r],1) ;

      // update the error
      for(i=Rstart; i<=Rend ; i++)
	ek[i] = temp[i] ;

      // Break if there is no more interior knot
      if(P.n()<=deg_+1){
	break ;
      }

      // Update the new index range for some of the knots 
      Rstart = Nl[r-deg_-1] ;
      Rend = Nr[r-S[r]] ;
      int span, oldspan ;
      oldspan = -1 ;
      for(k=Rstart;k<=Rend;k++){
	span = findSpan(ub[k]);
	if(span != oldspan){
	  Nl[span] = k ;
	}
	if(k+1<ub.n()) 
	  Nr[span] = k+1 ; 
	oldspan = span ;
      }
      for(k=r-S[r]+1;k<Nl.n()-1;k++){ 
      	Nl[k] = Nl[k+1] ; 
       	Nr[k] = Nr[k+1] ; 
      } 
      Nl.resize(Nl.n()-1) ; // Shrink Nl 
      Nr.resize(Nr.n()-1) ; // Shrink Nr
      // Update the new error bounds for some of the knots
      Rstart = maximum(r-deg_,deg_+1) ;
      Rend = minimum(r+deg_-S[r]+1,P.n()) ;
      s = S[Rstart] ;
      for(i=Rstart;i<=Rend;i++){
	if(U[i]<U[i+1]){
	  Br[i] = getRemovalBnd(i,s) ;
	  S[i] = s ;
	  s = 1;
	}
	else {
	  Br[i] = Infinity ;
	  S[i] = 1 ;
	  s++ ;
	}
      }
      for(i=Rend+1;i<Br.n()-1;i++){
      	Br[i] = Br[i+1] ;
	S[i] = S[i+1] ;
      }
      Br.resize(Br.n()-1) ; // Shrink Br
    }
    else{
      Br[r] = Infinity ;
    }
    
  }

} 
 
/*!
  \relates NurbsCurve
  \brief chord length parameterization

  Performs chord length parameterization from a vector of points.
  \latexonly
      The chord length parameterization works as follows:
	       \begin{itemize} 
	       \item The total chord length is defined as
	             \begin{equation} d = \sum_{k=1}^{n-1} | Q_k-Q_{k-1}| 
		     \end{equation}
	       \item $\bar{u}_0 = 0$ and $\bar{u}_{n-1} = 1$
	       \item \begin{equation}
	              \bar{u}_k = \bar{u}_{k-1}+\frac{| Q_k-Q_{k-1}| }{d}
		      \hspace{0.5in} k=1,\ldots,n-2
	             \end{equation}
	       \end{itemize}
	    Where $n$ is the size of the vector $Q$.
  \endlatexonly
  \htmlonly
    There are more details about this function in the LaTeX version.
  \endhtmlonly

  \param  Q  a vector of 3D points 
  \param ub  the result of chord length parameterization
  \return the total chord length of the points.

  \author    Philippe Lavoie    
  \date 24 January, 1997
*/
template <class T, int N>
T chordLengthParam(const Vector< Point_nD<T,N> >& Q, Vector<T> &ub){
  int i ;
  T d = T(0);

  ub.resize(Q.n()) ;
  ub[0] = 0 ; 
  for(i=1;i<ub.n();i++){
    d += norm(Q[i]-Q[i-1]) ;
  }
  if(d>0){
    for(i=1;i<ub.n()-1;++i){
      ub[i] = ub[i-1] + norm(Q[i]-Q[i-1])/d ;
    }
    ub[ub.n()-1] = 1.0 ; // In case there is some addition round-off
  }
  else{
    for(i=1;i<ub.n()-1;++i)
      ub[i] = T(i)/T(ub.n()-1) ;
    ub[ub.n()-1] = 1.0 ;
  }
  return d ;
}

/*!
  \brief chord length parameterization
  \relates NurbsCurve
  
  Performs chord length parameterization from a vector of points.
  \latexonly
  The chord length parameterization works as follows:
	       \begin{itemize} 
	       \item The total chord length is defined as
	             \begin{equation} d = \sum_{k=1}^{n-1} | Q_k-Q_{k-1}| 
		     \end{equation}
	       \item $\bar{u}_0 = 0$ and $\bar{u}_{n-1} = 1$
	       \item \begin{equation}
	              \bar{u}_k = \bar{u}_{k-1}+\frac{| Q_k-Q_{k-1}| }{d}
		      \hspace{0.5in} k=1,\ldots,n-2
	             \end{equation}
	       \end{itemize}
	       Where $n$ is the size of the vector $Q$.
  \endlatexonly
  \htmlonly
    There is more information about this function in the LaTeX version.
  \endhtmlonly

  \param  Q  a vector of 4D points 
  \param ub  the result of chord length parameterization

  \return the total chord length of the points.
  \author    Philippe Lavoie  
  \date 24 January, 1997
*/
template <class T, int N>
T chordLengthParamH(const Vector< HPoint_nD<T,N> >& Q, Vector<T> &ub){
  int i ;
  T d = 0.0 ;

  ub.resize(Q.n()) ;
  ub[0] = 0 ; 
  for(i=1;i<ub.n();i++){
    d += norm(Q[i]-Q[i-1]) ;
  }
  for(i=1;i<ub.n()-1;i++){
    ub[i] = ub[i-1] + norm(Q[i]-Q[i-1])/d ;
  }
  ub[ub.n()-1] = 1.0 ; // In case there is some addition round-off
  return d ;
}

/*!
  \brief Approximation of a curve bounded to a certain error

  It is a type II approximation: it starts with a lot of control
  points then tries to eliminate as much as it can as long as
  the curve stays within a certain error bound.

  The method uses least squares fitting along with knot
  removal techniques. It is the algorithm A9.10 on p 431 of 
  the NURBS book.
               
  \param  Q  the points to approximate
  \param degree  the degree of the approximation curve
  \param E  the maximum error allowed

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::globalApproxErrBnd(Vector< Point_nD<T,N> >& Q, int degC, T E){
  Vector<T> ub(Q.n()) ;
  chordLengthParam(Q,ub) ;

  globalApproxErrBnd(Q,ub,degC,E) ;
}

/*!
  \brief Approximation of a curve bounded to a certain error

  It is a type II approximation: it starts with a lot of control
  points then tries to eliminate as much as it can as long as
  the curve stays within a certain error bound.

  The method uses least squares fitting along with knot
  removal techniques. It is the algorithm A9.10 on p 431 of 
  the NURBS book.
               
  \param  Q  the points to approximate
  \param ub  the vector of parameters where the points are located
  \param degree  the degree of the approximation curve
  \param E  the maximum error allowed

  \warning ub and Q must be of the same size

  \author Philippe Lavoie 
  \code 24 January 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::globalApproxErrBnd(Vector< Point_nD<T,N> >& Q, Vector<T>& ub, int degC, T E){
  Vector<T> ek(Q.n()) ;
  Vector<T> Uh(Q.n()) ;
  NurbsCurve<T,N> tcurve ;
  int i,j,degL ; 

  if(ub.n() != Q.n()){
#ifdef USE_EXCEPTION
    throw NurbsInputError(ub.n(),Q.n()) ;
#else
    Error err("globalApproxErrBnd");
    err << "The data vector and the parameter vectors are not of the same size!\n" ;
    err << "Q.n() = " << Q.n() << ", ub.n() = " << ub.n() << endl ;
    err.fatal() ;
#endif
  }

  resize(Q.n(),1) ;

  // Initialize U
  deg_ = 1 ;
  for(i=0;i<ub.n();i++){
    U[i+deg_] = ub[i] ;
  }
  U[0] = 0 ;
  U[U.n()-1] = 1.0 ;
  // Initialize P
  for(i=0;i<P.n();i++)
    P[i] = Q[i] ;

  for(degL=1; degL<=degC+1 ; degL++){
    removeKnotsBound(ub,ek,E) ;

    if(degL==degC)
      break ;

    if(degL<degC){

      // Find the degree elevated knot vector
      Uh.resize(U.n()*2) ;

      Uh[0] = U[0] ;
      j = 1 ;
      for(i=1;i<U.n();++i){
	if(U[i]>U[i-1])
	  Uh[j++] = U[i-1] ;
	Uh[j++] = U[i] ;
      }
      Uh[j++] = U[U.n()-1] ;
      Uh.resize(j) ;
      tcurve = *this ;
      if(!leastSquares(Q,degL+1,Uh.n()-degL-1-1,ub,Uh)){
	*this = tcurve ;
	degreeElevate(1);
      }
    }
    else{
      tcurve = *this ;
      if(!leastSquares(Q,degL,P.n(),ub,U)){
	*this = tcurve ;
      }
    }


    // Project the points from curve to Q and update ek and ub
    //    for(i=0;i<Q.n();i++){
    for(i=0;i<Q.n();i++){
      T u_i ;
      Point_nD<T,N> r_i ;
      projectTo(Q[i],ub[i],u_i,r_i) ;
      ek[i] = norm(r_i-Q[i]) ;
      ub[i] = u_i ;
    }
  }
}

/*!
  \brief Approximation of a curve bounded to a certain error

  The algorithm is quite simplistic but in some cases gives
  better result than globalApproxErrBnd (when the error allowed
  is low and the data is very close to each other).
  
  This algorithm generates a first degree interpolation of the
  data points, degree elevates the curve by 1 degree recomputes
  the error around each points, removes the knots which are 
  within a certain error range then repeats the process until
  the desired degree is reached.

  \param Q  the points to approximate
  \param degC  the degree of the approximation curve
  \param E  the maximum error allowed

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::globalApproxErrBnd2(Vector< Point_nD<T,N> >& Q,
				       int degC,
				       T E){
  Vector<T> ub(Q.n()) ;
  Vector<T> ek(Q.n()) ;
  Vector<T> Uh(Q.n()) ;
  NurbsCurve<T,N> tcurve ;
  int i,degL ; 

  resize(Q.n(),1) ;

  chordLengthParam(Q,ub) ;

  // Initialize U
  deg_ = 1 ;
  for(i=0;i<ub.n();i++){
    U[i+deg_] = ub[i] ;
  }
  U[0] = 0 ;
  U[U.n()-1] = 1.0 ;
  // Initialize P
  for(i=0;i<P.n();i++)
    P[i] = Q[i] ;

  for(degL=1; degL<degC ; degL++){
    degreeElevate(1);

    // Project the points from curve to Q and update ek and ub
    //    for(i=0;i<Q.n();i++){
    for(i=0;i<Q.n();i++){
      T u_i ;
      Point_nD<T,N> r_i ;
      projectTo(Q[i],ub[i],u_i,r_i) ;
      ek[i] = norm(r_i-Q[i]) ;
      ub[i] = u_i ;
    }
    removeKnotsBound(ub,ek,E) ;    
  }
}

/*!
  \brief Approximation of a curve bounded to a certain error

  The algorithm is quite simplistic but in some cases gives
  better result than globalApproxErrBnd (when the error allowed
  is low and the data is very close to each other).
  
  This algorithm generates a first degree interpolation of the
  data points, degree elevates it to the degree requested then
  removes all the control points which are within the error
  bound.

  \param Q  the points to approximate
  \param degC  the degree of the approximation curve
  \param E  the maximum error allowed

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::globalApproxErrBnd3(Vector< Point_nD<T,N> >& Q,int degC,T E){
  //NurbsCurve<T,N> tCurve(1) ;
  Vector<T> ub(Q.n()) ;
  Vector<T> ek(Q.n()) ;
  int i ; 

  resize(Q.n(),1) ;

  chordLengthParam(Q,ub) ;

  // Initialize U
  deg_ = 1 ;
  for(i=0;i<ub.n();i++){
    U[i+deg_] = ub[i] ;
  }
  U[0] = 0 ;
  U[U.n()-1] = 1.0 ;

  // Initialize P
  for(i=0;i<P.n();i++)
    P[i] = Q[i] ;
  
  //removeKnotsBoundCurve(curve,ub,ek,E/10.0,curve) ;
  degreeElevate(degC-1) ;
  removeKnotsBound(ub,ek,E) ;
}


/*!
  \brief Approximation of a curve bounded to a certain error

  The algorithm is quite simplistic but in some cases gives
  better result than globalApproxErrBnd (when the error allowed
  is low and the data is very close to each other).
  
  This algorithm generates a first degree interpolation of the
  data points, degree elevates it to the degree requested then
  removes all the control points which are within the error
  bound.

  \param Q  the points to approximate
  \param ub  the parameter vector
  \param degC  the degree of the approximation curve
  \param E  the maximum error allowed

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::globalApproxErrBnd3(Vector< Point_nD<T,N> >& Q, 
				       const Vector<T> &ub,
				       int degC,
				       T E){
  Vector<T> ek(Q.n()) ;
  int i ; 

  resize(Q.n(),1) ;

  // Initialize U
  deg_ = 1 ;
  for(i=0;i<ub.n();i++){
    U[i+deg_] = ub[i] ;
  }
  U[0] = 0 ;
  U[U.n()-1] = 1.0 ;

  // Initialize P
  for(i=0;i<P.n();i++)
    P[i] = Q[i] ;
    
  degreeElevate(degC-1) ;
  removeKnotsBound(ub,ek,E) ;
}


/*!
  \brief project a point onto the curve

  It finds the closest point in the curve to a point $p$.
  For more information, see A6.4 and A6.5 on p 231 of the 
  NURBS book

  \param p  the point \a p being projected
  \param guess  initial guess
  \param u  the parametric value for which \a C(u) is closest to \a p.
  \param r  the point at \a C(u)
  \param e1  the minimal error
  \param e2  the maximal error
  \param maxTry  the maximum number of times to try before returning from the function

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::projectTo(const Point_nD<T,N>& p, T guess, T& u, Point_nD<T,N>& r, T e1, T e2,int maxTry) const{
  T un ;
  T c1, c2;
  Vector< Point_nD<T,N> > Cd ;
  Point_nD<T,N> c, cd,cdd ;
  int t = 0 ;
  u = guess ;

  if(u<U[0]) u = U[0] ;
  if(u>U[U.n()-1]) u = U[U.n()-1] ;

  while(1) {
    ++t ;
    if(t>maxTry){
      r = c ;
      return ;
    }
    c = pointAt(u) ;
    deriveAt(u,2,Cd) ;
    cd = Cd[1] ;
    cdd = Cd[2] ;
    c1 = norm2(c-p) ;
    if(c1<e1*e1){
      r = c ;
      return ;
    }
    c2 = norm((Point_nD<T,N>)(cd*(c-p))) ;
    //c2 *= c2 ;
    c2 /= norm(cd)*norm(c-p) ;
    //if(c2<e2*e2){
    if(c2<e2){
      r =c ;
      return ;
    }
    
    un = u- cd*(c-p)/(cdd*(c-p)+norm2(cd)) ;
    
    if(un<U[0]) un = U[0] ;
    if(un>U[U.n()-1]) un = U[U.n()-1] ;

    if(norm2((un-u)*cd)<e1*e1){
      r = c ;
      return ;
    }
    u = un ;
  }
}

/*!
  \brief degree elevate a curve a number of times

  For more information, see A5.9 on p 206 of the NURBS book
  
  \param t  the number of times to increase the degree of the curve

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::degreeElevate(int t){
  if(t<=0){
    return ;
  }

  NurbsCurve<T,N> c(*this) ;

  int i,j,k ;
  int n = c.ctrlPnts().n()-1;
  int p = c.deg_ ;
  int m = n+p+1;
  int ph = p+t ;
  int ph2 = ph/2 ;
  Matrix<T> bezalfs(p+t+1,p+1) ; // coefficients for degree elevating the Bezier segment
  Vector< HPoint_nD<T,N> > bpts(p+1) ; // pth-degree Bezier control points of the current segment
  Vector< HPoint_nD<T,N> > ebpts(p+t+1) ; // (p+t)th-degree Bezier control points of the  current segment
  Vector< HPoint_nD<T,N> > Nextbpts(p-1) ; // leftmost control points of the next Bezier segment
  Vector<T> alphas(p-1) ; // knot instertion alphas.

  // Compute the binomial coefficients
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

  resize(c.P.n()+c.P.n()*t,ph) ; // Allocate more control points than necessary

  int mh = ph ;
  int kind = ph+1 ;
  T ua = c.U[0] ;
  T ub = 0.0 ;
  int r=-1 ; 
  int oldr ;
  int a = p ;
  int b = p+1 ; 
  int cind = 1 ;
  int rbz,lbz = 1 ; 
  int mul,save,s;
  T alf;
  int first, last, kj ;
  T den,bet,gam,numer ;

  P[0] = c.P[0] ;
  for(i=0; i <= ph ; i++){
    U[i] = ua ;
  }

  // Initialize the first Bezier segment

  for(i=0;i<=p ;i++) 
    bpts[i] = c.P[i] ;

  while(b<m){ // Big loop thru knot vector
    i=b ;
    while(b<m && c.U[b] >= c.U[b+1]) // for some odd reasons... == doesn't work
      b++ ;
    mul = b-i+1 ; 
    mh += mul+t ;
    ub = c.U[b] ;
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
	alphas[k-mul-1] = numer/(c.U[a+k]-ua) ;
      }
      for(j=1;j<=r;j++){
	save = r-j ; s = mul+j ;
	for(k=p;k>=s;k--){
	  bpts[k] = alphas[k-s] * bpts[k]+(1.0-alphas[k-s])*bpts[k-1] ;
	}
	Nextbpts[save] = bpts[p] ;
      }
    }
    
    for(i=lbz;i<=ph;i++){ // Degree elevate Bezier,  only the points lbz,...,ph are used
      ebpts[i] = 0.0 ;
      mpi = minimum(p,i) ;
      for(j=maximum(0,i-t); j<=mpi ; j++)
	ebpts[i] += bezalfs(i,j)*bpts[j] ;
    }

    if(oldr>1){ // Must remove knot u=c.U[a] oldr times
      // if(oldr>2) // Alphas on the right do not change
      //	alfj = (ua-U[kind-1])/(ub-U[kind-1]) ;
      first = kind-2 ; last = kind ;
      den = ub-ua ;
      bet = (ub-U[kind-1])/den ;
      for(int tr=1; tr<oldr; tr++){ // Knot removal loop
	i = first ; j = last ;
	kj = j-kind+1 ;
	while(j-i>tr){ // Loop and compute the new control points for one removal step
	  if(i<cind){
	    alf=(ub-U[i])/(ua-U[i]) ;
	    P[i] = alf*P[i] + (1.0-alf)*P[i-1] ;
	  }
	  if( j>= lbz){
	    if(j-tr <= kind-ph+oldr){
	      gam = (ub-U[j-tr])/den ;
	      ebpts[kj] = gam*ebpts[kj] + (1.0-gam)*ebpts[kj+1] ;
	    }
	    else{
	      ebpts[kj] = bet*ebpts[kj]+(1.0-bet)*ebpts[kj+1] ;
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
    for(j=lbz; j<=rbz ; j++) { // load control points onto the curve
      P[cind++] = ebpts[j] ; 
    }

    if(b<m){ // Set up for next pass thru loop
      for(j=0;j<r;j++)
	bpts[j] = Nextbpts[j] ;
      for(j=r;j<=p;j++)
	bpts[j] = c.P[b-p+j] ;
      a=b ; 
      b++ ;
      ua = ub ;
    }
    else{
      for(i=0;i<=ph;i++)
	U[kind+i] = ub ;
    }
  }
  resize(mh-ph,ph) ; // Resize to the proper number of control points
}

/*!
  \brief It inserts a knot a number of times

  It inserts the knot u, r times and generates the curve nc.
  For more information, see A5.1 on page 151 of the NURBS book

  \param u  the knot to insert
  \param r  the number of times to insert \a u.
  \param nc  the resulting NURBS curve

  \return the number of knots inserted (0 if none)
  
  \warning the routine throws a NurbsError if the u value is not inside
           the clamped range.

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T, int N>
int NurbsCurve<T,N>::knotInsertion(T u, int r,NurbsCurve<T,N>& nc){
  // Compute k and s      u = [ u_k , u_k+1)  with u_k having multiplicity s
  int k=0,s=0 ;
  int i,j ;
  int p = deg_ ;

  if(u<U[deg_] || u>U[P.n()]){
#ifdef USE_EXCEPTION
    throw NurbsError();
#else
    Error err("knotInsertion");
    err << "The parametric value isn't inside a valid range." ; 
    err << "The valid range is between " << U[deg_] << " and " << U[P.n()] << endl ;
    err.fatal();
#endif
  }
  
  for(i=0;i<U.n();i++){
    if(U[i]>u) {
      k = i-1 ;
      break ;
    }
  }

  if(u<=U[k]){
    s = 1 ;
    for(i=k;i>deg_;i--){
      if(U[i]<=U[i-1])
	s++ ;
      else
	break ;
    }
  }
  else{
    s=0 ;
  }

  if((r+s)>p+1)
    r = p+1-s ;

  if(r<=0)
    return 0 ; 

  nc.resize(P.n()+r,deg_) ;

  // Load new knot vector
  for(i=0;i<=k;i++)
    nc.U[i] = U[i] ;
  for(i=1;i<=r;i++)
    nc.U[k+i] = u ;
  for(i=k+1;i<U.n(); i++)
    nc.U[i+r] = U[i] ;

  // Save unaltered control points
  Vector< HPoint_nD<T,N> > R(p+1) ;

  for(i=0; i<=k-p ; i++)
    nc.P[i] = P[i] ;
  for(i=k-s ; i< P.n() ; i++)
    nc.P[i+r] = P[i] ;
  for(i=0; i<=p-s; i++)
    R[i] = P[k-p+i] ;

  // Insert the knot r times
  int L=0 ;
  T alpha ;
  for(j=1; j<=r ; j++){
    L = k-p+j ;
    for(i=0;i<=p-j-s;i++){
      alpha = (u-U[L+i])/(U[i+k+1]-U[L+i]) ;
      R[i] = alpha*R[i+1] + (1.0-alpha)*R[i] ;
    }
    nc.P[L] = R[0] ;
    if(p-j-s > 0)
      nc.P[k+r-j-s] = R[p-j-s] ;
  }

  // Load remaining control points
  for(i=L+1; i<k-s ; i++){
    nc.P[i] = R[i-L] ;
  }
  return r ; 
}

/*!
  \brief Refine the curve knot vector

  For more information, see A5.4 on page 164 of the NURBS book

  \param X the new knots to insert in the knot vector

  \author Philippe Lavoie     
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::refineKnotVector(const Vector<T>& X){
  int n = P.n()-1 ;
  int p = deg_ ;
  int m = n+p+1 ;
  int a,b ;
  int r = X.n()-1 ;

  NurbsCurve<T,N> c(*this) ;

  resize(r+1+n+1,p) ;

  a = c.findSpan(X[0]) ;
  b = c.findSpan(X[r]) ;
  ++b ;
  int j ;
  for(j=0; j<=a-p ; j++)
    P[j] = c.P[j] ;
  for(j = b-1 ; j<=n ; j++)
    P[j+r+1] = c.P[j] ;
  for(j=0; j<=a ; j++)
    U[j] = c.U[j] ;
  for(j=b+p ; j<=m ; j++)
    U[j+r+1] = c.U[j] ;
  int i = b+p-1 ; 
  int k = b+p+r ;
  for(j=r; j>=0 ; j--){
    while(X[j] <= c.U[i] && i>a){
      P[k-p-1] = c.P[i-p-1] ;
      U[k] = c.U[i] ;
      --k ;
      --i ;
    }
    P[k-p-1] = P[k-p] ;
    for(int l=1; l<=p ; l++){
      int ind = k-p+l ;
      T alpha = U[k+l] - X[j] ;
      if(alpha==0.0)
	P[ind-1] = P[ind] ;
      else
	alpha /= U[k+l]-c.U[i-p+l] ;
      P[ind-1] = alpha*P[ind-1] + (1.0-alpha)*P[ind] ;
    }
    U[k] = X[j] ;
    --k ;
  }
}

/*!
  \brief global curve interpolation with points in 3D

  \param Q  the 3D points to interpolate
  \param d  the degree of the interpolation

  \warning The number of points to interpolate must be greater than
               the degree specified for the curve.
  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::globalInterp(const Vector< Point_nD<T,N> >& Q, int d){
  Vector<T> ub ;
  chordLengthParam(Q,ub) ;
  globalInterp(Q,ub,d) ;
}

/*!
  \brief global curve interpolation with points in 3D

  Global curve interpolation with points in 3D and with the parametric values
  specified.

  \param Q  the 3D points to interpolate
  \param ub  the parametric values 
  \param d  the degree of the interpolation

  \warning The number of points to interpolate must be greater than
               the degree specified for the curve.
  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int D>
void NurbsCurve<T,D>::globalInterp(const Vector< Point_nD<T,D> >& Q, const Vector<T>& ub, int d){
  int i,j ;

  if(d<=0){
#ifdef USE_EXCEPTION
    throw NurbsInputError() ;
#else
    Error err("globalInterp");
    err << "The degree specified is equal or smaller than 0\n" ;
    err << "deg = " << deg_ << endl ;
    err.fatal() ;
#endif
  }
  if(d>=Q.n()){
#ifdef USE_EXCEPTION
    throw NurbsInputError() ;
#else
    Error err("globalInterp");
    err << "The degree specified is greater then Q.n()+1\n" ;
    err << "Q.n() = " << Q.n() << ", and deg = " << deg_ << endl ;
    err.warning() ;
    d = Q.n()-1 ;
#endif
  }


  resize(Q.n(),d) ;
  Matrix_DOUBLE A(Q.n(),Q.n()) ;

  knotAveraging(ub,d,U) ;

  // Initialize the basis matrix A
  Vector<T> N(deg_+1) ;
  
  for(i=1;i<Q.n()-1;i++){
    int span = findSpan(ub[i]);
    basisFuns(ub[i],span,N) ;
    for(j=0;j<=deg_;j++) 
	A(i,span-deg_+j) = (double)N[j] ;
  }
  A(0,0)  = 1.0 ;
  A(Q.n()-1,Q.n()-1) = 1.0 ;

  // Init matrix for LSE
  Matrix_DOUBLE qq(Q.n(),D) ;
  Matrix_DOUBLE xx(Q.n(),D) ;
  for(i=0;i<Q.n();i++){
    const Point_nD<T,D>& qp = Q[i] ; // this makes the SGI compiler happy
    for(j=0; j<D;j++)
      qq(i,j) = (double)qp.data[j] ;
  }

  solve(A,qq,xx) ;

  // Store the data
  for(i=0;i<xx.rows();i++){
    for(j=0;j<D;j++)
      P[i].data[j] = (T)xx(i,j) ;
    P[i].w() = 1.0 ;
  }

}

/*!
  \brief global curve interpolation with the 1st derivatives of the points specified.

  Global curve interpolation with the 1st degree derivative
  specified for each of the points to interpolate.
  This will generate a number of control points 2 times greater 
  than the number of interpolation points.

  If the derivative specified is of unit length, \e i.e. the
  tangent vectors are given, then the derivative at each point
  will be multiplied by the chord length. A second multiplicative
  factor can be specified to make the derivative even greater.
  This second number should be close to 1.0. 

  For more information about the algorithm, refer to section
  9.2.4 on page 373 of the NURBS book.

  \param Q  the points to interpolate
  \param D  the first derivative at these points
  \param d  the degree of the curve
  \param unitD  a flag specifying if the derivatives are unit vectors
  \param a  a multiplicative factor for the tangent (must be greater 
	             than 0 or the function aborts).

  \warning The number of points to interpolate must be greater than
               the degree specified for the curve. 
  \author Philippe Lavoie 
  \date 3 September, 1997
*/
template <class T, int nD>
void NurbsCurve<T,nD>::globalInterpD(const Vector< Point_nD<T,nD> >& Q, const Vector< Point_nD<T,nD> >& D, int d, int unitD, T a){
  int i,j,n ;

  if(d<=1){
#ifdef USE_EXCEPTION
    throw NurbsInputError() ;
#else
    Error err("globalInterpD");
    err << "The degree specified is equal or smaller than 1\n" ;
    err << "deg = " << deg_ << endl ;
    err.fatal() ;
#endif
  }
  if(d>=Q.n()){
#ifdef USE_EXCEPTION
    throw NurbsInputError() ;
#else
    Error err("globalInterpD");
    err << "The degree specified is greater then Q.n()+1\n" ;
    err << "Q.n() = " << Q.n() << ", and deg = " << deg_ << endl ;
    err.warning() ;
    d = Q.n()-1 ;
#endif
  }
  if(a<=0){
#ifdef USE_EXCEPTION
    throw NurbsInputError() ;
#else
    Error err("globalInterpD");
    err << "The a value must be greater than 0\n" ;
    err << "It is presently " << a << endl ;
    err.fatal() ;
#endif
  }
  if(Q.n() != D.n()){
#ifdef USE_EXCEPTION
    throw NurbsInputError(Q.n(),D.n()) ;
#else
    Error err("globalInterpD") ;
    err << "The number of points to interpolate is different than\n the number of derivative points.\n" ;
    err << "Q.n() = " << Q.n() << ", D.n() = " << D.n() << endl ;
    err.fatal() ;
#endif
  }

  deg_ = d ;
  n = 2*Q.n() ; 

  resize(n,deg_) ;

  Vector<T> ub(Q.n()) ;

  T chordLength ;

  chordLength = chordLengthParam(Q,ub) ;

  if(unitD)
    chordLength *= a ;

  // Setup up knot vector
  switch(deg_){
  case 2:
    {
      for(i=0;i<=deg_;++i){
	U[i] = T(0) ; 
	U[U.n()-1-i] = T(1) ; 
      }
      for(i=0;i<ub.n()-1;++i){
	U[2*i+deg_] = ub[i] ;
	U[2*i+deg_+1] = (ub[i]+ub[i+1])/T(2) ;
      }
      break ; 
    }
  case 3: 
    {
      for(i=0;i<=deg_;++i){
	U[i] = T(0) ; 
	U[U.n()-1-i] = T(1) ; 
      }
      for(i=1;i<ub.n()-1;++i){
	U[deg_+2*i] = (2*ub[i]+ub[i+1])/T(3) ;
	U[deg_+2*i+1] = (ub[i]+2*ub[i+1])/T(3) ;
      }
      U[4] = ub[1]/T(2);
      U[U.n()-deg_-2] = (ub[ub.n()-1]+T(1))/T(2) ;
    }
  default:
    {
      Vector<T> ub2(2*Q.n()) ; 
      for(i=0;i<ub.n()-1;++i){
	ub2[2*i] = ub[i] ;
	ub2[2*i+1] = (ub[i]+ub[i+1])/2.0 ;
      }
      
      ub2[ub2.n()-2] = (ub2[ub2.n()-1]+ub2[ub2.n()-3])/2.0 ;
      knotAveraging(ub2,deg_,U) ;
    }
  }

  // Initialize the basis matrix A
  Matrix_DOUBLE A(n,n) ;
  Vector<T> N(deg_+1) ;
  Matrix<T> Nd(1,1) ;

  for(i=1;i<Q.n()-1;i++){
    int span = findSpan(ub[i]);
    basisFuns(ub[i],span,N) ;
    dersBasisFuns(1,ub[i],span,Nd) ;
    for(j=0;j<=deg_;j++) {
	A(2*i,span-deg_+j) = (double)N[j] ;
	A(2*i+1,span-deg_+j) = (double)Nd(1,j) ;
    }
  }
  A(0,0)  = 1.0 ;
  A(1,0) = -1.0 ;
  A(1,1) = 1.0 ;
  A(A.rows()-2,A.cols()-2) = -1.0 ;
  A(A.rows()-2,A.cols()-1) = 1.0 ;
  A(A.rows()-1,A.cols()-1) = 1.0 ;

  // Init matrix for LSE
  Matrix_DOUBLE qq(n,nD) ;
  Matrix_DOUBLE xx(n,nD) ;
  for(i=0;i<Q.n();i++){
    const Point_nD<T,nD>& qp = Q[i] ; // this makes the SGI compiler happy
    const Point_nD<T,nD>& dp = D[i] ;
    for(j=0; j<nD;j++){
      qq(2*i,j) = (double)qp.data[j] ;
      qq(2*i+1,j) = (double)dp.data[j] ;
      if(unitD)
	qq(2*i+1,j) *= chordLength ;
    }
  }

  T d0Factor = U[deg_+1]/deg_ ;
  T dnFactor = (1-U[U.n()-deg_-2])/deg_ ;

  // the following three assignment must be made to make the SGI compiler
  // a happy compiler
  const Point_nD<T,nD>& dp0 = D[0] ;
  const Point_nD<T,nD>& dpn = D[D.n()-1] ;
  const Point_nD<T,nD>& qpn = Q[Q.n()-1] ;
  for(j=0;j<nD;++j){
    qq(1,j) = d0Factor*double(dp0.data[j]) ;
    qq(A.cols()-2,j) = dnFactor*double(dpn.data[j]) ;
    if(unitD){
      qq(1,j) *= chordLength ;
      qq(A.cols()-2,j) *= chordLength ;
    }
    qq(A.cols()-1,j) = double(qpn.data[j]) ;
  }

  if(!solve(A,qq,xx)){
#ifdef USE_EXCEPTION
    throw NurbsError();
#else
    Error err("globablInterpD");
    err << "The matrix couldn't not be solved properly. There is no valid"
      " solutions for the input parameters you gave OR there is a "
      " computational error (using double float might solve the problem).";
    err.warning();
#endif
  }

  // Store the data
  for(i=0;i<xx.rows();i++){
    for(j=0;j<nD;j++)
      P[i].data[j] = (T)xx(i,j) ;
    P[i].w() = 1.0 ;
  }

  P[0] = Q[0] ;
  P[P.n()-1] = Q[Q.n()-1] ;
}

/*!
  \brief global curve interpolation with points in 4D

  Global curve interpolation with points in 4D
   
  \param Q  the points in 4D to interpolate
  \param d  the degree of the curve interpolation

  \warning The number of points to interpolate must be greater than
           the degree specified for the curve.
  \author  Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int D>
void NurbsCurve<T,D>::globalInterpH(const Vector< HPoint_nD<T,D> >& Q, int d){
  int i,j ;

  resize(Q.n(),d) ;
  Matrix_DOUBLE A(Q.n(),Q.n()) ;
  Vector<T> ub(Q.n()) ;

  chordLengthParamH(Q,ub) ;

  // Setup the Knot Vector for the curve
  for(i=0;i<=deg_;i++)
    U[i] = 0 ;
  for(i=P.n(); i<U.n(); i++)
    U[i] = 1.0 ;
  for(j=1; j<Q.n()-deg_ ; j++){
    T t=0 ;
    for(i=j; i< j+deg_ ; i++)
      t += ub[i] ;
    U[j+deg_] = t/(T)deg_ ;
  }
  
  // Initialize the basis matrix A
  Vector<T> N(deg_+1) ;

  for(i=1;i<Q.n()-1;i++){
    int span = findSpan(ub[i]);
    basisFuns(ub[i],span,N) ;
    for(j=0;j<=deg_;j++) 
	A(i,span-deg_+j) = (double)N[j] ;
  }
  A(0,0)  = 1.0 ;
  A(Q.n()-1,Q.n()-1) = 1.0 ;

  // Init matrix for LSE
  Matrix_DOUBLE qq(Q.n(),D+1) ;
  Matrix_DOUBLE xx(Q.n(),D+1) ;
  for(i=0;i<Q.n();i++)
    for(j=0; j<D+1;j++)
      qq(i,j) = (double)Q[i].data[j] ;

  solve(A,qq,xx) ;

  // Store the data
  for(i=0;i<xx.rows();i++){
    for(j=0;j<D+1;j++)
      P[i].data[j] = (T)xx(i,j) ;
  }

}

/*!
  \brief global curve interpolation with 4D points and a knot vector defined.

  Global curve interpolation with 4D points and a knot vector 
  defined.

  \param Q  the 3D points to interpolate
  \param Uc  the knot vector to set the curve to
  \param d  the degree of the interpolation   

  \warning The number of points to interpolate must be greater than
               the degree specified for the curve.
  \author    Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int D>
void NurbsCurve<T,D>::globalInterpH(const Vector< HPoint_nD<T,D> >& Q, const Vector<T>& Uc, int d){
  int i,j ;

  resize(Q.n(),d) ;
  Matrix_DOUBLE A(Q.n(),Q.n()) ;
  Vector<T> ub(Q.n()) ;

  if(Uc.n() != U.n()){
#ifdef USE_EXCEPTION
    throw NurbsInputError(Uc.n(),U.n()) ;
#else
    Error err("globalInterp");
    err << "Invalid dimension for the given Knot vector.\n" ;
    err << "U required = " << U.n() << ", U given = " << Uc.n() << endl ;
    err.fatal() ;
#endif
  }
  U = Uc ;
  chordLengthParamH(Q,ub) ;
  
  // Initialize the basis matrix A
  Vector<T> N(deg_+1) ;

  for(i=1;i<Q.n()-1;i++){
    int span = findSpan(ub[i]);
    basisFuns(ub[i],span,N) ;
    for(j=0;j<=deg_;j++) 
	A(i,span-deg_+j) = (double)N[j] ;
  }
  A(0,0)  = 1.0 ;
  A(Q.n()-1,Q.n()-1) = 1.0 ;

  // Init matrix for LSE
  Matrix_DOUBLE qq(Q.n(),D+1) ;
  Matrix_DOUBLE xx(Q.n(),D+1) ;
  for(i=0;i<Q.n();i++)
    for(j=0; j<D+1;j++)
      qq(i,j) = (double)Q[i].data[j] ;

  solve(A,qq,xx) ;

  // Store the data
  for(i=0;i<xx.rows();i++){
    for(j=0;j<D+1;j++)
      P[i].data[j] = (T)xx(i,j) ;
  }

}

/*!
  \brief global curve interpolation with 4D points, 
         a knot vector defined and the parametric value
	 vector defined.

  Global curve interpolation with 4D points, a knot vector
  defined and the parametric value vector defined.

  \param Q  the 3D points to interpolate
  \param ub  the parametric values vector
  \param Uc  the knot vector to set the curve to
  \param d  the degree of the interpolation  		 

  \warning The number of points to interpolate must be greater than
               the degree specified for the curve. Uc must be compatible with 
	       the values given for Q.n(), ub.n() and d.
  \author    Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int D>
void NurbsCurve<T,D>::globalInterpH(const Vector< HPoint_nD<T,D> >& Q, const Vector<T>& ub, const Vector<T>& Uc, int d){
  int i,j ;

  resize(Q.n(),d) ;
  Matrix_DOUBLE A(Q.n(),Q.n()) ;

  if(Uc.n() != U.n()){
#ifdef USE_EXCEPTION
    throw NurbsInputError(Uc.n(),U.n()) ;
#else
    Error err("globalInterp");
    err << "Invalid dimension for the given Knot vector.\n" ;
    err << "U required = " << U.n() << ", U given = " << Uc.n() << endl ;
    err.fatal() ;
#endif
  }
  U = Uc ;
  
  // Initialize the basis matrix A
  Vector<T> N(deg_+1) ;

  for(i=1;i<Q.n()-1;i++){
    int span = findSpan(ub[i]);
    basisFuns(ub[i],span,N) ;
    for(j=0;j<=deg_;j++) 
	A(i,span-deg_+j) = (double)N[j] ;
  }
  A(0,0)  = 1.0 ;
  A(Q.n()-1,Q.n()-1) = 1.0 ;

  // Init matrix for LSE
  Matrix_DOUBLE qq(Q.n(),D+1) ;
  Matrix_DOUBLE xx(Q.n(),D+1) ;
  for(i=0;i<Q.n();i++)
    for(j=0; j<D+1;j++)
      qq(i,j) = (double)Q[i].data[j] ;

  solve(A,qq,xx) ;

  // Store the data
  for(i=0;i<xx.rows();i++){
    for(j=0;j<D+1;j++)
      P[i].data[j] = (T)xx(i,j) ;
  }

}

template <class T, int N>
inline T pow2(T a){
  return a*a ;
}

/*!
  \brief Find the intersection point of two lines

  This routines finds the intersection point of two lines.
  The algorithm generates a plane from one of the lines 
  and finds the intersection point between this plane and the 
  other line.
  
  \param p1 a point in the first line
  \param t1 the tangent at p0t along the line 2
  \param p2 a point in the second line
  \param t2 the tangent at p1 along the line 2
  \param p the intersection point
  
  \return 1 if the lines intersect, 0 if they don't. 

  \author Philippe Lavoie
  \date 25 July, 1997
*/
template <class T, int N>
int intersectLine(const Point_nD<T,N>& p1, const Point_nD<T,N>& t1, const Point_nD<T,N>& p2, const Point_nD<T,N>& t2, Point_nD<T,N>& p){
  // a line is written like 
  // L(t) = Q + u*t
  // u is the parametric value P is a point in the line and T is the tangent
  // a plane is of the form
  // (X-P).v = 0 
  // where P is a point where the plane goes through and v is the normal to it
  // and X is (x,y,z)
  // solving for u

  Point_nD<T,N> v,px ;

  //px = crossProduct(t1,p1-p2) ;
  //v = crossProduct(px,t1) ;
  px = crossProduct(t1,t2) ; 
  v = crossProduct(px,t1) ; 
  
  T t = (p1-p2)*v ;
  T vw = v*t2 ;
  if(to2power(vw)<1e-7)
    return 0 ;
  t /= vw ;
  p = p2+(((p1-p2)*v)/vw)*t2 ;
  return 1 ;
}

#ifdef HAVE_TEMPLATE_OF_TEMPLATE
template <class T>
int intersectLine(const Point_nD<T,2>& p1, const Point_nD<T,2>& t1, const Point_nD<T,2>& p2, const Point_nD<T,2>& t2, Point_nD<T,2>& p){
  cout << "PLEASE, DEFINE THIS FUNCTION\n" ; 

  return 1 ;
}
#else

#ifdef TEMPLATE_SPECIALIZATION

template <>
int intersectLine(const Point_nD<float,2>& p1, const Point_nD<float,2>& t1, const Point_nD<float,2>& p2, const Point_nD<float,2>& t2, Point_nD<float,2>& p){
  cout << "PLEASE, DEFINE THIS FUNCTION\n" ; 

  return 1 ;
}

template <>
int intersectLine(const Point_nD<double,2>& p1, const Point_nD<double,2>& t1, const Point_nD<double,2>& p2, const Point_nD<double,2>& t2, Point_nD<double,2>& p){
  cout << "PLEASE, DEFINE THIS FUNCTION\n" ; 

  return 1 ;
}
#endif //TEMPLATE_SPECIALIZATION

#endif

/*!
  \brief generates a circular curve

  Generates parts of a circle, starting at angle \a as and 
  finishing at \a ae with a radius \a r and having the origin 
  located at \a O. The \a X and \a Y vector describe the local
  x-axis and the local y-axis of the circle.

  The degrees are specified in radians.

  \param O  the center of the circle
  \param X  unit length vector lying in the plane of the circle
  \param Y  unit length vector lying in the plane of the circle
  \param r  the radius of the circle
  \param as  start angle in radians measured with respect to $X$
  \param ae  end angle in radians measured with respect to $X$

  \author Philippe Lavoie
  \date 25 July, 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::makeCircle(const Point_nD<T,N>& O, const Point_nD<T,N>& X, const Point_nD<T,N>& Y, T r, double as, double ae){
  double theta,angle,dtheta ;
  int narcs ;
  while(ae<as)
    ae += 2*M_PI ;
  theta = ae-as ;
  if(theta<=M_PI/2.0)	
    narcs = 1 ;
  else{
    if(theta<=M_PI)
      narcs = 2 ;
    else{
      if(theta<=1.5*M_PI)
	narcs = 3 ;
      else
	narcs = 4 ;
    }
  }
  dtheta = theta/(double)narcs ;
  int n = 2*narcs+1 ; // n control points ;
  double w1 = cos(dtheta/2.0) ; // dtheta/2.0 is base angle

  Point_nD<T,N> P0,T0,P2,T2,P1 ;
  P0 = O + r*cos(as)*X + r*sin(as)*Y ; 
  T0 = -sin(as)*X + cos(as)*Y ; // initialize start values
  resize(n,2) ;
  
  P[0] = P0 ;
  int i ;
  int index = 0 ;
  angle = as ;
  for(i=1;i<=narcs;++i){
    angle += dtheta ;
    P2 = O+ r*cos(angle)*X + r*sin(angle)*Y ;  
    P[index+2] = P2 ;
    T2 = -sin(angle)*X + cos(angle)*Y ;
    intersectLine(P0,T0,P2,T2,P1) ;
    P[index+1] = P1 ;
    P[index+1] *= w1 ;
    index += 2 ;
    if(i<narcs){
      P0 = P2 ;
      T0 = T2 ;
    }
  }
  int j = 2*narcs+1 ; // load the knot vector
  for(i=0;i<3;++i){
      U[i] = 0.0 ;
      U[i+j] = 1.0 ;
  }
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
}

/*!
  \brief generates a circular curve

  Generates a circular curve of radius $r$ at origin $O$. The 
  curve is drawn in the $xy$-axis.

  \param O  the center of the circle
  \param r  the radius of the circle
  \param as  start angle measured with respect to the x-axis
  \param ae  end angle measured with respect to the y-axis

  \author Philippe Lavoie
  \date 25 July, 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::makeCircle(const Point_nD<T,N>& O, T r, double as, double ae){
  makeCircle(O,Point_nD<T,N>(1,0,0),Point_nD<T,N>(0,1,0),r,as,ae) ;
}

/*!
  \brief generates a circular curve

  Generates a circle of radius $r$ at origin $O$. The 
  curve is drawn in the $xy$-axis.

  \param O  the center of the circle
  \param r  the radius of the circle

  \author Philippe Lavoie
  \date 3 May, 1999
*/
template <class T, int N>
void NurbsCurve<T,N>::makeCircle(const Point_nD<T,N>& O, T r){
  resize(9,2);
  U[0] = U[1] = U[2] = 0 ;
  U[3] = U[4] = 0.25 ;
  U[5] = U[6] = 0.50 ;  
  U[7] = U[8] = 0.75 ;
  U[9] = U[10] = U[11] = 1 ;


  const T wm = T(0.707106781185) ;  // sqrt(2)/2

  P[0] = HPoint_nD<T,N>(r,0,0,1) ; 
  P[1] = HPoint_nD<T,N>(r*wm,r*wm,0,wm) ; 
  P[2] = HPoint_nD<T,N>(0,r,0,1) ; 
  P[3] = HPoint_nD<T,N>(-r*wm,r*wm,0,wm) ; 
  P[4] = HPoint_nD<T,N>(-r,0,0,1) ; 
  P[5] = HPoint_nD<T,N>(-r*wm,-r*wm,0,wm) ; 
  P[6] = HPoint_nD<T,N>(0,-r,0,1) ; 
  P[7] = HPoint_nD<T,N>(r*wm,-r*wm,0,wm) ; 
  P[8] = HPoint_nD<T,N>(r,0,0,1) ; 

  for(int i=8;i>=0;--i){
    P[i].x() += O.x() ; 
    P[i].y() += O.y() ; 
    P[i].z() += O.z() ; 
  }


  
}

/*!
  \brief Finds the union of two knot vectors
  \relates NurbsCurve

  Finds the union between two knot vectors
  
  \param Ua knot vector A
  \param Ub knot vector B
  
  \return the union of Ua and Ub
  
  \warning The result is useless unless the knot vectors being compared are 
           from a NURBS curve of a same degree
  \author    Philippe Lavoie
  \date 24 January, 1997
*/
template <class T>
Vector<T> knotUnion(const Vector<T>& Ua, const Vector<T>& Ub) {
  Vector<T> U(Ua.n()+Ub.n()) ;
  int done = 0 ;
  int i,ia,ib ;  
  T t ;

  i = ia = ib = 0 ;
  while(!done){
    if(Ua[ia] == Ub[ib]){
      t = Ua[ia] ;
      ++ia ; ++ib ;
    }
    else{
      if(Ua[ia]<Ub[ib]){
	t = Ua[ia] ;
	++ia ;
      }
      else{
	t = Ub[ib] ;
	++ib ;
      }
    }
    U[i++] = t ;
    done = (ia>=Ua.n() || ib>=Ub.n()) ;
  }
  U.resize(i) ;
  return U ;
}

/*!
  \brief Merges the knot vector of a curve with another knot vector
  
  Will merge the Knot vector U with the one from the curve  
  and it will refine the curve appropriately.
  
  \param Um  the knot vector to merge with

  \warning the knot U must be common with the one from the curve c
  \author    Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::mergeKnotVector(const Vector<T> &Um){
  int i,ia,ib ;
  // Find the knots to insert
  Vector<T> I(Um.n()) ;
  int done = 0 ;
  i = ia = ib = 0 ;
  while(!done) {
    if(Um[ib] == U[ia]){
      ++ib ; ++ia ;
    }
    else{
      I[i++] = Um[ib] ;
      ib++ ;
    }
    done = (ia>=U.n() || ib >= Um.n()) ;
  }
  I.resize(i) ;

  if(I.n()>0){
    // Refine the curve
    refineKnotVector(I) ;
  }
}


/*!
  \brief  Generate compatible curves from an array of curves
  \relates NurbsCurveArray

  This routine will put to the same degree all the curves in 
  the array and it will ensure that they have the same knot 
  vector.

  \param ca  the array of Nurbs curves 
  \warning the knot vector of all the curves must be in the range [0,1]
  \author    Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void generateCompatibleCurves(NurbsCurveArray<T,N> &ca){
  int i;
  NurbsCurve<T,N> tc ;

  if(ca.n()<=1) // Nothing to do... only 1 curve in the array
    return ;

  // Increase all the curves to the highest degree
  int p = 1 ;
  for(i=0;i<ca.n();i++)
    if(p<ca[i].deg_)
      p = ca[i].deg_ ;
  for(i=0;i<ca.n();i++){
    ca[i].degreeElevate(p-ca[i].deg_);
  }

  // Find a common Knot vector
  Vector<T> Uc(ca[0].U) ;
  for(i=1;i<ca.n();i++){
    Uc = knotUnion(Uc,ca[i].U) ;
  }

  // Refine the knot vectors
  for(i=0;i<ca.n();i++)
    ca[i].mergeKnotVector(Uc) ;
}

/*!
  \brief reads a NurbsCurve<T,N> from a file

  \param fin  an input file stream

  \return 0 if an error occurs, 1 otherwise

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T, int N>
int NurbsCurve<T,N>::read(ifstream &fin){
  if(!fin) {
    return 0 ;
  }
  int np,d;
  char *type ;
  type = new char[3] ;
  if(!fin.read(type,sizeof(char)*3)) { delete []type ; return 0 ;}
  int r1 = strncmp(type,"nc3",3) ;
  int r2 = strncmp(type,"nc4",3) ;
  if(!(r1==0 || r2==0)) {
    delete []type ;
    return 0 ;
  }
  int st ;
  char stc ;
  if(!fin.read((char*)&stc,sizeof(char))) { delete []type ; return 0 ;}
  if(!fin.read((char*)&np,sizeof(int))) { delete []type ; return 0 ;}
  if(!fin.read((char*)&d,sizeof(int))) { delete []type ; return 0 ;}

  st = stc - '0' ; 
  if(st != sizeof(T)){ // not of the same type size
    delete []type ;
    return 0 ; 
  }

  resize(np,d) ;
  
  if(!fin.read((char*)U.memory(),sizeof(T)*U.n())) { delete []type ; return 0 ;}
     

  T *p,*p2 ;
  if(!r1){
    p = new T[3*np] ;
    if(!fin.read((char*)p,sizeof(T)*3*np)) { delete []type ; return 0 ;}
    p2 = p ;
    for(int i=0;i<np;i++){
      P[i].x() = *(p++) ;
      P[i].y() = *(p++) ;
      P[i].z() = *(p++) ;
      P[i].w() = 1.0 ;
    }
    delete []p2 ;
  }
  else{
    p = new T[4*np] ;
    if(!fin.read((char*)p,sizeof(T)*4*np)) { delete []type ; return 0 ;}
    p2 = p ;
    for(int i=0;i<np;i++){
      P[i].x() = *(p++) ;
      P[i].y() = *(p++) ;
      P[i].z() = *(p++) ;
      P[i].w() = *(p++) ;
    }
    delete []p2 ;
  }

  delete []type ;
  return 1 ;
}

/*!
  \brief Reads a NurbsCurve<T,N> from a file.

  \param filename the filename to read the curve from 
  \return 0 if an error occurs, 1 otherwise

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T, int N>
int NurbsCurve<T,N>::read(const char* filename){
  ifstream fin(filename) ;
  if(!fin) {
    return 0 ;
  }
  return read(fin) ;
}

/*!
  \brief Writes a NurbsCurve<T,N> to a file.

  \param filename  the filename to write to.
  \return 0 if an error occurs, 1 otherwise

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T, int N>
int NurbsCurve<T,N>::write(const char* filename) const {
  ofstream fout(filename) ;  

  if(!fout)
    return 0 ;
  return write(fout) ;
}

/*!
  \brief Writes a NurbsCurve<T,N> to an output stream.

  \param fout  the output stream
  \return 0 if an error occurs, 1 otherwise

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
int NurbsCurve<T,N>::write(ofstream &fout) const {
  if(!fout)
    return 0 ;
  int pn = P.n() ;
  char st = '0' + sizeof(T) ;
  if(!fout.write((char*)&"nc4",sizeof(char)*3)) return 0 ;
  if(!fout.write((char*)&st,sizeof(char))) return 0 ; 
  if(!fout.write((char*)&pn,sizeof(int))) return 0 ;
  if(!fout.write((char*)&deg_,sizeof(int))) return 0 ;
  if(!fout.write((char*)U.memory(),sizeof(T)*U.n())) return 0 ;
  
  T *p,*p2 ;
  p = new T[P.n()*4] ;
  p2 = p ;
  for(int i=0;i<P.n();i++) {
    *p = P[i].x() ; p++ ;
    *p = P[i].y() ; p++ ;
    *p = P[i].z() ; p++ ;
    *p = P[i].w() ; p++ ;
  }
  if(!fout.write((char*)p2,sizeof(T)*P.n()*4)) return 0 ;
  delete []p2 ;
  return 1 ;
}

/*!
  \brief Computes the non-zero basis functions of the curve
  \relates NurbsCurve

  Computes the non-zero basis functions and puts the result 
  into N. N has a size of deg+1. To relate N to the basis 
  functions, N_{all}[span -deg +i] = N[i] for i=0... deg.

  \param u  the parametric value
  \param span  the non-zero span of the basis functions
  \param deg  the degree of the curve
  \param U  the knot vector on which the Basis functions are computed
  \param N  the non-zero basis functions

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T>
void nurbsBasisFuns(T u, int span, int deg, const Vector<T>& U, Vector<T>& N) {
  //Vector<T> left(deg+1), right(deg+1) ;
  T* left = (T*) alloca(2*(deg+1)*sizeof(T)) ;
  T* right = &left[deg+1] ;
  
  T temp,saved ;
   
  N.resize(deg+1) ;

  N[0] = 1.0 ;
  for(int j=1; j<= deg ; j++){
    left[j] = u-U[span+1-j] ;
    right[j] = U[span+j]-u ;
    saved = 0.0 ;
    for(int r=0 ; r<j; r++){
      temp = N[r]/(right[r+1]+left[j-r]) ;
      N[r] = saved+right[r+1] * temp ;
      saved = left[j-r] * temp ;
    }
    N[j] = saved ;
  }
  
}

/*!
  \brief Compute the derivatives functions at \a u of the NURBS curve
  \relates NurbsCurve

  For information on the algorithm, see A2.3 on p72 of 
  the NURBS book. The result is stored in the ders matrix, where 
  ders is of size (n+1,deg+1) and the derivative 
  C'(u) = ders(1,span-deg+j) where j=0...deg+1.

  \param n  the degree of the derivation
  \param u  the parametric value
  \param  span  the span for the basis functions
  \param deg  the degree of the curve
  \param U  the knot vector on which the Basis functions must be computed.
  \param ders  A matrix containing the derivatives of the curve.
  
  \warning \a n, \a u and \a span must be in a valid range.
  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T>
void nurbsDersBasisFuns(int n,T u, int span,  int deg, const Vector<T>& U, Matrix<T>& ders) {
  //Vector<T> left(deg+1),right(deg+1) ;
  T* left = (T*) alloca(2*(deg+1)*sizeof(T)) ;
  T* right = &left[deg+1] ;
  
  Matrix<T> ndu(deg+1,deg+1) ;
  T saved,temp ;
  int j,r ;

  ders.resize(n+1,deg+1) ;

  ndu(0,0) = 1.0 ;
  for(j=1; j<= deg ;j++){
    left[j] = u-U[span+1-j] ;
    right[j] = U[span+j]-u ;
    saved = 0.0 ;
    
    for(r=0;r<j ; r++){
      // Lower triangle
      ndu(j,r) = right[r+1]+left[j-r] ;
      temp = ndu(r,j-1)/ndu(j,r) ;
      // Upper triangle
      ndu(r,j) = saved+right[r+1] * temp ;
      saved = left[j-r] * temp ;
    }

    ndu(j,j) = saved ;
  }

  for(j=0;j<=deg;j++)
    ders(0,j) = ndu(j,deg) ;

  // Compute the derivatives
  Matrix<T> a(deg+1,deg+1) ;
  for(r=0;r<=deg;r++){
    int s1,s2 ;
    s1 = 0 ; s2 = 1 ; // alternate rows in array a
    a(0,0) = 1.0 ;
    // Compute the kth derivative
    for(int k=1;k<=n;k++){
      T d ;
      int rk,pk,j1,j2 ;
      d = 0.0 ;
      rk = r-k ; pk = deg-k ;

      if(r>=k){
	a(s2,0) = a(s1,0)/ndu(pk+1,rk) ;
	d = a(s2,0)*ndu(rk,pk) ;
      }

      if(rk>=-1){
	j1 = 1 ;
      }
      else{
	j1 = -rk ;
      }

      if(r-1 <= pk){
	j2 = k-1 ;
      }
      else{
	j2 = deg-r ;
      }

      for(j=j1;j<=j2;j++){
	a(s2,j) = (a(s1,j)-a(s1,j-1))/ndu(pk+1,rk+j) ;
	d += a(s2,j)*ndu(rk+j,pk) ;
      }
      
      if(r<=pk){
	a(s2,k) = -a(s1,k-1)/ndu(pk+1,r) ;
	d += a(s2,k)*ndu(r,pk) ;
      }
      ders(k,r) = d ;
      j = s1 ; s1 = s2 ; s2 = j ; // Switch rows
    }
  }

  // Multiply through by the correct factors
  r = deg ;
  for(int k=1;k<=n;k++){
    for(j=0;j<=deg;j++)
      ders(k,j) *= r ;
    r *= deg-k ;
  }

}


/*!
  \brief Decompose the curve into Bézier segments

  This function decomposes the curve into an array of 4D Bézier 
  segments.

  \param c  an array of Bézier segments

  \warning The end Bézier segments will not be valid if the NURBS curve 
           is not clamped.

  \author Philippe Lavoie
  \date 16 February 1997
*/
template <class T, int N>
void NurbsCurve<T,N>::decompose(NurbsCurveArray<T,N>& c) const {
  int i,m,a,b,nb,mult,j,r,save,s,k ;
  T numer,alpha ;
  T* alphas = (T*) alloca((deg_+1)*sizeof(T)) ; //Vector<T> alphas(deg+1) ;

  // all the curves will have the same vector
  Vector<T> nU ;
  nU.resize(2*(deg_+1)) ;
  for(i=0;i<nU.n()/2;i++)
    nU[i] = 0 ;
  for(i=nU.n()/2;i<nU.n();i++)
    nU[i] = 1 ;
  
  c.resize(P.n()-deg_) ;
  for(i=0;i<c.n();i++){
    c[i].resize(deg_+1,deg_) ;
    c[i].U = nU ;
  }
  
  m = P.n()+deg_ ;
  a = deg_ ;
  b = deg_+1 ;
  nb = 0 ;

  for(i=0;i<=deg_;i++)
    c[nb].P[i] = P[i] ;
  while(b<m){
    i = b ;
    while(b<m && U[b+1] <= U[b]) b++ ;
    mult = b-i+1 ;
    if(mult<deg_){
      numer = U[b]-U[a] ; // th enumerator of the alphas
      for(j=deg_;j>mult;j--) // compute and store the alphas
	alphas[j-mult-1] = numer/(U[a+j]-U[a]) ;
      r = deg_-mult ; // insert knot r times
      for(j=1;j<=r;j++){
	save=r-j;
	s=mult+j; // this many new points
	for(k=deg_;k>=s;k--){
	  alpha = alphas[k-s] ;
	  c[nb].P[k] = alpha*c[nb].P[k] + (1.0-alpha)*c[nb].P[k-1] ;	  
	}
	if(b<m) // control point of
	  c[nb+1].P[save] = c[nb].P[deg_]; // next segment
      }
    }
    nb++ ;
    if(b<m){ // initialize for next segment
      for(i=deg_-mult; i<= deg_ ; i++)
	c[nb].P[i] = P[b-deg_+i];
      a=b ;
      b++ ;
    }
  }
  c.resize(nb) ;

}

template <class T, int N>
inline Point_nD<T,N> project2D(const HPoint_nD<T,N>& p){
  Point_nD<T,N> pnt ;
  //if(p.z()==0.0){
    pnt.x() = p.x()/p.w() ;
    pnt.y() = p.y()/p.w() ;
    //}
    //else{
    //pnt.x() = (p.x()/p.w())/absolute(p.z()) ;
    //pnt.y() = (p.y()/p.w())/absolute(p.z()) ;
    //}
  return pnt ;
}

const float offX = 50 ;
const float offY = 70 ;

template <class T, int N>
inline void movePsP(Point_nD<T,N> &p, T magFact){
  p *= magFact ;
  p += Point_nD<T,N>(offX,offY,0) ;
  //p = p*magFact+Point_nD<T,N>(offX,offY,0)  ;
}

#ifdef HAVE_TEMPLATE_OF_TEMPLATE
template <class T>
inline void movePsP(Point_nD<T,2> &p, T magFact){
  p *= magFact ;
  p += Point_nD<T,2>(offX,offY) ;
  //p = p*magFact+Point_nD<T,N>(offX,offY,0)  ;
}
#else

template <>
inline void movePsP(Point_nD<float,2> &p, float magFact){
  p *= magFact ;
  p += Point_nD<float,2>(offX,offY) ;
  //p = p*magFact+Point_nD<T,N>(offX,offY,0)  ;
}

template <>
inline void movePsP(Point_nD<double,2> &p, double magFact){
  p *= magFact ;
  p += Point_nD<double,2>(offX,offY) ;
  //p = p*magFact+Point_nD<double,N>(offX,offY,0)  ;
}

#endif

/*!
  \brief Writes the curve in the postscript format to a file. 

  \param filename  the file to write the postscript file to
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
  \date 16 February 1997
*/
template <class T, int N>
int NurbsCurve<T,N>::writePS(const char* filename,int cp,T magFact, T dash, bool bOpen) const {

  ofstream fout(filename) ;  

  if(!fout)
    return 0 ;

  if(deg_<3){
    NurbsCurve<T,N> c3(*this) ;
    c3.degreeElevate(3-deg_) ;
    return c3.writePS(filename,cp,magFact,dash,bOpen) ;
  }

  NurbsCurveArray<T,N> Ca ;
  if (bOpen)
    decompose(Ca) ;
  else
    decomposeClosed(Ca) ;

  int guess =0 ;
  if(magFact<= T() ){
    magFact = T(1) ;
    guess = 1 ;
  }
  
  Matrix< Point_nD<T,N> > pnts(Ca.n(),deg_+1) ;
  int i,j ;

  //HPoint_nD<T,N> tp ;

  for(i=0;i<Ca.n();i++){
    for(j=0;j<deg_+1;j++){
      pnts(i,j) = project2D(Ca[i].P[j]) ;
    }
  }

  // find bounding box parameters
  T mx,my,Mx,My ;
  mx = Mx = pnts(0,0).x() ;
  my = My = pnts(0,0).y() ;

  for(i=0;i<Ca.n();i++){
    Point_nD<T,N> p ;
    int step ;
    step = 8 ;
    for(j=0;j<=step;j++){
      T u ;
      u = (T)j/(T)step ;
      p = project2D(Ca[i](u)) ;
      if(p.x() < mx)
	mx = p.x() ;
      if(p.x() > Mx)
	Mx = p.x() ;
      if(p.y() < my)
	my = p.y() ;
      if(p.y() > My) 
	My = p.y() ;      
    }
  }

  if(guess){
    //magFact = minimum((T)500/(T)(Mx-mx),(T)700/(T)(My-my)) ;
  }

  mx = mx*magFact+offX;
  my = my*magFact+offY;
  Mx = Mx*magFact+offX;
  My = My*magFact+offY;
  for(i=0;i<Ca.n();i++)
    for(j=0;j<deg_+1;j++)
	movePsP(pnts(i,j),magFact) ;

  fout << "%!PS-Adobe-2.1\n%%Title: " << filename << endl ;
  fout << "%%Creator: NurbsCurve<T,N>::writePS\n" ;
  fout << "%%BoundingBox: " << mx << ' ' << my << ' ' << Mx << ' ' << My << endl ;
  fout << "%%Pages: 0" << endl ;
  fout << "%%EndComments" << endl ;
  fout << "0 setlinewidth\n" ;
  fout << "0 setgray\n" ;
  fout << endl ;

  fout << "newpath\n" ;
  fout << pnts(0,0).x() << ' ' << pnts(0,0).y() << " moveto\n" ;
  for(i=0;i<Ca.n();i++){
    for(j=1;j<deg_+1;j++){
      fout << pnts(i,j).x() << ' ' << pnts(i,j).y() << ' ' ;
    }
    fout << "curveto\n" ;
  }
  fout << "stroke\n" ;

  if(cp>0){ // draw the control points of the original curve
    Vector< Point_nD<T,N> > pts(P.n()) ;
    for(i=0;i<P.n();i++){
      pts[i] = project2D(P[i]) ;
      movePsP(pts[i],magFact) ;
      fout << "newpath\n" ;
      fout << pts[i].x() << ' ' << pts[i].y() << "  3 0 360 arc\nfill\n" ;
    }
    if(dash>0)
      fout << "[" << dash << "] " << dash << " setdash\n" ;
    fout << "newpath\n" ;
    
    fout << pts[0].x() << ' ' << pts[0].y() << " moveto\n" ;
    for(i=1;i<P.n();i++)
      fout << pts[i].x() << ' ' << pts[i].y() << " lineto\n" ;
    fout << "stroke\n" ;
  }
  else{
    if(cp<0){
      Vector< Point_nD<T,N> > pts(P.n()*Ca.n()) ;
      int k=0 ;
      for(i=0;i<Ca.n();i++)
	for(j=0;j<deg_+1;j++){
	  pts[k] = project2D(Ca[i].P[j]) ;
	  movePsP(pts[k],magFact) ;
	  fout << "newpath\n" ;
	  fout << pts[k].x() << ' ' << pts[k].y() << "  3 0 360 arc\nfill\n" ;
	  k++ ;
	}
      if(dash>0)
	fout << "[" << dash << "] " << dash << " setdash\n" ;
      fout << "newpath\n" ;
      
      fout << pts[0].x() << ' ' << pts[0].y() << " moveto\n" ;
      for(i=1;i<k;i++)
	fout << pts[i].x() << ' ' << pts[i].y() << " lineto\n" ;
      fout << "stroke\n" ;      
    }
  }

  fout << "showpage\n%%EOF\n" ;
  return 1 ;
}

/*!
  \brief Writes a post-script file representing the curve

   Writes the curve in the postscript format to a file, it also
   draws the points defined in $points$ with their associated
   vectors if $vector$ is used.

  \param filename  the file to write the postscript file to
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
  \date 16 February 1997
*/
template <class T, int N>
int NurbsCurve<T,N>::writePSp(const char* filename,const Vector< Point_nD<T,N> >& points, const Vector< Point_nD<T,N> >& vectors, int cp, T magFact, T dash, bool bOpen) const {
  ofstream fout(filename) ;  

  if(!fout)
    return 0 ;

  if(deg_<3){
    NurbsCurve<T,N> c3(*this) ;
    c3.degreeElevate(3-deg_) ;
    return c3.writePSp(filename,points,vectors,cp,magFact,dash,bOpen) ;
  }

  // extract the Bezier segments 
  NurbsCurveArray<T,N> Ca ;
  if (bOpen)
    decompose(Ca) ;
  else
    decomposeClosed(Ca) ;

  int guess =0 ;
  if(magFact<=0){
    magFact = 1.0 ;
    guess = 1 ;
  }
  
  Matrix< Point_nD<T,N> > pnts(Ca.n(),deg_+1) ;
  int i,j ;
  for(i=0;i<Ca.n();i++){
    for(j=0;j<deg_+1;j++){
      pnts(i,j) = project2D(Ca[i].P[j]) ;
    }
  }


  // find bounding box parameters
  T mx,my,Mx,My ;
  mx = Mx = pnts(0,0).x() ;
  my = My = pnts(0,0).y() ;

  for(i=0;i<Ca.n();i++){
    Point_nD<T,N> p ;
    int step ;
    step = 8 ;
    for(j=0;j<=step;j++){
      T u ;
      u = (T)j/(T)step ;
      p = project2D(Ca[i](u)) ;
      if(p.x() < mx)
	mx = p.x() ;
      if(p.x() > Mx)
	Mx = p.x() ;
      if(p.y() < my)
	my = p.y() ;
      if(p.y() > My) 
	My = p.y() ;      
    }
  }

  if(guess){
    magFact = minimum((T)500/(T)(Mx-mx),(T)700/(T)(My-my)) ;
  }

  mx = mx*magFact+offX;
  my = my*magFact+offY;
  Mx = Mx*magFact+offX;
  My = My*magFact+offY;
  for(i=0;i<Ca.n();i++)
    for(j=0;j<deg_+1;j++)
	movePsP(pnts(i,j),magFact) ;

  fout << "%!PS-Adobe-2.1\n%%Title: " << filename << endl ;
  fout << "%%Creator: NurbsCurve<T,N>::writePS\n" ;
  fout << "%%BoundingBox: " << mx << ' ' << my << ' ' << Mx << ' ' << My << endl ;
  fout << "%%Pages: 0" << endl ;
  fout << "%%EndComments" << endl ;
  fout << "0 setlinewidth\n" ;
  fout << "0 setgray\n" ;
  fout << endl ;

  fout << "newpath\n" ;
  fout << pnts(0,0).x() << ' ' << pnts(0,0).y() << " moveto\n" ;
  for(i=0;i<Ca.n();i++){
    for(j=1;j<deg_+1;j++){
      fout << pnts(i,j).x() << ' ' << pnts(i,j).y() << ' ' ;
    }
    fout << "curveto\n" ;
  }
  fout << "stroke\n" ;

  if(cp>0){ // draw the control points of the original curve
    Vector< Point_nD<T,N> > pts(P.n()) ;
    for(i=0;i<P.n();i++){
      pts[i] = project2D(P[i]) ;
      movePsP(pts[i],magFact) ;
      fout << "newpath\n" ;
      fout << pts[i].x() << ' ' << pts[i].y() << "  3 0 360 arc\nfill\n" ;
    }
    if(dash>0)
      fout << "[" << dash << "] " << dash << " setdash\n" ;
    fout << "newpath\n" ;
    
    fout << pts[0].x() << ' ' << pts[0].y() << " moveto\n" ;
    for(i=1;i<P.n();i++)
      fout << pts[i].x() << ' ' << pts[i].y() << " lineto\n" ;
    fout << "stroke\n" ;
  }
  else{
    if(cp<0){
      Vector< Point_nD<T,N> > pts(P.n()*Ca.n()) ;
      int k=0 ;
      for(i=0;i<Ca.n();i++)
	for(j=0;j<deg_+1;j++){
	  pts[k] = project2D(Ca[i].P[j]) ;
	  movePsP(pts[k],magFact) ;
	  fout << "newpath\n" ;
	  fout << pts[k].x() << ' ' << pts[k].y() << "  3 0 360 arc\nfill\n" ;
	  k++ ;
	}
      if(dash>0)
	fout << "[" << dash << "] " << dash << " setdash\n" ;
      fout << "newpath\n" ;
      
      fout << pts[0].x() << ' ' << pts[0].y() << " moveto\n" ;
      for(i=1;i<k;i++)
	fout << pts[i].x() << ' ' << pts[i].y() << " lineto\n" ;
      fout << "stroke\n" ;      
    }
  }

  for(i=0;i<points.n();++i){
    Point_nD<T,N> p ;
    p = points[i] ;
    movePsP(p,magFact) ;
    fout << "newpath\n" ;
    fout << p.x() << ' ' << p.y() << "  3 0 360 arc\nfill\n" ;
  }

  if(vectors.n()==points.n()){
    for(i=0;i<points.n();++i){
      Point_nD<T,N> p,p2 ;
      p = points[i] ;
      p2 = points[i] + vectors[i] ;
      movePsP(p,magFact) ;
      movePsP(p2,magFact) ;
      fout << "newpath\n" ;
      fout << p.x() << ' ' << p.y() << " moveto\n" ;
      if(dash>0)
	fout << "[" << dash/2.0 << "] " << dash/2.0 << " setdash\n" ;
      fout << p2.x() << ' ' << p2.y() << " lineto\n" ;
      fout << "stroke\n" ;
    }
  }

  fout << "showpage\n%%EOF\n" ;
  return 1 ;
}

/*!
  \brief Writes the curve to a VRML file

  A circle is swept around the trajectory made by the curve.
  The resulting surface is saved as a VRML file.

  \param filename  the name of the VRML file to save to
  \param radius  the radius of the line
  \param K  the minimum number of interpolation
  \param color  the color of the line
  \param Nu  the number of points for the circle
  \param Nv  the number of points along the path
  \param u_s  the starting parametric value for \a u
  \param u_e  the end parametric value for \a u

  \return returns 1 on success, 0 otherwise.

  \author Philippe Lavoie
  \date 25 July 1997
*/
template <class T, int N>
int NurbsCurve<T,N>::writeVRML(const char* filename,T radius,int K, const Color& color,int Nu,int Nv, T u_s, T u_e) const{
  NurbsSurface<T,N> S ;
  NurbsCurve<T,N> C ;
  
  C.makeCircle(Point_nD<T,N>(0,0,0),Point_nD<T,N>(1,0,0),Point_nD<T,N>(0,0,1),radius,0,2*M_PI);
  S.sweep(*this,C,K) ;
  return S.writeVRML(filename,color,Nu,Nv,0,1,u_s,u_e) ;
}

/*!
  \brief Writes the curve to a VRML file

  A circle is swept around the trajectory made by the curve.
  The resulting surface is saved as a VRML file.

  \param filename  the name of the VRML file to save to
  \param radius  the radius of the line
  \param K  the minimum number of interpolation
  \param color  the color of the line
  \param Nu  the number of points for the circle
  \param Nv  the number of points along the path
  \param u_s  the starting parametric value for \a u
  \param u_e  the end parametric value for \a u

  \return returns 1 on success, 0 otherwise.

  \author Philippe Lavoie
  \date 4 May 1999
*/
template <class T, int N>
int NurbsCurve<T,N>::writeVRML97(const char* filename,T radius,int K, const Color& color,int Nu,int Nv, T u_s, T u_e) const{
  NurbsSurface<T,N> S ;
  NurbsCurve<T,N> C ;
  
  C.makeCircle(Point_nD<T,N>(0,0,0),Point_nD<T,N>(1,0,0),Point_nD<T,N>(0,0,1),radius,0,2*M_PI);
  S.sweep(*this,C,K) ;
  return S.writeVRML97(filename,color,Nu,Nv,0,1,u_s,u_e) ;
}

/*!
  \brief Writes the curve to a VRML file

  A circle is swept around the trajectory made by the curve.
  The resulting surface is saved as a VRML file.

  \param filename  the name of the ostream to write to
  \param radius  the radius of the line
  \param K  the minimum number of interpolation
  \param color  the color of the line
  \param Nu  the number of points for the circle
  \param Nv  the number of points along the path
  \param u_s  the starting parametric value for \a u
  \param u_e  the end parametric value for \a u

  \return returns 1 on success, 0 otherwise.

  \author Philippe Lavoie
  \date 25 July 1997
*/
template <class T, int N>
int NurbsCurve<T,N>::writeVRML(ostream &fout,T radius,int K, const Color& color,int Nu,int Nv, T u_s, T u_e) const{
  NurbsSurface<T,N> S ;
  NurbsCurve<T,N> C ;
  
  C.makeCircle(Point_nD<T,N>(0,0,0),Point_nD<T,N>(1,0,0),Point_nD<T,N>(0,0,1),radius,0,2*M_PI);
  S.sweep(*this,C,K) ;
  return S.writeVRML(fout,color,Nu,Nv,0,1,u_s,u_e) ;
}

/*!
  \brief Writes the curve to a VRML file

  A circle is swept around the trajectory made by the curve.
  The resulting surface is saved as a VRML file.

  \param filename  the name of the ostream to write to
  \param radius  the radius of the line
  \param K  the minimum number of interpolation
  \param color  the color of the line
  \param Nu  the number of points for the circle
  \param Nv  the number of points along the path
  \param u_s  the starting parametric value for \a u
  \param u_e  the end parametric value for \a u

  \return returns 1 on success, 0 otherwise.

  \author Philippe Lavoie
  \date 4 May 1999
*/
template <class T, int N>
int NurbsCurve<T,N>::writeVRML97(ostream &fout,T radius,int K, const Color& color,int Nu,int Nv, T u_s, T u_e) const{
  NurbsSurface<T,N> S ;
  NurbsCurve<T,N> C ;
  
  C.makeCircle(Point_nD<T,N>(0,0,0),Point_nD<T,N>(1,0,0),Point_nD<T,N>(0,0,1),radius,0,2*M_PI);
  S.sweep(*this,C,K) ;
  return S.writeVRML97(fout,color,Nu,Nv,0,1,u_s,u_e) ;
}

/*! 
  \brief Transforms a 2D curve into a 3D curve
  \relates NurbsCurve
   
  \param c2d  the curve in 2D
  \param c3d  the curve in 3D

  \warning The curve must be valid

  \author Philippe Lavoie 
  \date 16 October 1998
*/
template <class T>
void to3D(const NurbsCurve<T,2>& c2d, NurbsCurve<T,3>& c3d){
  c3d.resize(c2d.ctrlPnts().n(),c2d.degree()) ; 

  c3d.modKnot(c2d.knot()) ; 
  HPoint_nD<T,3> p(0) ; 
  for(int i=c2d.ctrlPnts().n()-1;i>=0;--i){
    p.x() = c2d.ctrlPnts()[i].x() ; 
    p.y() = c2d.ctrlPnts()[i].y() ; 
    p.w() = c2d.ctrlPnts()[i].w() ; 
    c3d.modCP(i,p) ; 
  }
}

template <class T>
void to3D(const NurbsCurve<T,3>& c2d, NurbsCurve<T,3>& c3d){
  c3d = c2d ; 
}

/*! 
  \brief Transforms a 3D curve into a 2D curve
  \relates NurbsCurve

  Actually it just puts the x,y and w value into a 2D curve. It doesn't handle
  the z value, if you want some perspective transformation,
  you should perform the needed transformation before hand
  on the 3D curve.

  \param c3d  the curve in 3D
  \param c2d  the curve in 2D

  \warning The curve must be valid

  \author Philippe Lavoie 
  \date 20 October 1998
*/
template <class T>
void to2D(const NurbsCurve<T,3>& c3d, NurbsCurve<T,2>& c2d){
  c2d.resize(c3d.ctrlPnts().n(),c3d.degree()) ; 

  c2d.modKnot(c3d.knot()) ; 
  HPoint_nD<T,2> p(0) ; 
  for(int i=c3d.ctrlPnts().n()-1;i>=0;--i){
    p.x() = c3d.ctrlPnts()[i].x() ; 
    p.y() = c3d.ctrlPnts()[i].y() ; 
    p.w() = c3d.ctrlPnts()[i].w() ; 
    c2d.modCP(i,p) ; 
  }
}

/*! 
  \brief Generates a knot vector using the averaging technique
  \relates NurbsCurve

  \latexonly
    The technique is as follows:
    \begin{itemize}
    \item $u_0 = \cdots = u_{deg} = 0$
    \item $u_{m-deg} = \cdots = u_{m-1} = 1$
    \item \begin{equation}
    u_{j+deg} = \frac{1}{deg}\sum_{i=j}^{j+deg+1}\bar{u}_i
    \hspace{0.5in} j= 1,\ldots,n-deg-1
    \end{equation}
    \end{itemize}
    where $n$ is the size of the $\bar{u}$ knot coefficient vector,
    $m=n+deg+1$ is the size of the knot vector and $deg$ is the 
    degree of the curve.
  \endlatexonly
  \htmlonly
   There is more information about this routine in the LaTeX version.
  \endhtmlonly

  \param uk  the knot coefficients
  \param deg  the degree of the curve associated with the knot vector
  \param U  an average knot vector

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T>
void knotAveraging(const Vector<T>& uk, int deg, Vector<T>& U){
  U.resize(uk.n()+deg+1) ;

  int j ;
  for(j=1;j<uk.n()-deg;++j){
    U[j+deg] = 0.0 ;
    for(int i=j;i<j+deg;++i)
      U[j+deg] += uk[i] ;
    U[j+deg] /= (T)deg ;
  }
  for(j=0;j<=deg;++j)
    U[j] = 0.0 ;
  for(j=U.n()-deg-1;j<U.n();++j)
    U[j] = 1.0 ;
}



/*!
  \brief Compute the parameters by averaging the knots
  \relates NurbsCurve

   Generates a parameter vector by averaging the knots from the 
   knot vector. 

  \param   U  the knot vector
  \param deg  the degree of the curve associated with the knot vector
  \param uk  the parameter vector

  \author Philippe Lavoie
  \date 26 August 1997
*/
template <class T>
void averagingKnots(const Vector<T>& U, int deg, Vector<T>& uk){
  uk.resize(U.n()-deg-1) ;

  int i,k ;

  uk[0] = U[0] ;
  uk[uk.n()-1] = U[U.n()-1] ;

  for(k=1;k<uk.n()-1;++k){
    uk[k] = 0.0 ;
    for(i=k+1;i<k+deg+1;++i)
      uk[k] += U[i] ;
    uk[k] /= deg ;
  }
}

/*!
  \brief Moves a point on the NURBS curve

  This modifies the curve such that the point $C(u)$ is moved
  by delta. 

  See section 11.5.1 of the NURBS book for an explanation of 
  the algorithm.

  \param   u  the point to move
  \param delta  the movement in 3D of the point at $C(u)$

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
int NurbsCurve<T,N>::movePoint(T u, const Point_nD<T,N>& delta) {
  BasicArray< Point_nD<T,N> > d(1) ;
  d[0] = delta ;
  return movePoint(u,d) ;
}

/*!
  \brief Moves a point in the NURBS curve

  This modifies the curve such that the point \a C(u) is moved
  by delta. Delta is a vector containing the movement as
  D^{(k)} where (k) specifies the derivative. Thus
  at D[0], this specifies the 0th derivative movement, at 
  D[1] it specifies the 1st derivative movement of the point.
  \e i.e. Suppose that C(u) = (10,20,3) then a
  D[0] = (10,10,10) will move the point to C(u) = (20,30,13) 
  
  See section 11.5.1 of the NURBS book for an explanation of 
  the algorithm.
  
  \param  u  the point to move
  \param  delta  the vector of movement

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T, int N>
int NurbsCurve<T,N>::movePoint(T u, const BasicArray< Point_nD<T,N> >& delta) {
  int i,j ;

  // setup B
  Matrix_DOUBLE B ;
  
  int n,m ; // n is the number of rows, m the number of columns

  m = deg_ + 1 ;
  n = delta.n() ;

  B.resize(n,m) ;
  
  int span = findSpan(u) ;

  n = 0 ;
  
  Matrix<T> R ;

  dersBasisFuns(delta.n()-1,u,span,R) ;


  for(i=0;i<delta.n();++i){
    if(delta[i].x()==0.0 && delta[i].y()==0.0 && delta[i].z()==0.0)
      continue ;
    for(j=0;j<m;++j){
      B(n,j) = (double)R(i,j) ;
    }
    ++n ;
  }

  Matrix_DOUBLE A  ;
  Matrix_DOUBLE Bt(transpose(B)) ;
  Matrix_DOUBLE BBt ;

  BBt = inverse(B*Bt) ;
  A = Bt*BBt ;

  Matrix_DOUBLE dD ;

  dD.resize(delta.n(),N) ;
  for(i=0;i<delta.n();++i){
    const Point_nD<T,N>& d = delta[i] ; // this makes the SGI compiler happy
    for(j=0;j<N;++j)
      dD(i,j) = (double)d.data[j] ;
  }

  Matrix_DOUBLE dP ;

  dP = A*dD ;

  for(i=0;i<m;++i){
    P[span-deg_+i].x() += dP(i,0)*P[span-deg_+i].w() ;
    P[span-deg_+i].y() += dP(i,1)*P[span-deg_+i].w() ;
    P[span-deg_+i].z() += dP(i,2)*P[span-deg_+i].w() ;
  }

  return 1 ;
}

/*!
  \brief Moves a point with some constraint

  This will modify the NURBS curve by respecting a certain 
  number of constraints. $u_r$ specifies the parameters on which
  the constraints should be applied. 
  The constraint are defined by $D$ which specifies the
  vector by which the points should move.
  
  For example, if you want to move the point C(0.5) by 
  (10,0,10) and fix the point C(0.6) on the current curve
  (a move of (0,0,0)). 
  u_r = 0.5, 0.6 and D = (10,0,10), (0,0,0)
  
  The \a u_r vector should be in an increasing order.
  
  See section 11.5.1 of the NURBS book for an explanation of 
  the algorithm.

  \param ur  the vector of parameters on which a constraint is applied
  \param D  a vector of the value of \a D_r^{(0)}

  \return 1 if the operation is possible, 0 if the problem is ill defined
               \e i.e. there isn't enough information to find a unique 
	       solution (the system is overdetermined) or that the system
	       has non-independant components.

  \warning ur and D must have the same size and the values inside ur should 
          not repeat and they should be ordered
  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T, int N>
int NurbsCurve<T,N>::movePoint(const BasicArray<T>& ur, const BasicArray< Point_nD<T,N> >& D) {
  BasicArray<int> fixCP(0) ;
  BasicArray<int> Dr(D.n()) ;
  BasicArray<int> Dk(D.n()) ;

  if(ur.n() != D.n()){
#ifdef USE_EXCEPTION
    throw NurbsInputError(ur.n(),D.n()) ;
#else
    Error err("movePoint(ur,D)");
    err << "The two input vectors are not of the same size\n" ;
    err << "ur.n()= " << ur.n() << ", D.n() = " << D.n() << endl ;
    err.fatal() ;
#endif
  }
  
  for(int i=0;i<Dr.n();++i){
    Dr[i] = i ;
  }
  Dk.reset(0) ;
  return movePoint(ur,D,Dr,Dk,fixCP) ;
}

/*!
  \brief Moves a point with some constraint

   This will modify the NURBS curve by respecting a certain 
   number of constraints. u_r specifies the parameters on which
   the constraints should be applied. 
   The constraint are defined by $D_r^{(k)}$ which requires 3
   vectors to fully qualify. \a D specifies the value
   of the constraint and D_r and D_k are used to specify
   on which parameter the constraint is applied and of what degree.
   
   For example, if you want to move the point C(0.5) by 
   (10,0,10) and fix the point C(0.6) on the current curve
   (a move of (0,0,0)) but change its 1st derivative by (0,20,0). 
   Then the following values must be inputed to the routine.
   u_r = [0.5,0.6], D = [(10,0,10), (0,0,0), (0,20,0)], 
   D_r = [0, 1, 1] and D_k = [0, 0, 1]. 
   
   The values in D should be ordered in respect with r and k.
   {\em i.e.} for D_i=D_{r_i}^{(k_i)}, then i < j implies 
   that r_i < r_j and that either r_i < r_j or k_i < k_j.
   
   See section 11.5.1 of the NURBS book for an explanation of 
   the algorithm.
   
   \param ur  the vector of parameters on which a constraint is applied
   \param D  a vector of the value of D_r^{(k)}
   \param Dr  a vector specifying the value of r for D
   \param Dk  a vector specifying the value of k for D

   \return 1 if the operation is possible, 0 if the problem is ill defined
               \e i.e. there isn't enough information to find a unique 
	       solution (the system is overdetermined) or that the system
	       has non-independant components.

  \warning The values inside ur should \e not repeat. D,Dr and Dk must
               be of the same size.
  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T, int N>
int NurbsCurve<T,N>::movePoint(const BasicArray<T>& ur, const BasicArray< Point_nD<T,N> >& D, const BasicArray_INT& Dr, const BasicArray_INT& Dk) {
  BasicArray_INT fixCP(0) ;

  if(D.n() != Dr.n() ){
#ifdef USE_EXCEPTION
    throw NurbsInputError(D.n(),Dr.n()) ;
#else
    Error err("movePoint(ur,D,Dr,Dk)");
    err << "The D,Dr,Dk vectors are not of the same size\n" ;
    err << "D.n()= " << D.n() << ", Dr.n() = " << Dr.n() 
	<< ", Dk.n() = " << Dk.n() << endl ;
    err.fatal() ;
#endif
  }

  if( D.n() !=Dk.n()){
#ifdef USE_EXCEPTION
    throw NurbsInputError(D.n(),Dk.n()) ;
#else
    Error err("movePoint(ur,D,Dr,Dk)");
    err << "The D,Dr,Dk vectors are not of the same size\n" ;
    err << "D.n()= " << D.n() << ", Dr.n() = " << Dr.n() 
	<< ", Dk.n() = " << Dk.n() << endl ;
    err.fatal() ;
#endif
  }  

  return movePoint(ur,D,Dr,Dk,fixCP) ;
}

/*!
  \brief Moves a point with some constraint

   This will modify the NURBS curve by respecting a certain 
   number of constraints. \a u_r specifies the parameters on which
   the constraints should be applied. 
   The constraint are defined by D_r^{(k)} which requires 3
   vectors to fully qualify. \a D specifies the value
   of the constraint and D_r and D_k are used to specify
   on which parameter the constraint is applied and of what degree.
   
   A second constraint \a fixCP consists of specifying which 
   control points can not be moved by the routine.
   
   For example, if you want to move the point C(0.5) by 
   (10,0,10) and fix the point C(0.6) on the current curve
   (a move of (0,0,0)) but change its 1st derivative by (0,20,0). 
   Doing this without modifying control point 4 . 
   Then the following values must be inputed to the routine.
   u_r = [0.5, 0.6], D = [(10,0,10), (0,0,0), (0,20,0)], 
   D_r = [0, 1, 1], D_k = [0, 0, 1] and fixCP= 4. 
   
   The values in D should be ordered in respect with r and k.
   \e i.e. for D_i=D_{r_i}^{(k_i)}, then i < j implies 
   that r_i < r_j and that either r_i < r_j or k_i < k_j.
   
   See section 11.5.1 of the NURBS book for an explanation of 
   the algorithm.

   \param ur  the vector of parameters on which a constraint is applied
   \param D  a vector of the value of D_r^{(k)}
   \param Dr  a vector specifying the value of r for D
   \param Dk  a vector specifying the value of k for D
   \param fixCP  a vector specifying which control points {\em can not} be
                 modified.
   \return 1 if the operation is possible, 0 if the problem is ill defined
               \e i.e. there isn't enough information to find a unique 
	       solution (the system is overdetermined) or that the system
	       has non-independant components.

  \warning The values of ur should \e not repeat. D,Dr and Dk must
               be of the same size.
  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T, int N>
int NurbsCurve<T,N>::movePoint(const BasicArray<T>& ur, const BasicArray< Point_nD<T,N> >& D, const BasicArray_INT& Dr, const BasicArray_INT& Dk, const BasicArray_INT& fixCP) {
  int i,j,n ;

  if(D.n() != Dr.n() ){
#ifdef USE_EXCEPTION
    throw NurbsInputError(D.n(),Dr.n()) ;
#else
    Error err("movePoint(ur,D,Dr,Dk)");
    err << "The D,Dr,Dk vectors are not of the same size\n" ;
    err << "D.n()= " << D.n() << ", Dr.n() = " << Dr.n() 
	<< ", Dk.n() = " << Dk.n() << endl ;
    err.fatal() ;
#endif
  }

  if( D.n() !=Dk.n()){
#ifdef USE_EXCEPTION
    throw NurbsInputError(D.n(),Dk.n()) ;
#else
    Error err("movePoint(ur,D,Dr,Dk)");
    err << "The D,Dr,Dk vectors are not of the same size\n" ;
    err << "D.n()= " << D.n() << ", Dr.n() = " << Dr.n() 
	<< ", Dk.n() = " << Dk.n() << endl ;
    err.fatal() ;
#endif
  }  

  // setup B
  Matrix_DOUBLE B ;
  
  B.resize(D.n(),P.n()) ;
  
  int span ;

  Matrix<T> R ;

  B.reset(0.0) ;

  for(i=0;i<D.n();++i){
    span = findSpan(ur[Dr[i]]) ;
    dersBasisFuns(Dk[i],ur[Dr[i]],span,R) ;
    for(j=0;j<=deg_;++j){
      B(i,span-deg_+j) = (double)R(Dk[i],j) ;
    }
  }

  // optimize B
  BasicArray_INT remove(B.cols()) ;
  BasicArray_INT map(B.cols()) ;
  remove.reset((int)1.0) ;

  for(j=0;j<B.cols();++j){
    for(i=0;i<B.rows();++i)
      if((B(i,j)*B(i,j))>1e-10){
	remove[j] = 0 ;
	break ;
      }
  }

  for(i=0;i<fixCP.n();++i){
    remove[fixCP[i]] = 1 ;
  }

  n = 0 ;
  for(i=0;i<B.cols();++i){
    if(!remove[i]){
      map[n] = i ;
      ++n ;
    }
  }

  map.resize(n) ;

  Matrix_DOUBLE Bopt(B.rows(),n) ;
  for(j=0;j<n;++j){
    for(i=0;i<B.rows();++i)
      Bopt(i,j) = B(i,map[j]) ;
  }

  Matrix_DOUBLE A  ;
  Matrix_DOUBLE Bt(transpose(Bopt)) ;
  Matrix_DOUBLE BBt ;

  BBt = inverse(Bopt*Bt) ;

  A = Bt*BBt ;

  Matrix_DOUBLE dD ;

  dD.resize(D.n(),N) ;
  for(i=0;i<D.n();++i){
    const Point_nD<T,N>& d = D[i] ; // this makes the SGI compiler happy
    for(j=0;j<N;++j)
      dD(i,j) = (double)d.data[j] ;
  }

  Matrix_DOUBLE dP ;

  dP = A*dD ;

  for(i=0;i<map.n();++i){
    P[map[i]].x() += dP(i,0)*P[map[i]].w() ;
    P[map[i]].y() += dP(i,1)*P[map[i]].w() ;
    P[map[i]].z() += dP(i,2)*P[map[i]].w() ;
  }

  return 1 ;
}

/*!
  \brief Splits the curve into two curves. 

  \param  u  splits at this parametric value
  \param  cl  the lower curve
  \param  cu  the upper curve

  \return 1 if the operation, 0 otherwise.

  \warning You \e must make sure that you split at a valid parametric 
               value. You can't split a curve at its end points.
  \author Philippe Lavoie
  \date 16 October 1997
*/
template <class T, int N>
int NurbsCurve<T,N>::splitAt(T u, NurbsCurve<T,N>& cl, NurbsCurve<T,N>& cu) const {
  if(u<= U[deg_])
    return 0 ;
  if(u>= U[U.n()-deg_-1])
    return 0 ;

  // get the multiplicity at u
  int s,j,i ;
  int span = findSpan(u) ;

  if(absolute(u-U[span])<1e-6)
    s = findMult(span) ;
  else
    s = 0 ;

  BasicArray<T> X(deg_+1-s) ;
  X.reset(u) ;

  cl = *this ;
  if(X.n()>0)
    cl.refineKnotVector(X) ;

  span = cl.findSpan(u)-deg_ ;
  // span is the begining of the upper curve
  cu.resize(cl.P.n()-span,deg_) ;
  for(i=cl.P.n()-1,j=cu.P.n()-1;j>=0;--j,--i){
    cu.P[j] = cl.P[i] ;
  }

  for(i=cl.U.n()-1,j=cu.U.n()-1;j>=0;--j,--i){
    cu.U[j] = cl.U[i] ;
  }
  cl.resize(span,deg_) ;

  return 1 ;
}

/*!
  \brief The curve is the result of mergin two curves

  \param cl  the lower curve
  \param cu  the upper curve

  \return 1 if the operation is succesfull, 0 otherwise.

  \warning You must make sure the the knot vectors are compatible, 
               \e i.e. that the end knots of the lower curve are the
	       same as the first knots of the upper curve. The curves
	       must also be of the same degree.
  \author Philippe Lavoie
  \date 16 October 1997
*/
template <class T, int N>
int NurbsCurve<T,N>::mergeOf(const NurbsCurve<T,N>& cl, const NurbsCurve<T,N> &cu){
  if(cl.deg_ != cu.deg_){
#ifdef USE_EXCEPTION
    throw NurbsInputError();
#else
    Error err("NurbsCurve<T,N>::mergeOf");
    err << " The two curves are not of the same degree\n" ;
    err.warning() ;
    return 0 ;
#endif
  }
  if((cl.U[cl.U.n()-1]-cu.U[0])*(cl.U[cl.U.n()-1]-cu.U[0])>1e-8){
#ifdef USE_EXCEPTION
    throw NurbsInputError();
#else
    Error err("NurbsCurve<T,N>::mergeOf");
    err << " The two knot vectors are not compatible.\n" ;
    err << "The first one is " << cl.U << endl ;
    err << "The second is    " << cu.U << endl ;
    err.warning() ;
    return 0 ;
#endif
  }
  if(norm2(cl.P[cl.P.n()-1]-cu.P[0])>1e-8){
#ifdef USE_EXCEPTION
    throw NurbsInputError();
#else
    Error err("NurbsCurve<T,N>::mergeOf");
    err << " The end control points are NOT the same.\n" ;
    err << " cl.P[n-1] = " << cl.P[cl.P.n()-1] << endl ;
    err << " cu.P[0] = " << cu.P[0] << endl ;
    err.warning() ;
    return 0 ;
#endif
  }
  resize(cl.P.n()+cu.P.n(),cl.deg_) ;

  int i ;
  for(i=0;i<cl.P.n();++i)
    P[i] = cl.P[i] ;
  for(;i<P.n();++i)
    P[i] = cu.P[i-cl.P.n()] ;

  for(i=0;i<cl.U.n();++i)
    U[i] = cl.U[i] ;
  for(;i<U.n();++i)
    U[i] = cu.U[i-cl.U.n()+deg_+1] ;

  return 1 ;
}

/*!
  \brief Determines the knot span index
  \relates NurbsCurve NurbsSurface

   Determines the knot span for which their exists non-zero basis 
   functions. The span is the index \a k for which the parameter 
   \a u is valid in the [u_k,u_{k+1}] range.

  \param   u  the parametric value
  \param   U  the knot vector
  \param deg  the degree of the curve

  \return the span index at \a u.

  \warning \a u must be in a valid range

  \author Philippe Lavoie
  \date 17 October 1997
  \modified 20 January, 1999 (Alejandro Frangi)
*/
template <class T>
int findSpan(T u, const Vector<T>& U, int deg) {
  if(u>=U[U.n()-deg-1]) 
    return U.n()-deg-1 ;
  if(u<=U[deg])
    return deg ;
  //AF
  int low = 0 ;
  int high = U.n()-deg ;
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
  \brief Generates a list of points from the curve

  Generates a list of points from the curve. The list is
  generated within a user specified tolerance.

  \param tolerance --> the tolerance for the tesselation.

  \return The list of points.

  \author Philippe Lavoie
  \date 17 October 1997
*/
template <class T, int N>
BasicList<Point_nD<T,N> > NurbsCurve<T,N>::tesselate(T tolerance,BasicList<T> *uk) const {
  BasicList<Point_nD<T,N> > list,list2 ;

  NurbsCurveArray<T,N> ca ;
  decompose(ca) ;

  if(ca.n()==1){
    // get the number of steps 
    T u = 0 ;
    Point_nD<T,N> maxD(0) ;
    Point_nD<T,N> prev ;

    Vector< Point_nD<T,N> > ders(2) ;

    deriveAt(u,1,ders) ;
    
    prev = ders[1] ;

    int i ;
    for(i=1;i<11;++i){
      u = T(i)/T(10) ;
      deriveAt(u,1,ders) ;
      Point_nD<T,N> delta = ders[1]-prev ;
      delta.x() = absolute(delta.x()) ;
      delta.y() = absolute(delta.y()) ;
      delta.z() = absolute(delta.z()) ;
      maxD = maximumRef(maxD,delta) ;
      prev = ders[1] ;
    }
    
    const T sqr2 = T(1.414241527) ;

    int n = (int)rint(sqr2*norm(maxD)/tolerance) ;

    n+=2 ;
    if(n<3) n = 3 ;

    for(i=0;i<n;++i){
      u = (U[U.n()-deg_-1]-U[deg_])*T(i)/T(n-1) + U[deg_] ;
      list.add(pointAt(u)) ;
      if(uk)
	uk->add(u) ;
    }

    return list ;
  }
  else{
    for(int i=0;i<ca.n();++i){
      list2 = ca[i].tesselate(tolerance,uk) ;

      // remove the last point from the list to elliminate
      list.erase((BasicNode<Point_nD<T,N> >*)list.last()) ;
      list.addElements(list2) ;      
    }
  }
  return list ;
}

/*!
  \brief Finds the parametric value of maximal influence
  \relates NurbsCurve NurbsSurface

  Finds the parametric value of maximal influence for a control
  point \a i. This finds the parametric value \a u were the basis 
  function \a N_{i,p}(u) is maximal. 
  
  This routine only works for N_{i,p}(u) where p=1,2,3. Other
  values of \a p are not supported. The reason is that the routine
  uses pre-computed equations to find the proper values.

  \param i  the i-th control point
  \param U  the knot vector
  \param p  the degree of the basis function
  \param u  the parametric value of maixmal influence
  
  \return 1 if the operation was succesfull, 0 otherwise
  \warning The knot vector must be properly constructed, 
           \latexonly i.e.
               \[
	       U=\{\underbrace{a,\ldots,a}_{p+1},u_{p+1},\ldots,u_{m-p-1},\underbrace{b,\ldots,b}_{p+1} \} 
	       \]
           \endlatexonly

  \author Philippe Lavoie
  \date 17 October 1997
*/
template <class T>
int maxInfluence(int i, const Vector<T>& U, int p, T &u){
  if(i>U.n()-p-2)
    return 0 ;
  switch(p){
  case 1: u = U[i+1] ; return 1 ; 
  case 2: { 
    T A = U[i] + U[i+1] - U[i+2] - U[i+3] ;
    if(A>=0){
      u = U[i] ;
      return 1 ;
    }
    else{
      u = (U[i]*U[i+1] - U[i+2]*U[i+3])/A ;
      return 1 ;
    }
  }
  case 3:{
    double A = U[i]-U[i+3] ;
    if(A>=0){ // 4 knots at the same place from U[i] to U[i+3]
      u = U[i] ;
      return 1 ;
    }
    A = U[i+1]-U[i+3] ;
    if(A>=0){ // three knots are equal 
      u = U[i+1] ;
      return 1 ;
    }
    // there are 4 points of possible interest
    // the 'good' one lie between U[i+1] and U[i+3]
    double a,b,c,d,e,X ;
    a = U[i] ;
    b = U[i+1] ;
    c = U[i+2] ;
    d = U[i+3] ;
    e = U[i+4] ;

    double t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13,t15,t16,t18;
    double t21,t22,t24,t25,t27,t28,t31,t32,t34,t35,t45,t46 ,t49 ,t52 ,t56;
    double t63 ,t66 ,t69 ,t88 ,t107,t110 ;
    double t115,t116,t117,t118,t119,t120,t121,t122,t124,t125,t127;
    double t133,t135,t136,t143,t151,t154 ;
    double t14,t17,t19,t20,t26,t29,t30,t33,t36,t37,t38,t39,t47,t55,t57 ;
    double t59,t62,t64,t65,t67,t70,t72,t73,t75,t76,t77,t78,t79,t80 ;
    double t81,t82,t83,t84,t85,t86,t87,t90,t91,t92,t93,t95,t96 ;
    double t97,t99,t101 ;

    t1 = b*e;
    t2 = b*b;
    t3 = b*a;
    t4 = b*d;
    t5 = b*c;
    t6 = d*e;
    t7 = c*d;
    t8 = c*e;
    t9 = c*a;
    t10 = a*e;
    t11 = d*a;
    t12 = a*a;
    t13 = -t1+t2+t3-t4-t5+t6+t7+t8-t9-t10-t11+t12;
    t14 = 1/t13;
    t15 = t2*a;
    t16 = t3*e;
    t17 = t3*c;
    t18 = t7*e;
    t19 = t3*d;
    t20 = t12*b;
    t21 = t2*t12;
    t22 = e*e;
    t24 = c*c;
    t25 = t24*t22;
    t26 = t25*t11;
    t27 = t12*t22;
    t28 = t27*t5;
    t29 = t24*t12;
    t30 = t29*t6;
    t31 = t27*t7;
    t32 = d*d;
    t33 = t32*t12;
    t34 = t33*t5;
    t35 = t33*t8;
    t36 = t29*t4;
    t37 = t33*t1;
    t38 = t12*a;
    t39 = t38*b;
    t45 = t21*t22-t26-t28+t30+t31-t34+t35-t36-t37+t39*t7+
          t39*t8-t38*c*t6+t39*t6;
    t46 = t2*b;
    t47 = t46*a;
    t49 = t15*t18;
    t52 = t3*t22*c*d;
    t55 = t24*d*e*t3;
    t56 = t2*t22;
    t57 = t56*t7;
    t59 = c*t32*t16;
    t62 = t7*e*t12*b;
    t63 = t56*t9;
    t64 = t56*t11;
    t65 = t24*t32;
    t66 = t65*t3;
    t67 = t46*c;
    t69 = t2*t32;
    t70 = t69*t9;
    t72 = t69*t8;
    t73 = t47*t8-3.0*t49+2.0*t52+2.0*t55+t57+2.0*t59-3.0*t62-t63-t64+t66+
          t67*t11-t70+t47*t6+t72;
    t75 = t2*t24;
    t76 = t75*t10;
    t77 = t69*t10;
    t78 = t75*t11;
    t79 = t32*t22;
    t80 = t79*t9;
    t81 = t79*t3;
    t82 = t75*t6;
    t83 = t25*t3;
    t84 = t65*t10;
    t85 = t65*t1;
    t86 = t79*t5;
    t87 = t25*t4;
    t88 = t27*t4;
    t90 = -t76-t77-t78-t80+t81+t82+t83-t84-t85-t86-t87-t88-t67*t6;
    t91 = t29*t1;
    t92 = t21*t8;
    t93 = t21*t6;
    t95 = t21*t7;
    t96 = t21*t24;
    t97 = t46*t12;
    t99 = t2*t38;
    t101 = t65*t22;
    t107 = -t91+2.0*t92+2.0*t93+t46*t38+2.0*t95+t96-t97*c-t99*c+t101+t21*t32-
            t99*d-t99*e-t97*e-t97*d;
    t110 = sqrt(t45+t73+t90+t107);

    X = t14*(2.0*t15-2.0*t16-2.0*t17+2.0*t18-2.0*t19+2.0*t20+2.0*t110)/2;

    if(c-b > 0){      

      // X might be near U[i+2] but due to floating point error, it won't
      // be detected. It happens during tests, so it's possible...
      /*
      if(absolute(X-c)<0.0001*c){
	Error error("maxInfluence");
	error << "A numerical error in computing the point of maximal influence" ;
	error.warning() ;
	u = X ;
	return 1 ;
      }
      */

      if(X> b && X<=c + 1e-6 ){ // adding 1e-6 because of float->double conversions
	u = (T)X ;
	return 1 ;
      }

      X = t14*(2.0*t15-2.0*t16-2.0*t17+2.0*t18-2.0*t19+2.0*t20-2.0*t110)/2;
      if(X>b && X<=c){
	u = (T)X ;
	return 1 ;
      }
    }

    t115 = -t9-t32-t6+t1-t3+t4+t10+t11+t8-t5-t22+t7;
    t116 = 1/t115;
    t117 = t6*a;
    t118 = t4*e;
    t119 = d*t22;
    t120 = t32*e;
    t121 = -t26+t28+t30-t31+t34-t35-t36-t37+2.0*t49-3.0*t52+2.0*t55-t57-3.0*
      t59;
    t122 = 2.0*t62+t63-t64+t66+t70-t72-t76-t77-t78+2.0*t80+2.0*t81+t82+t83-
      t84;
    t124 = t32*d;
    t125 = t124*b;
    t127 = t124*t22;
    t133 = t22*e;
    t135 = -t85+2.0*t86-t87-t88-t91-t125*t9-t127*c+t125*t8+t125*t10+t124*a*t8
      +t124*t133-t92+t93;
    t136 = t133*b;
    t143 = t32*t133;
    t151 = -t95+t136*t7-t136*t9+t133*c*t11+t136*t11+t69*t22+t96+t101-t143*c-b
      *t32*t133-t125*t22-t143*a-t127*a+t79*t12;
    t154 = sqrt(t121+t122+t135+t151);


    
    X=t116*(2.0*t117+2.0*t118-2.0*t17-2.0*t119-2.0*t120+2.0*t18+2.0*t154)/2.0;

    if(X>=c - 1e-6 && X<d){
      u = (T)X ;
      return 1 ;
    }

    X=t116*(2.0*t117+2.0*t118-2.0*t17-2.0*t119-2.0*t120+2.0*t18-2.0*t154)/2.0;

    if(X>=c && X<d){
      u = (T)X ;
      return 1 ;
    }

#ifdef USE_EXCEPTION
    throw NurbsComputationError();
#else
    Error error("maxInfluence") ;
    error << "It seems the program couldn't find a proper maxInfluence point\n." ;
    error << "so far u is set to " << u << endl ; 
    error << "and the input arguments were \n" ;
    error << "i = " << i << endl ; 
    error << "U = " << U << endl ; 
    error << "p = " << p << endl ; 
    error.warning() ; 
    return 0 ; 
#endif
  }
  default:{
#ifdef USE_EXCEPTION
    throw NurbsInputError();
#else
    Error error("maxInfluence");
    error << "The point of maximal influence is only computed for a degree of 3 or lower." ;
    error << "The degree given was " << p << endl ; 
    error.warning() ;
    return 0 ;
#endif
  }
  }
  return 0;
}

/*!
  \brief Computes the basis function 
  \relates NurbsCurve NurbsSurface

  Computes the \a i basis function of degree \a p at 
  parameter \a u. 

  \latexonly
    This is often noted as $N_{ip}(u)$.

    The B-spline basis function of $p$-degree is defined as
    \begin{eqnarray}
    N_{i,0}(u) & = & \left\{ \begin{array}{ll} 1 & \mbox{if $u_i \leq u < u_{i+1}$} \\ 0 & \mbox{otherwise}\end{array}\right. \nonumber \\
    N_{i,p}(u) & = & \frac{u-u_i}{u_{i+p}-u_i}N_{i,p-1}(u)+\frac{u_{i+p+1}-u}{u_{i+p+1}-u_{i+1}}N_{i+1,p-1}(u) \nonumber
    \end{eqnarray}
    
    where the $u_i$ define the knot vector $U = \{u_0,\ldots,u_m\}$
    as a nondecreasing sequence of real numbers, {\em i.e.}, 
    $u_i \leq u_{i+1}$ for $i=0,\ldots,m-1$. And $m$ is related
    to the number of control points $n$ and the degree of the curve
    $p$ with the relation $m = n + p + 1$. The knot vector has
    the form
    
    \begin{equation}
    U=\{\underbrace{a,\ldots,a}_{p+1},u_{p+1},\ldots,u_{m-p-1},\underbrace{b,\ldots,b}_{p+1} \} 
    \end{equation}

  \endlatexonly
  \htmlonly
    You can have more information in the LaTeX version.
  \endhtmlonly

  \param u  the parametric variable
  \param i  specifies which basis function to compute
  \param p  the degree to which the basis function is computed
  \param U  the knot vector

  \return the value of  \a N_{ip}(u)

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T>
T nurbsBasisFun(T u, int i, int p, const Vector<T>& U) {
  T Nip ;
  T saved,Uleft,Uright,temp ;
  
  if(p<1){
#ifdef USE_EXCEPTION
    throw NurbsInputError();
#else
    Error error("nurbsBasisFun") ;
    error << "You need to specify a valid degree for the basis function!\n" ;
    error << "p = " << p << " but it requires a value >0.\n" ;
    error.fatal() ; 
#endif
  }

  if((i==0 && u == U[p]) ||
     (i == U.n()-p-2 && u==U[U.n()-p-1])){
    Nip = 1.0 ;
    return Nip ;
  }
  if(u<U[i] || u>=U[i+p+1]){
    Nip = 0.0 ;
    return Nip;
  }

  T* N = (T*) alloca((p+1)*sizeof(T)) ; // Vector<T> N(p+1) ;

  int j ;
  for(j=0;j<=p;j++){
    if(u>=U[i+j] && u<U[i+j+1]) 
      N[j] = 1.0 ;
    else
      N[j] = 0.0 ;
  }
  for(int k=1; k<=p ; k++){
    if(N[0] == 0.0)
      saved = 0.0 ;
    else
      saved = ( (u-U[i])*N[0])/(U[i+k]-U[i]) ;
    for(j=0;j<p-k+1;j++){
      Uleft = U[i+j+1] ;
      Uright = U[i+j+k+1] ;
      if(N[j+1]==0.0){
	N[j] = saved ;
	saved = 0.0 ;
      }
      else {
	temp = N[j+1]/(Uright-Uleft) ;
	N[j] = saved+(Uright-u)*temp ;
	saved = (u-Uleft)*temp ;
      }
    }
  }
  Nip = N[0] ;

  return Nip ;  
}

template <class T, int N>
struct LengthData {
  int span ;
  const NurbsCurve<T,N>* c ; 
  LengthData(const NurbsCurve<T,N>* curve): c(curve) { }
};

template <class T, int N>
struct OpLengthFcn : public ClassPOvoid<T> {
  T operator()(T a, void* pnt){
    LengthData<T,N>* p = (LengthData<T,N>*)pnt ;
    return (p->c)->lengthF(a,p->span) ; 
  }
};


/*!
  \brief Computes the length of the curve

  Computes an approximation of the length of the curve
  using a numerical automatic integrator.
  
  That integrator uses a Chebyshev Series Expansion
  to perform its approximation. This is why you can
  change the value $n$ which sets the number of 
  elements in the series. 

  The method is simple, integrate between each span.
  This is necessary in case the tangant of a point
  at u_i is undefined. Add the result and return
  this as the approximation.

  \param n  the number of element in the Chebyshev series
  \param eps  the accepted relative error
  
  \return the length of the NURBS curve.

  \author Philippe Lavoie
  \date 22 September 1998
*/
template <class T, int N>
T NurbsCurve<T,N>::length(T eps,int n) const {
  T l = T() ;
  T err ; 

  static Vector<T> bufFcn ;

  if(bufFcn.n() != n){
    bufFcn.resize(n) ;
    intccini(bufFcn) ;
  }

  LengthData<T,N> data(this) ; 
  OpLengthFcn<T,N> op;

  for(int i=deg_;i<P.n();++i){
    if(U[i] >= U[i+1])
      continue ;
    data.span = i ; 
    l += intcc((ClassPOvoid<T>*)&op,(void*)&data,U[i],U[i+1],eps,bufFcn,err) ;
  }
  return l ; 
}

/*!
  \brief Computes the length of the curve inside [u_s,u_e]

  Computes an approximation of the length of the curve
  using a numerical automatic integrator. The length
  is computed for the range [u_s,u_e]
  
  That integrator uses a Chebyshev Series Expansion
  to perform its approximation. This is why you can
  change the value \a n which sets the number of 
  elements in the series. 

  The method is similar to the one used by length
  excepted that it needs to check for the range.

  \param  us  the starting range 
  \param  ue  the end of the range
  \param  n  the number of element in the Chebyshev series
  \param  eps  the accepted relative error

  \return the length of the NURBS curve.

  \warning ue must be greater than us and both must be in a valid range.

  \author Philippe Lavoie
  \date 22 September 1998
*/
template <class T, int N>
T NurbsCurve<T,N>::lengthIn(T us, T ue,T eps, int n) const {
  T l = T() ;
  T err ; 

  static Vector<T> bufFcn ;

  if(bufFcn.n() != n){
    bufFcn.resize(n) ;
    intccini(bufFcn) ;
  }

  LengthData<T,N> data(this) ; 
  OpLengthFcn<T,N> op;

  for(int i=deg_;i<P.n();++i){
    if(U[i] >= U[i+1])
      continue ;
    data.span = i ; 
    if(i<findSpan(us))
      continue ; 
    if(us>=U[i] && ue<=U[i+findMult(i)]){
      l = intcc((ClassPOvoid<T>*)&op,(void*)&data,us,ue,eps,bufFcn,err) ;
      break ;
    }
    if(us>=U[i]){
      l += intcc((ClassPOvoid<T>*)&op,(void*)&data,us,U[i+findMult(i)],eps,bufFcn,err) ;
      continue ;
    }
    if(ue<=U[i+findMult(i)]){
      l += intcc((ClassPOvoid<T>*)&op,(void*)&data,U[i],ue,eps,bufFcn,err) ;
      break ;
    }
    l += intcc((ClassPOvoid<T>*)&op,(void*)&data,U[i],U[i+findMult(i)],eps,bufFcn,err) ;
  }
  return l ; 
}

// the definitions are in f_nurbs.cpp and d_nurbs.cpp


/*!
  \brief The function used by length
  \a length needs to integrate a function over an interval
  to determine the length of the NURBS curve. 
  Well, this is the function.

  \param  u --> the parameter

  \return square root of the square of the x,y and z value

  \author Philippe Lavoie
  \date 22 September 1998
*/
template <class T, int N>
T NurbsCurve<T,N>::lengthF(T u) const {
  Point_nD<T,N> dd = firstDn(u) ; 
  T tmp = sqrt(dd.x()*dd.x()+dd.y()*dd.y()+dd.z()*dd.z()) ;
  return tmp ; 
}

/*!

  \a length needs to integrate a function over an interval
  to determine the length of the NURBS curve. 
  Well, this is the function.

  \param u  the parameter
  \param span  the span of the parameter

  \return square root of the square of the x,y and z value

  \author Philippe Lavoie
  \date 22 September 1998
*/
template <class T, int N>
T NurbsCurve<T,N>::lengthF(T u, int span) const {
  Point_nD<T,N> dd = firstDn(u,span) ; 
  T tmp = sqrt(dd.x()*dd.x()+dd.y()*dd.y()+dd.z()*dd.z()) ;
  return tmp ; 
}


/*!
  \brief Generate a straight line

  Generate a straight line going from point P0 to point P1
  of degree \a d.
  
  \param P0  the beginning of the line
  \param P1  the end of the line
  \param d  the degree of the curve

  \warning \a d must be greater or equal to 1
  \author Philippe Lavoie
  \date 22 September 1998
*/
template <class T, int N>
void NurbsCurve<T,N>::makeLine(const Point_nD<T,N>& P0, const Point_nD<T,N>& P1, int d) {
  if(d<2)
    d = 2 ;
  resize(2,1) ;
  P[0] = HPoint_nD<T,N>(P0) ;
  P[1] = HPoint_nD<T,N>(P1) ;
  U[0] = U[1] = 0 ; 
  U[2] = U[3] = 1 ;
  degreeElevate(d-1) ;
}

/*!
  \brief Computes the first derivative
  Computes the first derivative in the 4D homogenous space.

  \param u --> compute the derivative at this parameter

  \return The first derivative in homogenous space

  \warning \a u must be in the valid range

  \author Philippe Lavoie
  \date 13 October 1998
*/
template <class T, int D>
HPoint_nD<T,D> NurbsCurve<T,D>::firstD(T u) const {
  int span = findSpan(u) ; 
  int i ; 

  static Vector<T> N ;

  nurbsBasisFuns(u,span,deg_-1,U,N) ;

  HPoint_nD<T,D> Cd(0,0,0,0) ;
  HPoint_nD<T,D> Qi ;

  for(i=deg_-1;i>=0;--i){
    int j = span-deg_+i ; 
    Qi = (P[j+1]-P[j]);
    Qi *= T(deg_)/(U[j+deg_+1]-U[j+1]) ; 
    Cd += N[i]*Qi ; 
  }

  return Cd ;
}

/*!
  \brief Computes the first derivative

  Computes the first derivative in the honogenous space with the span given.

  \param u  compute the derivative at this parameter
  \param span  the span of u

  \return The first derivative of the point in the homogoneous space

  \warning \a u and span must be in a valid range

  \author Philippe Lavoie
  \date 13 October 1998
*/
template <class T, int D>
HPoint_nD<T,D> NurbsCurve<T,D>::firstD(T u, int span) const {
  int i ; 

  static Vector<T> N ;

  nurbsBasisFuns(u,span,deg_-1,U,N) ;

  HPoint_nD<T,D> Cd(0,0,0,0) ;
  HPoint_nD<T,D> Qi ;

  for(i=deg_-1;i>=0;--i){
    int j = span-deg_+i ; 
    Qi = (P[j+1]-P[j]);
    Qi *= T(deg_)/(U[j+deg_+1]-U[j+1]) ; 
    Cd += N[i]*Qi ; 
  }

  return Cd ;
}


/*!
  \brief Computes the first derivative

  Computes the first derivative in the normal space.
  
  \param u  compute the derivative at this parameter
  \param span  the span of u

  \return The first derivative in normal space

  \warning \a u and span must be in a valid range
  \author Philippe Lavoie
  \date 13 October 1998
*/
template <class T, int N>
Point_nD<T,N> NurbsCurve<T,N>::firstDn(T u) const {
  int span = findSpan(u) ; 
  Point_nD<T,N> Cp ; 
  HPoint_nD<T,N> Cd ; 
  Cd = firstD(u,span) ;

  Point_nD<T,N> pd(Cd.projectW()) ;
  T w = Cd.w() ;
  Cd = hpointAt(u,span) ; 
  pd -= w*project(Cd) ;
  pd /= Cd.w() ;

  return pd ;
}

/*!
  \brief Computes the first derivative

  Computes the first derivative in the normal space (3D or 2D).

  \param u --> compute the derivative at this parameter
  \param span --> the span of \a u

  \warning \a u and \a span must be in a valid range

  \author Philippe Lavoie 
  \date 13 October 1998
*/
template <class T, int N>
Point_nD<T,N> NurbsCurve<T,N>::firstDn(T u, int span) const {
  int i ; 
  Point_nD<T,N> Cp ; 
  HPoint_nD<T,N> Cd ; 
  Cd = firstD(u,span) ;

  Point_nD<T,N> pd(Cd.projectW()) ;
  T w = Cd.w() ;
  Cd = hpointAt(u,span) ; 
  pd -= w*project(Cd) ;
  pd /= Cd.w() ;

  return pd ;
}

/*!
  \brief A least squares curve approximation for closed curves
  
  \latexonly
    This routine solves the following problem: find the NURBS curve
    $C$ satisfying
    \begin{itemize}
    \item the $Q_k$ are approximated in the least squares
    sense, {\em i.e.}
    \[ \sum_{k=0}^{m} | Q_k-C(\bar{u}_k)|^2 \]
    in a minimum with respect to the $n$ variable $P_i$; the
    $\bar{u}$ are the precomputed parameter values.
    \end{itemize}

    The resulting curve will generally not pass through $Q_k$ and
    $C(\bar{u}_k)$ is not the closest point on $C(u)$ to $Q_k$.
  \endlatexonly
  \htmlonly
    This routine finds a closed NURBS curve that satisfy a least
    square criteria. The resulting curve will generally not pass
    through the input points (except the first point).
  \endhtmlonly

  For more details, see section 9.4.1 on page 491 of the NURBS 
  book.

  \param Qw  the vector of 3D points (wrapped around)
  \param degC   the degree of the curve
  \param nCP   the number of (distinct) control points in the new curve

  \author Alejandro Frangi 
  \date 30 July 1998
*/
template <class T, int N>
int NurbsCurve<T,N>::leastSquaresClosed(const Vector< Point_nD<T,N> >& Qw, int degC, int nCP){

  Vector<T> ub;
  Vector<T> Uk;
  chordLengthParamClosed(Qw,ub,degC) ;
  return leastSquaresClosed(Qw,degC,nCP,ub);
}

/*!
					
  \brief A least squares curve approximation for closed curves
  
  \latexonly
    This routine solves the following problem: find the NURBS curve
    $C$ satisfying
    \begin{itemize}
    \item the $Q_k$ are approximated in the least squares
    sense, {\em i.e.}
    \[ \sum_{k=0}^{m} | Q_k-C(\bar{u}_k)|^2 \]
    in a minimum with respect to the $n$ variable $P_i$; the
    $\bar{u}$ are the precomputed parameter values.
    \end{itemize}

    The resulting curve will generally not pass through $Q_k$ and
    $C(\bar{u}_k)$ is not the closest point on $C(u)$ to $Q_k$.
  \endlatexonly
  \htmlonly
    This routine finds a closed NURBS curve that satisfy a least
    square criteria. The resulting curve will generally not pass
    through the input points (except the first point).
  \endhtmlonly

  For more details, see section 9.4.1 on page 491 of the NURBS 
  book.

  \param Qw  the vector of 3D points (wrapped around)
  \param degC the degree of the curve
  \param nCP the number of (distinct) control points in the new curve
  \param ub the parameter values of Qw obtained via chordLengthParamClosed()

  \author Alejandro Frangi 
  \date 30 July 1998
*/
template <class T, int N>
int NurbsCurve<T,N>::leastSquaresClosed(const Vector< Point_nD<T,N> >& Qw, int degC, int nCP, const Vector<T>& ub){
  Vector<T> Uk;
  knotApproximationClosed(Uk,ub,nCP+degC-1,degC);
  return leastSquaresClosed(Qw,degC,nCP,ub,Uk);
}

/*!
  \brief A least squares curve approximation for closed curves
  
  \latexonly
    This routine solves the following problem: find the NURBS curve
    $C$ satisfying
    \begin{itemize}
    \item the $Q_k$ are approximated in the least squares
    sense, {\em i.e.}
    \[ \sum_{k=0}^{m} | Q_k-C(\bar{u}_k)|^2 \]
    in a minimum with respect to the $n$ variable $P_i$; the
    $\bar{u}$ are the precomputed parameter values.
    \end{itemize}

    The resulting curve will generally not pass through $Q_k$ and
    $C(\bar{u}_k)$ is not the closest point on $C(u)$ to $Q_k$.
  \endlatexonly
  \htmlonly
    This routine finds a closed NURBS curve that satisfy a least
    square criteria. The resulting curve will generally not pass
    through the input points (except the first point).
  \endhtmlonly

  For more details, see section 9.4.1 on page 491 of the NURBS 
  book.

  \param Qw  the vector of 4D points (wrapped around)
  \param degC  the degree of the curve
  \param nCP  the number of (distinct) control points in thenew curve
  \param ub  the parameter values of Qw obtained via chordLengthParamClosed()

  \author Alejandro Frangi 
  \date 30 July 1998
*/
template <class T, int N>
int NurbsCurve<T,N>::leastSquaresClosedH(const Vector< HPoint_nD<T,N> >& Qw, int degC, int nCP, const Vector<T>& ub){

  Vector<T> Uk;
  knotApproximationClosed(Uk,ub,nCP+degC-1,degC);

  return leastSquaresClosedH(Qw,degC,nCP,ub,Uk);
}

/*!
  \brief A least squares curve approximation for closed curves
  
  \latexonly
    This routine solves the following problem: find the NURBS curve
    $C$ satisfying
    \begin{itemize}
    \item the $Q_k$ are approximated in the least squares
    sense, {\em i.e.}
    \[ \sum_{k=0}^{m} | Q_k-C(\bar{u}_k)|^2 \]
    in a minimum with respect to the $n$ variable $P_i$; the
    $\bar{u}$ are the precomputed parameter values.
    \end{itemize}

    The resulting curve will generally not pass through $Q_k$ and
    $C(\bar{u}_k)$ is not the closest point on $C(u)$ to $Q_k$.
  \endlatexonly
  \htmlonly
    This routine finds a closed NURBS curve that satisfy a least
    square criteria. The resulting curve will generally not pass
    through the input points (except the first point).
  \endhtmlonly

  For more details, see section 9.4.1 on page 491 of the NURBS 
  book.

  \param Qw  the vector of 4D points (wrapped around)
  \param degC  the degree of the curve
  \param nCP  the number of (distinct) control points in the new curve
  \param ub  the parameter values of Qw obtained via chordLengthParamClosed()
  \param knots  the knots for the control points obtained via knotApproximationClosed()

  \author Alejandro Frangi 
  \date 30 July 1998
*/
template <class T, int D>
int NurbsCurve<T,D>::leastSquaresClosed(const Vector< Point_nD<T,D> >& Qw, int degC, int nCP, const Vector<T>& ub, const Vector<T>& knots){
  resize(nCP+degC,degC)  ;

  int n  = P.n()-1;
  int iN = nCP-1;
  int iM = Qw.n()-degC-1;
  int p  = degC ;

  if(ub.n() != Qw.n()){
#ifdef USE_EXCEPTION
    throw NurbsInputError(ub.n(),Qw.n());
#else
    Error err("leastSquaresClosed");
    err << "leastSquaresCurveC\n" ;
    err << "ub size is different than Qw's\n" ;
    err.fatal();
#endif
  }

  if( knots.n() != U.n()){
#ifdef USE_EXCEPTION
    throw NurbsInputError(knots.n(),U.n());
#else
    Error err("leastSquaresClosed");
    err << "The knot vector supplied doesn't have the proper size.\n" ;
    err << "It should be n+degC+1 = " << U.n() << " and it is " << knots.n() << endl ;
    err.fatal() ;
#endif
  }

  Matrix_DOUBLE N(iM+1,iN+1);
  Matrix_DOUBLE A(iN+1,iN+1);
  Matrix_DOUBLE R(iN+1,D);
  Matrix_DOUBLE Pi(iN+1,D);

  // Load knot vector
  U = knots;

  // Form matrix N (Eq. 9.66) p. 411
  N = 0;
  for (int i=0; i<=n; i++)
    for (int k=0; k<=iM; k++)
      N(k,i%(iN+1)) += basisFun(ub[k],i,p) ;

  // Form R (Eq. 9.67)
  R.reset(0.0);
  for (int i=0; i<=iN; i++)
    for (int k=0; k<=iM; k++){
      const Point_nD<T,D>& qp = Qw[k] ; // this makes the SGI compiler happy
      const double&  Nki = N(k,i);
      for (int j=0; j<D; j++)
        R(i,j) +=  Nki * qp.data[j] ;
    }

  // The system to solve
  A = N.transpose() * N ;
  solve(A,R,Pi);

  // Update control points
  for (int i=0; i<= n; i++)
    for (int k=0; k<D; k++){
      P[i].data[k] = Pi(i%(iN+1),k) ;
      P[i].w()     = 1.0;
    }
  return 1;
}


/*!
  \brief A least squares curve approximation for closed curves
  
  \latexonly
    This routine solves the following problem: find the NURBS curve
    $C$ satisfying
    \begin{itemize}
    \item the $Q_k$ are approximated in the least squares
    sense, {\em i.e.}
    \[ \sum_{k=0}^{m} | Q_k-C(\bar{u}_k)|^2 \]
    in a minimum with respect to the $n$ variable $P_i$; the
    $\bar{u}$ are the precomputed parameter values.
    \end{itemize}

    The resulting curve will generally not pass through $Q_k$ and
    $C(\bar{u}_k)$ is not the closest point on $C(u)$ to $Q_k$.
  \endlatexonly
  \htmlonly
    This routine finds a closed NURBS curve that satisfy a least
    square criteria. The resulting curve will generally not pass
    through the input points (except the first point).
  \endhtmlonly

  For more details, see section 9.4.1 on page 491 of the NURBS 
  book.

  \param Qw  the vector of 4D points (wrapped around)
  \param degC  the degree of the curve
  \param nCP  the number of (distinct) control points in the new curve
  \param ub  the parameter values of Qw obtained via chordLengthParamClosed()
  \param knots  the knots for the control points obtained via knotApproximationC()

  \author Alejandro Frangi 
  \date 30 July 1998
*/
template <class T, int D>
int NurbsCurve<T,D>::leastSquaresClosedH(const Vector< HPoint_nD<T,D> >& Qw, int degC, int nCP, const Vector<T>& ub, const Vector<T>& knots){

  resize(nCP+degC,degC)  ;

  int n  = P.n()-1;
  int iN = nCP-1;
  int iM = Qw.n()-degC-1;
  int p  = degC ;


  if(ub.n() != Qw.n()){
#ifdef USE_EXCEPTION
    throw NurbsInputError(ub.n(),Qw.n());
#else
    Error err("leastSquaresClosedH");
    err << "leastSquaresCurveC\n" ;
    err << "ub size is different than Qw's\n" ;
    err.fatal();
#endif
  }

  if( knots.n() != U.n()){
#ifdef USE_EXCEPTION
    throw NurbsInputError(knots.n(),U.n());
#else
    Error err("leastSquaresClosed");
    err << "The knot vector supplied doesn't have the proper size.\n" ;
    err << "It should be n+degC+1 = " << U.n() << " and it is " << knots.n() << endl ;
    err.fatal() ;
#endif
  }

  Matrix_DOUBLE N(iM+1,iN+1);
  Matrix_DOUBLE A(iN+1,iN+1);
  Matrix_DOUBLE R(iN+1,D+1);
  Matrix_DOUBLE Pi(iN+1,D+1);

  // Load knot vector
  U = knots;

  // Form matrix N (Eq. 9.66) p. 411
  N = 0;
  for (int i=0; i<=n; i++)
    for (int k=0; k<=iM; k++)
      N(k,i%(iN+1)) += basisFun(ub[k],i,p) ;

  // Form R (Eq. 9.67)
  R.reset(0.0);
  for (int i=0; i<=iN; i++)
    for (int k=0; k<=iM; k++){
      const HPoint_nD<T,D>& qp = Qw[k] ; // this makes the SGI compiler happy
      const double&  Nki = N(k,i);
      for (int j=0; j<D+1; j++)
        R(i,j) +=  Nki * qp.data[j] ;
    }

  // The system to solve
  A = N.transpose() * N ;
  solve(A,R,Pi);

  // Update control points
  for (int i=0; i<= n; i++)
    for (int k=0; k<D+1; k++){
      P[i].data[k] = Pi(i%(iN+1),k) ;
      //P[i].w()     = 1.0;
    }
  return 1;
}


/*!
  \brief chord length parameterization for a closed curve
  
  Performs chord length parameterization from a vector of
  points that are supposed to form a closed curve.

  \param Qw  a vector of 3D points (already wrapped around)
  \param ub  the result of chord length parameterization
  \param deg  the degree of the curve

  \return the total chord length of the points.
  \author Alejandro Frangi 
  \date 30 July, 1998
*/
template <class T, int N>
T chordLengthParamClosed(const Vector< Point_nD<T,N> >& Qw, Vector<T> &ub,int deg){

  int i ;
  T d = 0.0 ;

  ub.resize(Qw.n()) ;
  ub[0] = 0 ; 
  for(i=1;i<=ub.n()-deg;i++){
    d += norm(Qw[i]-Qw[i-1]) ;
  }
  if(d>0){
    for(i=1;i<ub.n();++i)
      ub[i] = ub[i-1] + norm(Qw[i]-Qw[i-1]);    
    // Normalization
    for(i=0;i<ub.n();++i)
      ub[i]/=  d;
  }
  else
    for(i=1;i<ub.n();++i)
      ub[i] = (T)(i)/(T)(ub.n()-2) ;

  return d ;
}

/*!
  \brief chord length parameterization for a closed curve
  
  Performs chord length parameterization from a vector of
  points that are supposed to form a closed curve.

  \param Q  a vector of 3D points (wrapped around)
  \param ub  the result of chord length parameterization
  \param deg  the degree of the curve

  \return the total chord length of the points.
  \author Alejandro Frangi  
  \date 30 July, 1998
*/
template <class T, int N>
T chordLengthParamClosedH(const Vector< HPoint_nD<T,N> >& Qw, Vector<T> &ub,int deg){

  int i ;
  T d = 0.0 ;

  ub.resize(Qw.n()) ;
  ub[0] = 0 ; 
  for(i=1;i<ub.n()-deg+1;i++){
    d += norm(Qw[i]-Qw[i-1]) ;
  }
  if(d>0){
    for(i=1;i<ub.n();++i)
      ub[i] = ub[i-1] + norm(Qw[i]-Qw[i-1]);    
    // Normalization
    for(i=0;i<ub.n();++i)
      ub[i]/=ub[ub.n()-deg] ;
  }
  else
    for(i=1;i<ub.n();++i)
      ub[i] = (T)(i)/(T)(ub.n()-deg) ;

  return d ;
}


/*!
  \brief refine the closed curve knot vector
  
  Adapted from algorithm A5.4 on page 164 of the NURBS book.

  \param X the knot vector to refine with

  \author Alejandro F Frangi 
  \date 17 July, 1998
*/
template <class T, int N>
void NurbsCurve<T,N>::refineKnotVectorClosed(const Vector<T>& X){

  int n = P.n()-1 ;
  int p = deg_ ;
  int m = n+p+1 ;
  int a,b ;
  int r = X.n()-1 ;


  NurbsCurve<T,N> c(*this) ;
  resize(r+1+n+1,p) ;
  a = c.findSpan(X[0]) ;
  b = c.findSpan(X[r]) ;
  ++b ;
  int j ;

  for(j=0; j<=a-p ; j++)
    P[j] = c.P[j] ;
  for(j = b-1 ; j<=n ; j++)
    P[j+r+1] = c.P[j] ;
  for(j=0; j<=a ; j++)
    U[j] = c.U[j] ;
  for(j=b+p ; j<=m ; j++)
    U[j+r+1] = c.U[j] ;

  int i = b+p-1 ; 
  int k = b+p+r ;
  for(j=r; j>=0 ; j--){
    while(X[j] <= c.U[i] && i>a){
      int ind = i-p-1;
      if (ind<0)
        ind += n + 1 ;
      P[k-p-1] = c.P[ind] ;
      U[k] = c.U[i] ;
      --k ;
      --i ;
    }
    P[k-p-1] = P[k-p] ;
    for(int l=1; l<=p ; l++){
      int ind = k-p+l ;
      T alpha = U[k+l] - X[j] ;
      if(alpha==0.0)
        P[ind-1] = P[ind] ;
      else {
        alpha /= U[k+l]-c.U[i-p+l] ;
        P[ind-1] = alpha*P[ind-1] + (1.0-alpha)*P[ind] ;
      }
    }
    U[k] = X[j] ;
    --k ;
  }
}


/*!
  \brief global closed curve interpolation with a list of points
  
  Global curve interpolation with points in 3D. This
  function will generate a closed curve with C(d-1)
  continuity between the parameters u=0 and u=1

  \param Q  the 3D points to interpolate
  \param d  the degree of the interpolation

  \warning The number of points to interpolate must be greater than
           the degree specified for the curve.
  \author Alejandro Frangi 
  \date 13 July, 1998
*/
template <class T, int N>
void NurbsCurve<T,N>::globalInterpClosed(const Vector< Point_nD<T,N> >& Qw, int d){
  Vector<T> ub ;
  Vector<T> Uc;

  chordLengthParamClosed(Qw,ub,d) ;  
  knotAveragingClosed(ub,d,Uc);

  globalInterpClosed(Qw,ub,Uc,d) ;
}

/*!
  \brief global close curve interpolation with points in homogenous space

  
  Global curve interpolation with points in homogenouse space with C(d-1)
  continuity in the wrap-around point. 

  \param Qw  the points in 4D to interpolate
  \param d the degree of the closed curve

  \warning The number of points to interpolate must be greater than
           the degree specified for the curve. The interpolation
           degree is only 3. The first and last interpolation points
           should be equal.

  \author   Alejandro Frangi          
  \date 13 July, 1998
*/
template <class T, int D>
void NurbsCurve<T,D>::globalInterpClosedH(const Vector< HPoint_nD<T,D> >& Qw, int d){

  Vector<T> ub ;
  Vector<T> Uc;

  chordLengthParamClosedH(Qw,ub,d) ;  
  knotAveragingClosed(ub,d,Uc);
  globalInterpClosedH(Qw,ub,Uc,d);

}

/*!
  \brief global curve interpolation with homogenous points
  
  Global curve interpolation with 4D points, a knot vector
  defined and the parametric value vector defined.

  \param Q  the 3D points to interpolate
  \param ub  the parametric values vector
  \param d the degree of the closed curve

  \warning The number of points to interpolate must be greater than
           the degree specified for the curve. Uc must be compatible with 
	   the values given for Q.n(), ub.n() and d.

  \author    Alejandro Frangi 
  \date 13 July, 1998
*/
template <class T, int D>
void NurbsCurve<T,D>::globalInterpClosed(const Vector< Point_nD<T,D> >& Qw, const Vector<T>& ub, int d){

  Vector<T> Uc;
  knotAveragingClosed(ub,d,Uc);
  globalInterpClosed(Qw,ub,Uc,d);
}

/*!
  \brief global curve interpolation with homogenous points
  
  Global curve interpolation with 4D points, a knot vector
  defined and the parametric value vector defined.

  \param Q  the 3D points to interpolate
  \param ub  the parametric values vector
  \param d the degree of the closed curve

  \warning The number of points to interpolate must be greater than
           the degree specified for the curve. Uc must be compatible with 
	   the values given for Q.n(), ub.n() and d.

  \author    Alejandro Frangi 
  \date 13 July, 1998
*/
template <class T, int D>
void NurbsCurve<T,D>::globalInterpClosedH(const Vector< HPoint_nD<T,D> >& Qw, const Vector<T>& ub, int d){

  Vector<T> Uc;
  knotAveragingClosed(ub,d,Uc);
  globalInterpClosedH(Qw,ub,Uc,d);
}

/*!
  \brief global curve interpolation with homogenous points
  
  Global curve interpolation with 4D points, a knot vector
  defined and the parametric value vector defined.The curve will have C(d-1)
  continuity at the point u=0 and u=1.

  \param Qw  the 3D points to interpolate (wrapped around)
  \param ub  the parametric values vector
  \param Uc  the knot vector computed using knotAveragingC
  \param d   the degree of the closed curve

  \warning The number of points to interpolate must be greater than
           the degree specified for the curve. Uc must be compatible with 
	   the values given for Q.n(), ub.n(). 
  \author  Alejandro Frangi 
  \date 13 July, 1998
*/
template <class T, int D>
void NurbsCurve<T,D>::globalInterpClosed(const Vector< Point_nD<T,D> >& Qw,
			const Vector<T>& ub, const Vector<T>& Uc, int d){
  int i,j ;

  resize(Qw.n(),d) ;

  int iN = Qw.n() - d - 1;
  Matrix_DOUBLE A(iN+1,iN+1) ;
  
  if(Uc.n() != U.n()){
#ifdef USE_EXCEPTION
    throw NurbsInputError(Uc.n(),U.n());
#else
    Error err("globalInterpClosedH");
    err << "Invalid dimension for the given Knot vector.\n" ;
    err << "U required = " << U.n() << ", U given = " << Uc.n() << endl ;
    err.fatal() ;
#endif
  }
  U = Uc ;

  // Initialize the basis matrix A
  Vector<T> N(d+1) ;

  for(i=0;i<=iN;i++){
    int span = findSpan(ub[i]);
    basisFuns(ub[i],span,N) ;
    for(j=span-d;j<=span;j++) 
      A(i,j%(iN+1)) = (double)N[j-span+d] ;
  }

  // Init matrix for LSE
  Matrix_DOUBLE qq(iN+1,D) ;
  Matrix_DOUBLE xx(iN+1,D) ;
  for(i=0;i<=iN ;i++)
    for(j=0; j<D;j++)
      qq(i,j) = (double)Qw[i].data[j] ;

  // AF: "solve" calls LU decomposition if A is square. I opted for
  // using the SVD routine which works better when the system of
  // equations is very large (more than 50 points). Probably since in
  // this cases the system matrix A is very sparse.
  SVDMatrix<double> svd(A) ;
  svd.solve(qq,xx) ;

  // Store the data
  for(i=0;i<xx.rows();i++){
    for(j=0;j<D;j++)
      P[i].data[j] = (T)xx(i,j) ;
    P[i].w() = 1.0 ; 
  }
  
  // Wrap around of control points
  for(i=0;i<d;i++){
    for(j=0;j<D;j++)
      P[xx.rows()+i].data[j] = (T)xx(i,j) ;
  }
}

/*!
  \brief global curve interpolation with homogenous points
  
  Global curve interpolation with 4D points, a knot vector
  defined and the parametric value vector defined.The curve will have C(d-1)
  continuity at the point u=0 and u=1.

  \param Qw  the 3D points to interpolate (wrapped around)
  \param ub  the parametric values vector
  \param Uc  the knot vector computed using knotAveragingClosed
  \param d   the degree of the closed curve

  \warning The number of points to interpolate must be greater than
           the degree specified for the curve. Uc must be compatible with 
	   the values given for Q.n(), ub.n(). 
  \author  Alejandro Frangi 
  \date 13 July, 1998
*/
template <class T, int D>
void NurbsCurve<T,D>::globalInterpClosedH(const Vector< HPoint_nD<T,D> >& Qw,
					  const Vector<T>& ub, const Vector<T>& Uc, int d){
  int i,j ;

  resize(Qw.n(),d) ;

  int iN = Qw.n() - d - 1;
  Matrix_DOUBLE A(iN+1,iN+1) ;
  
  if(Uc.n() != U.n()){
#ifdef USE_EXCEPTION
    throw NurbsInputError(Uc.n(),U.n());
#else
    Error err("globalInterpClosedH");
    err << "Invalid dimension for the given Knot vector.\n" ;
    err << "U required = " << U.n() << ", U given = " << Uc.n() << endl ;
    err.fatal() ;
#endif
  }
  U = Uc ;

  // Initialize the basis matrix A
  Vector<T> N(d+1) ;

  for(i=0;i<=iN;i++){
    int span = findSpan(ub[i]);
    basisFuns(ub[i],span,N) ;
    for(j=span-d;j<=span;j++) 
      A(i,j%(iN+1)) = (double)N[j-span+d] ;
  }

  // Init matrix for LSE
  Matrix_DOUBLE qq(iN+1,D+1) ;
  Matrix_DOUBLE xx(iN+1,D+1) ;
  for(i=0;i<=iN ;i++)
    for(j=0; j<D+1;j++)
      qq(i,j) = (double)Qw[i].data[j] ;

  // AF: "solve" calls LU decomposition if A is square. I opted for
  // using the SVD routine which works better when the system of
  // equations is very large (more than 50 points). Probably since in
  // this cases the system matrix A is very sparse.
  SVDMatrix<double> svd(A) ;
  svd.solve(qq,xx) ;

  // Store the data
  for(i=0;i<xx.rows();i++){
    for(j=0;j<D+1;j++)
      P[i].data[j] = (T)xx(i,j) ;
  }
  
  // Wrap around of control points
  for(i=0;i<d;i++){
    for(j=0;j<D+1;j++)
      P[xx.rows()+i].data[j] = (T)xx(i,j) ;
  }
}

/*!
   \brief decompose the closed curve into Bézier segments
   
   This function decomposes a closed curve into an array of Bézier 
   segments.

   \param c an array of Bézier segments

   \author Alejandro Frangi
   \date 30 July 1998
*/
template <class T, int D>
void NurbsCurve<T,D>::decomposeClosed(NurbsCurveArray<T,D>& c) const {

  int ix,b,nb,mult,j ;
  Vector<T> alphas(deg_+1) ;
  Vector<T> Uexpanded(U.n()+2*deg_) ;
  Vector< HPoint_nD<T,D> > Pexpanded(P.n()+2*deg_) ;

  int N = P.n() - deg_ - 1;
  int i ; 

  // Left side
  for (i=0; i<deg_; i++){
    Pexpanded[i] = P[P.n()-2*deg_+i] ;
    Uexpanded[i] = U[N+1-deg_+i] - 1     ;
  }

  // Copy
  for (i=0; i<P.n(); i++)
    Pexpanded[i+deg_] = P[i] ;
  for (i=0; i<U.n(); i++)
    Uexpanded[i+deg_] = U[i] ;
  
  // Right side
  for (i=0; i<deg_; i++){
    Pexpanded[i+deg_+P.n()] = P[deg_+i] ;
    Uexpanded[i+deg_+U.n()] = 1 + U[2*deg_+1+i] ;
  }
  // Now do the decomposition
  Vector<T> nU ;
  nU.resize(2*(deg_+1)) ;
  for(i=0;i<nU.n()/2;i++)
    nU[i] = 0 ;
  for(i=nU.n()/2;i<nU.n();i++)
    nU[i] = 1 ;
  
  c.resize(P.n()) ;

  for(i=0;i<c.n();i++){
    c[i].resize(deg_+1,deg_) ;
    c[i].U = nU ;
  }
    
  Vector<T> X(Uexpanded.n()*deg_) ;

  // Construct the refinement vector
  ix= 0;
  i = 0;
  b = 2*deg_;

  while ( b < U.n() ) {
    i = b;
    while( b < Uexpanded.n()-1 && Uexpanded[b+1] <= Uexpanded[b] ) b++ ;
    mult = b-i+1 ;

    if(mult<deg_){
      for(j=deg_;j>=mult;j--) {
        X[ix] = Uexpanded[b] ;
        ix++ ;
      }
    }
    b++;
  }

  X.resize(ix);

  NurbsCurve<T,D> cl = NurbsCurve(Pexpanded,Uexpanded,deg_);
  cl.refineKnotVectorClosed(X) ;

  // The number of Bezier segments coincides with the number of
  // distinct control points in a closed curve
  nb = P.n()-deg_;

  c.resize(nb);
  for (i=0; i<c.n(); i++)
    for (int k=0; k<=deg_; k++)
      c[i].P[k] = cl.P[i*(deg_+1)+k+2*deg_];
}

/*! 
  \brief generates a knot vector using the averaging technique for interpolation with closed curve.
  
  Generates a knot vector using the averaging technique for interpolation with closed curve. See eq 9.9 in the NURBS Book

  \param uk  the knot coefficients
  \param deg  the degree of the curve associated with the knot vector
  \param U  an average knot vector

  \author Alejandro Frangi
  \date 13 July, 1998
*/
template <class T>
void knotAveragingClosed(const Vector<T>& uk, int deg, Vector<T>& U){

  U.resize(uk.n()+deg+1) ;

  int i, j ;
  int index;
  int iN = uk.n() - deg - 1;
  int n  = uk.n() - 1;
  int m  = U.n()  - 1;
 
  // Build temporary average sequence
  // Data stored in range U[deg+1 .. n]
  for (j=0; j<=iN; j++) { 
    index = j+deg+1;
    U[index] = 0.0;
    for (i=j; i<=j+deg-1; i++) 
      U[index] += uk[i];
    U[index] /= deg;
  }

  // Now make the left and right periodic extensions
  // Left 
  for (j=0; j<deg; j++)    U[j] = U[j+iN+1] - 1;       
  // Right
  for (j=n+1; j<=m; j++)   U[j] = 1 + U[j-(iN+1)];
  
}

/*!
  \brief compute the knots for closed curve approximation
      
  Generates a parameter vector by using the algorithm in
  eqs (9.68) and (9.69) of page 412 in the NURBS Book

  \param   U  the knot vector
  \param ub  the parameter vector of the approximated points (wrapped)
  \param n+1  the number of total control points (including wrapped ones!)
  \param p   the degree of the curve associated with the knot vector

  \author Alejandro Frangi 
  \date 30 July 1998
*/
template <class T>
void knotApproximationClosed( Vector<T>& U, const  Vector<T>& ub, int n, int p){

  int i, j;  
  int iN = n - p ;
  U.resize(n+p+2) ;
  T d = ub.n()/(T)(n-p+1) ;
  T alpha;

  U = 0 ;
  // Initialize the internal knots
  for ( j=1; j<= n-p ; j++) {
    i          = int(j*d);
    alpha      = j*d - i;
    U[p+j]     = (1-alpha)*ub[i-1] + alpha*ub[i] ;
  }

  // Now make the left and right periodic extensions
  // Left 
  for (j=0; j<p; j++)       U[j] = U[j+iN+1] - 1;       
  // Right
  for (j=n+1; j<U.n(); j++)   U[j] = 1 + U[j-(iN+1)];
}

/*!
  \brief wraps d points to the end of a point vector

  Qw contains the same points that Q and wraps the end is 
  padded with the first d points from Q

  \input Q  a vector of 3D points 
  \input d  number of wrapped points
  \input Qw  a wrapped vector of 4D points
  \author    Alejandro Frangi
  \date 14 July, 1998
*/
template <class T, int N>
void wrapPointVector(const Vector<Point_nD<T,N> >& Q, int d, Vector<Point_nD<T,N> >& Qw){
  int i ;

  Qw = Q;
  Qw.resize(Q.n()+d);
  
  for (i=0; i<d; i++)
    Qw[i+Q.n()] = Q[i];

} 

/*!
  \brief wraps d points to the end of a point vector

  Qw contains the same points that Q and wraps the end is 
  padded with the first d points from Q

  \input Q  a vector of 3D points 
  \input d  number of wrapped points
  \input Qw  a wrapped vector of 4D points
  \author    Alejandro Frangi
  \date 14 July, 1998
*/
template <class T, int N>
void wrapPointVectorH(const Vector<HPoint_nD<T,N> >& Q, int d, Vector<HPoint_nD<T,N> >& Qw){
  int i ;

  Qw = Q;
  Qw.resize(Q.n()+d);
  
  for (i=0; i<d; i++)
    Qw[i+Q.n()] = Q[i];

} 

/*!
  \brief Writes the curve to in Display format as a LINE object

  This function writes a surface in LINE ascii format to
  interface with Display (Copyright 1993,1994,1995 David MacDonald,
  McConnell Brain Imaging Centre), Montreal Neurological Institute,
  McGill University.
  
  \param filename  the name of the file to save to
  \param iNu the number of straight segments in the line
  \param color the color of the line
  \param fA alpha blending parameter of the line

  \return returns 1 on success, 0 otherwise.

  \author Alejandro Frangi
  \date   21 January 1999
*/
template <class T, int N>
int NurbsCurve<T,N>::writeDisplayLINE(const char* filename, int iNu, const Color& color,T fA) const 
{

  int i;
  // COnvert the curve to 3D if it is not
  NurbsCurve<T,3> curve3D;
  to3D(*this,curve3D);
  // Output it
  T fDu = 1/T(iNu);
  ofstream fout(filename) ;
  if(!fout)
    return 0 ;
  // Save the object type
  const char LINE = 'l'+ ('A' - 'a');
  fout << LINE << ' ';       ;
  T fThickness = T(1.);
  // Save surface properties
  fout << fThickness << ' ' 
       << iNu << endl;
  // Fill points 
  Point_nD<T,3> p;
  for (T u = 0; u<1-fDu/2; u+=fDu, i++){
    p = T(-1)*curve3D.pointAt(u);
    fout << p.x() << ' ' << p.z() << ' ' << p.y() << endl;
  }
  // New line
  fout << endl ;
  // Elements
  fout << 1 << endl ;
  // New line
  fout << endl ;
  // Surface color RGBA (one color for the whole surface)
  T fR= T(color.r)/255;
  T fG= T(color.g)/255;
  T fB= T(color.b)/255;
  // Colour flag = ONE_COLOUR 
  fout << 0 << ' ' ;
  // The colour 
  fout << fR << ' '
       << fG << ' '
       << fB << ' '
       << fA << endl;
  // New line
  fout << endl ;
  fout << iNu << endl;
  // New line
  fout << endl ;
  // Save the dummy integers 
  for (i=0; i<iNu; i++)
    fout << i << ' ';
  fout << endl ;
  return 1;
}

/*!
  \brief Writes the curve to a file in Display format as a line object

  This function writes a surface in LINE ascii format to
  interface with Display (Copyright 1993,1994,1995 David MacDonald,
  McConnell Brain Imaging Centre), Montreal Neurological Institute,
  McGill University.
  
  \param filename  the name of the file to save to
  \param color the color of the curve
  \param Nu  the number of points along the path
  \param u_s  the starting parametric value for \a u
  \param u_e  the end parametric value for \a u

  \return returns 1 on success, 0 otherwise.

  \author Alejandro Frangi
  \date   15 Juanuary 1999
*/

template <class T, int N>
int NurbsCurve<T,N>::writeDisplayLINE(const char* filename,const Color& color, int iNu,T u_s, T u_e) const 
{

  int i;
  T fDu  = (u_e-u_s)/iNu;
  ofstream fout(filename) ;
  if(!fout)
    return 0 ;

  // Save the object type
  const char LINE = 'l'+ ('A' - 'a');
  fout << LINE << ' ';       ;

  float fThickness = 1;
  // Save surface properties
  fout << fThickness << ' ' 
       << iNu << endl;

  // Fill points 
  NurbsCurve<T,3> Curve;
  to3D(*this,Curve);
  Point_nD<T,3> p;
  T  u ;
  for ( u = u_s; u<u_e-fDu/2; u+=fDu, i++){
    // Requires sign inversion and swap of y-z
    p = -1.0*Curve.pointAt(u);
    fout << p.x() << ' ' << p.z() << ' ' << p.y() << endl;
  }

  // New line
  fout << endl ;
  // Elements
  fout << 1 << endl ;
  // New line
  fout << endl ;
  // Surface color RGBA (one color for the whole line)
  float fR=color.r/255.0;
  float fG=color.g/255.0;
  float fB=color.b/255.0;
  float fA=1;
  /* Colour flag = ONE_COLOUR */
  fout << 0 << ' ' ;
  /* The colour */
  fout << fR << ' '
       << fG << ' '
       << fB << ' '
       << fA << endl;
  
  // New line
  fout << endl ;

  fout << iNu << endl;

  // New line
  fout << endl ;

  // Save the dummy integers 
  for (i=0; i<iNu; i++)
    fout << i << ' ';
  fout << endl ;

  return 1;
}

/*!
  \brief set the tangent at a point
  
  \param u the parametric value of the point
  \param T the tangent value to set to
  
  \warning the tangent must be a unit length vector or an odd behavior 
           might occur

  \author Philippe Lavoie
  \date 2 March 1999
*/
template <class T, int N>
void NurbsCurve<T,N>::setTangent(T u, const Point_nD<T,N>& T0) {
  Point_nD<T,N> ders = derive3D(u,1) ;

  BasicArray<Point_nD<T,N> > D(2) ;
  BasicArray<int> dr(2) ;
  BasicArray<int> dk(2) ;
  BasicArray<T> ur(1) ;
 
  ur[0] = u ;
  dr[0] = 0 ; 
  dr[1] = 0 ; 
  dk[0] = 0 ;
  dk[1] = 1 ;
  D[0] = Point_nD<T,N>(0) ;

  T length = ders.norm() ;

  D[1] = T0 - ders/length ;
  D[1] *= length ;

  movePoint(ur,D,dr,dk) ;

}

/*!
  \brief set the tangent at the end points
  
  \param T0 the tangent at the begining of the curve
  \param T1 the tangent at the end of the curve
  
  \warning the tangent must be a unit length vector or and odd 
           behavior might occur

  \author Philippe Lavoie
  \date 2 March 1999
*/
template <class T, int N>
void NurbsCurve<T,N>::setTangentAtEnd(const Point_nD<T,N>& T0, const Point_nD<T,N>& T1) {
  Point_nD<T,N> ders0 = derive3D(U[deg_],1) ;
  Point_nD<T,N> ders1 = derive3D(U[P.n()],1) ;

  BasicArray<Point_nD<T,N> > D(4) ;
  BasicArray<int> dr(4) ;
  BasicArray<int> dk(4) ;
  BasicArray<T> ur(2) ;
 
  ur[0] = U[deg_] ;
  ur[1] = U[P.n()] ;
  D[0] = D[1] = Point_nD<T,N>(0) ;
  dr[0] = 0 ;
  dr[1] = 1 ;
  dr[2] = 0 ;
  dr[3] = 1 ;
  dk[0] = dk[1] = 0 ;
  dk[2] = dk[3] = 1 ;

  T length = ders0.norm() ;

  D[2] = T0 - ders0/length ;
  D[2] *= length ;

  length = ders1.norm();
  D[3] = T1 - ders1/length ;
  D[3] *= length ;

  movePoint(ur,D,dr,dk) ;

}


/*!
  \brief clamp a NURBS curve
  
  A clamped NURBS curve has degree+1 equal knots at both ends of the 
  knot vector.

  \author Philippe Lavoie
  \date 27 April 1999
*/
template <class T, int N>
void NurbsCurve<T,N>::clamp(){
  NurbsCurve<T,N> nc(*this) ;

  int n1 =  nc.knotInsertion(U[deg_],deg_,*this);
  int n2 =  knotInsertion(U[P.n()],deg_,nc);
  
  if(n1 || n2 ){
    U.resize(nc.U.n()-n1-n2) ;
    P.resize(U.n()-deg_-1) ;
    for(int i=U.n()-1;i>=0;--i){
      U[i] = nc.U[i+deg_] ;
      if(i<P.n())
	P[i] = nc.P[i+deg_] ; 
    }
  }

}

/*!
  \brief unclamp a NURBS curve
  
  An unclamped NURBS curve doesn't have any equal knots at both ends of
  the knot vector.

  \author Philippe Lavoie
  \date 27 April 1999
*/
template <class T, int N>
void NurbsCurve<T,N>::unclamp(){
  int n = P.n()-1 ;
  int i,j ;
  for(i=0;i<=deg_-2;++i){
    U[deg_-i-1] = U[deg_-i] - (U[n-i+1]-U[n-i]) ;
    int k=deg_-1 ;
    for(j=i;j>=0;--j){
      T alpha = (U[deg_]-U[k])/(U[deg_+j+1]-U[k]);
      P[j] = (P[j]-alpha*P[j+1])/(T(1)-alpha);
      --k ;
    }    
  }
  U[0] = U[1] - (U[n-deg_+2]-U[n-deg_+1]) ; // set first knot
  for(i=0;i<=deg_-2;++i){
    U[n+i+2] = U[n+i+1] + (U[deg_+i+1]-U[deg_+i]);
    for(j=i;j>=0;--j){
      T alpha = (U[n+1]-U[n-j])/(U[n-j+i+2]-U[n-j]);
      P[n-j] = (P[n-j]-(1.0-alpha)*P[n-j-1])/alpha ;
    }
  }
  U[n+deg_+1] = U[n+deg_] + (U[2*deg_]-U[2*deg_-1]); // set last knot
}

} // end namespace
