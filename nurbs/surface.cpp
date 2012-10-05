/*=============================================================================
        File: surface.cpp
     Purpose:       
    Revision: $Id: surface.cpp,v 1.2 2002/05/13 21:07:46 philosophil Exp $
  Created by: Philippe Lavoie          (3 Oct, 1996)
 Modified by: 

 Copyright notice:
          Copyright (C) 1996-1999 Philippe Lavoie
 
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
#include <surface.h>
#include "matrixMat.h"
#include <math.h>

/*!
 */
namespace PLib {

/*!
  \brief Find the minimal distance between a point and the surface.

  This is an iterative method to find the closest point to a 
  surface. 

  \param p  the minimal distance from that point
  \param guessU  a starting value for the parameter \a u, on exit this will 
	         be set to the value of the point on the surface closest 
		 to \a p.
  \param guessV  a starting value for the parameter \a v, on exit this 
	          will be set to the value of the point on the surface 
		  closest to \a p.
  \param error  when iterations have an error smaller than this value,
	          the function exits
  \param s  the size of the search in the parametric space.
  \param sep  the number of points initially looked at to find a minimal 
	          distance.
  \param maxiter  the maximal number of iterations
  \param um  the minimal parametric value for \a u
  \param uM  the maximal parametric value for \a u
  \param vm  the minimal parametric value for \a v
  \param vM  the maximal parametric value for \a v

  \return The value of the minimal distance between \a p and the surface. 
               The variables guessU and guessV now holds the parametric value 
	       of the surface point closest to \a p. 

  \warning It has not been tested with closed loop surfaces.

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T, int N>
T ParaSurface<T,N>::minDist2(const Point_nD<T,N>& p, T& guessU, T& guessV,T error,T s,int sep,int maxIter, T um, T uM, T vm, T vM) const {
  T d,d1,d2 ;
  Point_nD<T,N> p2 ;
  p2 = pointAt(guessU,guessV) ;
  d = norm2(p-p2) ;
  d2 = d1 = 0 ;
  int niter = 0 ;
  T u1,u2 ;
  T v1,v2 ;
  T step ;
  step = 2.0*s/(T)sep ;
  u1 = guessU-s ;
  u2 = guessU+s ;
  v1 = guessV-s ;
  v2 = guessV+s ;
  while(d>error && niter<maxIter) {
    if(u1<um)
      u1=um;
    if(u2>uM)
      u2 = uM ;
    if(v1<vm)
      v1 = vm ;
    if(v2>vM)
      v2 = vM ;
    T u,v ;
    d2 = d1 ;
    for(u=u1;u<u2;u+=step)
      for(v=v1;v<v2;v+=step){
	p2 = pointAt(u,v) ;
	d1 = norm2(p-p2) ;
	if(d1<d){
	  d = d1 ;
	  guessU = u ;
	  guessV = v ;
	}
      }
    s /= 2.0 ;
    u1 = guessU - s ;
    u2 = guessU + s ;
    v1 = guessV - s ;
    v2 = guessV + s ;
    step = 2.0*s/(T)sep ;
    if(d-d2==0) niter = maxIter ;
    if(step<error) niter = maxIter ;
    ++niter;
  }
  return d ;
}

/*!
  \brief Find the minimal distance between a point and the surface.

  This is an iterative method to find the closest point to a 
  surface. The method is slightly different than minDist2.

  \param p  the minimal distance from that point
  \param guessU  a starting value for the parameter \a u, on exit this
	          will be set to the value of the point on the surface 
		  closest to \a p.
  \param guessV  a starting value for the parameter \a v, on exit this 
	          will be set to the value of the point on the surface 
		  closest to \a p.
  \param error  when iterations have an error smaller than this value, 
	          the function exits
  \param s  the size of the search in the parametric space.
  \param sep  the number of points initially looked at to find a 
	          minimal distance.
  \param maxiter  the maximal number of iterations
  \param um  the minimal parametric value for \a u
  \param uM  the maximal parametric value for \a u
  \param vm  the minimal parametric value for \a v
  \param vM  the maximal parametric value for \a v

  \return The value of the minimal distance between \a p and the surface. 
               The variables guessU and guessV now holds the parametric value 
	       of the surface point closest to \a p. 

  \warning It has not been tested with closed loop surfaces.

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T, int N>
T ParaSurface<T,N>::minDist2b(const Point_nD<T,N>& p, T& guessU, T& guessV,T error,T s,int sep,int maxIter, T um, T uM, T vm, T vM) const {
  T d,d1,d2 ;
  Point_nD<T,N> p2 ;
  p2 = pointAt(guessU,guessV) ;
  d = norm2(p-p2) ;
  d2 = d1 = 0 ;
  int niter = 0 ;
  T u1,u2 ;
  T v1,v2 ;
  T step ;
  step = 2.0*s/(T)sep ;
  u1 = guessU-s ;
  u2 = guessU+s ;
  v1 = guessV-s ;
  v2 = guessV+s ;
  while(d>error && niter<maxIter) {
    if(u1<um)
      u1=um;
    if(u2>uM)
      u2 = uM ;
    if(v1<vm)
      v1 = vm ;
    if(v2>vM)
      v2 = vM ;
    T u,v ;
    d2 = d1 ;
    for(u=u1;u<=u2;u+=step)
      for(v=v1;v<=v2;v+=step){
	p2 = pointAt(u,v) ;
	d1 = norm2(p-p2) ;
	if(d1<d){
	  d = d1 ;
	  guessU = u ;
	  guessV = v ;
	}
      }
    s = step ;
    u1 = guessU - s ;
    u2 = guessU + s ;
    v1 = guessV - s ;
    v2 = guessV + s ;
    step = s/2.0 ; // *s/(T)sep ;
    if(d-d2==0) niter = maxIter ;
    if(step<error) niter = maxIter ;
    ++niter;
  }
  return d ;
  
}

template <class T, int N>
inline T to2power_xy(const Point_nD<T,N> &p){
  return (p.x()*p.x())+(p.y()*p.y()) ;
}

/*!
  \brief Find the minimal distance between a point and 
                               the surface in the x-y plane

  This is an iterative method to find the closest point to a 
  surface. The distance is search in the x-y plane. The z 
  component is \e not taken into account for the search.

  \param p  the minimal distance from that point
  \param guessU  a starting value for the parameter \a u, on exit this 
	          will be set to the value
		  of the point on the surface closest to \a p.
  \param guessV  a starting value for the parameter \a v, on exit this
	          will be set to the value
		  of the point on the surface closest to \a p.
  \param error  when iterations have an error smaller than this value, 
	     the function exits
  \param dU  if a parametric delta is smaller than this value, 
	     the function stops.
  \param s  the size of the search in the parametric space.
  \param sepU  the number of points initially looked at to find a 
	       minimal distance in the \a u direction
  \param sepV  the number of points initially looked at to find a 
               minimal distance in the \a v direction
  \param maxiter  the maximal number of iterations
  \param um  the minimal parametric value for \a u
  \param uM  the maximal parametric value for \a u
  \param vm  the minimal parametric value for \a v
  \param vM  the maximal parametric value for \a v

  \return The value of the minimal distance between \a p and the surface. 
               The variables guessU and guessV now holds the parametric 
	       value of the surface point closest to \a p. 

  \warning It has not been tested with closed loop surfaces.

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T, int N>
T ParaSurface<T,N>::minDist2xy(const Point_nD<T,N>& p, T& guessU, T& guessV,T error,T dU,T s,int sepU,int sepV,int maxIter, T um, T uM, T vm, T vM) const {
  T d,d1,d2,dz ;
  Point_nD<T,N> p2 ;
  p2 = pointAt(guessU,guessV) ;
  d = to2power_xy(p-p2) ;
  dz = to2power(p.z()-p2.z()) ;
  d2 = d1 = 0 ;
  int niter = 0 ;
  T u1,u2 ;
  T v1,v2 ;
  T stepU,stepV ;
  if(sepU>0){
    stepU = 2.0*s/(T)sepU ;
    u1 = guessU-s ;
    u2 = guessU+s ;
  }
  else{
    stepU = s ;
    u1 = guessU ;
    u2 = guessU ;
  }
  if(sepV>0){
    stepV = 2.0*s/(T)sepV ;
    v1 = guessV-s ;
    v2 = guessV+s ;
  }
  else{
    stepV = s ;
    v1 = guessV ;
    v2 = guessV ;
  }
  while(d>error && niter<maxIter) {
    if(u1<um)
      u1=um;
    if(u2>uM)
      u2 = uM ;
    if(v1<vm)
      v1 = vm ;
    if(v2>vM)
      v2 = vM ;
    T u,v ;
    d2 = d1 ;
    for(u=u1;u<=u2;u+=stepU)
      for(v=v1;v<=v2;v+=stepV){
	p2 = pointAt(u,v) ;
	d1 = to2power_xy(p-p2) ;
	if(d1<d){
	  d = d1 ;
	  dz = to2power(p.z()-p2.z()) ;
	  guessU = u ;
	  guessV = v ;
	}
      }
    if(d-d2==0) niter = maxIter ;
    if(stepU<dU) niter = maxIter ;
    if(stepV<dU) niter = maxIter ;
    s = stepU*0.55 ;
    u1 = guessU - s ;
    u2 = guessU + s ;
    s = stepV*0.55 ;
    v1 = guessV - s ;
    v2 = guessV + s ;
    stepU *= 0.5 ;
    stepV *= 0.5 ;
    ++niter;
  }
  return dz ;
  
}

/*!
  \brief Write the NURBS surface to a VRML file

  Writes a VRML file which represents the surface for the 
  parametric space \a [uS,uE] and \a [vS,vE].
  It does not optimize the number of points required to 
  represent the surface. 

  \param filename  the file name for the output VRML file
  \param Nu  the number of points in the \a u direction
  \param Nv  the number of points in the \a v direction
  \param uS  the starting value of \a u
  \param uE  the end value of \a u
  \param vS  the starting value of \a v
  \param vE  the end value of \a v

  \return 1 on success, 0 otherwise

  \warning The parametric surface must be valid

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
int ParaSurface<T,N>::writeVRML(const char* filename,const Color& color,int Nu,int Nv, T uS,T uE,T vS, T vE) const {
  ofstream fout(filename) ;

  if(!fout)
    return 0 ;
  return writeVRML(fout,color,Nu,Nv,uS,uE,vS,vE) ;
}


/*!
  \brief Write the NURBS surface to a VRML file

  Writes a VRML file which represents the surface for the 
  parametric space \a [uS,uE] and \a [vS,vE].
  It does not optimize the number of points required to 
  represent the surface. 

  \param filename  the file name for the output VRML file
  \param Nu  the number of points in the \a u direction
  \param Nv  the number of points in the \a v direction
  \param uS  the starting value of \a u
  \param uE  the end value of \a u
  \param vS  the starting value of \a v
  \param vE  the end value of \a v

  \return 1 on success, 0 otherwise

  \warning The parametric surface must be valid

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
int ParaSurface<T,N>::writeVRML(ostream &fout,const Color& color,int Nu,int Nv, T uS,T uE,T vS, T vE) const {

  fout << "#VRML V1.0 ascii\n" ;
  fout << "#  Generated by Phil's NURBS library\n" ;
  fout << "\nSeparator {\n" << "\tMaterialBinding { value PER_VERTEX_INDEXED }\n"  ;
    fout << "\tMaterial{\n\t\tambientColor 0.25 0.25 0.25\n\t\tdiffuseColor " << float(color.r/255.0) << ' ' << float(color.g/255.0) << ' ' << float(color.b/255.0) << "\n\t}\n" ;
  fout << "\tCoordinate3 {\n" ;
  fout << "\t\tpoint [\n" ;

  T u,v,du,dv ;

  if(Nu<=1)
    Nu = 2 ;  // Should I put a warning message ?
  if(Nv<=1)
    Nv = 2 ;  // Should I put a warning message ?

  u = uS ;
  v = vS ;
  du = (uE-uS)/(T)(Nu-1) ;
  dv = (vE-vS)/(T)(Nv-1) ;

  int i,j ;


  for(i=0;i<Nu;++i){
    v = vS ;
    for(j=0 ;j<Nv;++j){
      Point_nD<T,N> p ;
      p = pointAt(u,v) ;
      fout << "\t\t\t" << p.x() << ' ' << p.y() << ' ' << p.z() << ",\n" ;
      v += dv ;
    }
    u += du ;
  }
  
  fout << "\t\t]\n" ; // point [
  fout << "\t}\n" ; // cordinate3

  fout << "\tIndexedFaceSet{\n" ;
  fout << "\t\tcoordIndex[\n" ;
  
  for(i=0;i<Nu-1;++i){
    for(j=0;j<Nv-1;++j) {
      fout << "\t\t\t" << i*Nv+j << ", " << i*Nv+j+1 << ", " << (i+1)*Nv+j << ", -1,\n";
      fout << "\t\t\t" << i*Nv+j+1 << ", " << (i+1)*Nv+j+1<< ", " << (i+1)*Nv+j  << ", -1,\n";
    }
  }
  fout << "\t\t]\n" ;
  fout << "\t}\n" ; // IndexedFaceSet

  fout << "}\n" ;

  return 1 ;
}

template <class T>
inline T compare(int findMin, T a, T b){
  if(findMin)
    return minimum(a,b) ;
  return maximum(a,b) ;
}

// VCC includes curve.cpp which defines this function already
#ifndef INCLUDE_TEMPLATE_SOURCE
template <class T,int N>
inline T coordValue(CoordinateType coord, const Point_nD<T,N>& p){
  switch(coord){
  case coordX: return p.x() ; break ;
  case coordY: return p.y() ; break ;
  case coordZ: return p.z() ; break ;
  }
  return 0.0 ; // elliminates warning messages
}
#endif

/*!
  \brief Finds the minimal or maximal value on the curve
                            of the x,y or z coordinate.

  \param findMin  a flag indicatinf if we're looking for the minimal
	                value or the maximal value.
  \param coord  Which coordinate to find: x,y or z.
  \param minDu  The minimal distance between iterations in the parametric
	        space.
  \param sepU  the number of points initially looked at to find a minimal
	       distance in the U direction
  \param sepV  the number of points initially looked at to find a minimal
	       distance in the U direction
  \param maxiter  the maximal number of iterations
  \param um  the minimal parametric value for \a u
  \param uM  the maximal parametric value for \a u
  \param vm  the minimal parametric value for \a v
  \param  vM  the maximal parametric value for \a v

  \return The minimal value of $z$ along the curve

  \warning It has not been tested with closed loop curves.

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T, int N>
T ParaSurface<T,N>::extremum(int findMin, CoordinateType coord, T minDu, int sepU, int sepV, int maxIter, T um, T uM, T vm, T vM) const {
  T c,du,dv,d1,d2,result,guessU,guessV ;
  T minDv = minDu ;
  Point_nD<T,N> p2 ;

  guessU = 0 ;
  guessV = 0 ;

  // check for corner values...
  // because of the nature of += these value might be missed 
  // in the loop
  p2 = pointAt(um,vm) ;
  c = coordValue(coord,p2) ;
  p2 = pointAt(um,vM) ;
  c = compare(findMin,c,coordValue(coord,p2)) ;
  p2 = pointAt(uM,vm) ;
  c = compare(findMin,c,coordValue(coord,p2)) ;
  p2 = pointAt(uM,vM) ;
  c = compare(findMin,c,coordValue(coord,p2)) ; 
  result = c ;
  du = minDu*10.0 ;
  dv = minDv*10.0 ;
  d2 = d1 = 0 ;
  int niter = 0 ;
  T u1,u2 ;
  T v1,v2 ;
  T stepU,stepV ;
  T s ;
  s = uM - um ;
  stepU = s/(T)(sepU+1) ;
  stepV = s/(T)(sepV+1) ;
  u1 = um ;
  u2 = uM ;
  v1 = vm ;
  v2 = vM ;
  while((du>minDu || dv>minDv) && niter<maxIter) {
    if(u1<um)
      u1=um;
    if(u2>uM)
      u2 = uM ;
    if(v1<vm)
      v1 = vm ;
    if(v2>vM)
      v2 = vM ;
    T u,v ;
    d2 = c ;
    du = guessU ;
    dv = guessV ;
    for(u=u1;u<=u2;u+=stepU)
      for(v=v1;v<=v2;v+=stepV){
	p2 = pointAt(u,v) ;
	if(findMin){
	  d1 = minimum(c,coordValue(coord,p2)) ;
	  if(d1<c){
	    c = d1 ;
	    guessU = u ;
	    guessV = v ;
	    result = d1 ;
	  }
	}
	else{
	  d1 = maximum(c,coordValue(coord,p2)) ;
	  if(d1>c){
	    c = d1 ;
	  guessU = u ;
	  guessV = v ;
	  result = d1 ;
	  }
	}
      }
    s /= 2.0 ;
    u1 = guessU - s ;
    u2 = guessU + s ;
    v1 = guessV - s ;
    v2 = guessV + s ;
    stepU = 2.0*s/(T)sepU ;
    stepV = 2.0*s/(T)sepV ;
    if((c-d2)==0.0) niter = maxIter ;
    if(stepU<minDu) niter = maxIter ;
    if(stepV<minDv) niter = maxIter ;
    du = absolute(guessU-du) ;
    dv = absolute(guessV-dv) ;
    niter++;
  }
  return result ;
}

/*!
  \brief projects a point onto the surface

  Projects a point using Newton-Raphson's method. 
  \latexonly
  We want to solve for
	       \[ X'_u(u_i,v_i)\cdot \delta u + X'_v(u_i,v_i)\cdot \delta v
	         = P - X(u_i,v_i) \]
	       This is an over-determined system and the least square 
	       approximation is taken:
	       \[ \left[ \begin{array}{cc} 
	       X'_u(u_i,v_i) \cdot X'_u(u_i,v_i) & 
	       X'_u(u_i,v_i) \cdot X'_v(u_i,v_i) \\
	       X'_v(u_i,v_i) \cdot X'_u(u_i,v_i) & 
	       X'_v(u_i,v_i) \cdot X'_v(u_i,v_i) \end{array} \right]
	       \left[ \begin{array}{c} \delta u \\ \delta v \end{array}
	       \right]
	        = 
		\left[ \begin{array}{c}
		(P-X(u_i,v_i)) \cdot X'_u(u_i,v_i) \\
		(P-X(u_i,v_i)) \cdot X'_v(u_i,v_i) 
		\end{array} \right]
		\]

		If $||X'_u|| = 0$ and $||X'_v|| = 0$, then the equation is
		singular. In that case the routine returns with 0. 
		Use one of the other iterative method to find a suitable 
		projection, if this happens.
  \endlatexonly
  \htmlonly
   There is more information available in the LaTeX version
  \endhtmlonly

  \param P  the point to project
  \param u  the u parametric value of the result
  \param v  the v parametric value of the result
  \param maxI  the maximal number of iterations
  \param um  the minimal parametric value for \a u
  \param uM  the maximal parametric value for \a u
  \param vm  the minimal parametric value for \a v
  \param vM  the maximal parametric value for \a v

  \return 1 on sucess, 0 if their was a singularity in computation

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T, int N>
int ParaSurface<T,N>::projectOn(const Point_nD<T,N>& p, T& u, T& v, int maxI, const T um, const T uM, const T vm, const T vM) const {
  int i = 0 ;
  Point_nD<T,N> xu,xv,x,t ;
  Matrix< Point_nD<T,N> >ders ;
  Matrix_DOUBLE A(6,2) ;
  Matrix_DOUBLE B(6,1) ;
  Matrix_DOUBLE X(2,1) ;



  while(i < maxI){
    // make sure u and v are in bounds
    boundTo(u,um,uM);
    boundTo(v,vm,vM);

    // compute the over determined solution to the equation
    deriveAt(u,v,1,ders) ;
    xu = ders(1,0) ;
    xv = ders(0,1) ;
    x = ders(0,0) ;
    
    t = xu*xu ;
    A(0,0) = t.x() ;
    A(1,0) = t.y() ;
    A(2,0) = t.z() ;

    t = xu*xv ;
    A(0,1) = t.x() ;
    A(1,1) = t.y() ;
    A(2,1) = t.z() ;

    t = xv*xu ;
    A(3,0) = t.x() ;
    A(4,0) = t.y() ;
    A(5,0) = t.z() ;

    t = xv*xv ;
    A(3,1) = t.x() ;
    A(4,1) = t.y() ;
    A(5,1) = t.z() ;

    t = (p-x)*xu ;
    B(0,0) = t.x() ;
    B(1,0) = t.y() ;
    B(2,0) = t.z() ;

    t = (p-x)*xv ;
    B(3,0) = t.x() ;
    B(4,0) = t.y() ;
    B(5,0) = t.z() ;
    

    SVDMatrix<double> svd(A) ;
    if(!svd.solve(B,X))  // the matrix is singular
      return 0 ;

    // X now contains the du and dv ;

    if(T(X(0,0)) == T(0) && (T)X(1,0) == T(0)){
      // we are hopefully done
      return 1 ;
    }
    
    u += (T)X(0,0) ;
    v += (T)X(1,0) ;
    ++i ;
  }

  return 0 ;
}

/*!
  \brief Finds the intersection of two surfaces near a point. 
	
  The method used is similar to the one used to project a point on a surface.
  It's a modified Newton-Raphson's method.
	
  \param S  the surface to intersect with
  \param P  the point for the intersection
  \param u  the u parametric value of the intersection
  \param v  the v parametric value of the intersection
  \param s  the u parametric value of the intersection for S
  \param t  the v parametric value of the intersection for S
  \param maxI  the maximal number of iterations
  \param um  the minimal parametric value for \a u
  \param uM  the maximal parametric value for \a u
  \param vm  the minimal parametric value for \a v
  \param vM  the maximal parametric value for \a v

  \return 1 on sucess, 0 if their was a singularity in computation

  \author Philippe Lavoie
  \date 6 July 1998
*/
template <class T, int N>
int ParaSurface<T,N>::intersectWith(const ParaSurface<T,N> &S, Point_nD<T,N>& p, T& u, T& v, T& s, T& t, int maxI, T um, T uM, T vm, T vM) const{
  Point_nD<T,N> xu,xv,x1,x2,tmp,n1,n2,n12 ;

  Matrix< Point_nD<T,N> > ders ; 
  Matrix_DOUBLE A(6,2) ; 
  Matrix_DOUBLE B(6,1) ; 
  Matrix_DOUBLE X(2,1) ; 
  Matrix_DOUBLE A2(3,3) ;
  Matrix_DOUBLE B2(3,1) ; 
  Matrix_DOUBLE X2(3,1) ; 

  int done = 0 ; 
  int i = 0 ; 
  while(i<maxI){
    // make sure u and v are in bounds
    boundTo(u,um,uM) ; 
    boundTo(v,vm,vM) ; 
    boundTo(s,um,uM) ; 
    boundTo(t,vm,vM) ; 

    deriveAt(u,v,1,ders) ; 
    xu = ders(1,0) ; 
    xv = ders(0,1) ; 
    x1 = ders(0,0) ; 
    n1 = crossProduct(xu,xv) ; 
    
    tmp = xu*xu ; 
    A(0,0) = tmp.x() ;
    A(1,0) = tmp.y() ;
    A(2,0) = tmp.z() ;

    tmp = xu*xv ;
    A(0,1) = tmp.x() ;
    A(1,1) = tmp.y() ;
    A(2,1) = tmp.z() ;

    tmp = xv*xu ;
    A(3,0) = tmp.x() ;
    A(4,0) = tmp.y() ;
    A(5,0) = tmp.z() ;

    tmp = xv*xv ;
    A(3,1) = tmp.x() ;
    A(4,1) = tmp.y() ;
    A(5,1) = tmp.z() ;

    tmp = (p-x1)*xu ;
    B(0,0) = tmp.x() ;
    B(1,0) = tmp.y() ;
    B(2,0) = tmp.z() ;

    tmp = (p-x1)*xv ;
    B(3,0) = tmp.x() ;
    B(4,0) = tmp.y() ;
    B(5,0) = tmp.z() ;
    

    SVDMatrix<double> svd(A) ;
    if(!svd.solve(B,X))  // the matrix is singular
      return 0 ;

    // X now contains the du and dv ;

    if(T(X(0,0)) == T(0) && (T)X(1,0) == T(0)){
      done = 1 ; // we are hopefully done
      //return 1 ;
    }
    
    u += (T)X(0,0) ;
    v += (T)X(1,0) ;

    // now compute the ds and dt for the other surface

    S.deriveAt(s,t,1,ders) ; 
    xu = ders(1,0) ; 
    xv = ders(0,1) ; 
    x2 = ders(0,0) ; 
    n2 = crossProduct(xu,xv) ; 
    
    tmp = xu*xu ; 
    A(0,0) = tmp.x() ;
    A(1,0) = tmp.y() ;
    A(2,0) = tmp.z() ;

    tmp = xu*xv ;
    A(0,1) = tmp.x() ;
    A(1,1) = tmp.y() ;
    A(2,1) = tmp.z() ;

    tmp = xv*xu ;
    A(3,0) = tmp.x() ;
    A(4,0) = tmp.y() ;
    A(5,0) = tmp.z() ;

    tmp = xv*xv ;
    A(3,1) = tmp.x() ;
    A(4,1) = tmp.y() ;
    A(5,1) = tmp.z() ;

    tmp = (p-x2)*xu ;
    B(0,0) = tmp.x() ;
    B(1,0) = tmp.y() ;
    B(2,0) = tmp.z() ;

    tmp = (p-x2)*xv ;
    B(3,0) = tmp.x() ;
    B(4,0) = tmp.y() ;
    B(5,0) = tmp.z() ;
    

    svd.decompose(A) ;
    if(!svd.solve(B,X))  // the matrix is singular
      return 0 ;

    // X now contains the ds and dt ;

    if(T(X(0,0)) == T(0) && (T)X(1,0) == T(0)){
      if(done) {// we are hopefully done
	cerr << i << endl ; 
	return 1 ;
      }
      done = 0 ; 
    }
    
    s += (T)X(0,0) ;
    t += (T)X(1,0) ;

    // Finally we need to refine the point p 
    // this is one step behind refinement of F(u,v) and G(s,t)

    if(i>0){
      A2(0,0) = n1.x() ; 
      A2(0,1) = n1.y() ; 
      A2(0,2) = n1.z() ; 
      A2(1,0) = n2.x() ; 
      A2(1,1) = n2.y() ; 
      A2(1,2) = n2.z() ; 
      n12 = crossProduct(n1,n2) ; 
      A2(2,0) = n12.x() ; 
      A2(2,1) = n12.y() ; 
      A2(2,2) = n12.z() ; 
      
      B2(0,0) = x1*n1 ; 
      B2(1,0) = x2*n2 ; 
      const T alpha = 0.3 ;
      const T beta = 0.3 ; 
      const T lambda = 0.4 ; 
      B2(2,0) = (alpha*x1 + beta *x2 + lambda * p)*n12 ; 

      SVDMatrix<double> svd3(A2) ;
      if(!svd3.solve(B2,X2))  // the matrix is singular
	return 0 ;
      
      // X2 now contains the p2 ;
      p.x() = X2(0,0) ; 
      p.y() = X2(1,0) ; 
      p.z() = X2(2,0) ; 
    }

    ++i ;    
  }
  
  return 1 ; 
}


/*!
  \brief Finds the intersection of two surfaces near a point. 
		
  \param S  the surface to intersect with
  \param iter  the iteration point
  \param maxI  the maximal number of iterations
  \param um  the minimal parametric value for \a u
  \param uM  the maximal parametric value for \a u
  \param vm  the minimal parametric value for \a v
  \param vM  the maximal parametric value for \a v

  \return 1 on sucess, 0 if their was a singularity in computation

  \author Philippe Lavoie
  \date 6 July 1998
*/
template <class T, int N>
int ParaSurface<T,N>::intersectWith(const ParaSurface<T,N> &S, InterPoint<T,N> &iter, int maxI, T um, T uM, T vm, T vM) const{
  Point_nD<T,N> xu,xv,x1,x2,tmp,n1,n2,n12 ;

  Matrix< Point_nD<T,N> > ders ; 
  Matrix_DOUBLE A(6,2) ; 
  Matrix_DOUBLE B(6,1) ; 
  Matrix_DOUBLE X(2,1) ; 
  Matrix_DOUBLE A2(3,3) ;
  Matrix_DOUBLE B2(3,1) ; 
  Matrix_DOUBLE X2(3,1) ; 

  int done = 0 ; 
  int i = 0 ; 
  
  Point_nD<T,N> &p = iter.point ; 
  T &u = iter.paramA.u ; 
  T &v = iter.paramA.v ; 
  T &s = iter.paramB.u ; 
  T &t = iter.paramB.v ; 

  while(i<maxI){
    // make sure u and v are in bounds
    boundTo(u,um,uM) ; 
    boundTo(v,vm,vM) ; 
    boundTo(s,um,uM) ; 
    boundTo(t,vm,vM) ; 

    deriveAt(u,v,1,ders) ; 
    xu = ders(1,0) ; 
    xv = ders(0,1) ; 
    x1 = ders(0,0) ; 
    n1 = crossProduct(xu,xv) ; 
    
    tmp = xu*xu ; 
    A(0,0) = tmp.x() ;
    A(1,0) = tmp.y() ;
    A(2,0) = tmp.z() ;

    tmp = xu*xv ;
    A(0,1) = tmp.x() ;
    A(1,1) = tmp.y() ;
    A(2,1) = tmp.z() ;

    tmp = xv*xu ;
    A(3,0) = tmp.x() ;
    A(4,0) = tmp.y() ;
    A(5,0) = tmp.z() ;

    tmp = xv*xv ;
    A(3,1) = tmp.x() ;
    A(4,1) = tmp.y() ;
    A(5,1) = tmp.z() ;

    tmp = (p-x1)*xu ;
    B(0,0) = tmp.x() ;
    B(1,0) = tmp.y() ;
    B(2,0) = tmp.z() ;

    tmp = (p-x1)*xv ;
    B(3,0) = tmp.x() ;
    B(4,0) = tmp.y() ;
    B(5,0) = tmp.z() ;
    

    SVDMatrix<double> svd(A) ;
    if(!svd.solve(B,X)){  // the matrix is singular
      iter.tangent = crossProduct(n1,n2).unitLength() ; 
      return 0 ;
    }

    // X now contains the du and dv ;

    if(T(X(0,0)) == T(0) && (T)X(1,0) == T(0)){
      done = 1 ; // we are hopefully done
      //return 1 ;
    }
    
    u += (T)X(0,0) ;
    v += (T)X(1,0) ;

    // now compute the ds and dt for the other surface

    S.deriveAt(s,t,1,ders) ; 
    xu = ders(1,0) ; 
    xv = ders(0,1) ; 
    x2 = ders(0,0) ; 
    n2 = crossProduct(xu,xv) ; 
    
    tmp = xu*xu ; 
    A(0,0) = tmp.x() ;
    A(1,0) = tmp.y() ;
    A(2,0) = tmp.z() ;

    tmp = xu*xv ;
    A(0,1) = tmp.x() ;
    A(1,1) = tmp.y() ;
    A(2,1) = tmp.z() ;

    tmp = xv*xu ;
    A(3,0) = tmp.x() ;
    A(4,0) = tmp.y() ;
    A(5,0) = tmp.z() ;

    tmp = xv*xv ;
    A(3,1) = tmp.x() ;
    A(4,1) = tmp.y() ;
    A(5,1) = tmp.z() ;

    tmp = (p-x2)*xu ;
    B(0,0) = tmp.x() ;
    B(1,0) = tmp.y() ;
    B(2,0) = tmp.z() ;

    tmp = (p-x2)*xv ;
    B(3,0) = tmp.x() ;
    B(4,0) = tmp.y() ;
    B(5,0) = tmp.z() ;
    

    svd.decompose(A) ;
    if(!svd.solve(B,X)){  // the matrix is singular
      iter.tangent = crossProduct(n1,n2).unitLength() ; 
      return 0 ;
    }

    // X now contains the ds and dt ;

    if(T(X(0,0)) == T(0) && (T)X(1,0) == T(0)){
      if(done) {// we are hopefully done
	iter.tangent = crossProduct(n1,n2).unitLength() ; 
	return 1 ;
      }
      done = 0 ; 
    }
    
    s += (T)X(0,0) ;
    t += (T)X(1,0) ;

    // Finally we need to refine the point p 
    // this is one step behind refinement of F(u,v) and G(s,t)

    if(i>0){
      A2(0,0) = n1.x() ; 
      A2(0,1) = n1.y() ; 
      A2(0,2) = n1.z() ; 
      A2(1,0) = n2.x() ; 
      A2(1,1) = n2.y() ; 
      A2(1,2) = n2.z() ; 
      n12 = crossProduct(n1,n2) ; 
      A2(2,0) = n12.x() ; 
      A2(2,1) = n12.y() ; 
      A2(2,2) = n12.z() ; 
      
      B2(0,0) = x1*n1 ; 
      B2(1,0) = x2*n2 ; 
      const T alpha = 0.3 ;
      const T beta = 0.3 ; 
      const T lambda = 0.4 ; 
      B2(2,0) = (alpha*x1 + beta *x2 + lambda * p)*n12 ; 

      SVDMatrix<double> svd3(A2) ;
      if(!svd3.solve(B2,X2)){  // the matrix is singular
	iter.tangent = crossProduct(n1,n2).unitLength() ; 
	return 0 ;
      }
      
      // X2 now contains the p2 ;
      p.x() = X2(0,0) ; 
      p.y() = X2(1,0) ; 
      p.z() = X2(2,0) ; 
    }

    ++i ;    
  }
  
  iter.tangent = crossProduct(n1,n2).unitLength() ; 
  return 1 ; 
}

template <class T, int N>
inline int isNear(const SurfParam<T,N> &a, const SurfParam<T,N>& b, double tol=1e-5){
  double d2 = ((double)a.u-(double)b.u)*((double)a.u-(double)b.u);
  d2 += ((double)a.v-(double)b.v)*((double)a.v-(double)b.v) ;
  if(d2<tol*tol)
    return 1 ;
  return 0 ; 
}

template <class T, int N>
inline int isNear(const InterPoint<T,N>& a, const InterPoint<T,N> &b){
  return isNear(a.paramA,b.paramA) || isNear(a.paramB,b.paramB) ; 
}

template <class T, int N>
inline int onBoundary(const SurfParam<T,N>& a, T m = 0, T M=1){
  if(a.u<=m)
    return 1 ;
  if(a.u>=M)
    return 1 ; 
  return 0 ; 
}

template <class T, int N>
void intersectSurfaces(const ParaSurface<T,N> &surfA, const ParaSurface<T,N> &surfB, BasicList<InterPoint<T,N> > &points, int maxI, T um, T uM, T vm, T vM){
  points.reset() ;

  Point_nD<T,N> p ;

  p = surfA.pointAt(0.5,0.5) ;

  InterPoint<T,N> I0,I,Ilast ;

  I0.point = p ;

  surfA.intersectWith(surfB,I0,maxI,um,uM,vm,vM) ;

  I = Ilast = I0 ; 

  T d ;

  d = 0.01 ; // should be dependant on the control points locations
  T direction = 1 ; 

  int closed = 0 ; 
  int reach_bound = 0 ;

  const T error  = 0.1; 
  const T up_bound = 1.5 ;

  int n = 0 ; 

  while(1){
    points.add(I) ; 
    I.point += direction*d*I.tangent ; 
    surfA.intersectWith(surfB,I,maxI,um,uM,vm,vM) ; 
    if(isNear(I0,I)){
      closed = 1 ; 
      break ; 
    }
    if(onBoundary(I.paramA) || onBoundary(I.paramB)){
      reach_bound =1 ;
      break ; 
    }
    d = norm(Ilast.point-I.point) ; 
    d *= error/acos(I.tangent*Ilast.tangent/up_bound) ;
    if(d<0.01) // putting a lower boundary
      d = 0.01 ; 
    if(d>100)
      break ; 
    Ilast = I ; 
    cout << I.point << "\t" << I.paramA.u << "," << I.paramA.v << ":" << I.paramB.u << "," << I.paramB.v << "\t" << I.tangent << "|" << d << endl ; 
    ++n ; 
    if(n>100) 
      break ; 
  }
  if(reach_bound){
    // repeat but in the other direction
    direction *= -1 ;
    I = Ilast = I0 ; 
    n = 0 ; 
    while(1){
      points.add(I) ; 
      I.point += direction*d*I.tangent ; 
      surfA.intersectWith(surfB,I,maxI,um,uM,vm,vM) ; 
      if(isNear(I0,I)){
	closed = 1 ; 
	break ; 
      }
      if(onBoundary(I.paramA) || onBoundary(I.paramB)){
	reach_bound =1 ;
	break ; 
      }
      d = norm(Ilast.point-I.point) ; 
      d *= error/acos(I.tangent*Ilast.tangent/up_bound) ;
      if(d<0.01) // putting a lower boundary
	d = 0.01 ; 
      if(d>100)
	break ; 
      Ilast = I ; 
      cout << I.point << "\t" << I.paramA.u << "," << I.paramA.v << ":" << I.paramB.u << "," << I.paramB.v << "\t" << I.tangent << "|" << d << endl ; 
      ++n ; 
      if(n>100) 
	break ; 
    }    
  }
}

/*!
  \brief Write the NURBS surface to a VRML file

  Writes a VRML file which represents the surface for the 
  parametric space \a [uS,uE] and \a [vS,vE].
  It does not optimize the number of points required to 
  represent the surface. 

  \param filename  the file name for the output VRML file
  \param Nu  the number of points in the \a u direction
  \param Nv  the number of points in the \a v direction
  \param uS  the starting value of \a u
  \param uE  the end value of \a u
  \param vS  the starting value of \a v
  \param vE  the end value of \a v

  \return 1 on success, 0 otherwise

  \warning The parametric surface must be valid

  \author Philippe Lavoie
  \date 30 April, 1999
*/
template <class T, int N>
int ParaSurface<T,N>::writeVRML97(const char* filename,const Color& color,int Nu,int Nv, T uS,T uE,T vS, T vE) const {
  ofstream fout(filename) ;

  if(!fout)
    return 0 ;
  return writeVRML97(fout,color,Nu,Nv,uS,uE,vS,vE) ;
}


/*!
  \brief Write the NURBS surface to a VRML97 file

  Writes a VRML97 file which represents the surface for the 
  parametric space \a [uS,uE] and \a [vS,vE].
  It does not optimize the number of points required to 
  represent the surface. 

  \param filename  the file name for the output VRML file
  \param Nu  the number of points in the \a u direction
  \param Nv  the number of points in the \a v direction
  \param uS  the starting value of \a u
  \param uE  the end value of \a u
  \param vS  the starting value of \a v
  \param vE  the end value of \a v

  \return 1 on success, 0 otherwise

  \warning The parametric surface must be valid

  \author Philippe Lavoie
  \date 30 April, 1999
*/
template <class T, int N>
int ParaSurface<T,N>::writeVRML97(ostream &fout,const Color& color,int Nu,int Nv, T uS,T uE,T vS, T vE) const {

  fout << "#VRML V2.0 utf8\n" ;
  fout << "#  Generated by Phil's NURBS library\n" ;
  fout << "\nGroup {\n" ;
  fout << "\n  children [\n" ; 
  //fout << "\tDEF PS SphereSensor {}\n" ;
  fout << "\tDEF T Transform {\n"; 
  fout << "\t  children [\n" ;
  fout << "\t\tShape {\n" ;
  fout << "\t\t appearance Appearance {\n" ; 
  fout << "\t\t\tmaterial Material{ diffuseColor " << float(color.r/255.0) << ' ' << float(color.g/255.0) << ' ' << float(color.b/255.0) << " }\n" ;
  fout << "\t\t }\n" ; 
  fout << "\t\t geometry IndexedFaceSet {\n" ;
  fout << "\t\t\tsolid FALSE\n" ; 
  fout << "\t\t\tcoord Coordinate {\n" ;
  fout << "\t\t\t point [\n" ;

  T u,v,du,dv ;

  if(Nu<=1)
    Nu = 2 ;  // Should I put a warning message ?
  if(Nv<=1)
    Nv = 2 ;  // Should I put a warning message ?

  u = uS ;
  v = vS ;
  du = (uE-uS)/(T)(Nu-1) ;
  dv = (vE-vS)/(T)(Nv-1) ;

  int i,j ;

  Point_nD<T,N> p_min = pointAt(u,v) ;
  Point_nD<T,N> p_max = pointAt(u,v) ; 

  for(i=0;i<Nu;++i){
    v = vS ;
    for(j=0 ;j<Nv;++j){
      Point_nD<T,N> p ;
      p = pointAt(u,v) ;
      fout << "\t\t\t\t" << p.x() << ' ' << p.y() << ' ' << p.z() << ",\n" ;
      v += dv ;
      if(p.x()<p_min.x()) p_min.x() = p.x();
      if(p.y()<p_min.y()) p_min.y() = p.y();
      if(p.z()<p_min.z()) p_min.z() = p.z();
      if(p.x()>p_max.x()) p_max.x() = p.x();
      if(p.y()>p_max.y()) p_max.y() = p.y();
      if(p.z()>p_max.z()) p_max.z() = p.z();
    }
    u += du ;
  }
  
  fout << "\t\t\t ]\n" ; // point [
  fout << "\t\t\t}\n" ; // coord

  fout << "\t\t\t coordIndex [\n" ;
  
  for(i=0;i<Nu-1;++i){
    for(j=0;j<Nv-1;++j) {
      fout << "\t\t\t\t" << i*Nv+j << ", " << i*Nv+j+1 << ", " << (i+1)*Nv+j << ", -1,\n";
      fout << "\t\t\t\t" << i*Nv+j+1 << ", " << (i+1)*Nv+j+1<< ", " << (i+1)*Nv+j  << ", -1,\n";
    }
  }
  fout << "\t\t\t ]\n" ;
  fout << "\t\t\t}\n" ; // IndexedFaceSet
  fout << "\t\t}\n" ;
  fout << "\t ]\n" ; 
  fout << "\t}\n" ;
  fout << "  ]\n" ; 
  fout << "}\n" ; 

  Point_nD<T,N> p_mid((p_max.x()+p_min.x())/T(2),
		      (p_max.y()+p_min.y())/T(2),
		      (p_max.z()+p_min.z())/T(2));
  
  T x_axis = p_max.x() - p_min.x() ;
  T y_axis = p_max.y() - p_min.y() ; 

  T axis = (x_axis< y_axis) ? y_axis : x_axis ;
  axis *= T(2) ;

  fout << "Viewpoint {\n\t position " << p_mid.x() << ' ' << p_mid.y() << ' ' << p_max.z()+axis << "\n\t description \"top\"\n}\n" ; 


  fout << "NavigationInfo { type \"EXAMINE\" }\n" ; 

  return 1 ;
}




} // end namespace
