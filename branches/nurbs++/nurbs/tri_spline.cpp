/*=====================================================================
        File: tri_spline.cpp
     Purpose:       
    Revision: $Id: tri_spline.cpp,v 1.3 2002/05/17 18:24:21 philosophil Exp $
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

#include "tri_spline.h"

/*!
 */
namespace PLib {

  int triangularNumber(int deg){
    return ((deg+1)*(deg+2))/2 ;
  }
  
  static int row_start[5][5] = {
    {0,-1,-1,-1,-1},
    {0,2,-1,-1,-1},     
    {0,3,5,-1,-1},    
    {0,4,7,9,-1},
    {0,5,9,12,14}} ;



  /*
  static int row_start1[2] = {0,2} ;
  static int row_start2[3] = {0,3,5} ;
  static int row_start3[4] = {0,4,7,9} ;
  static int row_start4[5] = {0,5,9,12,14};

  template <int Deg>
    int row_start(int i) {
    return 0;
  }

  template <>
  inline int row_start<1>(int i){
    return row_start1[i] ;
  }

  template <>
  inline int row_start<2>(int i){
    return row_start2[i] ;
  }

  template <>
  inline int row_start<3>(int i){
    return row_start3[i] ;
  }

  template <>
  inline int row_start<4>(int i){
    return row_start4[i] ;
  }

  */

  inline int baryCoord(const int Deg, int i, int j, int k){
    return row_start[Deg][j] + i ;
  }

  inline void reverseBaryCoord(int deg, int b, int &i, int &j ,int &k)
    {
      j = deg ; 
      while(row_start[deg][j]>b)
	--j ;
      i = b - row_start[deg][j] ;
      k = deg - i - j ; 
    }

  
  template < class T>
    inline T basis(int deg, T* B, int i, int j, int k)
    {
      if(i<0 || j<0 || k<0)
	return 0 ;
      if(deg<=0)
	return 1;
      return B[baryCoord(deg,i,j,k)] ;
    }

  template <class T, int D>
    TriangularBSpline<T,D>::TriangularBSpline(int degree) : cp(triangularNumber(degree)), deg(degree){
      ;
    }

  //! Returns the (i,j,k) control point.
  template <class T, int D>
    Point_nD<T,D>& TriangularBSpline<T,D>::b(int i, int j, int )
    {
      //static int row_start[3] = {0, 3, 5};
      return cp[baryCoord(deg,i,j,-1)];
    }
  
  //! Returns the (i,j,k) control point.
  template <class T, int D>
    Point_nD<T,D> TriangularBSpline<T,D>::b(int i, int j, int ) const
    {
      //static int row_start[3] = {0, 3, 5};
      return cp[baryCoord(deg,i,j,-1)];
    }
  
  //! Evaluates the Bezier triangle at (u,v).
  template <class T, int D>
    Point_nD<T,D> TriangularBSpline<T,D>::operator()(T u, T v) const
    {
      T w = 1 - u - v;
      T u2 = u * u;
      T v2 = v * v;
      T w2 = w * w;
      switch(deg){
      case 2:
	return (w2*b(0,0,2) + (2*u*w)*b(1,0,1) + u2*b(2,0,0) +
		(2*v*w)*b(0,1,1) + (2*u*v)*b(1,1,0) + v2*b(0,2,0));
      case 4:{
	T u3 = u2 * u;
	T u4 = u3 * u;
	T v3 = v2 * v;
	T v4 = v3 * v;
	T w3 = w2 * w;
	T w4 = w3 * w;	
	return (w4*b(0,0,4) + (4*u*w3)*b(1,0,3) + (6*u2*w2)*b(2,0,2) +
	    (4*u3*w)*b(3,0,1) + u4*b(4,0,0) + (4*v*w3)*b(0,1,3) +
	    (12*u*v*w2)*b(1,1,2) + (12*u2*v*w)*b(2,1,1) + (4*u3*v)*b(3,1,0) +
	    (6*v2*w2)*b(0,2,2) + (12*u*v2*w)*b(1,2,1) + (6*u2*v2)*b(2,2,0) +
	    (4*v3*w)*b(0,3,1) + (4*u*v3)*b(1,3,0) + v4*b(0,4,0));
      }
      default:
	return Point_nD<T,D>(0,0,0) ;
      }
    }

  template <class T, int D>
    RTriangularBSpline<T,D>::RTriangularBSpline(int degree) :  cp(triangularNumber(degree)), deg(degree){
      ;
    }

  //! Returns the (i,j,k) control point.
  template <class T, int D>
    HPoint_nD<T,D>& RTriangularBSpline<T,D>::b(int i, int j, int )
    {
      //static int row_start[3] = {0, 3, 5};
      return cp[baryCoord(deg,i,j,-1)];
    }
  
  //! Returns the (i,j,k) control point.
  template <class T, int D>
    HPoint_nD<T,D> RTriangularBSpline<T,D>::b(int i, int j, int ) const
    {
      //static int row_start[3] = {0, 3, 5};
      return cp[baryCoord(deg,i,j,-1)];
    }
  
  
  //! Evaluates the Bezier triangle at (u,v).
  template <class T, int D>
    HPoint_nD<T,D> RTriangularBSpline<T,D>::operator()(T u, T v) const
    {
      T w = T(1) - u - v;
      /*
      T u2 = u * u;
      T v2 = v * v;
      T w2 = w * w;
      return (w2*b(0,0,2) + (2*u*w)*b(1,0,1) + u2*b(2,0,0) +
	      (2*v*w)*b(0,1,1) + (2*u*v)*b(1,1,0) + v2*b(0,2,0));
	      */
      T *tmp1 = new T[triangularNumber(deg)] ;
      T *tmp2 = new T[triangularNumber(deg)] ;

      T *B = tmp1 ;
      T *Bold = tmp2 ;

      int switchBtmp = -1 ;

      B[0] = Bold[0] = 1 ;

      int i,j,k ;
      for(int p = 1 ; p <= deg ; ++p){
	if(switchBtmp>0){
	  B = tmp2 ;
	  Bold = tmp1 ;
	}
	else{
	  B = tmp1 ;
	  Bold = tmp2 ;
	}
	switchBtmp *= -1 ;

	for(int s = 0; s<triangularNumber(p) ; ++s){
	  reverseBaryCoord(p,s,i,j,k) ;
	  B[baryCoord(p,i,j,k)] = 
	    u*basis(p-1,Bold,i-1,j,k) + 
	    v*basis(p-1,Bold,i,j-1,k) + 
	    w*basis(p-1,Bold,i,j,k-1) ;
	}
      }

      HPoint_nD<T,D> result(0,0,0,0) ;

      for(int s=0;s<triangularNumber(deg);++s){
	result += B[s]*cp[s] ;
      }
      
      return result ;
    }
  
  template <class T, int D>
    int RTriangularBSpline<T,D>::writeVRML(const char* filename,  const Color& color, int Nu, int Nv, int Nw) const{
    
    ofstream fout(filename) ;
    
    if(!fout)
      return 0 ;
    return writeVRML(fout,color,Nu,Nv,Nw) ;

  }

  template <class T, int D>
    int RTriangularBSpline<T,D>::writeVRML(ostream &fout,  const Color& color, int Nu, int Nv, int Nw) const{

    
    fout << "#VRML V1.0 ascii\n" ;
    fout << "#  Generated by Phil's NURBS library\n" ;
    fout << "\nSeparator {\n" << "\tMaterialBinding { value PER_VERTEX_INDEXED }\n"  ;
    fout << "\tMaterial{\n\t\tambientColor 0.25 0.25 0.25\n\t\tdiffuseColor " << float(color.r/255.0) << ' ' << float(color.g/255.0) << ' ' << float(color.b/255.0) << "\n\t}\n" ;
    fout << "\tCoordinate3 {\n" ;
    fout << "\t\tpoint [\n" ;

    T u,v,du,dv,w,dw ;
    
    const T uS = 0 ;
    const T uE = 1 ;
    const T vS = 0 ;
    const T vE = 1 ;
    const T wS = 0 ;
    const T wE = 1 ;

    if(Nu<=1)
      Nu = 2 ;  // Should I put a warning message ?
    if(Nv<=1)
      Nv = 2 ;  // Should I put a warning message ?
    if(Nw<=1)
      Nw = 2 ;  // Should I put a warning message ?
    
    u = uS ;
    v = vS ;
    w = wS ;
    du = (uE-uS)/(T)(Nu) ;
    dv = (vE-vS)/(T)(Nv) ;
    dw = (wE-wS)/(T)(Nw) ;

    Point_nD<T,3> p1,p2,p3,p4 ;
    
    int n =0 ;

    for(int i=0;i<Nu;++i){
      u = uS + T(i)*du ;
      for(int j=0;j<Nv;++j){
	v = vS + T(j)*dv ;
	if(u+v>=T(1)-du)
	  break ; 
	p1 = project(operator()(u,v)) ;
	p2 = project(operator()(u,v+dv)) ;
	p3 = project(operator()(u+du,v+dv)) ;
	p4 = project(operator()(u+du,v)) ;
	fout << "\t\t\t" << p1.x() << ' ' << p1.y() << ' ' << p1.z() << ",\n" ;
	fout << "\t\t\t" << p2.x() << ' ' << p2.y() << ' ' << p2.z() << ",\n" ;
	fout << "\t\t\t" << p3.x() << ' ' << p3.y() << ' ' << p3.z() << ",\n" ;
	fout << "\t\t\t" << p4.x() << ' ' << p4.y() << ' ' << p4.z() << ",\n" ;
	++n ; 
      }
    }
    
    for(int i=0;i<Nu;++i){
      u = uS + T(i)*du ;
      v = T(1)-u ;
      p1 = project(operator()(u,v)) ;
      p2 = project(operator()(u,v-du)) ;
      p3 = project(operator()(u+du,v-du)) ;
      fout << "\t\t\t" << p1.x() << ' ' << p1.y() << ' ' << p1.z() << ",\n" ;
      fout << "\t\t\t" << p3.x() << ' ' << p3.y() << ' ' << p3.z() << ",\n" ;
      fout << "\t\t\t" << p2.x() << ' ' << p2.y() << ' ' << p2.z() << ",\n" ;
      
    }
    

    fout << "\t\t]\n" ; // point [
    fout << "\t}\n" ; // cordinate3

    fout << "\tIndexedFaceSet{\n" ;
    fout << "\t\tcoordIndex[\n" ;
         
    for(int k=0;k<n;++k){
      fout << "\t\t\t" << 4*k << ", " << 4*k+1 << ", " << 4*k+2 << ", -1,\n" ;
      fout << "\t\t\t" << 4*k << ", " << 4*k+2 << ", " << 4*k+3 << ", -1,\n" ;
    }

    for(int k=0;k<Nu;++k){
      fout << "\t\t\t" << 3*k+4*n<< ", " << 3*k+1+4*n << ", " << 3*k+2+4*n << ", -1,\n" ;
    }

    fout << "\t\t]\n" ;
    fout << "\t}\n" ; // IndexedFaceSet
    
    fout << "}\n" ;
    
    return 1 ;
    

  }

  template < class T, int D>
    void convert(const NurbsSurface<T,D>& surf, RTriangularBSpline<T,D> &t1, RTriangularBSpline<T,D> &t2) {
    
    if(surf.degreeU() != surf.degreeV()){
#ifdef USE_EXCEPTION
      throw NurbsError();
#else
      Error error("convert");
      error << "The surface must have have the same degree in U and V.\n" ;
      error.fatal();
#endif
      return ; 
    }

    const Matrix<HPoint_nD<T,D> > &p = surf.ctrlPnts() ;

    switch(surf.degreeU()){
    case 1:
      t1.setDegree(2) ;
      t2.setDegree(2) ; 
      
      // lower left triangle:
      t1.b(0,0,2) = p(0,0);
      t1.b(1,0,1) = 0.5 * (p(0,0) + p(0,1));
      t1.b(2,0,0) = p(0,1);
      
      t1.b(0,1,1) = 0.5 * (p(0,0) + p(1,0));
      t1.b(1,1,0) = 0.5 * (p(0,0) + p(1,1));
      
      t1.b(0,2,0) = p(1,0);
      
      // upper right triangle:
      t2.b(0,0,2) = p(1,1);
      t2.b(1,0,1) = 0.5 * (p(1,1) + p(0,1));
      t2.b(2,0,0) = p(0,1);
      
      t2.b(0,1,1) = 0.5 * (p(1,1) + p(1,0));
      t2.b(1,1,0) = 0.5 * (p(0,0) + p(1,1));
      
      t2.b(0,2,0) = p(1,0);
      break ; 

    case 2:
      t1.setDegree(4) ;
      t2.setDegree(4) ; 

      // lower left triangle:
      t1.b(0,0,4) = p(0,0);
      t1.b(1,0,3) = 0.5 * (p(0,0) + p(0,1));
      t1.b(2,0,2) = (p(0,0) + 4.0 * p(0,1) + p(0,2)) / 6.0;
      t1.b(3,0,1) = 0.5 * (p(0,1) + p(0,2));
      t1.b(4,0,0) = p(0,2);
      
      t1.b(0,1,3) = 0.5 * (p(0,0) + p(1,0));
      t1.b(1,1,2) = (p(0,0) + p(1,1)) / 3.0 + (p(0,1) + p(1,0)) / 6.0;
      t1.b(2,1,1) = (p(0,0) + p(1,2)) / 6.0 + (p(0,1) + p(1,1)) / 3.0;
      t1.b(3,1,0) = 0.5 * (p(0,1) + p(1,2));
      
      t1.b(0,2,2) = (p(0,0) + 4.0 * p(1,0) + p(2,0)) / 6.0;
      t1.b(1,2,1) = (p(0,0) + p(2,1)) / 6.0 + (p(1,0) + p(1,1)) / 3.0;
      t1.b(2,2,0) = (p(0,0) + 4.0 * p(1,1) + p(2,2)) / 6.0;
      
      t1.b(0,3,1) = 0.5 * (p(1,0) + p(2,0));
      t1.b(1,3,0) = 0.5 * (p(1,0) + p(2,1));
      
      t1.b(0,4,0) = p(2,0);
      
      // upper right triangle:
      t2.b(0,0,4) = p(2,2);
      t2.b(1,0,3) = 0.5 * (p(2,2) + p(1,2));
      t2.b(2,0,2) = (p(2,2) + 4.0 * p(1,2) + p(0,2)) / 6.0;
      t2.b(3,0,1) = 0.5 * (p(1,2) + p(0,2));
      t2.b(4,0,0) = p(0,2);
      
      t2.b(0,1,3) = 0.5 * (p(2,2) + p(2,1));
      t2.b(1,1,2) = (p(2,2) + p(1,1)) / 3.0 + (p(1,2) + p(2,1)) / 6.0;
      t2.b(2,1,1) = (p(2,2) + p(0,1)) / 6.0 + (p(1,2) + p(1,1)) / 3.0;
      t2.b(3,1,0) = 0.5 * (p(0,1) + p(1,2));

      t2.b(0,2,2) = (p(2,2) + 4.0 * p(2,1) + p(2,0)) / 6.0;
      t2.b(1,2,1) = (p(2,2) + p(1,0)) / 6.0 + (p(2,1) + p(1,1)) / 3.0;
      t2.b(2,2,0) = (p(2,2) + 4.0 * p(1,1) + p(0,0)) / 6.0;
      
      t2.b(0,3,1) = 0.5 * (p(2,1) + p(2,0));
      t2.b(1,3,0) = 0.5 * (p(1,0) + p(2,1));
      
      t2.b(0,4,0) = p(2,0);
      break ; 

    default:
#ifdef USE_EXCEPTION
      throw NurbsError();
#else
      Error error("convert");
      error << "There is no routine to convert a NURBS surface of degree" << surf.degreeU() << endl ;
      error.fatal();
#endif

    }
  }

  template <class T, int D>
    void RTriangularBSpline<T,D>::setDegree(int d){
    cp.resize(triangularNumber(d));
    deg = d ;
  }

}
