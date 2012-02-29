/*=============================================================================
        File: nurbs.h
     Purpose:       
    Revision: $Id: nurbs.h,v 1.2 2002/05/13 21:07:46 philosophil Exp $
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
#ifndef _nurbs_nurbs_h_
#define _nurbs_nurbs_h_

#include "matrixRT.h"

#include "curve.h"
#include "matrix.h"
#include "matrixMat.h"
#include "error.h"
#include "image.h"

/*!
 */
namespace PLib {

  template <class T, int N> class NurbsSurface ;
  template <class T, int N> class NurbsCurve ;
  template <class T, int N> class NurbsCurveArray ;

  template <class T, int N> void generateCompatibleCurves(NurbsCurveArray<T,N> &ca);


  /*!
    \brief A NURBS curve class
    
    This class is used to represent and manipulate NURBS curve. 
    The curves are composed of points in 4D. They can have any 
    degree and have any number of control points.
    
    \author Philippe Lavoie 
    \date 4 Oct. 1996
  */
  template <class T, int N>
    class NurbsCurve : public ParaCurve<T,N>{
    public:
      NurbsCurve() ;
      NurbsCurve(const NurbsCurve<T,N>& nurb) ;
      NurbsCurve(const Vector< HPoint_nD<T,N> >& P1, const Vector<T> &U1, int deg=3) ;
      NurbsCurve(const Vector< Point_nD<T,N> >& P1, const Vector<T> &W, const Vector<T> &U1, int deg=3) ;
      virtual ~NurbsCurve() { } // empty destructor
      
      // Reference to internal data
      int degree() const //!< a reference to the degree of the curve
	{ return deg_ ; }
      const Vector< HPoint_nD<T,N> >& ctrlPnts() const //!< a reference to the vector of control points
	{ return P; }
      const HPoint_nD<T,N> ctrlPnts(int i) const //!< a reference to one of the control points
	{ return P[i] ; }
      const Vector<T>& knot() const //!< a reference to the vector of knots
	{ return U ; }
      T knot(int i) const //!< the i-th knot
	{ return U[i]; }

      // basic functions
      void resize(int n, int Deg) ;
      virtual void reset(const Vector< HPoint_nD<T,N> >& P1, const Vector<T> &U1, int deg) ;
      virtual NurbsCurve& operator=(const NurbsCurve<T,N>&) ;
      
      // Evaluattion functions
      virtual HPoint_nD<T,N> operator()(T u) const;
      HPoint_nD<T,N> hpointAt(T u) const  //!< calls operator()
	{ return operator()(u) ; }
      HPoint_nD<T,N> hpointAt(T u, int span) const ; 
      friend HPoint_nD<T,N> C(T u, const NurbsCurve<T,N>& nurb) {return nurb(u) ; } //!< a function interface to operator()
      friend Point_nD<T,N> Cp(T u, const NurbsCurve<T,N>& nurb) {return project(nurb(u)) ;} //!< returns the curvePoint in 3D
      
      
      // derivative functions
      void deriveAtH(T u, int, Vector< HPoint_nD<T,N> >&) const;
      void deriveAt(T u, int, Vector< Point_nD<T,N> >&) const;
      void deriveAtH(T u, int, int, Vector< HPoint_nD<T,N> >&) const;
      void deriveAt(T u, int, int, Vector< Point_nD<T,N> >&) const;
      Point_nD<T,N> derive3D(T u, int d) const;
      HPoint_nD<T,N> derive(T u, int d) const;
      Point_nD<T,N> normal(T u, const Point_nD<T,N>& v) const ;
      
      HPoint_nD<T,N> firstD(T u) const ; 
      HPoint_nD<T,N> firstD(T u, int span) const ; 
      Point_nD<T,N> firstDn(T u) const ; 
      Point_nD<T,N> firstDn(T u, int span) const ; 
      
      // Basis functions
      T basisFun(T u, int i, int p=-1) const ;
      void basisFuns(T u, int span, Vector<T>& N) const ;
      void dersBasisFuns(int n,T u, int span, Matrix<T>& N) const;
      
      // Knot functions
      T minKnot() const //! the minimal value for the knot vector
	{ return U[0] ; } 
      T maxKnot() const //!< the maximal value for the knot vector
	{ return U[U.n()-1] ; }
      int findSpan(T u) const ;
      void findMultSpan(T u, int& r, int& s) const;
      int findMult(int r) const;
      int findKnot(T u) const;
      T getRemovalBnd(int r, int s) const;
      
      void removeKnot(int r, int s, int num);
      void removeKnotsBound(const Vector<T>& ub, Vector<T>& ek, T E) ;
      int knotInsertion(T u, int r,NurbsCurve<T,N>& nc);
      void refineKnotVector(const Vector<T>& X);
      void refineKnotVectorClosed(const Vector<T>& X);
      void mergeKnotVector(const Vector<T> &Um);
      
      void clamp() ; 
      void unclamp();

      // Curve fitting functions
      int leastSquares(const Vector< Point_nD<T,N> >& Q, int degC, int n ) ;
      int leastSquares(const Vector< Point_nD<T,N> >& Q, int degC, int n, const Vector<T>& ub);
      int leastSquaresH(const Vector< HPoint_nD<T,N> >& Q, int degC, int n, const Vector<T>& ub);
      int leastSquares(const Vector< Point_nD<T,N> >& Q, int degC, int n, const Vector<T>& ub, const Vector<T>& knot);
      int leastSquaresH(const Vector< HPoint_nD<T,N> >& Q, int degC, int n, const Vector<T>& ub, const Vector<T>& knot);
      
      int leastSquaresClosed(const Vector< Point_nD<T,N> >& Q, int degC, int n ) ;
      int leastSquaresClosed(const Vector< Point_nD<T,N> >& Q, int degC, int n, const Vector<T>& ub);
      int leastSquaresClosedH(const Vector< HPoint_nD<T,N> >& Q, int degC, int n, const Vector<T>& ub);
      int leastSquaresClosed(const Vector< Point_nD<T,N> >& Q, int degC, int n, const Vector<T>& ub, const Vector<T>& knot);
      int leastSquaresClosedH(const Vector< HPoint_nD<T,N> >& Q, int degC, int n, const Vector<T>& ub, const Vector<T>& knot);
      
      void globalApproxErrBnd(Vector< Point_nD<T,N> >& Q, int deg, T E);
      void globalApproxErrBnd(Vector< Point_nD<T,N> >& Q, Vector<T>& ub, int deg, T E);
      void globalApproxErrBnd2(Vector< Point_nD<T,N> >& Q, int degC, T E);
      void globalApproxErrBnd3(Vector< Point_nD<T,N> >& Q, int degC, T E);
      void globalApproxErrBnd3(Vector< Point_nD<T,N> >& Q, const Vector<T> &ub, int degC, T E);
      
      void globalInterp(const Vector< Point_nD<T,N> >& Q, int d);
      void globalInterp(const Vector< Point_nD<T,N> >& Q, const Vector<T>& ub, int d);
      void globalInterpH(const Vector< HPoint_nD<T,N> >& Q, int d);
      void globalInterpH(const Vector< HPoint_nD<T,N> >& Q, const Vector<T>& U, int d);
      void globalInterpH(const Vector< HPoint_nD<T,N> >& Q, const Vector<T>& ub, const Vector<T>& U, int d);
      
      void globalInterpClosed(const Vector< Point_nD<T,N> >& Qw, int d);
      void globalInterpClosed(const Vector< Point_nD<T,N> >& Qw, const Vector<T>& ub, int d);
      void globalInterpClosedH(const Vector< HPoint_nD<T,N> >& Qw, int d);
      void globalInterpClosedH(const Vector< HPoint_nD<T,N> >& Qw, const Vector<T>& U, int d);
      void globalInterpClosedH(const Vector< HPoint_nD<T,N> >& Qw, const Vector<T>& ub, const Vector<T>& U, int d);
      void globalInterpClosed(const Vector< Point_nD<T,N> >& Qw, const Vector<T>& ub, const Vector<T>& Uc, int d);
      
      void globalInterpD(const Vector< Point_nD<T,N> >& Q, const Vector< Point_nD<T,N> >& D, int d, int unitD, T a=1.0);
      
      void projectTo(const Point_nD<T,N>& p, T guess, T& u, Point_nD<T,N>& r, T e1=0.001, T e2=0.001,int maxTry=100) const;
      
      
      T length(T eps=0.001,int n=100) const ; 
      T lengthIn(T us, T ue, T eps=0.001, int n=100) const ; 
      T lengthF(T) const ;
      T lengthF(T,int) const ;
      
      // Generate type of curve
      void makeCircle(const Point_nD<T,N>& O, const Point_nD<T,N>& X, const Point_nD<T,N>& Y, T r, double as, double ae);
      void makeCircle(const Point_nD<T,N>& O, T r, double as, double ae);
      void makeCircle(const Point_nD<T,N>& O, T r);
      void makeLine(const Point_nD<T,N>& P0, const Point_nD<T,N>& P1, int d) ; 
      virtual void degreeElevate(int t);
      
#ifndef HAVE_ISO_FRIEND_DECL
      friend void generateCompatibleCurves (NurbsCurveArray<T,N> &ca);
#else
      friend void generateCompatibleCurves <>(NurbsCurveArray<T,N> &ca);
#endif

      void decompose(NurbsCurveArray<T,N>& c) const;
      void decomposeClosed(NurbsCurveArray<T,N>& c) const ;
      
      int splitAt(T u, NurbsCurve<T,N>& cl, NurbsCurve<T,N>& cu) const;
      int mergeOf(const NurbsCurve<T,N>& cl, const NurbsCurve<T,N> &cu) ;
      
      // Modifies the NURBS curve
      void transform(const MatrixRT<T>& A) ;
      void modCP(int i,const HPoint_nD<T,N>& a) { P[i] = a ; } // To manipulate the value of the control point $P[i]$
      void modCPby(int i,const HPoint_nD<T,N>& a) { P[i] += a ; } // To manipulate the value of the control point $P[i]$
      virtual void modKnot(const Vector<T>& knotU) { if(knotU.n()-deg_-1==P.n()) U = knotU ; }  // to change the values of the knot vector only if the size is compatible with P.n
      
      int movePoint(T u, const Point_nD<T,N>& delta) ;
      int movePoint(T u, const BasicArray< Point_nD<T,N> >& delta) ;
      int movePoint(const BasicArray<T>& ur, const BasicArray< Point_nD<T,N> >& D);
      int movePoint(const BasicArray<T>& ur, const BasicArray< Point_nD<T,N> >& D, const BasicArray_INT& Dr, const BasicArray_INT& Dk) ;
      int movePoint(const BasicArray<T>& ur, const BasicArray< Point_nD<T,N> >& D, const BasicArray_INT& Dr, const BasicArray_INT& Dk, const BasicArray_INT& fixCP);
      
      void setTangent(T u, const Point_nD<T,N>& T0) ;
      void setTangentAtEnd(const Point_nD<T,N>& T0, const Point_nD<T,N>& T1) ;

      // I/O functions
      int read(const char*) ;
      int write(const char*) const ;
      virtual int read(ifstream &fin) ;
      int write(ofstream &fout) const ;
      int writePS(const char*,int cp=0,T magFact=T(-1),T dash=T(5), bool bOpen=true) const ;
      int writePSp(const char*,const Vector< Point_nD<T,N> >&,const Vector< Point_nD<T,N> >&, int cp=0,T magFact=0.0,T dash=5.0, bool bOpen=true) const ;
      
      int writeVRML(ostream &fout,T radius,int K, const Color& color,int Nu,int Nv, T u_s, T u_e) const ;
      int writeVRML(const char* filename,T radius,int K, const Color& color,int Nu,int Nv, T u_s, T u_e) const ;
      int writeVRML(const char* filename,T radius=1,int K=5, const Color& color=whiteColor,int Nu=20,int Nv=20) const { return writeVRML(filename,radius,K,color,Nu,Nv,U[0],U[U.n()-1]) ; } // writes the curve to a VRML file
      int writeVRML(ostream& fout,T radius=1,int K=5, const Color& color=whiteColor,int Nu=20,int Nv=20) const { return writeVRML(fout,radius,K,color,Nu,Nv,U[0],U[U.n()-1]) ; } // writes the curve to a VRML file

      int writeVRML97(const char* filename,T radius,int K, const Color& color,int Nu,int Nv, T u_s, T u_e) const ;
      int writeVRML97(ostream &fout,T radius,int K, const Color& color,int Nu,int Nv, T u_s, T u_e) const ;
      int writeVRML97(const char* filename,T radius=1,int K=5, const Color& color=whiteColor,int Nu=20,int Nv=20) const { return writeVRML97(filename,radius,K,color,Nu,Nv,U[0],U[U.n()-1]) ; } // writes the curve to a VRML file
      int writeVRML97(ostream& fout,T radius=1,int K=5, const Color& color=whiteColor,int Nu=20,int Nv=20) const { return writeVRML97(fout,radius,K,color,Nu,Nv,U[0],U[U.n()-1]) ; } // writes the curve to a VRML file

      int writeDisplayLINE(const char* filename, int iNu, const Color& color=blueColor,T fA=1) const ;
      int writeDisplayLINE(const char* filename,const Color& color, int iNu,T u_s, T u_e) const;
      void drawImg(Image_UBYTE& Img,unsigned char color=255,T step=0.01) ;
      void drawImg(Image_Color& Img,const Color& color,T step=0.01) ;
      void drawAaImg(Image_Color& Img, const Color& color, int precision=3,int alpha=1) ;
      void drawAaImg(Image_Color& Img, const Color& color, const NurbsCurve<T,3>& profile, int precision=3,int alpha=1) ;
      NurbsSurface<T,3> drawAaImg(Image_Color& Img, const Color& color, const NurbsCurve<T,3>& profile, const NurbsCurve<T,3> &scaling, int precision=3,int alpha=1) ;
      
      BasicList<Point_nD<T,N> > tesselate(T tolerance, BasicList<T> *uk) const ;
      
    protected:
      Vector< HPoint_nD<T,N> > P; // the vector of control points
      Vector<T> U ;  // the knot vector
      int deg_ ;  // the degree of the NURBS curve
    };
  
  typedef NurbsCurve<float,3> NurbsCurvef ;
  typedef NurbsCurve<double,3> NurbsCurved ;
  typedef NurbsCurve<float,2> NurbsCurve_2Df ;
  typedef NurbsCurve<double,2> NurbsCurve_2Dd ;
  
} // end namespace

typedef PLib::NurbsCurve<float,3> PlNurbsCurvef ;
typedef PLib::NurbsCurve<double,3> PlNurbsCurved ;
typedef PLib::NurbsCurve<float,2> PlNurbsCurve_2Df ;
typedef PLib::NurbsCurve<double,2> PlNurbsCurve_2Dd ;

/*!
 */
namespace PLib {
  
  template <class T, int N>
    T chordLengthParam(const Vector< Point_nD<T,N> >& Q, Vector<T> &ub);
  template <class T, int N>
    T chordLengthParamH(const Vector< HPoint_nD<T,N> >& Q, Vector<T> &ub);
  template <class T, int N>
    T chordLengthParamClosed(const Vector< Point_nD<T,N> >& Q, Vector<T> &ub, int deg);
  template <class T, int N>
    T chordLengthParamClosedH(const Vector< HPoint_nD<T,N> >& Q, Vector<T> &ub, int deg);
  template <class T>
    void binomialCoef(Matrix<T>& Bin) ;
  template <class T>
    Vector<T> knotUnion(const Vector<T>& Ua, const Vector<T>& Ub);
  template <class T>
    T nurbsBasisFun(T u, int i, int p, const Vector<T>& U) ; 
  template <class T>
    void nurbsBasisFuns(T u, int span, int deg, const Vector<T>& U, Vector<T>& N);
  template <class T>
    void nurbsDersBasisFuns(int n, T u, int span, int deg, const Vector<T>& U, Matrix<T>& ders) ;
  
  template <class T, int N> int intersectLine(const Point_nD<T,N>& p1, const Point_nD<T,N>& t1, const Point_nD<T,N>& p2, const Point_nD<T,N>& t2, Point_nD<T,N>& p);
#ifndef HAVE_TEMPLATE_OF_TEMPLATE
  template <> int intersectLine(const Point_nD<float,2>& p1, const Point_nD<float,2>& t1, const Point_nD<float,2>& p2, const Point_nD<float,2>& t2, Point_nD<float,2>& p);
  template <> int intersectLine(const Point_nD<double,2>& p1, const Point_nD<double,2>& t1, const Point_nD<double,2>& p2, const Point_nD<double,2>& t2, Point_nD<double,2>& p);
#endif
  
  template <class T> void knotAveraging(const Vector<T>& uk, int deg, Vector<T>& U) ;
  template <class T> void knotAveragingClosed(const Vector<T>& uk, int deg, Vector<T>& U) ;
  template <class T> void knotApproximationClosed( Vector<T>& U, const  Vector<T>& ub, int n, int p);
  template <class T> void averagingKnots(const Vector<T>& U, int deg, Vector<T>& uk);
  template <class T> int findSpan(T u, const Vector<T>& U, int deg);
  template <class T> int maxInfluence(int i, const Vector<T>& U, int p, T &u);
  
  template <class T> void to3D(const NurbsCurve<T,2>&, NurbsCurve<T,3>&);
  template <class T> void to3D(const NurbsCurve<T,3>&, NurbsCurve<T,3>&);
  template <class T> void to2D(const NurbsCurve<T,3>&, NurbsCurve<T,2>&);
  
  template <class T, int N> 
    void wrapPointVector(const Vector<Point_nD<T,N> >& Q, int d, Vector<Point_nD<T,N> >& Qw);
  template <class T, int N> 
    void wrapPointVectorH(const Vector<HPoint_nD<T,N> >& Q, int d, Vector<HPoint_nD<T,N> >& Qw);
  
  
  /*!
    \brief finds the matrix coordinate i,j of a Point_nD<T,N>
    
    Finds the closest coordinate of a matrix(i,j) to the (x,y)
    coordinates of a 3D point.
    
    \param p  the 3D point
    \param i  the coordinate for the row (corresponds to y)
    \param j  the coordinate for the column (corresponds to x)
    \param rows  the number of rows in the matrix
    \param cols  the number of columns in the matrix
    
    \return  1 if the coordinates are in the matrix range, 0 otherwise
    
    \author Philippe Lavoie
    \date 25 July 1997
  */
  template <class T, int N>
    inline int getCoordinates(const Point_nD<T,N>& p, int& i, int& j, int rows, int cols){
    i = int(rint(p.y())) ;
    j = int(rint(p.x())) ;
    if(i>=rows) return 0 ;
    if(j>=cols) return 0 ;
    if(i<0) return 0 ;
    if(j<0) return 0 ;
    return 1 ;
  }
  
  /*!
    \brief an array of NurbsCurve
    
    This class represents an array of NurbsCurve.
    
    \author Philippe Lavoie 
    \date 4 Oct. 1996
  */
  template <class T, int N>
    class NurbsCurveArray {
    public:
      int n () const { return sze ; }
      NurbsCurveArray(NurbsCurve<T,N>* Ca, int size) ;
      NurbsCurveArray() { C = 0 ; sze = 0 ; rsize = 0 ;}
      virtual ~NurbsCurveArray(){ if(C){ for(int i=0;i<rsize;i++) delete C[i];  delete []C ; }}
      
      virtual NurbsCurve<T,N>& operator[](int i) { return *(C[i]) ; }
      virtual NurbsCurve<T,N> operator[](int i) const { return *(C[i]) ; }
      
      virtual void resize(int s) ;
      void init(NurbsCurve<T,N>* Ca, int size) ;
      int read(const char *filename);
      int write(const char *filename);
      int writePS(const char*,int cp=0,T magFact=T(-1),T dash=T(5), bool bOpen=true) const ;
      int writePSp(const char*,const Vector< Point_nD<T,N> >&,const Vector< Point_nD<T,N> >&, int cp=0,T magFact=0.0,T dash=5.0, bool bOpen=true) const ;
      
    protected:
      NurbsCurve<T,N>& curve(int i) { return *(C[i]) ; }
      NurbsCurve<T,N> curve(int i) const { return *(C[i]) ; }
      
      int sze ; // the number of NURBS curves in the array
      int rsize ; // the number of space allocated for the array
      NurbsCurve<T,N>** C ; // An array of pointers to NURBS curves
    };
  
  typedef NurbsCurveArray<float,3> NurbsCurveArrayf ;
  typedef NurbsCurveArray<double,3> NurbsCurveArrayd ;
  typedef NurbsCurveArray<float,2> NurbsCurveArray_2Df ;
  typedef NurbsCurveArray<double,2> NurbsCurveArray_2Dd ;
  
} // end namespace 

typedef PLib::NurbsCurveArray<float,3> PlNurbsCurveArrayf ;
typedef PLib::NurbsCurveArray<double,3> PlNurbsCurveArrayd ;
typedef PLib::NurbsCurveArray<float,2> PlNurbsCurveArray_2Df ;
typedef PLib::NurbsCurveArray<double,2> PlNurbsCurveArray_2Dd ;



#ifdef INCLUDE_TEMPLATE_SOURCE
#include "nurbs.cpp"
#include "nurbsArray.cpp"
#endif

#endif 

