/*=============================================================================
        File: matrixRT.cpp
     Purpose:       
    Revision: $Id: matrixRT.cpp,v 1.2 2002/05/13 21:07:46 philosophil Exp $
  Created by: Philippe Lavoie          (25 July, 1997)
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

#include <matrixRT.h>

/*!
 */
namespace PLib {

/*!
  \brief  constructor with the angles and translation  parameters specified

  Sets the matrixRT to be a rotation of ax,ay,az then a translation x,y,z 
  around the origin.

  \param ax specifies the rotation around the x-axis
  \param ay  specifies the rotation around the y-axis
  \param az  specifies the rotation around the z-axis
  \param x  specifies the translation along the x-axis
  \param y  specifies the translation along the y-axis
  \param z  specifies the translation along the z-axis

  \author Philippe Lavoie 
  \date 25 July, 1997
 */
template <class T>
MatrixRT<T>::MatrixRT(T ax, T ay, T az, T x, T y, T z): Matrix<T>(4,4) {
  // the following is the same as
  // *this = C.translate(x,y,z)*B.rotate(ax,ay,az) ;
  rotate(ax,ay,az) ;
#ifdef COLUMN_ORDER
  m[12] = x ;
  m[13] = y ;
  m[14] = z ;  
#else
  m[3] = x ;
  m[7] = y ;
  m[11] = z ;
#endif
}

/*!
  \brief default constructor
  
  Default constructor

  \author Philippe Lavoie 
  \date 25 July, 1997
 */
template <class T>
MatrixRT<T>::MatrixRT() : Matrix<T>(4,4) {
  reset(0) ;
  diag(1.0) ;
}

/*!
  \brief copy constructor
  
  Copy constructor

  \param plM the matrix to copy

  \author Philippe Lavoie 
  \date 25 July, 1997
 */
template <class T>
MatrixRT<T>::MatrixRT(const Matrix<T>& plM) : Matrix<T>(4,4) {
  if(plM.rows() == 4 && plM.cols() == 4)
    *this = plM;
  else{
#ifdef USE_EXCEPTION
    throw(WrongSize2D(4,4,plM.rows(),plM.cols()));
#else
    Error error("MatrixRT<T>::MatrixRt(const Matrix<T>&)");
    error << "The matrix doesn't have the size (4,4)\n" ;
    error.fatal();
#endif
  }
}


/*!
  \brief Constructor from using a formated memory location

  \param p a vector of data composing the matrix

  \warning Make sure the ordering is adequate, i.e. either column or row wise.

  \author Philippe Lavoie 
  \date 25 July, 1997
 */
template <class T>
MatrixRT<T>::MatrixRT(T* p) : Matrix<T>(p,4,4) {
  // nothing to do, p should have all the proper data.
}

/*!
  \brief creates a rotation matrix of angle (ax,ay,az)

  The rotation are clockwise around the main axes. The rotation
  is performed in that order: a rotation around the z-axis, 
  then the y-axis and finally around the x-axis.

  \param ax  specifies the rotation around the x-axis
  \param ay  specifies the rotation around the y-axis
  \param az  specifies the rotation around the z-axis

  \sa rotateXYZ

  \author Philippe Lavoie 
  \date 25 July, 1997
*/
template <class T>
MatrixRT<T>& MatrixRT<T>::rotate(T ax,T ay, T az){
  T t1,t2,t4,t6,t7,t8,t10,t13 ;
  t1 = cos(az);
  t2 = cos(ay);
  t4 = sin(az);
  t6 = sin(ay);
  t7 = t1*t6;
  t8 = sin(ax);
  t10 = cos(ax);
  t13 = t4*t6;
#ifdef COLUMN_ORDER
  m[0] = t1*t2;
  m[4] = -t4*t2;
  m[8] = t6;
  m[12] = 0 ;
  m[1] = t7*t8+t4*t10;
  m[5] = -t13*t8+t1*t10;
  m[9] = -t2*t8;
  m[13] = 0 ;
  m[2] = -t7*t10+t4*t8;
  m[6] = t13*t10+t1*t8;
  m[10] = t2*t10;
  m[14] = m[3] = m[7] = m[11] = 0.0 ;
  m[15] = 1.0 ;
#else
  m[0] = t1*t2;
  m[1] = -t4*t2;
  m[2] = t6;
  m[3] = 0 ;
  m[4] = t7*t8+t4*t10;
  m[5] = -t13*t8+t1*t10;
  m[6] = -t2*t8;
  m[7] = 0 ;
  m[8] = -t7*t10+t4*t8;
  m[9] = t13*t10+t1*t8;
  m[10] = t2*t10;
  m[11] = m[12] = m[13] = m[14] = 0 ;
  m[15] = 1.0 ;
#endif
  return *this ;
}

/*!
  \brief creates a rotation matrix of angle (ax,ay,az)

  The rotation are clockwise around the main axes. The rotation
  is performed in that order: a rotation around the x-axis, 
  then the y-axis and finally around the z-axis.
  
  \param ax  specifies the rotation around the x-axis
  \param ay  specifies the rotation around the y-axis
  \param az  specifies the rotation around the z-axis

  \sa rotate
  
  \author Philippe Lavoie 
  \date 25 July, 1997
 */
template <class T>
MatrixRT<T>& MatrixRT<T>::rotateXYZ(T ax,T ay, T az){
  T t1,t2,t4,t5,t7,t8,t9,t17 ;
  t1 = (T)cos((double)az);
  t2 = (T)cos((double)ay);
  t4 = (T)sin((double)az);
  t5 = (T)cos((double)ax);
  t7 = (T)sin((double)ay);
  t8 = t1*t7;
  t9 = (T)sin((double)ax);
  t17 = t4*t7;
#ifdef COLUMN_ORDER
  m[0] = t1*t2;
  m[4] = -t4*t5+t8*t9;
  m[8] = t4*t9+t8*t5;
  m[12] = 0.0 ;
  m[1] = t4*t2;
  m[5] = t1*t5+t17*t9;
  m[9] = -t1*t9+t17*t5;
  m[13] = 0.0 ;
  m[2] = -t7;
  m[6] = t2*t9;
  m[10] = t2*t5;
  m[14] = m[3] = m[7] = m[11] = 0 ;
  m[15] = 1.0 ;
#else
  m[0] = t1*t2;
  m[1] = -t4*t5+t8*t9;
  m[2] = t4*t9+t8*t5;
  m[3] = 0.0 ;
  m[4] = t4*t2;
  m[5] = t1*t5+t17*t9;
  m[6] = -t1*t9+t17*t5;
  m[7] = 0.0 ;
  m[8] = -t7;
  m[9] = t2*t9;
  m[10] = t2*t5;
  m[11] = m[12] = m[13] = m[14] = 0 ;
  m[15] = 1.0 ;
#endif
  return *this ;
}

/*!
  \brief  Generates a translation matrix
  \param x  specifies the translation along the $x$-axis
  \param y  specifies the translation along the $y$-axis
  \param z  specifies the translation along the $z$-axis
  \warning This resets the rotation part of the matrix
  \author Philippe Lavoie 
  \date 25 July, 1997
 */
template <class T>
MatrixRT<T>& MatrixRT<T>::translate(T x, T y, T z){
  reset(0) ;
  diag(1.0) ;
#ifdef COLUMN_ORDER
  m[12] = x ;
  m[13] = y ;
  m[14] = z ;
#else
  m[3] = x ;
  m[7] = y ;
  m[11] = z ;
#endif
  return *this ;
}

/*!
  \brief Generates a scaling matrix  

  \param x  specifies the scaling along the $x$-axis
  \param y  specifies the scaling along the $y$-axis
  \param z  specifies the scaling along the $z$-axis

  \author Philippe Lavoie 
  \date 25 July, 1997
 */
template <class T>
MatrixRT<T>& MatrixRT<T>::scale(T x, T y, T z){
  reset(0) ;
  m[0] = x ;
  m[5] = y ;
  m[10] = z ;
  m[15] = 1.0 ;
  return *this ;
}

/*!
  \brief Multiplies a matrixRT with a HPoint_nD<T>.
  
  \param M  the matrix
  \param P  the control point
  \return the output control point rotated and trsnalted by M

  \author Philippe Lavoie 
  \date 25 July, 1997
 */
template <class T,int N>
HPoint_nD<T,N> operator*(const MatrixRT<T>& M, const HPoint_nD<T,N>& P){
  HPoint_nD<T,N> P2 ;

  P2.x() = M(0,0)*(T)P.x() + M(0,1)*(T)P.y() + M(0,2)*(T)P.z() + M(0,3)*(T)P.w() ;
  P2.y() = M(1,0)*(T)P.x() + M(1,1)*(T)P.y() + M(1,2)*(T)P.z() + M(1,3)*(T)P.w() ;
  P2.z() = M(2,0)*(T)P.x() + M(2,1)*(T)P.y() + M(2,2)*(T)P.z() + M(2,3)*(T)P.w() ;
  P2.w() = M(3,0)*(T)P.x() + M(3,1)*(T)P.y() + M(3,2)*(T)P.z() + M(3,3)*(T)P.w() ;

  return P2 ;
}

/*!
  \brief Multiplies a matrixRT with a point3D.
  
  \param M  the matrix
  \param P  the 3D point
  \return the output 3D point rotatated and translated by M

  \author Philippe Lavoie 
  \date 25 July, 1997
 */
template <class T,int N>
Point_nD<T,N> operator*(const MatrixRT<T>& M, const Point_nD<T,N>& P){
  Point_nD<T,N> P2 ;

  P2.x() = M(0,0)*(T)P.x() + M(0,1)*(T)P.y() + M(0,2)*(T)P.z() + M(0,3) ;
  P2.y() = M(1,0)*(T)P.x() + M(1,1)*(T)P.y() + M(1,2)*(T)P.z() + M(1,3) ;
  P2.z() = M(2,0)*(T)P.x() + M(2,1)*(T)P.y() + M(2,2)*(T)P.z() + M(2,3) ;

  return P2 ;
}

/*!
  \brief  Multiplies a matrixRT with a MatrixRT.

  \param m1  the first matrix
  \param m2  the second matrix
  
  \return a new matrixRT
  \warning Be aware of the way C++ handles function calls. If you want
               to generate a matrixRT from different ones, you can't do
	       \code A = A.translate(x,y,z)*A.rotate(ax,ay,az)\endcode, 
	       since the
	       translate and rotate return *this, when reaching the * 
	       operator, the function will be called with the same argument
	       on its left and right. Instead, use something like this:
	       \code A = A.translate(x,y,z)*B.rotate(ax,ay,az)\endcode.

  \author Philippe Lavoie 
  \date 25 July, 1997
 */
template <class T>
MatrixRT<T> operator*(const MatrixRT<T>& M1, const MatrixRT<T>& M2){
  MatrixRT<T> M ;
  T *m1,*m2,*m ;
  m1 = M1.m ;
  m2 = M2.m ;
  m = M.m ;
#ifdef COLUMN_ORDER
  m[0] = m1[0]*m2[0] + m1[4]*m2[1] + m1[8]*m2[2] + m1[12]*m2[3] ;
  m[4] = m1[0]*m2[4] + m1[4]*m2[5] + m1[8]*m2[6] + m1[12]*m2[7] ;
  m[8] = m1[0]*m2[8] + m1[4]*m2[9] + m1[8]*m2[10] + m1[12]*m2[11] ;
  m[12] = m1[0]*m2[12] + m1[4]*m2[13] + m1[8]*m2[14] + m1[12]*m2[15] ;
  
  m[1] = m1[1]*m2[0] + m1[5]*m2[1] + m1[9]*m2[2] + m1[13]*m2[3] ;
  m[5] = m1[1]*m2[4] + m1[5]*m2[5] + m1[9]*m2[6] + m1[13]*m2[7] ;
  m[9] = m1[1]*m2[8] + m1[5]*m2[9] + m1[9]*m2[10] + m1[13]*m2[11] ;
  m[13] = m1[1]*m2[12] + m1[5]*m2[13] + m1[9]*m2[14] + m1[13]*m2[15] ;
  
  m[2] = m1[2]*m2[0] + m1[6]*m2[1] + m1[10]*m2[2] + m1[14]*m2[3] ;
  m[6] = m1[2]*m2[4] + m1[6]*m2[5] + m1[10]*m2[6] + m1[14]*m2[7] ;
  m[10] = m1[2]*m2[8] + m1[6]*m2[9] + m1[10]*m2[10] + m1[14]*m2[11] ;
  m[14] = m1[2]*m2[12] + m1[6]*m2[13] + m1[10]*m2[14] + m1[14]*m2[15] ;
  
  m[3] = m1[3]*m2[0] + m1[7]*m2[1] + m1[11]*m2[2] + m1[15]*m2[3] ;
  m[7] = m1[3]*m2[4] + m1[7]*m2[5] + m1[11]*m2[6] + m1[15]*m2[7] ;
  m[11] = m1[3]*m2[8] + m1[7]*m2[9] + m1[11]*m2[10] + m1[15]*m2[11] ;
  m[15] = m1[3]*m2[12] + m1[7]*m2[13] + m1[11]*m2[14] + m1[15]*m2[15] ;
  
#else
  m[0] = m1[0]*m2[0] + m1[1]*m2[4] + m1[2]*m2[8] + m1[3]*m2[12] ;
  m[1] = m1[0]*m2[1] + m1[1]*m2[5] + m1[2]*m2[9] + m1[3]*m2[13] ;
  m[2] = m1[0]*m2[2] + m1[1]*m2[6] + m1[2]*m2[10] + m1[3]*m2[14] ;
  m[3] = m1[0]*m2[3] + m1[1]*m2[7] + m1[2]*m2[11] + m1[3]*m2[15] ;
  
  m[4] = m1[4]*m2[0] + m1[5]*m2[4] + m1[6]*m2[8] + m1[7]*m2[12] ;
  m[5] = m1[4]*m2[1] + m1[5]*m2[5] + m1[6]*m2[9] + m1[7]*m2[13] ;
  m[6] = m1[4]*m2[2] + m1[5]*m2[6] + m1[6]*m2[10] + m1[7]*m2[14] ;
  m[7] = m1[4]*m2[3] + m1[5]*m2[7] + m1[6]*m2[11] + m1[7]*m2[15] ;
  
  m[8] = m1[8]*m2[0] + m1[9]*m2[4] + m1[10]*m2[8] + m1[11]*m2[12] ;
  m[9] = m1[8]*m2[1] + m1[9]*m2[5] + m1[10]*m2[9] + m1[11]*m2[13] ;
  m[10] = m1[8]*m2[2] + m1[9]*m2[6] + m1[10]*m2[10] + m1[11]*m2[14] ;
  m[11] = m1[8]*m2[3] + m1[9]*m2[7] + m1[10]*m2[11] + m1[11]*m2[15] ;
  
  m[12] = m1[12]*m2[0] + m1[13]*m2[4] + m1[14]*m2[8] + m1[15]*m2[12] ;
  m[13] = m1[12]*m2[1] + m1[13]*m2[5] + m1[14]*m2[9] + m1[15]*m2[13] ;
  m[14] = m1[12]*m2[2] + m1[13]*m2[6] + m1[14]*m2[10] + m1[15]*m2[14] ;
  m[15] = m1[12]*m2[3] + m1[13]*m2[7] + m1[14]*m2[11] + m1[15]*m2[15] ;
#endif
  return M ;
}


/*!
  \brief The assignment operator with a matrix

  \param M  the matrix

  \return A reference to itself

  \warning The matrix \e must be of size \latexonly$4 \times 4$\endlatexonly 
           \htmlonly 4x4 \endhtmlonly

  \author Philippe Lavoie 
  \date 25 July, 1997
 */
template <class T>
MatrixRT<T>& MatrixRT<T>::operator=(const Matrix<T>& M) {
  if(M.rows() != 4 || M.cols() != 4){
    Error error("MatrixRT<T>::operator=") ;
    error << "Trying to assign with a matrix of dimensions" << 
      M.rows() << ' ' << M.cols() << endl ;
    error.fatal() ;
  }
  T *a,*b ;
  a = m-1 ;
  b = M[0] - 1 ;
  for(int i=0;i<16;++i){
    *(++a) = *(++b) ;
  }
  return *this ;
}

/*!
  \brief The assignment operator with a matrixRT.
  \param M the matrix
  \return A reference to itself
 
  \author Philippe Lavoie 
  \date 25 July, 1997
*/
template <class T>
MatrixRT<T>& MatrixRT<T>::operator=(const MatrixRT<T>& M) {
  T *a,*b ;
  a = m-1 ;
  b = M.m - 1 ;
  for(int i=0;i<16;++i){
    *(++a) = *(++b) ;
  }
  return *this ;
}



#ifdef NO_IMPLICIT_TEMPLATES

template class MatrixRT<float> ;
template class MatrixRT<double> ;

template Point_nD<float,3> operator*(const MatrixRT<float>& M, const Point_nD<float,3>& P) ;
template Point_nD<double,3> operator*(const MatrixRT<double>& M, const Point_nD<double,3>& P) ;
template HPoint_nD<float,3> operator*(const MatrixRT<float>& M, const HPoint_nD<float,3>& P) ;
template HPoint_nD<double,3> operator*(const MatrixRT<double>& M, const HPoint_nD<double,3>& P) ;
template MatrixRT<float> operator*(const MatrixRT<float>& M1, const MatrixRT<float>& M2) ;
template MatrixRT<double> operator*(const MatrixRT<double>& M1, const MatrixRT<double>& M2) ;

template Point_nD<float,2> operator*(const MatrixRT<float>& M, const Point_nD<float,2>& P) ;
template Point_nD<double,2> operator*(const MatrixRT<double>& M, const Point_nD<double,2>& P) ;
template HPoint_nD<float,2> operator*(const MatrixRT<float>& M, const HPoint_nD<float,2>& P) ;
template HPoint_nD<double,2> operator*(const MatrixRT<double>& M, const HPoint_nD<double,2>& P) ;


#else
#ifndef USING_VCC 

void stupidSparcMatrix(){
  Point_nD<float,3> a ;
  HPoint_nD<float,3> b ;
  MatrixRT<float> A ;
  MatrixRT<double> B ;
  a = A*a ;
  b = A*b ;
//  a = B*a ;
//  b = B*b ;
}
#endif // !USING_VCC 


#endif 

} // end namespace

