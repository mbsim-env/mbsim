/*============================================================================
        File: point_nd.h
     Purpose: 
    Revision: $Id: point_nd.h,v 1.2 2002/05/13 21:07:45 philosophil Exp $
  Created by: Philippe Lavoie          (26 January, 1996)
 Modified by: Martin Schuerch

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
#ifndef _Matrix_pointnD_h_
#define _Matrix_pointnD_h_

#include "matrix_global.h"
#include <memory.h>


namespace PLib {

  /*!
    \class Point_nD point_nd.h matrix/point_nd.h
    \brief A point in an \a n dimemsional space

    Defines a point in an n dimemsional space. It is assumed
    that a x,y and z value can be read and assigned to.

    \warning only N=2 and N=3 are specialized

    \author Philippe Lavoie (14 October, 1998)
  */
  template <class T, int N>
  struct Point_nD {
    //only the specialications make sense, because this class should only be used for
    // (float,2), (float,3) (double,2) (double,3)
  };


  template <>
  struct Point_nD<float,3> {
    typedef float T;
    T data[3] ;
    Point_nD()  { x() = y() = z() = 0 ;}
    Point_nD(T a)  { x() = y() = z() = a ;}
    Point_nD(T X, T Y, T Z)  {x()=X ; y()=Y ; z()=Z ;}
    Point_nD(const Point_nD& a) { memcpy((void*)data,(void*)a.data,3*sizeof(T));}

    inline T& x() { return data[0] ; }
    inline T& y() { return data[1] ; }
    inline T& z() { return data[2] ; }
    inline T x() const { return data[0] ; }
    inline T y() const { return data[1] ; }
    inline T z() const { return data[2] ; }
 
    Point_nD& operator=(const Point_nD& v) { x()=v.x() ; y()=v.y() ; z()=v.z() ;  return *this ;} ;
    Point_nD& operator+=(const Point_nD& v) {x()+=v.x() ; y()+= v.y() ; z()+=v.z() ;  return *this;} ;
    Point_nD& operator-=(const Point_nD& v) {x()-=v.x() ; y()-= v.y() ; z()-=v.z() ;  return *this;} ;
    Point_nD& operator*=(T v) {x()*=v ; y()*= v ; z()*= v;  return *this;} ;
    Point_nD& operator/=(T v) {x()/=v ; y()/= v ; z()/= v ; return *this;} ;

    Point_nD unitLength() const {  T d = norm(); Point_nD<T,3> u(x()/d,y()/d,z()/d); return u; }
    T norm2() const { return  data[0]*data[0] + data[1]*data[1] + data[2]*data[2]; }
    T norm() const { return sqrt( data[0]*data[0] + data[1]*data[1] + data[2]*data[2] ); }
  };


  template <>
  struct Point_nD<double,3> {
    typedef double T;
    T data[3] ;
    Point_nD()  { x() = y() = z() = 0 ;}
    Point_nD(T a)  { x() = y() = z() = a ;}
    Point_nD(T X, T Y, T Z)  {x()=X ; y()=Y ; z()=Z ;}
    Point_nD(const Point_nD& a) { memcpy((void*)data,(void*)a.data,3*sizeof(T));}

    inline T& x() { return data[0] ; }
    inline T& y() { return data[1] ; }
    inline T& z() { return data[2] ; }
    inline T x() const { return data[0] ; }
    inline T y() const { return data[1] ; }
    inline T z() const { return data[2] ; }
 
    Point_nD& operator=(const Point_nD& v) { x()=v.x() ; y()=v.y() ; z()=v.z() ;  return *this ;} ;
    Point_nD& operator+=(const Point_nD& v) {x()+=v.x() ; y()+= v.y() ; z()+=v.z() ;  return *this;} ;
    Point_nD& operator-=(const Point_nD& v) {x()-=v.x() ; y()-= v.y() ; z()-=v.z() ;  return *this;} ;
    Point_nD& operator*=(T v) {x()*=v ; y()*= v ; z()*= v;  return *this;} ;
    Point_nD& operator/=(T v) {x()/=v ; y()/= v ; z()/= v ; return *this;} ;

    Point_nD unitLength() const {  T d = norm(); Point_nD<T,3> u(x()/d,y()/d,z()/d); return u; }
    T norm2() const { return  data[0]*data[0] + data[1]*data[1] + data[2]*data[2]; }
    T norm() const { return sqrt( data[0]*data[0] + data[1]*data[1] + data[2]*data[2] ); }
  };


  template <>
  struct Point_nD<float,2> { 
    typedef float T;
    T data[2] ;
    Point_nD()  { x() = y() = 0 ;}
    Point_nD(T a)  { x() = y() = a ;}
    Point_nD(T X, T Y)  {x()=X ; y()=Y ; }
    Point_nD(const Point_nD<T,2>& a)  { memcpy((void*)data,(void*)a.data,2*sizeof(T));}

    inline T& x() { return data[0] ; }
    inline T& y() { return data[1] ; }
    inline T& z() { return dumbVar ; }
    inline T x() const { return data[0] ; }
    inline T y() const { return data[1] ; }
    inline T z() const { return T() ; }
 
    Point_nD& operator=(const Point_nD& v) { x()=v.x() ; y()=v.y() ; return *this ;} ;
    Point_nD& operator+=(const Point_nD& v) {x()+=v.x() ; y()+= v.y() ; return *this;} ;
    Point_nD& operator-=(const Point_nD& v) {x()-=v.x() ; y()-= v.y() ;   return *this;} ;
    Point_nD& operator*=(T v) {x()*=v ; y()*= v ; return *this;} ;
    Point_nD& operator/=(T v) {x()/=v ; y()/= v ; return *this;} ;
 
    Point_nD unitLength() const { T d = norm(); Point_nD<T,2> u(x()/d,y()/d); return u;}
    T norm2() const { return  data[0]*data[0] + data[1]*data[1]; }
    T norm() const { return sqrt( data[0]*data[0] + data[1]*data[1] ); }

  protected:
    static T dumbVar ; 
  };


  template <>
  struct Point_nD<double,2> { 
    typedef double T;
    T data[2] ;
    Point_nD()  { x() = y() = 0 ;}
    Point_nD(T a)  { x() = y() = a ;}
    Point_nD(T X, T Y)  {x()=X ; y()=Y ; }
    Point_nD(const Point_nD<T,2>& a)  { memcpy((void*)data,(void*)a.data,2*sizeof(T));}

    inline T& x() { return data[0] ; }
    inline T& y() { return data[1] ; }
    inline T& z() { return dumbVar ; }
    inline T x() const { return data[0] ; }
    inline T y() const { return data[1] ; }
    inline T z() const { return T() ; }
 
    Point_nD& operator=(const Point_nD& v) { x()=v.x() ; y()=v.y() ; return *this ;} ;
    Point_nD& operator+=(const Point_nD& v) {x()+=v.x() ; y()+= v.y() ; return *this;} ;
    Point_nD& operator-=(const Point_nD& v) {x()-=v.x() ; y()-= v.y() ;   return *this;} ;
    Point_nD& operator*=(T v) {x()*=v ; y()*= v ; return *this;} ;
    Point_nD& operator/=(T v) {x()/=v ; y()/= v ; return *this;} ;
 
    Point_nD unitLength() const { T d = norm(); Point_nD<T,2> u(x()/d,y()/d); return u;}
    T norm2() const { return  data[0]*data[0] + data[1]*data[1]; }
    T norm() const { return sqrt( data[0]*data[0] + data[1]*data[1] ); }

  protected:
    static T dumbVar ; 
  };



  template <class T>
  inline int operator<(const Point_nD<T,3>& a, const Point_nD<T,3>& b) { 
    return a.x()<b.x() || a.y()<b.y() || a.z()<b.z() ;}

  template <class T>
  inline int operator>(const Point_nD<T,3>& a, const Point_nD<T,3>& b) { 
    return a.x()>b.x() || a.y()>b.y() || a.z()>b.z() ;}

  template <class T>
  inline int operator<=(const Point_nD<T,3>& a, const Point_nD<T,3>& b) { 
    return a.x()<=b.x() || a.y()<=b.y() || a.z()<=b.z() ;}

  template <class T>
  inline int operator>=(const Point_nD<T,3>& a, const Point_nD<T,3>& b) { 
    return a.x()>=b.x() || a.y()>=b.y() || a.z()>=b.z() ;}



  template <class T>
  inline int operator<(const Point_nD<T,2>& a, const Point_nD<T,2>& b) { 
    return a.x()<b.x() || a.y()<b.y() ;}

  template <class T>
  inline int operator>(const Point_nD<T,2>& a, const Point_nD<T,2>& b) { 
    return a.x()>b.x() || a.y()>b.y() ;}

  template <class T>
  inline int operator<=(const Point_nD<T,2>& a, const Point_nD<T,2>& b) { 
    return a.x()<=b.x() || a.y()<=b.y()  ;}

  template <class T>
  inline int operator>=(const Point_nD<T,2>& a, const Point_nD<T,2>& b) { 
    return a.x()>=b.x() || a.y()>=b.y() ;}








  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: operator* --- multiplies a point in 3D with a float
     Multiplies a point in 3D with a float
          Input: a --> the floating point value
	         b --> the point in 3D
         Output: $ab$
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T>
  inline Point_nD<T,3> operator*(const T a,const Point_nD<T,3>& b) {
    Point_nD<T,3> mul(b.x()*a,b.y()*a,b.z()*a) ;
    return mul ;
  }

  inline Point_nD<float,3> operator*(const double a,const Point_nD<float,3>& b) {
    Point_nD<float,3> mul(b.x()*a,b.y()*a,b.z()*a) ;
    return mul ;
  }

  template <class T>
  inline Point_nD<T,2> operator*(const T a,const Point_nD<T,2>& b) {
    Point_nD<T,2> mul(b.x()*a,b.y()*a) ;
    return mul ;
  }

  inline Point_nD<float,2> operator*(const double a,const Point_nD<float,2>& b) {
    Point_nD<float,2> mul(b.x()*a,b.y()*a) ;
    return mul ;
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: operator* --- multiplies a point in 3D with a float
     Multiplies a point in 3D with a float
          Input: a --> the floating point value
	         b --> the point in 3D
         Output: $ab$
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T>
  inline Point_nD<T,3> operator*(const Point_nD<T,3>& b,const T a) {
    Point_nD<T,3> mul(b.x()*a,b.y()*a,b.z()*a) ;
    return mul ;
  }

  inline Point_nD<float,3> operator*(const Point_nD<float,3>& b,const double a) {
    Point_nD<float,3> mul(b.x()*a,b.y()*a,b.z()*a) ;
    return mul ;
  }

  template <class T>
  inline Point_nD<T,2> operator*(const Point_nD<T,2>& b,const T a) {
    Point_nD<T,2> mul(b.x()*a,b.y()*a) ;
    return mul ;
  }

  inline Point_nD<float,2> operator*(const Point_nD<float,2>& b,const double a) {
    Point_nD<float,2> mul(b.x()*a,b.y()*a) ;
    return mul ;
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: operator/ --- Divide a point in 3D by a float
     Divide a point in 3D by a float.
          Input: a --> the point in 3D
	         b --> the floating point value to divide with
         Output: $a/b$
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T>
  inline Point_nD<T,3> operator/(const Point_nD<T,3>& a,const T b) {
    Point_nD<T,3> div(a.x()/b,a.y()/b,a.z()/b) ;
    return div ;
  }

  inline Point_nD<float,3> operator/(const Point_nD<float,3>& a,const double b) {
    Point_nD<float,3> div(a.x()/b,a.y()/b,a.z()/b) ;
    return div ;
  }

  template <class T>
  inline Point_nD<T,2> operator/(const Point_nD<T,2>& a,const T b) {
    Point_nD<T,2> div(a.x()/b,a.y()/b) ;
    return div ;
  }

  inline Point_nD<float,2> operator/(const Point_nD<float,2>& a,const double b) {
    Point_nD<float,2> div(a.x()/b,a.y()/b) ;
    return div ;
  }

  /*!
    \fn T dot(const Point_nD<T,3>& a,const Point_nD<T,3>& b)
    \brief the dot product of two points in 3D

     The dot product of two points in 3D

     \param a  the first point in 3D
     \param b  the second point in 3D
     \return  \a a.b
     \author Philippe Lavoie 
     \date 24 January 1997
  */
  template <class T>
  inline T dot(const Point_nD<T,3>& a,const Point_nD<T,3>& b) {
    return a.x()*b.x() + a.y()*b.y() + a.z()*b.z() ;
  }

  /*!
    \fn T dot(const Point_nD<T,2>& a,const Point_nD<T,2>& b)
    \brief the dot product of two points in 2D

     The dot product of two points in 2D

     \param a  the first point in 2D
     \param b  the second point in 2D
     \return \a a.b
     \author Philippe Lavoie 
     \date 24 January 1997
  */
  template <class T>
  inline T dot(const Point_nD<T,2>& a,const Point_nD<T,2>& b) {
    return a.x()*b.x() + a.y()*b.y() ;
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: operator* --- the dot product of two points in 3D
     The dot product of two points in 3D
          Input: a --> the first point in 3D
	         b --> the second point in 3D
         Output: $a.b$
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T>
  inline T operator*(const Point_nD<T,3>& a,const Point_nD<T,3>& b) {
    return a.x()*b.x() + a.y()*b.y() + a.z()*b.z() ;
  }

  template <class T>
  inline T operator*(const Point_nD<T,2>& a,const Point_nD<T,2>& b) {
    return a.x()*b.x() + a.y()*b.y() ;
  }



  // Point3D definitions

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: operator+ --- the addition operator with points in 3D
     The addition operator with points in 3D
          Input: a --> the first point in 3D
	         b --> the second point in 3D
         Output: $a+b$
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T, int D>
  inline Point_nD<T,D> operator+(const Point_nD<T,D>& a,const Point_nD<T,D>& b) {
    Point_nD<T,D> sum(a) ;
    sum += b ;
    return sum ;
  }


  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: operator- --- the subtraction operator with points in 3D
     The subtraction operator with points in 3D
          Input: a --> the first point in 3D
	         b --> the second point in 3D
         Output: $a-b$
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T, int D>
  inline Point_nD<T,D> operator-(const Point_nD<T,D>& a,const Point_nD<T,D>& b) {
    Point_nD<T,D> diff(a) ;
    diff -= b ;
    return diff ;
  }


  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: operator== --- the equality operator with a float
     Finds if all the elements of the point in 3D are equal to $b$
          Input: a --> the point in 3D
	         b --> the floating point value
         Output: 1 if equal, 0 otherwise
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T>
  inline int operator==(const Point_nD<T,3>&a, float b) {
    if(a.x() == b && a.y() == b && a.z()==b)
      return 1 ;
    return 0 ;
  }

  template <class T>
  inline int operator==(const Point_nD<T,2>&a, float b) {
    if(a.x() == b && a.y() == b )
      return 1 ;
    return 0 ;
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: operator!= --- the inequality operator between points in 3D
     The inequality operator between points in 3D.
          Input: a --> the first point in 3D
	         b --> the second point in 3D
         Output: 1 if they are different, 0 otherwise
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T>
  inline int operator!=(const Point_nD<T,3>& a, const Point_nD<T,3>& b){
    if(a.x() == b.x() && a.y() == b.y() && a.z() == b.z())
      return 0 ;
    else
      return 1 ;
  }

  template <class T>
  inline int operator!=(const Point_nD<T,2>& a, const Point_nD<T,2>& b){
    if(a.x() == b.x() && a.y() == b.y() )
      return 0 ;
    else
      return 1 ;
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: operator== --- the equality operator between points in 3D
     The equality operator between points in 3D.
          Input: a --> the first point in 3D
	         b --> the second point in 3D
         Output: 1 if they are equal, 0 otherwise
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T>
  inline int operator==(const Point_nD<T,3>& a, const Point_nD<T,3>& b){
    if(a.x() == b.x() && a.y() == b.y() && a.z() == b.z())
      return 1 ;
    else
      return 0 ;
  }

  template <class T>
  inline int operator==(const Point_nD<T,2>& a, const Point_nD<T,2>& b){
    if(a.x() == b.x() && a.y() == b.y() )
      return 1 ;
    else
      return 0 ;
  }



  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: norm2 --- the sum of the square of all the elements of a point
     The sum of the square of all the elements of a point or
                 the length squared of the vector in 3D.
          Input: a --> the point
         Output: $a_x^2+a_y^2+a_z^2$
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T,int N>
  inline double norm2(const Point_nD<T,N>& a){
    double temp = 0 ;
    for(int i=N-1;i>=0;--i)
      temp += a.data[i]*a.data[i] ;
    return temp ;
  }

  template <class T,int N> 
  inline double norm(const Point_nD<T,N>& a) { return sqrt(norm2<T,N>(a)); }



  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: angle --- Finds the angle between two points in 3D
     Finds the angle between two points in 3D
          Input: a --> the first point in 3D
	         b --> the second point in 3D
         Output: The angle in radian between the two points
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T, int D>
  inline T angle(const Point_nD<T,D>& a,const Point_nD<T,D>& b) {
    if(b==0 || a==0 )
      return 0 ;
    return acos(dot(a,b)/norm(a)/norm(b)) ;
  }


  template <class T>
  inline Point_nD<T,3> crossProduct(const Point_nD<T,3>& a, const Point_nD<T,3>& b){
    Point_nD<T,3> prod ;
    prod.x() = a.y()*b.z() - a.z()*b.y() ;
    prod.y() = a.z()*b.x() - a.x()*b.z() ;
    prod.z() = a.x()*b.y() - a.y()*b.x() ;
    return prod ;
  }


  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: operator<< ---  the output operator of a point in 3D 
                                 to an ostream
     The output operator of a point in 3D to an ostream .
          Input:  os --> the ostream
               point --> the point to output
         Output: the ostream with the point 
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T>
  inline ostream& operator<<(ostream& os,const Point_nD<T,3>& point)
  {
    os << " " << point.x() << " " << point.y() << " " << point.z() << " " ;
    return os;	
  }

  template <class T>
  inline ostream& operator<<(ostream& os,const Point_nD<T,2>& point)
  {
    os << " " << point.x() << " " << point.y() << " " ;
    return os;	
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: operator>> --- the input operator from an istream
     Initialize a point in 3D from the input stream.
          Input:  os --> the input stream
               point <-- the point to initialize
         Output: the istream without the point
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T>
  inline istream& operator>>(istream& os, Point_nD<T,3>& point){
    float x,y,z ;
    os >> x >> y >> z ;
    point.x() = x ;
    point.y() = y ; 
    point.z() = z ;
    return os ;
  }

  template <class T>
  inline istream& operator>>(istream& os, Point_nD<T,2>& point){
    float x,y ;
    os >> x >> y  ;
    point.x() = x ;
    point.y() = y ; 
    return os ;
  }


  template <class T> T minimum(T a, T b);
  template <class T> T maximum(T a, T b);

  inline Point_nD<float,3> minimum(Point_nD<float,3> a, Point_nD<float,3>  b){
    Point_nD<float,3> m ;
    m.x() = minimum(a.x(),b.x()) ;
    m.y() = minimum(a.y(),b.y()) ;
    m.z() = minimum(a.z(),b.z()) ;
    return m ; 
  }

  inline Point_nD<double,3> minimum(Point_nD<double,3> a, Point_nD<double,3>  b){
    Point_nD<double,3> m ;
    m.x() = minimum(a.x(),b.x()) ;
    m.y() = minimum(a.y(),b.y()) ;
    m.z() = minimum(a.z(),b.z()) ;
    return m ; 
  }


  inline Point_nD<float,2> minimum(Point_nD<float,2> a, Point_nD<float,2>  b){
    Point_nD<float,2> m ;
    m.x() = minimum(a.x(),b.x()) ;
    m.y() = minimum(a.y(),b.y()) ;
    return m ; 
  }


  inline Point_nD<double,2> minimum(Point_nD<double,2> a, Point_nD<double,2>  b){
    Point_nD<double,2> m ;
    m.x() = minimum(a.x(),b.x()) ;
    m.y() = minimum(a.y(),b.y()) ;
    return m ; 
  }

  inline Point_nD<float,3> maximum(Point_nD<float,3> a,Point_nD<float,3>  b){
    Point_nD<float,3> m ;
    m.x() = maximum(a.x(),b.x()) ;
    m.y() = maximum(a.y(),b.y()) ;
    m.z() = maximum(a.z(),b.z()) ;
    return m ; 
  }

  inline Point_nD<double,3> maximum(Point_nD<double,3> a,Point_nD<double,3>  b){
    Point_nD<double,3> m ;
    m.x() = maximum(a.x(),b.x()) ;
    m.y() = maximum(a.y(),b.y()) ;
    m.z() = maximum(a.z(),b.z()) ;
    return m ; 
  }


  inline Point_nD<float,2> maximum(Point_nD<float,2> a,Point_nD<float,2>  b){
    Point_nD<float,2> m ;
    m.x() = maximum(a.x(),b.x()) ;
    m.y() = maximum(a.y(),b.y()) ;
    return m ; 
  }

  inline Point_nD<double,2> maximum(Point_nD<double,2> a,Point_nD<double,2>  b){
    Point_nD<double,2> m ;
    m.x() = maximum(a.x(),b.x()) ;
    m.y() = maximum(a.y(),b.y()) ;
    return m ; 
  }


  template <class T>
  inline Point_nD<T,3> minimumByRef(const Point_nD<T,3> &a,const Point_nD<T,3>  &b){
    Point_nD<T,3> m ;
    m.x() = minimum(a.x(),b.x()) ;
    m.y() = minimum(a.y(),b.y()) ;
    m.z() = minimum(a.z(),b.z()) ;
    return m ; 
  }


  template <class T>
  inline Point_nD<T,2> minimumByRef(const Point_nD<T,2> &a,const Point_nD<T,2>  &b){
    Point_nD<T,2> m ;
    m.x() = minimum(a.x(),b.x()) ;
    m.y() = minimum(a.y(),b.y()) ;
    return m ; 
  }


  template <class T>
  inline Point_nD<T,3> maximumByRef(const Point_nD<T,3> &a,const Point_nD<T,3>  &b){
    Point_nD<T,3> m ;
    m.x() = maximum(a.x(),b.x()) ;
    m.y() = maximum(a.y(),b.y()) ;
    m.z() = maximum(a.z(),b.z()) ;
    return m ; 
  }

  typedef Point_nD<float,3> Point3Df ;
  typedef Point_nD<double,3> Point3Dd ;

  typedef Point_nD<float,2> Point2Df ;
  typedef Point_nD<double,2> Point2Dd ;



} // end namespace

typedef PLib::Point_nD<float,3> PlPoint3Df ;
typedef PLib::Point_nD<double,3> PlPoint3Dd ;

typedef PLib::Point_nD<float,2> PlPoint2Df ;
typedef PLib::Point_nD<double,2> PlPoint2Dd ;



#endif
