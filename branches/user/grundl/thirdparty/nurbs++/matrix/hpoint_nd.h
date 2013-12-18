/*============================================================================
        File: hpoint_nd.h
     Purpose: 
    Revision: $Id: hpoint_nd.h,v 1.3 2002/05/22 17:06:47 philosophil Exp $
  Created by: Philippe Lavoie          (26 January 1999)
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
#ifndef _Matrix_hpointnD_h_
#define _Matrix_hpointnD_h_

#include "matrix_global.h"
#include "point_nd.h"


namespace PLib {

  /*!
    \class HPoint_nD hpoint_nd.h matrix/hpoint_nd.h
    \brief A class for point in the homogeneous space

    When using B-splines, homogeneous points are used. They are 
    defined with n+1 dimension points such as $(wx,wy,wz,w)$. 
    In 3D space, a homogenouse point is equivalent to 
    \latexonly
    $(\frac{x}{w},\frac{y}{w},\frac{z}{w})$.
    \endlatexonly
    \htmlonly
    (x/w,y/w,z/w).
    \endhtmlonly
    
    This class also defines basic mathematical operators for  
    homogeneous points.


    \warning Only classes for the case of N=2 or N=3 are defined.
        
    \author Philippe Lavoie 
    \date 4 October 1996
  */
  template <class T, int N>
  struct HPoint_nD {
    // only N=2,3 are defined for now
    //HPoint_nD() = 0 ; 
  };



  template <>
  struct HPoint_nD<float,3> {
    typedef float T;
    T *data ; 
    int created; // usefull to change the data pointer

    HPoint_nD(): data(new T[4]), created(1) { x() = y() = z() = w() = T(); }
    HPoint_nD(T *d,int c): data(d), created(c) { ; } 
    HPoint_nD(T a): data(new T[4]), created(1) { x() = y() = z() = w() = a; }
    HPoint_nD(T X, T Y, T Z, T W):  data(new T[4]), created(1) { x()=X ; y()=Y; z()=Z; w()=W;}
    HPoint_nD(const HPoint_nD& a): data(new T[4]), created(1) { memcpy((void*)data,(void*)a.data,4*sizeof(T)); }
    HPoint_nD(const Point_nD<T,3>& a): data(new T[4]), created(1) { memcpy((void*)data,(void*)a.data,(3)*sizeof(T));  data[3] = T(1); }
    ~HPoint_nD() { if(created) if(data) delete []data ; }

    inline T& x() { return data[0] ;}
    inline T x() const { return data[0] ; }
    inline T& y() { return data[1] ;}
    inline T y() const { return data[1] ;}
    inline T& z() { return data[2] ;}
    inline T z() const { return data[2] ;}
    inline T& w() { return data[3] ;}
    inline T w() const { return data[3] ;}
  
    HPoint_nD& operator=(const HPoint_nD& v) { data[0]=v.data[0]; data[1]=v.data[1]; data[2]=v.data[2]; data[3]=v.data[3]; return *this;}
    HPoint_nD& operator=(const Point_nD<T,3>& v) {x()=v.x() ; y()=v.y() ; z()=v.z() ; w()=1.0 ; return *this;}
    HPoint_nD& operator=(const T v) { x() = y() = z() = w() = v ; return *this ;} ;
    HPoint_nD& operator+=(const HPoint_nD& v) { data[0]+=v.data[0]; data[1]+=v.data[1]; data[2]+=v.data[2]; data[3]+=v.data[3]; return *this;}
    HPoint_nD& operator-=(const HPoint_nD& v) { data[0]-=v.data[0]; data[1]-=v.data[1]; data[2]-=v.data[2]; data[3]-=v.data[3]; return *this;}

    HPoint_nD& operator*=(T v) { data[0]*=v; data[1]*=v; data[2]*=v; data[3]*=v; return *this; }
    HPoint_nD& operator/=(T v) { data[0]/=v; data[1]/=v; data[2]/=v; data[3]/=v; return *this; }

    void move(const Point_nD<T,3>& m) {
      for(int i=3-1;i>=0;--i) {
        data[i] += m.data[i]*data[3];
      }
    }
    Point_nD<T,3> projectW() { return Point_nD<T,3>(x(),y(),z()) ; }
  };


  template <>
  struct HPoint_nD<double,3> {
    typedef double T;
    T *data ; 
    int created; // usefull to change the data pointer

    HPoint_nD(): data(new T[4]), created(1) { x() = y() = z() = w() = T(); }
    HPoint_nD(T *d,int c): data(d), created(c) { ; } 
    HPoint_nD(T a): data(new T[4]), created(1) { x() = y() = z() = w() = a; }
    HPoint_nD(T X, T Y, T Z, T W):  data(new T[4]), created(1) { x()=X ; y()=Y; z()=Z; w()=W;}
    HPoint_nD(const HPoint_nD& a): data(new T[4]), created(1) { memcpy((void*)data,(void*)a.data,4*sizeof(T)); }
    HPoint_nD(const Point_nD<T,3>& a): data(new T[4]), created(1) { memcpy((void*)data,(void*)a.data,(3)*sizeof(T));  data[3] = T(1); }
    ~HPoint_nD() { if(created) if(data) delete []data ; }

    inline T& x() { return data[0] ;}
    inline T x() const { return data[0] ; }
    inline T& y() { return data[1] ;}
    inline T y() const { return data[1] ;}
    inline T& z() { return data[2] ;}
    inline T z() const { return data[2] ;}
    inline T& w() { return data[3] ;}
    inline T w() const { return data[3] ;}
  
    HPoint_nD& operator=(const HPoint_nD& v) { data[0]=v.data[0]; data[1]=v.data[1]; data[2]=v.data[2]; data[3]=v.data[3]; return *this;}
    HPoint_nD& operator=(const Point_nD<T,3>& v) {x()=v.x() ; y()=v.y() ; z()=v.z() ; w()=1.0 ; return *this;}
    HPoint_nD& operator=(const T v) { x() = y() = z() = w() = v ; return *this ;} ;
    HPoint_nD& operator+=(const HPoint_nD& v) { data[0]+=v.data[0]; data[1]+=v.data[1]; data[2]+=v.data[2]; data[3]+=v.data[3]; return *this;}
    HPoint_nD& operator-=(const HPoint_nD& v) { data[0]-=v.data[0]; data[1]-=v.data[1]; data[2]-=v.data[2]; data[3]-=v.data[3]; return *this;}

    HPoint_nD& operator*=(T v) { data[0]*=v; data[1]*=v; data[2]*=v; data[3]*=v; return *this; }
    HPoint_nD& operator/=(T v) { data[0]/=v; data[1]/=v; data[2]/=v; data[3]/=v; return *this; }

    void move(const Point_nD<T,3>& m) {
      for(int i=3-1;i>=0;--i) {
        data[i] += m.data[i]*data[3];
      }
    }
    Point_nD<T,3> projectW() { return Point_nD<T,3>(x(),y(),z()) ; }
  };





  /**  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **
          class HPoint_2D --- A class for point in the homogeneous 3D space
     When using B-splines, homogeneous points are used. They are 
                 defined with 3D points as $(x,y,w)$. In 2D space, a 3D point 
	         is equivalent to $(\frac{x}{w},\frac{y}{w})$.

	         This class also defines basic mathematical operators for 3D 
	         homogeneous points.
     author Philippe Lavoie (14 October, 1998)
    Modified by:
   **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **/
  template <>
  struct HPoint_nD<float,2> {
    typedef float T;
    T *data ; 
    int created; // usefull to change the data pointer

    HPoint_nD(): data(new T[3]), created(1) { x() = y() = w() = T(); }
    HPoint_nD(T *d,int c): data(d), created(c) { ; } 
    HPoint_nD(T a): data(new T[3]), created(1) { x() = y() = w() = a; }
    HPoint_nD(T X, T Y, T W):  data(new T[3]), created(1) { x()=X ; y()=Y; w()=W;}
    HPoint_nD(T X, T Y, T Z, T W):  data(new T[3]), created(1) { x()=X ; y()=Y; w()=W;}
    HPoint_nD(const HPoint_nD& a): data(new T[3]), created(1) { memcpy((void*)data,(void*)a.data,3*sizeof(T)); }
    HPoint_nD(const Point_nD<T,2>& a): data(new T[3]), created(1) { memcpy((void*)data,(void*)a.data,(2)*sizeof(T));  data[2] = T(1); }
    ~HPoint_nD() { if(created) if(data) delete []data ; }

    inline T& x() { return data[0] ;}
    inline T x() const { return data[0] ; }
    inline T& y() { return data[1] ;}
    inline T y() const { return data[1] ;}
    inline T& z() { return  dumbVar;}
    inline T z() const { return T(0) ;}
    inline T& w() { return data[2] ;}
    inline T w() const { return data[2] ;}
  

    HPoint_nD<T,2>& operator=(const HPoint_nD<T,2>& v) { data[0]=v.data[0]; data[1]=v.data[1]; data[2]=v.data[2]; return *this;}
    HPoint_nD<T,2>& operator=(const Point_nD<T,2>& v) {x()=v.x() ; y()=v.y() ; w()=1.0 ; return *this;}
    HPoint_nD<T,2>& operator=(const T v) { x() = y() = w() = v ; return *this ;} ;

    HPoint_nD<T,2>& operator*=(T v) { data[0]*=v; data[1]*=v; data[2]*=v; return *this; }
    HPoint_nD<T,2>& operator/=(T v) { data[0]/=v; data[1]/=v; data[2]/=v; return *this; }

    HPoint_nD<T,2>& operator-=(const HPoint_nD<T,2>& v) { data[0]-=v.data[0]; data[1]-=v.data[1]; data[2]-=v.data[2]; return *this;}
    HPoint_nD<T,2>& operator+=(const HPoint_nD<T,2>& v) { data[0]+=v.data[0]; data[1]+=v.data[1]; data[2]+=v.data[2]; return *this;}

  void move(const Point_nD<T,2>& m) {
    for(int i=2-1;i>=0;--i) {
      data[i] += m.data[i]*data[2];
    }
  }
  Point_nD<T,2> projectW() { return Point_nD<T,2>(x(),y()) ; }

  protected:
    static T dumbVar ;
  };


  template <>
  struct HPoint_nD<double,2> {
    typedef double T;
    T *data ; 
    int created; // usefull to change the data pointer

    HPoint_nD(): data(new T[3]), created(1) { x() = y() = w() = T(); }
    HPoint_nD(T *d,int c): data(d), created(c) { ; } 
    HPoint_nD(T a): data(new T[3]), created(1) { x() = y() = w() = a; }
    HPoint_nD(T X, T Y, T W):  data(new T[3]), created(1) { x()=X ; y()=Y; w()=W;}
    HPoint_nD(T X, T Y, T Z, T W):  data(new T[3]), created(1) { x()=X ; y()=Y; w()=W;}
    HPoint_nD(const HPoint_nD& a): data(new T[3]), created(1) { memcpy((void*)data,(void*)a.data,3*sizeof(T)); }
    HPoint_nD(const Point_nD<T,2>& a): data(new T[3]), created(1) { memcpy((void*)data,(void*)a.data,(2)*sizeof(T));  data[2] = T(1); }
    ~HPoint_nD() { if(created) if(data) delete []data ; }

    inline T& x() { return data[0] ;}
    inline T x() const { return data[0] ; }
    inline T& y() { return data[1] ;}
    inline T y() const { return data[1] ;}
    inline T& z() { return  dumbVar;}
    inline T z() const { return T(0) ;}
    inline T& w() { return data[2] ;}
    inline T w() const { return data[2] ;}
  

    HPoint_nD<T,2>& operator=(const HPoint_nD<T,2>& v) { data[0]=v.data[0]; data[1]=v.data[1]; data[2]=v.data[2]; return *this;}
    HPoint_nD<T,2>& operator=(const Point_nD<T,2>& v) {x()=v.x() ; y()=v.y() ; w()=1.0 ; return *this;}
    HPoint_nD<T,2>& operator=(const T v) { x() = y() = w() = v ; return *this ;} ;

    HPoint_nD<T,2>& operator*=(T v) { data[0]*=v; data[1]*=v; data[2]*=v; return *this; }
    HPoint_nD<T,2>& operator/=(T v) { data[0]/=v; data[1]/=v; data[2]/=v; return *this; }

    HPoint_nD<T,2>& operator-=(const HPoint_nD<T,2>& v) { data[0]-=v.data[0]; data[1]-=v.data[1]; data[2]-=v.data[2]; return *this;}
    HPoint_nD<T,2>& operator+=(const HPoint_nD<T,2>& v) { data[0]+=v.data[0]; data[1]+=v.data[1]; data[2]+=v.data[2]; return *this;}

    void move(const Point_nD<T,2>& m) {
      for(int i=2-1;i>=0;--i) {
        data[i] += m.data[i]*data[2];
      }
    }

    Point_nD<T,2> projectW() { return Point_nD<T,2>(x(),y()) ; }

  protected:
    static T dumbVar ;
  };



  /*!
    \class NoInitHPoint_3D hpoint_nd.h matrix/hpoint_nd.h
    \brief An unitialized HPoint_3D

    Suppose you don't want to call the default constructor
    for a HPoint_3D. What do you do? Well you use this class.
    This is only usefull in arrays and the likes.

    \author Philippe Lavoie 
    \date 16 July 1998
  */
  template <class T, const int D>
  class NoInitHPoint_nD : public HPoint_nD<T,D> {
  public:
    NoInitHPoint_nD() : HPoint_nD<T,D>(0,0) { ; }
  };


  /*!
    \class NoInitHPoint_2D  hpoint_nd.h matrix/hpoint_nd.h
    \brief An unitialized HPoint_3D

    Suppose you don't want to call the default constructor
    for a HPoint_2D. What do you do? Well you use this class.
    This is only usefull in arrays and the likes.

    \author Philippe Lavoie 
    \date 16 July 1998
  */
  template <class T>
  class NoInitHPoint_2D : public HPoint_nD<T,2> {
  public:
    NoInitHPoint_2D() : HPoint_nD<T,2>(0,0) { ; }
  };



  /*!
    \fn HPoint_nD<T,D> operator+(const HPoint_nD<T,D>&a,const HPoint_nD<T,D>&b)
    \brief the addition operator for a HPoint_nD

    The addition operator for HPoint_3D

    \param a  the first point
    \param b  the second point
    \return $a+b$

    \author Philippe Lavoie 
    \date 24 January 1997
  */
  template <class T, int D>
  inline HPoint_nD<T,D> operator+(const HPoint_nD<T,D>&a,const HPoint_nD<T,D>&b) {
    HPoint_nD<T,D> sum(a) ;
    sum += b ;
    return sum ;
  }


  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: operator- --- the substraction operator for a HPoint_3D
     The substraction operator for a HPoint_3D
          Input: a --> the first vector
                 b --> the second vector
         Output: $a-b$
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T, int D>
  inline HPoint_nD<T,D> operator-(const HPoint_nD<T,D>&a,const HPoint_nD<T,D>&b) {
    HPoint_nD<T,D> diff(a) ;
    diff -= b ;
    return diff ;
  }


  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: dot --- The dot product between two points
     The dot product between two points in 4D.
          Input: a --> the first vector
                 b --> the second vector
         Output: $a.b$
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T>
  inline T dot(const HPoint_nD<T,3>&a,const HPoint_nD<T,3>&b) {
    return a.x()*b.x()+a.y()*b.y()+a.z()*b.z()+a.w()*b.w() ;
  }

  template <class T>
  inline T dot(const HPoint_nD<T,2>&a,const HPoint_nD<T,2>&b) {
    return a.x()*b.x()+a.y()*b.y()+a.w()*b.w() ;
  }


  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: operator* --- multiplying a point with a float
     Multiplying a point with a float
          Input: b --> the point to multiply
	         a --> the floating point value to multiply with
         Output: $ab$
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T>
  inline HPoint_nD<T,3> operator*(const HPoint_nD<T,3>&b, T a) {
    HPoint_nD<T,3> mul(b.x()*a,b.y()*a,b.z()*a,b.w()*a) ;
    return mul;
  }

  template <class T>
  inline HPoint_nD<T,2> operator*(const HPoint_nD<T,2>&b, T a) {
    HPoint_nD<T,3> mul(b.x()*a,b.y()*a,b.w()*a) ;
    return mul;
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: operator* --- multiplying a point with a float
     Multiplying a point with a float
          Input: a --> the floating point value to multiply with
                 b --> the point to multiply
         Output: $ab$
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T>
  inline HPoint_nD<T,3> operator*( T a, const HPoint_nD<T,3>&b) {
    HPoint_nD<T,3> mul(b.x()*a,b.y()*a,b.z()*a,b.w()*a) ;
    return mul;
    // the following code isn't as good because no convresion of double and float are made
    //HPoint_nD<T,N> mul(b) ;
    //return mul *= a;
  }

  inline HPoint_nD<float,3> operator*( double a, const HPoint_nD<float,3>&b) {
    HPoint_nD<float,3> mul(b.x()*a,b.y()*a,b.z()*a,b.w()*a) ;
    return mul;
  }


  template <class T>
  inline HPoint_nD<T,2> operator*(T a, const HPoint_nD<T,2>&b) {
    HPoint_nD<T,2> mul(b.x()*a,b.y()*a,b.w()*a) ;
    return mul;
  }

  inline HPoint_nD<float,2> operator*(double a, const HPoint_nD<float,2>&b) {
    HPoint_nD<float,2> mul(b.x()*a,b.y()*a,b.w()*a) ;
    return mul;
  }
  

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: operator* --- The dot product between two points
     The dot product between two points in 4D.
          Input: a --> the first vector
                 b --> the second vector
         Output: $a.b$
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T>
  inline T operator*(const HPoint_nD<T,3>&a,const HPoint_nD<T,3>&b) {
    return a.x()*b.x()+a.y()*b.y()+a.z()*b.z()+a.w()*b.w() ;
  }

  template <class T>
  inline T operator*(const HPoint_nD<T,2>&a,const HPoint_nD<T,2>&b) {
    return a.x()*b.x()+a.y()*b.y()+a.w()*b.w() ;
  }



   /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: operator/ --- Divide a point by a float
     Divide a point by a float
          Input: a --> the point in 4D
                 b --> the floating point value
         Output: $a/b$
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T>
  inline HPoint_nD<T,3> operator/(const HPoint_nD<T,3>&a,const T b) {
    HPoint_nD<T,3> div(a.x()/b,a.y()/b,a.z()/b,a.w()/b) ;
    return div ;
  }

  inline HPoint_nD<float,3> operator/(const HPoint_nD<float,3>&a,const double b) {
    HPoint_nD<float,3> div(float(a.x()/b),float(a.y()/b),float(a.z()/b),float(a.w()/b)) ;
    return div ;
  }

  template <class T>
  inline HPoint_nD<T,2> operator/(const HPoint_nD<T,2>&a,const T b) {
    HPoint_nD<T,2> div(a.x()/b,a.y()/b,a.w()/b) ;
    return div ;
  }

  inline HPoint_nD<float,2> operator/(const HPoint_nD<float,2>&a,const double b) {
    HPoint_nD<float,2> div(float(a.x()/b),float(a.y()/b),float(a.w()/b)) ;
    return div ;
  }


  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: operator== --- The equality operator with a float
     The equality operator with a float. Verifies if every elements
                 of $a$ are equal to $b$.
          Input: a --> the control point
	         b --> the floating point value
         Output: 1 if every elements of $a$ is equal to $b$, 0 otherwise
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T, int D>
  inline int operator==(const HPoint_nD<T,D>& a, T b) {
    int r ;
    r = 1 ;
    for(int i=3;i>=0;--i)
      r = r && a.data[i]==b ;
    return r ;
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: operator!= --- the inequality operator between two points
     The inequality operator between two points
          Input: a --> the first vector
                 b --> the second vector
         Output: 1 if they are different, 0 otherwise
   Restrictions: This might be of limited value since comparing two floating
                 point numbers doesn't usually give the proper result.
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T, int D>
  inline int operator!=(const HPoint_nD<T,D>& a, const HPoint_nD<T,D>& b){
    return !(a==b) ;
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: operator== --- the equality operator between two points
     The equality operator between two points
          Input: a --> the first vector
                 b --> the second vector
         Output: 1 if they are equal, 0 otherwise
   Restrictions: This might be of limited value since comparing two floating
                 point numbers doesn't usually give the proper result.
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T, int D>
  inline int operator==(const HPoint_nD<T,D>& a, const HPoint_nD<T,D>& b){
    int r = 1 ;
    for(int i=D;i>=0;--i)
      r = r && a.data[i]==b.data[i] ;
    return r ;
  }



  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: norm2 --- the sum of the square of all the elements of a point
     The sum of the square of all the elements of a point or the 
                 length of the vector in homogenous space.
          Input: a --> the point
         Output: $a_x^2+a_y^2+a_z^2+a_w^2$
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T,int N>
  inline T norm2(const HPoint_nD<T,N>& a){
    double temp = 0 ;
    for(int i=N-1;i>=0;--i)
      temp += a.data[i]*a.data[i] ;
    return temp ;
  }

  template <class T,int N> 
  inline double norm(const HPoint_nD<T,N>& a) { return sqrt(norm2(a)); }



  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: angle --- the angle between two control points
     The angle between two control points
          Input: a --> the first vector
                 b --> the second vector
         Output: the angle in radians between the two points
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T, int D>
  inline T angle(const HPoint_nD<T,D>& a,const HPoint_nD<T,D>& b) {
    if(b==0 || a==0 )
      return 0 ;
    return dot(a,b)/norm(a)/norm(b) ;
  }



  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
         Member: move --- translates a point in 4D by a point in 3D
     Translates a point in 4D by a point in 3D. This will only
                 modify the $x,y$ and $z$ component of the point in 4D.
	         It will not modify the $w$.
          Input: m --> the point in 3D
         Output: $a'_x = a_x+m_x, a'_y = a_y+m_y, a'_z = a_z+m_z, a'_w = a_w$
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  // is now a in the class definition

  //inline void HPoint_nD<T,D>::move(const Point_nD<T,D>& m); {


  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: operator<< --- the output operator to an ostream
     The output operator to an ostream
          Input:  os --> the ostream
               point --> the point to output
         Output: the ostream with the point 
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T>
  inline ostream& operator<<(ostream& os,const HPoint_nD<T,3>& point)
  {
    os << point.x() << " " << point.y() << " " << point.z() << " " << point.w() << " " ;
    return os;	
  }

  template <class T>
  inline ostream& operator<<(ostream& os,const HPoint_nD<T,2>& point)
  {
    os << point.x() << " " << point.y() << " " << point.w() << " " ;
    return os;	
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: distance3D --- the 3D distance between 2 control points
     The 3D distance between 2 control points
          Input: a --> the first point
                 b --> the second point
         Output: the distance
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T, int D>
  inline T distance3D(const HPoint_nD<T,D>& a, const HPoint_nD<T,D>& b) { 
    return norm(project(a)-project(b)) ;
  }

  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: operator>> --- the input operator from an istream
     Initialize a point in 4D from the input stream.
          Input:  os --> the input stream
               point <-- the point to initialize
         Output: the istream without the point
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  template <class T>
  inline istream& operator>>(istream& os, HPoint_nD<T,3>& point){
    float x,y,z,w ;
    os >> x >> y >> z >> w;
    point.data[0] = x ;
    point.data[1] = y ; 
    point.data[2] = z ;
    point.data[3] = w ;
    return os ;
  }

  template <class T>
  inline istream& operator>>(istream& os, HPoint_nD<T,2>& point){
    float x,y,w ;
    os >> x >> y >> w;
    point.data[0] = x ;
    point.data[1] = y ; 
    point.data[2] = w ;
    return os ;
  }



  /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        Routine: project --- projects a point in 4D to a point in 3D
     Projects a point in 4D to a point in 3D
          Input: a --> the point in 4D
         Output: the point in 3D $(a_x/a_w, a_y/a_w, a_z/a_w)$
   Restrictions:
     author Philippe Lavoie (24 January 1997)
    Modified by:
   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

  template <class T>
  inline Point_nD<T,3> project(const HPoint_nD<T,3>&a) {
    Point_nD<T,3> p(a.data[0]/a.data[3],a.data[1]/a.data[3],a.data[2]/a.data[3]) ;
    return p ;
  }

  template <class T>
  inline Point_nD<T,2> project(const HPoint_nD<T,2>&a) {
    Point_nD<T,2> p(a.x()/a.w(),a.y()/a.w()) ;
    return p ;
  }


  template <class T, int N>
  inline int operator<(const HPoint_nD<T,N>& a, const HPoint_nD<T,N>& b) { 
    return norm2(a)<norm2(b) ? 1 : 0 ;}
  template <class T, int N>
  inline int operator>(const HPoint_nD<T,N>& a, const HPoint_nD<T,N>& b) { 
    return norm2(a)>norm2(b) ? 1 : 0 ;}
  template <class T, int N>
  inline int operator<=(const HPoint_nD<T,N>& a, const HPoint_nD<T,N>& b) { 
    return norm2(a)<=norm2(b) ? 1 : 0 ;}
  template <class T, int N>
  inline int operator>=(const HPoint_nD<T,N>& a, const HPoint_nD<T,N>& b) { 
    return norm2(a)>=norm2(b) ? 1 : 0 ;}


  template <class T>
  inline HPoint_nD<T,2> minimum(HPoint_nD<T,2> a, HPoint_nD<T,2> b){
    HPoint_nD<T,2> m ;
    m.x() = minimum(a.x(),b.x()) ;
    m.y() = minimum(a.y(),b.y()) ;
    m.w() = minimum(a.w(),b.w()) ;
    return m ; 
  }

  template <class T>
  inline HPoint_nD<T,3> minimum(HPoint_nD<T,3> a, HPoint_nD<T,3> b){
    HPoint_nD<T,3> m ;
    m.x() = minimum(a.x(),b.x()) ;
    m.y() = minimum(a.y(),b.y()) ;
    m.z() = minimum(a.z(),b.z()) ;
    m.w() = minimum(a.w(),b.w()) ;
    return m ; 
  }


  template <class T>
  inline HPoint_nD<T,2> maximum(HPoint_nD<T,2> a, HPoint_nD<T,2> b){
    HPoint_nD<T,2> m ;
    m.x() = maximum(a.x(),b.x()) ;
    m.y() = maximum(a.y(),b.y()) ;
    m.w() = maximum(a.w(),b.w()) ;
    return m ; 
  }

  template <class T>
  inline HPoint_nD<T,3> maximum(HPoint_nD<T,3> a, HPoint_nD<T,3> b){
    HPoint_nD<T,3> m ;
    m.x() = maximum(a.x(),b.x()) ;
    m.y() = maximum(a.y(),b.y()) ;
    m.z() = maximum(a.z(),b.z()) ;
    m.w() = maximum(a.w(),b.w()) ;
    return m ; 
  }


  template <class T>
  inline HPoint_nD<T,3> minimumByRef(const HPoint_nD<T,3> &a, const HPoint_nD<T,3> &b){
    HPoint_nD<T,3> m ;
    m.x() = minimum(a.x(),b.x()) ;
    m.y() = minimum(a.y(),b.y()) ;
    m.z() = minimum(a.z(),b.z()) ;
    m.w() = minimum(a.w(),b.w()) ;
    return m ; 
  }


  template <class T>
  inline HPoint_nD<T,2> minimumByRef(const HPoint_nD<T,2> &a, const HPoint_nD<T,2> &b){
    HPoint_nD<T,2> m ;
    m.x() = minimum(a.x(),b.x()) ;
    m.y() = minimum(a.y(),b.y()) ;
    m.w() = minimum(a.w(),b.w()) ;
    return m ; 
  }


  template <class T>
  inline HPoint_nD<T,3> maximumByRef(const HPoint_nD<T,3> &a, const HPoint_nD<T,3> &b){
    HPoint_nD<T,3> m ;
    m.x() = maximum(a.x(),b.x()) ;
    m.y() = maximum(a.y(),b.y()) ;
    m.z() = maximum(a.z(),b.z()) ;
    m.w() = maximum(a.w(),b.w()) ;
    return m ; 
  }



  typedef HPoint_nD<float,3> HPoint3Df ;
  typedef HPoint_nD<double,3> HPoint3Dd ;

  typedef HPoint_nD<float,2> HPoint2Df ;
  typedef HPoint_nD<double,2> HPoint2Dd ;



} // end namespace

typedef PLib::HPoint_nD<float,3> PlHPoint3Df ;
typedef PLib::HPoint_nD<double,3> PlHPoint3Dd ;

typedef PLib::HPoint_nD<float,2> PlHPoint2Df ;
typedef PLib::HPoint_nD<double,2> PlHPoint2Dd ;

#endif
