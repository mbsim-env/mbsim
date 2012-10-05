/*============================================================================
        File: specialVcc.h
     Purpose: Hides some of the hugly things we have to do to make VC++ happy
              when compiling.
    Revision: $Id: specialVcc.h,v 1.2 2002/05/13 21:07:45 philosophil Exp $
  author Philippe Lavoie          (3 Oct, 1996)
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
#ifndef _Matrix_specialVcc_h_
#define _Matrix_specialVcc_h_


template <class T,int N> 
double norm(const Point_nD<T,N>& a);


template<>
struct Point_nD<float,3> { 
  typedef float T;
  T data[3] ;
  Point_nD()  { x() = y() = z() = 0 ;}
  Point_nD(T a)  { x() = y() = z() = a ;}
  Point_nD(T X, T Y, T Z)  {x()=X ; y()=Y ; z()=Z ;}
  Point_nD(const Point_nD<float,3>& a)  { memcpy((void*)data,(void*)a.data,3*sizeof(T));}

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
 
  Point_nD unitLength() const {
  	T d = norm(*this) ;
    Point_nD<T,3> u(x()/d,y()/d,z()/d) ;
    return u ;
  }
};

template<>
struct Point_nD<double,3> { 
  typedef double T;
  T data[3] ;
  Point_nD()  { x() = y() = z() = 0 ;}
  Point_nD(T a)  { x() = y() = z() = a ;}
  Point_nD(T X, T Y, T Z)  {x()=X ; y()=Y ; z()=Z ;}
  Point_nD(const Point_nD<float,3>& a)  { memcpy((void*)data,(void*)a.data,3*sizeof(T));}

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
 
  Point_nD unitLength() const {
  	T d = norm(*this) ;
    Point_nD<T,3> u(x()/d,y()/d,z()/d) ;
    return u ;
  }
};


template<>
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
 
  Point_nD unitLength() const {
	  T d = norm(*this) ;
    Point_nD<T,2> u(x()/d,y()/d) ;
    return u ;
  }

protected:
  static T dumbVar ; 
};

template<>
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
 
  Point_nD unitLength() const {
	  T d = norm(*this) ;
    Point_nD<T,2> u(x()/d,y()/d) ;
    return u ;
  }

protected:
  static T dumbVar ; 
};


template<>
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
  
  HPoint_nD<T,3>& operator=(const HPoint_nD<T,3>& v) {
    data[0] = v.data[0] ; 
    data[1] = v.data[1] ;
    data[2] = v.data[2] ;
    data[3] = v.data[3] ; 
    return *this ;
  }
  HPoint_nD<T,3>& operator=(const Point_nD<T,3>& v) {x()=v.x() ; y()=v.y() ; z()=v.z() ; w()=1.0 ; return *this;}
  HPoint_nD<T,3>& operator=(const T v) { x() = y() = z() = w() = v ; return *this ;} ;
  HPoint_nD<T,3>& operator+=(const HPoint_nD<T,3>& v) {
    data[0] += v.data[0] ; 
    data[1] += v.data[1] ;
    data[2] += v.data[2] ;
    data[3] += v.data[3] ; 
    return *this ;
  }
  HPoint_nD<T,3>& operator-=(const HPoint_nD<T,3>& v) {
    data[0] -= v.data[0] ; 
    data[1] -= v.data[1] ;
    data[2] -= v.data[2] ;
    data[3] -= v.data[3] ; 
    return *this ;
  }

  HPoint_nD<T,3>& operator*=(T v) {
    data[0] *= v ; 
    data[1] *= v ;
    data[2] *= v ;
    data[3] *= v ; 
    return *this ;
  }
  HPoint_nD<T,3>& operator/=(T v)  {
    data[0] /= v ; 
    data[1] /= v ;
    data[2] /= v ;
    data[3] /= v ; 
    return *this ;
  }


  void move(const Point_nD<T,3>& m) ;

  Point_nD<T,3> projectW() { return Point_nD<T,3>(x(),y(),z()) ; }
};

//template <class T>
template<>
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
  
  HPoint_nD<T,3>& operator=(const HPoint_nD<T,3>& v) {
    data[0] = v.data[0] ; 
    data[1] = v.data[1] ;
    data[2] = v.data[2] ;
    data[3] = v.data[3] ; 
    return *this ;
  }

  HPoint_nD<T,3>& operator=(const Point_nD<T,3>& v) {x()=v.x() ; y()=v.y() ; z()=v.z() ; w()=1.0 ; return *this;}
  HPoint_nD<T,3>& operator=(const T v) { x() = y() = z() = w() = v ; return *this ;} ;
  HPoint_nD<T,3>& operator+=(const HPoint_nD<T,3>& v) {
    data[0] += v.data[0] ; 
    data[1] += v.data[1] ;
    data[2] += v.data[2] ;
    data[3] += v.data[3] ; 
    return *this ;
  }

  HPoint_nD<T,3>& operator-=(const HPoint_nD<T,3>& v) {
    data[0] -= v.data[0] ; 
    data[1] -= v.data[1] ;
    data[2] -= v.data[2] ;
    data[3] -= v.data[3] ; 
    return *this ;
  }
  HPoint_nD<T,3>& operator*=(T v) {
    data[0] *= v ; 
    data[1] *= v ;
    data[2] *= v ;
    data[3] *= v ; 
    return *this ;
  }

  HPoint_nD<T,3>& operator/=(T v)  {
    data[0] /= v ; 
    data[1] /= v ;
    data[2] /= v ;
    data[3] /= v ; 
    return *this ;
  }


  void move(const Point_nD<T,3>& m) ;

  Point_nD<T,3> projectW() { return Point_nD<T,3>(x(),y(),z()) ; }
};

template<>
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
  

  HPoint_nD<T,2>& operator=(const HPoint_nD<T,2>& v) {
    data[0] = v.data[0] ; 
    data[1] = v.data[1] ;
    data[2] = v.data[2] ;
    return *this ;
  }

  HPoint_nD<T,2>& operator=(const Point_nD<T,2>& v) {x()=v.x() ; y()=v.y() ; w()=1.0 ; return *this;}
  HPoint_nD<T,2>& operator=(const T v) { x() = y() = w() = v ; return *this ;} ;

  HPoint_nD<T,2>& operator*=(T v) {
    data[0] *= v ; 
    data[1] *= v ;
    data[2] *= v ;
    return *this ;
  }

  HPoint_nD<T,2>& operator/=(T v)  {
  data[0] /= v ; 
  data[1] /= v ;
  data[2] /= v ;
  return *this ;
}

  HPoint_nD<T,2>& operator-=(const HPoint_nD<T,2>& v) {
    data[0] -= v.data[0] ; 
    data[1] -= v.data[1] ;
    data[2] -= v.data[2] ;
    return *this ;
  }

  HPoint_nD<T,2>& operator+=(const HPoint_nD<T,2>& v) ; 

  void move(const Point_nD<T,2>& m) ;
  Point_nD<T,2> projectW() { return Point_nD<T,2>(x(),y()) ; }

protected:
  static T dumbVar ;
};

template<>
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
  

  HPoint_nD<T,2>& operator=(const HPoint_nD<T,2>& v) {
    data[0] = v.data[0] ; 
    data[1] = v.data[1] ;
    data[2] = v.data[2] ;
    return *this ;
  }


  HPoint_nD<T,2>& operator=(const Point_nD<T,2>& v) {x()=v.x() ; y()=v.y() ; w()=1.0 ; return *this;}
  HPoint_nD<T,2>& operator=(const T v) { x() = y() = w() = v ; return *this ;} ;

  HPoint_nD<T,2>& operator*=(T v) {
    data[0] *= v ; 
    data[1] *= v ;
    data[2] *= v ;
    return *this ;
  }

  HPoint_nD<T,2>& operator/=(T v)  {
    data[0] /= v ; 
    data[1] /= v ;
    data[2] /= v ;
    return *this ;
  }
 
  HPoint_nD<T,2>& operator-=(const HPoint_nD<T,2>& v) {
    data[0] -= v.data[0] ; 
    data[1] -= v.data[1] ;
    data[2] -= v.data[2] ;
    return *this ;
  }
  HPoint_nD<T,2>& operator+=(const HPoint_nD<T,2>& v) ; 

  void move(const Point_nD<T,2>& m) ;
  Point_nD<T,2> projectW() { return Point_nD<T,2>(x(),y()) ; }

protected:
  static T dumbVar ;
};



#endif
