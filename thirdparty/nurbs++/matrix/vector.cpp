/*=============================================================================
        File: vector.cpp
     Purpose:
    Revision: $Id: vector.cpp,v 1.2 2002/05/13 21:07:45 philosophil Exp $
  Created by:    Philippe Lavoie          (3 Oct, 1996)
 Modified by: 

 Copyright notice:
          Copyright (C) 1996-1998 Philippe Lavoie
 
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

#ifndef VECTOR_SOURCES_
#define VECTOR_SOURCES_

#include "matrix_global.h"

#include "vector.h"

/*!
 */
namespace PLib {

/*!
  \brief the assignment operator

  The values of a vector are copied to this one

  \param b  the vector to copy

  \return a reference to itself

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T> Vector<T>& Vector<T>::operator=(const Vector<T> &b)
{
  if(this==&b)
    return *this ;

  if ( this->n() != b.n())
    {
      this->resize(b.n()) ;
    }

  this->sze = b.n() ;
  T *pa, *pb ;
  pa = this->x-1 ;
  pb = b.x-1 ;
  for(int i=this->n();i>0;--i){
    *(++pa) = *(++pb) ;
  }
  return *this;

}

/*!
  \brief the assignment operator with a BasicArray

  \param b  the BasicArray to copy
  \return a reference to itself

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Vector<T>& Vector<T>::operator=(const BasicArray<T> &b)
{
  if ( this->size() != b.size())
    {
      this->resize(b.size()) ;
    }
  T *ptr ;
  ptr = this->x - 1 ;
  for(int i=this->size()-1;i>=0;--i)
     *(++ptr) = b[i] ;

  return *this;
}

/*!
  \brief assigns all the components of the vector to a value

  All the components of the vector are assigned to the value \a d

  \param d  the value to assigned the vector to
  \return the value of \a d

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
T Vector<T>::operator=(const T d)
{
  const int sz = this->size(); 
  T *ptr ;
  ptr = this->x-1 ;
  for (int i = sz; i > 0; --i)
    *(++ptr) = d ;

  return d;	
}

/*!
  \brief the += operator

  Each component of the vector is increased by the components of 
  vector \a a.

  \param a  the vector to add to itself
  \return a reference to itself
  \warning The vectors must have the same size.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Vector<T>& Vector<T>::operator+=(const Vector<T> &a)
{
  if ( a.size() != this->size())
    {
#ifdef USE_EXCEPTION
      throw WrongSize(this->size(),a.size()) ;
#else
      Error error("Vector<T>::operator+=(Vector<T>&)");
      error << "Vector<T> a += Vector<T> b different sizes, a = " << size() << ", b = " << a.size() ;
      error.fatal() ;
#endif
    }
  const int sz = this->size();
  T *ptr,*aptr ;
  ptr = this->x-1 ;
  aptr = a.x-1 ;
  for (int i = sz; i >0; --i)
    *(++ptr) += *(++aptr) ;
  
  return *this;
}

/*!
  \brief the -= operator

  Each component of the vector is decreased by the components of vector \a a.

  \param a  the vector to substract from itself
  \return a reference to itself
  \warning The vectors must have the {\em same} size.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Vector<T>& Vector<T>::operator-=(const Vector<T> &a)
{
  if ( a.size() != this->size())
    {
#ifdef USE_EXCEPTION
      throw WrongSize(this->size(),a.size()) ;
#else
      Error error("Vector<T>::operator-=(Vector<T>&)");
      error << "Vector<T> a -= Vector<T> b different sizes, a = " << size() << ", b = " << a.size() ;
      error.fatal() ;
#endif
    }
  
  const int sz = this->size(); 
  T *ptr,*aptr ;
  ptr = this->x-1 ;
  aptr = a.x-1 ;
  for (int i = sz; i > 0; --i)
    *(++ptr) -= *(++aptr) ;
  
  return *this;
}

/*!
  \brief Adds two vectors

  \param a  the first vector to add
  \param b  the second vector to add
  \return the result of $a+b$

  \warning The vectors must have the {\em same} size.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Vector<T> operator+(const Vector<T>& a, const Vector<T> &b)	
{
  Vector<T> sum(a);
  sum += b ;
  
  return sum;
}

/*!
  \brief Substracts two vectors

  \param a  the first vector to add
  \param b  the second vector to add

  \return the result of a-b

  \warning The vectors must have the same size.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Vector<T> operator-(const Vector<T>& a, const Vector<T> &b)
{
  Vector<T> diff(a);
  diff -= b ;
  return diff;
}

/*!
  \brief the multiplicative operator

  \param a  the first vector
  \param b  the second vector to multiply with

  \return a*b

  \warning The vectors must have the same size.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
T operator*(const Vector<T> &a,const Vector<T> &b)
{
  if ( a.size() != b.size() )
    {
#ifdef USE_EXCEPTION
      throw WrongSize(a.size(),b.size()) ;
#else
      Error error("Vector<T>::operator=(Vector<T>&)");
      error << "Vector<T> a = Vector<T> b different sizes, a = " << a.size() << ", b = " << b.size() ;
      error.fatal() ;
#endif
    }

  int i, sz = a.size();
  T prod, zero = (T)0;
  T *aptr,*bptr ;
  aptr = a.x-1 ;
  bptr = b.x-1 ;
  prod = zero ;
  for (i = sz ; i > 0; --i)
    prod += (*(++aptr)) * (*(++bptr)) ;
  
  return prod;
}

/*!
  \brief multiplies a vector with a double 

  Multiplies all the elements of the vector \a v with a double \a d.

  \param v  the vector to multiply
  \param d  multiply the vector by this value

  \return a vector having value d.V

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Vector<T>  operator*(const Vector<T>& v, const double d)
{
  int i, sz = v.size();
  Vector<T> b(v.size());
  
  T *aptr,*bptr ;
  aptr = v.x-1 ;
  bptr = b.x-1 ;
  for (i = sz; i > 0; --i){
    *(++bptr) = (T)(d * (*(++aptr)) ) ;
  }
  
  return b;

}

/*!
  \brief multiplies the vector with a complex number

  \param v  the vector to multiply
  \param d  the complex value to multiply $v$ with

  \return a vector having \a d.v

  \warning If the vector is not of complex type, then the vector will 
           be multiplied by the real part of \a d. 

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Vector<T>  operator*(const Vector<T>& v, const Complex d)
{
  int i, sz = v.size();
  Vector<T> b = v;
  
  T *bptr ;
  bptr = b.x-1 ;
  for (i = sz; i > 0; --i){
    ++bptr ;
    (*bptr) = (T)(real(d)*(*bptr));
  }
  
  return b;

}


/*!
  \brief the equality operator

  \param a  the first vector to check
  \param b  the second vector to check   

  \return 1 if the vectors are equal, 0 otherwise

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
int operator==(const Vector<T> &a,const Vector<T> &b)
{
  if ( a.size() != b.size() )
    {
      return 0 ;
    }

  int i, sz = a.size();
  int l = 1;
  
  T *aptr,*bptr ;
  aptr = a.x-1 ;
  bptr = b.x-1 ;

  for (i = sz; i > 0; --i)
    l = l && ( *(++aptr) == *(++bptr) );
  
  return l;
}

/*!
  \brief copies the values of \a b to the vector starting from the index \a i.

  The values of the vector \a b replace the values of the vector 
  starting at the index \a i. 

  \param i  the index to start copying from 
  \param b  the vector to copy from 

  \warning The vector \a b must fit entirely into the vector when starting 
           at \a i.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T> 
void Vector<T>::as(int i, const Vector<T> &b)
{
  if ( (i + b.rows()) > rows() )
    {
#ifdef USE_EXCEPTION
      throw MatrixErr() ;
#else
      Error error("Vector<T>::as(int,Vector<T>)");
      error << "Vector is too long to fit at i= " << i << endl ;
      error.fatal() ;
#endif
    }

  T *aptr,*bptr ;
  aptr = &(this->x)[i]-1 ;
  bptr = b.x-1 ;
  for ( int j = b.rows(); j > 0; --j)
      *(++aptr) = *(++bptr) ;
}

/*!
  \brief extract a vector of size \a l starting at index \a i

  This extracts a vector of size \a l by copying the values from 
  the vector starting at index \a i.

  \param i  the index to start copying from
  \param l  the length of the new vector

  \return A vector representing the range \a [i..i+l] from this vector

  \warning The vector extracted must fit inside the vector starting at \a i.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Vector<T> Vector<T>::get(int i, int l)
{
  if ( (i + l) > rows() )
    {
#ifdef USE_EXCEPTION
      throw MatrixErr() ;
#else
      Error error("Vector<T>::get(int,Vector<T>)");
      error << "Vector is too long to extract from  i= " << i << "from l=" << l << endl ;
      error.fatal() ;
#endif
    }

  Vector<T> subvec(l) ;
  T *aptr, *bptr ;
  aptr = &(this->x)[i] - 1 ;
  bptr = subvec.x -1 ;
  for ( int j = l; j > 0; --j)
    *(++bptr) = *(++aptr) ;
  return subvec ;
}

/*!
  \brief finds the index of its minimal entry

  Scans the vector to find its minimal value and returns the 
  index of that value.

  \return the index of the minimal component.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
int Vector<T>::minIndex() const {
  T min = this->x[0] ;
  int index = 0 ;

  for(int i=1;i<this->n();i++){
    if(this->x[i]<=min){
      min = this->x[i] ;
      index = i ;
    }
  }
  return index ;
}


/*!
  \brief do a quick sort using the standard C library sort algorithm

  Performs a quick sort of the vector. The quick sort algorithm 
  used is the one from the standard C library.

  \return The vector is sorted in ascending order

  \warning Due to the use of the standard library, only certain types can 
           be used with this function: int, float and double.

  \author Philippe Lavoie 
  \date 21 May 1997
*/
template <class T>
void Vector<T>::qSortStd(){
#ifdef USE_EXCEPTION
  throw MatrixErr() ;
#else
  Error error("Vector<T>::qSort()");
  error << "qSortStd is not defined for that type.\nPlease defined it in your .C file!";
  error.fatal() ;
#endif
}


template <class T>
inline void swap(T& a, T& b){
  T temp = a ;
  a = b ;
  b = temp ;
}

/*!
  \brief do a quick sort using an optimized algorithm

  Do a quick sort of the vector using an algorithm based on the 
  one described in "Numerical Recipes in C". You should use this 
  method over using qSortStd since it is usually faster.

  \param M  regions smaller than this value are sorted using the 
	    insertion method, the default value of 7 is suitable for 
	    most cases.

  \return The vector is sorted in ascending order

  \warning Some types don't have comparison operators and are not 
               supported.

  \author Philippe Lavoie 
  \date 21 May 1997
*/
template <class T>
void Vector<T>::qSort(int M){
  const int Nstack=50 ; 
  int i,ir,j,k,l ;
  Vector<int> istack(Nstack) ;
  int jstack=0;
  T a ;
  T *v1,*v2  ;

  ir = this->sze-1 ;
  l = 0 ;
  
  while(1){
    if(ir-l<M){ // perform an insertion sort when the array is small enough
      v1 = &(this->x)[l] ;
      for(j=l+1;j<=ir;++j){
	a = *(++v1) ;
	v2 = v1 ;
	--v2 ;
	for(i=j-1;i>=0;--i){
	  if(*v2 <= a) break ;
	  *(v2+1) = *v2 ;
	  --v2 ;
	}
	++v2 ;
	*(v2) = a ;
      }
      if(jstack==0) break ;
      ir=istack[--jstack] ;
      l=istack[--jstack] ;
    }
    else{
      k=(l+ir) >> 1 ;
      swap((this->x)[k],(this->x)[l+1]) ;
      if(this->x[l+1] > this->x[ir]){
	swap((this->x)[l+1],(this->x)[ir]) ;
      }
      if((this->x)[l]> (this->x)[ir]){
	swap(this->x[l],this->x[ir]) ;
      }
      if(this->x[l+1] > this->x[l]){
	swap(this->x[l+1],this->x[l]) ;
      }
      i=l+1 ;
      j=ir ;
      a=this->x[l] ;
      v1 = &(this->x)[i] ;
      v2 = &(this->x)[j] ;
      while(1){
	while(*v1 < a) { ++i ; ++v1 ; }
	while(*v2 > a) { --j ; --v2 ; }
	if(j<i) break ;
	if(*v1 == *v2)  // both are equal to a...
	  break ;
	swap(this->x[i],this->x[j]) ;
      }
      this->x[l] = this->x[j] ;
      this->x[j] = a ;
      jstack += 2 ;
      if(jstack>=Nstack){
	istack.resize(istack.n()+Nstack) ; // increase the vector size
      }
      if(ir-i+1 >= j-l){
	istack[jstack-1]=ir ;
	istack[jstack-2] = i ;
	ir=j-1 ;
      }
      else{
	istack[jstack-1] = j-1 ;
	istack[jstack-2] = l ;
	l=i ;
      }
    }
  }
}


/*!
  \brief generates sorted index vector

  A sorted index vector is generated by this routine. It is based
  on a routine described in "Numercial Recipes in C".

  \param index  The index vector
  \param M  regions smaller than this value are sorted using the 
	    insertion method, the default value of 7 is suitable 
	    for most cases.

  \return index holds a sorted index vector

  \warning Some types don't have comparison operators and are not
           supported.

  \author Philippe Lavoie 
  \date 21 May 1997
*/
template <class T>
void Vector<T>::sortIndex(Vector<int>& index, int M) const{
  const int Nstack=50 ; 
  int i,ir,j,k,l,indext ;
  Vector<int> istack(Nstack) ;
  int jstack=0;
  T a ;

  ir = this->sze-1 ;
  l = 0 ;
  
  index.resize(this->sze) ;
  for(i=0;i<index.n();++i)
    index[i] = i ;

  while(1){
    if(ir-l<M){ // perform an insertion sort when the array is small enough
      for(j=l+1;j<=ir;++j){
	indext = index[j] ;
	a = this->x[indext] ;
	for(i=j-1;i>=0;--i){
	  if(this->x[index[i]] <= a) break ;
	  index[i+1] = index[i] ;
	}
	index[i+1] = indext ;
      }
      if(jstack==0) break ;
      ir=istack[--jstack] ;
      l=istack[--jstack] ;
    }
    else{
      k=(l+ir) >> 1 ;
      swap(index[k],index[l+1]) ;
      if(this->x[index[l+1]] > this->x[index[ir]]){
	swap(index[l+1],index[ir]) ;
      }
      if(this->x[index[l]]> this->x[index[ir]]){
	swap(index[l],index[ir]) ;
      }
      if(this->x[index[l+1]] > this->x[index[l]]){
	swap(index[l+1],index[l]) ;
      }
      i=l+1 ;
      j=ir ;
      indext = index[l] ;
      a=this->x[indext] ;
      while(1){
	while(this->x[index[i]] < a) { ++i ; }
	while(this->x[index[j]] > a) { --j ; }
	if(j<i) break ;
	if(this->x[index[i]] == this->x[index[j]])
	  break ;
	swap(index[i],index[j]) ;
      }
      index[l] = index[j] ;
      index[j] = indext ;
      jstack += 2 ;
      if(jstack>=Nstack){
	istack.resize(istack.n()+Nstack) ; // increase the vector size
      }
      if(ir-i+1 >= j-l){
	istack[jstack-1]=ir ;
	istack[jstack-2] = i ;
	ir=j-1 ;
      }
      else{
	istack[jstack-1] = j-1 ;
	istack[jstack-2] = l ;
	l=i ;
      }
    }
  }
}


}


#endif
