/*=============================================================================
        File: barray2d.cpp
     Purpose:
    Revision: $Id: barray2d.cpp,v 1.2 2002/05/13 21:07:45 philosophil Exp $
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

#ifndef BARRAY2D_SOURCES_
#define BARRAY2D_SOURCES_

#include "matrix_global.h"
#include <new>
#include "barray2d.h"

/*!
 */
namespace PLib {

/*!
  \brief initialize the 2D array

  \author Philippe Lavoie (24 January 1997)
*/
template <class T>
void 
initBasic2DArray(Basic2DArray<T> &a, const int r,const int c)
{
  if ( r <= 0  || c <= 0 )
    {
      return ;
    }
	
  a.rz = r;	a.cz = c;

  a.m = new T [a.rz*a.cz];
  a.created = 1 ;
#ifdef COLUMN_ORDER
  a.vm = new T*[a.cz] ;
#else
  a.vm = new T*[a.rz] ;
#endif
  
  T *p1 ;
  const int sze = a.rz*a.cz ;
  p1 = a.m-1 ;
  
  int i ;
  for(i = sze; i > 0; --i)
      *(++p1) = T() ;
#ifdef COLUMN_ORDER
  for(i=a.cz-1;i>=0;--i)
    a.vm[i] = &a.m[i*a.rz] ;
#else
  for(i=a.rz-1;i>=0;--i)
    a.vm[i] = &a.m[i*a.cz] ;
#endif
}

/*!
  \brief constructor  

  Constructs an array of size (0,0).

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Basic2DArray<T>::Basic2DArray() {
  by_columns = 0 ;
  width = 2 ;
  created = 1 ;
  m = 0; 
  vm = 0 ;
  init(); 
}

/*!
  \brief constructor  with the size specified

  Constructs an array of size (r,c).

  \param r  the number of rows 
  \param c  the number of columns

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T>
Basic2DArray<T>::Basic2DArray(const int r, const int c) {
    by_columns = 0 ;
    width = 2 ;
    created = 1 ;
    m = 0; 
    vm = 0 ;
    init(r,c); 
}


/*!
  \brief constructor from memory

  Constructs a matrix from memory. No memory is allocated for 
  this new basic2DArray and the memory will not be deallocated
  when the destructor is called. You are responsible for cleaning
  up the memory you allocated.

  \param p  the memory pointer
  \param r  the number of rows 
  \param c  the number of columns

  \warning Do not access the basic2DArray once you delete the memory
           segment used by the matrix since it now points to a dead
	   memory location.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Basic2DArray<T>::Basic2DArray(T* p, const int r, const int c) {
  created = 0 ;
  rz = r ;
  cz = c ;
  m = p ;
  by_columns = 0 ;
  width = 2 ; 
#ifdef COLUMN_ORDER
  vm = new T* [cz] ;
  for(int i=cz-1;i>=0;--i)
    vm[i] = &m[i*rz] ;
#else
  vm = new T* [rz] ;
  for(int i=rz-1;i>=0;--i)
    vm[i] = &m[i*cz] ;
#endif
}

/*!
  \brief copy constructor

  Copy constructor

  \param a  the Basic2DArray to copy from

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Basic2DArray<T>::Basic2DArray(const Basic2DArray<T> &a) 
{
  int i;
  created = 1 ;
  m = 0 ;
  init(a.rows(),a.cols());
  by_columns = a.by_columns;
  width = a.width ;
  
  T *p1,*pa ;
  int sz = a.rows()*a.cols() ;
  p1 = m-1 ;
  pa = a.m-1 ;

  for (i = sz; i > 0 ; --i)
      *(++p1) = *(++pa) ;
}

/*!
  \brief assignment operator

  Assignment operator

  \param a  the Basic2DArray to copy from

  \return a reference to itself

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Basic2DArray<T>& Basic2DArray<T>::operator=(const Basic2DArray<T> &a)
{
  int i;
  
  if ( this == &a )
    return *this;
  
  if(rows() != a.rows() || cols() != a.cols()){
    resize(a.rows(),a.cols()) ;
  }

  T *p1,*pa ;
  int sz = a.rows()*a.cols() ;
  p1 = m-1 ;
  pa = a.m-1 ;

  for (i = sz; i > 0; --i)
      *(++p1) = *(++pa) ;
  
  by_columns = a.by_columns;
  width = a.width ;
  
  return *this;
}

/*!
  \brief destructor

  Destructor

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
Basic2DArray<T>::~Basic2DArray()
{
  if(m && created){
    delete []m;
  }
  if(vm)
    delete []vm ;
}

/*!
  \brief a destructive resize of the matrix dimensions

  Changes the matrix dimensions and intialize it to 0.

  \param  nr  the new number of rows
  \param nc  the new number of columns

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
void Basic2DArray<T>::resize(const int nr, const int nc)
{
  if(rows()>1 || cols()>1){
    if(m && created)
      delete []m ;
    if(vm)
      delete []vm ;
  }
  else{
    if(m && created)
      delete []m ;
    if(vm)
      delete []vm ;
  }
    
  init(nr, nc);
}


/*!
  \brief a non-destructive resize of the matrix dimensions

  Changes the matrix dimensions in a non-destructive manner.
  The old elements are still accessible at the same coordinates.
  
  This is an expensive function computationally wise due to the 
  copying which needs to be done.
  
  \param nr  the new number of rows
  \param nc  the new number of columns

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
void 
resizeKeepBasic2DArray(Basic2DArray<T> &a,const int nr,const int nc)
{

  if(nr==a.rz && nc==a.cz){
    return ;
  }

  T *mn ;
  T *p,*pn ;

  mn = new T[nr*nc] ;
  
  int i,j ;


#ifdef COLUMN_ORDER
  for(j=0;j<minimum(nc,a.cz);j++){
    p = &a.m[j*a.rz] -1;
    pn = &mn[j*nr] -1 ; 
    for(i=0;i<minimum(nr,a.rz);i++){
      *(++pn) = *(++p) ;
    }
    for(i=a.rz;i<nr;i++)
      *(++pn) = T() ;
  }
  
  for(j=a.cz;j<nc;j++){
    pn = &mn[j*nr]-1 ;
    for(i=0;i<nr;i++)
      *(++pn) = T() ;
  }
#else
  for(i=0;i<minimum(nr,a.rz);i++){
    p = &a.m[i*a.cz] -1;
    pn = &mn[i*nc] -1 ;
    for(j=0;j<minimum(nc,a.cz);j++){
      *(++pn) = *(++p) ;
    }
    for(j=a.cz;j<nc;j++)
      *(++pn) = T() ;
  }

  for(i=a.rz;i<nr;i++){
    pn = &mn[i*nc]-1 ;
    for(j=0;j<nc;j++)
      *(++pn) = T() ;
  }
#endif

  
  a.rz = nr ;
  a.cz = nc ;
  
  if(a.m && a.created)
    delete []a.m ;
  a.m = mn ;
  if(a.vm)
    delete []a.vm ;
#ifdef COLUMN_ORDER
  a.vm = new T* [a.cz] ;
  for(i=0;i<a.cz;++i)
    a.vm[i] = &a.m[i*a.rz] ;
#else
  a.vm = new T* [a.rz] ;
  for(i=0;i<a.rz;++i)
    a.vm[i] = &a.m[i*a.cz] ;
#endif
}

/*!
  \brief reset all values of the 2D array to \a v

  Reset all values of the 2D array to \a v

  \param v  resets the elements of the 2D array to this value

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
void Basic2DArray<T>::reset(const T v)
{
  T *p1 ;
  p1 = m-1 ;
  const int size = rows()*cols() ;
  for(int i=size;i>0; --i)
    *(++p1) = v ;
}

#ifdef DEBUG_PLIB

/*!
  \brief accesses an element of a 2D array

  Access an element of a 2D array. If the DEBUG_PLIB
  flag has been set, then a fatal error will be issued if the 
  element index are not in a valid range.
  
  \param i  the row of the element
  \param j  the column of the element

  \return vm[i][j] or vm[j][i], depending on COLUMN_ORDER.
  \warning
  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
T& Basic2DArray<T>::elem(const int i,const int j)
{
  if ( i < 0  || rows() <= i || j < 0 || cols() <= j )
    {
#ifdef USE_EXCEPTION
      throw OutOfBound2D(i,j,0,rows()-1,0,cols()-1) ;
#else
      Error error("T& Basic2DArray<T>::elem(int,int)") ;
      if (i < 0 || rows() <= i){
	error << "bad first index " << i << endl ;
      }
      if (j < 0 || cols() <= j){
	error << "bad second index " << j << endl ;
      }
      error.fatal() ;
#endif
    }
#ifdef COLUMN_ORDER
  return vm[j][i] ;
#else
  return vm[i][j] ;
#endif
}

/*!
  \brief accesses an element of a 2D array

  Access an element of a 2D array. If the DEBUG_LIB
  flag has been set, then a fatal error will be issued if the 
  element index are not in a valid range.

  \param i  the row of the element
  \param j  the column of the element

  \return \a vm[i][j] or \a vm[j][i], depending on COLUMN_ORDER.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
T Basic2DArray<T>::elem(const int i,const int j) const
{
  if ( i < 0  || rows() <= i || j < 0 || cols() <= j )
    {
#ifdef USE_EXCEPTION
      throw OutOfBound2D(i,j,0,rows()-1,0,cols()-1) ;
#else
      Error error("T Basic2DArray<T>::elem(int,int) const") ;
      if (i < 0 || rows() <= i){
	error << "bad first index " << i << endl ;
      }
      if (j < 0 || cols() <= j){
	error << "bad second index " << j << endl ;
      }
      error.fatal() ;
#endif
    }
#ifdef COLUMN_ORDER
  return vm[j][i] ;
#else
  return vm[i][j] ;
#endif
}
#endif

/*!
  \brief reads a 2D arra from an istream

  Reads a matrix from an istream

  \param is  the input stream
  \param a  the 2D array to initialize

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
istream& 
operator>> (istream& is,Basic2DArray<T>& a)
{
  int i, j;
  int r = a.rows(), c = a.cols();
  
  if ( a.by_columns )
    for (j = 0; j < c; j++)
      for (i = 0; i < r; i++)
	is >>  a.elem(i,j) ;
  else
    for (i = 0; i < r; i++)
      for (j = 0; j < c; j++)
	is >>  a.elem(i,j) ;
  
  return is;	
}

/*!
  \brief writes a 2D array to an ostream
  Writes a matrix to an ostream
  \param os  the output stream
  \return 
  \warning
  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
ostream& 
Basic2DArray<T>::print(ostream& os) const
{
  int i, j;
  const int r = rows();
  const int c = cols();
  
  if ( by_columns )
    for (j = 0; j < c; j++)
      {
	for (i = 0; i < r; i++) {
	  os <<  setw(width) << elem(i,j) << ' ';
	}
	os << '\n';
      }  
  else
    for (i = 0; i < r; i++)
      {
	for (j = 0; j < c; j++){
	  os <<  setw(width) << elem(i,j) << ' ' ;
	}
	os << '\n';
      }

  return os;
}

/*!
  \brief writes a 2D array to an ostream
  \param os  the output stream
                a  the 2D array to output
  \return 
  \warning
  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
ostream& 
operator<<(ostream& os,const Basic2DArray<T>& a)
{
  return a.print(os) ; 
}

}


#endif
