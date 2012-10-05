/*=============================================================================
        File: barray.cpp
     Purpose:       
    Revision: $Id: barray.cpp,v 1.3 2002/05/14 19:14:28 philosophil Exp $
  Created by: Philippe Lavoie          (3 Oct, 1996)
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

#ifndef BARRAY_SOURCES_
#define BARRAY_SOURCES_


#include "matrix_global.h"
#include <new>
#include "barray.h"
#include <cstring>

/*!
 */
namespace PLib {

/*!
  \brief default constructor

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template<class T>
BasicArray<T>::BasicArray() : rsize(1), wdth(1),sze(1)
{
  x = new T [1];
  x[0] = (T) 0 ;
  destruct = 1 ;
}

/*!
  \brief constructor with size specified

  \param ni  the size of the basic array

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template<class T>
BasicArray<T>::BasicArray(const int ni) {
  x = 0 ;
  sze = wdth = rsize = 0 ;
  resize(ni) ;
  destruct = 1 ;
}

/*!
  \brief copy constructor from a pointer

  The BasicArray is constructed from a pointer and a specified
  size. The desctructor of the class will {\bf not} delete this
  pointer when it is finished with it. It is up to the function
  which created it to delete it.

  \param ap  a pointer to an array
  \param size  the size of the ap array

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template<class T>
BasicArray<T>::BasicArray(T* ap, const int size) : rsize(size), wdth(size+1), sze(size)
{
  x = ap ;
  destruct = 0 ;
}

/*!
  \brief copy constructor
  
  \param f2  the BasicArray to copy

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template<class T> BasicArray<T>::BasicArray(const BasicArray<T>& f2) 
{
  rsize = sze = 0 ;
  x = 0 ;
  resize(f2.size());
  T *p2,*p1 ;
  p1 = x-1 ;
  p2 = f2.x - 1 ;
  
  for (int k = rsize; k >0; --k)
    *(++p1) = *(++p2) ;
  destruct = 1 ;
}

/*!
  \brief constructor from a linked list

  \param list  the linked list to copy

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template<class T> BasicArray<T>::BasicArray(BasicList<T> &list) 
{
  BasicNode<T> *node = list.goToFirst() ;
  rsize = sze = 0 ;
  x = 0 ;
  resize(list.size());

  for (int k = rsize; k>0; --k){
    x[k] = *node->data ;
    node = list.goToNext() ;
  }
  destruct = 1 ;
}

/*!
  \brief destructor

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template<class T> BasicArray<T>::~BasicArray()
{
  clear();
}

/*!
  \brief clear

  Clears the memory and reset the size to 0. You must resize the basic array
  before using it again.

  \author Philippe Lavoie 
  \date 17 September 1999
*/
template<class T> void BasicArray<T>::clear()
{
  if(destruct){
    if ( x ) delete []x;
    x = 0;
    rsize = sze = 0;                 
  }
}

/*!
  \brief assignment operator

  \param f2  the BasicArray to copy from
  \return a reference to itself

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template<class T> 
BasicArray<T>& BasicArray<T>::operator=(const BasicArray<T>& f2)
{
  if ( this == &f2 )
    return *this;

  resize(f2.size()) ;
  
  // wdth = f2.wdth;
  T *p1,*p2 ;
  p1 = x-1 ;
  p2 = f2.x-1 ;

  for (int k = sze; k >0; --k)
    *(++p1) = *(++p2) ;

  return *this;	
}

/*!
  \brief shortens the array without destroying the components

  \param nsize  the new size of the vector

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template<class T>
void 
BasicArray<T>::trim(const int nsize)	
{
  if ( nsize >= 0 && nsize <= rsize )
    sze = nsize;
}

/*!
  \brief reset all values of the vetor to \a val

  \param val  resets the elements of the array to this value

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template<class T>
void 
BasicArray<T>::reset(const T val)
{
  T *p1 ;
  p1 = x-1 ;
  for (int k = sze; k >0; --k)
    *(++p1) = val;
}

/*!
  \brief a non-destructive resize of the vector's length

  Will change the size of the vector to \a nsize. If the size 
  is increased, the old data is still accessible at the same 
  old index.

  This version of resize uses memcpy to perform the transfer
  of data. Some types of variables might need to overwrite
  this behavior. 

  \param nsize  the new size of the vector

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
void resizeBasicArray(BasicArray<T>& a, int nsize)
{
  int k;
  
  if ( nsize == a.rsize ){
    a.sze = nsize ;
    return;			// nothing to do
  }
  
  if(a.sze>nsize){
    a.sze = nsize ;
    return ;
  }

  if((a.sze<nsize) && (nsize<a.rsize)){
    for(k=a.sze;k<nsize;++k)
      a.x[k] = T() ;
    a.sze = nsize ;
    return ;
  }

  T *xn ; 
  T *p,*pn ;
  xn = new T[nsize];
  p = a.x ;
  pn = xn ;
  
  if ( a.x )
    {
      // copy the old data
      memcpy((void*)xn,(void*)a.x,a.sze*sizeof(T)) ;
      if(a.sze<nsize)
	memset((void*)(xn+a.sze),0,(nsize-a.sze)*sizeof(T)) ;
      if(a.destruct)
	delete []a.x;
    }
  else
    memset((void*)xn,0,nsize*sizeof(T)) ;
  
  a.rsize = nsize;
  a.sze = a.rsize ;
  a.x = xn;
  a.destruct = 1 ;
  a.wdth = a.rsize + 1;
}


/*!
  \brief stores an input string into a vector

  \param is  the input stream
  \param arry  the array initialized by \a is

  \return \a is after setting \a arry

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template<class T>
istream&  
operator>> (istream& is, BasicArray<T>& arry)
{
  T new_x;
  int k, kend;

  if ( is.eof() )
    return is;
  
  
  kend = arry.size();
  
  for (k = 0; k < kend; ++k)
	{		
	  is >> new_x;	  
	  // end of file reached, abort the read
	  if ( is.eof() || is.fail() )
	    return is;
	  
	  arry[k] = new_x;		
	}
  
  return is;
	
}

/*!
  \brief sends a BasicArray to an ostream
  
  Sends a BasicArray to an ostream

  \param os  output stream

  \return \a os with \a arry piped into it.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template<class T>
ostream& BasicArray<T>::print(ostream& os) const
{
  const int iend = size();
  
  for (int i = 0; i < iend;  )
    {
      os << x[i] ; // << endl;
      if ( (++i % wdth) == 0 )
	os << '\n';
      else
	os << "   "; 
    }
  os << '\n' ;
  
  return os;	
}


/*!
  \brief sends a BasicArray to the ostream
  
  \param os  output stream
  \param arry  the array to send to the stream

  \return \a os with \a arry piped into it.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template<class T>
ostream&  
operator<<(ostream& os,const BasicArray<T>& arry)
{
  return arry.print(os);	
}


/*!
  \brief compares two arrays to see if they're different
  
  This checks if one of the components of the two arrays are 
  different. A check is made after each comparison to see if 
  they are the same, this speeds up the calculation if the 
  arrays have early different components, otherwise the use of 
  the == operator would yield a faster result.

  \param a  the first array to compare
  \param b  the second array to compare

  \return 1 if the arrays are {\em not} identical, 0 otherwise.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template<class T>
int  
operator!=(const BasicArray<T> &a,const BasicArray<T> &b) 
{
  if ( a.size() != b.size() )
    return 1;
  
  const int sz = a.size();
  int l = 1;
  
  for (int i = 0; i < sz; ++i){
    l = l && ( a[i] == b[i] );
    if(!l) break ;
  }
  
  return (l==0 ? 1 : 0);
  
}

/*!
  \brief checks if two arrays are identical
  
  \param a  the first array to compare
  \param b  the second array to compare
  \return 1 if the arrays are identical, 0 otherwise
  \warning
  \author Philippe Lavoie 
  \date 24 January 1997
*/
template<class T>
int  
operator==(const BasicArray<T> &a,const BasicArray<T> &b) 
{
  if ( a.size() != b.size() )
    return 0;
  
  const int sz = a.size();
  int l = 1;
  
  for (int i = 0; i < sz; ++i)
    l = l && ( a[i] == b[i] );
  
  return l; 
}

#ifdef DEBUG_PLIB
template <class T>
T& BasicArray<T>::operator[](const int i) {
  if(i<0 || i>=sze){
#ifdef USE_EXCEPTION
    throw OutOfBound(i,0,n()-1) ;
#else
    Error error("BasicArray<T>::operator[]") ;
    error << "Error Accessing the vector (press C-c to stop)\n" ;
    error << "Bad index " << i << endl ;
    error.fatal() ;
#endif
  }
  return x[i] ;
  
}

template <class T>
T  BasicArray<T>::operator[](const int i) const {
  if(i<0 || i>=n()){
#ifdef USE_EXCEPTION
    throw OutOfBound(i,0,n()-1) ;
#else
    Error error("BasicArray<T>::operator[]") ;
    error << "Error Accessing the vector (press C-c to stop)\n" ;
    error << "Bad index " << i << endl ;
    error.fatal() ;
#endif
  }
  return x[i] ;
}

#endif

/*!
  \brief adds a new element at the end of the vector
  
  
  \param i the element to add
  \param end_buffer allocate that amount of data if the size needs to be increased
  \param end_mult multiplies the size of the buffer by that amount if multiplier is greater than 1.

  \return a reference to the element

  \author Philippe Lavoie 
  \date 12 September 1999
*/
template<class T>  
T& BasicArray<T>::push_back(const T i, int end_buffer, double end_mult){
  if(sze>=rsize){
    int n = sze ; 
    if(end_mult>1.0){
      sze = (int)( double(rsize)*end_mult) ;
      resize(sze);
      resize(n);
    }
    else{
      if(end_buffer<1)
	end_buffer = 1 ; 
      resize(sze+end_buffer);
      resize(n);
    }
  }
  x[sze] = i ; 
  return x[sze];
}

}


#endif //BARRAY_SOURCES
