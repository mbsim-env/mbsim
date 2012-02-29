/*=============================================================================
        File: barray2d.h
     Purpose:       
    Revision: $Id: barray2d.h,v 1.3 2002/05/17 14:52:12 philosophil Exp $
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

#ifndef _Matrix_barray2d_h_
#define _Matrix_barray2d_h_


#include <fstream>
#include <iomanip>
#include "specialType.h"

// Predefining every friend functions 
// This is required by latest ISO C++ draft

/*!
 */
namespace PLib {
  template <class T> class Basic2DArray ;

  template <class T> istream& operator>>(istream& is, Basic2DArray<T>& ary);
  template <class T> ostream& operator<<(ostream& os, const Basic2DArray<T>& ary);


#include "galloc2d.h"

/*!
  \brief A basic templated array class in two dimensions

  This is a basis array class for two dimensionnal storage and 
  retrieval.

  \author Philippe Lavoie 
  \date 4 Oct. 1996
*/
template<class T> class Basic2DArray
{
public:
  int rows() const //!< The number or rows
    { return rz;} 
  int cols() const //!< The number of columns
    { return cz;} 
  Basic2DArray() ;
  Basic2DArray(const int r, const int c) ;
  Basic2DArray(const Basic2DArray<T>& f2);
  Basic2DArray(T* p, const int r, const int c) ;
  
  virtual ~Basic2DArray();
  
  Basic2DArray<T>& operator=(const Basic2DArray<T>& f2);
  
  void resize(const int nr, const int nc); 
  void resize(const Basic2DArray<T>& A) { resize(A.rows(),A.cols()) ; }
  void resizeKeep(const int nr, const int nc) { resizeKeepBasic2DArray(*this,nr,nc) ; }

  void reset(const T val = 0.0);
  T operator=(const T val) //!< resets every elements to val
    { reset(val); return val; } 

  T* operator[](const int i) //!< returns a pointer to the start of a row \a i
    { return vm[i]; } 
  T* operator[](const int i) const //!< returns a pointer to the start of a row \ i
    { return vm[i];} 
  
  T& operator()(const int i,const int j)  //!< calls elem(i,j)   
    { return elem(i,j); } 
  T  operator()(const int i,const int j) const //!< calls elem(i,j)
    { return elem(i,j); } 

  void io_elem_width(int w) //!< width of an element in output streams
    { width = w ; }
  void io_by_rows()   //!< row at a time ASCII I/O
    { by_columns = 0; }  
  void io_by_columns()  //!< column at a time ASCII I/O
    { by_columns = 1; } 

  ostream& print(ostream& os) const ; 

#ifdef HAVE_ISO_FRIEND_DECL
  friend istream& operator>> <>(istream& is, Basic2DArray<T>& ary);
  friend ostream& operator<< <>(ostream& os, const Basic2DArray<T>& ary);
#else
  friend istream& operator>> (istream& is, Basic2DArray<T>& ary);
  friend ostream& operator<< (ostream& os, const Basic2DArray<T>& ary);
#endif

#ifdef DEBUG_PLIB
  T& elem(const int i,const int j);  // returns an error message if the index is out of range
  T  elem(const int i,const int j) const;   // returns an error message if the index is out of range
#else
#ifdef COLUMN_ORDER
  T& elem(const int i,const int j) 
    { return vm[j][i] ; }  // no error message are generated if the index are out of range
  T  elem(const int i,const int j) const
    { return vm[j][i] ; }  // no error message are generated if the index are out of range
#else
  T& elem(const int i,const int j) 
    { return vm[i][j] ; }  // no error message are generated if the index are out of range
  T  elem(const int i,const int j) const
    { return vm[i][j] ; }  // no error message are generated if the index are out of range
#endif
#endif
  
  FRIEND_2DARRAY_ALLOCATOR


  
protected:
  int by_columns; //!< If the output is done by columns
  int width; //!< the size of the output columns
  int rz; //!< the number of rows
  int cz; //!< the number of columns
  T *m;  //!< the memory is allocated here
  T **vm ;  //!< for referencing, vm[i][j] is faster than m[i+cols*j]
  int created ; //!< set if the class allocated the memory

  void init(const int r = 1, const int c = 1) //!< Calls a global function to perform the initialization, necessary to overcome some template problems
    { initBasic2DArray(*this,r,c); }

};

} // end namespace

typedef PLib::Basic2DArray<int> Array2D_INT ;            
typedef PLib::Basic2DArray<char> Array2D_BYTE ;          
typedef PLib::Basic2DArray<double> Array2D_DOUBLE ;      
typedef PLib::Basic2DArray<Complex> Array2D_COMPLEX ;    
typedef PLib::Basic2DArray<unsigned char> Array2D_UBYTE ;
typedef PLib::Basic2DArray<PLib::Point3Df> Array2D_Point3Df ;
typedef PLib::Basic2DArray<PLib::HPoint3Df> Array2D_HPoint3Df ;
typedef PLib::Basic2DArray<PLib::Point3Dd> Array2D_Point3Dd ;
typedef PLib::Basic2DArray<PLib::HPoint3Dd> Array2D_HPoint3Dd ;
typedef PLib::Basic2DArray<PLib::Coordinate> Array2D_Coordinate ;

#ifdef INCLUDE_TEMPLATE_SOURCE
#include "barray2d.cpp"
#include "barray2d_hpoint.cpp"
#endif



#endif

