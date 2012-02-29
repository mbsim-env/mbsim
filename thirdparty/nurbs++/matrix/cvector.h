/*=====================================================================
        File: cvector.h
     Purpose:       
    Revision: $Id: cvector.h,v 1.2 2002/05/13 21:07:45 philosophil Exp $
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
=====================================================================*/

#ifndef _Matrix_cvector_h_
#define _Matrix_cvector_h_



#include <math.h>
#include "vector.h"

namespace PLib {

  /*!
    \class CVector cvector.h matrix/cvector.h
    \brief A circular vector class
    
    A circular vector class based on the vector class. 
    The usefullness is limited since you can't perform 
    mathematical operators with these vectors and obtain 
    meaningfull results.
    
    \author Philippe Lavoie 
    \date 4 October 1996
  */
  template<class T> class CVector : public Vector<T>
  {
  public:
    CVector() : Vector<T>(), index(0) {;}
    CVector(const int r) : Vector<T>(r), index(0) {;}
    CVector(const CVector<T>& v) : Vector<T>(v), index(v.index) {;}
    CVector(const Vector<T>& v) : Vector<T>(v), index(0)  {;}
    CVector(const BasicArray<T>& v) : Vector<T>(v), index(0)  {;}
    virtual ~CVector() {}
    
    T& operator[](const int i) { return x[i%sze]; }
    T  operator[](const int i) const   { return x[i%sze]; }
    
    void put(T v) { x[index] = v ; index = (index+1)%sze; }
    
  protected:
    int index ;
	  
  };
  
} // end namespace

typedef PLib::CVector<int> CVector_INT ;
typedef PLib::CVector<char> CVector_BYTE ;
typedef PLib::CVector<float> CVector_FLOAT ;
typedef PLib::CVector<double> CVector_DOUBLE ;
typedef PLib::CVector<unsigned char> CVector_UBYTE ;

#endif 
