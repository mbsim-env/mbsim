/*=============================================================================
        File: filter.cpp
     Purpose:
    Revision: $Id: filter.cpp,v 1.2 2002/05/13 21:07:45 philosophil Exp $
  Created by: Philippe Lavoie          (18 February 1999)
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

#ifndef FILTER_SOURCES
#define FILTER_SOURCES

#include "filter.h"
#include "vector.h"

/*!
 */
namespace PLib {

  /*!
   */
  namespace Filter{

    template <class T>
      void median(const Basic2DArray<T>& a, Basic2DArray<T>& b){
      
      Vector<T> med(9) ; 
      b.resize(a.rows(),a.cols());
      for(int i=a.rows()-2;i>0;--i)
	for(int j=a.cols()-2;j>0;--j){
	  int n = -1 ; 
	  for(int k=-1;k<2;++k)
	    for(int l=-1;l<2;++l){
	      med[++n] = a(i+k,j+l) ;
	    }
	  med.qSort();
	  b(i,j) = med[4] ; 
	}
      // have to handle the borders
      for(int i=0;i<a.rows();++i){
	b(i,0) = b(i,1) ; 
	b(i,b.cols()-1) = b(i,b.cols()-2) ;
      }
      for(int i=0;i<a.cols();++i){
	b(0,i) = b(1,i) ; 
	b(b.rows()-1,i) = b(b.rows()-2,i) ;
      }
      // and handling the corners
      b(0,0) = b(1,1) ; 
      b(0,b.cols()-1) = b(1,b.cols()-2) ; 
      b(b.rows()-1,0) = b(b.rows()-2,1) ; 
      b(b.rows()-1,b.cols()-1) = b(b.rows()-2,b.cols()-2) ;           
    }
    
    template <class T>
      void medianT(const Basic2DArray<T>& a, Basic2DArray<T>& b, T value, int op){
      
      Vector<T> med(9) ; 
      b.resize(a.rows(),a.cols());

      if(op>0){

	for(int i=a.rows()-2;i>0;--i)
	  for(int j=a.cols()-2;j>0;--j){
	    int n = -1 ; 
	    for(int k=-1;k<2;++k)
	      for(int l=-1;l<2;++l){
		med[++n] = a(i+k,j+l) ;
	      }
	    med.qSort();
	    --n ;
	    while(n>0 && med[n]>=value){
	      --n ;
	    }
	    b(i,j) = med[n/2] ; 
	  }
      }
      else{

	for(int i=a.rows()-2;i>0;--i)
	  for(int j=a.cols()-2;j>0;--j){
	    int n = -1 ; 
	    for(int k=-1;k<2;++k)
	      for(int l=-1;l<2;++l){
		med[++n] = a(i+k,j+l) ;
	      }
	    med.qSort();
	    n=0;
	    while(n>0 && med[n]<=value){
	      ++n ;
	    }
	    b(i,j) = med[n+(9-n)/2] ; 
	  }
      }
      // have to handle the borders
      for(int i=0;i<a.rows();++i){
	b(i,0) = b(i,1) ; 
	b(i,b.cols()-1) = b(i,b.cols()-2) ;
      }
      for(int i=0;i<a.cols();++i){
	b(0,i) = b(1,i) ; 
	b(b.rows()-1,i) = b(b.rows()-2,i) ;
      }
      // and handling the corners
      b(0,0) = b(1,1) ; 
      b(0,b.cols()-1) = b(1,b.cols()-2) ; 
      b(b.rows()-1,0) = b(b.rows()-2,1) ; 
      b(b.rows()-1,b.cols()-1) = b(b.rows()-2,b.cols()-2) ;           
    }
    
  }

}

#endif
