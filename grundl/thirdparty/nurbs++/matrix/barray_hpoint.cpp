/*=============================================================================
        File: barray_hpoint.cpp
     Purpose:       
    Revision: $Id: barray_hpoint.cpp,v 1.4 2002/05/21 15:52:03 philosophil Exp $
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

#include "barray.cpp"

namespace PLib {

template <>
void resizeBasicArray(BasicArray<HPoint_nD<float,2> >& a, int nsize){
  resizeBasicArrayHPoint(a,nsize) ;
}

template <>
void resizeBasicArray<HPoint_nD<float,3> >(BasicArray<HPoint_nD<float,3> >& a, int nsize){
  resizeBasicArrayHPoint(a,nsize) ;
}

template <>
void resizeBasicArray(BasicArray<HPoint_nD<double,2> >& a, int nsize){
  resizeBasicArrayHPoint(a,nsize) ;
}

template <>
void resizeBasicArray(BasicArray<HPoint_nD<double,3> >& a, int nsize){
  resizeBasicArrayHPoint(a,nsize) ;
}

template <class T, const int D>
void resizeBasicArrayHPoint(BasicArray<HPoint_nD<T,D> >& a, int nsize){
  int k=0; 
  
  if ( nsize == a.rsize ){
    a.sze = nsize ;
    return;			// nothing to do
  }
  
  if(a.sze>nsize){
    a.sze = nsize ;
    return ;
  }

  if((a.sze<nsize) && (nsize<a.rsize)){
    memset((void*)(a.x[k].data),0,(nsize-a.sze)*(D+1)*sizeof(T)) ; 
    a.sze = nsize ;
    return ;
  }

  HPoint_nD<T,D> *xn ; 

  xn = new NoInitHPoint_nD<T,D>[nsize] ; 
  T* data = new T[nsize*(D+1)];
  
  for(k=nsize-1;k>=0;--k){
    xn[k].data = data+ k*(D+1);
  }
  // The first data object will be responsible to delete the data array.
  xn[0].created = 1; 

  if ( a.x )    {
      // copy the old data
    memcpy((void*)(xn[0].data),(void*)(a.x[0].data),a.sze*(D+1)*sizeof(T)) ; 
    if(a.sze<nsize)
      memset((void*)(xn[a.sze].data),0,(D+1)*sizeof(T)*(nsize-a.sze)) ;
    if(a.sze>0 && a.destruct){
      delete []a.x;
    }
  }
  else
    memset((void*)(xn[0].data),0,nsize*(D+1)*sizeof(T)) ;
  
  a.rsize = nsize;
  a.sze = a.rsize ;
  a.x = xn;
  a.destruct = 1 ;
  
  a.wdth = a.rsize + 1; 
}



#ifdef NO_IMPLICIT_TEMPLATES

  template class BasicArray<HPoint3Df> ;
  template void resizeBasicArrayHPoint(BasicArray<HPoint3Df>&,int) ;
  template int operator!=(const BasicArray<HPoint3Df>&,const BasicArray<HPoint3Df>&); 
  template int operator==(const BasicArray<HPoint3Df>&,const BasicArray<HPoint3Df>&); 
  template istream& operator>>(istream& is, BasicArray<HPoint3Df>& ary);
  template ostream& operator<<(ostream& os, const BasicArray<HPoint3Df>& ary);

  template class BasicArray<HPoint3Dd> ;
  template void resizeBasicArrayHPoint(BasicArray<HPoint3Dd>&,int) ;
  template int operator!=(const BasicArray<HPoint3Dd>&,const BasicArray<HPoint3Dd>&); 
  template int operator==(const BasicArray<HPoint3Dd>&,const BasicArray<HPoint3Dd>&); 
  template istream& operator>>(istream& is, BasicArray<HPoint3Dd>& ary);
  template ostream& operator<<(ostream& os, const BasicArray<HPoint3Dd>& ary);

  template class BasicArray<HPoint2Df> ;
  template void resizeBasicArrayHPoint(BasicArray<HPoint2Df>&,int) ;
  template int operator!=(const BasicArray<HPoint2Df>&,const BasicArray<HPoint2Df>&); 
  template int operator==(const BasicArray<HPoint2Df>&,const BasicArray<HPoint2Df>&); 
  template istream& operator>>(istream& is, BasicArray<HPoint2Df>& ary);
  template ostream& operator<<(ostream& os, const BasicArray<HPoint2Df>& ary);

  template class BasicArray<HPoint2Dd> ;
  template void resizeBasicArrayHPoint(BasicArray<HPoint2Dd>&,int) ;
  template int operator!=(const BasicArray<HPoint2Dd>&,const BasicArray<HPoint2Dd>&); 
  template int operator==(const BasicArray<HPoint2Dd>&,const BasicArray<HPoint2Dd>&); 
  template istream& operator>>(istream& is, BasicArray<HPoint2Dd>& ary);
  template ostream& operator<<(ostream& os, const BasicArray<HPoint2Dd>& ary);

#endif

}
