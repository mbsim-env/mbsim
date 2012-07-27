/*=============================================================================
        File: barray2d.cpp
     Purpose:
    Revision: $Id: barray2d_hpoint.cpp,v 1.2 2002/05/13 21:07:45 philosophil Exp $
  Created by: Philippe Lavoie          (3 Oct, 1996)
 Modified by: 

 Copyright notice:
          Copyright (C) 1996-2002 Philippe Lavoie
 
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

#include "barray2d.cpp"

namespace PLib {

template <class T, const int D>
void initBasic2DArrayHPoint(Basic2DArray<HPoint_nD<T,D> >& a, const int r,const int c)
{
  if ( r <= 0  || c <= 0 )
    {
      if(a.m)
	delete []a.m ;
      if(a.vm)
	delete []a.vm ;
      a.rz = r ;
      a.cz = c ;
      return ;
    }
	
  a.rz = r;	a.cz = c;

  a.created = 1 ;
#ifdef COLUMN_ORDER
  a.vm = new HPoint_nD<T,D>* [a.cz] ;
#else
  a.vm = new HPoint_nD<T,D>* [a.rz] ;
#endif

  T* dn ;
  dn = new T[a.rz*a.cz*(D+1)] ;
  int i ;

  a.m = new NoInitHPoint_nD<T,D>[a.rz*a.cz] ;
  // At this point the elements data pointer points to null
  // and their created status is 0.
  // The following is done so that the data point to 
  // their proper place inside the consecutive memory
  // array allocated inside dn.

#ifdef COLUMN_ORDER
  const int sze = a.rz*a.cz ;
  for(i=sze-1;i>=0;--i){
    a.m[i].data = dn+i*(D+1) ;  
  }
#else
  for(i=a.rz-1;i>=0;--i){
    for(int j=a.cz-1;j>=0;--j){
      a.m[i*a.cz+j].data = dn+(a.rz*j+i)*(D+1) ; 
    }
  }
#endif 

  memset((void*)dn,0,(a.cz*a.rz*(D+1))*sizeof(T)) ;

  // Ok so everyone points to the consecutive array
  // but someone needs to delete it when the times
  // come. That's the job of the first HPoint.
  if((a.rz*a.cz)>0)
    a.m[0].created = 1;

#ifdef COLUMN_ORDER
  for(i=a.cz-1;i>=0;--i)
    a.vm[i] = &a.m[i*a.rz] ;
#else
  for(i=a.rz-1;i>=0;--i)
    a.vm[i] = &a.m[i*a.cz] ;
#endif
}

template <class T, const int D>
void resizeKeepBasic2DArrayHPoint(Basic2DArray<HPoint_nD<T,D> > &a, const int nr,const int nc)
{
  if(nr==a.rz && nc==a.cz){
    return ;
  }

  HPoint_nD<T,D> *mn ;
  HPoint_nD<T,D> *p,*pn ;

  pn = 0 ;

  mn = new NoInitHPoint_nD<T,D>[nr*nc] ;
  // At this point the elements data pointer points to null
  // and their created status is 0.
  // The following is done so that the data point to 
  // their proper place inside the consecutive memory
  // array allocated inside dn.

  T* dn ;
  dn = new T[nr*nc*(D+1)] ;
  memset((void*)dn,0,(nr*nc*(D+1))*sizeof(T)) ;

  int i,j ;
#ifdef COLUMN_ORDER
  for(i=0;i<nr*nc;++i){
    mn[i].created = 0 ; 
    mn[i].data = &dn[i*(D+1)] ;
  }
  for(j=0;j<minimum(nc,a.cz);j++){
    p = &a.m[j*a.rz] -1;
    pn = &mn[j*nr] -1 ;
    for(i=0;i<minimum(nr,a.rz);i++){
      *(++pn) = *(++p) ;
    }
  }

  for(j=a.cz;j<nc;j++)
    pn = &mn[j*nr+a.rz]-1 ;
    for(i=a.rz;i<nr;i++){
      *(pn++) = HPoint_nD<T,D>(0) ;
  }

#else
  for(i=0;i<nr;++i){
    for(j=0;j<nc;++j){
      mn[i*nc+j].created = 0 ; 
      mn[i*nc+j].data = &dn[(nr*j+i)*(D+1)] ;
    }
  }
  for(i=0;i<minimum(nr,a.rz);i++){
    p = &a.m[i*a.cz] -1;
    pn = &mn[i*nc] -1 ;
    for(j=0;j<minimum(nc,a.cz);j++){
      *(++pn) = *(++p) ;
    }
  }

  for(i=a.rz;i<nr;i++){
    pn = &mn[i*nc+a.cz]-1 ;
    for(j=a.cz;j<nc;j++)
      *(pn++) = HPoint_nD<T,D>(0) ;
  }

#endif

  a.rz = nr ;
  a.cz = nc ;

  if(a.m && a.created){
    delete []a.m ;
  }
  a.created = 1 ;
  a.m = mn ;
  // Ok so everyone points to the consecutive array
  // but someone needs to delete it when the times
  // come. That's the job of the first HPoint.
  if((nr*nc)>0)
    a.m[0].created = 1 ;
  if(a.vm)
    delete []a.vm ;
#ifdef COLUMN_ORDER
  a.vm = new HPoint_nD<T,D>* [a.cz] ;
  for(i=0;i<a.cz;++i)
    a.vm[i] = &a.m[i*a.rz] ;
#else
  a.vm = new HPoint_nD<T,D>* [a.rz] ;
  for(i=0;i<a.rz;++i)
    a.vm[i] = &a.m[i*a.cz] ;
#endif
}

template<>
void initBasic2DArray(Basic2DArray<HPoint_nD<float,2> >& a, const int nr, const int nc){
  initBasic2DArrayHPoint(a,nr,nc) ;
}

template<>
void initBasic2DArray(Basic2DArray<HPoint_nD<float,3> >& a, const int nr, const int nc){
  initBasic2DArrayHPoint(a,nr,nc) ;
}

template<>
void initBasic2DArray(Basic2DArray<HPoint_nD<double,2> >& a, const int nr, const int nc){
  initBasic2DArrayHPoint(a,nr,nc) ;
}

template<>
void initBasic2DArray(Basic2DArray<HPoint_nD<double,3> >& a, const int nr, const int nc){
  initBasic2DArrayHPoint(a,nr,nc) ;
}

template<>
void resizeKeepBasic2DArray(Basic2DArray<HPoint_nD<float,2> >& a, const int nr, const int nc){
  resizeKeepBasic2DArrayHPoint(a,nr,nc) ;
}

template<>
void resizeKeepBasic2DArray(Basic2DArray<HPoint_nD<float,3> >& a, const int nr, const int nc){
  resizeKeepBasic2DArrayHPoint(a,nr,nc) ;
}

template<>
void resizeKeepBasic2DArray(Basic2DArray<HPoint_nD<double,2> >& a, const int nr, const int nc){
  resizeKeepBasic2DArrayHPoint(a,nr,nc) ;
}

template<>
void resizeKeepBasic2DArray(Basic2DArray<HPoint_nD<double,3> >& a, const int nr, const int nc){
  resizeKeepBasic2DArrayHPoint(a,nr,nc) ;
}

#ifdef NO_IMPLICIT_TEMPLATES

template class Basic2DArray<HPoint3Df> ;
template void initBasic2DArrayHPoint(Basic2DArray<HPoint3Df>&,const int,const int) ;
template void resizeKeepBasic2DArrayHPoint(Basic2DArray<HPoint3Df>&,const int,const int) ;
template istream& operator>>(istream& is, Basic2DArray<HPoint3Df>& ary);
template ostream& operator<<(ostream& os, const Basic2DArray<HPoint3Df>& ary);

template class Basic2DArray<HPoint2Df> ;
template void initBasic2DArrayHPoint(Basic2DArray<HPoint2Df>&,const int,const int) ;
template void resizeKeepBasic2DArrayHPoint(Basic2DArray<HPoint2Df>&,const int,const int) ;
template istream& operator>>(istream& is, Basic2DArray<HPoint2Df>& ary);
template ostream& operator<<(ostream& os, const Basic2DArray<HPoint2Df>& ary);

template class Basic2DArray<HPoint3Dd> ;
template void initBasic2DArrayHPoint(Basic2DArray<HPoint3Dd>&,const int,const int) ;
template void resizeKeepBasic2DArrayHPoint(Basic2DArray<HPoint3Dd>&,const int,const int) ;
template istream& operator>>(istream& is, Basic2DArray<HPoint3Dd>& ary);
template ostream& operator<<(ostream& os, const Basic2DArray<HPoint3Dd>& ary);

template class Basic2DArray<HPoint2Dd> ;
template void initBasic2DArrayHPoint(Basic2DArray<HPoint2Dd>&,const int,const int) ;
template void resizeKeepBasic2DArrayHPoint(Basic2DArray<HPoint2Dd>&,const int,const int) ;
template istream& operator>>(istream& is, Basic2DArray<HPoint2Dd>& ary);
template ostream& operator<<(ostream& os, const Basic2DArray<HPoint2Dd>& ary);

#endif

}
