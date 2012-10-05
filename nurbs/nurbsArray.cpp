/*=====================================================================
        File: nurbsArray.h
     Purpose:       
    Revision: $Id: nurbsArray.cpp,v 1.2 2002/05/13 21:07:46 philosophil Exp $
  Created by: Philippe Lavoie          (7 Oct, 1997)
  

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
=====================================================================*/

#include <nurbs.h>
#include <nurbsS.h>
#include <string.h>

/*!
 */
namespace PLib {

/*!
  \brief Constructor from a pointer to an array of curves

  \author Philippe Lavoie
  \date 24 January 1997 
*/
template <class T, int N>
NurbsCurveArray<T,N>::NurbsCurveArray(NurbsCurve<T,N>* Ca, int s){
  sze = rsize = 0 ;
  resize(s) ;
  for(int i=0;i<n();++i)
    C[i] = &Ca[i] ;
}

/*!
  \brief Initialize the array of curves with a vector of nurbs curve

  \param ca  a pointer to a vector of NURBS curve
  \param size the size of the array

  \author Philippe Lavoie
  \date 24 January 1997 
*/
template <class T, int N>
void NurbsCurveArray<T,N>::init(NurbsCurve<T,N>* ca,int size){
  resize(size) ;
  for(int i=0;i<n();++i)
    C[i] = &ca[i] ;
}

/*!
  \brief Resize the NurbsCurveArray

  \param size the new size

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T, int N>
void NurbsCurveArray<T,N>::resize(int size) {
  int i ;
  if(size<=rsize){
    sze = size ;
    return ;
  }
  NurbsCurve<T,N>** t;
  t = new NurbsCurve<T,N>* [size] ;
  if(C){
    for(i=0;i<rsize;++i)
      t[i] = C[i] ;
    delete []C ;
  }
  for(i=rsize;i<size;++i)
    t[i] = new NurbsCurve<T,N> ;
  C = t ;
  sze = size ;
  rsize = size ;
}

/*!
  \brief Reads a NurbsCurveArray from a file

  \param filename  the filename to read the curve array from 

  \return 0 if an error occurs, 1 otherwise

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T, int N>
int NurbsCurveArray<T,N>::read(const char* filename){
  ifstream fin(filename) ;
  if(!fin) {
    return 0 ;
  }
  int np,d;
  int na ;
  char *type ;
  type = new char[3] ;
  if(!fin.read(type,sizeof(char)*3)) return 0 ;
  int r1 = strncmp(type,"nca",3) ;
  if(!(r1==0))
    return 0 ;
  if(!fin.read((char*)&na,sizeof(int))) return 0 ;

  resize(na) ;

  int nread = 0 ;
  while(nread < na){
    if(!fin.read((char*)&np,sizeof(int))) return 0 ;
    if(!fin.read((char*)&d,sizeof(int))) return 0 ;
    
    operator[](nread).resize(np,d) ;
    
    if(!fin.read((char*)operator[](nread).knot().memory(),sizeof(T)*operator[](nread).knot().n())) return 0 ;
    
    T *p,*p2 ;
    p = new T[4*np] ;
    if(!fin.read((char*)p,sizeof(T)*4*np)) return 0 ;
    p2 = p ;
    for(int i=0;i<np;++i){
      HPoint_nD<T,N> t ;
      t.x() = *(p++) ;
      t.y() = *(p++) ;
      t.z() = *(p++) ;
      t.w() = *(p++) ;
      operator[](nread).modCP(i,t) ;
    }
    delete []p2 ;
    ++nread ;
  }

  delete []type ;
  return 1 ;
}


/*!
  \brief Writes a NurbsCurveArray from a file

  \param filename --> the filename to read the curve array from 

  \return 0 if an error occurs, 1 otherwise

  \author Philippe Lavoie
  \date 24 January 1997 
*/
template <class T, int N>
int NurbsCurveArray<T,N>::write(const char* filename){
  ofstream fout(filename) ;
  if(!fout) {
    return 0 ;
  }
  if(!fout.write((char*)&"nca",sizeof(char)*3)) return 0 ;
  if(!fout.write((char*)&sze,sizeof(int))) return 0 ;

  int nwrote = 0 ;
  while(nwrote < sze){
    int nn = operator[](nwrote).ctrlPnts().n() ;
    int nd = operator[](nwrote).degree() ; 
    if(!fout.write((char*)&nn,sizeof(int))) return 0 ;
    if(!fout.write((char*)&nd,sizeof(int))) return 0 ;
    if(!fout.write((char*)operator[](nwrote).knot().memory(),sizeof(T)*operator[](nwrote).knot().n())) return 0 ;
        
    T *p,*p2 ;
    p = new T[4*operator[](nwrote).ctrlPnts().n()] ;
    p2 = p ;
    for(int i=0;i<operator[](nwrote).ctrlPnts().n();++i){
      *p = operator[](nwrote).ctrlPnts()[i].x() ; ++p ;
      *p = operator[](nwrote).ctrlPnts()[i].y() ; ++p ;
      *p = operator[](nwrote).ctrlPnts()[i].z() ; ++p ;
      *p = operator[](nwrote).ctrlPnts()[i].w() ; ++p ;
    }
    if(!fout.write((char*)p2,sizeof(T)*4*operator[](nwrote).ctrlPnts().n())) return 0 ;
    delete []p2 ;

    ++nwrote ;
  }

  return 1 ;
}

/*!
  \brief Constructor from a pointer to an array of curves

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T, int N>
NurbsSurfaceArray<T,N>::NurbsSurfaceArray(NurbsSurface<T,N>* Sa, int s){
  sze = rsize = 0 ;
  resize(s) ;
  for(int i=0;i<n();++i)
    S[i] = &Sa[i] ;
}

/*!
  \brief Initialize the array of curves with a vector of nurbs curve

  \param ca  a pointer to a vector of NURBS curve
  \param size  the size of the array

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T, int N>
void NurbsSurfaceArray<T,N>::init(NurbsSurface<T,N>* Sa,int size){
  resize(size) ;
  for(int i=0;i<n();++i)
    S[i] = &Sa[i] ;
}

/*!
  \brief Resize the NurbsSurfaceArray

  \param size  the new size

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T, int N>
void NurbsSurfaceArray<T,N>::resize(int size) {
  int i ;
  if(size<=rsize){
    sze = size ;
    return ;
  }
  NurbsSurface<T,N>** t;
  t = new NurbsSurface<T,N>* [size] ;
  if(S){
    for(i=0;i<rsize;++i)
      t[i] = S[i] ;
    delete []S ;
  }
  for(i=rsize;i<size;++i)
    t[i] = new NurbsSurface<T,N> ;
  S = t ;
  sze = size ;
  rsize = size ;
}

/*!
  \brief Copy one surface array to another

  \param S  the array to copy

  \return a reference to itself

  \author Philippe Lavoie
  \date 24 January 1997
*/
template <class T, int N>
NurbsSurfaceArray<T,N>& NurbsSurfaceArray<T,N>::operator=(const NurbsSurfaceArray<T,N>& Sa){
  resize(Sa.n()) ;

  for(int i=0;i<n();++i)
    *(S[i]) = Sa[i] ;
  return *this ;
}


// duplicating some codes from nurbs.cpp
#ifndef INCLUDE_TEMPLATE_SOURCE

template <class T, int N>
inline Point_nD<T,N> project2D(const HPoint_nD<T,N>& p){
  Point_nD<T,N> pnt ;
  if(absolute(p.z()+T(1))>0.0001){
    pnt.x() = p.x()/p.w() ;
    pnt.y() = p.y()/p.w() ;
    //pnt.x() /= p.z()+1 ; 
    //pnt.y() /= p.z()+1 ; 
  }
  else{
    pnt.x() = p.x()/p.w();
    pnt.y() = p.y()/p.w();
  }
  return pnt ;
}

const float offX = 50 ;
const float offY = 70 ;

template <class T>
inline void movePsP(Point_nD<T,3> &p, T magFact){
  p *= magFact ;
  p += Point_nD<T,3>(offX,offY,0) ;
  //p = p*magFact+Point_nD<T,N>(offX,offY,0)  ;
}

template <class T>
inline void movePsP(Point_nD<T,2> &p, T magFact){
  p *= magFact ;
  p += Point_nD<T,2>(offX,offY) ;
  //p = p*magFact+Point_nD<T,N>(offX,offY,0)  ;
}

#endif // !INCLUDE_TEMPLATE_SOURCE


/*!
  \brief Writes a post-script file representing an array of curves

  \param filename  the file to write the postscript file to
  \param cp  a flag indicating if the control points should be 
	                drawn, 0 = no and 1 = yes
  \param magFact  a magnification factor, the 2D point of the control
                        points will be magnified by this value. The size is
		        measured in postscript points. If the magFact is 
		        set to a value smaller or equal to 0, than the 
		        program will try to guess a magnification factor 
		        such that the curve is large enough to fill the 
		        page.
  \param dash  the size of the dash in postscript points . A size 
		           smaller or equal to 0 indicates that 
			   the line joining the control points is plain.

  \return 0 if an error occurs, 1 otherwise

  \warning If the weights of the curve are not all at 1, the result might 
               not be representative of the true NURBS curve.

  \author Philippe Lavoie
  \date 7 October 1998
*/
template <class T, int N>
int NurbsCurveArray<T,N>::writePS(const char* filename,int cp,T magFact, T dash, bool ) const {

  ofstream fout(filename) ;  

  if(!fout)
    return 0 ;

  if(curve(0).degree()<3){
    NurbsCurveArray<T,N> a3 ;
    a3.resize(n()) ;
    for(int i=0;i<sze;++i){
      a3[i] = curve(i) ;
      a3[i].degreeElevate(3 - curve(i).degree()) ;
    }
    return a3.writePS(filename,cp,magFact,dash) ;
  }
  if(curve(0).degree()>3){
    return 0 ; 
  }

  // find bounding box parameters
  T mx,my,Mx,My ;
  mx = Mx = 0 ;
  my = My = 0 ;

  for(int i=0;i<n();++i){
    Point_nD<T,N> p ;
    int step ;
    step = curve(i).ctrlPnts().n() + 5 ;
    for(int j=0;j<=step;++j){
      T u ;
      u = (T)j/(T)step ;
      p = project2D(curve(i)(u)) ;
      if(i==0){
	mx = Mx = p.x() ;
	my = My = p.y() ;
      }
      if(p.x() < mx)
	mx = p.x() ;
      if(p.x() > Mx)
	Mx = p.x() ;
      if(p.y() < my)
	my = p.y() ;
      if(p.y() > My) 
	My = p.y() ;      
    }
  }

  int guess =0 ;
  if(magFact<= T() ){
    magFact = T(1) ;
    guess = 1 ;
  }
  
  if(guess){
    //magFact = minimum((T)500/(T)(Mx-mx),(T)700/(T)(My-my)) ;
  }

  mx = mx*magFact+offX;
  my = my*magFact+offY;
  Mx = Mx*magFact+offX;
  My = My*magFact+offY;


  fout << "%!PS-Adobe-2.1\n%%Title: " << filename << endl ;
  fout << "%%Creator: NurbsCurve<T,N>::writePS\n" ;
  fout << "%%BoundingBox: " << mx << ' ' << my << ' ' << Mx << ' ' << My << endl ;
  fout << "%%Pages: 0" << endl ;
  fout << "%%EndComments" << endl ;
  fout << "0 setlinewidth\n" ;
  fout << "0 setgray\n" ;
  fout << endl ;

  for(int k=0;k<n();++k){
    NurbsCurveArray<T,N> Ca ;
    curve(k).decompose(Ca) ;
    int deg = curve(k).degree() ;

    Matrix< Point_nD<T,N> > pnts(Ca.n(),deg+1) ;
    int i,j ;

    for(i=0;i<Ca.n();++i){
      for(j=0;j<deg+1;++j){
	pnts(i,j) = project2D(Ca[i].ctrlPnts()[j]) ;
	movePsP(pnts(i,j),magFact) ;
      }
    }

    fout << "newpath\n" ;
    fout << pnts(0,0).x() << ' ' << pnts(0,0).y() << " moveto\n" ;
    for(i=0;i<Ca.n();++i){
      for(j=1;j<deg+1;++j){
	fout << pnts(i,j).x() << ' ' << pnts(i,j).y() << ' ' ;
      }
      fout << "curveto\n" ;
    }
    fout << "stroke\n" ;

    if(cp>0){ // draw the control points of the original curve
      Vector< Point_nD<T,N> > pts(curve(k).ctrlPnts().n()) ;
      for(i=0;i<curve(k).ctrlPnts().n();++i){
	pts[i] = project2D(curve(k).ctrlPnts()[i]) ;
	movePsP(pts[i],magFact) ;
	fout << "newpath\n" ;
	fout << pts[i].x() << ' ' << pts[i].y() << "  3 0 360 arc\nfill\n" ;
      }
      if(dash>0)
	fout << "[" << dash << "] " << dash << " setdash\n" ;
      fout << "newpath\n" ;
    
      fout << pts[0].x() << ' ' << pts[0].y() << " moveto\n" ;
      for(i=1;i<curve(k).ctrlPnts().n();++i)
	fout << pts[i].x() << ' ' << pts[i].y() << " lineto\n" ;
      fout << "stroke\n" ;
    }
    else{
      if(cp<0){
	Vector< Point_nD<T,N> > pts(curve(k).ctrlPnts().n()*Ca.n()) ;
	int l=0 ;
	for(i=0;i<Ca.n();++i)
	  for(j=0;j<deg+1;++j){
	    pts[l] = project2D(Ca[i].ctrlPnts()[j]) ;
	    movePsP(pts[l],magFact) ;
	    fout << "newpath\n" ;
	    fout << pts[l].x() << ' ' << pts[l].y() << "  3 0 360 arc\nfill\n" ;
	    ++l ;
	  }
	if(dash>0)
	  fout << "[" << dash << "] " << dash << " setdash\n" ;
	fout << "newpath\n" ;
	
	fout << pts[0].x() << ' ' << pts[0].y() << " moveto\n" ;
	for(i=1;i<l;++i)
	  fout << pts[i].x() << ' ' << pts[i].y() << " lineto\n" ;
	fout << "stroke\n" ;      
      }
    }
  }

  fout << "showpage\n%%EOF\n" ;
  return 1 ;
}

/*!
  \brief Writes a post-script file representing the array of curves

   Writes the array of curves in the postscript format to a 
               file, it also draws the points defined in $points$ with 
	       their associated vectors if $vector$ is used.
  \param filename  the file to write the postscript file to
  \param points  draws these additional points as empty circles
  \param vectors  specify a vector associated with the points
		         (this can be an empty vector)
  \param cp  a flag indicating if the control points should be 
		         drawn, 0 = no and 1 = yes
  \param magFact  a magnification factor, the 2D point of the control
		         points will be magnified by this value. The size 
			 is measured in postscript points. If the magFact 
			 is set to a value smaller or equal to 0, than the 
			 program will try to guess a magnification factor 
			 such that the curve is large enough to fill the 
			 page.
  \param dash  the size of the dash in postscript points . A size
		            smaller or equal to 0 indicates that 
			    the line joining the control points is plain.

  \return 0 if an error occurs, 1 otherwise

  \warning If the weights of the curve are not all at 1, the result might 
               not be representative of the true NURBS curve. If vectors is
	       used, then it must be of the same size as points. If a vector
	       element is (0,0,0) it will not be drawn.

  \author Philippe Lavoie
  \date 7 October 1998
*/
template <class T, int N>
int NurbsCurveArray<T,N>::writePSp(const char* filename,const Vector< Point_nD<T,N> >& points, const Vector< Point_nD<T,N> >& vectors, int cp, T magFact, T dash, bool ) const {

  ofstream fout(filename) ;  

  if(!fout)
    return 0 ;

  if(curve(0).degree()<3){
    NurbsCurveArray<T,N> a3 ;
    a3.resize(n()) ;
    for(int i=0;i<n();++i){
      a3[i] = curve(i) ;
      a3[i].degreeElevate(3-curve(i).degree()) ;
    }
    return a3.writePSp(filename,points,vectors,cp,magFact,dash) ;
  }
  if(curve(0).degree()>3){
    return 0 ; 
  }


  // find bounding box parameters
  T mx,my,Mx,My ;
  mx = Mx = 0 ;
  my = My = 0 ;

  for(int i=0;i<n();++i){
    Point_nD<T,N> p ;
    int step ;
    step = curve(i).ctrlPnts().n() + 5 ;
    for(int j=0;j<=step;++j){
      T u ;
      u = (T)j/(T)step ;
      p = project2D(curve(i)(u)) ;
      if(i==0){
	mx = Mx = p.x() ;
	my = My = p.y() ;
      }
      if(p.x() < mx)
	mx = p.x() ;
      if(p.x() > Mx)
	Mx = p.x() ;
      if(p.y() < my)
	my = p.y() ;
      if(p.y() > My) 
	My = p.y() ;      
    }
  }

  int guess =0 ;
  if(magFact<= T() ){
    magFact = T(1) ;
    guess = 1 ;
  }
  
  if(guess){
    //magFact = minimum((T)500/(T)(Mx-mx),(T)700/(T)(My-my)) ;
  }

  mx = mx*magFact+offX;
  my = my*magFact+offY;
  Mx = Mx*magFact+offX;
  My = My*magFact+offY;


  fout << "%!PS-Adobe-2.1\n%%Title: " << filename << endl ;
  fout << "%%Creator: NurbsCurve<T,N>::writePS\n" ;
  fout << "%%BoundingBox: " << mx << ' ' << my << ' ' << Mx << ' ' << My << endl ;
  fout << "%%Pages: 0" << endl ;
  fout << "%%EndComments" << endl ;
  fout << "0 setlinewidth\n" ;
  fout << "0 setgray\n" ;
  fout << endl ;

  for(int k=0;k<n();++k){
    NurbsCurveArray<T,N> Ca ;
    curve(k).decompose(Ca) ;
    int deg = curve(k).degree() ;

    Matrix< Point_nD<T,N> > pnts(Ca.n(),deg+1) ;
    int i,j ;

    for(i=0;i<Ca.n();++i){
      for(j=0;j<deg+1;++j){
	pnts(i,j) = project2D(Ca[i].ctrlPnts()[j]) ;
	movePsP(pnts(i,j),magFact) ;
      }
    }

    fout << "newpath\n" ;
    fout << pnts(0,0).x() << ' ' << pnts(0,0).y() << " moveto\n" ;
    for(i=0;i<Ca.n();++i){
      for(j=1;j<deg+1;++j){
	fout << pnts(i,j).x() << ' ' << pnts(i,j).y() << ' ' ;
      }
      fout << "curveto\n" ;
    }
    fout << "stroke\n" ;

    if(cp>0){ // draw the control points of the original curve
      Vector< Point_nD<T,N> > pts(curve(k).ctrlPnts().n()) ;
      for(i=0;i<curve(k).ctrlPnts().n();++i){
	pts[i] = project2D(curve(k).ctrlPnts()[i]) ;
	movePsP(pts[i],magFact) ;
	fout << "newpath\n" ;
	fout << pts[i].x() << ' ' << pts[i].y() << "  3 0 360 arc\nfill\n" ;
      }
      if(dash>0)
	fout << "[" << dash << "] " << dash << " setdash\n" ;
      fout << "newpath\n" ;
    
      fout << pts[0].x() << ' ' << pts[0].y() << " moveto\n" ;
      for(i=1;i<curve(k).ctrlPnts().n();++i)
	fout << pts[i].x() << ' ' << pts[i].y() << " lineto\n" ;
      fout << "stroke\n" ;
    }
    else{
      if(cp<0){
	Vector< Point_nD<T,N> > pts(curve(k).ctrlPnts().n()*Ca.n()) ;
	int l=0 ;
	for(i=0;i<Ca.n();++i)
	  for(j=0;j<deg+1;++j){
	    pts[l] = project2D(Ca[i].ctrlPnts()[j]) ;
	    movePsP(pts[l],magFact) ;
	    fout << "newpath\n" ;
	    fout << pts[l].x() << ' ' << pts[l].y() << "  3 0 360 arc\nfill\n" ;
	    l++ ;
	  }
	if(dash>0)
	  fout << "[" << dash << "] " << dash << " setdash\n" ;
	fout << "newpath\n" ;
	
	fout << pts[0].x() << ' ' << pts[0].y() << " moveto\n" ;
	for(i=1;i<l;++i)
	  fout << pts[i].x() << ' ' << pts[i].y() << " lineto\n" ;
	fout << "stroke\n" ;      
      }
    }
  }

  for(int i=0;i<points.n();++i){
    Point_nD<T,N> p ;
    p = points[i] ;
    movePsP(p,magFact) ;
    fout << "newpath\n" ;
    fout << p.x() << ' ' << p.y() << "  3 0 360 arc\nfill\n" ;
  }

  if(vectors.n()==points.n()){
    for(int i=0;i<points.n();++i){
      Point_nD<T,N> p,p2 ;
      p = points[i] ;
      p2 = points[i] + vectors[i] ;
      movePsP(p,magFact) ;
      movePsP(p2,magFact) ;
      fout << "newpath\n" ;
      fout << p.x() << ' ' << p.y() << " moveto\n" ;
      if(dash>0)
	fout << "[" << dash/2.0 << "] " << dash/2.0 << " setdash\n" ;
      fout << p2.x() << ' ' << p2.y() << " lineto\n" ;
      fout << "stroke\n" ;
    }
  }

  fout << "showpage\n%%EOF\n" ;
  return 1 ;
}

} // end namespace
