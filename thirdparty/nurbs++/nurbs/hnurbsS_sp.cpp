/*=====================================================================
        File: hnurbsS_sp.cpp
     Purpose:       
    Revision: $Id: hnurbsS_sp.cpp,v 1.2 2002/05/13 21:07:46 philosophil Exp $
  Created by: Philippe Lavoie          (14 May, 1998)
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
=====================================================================*/
#include <hnurbsS_sp.h>
#include <string.h>

/*!
 */
namespace PLib {

/*!
  \brief Updates the basis value in the U direction

  Updates the basis value at which a control point has 
  maximal influence. It also finds where the control point
  has maximal influence. 

  \warning The degrre in U of the surface must be of 3 or less.

  \author Philippe Lavoie
  \date 14 May, 1998
*/
template <class T, int N>
void HNurbsSurfaceSP<T,N>::updateMaxU() {
  if(this->degU>3){
#ifdef USE_EXCEPTION
    throw NurbsError();
#else
    Error error("HNurbsSurfaceSP<T,N>::updateMaxU()") ;
    error << "This class doesn't support surfaces having a degree of 4 and higher.\n" ;
    error.fatal() ;
#endif
  }
  else{
    maxU.resize(this->P.rows()) ;
    maxAtU_.resize(this->P.rows()) ;
    for(int i=0;i<this->P.rows();++i){
      if(!maxInfluence(i,this->U,this->degU,maxAtU_[i]))
	cerr << "Problem in maxInfluence U!\n" ;
      maxU[i] = nurbsBasisFun(maxAtU_[i],i,this->degU,this->U) ;
    }
    
  }
}

/*!
  \brief Updates the basis value in the V direction

  Updates the basis value at which a control point has 
  maximal influence. It also finds where the control point
  has maximal influence. 

  \warning The degrre in V of the surface must be of 3 or less.

  \author Philippe Lavoie
  \date 14 May, 1998
*/
template <class T, int N>
void HNurbsSurfaceSP<T,N>::updateMaxV() {
  if(this->degV>3){
#ifdef USE_EXCEPTION
    throw NurbsError();
#else
    Error error("HNurbsSurfaceSP<T,N>::updateMaxV()") ;
    error << "This class doesn't support surfaces having a degree of 4 and higher.\n" ;
    error.fatal() ;
#endif
  }
  else{
    maxV.resize(this->P.cols()) ;
    maxAtV_.resize(this->P.cols()) ;
    for(int i=0;i<this->P.cols();++i){
      if(!maxInfluence(i,this->V,this->degV,maxAtV_[i]))
	cerr << "Problem in maxInfluence V!\n" ;
      maxV[i] = nurbsBasisFun(maxAtV_[i],i,this->degV,this->V) ;
    }
    
  }
}

/*!
  \brief Modifies the surface point by a certain value.

  \param i  the row of the surface point
  \param j  the column of the surface point
  \param a  modify the surface point by this value

  \warning The degree in U and V of the surface must be of 3 or less.

  \author Philippe Lavoie
  \date 14 May, 1998
*/
template<>
template <class T, int N>
void HNurbsSurfaceSP<T,N>::modSurfCPby(int i, int j, const HPoint_nD<T,N>& a) {
  this->offset(i,j) +=  a / (maxU[i]*maxV[j]) ; 
  if(this->baseLevel_){
    Point_nD<T,N> vecOffset ; 
    vecOffset = this->offset(i,j).x()*this->ivec(i,j) +
      this->offset(i,j).y()*this->jvec(i,j) +
      this->offset(i,j).z()*this->kvec(i,j) ;
    this->P(i,j).x() = this->baseSurf.ctrlPnts()(i,j).x()+vecOffset.x() ;
    this->P(i,j).y() = this->baseSurf.ctrlPnts()(i,j).y()+vecOffset.y() ;
    this->P(i,j).z() = this->baseSurf.ctrlPnts()(i,j).z()+vecOffset.z() ;
  }
  else
    this->P(i,j) = this->offset(i,j) ; 
}

/*!
  \brief Moves the surface point only

  Moves only the specified surface point. The other surface
  points normally affected by moving this point are {\em not}
  moved.
  
  The point a is in the 4D homogenous space, but only
  the x,y,z value are used. The weight is not moved by 
  this function.

  \param i  the row of the surface point to move
  \param j  the column of the surface point to move
  \param a  move that surface point by that amount.

  \warning The degree of the curve must be of 3 or less.

  \author Philippe Lavoie
  \date 7 June, 1998
*/
template <class T, int N>
void HNurbsSurfaceSP<T,N>::modOnlySurfCPby(int i, int j, const HPoint_nD<T,N>& a){
  int k ; 

  this->P = this->offset ; 

  // by definition the offset has w = 0 , but this isn't valid for
  // the control points, increasing the w by 1, will generate a valid surface
  if(this->baseLevel_)
    for(k=0;k<this->P.rows();++k)
      for(int l=0;l<this->P.cols();++l)
	this->P(k,l).w() += T(1) ; 

  int sizeU, sizeV ;

  sizeU = 2*this->degU+3 ; 
  if(i-this->degU-1<0) sizeU += i-this->degU-1 ; 
  if(i+this->degU+1>=this->P.rows()) sizeU -= i+this->degU+1-this->P.rows() ;

  sizeV = 2*this->degV+3 ;
  if(j-this->degV-1<0) sizeV += j-this->degV-1 ; 
  if(j+this->degV+1>=this->P.cols()) sizeV -= j+this->degV+1-this->P.cols() ;
  
  Vector<T> u(sizeU) ;
  Vector<T> v(sizeV) ;
  Vector<Point_nD<T,N> > pts(sizeU*sizeV) ; 
  Vector<int> pu(sizeU*sizeV) ;
  Vector<int> pv(sizeU*sizeV) ;

  int n=0;
  int nu = 0 ;
  int nv = 0 ; 
  for(k=i-this->degU-1;k<=i+this->degU+1;++k){
    if(k<0)
      continue ;
    if(k>=this->P.rows())
      break ; 
    nv = 0 ;
    for(int l=j-this->degV-1;l<=j+this->degV+1;++l){
      if(l<0)
	continue ;
      if(l>=this->P.cols())
	break ; 
      if( k == i && j==l){
	pts[n].x() = a.x() ; 
	pts[n].y() = a.y() ; 
	pts[n].z() = a.z() ; 
      }
      //else
      //pts[n] = Point_nD<T,N>(0,0,0) ;
      pu[n] = nu ; 
      pv[n] = nv ; 
      if(k==i){
	v[nv] = maxAtV_[l] ; // only need to initialise this once
      }
      ++n ;
      ++nv ; 
    }  
    u[nu] = maxAtU_[k] ;
    ++nu ; 
  }

  u.resize(nu) ;
  v.resize(nv) ; 
  pts.resize(n) ;
  pu.resize(n) ; 
  pv.resize(n) ; 

  if(NurbsSurface<T,N>::movePoint(u,v,pts,pu,pv)){
    this->offset = this->P ; 
    // an offset shouldn't have a weight value.
    if(this->baseLevel_)
      for(k=0;k<this->P.rows();++k)
	for(int l=0;l<this->P.cols();++l)
	  this->offset(k,l).w() -= T(1) ; 
  }
  updateSurface(); 
}

/*
  not defined for now. I'm not sure we need it/want it.

 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  Modifies the surface point to a certain value.
  \param i  the row of the surface point
	       j  the column of the surface point
	       a  modify the surface point to this value
  \return
  \warning The degree in U and V of the surface must be of 3 or less.

  \author Philippe Lavoie
  \date 14 May, 1998
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*void HNurbsSurfaceSP<T,N>::modSurfCP(int i, int j, const HPoint_nD<T,N>& a) {
  modSurfCPby(i,j,a-surfP(i,j)) ;
}
*/

/*!
  \brief Adds a level to this HNURBS surface

  \param n  the number of new knots between each knots.
  \param s  the multiplicity of each of these knots

  \return a pointer to the new level or 0 if there was an error.

  \warning returns 0, if there is already a nextlevel.

  \author Philippe Lavoie
  \date 14 May 1998
*/
template <class T, int N>
HNurbsSurfaceSP<T,N>* HNurbsSurfaceSP<T,N>::addLevel(int n, int s) {
  HNurbsSurfaceSP<T,N> *newLevel ;

  if(this->nextLevel_)
    return 0 ;

  Vector<T> newU,newV ;
  
  splitUV(n,s,n,s,newU,newV) ;

  newLevel = new HNurbsSurfaceSP<T,N>(this,newU,newV) ;

  return newLevel ;
}

/*!
  \brief Adds a level to this HNURBS surface

  \param n  the number of new knots between each knots.

  \return a pointer to the new level or 0 if there was an error.
  \warning returns 0, if there is already a nextlevel.

  \author Philippe Lavoie
  \date 14 May 1998
*/
template <class T, int N>
HNurbsSurfaceSP<T,N>* HNurbsSurfaceSP<T,N>::addLevel() {
  HNurbsSurfaceSP<T,N> *newLevel ;

  if(this->nextLevel_)
    return 0 ;

  newLevel = new HNurbsSurfaceSP<T,N>(this) ;

  return newLevel ;
}

/*!
  \brief Copies a HNurbs Surface and all it children

  \param ns  the HNurbs surface to copy

  \author Philippe Lavoie
  \date 14 May 1998
*/
template <class T, int N>
void HNurbsSurfaceSP<T,N>::copy(const HNurbsSurface<T,N>& nS){
  HNurbsSurface<T,N> *levelP ;
  levelP = nS.nextLevel() ;

  NurbsSurface<T,N>::operator=(nS) ;
  this->rU = nS.rU ;
  this->rV = nS.rV ;
  this->offset = nS.offset ;

  updateMaxUV() ; 

  this->firstLevel_ = this ;

  if(levelP){
    HNurbsSurfaceSP<T,N> *newLevel ;
    newLevel =  new HNurbsSurfaceSP<T,N>(this) ; 
    newLevel->copy(*levelP) ;
    this->nextLevel_ = newLevel ;
    this->lastLevel_ = this->nextLevel_->lastLevel() ;
  }
  else{
    this->lastLevel_ = this ;
  }

}

/*!
  \brief Updates the NURBS surface

  Updates the NURBS surface according to the offset values
  and its base level. You can update only one control point 
  from the surface if you specify a value for i and j or you 
  can update all the points if i0 or j0 is below 0.

  \param i0  the row of the control point to update 
  \param j0  the column of the control point to update 

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
void HNurbsSurfaceSP<T,N>::updateSurface(int i0, int j0){
  if(i0>=0 && j0>=0){
    if(this->offset(i0,j0).x()==0.0 && this->offset(i0,j0).y()==0.0 && this->offset(i0,j0).z()==0.0)
      return ;
  }
  if(this->baseLevel_){
    if(this->initBase()){
      this->P = this->baseSurf.ctrlPnts() ;
      this->U = this->baseSurf.knotU() ;
      this->V = this->baseSurf.knotV() ;
      this->degU = this->baseSurf.degreeU() ;
      this->degV = this->baseSurf.degreeV() ;
      updateMaxUV() ; 
    }
    if(i0>=0 && j0>=0){
      Point_nD<T,N> vecOffset ;
      vecOffset = this->offset(i0,j0).x()*this->ivec(i0,j0) +
	this->offset(i0,j0).y()*this->jvec(i0,j0) +
	this->offset(i0,j0).z()*this->kvec(i0,j0) ;
      this->P(i0,j0).x() = this->baseSurf.ctrlPnts()(i0,j0).x()+vecOffset.x() ;
      this->P(i0,j0).y() = this->baseSurf.ctrlPnts()(i0,j0).y()+vecOffset.y() ;
      this->P(i0,j0).z() = this->baseSurf.ctrlPnts()(i0,j0).z()+vecOffset.z() ;
    }
    else{
      for(int i=0;i<this->P.rows();++i)
	for(int j=0;j<this->P.cols();++j){
	  if(this->offset(i,j).x() != 0 || 
	     this->offset(i,j).y() != 0 || this->offset(i,j).z() || 0){
	    Point_nD<T,N> vecOffset ;
	    vecOffset = this->offset(i,j).x()*this->ivec(i,j) +
	      this->offset(i,j).y()*this->jvec(i,j) +
	      this->offset(i,j).z()*this->kvec(i,j) ;
	    this->P(i,j).x() = this->baseSurf.ctrlPnts()(i,j).x()+vecOffset.x() ;
	    this->P(i,j).y() = this->baseSurf.ctrlPnts()(i,j).y()+vecOffset.y() ;
	    this->P(i,j).z() = this->baseSurf.ctrlPnts()(i,j).z()+vecOffset.z() ;
	  }
	}
    }
  }
  else{
    if(i0>=0 && j0>=0)
      this->P(i0,j0) = this->offset(i0,j0) ;
    else{
      for(int i=0;i<this->P.rows();++i)
	for(int j=0;j<this->P.cols();++j){
	  this->P(i,j) = this->offset(i,j) ;
	}
    }
  }

  this->updateN++ ;
}

/*!
  \brief Update the surface for all the levels

  \param upLevel  updates the levels up to this level of detail

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
void HNurbsSurfaceSP<T,N>::updateLevels(int upLevel){
  if(!okMax())
    updateMaxUV() ; 
  if(upLevel>=0){
    if(this->level()<=upLevel){
      this->updateSurface() ;
    }
  }
  else{
    this->updateSurface() ;
  }

  if(upLevel>this->level() || upLevel<0){
    if(this->nextLevel_)
      ((HNurbsSurfaceSP<T,N>*)this->nextLevel_)->updateLevels(upLevel) ;
  }
}

/*!
  \brief Read a HNURBS surface from an input file stream.

  \param fin  the input file stream

  \return 0 if an error occurs, 1 otherwise

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
int HNurbsSurfaceSP<T,N>::read(ifstream &fin){
  if(!fin) {
    return 0 ;
  }
  int nu,nv,du,dv;
  char *type ;
  type = new char[4] ;
  if(!fin.read(type,sizeof(char)*4)) { delete []type ; return 0 ;}
  int r1 = strncmp(type,"hns3",4) ;
  int r2 = strncmp(type,"hns4",4) ;
  int r3 = strncmp(type,"hnso",4) ;
  if(!(r1 || r2 || r3)) 
    return 0 ;
  T *p,*p2 ;
  if(!r1 || !r2){
    if(!fin.read((char*)&nu,sizeof(int))) { delete []type ; return 0 ;}
    if(!fin.read((char*)&nv,sizeof(int))) { delete []type ; return 0 ;}
    if(!fin.read((char*)&du,sizeof(int))) { delete []type ; return 0 ;}
    if(!fin.read((char*)&dv,sizeof(int))) { delete []type ; return 0 ;}
    
    this->resize(nu,nv,du,dv) ;
    
    if(!fin.read((char*)this->U.memory(),sizeof(T)*this->U.n())) { delete []type ; return 0 ;}
    if(!fin.read((char*)this->V.memory(),sizeof(T)*this->V.n())) { delete []type ; return 0 ;}
    
    if(!r1){
      p = new T[3*nu*nv] ;
      if(!fin.read((char*)p,sizeof(T)*3*nu*nv)) { delete []type ; return 0 ;}
      p2 = p ;
      for(int i=0;i<nu;i++)
	for(int j=0;j<nv;j++){
	  this->P(i,j).x() = *(p++) ;
	  this->P(i,j).y() = *(p++) ;
	  this->P(i,j).z() = *(p++) ;
	  this->P(i,j).w() = 1.0 ;
	}
      delete []p2 ;
    }
    else{
      p = new T[4*nu*nv] ;
      if(!fin.read((char*)p,sizeof(T)*4*nu*nv)) { delete []type ; return 0 ;}
      p2 = p ;
      for(int i=0;i<nu;i++)
	for(int j=0;j<nv;j++){
	  this->P(i,j).x() = *(p++) ;
	  this->P(i,j).y() = *(p++) ;
	  this->P(i,j).z() = *(p++) ;
	  this->P(i,j).w() = *(p++) ;
	}
      delete []p2 ;
    }
    this->offset = this->P ;
    this->updateSurface() ;
  }
  else { // reading the offset information
    int ru,rv ;
    if(!fin.read((char*)&ru,sizeof(int))) { delete []type ; return 0 ;}
    if(!fin.read((char*)&rv,sizeof(int))) { delete []type ; return 0 ;}
    this->rU.resize(ru) ;
    this->rV.resize(rv) ;
    if(this->rU.n()>0)
      if(!fin.read((char*)this->rU.memory(),sizeof(T)*this->rU.n())) { delete []type ; return 0 ;}
    if(this->rV.n()>0)
      if(!fin.read((char*)this->rV.memory(),sizeof(T)*this->rV.n())) { delete []type ; return 0 ;}
    
    if(!fin.read((char*)&nu,sizeof(int))) { delete []type ; return 0 ;}
    if(!fin.read((char*)&nv,sizeof(int))) { delete []type ; return 0 ;}

    p = new T[4*nu*nv] ;
    if(!fin.read((char*)p,sizeof(T)*4*nu*nv)) { delete []type ; return 0 ;}
    p2 = p ;
    this->offset.resize(nu,nv) ;
    for(int i=0;i<nu;i++)
      for(int j=0;j<nv;j++){
	this->offset(i,j).x() = *(p++) ;
	this->offset(i,j).y() = *(p++) ;
	this->offset(i,j).z() = *(p++) ;
	this->offset(i,j).w() = *(p++) ;
      }
    delete []p2 ;    
    this->baseUpdateN-- ;
    this->updateSurface() ;
  }

  // now we must read all the level information (if any)
  char *ptxt ;
  ptxt = new char[7] ;
  // the following line is used so that purify doesn't give a warning
  ptxt[0] = ptxt[1] = ptxt[2] = ptxt[3] = ptxt[4] = ptxt[5] = ptxt[6] = 0 ;
  int mark  = fin.tellg() ;
  if(!fin.read(ptxt,sizeof(char)*5))
    { delete []ptxt ; delete []type ; return 1 ; }
  char *pat ;
  pat = strstr(ptxt,"level") ;
  if(pat){
    // insert a new level
    HNurbsSurfaceSP<T,N> *newPatch = new HNurbsSurfaceSP<T,N>(this) ; 
    if(newPatch){
      if(!newPatch->read(fin)) 
	return 0 ;      
    }
    else
      return 0 ;
  }
  else{ 
    // something else is after the HNurbs
    // restore were the file was before the read
    fin.seekg(mark);
  }

  delete []ptxt ;
  delete []type ;
  return 1 ;
}

} // end namespace
