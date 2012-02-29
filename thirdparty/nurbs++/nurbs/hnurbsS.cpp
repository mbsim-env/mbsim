/*=============================================================================
        File: hnurbsS.cpp
     Purpose:       
    Revision: $Id: hnurbsS.cpp,v 1.3 2002/05/17 18:24:21 philosophil Exp $
  Created by: Philippe Lavoie    (7 October 1997)
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
#include <hnurbsS.h>
#include <string.h>

/*!
 */
namespace PLib {

/*!
  \brief The basic constructor

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
HNurbsSurface<T,N>::HNurbsSurface():NurbsSurface<T,N>(), fixedOffset(0){

  baseLevel_ = 0 ;
  nextLevel_ = 0 ;
  firstLevel_ = lastLevel_ = this ;

  rU.resize(0) ;
  rV.resize(0) ;
  updateN = 0 ;
  baseUpdateN = 0 ;

  level_ = 0 ;
}

/*!
  \brief Constructor with a base level

  \param base  the base level
  \param xU  the U knots to insert in this level
  \param xV  the V knots to insert in this level

  \warning The base pointer must be pointing to a valid object

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
HNurbsSurface<T,N>::HNurbsSurface(HNurbsSurface<T,N>* base, const Vector<T>& xU, const Vector<T>& xV):NurbsSurface<T,N>(),fixedOffset(0){

  if(!base){
    Error error("HNurbsSurface<T,N> constructor") ;
    error << "Initializing a HNurbsSurface<T,N> with a null base pointer!" ;
    error.fatal() ;
  }
  if(base->nextLevel_){
    Error error("HNurbsSurface<T,N> constructor") ;
    error << "You're trying to replace an existing level, this is not allowed." ;
    error.fatal() ;
  }


  nextLevel_ = 0 ;
  firstLevel_ = base->firstLevel_ ;
  lastLevel_ = this ;
  baseLevel_ = (HNurbsSurface<T,N>*)base ;

  // update the information in the previous levels
  baseLevel_->nextLevel_ = this ;
  HNurbsSurface<T,N>* l ;
  l = baseLevel_ ;
  while(l){
    l->lastLevel_ = this ;
    l = l->baseLevel_ ;
  }


  level_ = base->level_ + 1 ;

  rU = xU ;
  rV = xV ;

  updateN = 0 ;
  baseUpdateN = baseLevel_->modifiedN()-1 ; // Set it so that initBaseLevel will run

  initBase() ;
  offset.resize(baseSurf.ctrlPnts()) ;

  P = baseSurf.ctrlPnts() ;
  U = baseSurf.knotU() ;
  V = baseSurf.knotV() ;
  degU = baseSurf.degreeU() ;
  degV = baseSurf.degreeV() ;

  //updateSurface() ;

}

/*!
  \brief Constructor with a base level

  \param base  the base level
  \param xU  the U knots to insert in this level
  \param xV  the V knots to insert in this level

  \warning The base pointer must be pointing to a valid object

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
HNurbsSurface<T,N>::HNurbsSurface(HNurbsSurface<T,N>* base):NurbsSurface<T,N>(),fixedOffset(0){

  if(!base){
    Error error("HNurbsSurface<T,N> constructor") ;
    error << "Initializing a HNurbsSurface<T,N> with a null base pointer!" ;
    error.fatal() ;
  }
  if(base->nextLevel_){
    Error error("HNurbsSurface<T,N> constructor") ;
    error << "You're trying to replace an existing level, this is not allowed." ;
    error.fatal() ;
  }

  nextLevel_ = 0 ;
  firstLevel_ = base->firstLevel_ ;
  lastLevel_ = this ;
  baseLevel_ = (HNurbsSurface<T,N>*)base ;

  // update the information in the previous levels
  baseLevel_->nextLevel_ = this ;
  HNurbsSurface<T,N>* l ;
  l = baseLevel_ ;
  while(l){
    l->lastLevel_ = this ;
    l = l->baseLevel_ ;
  }

  level_ = base->level_ + 1 ;

  updateN = 0 ;
  rU.resize(0) ;
  rV.resize(0) ;

  baseUpdateN = baseLevel_->modifiedN()-1 ; // Set it so that initBase will run
  initBase() ;
  offset.resize(baseSurf.ctrlPnts()) ;
  P = baseSurf.ctrlPnts() ;
  U = baseSurf.knotU() ;
  V = baseSurf.knotV() ;
  degU = baseSurf.degreeU() ;
  degV = baseSurf.degreeV() ;
  //updateSurface() ;

}

/*!
  \brief Constructs a base HNURBS

  Constructs a base HNURBS. This HNURBS surface is set to level 0.
  And it corresponds to the NURBS surface. 
	       
  This constructor does not transform the NURBS surface into
  a HNURBS surface. It only copies the values from the NURBS
  surface as it's base offset values.

  \param S  the NURBS surface at level 0

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
HNurbsSurface<T,N>::HNurbsSurface(const NurbsSurface<T,N>& S):NurbsSurface<T,N>(S),fixedOffset(0){

  baseLevel_ = 0 ;
  nextLevel_ = 0 ; 
  firstLevel_ = lastLevel_ = this ;

  level_ = 0 ;
  updateN = 0 ;
  baseUpdateN = 0 ;

  rU.resize(0) ;
  rV.resize(0) ;

  offset = P ;
}

/*!
  \brief The copy constructor

  \param S  the HNURBS surface to copy

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
HNurbsSurface<T,N>::HNurbsSurface(const HNurbsSurface<T,N>& S):NurbsSurface<T,N>(S),fixedOffset(0) {

  baseLevel_ = nextLevel_ = 0 ;
  firstLevel_ = lastLevel_ = this ;

  level_ = 0 ;
  updateN = 0 ;
  baseUpdateN = 0 ;

  rU = S.rU ; 
  rV = S.rV ; 

  offset = S.offset ;

  this->copy(S) ;

}

/*!
  \brief A level constructor

  \param base the base of this level
  \param the values for this new level

  \warning The base pointer must be pointing to a valid object

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
HNurbsSurface<T,N>::HNurbsSurface(HNurbsSurface<T,N> *base, const HNurbsSurface<T,N> &surf):NurbsSurface<T,N>(surf),fixedOffset(0) {

  if(!base){
    Error error("HNurbsSurface<T,N> constructor") ;
    error << "Initializing a HNurbsSurface<T,N> with a null base pointer!" ;
    error.fatal() ;
  }
  if(base->nextLevel_){
    Error error("HNurbsSurface<T,N> constructor") ;
    error << "You're trying to replace an existing level, this is not allowed." ;
    error.fatal() ;
  }

  baseLevel_ = (HNurbsSurface<T,N>*)base ;
  nextLevel_ = 0 ;
  lastLevel_ = this ;

  // update the information in the previous levels
  baseLevel_->nextLevel_ = this ;
  HNurbsSurface<T,N>* l ;
  l = baseLevel_ ;
  while(l){
    l->lastLevel_ = this ;
    l = l->baseLevel_ ;
  }

  firstLevel_ = base->firstLevel_ ;
  level_ = base->level_+1 ;
  baseUpdateN = base->modifiedN()-1 ;

  initBase() ; 

  updateN = 0 ;

  this->copy(surf) ;

}

/*!
  \brief Copies a HNurbs Surface and all it children

  \param ns  the HNurbs surface to copy

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
void HNurbsSurface<T,N>::copy(const HNurbsSurface<T,N>& nS){
  HNurbsSurface<T,N> *levelP ;
  levelP = nS.nextLevel() ;

  NurbsSurface<T,N>::operator=(nS) ;
  rU = nS.rU ;
  rV = nS.rV ;
  offset = nS.offset ;
  fixedOffset= nS.fixedOffset ;

  firstLevel_ = this ;

  if(levelP){
    HNurbsSurface<T,N> *newLevel = new HNurbsSurface(this,*levelP) ;
    nextLevel_ = newLevel ;
    lastLevel_ = nextLevel_->lastLevel() ;
  }
  else{
    lastLevel_ = this ;
  }

}

/*!
  \brief updates the NURBS surface

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
void HNurbsSurface<T,N>::updateSurface(int i0, int j0){
  if(i0>=0 && j0>=0){
    if(offset(i0,j0).x()==0.0 && offset(i0,j0).y()==0.0 && offset(i0,j0).z()==0.0)
      return ;
  }
  if(baseLevel_){
    if(initBase()){
      P = baseSurf.ctrlPnts() ;
      U = baseSurf.knotU() ;
      V = baseSurf.knotV() ;
      degU = baseSurf.degreeU() ;
      degV = baseSurf.degreeV() ;
    }
    if(i0>=0 && j0>=0){
      Point_nD<T,N> vecOffset ;
      if(fixedOffset){
	vecOffset = offset(i0,j0).x()*ivec(0,0) +
	  offset(i0,j0).y()*jvec(0,0) +
	  offset(i0,j0).z()*kvec(0,0) ;
      }
      else{
	vecOffset = offset(i0,j0).x()*ivec(i0,j0) +
	  offset(i0,j0).y()*jvec(i0,j0) +
	  offset(i0,j0).z()*kvec(i0,j0) ;
      }
      P(i0,j0).x() = baseSurf.ctrlPnts()(i0,j0).x()+vecOffset.x() ;
      P(i0,j0).y() = baseSurf.ctrlPnts()(i0,j0).y()+vecOffset.y() ;
      P(i0,j0).z() = baseSurf.ctrlPnts()(i0,j0).z()+vecOffset.z() ;
    }
    else{
      for(int i=0;i<P.rows();++i)
	for(int j=0;j<P.cols();++j){
	  if(offset(i,j).x() != 0 || 
	     offset(i,j).y() != 0 || offset(i,j).z() || 0){
	    Point_nD<T,N> vecOffset ;
	    if(fixedOffset){
	      vecOffset = offset(i,j).x()*ivec(0,0) +
		offset(i,j).y()*jvec(0,0) +
		offset(i,j).z()*kvec(0,0) ;
	    }
	    else{
	      vecOffset = offset(i,j).x()*ivec(i,j) +
		offset(i,j).y()*jvec(i,j) +
		offset(i,j).z()*kvec(i,j) ;
	    }
	    P(i,j).x() = baseSurf.ctrlPnts()(i,j).x()+vecOffset.x() ;
	    P(i,j).y() = baseSurf.ctrlPnts()(i,j).y()+vecOffset.y() ;
	    P(i,j).z() = baseSurf.ctrlPnts()(i,j).z()+vecOffset.z() ;
	  }
	}
    }
  }
  else{
    if(i0>=0 && j0>=0)
      P(i0,j0) = offset(i0,j0) ;
    else{
      for(int i=0;i<P.rows();++i)
	for(int j=0;j<P.cols();++j){
	  P(i,j) = offset(i,j) ;
	}
    }
  }

  ++updateN ;
}


/*!
  \brief Initialize the base surface 

  Initialize the base surface from the previous level if it has 
  been modified.

  \param force  if set, this forces an update of the base surface

  \return 1 if the base surface is modified, 0 otherwise.

  \author Philippe Lavoie
  \date 15 April 1998
*/
template <class T, int N>
int HNurbsSurface<T,N>::initBase(int force){
  if(!baseLevel_){
    return 0; // this means that the surface is the base level
  }

  // make sure none of the base level need updating
  if(baseLevel_->initBase())
    force = 1 ;
  

  // Only initialize the base level if it isn't up to date
  if(baseLevel_->modifiedN() == baseUpdateN && !force){
    return 0;
  }

  baseUpdateN = baseLevel_->modifiedN() ;

  baseSurf = *baseLevel_ ;

  if(rU.n()>0)
    baseSurf.refineKnotU(rU) ;
  if(rV.n()>0)
    baseSurf.refineKnotV(rV) ;

  Vector<T> maxU,maxV ;
  int i,j ;
  
  if(baseSurf.degreeU()>3)
    averagingKnots(baseSurf.knotU(),baseSurf.degreeU(),maxU) ;
  else{
    maxU.resize(baseSurf.ctrlPnts().rows()) ;
    for(i=0;i<baseSurf.ctrlPnts().rows();++i)
      if(!maxInfluence(i,baseSurf.knotU(),baseSurf.degreeU(),maxU[i]))
	cerr << "Problem in maxInfluence U!\n" ;
  }

  if(baseSurf.degreeV()>3)
    averagingKnots(baseSurf.knotV(),baseSurf.degreeV(),maxV) ;
  else{
    maxV.resize(baseSurf.ctrlPnts().cols()) ;
    for(i=0;i<baseSurf.ctrlPnts().cols();++i)
      if(!maxInfluence(i,baseSurf.knotV(),baseSurf.degreeV(),maxV[i]))
	cerr << "Problem in maxInfluence V!\n" ;
  }

  if(fixedOffset){
    if(ivec.rows() != 1 || ivec.cols()!=1){
      ivec.resize(1,1);
      jvec.resize(1,1);
      kvec.resize(1,1);
    }
  }
  else{
    ivec.resize(maxU.n(),maxV.n()) ;
    jvec.resize(maxU.n(),maxV.n()) ;
    kvec.resize(maxU.n(),maxV.n()) ;

    Matrix< Point_nD<T,N> > ders ;
    
    for(i=0;i<maxU.n();++i)
      for(j=0;j<maxV.n();++j){
	baseSurf.deriveAt(maxU[i],maxV[j],1,ders) ;
	// It is possible that there are no valid derivative at that point
	// we then move to a nearby point to get a valid value
	Point_nD<T,N> norm = crossProduct(ders(0,1),ders(1,0)) ;
	if(norm.x() == T(0) &&
	   norm.y() == T(0) &&
	   norm.z() == T(0)){
	  const T delta = 0.00001 ;
	  T nt = 1.0 ; 
	  Matrix< Point_nD<T,N> > dersT(ders) ;
	  while(norm.x() == T(0) && 
		norm.y() == T(0) &&
		norm.z() == T(0)){
	    if( nt*delta >(baseSurf.knotU()[baseSurf.knotU().n()-1]-baseSurf.knotU()[0])){
	      Error error("initBase");
	      error << "Can't compute the derivative.\n" ;
	      error.fatal() ;
	    }
	    T nd ;
	    
	    ders.reset(0) ;
	    
	    nd = 0 ;
	    if(i != 0){
	      baseSurf.deriveAt(maxU[i] - nt*delta, maxV[j],1, dersT) ;
	      ders += dersT ;
	      ++nd ;
	    }
	    if(i != maxU.n()-1){
	      baseSurf.deriveAt(maxU[i] + nt*delta, maxV[j],1, dersT) ;
	      ders += dersT ;
	      ++nd ;
	    }
	    if(j != 0){
	      baseSurf.deriveAt(maxU[i], maxV[j] - nt*delta,1, dersT) ;
	      ders += dersT ;
	      ++nd ;
	    }
	    if(j != maxV.n()-1){
	      baseSurf.deriveAt(maxU[i], maxV[j] + nt*delta,1, dersT) ;
	      ders += dersT ;
	      ++nd ;
	    }
	    
	    if(nd==0){
	      Error error("initBase");
	      error << "Can't compute the derivative.\n" ;
	      error.fatal() ;
	    }
	    
	    ders /= nd ;
	    
	    norm = crossProduct(ders(0,1),ders(1,0)) ;
	    nt *= 10.0 ; 
	  }
	}	
	
	ivec(i,j) = ders(0,1).unitLength() ;
	kvec(i,j) = crossProduct(ders(0,1),ders(1,0)).unitLength() ;
	jvec(i,j) = crossProduct(kvec(i,j),ivec(i,j)).unitLength() ;
      }
  }
    
  return 1 ;
}

/*!
  \brief Specifies the level that modifies the point

  Specifies what level modifies the point \a (u,v)

  \param u  the \a u parametric value
  \param v  the \a v parametric value

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
int HNurbsSurface<T,N>::modifies(T u, T v){
  if(nextLevel_){
    int mod = nextLevel_->modifies(u,v) ;
    if(mod>=0)
      return mod ;
  }

  if(u<knotU()[0] || u>knotU()[knotU().n()-1])
    return -1 ;
  if(v<knotV()[0] || v>knotU()[knotV().n()-1])
    return -1 ;

  int su = findSpanU(u) ;
  int sv = findSpanV(v) ;

  for(int i=0;i<=degU;++i)
    for(int j=0;j<=degV;++j){
      if(offset(su-degU+i,sv+degV+j) != HPoint_nD<T,N>(0,0,0,0))
	return level_ ;
    }

  return -1 ;
}

/*!
  \brief finds the homogenous point at (u,v) for a certain level of detail.

  \param u  the \a u parametric value
  \param  v  the \a v parametric value
  \param  lod  the level of detail3
  
  \return the homogenous point at (u,v) and at the level of detail specivied by
               lod.

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
HPoint_nD<T,N> HNurbsSurface<T,N>::hpointAt(T u, T v, int lod) const {
  if(level_==lod)
    return NurbsSurface<T,N>::operator()(u,v) ;

  if(nextLevel_)
    return nextLevel_->hpointAt(u,v,lod) ; 
  
  return NurbsSurface<T,N>::operator()(u,v) ;
}

/*!
  \brief the maximum level of detail

  Finds the maximum level of detail available from this HNURBS
  surface

  \return the maximum level of detail.

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
int HNurbsSurface<T,N>::maxLevel() const{
  if(lastLevel_)
    return lastLevel_->level_ ;
  return level_ ; 
}

/*!
  \brief Finds the derivative of the point \a (u,v)

  Computes the matrix of derivatives at \a (u,v) .
  The value of skl(k,l) represents the 
  derivative of the surface $S(u,v)$ with respect to 
  \a u, \a k times and to \a v, \a l times.

  \param u  the \a u parametric value
  \param v  the \a v parametric value
  \param ders  the Matrix of derivatives.

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
void HNurbsSurface<T,N>::deriveAtH(T u, T v, int d, Matrix< HPoint_nD<T,N> >& ders,int lod) const {
  if(level_==lod){
    NurbsSurface<T,N>::deriveAtH(u,v,d,ders) ;
    return ;
  }

  if(nextLevel_){
    nextLevel_->deriveAtH(u,v,d,ders,lod) ; 
    return ;
  }
  
  NurbsSurface<T,N>::deriveAtH(u,v,d,ders) ;
}

/*!
  \brief Finds the derivative of the point \a (u,v)

  Computes the matrix of derivatives at \a u,v. 
  The value of skl(k,l) represents the 
  derivative of the surface \a S(u,v) with respect to 
  \a u, \a k times and to \a v, \a l times.

  \param u  the \a u parametric value
  \param v  the \a v parametric value
  \param ders  the Matrix of derivatives.

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
void HNurbsSurface<T,N>::deriveAt(T u, T v, int d, Matrix< Point_nD<T,N> >& ders, int lod) const {
  if(level_==lod){
    NurbsSurface<T,N>::deriveAt(u,v,d,ders) ;
    return ;
  }

  if(nextLevel_){
    nextLevel_->deriveAt(u,v,d,ders,lod) ; 
    return ;

  NurbsSurface<T,N>::deriveAt(u,v,d,ders) ;
  }
}

/*!
  \brief Destructor

  Deletes all the levels.

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
HNurbsSurface<T,N>::~HNurbsSurface(){
  if(nextLevel_)
    delete nextLevel_ ;
  lastLevel_ = 0 ;
  if(baseLevel_){
    baseLevel_->nextLevel_ = 0 ;
    baseLevel_->lastLevel_ = baseLevel_ ;
  }
  baseLevel_ = 0 ;
  nextLevel_ = 0 ;
  firstLevel_ = 0 ;
}

/*!
  \brief Update the surface for all the levels

  \param upLevel  updates the levels up to this level of detail

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
void HNurbsSurface<T,N>::updateLevels(int upLevel){
  if(upLevel>=0){
    if(level_ <= upLevel){
      this->updateSurface() ;
    }
  }
  else{
    this->updateSurface() ;
  }

  if(upLevel>level() || upLevel<0){
    if(nextLevel_)
      nextLevel_->updateLevels(upLevel) ;
  }
}

/*!
  \brief Insert n knots betwen each knots

  Insert nu knots betwen each knots in the U vector and
  nv knots between each knots in the V vector. 
  
  This does not perform a split. It just generates
  a suitable rU and rV vector. It is suggested that 
  splitting should be done for the level above, not
  the local level.

  \param nu  the number of new knots between each knots in U.
  \param nv  the number of new knots between each knots in V.
  \param nU  the new refinement knot vector in U
  \param nV  the new refinement knot vector in V

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
void HNurbsSurface<T,N>::splitUV(int nu, int nv, Vector<T> &nU, Vector<T> &nV){

  nU.resize(knotU().n()*nu) ;
  nV.resize(knotV().n()*nv) ;
  
  int i,j,n ;

  n = 0 ; 
  for(i=1;i<knotU().n();++i){
    if(knotU()[i] >knotU()[i-1]){
      T a = knotU()[i-1] ;
      T b = knotU()[i] ;


      for(j=0;j<nu;++j){
	nU[n] = a + (b-a)*T(j+1)/T(nu+1) ;
	++n ;
      }
    }
  }  
  nU.resize(n) ;

  n = 0 ;
  for(i=1;i<knotV().n();++i){
    if(knotV()[i] > knotV()[i-1]){
      T a = knotV()[i-1] ;
      T b = knotV()[i] ;

      for(j=0;j<nv;++j){
	nV[n] = a + (b-a)*T(j+1)/T(nv+1) ;
	++n ;
      }
    }
  }  
  nV.resize(n) ;

}
 
/*!
  \brief Insert n knots betwen each knots

  Insert nu knots betwen each knots in the U vector and
  nv knots between each knots in the V vector. 
  
  This doesn't not perform a split. It just generates
  a suitable rU and rV vector. It is suggested that 
  splitting should be done for the level above, not
  the local level.

  \param nu  the number of new knots between each knots in U.
  \param su  the multiplicity of the each new knot in U
  \param nv  the number of new knots between each knots in V.
  \param  sv  the multiplicity of the each new knot in V
  \param nU  the new refinement knot vector in U
  \param nV  the new refinement knot vector in V

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
void HNurbsSurface<T,N>::splitUV(int nu, int su, int nv, int sv, Vector<T> &nU, Vector<T> &nV){

  int i,j,n ;

  if(su<=0)
    su = degU  ;
  if(sv<=0)
    sv = degV  ;
  if(su>degU+1)
    su = degU+1 ;
  if(sv>degV+1)
    sv = degV+1 ;

  nU.resize(knotU().n()*nu*su) ;
  nV.resize(knotV().n()*nv*sv) ;
  
  n = 0 ; 
  for(i=1;i<knotU().n();++i){
    if(knotU()[i] >knotU()[i-1]){
      T a = knotU()[i-1] ;
      T b = knotU()[i] ;


      for(j=0;j<nu;++j){
	T u = a + (b-a)*T(j+1)/T(nu+1) ;
	for(int l=0;l<su;++l){
	  nU[n] = u ;
	  ++n ;
	}
      }
    }
  }  
  nU.resize(n) ;

  n = 0 ;
  for(i=1;i<knotV().n();++i){
    if(knotV()[i] > knotV()[i-1]){
      T a = knotV()[i-1] ;
      T b = knotV()[i] ;

      for(j=0;j<nv;++j){
	T v = a + (b-a)*T(j+1)/T(nv+1) ;
	for(int l=0;l<sv;++l){
	  nV[n] = v ;
	  ++n ;
	}
      }
    }
  }  
  nV.resize(n) ;

}
 
/*!
  \brief Adds a level to this HNURBS surface

  \param n  the number of new knots between each knots.

  \return a pointer to the new level or 0 if there was an error.

  \warning returns 0, if there is already a nextlevel.

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
HNurbsSurface<T,N>* HNurbsSurface<T,N>::addLevel(int n) {
  HNurbsSurface<T,N> *newLevel ;

  if(nextLevel_)
    return 0 ;

  Vector<T> newU,newV ;
  
  splitUV(n,n,newU,newV) ;

  newLevel = new HNurbsSurface(this,newU,newV) ;

  return newLevel ;
}

/*!
  \brief Adds a level to this HNURBS surface

  \param n  the number of new knots between each knots.

  \return a pointer to the new level or 0 if there was an error.

  \warning returns 0, if there is already a nextlevel.

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
HNurbsSurface<T,N>* HNurbsSurface<T,N>::addLevel() {
  HNurbsSurface<T,N> *newLevel ;

  if(nextLevel_)
    return 0 ;

  newLevel = new HNurbsSurface(this) ;

  return newLevel ;
}

/*!
  \brief generates an iso curve in the U direction

  Generates an iso-parametric curve which goes through the
  parametric value u along the U direction.

  \param u  the U parametric value
  \param c  the iso-parametric curve
  \param lod  the level of detail to draw the curve with

  \return 0 if an error occured, 1 otherwise.

  \warning the parametric value \a u must be in a valid range

  \author Philippe Lavoie
  \date 7 October, 1997
*/
template <class T, int N>
int HNurbsSurface<T,N>::isoCurveU(T u, NurbsCurve<T,N>& c,int lod) const {
  if(lod>=0 && level_>lod)
    return 0;

  if(lod==level_ || lod<0){
    NurbsSurface<T,N>::isoCurveU(u,c) ; 
    return 1 ;
  }
  
  if(nextLevel_){
    return nextLevel_->isoCurveU(u,c,lod) ;
  }

  return 0 ;
}

/*!
  \brief generates an iso curve in the V direction
  
  Generates an iso-parametric curve which goes through the
  parametric value v along the V direction.

  \param v  the V parametric value
  \param c  the iso-parametric curve
  \param lod  the level of detail to draw the curve with 

  \return 0 if an error occured, 1 otherwise
  \warning the parametric value \a v must be in a valid range

  \author Philippe Lavoie
  \date 7 October, 1997
*/
template <class T, int N>
int HNurbsSurface<T,N>::isoCurveV(T v, NurbsCurve<T,N>& c,int lod) const {
  if(lod>=0 && level_>lod)
    return 0;

  if(lod==level_ || lod<0){
    NurbsSurface<T,N>::isoCurveV(v,c) ; 
    return 1 ;
  }
  
  if(nextLevel_){
    return nextLevel_->isoCurveV(v,c,lod) ;
  }

  return 0 ;
}



/*! 
  \brief Read a HNURBS surface from a file stream

  \param fin  the input file stream

  \return 0 if an error occurs, 1 otherwise

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
int HNurbsSurface<T,N>::read(ifstream &fin){
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
  char stc ; 
  if(!r1 || !r2){
    if(!fin.read((char*)&stc,sizeof(char))) { delete []type ; return 0 ;}

    int st = stc - '0' ; 
    if(st != sizeof(T)){ // does not have the same data size
      delete []type ;
      return 0 ; 
    }

    if(!fin.read((char*)&nu,sizeof(int))) { delete []type ; return 0 ;}
    if(!fin.read((char*)&nv,sizeof(int))) { delete []type ; return 0 ;}
    if(!fin.read((char*)&du,sizeof(int))) { delete []type ; return 0 ;}
    if(!fin.read((char*)&dv,sizeof(int))) { delete []type ; return 0 ;}
    
    resize(nu,nv,du,dv) ;
    
    if(!fin.read((char*)U.memory(),sizeof(T)*U.n())) { delete []type ; return 0 ;}
    if(!fin.read((char*)V.memory(),sizeof(T)*V.n())) { delete []type ; return 0 ;}
    
    if(!r1){
      p = new T[3*nu*nv] ;
      if(!fin.read((char*)p,sizeof(T)*3*nu*nv)) { delete []type ; return 0 ;}
      p2 = p ;
      for(int i=0;i<nu;i++)
	for(int j=0;j<nv;j++){
	  P(i,j).x() = *(p++) ;
	  P(i,j).y() = *(p++) ;
	  P(i,j).z() = *(p++) ;
	  P(i,j).w() = 1.0 ;
	}
      delete []p2 ;
    }
    else{
      p = new T[4*nu*nv] ;
      if(!fin.read((char*)p,sizeof(T)*4*nu*nv)) { delete []type ; return 0 ;}
      p2 = p ;
      for(int i=0;i<nu;i++)
	for(int j=0;j<nv;j++){
	  P(i,j).x() = *(p++) ;
	  P(i,j).y() = *(p++) ;
	  P(i,j).z() = *(p++) ;
	  P(i,j).w() = *(p++) ;
	}
      delete []p2 ;
    }
    offset = P ;
    this->updateSurface() ;
  }
  else { // reading the offset information
    int ru,rv ;
    if(!fin.read((char*)&ru,sizeof(int))) { delete []type ; return 0 ;}
    if(!fin.read((char*)&rv,sizeof(int))) { delete []type ; return 0 ;}
    rU.resize(ru) ;
    rV.resize(rv) ;
    if(rU.n()>0)
      if(!fin.read((char*)rU.memory(),sizeof(T)*rU.n())) { delete []type ; return 0 ;}
    if(rV.n()>0)
      if(!fin.read((char*)rV.memory(),sizeof(T)*rV.n())) { delete []type ; return 0 ;}
    
    if(!fin.read((char*)&nu,sizeof(int))) { delete []type ; return 0 ;}
    if(!fin.read((char*)&nv,sizeof(int))) { delete []type ; return 0 ;}

    p = new T[4*nu*nv] ;
    if(!fin.read((char*)p,sizeof(T)*4*nu*nv)) { delete []type ; return 0 ;}
    p2 = p ;
    offset.resize(nu,nv) ;
    for(int i=0;i<nu;i++)
      for(int j=0;j<nv;j++){
	offset(i,j).x() = *(p++) ;
	offset(i,j).y() = *(p++) ;
	offset(i,j).z() = *(p++) ;
	offset(i,j).w() = *(p++) ;
      }
    delete []p2 ;    
    --baseUpdateN ;
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
    HNurbsSurface<T,N> *newPatch = this->addLevel() ;
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

/*! 
  \brief Reads a HNURBS surface from a file

  \param filename  the filename to read from 

  \return 0 if an error occurs, 1 otherwise

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
int HNurbsSurface<T,N>::read(const char* filename){
  ifstream fin(filename) ;
  if(!fin) {
    return 0 ;
  }

  return this->read(fin) ;
}

/*! 
  \brief Write a HNURBS surface to a file stream

  \param fout  the output filestream to write to.

  \return 1 on success, 0 on failure

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
int HNurbsSurface<T,N>::write(ofstream &fout) const {
  if(!fout)
    return 0 ;
  if(!baseLevel_){
    int prows = P.rows();
    int pcols = P.cols();
    char st = '0' + sizeof(T) ; 
    if(!fout.write((char*)&"hns4",sizeof(char)*4)) return 0 ;
    if(!fout.write((char*)&st,sizeof(char))) return 0 ; 
    if(!fout.write((char*)&prows,sizeof(int))) return 0 ;
    if(!fout.write((char*)&pcols,sizeof(int))) return 0 ;
    if(!fout.write((char*)&degU,sizeof(int))) return 0 ;
    if(!fout.write((char*)&degV,sizeof(int))) return 0 ;
    if(!fout.write((char*)U.memory(),sizeof(T)*U.n())) return 0 ;
    if(!fout.write((char*)V.memory(),sizeof(T)*V.n())) return 0 ;
    
    T *p,*p2 ;
    p = new T[P.rows()*P.cols()*4] ;
    p2 = p ;
    for(int i=0;i<P.rows();i++) 
      for(int j=0;j<P.cols();j++){
	*p = offset(i,j).x() ; p++ ;
	*p = offset(i,j).y() ; p++ ;
	*p = offset(i,j).z() ; p++ ;
	*p = offset(i,j).w() ; p++ ;
      }
    if(!fout.write((char*)p2,sizeof(T)*P.rows()*P.cols()*4)) return 0 ;
    delete []p2 ;
  }
  else{
    if(!fout.write((char*)&"hnso",sizeof(char)*4)) return 0 ;
    int run = rU.n();
    int rvn = rV.n();
    if(!fout.write((char*)&run,sizeof(int))) return 0 ;
    if(!fout.write((char*)&rvn,sizeof(int))) return 0 ;
    if(rU.n()>0)
      if(!fout.write((char*)rU.memory(),sizeof(T)*rU.n())) return 0 ;
    if(rV.n()>0)
      if(!fout.write((char*)rV.memory(),sizeof(T)*rV.n())) return 0 ;
    int orows = offset.rows();
    int ocols = offset.cols() ;
    if(!fout.write((char*)&orows,sizeof(int))) return 0 ;
    if(!fout.write((char*)&ocols,sizeof(int))) return 0 ;
    T *p,*p2 ;

    p = new T[offset.rows()*offset.cols()*4] ;
    p2 = p ;
    for(int i=0;i<offset.rows();i++) 
      for(int j=0;j<offset.cols();j++){
	*p = offset(i,j).x() ; p++ ;
	*p = offset(i,j).y() ; p++ ;
	*p = offset(i,j).z() ; p++ ;
	*p = offset(i,j).w() ; p++ ;
      }
    if(!fout.write((char*)p2,sizeof(T)*offset.rows()*offset.cols()*4)) 
      return 0 ;
    delete []p2 ;    
  }
  if(nextLevel_){
    if(!fout.write((char*)&"level",sizeof(char)*5)) return 0 ;
    if(!nextLevel_->write(fout)) return 0 ;
  }
  return 1 ;
}

/*! 
  \brief write a HNURBS surface to a file

  \param filename  the filename to write to

  \return 1 on success, 0 on failure

  \author Philippe Lavoie
  \date 7 October 1997
*/
template <class T, int N>
int HNurbsSurface<T,N>::write(const char* filename) const {
  ofstream fout(filename) ;    
  if(!fout)
    return 0 ;
  return write(fout);
}

/*! 
  Generates a NURBS surface.
  \param surf <-- the NURBS surface corresponding to the HNURBS surface
  \return 
  \warning 

  \author Philippe Lavoie
  \date 7 October 1997
*/
//template <class T, int N>
//void HNurbsSurface<T,N>::toNurbs(NurbsSurface<T,N>& surf) const {
  
//}

template <class T>
inline void mergeInto(const Vector<T> &a, Vector<T> &b) {
  // merge a in b ;
  int n = b.n() ;
  b.resize(b.n()+a.n()) ;  
  for(int i=0;i<a.n();++i)
    b[n+i] = a[i] ;
  b.qSort() ;
}

/*! 
  \brief Refine both knot vectors

  \param nU  the U knot vector to refine from 
  \param nV  the V knot vector to refine from 

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void HNurbsSurface<T,N>::refineKnots(const Vector<T>& nU, const Vector<T>& nV){
  refineKnotU(nU) ;
  mergeInto(nU,rU) ;

  initBase(1) ;

  refineKnotV(nV) ;  
  mergeInto(nV,rV) ;
}

/*! 
  \brief  Refines the U knot vector

  \param X  the knot vector to refine from

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void HNurbsSurface<T,N>::refineKnotU(const Vector<T>& X) {
  updateSurface() ;
  Vector<T> Xu(X) ;
  int i,j ;
  j = 0 ;
  for(i=0;i<X.n();++i){
    if(X[i]>=U[0] && X[i]<=U[U.n()-1]){
      Xu[j] = X[i] ;
      ++j ;
    }
  }
  Xu.resize(j) ;

  if(Xu.n()>0){
    if(nextLevel_){
      nextLevel_->refineKnotU(Xu) ;
    }
    
    NurbsSurface<T,N> osurf(degU,degV,U,V,offset) ;
    
    osurf.refineKnotU(Xu) ;
    
    offset.resize(osurf.ctrlPnts().rows(),osurf.ctrlPnts().cols()) ;
    for(i=0;i<offset.rows();++i)
      for(j=0;j<offset.cols();++j)
	offset(i,j) = osurf.ctrlPnts()(i,j) ;

    if(!baseLevel_){
      NurbsSurface<T,N>::refineKnotU(Xu) ;
    }
  }
}

/*! 
  \brief  Refines the V knot vector

  \param X  the knot vector to refine from

  \author Philippe Lavoie
  \date 24 January, 1997
*/
template <class T, int N>
void HNurbsSurface<T,N>::refineKnotV(const Vector<T>& X) {
  updateSurface() ;
  Vector<T> Xv(X) ;
  int i,j ;
  j = 0 ;
  for(i=0;i<X.n();++i){
    if(X[i]>=V[0] && X[i]<=V[V.n()-1]){
      Xv[j] = X[i] ;
      ++j ;
    }
  }
  Xv.resize(j) ;

  if(Xv.n()>0){
    if(nextLevel_){
      nextLevel_->refineKnotV(Xv) ;
    }
    
    NurbsSurface<T,N> osurf(degU,degV,U,V,offset) ;
    
    osurf.refineKnotV(Xv) ;
    
    offset.resize(osurf.ctrlPnts().rows(),osurf.ctrlPnts().cols()) ;
    for(i=0;i<offset.rows();++i)
      for(j=0;j<offset.cols();++j)
	offset(i,j) = osurf.ctrlPnts()(i,j) ;
    if(!baseLevel_){
      NurbsSurface<T,N>::refineKnotV(Xv) ;
    }
  }
}

/*! 
  \brief  Move a point on the surface

  This moves the point \a s(u,v) by delta. As this is a HNURBS
  surface. It moves the offset surface by delta, it doesn't move 
  the surface point per say.

  \param u  the parameter in the u direction
  \param v  the parameter in the v direction
  \param delta  the displacement of the point \a s(u,v)

  \return 1 if the operation was succesfull, 0 otherwise

  \warning u and v must be in a valid range.

  \author Philippe Lavoie
  \date 3 June 1998
*/
template <class T, int N>
int HNurbsSurface<T,N>::movePointOffset(T u, T v, const Point_nD<T,N>& delta){
  P = offset ; 

  // by definition the offset has w = 0 , but this isn't valid for
  // the control points, increasing the w by 1, will generate a valid surface
  if(baseLevel_)
    for(int i=0;i<P.rows();++i)
      for(int j=0;j<P.cols();++j){
	P(i,j).w() += T(1) ; 
      }

  if(NurbsSurface<T,N>::movePoint(u,v,delta)){
    offset = P ;
    // need to reset the offset weights
    if(baseLevel_)
      for(int i=0;i<P.rows();++i)
	for(int j=0;j<P.cols();++j){
	  P(i,j).w() -= T(1) ; 
      }
    
    P = baseSurf.ctrlPnts() ; 
    updateSurface() ; 
    return 1 ;
  }
  updateSurface() ; 
  return 0 ;
}

/*! 
  \brief  Scales the object

  \param s  the scaling factor

  \author Philippe Lavoie
  \date 11 June 1998
*/
template <class T, int N>
void HNurbsSurface<T,N>::scale(const Point_nD<T,N>& s){
  for(int i=0;i<offset.rows();++i)
    for(int j=0;j<offset.cols();++j){
      offset(i,j).x() *= s.x() ; 
      offset(i,j).y() *= s.y() ; 
      offset(i,j).z() *= s.z() ; 
    }

  if(nextLevel_){
    nextLevel_->scale(s) ; 
  }
}


/*! 
  \brief  The offset vector are fixed

  Fixes the offset vector direction to a unique value. The offset
  vector's direction won't depend on its base layer.
  
  \param I  the i vector
  \param J  the j vector
  \param  K  the k vector

  \author Philippe Lavoie
  \date 11 June 1998 
*/
template <class T, int N>
void HNurbsSurface<T,N>::setFixedOffsetVector(const Point_nD<T,N> &I, const Point_nD<T,N> &J, const Point_nD<T,N>& K){
  fixedOffset = 1;
  initBase();
  ivec(0,0) = I;
  jvec(0,0) = J;
  kvec(0,0) = K;
  updateSurface();
}

/*! 
  \brief  The offset vector are variable

  Fixes the offsset vector direction to a variable value. The value
  depends on its base layer.


  \author Philippe Lavoie
  \date 11 June 1998 
*/
template <class T, int N>
void HNurbsSurface<T,N>::setVariableOffsetVector(){
  fixedOffset = 0 ; 
  initBase();
  updateSurface();
}

} //end namespace
