/*=============================================================================
        File: hnurbs.cpp
     Purpose:       
    Revision: $Id: hnurbs.cpp,v 1.2 2002/05/13 21:07:46 philosophil Exp $
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
#include <hnurbs.h>

HNurbsCurveNode::HNurbsCurveNode():u0(u0_),u1(u1_){
  prev = 0 ;
  next = 0 ;
  curve = new NurbsCurve ;
  u0 = 0 ;
  u1 = 1 ;
  uD = 1 ;
}

HNurbsCurveNode::HNurbsCurveNode(const NurbsCurve &c, T uS, T uE):u0(u0_),u1(u1_){
  prev = 0 ;
  next = 0 ;
  curve = new NurbsCurve(c) ;
  u0 = uS ;
  u1 = uE ;
  uD = uE-uS ;
}

HPoint_nD<T,N> HNurbsCurveNode::operator()(T u) const {
  if(u<u0 || u>u1)
    return HPoint_nD<T,N>(0,0,0,0) ;
  
  u -= u0_ ;
  u /= uD ;
  return (*curve)(u) ;
}

void HNurbsCurveNode::deriveAt(T u, int d, Vector< Point_nD<T,N> >& ders) const {
  ders.resize(d+1) ;
  ders.reset(0) ;
  if(u<u0 || u>u1)
    return ;
  
  u -= u0_ ;
  u /= uD ;
  curve->deriveAt(u,d,ders) ;
}

void HNurbsCurveNode::deriveAt(T u, int d, Vector< HPoint_nD<T,N> >& ders) const {
  ders.resize(d+1) ;
  ders.reset(0) ;
  if(u<u0 || u>u1)
    return ;
  
  u -= u0_ ;
  u /= uD ;
  curve->deriveAt(u,d,ders) ;
}


HNurbsCurve::HNurbsCurve(){
  first = 0 ;
  last = 0 ;
}

void HNurbsCurve::add(const NurbsCurve& curve, T uS, T uE) {
  HNurbsCurveNode *nC ;
  nC = new HNurbsCurveNode(curve,uS,uE) ;
  nC->prev = last ;
  if(last)
    last->next = nC ;
  if(!first)
    first = nC ;
  last = nC ;
}

void HNurbsCurve::remove(void){
  HNurbsCurveNode *tC ;
  if(last){
    tC = last ;
    last = last->prev;
    last->next = 0 ;
    delete tC ;
  }
}

HPoint_nD<T,N> HNurbsCurve::operator()(T u) const{
  HNurbsCurveNode *c ;
  HPoint_nD<T,N> result(0,0,0,0) ;
  HPoint_nD<T,N> temp ;
  Point_nD<T,N> temp2 ;
  c = first ;
  while(c){
    temp = (*c)(u) ;
    if(temp.w()!= 0){
      temp2 = project(temp) ;
      result.x() += temp2.x() ; 
      result.y() += temp2.y() ; 
      result.z() += temp2.z() ; 
    }
    c = c->next ;
  }
  result.w() = 1.0 ;
  return result ;
}

void HNurbsCurve::deriveAt(T u,int d, Vector< HPoint_nD<T,N> >& ders) const{
  HNurbsCurveNode *c ;
  Vector< HPoint_nD<T,N> > dTemp(d+1) ;

  ders.resize(d+1) ;
  ders.reset(0) ;
  c = first ;
  while(c){
    c->deriveAt(u,d,dTemp) ;
    ders += dTemp ;
    c = c->next ;
  }
}
void HNurbsCurve::deriveAt(T u,int d, Vector< Point_nD<T,N> >& ders) const{
  HNurbsCurveNode *c ;
  Vector< Point_nD<T,N> > dTemp(d+1) ;

  ders.resize(d+1) ;
  ders.reset(0) ;
  c = first ;
  while(c){
    c->deriveAt(u,d,dTemp) ;
    ders += dTemp ;
    c = c->next ;
  }
}



void HNurbsCurve::draw(Image_Color& img, const Color& col) const {
  const int n=100 ;
  T dU = 1.0/T(n) ;
  T uP = 0 ;
  HPoint_nD<T,N> p,c ;
  Point_nD<T,N> p3,c3 ;
  
  p = point4D(uP) ;
  for(T u=dU; u<1.0; u += dU){
    c = point4D(u) ;
    p3 = project(p) ;
    c3 = project(c) ;
    img.drawLine((int)p3.y(),(int)p3.x(),(int)c3.y(),(int)c3.x(),col) ;
    p = c ;
  }
  c = point4D(1.0) ;
  p3 = project(p) ;
  c3 = project(c) ;
  img.drawLine((int)p3.y(),(int)p3.x(),(int)c3.y(),(int)c3.x(),col) ;
}

void HNurbsCurve::draw(Image_UBYTE &img, unsigned char col) const {
  const int n=100 ;
  T dU = 1.0/T(n) ;
  HPoint_nD<T,N> p,c ;
  Point_nD<T,N> p3,c3 ;

  p3 = point3D(0.0) ;
  for(T u=dU;u<1.0;u+=dU){
    c3 = point3D(u) ;
    img.drawLine((int)p3.y(),(int)p3.x(),(int)c3.y(),(int)c3.x(),col) ;
    p3 = c3 ;
  }
  c3 = point3D(1.0);
  img.drawLine((int)p3.y(),(int)p3.x(),(int)c3.y(),(int)c3.x(),col) ;
}

const int MaxRandom = 32768 ; // 2^15


void HNurbsCurve::reset(){
  HNurbsCurveNode* t ;
  while(last){
    t = last->prev ;
    if(last->curve)
      delete last->curve ;
    delete last ;
    last = t ;
  } 
}


// needs to be tested, and checked if this actually optimizes the method used in interpolate
void setRandomVector(Vector_INT& rV, int rangeSize, int rangeOffset, int startAt=0, int  finishAt=-1){
  if(finishAt<0)
    finishAt= rV.n-1 ;
  if(rV.n<=finishAt){
    cerr << "You need to set the random vector's size to the proper value PRIOR to calling this function!\n" ;
    while(1){;}
  }
  
  // write from 0 and offset it afterwards...
  int i ;
  for(i=startAt;i<=finishAt;++i){
    double r = double(rand()) ;
    int l ;
    r /= double(MaxRandom) ; // r = [0,1] 
    r *= double(rangeSize) ; // r=[0,rangeSize] ;
    --rangeSize ;
    l = int(r) ;
    for(int j=startAt;j<i;++j){
      if(rV[j]<=l){
	++l ;
      }
      else{
	int k=j ;
	for(j=i;j>k;--j){
	  rV[j] = rV[j-1] ;
	}
	rV[k] = l ;
	break ;
      }
    }
  }

  for(i=0;i<=finishAt-startAt;i++)
    rV[i] += rangeOffset ;
}

void HNurbsCurve::interpolate(const Vector< Point_nD<T,N> > &Pts, int deg, T acceptError, int nSample, int maxTries, int nInitPoints, int nPoints){
  // Get a first interpolation of the data points
  if(nInitPoints<0) nInitPoints = deg*2 ;
  if(nPoints<0) nPoints = deg*2 ;

  if(nPoints<deg+1 || nInitPoints < deg+1){
    cerr << "Using a number of points smaller than deg+1!\n" ;
    while(1){;}
  }
  if(Pts.n<deg+1){
    cerr << "Not enough points to interpolate !\n" ;
    while(1){;} 
  }

  int i,j,k ;
  
  NurbsCurve sampleC,minC ;
  Vector< Point_nD<T,N> > sampleP ;
  Vector_INT sampleI ;
  Vector<T> error ;
  T minError = -1.0 ;

  sampleP.resize(nInitPoints) ;
  sampleI.resize(nInitPoints) ;
  error.resize(Pts.n) ;
  srand(123) ;

  for(k=0;k<nSample;++k){
    sampleI[0] = 0 ;
    sampleI[sampleP.n-1] = Pts.n-1 ;
    for(i=1;i<sampleP.n-1;++i){
      int ok = 0 ;
      while(!ok){
	ok = 1 ;
	double r = double(rand()) ;
	r /= double(MaxRandom) ; // r = [0,1] 
	r *= Pts.n-2 ; // r=[0,Pts.n-2] ;
	r += 1 ; // r=[1,Pts.n-1]
	sampleI[i] = int(r) ;
	for(j=0;j<i;++j){
	  if(sampleI[j] == sampleI[i]){
	    ok = 0 ;
	    break ;
	  }
	}
      }
    }
    sampleI.qSort() ;
    for(i=0;i<sampleP.n;++i)
      sampleP[i] = Pts[sampleI[i]] ;
    sampleC.globalInterp(sampleP,deg) ;
    T u = 0 ;
    for(i=0;i<Pts.n;++i){
      T e = sampleC.minDist2(Pts[i],u) ;
      error[i] = e ;
    }
    error.qSort() ;
    if(minError>error[error.n/2] || minError<0){
      minError = error[error.n/2] ;
      minC = sampleC ;
    }
  }
  reset() ;
  add(minC,0,1) ;
  
  //  cerr << "Done first interpolation = " << minError << endl ;
  
  // Finding the maximal error region and modifying the HNURBS until there is no more errors...
  T maxError ; 
  Vector_INT regionI(Pts.n) ;
  Vector<T> regionU(Pts.n) ;
  sampleP.resize(nPoints) ;
  sampleI.resize(nPoints) ;
  Vector<T> errorM,errorT ;
  int tryN = 0; 
  while(tryN < maxTries){
    // Find max error between regions with no errors
    T minU,maxU,tempError,u ; 
    int maxI = 0 ;

    int index = 0 ;
    regionI[0] = 0 ;
    regionU[0] = 0.0 ;
    maxError = -1.0 ;
    u = 0 ;
    
    for(i=1;i<Pts.n-1;++i){ // the end points have always an error of 0
      tempError = minDist2(Pts[i],u) ;
      if(tempError<acceptError){
	++index ;
	regionI[index] = i ;
	regionU[index] = u ;
	continue ;
      }
      if(tempError>maxError){
	maxError = tempError ;
	maxI = index ;  // The error is located between region maxI and maxI+1
      }
    }
    ++index ;
    regionI[index] = Pts.n-1 ;
    regionU[index] = 1.0 ;
    ++index ;
    if(maxError<acceptError){
      tryN = maxTries ;
      break ;
    }
    
    // for the max region we add a curve fit
    // 1st) find a region of sufficient size
    int regionS = maxI ;
    int regionE = maxI+1 ;
    
    while((regionI[regionE]-regionI[regionS]) < 2*nPoints){ // must use a bigger region
      if(regionS>0) --regionS ;
      if(regionE<index-1) ++regionE ;      
    }
    
    int regionSize = regionI[regionE]-regionI[regionS] ;
    int regionStart = regionI[regionS] ;
    
    error.resize(regionSize) ;
    errorM.resize(regionSize) ;

    minError = -1.0 ;
    for(k=0;k<nSample;++k){
      sampleI[0] = regionStart ;
      sampleI[sampleP.n-1] = regionI[regionE] ;
      if(regionSize==nPoints){
	k=nSample ;
	for(i=1;i<sampleP.n-1;++i)
	  sampleI[i] = regionStart+i ;
      }
      else{
	for(i=1;i<sampleP.n-1;++i){
	  int ok = 0 ;
	  while(!ok){
	    ok = 1 ;
	    double r = double(rand()) ;
	    r /= double(MaxRandom) ; // r = [0,1] 
	    r *= regionSize-2 ; // r=[0,regionSize-2] ;
	    r += 1 ; // r=[1,regionSize-1]
	    sampleI[i] = int(r)+regionStart ;
	    for(j=0;j<i;++j){
	      if(sampleI[j] == sampleI[i]){
		ok = 0 ;
		break ;
	      }
	    }
	  }
	}
      }
      sampleI.qSort() ;
      u = regionU[regionS] ;
      for(i=1;i<sampleP.n-1;++i){
	//T closest = minDist2(Pts[sampleI[i]],u) ;
	sampleP[i] = Pts[sampleI[i]] - point3D(u) ;
      }
      sampleP[0] = Point_nD<T,N>(0,0,0) ;
      sampleP[sampleP.n-1] = Point_nD<T,N>(0,0,0) ;
      sampleC.globalInterp(sampleP,deg) ;
      u = regionU[regionS] ;
      for(i=0;i<error.n;++i){
	//T closestE = minDist2(Pts[regionStart+i],u) ;
	Point_nD<T,N> closest = point3D(u) ;
	T u2 = u ;
	u2 -= regionU[regionS] ;
	u2 /= regionU[regionE]-regionU[regionS] ;
	if(u2<0 || u2> 1.0) {
	  cerr << "Error in setting up u2!\n" ;
	  while(1) {; }
	}
	error[i] = abs2(closest+project(sampleC(u2))-Pts[regionStart+i]) ;
      }
      error.qSort() ;
      if(minError>error[error.n/2] || minError<0){
	minError = error[error.n/2] ;
	minC = sampleC ;
	minU = regionU[regionS] ;
	maxU = regionU[regionE] ;
	errorM = error ;
      }
    }
    add(minC,regionU[regionS],regionU[regionE]) ;
    //cerr << "Testing Error analysis\n" ;
    errorT.resize(errorM.n) ;
    u = regionU[regionS] ;
    for(i=0;i<errorT.n;++i) {
      T closestE = minDist2(Pts[i+regionStart],u) ;
      errorT[i] = closestE ;
      //cerr << "error at " << i << " = " << errorM[i] << " or " << errorT[i] << "for point " << Pts[i+regionStart] << " and " << point3D(u) << endl ;
    }
    errorT.qSort() ;
    

    //    cerr << "At try  " << tryN << " maxError = " << maxError << " and now it is " << errorT[errorT.n-1] << " for region " << maxI << "[ " << regionStart << ", " << regionI[regionE] << "]" <<  index << endl ;
    ++tryN ;
  }

}
