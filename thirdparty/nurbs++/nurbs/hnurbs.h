/*=============================================================================
        File: hnurbsS.h
     Purpose:       
    Revision: $Id: hnurbs.h,v 1.2 2002/05/13 21:07:46 philosophil Exp $
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
#ifndef _nurbs_hnurbs_h_
#define _nurbs_hnurbs_h_

#include "nurbs.h"

class PlHNurbsCurveNode ;

class PlHNurbsCurve : public PlParaCurve {
public:
  PlHNurbsCurve() ;
  virtual ~PlHNurbsCurve() { reset() ; }
  
  void add(const PlNurbsCurve& curve, T uS, T uE) ;
  void remove(void) ;
  void reset(void) ;

  T minKnot() const { return 0.0 ; }
  T maxKnot() const { return 1.0 ; }

  HPoint_nD<T,N> operator()(T u) const;
  void deriveAt(T u, int, PlVector< HPoint_nD<T,N> >&) const;
  void deriveAt(T u, int, PlVector< Point_nD<T,N> >&) const;

  void interpolate(const PlVector< Point_nD<T,N> > &Pts, int deg, T acceptError=0.5, int nSample=100,  int maxTries=100, int nInitPoints=-1, int nPoints=-1);

  void draw(Image_Color& img, const Color& col) const ;
  void draw(Image_UBYTE& img, unsigned char col) const ;
private:
  PlHNurbsCurveNode *first, *last ;
};

class PlHNurbsCurveNode: public PlParaCurve{
public:
  T &u0, &u1 ;


  PlHNurbsCurveNode *prev,*next ;
  PlNurbsCurve *curve ;

  PlHNurbsCurveNode() ;
  PlHNurbsCurveNode(const PlNurbsCurve& c, T uS, T uE) ;


  T minKnot() const { return u0_ ; }
  T maxKnot() const { return u1_ ; }

  HPoint_nD<T,N> operator()(T u) const;
  void deriveAt(T u, int, PlVector< HPoint_nD<T,N> >&) const;
  void deriveAt(T u, int, PlVector< Point_nD<T,N> >&) const;

protected:
  T u0_,u1_ ;
  T uD ; 
};


#endif // _nurbs_hnurbs_h_
