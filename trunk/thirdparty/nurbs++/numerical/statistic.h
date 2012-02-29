/*=============================================================================
        File: statistic.h
     Purpose:       
    Revision: $Id: statistic.h,v 1.2 2002/05/13 21:07:45 philosophil Exp $
  Created by: Philippe Lavoie          (22 Oct, 1997)
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
#ifndef _matrix_statistic_hh
#define _matrix_statistic_hh

#include "barray.h"

/*!
 */
namespace PLib {
  
  void initStatistic();

  extern int MaximumIterations ;

  template <class T>
    inline T Precision(){
    return 0 ;
  }
  
  template <> inline double Precision<double>() { return 3e-7 ; }
  template <> inline float Precision<float>() { return 3e-7 ; }

  template <class T>
    inline T MinimumFloat(){
    return 0 ; 
  }

  template <> inline double MinimumFloat<double>() { return 1e-30 ; }
  template <> inline float MinimumFloat<float>() { return 1e-30 ; }

  template <class T> T lnOfGamma(T xx) ;
  template <class T> T factorial(int n) ;
  template <class T> T lnOfFactorial(int n) ;
  template <class T> T binomialCoefficient(int n, int k) ;
  template <class T> T beta(T z, T w) ;
  template <class T> T gammaP(T a, T x);
  template <class T> T gammaQ(T a, T x);
  template <class T> T gammaSerie(T a, T x, T& gln) ;
  template <class T> T gammaSerieCF(T a, T x, T& gln) ;
  template <class T> T errorFcn(T x);
  template <class T> T errorFcnC(T x);
  template <class T> T errorFcnChebyshevC(T x);
  template <class T> void kendallTau(const BasicArray<T>& data1, const BasicArray<T>& data2, T &tau, T &z, T& prob);
  
  void kendallTau(const BasicArray<int>& data1, const BasicArray<int>& data2, float &tau, float &z, float& prob);
  void kendallTau(const BasicArray<int>& data1, const BasicArray<int>& data2, double &tau, double &z, double& prob);

}

#endif
