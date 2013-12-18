/*=============================================================================
        File: statistics.cpp
     Purpose:
    Revision: $Id: statistic_.cpp,v 1.2 2002/05/13 21:07:45 philosophil Exp $
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

#include "statistic.cpp"

namespace PLib{

  int MaximumIterations ;
  
#ifdef NO_IMPLICIT_TEMPLATES

  template  float lnOfGamma(float xx) ;
  template  float factorial<float>(int n) ;
  template  float lnOfFactorial<float>(int n) ;
  template  float binomialCoefficient<float>(int n, int k) ;
  template  float beta(float z, float w) ;
  template  float gammaP(float a, float x);
  template  float gammaQ(float a, float x);
  template  float gammaSerie(float a, float x, float& gln) ;
  template  float gammaSerieCF(float a, float x, float& gln) ;
  template  float errorFcn(float x);
  template  float errorFcnC(float x);
  template  float errorFcnChebyshevC(float x);
  template  void kendallTau(const BasicArray<float>& data1, const BasicArray<float>& data2, float &tau, float &z, float& prob);

  template  double lnOfGamma(double xx) ;
  template  double factorial<double>(int n) ;
  template  double lnOfFactorial<double>(int n) ;
  template  double binomialCoefficient<double>(int n, int k) ;
  template  double beta(double z, double w) ;
  template  double gammaP(double a, double x);
  template  double gammaQ(double a, double x);
  template  double gammaSerie(double a, double x, double& gln) ;
  template  double gammaSerieCF(double a, double x, double& gln) ;
  template  double errorFcn(double x);
  template  double errorFcnC(double x);
  template  double errorFcnChebyshevC(double x);
  template  void kendallTau(const BasicArray<double>& data1, const BasicArray<double>& data2, double &tau, double &z, double& prob);

#endif

}
