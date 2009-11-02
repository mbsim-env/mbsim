/* Copyright (C) 2004-2008  Robert Huber, Thorsten Schindler
 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 *  
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * Lesser General Public License for more details. 
 *  
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA

 *
 * Contact:
 *   rzander@users.berlios.de
 *
 */
#ifndef PPOLYNOM
#define PPOLYNOM

#include<iostream>
#include "fmatvec.h"
#include "userfunction.h"

#ifndef NO_ISO_14882
using namespace std;
#endif

using namespace fmatvec;

namespace MBSim {
  /*! Class for piecewise-polynomials and cubic spline interpolation
   * 
   * Robert Huber, Thorsten Schindler
   * new: 31.08.2006
   * last update: 21.05.2008 (C++ beauty treatment, PLinear)
   * verified with matlab /octave: 05.09.2006
   * 
   * Spline / PP-Form info
   * Piecewise polynomial: c0* xloc^n + c1* xloc^(n-1) + c2* xloc^(n-2) + ... + cn
   * with [c0 c1 c2 ... cn] ith row vector of coefs-matrix 
   * breaks(i) << x << breaks(i+1) defines i and xloc = x-breaks(i)
  
   * Ex. cubic spline with
   * breaks= [0; 0.3; 0.5]
   * coefs= [d1 c1 b1 a1;  
   *       d2 c2 b2 a2]
   * -> S(x) = a1 + b1*xloc + c1*xloc^2 + d1*xloc^3 for x\in[0;0.3] and x_loc = x
   * -> S(x) = a2 + b2*xloc + c2*xloc^2 + d2*xloc^3 for x\in[0.3;0.5] and xloc = x - 0.3;
 
   * Cubic spline
   * (xi,fi) i=1..N is being interpolated by N-1 piecewise polynomials Si of degree 3 yielding a global C^2 curve
   * for uniqueness TWO additional boundary conditions are necessary (periodic / natural)
   * 
   * Piecewise linear polynomial
   * (xi,fi) i=1..N is being interpolated by N-1 piecewise polynomials Si of degree 1 yielding a globally weak differentiable curve
   * in the context of this class the second derivative is defined to be zero everywhere (which is mathematically wrong)
   */
  class PPolynom : public UserFunction {
    protected:
      /** matrix of polynomial Coefficents */
      Mat coefs;
      /** vector of breaks (interval boundaries) */
      Vec breaks;
      /** number of defined piecewise polynomials */
      int nPoly;
      /** order of polynomial (3 for cubic polynomials) */
      int order;
      /** for internal use in ppeval functions */
      int index;
      
      /*! Calculation of periodic spline by interpolation */  
      void calculateSplinePeriodic(const Vec &x, const Vec &f);
      /*! Calculation of natural spline by interpolation */  
      void calculateSplineNatural(const Vec &x, const Vec &f);
      /* Calculation of piecewise linear interpolation */
      void calculatePLinear(const Vec &x, const Vec &f);
      
    public:
      /*! Constructor */
      PPolynom() {}
      /*! Destructor */
      virtual ~PPolynom() {}
      
      /*! setXF: Interpolation
       * @param x vector of ordered x values
       * @param f corresponding f(x) values
       * @param InterpolationMethod	'csplinePer' -> cubic Spline with periodic end conditions
       * 											S(x1) = S(xN) -> f(0)=f(end)
       *											S'(x1) = S'(xN)
       *											S''(x1) = S''(xN)
       * 							'csplineNat' -> cubic Spline with natural end conditions
       * 											S''(x1) = S''(xN) = 0
       * 							'plinear'	-> piecewise linear function
       */
      void setXF(const Vec &x, const Vec &f, std::string InterpolationMethod); 
      /*! Function evaluation */
      Vec operator()(double x);
      /*! First derivative */
      Vec diff1(double x);
      /*! Second derivative */
      Vec diff2(double x);	
      
      /*! Get polynomial coefficients */
      Mat getCoefs();
      /*! Get interval boundaries */
      Vec getBreaks();
      /*! Set piecewise polynomial */
      void setPP(const Mat &coefs_u, const Vec &breaks_u);
  };
  	
  inline Mat PPolynom::getCoefs() {return coefs;}
  inline Vec PPolynom::getBreaks() {return breaks;}
  inline void PPolynom::setPP(const Mat &coefs_u, const Vec &breaks_u) {
	coefs = coefs_u; 
	breaks = breaks_u;
	index = 0;
	nPoly = coefs.rows();
	order = coefs.cols()-1;
  }
  
  /*! Class for multidimensional piecewise polynomials and cubic spline interpolation
   * 
   * (c) 2008 Thorsten Schindler
   * Contact: schindler@amm.mw.tum.de
   * 
   * VERSION 08.08.08
   */
  class MDPPolynom : public UserFunction {
    protected:
      /** component functions */
      vector<PPolynom*> components;
      /** number of component functions */
      int M;
      
    public:
      /*! Constructor */
      MDPPolynom() {}
      /*! Destructor */
      virtual ~MDPPolynom() {for(int i=0;i<M;i++) delete components[i];}
      
      /*! setXF: Interpolation
       * @param xf matrix of rowwise ordered x and corresponding f values arranged in a vector
       * @param InterpolationMethod	(see PPolynom)
       */
      void setXF(const vector<Mat> &xf, std::string InterpolationMethod); 
      /*! Function evaluation */
      Vec operator()(double x);
      /*! First derivative */
      Vec diff1(double x);
      /*! Second derivative */
      Vec diff2(double x);	
  };
}

#endif /* _PPOLYNOM_H_ */
