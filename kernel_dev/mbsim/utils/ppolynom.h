/* Copyright (C) 2004-2006  Robert Huber
 
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

#include <fmatvec.h>
#include <iostream>
#include <mbsim/userfunction.h>

namespace MBSim {

  //////////////////////////////////////////////////
  //   ROBERT HUBER
  //   new: 31.08.2006
  //   last update: 26.09.2006
  //   Verified with matlab /octave: 05.09.2006
  /////////////////////////////////////////////////


  /** class for piecewise-polynominal (PPolynom) forms and cubic Spline Interpolation*/
  class PPolynom : public UserFunction {
    protected:   // Attribute
      fmatvec::Mat coefs; 		/** Matrix of Polynom Coefficents*/
      fmatvec::Vec breaks;		/** Vector of breaks (intervall boundarys)*/
      int nPoly;		/** Number of defined piecewise Polynoms*/
      int order;		/** Order of Poly. (3 for cubic Poly.)*/
      int index;          /** for internal use in ppeval functions*/
      // Methoden zum Berechnen der pp-Form (werden von Konstruktor aufgerufen)  
      void calculateSplinePeriodic(const fmatvec::Vec &x, const fmatvec::Vec &f); 
      void calculateSplineNatural(const fmatvec::Vec &x, const fmatvec::Vec &f); 
    public:
      /**@name Konstrukoren */
      //@{
      /** Default Konstruktor  */
      PPolynom() {}
      /** setXF: Interpolation 
	@param x Vector of x Values
	@param f corresponding f(x) values
	@param InterpolationMethod  'csplinePer' -> Cubic Spline with periodic end conditions
	'csplineNat' -> Cubic Spline with natural end conditions (d2fdx2=0 at the boundaries)
Bemerkung: erster und letzter Wert von f muessen ueberein- 
stimmen, damit sich glatte Kontur ergibt (z.B. Nockenkontur) */
      void setXF(const fmatvec::Vec &x, const fmatvec::Vec &f, std::string InterpolationMethod); 
      //@}
      /** @name Evaluate piecewise polynomial*/
      //@{ /** function evaluation */
      fmatvec::Vec operator()(double x);
      /** first derivativ    d/ dx */
      fmatvec::Vec diff1(double x);
      /** second derivative  d^2 / dx^2*/
      fmatvec::Vec diff2(double x);	
      //@}
      /** @name Standardmethoden zur Manipulation der Attribute*/
      //@{
      fmatvec::Mat getCoefs() {return coefs;}
      fmatvec::Vec getBreaks(){return breaks;}
      void setPP(const fmatvec::Mat &coefs_u, const fmatvec::Vec &breaks_u) {
	coefs = coefs_u; 
	breaks = breaks_u;
	index=0;
	nPoly  = coefs.rows();
	order  = coefs.cols()-1;
      }
      //@}
  };	

}

#endif
