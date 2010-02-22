/* Copyright (C) 2004-2009 MBSim Development Team
 *
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
 * Contact: mfoerg@users.berlios.de
 */

#ifndef PPOLYNOM
#define PPOLYNOM

#include "fmatvec.h"
#include "mbsim/utils/function.h"

namespace MBSim {

  /*! 
   * \brief class for piecewise-polynomials and cubic spline interpolation
   * \author Robert Huber
   * \date 2006-08-31 initial commit
   * \date 2006-09-05 verified with matlab /octave (Robert Huber)
   * \date 2008-05-21 C++ beauty treatment, PLinear (Thorsten Schindler)
   * \date 2009-08-11 kernel_dev (Thorsten Schindler)
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
  class PPolynom : public DifferentiableFunction1<fmatvec::Vec> {
    public:
      /*! 
       * \brief constructor
       */
      PPolynom() : DifferentiableFunction1<fmatvec::Vec>() { addDerivative(new PPolynom::ZerothDerivative(this)); addDerivative(new PPolynom::FirstDerivative(this)); addDerivative(new PPolynom::SecondDerivative(this)); }

      /*! 
       * \brief destructor
       */
      virtual ~PPolynom() {}

      /*! 
       * \brief set interpolation
       * @param x vector of ordered x values
       * @param f corresponding f(x) values (rowwise)
       * @param InterpolationMethod     'csplinePer' -> cubic Spline with periodic end conditions (two-times continuously differentiable)
       *                                                                                        S(x1) = S(xN) -> f(0)=f(end)
       *                                                                                        S'(x1) = S'(xN)
       *                                                                                        S''(x1) = S''(xN)
       *                                'csplineNat' -> cubic Spline with natural end conditions (two-times continuously differentiable)
       *                                                                                        S''(x1) = S''(xN) = 0
       *                                'plinear'    -> piecewise linear function (weak differentiable)
       */
      void setXF(const fmatvec::Vec &x, const fmatvec::Mat &f, std::string InterpolationMethod);

      /*! 
       * \return polynomial coefficients
       */
      std::vector<fmatvec::Mat> getCoefs();

      /*! 
       * \return interval boundaries
       */
      fmatvec::Vec getBreaks();

      /*!
       * \brief set piecewise polynomial
       * \param polynomial coefficients
       * \param interval boundaries
       */
      void setPP(const std::vector<fmatvec::Mat> &coefs_u, const fmatvec::Vec &breaks_u);
        
      /**
       * \brief initialize function with XML code
       * \param XML element
       */
      virtual void initializeUsingXML(TiXmlElement *element);

    protected:
      /** 
       * \brief vector of polynomial coefficents
       */
      std::vector<fmatvec::Mat> coefs;

      /**
       * \brief vector of breaks (interval boundaries)
       */
      fmatvec::Vec breaks;

      /**
       * \brief number of defined piecewise polynomials 
       */
      int nPoly;

      /**
       * \brief order of polynomial (3 for cubic polynomials) 
       */
      int order;

      /** 
       * \brief for internal use in ppeval functions 
       */
      int index;

      /*! 
       * \brief calculation of periodic spline by interpolation
       * \param interpolated arguments
       * \param interpolated function values
       */  
      void calculateSplinePeriodic(const fmatvec::Vec &x, const fmatvec::Mat &f);

      /*! 
       * \brief calculation of natural spline by interpolation
       * \param interpolated arguments
       * \param interpolated function values
       */  
      void calculateSplineNatural(const fmatvec::Vec &x, const fmatvec::Mat &f);

      /* 
       * \brief calculation of piecewise linear interpolation
       * \param interpolated arguments
       * \param interpolated function values
       *
       * the first derivative is weak and the second derivative is zero elsewhere although it should be distributionally at the corners
       */
      void calculatePLinear(const fmatvec::Vec &x, const fmatvec::Mat &f);

      /**
       * piecewise polynomial interpolation - zeroth derivative
       */
      class ZerothDerivative : public Function1<fmatvec::Vec,double> {
        public:
          ZerothDerivative(PPolynom *polynom) : Function1<fmatvec::Vec,double>(), parent(polynom), xSave(0), ySave(fmatvec::Vec(0)) {}
          virtual ~ZerothDerivative() {}

          /* INHERITED INTERFACE OF FUNCTION */
          fmatvec::Vec operator()(const double& x, const void * =NULL);
          /***************************************************/

        private:
          PPolynom *parent;
          double xSave;
          fmatvec::Vec ySave;
      };

      /**
       * piecewise polynomial interpolation - first derivative
       */
      class FirstDerivative : public Function1<fmatvec::Vec,double> {
        public:
          FirstDerivative(PPolynom *polynom) : Function1<fmatvec::Vec,double>(), parent(polynom), xSave(0), ySave(fmatvec::Vec(0)) {}
          virtual ~FirstDerivative() {}

          /* INHERITED INTERFACE OF FUNCTION */
          fmatvec::Vec operator()(const double& x, const void * =NULL);
          /***************************************************/

        private:
          PPolynom *parent;
          double xSave;
          fmatvec::Vec ySave;
      };

      /**
       * piecewise polynomial interpolation - second derivative
       */
      class SecondDerivative : public Function1<fmatvec::Vec,double> {
        public:
          SecondDerivative(PPolynom *polynom) : Function1<fmatvec::Vec,double>(), parent(polynom), xSave(0), ySave(fmatvec::Vec(0)) {}
          virtual ~SecondDerivative() {}

          /* INHERITED INTERFACE OF FUNCTION */
          fmatvec::Vec operator()(const double& x, const void * =NULL);
          /***************************************************/

        private:
          PPolynom *parent;
          double xSave;
          fmatvec::Vec ySave;
      };
  };

  inline std::vector<fmatvec::Mat> PPolynom::getCoefs() { return coefs; }
  inline fmatvec::Vec PPolynom::getBreaks() { return breaks; }
  inline void PPolynom::setPP(const std::vector<fmatvec::Mat> &coefs_u, const fmatvec::Vec &breaks_u) {
    coefs = coefs_u; 
    breaks = breaks_u;
    index = 0;
    nPoly = (coefs[0]).rows();
    order = coefs.size()-1;
  }

}

#endif /* PPOLYNOM */

