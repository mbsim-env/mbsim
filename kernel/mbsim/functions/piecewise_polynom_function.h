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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef PIECEWISE_POLYNOM
#define PIECEWISE_POLYNOM

#include "fmatvec/fmatvec.h"
#include "mbsim/functions/function.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/eps.h"
#include "mbsim/mbsim_event.h"

namespace MBSim {

  template<typename Sig> class PiecewisePolynomFunction; 

  /*! 
   * \brief class for piecewise-polynomials and cubic spline interpolation
   * \author Robert Huber
   * \date 2006-08-31 initial commit
   * \date 2006-09-05 verified with matlab /octave (Robert Huber)
   * \date 2008-05-21 C++ beauty treatment, PLinear (Thorsten Schindler)
   * \date 2009-08-11 kernel_dev (Thorsten Schindler)
   * \todo add deletes TODO
   * 
   * Spline / PP-Form info
   * Piecewise polynomial: \f$ c0 xloc^n + c1 xloc^{n-1} + c2 xloc^{n-2} + \dots + cn \f$
   * with [c0 c1 c2 ... cn] ith row vector of coefs-matrix 
   * breaks(i) << x << breaks(i+1) defines i and xloc = x-breaks(i)

   * Ex. cubic spline with
   * breaks= [0; 0.3; 0.5]
   * coefs= [d1 c1 b1 a1;  
   *       d2 c2 b2 a2]
   * \f[ S(x) = a1 + b1 xloc + c1 xloc^2 + d1 xloc^3 for x\in[0;0.3] \text{and} x_loc = x \f]
   * \f[ S(x) = a2 + b2 xloc + c2 xloc^2 + d2 xloc^3 for x\in[0.3;0.5] \text{and} xloc = x - 0.3 \f]

   * Cubic spline
   * (xi,fi) i=1..N is being interpolated by N-1 piecewise polynomials Si of degree 3 yielding a global \f$ C^2 \f$ curve
   * for uniqueness TWO additional boundary conditions are necessary (periodic / natural)
   * 
   * Piecewise linear polynomial
   * (xi,fi) i=1..N is being interpolated by N-1 piecewise polynomials Si of degree 1 yielding a globally weak differentiable curve
   * in the context of this class the second derivative is defined to be zero everywhere (which is mathematically wrong)
   */
  template<typename Ret, typename Arg>
  class PiecewisePolynomFunction<Ret(Arg)> : public MBSim::Function<Ret(Arg)> {
    protected:
    using B = fmatvec::Function<Ret(Arg)>; 

    public:
      enum InterpolationMethod {
        cSplinePeriodic,
        cSplineNatural,
        piecewiseLinear,
        useBreaksAndCoefs
      };

      enum ExtrapolationMethod {
        error,
        continuePolynom,
        linear,
      };

      PiecewisePolynomFunction() : index(0), interpolationMethod(cSplineNatural), extrapolationMethod(error), f(this), fd(this), fdd(this) { }
      virtual ~PiecewisePolynomFunction() = default;

      int getArgSize() const { return 1; }

      std::pair<int, int> getRetSize() const { return std::make_pair(coefs[0].cols(),1); }

      void init(Element::InitStage stage, const InitConfigSet &config=InitConfigSet()) {
        Function<Ret(Arg)>::init(stage, config);
        if(stage==Element::preInit) {
          if(interpolationMethod!=useBreaksAndCoefs) {
            if(y.rows() != x.size())
              this->throwError("Dimension missmatch in size of x");
            for(int i=1; i<x.size(); i++)
              if(x(i) <= x(i-1))
                this->throwError("Values of x must be strictly monotonic increasing!");
            calculateSpline();
          }
          else {
            for(int i=1; i<breaks.size(); i++)
              if(breaks(i) < breaks(i-1))
                this->throwError("Values of breaks must be monotonic increasing!");
            for(int j=0; j<static_cast<int>(coefs.size()); j++) {
              if(coefs[j].rows()!=breaks.size()-1)
                this->throwError("Dimension missmatch in size of breaks and rows of coefficients!");
              if(coefs[0].cols()!=coefs[j].cols())
                this->throwError("Dimension missmatch in columns of coefficients!");
            }

            // remove duplicate entries from breaks
            std::set<int> removeIdx;
            for(int i=0; i<breaks.size()-1; ++i)
              if(breaks(i)==breaks(i+1))
                removeIdx.emplace(i);
            if(!removeIdx.empty()) {
              auto breaksOld(std::move(breaks));
              auto coefsOld(std::move(coefs));
              breaks.resize(breaksOld.size()-removeIdx.size());
              coefs.resize(coefsOld.size());
              for(auto &c : coefs)
                c.resize(coefsOld[0].rows()-removeIdx.size(),coefsOld[0].cols());
              int ii=0, i=0;
              for(i=0; i<breaksOld.size()-1; ++i) {
                if(removeIdx.find(i)!=removeIdx.end())
                  continue;
                breaks(ii)=breaksOld(i);
                for(int o=0; o<static_cast<int>(coefs.size()); o++)
                  coefs[o].set(ii,coefsOld[o].row(i));
                ++ii;
              }
              breaks(ii)=breaksOld(i);
            }

          }
          nPoly = (coefs[0]).rows();
          order = coefs.size()-1;
        }
      }

      void calculateSpline() {
        if(interpolationMethod == cSplinePeriodic) calculateSplinePeriodic();
        else if(interpolationMethod == cSplineNatural) calculateSplineNatural();
        else if(interpolationMethod == piecewiseLinear) calculatePLinear();
        else this->throwError("(PiecewisePolynomFunction::init): No valid method to calculate pp-form");
      }

      void reset() {
        index = 0;
        f.reset();
        fd.reset();
        fdd.reset();
        coefs.clear();
      }

      Ret operator()(const Arg &x) { return f(ToDouble<Arg>::cast(x)); }

      // we do not use B::DRetDArg as return type here since SWIG cannot wrap the propably
      // -> hence we resolve B::DRetDArg manually here to fmatvec::Der<Ret, Arg>::type
      typename fmatvec::Der<Ret, Arg>::type parDer(const Arg &x) { return fd(ToDouble<Arg>::cast(x)); }
      typename fmatvec::Der<Ret, Arg>::type parDerDirDer(const Arg &argDir, const Arg &arg) { return fdd(ToDouble<Arg>::cast(arg))*ToDouble<Arg>::cast(argDir); }

      /*!
       * \return interval boundaries
       */
      fmatvec::VecV getBreaks() { return breaks; }

      /*!
       * \brief set interpolation
       * @param InterpolationMethod     'cSplinePeriodic' -> cubic Spline with periodic end conditions (two-times continuously differentiable)
       *                                                                                        S(x1) = S(xN) -> f(0)=f(end)
       *                                                                                        S'(x1) = S'(xN)
       *                                                                                        S''(x1) = S''(xN)
       *                                'cSplineNatural' -> cubic Spline with natural end conditions (two-times continuously differentiable)
       *                                                                                        S''(x1) = S''(xN) = 0
       *                                'piecewiseLinear'    -> piecewise linear function (weak differentiable)
       */
      void setInterpolationMethod(InterpolationMethod method_) { interpolationMethod = method_; }

      void setExtrapolationMethod(ExtrapolationMethod method_) { extrapolationMethod = method_; }

      void setx(const fmatvec::VecV &x_) { x <<= x_; }
      void sety(const fmatvec::MatV &y_) { y <<= y_; }
      void setxy(const fmatvec::MatV &xy) {
        if(xy.cols() <= 1)
          this->throwError("Dimension missmatch in size of xy");
        x <<= xy.col(0);
        y <<= xy(fmatvec::RangeV(0, xy.rows() - 1), fmatvec::RangeV(1, xy.cols() - 1));
      }

      /*!
       * \brief set polynomial coefficients
       * \param polynomial coefficients
       */
      void setCoefficientsArray(const std::vector<fmatvec::MatV> &allCoefs);

      // deprecated interface
      void setCoefficients(const std::vector<fmatvec::MatV> &allCoefs) { setCoefficientsArray(allCoefs); }

      void addCoefficients(const fmatvec::MatV &coef);

      /*!
       * \brief set interval boundaries
       * \param interval boundaries
       */
      void setBreaks(const fmatvec::VecV &breaks_u) {
        interpolationMethod=useBreaksAndCoefs;
        breaks <<= breaks_u;
      }

      /**
       * \brief initialize function with XML code
       * \param XML element
       */
      virtual void initializeUsingXML(xercesc::DOMElement *element);

    protected:
      /** 
       * \brief vector of polynomial coefficents
       */
      std::vector<fmatvec::MatV> coefs;

      /**
       * \brief vector of breaks (interval boundaries)
       */
      fmatvec::VecV breaks;

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

      fmatvec::VecV x;
      fmatvec::MatV y;

      /**
       * \brief interpolation interpolationMethod
       */
      InterpolationMethod interpolationMethod;

      ExtrapolationMethod extrapolationMethod;

      /*! 
       * \brief calculation of periodic spline by interpolation
       * \param interpolated arguments
       * \param interpolated function values
       */  
      void calculateSplinePeriodic();

      /*! 
       * \brief calculation of natural spline by interpolation
       * \param interpolated arguments
       * \param interpolated function values
       */  
      void calculateSplineNatural();

      /* 
       * \brief calculation of piecewise linear interpolation
       * \param interpolated arguments
       * \param interpolated function values
       *
       * the first derivative is weak and the second derivative is zero elsewhere although it should be distributionally at the corners
       */
      void calculatePLinear();

#ifndef SWIG
     /**
       * piecewise polynomial interpolation - zeroth derivative
       */
      class ZerothDerivative {
        public:
          ZerothDerivative(PiecewisePolynomFunction<Ret(Arg)> *polynom) : parent(polynom), xSave(0), ySave(), firstCall(true) {}

          void reset() { firstCall = true; }

          Ret operator()(const double &x);

        private:
          PiecewisePolynomFunction<Ret(Arg)> *parent;
          double xSave;
          fmatvec::VecV ySave;
          bool firstCall;
      };

      /**
       * piecewise polynomial interpolation - first derivative
       */
      class FirstDerivative {
        public:
          FirstDerivative(PiecewisePolynomFunction<Ret(Arg)> *polynom) : parent(polynom), xSave(0), ySave(), firstCall(true) {}

          void reset() { firstCall = true; }

          Ret operator()(const double& x);

        private:
          PiecewisePolynomFunction<Ret(Arg)> *parent;
          double xSave;
          fmatvec::VecV ySave;
          bool firstCall;
      };

      /**
       * piecewise polynomial interpolation - second derivative
       */
      class SecondDerivative {
        public:
          SecondDerivative(PiecewisePolynomFunction<Ret(Arg)> *polynom) : parent(polynom), xSave(0), ySave(), firstCall(true) {}

          void reset() { firstCall = true; }

          Ret operator()(const double& x);

        private:
          PiecewisePolynomFunction<Ret(Arg)> *parent;
          double xSave;
          fmatvec::VecV ySave;
          bool firstCall;
      };
#endif

    private:
      ZerothDerivative f;
      FirstDerivative fd;
      SecondDerivative fdd;
  };

  // the code is moved to piecewise_polynom_function_impl to avoid SWIG parsing errors
  #include "piecewise_polynom_function_impl.h"

}

#endif /* PPOLYNOM */
