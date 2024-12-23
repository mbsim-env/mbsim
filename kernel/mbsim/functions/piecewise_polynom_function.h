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
  class PiecewisePolynomFunction<Ret(Arg)> : public Function<Ret(Arg)> {
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

      int getArgSize() const { return 1; }

      std::pair<int, int> getRetSize() const { return std::make_pair(coefs[0].cols(),1); }

      void init(Element::InitStage stage, const InitConfigSet &config) {
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

      Ret operator()(const Arg &x) { return f(x); }
      typename B::DRetDArg parDer(const Arg &x) { return fd(x); }
      typename B::DRetDArg parDerDirDer(const Arg &argDir, const Arg &arg) { return fdd(arg)*ToDouble<Arg>::cast(argDir); }

      /*! 
       * \return polynomial coefficients
       */
      std::vector<fmatvec::MatV> getCoefficients() { return coefs; }

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
      void setCoefficients(const std::vector<fmatvec::MatV> &coefs_u) { coefs = coefs_u; }

      /*!
       * \brief set interval boundaries
       * \param interval boundaries
       */
      void setBreaks(const fmatvec::VecV &breaks_u) { breaks <<= breaks_u; }

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

     /**
       * piecewise polynomial interpolation - zeroth derivative
       */
      class ZerothDerivative {
        public:
          ZerothDerivative(PiecewisePolynomFunction<Ret(Arg)> *polynom) : parent(polynom), xSave(0), ySave(), firstCall(true) {}

          void reset() { firstCall = true; }

          Ret operator()(const Arg &x);

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

          Ret operator()(const Arg& x);

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

          Ret operator()(const Arg& x);

        private:
          PiecewisePolynomFunction<Ret(Arg)> *parent;
          double xSave;
          fmatvec::VecV ySave;
          bool firstCall;
      };

    private:
      ZerothDerivative f;
      FirstDerivative fd;
      SecondDerivative fdd;
  };

  template<typename Ret, typename Arg>
  void PiecewisePolynomFunction<Ret(Arg)>::calculateSplinePeriodic() {
    double hi, hii;
    int N = x.size();
    if(nrm2(y.row(0)-y.row(y.rows()-1))>epsroot) this->throwError("(PiecewisePolynomFunction::calculateSplinePeriodic): f(0)= "+fmatvec::toString(y.row(0))+"!="+fmatvec::toString(y.row(y.rows()-1))+" =f(end)");
    fmatvec::SqrMat C(N-1,fmatvec::INIT,0.0);
    fmatvec::Mat rs(N-1,y.cols(),fmatvec::INIT,0.0);

    // Matrix C and vector rs C*c=rs
    for(int i=0; i<N-3;i++) {
      hi = x(i+1) - x(i);
      hii = x(i+2)-x(i+1);
      C(i,i) = hi;
      C(i,i+1) = 2*(hi+hii);
      C(i,i+2) = hii;
      rs.set(i, 3.*((y.row(i+2)-y.row(i+1))/hii - (y.row(i+1)-y.row(i))/hi));
    }

    // last but one row
    hi = x(N-2)-x(N-3);
    hii = x(N-1)-x(N-2);
    C(N-3,N-3) = hi;
    C(N-3,N-2)= 2*(hi+hii);
    C(N-3,0)= hii;
    rs.set(N-3, 3.*((y.row(N-1)-y.row(N-2))/hii - (y.row(N-2)-y.row(N-3))/hi));

    // last row
    double h1 = x(1)-x(0);
    double hN_1 = x(N-1)-x(N-2);
    C(N-2,0) = 2*(h1+hN_1);
    C(N-2,1) = h1;
    C(N-2,N-2)= hN_1;
    rs.set(N-2, 3.*((y.row(1)-y.row(0))/h1 - (y.row(0)-y.row(N-2))/hN_1));

    // solve C*c = rs -> TODO BETTER: RANK-1-MODIFICATION FOR LINEAR EFFORT (Simeon - Numerik 1)
    fmatvec::Mat c = slvLU(C,rs);
    fmatvec::Mat ctmp(N,y.cols());
    ctmp.set(N-1, c.row(0)); // CN = c1
    ctmp.set(fmatvec::RangeV(0,N-2),fmatvec::RangeV(0,y.cols()-1), c);

    // vector ordering of the further coefficients
    fmatvec::Mat d(N-1,y.cols(),fmatvec::INIT,0.0);
    fmatvec::Mat b(N-1,y.cols(),fmatvec::INIT,0.0);
    fmatvec::Mat a(N-1,y.cols(),fmatvec::INIT,0.0);

    for(int i=0; i<N-1; i++) {
      hi = x(i+1)-x(i);  
      a.set(i, y.row(i));
      d.set(i, (ctmp.row(i+1) - ctmp.row(i) ) / 3. / hi);
      b.set(i, (y.row(i+1)-y.row(i)) / hi - (ctmp.row(i+1) + 2.*ctmp.row(i) ) / 3. * hi);
    }

    breaks.resize(N);
    breaks = x;
    coefs.clear();
    coefs.push_back(d);
    coefs.push_back(c);
    coefs.push_back(b);
    coefs.push_back(a);
  }

  template<typename Ret, typename Arg>
  void PiecewisePolynomFunction<Ret(Arg)>::calculateSplineNatural() {
    // first row
    int i=0;
    int N = x.size();
    if(N<4) this->throwError("(PiecewisePolynomFunction::calculateSplineNatural): number of datapoints does not match, must be greater 3");
    fmatvec::SqrMat C(N-2,fmatvec::INIT,0.0);
    fmatvec::Mat rs(N-2,y.cols(),fmatvec::INIT,0.0);
    double hi = x(i+1)-x(i);
    double hii = x(i+2)-x(i+1);
    C(i,i) = 2*hi+2*hii;
    C(i,i+1) = hii;
    rs.set(i, 3.*(y.row(i+2)-y.row(i+1))/hii - 3.*(y.row(i+1)-y.row(i))/hi);

    // last row
    i = (N-3);
    hi = x(i+1)-x(i);
    hii = x(i+2)-x(i+1);
    C(i,i-1) = hi;
    C(i,i) = 2*hii + 2*hi;
    rs.set(i, 3.*(y.row(i+2)-y.row(i+1))/hii - 3.*(y.row(i+1)-y.row(i))/hi);

    for(i=1;i<N-3;i++) { 
      hi = x(i+1)-x(i);
      hii = x(i+2)-x(i+1);
      C(i,i-1) = hi;
      C(i,i) = 2*(hi+hii);
      C(i,i+1) = hii;
      rs.set(i, 3.*(y.row(i+2)-y.row(i+1))/hii - 3.*(y.row(i+1)-y.row(i))/hi);
    }

    // solve C*c = rs with C tridiagonal
    fmatvec::Mat C_rs(N-2,N-1+y.cols(),fmatvec::INIT,0.0);
    C_rs.set(fmatvec::RangeV(0,N-3),fmatvec::RangeV(0,N-3), C); // C_rs=[C rs] for Gauss in matrix
    C_rs.set(fmatvec::RangeV(0,N-3),fmatvec::RangeV(N-2,N-2+y.cols()-1), rs);
    for(i=1; i<N-2; i++) C_rs.set(i, C_rs.row(i) - C_rs.row(i-1)*C_rs(i,i-1)/C_rs(i-1,i-1)); // C_rs -> upper triangular matrix C1 -> C1 rs1
    fmatvec::Mat rs1 = C_rs(fmatvec::RangeV(0,N-3),fmatvec::RangeV(N-2,N-2+y.cols()-1));
    fmatvec::Mat C1 = C_rs(fmatvec::RangeV(0,N-3),fmatvec::RangeV(0,N-3));
    fmatvec::Mat c(N-2,y.cols(),fmatvec::INIT,0.0);
    for(i=N-3;i>=0 ;i--) { // backward substitution
      fmatvec::RowVecV sum_ciCi(y.cols(),fmatvec::NONINIT);
      sum_ciCi.init(0.);
      for(int ii=i+1; ii<=N-3; ii++) sum_ciCi = sum_ciCi + C1(i,ii)*c.row(ii);
      c.set(i, (rs1.row(i) - sum_ciCi)/C1(i,i));
    }
    fmatvec::Mat ctmp(N,y.cols(),fmatvec::INIT,0.0);
    ctmp.set(fmatvec::RangeV(1,N-2),fmatvec::RangeV(0,y.cols()-1), c); // c1=cN=0 natural splines c=[ 0; c; 0]

    // vector ordering of the further coefficients
    fmatvec::Mat d(N-1,y.cols(),fmatvec::INIT,0.0);
    fmatvec::Mat b(N-1,y.cols(),fmatvec::INIT,0.0);
    fmatvec::Mat a(N-1,y.cols(),fmatvec::INIT,0.0);

    for(i=0; i<N-1; i++) {
      hi = x(i+1)-x(i);  
      a.set(i, y.row(i));
      d.set(i, (ctmp.row(i+1) - ctmp.row(i) ) / 3. / hi);
      b.set(i, (y.row(i+1)-y.row(i)) / hi - (ctmp.row(i+1) + 2.*ctmp.row(i) ) / 3. * hi);
    }

    breaks.resize(N);
    breaks = x;
    coefs.clear();
    coefs.push_back(d);
    coefs.push_back(ctmp(fmatvec::RangeV(0,N-2),fmatvec::RangeV(0,y.cols()-1)));
    coefs.push_back(b);
    coefs.push_back(a);
  }

  template<typename Ret, typename Arg>
  void PiecewisePolynomFunction<Ret(Arg)>::calculatePLinear() {
    int N = x.size(); // number of supporting points

    breaks.resize(N);
    breaks = x;

    fmatvec::Mat m(N-1,y.cols(),fmatvec::INIT,0.0);
    fmatvec::Mat a(N-1,y.cols(),fmatvec::INIT,0.0);
    for(int i=1;i<N;i++) {
      m.set(i-1, (y.row(i)-y.row(i-1))/(x(i)-x(i-1))); // slope
      a.set(i-1, y.row(i-1));
    }
    coefs.clear();
    coefs.push_back(m);
    coefs.push_back(a);
  }
          
  template<typename Ret, typename Arg>
  Ret PiecewisePolynomFunction<Ret(Arg)>::ZerothDerivative::operator()(const Arg& x_) {
    double x = ToDouble<Arg>::cast(x_);
    if(parent->extrapolationMethod==error) {
      if(x-1e-13>(parent->breaks)(parent->nPoly)) 
        throw std::runtime_error("(PiecewisePolynomFunction::operator()): x out of range! x= "+fmatvec::toString(x)+", upper bound= "+fmatvec::toString((parent->breaks)(parent->nPoly)));
      if(x+1e-13<(parent->breaks)(0)) 
        throw std::runtime_error("(PiecewisePolynomFunction::operator()): x out of range! x= "+fmatvec::toString(x)+", lower bound= "+fmatvec::toString((parent->breaks)(0)));
    }
    if(parent->extrapolationMethod==linear && (x<parent->breaks(0) || x>parent->breaks(parent->nPoly))) {
      int idx = x<parent->breaks(0) ? 0 : parent->nPoly;
      auto xv = FromDouble<Arg>::cast(parent->breaks(idx));
      auto value = parent->f(xv);
      auto der = parent->fd(xv);
      return value + der * (x - parent->breaks(idx));
    }

    if ((fabs(x-xSave)<macheps) && !firstCall)
      return FromVecV<Ret>::cast(ySave);
    else {
      firstCall = false;
      if(x<(parent->breaks)(parent->index)) // saved index still OK? otherwise search downwards
        while((parent->index) > 0 && (parent->breaks)(parent->index) > x)
          (parent->index)--;
      else if(x>(parent->breaks)((parent->index)+1)) { // saved index still OK? otherwise search upwards
        while((parent->index) < (parent->nPoly) && (parent->breaks)(parent->index) <= x)
          (parent->index)++;
        (parent->index)--;  
      }

      const double dx = x - (parent->breaks)(parent->index); // local coordinate
      fmatvec::VecV yi = trans(((parent->coefs)[0]).row(parent->index));
      for(int i=1;i<=(parent->order);i++) // Horner scheme
        yi = yi*dx+trans(((parent->coefs)[i]).row(parent->index));
      xSave=x;
      ySave <<= yi;
      return FromVecV<Ret>::cast(yi);
    }
  }

  template<typename Ret, typename Arg>
  Ret PiecewisePolynomFunction<Ret(Arg)>::FirstDerivative::operator()(const Arg& x_) {
    double x = ToDouble<Arg>::cast(x_);
    if(parent->extrapolationMethod==error) {
      if(x-1e-13>(parent->breaks)(parent->nPoly)) throw std::runtime_error("(PiecewisePolynomFunction::diff1): x out of range! x= "+fmatvec::toString(x)+", upper bound= "+fmatvec::toString((parent->breaks)(parent->nPoly)));
      if(x+1e-13<(parent->breaks)(0)) throw std::runtime_error("(PiecewisePolynomFunction::diff1): x out of range!   x= "+fmatvec::toString(x)+" lower bound= "+fmatvec::toString((parent->breaks)(0)));
    }
    if(parent->extrapolationMethod==linear && (x<parent->breaks(0) || x>parent->breaks(parent->nPoly))) {
      int idx = x<parent->breaks(0) ? 0 : parent->nPoly;
      auto xv = FromDouble<Arg>::cast(parent->breaks(idx));
      auto der = parent->fd(xv);
      return der;
    }

    if ((fabs(x-xSave)<macheps) && !firstCall)
      return FromVecV<Ret>::cast(ySave);
    else {
      firstCall = false;
      if(x<(parent->breaks)(parent->index)) // saved index still OK? otherwise search downwards
        while((parent->index) > 0 && (parent->breaks)(parent->index) > x)
          (parent->index)--;
      else if(x>(parent->breaks)((parent->index)+1)) { // saved index still OK? otherwise search upwards
        while((parent->index) < (parent->nPoly) && (parent->breaks)(parent->index) <= x)
          (parent->index)++;
        (parent->index)--;  
      }

      double dx = x - (parent->breaks)(parent->index);
      fmatvec::VecV yi = trans(((parent->coefs)[0]).row(parent->index))*double(parent->order);
      for(int i=1;i<parent->order;i++)
        yi = yi*dx+trans(((parent->coefs)[i]).row(parent->index))*double((parent->order)-i);
      xSave=x;
      ySave <<= yi;
      return FromVecV<Ret>::cast(yi);
    }
  }

  template<typename Ret, typename Arg>
  Ret PiecewisePolynomFunction<Ret(Arg)>::SecondDerivative::operator()(const Arg& x_) {
    double x = ToDouble<Arg>::cast(x_);
    if(parent->extrapolationMethod==error) {
      if(x-1e-13>(parent->breaks)(parent->nPoly)) throw std::runtime_error("(PiecewisePolynomFunction::diff2): x out of range!   x= "+fmatvec::toString(x)+" upper bound= "+fmatvec::toString((parent->breaks)(parent->nPoly)));
      if(x+1e-13<(parent->breaks)(0)) throw std::runtime_error("(PiecewisePolynomFunction::diff2): x out of range!   x= "+fmatvec::toString(x)+" lower bound= "+fmatvec::toString((parent->breaks)(0)));
    }
    if(parent->extrapolationMethod==linear && (x<parent->breaks(0) || x>parent->breaks(parent->nPoly)))
      return FromVecV<Ret>::cast(fmatvec::VecV(parent->getRetSize().first, fmatvec::INIT, 0.0));

    if ((fabs(x-xSave)<macheps) && !firstCall)
      return FromVecV<Ret>::cast(ySave);
    else {
      firstCall = false;
      if(x<(parent->breaks)(parent->index)) // saved index still OK? otherwise search downwards
        while((parent->index) > 0 && (parent->breaks)(parent->index) > x)
          (parent->index)--;
      else if(x>(parent->breaks)((parent->index)+1)) { // saved index still OK? otherwise search upwards
        while((parent->index) < (parent->nPoly) && (parent->breaks)(parent->index) <= x)
          (parent->index)++;
        (parent->index)--;  
      }

      double dx = x - (parent->breaks)(parent->index);
      fmatvec::VecV yi = trans(((parent->coefs)[0]).row(parent->index))*double(parent->order)*double((parent->order)-1);
      for(int i=1;i<=((parent->order)-2);i++)
        yi = yi*dx+trans(((parent->coefs)[i]).row(parent->index))*double((parent->order)-i)*double((parent->order)-i-1);
      xSave=x;
      ySave <<= yi;
      return FromVecV<Ret>::cast(yi);
    }
  }

  template<typename Ret, typename Arg>
  void PiecewisePolynomFunction<Ret(Arg)>::initializeUsingXML(xercesc::DOMElement * element) {
    xercesc::DOMElement *e;
    fmatvec::VecV x;
    fmatvec::MatV y;
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"x");
    if (e) {
      setx(MBXMLUtils::E(e)->getText<fmatvec::Vec>());
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"y");
      sety(MBXMLUtils::E(e)->getText<fmatvec::Mat>(x.size(), 0));
    }
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"xy");
    if (e) {
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"xy");
      setxy(MBXMLUtils::E(e)->getText<fmatvec::Mat>());
    }
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"interpolationMethod");
    if(e) { 
      std::string str=MBXMLUtils::X()%MBXMLUtils::E(e)->getFirstTextChild()->getData();
      str=str.substr(1,str.length()-2);
      if(str=="cSplinePeriodic")      interpolationMethod=cSplinePeriodic;
      else if(str=="cSplineNatural")  interpolationMethod=cSplineNatural;
      else if(str=="piecewiseLinear") interpolationMethod=piecewiseLinear;
      else interpolationMethod=useBreaksAndCoefs;
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"breaks");
    if(e) { 
      // set breaks
      setBreaks(MBXMLUtils::E(e)->getText<fmatvec::Vec>());

      // count number of columns = dimension of the vector returned by this function
      int nrCols = 0;
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"coefficients");
      while(e) {
        nrCols++;
        e=MBXMLUtils::E(e)->getNextElementSiblingNamed(MBSIM%"coefficients");
      }

      // read all coefficients element and convert to coefs
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"coefficients");
      auto c=MBXMLUtils::E(e)->getText<fmatvec::Mat>();
      std::vector<fmatvec::MatV> coefs(c.cols(), fmatvec::MatV(c.rows(),nrCols)); // init coefs with proper size
      bool first=true;
      int col = 0;
      while(e) {
        if(!first) // avoid double read of first coefficients element
          c=MBXMLUtils::E(e)->getText<fmatvec::Mat>();
        if(c.cols()!=static_cast<int>(coefs.size()))
          this->throwError("The number of columns in the coefficients elements differ.");
        if(c.rows()!=static_cast<int>(coefs[0].rows()))
          this->throwError("The number of rows in the coefficients elements differ.");
        for(int deg=0; deg<c.cols(); deg++)
          coefs[deg].set(col, c.col(deg));
        e=MBXMLUtils::E(e)->getNextElementSiblingNamed(MBSIM%"coefficients");
        col++;
        first=false;
      }
      setCoefficients(coefs);
      interpolationMethod=useBreaksAndCoefs;
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"extrapolationMethod");
    if(e) { 
      std::string str=MBXMLUtils::X()%MBXMLUtils::E(e)->getFirstTextChild()->getData();
      str=str.substr(1,str.length()-2);
      if(str=="error")      extrapolationMethod=error;
      else if(str=="continuePolynom")  extrapolationMethod=continuePolynom;
      else if(str=="linear") extrapolationMethod=linear;
      else this->throwError("Unknown extrapolationMethod.");
    }
  }

}

#endif /* PPOLYNOM */
