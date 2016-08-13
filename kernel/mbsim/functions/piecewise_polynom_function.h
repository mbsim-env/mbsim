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
#include "mbsim/element.h"
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
        piecewiseLinear
      };

      PiecewisePolynomFunction() : index(0), method(cSplineNatural), f(this), fd(this), fdd(this) { }

      int getArgSize() const { return 1; }

      void init(Element::InitStage stage) {
        Function<Ret(Arg)>::init(stage);
        if(stage==Element::preInit) {
          if(y.rows() != x.size())
            THROW_MBSIMERROR("Dimension missmatch in size of x");
          if(x.size()) calculateSpline();
          nPoly = (coefs[0]).rows();
          order = coefs.size()-1;
        }
      }

      void calculateSpline() {
        if(method == cSplinePeriodic) calculateSplinePeriodic();
        else if(method == cSplineNatural) calculateSplineNatural();
        else if(method == piecewiseLinear) calculatePLinear();
        else THROW_MBSIMERROR("(PiecewisePolynomFunction::init): No valid method to calculate pp-form");
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
      Ret parDerParDer(const double &x) { return fdd(x); }
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
      void setInterpolationMethod(InterpolationMethod method_) { method = method_; }

      void setx(const fmatvec::VecV &x_) { x = x_; }
      void sety(const fmatvec::MatV &y_) { y = y_; }
      void setxy(const fmatvec::MatV &xy) {
        if(xy.cols() <= 1)
          THROW_MBSIMERROR("Dimension missmatch in size of xy");
        x = xy.col(0);
        y = xy(fmatvec::Index(0, xy.rows() - 1), fmatvec::Index(1, xy.cols() - 1));
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
      void setBreaks(const std::vector<fmatvec::MatV> &coefs_u, const fmatvec::VecV &breaks_u) { breaks = breaks_u; }

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
       * \brief interpolation method
       */
      InterpolationMethod method;

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
    if(nrm2(y.row(0)-y.row(y.rows()-1))>epsroot()) THROW_MBSIMERROR("(PiecewisePolynomFunction::calculateSplinePeriodic): f(0)= "+numtostr(y.row(0))+"!="+numtostr(y.row(y.rows()-1))+" =f(end)");
    fmatvec::SqrMat C(N-1,fmatvec::INIT,0.0);
    fmatvec::Mat rs(N-1,y.cols(),fmatvec::INIT,0.0);

    // Matrix C and vector rs C*c=rs
    for(int i=0; i<N-3;i++) {
      hi = x(i+1) - x(i);
      hii = x(i+2)-x(i+1);
      C(i,i) = hi;
      C(i,i+1) = 2*(hi+hii);
      C(i,i+2) = hii;
      rs.row(i) = 3.*((y.row(i+2)-y.row(i+1))/hii - (y.row(i+1)-y.row(i))/hi);
    }

    // last but one row
    hi = x(N-2)-x(N-3);
    hii = x(N-1)-x(N-2);
    C(N-3,N-3) = hi;
    C(N-3,N-2)= 2*(hi+hii);
    C(N-3,0)= hii;
    rs.row(N-3) = 3.*((y.row(N-1)-y.row(N-2))/hii - (y.row(N-2)-y.row(N-3))/hi);

    // last row
    double h1 = x(1)-x(0);
    double hN_1 = x(N-1)-x(N-2);
    C(N-2,0) = 2*(h1+hN_1);
    C(N-2,1) = h1;
    C(N-2,N-2)= hN_1;
    rs.row(N-2) = 3.*((y.row(1)-y.row(0))/h1 - (y.row(0)-y.row(N-2))/hN_1);

    // solve C*c = rs -> TODO BETTER: RANK-1-MODIFICATION FOR LINEAR EFFORT (Simeon - Numerik 1)
    fmatvec::Mat c = slvLU(C,rs);
    fmatvec::Mat ctmp(N,y.cols());
    ctmp.row(N-1) = c.row(0); // CN = c1
    ctmp(0,0,N-2,y.cols()-1)= c;

    // vector ordering of the further coefficients
    fmatvec::Mat d(N-1,y.cols(),fmatvec::INIT,0.0);
    fmatvec::Mat b(N-1,y.cols(),fmatvec::INIT,0.0);
    fmatvec::Mat a(N-1,y.cols(),fmatvec::INIT,0.0);

    for(int i=0; i<N-1; i++) {
      hi = x(i+1)-x(i);  
      a.row(i) = y.row(i);
      d.row(i) = (ctmp.row(i+1) - ctmp.row(i) ) / 3. / hi;
      b.row(i) = (y.row(i+1)-y.row(i)) / hi - (ctmp.row(i+1) + 2.*ctmp.row(i) ) / 3. * hi;
    }

    breaks.resize(N);
    breaks = x;
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
    fmatvec::SqrMat C(N-2,fmatvec::INIT,0.0);
    fmatvec::Mat rs(N-2,y.cols(),fmatvec::INIT,0.0);
    double hi = x(i+1)-x(i);
    double hii = x(i+2)-x(i+1);
    C(i,i) = 2*hi+2*hii;
    C(i,i+1) = hii;
    rs.row(i) = 3.*(y.row(i+2)-y.row(i+1))/hii - 3.*(y.row(i+1)-y.row(i))/hi;

    // last row
    i = (N-3);
    hi = x(i+1)-x(i);
    hii = x(i+2)-x(i+1);
    C(i,i-1) = hi;
    C(i,i) = 2*hii + 2*hi;
    rs.row(i) = 3.*(y.row(i+2)-y.row(i+1))/hii - 3.*(y.row(i+1)-y.row(i))/hi;

    for(i=1;i<N-3;i++) { 
      hi = x(i+1)-x(i);
      hii = x(i+2)-x(i+1);
      C(i,i-1) = hi;
      C(i,i) = 2*(hi+hii);
      C(i,i+1) = hii;
      rs.row(i) = 3.*(y.row(i+2)-y.row(i+1))/hii - 3.*(y.row(i+1)-y.row(i))/hi;
    }

    // solve C*c = rs with C tridiagonal
    fmatvec::Mat C_rs(N-2,N-1+y.cols(),fmatvec::INIT,0.0);
    C_rs(0,0,N-3,N-3) = C; // C_rs=[C rs] for Gauss in matrix
    C_rs(0,N-2,N-3,N-2+y.cols()-1) = rs;
    for(i=1; i<N-2; i++) C_rs.row(i) = C_rs.row(i) - C_rs.row(i-1)*C_rs(i,i-1)/C_rs(i-1,i-1); // C_rs -> upper triangular matrix C1 -> C1 rs1
    fmatvec::Mat rs1 = C_rs(0,N-2,N-3,N-2+y.cols()-1);
    fmatvec::Mat C1 = C_rs(0,0,N-3,N-3);
    fmatvec::Mat c(N-2,y.cols(),fmatvec::INIT,0.0);
    for(i=N-3;i>=0 ;i--) { // backward substitution
      fmatvec::RowVecV sum_ciCi(y.cols(),fmatvec::NONINIT);
      sum_ciCi.init(0.);
      for(int ii=i+1; ii<=N-3; ii++) sum_ciCi = sum_ciCi + C1(i,ii)*c.row(ii);
      c.row(i)= (rs1.row(i) - sum_ciCi)/C1(i,i);
    }
    fmatvec::Mat ctmp(N,y.cols(),fmatvec::INIT,0.0);
    ctmp(1,0,N-2,y.cols()-1) = c; // c1=cN=0 natural splines c=[ 0; c; 0]

    // vector ordering of the further coefficients
    fmatvec::Mat d(N-1,y.cols(),fmatvec::INIT,0.0);
    fmatvec::Mat b(N-1,y.cols(),fmatvec::INIT,0.0);
    fmatvec::Mat a(N-1,y.cols(),fmatvec::INIT,0.0);

    for(i=0; i<N-1; i++) {
      hi = x(i+1)-x(i);  
      a.row(i) = y.row(i);
      d.row(i) = (ctmp.row(i+1) - ctmp.row(i) ) / 3. / hi;
      b.row(i) = (y.row(i+1)-y.row(i)) / hi - (ctmp.row(i+1) + 2.*ctmp.row(i) ) / 3. * hi;
    }

    breaks.resize(N);
    breaks = x;
    coefs.push_back(d);
    coefs.push_back(ctmp(0,0,N-2,y.cols()-1));
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
      m.row(i-1) = (y.row(i)-y.row(i-1))/(x(i)-x(i-1)); // slope
      a.row(i-1) = y.row(i-1);
    }
    coefs.push_back(m);
    coefs.push_back(a);
  }
          
  template<typename Ret, typename Arg>
  Ret PiecewisePolynomFunction<Ret(Arg)>::ZerothDerivative::operator()(const Arg& x_) {
    double x = ToDouble<Arg>::cast(x_);
    if(x>(parent->breaks)(parent->nPoly)) 
      throw MBSimError("(PiecewisePolynomFunction::operator()): x out of range! x= "+numtostr(x)+", upper bound= "+numtostr((parent->breaks)(parent->nPoly)));
    if(x<(parent->breaks)(0)) 
      throw MBSimError("(PiecewisePolynomFunction::operator()): x out of range! x= "+numtostr(x)+", lower bound= "+numtostr((parent->breaks)(0)));

    if ((fabs(x-xSave)<macheps()) && !firstCall)
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
      ySave=yi;
      return FromVecV<Ret>::cast(yi);
    }
  }

  template<typename Ret, typename Arg>
  Ret PiecewisePolynomFunction<Ret(Arg)>::FirstDerivative::operator()(const Arg& x_) {
    double x = ToDouble<Arg>::cast(x_);
    if(x>(parent->breaks)(parent->nPoly)) throw MBSimError("(PiecewisePolynomFunction::diff1): x out of range! x= "+numtostr(x)+", upper bound= "+numtostr((parent->breaks)(parent->nPoly)));
    if(x<(parent->breaks)(0)) throw MBSimError("(PiecewisePolynomFunction::diff1): x out of range!   x= "+numtostr(x)+" lower bound= "+numtostr((parent->breaks)(0)));

    if ((fabs(x-xSave)<macheps()) && !firstCall)
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
      ySave=yi;
      return FromVecV<Ret>::cast(yi);
    }
  }

  template<typename Ret, typename Arg>
  Ret PiecewisePolynomFunction<Ret(Arg)>::SecondDerivative::operator()(const Arg& x_) {
    double x = ToDouble<Arg>::cast(x_);
    if(x>(parent->breaks)(parent->nPoly)) throw MBSimError("(PiecewisePolynomFunction::diff2): x out of range!   x= "+numtostr(x)+" upper bound= "+numtostr((parent->breaks)(parent->nPoly)));
    if(x<(parent->breaks)(0)) throw MBSimError("(PiecewisePolynomFunction::diff2): x out of range!   x= "+numtostr(x)+" lower bound= "+numtostr((parent->breaks)(0)));

    if ((fabs(x-xSave)<macheps()) && !firstCall)
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
      ySave=yi;
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
      setx(Element::getVec(e));
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"y");
      sety(Element::getMat(e, x.size(), 0));
    }
    else {
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"xy");
      setxy(Element::getMat(e));
    }
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"interpolationMethod");
    if(e) { 
      std::string str=MBXMLUtils::X()%MBXMLUtils::E(e)->getFirstTextChild()->getData();
      str=str.substr(1,str.length()-2);
      if(str=="cSplinePeriodic") method=cSplinePeriodic;
      else if(str=="cSplineNatural") method=cSplineNatural;
      else if(str=="piecewiseLinear") method=piecewiseLinear;
    }
  }

}

#endif /* PPOLYNOM */
