/* Copyright (C) 2004-2016 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */

#ifndef _TWO_DIMENSIONAL_PIECEWISE_POLYNOM_FUNCTION_H_
#define _TWO_DIMENSIONAL_PIECEWISE_POLYNOM_FUNCTION_H_

#include "mbsim/functions/piecewise_polynom_function.h"

namespace MBSim {

  template<typename Sig> class TwoDimensionalPiecewisePolynomFunction; 

  template<typename Ret, typename Arg1, typename Arg2>
  class TwoDimensionalPiecewisePolynomFunction<Ret(Arg1, Arg2)> : public Function<Ret(Arg1, Arg2)> {
    public:
      enum InterpolationMethod {
        cSplinePeriodic,
        cSplineNatural,
        piecewiseLinear
      };

      TwoDimensionalPiecewisePolynomFunction() : method(cSplineNatural) {
        f1.setParent(this);
        f2.setParent(this);
      }

      virtual void initializeUsingXML(xercesc::DOMElement *element) {
        xercesc::DOMElement * e = MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"x");
        if(e) {
          setx(Element::getVec(e));
          e = MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"y");
          sety(Element::getVec(e));
          e = MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"z");
          setz(Element::getMat(e, y.size(), x.size()));
        }
        e = MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"xyz");
        if(e) setxyz(Element::getMat(e));
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"interpolationMethod");
        if(e) { 
          std::string str=MBXMLUtils::X()%MBXMLUtils::E(e)->getFirstTextChild()->getData();
          str=str.substr(1,str.length()-2);
          if(str=="cSplinePeriodic") method=cSplinePeriodic;
          else if(str=="cSplineNatural") method=cSplineNatural;
          else if(str=="piecewiseLinear") method=piecewiseLinear;
        }
      }

      virtual Ret operator()(const Arg1& xVal, const Arg2& yVal) {
        f2.sety(f1(xVal));
        f2.reset();
        f2.calculateSpline();
        return f2(yVal);
      }

      typename fmatvec::Der<Ret, Arg1>::type parDer1(const Arg1 &xVal, const Arg2 &yVal) {
        f2.sety(f1.parDer(xVal));
        f2.reset();
        f2.calculateSpline();
        return f2(yVal);
      }

      typename fmatvec::Der<Ret, Arg2>::type parDer2(const Arg1 &xVal, const Arg2 &yVal) {
        f2.sety(f1(xVal));
        f2.reset();
        f2.calculateSpline();
        return f2.parDer(yVal);
      }

      typename fmatvec::Der<Ret, Arg1>::type parDer1DirDer1(const Arg1 &xdVal, const Arg1 &xVal, const Arg2 &yVal) {
        f2.sety(f1.parDerDirDer(xdVal,xVal));
        f2.reset();
        f2.calculateSpline();
        return f2(yVal);
      }

      typename fmatvec::Der<Ret, Arg1>::type parDer1DirDer2(const Arg2 &ydVal, const Arg1 &xVal, const Arg2 &yVal) {
        f2.sety(f1.parDer(xVal));
        f2.reset();
        f2.calculateSpline();
        return f2.parDer(yVal)*ydVal;
      }

      typename fmatvec::Der<Ret, Arg2>::type parDer2DirDer1(const Arg1 &xdVal, const Arg1 &xVal, const Arg2 &yVal) {
        f2.sety(f1.parDer(xVal)*xdVal);
        f2.reset();
        f2.calculateSpline();
        return f2.parDer(yVal);
      }

      typename fmatvec::Der<Ret, Arg2>::type parDer2DirDer2(const Arg2 &ydVal, const Arg1 &xVal, const Arg2 &yVal) {
        f2.sety(f1(xVal));
        f2.reset();
        f2.calculateSpline();
        return f2.parDerDirDer(ydVal,yVal);
      }

      void setx(const fmatvec::VecV &x_) { x = x_; }

      void sety(const fmatvec::VecV &y_) { y = y_; }

      void setz(const fmatvec::MatV &z_) { z = z_; }

      void setxyz(const fmatvec::MatV &xyz) {
        if(xyz.rows() <= 1 or xyz.cols() <= 1)
          THROW_MBSIMERROR("Dimension missmatch in size of xyz");
        x = xyz.row(0)(fmatvec::Index(1,xyz.cols()-1)).T();
        y = xyz.col(0)(fmatvec::Index(1,xyz.rows()-1));
        z = xyz(fmatvec::Index(1,xyz.rows()-1),fmatvec::Index(1,xyz.cols()-1));
      }

      void setInterpolationMethod(InterpolationMethod method_) { method = method_; }

      void init(Element::InitStage stage) {
        Function<Ret(Arg1, Arg2)>::init(stage);
        if(stage==Element::preInit) {
          if (z.cols() != x.size())
            THROW_MBSIMERROR("Dimension missmatch in xSize");
          if (z.rows() != y.size())
            THROW_MBSIMERROR("Dimension missmatch in ySize");
          for (int i = 1; i < x.size(); i++)
            if (x(i - 1) >= x(i))
              THROW_MBSIMERROR("x values must be strictly monotonic increasing!");
          for (int i = 1; i < y.size(); i++)
            if (y(i - 1) >= y(i))
              THROW_MBSIMERROR("y values must be strictly monotonic increasing!");
          f1.setx(x);
          f1.sety(z.T());
          f1.setInterpolationMethod(static_cast<typename PiecewisePolynomFunction<fmatvec::VecV(Arg1)>::InterpolationMethod>(method));
          f2.setx(y);
          f2.sety(y);
          f2.setInterpolationMethod(static_cast<typename PiecewisePolynomFunction<Ret(Arg2)>::InterpolationMethod>(method));
        }
        f1.init(stage);
        f2.init(stage);
      }

    protected:
      fmatvec::VecV x;
      fmatvec::VecV y;
      fmatvec::MatV z;
      PiecewisePolynomFunction<fmatvec::VecV(Arg1)> f1;
      PiecewisePolynomFunction<Ret(Arg2)> f2;
      InterpolationMethod method;
  };
}

#endif
