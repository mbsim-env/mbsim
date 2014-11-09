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
 * Contact: markus.ms.schneider@gmail.com
 */

#ifndef _TABULAR_FUNCTIONS_H_
#define _TABULAR_FUNCTIONS_H_

#include <iostream>
#include "mbsim/functions/function.h"
#include "mbsim/objectfactory.h"
#include "mbsim/element.h"
#include "mbsim/utils/utils.h"

namespace MBSim {

  template<typename Sig> class TabularFunction; 

  template<typename Ret, typename Arg>
  class TabularFunction<Ret(Arg)> : public Function<Ret(Arg)> {

    public:
      TabularFunction() : xIndexOld(0) { }
      TabularFunction(const fmatvec::VecV &x_, const fmatvec::MatV &y_) : x(x_), y(y_), xIndexOld(0) { }
      Ret operator()(const Arg& xVal_) {
        double xVal = ToDouble<Arg>::cast(xVal_);
        int i = xIndexOld;
        if (xVal <= x(0)) {
          xIndexOld = 0;
          return FromVecV<Ret>::cast(trans(y.row(0)));
        }
        else if (xVal >= x(xSize - 1)) {
          xIndexOld = xSize - 1;
          return FromVecV<Ret>::cast(trans(y.row(xSize - 1)));
        }
        else if (xVal <= x(i)) {
          while (xVal < x(i))
            i--;
        }
        else {
          do
            i++;
          while (xVal > x(i));
          i--;
        }
        xIndexOld = i;
        return FromVecV<Ret>::cast(trans(y.row(i) + (xVal - x(i)) * (y.row(i + 1) - y.row(i)) / (x(i + 1) - x(i))));
      }
      void initializeUsingXML(xercesc::DOMElement * element) {
        xercesc::DOMElement *e = MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"x");
        if (e) {
          x = Element::getVec(e);
          e = MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"y");
          y = Element::getMat(e, x.size(), 0);
        }
        e = MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"xy");
        if (e) {
          fmatvec::MatV xy = Element::getMat(e);
          assert(xy.cols() > 1);
          x = xy.col(0);
          y = xy(fmatvec::Range<fmatvec::Var, fmatvec::Var>(0, xy.rows() - 1), fmatvec::Range<fmatvec::Var, fmatvec::Var>(1, xy.cols() - 1));
        }
      }
      void init(Element::InitStage stage) {
        Function<Ret(Arg)>::init(stage);
        if(stage==Element::preInit) {
          for (int i = 1; i < x.size(); i++)
            assert(x(i) > x(i - 1));
          assert(x.size() == y.rows());
          xSize = x.size();
        }
      }
    protected:
      fmatvec::VecV x;
      fmatvec::MatV y;
    private:
      int xIndexOld, xSize;
  };

  template<typename Sig> class PeriodicTabularFunction; 

  template<typename Ret, typename Arg>
  class PeriodicTabularFunction<Ret(Arg)> : public TabularFunction<Ret(Arg)> {
    public:
      PeriodicTabularFunction() { }
      PeriodicTabularFunction(const fmatvec::VecV &x_, const fmatvec::MatV &y_) : TabularFunction<Ret(Arg)>(x_, y_) { }
      Ret operator()(const Arg& x) {
        double xValTmp = ToDouble<Arg>::cast(x);
        while (xValTmp < xMin)
          xValTmp += xDelta;
        while (xValTmp > xMax)
          xValTmp -= xDelta;
        return TabularFunction<Ret(Arg)>::operator()(xValTmp);
      }
      void initializeUsingXML(xercesc::DOMElement *element) {
        TabularFunction<Ret(Arg)>::initializeUsingXML(element);
      }
      void init(Element::InitStage stage) {
        Function<Ret(Arg)>::init(stage);
        if(stage==Element::preInit) {
          xMin = TabularFunction<Ret(Arg)>::x(0);
          xMax = this->x(TabularFunction<Ret(Arg)>::x.size() - 1);
          xDelta = xMax - xMin;
        }
      }
    private:
      double xMin, xMax, xDelta;
  };

  template<typename Sig> class TwoDimensionalTabularFunction; 

  template<typename Ret, typename Arg1, typename Arg2>
  class TwoDimensionalTabularFunction<Ret(Arg1,Arg2)> : public Function<Ret(Arg1, Arg2)> {
    public:
      TwoDimensionalTabularFunction() :
          xSize(0), ySize(0), x0Index(0), x1Index(0), y0Index(0), y1Index(0), func_value(1, fmatvec::INIT, 0), xy(4, fmatvec::INIT, 1), XYval(4, fmatvec::INIT, 0), XYfac(4, 4, fmatvec::INIT, 0) {
      }
      /* INHERITED INTERFACE OF FUNCTION2 */
      virtual void initializeUsingXML(xercesc::DOMElement *element) {
        xercesc::DOMElement * e = MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"xValues");
        fmatvec::Vec x_ = Element::getVec(e);
        setXValues(x_);
        e = MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"yValues");
        fmatvec::Vec y_ = Element::getVec(e);
        setYValues(y_);
        e = MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"xyValues");
        fmatvec::Mat xy_ = Element::getMat(e, y_.size(), x_.size());
        setXYMat(xy_);
      }
      virtual Ret operator()(const Arg1& x_, const Arg2& y_) {
        double x = ToDouble<Arg1>::cast(x_);
        double y = ToDouble<Arg2>::cast(y_);
        calcIndex(&x, xVec, &xSize, &x0Index, &x1Index);
        calcIndex(&y, yVec, &ySize, &y0Index, &y1Index);

        xy(1) = x;
        xy(2) = y;
        xy(3) = x * y;
        const double x0 = xVec(x0Index);
        const double x1 = xVec(x1Index);
        const double y0 = yVec(y0Index);
        const double y1 = yVec(y1Index);
        const double nenner = (x0 - x1) * (y0 - y1);
        XYval(0) = XY(y0Index, x0Index);
        XYval(1) = XY(y0Index, x1Index);
        XYval(2) = XY(y1Index, x0Index);
        XYval(3) = XY(y1Index, x1Index);
        XYfac(0, 0) = x1 * y1;
        XYfac(0, 1) = -x0 * y1;
        XYfac(0, 2) = -x1 * y0;
        XYfac(0, 3) = x0 * y0;
        XYfac(1, 0) = -y1;
        XYfac(1, 1) = y1;
        XYfac(1, 2) = y0;
        XYfac(1, 3) = -y0;
        XYfac(2, 0) = -x1;
        XYfac(2, 1) = x0;
        XYfac(2, 2) = x1;
        XYfac(2, 3) = -x0;
        XYfac(3, 0) = 1.;
        XYfac(3, 1) = -1.;
        XYfac(3, 2) = -1.;
        XYfac(3, 3) = 1.;

        return FromDouble<Ret>::cast(trans(1. / nenner * XYfac * XYval) * xy);
      }
      /***************************************************/
      /* GETTER / SETTER */
      void setXValues(const fmatvec::Vec &xVec_) {
        xVec << xVec_;
        xSize = xVec.size();

        for (int i = 1; i < xVec.size(); i++)
          if (xVec(i - 1) >= xVec(i))
            THROW_MBSIMERROR("xVec must be strictly monotonic increasing!");
        xSize = xVec.size();
      }
      void setYValues(const fmatvec::Vec &yVec_) {
        yVec << yVec_;
        ySize = yVec.size();

        for (int i = 1; i < yVec.size(); i++)
          if (yVec(i - 1) >= yVec(i))
            THROW_MBSIMERROR("yVec must be strictly monotonic increasing!");
      }
      void setXYMat(const fmatvec::Mat &XY_) {
        XY << XY_;

        if (xSize == 0)
          fmatvec::Atom::msg(fmatvec::Atom::Warn) << "It is strongly recommended to set x file first! Continuing anyway." << std::endl;
        else if (ySize == 0)
          fmatvec::Atom::msg(fmatvec::Atom::Warn) << "It is strongly recommended to set y file first! Continuing anyway." << std::endl;
        else {
          if (XY.cols() != xSize)
            THROW_MBSIMERROR("Dimension missmatch in xSize");
          else if (XY.rows() != ySize)
            THROW_MBSIMERROR("Dimension missmatch in ySize");
        }
      }

      double getxMin() {
        return xVec(0);
      }
      double getxMax() {
        return xVec(xVec.size() - 1);
      }
      double getyMin() {
        return yVec(0);
      }
      double getyMax() {
        return yVec(yVec.size() - 1);
      }

      /***************************************************/

    protected:
      fmatvec::Vec xVec;
      fmatvec::Vec yVec;
      fmatvec::Mat XY;

      int xSize;
      int ySize;
      int x0Index, x1Index;
      int y0Index, y1Index;

      fmatvec::Vec func_value;
      fmatvec::Vec xy;
      fmatvec::Vec XYval;
      fmatvec::Mat XYfac;

      void calcIndex(const double * x, fmatvec::Vec X, int * xSize, int * xIndexMinus, int * xIndexPlus) {
        if (*x <= X(0)) {
          *xIndexPlus = 1;
          *xIndexMinus = 0;
          fmatvec::Atom::msg(fmatvec::Atom::Warn) << "TwoDimensionalTabularFunction: Value (" << *x << ") is smaller than the smallest table value(" << X(0) << ")!" << std::endl;
        }
        else if (*x >= X(*xSize - 1)) {
          *xIndexPlus = *xSize - 1;
          *xIndexMinus = *xSize - 2;
          fmatvec::Atom::msg(fmatvec::Atom::Warn) << "TwoDimensionalTabularFunction: Value (" << *x << ") is greater than the greatest table value(" << X(*xSize - 1) << ")!" << std::endl;
        }
        else {
          if (*x < X(*xIndexPlus))
            while (*x < X(*xIndexPlus - 1) && *xIndexPlus > 1)
              (*xIndexPlus)--;
          else if (*x > X(*xIndexPlus))
            while (*x > X(*xIndexPlus) && *xIndexPlus < *xSize - 1)
              (*xIndexPlus)++;
          *xIndexMinus = *xIndexPlus - 1;
        }
      }

  };
}

#endif
