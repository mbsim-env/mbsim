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

#ifndef _TWO_DIMENSIONAL_TABULAR_FUNCTION_H_
#define _TWO_DIMENSIONAL_TABULAR_FUNCTION_H_

#include "mbsim/functions/function.h"
#include "mbsim/utils/utils.h"

namespace MBSim {

  template<typename Sig> class TwoDimensionalTabularFunction; 

  template<typename Ret, typename Arg1, typename Arg2>
  class TwoDimensionalTabularFunction<Ret(Arg1, Arg2)> : public Function<Ret(Arg1, Arg2)> {
    public:
      TwoDimensionalTabularFunction() :  zVal(4,fmatvec::INIT,1), zInd(4,fmatvec::INIT,0), zFac(4, 4,fmatvec::INIT,0) { }
      /* INHERITED INTERFACE OF FUNCTION2 */
      int getArg1Size() const override { return 1; }
      int getArg2Size() const override { return 1; }
      std::pair<int, int> getRetSize() const override { return std::make_pair(1,1); }
      void initializeUsingXML(xercesc::DOMElement *element) override {
        xercesc::DOMElement * e = MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"x");
        if(e) {
          setx(MBXMLUtils::E(e)->getText<fmatvec::Vec>());
          e = MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"y");
          sety(MBXMLUtils::E(e)->getText<fmatvec::Vec>());
          e = MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"z");
          setz(MBXMLUtils::E(e)->getText<fmatvec::Mat>(y.size(), x.size()));
        }
        e = MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"xyz");
        if(e) setxyz(MBXMLUtils::E(e)->getText<fmatvec::Mat>());
      }
      Ret operator()(const Arg1& xVal_, const Arg2& yVal_) override {
        double xVal = ToDouble<Arg1>::cast(xVal_);
        double yVal = ToDouble<Arg2>::cast(yVal_);
        calcIndex(xVal, x, x.size(), x0Index, x1Index);
        calcIndex(yVal, y, y.size(), y0Index, y1Index);

        zVal(1) = xVal;
        zVal(2) = yVal;
        zVal(3) = xVal * yVal;
        const double x0 = x(x0Index);
        const double x1 = x(x1Index);
        const double y0 = y(y0Index);
        const double y1 = y(y1Index);
        const double nenner = (x0 - x1) * (y0 - y1);
        zInd(0) = z(y0Index, x0Index);
        zInd(1) = z(y0Index, x1Index);
        zInd(2) = z(y1Index, x0Index);
        zInd(3) = z(y1Index, x1Index);
        zFac(0, 0) = x1 * y1;
        zFac(0, 1) = -x0 * y1;
        zFac(0, 2) = -x1 * y0;
        zFac(0, 3) = x0 * y0;
        zFac(1, 0) = -y1;
        zFac(1, 1) = y1;
        zFac(1, 2) = y0;
        zFac(1, 3) = -y0;
        zFac(2, 0) = -x1;
        zFac(2, 1) = x0;
        zFac(2, 2) = x1;
        zFac(2, 3) = -x0;
        zFac(3, 0) = 1.;
        zFac(3, 1) = -1.;
        zFac(3, 2) = -1.;
        zFac(3, 3) = 1.;

        return FromDouble<Ret>::cast((trans(zFac*zInd)*zVal)/nenner);
      }
      /***************************************************/
      /* GETTER / SETTER */
      void setx(const fmatvec::VecV &x_) { x = x_; }
      void sety(const fmatvec::VecV &y_) { y = y_; }
      void setz(const fmatvec::MatV &z_) { z = z_; }
      void setxyz(const fmatvec::MatV &xyz) {
        if(xyz.rows() <= 1 or xyz.cols() <= 1)
          THROW_MBSIMERROR("Dimension missmatch in size of xyz");
        x = xyz.row(0)(fmatvec::RangeV(1,xyz.cols()-1)).T();
        y = xyz.col(0)(fmatvec::RangeV(1,xyz.rows()-1));
        z = xyz(fmatvec::RangeV(1,xyz.rows()-1),fmatvec::RangeV(1,xyz.cols()-1));
      }

      double getxMin() { return x(0); }
      double getxMax() { return x(x.size() - 1); }
      double getyMin() { return y(0); }
      double getyMax() { return y(y.size() - 1); }
      /***************************************************/

      void init(Element::InitStage stage, const InitConfigSet &config) override {
        Function<Ret(Arg1, Arg2)>::init(stage, config);
        if(stage==Element::preInit) {
          if (z.cols() != x.size())
            THROW_MBSIMERROR("Dimension missmatch in size of x");
          if (z.rows() != y.size())
            THROW_MBSIMERROR("Dimension missmatch in size of y");
          for (int i = 1; i < x.size(); i++)
            if (x(i - 1) >= x(i))
              THROW_MBSIMERROR("x values must be strictly monotonic increasing!");
          for (int i = 1; i < y.size(); i++)
            if (y(i - 1) >= y(i))
              THROW_MBSIMERROR("y values must be strictly monotonic increasing!");
        }
      }
    protected:
      fmatvec::VecV x;
      fmatvec::VecV y;
      fmatvec::MatV z;

      int x0Index{0}, x1Index{0};
      int y0Index{0}, y1Index{0};

      fmatvec::VecV zVal;
      fmatvec::VecV zInd;
      fmatvec::MatV zFac;

      void calcIndex(double x, const fmatvec::VecV &X, int xSize, int &xIndexMinus, int &xIndexPlus) {
        if (x <= X(0)) {
          xIndexPlus = 1;
          xIndexMinus = 0;
          fmatvec::Atom::msg(fmatvec::Atom::Warn) << "TwoDimensionalTabularFunction: Value (" << x << ") is smaller than the smallest table value(" << X(0) << ")!" << std::endl;
        }
        else if (x >= X(xSize - 1)) {
          xIndexPlus = xSize - 1;
          xIndexMinus = xSize - 2;
          fmatvec::Atom::msg(fmatvec::Atom::Warn) << "TwoDimensionalTabularFunction: Value (" << x << ") is greater than the greatest table value(" << X(xSize - 1) << ")!" << std::endl;
        }
        else {
          if (x < X(xIndexPlus))
            while (x < X(xIndexPlus - 1) && xIndexPlus > 1)
              xIndexPlus--;
          else if (x > X(xIndexPlus))
            while (x > X(xIndexPlus) && xIndexPlus < xSize - 1)
              xIndexPlus++;
          xIndexMinus = xIndexPlus - 1;
        }
      }
  };
}

#endif
