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

#ifndef _TABULAR_FUNCTION_H_
#define _TABULAR_FUNCTION_H_

#include "mbsim/functions/function.h"
#include "mbsim/utils/utils.h"

namespace MBSim {

  template<typename Sig> class TabularFunction; 

  template<typename Ret, typename Arg>
  class TabularFunction<Ret(Arg)> : public Function<Ret(Arg)> {

    public:
      TabularFunction() : xIndexOld(0) { }
      TabularFunction(const fmatvec::VecV &x_, const fmatvec::MatV &y_) : x(x_), y(y_), xIndexOld(0) { }
      int getArgSize() const { return 1; }
      std::pair<int, int> getRetSize() const { return std::make_pair(y.cols(),1); }
      Ret operator()(const Arg& xVal_) {
        double xVal = ToDouble<Arg>::cast(xVal_);
        int i = xIndexOld;
        if (xVal <= x(0)) {
          xIndexOld = 0;
          return FromVecV<Ret>::cast(trans(y.row(0)));
        }
        else if (xVal >= x(x.size() - 1)) {
          xIndexOld = x.size() - 1;
          return FromVecV<Ret>::cast(trans(y.row(x.size() - 1)));
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
          setx(Element::getVec(e));
          e = MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"y");
          sety(Element::getMat(e, x.size(), 0));
        }
        e = MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"xy");
        if (e) setxy(Element::getMat(e));
      }
      void setx(const fmatvec::VecV &x_) { x = x_; }
      void sety(const fmatvec::MatV &y_) { y = y_; }
      void setxy(const fmatvec::MatV &xy) {
        if(xy.cols() <= 1)
          THROW_MBSIMERROR("Dimension missmatch in size of xy");
        x = xy.col(0);
        y = xy(fmatvec::RangeV(0, xy.rows() - 1), fmatvec::RangeV(1, xy.cols() - 1));
      }
      void init(Element::InitStage stage) {
        Function<Ret(Arg)>::init(stage);
        if(stage==Element::preInit) {
          for(int i=1; i<x.size(); i++)
            if(x(i) <= x(i-1))
              THROW_MBSIMERROR("Values of x must be strictly monotonic increasing!");
          if(y.rows() != x.size())
            THROW_MBSIMERROR("Dimension missmatch in size of x");
        }
      }
    protected:
      fmatvec::VecV x;
      fmatvec::MatV y;
    private:
      int xIndexOld;
  };
}

#endif
