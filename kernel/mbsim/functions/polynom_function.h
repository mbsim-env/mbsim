/* Copyright (C) 2004-2014 MBSim Development Team
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

#ifndef _POLYNOM_FUNCTION_H_
#define _POLYNOM_FUNCTION_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<typename Sig> class PolynomFunction; 

  template<typename Ret, typename Arg>
  class PolynomFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    using B = fmatvec::Function<Ret(Arg)>; 
    public:
      PolynomFunction() = default;
      PolynomFunction(const fmatvec::VecV &a_) : a(a_) { }
      void init(Element::InitStage stage, const InitConfigSet &config) override {
        Function<Ret(Arg)>::init(stage, config);
        if(stage == Element::preInit) {
          ad.resize(a.size()-1);
          add.resize(ad.size()-1);
          for(int i=1; i<a.size(); i++)
            ad.e(i-1) = double(i)*a.e(i);
          for(int i=1; i<ad.size(); i++)
            add.e(i-1) = double(i)*ad(i);
        }
      }
      int getArgSize() const override { return 1; }
      std::pair<int, int> getRetSize() const override { return std::make_pair(1,1); }
      Ret operator()(const Arg &x_) override {
        double x = ToDouble<Arg>::cast(x_);
        double value=a(a.size()-1);
        for (int i=int(a.size())-2; i>-1; i--)
          value=value*x+a.e(i);
        return FromDouble<Ret>::cast(value);
      }
      typename B::DRetDArg parDer(const Arg &x_) override {
        double x = ToDouble<Arg>::cast(x_);
        double value=ad(ad.size()-1);
        for (int i=int(ad.size())-2; i>-1; i--)
          value=value*x+ad.e(i);
        return FromDouble<typename B::DRetDArg>::cast(value);
      }
      typename B::DRetDArg parDerDirDer(const Arg &xDir_, const Arg &x_) override {
        double x = ToDouble<Arg>::cast(x_);
        double xDir = ToDouble<Arg>::cast(xDir_);
        double value=add(add.size()-1);
        for (int i=int(add.size())-2; i>-1; i--)
          value=value*x+add.e(i);
        return FromDouble<typename B::DRetDArg>::cast(value*xDir);
      }
      typename B::DDRetDDArg parDerParDer(const Arg &x_) override {
        double x = ToDouble<Arg>::cast(x_);
        double value=add(add.size()-1);
        for (int i=int(add.size())-2; i>-1; i--)
          value=value*x+add.e(i);
        return FromDouble<typename B::DDRetDDArg>::cast(value);
      }

      void initializeUsingXML(xercesc::DOMElement *element) override {
        a = MBXMLUtils::E(MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"coefficients"))->getText<fmatvec::Vec>();
      }

    private:
      fmatvec::VecV a, ad, add;
  };

}

#endif
