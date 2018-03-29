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

#ifndef _QUADRATIC_FUNCTION_H_
#define _QUADRATIC_FUNCTION_H_

#include "mbsim/functions/function.h"
#include "mbsim/utils/utils.h"

namespace MBSim {

  template<typename Sig> class QuadraticFunction; 

  template<typename Ret, typename Arg>
  class QuadraticFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    using B = fmatvec::Function<Ret(Arg)>; 
    private:
      double a0{0}, a1{0}, a2;
    public:
      QuadraticFunction(double a2_=0) :  a2(a2_) { }
      QuadraticFunction(double a1_, double a2_) : a0(0), a1(a1_), a2(a2_) { }
      QuadraticFunction(double a0_, double a1_, double a2_) : a0(a0_), a1(a1_), a2(a2_) { }
      int getArgSize() const override { return 1; }
      std::pair<int, int> getRetSize() const override { return std::make_pair(1,1); }
      Ret operator()(const Arg &x_) override {
        double x = ToDouble<Arg>::cast(x_);
        return FromDouble<Ret>::cast(a0+(a1+a2*x)*x);
      }
      typename B::DRetDArg parDer(const Arg &x_) override {
        double x = ToDouble<Arg>::cast(x_);
        return FromDouble<typename B::DRetDArg>::cast(a1+2.*a2*x);
      }
      typename B::DRetDArg parDerDirDer(const Arg &xDir_, const Arg &x) override {
        double xDir = ToDouble<Arg>::cast(xDir_);
        return FromDouble<typename B::DRetDArg>::cast(2.*a2*xDir);
      }
      typename B::DDRetDDArg parDerParDer(const Arg &x) override {
        return FromDouble<typename B::DDRetDDArg>::cast(2.*a2);
      }
      void initializeUsingXML(xercesc::DOMElement *element) override {
        xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"a0");
        if(e) a0=MBXMLUtils::E(e)->getText<double>();
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"a1");
        if(e) a1=MBXMLUtils::E(e)->getText<double>();
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"a2");
        a2=MBXMLUtils::E(e)->getText<double>();
      }
  };

}

#endif
