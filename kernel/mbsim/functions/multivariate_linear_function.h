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

#ifndef _MULTIVARIATE_LINEAR_FUNCTION_H_
#define _MULTIVARIATE_LINEAR_FUNCTION_H_

#include "mbsim/functions/function.h"
#include "mbsim/utils/utils.h"

namespace MBSim {

  template<typename Sig> class MultivariateLinearFunction; 

  template<typename Ret, typename Arg>
  class MultivariateLinearFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    using B = fmatvec::Function<Ret(Arg)>; 
    private:
      double a0{0};
      fmatvec::RowVecV a1;
      fmatvec::RowVecV zero;
    public:
      MultivariateLinearFunction() = default;
      MultivariateLinearFunction(const fmatvec::RowVecV &a1_) { seta1(a1_); }
      MultivariateLinearFunction(double a0_, const fmatvec::RowVecV &a1_) : a0(a0_) { seta1(a1_); }
      MultivariateLinearFunction(const fmatvec::VecV &a1_) { seta1(a1_); }
      MultivariateLinearFunction(double a0_, const fmatvec::VecV &a1_) : a0(a0_) { seta1(a1_); }
      void seta0(double a0_) { a0 = a0_; }
      void seta1(const fmatvec::RowVecV &a1_) {
        a1 <<= a1_;
        zero.resize(a1.size(), fmatvec::INIT, 0.0);
      }
      void seta1(const fmatvec::VecV &a1_) { seta1(a1_.T()); }
      std::pair<int, int> getRetSize() const override { return std::make_pair(1,1); }
      Ret operator()(const Arg &x) override { return FromDouble<Ret>::cast(a1*x+a0); }
      typename B::DRetDArg parDer(const Arg &x) override { return a1; }
      typename B::DRetDArg parDerDirDer(const Arg &xDir, const Arg &x) override { return zero; }
      void initializeUsingXML(xercesc::DOMElement *element) override {
        xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"a0");
        if(e) seta0(MBXMLUtils::E(e)->getText<double>());
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"a1");
        seta1(MBXMLUtils::E(e)->getText<fmatvec::VecV>());
      }
  };

}

#endif
