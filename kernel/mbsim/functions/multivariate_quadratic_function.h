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

#ifndef _MULTIVARIATE_QUADRATIC_FUNCTION_H_
#define _MULTIVARIATE_QUADRATIC_FUNCTION_H_

#include "multivariate_linear_function.h"
#include "mbsim/utils/utils.h"

namespace MBSim {

  template<typename Sig> class MultivariateQuadraticFunction; 

  template<typename Ret, typename Arg>
  class MultivariateQuadraticFunction<Ret(Arg)> : public MultivariateLinearFunction<Ret(Arg)> {
    using B = MultivariateLinearFunction<Ret(Arg)>; 
    private:
      fmatvec::SqrMatV a2;
      fmatvec::SqrMatV a2pa2T;
    public:
      MultivariateQuadraticFunction() = default;
      void seta2(const fmatvec::SqrMatV &a2_) {
        a2 <<= a2_;
        a2pa2T <<= a2 + a2.T();
      }
      Ret operator()(const Arg &x) override {
        return FromDouble<Ret>::cast(B::a0+B::a1*x+x.T()*a2*x);
      }
      typename B::DRetDArg parDer(const Arg &x) override {
        return B::a1+x.T()*a2pa2T;
      }
      typename B::DRetDArg parDerDirDer(const Arg &xDir, const Arg &x) override {
        return xDir.T()*a2pa2T;
      }
      void initializeUsingXML(xercesc::DOMElement *element) override {
        MultivariateLinearFunction<Ret(Arg)>::initializeUsingXML(element);
        auto e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"a2");
        seta2(MBXMLUtils::E(e)->getText<fmatvec::SqrMatV>());
      }
  };

}

#endif
