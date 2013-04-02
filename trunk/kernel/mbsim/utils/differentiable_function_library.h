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

#ifndef DIFFERENTIABLE_FUNCTION_LIBRARY_H_
#define DIFFERENTIABLE_FUNCTION_LIBRARY_H_

#ifdef HAVE_SYMBOLIC_SX_SX_HPP

#include <mbsim/utils/function.h>
#include <symbolic/fx/sx_function.hpp>

namespace MBSim {

  template <class Ret>
  class Casadi {
  };

  template <class Col>
  class Casadi<fmatvec::Vector<Col,double> > {
    public:
      static fmatvec::Vector<Col,double> cast(const CasADi::Matrix<double> &x) {
        fmatvec::Vector<Col,double> y(x.size1(),fmatvec::NONINIT);
        for(int i=0; i<x.size1(); i++)
          y.e(i) = x(i,0).toScalar();
        return y;
      }
  };

  template <>
  class Casadi<double> {
    public:
      static double cast(const CasADi::Matrix<double> &x) {
        return x.toScalar();
      }
  };

  template <class Ret>
  class CasadiFunction1 : public Function1<Ret,double> {
    CasADi::SXFunction f;
    public:
    CasadiFunction1(const CasADi::SXFunction &f_) : f(f_) {
      f.init();
    }
    CasadiFunction1(const CasADi::FX &f_) : f(CasADi::SXFunction(f_)) {
      f.init();
    }
    CasADi::SXFunction& getSXFunction() {return f;} 

    std::string getType() const { return "CasadiFunction1"; }

    Ret operator()(const double& x_, const void * =NULL) {
      f.setInput(x_);
      f.evaluate();
      return Casadi<Ret>::cast(f.output());
    }
  };

  // The following classes may be faster when calculating the function value
  // together with the first and second derivative
 
//  template <class Ret>
//  class CasadiFunctionDerivatives1 : public Function1<Ret,double> {
//    private:
//      CasADi::SXFunction f;
//      CasADi::FX fder1, fder2;
//    public:
//    CasadiFunctionDerivatives1(const CasADi::SXFunction &f_) : f(f_) {
//      f.init();
//      fder1 = f.jacobian();
//      fder1.init();
//      fder2 = fder1.jacobian();
//      fder2.init();
//    }
//    std::string getType() const { return "CasadiDiffFunction"; }
//
//    CasADi::FX& getFXFunction() {return fder2;} 
//
//    Ret operator()(const double& x_, const void * =NULL) {
//      fder2.setInput(x_);
//      fder2.evaluate();
//      return Casadi<Ret>::cast(fder2.output(2));
//    }
//  };
//
//  template <class Ret>
//  class CasadiFXFunction : public Function1<Ret,double> {
//    private:
//    CasADi::FX &f;
//    int index;
//    public:
//    CasadiFXFunction(CasADi::FX &f_, int index_=0) : f(f_), index(index_) {}
//
//    Ret operator()(const double& x_, const void * =NULL) {
//      return Casadi<Ret>::cast(f.output(2-index));
//    }
//  };

}

#endif

#endif
