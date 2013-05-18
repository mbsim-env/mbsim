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

#ifndef SYMBOLIC_FUNCTION_H_
#define SYMBOLIC_FUNCTION_H_

#include <mbsim/utils/function.h>
#include <casadi/symbolic/fx/sx_function.hpp>
#include "casadi/symbolic/matrix/matrix_tools.hpp"
#include "mbxmlutilstinyxml/casadiXML.h"

namespace MBSim {

  template <class Arg>
  class ToCasadi {
  };

  template <>
  class ToCasadi<double> {
    public:
      static double cast(const double &x) {
        return x;
      }
  };

  template <class Col>
  class ToCasadi<fmatvec::Vector<Col,double> > {
    public:
      static std::vector<double> cast(const fmatvec::Vector<Col,double> &x) {
        std::vector<double> y(x.size());
        for(int i=0; i<x.size(); i++)
          y[i] = x.e(i);
        return y; 
      }
  };

  template <class Ret>
  class FromCasadi {
  };

  template <class Col>
  class FromCasadi<fmatvec::Vector<Col,double> > {
    public:
      static fmatvec::Vector<Col,double> cast(const CasADi::Matrix<double> &x) {
        fmatvec::Vector<Col,double> y(x.size1(),fmatvec::NONINIT);
        for(int i=0; i<x.size1(); i++)
          y.e(i) = x(i,0).toScalar();
        return y;
      }
  };

  template <class Row>
  class FromCasadi<fmatvec::Matrix<fmatvec::General,Row,fmatvec::Var,double> > {
    public:
      static fmatvec::Matrix<fmatvec::General,Row,fmatvec::Var,double> cast(const CasADi::Matrix<double> &A) {
        fmatvec::Matrix<fmatvec::General,Row,fmatvec::Var,double> B(A.size1(),A.size2(),fmatvec::NONINIT);
        for(int i=0; i<A.size1(); i++)
          for(int j=0; j<A.size2(); j++)
            B.e(i,j) = A(i,j).toScalar();
        return B;
      }
  };

  template <>
  class FromCasadi<double> {
    public:
      static double cast(const CasADi::Matrix<double> &x) {
        return x.toScalar();
      }
  };

  template <class Ret, class Arg>
  class SymbolicFunction1 : public Function1<Ret,Arg> {
    CasADi::SXFunction f;
    public:
    SymbolicFunction1() {}
    SymbolicFunction1(const CasADi::SXFunction &f_) : f(f_) {
      f.init();
    }
    SymbolicFunction1(const CasADi::FX &f_) : f(CasADi::SXFunction(f_)) {
      f.init();
    }
    CasADi::SXFunction& getSXFunction() {return f;} 

    std::string getType() const { return "SymbolicFunction1"; }

    Ret operator()(const Arg& x, const void * =NULL) {
      f.setInput(ToCasadi<Arg>::cast(x));
      f.evaluate();
      return FromCasadi<Ret>::cast(f.output());
    }

    void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
      f=CasADi::createCasADiSXFunctionFromXML(element->FirstChildElement());
      f.init();
      assert(f.getNumInputs()==1);
      assert(f.getNumOutputs()==1);
    }
  };

  template <class Ret, class Arg1, class Arg2>
  class SymbolicFunction2 : public Function2<Ret,Arg1,Arg2> {
    CasADi::SXFunction f;
    public:
    SymbolicFunction2() {}
    SymbolicFunction2(const CasADi::SXFunction &f_) : f(f_) {
      f.init();
    }
    SymbolicFunction2(const CasADi::FX &f_) : f(CasADi::SXFunction(f_)) {
      f.init();
    }
    CasADi::SXFunction& getSXFunction() {return f;} 

    std::string getType() const { return "SymbolicFunction2"; }

    Ret operator()(const Arg1& x1, const Arg2& x2, const void * =NULL) {
      f.setInput(ToCasadi<Arg1>::cast(x1),0);
      f.setInput(ToCasadi<Arg2>::cast(x2),1);
      f.evaluate();
      return FromCasadi<Ret>::cast(f.output());
    }

    void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {
      f=CasADi::createCasADiSXFunctionFromXML(element->FirstChildElement());
      f.init();
      assert(f.getNumInputs()==2);
      assert(f.getNumOutputs()==1);
    }
  };

  template <class Ret, class Arg1, class Arg2, class Arg3>
  class SymbolicFunction3 : public Function3<Ret,Arg1,Arg2,Arg3> {
    CasADi::SXFunction f;
    public:
    SymbolicFunction3(const CasADi::SXFunction &f_) : f(f_) {
      f.init();
    }
    SymbolicFunction3(const CasADi::FX &f_) : f(CasADi::SXFunction(f_)) {
      f.init();
    }
    CasADi::SXFunction& getSXFunction() {return f;} 

    std::string getType() const { return "SymbolicFunction3"; }

    Ret operator()(const Arg1& x1, const Arg2& x2, const Arg3& x3, const void * =NULL) {
      f.setInput(ToCasadi<Arg1>::cast(x1),0);
      f.setInput(ToCasadi<Arg2>::cast(x2),1);
      f.setInput(ToCasadi<Arg3>::cast(x3),2);
      f.evaluate();
      return FromCasadi<Ret>::cast(f.output());
    }
  };

  // The following classes may be faster when calculating the function value
  // together with the first and second derivative
 
//  template <class Ret>
//  class SymbolicFunctionDerivatives1 : public Function1<Ret,double> {
//    private:
//      CasADi::SXFunction f;
//      CasADi::FX fder1, fder2;
//    public:
//    SymbolicFunctionDerivatives1(const CasADi::SXFunction &f_) : f(f_) {
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
//      return FromCasadi<Ret>::cast(fder2.output(2));
//    }
//  };
//
//  template <class Ret>
//  class SymbolicFXFunction : public Function1<Ret,double> {
//    private:
//    CasADi::FX &f;
//    int index;
//    public:
//    SymbolicFXFunction(CasADi::FX &f_, int index_=0) : f(f_), index(index_) {}
//
//    Ret operator()(const double& x_, const void * =NULL) {
//      return FromCasadi<Ret>::cast(f.output(2-index));
//    }
//  };

}

#endif
