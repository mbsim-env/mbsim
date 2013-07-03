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
#include <fmatvec/function.h>
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

template<typename Sig>
class SymbolicFunction;

template<typename Ret, typename Arg>
  class SymbolicFunction<Ret(Arg)> : public fmatvec::Function<Ret(Arg)> {
    CasADi::SXFunction f;
    public:
    SymbolicFunction() {}
    SymbolicFunction(const CasADi::SXFunction &f_) : f(f_) {
      f.init();
    }
    SymbolicFunction(const CasADi::FX &f_) : f(CasADi::SXFunction(f_)) {
      f.init();
    }
    CasADi::SXFunction& getSXFunction() {return f;} 

    typename fmatvec::Size<Arg>::type getArg1Size() const {
      f.inputExpr(0).size();
    }

    std::string getType() const { return "SymbolicFunction1"; }

    Ret operator()(const Arg& x) {
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

template<typename Ret, typename Arg1, typename Arg2>
  class SymbolicFunction<Ret(Arg1, Arg2)> : public fmatvec::Function<Ret(Arg1, Arg2)> {
    CasADi::SXFunction f, dfdx1, dfdx2, ddfx1dx1, ddfx1dx2, ddfx2dx1, ddfx2dx2;
    public:
    SymbolicFunction() {}
    SymbolicFunction(const CasADi::SXFunction &f_) : f(f_) {
      f.init();
      dfdx1 = CasADi::SXFunction(f.inputExpr(),f.jac(0));
      dfdx1.init();
      dfdx2 = CasADi::SXFunction(f.inputExpr(),f.jac(1));
      dfdx2.init();
      int nq = getArg1Size();
      std::vector<CasADi::SX> sqd(nq);
      for(int i=0; i<nq; i++) {
        std::stringstream stream;
        stream << "qd" << i;
        sqd[i] = CasADi::SX(stream.str());
      }
      std::vector<CasADi::SXMatrix> input2(3);
      input2[0] = sqd;
      input2[1] = f.inputExpr(0);
      input2[2] = f.inputExpr(1);
      CasADi::SXMatrix Jd1(3,nq);
      CasADi::SXMatrix Jd2(3,nq);
      for(int j=0; j<nq; j++) {
        Jd1(CasADi::Slice(0,3),CasADi::Slice(j,j+1)) = dfdx1.jac(0)(CasADi::Slice(j,nq*3,nq),CasADi::Slice(0,nq)).mul(sqd);
        Jd2(CasADi::Slice(0,3),CasADi::Slice(j,j+1)) = dfdx1.jac(1)(CasADi::Slice(j,nq*3,nq),CasADi::Slice(0,1));
      }
      ddfx1dx1 = CasADi::SXFunction(input2,Jd1);
      ddfx1dx1.init();
      ddfx1dx2 = CasADi::SXFunction(f.inputExpr(),Jd2);
      ddfx1dx2.init();

      CasADi::SXMatrix djT1 = dfdx2.jac(0).mul(sqd);
      CasADi::SXMatrix djT2 = dfdx2.jac(1);
      ddfx2dx1 = CasADi::SXFunction(input2,djT1);
      ddfx2dx1.init();
      ddfx2dx2 = CasADi::SXFunction(f.inputExpr(),djT2);
      ddfx2dx2.init();
    }
//    SymbolicFunction(const CasADi::FX &f_) : f(CasADi::SXFunction(f_)) {
//      f.init();
//      dfdx1 = CasADi::SXFunction(f.inputExpr(),f.jac(0));
//      dfdx1.init();
//      dfdx2 = CasADi::SXFunction(f.inputExpr(),f.jac(1));
//      dfdx2.init();
//    }
    CasADi::SXFunction& getSXFunction() {return f;} 

    typename fmatvec::Size<Arg1>::type getArg1Size() const {
      f.inputExpr(0).size();
    }

    typename fmatvec::Size<Arg2>::type getArg2Size() const {
      f.inputExpr(1).size();
    }

    std::string getType() const { return "SymbolicFunction2"; }

    Ret operator()(const Arg1& x1, const Arg2& x2) {
      f.setInput(ToCasadi<Arg1>::cast(x1),0);
      f.setInput(ToCasadi<Arg2>::cast(x2),1);
      f.evaluate();
      return FromCasadi<Ret>::cast(f.output());
    }

    typename fmatvec::Der<Ret, Arg1>::type parDer1(const Arg1 &x1, const Arg2 &x2) {
      dfdx1.setInput(ToCasadi<Arg1>::cast(x1),0);
      dfdx1.setInput(ToCasadi<Arg2>::cast(x2),1);
      dfdx1.evaluate();
      return FromCasadi<typename fmatvec::Der<Ret, Arg1>::type>::cast(dfdx1.output());
    }

    typename fmatvec::Der<Ret, Arg2>::type parDer2(const Arg1 &x1, const Arg2 &x2) {
      dfdx2.setInput(ToCasadi<Arg1>::cast(x1),0);
      dfdx2.setInput(ToCasadi<Arg2>::cast(x2),1);
      dfdx2.evaluate();
      return FromCasadi<typename fmatvec::Der<Ret, Arg2>::type>::cast(dfdx2.output());
    }

    typename fmatvec::Der<Ret, Arg1>::type parDer1DirDer1(const Arg1 &xd1, const Arg1 &x1, const Arg2 &x2) {
      ddfx1dx1.setInput(ToCasadi<Arg1>::cast(xd1),0);
      ddfx1dx1.setInput(ToCasadi<Arg1>::cast(x1),1);
      ddfx1dx1.setInput(ToCasadi<Arg2>::cast(x2),2);
      ddfx1dx1.evaluate();
      return FromCasadi<typename fmatvec::Der<Ret, Arg1>::type>::cast(ddfx1dx1.output());
    }

    typename fmatvec::Der<typename fmatvec::Der<Ret, Arg1>::type, Arg2>::type parDer1ParDer2(const Arg1 &x1, const Arg2 &x2) {
      ddfx1dx2.setInput(ToCasadi<Arg1>::cast(x1),0);
      ddfx1dx2.setInput(ToCasadi<Arg2>::cast(x2),1);
      ddfx1dx2.evaluate();
      return FromCasadi<typename fmatvec::Der<typename fmatvec::Der<Ret, Arg1>::type, Arg2>::type>::cast(ddfx1dx2.output());
    }

    typename fmatvec::Der<Ret, Arg2>::type parDer2DirDer1(const Arg1 &xd1, const Arg1 &x1, const Arg2 &x2) {
      ddfx2dx1.setInput(ToCasadi<Arg1>::cast(xd1),0);
      ddfx2dx1.setInput(ToCasadi<Arg1>::cast(x1),1);
      ddfx2dx1.setInput(ToCasadi<Arg2>::cast(x2),2);
      ddfx2dx1.evaluate();
      return FromCasadi<typename fmatvec::Der<Ret, Arg2>::type>::cast(ddfx2dx1.output());
    }

    typename fmatvec::Der<typename fmatvec::Der<Ret, Arg2>::type, Arg2>::type parDer2ParDer2(const Arg1 &x1, const Arg2 &x2) {
      ddfx2dx2.setInput(ToCasadi<Arg1>::cast(x1),0);
      ddfx2dx2.setInput(ToCasadi<Arg2>::cast(x2),1);
      ddfx2dx2.evaluate();
      return FromCasadi<typename fmatvec::Der<typename fmatvec::Der<Ret, Arg2>::type, Arg2>::type>::cast(ddfx2dx2.output());
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
