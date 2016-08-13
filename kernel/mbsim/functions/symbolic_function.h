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

#include <mbsim/functions/function.h>
#include <mbxmlutilshelper/casadiXML.h>
#include "mbsim/mbsim_event.h"
#include <casadi/core/function/function.hpp>

namespace MBSim {

  //! A fmatvec like jacobian using casadi
  casadi::SX jac(const casadi::SX &f, const casadi::SX &a) {
    // matrix or row vector with respect to scalar does not change the shape
    if(f.size2()>1 && a.size1()==1 && a.size2()==1)
    {
      casadi::SX fr=casadi::SX::reshape(f, f.size1()*f.size2(), 1); // reshape to column vector
      casadi::SX frj=casadi::SX::jacobian(fr, a); // calculate jacobian
      return casadi::SX::reshape(frj, f.size1(), f.size2()); // reshape back to original shape
    }
    // column vector or scalar with respect to column vector or scalar addes a second dimension
    else
      return casadi::SX::jacobian(f, a);
  }

  //! convert double to casadi::DM
  casadi::DM c(const double &x) {
    return casadi::DM(x);
  }

  //! convert fmatvec::Vector to casadi::DM
  template<class Col>
  casadi::DM c(const fmatvec::Vector<Col,double> &x) {
    casadi::DM y(x.size(), 1);
    for(int i=0; i<x.size(); i++)
      y(i) = x.e(i);
    return y; 
  }

  // helper classes to convert casadi::DM to any value
  template <class Ret>
  class FromCasadi {
    public:
      static Ret cast(const casadi::Matrix<double> &x) {
        throw MBSimError("FromCasadi::cast not implemented for current type.");
      }
  };

  template <class Col>
  class FromCasadi<fmatvec::Vector<Col,double> > {
    public:
      static fmatvec::Vector<Col,double> cast(const casadi::Matrix<double> &x) {
        fmatvec::Vector<Col,double> y(x.size1(),fmatvec::NONINIT);
        for(int i=0; i<x.size1(); i++)
          y.e(i) = x(i,0).scalar();
        return y;
      }
  };

  template <class Col>
  class FromCasadi<fmatvec::RowVector<Col,double> > {
    public:
      static fmatvec::RowVector<Col,double> cast(const casadi::Matrix<double> &x) {
        fmatvec::RowVector<Col,double> y(x.size2(),fmatvec::NONINIT);
        for(int i=0; i<x.size2(); i++)
          y.e(i) = x(0,i).scalar();
        return y;
      }
  };

  template <class Row, class Col>
  class FromCasadi<fmatvec::Matrix<fmatvec::General,Row,Col,double> > {
    public:
      static fmatvec::Matrix<fmatvec::General,Row,Col,double> cast(const casadi::Matrix<double> &A) {
        fmatvec::Matrix<fmatvec::General,Row,Col,double> B(A.size1(),A.size2(),fmatvec::NONINIT);
        for(int i=0; i<A.size1(); i++)
          for(int j=0; j<A.size2(); j++)
            B.e(i,j) = A(i,j).scalar();
        return B;
      }
  };

  template <>
  class FromCasadi<double> {
    public:
      static double cast(const casadi::Matrix<double> &x) {
        return x.scalar();
      }
  };

  //! convert casadi::DM any value
  template<class T>
  T c(const casadi::DM &x) {
    return FromCasadi<T>::cast(x);
  }

  template<typename Sig> class SymbolicFunction;

  template<typename Ret, typename Arg>
  class SymbolicFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    using B = fmatvec::Function<Ret(Arg)>; 
    casadi::SX ret, arg;
    mutable casadi::SX pd_;
    casadi::Function f, pd, dd, pddd, pdpd;
    public:
    SymbolicFunction() {}
    SymbolicFunction(const casadi::SX &ret_, const casadi::SX &arg_) : ret(ret_), arg(arg_) {
      checkFunctionIODim();
    }

    void init(Element::InitStage stage) {
      Function<Ret(Arg)>::init(stage);
      if(stage == Element::preInit) {
        casadi::SX argd=casadi::SX::sym("argd", getArgSize());

                   pd_ = jac(ret, arg);
        casadi::SX dd_ = jtimes(ret, arg, argd);
        casadi::SX pddd_ = jac(dd_, arg);
        casadi::SX pdpd_ = jac(pd_, arg);

        f = casadi::Function("noname", {arg}, {ret});
        pd = casadi::Function("noname", {arg}, {pd_});
        dd = casadi::Function("noname", {argd, arg}, {dd_});
        pddd = casadi::Function("noname", {argd, arg}, {pddd_});
        pdpd = casadi::Function("noname", {arg}, {pdpd_});
      }
    }

    std::pair<int, int> getRetSize() const override {
      return std::make_pair(this->ret.size1(), this->ret.size2());
    }

    int getArgSize() const override {
      return arg.size1();
    }

    bool constParDer() const override {
      if(pd_.is_empty(true))
        pd_ = jac(ret, arg);
      return pd_.is_constant();
    }

    Ret operator()(const Arg& x) override {
      return c<Ret>(f(std::vector<casadi::SX>{c(x)})[0]);
    }

    typename B::DRetDArg parDer(const Arg &x) override {
      return c<typename B::DRetDArg>(pd(std::vector<casadi::SX>{c(x)})[0]);
    }

    Ret dirDer(const Arg &xd, const Arg &x) override {
      return c<Ret>(dd(std::vector<casadi::SX>{c(xd), c(x)})[0]);
    }

    typename B::DRetDArg parDerDirDer(const Arg &xd, const Arg &x) override {
      return c<typename B::DRetDArg>(pddd(std::vector<casadi::SX>{c(xd), c(x)})[0]);
    }

    typename B::DDRetDDArg parDerParDer(const Arg &x) override {
      return c<typename B::DDRetDDArg>(pdpd(std::vector<casadi::SX>{c(x)})[0]);
    }

    Ret dirDerDirDer(const Arg &argDir_1, const Arg &argDir_2, const Arg &arg) override {
      throw std::runtime_error("mfmf");
    }

    void initializeUsingXML(xercesc::DOMElement *element) override {
      auto io=casadi::createCasADiFunctionFromXML(element->getFirstElementChild());
      arg=io.first[0];
      ret=io.second[0];
      // check symbolic function arguments: we need to throw errors during initializeUsingXML to enable the ObjectFactory
      // to test other possible combinations (more general ones)
      checkFunctionIODim();
    }

    private:

    void checkFunctionIODim() {
      // check function: only scalar and vector arguments are supported
      if(arg.size2()!=1) THROW_MBSIMERROR("Matrix parameter are not allowed.");
      // check function <-> template argument dimension
      if(this->argSize!=0 && arg.size1()!=this->argSize)
        THROW_MBSIMERROR("The dimension of a parameter does not match.");
      if(this->retSize1!=0 && ret.size1()!=this->retSize1)
        THROW_MBSIMERROR("The output dimension does not match.");
      if(this->retSize2!=0 && ret.size2()!=this->retSize2)
        THROW_MBSIMERROR("The output dimension does not match.");
    }
  };

  template<typename Ret, typename Arg1, typename Arg2>
  class SymbolicFunction<Ret(Arg1, Arg2)> : public Function<Ret(Arg1, Arg2)> {
    using B = fmatvec::Function<Ret(Arg1, Arg2)>; 
    casadi::SX ret, arg1, arg2;
    mutable casadi::SX pd1_, pd2_;
    casadi::Function f, pd1, pd2, pd1dd1, pd1dd2, pd2dd1, pd2dd2;
    public:
    SymbolicFunction() {}
    SymbolicFunction(const casadi::SX &ret_, const casadi::SX &arg1_, const casadi::SX &arg2_) : ret(ret_), arg1(arg1_), arg2(arg2_) {
      checkFunctionIODim();
    }

    void init(Element::InitStage stage) {
      Function<Ret(Arg1, Arg2)>::init(stage);
      if(stage == Element::preInit) {
        casadi::SX arg1d=casadi::SX::sym("arg1d", getArg1Size());
        casadi::SX arg2d=casadi::SX::sym("arg2d", getArg2Size());

                   pd1_ = jac(ret, arg1);
                   pd2_ = jac(ret, arg2);
        casadi::SX dd1_ = jtimes(ret, arg1, arg1d);
        casadi::SX dd2_ = jtimes(ret, arg2, arg2d);
        casadi::SX pd1dd1_ = jac(dd1_, arg1);
        casadi::SX pd1dd2_ = jac(dd2_, arg1);
        casadi::SX pd2dd1_ = jac(dd1_, arg2);
        casadi::SX pd2dd2_ = jac(dd2_, arg2);

        f = casadi::Function("noname", {arg1, arg2}, {ret});
        pd1 = casadi::Function("noname", {arg1, arg2}, {pd1_});
        pd2 = casadi::Function("noname", {arg1, arg2}, {pd2_});
        pd1dd1 = casadi::Function("noname", {arg1d, arg1, arg2}, {pd1dd1_});
        pd1dd2 = casadi::Function("noname", {arg2d, arg1, arg2}, {pd1dd2_});
        pd2dd1 = casadi::Function("noname", {arg1d, arg1, arg2}, {pd2dd1_});
        pd2dd2 = casadi::Function("noname", {arg2d, arg1, arg2}, {pd2dd2_});
      }
    }

    std::pair<int, int> getRetSize() const override {
      return std::make_pair(ret.size1(), ret.size2());
    }

    int getArg1Size() const override {
      return arg1.size1();
    }

    int getArg2Size() const override {
      return arg2.size1();
    }

    bool constParDer1() const override {
      if(pd1_.is_empty(true))
        pd1_ = jac(ret, arg1);
      return pd1_.is_constant();
    }

    bool constParDer2() const override {
      if(pd2_.is_empty(true))
        pd2_ = jac(ret, arg2);
      return pd2_.is_constant();
    }

    Ret operator()(const Arg1& x1, const Arg2& x2) override {
      return c<Ret>(f(std::vector<casadi::SX>{c(x1), c(x2)})[0]);
    }

    typename B::DRetDArg1 parDer1(const Arg1 &x1, const Arg2 &x2) override {
      return c<typename B::DRetDArg1>(pd1(std::vector<casadi::SX>{c(x1), c(x2)})[0]);
    }

    Ret dirDer1(const Arg1 &arg1Dir, const Arg1 &arg1, const Arg2 &arg2) override {
      throw std::runtime_error("mfmf");
    }

    typename B::DRetDArg2 parDer2(const Arg1 &x1, const Arg2 &x2) override {
      return c<typename B::DRetDArg2>(pd2(std::vector<casadi::SX>{c(x1), c(x2)})[0]);
    }

    Ret dirDer2(const Arg2 &arg2Dir, const Arg1 &arg1, const Arg2 &arg2) override {
      throw std::runtime_error("mfmf");
    }

    typename B::DDRetDDArg1 parDer1ParDer1(const Arg1 &arg1, const Arg2 &arg2) override {
      throw std::runtime_error("mfmf");
    }

    typename B::DRetDArg1 parDer1DirDer1(const Arg1 &xd1, const Arg1 &x1, const Arg2 &x2) override {
      return c<typename B::DRetDArg1>(pd1dd1(std::vector<casadi::SX>{c(xd1), c(x1), c(x2)})[0]);
    }

    Ret dirDer1DirDer1(const Arg1 &arg1Dir_1, const Arg1 &arg1Dir_2, const Arg1 &arg1, const Arg2 &arg2) override {
      throw std::runtime_error("mfmf");
    }

    typename B::DDRetDDArg2 parDer2ParDer2(const Arg1 &arg1, const Arg2 &arg2) override {
      throw std::runtime_error("mfmf");
    }

    typename B::DRetDArg1 parDer1DirDer2(const Arg2 &xd2, const Arg1 &x1, const Arg2 &x2) override {
      return c<typename B::DRetDArg1>(pd1dd2(std::vector<casadi::SX>{c(xd2), c(x1), c(x2)})[0]);
    }

    typename B::DRetDArg2 parDer2DirDer1(const Arg1 &xd1, const Arg1 &x1, const Arg2 &x2) override {
      return c<typename B::DRetDArg2>(pd2dd1(std::vector<casadi::SX>{c(xd1), c(x1), c(x2)})[0]);
    }

    Ret dirDer2DirDer2(const Arg2 &arg2Dir_1, const Arg2 &arg2Dir_2, const Arg1 &arg1, const Arg2 &arg2) override {
      throw std::runtime_error("mfmf");
    }

    typename B::DDRetDArg1DArg2 parDer1ParDer2(const Arg1 &arg1, const Arg2 &arg2) override {
      throw std::runtime_error("mfmf");
    }

    Ret dirDer2DirDer1(const Arg2 &arg1Dir, const Arg1 &arg1, const Arg2 &arg2) override {
      throw std::runtime_error("mfmf");
    }

    typename B::DRetDArg2 parDer2DirDer2(const Arg2 &xd2, const Arg1 &x1, const Arg2 &x2) override {
      return c<typename B::DRetDArg2>(pd2dd2(std::vector<casadi::SX>{c(xd2), c(x1), c(x2)})[0]);
    }

    void initializeUsingXML(xercesc::DOMElement *element) {
      auto io=casadi::createCasADiFunctionFromXML(element->getFirstElementChild());
      arg1=io.first[0];
      arg2=io.first[1];
      ret=io.second[0];
      // check symbolic function arguments: we need to throw errors during initializeUsingXML to enable the ObjectFactory
      // to test other possible combinations (more general ones)
      checkFunctionIODim();
    }

    private:

    void checkFunctionIODim() {
      // check function: only scalar and vector arguments are supported
      if(arg1.size2()!=1) THROW_MBSIMERROR("Matrix parameter are not allowed.");
      if(arg2.size2()!=1) THROW_MBSIMERROR("Matrix parameter are not allowed.");
      // check function <-> template argument dimension
      if(this->arg1Size!=0 && arg1.size1()!=this->arg1Size)
        THROW_MBSIMERROR("The dimension of a parameter does not match.");
      if(this->arg2Size!=0 && arg2.size1()!=this->arg2Size)
        THROW_MBSIMERROR("The dimension of a parameter does not match.");
      if(this->retSize1!=0 && ret.size1()!=this->retSize1)
        THROW_MBSIMERROR("The output dimension does not match.");
      if(this->retSize2!=0 && ret.size2()!=this->retSize2)
        THROW_MBSIMERROR("The output dimension does not match.");
    }
  };

  template<typename Ret, typename Arg1>
  class SymbolicFunction<Ret(Arg1, double)> : public Function<Ret(Arg1, double)> {
    using B = fmatvec::Function<Ret(Arg1, double)>; 
    casadi::SX ret, arg1, arg2;
    mutable casadi::SX pd1_, pd2_;
    casadi::Function f, pd1, pd2, pd1dd1, pd1dd2, pd1pd2, pd2dd1, pd2dd2, pd2pd2;
    public:
    SymbolicFunction() {}
    SymbolicFunction(const casadi::SX &ret_, const casadi::SX &arg1_, const casadi::SX &arg2_) : ret(ret_), arg1(arg1_), arg2(arg2_) {
      checkFunctionIODim();
    }

    void init(Element::InitStage stage) {
      Function<Ret(Arg1, double)>::init(stage);
      if(stage == Element::preInit) {
        casadi::SX arg1d=casadi::SX::sym("arg1d", getArg1Size());
        casadi::SX arg2d=casadi::SX::sym("arg2d", getArg2Size());

                   pd1_ = jac(ret, arg1);
                   pd2_ = jac(ret, arg2);
        casadi::SX dd1_ = jtimes(ret, arg1, arg1d);
        casadi::SX dd2_ = jtimes(ret, arg2, arg2d);
        casadi::SX pd1dd1_ = jac(dd1_, arg1);
        casadi::SX pd1dd2_ = jac(dd2_, arg1);
        casadi::SX pd1pd2_ = jac(pd1_, arg2);
        casadi::SX pd2dd1_ = jac(dd1_, arg2);
        casadi::SX pd2dd2_ = jac(dd2_, arg2);
        casadi::SX pd2pd2_ = jac(pd2_, arg2);

        f = casadi::Function("noname", {arg1, arg2}, {ret});
        pd1 = casadi::Function("noname", {arg1, arg2}, {pd1_});
        pd2 = casadi::Function("noname", {arg1, arg2}, {pd2_});
        pd1dd1 = casadi::Function("noname", {arg1d, arg1, arg2}, {pd1dd1_});
        pd1dd2 = casadi::Function("noname", {arg2d, arg1, arg2}, {pd1dd2_});
        pd1pd2 = casadi::Function("noname", {arg1, arg2}, {pd1pd2_});
        pd2dd1 = casadi::Function("noname", {arg1d, arg1, arg2}, {pd2dd1_});
        pd2dd2 = casadi::Function("noname", {arg2d, arg1, arg2}, {pd2dd2_});
        pd2pd2 = casadi::Function("noname", {arg1, arg2}, {pd2pd2_});
      }
    }

    std::pair<int, int> getRetSize() const override {
      return std::make_pair(ret.size1(), ret.size2());
    }

    int getArg1Size() const override {
      return arg1.size1();
    }

    int getArg2Size() const override {
      return arg2.size1();
    }

    bool constParDer1() const override {
      if(pd1_.is_empty(true))
        pd1_ = jac(ret, arg1);
      return pd1_.is_constant();
    }

    bool constParDer2() const override {
      if(pd2_.is_empty(true))
        pd2_ = jac(ret, arg2);
      return pd2_.is_constant();
    }

    Ret operator()(const Arg1& x1, const double& x2) override {
      return c<Ret>(f(std::vector<casadi::SX>{c(x1), c(x2)})[0]);
    }

    typename B::DRetDArg1 parDer1(const Arg1 &x1, const double &x2) override {
      return c<typename B::DRetDArg1>(pd1(std::vector<casadi::SX>{c(x1), c(x2)})[0]);
    }

    typename B::DRetDArg2 parDer2(const Arg1 &x1, const double &x2) override {
      return c<typename B::DRetDArg2>(pd2(std::vector<casadi::SX>{c(x1), c(x2)})[0]);
    }

    typename B::DRetDArg1 parDer1DirDer1(const Arg1 &xd1, const Arg1 &x1, const double &x2) override {
      return c<typename B::DRetDArg1>(pd1dd1(std::vector<casadi::SX>{c(xd1), c(x1), c(x2)})[0]);
    }

    typename B::DRetDArg1 parDer1DirDer2(const double &xd2, const Arg1 &x1, const double &x2) override {
      return c<typename B::DRetDArg1>(pd1dd2(std::vector<casadi::SX>{c(xd2), c(x1), c(x2)})[0]);
    }

    typename B::DDRetDArg1DArg2 parDer1ParDer2(const Arg1 &x1, const double &x2) override {
      return c<typename B::DDRetDArg1DArg2>(pd1pd2(std::vector<casadi::SX>{c(x1), c(x2)})[0]);
    }

    typename B::DRetDArg2 parDer2DirDer1(const Arg1 &xd1, const Arg1 &x1, const double &x2) override {
      return c<typename B::DRetDArg2>(pd2dd1(std::vector<casadi::SX>{c(xd1), c(x1), c(x2)})[0]);
    }

    typename B::DRetDArg2 parDer2DirDer2(const double &xd2, const Arg1 &x1, const double &x2) override {
      return c<typename B::DRetDArg2>(pd2dd2(std::vector<casadi::SX>{c(xd2), c(x1), c(x2)})[0]);
    }

    typename B::DDRetDDArg2 parDer2ParDer2(const Arg1 &x1, const double &x2) override {
      return c<typename B::DDRetDDArg2>(pd2pd2(std::vector<casadi::SX>{c(x1), c(x2)})[0]);
    }

    void initializeUsingXML(xercesc::DOMElement *element) {
      auto io=casadi::createCasADiFunctionFromXML(element->getFirstElementChild());
      arg1=io.first[0];
      arg2=io.first[1];
      ret=io.second[0];
      // check symbolic function arguments: we need to throw errors during initializeUsingXML to enable the ObjectFactory
      // to test other possible combinations (more general ones)
      checkFunctionIODim();
    }

    private:

    void checkFunctionIODim() {
      // check function: only scalar and vector arguments are supported
      if(arg1.size2()!=1) THROW_MBSIMERROR("Matrix parameter are not allowed.");
      if(arg2.size2()!=1) THROW_MBSIMERROR("Matrix parameter are not allowed.");
      // check function <-> template argument dimension
      if(this->arg1Size!=0 && arg1.size1()!=this->arg1Size)
        THROW_MBSIMERROR("The dimension of a parameter does not match.");
      if(arg2.size1()!=1)
        THROW_MBSIMERROR("The dimension of a parameter does not match.");
      if(this->retSize1!=0 && ret.size1()!=this->retSize1)
        THROW_MBSIMERROR("The output dimension does not match.");
      if(this->retSize2!=0 && ret.size2()!=this->retSize2)
        THROW_MBSIMERROR("The output dimension does not match.");
    }
  };

}

#endif
