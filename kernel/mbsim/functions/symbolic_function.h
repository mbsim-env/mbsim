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

  //! A fmatvec like parDer function for casadi SX.
  //! This function handles all type f except fmatvec::RotMat3.
  template<typename Ret=void>
  casadi::SX parDerSX(const casadi::SX &f, const casadi::SX &arg) {
    assert(arg.size2()==1);
    if(f.size2()>1) { // f is a matrix
      if(arg.size1()==1) { // arg is a scalar -> jac is a matrix but this cannot be handled by casadi directly
        // calcualte jacobian using casadi -> reshape to matrix
        return casadi::SX::reshape(casadi::SX::jacobian(f, arg), f.size1(), f.size2());
      }
      else { // arg is a vector -> not possible (would be a tensor of order 3 which cannot be handled by casadi and fmatvec)
        return casadi::SX::nan(); // return a scalar NaN -> will throw later if this is tried to evaluate
      }
    }
    else // f is a vector -> jac may get a matrix if arg is a vector (handled by casadi)
      return casadi::SX::jacobian(f, arg);
  }

  //! A fmatvec like parDer function for casadi SX.
  //! This function handles only the f type fmatvec::RotMat3.
  template<>
  casadi::SX parDerSX<fmatvec::RotMat3>(const casadi::SX &f, const casadi::SX &arg) {
    assert(f.size1()==3);
    assert(f.size2()==3);
    assert(arg.size2()==1);
    casadi::SX ret(3, arg.size1());
    for(int i=0; i<arg.size1(); ++i) {
      // wtilde = df/dargi * f
      casadi::SX wtilde=casadi::SX::mtimes(parDerSX(f, arg(i)), f.T());
      // ret = tilde(wtilde)
      ret(0,i)=wtilde(2,1);
      ret(1,i)=wtilde(0,2);
      ret(2,i)=wtilde(1,0);
    }
    return ret;
  }

  //! A fmatvec like dirDer function for casadi SX.
  //! This function handles all type f except fmatvec::RotMat3.
  template<typename Ret=void>
  casadi::SX dirDerSX(const casadi::SX &f, const casadi::SX &arg, const casadi::SX &argd) {
    assert(arg.size2()==1);
    assert(argd.size2()==1);
    assert(arg.size1()==argd.size1());
    if(f.size2()==1)
      return jtimes(f, arg, argd);
    else
      return casadi::SX::reshape(jtimes(casadi::SX::reshape(f, f.size1()*f.size2(), 1), arg, argd), f.size1(), f.size2());
  }

  //! A fmatvec like dirDer function for casadi SX.
  //! This function handles only the f type fmatvec::RotMat3.
  template<>
  casadi::SX dirDerSX<fmatvec::RotMat3>(const casadi::SX &f, const casadi::SX &arg, const casadi::SX &argd) {
    assert(f.size1()==3);
    assert(f.size2()==3);
    assert(arg.size2()==1);
    assert(argd.size2()==1);
    assert(arg.size1()==argd.size1());
    return casadi::SX::mtimes(parDerSX<fmatvec::RotMat3>(f, arg), argd);
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

  template<>
  class FromCasadi<fmatvec::RotMat3> {
    public:
      static fmatvec::RotMat3 cast(const casadi::Matrix<double> &A) {
        assert(A.size1()==3);
        assert(A.size2()==3);
        fmatvec::RotMat3 B(fmatvec::NONINIT);
        for(int i=0; i<3; i++)
          for(int j=0; j<3; j++)
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
  private:
    using B = fmatvec::Function<Ret(Arg)>; 
    casadi::SX ret, arg;
    mutable casadi::SX pd_;
    casadi::SX pddd_, pdpd_;
    casadi::Function f, pd, dd, pddd, pdpd, dddd;
  public:
    SymbolicFunction() {}
    SymbolicFunction(const casadi::SX &ret_, const casadi::SX &arg_) : ret(ret_), arg(arg_) {
      checkFunctionIODim();
    }

    void init(Element::InitStage stage) {
      Function<Ret(Arg)>::init(stage);
      if(stage == Element::preInit) {
        casadi::SX argd=casadi::SX::sym("argd", getArgSize());
        casadi::SX argd_2=casadi::SX::sym("argd_2", getArgSize());

        pd_ = parDerSX<Ret>(ret, arg);
        casadi::SX dd_ = dirDerSX<Ret>(ret, arg, argd);
        pddd_ = parDerSX(dd_, arg);
        pdpd_ = parDerSX(pd_, arg);
        casadi::SX dddd_ = dirDerSX(dd_, arg, argd_2);

        f = casadi::Function("noname", {arg}, {ret});
        pd = casadi::Function("noname", {arg}, {pd_});
        dd = casadi::Function("noname", {argd, arg}, {dd_});
        pddd = casadi::Function("noname", {argd, arg}, {pddd_});
        pdpd = casadi::Function("noname", {arg}, {pdpd_});
        dddd = casadi::Function("noname", {argd_2, argd, arg}, {dddd_});
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
        pd_ = parDerSX<Ret>(ret, arg);
      return pd_.is_constant();
    }

    Ret operator()(const Arg& x) override {
      return c<Ret>(f(std::vector<casadi::SX>{c(x)})[0]);
    }

    typename B::DRetDArg parDer(const Arg &x) override {
      if(pd_(0,0).scalar().isNan())
        THROW_MBSIMERROR("Cannot calculate this partial derivative, would be a tensor of order 3.");
      return c<typename B::DRetDArg>(pd(std::vector<casadi::SX>{c(x)})[0]);
    }

    Ret dirDer(const Arg &xd, const Arg &x) override {
      return c<Ret>(dd(std::vector<casadi::SX>{c(xd), c(x)})[0]);
    }

    typename B::DRetDArg parDerDirDer(const Arg &xd, const Arg &x) override {
      if(pddd_(0,0).scalar().isNan())
        THROW_MBSIMERROR("Cannot calculate this partial derivative, would be a tensor of order 3.");
      return c<typename B::DRetDArg>(pddd(std::vector<casadi::SX>{c(xd), c(x)})[0]);
    }

    typename B::DDRetDDArg parDerParDer(const Arg &x) override {
      if(pdpd_(0,0).scalar().isNan())
        THROW_MBSIMERROR("Cannot calculate this partial derivative, would be a tensor of order 3.");
      return c<typename B::DDRetDDArg>(pdpd(std::vector<casadi::SX>{c(x)})[0]);
    }

    Ret dirDerDirDer(const Arg &argDir_1, const Arg &argDir_2, const Arg &arg) override {
      return c<Ret>(dddd(std::vector<casadi::SX>{c(argDir_1), c(argDir_2), c(arg)})[0]);
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
      if(arg.size2()!=1) THROW_MBSIMERROR("Matrix parameters are not allowed.");
      // check function <-> template argument dimension
      if(this->argSize!=0 && arg.size1()!=this->argSize)
        THROW_MBSIMERROR("The dimension of the parameter does not match.");
      if(this->retSize1!=0 && ret.size1()!=this->retSize1)
        THROW_MBSIMERROR("The output row dimension does not match.");
      if(this->retSize2!=0 && ret.size2()!=this->retSize2)
        THROW_MBSIMERROR("The output column dimension does not match.");
    }
  };

  template<typename Ret, typename Arg1, typename Arg2>
  class SymbolicFunction<Ret(Arg1, Arg2)> : public Function<Ret(Arg1, Arg2)> {
  private:
    using B = fmatvec::Function<Ret(Arg1, Arg2)>; 
    casadi::SX ret, arg1, arg2;
    mutable casadi::SX pd1_, pd2_;
    casadi::SX pd1dd1_, pd1dd2_, pd1pd2_, pd2dd1_, pd2dd2_, pd2pd2_;
    casadi::Function f, pd1, pd2, pd1dd1, pd1dd2, pd1pd2, pd2dd1, pd2dd2, pd2pd2;
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

        pd1_ = parDerSX<Ret>(ret, arg1);
        pd2_ = parDerSX<Ret>(ret, arg2);
        casadi::SX dd1_ = dirDerSX<Ret>(ret, arg1, arg1d);
        casadi::SX dd2_ = dirDerSX<Ret>(ret, arg2, arg2d);
        pd1dd1_ = parDerSX(dd1_, arg1);
        pd1dd2_ = parDerSX(dd2_, arg1);
        pd1pd2_ = parDerSX(pd1_, arg2);
        pd2dd1_ = parDerSX(dd1_, arg2);
        pd2dd2_ = parDerSX(dd2_, arg2);
        pd2pd2_ = parDerSX(pd2_, arg2);

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
        pd1_ = parDerSX<Ret>(ret, arg1);
      return pd1_.is_constant();
    }

    bool constParDer2() const override {
      if(pd2_.is_empty(true))
        pd2_ = parDerSX<Ret>(ret, arg2);
      return pd2_.is_constant();
    }

    Ret operator()(const Arg1& x1, const Arg2& x2) override {
      return c<Ret>(f(std::vector<casadi::SX>{c(x1), c(x2)})[0]);
    }

    typename B::DRetDArg1 parDer1(const Arg1 &x1, const Arg2 &x2) override {
      if(pd1_(0,0).scalar().isNan())
        THROW_MBSIMERROR("Cannot calculate this partial derivative, would be a tensor of order 3.");
      return c<typename B::DRetDArg1>(pd1(std::vector<casadi::SX>{c(x1), c(x2)})[0]);
    }

    Ret dirDer1(const Arg1 &arg1Dir, const Arg1 &arg1, const Arg2 &arg2) override {
      THROW_MBSIMERROR("Not implemented");
    }

    typename B::DRetDArg2 parDer2(const Arg1 &x1, const Arg2 &x2) override {
      if(pd2_(0,0).scalar().isNan())
        THROW_MBSIMERROR("Cannot calculate this partial derivative, would be a tensor of order 3.");
      return c<typename B::DRetDArg2>(pd2(std::vector<casadi::SX>{c(x1), c(x2)})[0]);
    }

    Ret dirDer2(const Arg2 &arg2Dir, const Arg1 &arg1, const Arg2 &arg2) override {
      THROW_MBSIMERROR("Not implemented");
    }

    typename B::DDRetDDArg1 parDer1ParDer1(const Arg1 &arg1, const Arg2 &arg2) override {
      THROW_MBSIMERROR("Not implemented");
    }

    typename B::DRetDArg1 parDer1DirDer1(const Arg1 &xd1, const Arg1 &x1, const Arg2 &x2) override {
      if(pd1dd1_(0,0).scalar().isNan())
        THROW_MBSIMERROR("Cannot calculate this partial derivative, would be a tensor of order 3.");
      return c<typename B::DRetDArg1>(pd1dd1(std::vector<casadi::SX>{c(xd1), c(x1), c(x2)})[0]);
    }

    Ret dirDer1DirDer1(const Arg1 &arg1Dir_1, const Arg1 &arg1Dir_2, const Arg1 &arg1, const Arg2 &arg2) override {
      THROW_MBSIMERROR("Not implemented");
    }

    typename B::DDRetDDArg2 parDer2ParDer2(const Arg1 &arg1, const Arg2 &arg2) override {
      if(pd2pd2_(0,0).scalar().isNan())
        THROW_MBSIMERROR("Cannot calculate this partial derivative, would be a tensor of order 3.");
      return c<typename B::DDRetDDArg2>(pd2pd2(std::vector<casadi::SX>{c(arg1), c(arg2)})[0]);
    }

    typename B::DRetDArg1 parDer1DirDer2(const Arg2 &xd2, const Arg1 &x1, const Arg2 &x2) override {
      if(pd1dd2_(0,0).scalar().isNan())
        THROW_MBSIMERROR("Cannot calculate this partial derivative, would be a tensor of order 3.");
      return c<typename B::DRetDArg1>(pd1dd2(std::vector<casadi::SX>{c(xd2), c(x1), c(x2)})[0]);
    }

    typename B::DRetDArg2 parDer2DirDer1(const Arg1 &xd1, const Arg1 &x1, const Arg2 &x2) override {
      if(pd2dd1_(0,0).scalar().isNan())
        THROW_MBSIMERROR("Cannot calculate this partial derivative, would be a tensor of order 3.");
      return c<typename B::DRetDArg2>(pd2dd1(std::vector<casadi::SX>{c(xd1), c(x1), c(x2)})[0]);
    }

    Ret dirDer2DirDer2(const Arg2 &arg2Dir_1, const Arg2 &arg2Dir_2, const Arg1 &arg1, const Arg2 &arg2) override {
      THROW_MBSIMERROR("Not implemented");
    }

    typename B::DDRetDArg1DArg2 parDer1ParDer2(const Arg1 &arg1, const Arg2 &arg2) override {
      if(pd1pd2_(0,0).scalar().isNan())
        THROW_MBSIMERROR("Cannot calculate this partial derivative, would be a tensor of order 3.");
      return c<typename B::DDRetDArg1DArg2>(pd1pd2(std::vector<casadi::SX>{c(arg1), c(arg2)})[0]);
    }

    Ret dirDer2DirDer1(const Arg2 &arg1Dir, const Arg1 &arg1, const Arg2 &arg2) override {
      THROW_MBSIMERROR("Not implemented");
    }

    typename B::DRetDArg2 parDer2DirDer2(const Arg2 &xd2, const Arg1 &x1, const Arg2 &x2) override {
      if(pd2dd2_(0,0).scalar().isNan())
        THROW_MBSIMERROR("Cannot calculate this partial derivative, would be a tensor of order 3.");
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
      if(arg1.size2()!=1) THROW_MBSIMERROR("Matrix parameters are not allowed.");
      if(arg2.size2()!=1) THROW_MBSIMERROR("Matrix parameters are not allowed.");
      // check function <-> template argument dimension
      if(this->arg1Size!=0 && arg1.size1()!=this->arg1Size)
        THROW_MBSIMERROR("The dimension of the first parameter does not match.");
      if(this->arg2Size!=0 && arg2.size1()!=this->arg2Size)
        THROW_MBSIMERROR("The dimension of the second parameter does not match.");
      if(this->retSize1!=0 && ret.size1()!=this->retSize1)
        THROW_MBSIMERROR("The output row dimension does not match.");
      if(this->retSize2!=0 && ret.size2()!=this->retSize2)
        THROW_MBSIMERROR("The output column dimension does not match.");
    }
  };

}

#endif
