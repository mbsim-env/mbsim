/* Copyright (C) 2004-2016 MBSim Development Team
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

#ifndef _BIDIRECTIONAL_FUNCTION_H_
#define _BIDIRECTIONAL_FUNCTION_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<typename Sig> class BidirectionalFunction; 

  template<typename Ret, typename Arg>
  class BidirectionalFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    using B = fmatvec::Function<Ret(Arg)>; 
    public:
      BidirectionalFunction(Function<Ret(Arg)> *fn_=nullptr, Function<Ret(Arg)> *fp_=nullptr) : fn(fn_), fp(fp_) {
        if(fn) fn->setParent(this);
        if(fp) fp->setParent(this);
      }
      ~BidirectionalFunction() override {
        delete fn;
        delete fp;
      }
      int getArgSize() const override {
        return fn->getArgSize();
      }
      std::pair<int, int> getRetSize() const override {
        return fp->getRetSize();
      }
      Ret operator()(const Arg &x) override {
        return (ToDouble<Arg>::cast(x)>=0)?(*fp)(x):(*fn)(-x);
      }
      typename B::DRetDArg parDer(const double &x) override {
        return (ToDouble<Arg>::cast(x)>=0)?fp->parDer(x):fn->parDer(-x);
      }
      typename B::DRetDArg parDerDirDer(const Arg &xDir, const Arg &x) override {
        return (ToDouble<Arg>::cast(x)>=0)?fp->parDerDirDer(xDir,x):fn->parDerDirDer(-xDir,-x);
      }
      typename B::DDRetDDArg parDerParDer(const double &x) override {
        return (ToDouble<Arg>::cast(x)>=0)?fp->parDerParDer(x):fn->parDerParDer(-x);
      }
      void setNegativeDirectionalFunction(Function<Ret(Arg)> *fn_) {
        fn = fn_;
        fn->setParent(this);
        fn->setName("NegativeDirectional");
      }
      void setPositiveDirectionalFunction(Function<Ret(Arg)> *fp_) {
        fp = fp_;
        fp->setParent(this);
        fp->setName("PostiveDirectional");
      }
      void initializeUsingXML(xercesc::DOMElement *element) override {
        xercesc::DOMElement *e;
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"negativeDirectionalFunction");
        setNegativeDirectionalFunction(ObjectFactory::createAndInit<Function<Ret(Arg)> >(e->getFirstElementChild()));
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"positiveDirectionalFunction");
        setPositiveDirectionalFunction(ObjectFactory::createAndInit<Function<Ret(Arg)> >(e->getFirstElementChild()));
      }
      void init(Element::InitStage stage, const InitConfigSet &config) override {
        Function<Ret(Arg)>::init(stage, config);
        fn->init(stage, config);
        fp->init(stage, config);
      }
    private:
      Function<Ret(Arg)> *fn, *fp;
  };

}

#endif
