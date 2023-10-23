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

#ifndef _NESTED_FUNCTION_H_
#define _NESTED_FUNCTION_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<typename Sig> class CompositeFunction; 

  /**
   * Defines a function \f$ f \f$ to type
   * \f[ y = f(x) \f]
   * If the outer function \f$ o \f$ is of type
   * \f[ o(a) \f]
   * then a single inner function \f$ i \f$ must be given and
   * \f[ y = f(x) = o(i(x)) \f]
   * If the outer function \f$ f \f$ is of type
   * \f[ o(a, b) \f]
   * then two inner functions \f$ i1, i2 \f$ must be given and
   * \f[ y = f(x) = o(i1(x), i2(x)) \f]
   */
  template<typename Ret, typename Argo, typename Argi> 
  class CompositeFunction<Ret(Argo(Argi))> : public Function<Ret(Argi)> {
    using B = Function<Ret(Argi)>; 
    public:
      CompositeFunction() = default;
      CompositeFunction(Function<Ret(Argo)> *fo_, Function<Argo(Argi)> *fi_) {
        setOuterFunction(fo_);
        setInnerFunction(fi_);
      }
      ~CompositeFunction() override {
        delete fo1;
        delete fo2;
        for (auto & i : fi) delete i;
      }
      int getArgSize() const override {
        return fi[0]->getArgSize();
      }
      std::pair<int, int> getRetSize() const override {
        return fo1?fo1->getRetSize():fo2->getRetSize();
      }
      Ret operator()(const Argi &arg) override {
        return fo1?(*fo1)((*fi[0])(arg)):(*fo2)((*fi[0])(arg),(*fi[1])(arg));
      }
      typename B::DRetDArg parDer(const Argi &arg) override {
        return fo1?fo1->parDer((*fi[0])(arg))*fi[0]->parDer(arg):fo2->parDer1((*fi[0])(arg),(*fi[1])(arg))*fi[0]->parDer(arg)+fo2->parDer2((*fi[0])(arg),(*fi[1])(arg))*fi[1]->parDer(arg);
      }
      typename B::DRetDArg parDerDirDer(const Argi &argDir, const Argi &arg) override {
        return fo1?fo1->parDerDirDer(fi[0]->parDer(arg)*argDir,(*fi[0])(arg))*fi[0]->parDer(arg)+fo1->parDer((*fi[0])(arg))*fi[0]->parDerDirDer(argDir,arg):fo2->parDer1DirDer1(fi[0]->parDer(arg)*argDir,(*fi[0])(arg),(*fi[0])(arg))*fi[0]->parDer(arg)+fo2->parDer1DirDer2(fi[1]->parDer(arg)*argDir,(*fi[0])(arg),(*fi[0])(arg))*fi[0]->parDer(arg)+fo2->parDer2DirDer1(fi[0]->parDer(arg)*argDir,(*fi[0])(arg),(*fi[0])(arg))*fi[1]->parDer(arg)+fo2->parDer2DirDer2(fi[1]->parDer(arg)*argDir,(*fi[0])(arg),(*fi[0])(arg))*fi[1]->parDer(arg)+fo2->parDer1((*fi[0])(arg),(*fi[1])(arg))*fi[0]->parDerDirDer(argDir,arg)+fo2->parDer2((*fi[0])(arg),(*fi[1])(arg))*fi[1]->parDerDirDer(argDir,arg);
      }
      void setOuterFunction(Function<Ret(Argo)> *fo_) {
        fo1 = fo_;
        fo1->setParent(this);
        fo1->setName("Outer");
      }
      void setOuterFunction(Function<Ret(Argo,Argo)> *fo_) {
        fo2 = fo_;
        fo2->setParent(this);
        fo2->setName("Outer");
      }
      void setInnerFunction(Function<Argo(Argi)> *fi_) {
        fi_->setParent(this);
        fi_->setName("Inner");
        fi.resize(1,fi_);
      }
      void addInnerFunction(Function<Argo(Argi)> *fi_) {
        fi_->setParent(this);
        fi_->setName("Inner");
        fi.push_back(fi_);
      }
      void initializeUsingXML(xercesc::DOMElement *element) override {
        xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"innerFunction");
        if(e) setInnerFunction(ObjectFactory::createAndInit<Function<Argo(Argi)>>(e->getFirstElementChild()));
        else {
          e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"innerFunctions")->getFirstElementChild();
          for(int i=0; i<2; i++) {
            addInnerFunction(ObjectFactory::createAndInit<Function<Argo(Argi)>>(e));
            e=e->getNextElementSibling();
          }
        }
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"outerFunction");
        if(fi.size()==1)
          setOuterFunction(ObjectFactory::createAndInit<Function<Ret(Argo)>>(e->getFirstElementChild()));
        else if(fi.size()==2)
          setOuterFunction(ObjectFactory::createAndInit<Function<Ret(Argo,Argo)>>(e->getFirstElementChild()));
      }
      void init(Element::InitStage stage, const InitConfigSet &config) override {
        Function<Ret(Argi)>::init(stage, config);
        if(fo1) fo1->init(stage, config);
        if(fo2) fo2->init(stage, config);
        for (auto & i : fi) i->init(stage, config);
      }
    private:
      Function<Ret(Argo)> *fo1{nullptr};
      Function<Ret(Argo,Argo)> *fo2{nullptr};
      std::vector<Function<Argo(Argi)>*> fi;
  };
}

#endif
