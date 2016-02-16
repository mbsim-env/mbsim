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

#ifndef _BINARY_NESTED_FUNCTION_H_
#define _BINARY_NESTED_FUNCTION_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<typename Sig> class BinaryNestedFunction; 

  // BinaryNestedFunction with a double as inner argument (2nd derivative defined)
  template<typename Ret, typename Argo1, typename Argo2>
  class BinaryNestedFunction<Ret(Argo1(double),Argo2(double))> : public Function<Ret(double)> {
    public:
      BinaryNestedFunction(Function<Ret(Argo1,Argo2)> *fo_=0, Function<Argo1(double)> *fi1_=0, Function<Argo2(double)> *fi2_=0) : fo(fo_), fi1(fi1_), fi2(fi2_) {
        if(fo)
          fo->setParent(this);
        if(fi1)
          fi1->setParent(this);
        if(fi2)
          fi2->setParent(this);
      }
      ~BinaryNestedFunction() {
        delete fo;
        delete fi1;
        delete fi2;
      }
      typename fmatvec::Size<double>::type getArg1Size() const {
        return fi1->getArgSize();
      }
      typename fmatvec::Size<double>::type getArg2Size() const {
        return fi2->getArgSize();
      }
      Ret operator()(const double &arg) {
        return (*fo)((*fi1)(arg),(*fi2)(arg));
      }
      typename fmatvec::Der<Ret, double>::type parDer(const double &arg) {
        return fo->parDer1((*fi1)(arg),(*fi2)(arg))*fi1->parDer(arg) + fo->parDer2((*fi1)(arg),(*fi2)(arg))*fi2->parDer(arg);
      }
      typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &arg) {
        return fo->parDer1DirDer1(fi1->parDer(arg),(*fi1)(arg),(*fi2)(arg))*fi1->parDer(arg) \
          + fo->parDer1DirDer2(fi2->parDer(arg),(*fi1)(arg),(*fi2)(arg))*fi1->parDer(arg) \
          + fo->parDer2DirDer1(fi1->parDer(arg),(*fi1)(arg),(*fi2)(arg))*fi2->parDer(arg) \
          + fo->parDer2DirDer2(fi2->parDer(arg),(*fi1)(arg),(*fi2)(arg))*fi2->parDer(arg) \
          + fo->parDer1((*fi1)(arg),(*fi2)(arg))*fi1->parDerParDer(arg) \
          + fo->parDer2((*fi1)(arg),(*fi2)(arg))*fi2->parDerParDer(arg);
      }
      void setOuterFunction(Function<Ret(Argo1,Argo2)> *fo_) {
        fo = fo_;
        fo->setParent(this);
        fo->setName("Outer");
      }
      void setFirstInnerFunction(Function<Argo1(double)> *fi_) {
        fi1 = fi_;
        fi1->setParent(this);
        fi1->setName("FirstInner");
      }
      void setSecondInnerFunction(Function<Argo2(double)> *fi_) {
        fi2 = fi_;
        fi2->setParent(this);
        fi2->setName("SecondInner");
      }
      void initializeUsingXML(xercesc::DOMElement *element) {
        xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"outerFunction");
        setOuterFunction(ObjectFactory::createAndInit<Function<Ret(Argo1,Argo2)> >(e->getFirstElementChild()));
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"firstInnerFunction");
        setFirstInnerFunction(ObjectFactory::createAndInit<Function<Argo1(double)> >(e->getFirstElementChild()));
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"secondInnerFunction");
        setSecondInnerFunction(ObjectFactory::createAndInit<Function<Argo2(double)> >(e->getFirstElementChild()));
      }
      void init(Element::InitStage stage) {
        Function<Ret(double)>::init(stage);
        fo->init(stage);
        fi1->init(stage);
        fi2->init(stage);
      }
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent) {
        return 0;
      } 
    private:
      Function<Ret(Argo1,Argo2)> *fo;
      Function<Argo1(double)> *fi1;
      Function<Argo2(double)> *fi2;
  };

  // BinaryNestedFunction with a vector as inner argument (no 2nd derivative defined)
  template<typename Ret, typename Argo1, typename Argo2, typename Argi>
  class BinaryNestedFunction<Ret(Argo1(Argi),Argo2(Argi))> : public Function<Ret(Argi)> {
    public:
      BinaryNestedFunction(Function<Ret(Argo1,Argo2)> *fo_=0, Function<Argo1(Argi)> *fi1_=0, Function<Argo2(Argi)> *fi2_=0) : fo(fo_), fi1(fi1_), fi2(fi2_) {
        if(fo)
          fo->setParent(this);
        if(fi1)
          fi1->setParent(this);
        if(fi2)
          fi2->setParent(this);
      }
      ~BinaryNestedFunction() {
        delete fo;
        delete fi1;
        delete fi2;
      }
      typename fmatvec::Size<Argi>::type getArg1Size() const {
        return fi1->getArgSize();
      }
      typename fmatvec::Size<Argi>::type getArg2Size() const {
        return fi2->getArgSize();
      }
      Ret operator()(const Argi &arg) {
        return (*fo)((*fi1)(arg),(*fi2)(arg));
      }
      typename fmatvec::Der<Ret, Argi>::type parDer(const Argi &arg) {
        return fo->parDer1((*fi1)(arg),(*fi2)(arg))*fi1->parDer(arg) + fo->parDer2((*fi1)(arg),(*fi2)(arg))*fi2->parDer(arg);
      }
      void setOuterFunction(Function<Ret(Argo1,Argo2)> *fo_) {
        fo = fo_;
        fo->setParent(this);
        fo->setName("Outer");
      }
      void setFirstInnerFunction(Function<Argo1(Argi)> *fi_) {
        fi1 = fi_;
        fi1->setParent(this);
        fi1->setName("FirstInner");
      }
      void setSecondInnerFunction(Function<Argo2(Argi)> *fi_) {
        fi2 = fi_;
        fi2->setParent(this);
        fi2->setName("SecondInner");
      }
      void initializeUsingXML(xercesc::DOMElement *element) {
        xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"outerFunction");
        setOuterFunction(ObjectFactory::createAndInit<Function<Ret(Argo1,Argo2)> >(e->getFirstElementChild()));
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"firstInnerFunction");
        setFirstInnerFunction(ObjectFactory::createAndInit<Function<Argo1(Argi)> >(e->getFirstElementChild()));
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"secondInnerFunction");
        setSecondInnerFunction(ObjectFactory::createAndInit<Function<Argo2(Argi)> >(e->getFirstElementChild()));
      }
      void init(Element::InitStage stage) {
        Function<Ret(Argi)>::init(stage);
        fo->init(stage);
        fi1->init(stage);
        fi2->init(stage);
      }
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent) {
        return 0;
      } 
    private:
      Function<Ret(Argo1,Argo2)> *fo;
      Function<Argo1(Argi)> *fi1;
      Function<Argo2(Argi)> *fi2;
  };

}

#endif
