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

#ifndef _NESTED_FUNCTIONS_H_
#define _NESTED_FUNCTIONS_H_

#include "mbsim/functions/function.h"
#include "mbsim/objectfactory.h"

namespace MBSim {

  template<typename Sig> class NestedFunction; 

  // NestedFunction with a double as inner argument (including 2nd derivative)
  template<typename Ret, typename Argo> 
    class NestedFunction<Ret(Argo(double))> : public Function<Ret(double)> {
      public:
        NestedFunction(Function<Ret(Argo)> *fo_=0, Function<Argo(double)> *fi_=0) : fo(fo_), fi(fi_) {
          if(fo)
            fo->setParent(this);
          if(fi)
            fi->setParent(this);
        }
        ~NestedFunction() {
          delete fo;
          delete fi;
        }
        typename fmatvec::Size<double>::type getArgSize() const {
          return fi->getArgSize();
        }
        Ret operator()(const double &arg) {
          return (*fo)((*fi)(arg));
        }
        typename fmatvec::Der<Ret, double>::type parDer(const double &arg) {
          return fo->parDer((*fi)(arg))*fi->parDer(arg);
        }
        typename fmatvec::Der<Ret, double>::type parDerDirDer(const double &argDir, const double &arg) {
          return fo->parDerDirDer(fi->parDer(arg)*argDir,(*fi)(arg))*fi->parDer(arg) + fo->parDer((*fi)(arg))*fi->parDerDirDer(argDir,arg);
        }
        typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &arg) {
          return fo->parDerDirDer(fi->parDer(arg),(*fi)(arg))*fi->parDer(arg) + fo->parDer((*fi)(arg))*fi->parDerParDer(arg);
        }
        void setOuterFunction(Function<Ret(Argo)> *fo_) {
          fo = fo_;
          fo->setParent(this);
          fo->setName("Outer");
        }
        void setInnerFunction(Function<Argo(double)> *fi_) {
          fi = fi_;
          fi->setParent(this);
          fi->setName("Inner");
        }
        void initializeUsingXML(xercesc::DOMElement *element) {
          xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"outerFunction");
          setOuterFunction(ObjectFactory::createAndInit<Function<Ret(Argo)> >(e->getFirstElementChild()));
          e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"innerFunction");
          setInnerFunction(ObjectFactory::createAndInit<Function<Argo(double)> >(e->getFirstElementChild()));
        }
        void init(Element::InitStage stage) {
          Function<Ret(double)>::init(stage);
          fo->init(stage);
          fi->init(stage);
        }
        xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent) {
          return 0;
        } 
        std::vector<Element*> getDependencies() const { return cat<Element*>(fo->getDependencies(),fi->getDependencies()); }
      private:
        Function<Ret(Argo)> *fo;
        Function<Argo(double)> *fi;
    };

  // VectorValuedFunction with a vector as inner argument (no 2nd derivative defined)
  template<typename Ret, typename Argo, typename Argi> 
    class NestedFunction<Ret(Argo(Argi))> : public Function<Ret(Argi)> {
      public:
        NestedFunction(Function<Ret(Argo)> *fo_=0, Function<Argo(Argi)> *fi_=0) : fo(fo_), fi(fi_) {
          if(fo)
            fo->setParent(this);
          if(fi)
            fi->setParent(this);
        }
        ~NestedFunction() {
          delete fo;
          delete fi;
        }
        typename fmatvec::Size<Argi>::type getArgSize() const {
          return fi->getArgSize();
        }
        Ret operator()(const Argi &arg) {
          return (*fo)((*fi)(arg));
        }
        typename fmatvec::Der<Ret, Argi>::type parDer(const Argi &arg) {
          return fo->parDer((*fi)(arg))*fi->parDer(arg);
        }
        typename fmatvec::Der<Ret, Argi>::type parDerDirDer(const Argi &argDir, const Argi &arg) {
          return fo->parDerDirDer(fi->parDer(arg)*argDir,(*fi)(arg))*fi->parDer(arg) + fo->parDer((*fi)(arg))*fi->parDerDirDer(argDir,arg);
        }
        void setOuterFunction(Function<Ret(Argo)> *fo_) {
          fo = fo_;
          fo->setParent(this);
          fo->setName("Outer");
        }
        void setInnerFunction(Function<Argo(Argi)> *fi_) {
          fi = fi_;
          fi->setParent(this);
          fi->setName("Inner");
        }
        void initializeUsingXML(xercesc::DOMElement *element) {
          xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"outerFunction");
          setOuterFunction(ObjectFactory::createAndInit<Function<Ret(Argo)> >(e->getFirstElementChild()));
          e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"innerFunction");
          setInnerFunction(ObjectFactory::createAndInit<Function<Argo(Argi)> >(e->getFirstElementChild()));
        }
        void init(Element::InitStage stage) {
          Function<Ret(Argi)>::init(stage);
          fo->init(stage);
          fi->init(stage);
        }
        xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent) {
          return 0;
        } 
        std::vector<Element*> getDependencies() const { return cat<Element*>(fo->getDependencies(),fi->getDependencies()); }
      private:
        Function<Ret(Argo)> *fo;
        Function<Argo(Argi)> *fi;
    };

  template<typename Sig> class BinaryNestedFunction; 

  // BinaryNestedFunction with a double as inner argument (2nd derivative defined)
  template<typename Ret, typename Argo> 
    class BinaryNestedFunction<Ret(Argo(double))> : public Function<Ret(double)> {
      public:
        BinaryNestedFunction(Function<Ret(Argo,Argo)> *fo_=0, Function<Argo(double)> *fi1_=0, Function<Argo(double)> *fi2_=0) : fo(fo_), fi1(fi1_), fi2(fi2_) {
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
        void setOuterFunction(Function<Ret(Argo,Argo)> *fo_) {
          fo = fo_;
          fo->setParent(this);
          fo->setName("Outer");
        }
        void setFirstInnerFunction(Function<Argo(double)> *fi_) {
          fi1 = fi_;
          fi1->setParent(this);
          fi1->setName("FirstInner");
        }
        void setSecondInnerFunction(Function<Argo(double)> *fi_) {
          fi2 = fi_;
          fi2->setParent(this);
          fi2->setName("SecondInner");
        }
        void initializeUsingXML(xercesc::DOMElement *element) {
          xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"outerFunction");
          setOuterFunction(ObjectFactory::createAndInit<Function<Ret(Argo,Argo)> >(e->getFirstElementChild()));
          e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"firstInnerFunction");
          setFirstInnerFunction(ObjectFactory::createAndInit<Function<Argo(double)> >(e->getFirstElementChild()));
          e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"secondInnerFunction");
          setSecondInnerFunction(ObjectFactory::createAndInit<Function<Argo(double)> >(e->getFirstElementChild()));
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
        std::vector<Element*> getDependencies() const { return cat<Element*>(fo->getDependencies(),cat<Element*>(fi1->getDependencies(),fi2->getDependencies())); }
      private:
        Function<Ret(Argo,Argo)> *fo;
        Function<Argo(double)> *fi1, *fi2;
    };

  // BinaryNestedFunction with a vector as inner argument (no 2nd derivative defined)
  template<typename Ret, typename Argo, typename Argi> 
    class BinaryNestedFunction<Ret(Argo(Argi))> : public Function<Ret(Argi)> {
      public:
        BinaryNestedFunction(Function<Ret(Argo,Argo)> *fo_=0, Function<Argo(Argi)> *fi1_=0, Function<Argo(Argi)> *fi2_=0) : fo(fo_), fi1(fi1_), fi2(fi2_) {
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
        void setOuterFunction(Function<Ret(Argo,Argo)> *fo_) {
          fo = fo_;
          fo->setParent(this);
          fo->setName("Outer");
        }
        void setFirstInnerFunction(Function<Argo(Argi)> *fi_) {
          fi1 = fi_;
          fi1->setParent(this);
          fi1->setName("FirstInner");
        }
        void setSecondInnerFunction(Function<Argo(Argi)> *fi_) {
          fi2 = fi_;
          fi2->setParent(this);
          fi2->setName("SecondInner");
        }
        void initializeUsingXML(xercesc::DOMElement *element) {
          xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"outerFunction");
          setOuterFunction(ObjectFactory::createAndInit<Function<Ret(Argo,Argo)> >(e->getFirstElementChild()));
          e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"firstInnerFunction");
          setFirstInnerFunction(ObjectFactory::createAndInit<Function<Argo(Argi)> >(e->getFirstElementChild()));
          e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"secondInnerFunction");
          setSecondInnerFunction(ObjectFactory::createAndInit<Function<Argo(Argi)> >(e->getFirstElementChild()));
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
        std::vector<Element*> getDependencies() const { return cat<Element*>(fo->getDependencies(),cat<Element*>(fi1->getDependencies(),fi2->getDependencies())); }
      private:
        Function<Ret(Argo,Argo)> *fo;
        Function<Argo(Argi)> *fi1, *fi2;
    };

}

#endif

