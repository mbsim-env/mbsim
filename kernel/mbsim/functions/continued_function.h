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

#ifndef _CONTINUED_FUNCTION_H_
#define _CONTINUED_FUNCTION_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<typename Sig> class ContinuedFunction; 

  template<typename Ret, typename Arg> 
  class ContinuedFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    public:
      ContinuedFunction() : f(NULL), rule(NULL) { }
      ~ContinuedFunction() { delete f; delete rule; }
      typename fmatvec::Size<Arg>::type getArgSize() const { return f->getArgSize(); }
      Ret operator()(const Arg &x) { return (*f)((*rule)(x)); }
      typename fmatvec::Der<Ret, Arg>::type parDer(const Arg &x) { return f->parDer((*rule)(x)); }
      typename fmatvec::Der<Ret, Arg>::type parDerDirDer(const Arg &xDir, const Arg &x) { return f->parDerDirDer(xDir,(*rule)(x)); }
      typename fmatvec::Der<typename fmatvec::Der<Ret, Arg>::type, Arg>::type parDerParDer(const Arg &x) { return f->parDerParDer((*rule)(x)); }
      void setFunction(Function<Ret(Arg)> *f_) {
        f = f_;
        f->setParent(this);
        f->setName("Function");
      }
      void setContinuationRule(Function<Arg(Arg)> *rule_) { 
        rule = rule_; 
        rule->setParent(this);
        rule->setName("ContinuationRule");
      }
      void initializeUsingXML(xercesc::DOMElement *element) {
        xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"function");
        setFunction(ObjectFactory::createAndInit<Function<Ret(Arg)> >(e->getFirstElementChild()));
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"continuationRule");
        setContinuationRule(ObjectFactory::createAndInit<Function<Arg(Arg)> >(e->getFirstElementChild()));
      }
      void init(Element::InitStage stage) {
        if(stage == Element::unknownStage) {
          Function<Ret(Arg)>::init(stage);
//          Ret y0 = f(FromDouble<Arg>::cast(0));
//          Ret y1 = f(FromDouble<Arg>::cast(T));
        }
        else
          Function<Ret(Arg)>::init(stage);
        f->init(stage);
        rule->init(stage);
      }
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent) {
        return 0;
      }
    private:
      Function<Ret(Arg)> *f;
      Function<Arg(Arg)> *rule;
  };

}

#endif
