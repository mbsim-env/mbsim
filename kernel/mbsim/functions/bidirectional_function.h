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
    public:
      BidirectionalFunction(Function<Ret(Arg)> *fn_=0, Function<Ret(Arg)> *fp_=0) : fn(fn_), fp(fp_) {
        if(fn) fn->setParent(this);
        if(fp) fp->setParent(this);
      }
      ~BidirectionalFunction() {
        delete fn;
        delete fp;
      }
      typename fmatvec::Size<double>::type getArgSize() const {
        return fn->getArgSize();
      }
      Ret operator()(const Arg &x) {
        return (ToDouble<Arg>::cast(x)>=0)?(*fp)(x):(*fn)(-x);
      }
      typename fmatvec::Der<Ret, double>::type parDer(const double &x) {
        return (ToDouble<Arg>::cast(x)>=0)?fp->parDer(x):fn->parDer(-x);
      }
      typename fmatvec::Der<Ret, Arg>::type parDerDirDer(const Arg &xDir, const Arg &x) {
        return (ToDouble<Arg>::cast(x)>=0)?fp->parDerDirDer(xDir,x):fn->parDerDirDer(-xDir,-x);
      }
      typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &x) {
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
      void initializeUsingXML(xercesc::DOMElement *element) {
        xercesc::DOMElement *e;
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"negativeDirectionalFunction");
        setNegativeDirectionalFunction(ObjectFactory::createAndInit<Function<Ret(Arg)> >(e->getFirstElementChild()));
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"positiveDirectionalFunction");
        setPositiveDirectionalFunction(ObjectFactory::createAndInit<Function<Ret(Arg)> >(e->getFirstElementChild()));
      }
      void init(Element::InitStage stage) {
        Function<Ret(Arg)>::init(stage);
        fn->init(stage);
        fp->init(stage);
      }
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent) {
        return 0;
      }
    private:
      Function<Ret(Arg)> *fn, *fp;
  };

}

#endif
