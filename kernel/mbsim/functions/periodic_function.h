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

#ifndef _PERIODIC_FUNCTION_H_
#define _PERIODIC_FUNCTION_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<typename Sig> class PeriodicFunction; 

  template<typename Ret> 
  class PeriodicFunction<Ret(double)> : public Function<Ret(double)> {
    public:
      PeriodicFunction() : f(NULL), T(2*M_PI) { }
      ~PeriodicFunction() { delete f; }
      typename fmatvec::Size<double>::type getArgSize() const { return 1; }
      Ret operator()(const double &x) { return (*f)(x-T*floor(x/T)); }
      typename fmatvec::Der<Ret, double>::type parDer(const double &x) { return f->parDer(x-T*floor(x/T)); }
      typename fmatvec::Der<Ret, double>::type parDerDirDer(const double &xDir, const double &x) { return f->parDerDirDer(xDir,x-T*floor(x/T)); }
      typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &x) { return f->parDerParDer(x-T*floor(x/T)); }
      void setFunction(Function<Ret(double)> *f_) {
        f = f_;
        f->setParent(this);
        f->setName("Function");
      }
      void setPeriod(double T_) { T = T_; }
      void initializeUsingXML(xercesc::DOMElement *element) {
        xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"function");
        setFunction(ObjectFactory::createAndInit<Function<Ret(double)> >(e->getFirstElementChild()));
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"period");
        setPeriod(Element::getDouble(e));
      }
      void init(Element::InitStage stage) {
        if(stage == Element::unknownStage) {
          Function<Ret(double)>::init(stage);
//          Ret y0 = f(FromDouble<double>::cast(0));
//          Ret y1 = f(FromDouble<double>::cast(T));
        }
        else
          Function<Ret(double)>::init(stage);
        f->init(stage);
      }
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent) {
        return 0;
      }
    private:
      Function<Ret(double)> *f;
      double T;
  };

  template<typename Ret, typename Arg> 
  class PeriodicFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    public:
      PeriodicFunction() : f(NULL), T(2*M_PI) { }
      ~PeriodicFunction() { delete f; }
      typename fmatvec::Size<Arg>::type getArgSize() const { return f->getArgSize(); }
      Ret operator()(const Arg &x) { return (*f)(ToDouble<Arg>::cast(x)-T*floor(ToDouble<Arg>::cast(x)/T)); }
      typename fmatvec::Der<Ret, Arg>::type parDer(const Arg &x) { return f->parDer(ToDouble<Arg>::cast(x)-T*floor(ToDouble<Arg>::cast(x)/T)); }
      typename fmatvec::Der<Ret, Arg>::type parDerDirDer(const Arg &xDir, const Arg &x) { return f->parDerDirDer(xDir,ToDouble<Arg>::cast(x)/T); }
      void setFunction(Function<Ret(Arg)> *f_) {
        f = f_;
        f->setParent(this);
        f->setName("Function");
      }
      void setPeriod(double T_) { T = T_; }
      void initializeUsingXML(xercesc::DOMElement *element) {
        xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"function");
        setFunction(ObjectFactory::createAndInit<Function<Ret(Arg)> >(e->getFirstElementChild()));
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"period");
        setPeriod(Element::getDouble(e));
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
      }
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent) {
        return 0;
      }
    private:
      Function<Ret(Arg)> *f;
      double T;
  };

}

#endif
