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

#ifndef _PIECEWISE_DEFINED_FUNCTIONS_H_
#define _PIECEWISE_DEFINED_FUNCTIONS_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<typename Sig> class LimitedFunction; 

  template <typename Ret, typename Arg>
  struct LimitedFunction<Ret(Arg)> {
    LimitedFunction(Function<Ret(Arg)> *function_, double limit_) : function(function_), limit(limit_) { }
    Function<Ret(Arg)> *function;
    double limit;
  };

  template<typename Sig> class PiecewiseDefinedFunction; 

  template<typename Ret, typename Arg>
  class PiecewiseDefinedFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    public:
      PiecewiseDefinedFunction() : contDiff(0) { a.push_back(0); }
      ~PiecewiseDefinedFunction() {
        for(unsigned int i=0; i<function.size(); i++)
          delete function[i];
      }
      void addLimitedFunction(const LimitedFunction<Ret(Arg)> &limitedFunction) {
        function.push_back(limitedFunction.function);
        limitedFunction.function->setParent(this);
        a.push_back(limitedFunction.limit);
      }
      void setContinouslyDifferentiable(Arg contDiff_) { contDiff = contDiff_; }
      Ret zeros(const Ret &x) { return Ret(x.size()); }
      Ret operator()(const Arg &x) {
        for(unsigned int i=0; i<a.size(); i++)
          if(x<=a[i+1])
            return (*function[i])(x);
        if(contDiff==0)
          return yEnd;
        else if(contDiff==1)
          return yEnd+ysEnd*(x-a[a.size()-1]);
        else
          return yEnd+(ysEnd+0.5*yssEnd*(x-a[a.size()-1]))*(x-a[a.size()-1]);
      }
      typename fmatvec::Der<Ret, double>::type parDer(const double &x) {
        for(unsigned int i=0; i<a.size(); i++)
          if(x<=a[i+1])
            return function[i]->parDer(x);
        if(contDiff==0)
          return zeros(yEnd);
        else if(contDiff==1)
          return ysEnd;
        else
          return ysEnd+yssEnd*(x-a[a.size()-1]);
      }
      typename fmatvec::Der<Ret, Arg>::type parDerDirDer(const Arg &xDir, const Arg &x) {
        for(unsigned int i=0; i<a.size(); i++)
          if(x<=a[i+1])
            return function[i]->parDerDirDer(xDir,x);
        if(contDiff==0)
          return zeros(yEnd);
        else if(contDiff==1)
          return zeros(yEnd);
        else
          return yssEnd;
      }
      typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &x) {
        for(unsigned int i=0; i<a.size(); i++)
          if(x<=a[i+1])
            return function[i]->parDerParDer(x);
        if(contDiff==0)
          return zeros(yEnd);
        else if(contDiff==1)
          return zeros(yEnd);
        else
          return yssEnd;
      }

      void initializeUsingXML(xercesc::DOMElement *element) {
        xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"limitedFunctions");
        xercesc::DOMElement *ee=e->getFirstElementChild();
        while(ee && MBXMLUtils::E(ee)->getTagName()==MBSIM%"LimitedFunction") {
          addLimitedFunction(LimitedFunction<Ret(Arg)>(ObjectFactory::createAndInit<Function<Ret(Arg)> >(MBXMLUtils::E(ee)->getFirstElementChildNamed(MBSIM%"function")->getFirstElementChild()),Element::getDouble(MBXMLUtils::E(ee)->getFirstElementChildNamed(MBSIM%"limit"))));
          ee=ee->getNextElementSibling();
        }
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"continouslyDifferentiable");
        if(e) contDiff=Element::getDouble(e);
      }
      void init(Element::InitStage stage) {
        Function<Ret(Arg)>::init(stage);
        for(typename std::vector<Function<Ret(Arg)> *>::iterator it=function.begin(); it!=function.end(); it++)
          (*it)->init(stage);
        if(stage==Element::preInit) {
          yEnd = (*function[function.size()-1])(a[a.size()-1]);
          if(contDiff>0) {
            ysEnd = function[function.size()-1]->parDer(a[a.size()-1]);
            if(contDiff>1)
              yssEnd = function[function.size()-1]->parDerParDer(a[a.size()-1]);
          }
        }
      }
    private:
      std::vector<Function<Ret(Arg)> *> function;
      std::vector<double> a;
      int contDiff;
      Ret yEnd, ysEnd, yssEnd;
  };

  template<>
    inline double PiecewiseDefinedFunction<double(double)>::zeros(const double &x) { return 0; } 

}

#endif
