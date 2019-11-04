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

#ifndef _MBSIM_SYMBOLIC_FUNCTION_H_
#define _MBSIM_SYMBOLIC_FUNCTION_H_

#include <mbsim/functions/function.h>
#include <fmatvec/symbolic_function.h>

namespace MBSim {

  template<typename Sig> class SymbolicFunction;

  template<typename Ret, typename Arg>
  class SymbolicFunction<Ret(Arg)> : public Function<Ret(Arg)>, public fmatvec::SymbolicFunction<Ret(Arg)> {

  public:
    SymbolicFunction() = default;

    void init(Element::InitStage stage, const InitConfigSet &config) override {
      Function<Ret(Arg)>::init(stage, config);
      if(stage == Element::preInit) {
        fmatvec::SymbolicFunction<Ret(Arg)>::init();
//        checkFunctionIODim();
      }
    }

    void initializeUsingXML(xercesc::DOMElement *element) override {
      Function<Ret(Arg)>::initializeUsingXML(element);

      xercesc::DOMElement *input=MBXMLUtils::E(MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"inputs"))->getFirstElementChildNamed(MBSIM%"input");
      this->setIndependentVariable(typename fmatvec::SymbolicFunction<Ret(Arg)>::ArgS(MBXMLUtils::E(input)->getText<std::string>().c_str()));
      input=input->getNextElementSibling();
      if(input)
        throw MBXMLUtils::DOMEvalException("Exactly one input must be given.", MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"inputs"));

      xercesc::DOMElement *output=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"output");
      this->setDependentFunction(typename fmatvec::SymbolicFunction<Ret(Arg)>::RetS(MBXMLUtils::E(output)->getText<std::string>().c_str()));

      // check symbolic function arguments: we need to throw errors during initializeUsingXML to enable the ObjectFactory
      // to test other possible combinations (more general ones)
//      checkFunctionIODim();
    }

  private:

//    void checkFunctionIODim() {
//      // check function <-> template argument dimension
//      if(this->argSize!=0 && fmatvec::Helper<typename fmatvec::SymbolicFunction<Ret(Arg)>::ArgS>::size1(this->argS)!=this->argSize)
//        this->throwError("The dimension of the parameter does not match.");
//      if(this->retSize1!=0 && fmatvec::Helper<typename fmatvec::SymbolicFunction<Ret(Arg)>::RetS>::size1(this->retS)!=this->retSize1)
//        this->throwError("The output row dimension does not match.");
//      if(this->retSize2!=0 && fmatvec::Helper<typename fmatvec::SymbolicFunction<Ret(Arg)>::RetS>::size2(this->retS)!=this->retSize2)
//        this->throwError("The output column dimension does not match.");
//    }
  };

  template<typename Ret, typename Arg1, typename Arg2>
  class SymbolicFunction<Ret(Arg1, Arg2)> : public Function<Ret(Arg1, Arg2)>,
                                            public fmatvec::SymbolicFunction<Ret(Arg1, Arg2)> {

  public:
    SymbolicFunction() = default;

    void init(Element::InitStage stage, const InitConfigSet &config) override {
      Function<Ret(Arg1, Arg2)>::init(stage, config);
      if(stage == Element::preInit) {
        fmatvec::SymbolicFunction<Ret(Arg1, Arg2)>::init();
//        checkFunctionIODim();
      }
    }

    void initializeUsingXML(xercesc::DOMElement *element) override {
      Function<Ret(Arg1, Arg2)>::initializeUsingXML(element);

      xercesc::DOMElement *input=MBXMLUtils::E(MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"inputs"))->getFirstElementChildNamed(MBSIM%"input");
      this->setIndependentVariable1(typename fmatvec::SymbolicFunction<Ret(Arg1,Arg2)>::Arg1S(MBXMLUtils::E(input)->getText<std::string>().c_str()));
      input=input->getNextElementSibling();
      this->setIndependentVariable2(typename fmatvec::SymbolicFunction<Ret(Arg1,Arg2)>::Arg2S(MBXMLUtils::E(input)->getText<std::string>().c_str()));
      input=input->getNextElementSibling();
      if(input)
        throw MBXMLUtils::DOMEvalException("Exactly two input must be given.", MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"inputs"));

      xercesc::DOMElement *output=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"output");
      this->setDependentFunction(typename fmatvec::SymbolicFunction<Ret(Arg1,Arg2)>::RetS(MBXMLUtils::E(output)->getText<std::string>().c_str()));

      // check symbolic function arguments: we need to throw errors during initializeUsingXML to enable the ObjectFactory
      // to test other possible combinations (more general ones)
//      checkFunctionIODim();
    }

  private:

//    void checkFunctionIODim() {
//      // check function <-> template argument dimension
//      if(this->arg1Size!=0 && fmatvec::Helper<typename fmatvec::SymbolicFunction<Ret(Arg1,Arg2)>::Arg1S>::size1(this->arg1S)!=this->arg1Size)
//        this->throwError("The dimension of the parameter does not match.");
//      if(this->arg2Size!=0 && fmatvec::Helper<typename fmatvec::SymbolicFunction<Ret(Arg1,Arg2)>::Arg2S>::size1(this->arg2S)!=this->arg2Size)
//        this->throwError("The dimension of the parameter does not match.");
//      if(this->retSize1!=0 && fmatvec::Helper<typename fmatvec::SymbolicFunction<Ret(Arg1,Arg2)>::RetS>::size1(this->retS)!=this->retSize1)
//        this->throwError("The output row dimension does not match.");
//      if(this->retSize2!=0 && fmatvec::Helper<typename fmatvec::SymbolicFunction<Ret(Arg1,Arg2)>::RetS>::size2(this->retS)!=this->retSize2)
//        this->throwError("The output column dimension does not match.");
//    }
  };

}

#endif
