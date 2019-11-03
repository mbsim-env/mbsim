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
      }
    }

//    void initializeUsingXML(xercesc::DOMElement *element) override {
//      auto io=casadi::createCasADiFunctionFromXML(element->getFirstElementChild());
//      arg=io.first[0];
//      ret=io.second[0];
//      // check symbolic function arguments: we need to throw errors during initializeUsingXML to enable the ObjectFactory
//      // to test other possible combinations (more general ones)
//      checkFunctionIODim();
//    }
//
//  private:
//
//    void checkFunctionIODim() {
//      // check function: only scalar and vector arguments are supported
//      if(arg.size2()!=1) this->throwError("Matrix parameters are not allowed.");
//      // check function <-> template argument dimension
//      if(this->argSize!=0 && arg.size1()!=this->argSize)
//        this->throwError("The dimension of the parameter does not match.");
//      if(this->retSize1!=0 && ret.size1()!=this->retSize1)
//        this->throwError("The output row dimension does not match.");
//      if(this->retSize2!=0 && ret.size2()!=this->retSize2)
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
      }
    }

//    void initializeUsingXML(xercesc::DOMElement *element) override {
//      auto io=casadi::createCasADiFunctionFromXML(element->getFirstElementChild());
//      arg=io.first[0];
//      ret=io.second[0];
//      // check symbolic function arguments: we need to throw errors during initializeUsingXML to enable the ObjectFactory
//      // to test other possible combinations (more general ones)
//      checkFunctionIODim();
//    }
//
//  private:
//
//    void checkFunctionIODim() {
//      // check function: only scalar and vector arguments are supported
//      if(arg.size2()!=1) this->throwError("Matrix parameters are not allowed.");
//      // check function <-> template argument dimension
//      if(this->argSize!=0 && arg.size1()!=this->argSize)
//        this->throwError("The dimension of the parameter does not match.");
//      if(this->retSize1!=0 && ret.size1()!=this->retSize1)
//        this->throwError("The output row dimension does not match.");
//      if(this->retSize2!=0 && ret.size2()!=this->retSize2)
//        this->throwError("The output column dimension does not match.");
//    }
  };

}

#endif
