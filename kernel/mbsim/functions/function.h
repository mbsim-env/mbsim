/* Copyright (C) MBSim Development Team
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
 */

#ifndef _MBSIM_FUNCTION_H_
#define _MBSIM_FUNCTION_H_

#include "mbsim/element.h"
#include "fmatvec/function.h"
#include <string>
#include <xercesc/dom/DOMElement.hpp>
#include "mbsim/utils/initconfigenum.h"

namespace {
  inline std::string uniqueDummyName(void *p) {
    std::stringstream str;
    str<<"Function_"<<p;
    return str.str();
  }
}

namespace MBSim {

  extern const InitConfigEnum noDer;
  extern const InitConfigEnum noDerDer; // this implies noDer

  /*! This class is just to have a none template base class for all MBSim function classes. */
  class FunctionBase : public Element {
    public:
      //! Function's have no name hence use a unique dummy name for Element ctor.
      //! The function name is normally changed later by the corresponding setter methode which added the Function a another object.
      FunctionBase() : Element(uniqueDummyName(this)) { plotFeature[plotRecursive]=false; }
      virtual Element* getDependency() const { return nullptr; }
  };

  /*! Base Function object for MBSim.
   * Adds just some XML functionallity to the fmatvec::Function.
   * Also derives from FunctionBase (to have a none templated base for all functions) which is itself derived from Element. */
  template<typename Sig>
  class Function : public fmatvec::Function<Sig>, public FunctionBase {
    public:
      //! Most Function's have no name hence use a unique dummy name for Element ctor
      //! See also FunctionBase::FunctionBase.
      Function() : fmatvec::Function<Sig>(), FunctionBase() {}

      void initializeUsingXML(xercesc::DOMElement *element) override;
  };

  template<typename Sig>
  void Function<Sig>::initializeUsingXML(xercesc::DOMElement *element) {
    FunctionBase::initializeUsingXML(element);
    if(MBXMLUtils::E(element)->hasAttribute("name"))
      throw MBXMLUtils::DOMEvalException("No 'name' attribute allowed for Function's.", element);
  }

}

#endif
