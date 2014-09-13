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
#include <xercesc/util/XercesDefs.hpp>

namespace XERCES_CPP_NAMESPACE {
  class DOMNode;
  class DOMElement;
}

namespace {
  std::string uniqueDummyName(void *p) {
    std::stringstream str;
    str<<"Function_"<<p;
    return str.str();
  }
}

namespace MBSim {

  /*! This class is just to have a none template base class for all MBSim function classed. */
  class FunctionBase : public Element {
    public:
      //! Most Function's have no name hence use a unique dummy name for Element ctor
      FunctionBase() : Element(uniqueDummyName(this)) {}

      //! ctor variante with name
      FunctionBase(const std::string &name_) : Element(name_) {}
  };

  /*! Base Function object for MBSim.
   * Adds just some XML functionallity to the fmatvec::Function.
   * Also derives from FunctionBase (to have a none templated base for all functions) which is itself derived from Element. */
  template<typename Sig>
  class Function : public fmatvec::Function<Sig>, public FunctionBase {
    public:
      //! Most Function's have no name hence use a unique dummy name for Element ctor
      Function() : fmatvec::Function<Sig>(), FunctionBase() {}

      //! ctor variante with name
      Function(const std::string &name_) : fmatvec::Function<Sig>(), FunctionBase(name_) {}
  };

}

#endif
