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

namespace MBSim {

  /*! Base Function object for MBSim.
   * Adds just some XML functionallity to the base fmatvec::Function. */
  template<typename Sig>
  class Function : public fmatvec::Function<Sig>, public Element {
    public:
      //! Function have no name hence use "dummy" for Element ctor
      Function() : fmatvec::Function<Sig>(), Element("dummy") {}
  };

}

#endif
