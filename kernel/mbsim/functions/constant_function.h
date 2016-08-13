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

#ifndef _CONSTANT_FUNCTION_H_
#define _CONSTANT_FUNCTION_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<typename Sig> class ConstantFunction; 

  template<typename Ret, typename Arg>
  class ConstantFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    using B = fmatvec::Function<Ret(Arg)>; 
    protected:
      double a0;
    public:
      ConstantFunction(double a0_=0) : a0(a0_) {}
      int getArgSize() const { return 1; }
      Ret operator()(const Arg &x) { return FromDouble<Ret>::cast(a0); }
      typename B::DRetDArg parDer(const Arg &x) { return FromDouble<Ret>::cast(0); }
      typename B::DRetDArg parDerDirDer(const Arg &xDir, const Arg &x) { return FromDouble<Ret>::cast(0); }
      Ret parDerParDer(const double &x) { return FromDouble<Ret>::cast(0); }
      void initializeUsingXML(xercesc::DOMElement *element) {
        xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"a0");
        a0=Element::getDouble(e);
      }
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent) { return 0; }
  };

}

#endif
