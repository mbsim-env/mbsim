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

#ifndef _LINEAR_FUNCTION_H_
#define _LINEAR_FUNCTION_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<typename Sig> class LinearFunction; 

  template<typename Ret, typename Arg>
  class LinearFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    private:
      double a0, a1;
    public:
      LinearFunction(double a1_=0) : a0(0), a1(a1_) { }
      LinearFunction(double a0_, double a1_) : a0(a0_), a1(a1_) { }
      typename fmatvec::Size<double>::type getArgSize() const { return 1; }
      Ret operator()(const Arg &x) { return FromDouble<Ret>::cast(a1*ToDouble<Arg>::cast(x)+a0); }
      typename fmatvec::Der<Ret, Arg>::type parDer(const Arg &x) { return FromDouble<Ret>::cast(a1); }
      typename fmatvec::Der<Ret, Arg>::type parDerDirDer(const Arg &xDir, const Arg &x) { return FromDouble<Ret>::cast(0); }
      typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &x) { return FromDouble<Ret>::cast(0); }
      void initializeUsingXML(xercesc::DOMElement *element) {
        xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"a0");
        if(e) a0=Element::getDouble(e);
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"a1");
        a1=Element::getDouble(e);
      }
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent) { return 0; } 
      void seta0(double a0_) { a0 = a0_; }
      void seta1(double a1_) { a1 = a1_; }
  };

}

#endif
