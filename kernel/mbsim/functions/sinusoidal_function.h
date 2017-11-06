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

#ifndef _SINUSOIDAL_FUNCTION_H_
#define _SINUSOIDAL_FUNCTION_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<typename Sig> class SinusoidalFunction; 

  template<typename Ret, typename Arg>
  class SinusoidalFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    using B = fmatvec::Function<Ret(Arg)>; 
    protected:
      double A, f, phi0, y0;
    public:
      SinusoidalFunction(double A_=0, double f_=0, double phi0_=0, double y0_=0) : A(A_), f(f_), phi0(phi0_), y0(y0_) { }
      int getArgSize() const { return 1; }
      std::pair<int, int> getRetSize() const { return std::make_pair(1,1); }
      Ret operator()(const Arg &x) {  
        return FromDouble<Ret>::cast(y0+A*sin(2.*M_PI*f*ToDouble<Arg>::cast(x)+phi0));
      }
      typename B::DRetDArg parDer(const Arg &x) {  
        double om = 2.*M_PI*f;
        return FromDouble<Ret>::cast(A*om*cos(om*ToDouble<Arg>::cast(x)+phi0));
      }
      typename B::DRetDArg parDerDirDer(const Arg &xDir, const Arg &x) { 
        double om = 2.*M_PI*f;
        return FromDouble<Ret>::cast(-A*om*om*sin(om*ToDouble<Arg>::cast(x)+phi0)*ToDouble<Arg>::cast(xDir));
      }
      Ret parDerParDer(const double &x) {  
        double om = 2.*M_PI*f;
        return FromDouble<Ret>::cast(-A*om*om*sin(om*x+phi0));
      }
      void initializeUsingXML(xercesc::DOMElement *element) {
        xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"amplitude");
        A=MBXMLUtils::E(e)->getText<double>();
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"frequency");
        f=MBXMLUtils::E(e)->getText<double>();
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"phase");
        if(e) phi0=MBXMLUtils::E(e)->getText<double>();
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"offset");
        if(e) y0=MBXMLUtils::E(e)->getText<double>();
      }
  };

}

#endif
