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

#ifndef _IDENTITY_FUNCTION_H_
#define _IDENTITY_FUNCTION_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<typename Sig> class IdentityFunction; 

  template<typename Ret, typename Arg>
  class IdentityFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    public:
      typename fmatvec::Size<double>::type getArgSize() const { return 1; }
      Ret operator()(const Arg &x) { return FromDouble<Ret>::cast(ToDouble<Arg>::cast(x)); }
      typename fmatvec::Der<Ret, Arg>::type parDer(const Arg &x) { return FromDouble<Ret>::cast(1); }
      typename fmatvec::Der<Ret, Arg>::type parDerDirDer(const Arg &xDir, const Arg &x) { return FromDouble<Ret>::cast(0); }
      typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDerParDer(const double &x) { return FromDouble<Ret>::cast(0); }
  };

}

#endif