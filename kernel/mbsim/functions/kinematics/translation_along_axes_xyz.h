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
 * Contact: martin.o.foerg@gmail.com
 */

#ifndef _TRANSLATION_ALONG_AXES_XYZ_H_
#define _TRANSLATION_ALONG_AXES_XYZ_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<typename Arg>
  class TranslationAlongAxesXYZ : public Function<fmatvec::Vec3(Arg)> {
    using B = fmatvec::Function<fmatvec::Vec3(Arg)>; 
    private:
      fmatvec::Vec3 r;
      fmatvec::Mat3xV A;
    public:
      TranslationAlongAxesXYZ() : A(3) { A.e(0,0) = 1; A.e(1,1) = 1; A.e(2,2) = 1; }
      int getArgSize() const override { return 3; }
      fmatvec::Vec3 operator()(const Arg &q) override { 
        r.e(0) = q.e(0);
        r.e(1) = q.e(1);
        r.e(2) = q.e(2);
        return r; 
      }
      typename B::DRetDArg parDer(const Arg &arg) override { return A; }
      typename B::DRetDArg parDerDirDer(const Arg &arg1Dir, const Arg &arg1) override { return typename B::DRetDArg(3); }
      bool constParDer() const override { return true; }
  };

}

#endif
