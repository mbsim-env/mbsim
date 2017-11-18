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

#ifndef _ROTATION_ABOUT_Y_AXIS_H_
#define _ROTATION_ABOUT_Y_AXIS_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<class Arg> 
  class RotationAboutYAxis : public Function<fmatvec::RotMat3(Arg)> {
    using B = fmatvec::Function<fmatvec::RotMat3(Arg)>; 
    private:
      fmatvec::RotMat3 A;
      fmatvec::Vec3 a;
    public:
      RotationAboutYAxis() { a.e(1) = 1; A.e(1,1) = 1; }
      int getArgSize() const override { return 1; }
      fmatvec::RotMat3 operator()(const Arg &q) override {
        double alpha = ToDouble<Arg>::cast(q);
        const double cosq=cos(alpha);
        const double sinq=sin(alpha);
        A.e(0,0) = cosq;
        A.e(2,0) = -sinq;
        A.e(0,2) = sinq;
        A.e(2,2) = cosq;
        return A;
      }
      typename B::DRetDArg parDer(const Arg &q) override { return a; }
      typename B::DRetDArg parDerDirDer(const Arg &qd, const Arg &q) override { return typename B::DRetDArg(1); }
      bool constParDer() const override { return true; }
  };

}

#endif
