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

#ifndef _ROTATION_ABOUT_AXES_XZ_H_
#define _ROTATION_ABOUT_AXES_XZ_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<class Arg> 
  class RotationAboutAxesXZ : public Function<fmatvec::RotMat3(Arg)> {
    using B = fmatvec::Function<fmatvec::RotMat3(Arg)>; 
    private:
      fmatvec::RotMat3 A;
      fmatvec::Mat3xV J, Jd;
    public:
      RotationAboutAxesXZ() : J(2), Jd(2) { J.e(0,0) = 1; }
      int getArgSize() const { return 2; }
      fmatvec::RotMat3 operator()(const Arg &q) {
        double a=q.e(0);
        double b=q.e(1);
        double cosa = cos(a);
        double sina = sin(a);
        double cosb = cos(b);
        double sinb = sin(b);

        A.e(0,0) = cosb;
        A.e(1,0) = cosa*sinb;
        A.e(2,0) = sina*sinb;
        A.e(0,1) = -sinb;
        A.e(1,1) = cosa*cosb;
        A.e(2,1) = sina*cosb;
        A.e(1,2) = -sina;
        A.e(2,2) = cosa;

        return A;
      }
      typename B::DRetDArg parDer(const Arg &q) {
        double a = q.e(0);
        J.e(1,1) = -sin(a);
        J.e(2,1) = cos(a);
        return J;
      }
      typename B::DRetDArg parDerDirDer(const Arg &qd, const Arg &q) {
        double a = q.e(0);
        double ad = qd.e(0);
        Jd.e(1,1) = -cos(a)*ad;
        Jd.e(2,1) = -sin(a)*ad;
        return Jd;
      }
  };

}

#endif
