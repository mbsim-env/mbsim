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

#ifndef _ROTATION_ABOUT_AXES_ZY_H_
#define _ROTATION_ABOUT_AXES_ZY_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<class Arg> 
  class RotationAboutAxesZY : public Function<fmatvec::RotMat3(Arg)> {
    using B = fmatvec::Function<fmatvec::RotMat3(Arg)>; 
    private:
      fmatvec::RotMat3 A;
      fmatvec::Mat3xV J, Jd;
    public:
      RotationAboutAxesZY() : J(2), Jd(2) { J.e(2,1) = 1; }
      int getArgSize() const override { return 2; }
      fmatvec::RotMat3 operator()(const Arg &q) override {
        double b=q.e(0);
        double g=q.e(1);
        double cosb = cos(b);
        double cosg = cos(g);
        double sing = sin(g);
        double sinb = sin(b);

        A.e(0,0) = cosb*cosg;
        A.e(0,1) = -sing;
        A.e(0,2) = cosg*sinb;
        A.e(1,0) = cosb*sing;
        A.e(1,1) = cosg;
        A.e(1,2) = sing*sinb;
        A.e(2,0) = -sinb;
        A.e(2,2) = cosb;
        return A;
      }
      typename B::DRetDArg parDer(const Arg &q) override {
        double g = q.e(1);
        J.e(0,0) = -sin(g);
        J.e(1,0) = cos(g) ;
        return J;
      }
      typename B::DRetDArg parDerDirDer(const Arg &qd, const Arg &q) override {
        double g = q.e(1);
        double gd = qd.e(1);
        Jd.e(0,0) = -gd*cos(g);
        Jd.e(1,0) = -gd*sin(g);
        return Jd;
      }
  };

}

#endif
