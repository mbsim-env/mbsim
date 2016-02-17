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

#ifndef _ROTATION_ABOUT_AXES_YZ_H_
#define _ROTATION_ABOUT_AXES_YZ_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<class Arg> 
  class RotationAboutAxesYZ : public Function<fmatvec::RotMat3(Arg)> {
    private:
      fmatvec::RotMat3 A;
      fmatvec::Mat3xV J, Jd;
    public:
      RotationAboutAxesYZ() : J(2), Jd(2) { J.e(1,0) = 1; }
      typename fmatvec::Size<Arg>::type getArgSize() const { return 2; }
      fmatvec::RotMat3 operator()(const Arg &q) {
        double b=q.e(0);
        double g=q.e(1);
        double cosb = cos(b);
        double sinb = sin(b);
        double cosg = cos(g);
        double sing = sin(g);

        A.e(0,0) = cosb*cosg;
        A.e(1,0) = sing;
        A.e(2,0) = -sinb*cosg;
        A.e(0,1) = -cosb*sing;
        A.e(1,1) = cosg;
        A.e(2,1) = sinb*sing;
        A.e(0,2) = sinb;
        A.e(2,2) = cosb;
        return A;
      }
      typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDer(const Arg &q) {
        double beta = q.e(0);
        J.e(0,1) = sin(beta);
        J.e(2,1) = cos(beta);
        return J;
      }
      typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDerDirDer(const Arg &qd, const Arg &q) {
        double beta = q.e(0);
        double betad = qd.e(0);
        Jd.e(0,1) = cos(beta)*betad;
        Jd.e(2,1) = -sin(beta)*betad;
        return Jd;
      }
  };

}

#endif
