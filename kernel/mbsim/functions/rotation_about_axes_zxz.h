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

#ifndef _ROTATION_ABOUT_AXES_ZXZ_H_
#define _ROTATION_ABOUT_AXES_ZXZ_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<class Arg> 
  class RotationAboutAxesZXZ : public Function<fmatvec::RotMat3(Arg)> {
    private:
      fmatvec::RotMat3 A;
      fmatvec::Mat3xV J, Jd;
    public:
      RotationAboutAxesZXZ() : J(3), Jd(3) { J.e(2,0) = 1; }
      typename fmatvec::Size<Arg>::type getArgSize() const { return 3; }
      fmatvec::RotMat3 operator()(const Arg &q) {
        double psi=q.e(0);
        double theta=q.e(1);
        double phi=q.e(2);
        double spsi = sin(psi);
        double stheta = sin(theta);
        double sphi = sin(phi);
        double cpsi = cos(psi);
        double ctheta = cos(theta);
        double cphi = cos(phi);
        A.e(0,0) = cpsi*cphi-spsi*ctheta*sphi;
        A.e(1,0) = spsi*cphi+cpsi*ctheta*sphi;
        A.e(2,0) = stheta*sphi;
        A.e(0,1) = -cpsi*sphi-spsi*ctheta*cphi;
        A.e(1,1) = -spsi*sphi+cpsi*ctheta*cphi;
        A.e(2,1) = stheta*cphi;
        A.e(0,2) = spsi*stheta;
        A.e(1,2) = -cpsi*stheta;
        A.e(2,2) = ctheta;
        return A;
      }
      typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDer(const Arg &q) {
        double psi=q.e(0);
        double theta=q.e(1);
        //J.e(0,0) = 0;
        J.e(0,1) = cos(psi);
        J.e(0,2) = sin(psi)*sin(theta);
        //J.e(1,0) = 0;
        J.e(1,1) = sin(psi);
        J.e(1,2) = -cos(psi)*sin(theta);
        //J.e(2,0) = 1;
        //J.e(2,1) = 1;
        J.e(2,2) = cos(theta);
        return J;
      }
      typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDerDirDer(const Arg &qd, const Arg &q) {
        double psi=q.e(0);
        double theta=q.e(1);
        double psid=qd.e(0);
        double thetad=qd.e(1);
        Jd.e(0,1) = -sin(psi)*psid;
        Jd.e(0,2) = cos(psi)*sin(theta)*psid + sin(psi)*cos(theta)*thetad;
        Jd.e(1,1) = cos(psi)*psid;
        Jd.e(1,2) = sin(psi)*sin(theta)*psid - cos(psi)*cos(theta)*thetad;
        Jd.e(2,2) = -sin(theta)*thetad;
        return Jd;
      }
  };

}

#endif
