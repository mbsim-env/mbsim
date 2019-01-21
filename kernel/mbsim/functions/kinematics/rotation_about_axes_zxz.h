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

#include "mbsim/functions/kinematics/rotation_about_three_axes.h"
#include "mbsim/functions/kinematics/rotation_about_axes_zxz_mapping.h"
#include "mbsim/functions/kinematics/rotation_about_axes_zxz_transformed_mapping.h"

namespace MBSim {

  template<class Arg> 
  class RotationAboutAxesZXZ : public RotationAboutThreeAxes<Arg> {
    using B = fmatvec::Function<fmatvec::RotMat3(Arg)>; 
    using RotationAboutThreeAxes<Arg>::A;
    using RotationAboutThreeAxes<Arg>::J;
    using RotationAboutThreeAxes<Arg>::Jd;
    public:
      RotationAboutAxesZXZ() { J.e(2,0) = 1; }
      fmatvec::RotMat3 operator()(const Arg &q) override {
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
      typename B::DRetDArg parDer(const Arg &q) override {
        double psi=q.e(0);
        double theta=q.e(1);
        double sint = sin(theta);
        if(fabs(sint)<=1e-13)
          Element::throwError("Singularity in rotation.");
        double cosp = cos(psi);
        double sinp = sin(psi);
        //J.e(0,0) = 0;
        J.e(0,1) = cosp;
        J.e(0,2) = sinp*sint;
        //J.e(1,0) = 0;
        J.e(1,1) = sinp;
        J.e(1,2) = -cosp*sint;
        //J.e(2,0) = 1;
        //J.e(2,1) = 0;
        J.e(2,2) = cos(theta);
        return J;
      }
      typename B::DRetDArg parDerDirDer(const Arg &qd, const Arg &q) override {
        double psi=q.e(0);
        double theta=q.e(1);
        double psid=qd.e(0);
        double thetad=qd.e(1);
        double sint = sin(theta);
        double cost = cos(theta);
        double cosp = cos(psi);
        double sinp = sin(psi);
        Jd.e(0,1) = -sinp*psid;
        Jd.e(0,2) = cosp*sint*psid + sinp*cost*thetad;
        Jd.e(1,1) = cosp*psid;
        Jd.e(1,2) = sinp*sint*psid - cosp*cost*thetad;
        Jd.e(2,2) = -sint*thetad;
        return Jd;
      }
      Function<fmatvec::MatV(Arg)>* getMappingFunction() const override { return new RotationAboutAxesZXZMapping<Arg>; }
      Function<fmatvec::MatV(Arg)>* getTransformedMappingFunction() const override { return new RotationAboutAxesZXZTransformedMapping<Arg>; }
  };

}

#endif
