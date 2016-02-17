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

#ifndef _ROTATION_ABOUT_AXES_XYZ_TRANSFORMED_H_
#define _ROTATION_ABOUT_AXES_XYZ_TRANSFORMED_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<class Arg> 
  class RotationAboutAxesXYZTransformed : public Function<fmatvec::RotMat3(Arg)> {
    private:
      fmatvec::RotMat3 A;
      fmatvec::Mat3xV J, Jd;
    public:
      RotationAboutAxesXYZTransformed() : J(3), Jd(3) { J.e(2,2) = 1; }
      typename fmatvec::Size<Arg>::type getArgSize() const { return 3; }
      fmatvec::RotMat3 operator()(const Arg &q) {
        double a=q.e(0);
        double b=q.e(1);
        double g=q.e(2);
        double cosa = cos(a);
        double sina = sin(a);
        double cosb = cos(b);
        double sinb = sin(b);
        double cosg = cos(g);
        double sing = sin(g);
        A.e(0,0) = cosb*cosg;
        A.e(1,0) = sina*sinb*cosg+cosa*sing;
        A.e(2,0) = -cosa*sinb*cosg+sina*sing;
        A.e(0,1) = -cosb*sing;
        A.e(1,1) = -sing*sinb*sina+cosa*cosg;
        A.e(2,1) = cosa*sinb*sing+sina*cosg;
        A.e(0,2) = sinb;
        A.e(1,2) = -sina*cosb;
        A.e(2,2) = cosa*cosb;
        return A;
      }
      typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDer(const Arg &q) {
        double b = q.e(1);
        double g = q.e(2);
        J.e(0,0) = cos(b)*cos(g);
        J.e(0,1) = sin(g);
        //J.e(0,2) = 0;
        J.e(1,0) = -cos(b)*sin(g);
        J.e(1,1) = cos(g);
        //J.e(1,2) = 0;
        J.e(2,0) = sin(b);
        //J.e(2,1) = 0;
        //J.e(2,2) = 1;
        return J;
      }
      typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDerDirDer(const Arg &qd, const Arg &q) {
        double b = q.e(1);
        double g = q.e(2);
        double bd = qd.e(1);
        double gd = qd.e(2);
        Jd.e(0,0) = -sin(b)*cos(g)*bd - cos(b)*sin(g)*gd;
        Jd.e(0,1) = cos(g)*gd;
        //Jd.e(0,2) = 0;
        Jd.e(1,0) = sin(b)*sin(g)*bd - cos(b)*cos(g)*gd;
        Jd.e(1,1) = -sin(g)*gd;
        //Jd.e(1,2) = 0; 
        Jd.e(2,0) = cos(b)*bd;
        //Jd.e(2,1) = 0;
        //Jd.e(2,2) = 0;
        return Jd;
      }
  };

}

#endif
