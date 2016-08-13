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

#ifndef _ROTATION_ABOUT_AXES_ZYX_H_
#define _ROTATION_ABOUT_AXES_ZYX_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<class Arg> 
  class RotationAboutAxesZYX : public Function<fmatvec::RotMat3(Arg)> {
    using B = fmatvec::Function<fmatvec::RotMat3(Arg)>; 
    private:
      fmatvec::RotMat3 A;
      fmatvec::Mat3xV J, Jd;
    public:
      RotationAboutAxesZYX() : J(3), Jd(3) { J.e(2,0) = 1; }
      int getArgSize() const { return 3; }
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
        A.e(0,0) = cosa*cosb;
        A.e(1,0) = sina*cosb;
        A.e(2,0) = -sinb;
        A.e(0,1) = -sina*cosg+cosa*sinb*sing;
        A.e(1,1) = cosa*cosg+sina*sinb*sing;
        A.e(2,1) = cosb*sing;
        A.e(0,2) = sina*sing+cosa*sinb*cosg;
        A.e(1,2) = -cosa*sing+sina*sinb*cosg;
        A.e(2,2) = cosb*cosg;
        return A;
      }
      typename B::DRetDArg parDer(const Arg &q) {
        double a = q.e(0);
        double b = q.e(1);
        double cosa = cos(a);
        double sina = sin(a);
        double cosb = cos(b);
        //J.e(0,0) = 0;
        J.e(0,1) = -sina;
        J.e(0,2) = cosa*cosb;
        //J.e(1,0) = 0;
        J.e(1,1) = cosa;
        J.e(1,2) = sina*cosb;
        //J.e(2,0) = 1;
        //J.e(2,1) = 0;
        J.e(2,2) = -sin(b);
        return J;
      }
      typename B::DRetDArg parDerDirDer(const Arg &qd, const Arg &q) {
        double a = q.e(0);
        double b = q.e(1);
        double ad = qd.e(0);
        double bd = qd.e(1);
        double cosa = cos(a);
        double sina = sin(a);
        double cosb = cos(b);
        double sinb = sin(b);
        Jd.e(0,1) = -cosa*ad;
        Jd.e(0,2) = -sina*cosb*ad - cosa*sinb*bd;
        Jd.e(1,1) = -sina*ad;
        Jd.e(1,2) = cosa*cosb*ad - sina*sinb*bd;
        Jd.e(2,2) = -cosb*bd;
        return Jd;
      }
  };

}

#endif
