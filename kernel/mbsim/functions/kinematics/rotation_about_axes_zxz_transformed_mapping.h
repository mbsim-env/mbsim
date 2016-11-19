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

#ifndef _ROTATION_ABOUT_AXES_ZXZ_TRANSFORMED_MAPPING_H_
#define _ROTATION_ABOUT_AXES_ZXZ_TRANSFORMED_MAPPING_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<class Arg> 
  class RotationAboutAxesZXZTransformedMapping : public Function<fmatvec::MatV(Arg)> {
    private:
      fmatvec::MatV T;
    public:
      RotationAboutAxesZXZTransformedMapping() : T(3,3,fmatvec::Eye()) { }
      int getArgSize() const { return 3; }
      fmatvec::MatV operator()(const Arg &q) {
        double theta = q.e(1);
        double phi = q.e(2);
        double cos_theta = cos(theta);
        double sin_theta = sin(theta);
        double cos_phi = cos(phi);
        double sin_phi = sin(phi);
        double tan_theta = sin_theta/cos_theta;

        T.e(0,0) = sin_phi/sin_theta;
        T.e(0,1) = cos_phi/sin_theta;
        T.e(1,0) = cos_phi;
        T.e(1,1) = -sin_phi;
        T.e(2,0) = -sin_phi/tan_theta;
        T.e(2,1) = -cos_phi/tan_theta;

        return T;
      }
  };

}

#endif
