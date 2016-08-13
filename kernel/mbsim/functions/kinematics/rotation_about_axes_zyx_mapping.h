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

#ifndef _ROTATION_ABOUT_AXES_ZYX_MAPPING_H_
#define _ROTATION_ABOUT_AXES_ZYX_MAPPING_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<class Arg> 
  class RotationAboutAxesZYXMapping : public Function<fmatvec::MatV(Arg)> {
    private:
      fmatvec::MatV T;
    public:
      RotationAboutAxesZYXMapping() : T(3,3) { T.e(0,2) = 1; }
      int getArgSize() const { return 3; }
      fmatvec::MatV operator()(const Arg &q) {
        double alpha = q.e(0);
        double beta = q.e(1);
        double cos_beta = cos(beta);
        double sin_beta = sin(beta);
        double cos_alpha = cos(alpha);
        double sin_alpha = sin(alpha);
        double tan_beta = sin_beta/cos_beta;
        T.e(0,0) = cos_alpha*tan_beta;
        T.e(0,1) = sin_alpha*tan_beta;
        T.e(1,0) = -sin_alpha;
        T.e(1,1) = cos_alpha;
        T.e(2,0) = cos_alpha/cos_beta;
        T.e(2,1) = sin_alpha/cos_beta;
        return T;
      }
  };

}

#endif
