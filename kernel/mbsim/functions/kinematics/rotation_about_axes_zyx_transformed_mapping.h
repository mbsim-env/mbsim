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

#ifndef _ROTATION_ABOUT_AXES_ZYX_TRANSFORMED_MAPPING_H_
#define _ROTATION_ABOUT_AXES_ZYX_TRANSFORMED_MAPPING_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<class Arg> 
  class RotationAboutAxesZYXTransformedMapping : public Function<fmatvec::MatV(Arg)> {
    private:
      fmatvec::MatV T;
    public:
      RotationAboutAxesZYXTransformedMapping() : T(3,3) { T.e(2,0) = 1; }
      int getArgSize() const override { return 3; }
      fmatvec::MatV operator()(const Arg &q) override {
        double beta = q.e(1);
        double gamma = q.e(2);
        double cos_beta = cos(beta);
        if(fabs(cos_beta)<=1e-13)
          Element::throwError("Singularity in rotation.");
        double sin_beta = sin(beta);
        double cos_gamma = cos(gamma);
        double sin_gamma = sin(gamma);
        double tan_beta = sin_beta/cos_beta;
        T.e(0,1) = sin_gamma/cos_beta;
        T.e(0,2) = cos_gamma/cos_beta;
        T.e(1,1) = cos_gamma;
        T.e(1,2) = -sin_gamma;
        T.e(2,1) = tan_beta*sin_gamma;
        T.e(2,2) = tan_beta*cos_gamma;
        return T;
      }
  };

}

#endif
