/* Copyright (C) 2004-2020 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/functions/contact/funcpair_spatialcontour_spatialcontour.h"
#include "mbsim/frames/frame.h"
#include "mbsim/contours/contour.h"

using namespace fmatvec;

namespace MBSim {

  Vec FuncPairSpatialContourSpatialContour::operator()(const Vec &zeta) {
    Vec3 Wu1 = contour1->evalWu(zeta(I1));
    Vec3 Wv1 = contour1->evalWv(zeta(I1));
    Vec3 Wn1 = contour1->evalWn(zeta(I1));
    Vec3 Wu2 = contour2->evalWu(zeta(I2));
    Vec3 Wv2 = contour2->evalWv(zeta(I2));
    Vec3 WrD = contour2->evalPosition(zeta(I2)) - contour1->evalPosition(zeta(I1));
    Vec res(4,NONINIT);
    res(0) = Wu1.T() * WrD;
    res(1) = Wv1.T() * WrD;
    res(2) = Wn1.T() * Wu2;
    res(3) = Wn1.T() * Wv2;
    return res;
  }

}
