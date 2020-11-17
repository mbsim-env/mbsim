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
#include "mbsim/functions/contact/funcpair_spatialcontour_circle.h"
#include "mbsim/frames/frame.h"
#include "mbsim/contours/contour.h"
#include "mbsim/contours/circle.h"

using namespace fmatvec;

namespace MBSim {

  Vec FuncPairSpatialContourCircle::operator()(const Vec &alpha) {
    Vec3 Wd = evalWrD(alpha);
    Vec3 Wu = contour->evalWu(alpha);
    Vec3 Wv = contour->evalWv(alpha);
    Vec2 Wt(NONINIT);
    Wt(0) = Wu.T() * Wd;
    Wt(1) = Wv.T() * Wd;
    return Wt;
  }

  Vec3 FuncPairSpatialContourCircle::evalWrD(const Vec &alpha) {
    Vec3 Wn = contour->evalWn(alpha);
    Vec3 Wb = circle->getFrame()->evalOrientation().col(2);
    double t_EC = Wn.T()*Wb;
    if(t_EC>0) {
      Wb *= -1.;
      t_EC *= -1;	
    }
    Vec3 z_EC = Wn - t_EC*Wb;
    double z_EC_nrm2 = nrm2(z_EC);
    Vec3 WrD;
    if(z_EC_nrm2 <= 1e-8)
      WrD = circle->getFrame()->getPosition() - contour->evalPosition(alpha);
    else
      WrD = (circle->getFrame()->getPosition() - (circle->getRadius()/z_EC_nrm2)*z_EC) - contour->evalPosition(alpha);
    return WrD;
  }

}
