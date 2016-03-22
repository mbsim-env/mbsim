/* Copyright (C) 2004-2010 MBSim Development Team
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
#include "mbsimFlexibleBody/functions_contact.h"
#include "mbsim/contours/circle.h"
#include "mbsimFlexibleBody/contours/nurbs_disk_2s.h"

using namespace fmatvec;

namespace MBSimFlexibleBody {

  double FuncPairCircleNurbsDisk2s::operator()(const double &alpha) {
    //Parameters of the AWK of the nurbs disk and the circle
            SqrMat3 AWK_disk   = nurbsdisk->getOrientation(t);
            Vec3 r_disk   = nurbsdisk->getPosition(t);
            SqrMat3 AWK_circle = circle->getFrame()->getOrientation(t);
    
            //Point on the Circle
            Vec3 WP_circle(INIT,0.);  //world-coordinates of the point on the circle
            WP_circle(0) = cos(alpha);
            WP_circle(1) = sin(alpha);
            WP_circle = circle->getFrame()->getPosition() + circle->getRadius() * AWK_circle * WP_circle;
    
            //derivatives of a point on the circle in world-coordinates with respect to the circle-parameter alpha
            Vec dWP_circle(3,INIT, 0.);
            dWP_circle(0) = -sin(alpha);
            dWP_circle(1) = cos(alpha);
            Vec circle_tangent = circle->getRadius() * AWK_circle * dWP_circle; //not normalised tangent on the circle
    
            //compute radial and azimuthal nurbsdisk-coordinates out of alpha (saved in the LagrangeParameterPosition)
//            MBSim::ContourPointData cp_nurbsdisk;
            Vec2 zeta = nurbsdisk->transformCW(t, AWK_disk.T() * (WP_circle - r_disk) )(0,1); // position of the point in the cylinder-coordinates of the disk
    
            //compute the derivates of the radial and the azimuthal coordinates with respect to alpha
            SqrMat A_inv(3,EYE);
            A_inv(0,0)=  cos(zeta(1));
            A_inv(0,1)=  sin(zeta(1));
            A_inv(1,0)= -sin(zeta(1)) / zeta(0);
            A_inv(1,1)=  cos(zeta(1)) / zeta(0);
            Vec drphidalpha = A_inv * AWK_disk.T()* circle_tangent; // AWK_disk * A_inv * trans(AWK_disk)* circle_tangent CHANGED
    
            //compution of the single elements in the function
            Vec nurbs_radial_tangent    = nurbsdisk->getWu(t,zeta);
            Vec nurbs_azimuthal_tangent = nurbsdisk->getWv(t,zeta);
    
            return AWK_disk.col(2).T() * (circle_tangent - (nurbs_radial_tangent *  drphidalpha(0)+ nurbs_azimuthal_tangent * drphidalpha(1)));
  }

  Vec3 FuncPairCircleNurbsDisk2s::getWrD(const double &alpha) {
    //point on the circle
    Vec WP_circle(3,INIT,0.);
    WP_circle(0) = cos(alpha);
    WP_circle(1) = sin(alpha);
    WP_circle = circle->getFrame()->getPosition(t) + circle->getRadius() * circle->getFrame()->getOrientation(t) * WP_circle;

    //get the position on the nurbsdisk
    Vec2 zeta = nurbsdisk->transformCW(t, nurbsdisk->getOrientation(t).T()*(WP_circle - nurbsdisk->getPosition(t)))(0,1); // position of the point in the cylinder-coordinates of the disk
    Vec WP_nurbsdisk = nurbsdisk->getPosition(t,zeta);

    return WP_circle - WP_nurbsdisk;
  }

}
