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
 * Contact: martin.o.foerg@googlemail.com
 *          rzander@users.berlios.de
 *          thschindler@users.berlios.de
 */

#include <config.h> 
#include "point_frustum.h"
#include "mbsim/contours/frustum.h"
#include "mbsim/contours/point.h"
#include "mbsim/utils/eps.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsPointFrustum::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      ifrustum = 1;
      point = static_cast<Point*>(contour[0]);
      frustum = static_cast<Frustum*>(contour[1]);
    }
    else {
      ipoint = 1;
      ifrustum = 0;
      point = static_cast<Point*>(contour[1]);
      frustum = static_cast<Frustum*>(contour[0]);
    }
  }

  void ContactKinematicsPointFrustum::updateg(Vec &g, ContourPointData* cpData) {
    double eps = 5.e-2; // tolerance for rough contact description
    Vec3 Wd = point->getFrame()->getPosition() - frustum->getFrame()->getPosition(); // difference vector of Point and Frustum basis point in inertial FR
    Vec3 Wa = frustum->getFrame()->getOrientation().col(1); // (height) axis in inertial FR
    Vec2 r = frustum->getRadii(); // radii of Frustum
    double h = frustum->getHeight(); // height of Frustum	    
    double s = Wd.T()*Wa; // projection of difference vector on axis
    double d = sqrt(pow(nrm2(Wd),2)-pow(s,2)); // distance Point to Frustum axis
    if(fabs(h)<epsroot())
      throw MBSimError("ERROR: Frustum with height = 0!");
    double r_h = r(0) + (r(1)-r(0))/h * s; // radius of Frustum at s
    bool outCont = frustum->getOutCont(); // contact on outer surface?


    if(s<0 || s>h  || d < (r_h-eps) || d > (r_h+eps))
      g(0) = 1.;
    else {
      double  phi = atan((r(1) - r(0))/h); // half cone angle
      if(outCont) { // contact on outer surface

        Vec3 b = Wd-s*Wa;
        b /= d;
        cpData[ifrustum].getFrameOfReference().getOrientation().set(0,  cos(phi)*b - sin(phi)*Wa);
        cpData[ipoint].getFrameOfReference().getOrientation().set(0, -cpData[ifrustum].getFrameOfReference().getOrientation().col(0));
        g(0) = (d-r_h)*cos(phi);
      }
      else { // contact on inner surface
        double  phi = atan((r(1) - r(0))/h); // half cone angle
        Vec3 b = Wd-s*Wa;
        b /= d;
        cpData[ifrustum].getFrameOfReference().getOrientation().set(0, sin(phi)*Wa - cos(phi)*b);
        cpData[ipoint].getFrameOfReference().getOrientation().set(0, -cpData[ifrustum].getFrameOfReference().getOrientation().col(0));
        g(0) = (r_h-d)*cos(phi);
      }

    //Set positions
    cpData[ipoint].getFrameOfReference().getPosition()= point->getFrame()->getPosition();
    cpData[ifrustum].getFrameOfReference().getPosition() =  cpData[ipoint].getFrameOfReference().getPosition() + cpData[ipoint].getFrameOfReference().getOrientation().col(0)*g;

    /*Set tangential (=friction) directions*/
    // radial direction
    if(outCont)
      cpData[ifrustum].getFrameOfReference().getOrientation().set(1, (Wa + sin(phi)*cpData[ifrustum].getFrameOfReference().getOrientation().col(0))/cos(phi));
    else
      cpData[ifrustum].getFrameOfReference().getOrientation().set(1, (Wa - sin(phi)*cpData[ifrustum].getFrameOfReference().getOrientation().col(0))/cos(phi));

    cpData[ipoint].getFrameOfReference().getOrientation().set(1, -cpData[ifrustum].getFrameOfReference().getOrientation().col(1));

    //azimuthal
    cpData[ifrustum].getFrameOfReference().getOrientation().set(2, crossProduct(cpData[ifrustum].getFrameOfReference().getOrientation().col(0),cpData[ifrustum].getFrameOfReference().getOrientation().col(1)));
    cpData[ipoint].getFrameOfReference().getOrientation().set(2, cpData[ifrustum].getFrameOfReference().getOrientation().col(2));



    }
  }

}

