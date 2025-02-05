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
 */

#include <config.h> 
#include "point_frustum.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/frustum.h"
#include "mbsim/contours/point.h"
#include "mbsim/utils/eps.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsPointFrustum::assignContours(const vector<Contour*> &contour) {
    ContactKinematics::assignContours(contour);
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

  void ContactKinematicsPointFrustum::updateg(SingleContact &contact, int i) {
    double eps = 5.e-2; // tolerance for rough contact description
    Vec3 Wd = point->getFrame()->evalPosition() - frustum->getFrame()->evalPosition(); // difference vector of Point and Frustum basis point in inertial FR
    Vec3 Wa = frustum->getFrame()->evalOrientation().col(1); // (height) axis in inertial FR
    Vec2 r = frustum->getRadii(); // radii of Frustum
    double h = frustum->getHeight(); // height of Frustum	    
    double s = Wd.T()*Wa; // projection of difference vector on axis
    double d = sqrt(pow(nrm2(Wd),2)-pow(s,2)); // distance Point to Frustum axis
    if(fabs(h)<epsroot)
      throw runtime_error("Frustum with height = 0!");
    double r_h = r(0) + (r(1)-r(0))/h * s; // radius of Frustum at s
    bool outCont = frustum->getOutCont(); // contact on outer surface?

    double g;
    if(s<0 || s>h  || d < (r_h-eps) || d > (r_h+eps))
      g = 1.;
    else {
      double  phi = atan((r(1) - r(0))/h); // half cone angle
      if(outCont) { // contact on outer surface

        Vec3 b = Wd-s*Wa;
        b /= d;
        contact.getContourFrame(ifrustum)->getOrientation(false).set(0,  cos(phi)*b - sin(phi)*Wa);
        contact.getContourFrame(ipoint)->getOrientation(false).set(0, -contact.getContourFrame(ifrustum)->getOrientation(false).col(0));
        g = (d-r_h)*cos(phi);
      }
      else { // contact on inner surface
        double  phi = atan((r(1) - r(0))/h); // half cone angle
        Vec3 b = Wd-s*Wa;
        b /= d;
        contact.getContourFrame(ifrustum)->getOrientation(false).set(0, sin(phi)*Wa - cos(phi)*b);
        contact.getContourFrame(ipoint)->getOrientation(false).set(0, -contact.getContourFrame(ifrustum)->getOrientation(false).col(0));
        g = (r_h-d)*cos(phi);
      }

      //Set positions
      contact.getContourFrame(ipoint)->setPosition(point->getFrame()->getPosition());
      contact.getContourFrame(ifrustum)->setPosition(contact.getContourFrame(ipoint)->getPosition(false) + contact.getContourFrame(ipoint)->getOrientation(false).col(0)*g);

      /*Set tangential (=friction) directions*/
      // radial direction
      if(outCont)
        contact.getContourFrame(ifrustum)->getOrientation(false).set(1, (Wa + sin(phi)*contact.getContourFrame(ifrustum)->getOrientation(false).col(0))/cos(phi));
      else
        contact.getContourFrame(ifrustum)->getOrientation(false).set(1, (Wa - sin(phi)*contact.getContourFrame(ifrustum)->getOrientation(false).col(0))/cos(phi));

      contact.getContourFrame(ipoint)->getOrientation(false).set(1, -contact.getContourFrame(ifrustum)->getOrientation(false).col(1));

      //azimuthal
      contact.getContourFrame(ifrustum)->getOrientation(false).set(2, crossProduct(contact.getContourFrame(ifrustum)->getOrientation(false).col(0),contact.getContourFrame(ifrustum)->getOrientation(false).col(1)));
      contact.getContourFrame(ipoint)->getOrientation(false).set(2, contact.getContourFrame(ifrustum)->getOrientation(false).col(2));
    }
    contact.getGeneralizedRelativePosition(false)(0) = g;
  }

}

