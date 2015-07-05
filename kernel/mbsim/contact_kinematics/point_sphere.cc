/* Copyright (C) 2004-2013 MBSim Development Team
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
#include "point_sphere.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/utils/eps.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsPointSphere::assignContours(const vector<Contour*> &contour) {
    ipoint = 0; isphere = 1;
    point = static_cast<Point*>(contour[0]);
    sphere = static_cast<Sphere*>(contour[1]);
  }

  void ContactKinematicsPointSphere::updateg(double t, double &g, ContourPointData *cpData, int index) {
    Vec3 Wd = sphere->getFrame()->getPosition(t) - point->getFrame()->getPosition(t);
    double l = nrm2(Wd);
    Wd = Wd/l;
    g = l-sphere->getRadius();
    Vec3 t_;
    if(fabs(Wd(0))<epsroot() && fabs(Wd(1))<epsroot()) {
      t_(0) = 1.;
      t_(1) = 0.;
      t_(2) = 0.;
    }
    else {
      t_(0) = -Wd(1);
      t_(1) = Wd(0);
      t_(2) = 0.0;
    }
    t_ = t_/nrm2(t_);
    cpData[ipoint].getFrameOfReference().getOrientation(false).set(0, Wd);
    cpData[isphere].getFrameOfReference().getOrientation(false).set(0, -cpData[ipoint].getFrameOfReference().getOrientation(false).col(0));
    cpData[ipoint].getFrameOfReference().getOrientation(false).set(1, t_);
    cpData[isphere].getFrameOfReference().getOrientation(false).set(1, -cpData[ipoint].getFrameOfReference().getOrientation(false).col(1));
    cpData[ipoint].getFrameOfReference().getOrientation(false).set(2, crossProduct(Wd,t_));
    cpData[isphere].getFrameOfReference().getOrientation(false).set(2, cpData[ipoint].getFrameOfReference().getOrientation(false).col(2));
    cpData[ipoint].getFrameOfReference().setPosition(point->getFrame()->getPosition());
    cpData[isphere].getFrameOfReference().setPosition(sphere->getFrame()->getPosition() - sphere->getRadius() * Wd);
  }

}
