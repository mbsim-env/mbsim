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
 * Contact: mfoerg@users.berlios.de
 */

#include <config.h> 
#include "point_circlesolid.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/circle_solid.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsPointCircleSolid::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      icirclesolid = 1;
      point = static_cast<Point*>(contour[0]);
      circlesolid = static_cast<CircleSolid*>(contour[1]);
    } 
    else {
      ipoint = 1;
      icirclesolid = 0;
      point = static_cast<Point*>(contour[1]);
      circlesolid = static_cast<CircleSolid*>(contour[0]);
    }
  }

  void ContactKinematicsPointCircleSolid::updateg(Vec &g, ContourPointData *cpData) {
    const Vec WrD = -circlesolid->getFrame()->getPosition() + point->getFrame()->getPosition();
    
    cpData[icirclesolid].getFrameOfReference().getOrientation().col(0) = WrD/nrm2(WrD);
    cpData[ipoint].getFrameOfReference().getOrientation().col(0) = -cpData[icirclesolid].getFrameOfReference().getOrientation().col(0);
    
    cpData[icirclesolid].getFrameOfReference().getOrientation().col(2) = circlesolid->getFrame()->getOrientation().col(2);
    cpData[ipoint].getFrameOfReference().getOrientation().col(2) = point->getFrame()->getOrientation().col(2);
    
    cpData[icirclesolid].getFrameOfReference().getOrientation().col(1) = crossProduct(cpData[icirclesolid].getFrameOfReference().getOrientation().col(2), cpData[icirclesolid].getFrameOfReference().getOrientation().col(0));
    cpData[ipoint].getFrameOfReference().getOrientation().col(1) = -cpData[icirclesolid].getFrameOfReference().getOrientation().col(1);
    
    cpData[icirclesolid].getFrameOfReference().getPosition() = circlesolid->getFrame()->getPosition() + cpData[icirclesolid].getFrameOfReference().getOrientation().col(0)*circlesolid->getRadius();
    cpData[ipoint].getFrameOfReference().getPosition() = point->getFrame()->getPosition();

    g(0) = cpData[icirclesolid].getFrameOfReference().getOrientation().col(0).T()*WrD - circlesolid->getRadius();
  }

  void ContactKinematicsPointCircleSolid::updatewb(Vec &wb, const Vec &g, ContourPointData *cpData) {
    throw MBSimError("Sorry, not implemented yet.");
  }
}

