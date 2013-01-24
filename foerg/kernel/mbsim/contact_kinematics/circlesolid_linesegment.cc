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
 */

#include <config.h> 
#include "circlesolid_linesegment.h"
#include "mbsim/contours/line_segment.h"
#include "mbsim/contours/circle_solid.h"


using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsCircleSolidLineSegment::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<CircleSolid*>(contour[0])) {
      icircle = 0; iline = 1;
      circlesolid = static_cast<CircleSolid*>(contour[0]);
      linesegment = static_cast<LineSegment*>(contour[1]);
    } 
    else {
      icircle = 1; iline = 0;
      circlesolid = static_cast<CircleSolid*>(contour[1]);
      linesegment = static_cast<LineSegment*>(contour[0]);
    }
  }

  void ContactKinematicsCircleSolidLineSegment::updateg(Vec &g, ContourPointData *cpData) {

    const Vec3 WC=circlesolid->getFrame()->getPosition();
    const Vec3 WL=linesegment->getFrame()->getPosition();
    const Vec3 WLdir=linesegment->getFrame()->getOrientation().col(1);
    const Vec3 WL0=WL+linesegment->getBounds()(0)*WLdir;
    const double s=WLdir.T() * (-WL0+WC);

    if ((s>=0) && (s<=linesegment->getSegmentLength())) {
      cpData[iline].getFrameOfReference().setOrientation(linesegment->getFrame()->getOrientation());
      cpData[icircle].getFrameOfReference().getOrientation().set(0, -linesegment->getFrame()->getOrientation().col(0));
      cpData[icircle].getFrameOfReference().getOrientation().set(1, -linesegment->getFrame()->getOrientation().col(1));
      cpData[icircle].getFrameOfReference().getOrientation().set(2, linesegment->getFrame()->getOrientation().col(2));
      g(0) = cpData[iline].getFrameOfReference().getOrientation().col(0).T()*(WC - WL) - circlesolid->getRadius();
      cpData[icircle].getFrameOfReference().setPosition(WC - cpData[iline].getFrameOfReference().getOrientation().col(0)*circlesolid->getRadius());
      cpData[iline].getFrameOfReference().setPosition(cpData[icircle].getFrameOfReference().getPosition() - cpData[iline].getFrameOfReference().getOrientation().col(0)*g(0));
    }
    else {
      cpData[iline].getFrameOfReference().getPosition() = (s<0)?WL0:WL+linesegment->getBounds()(1)*WLdir;
      const Vec3 WrD = -WC + cpData[iline].getFrameOfReference().getPosition();
      cpData[icircle].getFrameOfReference().getOrientation().set(0, WrD/nrm2(WrD));
      cpData[iline].getFrameOfReference().getOrientation().set(0, -cpData[icircle].getFrameOfReference().getOrientation().col(0));
      cpData[icircle].getFrameOfReference().getOrientation().set(2, circlesolid->getFrame()->getOrientation().col(2));
      cpData[iline].getFrameOfReference().getOrientation().set(2, linesegment->getFrame()->getOrientation().col(2));
      cpData[icircle].getFrameOfReference().getOrientation().set(1, crossProduct(cpData[icircle].getFrameOfReference().getOrientation().col(2), cpData[icircle].getFrameOfReference().getOrientation().col(0)));
      cpData[iline].getFrameOfReference().getOrientation().set(1, -cpData[icircle].getFrameOfReference().getOrientation().col(1));
      cpData[icircle].getFrameOfReference().getPosition() = WC + cpData[icircle].getFrameOfReference().getOrientation().col(0)*circlesolid->getRadius();
      g(0) = cpData[icircle].getFrameOfReference().getOrientation().col(0).T()*WrD - circlesolid->getRadius();
    }
  }

  void ContactKinematicsCircleSolidLineSegment::updatewb(Vec &wb, const Vec &g, ContourPointData *cpData) {
    throw MBSimError("Not implemented!");
  }
      
  void ContactKinematicsCircleSolidLineSegment::computeCurvatures(Vec &r, ContourPointData* cpData) {
    r(icircle)=circlesolid->computeCurvature(cpData[icircle]);
    r(iline)=linesegment->computeCurvature(cpData[iline]);
  }

}

