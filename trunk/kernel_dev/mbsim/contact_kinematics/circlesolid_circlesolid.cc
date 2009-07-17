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
 * Contact: mfoerg@users.berlios.de
 *          rzander@users.berlios.de
 */

#include <config.h> 
#include "mbsim/contact_kinematics/circlesolid_circlesolid.h"
#include "mbsim/contours/circle_solid.h"
#include "mbsim/contours/circle_hollow.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsCircleSolidCircleSolid::assignContours(const vector<Contour*> &contour) {
    icircle0 = 0; 
    icircle1 = 1;
    circle0 = static_cast<CircleSolid*>(contour[0]);
    circle1 = static_cast<CircleSolid*>(contour[1]);
  }

  void ContactKinematicsCircleSolidCircleSolid::updateg(Vec &g, ContourPointData *cpData) {
    Vec WrD = circle0->getFrame()->getPosition() - circle1->getFrame()->getPosition();
    cpData[icircle1].getFrameOfReference().getOrientation().col(0) = WrD/nrm2(WrD);
    cpData[icircle0].getFrameOfReference().getOrientation().col(0) = -cpData[icircle1].getFrameOfReference().getOrientation().col(0);
    cpData[icircle0].getFrameOfReference().getOrientation().col(2) = circle0->getFrame()->getOrientation().col(2);
    cpData[icircle1].getFrameOfReference().getOrientation().col(2) = circle1->getFrame()->getOrientation().col(2);
    cpData[icircle0].getFrameOfReference().getOrientation().col(1) = crossProduct(cpData[icircle0].getFrameOfReference().getOrientation().col(2),cpData[icircle0].getFrameOfReference().getOrientation().col(0));
    cpData[icircle1].getFrameOfReference().getOrientation().col(1) = -cpData[icircle0].getFrameOfReference().getOrientation().col(1);
    cpData[icircle0].getFrameOfReference().getPosition() = circle0->getFrame()->getPosition() + cpData[icircle0].getFrameOfReference().getOrientation().col(0)*circle0->getRadius();
    cpData[icircle1].getFrameOfReference().getPosition() = circle1->getFrame()->getPosition() + cpData[icircle1].getFrameOfReference().getOrientation().col(0)*circle1->getRadius();

    g(0) = trans(cpData[icircle1].getFrameOfReference().getOrientation().col(0))*WrD - circle0->getRadius() - circle1->getRadius();
  }
}

