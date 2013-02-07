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
#include "mbsim/contact_kinematics/point_area.h"
#include "mbsim/contours/area.h"
#include "mbsim/contours/point.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsPointArea::assignContours(const vector<Contour*> &contour) {
    if (dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      iarea = 1;
      point = static_cast<Point*>(contour[0]);
      area = static_cast<Area*>(contour[1]);
    }
    else {
      ipoint = 1;
      iarea = 0;
      point = static_cast<Point*>(contour[1]);
      area = static_cast<Area*>(contour[0]);
    }
  }

  void ContactKinematicsPointArea::updateg(fmatvec::Vec &g, ContourPointData *cpData, int index) {
    Vec3 Ar = area->getFrame()->getOrientation().T() * (point->getFrame()->getPosition() - area->getFrame()->getPosition());
    if(fabs(Ar(1)) <= area->getLimitY() / 2 and fabs(Ar(2)) <= area->getLimitZ()  / 2){
      g(0) = Ar(0);
      cpData[ipoint].getFrameOfReference().getPosition() = point->getFrame()->getPosition();
      cpData[iarea].getFrameOfReference().getPosition() = point->getFrame()->getPosition() - g(0) * area->getFrame()->getOrientation().col(0);
      cpData[iarea].getFrameOfReference().getOrientation() = area->getFrame()->getOrientation();
      cpData[ipoint].getFrameOfReference().getOrientation().set(0,-area->getFrame()->getOrientation().col(0));
      cpData[ipoint].getFrameOfReference().getOrientation().set(1,-area->getFrame()->getOrientation().col(1));
      cpData[ipoint].getFrameOfReference().getOrientation().set(2,area->getFrame()->getOrientation().col(2));
    }
    else
      g(0) = 1.;

  }

}

