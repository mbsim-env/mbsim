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
#include "mbsim/contact_kinematics/point_rectangle.h"
#include "mbsim/contours/rectangle.h"
#include "mbsim/contours/point.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsPointRectangle::assignContours(const vector<Contour*> &contour) {
    if (dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      irectangle = 1;
      point = static_cast<Point*>(contour[0]);
      rectangle = static_cast<Rectangle*>(contour[1]);
    }
    else {
      ipoint = 1;
      irectangle = 0;
      point = static_cast<Point*>(contour[1]);
      rectangle = static_cast<Rectangle*>(contour[0]);
    }
  }

  void ContactKinematicsPointRectangle::updateg(fmatvec::Vec &g, ContourPointData *cpData, int index) {
    Vec3 Ar = rectangle->getFrame()->getOrientation().T() * (point->getFrame()->getPosition() - rectangle->getFrame()->getPosition());
    if(fabs(Ar(1)) <= rectangle->getYLength()/2 and fabs(Ar(2)) <= rectangle->getZLength()/2){
      g(0) = Ar(0);
      if(g(0) < -rectangle->getThickness())
        g(0) = 1;
      else {
        cpData[ipoint].getFrameOfReference().getPosition() = point->getFrame()->getPosition();
        cpData[irectangle].getFrameOfReference().getPosition() = point->getFrame()->getPosition() - g(0) * rectangle->getFrame()->getOrientation().col(0);
        cpData[irectangle].getFrameOfReference().getOrientation() = rectangle->getFrame()->getOrientation();
        cpData[ipoint].getFrameOfReference().getOrientation().set(0,-rectangle->getFrame()->getOrientation().col(0));
        cpData[ipoint].getFrameOfReference().getOrientation().set(1,-rectangle->getFrame()->getOrientation().col(1));
        cpData[ipoint].getFrameOfReference().getOrientation().set(2,rectangle->getFrame()->getOrientation().col(2));
      }
    }
    else
      g(0) = 1.;

  }

}

