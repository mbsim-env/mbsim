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
#include "mbsim/contact_kinematics/point_plate.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/plate.h"
#include "mbsim/contours/point.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsPointPlate::assignContours(const vector<Contour*> &contour) {
    if (dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      iplate = 1;
      point = static_cast<Point*>(contour[0]);
      plate = static_cast<Plate*>(contour[1]);
    }
    else {
      ipoint = 1;
      iplate = 0;
      point = static_cast<Point*>(contour[1]);
      plate = static_cast<Plate*>(contour[0]);
    }
  }

  void ContactKinematicsPointPlate::updateg(double &g, std::vector<ContourFrame*> &cFrame, int index) {
    Vec3 Ar = plate->getFrame()->evalOrientation().T() * (point->getFrame()->evalPosition() - plate->getFrame()->evalPosition());
    if(fabs(Ar(1)) <= plate->getYLength()/2 and fabs(Ar(2)) <= plate->getZLength()/2){
      g = Ar(0);
      if(g < -plate->getThickness())
        g = 1;
      else {
        cFrame[ipoint]->getPosition(false) = point->getFrame()->getPosition();
        cFrame[iplate]->getPosition(false) = point->getFrame()->getPosition() - g * plate->getFrame()->getOrientation().col(0);
        cFrame[iplate]->getOrientation(false) = plate->getFrame()->getOrientation();
        cFrame[ipoint]->getOrientation(false).set(0,-plate->getFrame()->getOrientation().col(0));
        cFrame[ipoint]->getOrientation(false).set(1,-plate->getFrame()->getOrientation().col(1));
        cFrame[ipoint]->getOrientation(false).set(2,plate->getFrame()->getOrientation().col(2));
      }
    }
    else
      g = 1.;

  }

}

