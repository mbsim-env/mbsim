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
#include "sphere_plate.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/plate.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/utils/contact_utils.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsSpherePlate::assignContours(const vector<Contour*> &contour) {
    if (dynamic_cast<Sphere*>(contour[0])) {
      isphere = 0;
      iplate = 1;
      sphere = static_cast<Sphere*>(contour[0]);
      plate = static_cast<Plate*>(contour[1]);
    }
    else {
      isphere = 1;
      iplate = 0;
      sphere = static_cast<Sphere*>(contour[1]);
      plate = static_cast<Plate*>(contour[0]);
    }
  }

  void ContactKinematicsSpherePlate::updateg(SingleContact &contact, int i) {

    Vec3 sphereInRect = plate->getFrame()->evalOrientation().T() * (sphere->getFrame()->evalPosition() - plate->getFrame()->evalPosition());

    double g;
    if ((plate->getYLength() / 2. + sphere->getRadius() < fabs(sphereInRect(1))) or (plate->getZLength() / 2. + sphere->getRadius() < fabs(sphereInRect(2)))) {
      contact.getGeneralizedRelativePosition(false)(0) = 1.;
      return;
    }

    contact.getContourFrame(iplate)->setOrientation(plate->getFrame()->getOrientation());
    contact.getContourFrame(isphere)->getOrientation(false).set(0, -plate->getFrame()->getOrientation().col(0));
    contact.getContourFrame(isphere)->getOrientation(false).set(1, -plate->getFrame()->getOrientation().col(1));
    contact.getContourFrame(isphere)->getOrientation(false).set(2, plate->getFrame()->getOrientation().col(2));

    Vec3 Wn = contact.getContourFrame(iplate)->getOrientation(false).col(0);

    Vec3 Wd = sphere->getFrame()->getPosition() - plate->getFrame()->getPosition();

    g = Wn.T() * Wd - sphere->getRadius();

    //assume that ball is far below rectangel --> no contact
    if(g < -sphere->getRadius()) {
      contact.getGeneralizedRelativePosition(false)(0) = 1.;
      return;
    }

    contact.getContourFrame(isphere)->setPosition(sphere->getFrame()->getPosition() - Wn * sphere->getRadius());
    contact.getContourFrame(iplate)->setPosition(contact.getContourFrame(isphere)->getPosition(false) - Wn * g);
    contact.getGeneralizedRelativePosition(false)(0) = g;
  }

}
