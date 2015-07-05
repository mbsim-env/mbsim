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
#include "sphere_rectangle.h"
#include "mbsim/contours/rectangle.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/utils/contact_utils.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsSphereRectangle::assignContours(const vector<Contour*> &contour) {
    if (dynamic_cast<Sphere*>(contour[0])) {
      isphere = 0;
      irectangle = 1;
      sphere = static_cast<Sphere*>(contour[0]);
      rectangle = static_cast<Rectangle*>(contour[1]);
    }
    else {
      isphere = 1;
      irectangle = 0;
      sphere = static_cast<Sphere*>(contour[1]);
      rectangle = static_cast<Rectangle*>(contour[0]);
    }
  }

  void ContactKinematicsSphereRectangle::updateg(double t, double &g, ContourPointData *cpData, int index) {

    Vec3 sphereInRect = rectangle->getFrame()->getOrientation(t).T() * (sphere->getFrame()->getPosition(t) - rectangle->getFrame()->getPosition(t));

    if ((rectangle->getYLength() / 2. + sphere->getRadius() < fabs(sphereInRect(1))) or (rectangle->getZLength() / 2. + sphere->getRadius() < fabs(sphereInRect(2)))) {
      g = 1.;
      return;
    }

    cpData[irectangle].getFrameOfReference().setOrientation(rectangle->getFrame()->getOrientation());
    cpData[isphere].getFrameOfReference().getOrientation(false).set(0, -rectangle->getFrame()->getOrientation().col(0));
    cpData[isphere].getFrameOfReference().getOrientation(false).set(1, -rectangle->getFrame()->getOrientation().col(1));
    cpData[isphere].getFrameOfReference().getOrientation(false).set(2, rectangle->getFrame()->getOrientation().col(2));

    Vec3 Wn = cpData[irectangle].getFrameOfReference().getOrientation(false).col(0);

    Vec3 Wd = sphere->getFrame()->getPosition() - rectangle->getFrame()->getPosition();

    g = Wn.T() * Wd - sphere->getRadius();

    //assume that ball is far below rectangel --> no contact
    if(g < -sphere->getRadius()) {
      g = 1.;
      return;
    }

    cpData[isphere].getFrameOfReference().setPosition(sphere->getFrame()->getPosition() - Wn * sphere->getRadius());
    cpData[irectangle].getFrameOfReference().setPosition(cpData[isphere].getFrameOfReference().getPosition(false) - Wn * g);
  }

  void ContactKinematicsSphereRectangle::updatewb(double t, Vec &wb, double g, ContourPointData *cpData) {
    throw new MBSimError("ContactKinematicsSphereRectangle::updatewb(): not implemented yet");
  }
}

