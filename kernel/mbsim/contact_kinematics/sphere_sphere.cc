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
#include "sphere_sphere.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/utils/eps.h"
#include "mbsim/utils/contact_utils.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsSphereSphere::assignContours(const vector<Contour*> &contour) {
    isphere0 = 0; isphere1 = 1;
    sphere0 = static_cast<Sphere*>(contour[0]);
    sphere1 = static_cast<Sphere*>(contour[1]);
  }

  void ContactKinematicsSphereSphere::updateg(SingleContact &contact, int i) {
    Vec3 Wd = sphere1->getFrame()->evalPosition() - sphere0->getFrame()->evalPosition();
    double l = nrm2(Wd);
    Wd = Wd/l;
    double g = l-sphere0->getRadius()-sphere1->getRadius();
    Vec3 t_ = orthonormal(Wd);
    contact.getContourFrame(isphere0)->getOrientation(false).set(0, Wd);
    contact.getContourFrame(isphere1)->getOrientation(false).set(0, -Wd);
    contact.getContourFrame(isphere0)->getOrientation(false).set(1, t_);
    contact.getContourFrame(isphere1)->getOrientation(false).set(1, -t_);
    contact.getContourFrame(isphere0)->getOrientation(false).set(2, crossProduct(Wd,t_));
    contact.getContourFrame(isphere1)->getOrientation(false).set(2, contact.getContourFrame(isphere0)->getOrientation(false).col(2));
    contact.getContourFrame(isphere0)->setPosition(sphere0->getFrame()->getPosition() + sphere0->getRadius() * Wd);
    contact.getContourFrame(isphere1)->setPosition(sphere1->getFrame()->getPosition() - sphere1->getRadius() * Wd);
    contact.getGeneralizedRelativePosition(false)(0) = g;
  }

}
