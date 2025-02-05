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
#include "circle_line.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/line.h"
#include "mbsim/contours/circle.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsCircleLine::assignContours(const vector<Contour*> &contour) {
    ContactKinematics::assignContours(contour);
    if(dynamic_cast<Circle*>(contour[0])) {
      icircle = 0; iline = 1;
      circle = static_cast<Circle*>(contour[0]);
      line = static_cast<Line*>(contour[1]);
    } 
    else {
      icircle = 1; iline = 0;
      circle = static_cast<Circle*>(contour[1]);
      line = static_cast<Line*>(contour[0]);
    }
  }

  void ContactKinematicsCircleLine::updateg(SingleContact &contact, int i) {

    contact.getContourFrame(iline)->setOrientation(line->getFrame()->evalOrientation());
    contact.getContourFrame(icircle)->getOrientation(false).set(0, -line->getFrame()->getOrientation().col(0));
    contact.getContourFrame(icircle)->getOrientation(false).set(2, circle->getFrame()->evalOrientation().col(2));
    contact.getContourFrame(icircle)->getOrientation(false).set(1, crossProduct(contact.getContourFrame(icircle)->getOrientation(false).col(2), contact.getContourFrame(icircle)->getOrientation(false).col(0)));

    Vec3 Wn = contact.getContourFrame(iline)->getOrientation(false).col(0);

    Vec3 Wd = circle->getFrame()->evalPosition() - line->getFrame()->evalPosition();

    double g = Wn.T()*Wd - circle->getRadius();

    contact.getContourFrame(icircle)->setPosition(circle->getFrame()->getPosition() - Wn*circle->getRadius());
    contact.getContourFrame(iline)->setPosition(contact.getContourFrame(icircle)->getPosition(false) - Wn*g);
    contact.getGeneralizedRelativePosition(false)(0) = g;
  }

  void ContactKinematicsCircleLine::updatewb(SingleContact &contact, int i) {

    Vec3 v2 = contact.getContourFrame(icircle)->evalOrientation().col(2);
    Vec3 n1 = contact.getContourFrame(iline)->evalOrientation().col(0);
    Vec3 u1 = contact.getContourFrame(iline)->getOrientation().col(1);
    Vec3 u2 = contact.getContourFrame(icircle)->getOrientation().col(1);
    Vec3 vC1 = contact.getContourFrame(iline)->evalVelocity();
    Vec3 vC2 = contact.getContourFrame(icircle)->evalVelocity();
    Vec3 Om1 = contact.getContourFrame(iline)->evalAngularVelocity();
    Vec3 Om2 = contact.getContourFrame(icircle)->evalAngularVelocity();
    double r = circle->getRadius();

    Vec3 s2 = u2*r;
    double ad2 = -v2.T()*(Om2-Om1);
    double ad1 = u1.T()*(vC2-vC1) + (u1.T()*s2)*ad2;

    if(contact.isNormalForceLawSetValued())
      contact.getwb(false)(0) += n1.T()*(-crossProduct(Om1,vC2-vC1) - crossProduct(Om1,u1)*ad1 + crossProduct(Om2,s2)*ad2);
    if(contact.isTangentialForceLawSetValuedAndActive())
      contact.getwb(false)(contact.isNormalForceLawSetValued()) += u1.T()*(-crossProduct(Om1,vC2-vC1) - crossProduct(Om1,u1)*ad1 + crossProduct(Om2,s2)*ad2);
  }
      
}
