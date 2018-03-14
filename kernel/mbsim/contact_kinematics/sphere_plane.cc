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
#include "sphere_plane.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/plane.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/utils/contact_utils.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsSpherePlane::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Sphere*>(contour[0])) {
      isphere = 0; iplane = 1;
      sphere = static_cast<Sphere*>(contour[0]);
      plane = static_cast<Plane*>(contour[1]);
    } 
    else {
      isphere = 1; iplane = 0;
      sphere = static_cast<Sphere*>(contour[1]);
      plane = static_cast<Plane*>(contour[0]);
    }
  }

  void ContactKinematicsSpherePlane::updateg(SingleContact &contact, int i) {
    contact.getContourFrame(iplane)->setOrientation(plane->getFrame()->evalOrientation());
    contact.getContourFrame(isphere)->getOrientation(false).set(0, -plane->getFrame()->getOrientation().col(0));
    contact.getContourFrame(isphere)->getOrientation(false).set(1, -plane->getFrame()->getOrientation().col(1));
    contact.getContourFrame(isphere)->getOrientation(false).set(2, plane->getFrame()->getOrientation().col(2));

    Vec3 Wn = contact.getContourFrame(iplane)->getOrientation(false).col(0);

    Vec3 Wd = sphere->getFrame()->evalPosition() - plane->getFrame()->evalPosition();

    double g = Wn.T()*Wd - sphere->getRadius();

    contact.getContourFrame(isphere)->setPosition(sphere->getFrame()->getPosition() - Wn*sphere->getRadius());
    contact.getContourFrame(iplane)->setPosition(contact.getContourFrame(isphere)->getPosition(false) - Wn*g);
    contact.getGeneralizedRelativePosition(false)(0) = g;
  }

  void ContactKinematicsSpherePlane::updatewb(SingleContact &contact, int i) {
    Vec3 n1 = contact.getContourFrame(iplane)->evalOrientation().col(0);
    Vec3 n2 = contact.getContourFrame(isphere)->evalOrientation().col(0);
    Vec3 vC1 = contact.getContourFrame(iplane)->evalVelocity();
    Vec3 vC2 = contact.getContourFrame(isphere)->evalVelocity();
    Vec3 Om1 = contact.getContourFrame(iplane)->evalAngularVelocity();
    Vec3 Om2 = contact.getContourFrame(isphere)->evalAngularVelocity();

    Vec2 zeta = computeAnglesOnUnitSphere(sphere->getFrame()->evalOrientation().T()*n2);

    Vec3 u1 = contact.getContourFrame(iplane)->getOrientation().col(1);
    Vec3 v1 = contact.getContourFrame(iplane)->getOrientation().col(2);
    Vec3 u2 = sphere->evalWu(zeta);
    Vec3 v2 = crossProduct(n2,u2);

    Mat3x2 R1 = plane->evalWR(zeta);
    Mat3x2 R2 = sphere->evalWR(zeta);
    Mat3x2 U2 = sphere->evalWU(zeta);
    Mat3x2 V2 = sphere->evalWV(zeta);

    SqrMat A(4,NONINIT);
    A(RangeV(0,0),RangeV(0,1)) = -u1.T()*R1;
    A(RangeV(0,0),RangeV(2,3)) = u1.T()*R2;
    A(RangeV(1,1),RangeV(0,1)) = -v1.T()*R1;
    A(RangeV(1,1),RangeV(2,3)) = v1.T()*R2;
    A(RangeV(2,2),RangeV(0,1)).init(0);
    A(RangeV(2,2),RangeV(2,3)) = n1.T()*U2;
    A(RangeV(3,3),RangeV(0,1)).init(0);
    A(RangeV(3,3),RangeV(2,3)) = n1.T()*V2;

    Vec b(4,NONINIT);
    b(0) = -u1.T()*(vC2-vC1);
    b(1) = -v1.T()*(vC2-vC1);
    b(2) = -v2.T()*(Om2-Om1);
    b(3) = u2.T()*(Om2-Om1);
    Vec zetad =  slvLU(A,b);
    Vec zetad1 = zetad(0,1);
    Vec zetad2 = zetad(2,3);

    Mat3x3 tOm1 = tilde(Om1);
    Mat3x3 tOm2 = tilde(Om2);
    contact.getwb(false)(0) += n1.T()*(-tOm1*(vC2-vC1) - tOm1*R1*zetad1 + tOm2*R2*zetad2);

    if(contact.getwb(false).size() > 1) contact.getwb(false)(1) += u1.T()*(-tOm1*(vC2-vC1) - tOm1*R1*zetad1 + tOm2*R2*zetad2);
    if(contact.getwb(false).size() > 2) contact.getwb(false)(2) += v1.T()*(-tOm1*(vC2-vC1) - tOm1*R1*zetad1 + tOm2*R2*zetad2);
  }

}
