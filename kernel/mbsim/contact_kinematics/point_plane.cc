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
#include "point_plane.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/plane.h"
#include "mbsim/contours/point.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsPointPlane::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      iplane = 1;
      point = static_cast<Point*>(contour[0]);
      plane = static_cast<Plane*>(contour[1]);
    }
    else {
      ipoint = 1;
      iplane = 0;
      point = static_cast<Point*>(contour[1]);
      plane = static_cast<Plane*>(contour[0]);
    }
  }

  void ContactKinematicsPointPlane::updateg(SingleContact &contact, int i) {
    contact.getContourFrame(iplane)->setOrientation(plane->getFrame()->evalOrientation()); // data of possible contact point
    contact.getContourFrame(ipoint)->getOrientation(false).set(0, -plane->getFrame()->getOrientation().col(0));
    contact.getContourFrame(ipoint)->getOrientation(false).set(1, -plane->getFrame()->getOrientation().col(1));
    contact.getContourFrame(ipoint)->getOrientation(false).set(2, plane->getFrame()->getOrientation().col(2));

    Vec3 Wn = contact.getContourFrame(iplane)->getOrientation(false).col(0); // normal is first vector of coordinate orientation

    Vec3 Wd =  point->getFrame()->evalPosition() - plane->getFrame()->evalPosition();

    double g = Wn.T()*Wd; // distance

    contact.getContourFrame(ipoint)->setPosition(point->getFrame()->getPosition()); // possible contact locations
    contact.getContourFrame(iplane)->setPosition(contact.getContourFrame(ipoint)->getPosition(false) - Wn*g);
    contact.getGeneralizedRelativePosition(false)(0) = g;
  }

  void ContactKinematicsPointPlane::updatewb(SingleContact &contact, int i) {
    std::runtime_error("(ContactKinematicsPointPlane::updatewb): Not implemented.");

    Vec3 v1 = contact.getContourFrame(iplane)->evalOrientation().col(2); // second tangential vector in contact
    Vec3 n1 = contact.getContourFrame(iplane)->getOrientation().col(0); // normal in contact
    Vec3 u1 = contact.getContourFrame(iplane)->getOrientation().col(1); // first tangential vector in contact
    Vec3 vC1 = contact.getContourFrame(iplane)->evalVelocity(); // velocity of possible plane contact
    Vec3 vC2 = contact.getContourFrame(ipoint)->evalVelocity(); // velocity of point
    Vec3 Om1 = contact.getContourFrame(iplane)->evalAngularVelocity(); // angular velocity of possible plane contact

    Vec3 &s1 = u1;
    Vec3 &t1 = v1;

    Mat3x2 R1;
    R1.set(0, s1);
    R1.set(1, t1);

    SqrMat A(2,NONINIT);
    A(RangeV(0,0),RangeV(0,1)) = -u1.T()*R1; // first matrix row
    A(RangeV(1,1),RangeV(0,1)) = -v1.T()*R1; // second matrix row

    Vec b(2,NONINIT);
    b(0) = -u1.T()*(vC2-vC1);
    b(1) = -v1.T()*(vC2-vC1);
    Vec zetad1 =  slvLU(A,b);

    Mat3x3 tOm1 = tilde(Om1); // tilde operator
    contact.getwb(false)(0) += n1.T()*(-tOm1*(vC2-vC1) - tOm1*R1*zetad1); // acceleration in terms of contour parametrisation

    if(contact.getwb(false).size() > 1) {
      contact.getwb(false)(1) += u1.T()*(-tOm1*(vC2-vC1) - tOm1*R1*zetad1);
      contact.getwb(false)(2) += v1.T()*(-tOm1*(vC2-vC1) - tOm1*R1*zetad1);
    }
  }

}
