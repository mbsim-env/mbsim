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
#include "mbsim/contact_kinematics/circle_plane.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/plane.h"
#include "mbsim/contours/circle.h"
#include "mbsim/utils/contact_utils.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsCirclePlane::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Circle*>(contour[0])) {
      icircle = 0;
      iplane = 1;
      circle = static_cast<Circle*>(contour[0]);
      plane = static_cast<Plane*>(contour[1]);
    } 
    else {
      icircle = 1;
      iplane = 0;
      circle = static_cast<Circle*>(contour[1]);
      plane = static_cast<Plane*>(contour[0]);
    }
  }

  void ContactKinematicsCirclePlane::updateg(SingleContact &contact, int i) {
    contact.getContourFrame(iplane)->setOrientation(plane->getFrame()->evalOrientation());

    Vec3 Wd;
    Vec3 Wn = contact.getContourFrame(iplane)->getOrientation(false).col(0);
    Vec3 Wb = circle->getFrame()->evalOrientation().col(2);
    double t_EC = Wn.T()*Wb;
    if(t_EC>0) {
      Wb *= -1.;
      t_EC *= -1;	
    }
    Vec3 z_EC = Wn - t_EC*Wb;
    double z_EC_nrm2 = nrm2(z_EC);
    if(z_EC_nrm2 <= 1e-8) { // infinite possible contact points
      Wd = circle->getFrame()->getPosition() - plane->getFrame()->getPosition();
    } 
    else { // exactly one possible contact point
      Wd =  (circle->getFrame()->getPosition() - (circle->getRadius()/z_EC_nrm2)*z_EC) - plane->getFrame()->getPosition();
    }
    double g = Wn.T()*Wd;
    contact.getContourFrame(icircle)->setPosition(circle->getFrame()->getPosition() - (circle->getRadius()/z_EC_nrm2)*z_EC);
    contact.getContourFrame(iplane)->setPosition(contact.getContourFrame(icircle)->getPosition(false) - Wn*g);
    contact.getGeneralizedRelativePosition(false)(0) = g;

    contact.getContourFrame(icircle)->setEta(computeAngleOnUnitCircle(circle->getFrame()->evalOrientation().T()*(z_EC/(-z_EC_nrm2))));
    contact.getContourFrame(icircle)->getOrientation(false).set(0, -plane->getFrame()->getOrientation().col(0));
    contact.getContourFrame(icircle)->getOrientation(false).set(1, circle->evalWu(contact.getContourFrame(icircle)->getZeta()));
    contact.getContourFrame(icircle)->getOrientation(false).set(2, crossProduct(contact.getContourFrame(icircle)->getOrientation(false).col(0),contact.getContourFrame(icircle)->getOrientation(false).col(1)));
  }

  void ContactKinematicsCirclePlane::updatewb(SingleContact &contact, int i) {

    Vec3 n1 = contact.getContourFrame(iplane)->evalOrientation().col(0);
    Vec3 u1 = contact.getContourFrame(iplane)->getOrientation().col(1);
    Vec3 v1 = contact.getContourFrame(iplane)->getOrientation().col(2);
    Vec3 u2 = contact.getContourFrame(icircle)->evalOrientation().col(1);
    Vec3 v2 = contact.getContourFrame(icircle)->evalOrientation().col(2);
    Vec3 vC1 = contact.getContourFrame(iplane)->evalVelocity();
    Vec3 vC2 = contact.getContourFrame(icircle)->evalVelocity();
    Vec3 Om1 = contact.getContourFrame(iplane)->evalAngularVelocity();
    Vec3 Om2 = contact.getContourFrame(icircle)->evalAngularVelocity();

    Mat3x2 R1 = plane->evalWR(contact.getContourFrame(iplane)->getZeta());
    Mat3x2 KU2, KV2;
    KU2(0,0) = -cos(contact.getContourFrame(icircle)->getEta());
    KU2(1,0) = -sin(contact.getContourFrame(icircle)->getEta());
    KV2(0,0) = -sin(contact.getContourFrame(icircle)->getEta())*sin(contact.getContourFrame(icircle)->getXi());
    KV2(1,0) = cos(contact.getContourFrame(icircle)->getEta())*sin(contact.getContourFrame(icircle)->getXi());
    KV2(0,1) = cos(contact.getContourFrame(icircle)->getEta())*cos(contact.getContourFrame(icircle)->getXi());
    KV2(1,1) = sin(contact.getContourFrame(icircle)->getEta())*cos(contact.getContourFrame(icircle)->getXi());
    KV2(2,1) = -sin(contact.getContourFrame(icircle)->getXi());
    Mat3x2 U2 = circle->getFrame()->getOrientation()*KU2;
    Mat3x2 V2 = circle->getFrame()->getOrientation()*KV2;
    Mat3x2 R2;
    R2.set(0,u2);

    SqrMat A(4,NONINIT);
    A.set(RangeV(0,0),RangeV(0,1), -u1.T()*R1);
    A.set(RangeV(0,0),RangeV(2,3), u1.T()*R2);
    A.set(RangeV(1,1),RangeV(0,1), -v1.T()*R1);
    A.set(RangeV(1,1),RangeV(2,3), v1.T()*R2);
    A.set(RangeV(2,2),RangeV(0,1), RowVec(2));
    A.set(RangeV(2,2),RangeV(2,3), n1.T()*U2);
    A.set(RangeV(3,3),RangeV(0,1), RowVec(2));
    A.set(RangeV(3,3),RangeV(2,3), n1.T()*V2);

    Vec b(4,NONINIT);
    b(0) = -u1.T()*(vC2-vC1);
    b(1) = -v1.T()*(vC2-vC1);
    b(2) = -v2.T()*(Om2-Om1);
    b(3) = u2.T()*(Om2-Om1);
    Vec zetad =  slvLU(A,b);
    Vec zetad1 = zetad(RangeV(0,1));
    Vec zetad2 = zetad(RangeV(2,3));

    Mat3x3 tOm1 = tilde(Om1);
    Mat3x3 tOm2 = tilde(Om2);

    if(contact.isNormalForceLawSetValued())
      contact.getwb(false)(0) += n1.T()*(-tOm1*(vC2-vC1) - tOm1*R1*zetad1 + tOm2*R2*zetad2);
    if(contact.isTangentialForceLawSetValuedAndActive()) {
      contact.getwb(false)(contact.isNormalForceLawSetValued()) += u1.T()*(-tOm1*(vC2-vC1) - tOm1*R1*zetad1 + tOm2*R2*zetad2);
      if(contact.getFrictionDirections()>1)
        contact.getwb(false)(contact.isNormalForceLawSetValued()+1) += v1.T()*(-tOm1*(vC2-vC1) - tOm1*R1*zetad1 + tOm2*R2*zetad2);
    }
  }

}

