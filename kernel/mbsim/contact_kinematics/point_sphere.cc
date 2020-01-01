/* Copyright (C) 2004-2013 MBSim Development Team
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
#include "point_sphere.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/utils/contact_utils.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsPointSphere::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      isphere = 1;
      point = static_cast<Point*>(contour[0]);
      sphere = static_cast<Sphere*>(contour[1]);
    }
    else {
      ipoint = 1;
      isphere = 0;
      point = static_cast<Point*>(contour[1]);
      sphere = static_cast<Sphere*>(contour[0]);
    }
  }

  void ContactKinematicsPointSphere::updateg(SingleContact &contact, int i) {
    const Vec3 WrD = point->getFrame()->evalPosition() - sphere->getFrame()->evalPosition();

    contact.getContourFrame(isphere)->getOrientation(false).set(0, WrD/nrm2(WrD));
    contact.getContourFrame(isphere)->setZeta(computeAnglesOnUnitSphere(sphere->getFrame()->evalOrientation().T()*contact.getContourFrame(isphere)->getOrientation(false).col(0)));
    contact.getContourFrame(isphere)->getOrientation(false).set(1, sphere->evalWu(contact.getContourFrame(isphere)->getZeta(false)));
    contact.getContourFrame(isphere)->getOrientation(false).set(2, crossProduct(contact.getContourFrame(isphere)->getOrientation(false).col(0),contact.getContourFrame(isphere)->getOrientation(false).col(1)));

    contact.getContourFrame(ipoint)->getOrientation(false).set(0, -contact.getContourFrame(isphere)->getOrientation(false).col(0));
    contact.getContourFrame(ipoint)->setZeta(computeAnglesOnUnitSphere(point->getFrame()->getOrientation().T()*contact.getContourFrame(ipoint)->getOrientation(false).col(0)));
    contact.getContourFrame(ipoint)->getOrientation(false).set(1, -contact.getContourFrame(isphere)->getOrientation(false).col(1));
    contact.getContourFrame(ipoint)->getOrientation(false).set(2, contact.getContourFrame(isphere)->getOrientation(false).col(2));

    contact.getContourFrame(isphere)->setPosition(sphere->getFrame()->getPosition() + contact.getContourFrame(isphere)->getOrientation(false).col(0)*sphere->getRadius());
    contact.getContourFrame(ipoint)->setPosition(point->getFrame()->getPosition());

    contact.getGeneralizedRelativePosition(false)(0) = contact.getContourFrame(isphere)->getOrientation(false).col(0).T()*WrD - sphere->getRadius();
  }

  void ContactKinematicsPointSphere::updatewb(SingleContact &contact, int i) {

    const Vec3 n1 = contact.getContourFrame(ipoint)->evalOrientation().col(0);
    const Vec3 u1 = contact.getContourFrame(ipoint)->getOrientation().col(1);
    const Vec3 v1 = contact.getContourFrame(ipoint)->getOrientation().col(2);
    const Mat3x2 U1 = point->evalWU(contact.getContourFrame(ipoint)->getZeta());
    const Mat3x2 V1 = point->evalWV(contact.getContourFrame(ipoint)->getZeta());
    const Mat3x2 N1 = point->evalWN(contact.getContourFrame(ipoint)->getZeta());

    const Vec3 u2 = contact.getContourFrame(isphere)->evalOrientation().col(1);
    const Vec3 v2 = contact.getContourFrame(isphere)->getOrientation().col(2);
    const Mat3x2 R2 = sphere->evalWR(contact.getContourFrame(isphere)->getZeta());
    const Mat3x2 U2 = sphere->evalWU(contact.getContourFrame(isphere)->getZeta());
    const Mat3x2 V2 = sphere->evalWV(contact.getContourFrame(isphere)->getZeta());

    const Vec3 vC1 = contact.getContourFrame(ipoint)->evalVelocity();
    const Vec3 vC2 = contact.getContourFrame(isphere)->evalVelocity();
    const Vec3 Om1 = contact.getContourFrame(ipoint)->evalAngularVelocity();
    const Vec3 Om2 = contact.getContourFrame(isphere)->evalAngularVelocity();

    SqrMat A(4,NONINIT);
    A(RangeV(0,0),RangeV(0,1)).init(0);// = -u1.T()*R1;
    A(RangeV(0,0),RangeV(2,3)) = u1.T()*R2;
    A(RangeV(1,1),RangeV(0,1)).init(0);// = -v1.T()*R1;
    A(RangeV(1,1),RangeV(2,3)) = v1.T()*R2;
    A(RangeV(2,2),RangeV(0,1)) = u2.T()*N1;
    A(RangeV(2,2),RangeV(2,3)) = n1.T()*U2;
    A(RangeV(3,3),RangeV(0,1)) = v2.T()*N1;
    A(RangeV(3,3),RangeV(2,3)) = n1.T()*V2;

    Vec b(4,NONINIT);
    b(0) = -u1.T()*(vC2-vC1);
    b(1) = -v1.T()*(vC2-vC1);
    b(2) = -v2.T()*(Om2-Om1);
    b(3) = u2.T()*(Om2-Om1);
    Vec zetad =  slvLU(A,b);
    Vec zetad1 = zetad(RangeV(0,1));
    Vec zetad2 = zetad(RangeV(2,3));

    const Mat3x3 tOm1 = tilde(Om1);
    const Mat3x3 tOm2 = tilde(Om2);

    if(contact.isNormalForceLawSetValued())
      contact.getwb(false)(0) += ((vC2-vC1).T()*N1/**-n1.T()*tOm1*R1**/)*zetad1+n1.T()*(tOm2*R2*zetad2-tOm1*(vC2-vC1));
    if(contact.isTangentialForceLawSetValuedAndActive()) {
      contact.getwb(false)(contact.isNormalForceLawSetValued()) += ((vC2-vC1).T()*U1/**-u1.T()*tOm1*R1**/)*zetad1+u1.T()*(tOm2*R2*zetad2-tOm1*(vC2-vC1));
      if(contact.getFrictionDirections()>1)
        contact.getwb(false)(contact.isNormalForceLawSetValued()+1) += ((vC2-vC1).T()*V1/**-v1.T()*tOm1*R1**/)*zetad1+v1.T()*(tOm2*R2*zetad2-tOm1*(vC2-vC1));
    }
  }

}
