/* Copyright (C) 2004-2022 MBSim Development Team
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
#include "point_cylinder.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/cylinder.h"
#include "mbsim/utils/contact_utils.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsPointCylinder::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      icylinder = 1;
      point = static_cast<Point*>(contour[0]);
      cylinder = static_cast<Cylinder*>(contour[1]);
    } 
    else {
      ipoint = 1;
      icylinder = 0;
      point = static_cast<Point*>(contour[1]);
      cylinder = static_cast<Cylinder*>(contour[0]);
    }
  }

  void ContactKinematicsPointCylinder::updateg(SingleContact &contact, int i) {
    const Vec3 WrP = point->getFrame()->evalPosition() - cylinder->getFrame()->evalPosition();
    const Vec3 Wez = cylinder->getFrame()->getOrientation().col(2);
    double s = Wez.T()*WrP;

    const Vec3 Wezs = s*Wez;

    const Vec3 WrD = point->getFrame()->getPosition() - Wezs;
    const Vec3 Wn = WrD/nrm2(WrD);
    
    contact.getContourFrame(icylinder)->getOrientation(false).set(0, Wn);
    contact.getContourFrame(icylinder)->setEta(computeAngleOnUnitCircle(cylinder->getFrame()->evalOrientation().T()*contact.getContourFrame(icylinder)->getOrientation(false).col(0)));
    contact.getContourFrame(icylinder)->setXi(s);
    contact.getContourFrame(icylinder)->getOrientation(false).set(2, cylinder->getFrame()->getOrientation(false).col(2));
    contact.getContourFrame(icylinder)->getOrientation(false).set(1, crossProduct(contact.getContourFrame(icylinder)->getOrientation(false).col(2), contact.getContourFrame(icylinder)->getOrientation(false).col(0)));

    contact.getContourFrame(ipoint)->getOrientation(false).set(0, -contact.getContourFrame(icylinder)->getOrientation(false).col(0));
    contact.getContourFrame(ipoint)->getOrientation(false).set(1, -contact.getContourFrame(icylinder)->getOrientation(false).col(1));
    contact.getContourFrame(ipoint)->getOrientation(false).set(2, contact.getContourFrame(icylinder)->getOrientation(false).col(2));
    
    contact.getContourFrame(ipoint)->setPosition(point->getFrame()->getPosition());
    contact.getContourFrame(icylinder)->setPosition(cylinder->getFrame()->getPosition() + Wezs + cylinder->getRadius()*Wn);

    contact.getGeneralizedRelativePosition(false)(0) = contact.getContourFrame(icylinder)->getOrientation(false).col(0).T()*WrD - cylinder->getRadius();
  }

  void ContactKinematicsPointCylinder::updatewb(SingleContact &contact, int i) {

    const Vec3 n1 = contact.getContourFrame(ipoint)->evalOrientation().col(0);
    const Vec3 u1 = contact.getContourFrame(ipoint)->getOrientation().col(1);
    const Vec3 v1 = contact.getContourFrame(ipoint)->getOrientation().col(2);
    const Mat3x2 U1 = point->evalWU(contact.getContourFrame(ipoint)->getZeta());
    const Mat3x2 V1 = point->evalWV(contact.getContourFrame(ipoint)->getZeta());
    const Mat3x2 N1 = point->evalWN(contact.getContourFrame(ipoint)->getZeta());

    const Vec3 u2 = contact.getContourFrame(icylinder)->evalOrientation().col(1);
    const Vec3 v2 = contact.getContourFrame(icylinder)->getOrientation().col(2);
    const Mat3x2 R2 = cylinder->evalWR(contact.getContourFrame(icylinder)->getZeta());
    const Mat3x2 U2 = cylinder->evalWU(contact.getContourFrame(icylinder)->getZeta());
    const Mat3x2 V2 = cylinder->evalWV(contact.getContourFrame(icylinder)->getZeta());

    const Vec3 vC1 = contact.getContourFrame(ipoint)->evalVelocity();
    const Vec3 vC2 = contact.getContourFrame(icylinder)->evalVelocity();
    const Vec3 Om1 = contact.getContourFrame(ipoint)->evalAngularVelocity();
    const Vec3 Om2 = contact.getContourFrame(icylinder)->evalAngularVelocity();

    SqrMat A(4,NONINIT);
    A.set(RangeV(0,0),RangeV(0,1), RowVec(2)); // = -u1.T()*R1;
    A.set(RangeV(0,0),RangeV(2,3), u1.T()*R2);
    A.set(RangeV(1,1),RangeV(0,1), RowVec(2)); // = -v1.T()*R1;
    A.set(RangeV(1,1),RangeV(2,3), v1.T()*R2);
    A.set(RangeV(2,2),RangeV(0,1), u2.T()*N1);
    A.set(RangeV(2,2),RangeV(2,3), n1.T()*U2);
    A.set(RangeV(3,3),RangeV(0,1), v2.T()*N1);
    A.set(RangeV(3,3),RangeV(2,3), n1.T()*V2);

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
