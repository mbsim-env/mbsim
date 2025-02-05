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
#include "mbsim/contact_kinematics/circle_circle.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/circle.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsCircleCircle::assignContours(const vector<Contour*> &contour) {
    ContactKinematics::assignContours(contour);
    if(not(static_cast<Circle*>(contour[0])->getSolid())) {
      circle0 = static_cast<Circle*>(contour[1]);
      circle1 = static_cast<Circle*>(contour[0]);
      icircle0 = 1;
      icircle1 = 0;
    }
    else {
      circle0 = static_cast<Circle*>(contour[0]);
      circle1 = static_cast<Circle*>(contour[1]);
      icircle0 = 0;
      icircle1 = 1;
    }
    rEff0 = circle0->getRadius();
    rEff1 = circle1->getSign()*circle1->getRadius();
  }

  void ContactKinematicsCircleCircle::updateg(SingleContact &contact, int i) {
    Vec3 WrD = circle0->getFrame()->evalPosition() - circle1->getFrame()->evalPosition();
    contact.getContourFrame(icircle1)->getOrientation(false).set(0, circle1->getSign()*WrD/nrm2(WrD));
    contact.getContourFrame(icircle0)->getOrientation(false).set(0, -contact.getContourFrame(icircle1)->getOrientation(false).col(0));
    contact.getContourFrame(icircle0)->getOrientation(false).set(2, circle0->getFrame()->getOrientation(false).col(2));
    contact.getContourFrame(icircle1)->getOrientation(false).set(2, circle1->getSign()*circle1->getFrame()->getOrientation(false).col(2));
    contact.getContourFrame(icircle0)->getOrientation(false).set(1, crossProduct(contact.getContourFrame(icircle0)->getOrientation(false).col(2),contact.getContourFrame(icircle0)->getOrientation(false).col(0)));
    contact.getContourFrame(icircle1)->getOrientation(false).set(1, crossProduct(contact.getContourFrame(icircle1)->getOrientation(false).col(2),contact.getContourFrame(icircle1)->getOrientation(false).col(0)));
    contact.getContourFrame(icircle0)->setPosition(circle0->getFrame()->getPosition() + contact.getContourFrame(icircle0)->getOrientation(false).col(0)*rEff0);
    contact.getContourFrame(icircle1)->setPosition(circle1->getFrame()->getPosition() + contact.getContourFrame(icircle1)->getOrientation(false).col(0)*rEff1);

    contact.getGeneralizedRelativePosition(false)(0) = contact.getContourFrame(icircle1)->getOrientation(false).col(0).T()*WrD - rEff0 - rEff1;
  }
      
  void ContactKinematicsCircleCircle::updatewb(SingleContact &contact, int i) {

    const Vec3 KrPC1 = circle0->getFrame()->evalOrientation().T()*(contact.getContourFrame(icircle0)->evalPosition() - circle0->getFrame()->evalPosition());
    Vec2 zeta1;
    zeta1(0)=(KrPC1(1)>0) ? acos(KrPC1(0)/nrm2(KrPC1)) : 2.*M_PI - acos(KrPC1(0)/nrm2(KrPC1));
    const Vec3 n1 = contact.getContourFrame(icircle0)->getOrientation().col(0); //crossProduct(s1, t1);
    const Vec3 u1 = circle0->evalWu(zeta1);
    const Vec3 R1 = circle0->evalWs(zeta1);
    const Vec3 N1 = circle0->evalParDer1Wn(zeta1);
    const Vec3 U1 = circle0->evalParDer1Wu(zeta1);

    const Vec3 KrPC2 = circle1->getFrame()->evalOrientation().T()*(contact.getContourFrame(icircle1)->evalPosition() - circle1->getFrame()->evalPosition());
    Vec2 zeta2;
    zeta2(0)=(KrPC2(1)>0) ? acos(KrPC2(0)/nrm2(KrPC2)) : 2.*M_PI - acos(KrPC2(0)/nrm2(KrPC2));
    const Vec3 n2 = contact.getContourFrame(icircle1)->getOrientation().col(0); //crossProduct(s1, t1);
    const Vec3 u2 = circle1->evalWu(zeta2);
    const Vec3 R2 = circle1->evalWs(zeta2);
    const Vec3 U2 = circle1->evalParDer1Wu(zeta2);
    const Vec3 v2 = crossProduct(n2, u2);

    const Vec3 vC1 = contact.getContourFrame(icircle0)->getVelocity();
    const Vec3 vC2 = contact.getContourFrame(icircle1)->getVelocity();
    const Vec3 Om1 = contact.getContourFrame(icircle0)->getAngularVelocity();
    const Vec3 Om2 = contact.getContourFrame(icircle1)->getAngularVelocity();

    SqrMat A(2,NONINIT);
    A(0,0)=-u1.T()*R1;
    A(0,1)=u1.T()*R2;
    A(1,0)=u2.T()*N1;
    A(1,1)=n1.T()*U2;
    Vec b(2,NONINIT);
    b(0)=-u1.T()*(vC2-vC1);
    b(1)=-v2.T()*(Om2-Om1);
    const Vec zetad = slvLU(A,b);

    const Mat3x3 tOm1 = tilde(Om1);
    const Mat3x3 tOm2 = tilde(Om2);
    
    if(contact.isNormalForceLawSetValued())
      contact.getwb(false)(0) += ((vC2-vC1).T()*N1-n1.T()*tOm1*R1)*zetad(0)+n1.T()*tOm2*R2*zetad(1)-n1.T()*tOm1*(vC2-vC1);
    if(contact.isTangentialForceLawSetValuedAndActive())
      contact.getwb(false)(contact.isNormalForceLawSetValued()) += ((vC2-vC1).T()*U1-u1.T()*tOm1*R1)*zetad(0)+u1.T()*tOm2*R2*zetad(1)-u1.T()*tOm1*(vC2-vC1);
  }
}
