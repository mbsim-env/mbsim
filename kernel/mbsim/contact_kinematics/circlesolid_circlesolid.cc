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
 *          rzander@users.berlios.de
 */

#include <config.h> 
#include "mbsim/contact_kinematics/circlesolid_circlesolid.h"
#include "mbsim/frame.h"
#include "mbsim/contours/circle_solid.h"
#include "mbsim/contours/circle_hollow.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsSolidCircleSolidCircle::assignContours(const vector<Contour*> &contour) {
    icircle0 = 0; 
    icircle1 = 1;
    circle0 = static_cast<SolidCircle*>(contour[0]);
    circle1 = static_cast<SolidCircle*>(contour[1]);
  }

  void ContactKinematicsSolidCircleSolidCircle::updateg(double t, double &g, std::vector<Frame*> &cFrame, int index) {
    Vec3 WrD = circle0->getFrame()->getPosition(t) - circle1->getFrame()->getPosition(t);
    cFrame[icircle1]->getOrientation(false).set(0, WrD/nrm2(WrD));
    cFrame[icircle0]->getOrientation(false).set(0, -cFrame[icircle1]->getOrientation(false).col(0));
    cFrame[icircle0]->getOrientation(false).set(2, circle0->getFrame()->getOrientation().col(2));
    cFrame[icircle1]->getOrientation(false).set(2, circle1->getFrame()->getOrientation().col(2));
    cFrame[icircle0]->getOrientation(false).set(1, crossProduct(cFrame[icircle0]->getOrientation(false).col(2),cFrame[icircle0]->getOrientation(false).col(0)));
    cFrame[icircle1]->getOrientation(false).set(1, -cFrame[icircle0]->getOrientation(false).col(1));
    cFrame[icircle0]->setPosition(circle0->getFrame()->getPosition() + cFrame[icircle0]->getOrientation(false).col(0)*circle0->getRadius());
    cFrame[icircle1]->setPosition(circle1->getFrame()->getPosition() + cFrame[icircle1]->getOrientation(false).col(0)*circle1->getRadius());

    g = cFrame[icircle1]->getOrientation(false).col(0).T()*WrD - circle0->getRadius() - circle1->getRadius();
  }
      
  void ContactKinematicsSolidCircleSolidCircle::updatewb(double t, Vec &wb, double g, std::vector<Frame*> &cFrame) {
    throw; // TODO: check implementation for the example that throws this exception

    const Vec3 KrPC1 = circle0->getFrame()->getOrientation(t).T()*(cFrame[icircle0]->getPosition(t) - circle0->getFrame()->getPosition(t));
    Vec2 zeta1;
    zeta1(0)=(KrPC1(1)>0) ? acos(KrPC1(0)/nrm2(KrPC1)) : 2.*M_PI - acos(KrPC1(0)/nrm2(KrPC1));
    const Vec3 n1 = cFrame[icircle0]->getOrientation().col(0); //crossProduct(s1, t1);
    const Vec3 u1 = circle0->getWu(t,zeta1);
    const Vec3 R1 = circle0->getWs(t,zeta1);
    const Vec3 N1 = circle0->getParDer1Wn(t,zeta1);
    const Vec3 U1 = circle0->getParDer1Wu(t,zeta1);

    const Vec3 KrPC2 = circle1->getFrame()->getOrientation(t).T()*(cFrame[icircle1]->getPosition(t) - circle1->getFrame()->getPosition(t));
    Vec2 zeta2;
    zeta2(0)=(KrPC2(1)>0) ? acos(KrPC2(0)/nrm2(KrPC2)) : 2.*M_PI - acos(KrPC2(0)/nrm2(KrPC2));
    const Vec3 n2 = cFrame[icircle1]->getOrientation().col(0); //crossProduct(s1, t1);
    const Vec3 u2 = circle1->getWu(t,zeta2);
    const Vec3 R2 = circle1->getWs(t,zeta2);
    const Vec3 U2 = circle1->getParDer1Wu(t,zeta2);
    const Vec3 v2 = crossProduct(n2, u2);

    const Vec3 vC1 = cFrame[icircle0]->getVelocity();
    const Vec3 vC2 = cFrame[icircle1]->getVelocity();
    const Vec3 Om1 = cFrame[icircle0]->getAngularVelocity();
    const Vec3 Om2 = cFrame[icircle1]->getAngularVelocity();

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
    
    wb(0) += ((vC2-vC1).T()*N1-n1.T()*tOm1*R1)*zetad(0)+n1.T()*tOm2*R2*zetad(1)-n1.T()*tOm1*(vC2-vC1);
    if (wb.size()>1) 
      wb(1) += ((vC2-vC1).T()*U1-u1.T()*tOm1*R1)*zetad(0)+u1.T()*tOm2*R2*zetad(1)-u1.T()*tOm1*(vC2-vC1);
  }
}

