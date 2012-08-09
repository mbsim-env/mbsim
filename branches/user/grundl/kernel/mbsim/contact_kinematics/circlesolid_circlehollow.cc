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
#include "circlesolid_circlehollow.h"
#include "mbsim/contour.h"
#include "mbsim/contours/circle_solid.h"
#include "mbsim/contours/circle_hollow.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsCircleSolidCircleHollow::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<CircleSolid*>(contour[0])) {
      icircle0 = 0; icircle1 = 1;
      circle0 = static_cast<CircleSolid*>(contour[0]);
      circle1 = static_cast<CircleHollow*>(contour[1]);
    } 
    else {
      icircle0 = 1; icircle1 = 0;
      circle0 = static_cast<CircleSolid*>(contour[1]);
      circle1 = static_cast<CircleHollow*>(contour[0]);
    }
  }

  void ContactKinematicsCircleSolidCircleHollow::updateg(Vec &g, ContourPointData *cpData, int index) {
    Vec WrD = circle0->getFrame()->getPosition() - circle1->getFrame()->getPosition();
    cpData[icircle1].getFrameOfReference().getOrientation().col(0) = - WrD/nrm2(WrD);
    cpData[icircle0].getFrameOfReference().getOrientation().col(0) = -cpData[icircle1].getFrameOfReference().getOrientation().col(0);
    cpData[icircle0].getFrameOfReference().getOrientation().col(2) = circle0->getFrame()->getOrientation().col(2);
    cpData[icircle1].getFrameOfReference().getOrientation().col(2) = circle1->getFrame()->getOrientation().col(2);
    cpData[icircle0].getFrameOfReference().getOrientation().col(1) = crossProduct(cpData[icircle0].getFrameOfReference().getOrientation().col(2),cpData[icircle0].getFrameOfReference().getOrientation().col(0));
    cpData[icircle1].getFrameOfReference().getOrientation().col(1) = -cpData[icircle0].getFrameOfReference().getOrientation().col(1);
    cpData[icircle0].getFrameOfReference().getPosition() = circle0->getFrame()->getPosition() + cpData[icircle0].getFrameOfReference().getOrientation().col(0)*circle0->getRadius();
    cpData[icircle1].getFrameOfReference().getPosition() = circle1->getFrame()->getPosition() + cpData[icircle0].getFrameOfReference().getOrientation().col(0)*circle1->getRadius();

    g(0) = circle1->getRadius() - cpData[icircle0].getFrameOfReference().getOrientation().col(0).T()*WrD - circle0->getRadius();
  }
      
  void ContactKinematicsCircleSolidCircleHollow::updatewb(Vec &wb, const Vec &g, ContourPointData *cpData) {
    const Vec KrPC1 = circle0->getFrame()->getOrientation().T()*(cpData[icircle0].getFrameOfReference().getPosition() - circle0->getFrame()->getPosition());
    const double zeta1=(KrPC1(1)>0) ? acos(KrPC1(0)/nrm2(KrPC1)) : 2.*M_PI - acos(KrPC1(0)/nrm2(KrPC1));
    const double sa1=sin(zeta1);
    const double ca1=cos(zeta1);
    const double r1=circle0->getRadius();
    Vec Ks1(3, NONINIT);
    Ks1(0)=-r1*sa1;
    Ks1(1)=r1*ca1;
    Ks1(2)=0;
    Vec Kt1(3, NONINIT);
    Kt1(0)=0;
    Kt1(1)=0;
    Kt1(2)=1;
    const Vec s1=circle0->getFrame()->getOrientation()*Ks1;
    const Vec t1=circle0->getFrame()->getOrientation()*Kt1;
    Vec n1=crossProduct(s1, t1);
    n1/=nrm2(n1);
    const Vec u1=s1/nrm2(s1);
    const Vec R1(s1);
    Vec KN1(3,NONINIT);
    KN1(0)=-sa1;
    KN1(1)=ca1;
    KN1(2)=0;
    const Vec N1=circle0->getFrame()->getOrientation()*KN1;
    Vec KU1(3,NONINIT);
    KU1(0)=-ca1;
    KU1(1)=-sa1;
    KU1(2)=0;
    const Vec U1=circle0->getFrame()->getOrientation()*KU1;

    const Vec KrPC2 = circle1->getFrame()->getOrientation().T()*(cpData[icircle1].getFrameOfReference().getPosition() - circle1->getFrame()->getPosition());
    const double zeta2=(KrPC2(1)>0) ? acos(KrPC2(0)/nrm2(KrPC2)) : 2.*M_PI - acos(KrPC2(0)/nrm2(KrPC2));
    const double sa2=sin(zeta2);
    const double ca2=cos(zeta2);
    const double r2=circle1->getRadius();
    Vec Ks2(3, NONINIT);
    Ks2(0)=-r2*sa2;
    Ks2(1)=r2*ca2;
    Ks2(2)=0;
    Vec Kt2(3, NONINIT);
    Kt2(0)=0;
    Kt2(1)=0;
    Kt2(2)=1;
    const Vec s2=circle1->getFrame()->getOrientation()*Ks2;
    const Vec t2=circle1->getFrame()->getOrientation()*Kt2;
    Vec n2=-crossProduct(s2, t2);
    n2/=nrm2(n2);
    const Vec u2=s2/nrm2(s2);
    const Vec v2=crossProduct(n2, u2);
    const Vec R2(s2);
    Vec KU2(3,NONINIT);
    KU2(0)=-ca2;
    KU2(1)=-sa2;
    KU2(2)=0;
    const Vec U2=circle1->getFrame()->getOrientation()*KU2;

    const Vec vC1 = cpData[icircle0].getFrameOfReference().getVelocity();
    const Vec vC2 = cpData[icircle1].getFrameOfReference().getVelocity();
    const Vec Om1 = cpData[icircle0].getFrameOfReference().getAngularVelocity();
    const Vec Om2 = cpData[icircle1].getFrameOfReference().getAngularVelocity();

    SqrMat A(2,2,NONINIT);
    A(0,0)=-u1.T()*R1;
    A(0,1)=u1.T()*R2;
    A(1,0)=u2.T()*N1;
    A(1,1)=n1.T()*U2;
    Vec b(2,NONINIT);
    b(0)=-u1.T()*(vC2-vC1);
    b(1)=-v2.T()*(Om2-Om1);
    const Vec zetad = slvLU(A,b);

    const Mat tOm1 = tilde(Om1);
    const Mat tOm2 = tilde(Om2);
    
    wb(0) += ((vC2-vC1).T()*N1-n1.T()*tOm1*R1)*zetad(0)+n1.T()*tOm2*R2*zetad(1)-n1.T()*tOm1*(vC2-vC1);
    if (wb.size()>1) 
      wb(1) += ((vC2-vC1).T()*U1-u1.T()*tOm1*R1)*zetad(0)+u1.T()*tOm2*R2*zetad(1)-u1.T()*tOm1*(vC2-vC1);
  }
}

