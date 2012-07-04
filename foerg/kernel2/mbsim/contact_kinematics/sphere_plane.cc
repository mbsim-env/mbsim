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

  void ContactKinematicsSpherePlane::updateg(Vec &g, ContourPointData *cpData) {
    cpData[iplane].getFrameOfReference().setOrientation(plane->getFrame()->getOrientation());
    cpData[isphere].getFrameOfReference().getOrientation().set(0, -plane->getFrame()->getOrientation().col(0));
    cpData[isphere].getFrameOfReference().getOrientation().set(1, -plane->getFrame()->getOrientation().col(1));
    cpData[isphere].getFrameOfReference().getOrientation().set(2, plane->getFrame()->getOrientation().col(2));

    Vec3 Wn = cpData[iplane].getFrameOfReference().getOrientation().col(0);

    Vec3 Wd = sphere->getFrame()->getPosition() - plane->getFrame()->getPosition();

    g(0) = Wn.T()*Wd - sphere->getRadius();


    cpData[isphere].getFrameOfReference().setPosition(sphere->getFrame()->getPosition() - Wn*sphere->getRadius());
    cpData[iplane].getFrameOfReference().setPosition(cpData[isphere].getFrameOfReference().getPosition() - Wn*g(0));
  }

  void ContactKinematicsSpherePlane::updatewb(Vec &wb, const Vec &g, ContourPointData *cpData) {
    Vec3 v1 = cpData[iplane].getFrameOfReference().getOrientation().col(2);
    Vec3 n1 = cpData[iplane].getFrameOfReference().getOrientation().col(0);
    Vec3 n2 = cpData[isphere].getFrameOfReference().getOrientation().col(0);
    Vec3 u1 = cpData[iplane].getFrameOfReference().getOrientation().col(1);
    Vec3 vC1 = cpData[iplane].getFrameOfReference().getVelocity();
    Vec3 vC2 = cpData[isphere].getFrameOfReference().getVelocity();
    Vec3 Om1 = cpData[iplane].getFrameOfReference().getAngularVelocity();
    Vec3 Om2 = cpData[isphere].getFrameOfReference().getAngularVelocity();

    Vec3 KrPC2 = sphere->getFrame()->getOrientation().T()*(cpData[isphere].getFrameOfReference().getPosition() - sphere->getFrame()->getPosition());
    Vec2 zeta2 = computeAnglesOnUnitSphere(KrPC2/sphere->getRadius());
    double a2 = zeta2(0);
    double b2 = zeta2(1);
    Vec3 &s1 = u1;
    Vec3 &t1 = v1;

    double r = sphere->getRadius();
    Mat32 KR2(NONINIT);
    Vec3 Ks2(NONINIT);
    Ks2(0) = -r*sin(a2)*cos(b2);
    Ks2(1) = r*cos(a2)*cos(b2);
    Ks2(2) = 0;

    Vec3 Kt2(NONINIT);
    Kt2(0) = -r*cos(a2)*sin(b2);
    Kt2(1) = -r*sin(a2)*sin(b2);
    Kt2(2) = r*cos(b2);

    Vec3 s2 = sphere->getFrame()->getOrientation()*Ks2;
    Vec3 t2 = sphere->getFrame()->getOrientation()*Kt2;
    Vec3 u2 = s2/nrm2(s2);
    Vec3 v2 = crossProduct(n2,u2);

    Mat32 R1;
    R1.set(0, s1);
    R1.set(1, t1);

    Mat32 R2;
    R2.set(0, s2);
    R2.set(1, t2);

    Mat32 KU2(NONINIT);
    KU2(0,0) = -cos(a2);
    KU2(1,0) = -sin(a2);
    KU2(2,0) = 0;
    KU2(0,1) = 0;
    KU2(1,1) = 0;
    KU2(2,1) = 0;

    Mat32 KV2(NONINIT);
    KV2(0,0) = sin(a2)*sin(b2);
    KV2(1,0) = -cos(a2)*sin(b2);
    KV2(2,0) = 0;
    KV2(0,1) = -cos(a2)*cos(b2);
    KV2(1,1) = -sin(a2)*cos(b2);
    KV2(2,1) = -sin(b2);

    Mat32 U2 = sphere->getFrame()->getOrientation()*KU2;
    Mat32 V2 = sphere->getFrame()->getOrientation()*KV2;

    SqrMat A(4,4,NONINIT);
    A(Index(0,0),Index(0,1)) = -u1.T()*R1;
    A(Index(0,0),Index(2,3)) = u1.T()*R2;
    A(Index(1,1),Index(0,1)) = -v1.T()*R1;
    A(Index(1,1),Index(2,3)) = v1.T()*R2;
    A(Index(2,2),Index(0,1)).init(0);
    A(Index(2,2),Index(2,3)) = n1.T()*U2;
    A(Index(3,3),Index(0,1)).init(0);
    A(Index(3,3),Index(2,3)) = n1.T()*V2;

    Vec b(4,NONINIT);
    b(0) = -u1.T()*(vC2-vC1);
    b(1) = -v1.T()*(vC2-vC1);
    b(2) = -v2.T()*(Om2-Om1);
    b(3) = u2.T()*(Om2-Om1);
    Vec zetad =  slvLU(A,b);
    Vec zetad1 = zetad(0,1);
    Vec zetad2 = zetad(2,3);

    Mat33 tOm1 = tilde(Om1);
    Mat33 tOm2 = tilde(Om2);
    wb(0) += n1.T()*(-tOm1*(vC2-vC1) - tOm1*R1*zetad1 + tOm2*R2*zetad2);

    if(wb.size() > 1) wb(1) += u1.T()*(-tOm1*(vC2-vC1) - tOm1*R1*zetad1 + tOm2*R2*zetad2);
    if(wb.size() > 2) wb(2) += v1.T()*(-tOm1*(vC2-vC1) - tOm1*R1*zetad1 + tOm2*R2*zetad2);
  }

}

