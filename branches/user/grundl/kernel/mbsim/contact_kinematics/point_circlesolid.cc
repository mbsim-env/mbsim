/* Copyright (C) 2004-2010 MBSim Development Team
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
#include "point_circlesolid.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/circle_solid.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsPointCircleSolid::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      icirclesolid = 1;
      point = static_cast<Point*>(contour[0]);
      circlesolid = static_cast<CircleSolid*>(contour[1]);
    } 
    else {
      ipoint = 1;
      icirclesolid = 0;
      point = static_cast<Point*>(contour[1]);
      circlesolid = static_cast<CircleSolid*>(contour[0]);
    }
  }

  void ContactKinematicsPointCircleSolid::updateg(Vec &g, ContourPointData *cpData, int index) {
    const Vec3 WrD = -circlesolid->getFrame()->getPosition() + point->getFrame()->getPosition();
    
    cpData[icirclesolid].getFrameOfReference().getOrientation().set(0, WrD/nrm2(WrD));
    cpData[ipoint].getFrameOfReference().getOrientation().set(0, -cpData[icirclesolid].getFrameOfReference().getOrientation().col(0));
    
    cpData[icirclesolid].getFrameOfReference().getOrientation().set(2, circlesolid->getFrame()->getOrientation().col(2));
    cpData[ipoint].getFrameOfReference().getOrientation().set(2, point->getFrame()->getOrientation().col(2));
    
    cpData[icirclesolid].getFrameOfReference().getOrientation().set(1, crossProduct(cpData[icirclesolid].getFrameOfReference().getOrientation().col(2), cpData[icirclesolid].getFrameOfReference().getOrientation().col(0)));
    cpData[ipoint].getFrameOfReference().getOrientation().set(1, -cpData[icirclesolid].getFrameOfReference().getOrientation().col(1));
    
    cpData[icirclesolid].getFrameOfReference().getPosition() = circlesolid->getFrame()->getPosition() + cpData[icirclesolid].getFrameOfReference().getOrientation().col(0)*circlesolid->getRadius();
    cpData[ipoint].getFrameOfReference().getPosition() = point->getFrame()->getPosition();

    g(0) = cpData[icirclesolid].getFrameOfReference().getOrientation().col(0).T()*WrD - circlesolid->getRadius();
  }

  void ContactKinematicsPointCircleSolid::updatewb(Vec &wb, const Vec &g, ContourPointData *cpData) {

    const Vec KrPC1 = circlesolid->getFrame()->getOrientation().T()*(cpData[icirclesolid].getFrameOfReference().getPosition() - circlesolid->getFrame()->getPosition());
    const double zeta1=(KrPC1(1)>0) ? acos(KrPC1(0)/nrm2(KrPC1)) : 2.*M_PI - acos(KrPC1(0)/nrm2(KrPC1));
    const double sa1=sin(zeta1);
    const double ca1=cos(zeta1);
    const double r1=circlesolid->getRadius();
    Vec Ks1(3, NONINIT);
    Ks1(0)=-r1*sa1;
    Ks1(1)=r1*ca1;
    Ks1(2)=0;
    Vec Kt1(3, NONINIT);
    Kt1(0)=0;
    Kt1(1)=0;
    Kt1(2)=1;
    const Vec s1=circlesolid->getFrame()->getOrientation()*Ks1;
    const Vec t1=circlesolid->getFrame()->getOrientation()*Kt1;
    Vec n1=crossProduct(s1, t1);
    n1/=nrm2(n1);
    const Vec u1=s1/nrm2(s1);
    const Vec R1(s1);
    Vec KN1(3,NONINIT);
    KN1(0)=-sa1;
    KN1(1)=ca1;
    KN1(2)=0;
    const Vec N1=circlesolid->getFrame()->getOrientation()*KN1;
    Vec KU1(3,NONINIT);
    KU1(0)=-ca1;
    KU1(1)=-sa1;
    KU1(2)=0;
    const Vec U1=circlesolid->getFrame()->getOrientation()*KU1;

    const Vec vC1 = cpData[icirclesolid].getFrameOfReference().getVelocity();
    const Vec vC2 = cpData[ipoint].getFrameOfReference().getVelocity();
    const Vec Om1 = cpData[icirclesolid].getFrameOfReference().getAngularVelocity();

    const double zetad = u1.T()*(vC2-vC1)/(u1.T()*R1);

    const Mat tOm1 = tilde(Om1);

    wb(0) += ((vC2-vC1).T()*N1-n1.T()*tOm1*R1)*zetad-n1.T()*tOm1*(vC2-vC1);
    if (wb.size()>1) 
      wb(1) += ((vC2-vC1).T()*U1-u1.T()*tOm1*R1)*zetad-u1.T()*tOm1*(vC2-vC1);
  }
}

