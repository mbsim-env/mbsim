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

  void ContactKinematicsPointCircleSolid::updateg(double t, double &g, ContourPointData *cpData, int index) {
    const Vec3 WrD = -circlesolid->getFrame()->getPosition(t) + point->getFrame()->getPosition(t);
    
    cpData[icirclesolid].getFrameOfReference().getOrientation(false).set(0, WrD/nrm2(WrD));
    cpData[ipoint].getFrameOfReference().getOrientation(false).set(0, -cpData[icirclesolid].getFrameOfReference().getOrientation(false).col(0));
    
    cpData[icirclesolid].getFrameOfReference().getOrientation(false).set(2, circlesolid->getFrame()->getOrientation().col(2));
    cpData[ipoint].getFrameOfReference().getOrientation(false).set(2, point->getFrame()->getOrientation().col(2));
    
    cpData[icirclesolid].getFrameOfReference().getOrientation(false).set(1, crossProduct(cpData[icirclesolid].getFrameOfReference().getOrientation(false).col(2), cpData[icirclesolid].getFrameOfReference().getOrientation(false).col(0)));
    cpData[ipoint].getFrameOfReference().getOrientation(false).set(1, -cpData[icirclesolid].getFrameOfReference().getOrientation(false).col(1));
    
    cpData[icirclesolid].getFrameOfReference().setPosition(circlesolid->getFrame()->getPosition() + cpData[icirclesolid].getFrameOfReference().getOrientation(false).col(0)*circlesolid->getRadius());
    cpData[ipoint].getFrameOfReference().setPosition(point->getFrame()->getPosition());

    g = cpData[icirclesolid].getFrameOfReference().getOrientation(false).col(0).T()*WrD - circlesolid->getRadius();
  }

  void ContactKinematicsPointCircleSolid::updatewb(double t, Vec &wb, double g, ContourPointData *cpData) {
    throw; // TODO: check implementation for the example that throws this exception

    const Vec KrPC1 = circlesolid->getFrame()->getOrientation(t).T()*(cpData[icirclesolid].getFrameOfReference().getPosition(t) - circlesolid->getFrame()->getPosition(t));
    Vec2 zeta1;
    zeta1(0)=(KrPC1(1)>0) ? acos(KrPC1(0)/nrm2(KrPC1)) : 2.*M_PI - acos(KrPC1(0)/nrm2(KrPC1));
    cpData[icirclesolid].setLagrangeParameterPosition(zeta1);

    const Vec3 n1 = cpData[icirclesolid].getFrameOfReference().getOrientation().col(0); //crossProduct(s1, t1);
    const Vec3 u1 = circlesolid->getWu(t,cpData[icirclesolid]);
    const Vec3 R1 = circlesolid->getWs(t,cpData[icirclesolid]);
    const Vec3 N1 = circlesolid->getParDer1Wn(t,cpData[icirclesolid]);
    const Vec3 U1 = circlesolid->getParDer1Wu(t,cpData[icirclesolid]);

    const Vec vC1 = cpData[icirclesolid].getFrameOfReference().getVelocity(t);
    const Vec vC2 = cpData[ipoint].getFrameOfReference().getVelocity(t);
    const Vec Om1 = cpData[icirclesolid].getFrameOfReference().getAngularVelocity();

    const double zetad = u1.T()*(vC2-vC1)/(u1.T()*R1);

    const Mat tOm1 = tilde(Om1);

    wb(0) += ((vC2-vC1).T()*N1-n1.T()*tOm1*R1)*zetad-n1.T()*tOm1*(vC2-vC1);
    if (wb.size()>1) 
      wb(1) += ((vC2-vC1).T()*U1-u1.T()*tOm1*R1)*zetad-u1.T()*tOm1*(vC2-vC1);
  }
}

