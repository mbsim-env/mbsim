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
#include "point_plane.h"
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

  void ContactKinematicsPointPlane::updateg(Vec &g, ContourPointData* cpData, int index) {
    cpData[iplane].getFrameOfReference().setOrientation(plane->getFrame()->getOrientation()); // data of possible contact point
    cpData[ipoint].getFrameOfReference().getOrientation().col(0) = -plane->getFrame()->getOrientation().col(0);
    cpData[ipoint].getFrameOfReference().getOrientation().col(1) = -plane->getFrame()->getOrientation().col(1);
    cpData[ipoint].getFrameOfReference().getOrientation().col(2) = plane->getFrame()->getOrientation().col(2);

    Vec Wn = cpData[iplane].getFrameOfReference().getOrientation().col(0); // normal is first vector of coordinate orientation

    Vec Wd =  point->getFrame()->getPosition() - plane->getFrame()->getPosition();

    g(0) = Wn.T()*Wd; // distance

    cpData[ipoint].getFrameOfReference().setPosition(point->getFrame()->getPosition()); // possible contact locations
    cpData[iplane].getFrameOfReference().setPosition(cpData[ipoint].getFrameOfReference().getPosition() - Wn*g(0));
  }

  void ContactKinematicsPointPlane::updatewb(Vec &wb, const Vec &g, ContourPointData *cpData) {
    if(wb.size()) { // check whether contact is closed

      Vec v1 = cpData[iplane].getFrameOfReference().getOrientation().col(2); // second tangential vector in contact
      Vec n1 = cpData[iplane].getFrameOfReference().getOrientation().col(0); // normal in contact
      Vec u1 = cpData[iplane].getFrameOfReference().getOrientation().col(1); // first tangential vector in contact
      Vec vC1 = cpData[iplane].getFrameOfReference().getVelocity(); // velocity of possible plane contact
      Vec vC2 = cpData[ipoint].getFrameOfReference().getVelocity(); // velocity of point
      Vec Om1 = cpData[iplane].getFrameOfReference().getAngularVelocity(); // angular velocity of possible plane contact

      Vec s1 = u1;
      Vec t1 = v1;

      Mat R1(3,2);
      R1.col(0) = s1;
      R1.col(1) = t1;

      SqrMat A(2,2,NONINIT);
      A(Index(0,0),Index(0,1)) = -u1.T()*R1; // first matrix row
      A(Index(1,1),Index(0,1)) = -v1.T()*R1; // second matrix row

      Vec b(2,NONINIT);
      b(0) = -u1.T()*(vC2-vC1);
      b(1) = -v1.T()*(vC2-vC1);
      Vec zetad1 =  slvLU(A,b);

      Mat tOm1 = tilde(Om1); // tilde operator
      wb(0) += n1.T()*(-tOm1*(vC2-vC1) - tOm1*R1*zetad1); // acceleration in terms of contour parametrisation

      if(wb.size() > 1) {
        wb(1) += u1.T()*(-tOm1*(vC2-vC1) - tOm1*R1*zetad1);
        wb(2) += v1.T()*(-tOm1*(vC2-vC1) - tOm1*R1*zetad1);
      }
    }
  }

}

