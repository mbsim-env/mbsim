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
#include "point_line.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/line.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsPointLine::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0; iline = 1;
      point = static_cast<Point*>(contour[0]);
      line = static_cast<Line*>(contour[1]);
    } 
    else {
      ipoint = 1; iline = 0;
      point = static_cast<Point*>(contour[1]);
      line = static_cast<Line*>(contour[0]);
    }
  }

  void ContactKinematicsPointLine::updateg(Vec &g, ContourPointData *cpData, int index) {
    cpData[iline].getFrameOfReference().setOrientation(line->getFrame()->getOrientation());
    cpData[ipoint].getFrameOfReference().getOrientation().set(0, -line->getFrame()->getOrientation().col(0));
    cpData[ipoint].getFrameOfReference().getOrientation().set(1, -line->getFrame()->getOrientation().col(1));
    cpData[ipoint].getFrameOfReference().getOrientation().set(2, line->getFrame()->getOrientation().col(2));

    Vec3 Wn = cpData[iline].getFrameOfReference().getOrientation().col(0);

    Vec3 Wd =  point->getFrame()->getPosition() - line->getFrame()->getPosition();

    g(0) = Wn.T()*Wd;

    cpData[ipoint].getFrameOfReference().setPosition(point->getFrame()->getPosition());
    cpData[iline].getFrameOfReference().setPosition(cpData[ipoint].getFrameOfReference().getPosition() - Wn*g(0));
  }

  void ContactKinematicsPointLine::updatewb(Vec &wb, const Vec &g, ContourPointData *cpData) {
    Vec3 n1 = cpData[iline].getFrameOfReference().getOrientation().col(0);
    Vec3 u1 = cpData[iline].getFrameOfReference().getOrientation().col(1);
    Vec3 vC1 = cpData[iline].getFrameOfReference().getVelocity();
    Vec3 vC2 = cpData[ipoint].getFrameOfReference().getVelocity();
    Vec3 Om1 = cpData[iline].getFrameOfReference().getAngularVelocity();
    // Vec3 Om2 = cpData[ipoint].getFrameOfReference().getAngularVelocity();

    double sd1 = u1.T()*(vC2 - vC1); 

    wb(0) += n1.T()*(-crossProduct(Om1,vC2-vC1) - crossProduct(Om1,u1)*sd1);

    if(wb.size() > 1) 
      wb(1) += u1.T()*(-crossProduct(Om1,vC2-vC1) - crossProduct(Om1,u1)*sd1);
  }
}

