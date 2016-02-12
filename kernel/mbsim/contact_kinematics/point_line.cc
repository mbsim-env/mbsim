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
#include "mbsim/frame.h"
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

  void ContactKinematicsPointLine::updateg(double t, double &g, std::vector<Frame*> &cFrame, int index) {
    cFrame[iline]->setOrientation(line->getFrame()->getOrientation(t));
    cFrame[ipoint]->getOrientation(false).set(0, -line->getFrame()->getOrientation().col(0));
    cFrame[ipoint]->getOrientation(false).set(1, -line->getFrame()->getOrientation().col(1));
    cFrame[ipoint]->getOrientation(false).set(2, line->getFrame()->getOrientation().col(2));

    Vec3 Wn = cFrame[iline]->getOrientation(false).col(0);

    Vec3 Wd =  point->getFrame()->getPosition(t) - line->getFrame()->getPosition(t);

    g = Wn.T()*Wd;

    cFrame[ipoint]->setPosition(point->getFrame()->getPosition());
    cFrame[iline]->setPosition(cFrame[ipoint]->getPosition(false) - Wn*g);
  }

  void ContactKinematicsPointLine::updatewb(double t, Vec &wb, double g, std::vector<Frame*> &cFrame) {
    Vec3 n1 = cFrame[iline]->getOrientation(t).col(0);
    Vec3 u1 = cFrame[iline]->getOrientation().col(1);
    Vec3 vC1 = cFrame[iline]->getVelocity(t);
    Vec3 vC2 = cFrame[ipoint]->getVelocity(t);
    Vec3 Om1 = cFrame[iline]->getAngularVelocity(t);
    // Vec3 Om2 = cFrame[ipoint]->getAngularVelocity();

    double sd1 = u1.T()*(vC2 - vC1); 

    wb(0) += n1.T()*(-crossProduct(Om1,vC2-vC1) - crossProduct(Om1,u1)*sd1);

    if(wb.size() > 1) 
      wb(1) += u1.T()*(-crossProduct(Om1,vC2-vC1) - crossProduct(Om1,u1)*sd1);
  }
}

