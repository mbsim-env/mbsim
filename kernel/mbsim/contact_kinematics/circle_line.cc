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
#include "circle_line.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/line.h"
#include "mbsim/contours/circle.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsCircleLine::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Circle*>(contour[0])) {
      icircle = 0; iline = 1;
      circle = static_cast<Circle*>(contour[0]);
      line = static_cast<Line*>(contour[1]);
    } 
    else {
      icircle = 1; iline = 0;
      circle = static_cast<Circle*>(contour[1]);
      line = static_cast<Line*>(contour[0]);
    }
  }

  void ContactKinematicsCircleLine::updateg(double &g, std::vector<ContourFrame*> &cFrame, int index) {

    cFrame[iline]->setOrientation(line->getFrame()->evalOrientation());
    cFrame[icircle]->getOrientation(false).set(0, -line->getFrame()->getOrientation().col(0));
    cFrame[icircle]->getOrientation(false).set(1, -line->getFrame()->getOrientation().col(1));
    cFrame[icircle]->getOrientation(false).set(2, line->getFrame()->getOrientation().col(2));

    Vec3 Wn = cFrame[iline]->getOrientation(false).col(0);

    Vec3 Wd = circle->getFrame()->evalPosition() - line->getFrame()->evalPosition();

    g = Wn.T()*Wd - circle->getRadius();

    cFrame[icircle]->setPosition(circle->getFrame()->getPosition() - Wn*circle->getRadius());
    cFrame[iline]->setPosition(cFrame[icircle]->getPosition(false) - Wn*g);
  }

  void ContactKinematicsCircleLine::updatewb(Vec &wb, double g, std::vector<ContourFrame*> &cFrame) {

    Vec3 v2 = cFrame[icircle]->evalOrientation().col(2);
    Vec3 n1 = cFrame[iline]->evalOrientation().col(0);
    // Vec3 n2 = cFrame[icircle]->getOrientation().col(0);
    Vec3 u1 = cFrame[iline]->getOrientation().col(1);
    Vec3 u2 = cFrame[icircle]->getOrientation().col(1);
    Vec3 vC1 = cFrame[iline]->evalVelocity();
    Vec3 vC2 = cFrame[icircle]->evalVelocity();
    Vec3 Om1 = cFrame[iline]->evalAngularVelocity();
    Vec3 Om2 = cFrame[icircle]->evalAngularVelocity();
    double r = circle->getRadius();

    double ad2 = -v2.T()*(Om2-Om1);
    double ad1 = u1.T()*(vC2-vC1) - r*ad2;
    Vec3 s2 = u2*r;

    wb(0) += n1.T()*(-crossProduct(Om1,vC2-vC1) - crossProduct(Om1,u1)*ad1 + crossProduct(Om2,s2)*ad2);
    
    if(wb.size() > 1) 
      wb(1) += u1.T()*(-crossProduct(Om1,vC2-vC1) - crossProduct(Om1,u1)*ad1 + crossProduct(Om2,s2)*ad2);
  }
      
}

