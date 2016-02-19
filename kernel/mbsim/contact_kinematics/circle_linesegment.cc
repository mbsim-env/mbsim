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
#include "circle_linesegment.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/line_segment.h"
#include "mbsim/contours/circle.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsCircleLineSegment::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Circle*>(contour[0])) {
      icircle = 0; iline = 1;
      circle = static_cast<Circle*>(contour[0]);
      linesegment = static_cast<LineSegment*>(contour[1]);
    } 
    else {
      icircle = 1; iline = 0;
      circle = static_cast<Circle*>(contour[1]);
      linesegment = static_cast<LineSegment*>(contour[0]);
    }
  }

  void ContactKinematicsCircleLineSegment::updateg(double t, double &g, std::vector<ContourFrame*> &cFrame, int index) {

    const Vec3 WC=circle->getFrame()->getPosition(t);
    const Vec3 WL=linesegment->getFrame()->getPosition(t);
    const Vec3 WLdir=linesegment->getFrame()->getOrientation().col(1);
    const Vec3 WL0=WL-linesegment->getLength()/2*WLdir;
    const double s=WLdir.T() * (-WL0+WC);

    if ((s>=0) && (s<=linesegment->getLength())) {
      cFrame[iline]->setOrientation(linesegment->getFrame()->getOrientation());
      cFrame[icircle]->getOrientation(false).set(0, -linesegment->getFrame()->getOrientation(false).col(0));
      cFrame[icircle]->getOrientation(false).set(1, -linesegment->getFrame()->getOrientation(false).col(1));
      cFrame[icircle]->getOrientation(false).set(2, linesegment->getFrame()->getOrientation(false).col(2));
      g = cFrame[iline]->getOrientation(false).col(0).T()*(WC - WL) - circle->getRadius();
      cFrame[icircle]->setPosition(WC - cFrame[iline]->getOrientation(false).col(0)*circle->getRadius());
      cFrame[iline]->setPosition(cFrame[icircle]->getPosition(false) - cFrame[iline]->getOrientation(false).col(0)*g);
    }
    else {
      cFrame[iline]->setPosition((s<0)?WL0:WL+linesegment->getLength()/2*WLdir);
      const Vec3 WrD = -WC + cFrame[iline]->getPosition(false);
      cFrame[icircle]->getOrientation(false).set(0, WrD/nrm2(WrD));
      cFrame[iline]->getOrientation(false).set(0, -cFrame[icircle]->getOrientation(false).col(0));
      cFrame[icircle]->getOrientation(false).set(2, circle->getFrame()->getOrientation(false).col(2));
      cFrame[iline]->getOrientation(false).set(2, linesegment->getFrame()->getOrientation(false).col(2));
      cFrame[icircle]->getOrientation(false).set(1, crossProduct(cFrame[icircle]->getOrientation(false).col(2), cFrame[icircle]->getOrientation(false).col(0)));
      cFrame[iline]->getOrientation(false).set(1, -cFrame[icircle]->getOrientation(false).col(1));
      cFrame[icircle]->setPosition(WC + cFrame[icircle]->getOrientation(false).col(0)*circle->getRadius());
      g = cFrame[icircle]->getOrientation(false).col(0).T()*WrD - circle->getRadius();
    }
  }

  void ContactKinematicsCircleLineSegment::updatewb(double t, Vec &wb, double g, std::vector<ContourFrame*> &cFrame) {
    const Vec3 WC=circle->getFrame()->getPosition(t);
    const Vec3 WL=linesegment->getFrame()->getPosition(t);
    const Vec3 WLdir=linesegment->getFrame()->getOrientation().col(1);
    const Vec3 WL0=WL-linesegment->getLength()/2*WLdir;
    const double s=WLdir.T() * (-WL0+WC);

    if ((s>=0) && (s<=linesegment->getLength())) {
      Vec3 v2 = cFrame[icircle]->getOrientation(t).col(2);
      Vec3 n1 = cFrame[iline]->getOrientation(t).col(0);
      Vec3 u1 = cFrame[iline]->getOrientation().col(1);
      Vec3 u2 = cFrame[icircle]->getOrientation().col(1);
      Vec3 vC1 = cFrame[iline]->getVelocity(t);
      Vec3 vC2 = cFrame[icircle]->getVelocity(t);
      Vec3 Om1 = cFrame[iline]->getAngularVelocity();
      Vec3 Om2 = cFrame[icircle]->getAngularVelocity();
      double r = circle->getRadius();

      double ad2 = -v2.T()*(Om2-Om1);
      double ad1 = u1.T()*(vC2-vC1) - r*ad2;
      Vec3 s2 = u2*r;

      wb(0) += n1.T()*(-crossProduct(Om1,vC2-vC1) - crossProduct(Om1,u1)*ad1 + crossProduct(Om2,s2)*ad2);

      if(wb.size() > 1) 
        wb(1) += u1.T()*(-crossProduct(Om1,vC2-vC1) - crossProduct(Om1,u1)*ad1 + crossProduct(Om2,s2)*ad2);
    }
    else
      throw runtime_error("ContactKinematicsCircleLineSegment::updatewb not implemented for contact on edge.");
  }
      
}

