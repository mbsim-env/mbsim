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
#include "mbsim/frames/contour_frame.h"
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

  void ContactKinematicsPointLine::updateg(SingleContact &contact, int i) {
    contact.getContourFrame(iline)->setOrientation(line->getFrame()->evalOrientation());
    contact.getContourFrame(ipoint)->getOrientation(false).set(0, -line->getFrame()->getOrientation().col(0));
    contact.getContourFrame(ipoint)->getOrientation(false).set(1, -line->getFrame()->getOrientation().col(1));
    contact.getContourFrame(ipoint)->getOrientation(false).set(2, line->getFrame()->getOrientation().col(2));

    Vec3 Wn = contact.getContourFrame(iline)->getOrientation(false).col(0);

    Vec3 Wd =  point->getFrame()->evalPosition() - line->getFrame()->evalPosition();

    double g = Wn.T()*Wd;

    contact.getContourFrame(ipoint)->setPosition(point->getFrame()->getPosition());
    contact.getContourFrame(iline)->setPosition(contact.getContourFrame(ipoint)->getPosition(false) - Wn*g);
    contact.getGeneralizedRelativePosition(false)(0) = g;
  }

  void ContactKinematicsPointLine::updatewb(SingleContact &contact, int i) {
    const Vec3 n1 = contact.getContourFrame(iline)->evalOrientation().col(0);
    const Vec3 u1 = contact.getContourFrame(iline)->getOrientation().col(1);
    const Vec3 &R1 = u1;
    const Vec3 paruPart1 = crossProduct(contact.getContourFrame(iline)->evalAngularVelocity(),u1);
    const Vec3 parnPart1 = crossProduct(contact.getContourFrame(iline)->getAngularVelocity(),n1);
    const Vec3 parWvCParEta1 = crossProduct(contact.getContourFrame(iline)->getAngularVelocity(),R1);
    const Vec3 vC1 = contact.getContourFrame(iline)->evalVelocity();
    const Vec3 vC2 = contact.getContourFrame(ipoint)->evalVelocity();

    double sd1 = -u1.T()*(vC2-vC1);

    contact.getwb(false)(0) += parnPart1.T()*(vC2-vC1)+n1.T()*(parWvCParEta1*sd1);
    if(contact.getwb(false).size() > 1)
      contact.getwb(false)(1) += paruPart1.T()*(vC2-vC1)+u1.T()*(parWvCParEta1*sd1);
  }

}
