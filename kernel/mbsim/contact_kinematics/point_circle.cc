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
#include "point_circle.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/circle.h"
#include "mbsim/utils/contact_utils.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsPointCircle::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      icircle = 1;
      point = static_cast<Point*>(contour[0]);
      circle = static_cast<Circle*>(contour[1]);
    } 
    else {
      ipoint = 1;
      icircle = 0;
      point = static_cast<Point*>(contour[1]);
      circle = static_cast<Circle*>(contour[0]);
    }
  }

  void ContactKinematicsPointCircle::updateg(SingleContact &contact, int i) {
    const Vec3 WrD = point->getFrame()->evalPosition() - circle->getFrame()->evalPosition();
    
    contact.getContourFrame(icircle)->getOrientation(false).set(0, WrD/nrm2(WrD));
    contact.getContourFrame(icircle)->setEta(computeAngleOnUnitCircle(circle->getFrame()->evalOrientation().T()*contact.getContourFrame(icircle)->getOrientation(false).col(0)));
    contact.getContourFrame(icircle)->getOrientation(false).set(2, circle->getFrame()->getOrientation(false).col(2));
    contact.getContourFrame(icircle)->getOrientation(false).set(1, crossProduct(contact.getContourFrame(icircle)->getOrientation(false).col(2), contact.getContourFrame(icircle)->getOrientation(false).col(0)));

    contact.getContourFrame(ipoint)->getOrientation(false).set(0, -contact.getContourFrame(icircle)->getOrientation(false).col(0));
    contact.getContourFrame(ipoint)->getOrientation(false).set(1, -contact.getContourFrame(icircle)->getOrientation(false).col(1));
    contact.getContourFrame(ipoint)->getOrientation(false).set(2, contact.getContourFrame(icircle)->getOrientation(false).col(2));
    
    contact.getContourFrame(icircle)->setPosition(circle->getFrame()->getPosition() + contact.getContourFrame(icircle)->getOrientation(false).col(0)*circle->getRadius());
    contact.getContourFrame(ipoint)->setPosition(point->getFrame()->getPosition());

    contact.getGeneralizedRelativePosition(false)(0) = contact.getContourFrame(icircle)->getOrientation(false).col(0).T()*WrD - circle->getRadius();
  }

  void ContactKinematicsPointCircle::updatewb(SingleContact &contact, int i) {

    const Vec3 n1 = contact.getContourFrame(icircle)->getOrientation().col(0);
    const Vec3 u1 = circle->evalWu(contact.getContourFrame(icircle)->getZeta());
    const Vec3 R1 = circle->evalWs(contact.getContourFrame(icircle)->getZeta());
    const Vec3 N1 = circle->evalParDer1Wn(contact.getContourFrame(icircle)->getZeta());
    const Vec3 U1 = circle->evalParDer1Wu(contact.getContourFrame(icircle)->getZeta());

    const Vec vC1 = contact.getContourFrame(icircle)->evalVelocity();
    const Vec vC2 = contact.getContourFrame(ipoint)->evalVelocity();
    const Vec Om1 = contact.getContourFrame(icircle)->getAngularVelocity();

    const double zetad = u1.T()*(vC2-vC1)/(u1.T()*R1);

    const Mat tOm1 = tilde(Om1);

    if(contact.isNormalForceLawSetValued())
      contact.getwb(false)(0) += ((vC2-vC1).T()*N1-n1.T()*tOm1*R1)*zetad-n1.T()*tOm1*(vC2-vC1);
    if(contact.isTangentialForceLawSetValuedAndActive())
      contact.getwb(false)(contact.isNormalForceLawSetValued()) += ((vC2-vC1).T()*U1-u1.T()*tOm1*R1)*zetad-u1.T()*tOm1*(vC2-vC1);
  }
}
