/* Copyright (C) 2004-2016 MBSim Development Team
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
#include "point_spatialcontour.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/point.h"
#include "mbsim/functions/contact/funcpair_spatialcontour_point.h"
#include "mbsim/utils/spatial_contact_search.h"
#include "mbsim/utils/contact_utils.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContactKinematicsPointSpatialContour::~ContactKinematicsPointSpatialContour() {
    delete func;
  }

  void ContactKinematicsPointSpatialContour::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      ispatialcontour = 1;
      point = static_cast<Point*>(contour[0]);
      spatialcontour = static_cast<Contour*>(contour[1]);
    }
    else {
      ipoint = 1;
      ispatialcontour = 0;
      point = static_cast<Point*>(contour[1]);
      spatialcontour = static_cast<Contour*>(contour[0]);
    }
    func = new FuncPairSpatialContourPoint(point,spatialcontour);
  }

  void ContactKinematicsPointSpatialContour::setInitialGuess(const fmatvec::MatV &zeta0_) {
    if(zeta0_.rows()) {
      if(zeta0_.rows() != 1 or zeta0_.cols() != 2) throw runtime_error("(ContactKinematicsPointSpatialContour::assignContours): size of zeta0 does not match");
      zeta0 = zeta0_.row(0).T();
    }
  }

  void ContactKinematicsPointSpatialContour::updateg(SingleContact &contact, int i) {

    SpatialContactSearch search(func);
    search.setTolerance(tol);

    if ((!spatialcontour->getEtaNodes().empty()) && (!spatialcontour->getXiNodes().empty()))
      search.setNodes(spatialcontour->getEtaNodes(), spatialcontour->getXiNodes());
    else
      search.setEqualSpacing(10, 10, 0, 0, 0.1, 0.1);

    if (!searchAllCP) {
      search.setInitialValue(zeta0);
    }
    else {
      search.setSearchAll(true);
      searchAllCP = false;
    }

    zeta0 = search.slv();
    contact.getContourFrame(ispatialcontour)->setZeta(zeta0);

    contact.getContourFrame(ispatialcontour)->getOrientation(false).set(0, spatialcontour->evalWn(contact.getContourFrame(ispatialcontour)->getZeta(false)));
    contact.getContourFrame(ispatialcontour)->getOrientation(false).set(1, spatialcontour->evalWu(contact.getContourFrame(ispatialcontour)->getZeta(false)));
    contact.getContourFrame(ispatialcontour)->getOrientation(false).set(2, spatialcontour->evalWv(contact.getContourFrame(ispatialcontour)->getZeta(false)));

    contact.getContourFrame(ipoint)->getOrientation(false).set(0, -contact.getContourFrame(ispatialcontour)->getOrientation(false).col(0));
    contact.getContourFrame(ipoint)->getOrientation(false).set(1, -contact.getContourFrame(ispatialcontour)->getOrientation(false).col(1));
    contact.getContourFrame(ipoint)->getOrientation(false).set(2, contact.getContourFrame(ispatialcontour)->getOrientation(false).col(2));

    contact.getContourFrame(ispatialcontour)->setPosition(spatialcontour->evalPosition(contact.getContourFrame(ispatialcontour)->getZeta(false)));
    contact.getContourFrame(ipoint)->setPosition(point->getFrame()->evalPosition());

    double g;
    if(spatialcontour->isZetaOutside(contact.getContourFrame(ispatialcontour)->getZeta(false)))
      g = 1;
    else
      g = contact.getContourFrame(ispatialcontour)->getOrientation(false).col(0).T() * (contact.getContourFrame(ipoint)->getPosition(false) - contact.getContourFrame(ispatialcontour)->getPosition(false));
    if(g < -spatialcontour->getThickness()) g = 1;
    contact.getGeneralizedRelativePosition(false)(0) = g;
  }

  void ContactKinematicsPointSpatialContour::updatewb(SingleContact &contact, int i) {
    const Vec3 n1 = contact.getContourFrame(ipoint)->evalOrientation().col(0);
    const Vec3 u1 = contact.getContourFrame(ipoint)->getOrientation().col(1);
    const Vec3 v1 = contact.getContourFrame(ipoint)->getOrientation().col(2);
    Vec2 zeta1 = computeAnglesOnUnitSphere(point->getFrame()->evalOrientation().T()*n1);
    const Mat3x2 U1 = point->evalWU(zeta1);
    const Mat3x2 V1 = point->evalWV(zeta1);
    const Mat3x2 N1 = point->evalWN(zeta1);
    const Vec3 paruPart1 = crossProduct(contact.getContourFrame(ipoint)->evalAngularVelocity(),u1);
    const Vec3 parvPart1 = crossProduct(contact.getContourFrame(ipoint)->getAngularVelocity(),v1);
    const Vec3 parnPart1 = crossProduct(contact.getContourFrame(ipoint)->getAngularVelocity(),n1);

    const Vec3 u2 = contact.getContourFrame(ispatialcontour)->evalOrientation().col(1);
    const Vec3 v2 = contact.getContourFrame(ispatialcontour)->getOrientation().col(2);
    const Mat3x2 R2 = spatialcontour->evalWR(contact.getContourFrame(ispatialcontour)->getZeta());
    const Mat3x2 U2 = spatialcontour->evalWU(contact.getContourFrame(ispatialcontour)->getZeta());
    const Mat3x2 V2 = spatialcontour->evalWV(contact.getContourFrame(ispatialcontour)->getZeta());
    const Vec3 paruPart2 = spatialcontour->evalParWuPart(contact.getContourFrame(ispatialcontour)->getZeta());
    const Vec3 parvPart2 = spatialcontour->evalParWvPart(contact.getContourFrame(ispatialcontour)->getZeta());
    const Mat3x2 parWvCParZeta2 = spatialcontour->evalParWvCParZeta(contact.getContourFrame(ispatialcontour)->getZeta());

    const Vec3 vC1 = contact.getContourFrame(ipoint)->evalVelocity();
    const Vec3 vC2 = contact.getContourFrame(ispatialcontour)->evalVelocity();

    SqrMat A(4,NONINIT);
    A(RangeV(0,0),RangeV(0,1)).init(0);
    A(RangeV(0,0),RangeV(2,3)) = u1.T()*R2;
    A(RangeV(1,1),RangeV(0,1)).init(0);
    A(RangeV(1,1),RangeV(2,3)) = v1.T()*R2;
    A(RangeV(2,2),RangeV(0,1)) = u2.T()*N1;
    A(RangeV(2,2),RangeV(2,3)) = n1.T()*U2;
    A(RangeV(3,3),RangeV(0,1)) = v2.T()*N1;
    A(RangeV(3,3),RangeV(2,3)) = n1.T()*V2;

    Vec b(4,NONINIT);
    b(0) = -u1.T()*(vC2-vC1);
    b(1) = -v1.T()*(vC2-vC1);
    b(2) = -u2.T()*parnPart1-n1.T()*paruPart2;
    b(3) = -v2.T()*parnPart1-n1.T()*parvPart2;
    Vec zetad =  slvLU(A,b);
    Vec zetad1 = zetad(0,1);
    Vec zetad2 = zetad(2,3);

    contact.getwb(false)(0) += (N1*zetad1+parnPart1).T()*(vC2-vC1)+n1.T()*parWvCParZeta2*zetad2;
    if (contact.getwb(false).size()>1) {
      contact.getwb(false)(1) += (U1*zetad1+paruPart1).T()*(vC2-vC1)+u1.T()*parWvCParZeta2*zetad2;
      if (contact.getwb(false).size()>2)
        contact.getwb(false)(2) += (V1*zetad1+parvPart1).T()*(vC2-vC1)+v1.T()*parWvCParZeta2*zetad2;
    }
  }

}
