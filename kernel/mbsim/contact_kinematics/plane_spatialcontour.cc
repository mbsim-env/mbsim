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
#include "plane_spatialcontour.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/plane.h"
#include "mbsim/functions/contact/funcpair_spatialcontour_plane.h"
#include "mbsim/utils/spatial_contact_search.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContactKinematicsPlaneSpatialContour::~ContactKinematicsPlaneSpatialContour() {
    delete func;
  }

  void ContactKinematicsPlaneSpatialContour::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Plane*>(contour[0])) {
      iplane = 0;
      ispatialcontour = 1;
      plane = static_cast<Plane*>(contour[0]);
      spatialcontour = static_cast<Contour*>(contour[1]);
    }
    else {
      iplane = 1;
      ispatialcontour = 0;
      plane = static_cast<Plane*>(contour[1]);
      spatialcontour = static_cast<Contour*>(contour[0]);
    }
    func = new FuncPairSpatialContourPlane(plane,spatialcontour);
    zeta0.resize(maxNumContacts);
  }

  void ContactKinematicsPlaneSpatialContour::setInitialGuess(const fmatvec::MatV &zeta0_) {
    if(zeta0_.rows()) {
      if(zeta0_.rows() != maxNumContacts or zeta0_.cols() != 2) throw runtime_error("(ContactKinematicsPlaneSpatialContour::assignContours): size of zeta0 does not match");
      for(int i=0; i<maxNumContacts; i++)
        zeta0[i] = zeta0_.row(i).T();
    }
  }

  void ContactKinematicsPlaneSpatialContour::updateg(SingleContact &contact, int i) {
    SpatialContactSearch search(func);
    search.setTolerance(tol);

    if ((!spatialcontour->getEtaNodes().empty()) && (!spatialcontour->getXiNodes().empty()))
      search.setNodes(spatialcontour->getEtaNodes(), spatialcontour->getXiNodes());
    else
      search.setEqualSpacing(10, 10, 0, 0, 0.1, 0.1);

    if (!searchAllCP) {
      search.setInitialValue(zeta0[i]);
    }
    else {
      search.setSearchAll(true);
      searchAllCP = false;
    }

    zeta0[i] = search.slv();
    contact.getContourFrame(ispatialcontour)->setZeta(zeta0[i]);

    contact.getContourFrame(iplane)->setOrientation(plane->getFrame()->evalOrientation());
    contact.getContourFrame(ispatialcontour)->getOrientation(false).set(0, -plane->getFrame()->getOrientation().col(0));
    contact.getContourFrame(ispatialcontour)->getOrientation(false).set(1, -plane->getFrame()->getOrientation().col(1));
    contact.getContourFrame(ispatialcontour)->getOrientation(false).set(2, plane->getFrame()->getOrientation().col(2));

    contact.getContourFrame(ispatialcontour)->setPosition(spatialcontour->evalPosition(contact.getContourFrame(ispatialcontour)->getZeta(false)));

    Vec3 Wn = contact.getContourFrame(iplane)->getOrientation(false).col(0);

    double g;
    if(spatialcontour->isZetaOutside(contact.getContourFrame(ispatialcontour)->getZeta(false)))
      g = 1;
    else
      g = Wn.T()*(contact.getContourFrame(ispatialcontour)->getPosition(false) - plane->getFrame()->getPosition());
    if(g < -spatialcontour->getThickness()) g = 1;

    contact.getContourFrame(iplane)->setPosition(contact.getContourFrame(ispatialcontour)->getPosition(false) - Wn*g);
    contact.getGeneralizedRelativePosition(false)(0) = g;
  }

  void ContactKinematicsPlaneSpatialContour::updatewb(SingleContact &contact, int i) {
    const Vec3 n1 = contact.getContourFrame(iplane)->evalOrientation().col(0);
    const Vec3 u1 = contact.getContourFrame(iplane)->getOrientation().col(1);
    const Vec3 v1 = contact.getContourFrame(iplane)->getOrientation().col(2);
    const Mat3x2 R1 = plane->evalWR(Vec2(NONINIT));

    const Vec3 u2 = spatialcontour->evalWu(contact.getContourFrame(ispatialcontour)->getZeta());
    const Vec3 v2 = spatialcontour->evalWv(contact.getContourFrame(ispatialcontour)->getZeta());
    const Mat3x2 R2 = spatialcontour->evalWR(contact.getContourFrame(ispatialcontour)->getZeta());
    const Mat3x2 U2 = spatialcontour->evalWU(contact.getContourFrame(ispatialcontour)->getZeta());
    const Mat3x2 V2 = spatialcontour->evalWV(contact.getContourFrame(ispatialcontour)->getZeta());

    const Vec3 vC1 = contact.getContourFrame(iplane)->evalVelocity();
    const Vec3 vC2 = contact.getContourFrame(ispatialcontour)->evalVelocity();
    const Vec3 Om1 = contact.getContourFrame(iplane)->evalAngularVelocity();
    const Vec3 Om2 = contact.getContourFrame(ispatialcontour)->evalAngularVelocity();

    SqrMat A(4,NONINIT);
    A(RangeV(0,0),RangeV(0,1)) = -u1.T()*R1;
    A(RangeV(0,0),RangeV(2,3)) = u1.T()*R2;
    A(RangeV(1,1),RangeV(0,1)) = -v1.T()*R1;
    A(RangeV(1,1),RangeV(2,3)) = v1.T()*R2;
    A(RangeV(2,2),RangeV(0,1)).init(0); // = u2.T()*N1;
    A(RangeV(2,2),RangeV(2,3)) = n1.T()*U2;
    A(RangeV(3,3),RangeV(0,1)).init(0); // = v2.T()*N1;
    A(RangeV(3,3),RangeV(2,3)) = n1.T()*V2;

    Vec b(4,NONINIT);
    b(0) = -u1.T()*(vC2-vC1);
    b(1) = -v1.T()*(vC2-vC1);
    b(2) = -v2.T()*(Om2-Om1);
    b(3) = u2.T()*(Om2-Om1);
    Vec zetad =  slvLU(A,b);
    Vec zetad1 = zetad(0,1);
    Vec zetad2 = zetad(2,3);

    const Mat3x3 tOm1 = tilde(Om1);
    const Mat3x3 tOm2 = tilde(Om2);

    contact.getwb(false)(0) += (/**(vC2-vC1).T()*N1**/-n1.T()*tOm1*R1)*zetad1+n1.T()*(tOm2*R2*zetad2-tOm1*(vC2-vC1));
    if (contact.getwb(false).size()>1) {
      contact.getwb(false)(1) += (/**(vC2-vC1).T()*U1**/-u1.T()*tOm1*R1)*zetad1+u1.T()*(tOm2*R2*zetad2-tOm1*(vC2-vC1));
      if (contact.getwb(false).size()>2)
        contact.getwb(false)(2) += (/**(vC2-vC1).T()*V1**/-v1.T()*tOm1*R1)*zetad1+v1.T()*(tOm2*R2*zetad2-tOm1*(vC2-vC1));
    }
  }

}
