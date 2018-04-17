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
#include "point_flexibleplanarcontour.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/point.h"
#include "mbsimFlexibleBody/contours/flexible_contour.h"
#include "mbsim/functions/contact/funcpair_planarcontour_point.h"
#include "mbsim/utils/planar_contact_search.h"

using namespace fmatvec;
using namespace std;

namespace MBSimFlexibleBody {

  ContactKinematicsPointFlexiblePlanarContour::~ContactKinematicsPointFlexiblePlanarContour() {
    delete func;
  }

  void ContactKinematicsPointFlexiblePlanarContour::assignContours(const vector<MBSim::Contour*> &contour) {
    if (dynamic_cast<MBSim::Point*>(contour[0])) {
      ipoint = 0;
      iplanarcontour = 1;
      point = static_cast<MBSim::Point*>(contour[0]);
      planarcontour = static_cast<FlexibleContour*>(contour[1]);
    }
    else {
      ipoint = 1;
      iplanarcontour = 0;
      point = static_cast<MBSim::Point*>(contour[1]);
      planarcontour = static_cast<FlexibleContour*>(contour[0]);
    }
    func = new MBSim::FuncPairPlanarContourPoint(point, planarcontour);
  }

  void ContactKinematicsPointFlexiblePlanarContour::setInitialGuess(const fmatvec::MatV &zeta0_) {
    if(zeta0_.rows()) {
      if(zeta0_.rows() != 1 or zeta0_.cols() != 1) throw runtime_error("(ContactKinematicsPointFlexiblePlanarContour::assignContours): size of zeta0 does not match");
      zeta0 = zeta0_(0,0);
    }
  }

  void ContactKinematicsPointFlexiblePlanarContour::updateg(MBSim::SingleContact &contact, int i) {
    MBSim::PlanarContactSearch search(func);
    search.setTolerance(tol);
    search.setNodes(planarcontour->getEtaNodes());

    if(!searchAllCP)
      search.setInitialValue(zeta0);
    else {
      search.setSearchAll(true);
      searchAllCP=false;
    }

    zeta0 = search.slv();
    contact.getContourFrame(iplanarcontour)->setEta(zeta0);

    contact.getContourFrame(iplanarcontour)->getOrientation(false).set(0, planarcontour->evalWn(contact.getContourFrame(iplanarcontour)->getZeta(false)));
    contact.getContourFrame(iplanarcontour)->getOrientation(false).set(1, planarcontour->evalWu(contact.getContourFrame(iplanarcontour)->getZeta(false)));
    contact.getContourFrame(iplanarcontour)->getOrientation(false).set(2, planarcontour->evalWv(contact.getContourFrame(iplanarcontour)->getZeta(false)));
    contact.getContourFrame(ipoint)->getOrientation(false).set(0, -contact.getContourFrame(iplanarcontour)->getOrientation(false).col(0));
    contact.getContourFrame(ipoint)->getOrientation(false).set(1, -contact.getContourFrame(iplanarcontour)->getOrientation(false).col(1));
    contact.getContourFrame(ipoint)->getOrientation(false).set(2, contact.getContourFrame(iplanarcontour)->getOrientation(false).col(2));
    contact.getContourFrame(iplanarcontour)->setPosition(planarcontour->evalPosition(contact.getContourFrame(iplanarcontour)->getZeta(false)));
    contact.getContourFrame(ipoint)->setPosition(point->getFrame()->evalPosition());

    double g;
    if(planarcontour->isZetaOutside(contact.getContourFrame(iplanarcontour)->getZeta(false)))
      g = 1;
    else
      g = contact.getContourFrame(iplanarcontour)->getOrientation(false).col(0).T() * (contact.getContourFrame(ipoint)->getPosition(false) - contact.getContourFrame(iplanarcontour)->getPosition(false));
    if(g < -planarcontour->getThickness()) g = 1;
    contact.getGeneralizedRelativePosition(false)(0) = g;
  }

  void ContactKinematicsPointFlexiblePlanarContour::updatewb(MBSim::SingleContact &contact, int i) {
    const Vec3 n1 = contact.getContourFrame(ipoint)->evalOrientation().col(0);
    const Vec3 u1 = contact.getContourFrame(ipoint)->getOrientation().col(1);
    const Vec3 N1 = u1;

    const Vec3 u2 = contact.getContourFrame(iplanarcontour)->evalOrientation().col(1);
    const Vec3 v2 = contact.getContourFrame(iplanarcontour)->getOrientation().col(2);
    const Vec3 R2 = planarcontour->evalWs(contact.getContourFrame(iplanarcontour)->getZeta());
    const Vec3 U2 = planarcontour->evalParDer1Wu(contact.getContourFrame(iplanarcontour)->getZeta());

    const Vec3 vC1 = contact.getContourFrame(ipoint)->evalVelocity();
    const Vec3 vC2 = contact.getContourFrame(iplanarcontour)->evalVelocity();
    const Vec3 Om1 = contact.getContourFrame(ipoint)->evalAngularVelocity();
    const Vec3 Om2 = contact.getContourFrame(iplanarcontour)->evalAngularVelocity();

    Vec3 U2_t = planarcontour->evalWu_t(contact.getContourFrame(iplanarcontour)->getZeta());
    Vec3 R2_t = planarcontour->evalWs_t(contact.getContourFrame(iplanarcontour)->getZeta());

    SqrMat A(2,NONINIT);
    A(0,0)=0;
    A(0,1)=u1.T()*R2;
    A(1,0)=u2.T()*N1;
    A(1,1)=n1.T()*U2;
    Vec b(2,NONINIT);
    b(0)=-u1.T()*(vC2-vC1);
    b(1)=-v2.T()*(Om2-Om1)-n1.T()*U2_t;
    const Vec zetad = slvLU(A,b);

    const Mat3x3 tOm1 = tilde(Om1);
    const Mat3x3 tOm2 = tilde(Om2);

    contact.getwb(false)(0) += (vC2-vC1).T()*N1*zetad(0)+n1.T()*((tOm2*R2+R2_t)*zetad(1)-tOm1*(vC2-vC1));
    if (contact.getwb(false).size()>1) {
      const Vec3 U1=-n1;
      contact.getwb(false)(1) += (vC2-vC1).T()*U1*zetad(0)+u1.T()*((tOm2*R2+R2_t)*zetad(1)-tOm1*(vC2-vC1));
    }
  }

}
