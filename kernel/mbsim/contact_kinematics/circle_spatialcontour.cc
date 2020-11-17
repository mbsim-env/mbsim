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
#include "mbsim/contact_kinematics/circle_spatialcontour.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/spatial_contour.h"
#include "mbsim/contours/circle.h"
#include "mbsim/functions/contact/funcpair_spatialcontour_circle.h"
#include "mbsim/utils/spatial_contact_search.h"
#include "mbsim/utils/contact_utils.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContactKinematicsCircleSpatialContour::~ContactKinematicsCircleSpatialContour() {
    delete func;
  }

  void ContactKinematicsCircleSpatialContour::assignContours(const vector<Contour*> &contour) {
    ContactKinematics::assignContours(contour);
    if(dynamic_cast<Circle*>(contour[0])) {
      icircle = 0;
      ispatialcontour = 1;
      circle = static_cast<Circle*>(contour[0]);
      spatialcontour = static_cast<SpatialContour*>(contour[1]);
    } 
    else {
      icircle = 1;
      ispatialcontour = 0;
      circle = static_cast<Circle*>(contour[1]);
      spatialcontour = static_cast<SpatialContour*>(contour[0]);
    }
    func = new FuncPairSpatialContourCircle(circle,spatialcontour);
  }

  void ContactKinematicsCircleSpatialContour::setInitialGuess(const MatV &zeta0_) {
    if(zeta0_.rows()) {
      if(zeta0_.rows() != maxNumContacts or zeta0_.cols() != 2) throw runtime_error("(ContactKinematicsCircleSpatialContour::assignContours): size of zeta0 does not match");
      for(int i=0; i<maxNumContacts; i++) {
	curis(2*i) = zeta0_(i,0);
	curis(2*i+1) = zeta0_(i,1);
      }
    }
  }

  void ContactKinematicsCircleSpatialContour::updateg(SingleContact &contact, int i) {
    SpatialContactSearch search(func);
    search.setTolerance(tol);

    if ((!spatialcontour->getEtaNodes().empty()) && (!spatialcontour->getXiNodes().empty()))
      search.setNodes(Vec(spatialcontour->getEtaNodes()), Vec(spatialcontour->getXiNodes()));
    else
      search.setEqualSpacing(10, 10, 0, 0, 0.1, 0.1);

    search.setInitialValue(curis(RangeV(2*i,2*i+1)));

    nextis.set(RangeV(2*i,2*i+1),search.slv());
    contact.getContourFrame(ispatialcontour)->setZeta(nextis(RangeV(2*i,2*i+1)));

    contact.getContourFrame(ispatialcontour)->getOrientation(false).set(0, spatialcontour->evalWn(contact.getContourFrame(ispatialcontour)->getZeta(false)));
    contact.getContourFrame(ispatialcontour)->getOrientation(false).set(1, spatialcontour->evalWu(contact.getContourFrame(ispatialcontour)->getZeta(false)));
    contact.getContourFrame(ispatialcontour)->getOrientation(false).set(2, crossProduct(contact.getContourFrame(ispatialcontour)->getOrientation(false).col(0),contact.getContourFrame(ispatialcontour)->getOrientation(false).col(1)));

    contact.getContourFrame(ispatialcontour)->setPosition(spatialcontour->evalPosition(contact.getContourFrame(ispatialcontour)->getZeta(false)));

    Vec3 Wb = circle->getFrame()->evalOrientation().col(2);
    double t_EC = contact.getContourFrame(ispatialcontour)->getOrientation(false).col(0).T()*Wb;
    if(t_EC>0) {
      Wb *= -1.;
      t_EC *= -1;
    }
    Vec3 z_EC = contact.getContourFrame(ispatialcontour)->getOrientation(false).col(0) - t_EC*Wb;
    double z_EC_nrm2 = nrm2(z_EC);
    Vec3 WrD;
    if(z_EC_nrm2 <= 1e-8)
      WrD = circle->getFrame()->getPosition() - contact.getContourFrame(ispatialcontour)->getPosition();
    else
      WrD = (circle->getFrame()->getPosition() - (circle->getRadius()/z_EC_nrm2)*z_EC) - contact.getContourFrame(ispatialcontour)->getPosition();

    double g;
    if(spatialcontour->isZetaOutside(contact.getContourFrame(ispatialcontour)->getZeta(false)))
      g = 1;
    else
      g = contact.getContourFrame(ispatialcontour)->getOrientation(false).col(0).T() * WrD;
//      g = contact.getContourFrame(ispatialcontour)->getOrientation(false).col(0).T() * func->evalWrD(contact.getContourFrame(ispatialcontour)->getZeta(false));
    if(g < -spatialcontour->getThickness()) g = 1;
    contact.getGeneralizedRelativePosition(false)(0) = g;

    contact.getContourFrame(icircle)->setPosition(contact.getContourFrame(ispatialcontour)->getPosition(false) + contact.getContourFrame(ispatialcontour)->getOrientation(false).col(0)*g);
    contact.getContourFrame(icircle)->setEta(computeAngleOnUnitCircle(circle->getFrame()->evalOrientation().T()*(z_EC/(-z_EC_nrm2))));
    contact.getContourFrame(icircle)->getOrientation(false).set(0, -contact.getContourFrame(ispatialcontour)->getOrientation(false).col(0));
    contact.getContourFrame(icircle)->setXi(asin((circle->getFrame()->evalOrientation().T()*contact.getContourFrame(icircle)->getOrientation(false).col(0))(2)));
    contact.getContourFrame(icircle)->getOrientation(false).set(1, circle->evalWu(contact.getContourFrame(icircle)->getZeta(false)));
    contact.getContourFrame(icircle)->getOrientation(false).set(2, crossProduct(contact.getContourFrame(icircle)->getOrientation(false).col(0),contact.getContourFrame(icircle)->getOrientation(false).col(1)));
  }
}
