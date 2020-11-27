/* Copyright (C) 2004-2020 MBSim Development Team
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
#include "mbsim/contact_kinematics/sphere_spatialcontour.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/spatial_contour.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/functions/contact/funcpair_spatialcontour_sphere.h"
#include "mbsim/utils/nonlinear_algebra.h"
#include "mbsim/utils/contact_utils.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContactKinematicsSphereSpatialContour::~ContactKinematicsSphereSpatialContour() {
    delete func;
  }

  void ContactKinematicsSphereSpatialContour::assignContours(const vector<Contour*> &contour) {
    ContactKinematics::assignContours(contour);
    if(dynamic_cast<Sphere*>(contour[0])) {
      isphere = 0;
      ispatialcontour = 1;
      sphere = static_cast<Sphere*>(contour[0]);
      spatialcontour = static_cast<SpatialContour*>(contour[1]);
    } 
    else {
      isphere = 1;
      ispatialcontour = 0;
      sphere = static_cast<Sphere*>(contour[1]);
      spatialcontour = static_cast<SpatialContour*>(contour[0]);
    }
    func = new FuncPairSpatialContourSphere(sphere,spatialcontour);
  }

  void ContactKinematicsSphereSpatialContour::setInitialGuess(const MatV &zeta0_) {
    if(zeta0_.rows()) {
      if(zeta0_.rows() != maxNumContacts or zeta0_.cols() != 2) throw runtime_error("(ContactKinematicsSphereSpatialContour::setInitialGuess): size of zeta0 does not match");
      for(int i=0; i<maxNumContacts; i++) {
	curis(2*i) = zeta0_(i,0);
	curis(2*i+1) = zeta0_(i,1);
      }
    }
  }

  void ContactKinematicsSphereSpatialContour::search() {
    MultiDimNewtonMethod search(func, nullptr);
    search.setTolerance(tol);
    for(int i=0; i<maxNumContacts; i++) {
      nextis.set(RangeV(2*i,2*i+1),search.solve(curis(RangeV(2*i,2*i+1))));
      if(search.getInfo()!=0)
	throw std::runtime_error("(ContactKinematicsSphereSpatialContour:updateg): contact search failed!");
    }
  }

  void ContactKinematicsSphereSpatialContour::updateg(SingleContact &contact, int i) {
    contact.getContourFrame(ispatialcontour)->setZeta(nextis(RangeV(2*i,2*i+1)));

    contact.getContourFrame(ispatialcontour)->getOrientation(false).set(0, spatialcontour->evalWn(contact.getContourFrame(ispatialcontour)->getZeta(false)));
    contact.getContourFrame(ispatialcontour)->getOrientation(false).set(1, spatialcontour->evalWu(contact.getContourFrame(ispatialcontour)->getZeta(false)));
    contact.getContourFrame(ispatialcontour)->getOrientation(false).set(2, crossProduct(contact.getContourFrame(ispatialcontour)->getOrientation(false).col(0),contact.getContourFrame(ispatialcontour)->getOrientation(false).col(1)));


    contact.getContourFrame(isphere)->getOrientation(false).set(0, -contact.getContourFrame(ispatialcontour)->getOrientation(false).col(0));
    contact.getContourFrame(isphere)->setZeta(computeAnglesOnUnitSphere(sphere->getFrame()->getOrientation().T()*contact.getContourFrame(isphere)->getOrientation(false).col(0)));
    contact.getContourFrame(isphere)->getOrientation(false).set(1, sphere->evalWu(contact.getContourFrame(isphere)->getZeta(false)));
    contact.getContourFrame(isphere)->getOrientation(false).set(2, crossProduct(contact.getContourFrame(isphere)->getOrientation(false).col(0),contact.getContourFrame(isphere)->getOrientation(false).col(1)));

    contact.getContourFrame(ispatialcontour)->setPosition(spatialcontour->evalPosition(contact.getContourFrame(ispatialcontour)->getZeta(false)));
    contact.getContourFrame(isphere)->setPosition(sphere->getFrame()->evalPosition()+sphere->getRadius()*contact.getContourFrame(isphere)->getOrientation(false).col(0));

    double g;
    if(spatialcontour->isZetaOutside(contact.getContourFrame(ispatialcontour)->getZeta(false)))
      g = 1;
    else
      g = contact.getContourFrame(ispatialcontour)->getOrientation(false).col(0).T() * (contact.getContourFrame(isphere)->getPosition(false) - contact.getContourFrame(ispatialcontour)->getPosition(false));
    if(g < -spatialcontour->getThickness()) g = 1;
    contact.getGeneralizedRelativePosition(false)(0) = g;
  }
}
