/* Copyright (C) 2004-2015 MBSim Development Team
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
#include "mbsim/contact_kinematics/point_extrusion.h"
#include "mbsim/contours/point.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/functions/contact/funcpair_planarcontour_point.h"
#include "mbsim/utils/nonlinear_algebra.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSim {

  ContactKinematicsPointExtrusion::~ContactKinematicsPointExtrusion() {
    delete func;
  }

  void ContactKinematicsPointExtrusion::assignContours(const vector<Contour*>& contour) {
    if (dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      iextrusion = 1;
      point = static_cast<Point*>(contour[0]);
      extrusion = static_cast<Contour*>(contour[1]);
    }
    else {
      ipoint = 1;
      iextrusion = 0;
      point = static_cast<Point*>(contour[1]);
      extrusion = static_cast<Contour*>(contour[0]);
    }
    func = new FuncPairPlanarContourPoint(point, extrusion); // root function for searching contact parameters
  }

  void ContactKinematicsPointExtrusion::setInitialGuess(const MatV &zeta0_) {
    if(zeta0_.rows()) {
      if(zeta0_.rows() != 1 or zeta0_.cols() != 1) throw runtime_error("(ContactKinematicsPointExtrusion::assignContours): size of zeta0 does not match");
      curis(0) = zeta0_(0,0);
    }
  }

  void ContactKinematicsPointExtrusion::determineInitialGuess() {
    NewtonMethod search(func, nullptr);
    search.setTolerance(tol);
    double nrd = 1e13;
    double eta = 0;
    vector<double> zeta0 = searchPossibleContactPoints(func,eta,extrusion->getEtaNodes(),tol);
    for(size_t i=0; i<zeta0.size(); i++) {
      eta = zeta0[i];
      Vec2 zeta_;
      zeta_(0) = search.solve(eta);
      double nrd_ = nrm2(point->getFrame()->evalPosition()-extrusion->evalPosition(zeta_));
      if(nrd_<nrd) {
	curis(0) = zeta_(0);
	nrd = nrd_;
      }
    }
  }

  void ContactKinematicsPointExtrusion::updateg(SingleContact &contact, int i) {
    NewtonMethod search(func, nullptr);
    search.setTolerance(tol);
    nextis(0) = search.solve(curis(0));
    if(search.getInfo()!=0)
      throw std::runtime_error("(ContactKinematicsPointExtrusion:updateg): contact search failed!");

    contact.getContourFrame(iextrusion)->setEta(nextis(0));

    contact.getContourFrame(iextrusion)->setPosition(extrusion->evalPosition(contact.getContourFrame(iextrusion)->getZeta(false)));
    contact.getContourFrame(iextrusion)->getOrientation(false).set(0, extrusion->evalWn(contact.getContourFrame(iextrusion)->getZeta(false)));
    contact.getContourFrame(iextrusion)->getOrientation(false).set(1, extrusion->evalWu(contact.getContourFrame(iextrusion)->getZeta(false)));
    contact.getContourFrame(iextrusion)->getOrientation(false).set(2, extrusion->evalWv(contact.getContourFrame(iextrusion)->getZeta(false)));

    contact.getContourFrame(ipoint)->setPosition(point->getFrame()->evalPosition()); // position of point
    contact.getContourFrame(ipoint)->getOrientation(false).set(0, -contact.getContourFrame(iextrusion)->getOrientation(false).col(0));
    contact.getContourFrame(ipoint)->getOrientation(false).set(1, -contact.getContourFrame(iextrusion)->getOrientation(false).col(1));
    contact.getContourFrame(ipoint)->getOrientation(false).set(2, contact.getContourFrame(iextrusion)->getOrientation(false).col(2));

    Vec3 Wd = contact.getContourFrame(ipoint)->getPosition(false) - contact.getContourFrame(iextrusion)->getPosition(false);
    contact.getContourFrame(iextrusion)->setXi(contact.getContourFrame(iextrusion)->getOrientation(false).col(2).T() * Wd); // get contact parameter of second tangential direction
    contact.getContourFrame(iextrusion)->getPosition(false) += contact.getContourFrame(iextrusion)->getXi(false) * contact.getContourFrame(iextrusion)->getOrientation(false).col(2);

    double g;
    if(extrusion->isZetaOutside(contact.getContourFrame(iextrusion)->getZeta(false)))
      g = 1;
    else
      g = contact.getContourFrame(iextrusion)->getOrientation(false).col(0).T() * Wd;
    if(g < -extrusion->getThickness()) g = 1;
    contact.getGeneralizedRelativePosition(false)(0) = g;
  }

}
