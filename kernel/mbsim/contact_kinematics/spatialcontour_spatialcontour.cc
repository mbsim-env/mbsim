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
#include "mbsim/contact_kinematics/spatialcontour_spatialcontour.h"
#include "mbsim/contours/contour.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/functions/contact/funcpair_spatialcontour_spatialcontour.h"
#include "mbsim/utils/nonlinear_algebra.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContactKinematicsSpatialContourSpatialContour::~ContactKinematicsSpatialContourSpatialContour() {
    delete func;
  }

  void ContactKinematicsSpatialContourSpatialContour::assignContours(const vector<Contour*> &contour) {
    ContactKinematics::assignContours(contour);
    func = new FuncPairSpatialContourSpatialContour(contour[0],contour[1]);
  }

  void ContactKinematicsSpatialContourSpatialContour::setInitialGuess(const MatV &zeta0_) {
    if(zeta0_.rows()) {
      if(zeta0_.rows() != maxNumContacts or zeta0_.cols() != 4) throw runtime_error("(ContactKinematics::setInitialGuess): size of zeta0 does not match");
      for(int i=0; i<maxNumContacts; i++) {
	curis(4*i) = zeta0_(i,0);
	curis(4*i+1) = zeta0_(i,1);
	curis(4*i+2) = zeta0_(i,2);
	curis(4*i+3) = zeta0_(i,3);
      }
    }
  }

  void ContactKinematicsSpatialContourSpatialContour::updateg(SingleContact &contact, int i) {
    MultiDimNewtonMethod search(func, nullptr);
    search.setTolerance(tol);
    nextis.set(RangeV(4*i,4*i+3),search.solve(curis(RangeV(4*i,4*i+3))));
    if(search.getInfo()!=0)
      throw std::runtime_error("(ContactKinematics:updateg): contact search failed!");

    contact.getContourFrame(0)->setZeta(nextis(RangeV(4*i,4*i+1)));
    contact.getContourFrame(1)->setZeta(nextis(RangeV(4*i+2,4*i+3)));

    for(int i=0; i<2; i++) {
      contact.getContourFrame(i)->getOrientation(false).set(0, contour[i]->evalWn(contact.getContourFrame(i)->getZeta(false)));
      contact.getContourFrame(i)->getOrientation(false).set(1, contour[i]->evalWu(contact.getContourFrame(i)->getZeta(false)));
      contact.getContourFrame(i)->getOrientation(false).set(2, crossProduct(contact.getContourFrame(i)->getOrientation(false).col(0),contact.getContourFrame(i)->getOrientation(false).col(1)));
      contact.getContourFrame(i)->setPosition(contour[i]->evalPosition(contact.getContourFrame(i)->getZeta(false)));
    }

    Vec3 Wn = contact.getContourFrame(0)->getOrientation(false).col(0);

    double g;
    if(contour[0]->isZetaOutside(contact.getContourFrame(0)->getZeta(false)) or contour[1]->isZetaOutside(contact.getContourFrame(1)->getZeta(false)))
      g = 1;
    else
      g = Wn.T()*(contact.getContourFrame(1)->getPosition(false) - contact.getContourFrame(0)->getPosition(false));
    if(g < -contour[0]->getThickness() or g < -contour[1]->getThickness()) g = 1;

    contact.getGeneralizedRelativePosition(false)(0) = g;
  }

}
