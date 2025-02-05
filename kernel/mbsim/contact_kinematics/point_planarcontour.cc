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
#include "point_planarcontour.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/point.h"
#include "mbsim/functions/contact/funcpair_planarcontour_point.h"
#include "mbsim/utils/nonlinear_algebra.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContactKinematicsPointPlanarContour::~ContactKinematicsPointPlanarContour() {
    delete func;
  }

  void ContactKinematicsPointPlanarContour::assignContours(const vector<Contour*> &contour) {
    ContactKinematics::assignContours(contour);
    if (dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      iplanarcontour = 1;
      point = static_cast<Point*>(contour[0]);
      planarcontour = static_cast<Contour*>(contour[1]);
    }
    else {
      ipoint = 1;
      iplanarcontour = 0;
      point = static_cast<Point*>(contour[1]);
      planarcontour = static_cast<Contour*>(contour[0]);
    }
    func = new FuncPairPlanarContourPoint(point, planarcontour);
  }

  void ContactKinematicsPointPlanarContour::setInitialGuess(const fmatvec::MatV &zeta0_) {
    if(zeta0_.rows()) {
      if(zeta0_.rows() != 1 or zeta0_.cols() != 1) throw runtime_error("(ContactKinematicsPointPlanarContour::assignContours): size of zeta0 does not match");
      curis(0) = zeta0_(0,0);
    }
  }

  void ContactKinematicsPointPlanarContour::search() {
    NewtonMethod search(func, nullptr);
    search.setTolerance(tol);
    if(iGS or gS) {
      double nrd = 1e13;
      double eta = 0;
      vector<double> zeta0 = searchPossibleContactPoints(func,eta,planarcontour->getEtaNodes(),tol);
      for(size_t i=0; i<zeta0.size(); i++) {
	eta = zeta0[i];
	Vec2 zeta_;
	zeta_(0) = search.solve(eta);
	double nrd_ = nrm2(point->getFrame()->evalPosition()-planarcontour->evalPosition(zeta_));
	if(nrd_<nrd) {
	  nextis(0) = zeta_(0);
	  nrd = nrd_;
	}
      }
      if(iGS) {
	curis(0) = nextis(0);
	iGS = false;
      }
    }
    else {
      nextis(0) = search.solve(curis(0));
      if(search.getInfo()!=0)
	throw std::runtime_error("(ContactKinematicsPointPlanarContour:updateg): contact search failed!");
    }
  }

  void ContactKinematicsPointPlanarContour::updateg(SingleContact &contact, int i) {
    contact.getContourFrame(iplanarcontour)->setEta(nextis(0));

    contact.getContourFrame(iplanarcontour)->getOrientation(false).set(0, planarcontour->evalWn(contact.getContourFrame(iplanarcontour)->getZeta(false)));
    contact.getContourFrame(iplanarcontour)->getOrientation(false).set(1, planarcontour->evalWu(contact.getContourFrame(iplanarcontour)->getZeta(false)));
    contact.getContourFrame(iplanarcontour)->getOrientation(false).set(2, crossProduct(contact.getContourFrame(iplanarcontour)->getOrientation(false).col(0),contact.getContourFrame(iplanarcontour)->getOrientation(false).col(1)));
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

  void ContactKinematicsPointPlanarContour::updatewb(SingleContact &contact, int i) {

    const Vec3 n1 = contact.getContourFrame(ipoint)->evalOrientation().col(0);
    const Vec3 u1 = contact.getContourFrame(ipoint)->getOrientation().col(1);
    const Vec3 N1 = u1;
    const Vec3 paruPart1 = crossProduct(contact.getContourFrame(ipoint)->evalAngularVelocity(),u1);
    const Vec3 parnPart1 = crossProduct(contact.getContourFrame(ipoint)->getAngularVelocity(),n1);

    const Vec3 u2 = contact.getContourFrame(iplanarcontour)->evalOrientation().col(1);
    const Vec3 R2 = planarcontour->evalWs(contact.getContourFrame(iplanarcontour)->getZeta());
    const Vec3 U2 = planarcontour->evalParDer1Wu(contact.getContourFrame(iplanarcontour)->getZeta());
    const Vec3 paruPart2 = planarcontour->evalParWuPart(contact.getContourFrame(iplanarcontour)->getZeta());
    const Vec3 parWvCParEta2 = planarcontour->evalParWvCParEta(contact.getContourFrame(iplanarcontour)->getZeta());

    const Vec3 vC1 = contact.getContourFrame(ipoint)->evalVelocity();
    const Vec3 vC2 = contact.getContourFrame(iplanarcontour)->evalVelocity();

    SqrMat A(2,NONINIT);
    A(0,0)=0;
    A(0,1)=u1.T()*R2;
    A(1,0)=u2.T()*N1;
    A(1,1)=n1.T()*U2;
    Vec b(2,NONINIT);
    b(0)=-u1.T()*(vC2-vC1);
    b(1)=-u2.T()*parnPart1-n1.T()*paruPart2;
    const Vec zetad = slvLU(A,b);

    if(contact.isNormalForceLawSetValued())
      contact.getwb(false)(0) += (N1*zetad(0)+parnPart1).T()*(vC2-vC1)+n1.T()*parWvCParEta2*zetad(1);
    if(contact.isTangentialForceLawSetValuedAndActive()) {
      const Vec3 U1=-n1;
      contact.getwb(false)(contact.isNormalForceLawSetValued()) += (U1*zetad(0)+paruPart1).T()*(vC2-vC1)+u1.T()*parWvCParEta2*zetad(1);
    }
  }

}
