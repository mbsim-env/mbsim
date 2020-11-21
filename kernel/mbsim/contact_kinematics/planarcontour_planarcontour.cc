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
#include "mbsim/contact_kinematics/planarcontour_planarcontour.h"
#include "mbsim/contours/contour.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/functions/contact/funcpair_planarcontour_planarcontour.h"
#include "mbsim/utils/nonlinear_algebra.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContactKinematicsPlanarContourPlanarContour::~ContactKinematicsPlanarContourPlanarContour() {
    delete func;
  }

  void ContactKinematicsPlanarContourPlanarContour::assignContours(const vector<Contour*> &contour) {
    ContactKinematics::assignContours(contour);
    func = new FuncPairPlanarContourPlanarContour(contour[0],contour[1]);
  }

  void ContactKinematicsPlanarContourPlanarContour::setInitialGuess(const MatV &zeta0_) {
    if(zeta0_.rows()) {
      if(zeta0_.rows() != maxNumContacts or zeta0_.cols() != 2) throw runtime_error("(ContactKinematics::setInitialGuess): size of zeta0 does not match");
      for(int i=0; i<maxNumContacts; i++) {
	curis(2*i) = zeta0_(i,0);
	curis(2*i+1) = zeta0_(i,1);
      }
    }
  }

  void ContactKinematicsPlanarContourPlanarContour::updateg(SingleContact &contact, int i) {
    MultiDimNewtonMethod search(func, nullptr);
    search.setTolerance(tol);
    nextis.set(RangeV(2*i,2*i+1),search.solve(curis(RangeV(2*i,2*i+1))));
    if(search.getInfo()!=0)
      throw std::runtime_error("(ContactKinematics:updateg): contact search failed!");

    contact.getContourFrame(0)->setEta(nextis(2*i));
    contact.getContourFrame(1)->setEta(nextis(2*i+1));

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

  void ContactKinematicsPlanarContourPlanarContour::updatewb(SingleContact &contact, int i) {

    const Vec3 u2 = contact.getContourFrame(1)->evalOrientation().col(1);
    const Vec3 R2 = contour[1]->evalWs(contact.getContourFrame(1)->getZeta());
    const Vec3 U2 = contour[1]->evalParDer1Wu(contact.getContourFrame(1)->getZeta());
    const Vec3 paruPart2 = contour[1]->evalParWuPart(contact.getContourFrame(1)->getZeta());
    const Vec3 parWvCParZeta2 = contour[1]->evalParWvCParEta(contact.getContourFrame(1)->getZeta());

    const Vec3 u1 = contact.getContourFrame(0)->evalOrientation().col(1);
    const Vec3 n1 = contact.getContourFrame(0)->evalOrientation().col(0);
    const Vec3 R1 = contour[0]->evalWs(contact.getContourFrame(0)->getZeta());
    const Vec3 U1 = contour[0]->evalParDer1Wu(contact.getContourFrame(0)->getZeta());
    const Vec3 N1 = contour[0]->evalParDer1Wn(contact.getContourFrame(0)->getZeta());
    const Vec3 paruPart1 = contour[0]->evalParWuPart(contact.getContourFrame(0)->getZeta());
    const Vec3 parnPart1 = contour[0]->evalParWnPart(contact.getContourFrame(0)->getZeta());
    const Vec3 parWvCParZeta1 = contour[0]->evalParWvCParEta(contact.getContourFrame(0)->getZeta());

    const Vec3 vC2 = contact.getContourFrame(1)->evalVelocity();
    const Vec3 vC1 = contact.getContourFrame(0)->evalVelocity();

    SqrMat A(2,NONINIT);
    A(0,0) = -u1.T()*R1;
    A(0,1) = u1.T()*R2;
    A(1,0) = u2.T()*N1;
    A(1,1) = n1.T()*U2;

    Vec b(2,NONINIT);
    b(0) = -u1.T()*(vC2-vC1);
    b(1) = -u2.T()*parnPart1-n1.T()*paruPart2;
    Vec zetad =  slvLU(A,b);
    double zetad1 = zetad(0);
    double zetad2 = zetad(1);

    if(contact.isNormalForceLawSetValued())
      contact.getwb(false)(0) += (N1*zetad1+parnPart1).T()*(vC2-vC1)+n1.T()*(parWvCParZeta2*zetad2-parWvCParZeta1*zetad1);
    if(contact.isTangentialForceLawSetValuedAndActive()) {
      contact.getwb(false)(contact.isNormalForceLawSetValued()) += (U1*zetad1+paruPart1).T()*(vC2-vC1)+u1.T()*(parWvCParZeta2*zetad2-parWvCParZeta1*zetad1);
    }
  }
}
