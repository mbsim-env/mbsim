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
#include "mbsim/contact_kinematics/circle_planarcontour.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/circle.h"
#include "mbsim/functions/contact/funcpair_planarcontour_circle.h"
#include "mbsim/utils/planar_contact_search.h"
#include "mbsim/utils/eps.h"
#include "mbsim/utils/utils.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContactKinematicsCirclePlanarContour::~ContactKinematicsCirclePlanarContour() {
    delete func;
  }

  void ContactKinematicsCirclePlanarContour::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Circle*>(contour[0])) {
      icircle = 0;
      iplanarcontour = 1;
      circle = static_cast<Circle*>(contour[0]);
      planarcontour = static_cast<Contour*>(contour[1]);
    }
    else {
      icircle = 1; 
      iplanarcontour = 0;
      circle = static_cast<Circle*>(contour[1]);
      planarcontour = static_cast<Contour*>(contour[0]);
    }
    func = new FuncPairPlanarContourCircle(circle,planarcontour);
    zeta0.resize(maxNumContacts);
  }

  void ContactKinematicsCirclePlanarContour::setInitialGuess(const fmatvec::MatV &zeta0_) {
   if(zeta0_.rows()) {
      if(zeta0_.rows() != maxNumContacts or zeta0_.cols() != 1) throw runtime_error("(ContactKinematicsLinePlanarContour::assignContours): size of zeta0 does not match");
      for(int i=0; i<maxNumContacts; i++)
        zeta0[i] = zeta0_(i,0);
    }
  }

  void ContactKinematicsCirclePlanarContour::updateg(SingleContact &contact, int i) {
    PlanarContactSearch search(func);
    search.setTolerance(tol);
    search.setNodes(planarcontour->getEtaNodes());

    if(searchAllCP==false)
      search.setInitialValue(zeta0[i]);
    else {
      search.setSearchAll(true);
      searchAllCP=false;
    }

    zeta0[i] = search.slv();
    contact.getContourFrame(iplanarcontour)->setEta(zeta0[i]);

    contact.getContourFrame(iplanarcontour)->getOrientation(false).set(0, planarcontour->evalWn(contact.getContourFrame(iplanarcontour)->getZeta(false)));
    contact.getContourFrame(iplanarcontour)->getOrientation(false).set(1, planarcontour->evalWu(contact.getContourFrame(iplanarcontour)->getZeta(false)));
    contact.getContourFrame(iplanarcontour)->getOrientation(false).set(2, planarcontour->evalWv(contact.getContourFrame(iplanarcontour)->getZeta(false)));
    contact.getContourFrame(icircle)->getOrientation(false).set(0, -contact.getContourFrame(iplanarcontour)->getOrientation(false).col(0));
    contact.getContourFrame(icircle)->getOrientation(false).set(2, circle->getFrame()->evalOrientation().col(2));
    contact.getContourFrame(icircle)->getOrientation(false).set(1, crossProduct(contact.getContourFrame(icircle)->getOrientation(false).col(2),contact.getContourFrame(icircle)->getOrientation(false).col(0)));
    contact.getContourFrame(iplanarcontour)->setPosition(planarcontour->evalPosition(contact.getContourFrame(iplanarcontour)->getZeta(false)));
    contact.getContourFrame(icircle)->setPosition(circle->getFrame()->evalPosition()+circle->getRadius()*contact.getContourFrame(icircle)->getOrientation(false).col(0));

    double g;
    if(planarcontour->isZetaOutside(contact.getContourFrame(iplanarcontour)->getZeta(false)))
      g = 1;
    else
      g = contact.getContourFrame(iplanarcontour)->getOrientation(false).col(0).T() * (contact.getContourFrame(icircle)->getPosition(false) - contact.getContourFrame(iplanarcontour)->getPosition(false));
    if(g < -planarcontour->getThickness()) g = 1;
    contact.getGeneralizedRelativePosition(false)(0) = g;
  }

  void ContactKinematicsCirclePlanarContour::updatewb(SingleContact &contact, int i) {
    
    const Vec3 n1 = contact.getContourFrame(icircle)->evalOrientation().col(0);
    const Vec3 u1 = contact.getContourFrame(icircle)->getOrientation().col(1);
    const Vec3 R1 = circle->getRadius()*u1;
    const Vec3 N1 = u1;
    const Vec3 paruPart1 = crossProduct(contact.getContourFrame(icircle)->evalAngularVelocity(),u1);
    const Vec3 parnPart1 = crossProduct(contact.getContourFrame(icircle)->getAngularVelocity(),n1);
    const Vec3 parWvCParEta1 = crossProduct(contact.getContourFrame(icircle)->getAngularVelocity(),R1);

    const Vec3 u2 = contact.getContourFrame(iplanarcontour)->evalOrientation().col(1);
    const Vec3 R2 = planarcontour->evalWs(contact.getContourFrame(iplanarcontour)->getZeta());
    const Vec3 U2 = planarcontour->evalParDer1Wu(contact.getContourFrame(iplanarcontour)->getZeta());
    const Vec3 paruPart2 = planarcontour->evalParWuPart(contact.getContourFrame(iplanarcontour)->getZeta());
    const Vec3 parWvCParEta2 = planarcontour->evalParWvCParEta(contact.getContourFrame(iplanarcontour)->getZeta());

    const Vec3 vC1 = contact.getContourFrame(icircle)->evalVelocity();
    const Vec3 vC2 = contact.getContourFrame(iplanarcontour)->evalVelocity();

    SqrMat A(2,NONINIT);
    A(0,0)=-u1.T()*R1;
    A(0,1)=u1.T()*R2;
    A(1,0)=u2.T()*N1;
    A(1,1)=n1.T()*U2;
    Vec b(2,NONINIT);
    b(0)=-u1.T()*(vC2-vC1);
    b(1)=-n1.T()*(paruPart2-paruPart1);
    const Vec zetad = slvLU(A,b);

    contact.getwb(false)(0) += (N1*zetad(0)+parnPart1).T()*(vC2-vC1)+n1.T()*(parWvCParEta1*zetad(0)+parWvCParEta2*zetad(1));
    if (contact.getwb(false).size()>1) {
      const Vec3 U1=-n1;
      contact.getwb(false)(1) += (U1*zetad(0)+paruPart1).T()*(vC2-vC1)+u1.T()*(parWvCParEta1*zetad(0)+parWvCParEta2*zetad(1));
    }
  }

}
