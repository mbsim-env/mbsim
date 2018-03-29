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
#include "line_planarcontour.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/line.h"
#include "mbsim/functions/contact/funcpair_planarcontour_line.h"
#include "mbsim/utils/planar_contact_search.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContactKinematicsLinePlanarContour::~ContactKinematicsLinePlanarContour() {
    delete func;
  }

  void ContactKinematicsLinePlanarContour::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Line*>(contour[0])) {
      iline = 0;
      iplanarcontour = 1;
      line = static_cast<Line*>(contour[0]);
      planarcontour = static_cast<Contour*>(contour[1]);
    } 
    else {
      iline = 1;
      iplanarcontour = 0;
      line = static_cast<Line*>(contour[1]);
      planarcontour = static_cast<Contour*>(contour[0]);
    }
    func = new FuncPairPlanarContourLine(line,planarcontour);
    zeta0.resize(maxNumContacts);
  }

  void ContactKinematicsLinePlanarContour::setInitialGuess(const fmatvec::MatV &zeta0_) {
    if(zeta0_.rows()) {
      if(zeta0_.rows() != maxNumContacts or zeta0_.cols() != 1) throw runtime_error("(ContactKinematicsLinePlanarContour::assignContours): size of zeta0 does not match");
      for(int i=0; i<maxNumContacts; i++)
        zeta0[i] = zeta0_(i,0);
    }
  }

  void ContactKinematicsLinePlanarContour::updateg(SingleContact &contact, int i) {

    PlanarContactSearch search(func);
    search.setTolerance(tol);
    search.setNodes(planarcontour->getEtaNodes());

    if (!searchAllCP) {
      search.setInitialValue(zeta0[i]);
    }
    else {
      search.setSearchAll(true);
      searchAllCP = false;
    }

    zeta0[i] = search.slv();
    contact.getContourFrame(iplanarcontour)->setEta(zeta0[i]);

    contact.getContourFrame(iline)->setOrientation(line->getFrame()->evalOrientation());
    contact.getContourFrame(iplanarcontour)->getOrientation(false).set(0, -line->getFrame()->getOrientation().col(0));
    contact.getContourFrame(iplanarcontour)->getOrientation(false).set(1, -line->getFrame()->getOrientation().col(1));
    contact.getContourFrame(iplanarcontour)->getOrientation(false).set(2, line->getFrame()->getOrientation().col(2));

    contact.getContourFrame(iplanarcontour)->setPosition(planarcontour->evalPosition(contact.getContourFrame(iplanarcontour)->getZeta()));

    Vec3 Wn = contact.getContourFrame(iline)->getOrientation(false).col(0);

    double g;
    if(planarcontour->isZetaOutside(contact.getContourFrame(iplanarcontour)->getZeta()))
      g = 1;
    else
      g = Wn.T()*(contact.getContourFrame(iplanarcontour)->getPosition(false) - line->getFrame()->getPosition());
    if(g < -planarcontour->getThickness()) g = 1;

    contact.getContourFrame(iline)->setPosition(contact.getContourFrame(iplanarcontour)->getPosition(false) - Wn*g);
    contact.getGeneralizedRelativePosition(false)(0) = g;
  }

  void ContactKinematicsLinePlanarContour::updatewb(SingleContact &contact, int i) {
    const Vec3 n1 = contact.getContourFrame(iline)->evalOrientation().col(0);
    const Vec3 u1 = contact.getContourFrame(iline)->getOrientation().col(1);
    const Vec3 R1 = u1;

    const Vec3 v2 = planarcontour->evalWv(contact.getContourFrame(iplanarcontour)->getZeta());
    const Vec3 R2 = planarcontour->evalWs(contact.getContourFrame(iplanarcontour)->getZeta());
    const Vec3 U2 = planarcontour->evalParDer1Wu(contact.getContourFrame(iplanarcontour)->getZeta());

    const Vec3 vC1 = contact.getContourFrame(iline)->evalVelocity();
    const Vec3 vC2 = contact.getContourFrame(iplanarcontour)->evalVelocity();
    const Vec3 Om1 = contact.getContourFrame(iline)->evalAngularVelocity();
    const Vec3 Om2 = contact.getContourFrame(iplanarcontour)->evalAngularVelocity();

    SqrMat A(2,NONINIT);
    A(0,0) = -u1.T()*R1;
    A(0,1) = u1.T()*R2;
    A(1,0) = 0; // = u2.T()*N1;
    A(1,1) = n1.T()*U2;

    Vec b(2,NONINIT);
    b(0) = -u1.T()*(vC2-vC1);
    b(1) = -v2.T()*(Om2-Om1);
    Vec zetad =  slvLU(A,b);

    const Mat3x3 tOm1 = tilde(Om1);
    const Mat3x3 tOm2 = tilde(Om2);

    contact.getwb(false)(0) += (/**(vC2-vC1).T()*N1**/-n1.T()*tOm1*R1)*zetad(0)+n1.T()*(tOm2*R2*zetad(1)-tOm1*(vC2-vC1));
    if (contact.getwb(false).size()>1)
      contact.getwb(false)(1) += (/**(vC2-vC1).T()*U1**/-u1.T()*tOm1*R1)*zetad(0)+u1.T()*(tOm2*R2*zetad(1)-tOm1*(vC2-vC1));
  }

}
