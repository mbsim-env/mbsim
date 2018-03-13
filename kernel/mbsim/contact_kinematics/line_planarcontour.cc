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
  }

  void ContactKinematicsLinePlanarContour::setInitialGuess(const fmatvec::VecV &zeta0_) {
    if(zeta0_.size()) {
      if(zeta0_.size() != 1) throw runtime_error("(ContactKinematicsLinePlanarContour::assignContours): size of zeta0 does not match");
      zeta0 = zeta0_(0);
    }
  }

  void ContactKinematicsLinePlanarContour::updateg(double &g, vector<ContourFrame*> &cFrame, int index) {

    PlanarContactSearch search(func);
    search.setTolerance(tol);
    search.setNodes(planarcontour->getEtaNodes());

    if (!searchAllCP) {
      search.setInitialValue(zeta0);
    }
    else {
      search.setSearchAll(true);
      searchAllCP = false;
    }

    zeta0 = search.slv();
    cFrame[iplanarcontour]->setEta(zeta0);

    cFrame[iline]->setOrientation(line->getFrame()->evalOrientation());
    cFrame[iplanarcontour]->getOrientation(false).set(0, -line->getFrame()->getOrientation().col(0));
    cFrame[iplanarcontour]->getOrientation(false).set(1, -line->getFrame()->getOrientation().col(1));
    cFrame[iplanarcontour]->getOrientation(false).set(2, line->getFrame()->getOrientation().col(2));

    cFrame[iplanarcontour]->setPosition(planarcontour->evalPosition(cFrame[iplanarcontour]->getZeta()));

    Vec3 Wn = cFrame[iline]->getOrientation(false).col(0);

    if(planarcontour->isZetaOutside(cFrame[iplanarcontour]->getZeta()))
      g = 1;
    else
      g = Wn.T()*(cFrame[iplanarcontour]->getPosition(false) - line->getFrame()->getPosition());
    if(g < -planarcontour->getThickness()) g = 1;

    cFrame[iline]->setPosition(cFrame[iplanarcontour]->getPosition(false) - Wn*g);
  }

  void ContactKinematicsLinePlanarContour::updatewb(Vec &wb, double g, vector<ContourFrame*> &cFrame) {
    const Vec3 n1 = cFrame[iline]->evalOrientation().col(0);
    const Vec3 u1 = cFrame[iline]->getOrientation().col(1);
    const Vec3 R1 = u1;

    const Vec3 v2 = planarcontour->evalWv(cFrame[iplanarcontour]->getZeta());
    const Vec3 R2 = planarcontour->evalWs(cFrame[iplanarcontour]->getZeta());
    const Vec3 U2 = planarcontour->evalParDer1Wu(cFrame[iplanarcontour]->getZeta());

    const Vec3 vC1 = cFrame[iline]->evalVelocity();
    const Vec3 vC2 = cFrame[iplanarcontour]->evalVelocity();
    const Vec3 Om1 = cFrame[iline]->evalAngularVelocity();
    const Vec3 Om2 = cFrame[iplanarcontour]->evalAngularVelocity();

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

    wb(0) += (/**(vC2-vC1).T()*N1**/-n1.T()*tOm1*R1)*zetad(0)+n1.T()*(tOm2*R2*zetad(1)-tOm1*(vC2-vC1));
    if (wb.size()>1)
      wb(1) += (/**(vC2-vC1).T()*U1**/-u1.T()*tOm1*R1)*zetad(0)+u1.T()*(tOm2*R2*zetad(1)-tOm1*(vC2-vC1));
  }

}

