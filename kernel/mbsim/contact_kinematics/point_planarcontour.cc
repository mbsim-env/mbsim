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
#include "mbsim/utils/planar_contact_search.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContactKinematicsPointPlanarContour::~ContactKinematicsPointPlanarContour() {
    delete func;
  }

  void ContactKinematicsPointPlanarContour::assignContours(const vector<Contour*> &contour) {
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

  void ContactKinematicsPointPlanarContour::updateg(double &g, std::vector<ContourFrame*> &cFrame, int index) {
    
    PlanarContactSearch search(func);
    search.setNodes(planarcontour->getEtaNodes());

    if(!searchAllCP)
      search.setInitialValue(cFrame[iplanarcontour]->getEta());
    else {
      search.setSearchAll(true);
      searchAllCP=false;
    }

    cFrame[iplanarcontour]->setEta(search.slv());

    cFrame[iplanarcontour]->getOrientation(false).set(0, planarcontour->evalWn(cFrame[iplanarcontour]->getZeta()));
    cFrame[iplanarcontour]->getOrientation(false).set(1, planarcontour->evalWu(cFrame[iplanarcontour]->getZeta()));
    cFrame[iplanarcontour]->getOrientation(false).set(2, planarcontour->evalWv(cFrame[iplanarcontour]->getZeta()));
    cFrame[ipoint]->getOrientation(false).set(0, -cFrame[iplanarcontour]->getOrientation(false).col(0));
    cFrame[ipoint]->getOrientation(false).set(1, -cFrame[iplanarcontour]->getOrientation(false).col(1));
    cFrame[ipoint]->getOrientation(false).set(2, cFrame[iplanarcontour]->getOrientation(false).col(2));
    cFrame[iplanarcontour]->setPosition(planarcontour->evalPosition(cFrame[iplanarcontour]->getZeta()));
    cFrame[ipoint]->setPosition(point->getFrame()->evalPosition());

    if(planarcontour->isZetaOutside(cFrame[iplanarcontour]->getZeta()))
      g = 1;
    else
      g = cFrame[iplanarcontour]->getOrientation(false).col(0).T() * (cFrame[ipoint]->getPosition(false) - cFrame[iplanarcontour]->getPosition(false));
    if(g < -planarcontour->getThickness()) g = 1;
  }

  void ContactKinematicsPointPlanarContour::updatewb(Vec &wb, double g, vector<ContourFrame*> &cFrame) {

    const Vec3 n1 = cFrame[ipoint]->evalOrientation().col(0);
    const Vec3 u1 = cFrame[ipoint]->getOrientation().col(1);
    const Vec3 N1 = u1;

    const Vec3 u2 = cFrame[iplanarcontour]->evalOrientation().col(1);
    const Vec3 v2 = cFrame[iplanarcontour]->getOrientation().col(2);
    const Vec3 R2 = planarcontour->evalWs(cFrame[iplanarcontour]->getZeta());
    const Vec3 U2 = planarcontour->evalParDer1Wu(cFrame[iplanarcontour]->getZeta());

    const Vec3 vC1 = cFrame[ipoint]->evalVelocity();
    const Vec3 vC2 = cFrame[iplanarcontour]->evalVelocity();
    const Vec3 Om1 = cFrame[ipoint]->evalAngularVelocity();
    const Vec3 Om2 = cFrame[iplanarcontour]->evalAngularVelocity();

    SqrMat A(2,NONINIT);
    A(0,0)=0;
    A(0,1)=u1.T()*R2;
    A(1,0)=u2.T()*N1;
    A(1,1)=n1.T()*U2;
    Vec b(2,NONINIT);
    b(0)=-u1.T()*(vC2-vC1);
    b(1)=-v2.T()*(Om2-Om1);
    const Vec zetad = slvLU(A,b);

    const Mat3x3 tOm1 = tilde(Om1);
    const Mat3x3 tOm2 = tilde(Om2);

    wb(0) += (vC2-vC1).T()*N1*zetad(0)+n1.T()*(tOm2*R2*zetad(1)-tOm1*(vC2-vC1));
    if (wb.size()>1) {
      const Vec3 U1=-n1;
      wb(1) += (vC2-vC1).T()*U1*zetad(0)+u1.T()*(tOm2*R2*zetad(1)-tOm1*(vC2-vC1));
    }
  }

}
