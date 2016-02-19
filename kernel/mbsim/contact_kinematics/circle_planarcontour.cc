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
      icontour1s = 1;
      circle = static_cast<Circle*>(contour[0]);
      contour1s = static_cast<Contour*>(contour[1]);
    } 
    else {
      icircle = 1; 
      icontour1s = 0;
      circle = static_cast<Circle*>(contour[1]);
      contour1s = static_cast<Contour*>(contour[0]);
    }
    func = new FuncPairPlanarContourCircle(circle,contour1s);

//    if (dynamic_cast<Contour*>(contour1s)) {
//      double minRadius=1./epsroot();
//      for (double alpha=contour1s->getAlphaStart(); alpha<=contour1s->getAlphaEnd(); alpha+=(contour1s->getAlphaEnd()-contour1s->getAlphaStart())*1e-4) {
//        zeta(0) = alpha;
//        double radius=1./contour1s->getCurvature(zeta);
//        minRadius=(radius<minRadius)?radius:minRadius;
//      }
//      if (circle->getRadius()>minRadius)
//        throw MBSimError("Just one contact point is allowed in Contactpairing Contour-SolidCircle, but either the circle radius is to big or the minimal Radius of Contour is to small.\n minimal radius of Contour="+numtostr(minRadius)+"\n Radius of SolidCircle="+numtostr(circle->getRadius()));
//    }

  }

  void ContactKinematicsCirclePlanarContour::updateg(double t, double &g, std::vector<ContourFrame*> &cFrame, int index) {
    func->setTime(t);
    PlanarContactSearch search(func);
    search.setNodes(contour1s->getEtaNodes());

    if(searchAllCP==false)
      search.setInitialValue(cFrame[icontour1s]->getEta());
    else { 
      search.setSearchAll(true);
      searchAllCP=false;
    }

    cFrame[icontour1s]->setEta(search.slv());

    if (cFrame[icontour1s]->getEta() < contour1s->getEtaNodes()[0] || cFrame[icontour1s]->getEta() > contour1s->getEtaNodes()[contour1s->getEtaNodes().size()-1]) {
      g = 1.0;
      return;
    }
    cFrame[icontour1s]->getOrientation(false).set(0, contour1s->getWn(t,cFrame[icontour1s]->getZeta()));
    cFrame[icontour1s]->getOrientation(false).set(1, contour1s->getWu(t,cFrame[icontour1s]->getZeta()));
    cFrame[icontour1s]->getOrientation(false).set(2, contour1s->getWv(t,cFrame[icontour1s]->getZeta()));
    cFrame[icircle]->getOrientation(false).set(0, -cFrame[icontour1s]->getOrientation(false).col(0));
    cFrame[icircle]->getOrientation(false).set(2, cFrame[icircle]->getOrientation(false).col(2));
    cFrame[icircle]->getOrientation(false).set(1, crossProduct(cFrame[icircle]->getOrientation(false).col(2),cFrame[icircle]->getOrientation(false).col(0)));
    cFrame[icontour1s]->setPosition(contour1s->getPosition(t,cFrame[icontour1s]->getZeta()));
    cFrame[icircle]->setPosition(circle->getFrame()->getPosition(t)+circle->getRadius()*cFrame[icircle]->getOrientation(false).col(0));

    g = cFrame[icontour1s]->getOrientation(false).col(0).T() * (cFrame[icircle]->getPosition(false) - cFrame[icontour1s]->getPosition(false));
    if(g < -contour1s->getThickness()) g = 1;
  }

  void ContactKinematicsCirclePlanarContour::updatewb(double t, Vec &wb, double g, std::vector<ContourFrame*> &cFrame) {
    
    const Vec3 n1 = cFrame[icircle]->getOrientation(t).col(0);
    const Vec3 u1 = cFrame[icircle]->getOrientation().col(1);
    const Vec3 R1 = circle->getRadius()*u1;
    const Vec3 N1 = u1;

    const Vec3 u2 = cFrame[icontour1s]->getOrientation(t).col(1);
    const Vec3 v2 = cFrame[icontour1s]->getOrientation().col(2);
    const Vec3 R2 = contour1s->getWs(t,cFrame[icontour1s]->getZeta());
    const Vec3 U2 = contour1s->getParDer1Wu(t,cFrame[icontour1s]->getZeta());

    const Vec3 vC1 = cFrame[icircle]->getVelocity(t);
    const Vec3 vC2 = cFrame[icontour1s]->getVelocity(t);
    const Vec3 Om1 = cFrame[icircle]->getAngularVelocity(t);
    const Vec3 Om2 = cFrame[icontour1s]->getAngularVelocity(t);

    SqrMat A(2,NONINIT);
    A(0,0)=-u1.T()*R1;
    A(0,1)=u1.T()*R2;
    A(1,0)=u2.T()*N1;
    A(1,1)=n1.T()*U2;
    Vec b(2,NONINIT);
    b(0)=-u1.T()*(vC2-vC1);
    b(1)=-v2.T()*(Om2-Om1);
    const Vec zetad = slvLU(A,b);

    const Mat3x3 tOm1 = tilde(Om1);
    const Mat3x3 tOm2 = tilde(Om2);
    
    wb(0) += ((vC2-vC1).T()*N1-n1.T()*tOm1*R1)*zetad(0)+n1.T()*tOm2*R2*zetad(1)-n1.T()*tOm1*(vC2-vC1);
    if (wb.size()>1) {
      const Vec3 U1=-n1;
      wb(1) += ((vC2-vC1).T()*U1-u1.T()*tOm1*R1)*zetad(0)+u1.T()*tOm2*R2*zetad(1)-u1.T()*tOm1*(vC2-vC1);
    }
  }

}
