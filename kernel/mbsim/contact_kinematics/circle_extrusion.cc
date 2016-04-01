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
#include "mbsim/contact_kinematics/circle_extrusion.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/circle.h"
#include "mbsim/functions/contact/funcpair_planarcontour_circle.h"
#include "mbsim/utils/planar_contact_search.h"
#include "mbsim/utils/eps.h"
#include "mbsim/utils/utils.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContactKinematicsCircleExtrusion::~ContactKinematicsCircleExtrusion() {
    delete func; 
  }

  void ContactKinematicsCircleExtrusion::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Circle*>(contour[0])) {
      icircle = 0;
      iextrusion = 1;
      circle = static_cast<Circle*>(contour[0]);
      extrusion = static_cast<Contour*>(contour[1]);
    } 
    else {
      icircle = 1; 
      iextrusion = 0;
      circle = static_cast<Circle*>(contour[1]);
      extrusion = static_cast<Contour*>(contour[0]);
    }
    func = new FuncPairPlanarContourCircle(circle,extrusion);

//    if (dynamic_cast<Contour*>(extrusion)) {
//      double minRadius=1./epsroot();
//      for (double alpha=extrusion->getAlphaStart(); alpha<=extrusion->getAlphaEnd(); alpha+=(extrusion->getAlphaEnd()-extrusion->getAlphaStart())*1e-4) {
//        zeta(0) = alpha;
//        double radius=1./extrusion->getCurvature(zeta);
//        minRadius=(radius<minRadius)?radius:minRadius;
//      }
//      if (circle->getRadius()>minRadius)
//        throw MBSimError("Just one contact circle is allowed in Contactpairing Contour-SolidCircle, but either the circle radius is to big or the minimal Radius of Contour is to small.\n minimal radius of Contour="+numtostr(minRadius)+"\n Radius of SolidCircle="+numtostr(circle->getRadius()));
//    }

  }

  void ContactKinematicsCircleExtrusion::updateg(double &g, std::vector<ContourFrame*> &cFrame, int index) {
    PlanarContactSearch search(func);
    search.setNodes(extrusion->getEtaNodes());

    if(searchAllCP==false)
      search.setInitialValue(cFrame[iextrusion]->getEta());
    else { 
      search.setSearchAll(true);
      searchAllCP=false;
    }

    cFrame[iextrusion]->setEta(search.slv());

    cFrame[iextrusion]->getOrientation(false).set(0, extrusion->getWn(cFrame[iextrusion]->getZeta()));
    cFrame[iextrusion]->getOrientation(false).set(1, extrusion->getWu(cFrame[iextrusion]->getZeta()));
    cFrame[iextrusion]->getOrientation(false).set(2, extrusion->getWv(cFrame[iextrusion]->getZeta()));
    cFrame[icircle]->getOrientation(false).set(0, -cFrame[iextrusion]->getOrientation(false).col(0));
    cFrame[icircle]->getOrientation(false).set(2, circle->getFrame()->evalOrientation().col(2));
    cFrame[icircle]->getOrientation(false).set(1, crossProduct(cFrame[icircle]->getOrientation(false).col(2),cFrame[icircle]->getOrientation(false).col(0)));
    cFrame[iextrusion]->setPosition(extrusion->getPosition(cFrame[iextrusion]->getZeta()));
    cFrame[icircle]->setPosition(circle->getFrame()->evalPosition()+circle->getRadius()*cFrame[icircle]->getOrientation(false).col(0));

    Vec3 Wd = cFrame[icircle]->getPosition(false) - cFrame[iextrusion]->getPosition(false);
    cFrame[iextrusion]->setXi(cFrame[iextrusion]->getOrientation(false).col(2).T() * Wd); // get contact parameter of second tangential direction
    cFrame[iextrusion]->getPosition(false) += cFrame[iextrusion]->getXi() * cFrame[iextrusion]->getOrientation(false).col(2);

    if(extrusion->isZetaOutside(cFrame[iextrusion]->getZeta()))
      g = 1;
    else
      g = cFrame[iextrusion]->getOrientation(false).col(0).T() * (cFrame[icircle]->getPosition(false) - cFrame[iextrusion]->getPosition(false));
    if(g < -extrusion->getThickness()) g = 1;
  }

  void ContactKinematicsCircleExtrusion::updatewb(Vec &wb, double g, std::vector<ContourFrame*> &cFrame) {
    
    const Vec3 n1 = cFrame[icircle]->evalOrientation().col(0);
    const Vec3 u1 = cFrame[icircle]->getOrientation().col(1);
    const Vec3 R1 = circle->getRadius()*u1;
    const Vec3 N1 = u1;

    const Vec3 u2 = cFrame[iextrusion]->evalOrientation().col(1);
    const Vec3 v2 = cFrame[iextrusion]->getOrientation().col(2);
    const Vec3 R2 = extrusion->getWs(cFrame[iextrusion]->getZeta());
    const Vec3 U2 = extrusion->getParDer1Wu(cFrame[iextrusion]->getZeta());

    const Vec3 vC1 = cFrame[icircle]->evalVelocity();
    const Vec3 vC2 = cFrame[iextrusion]->evalVelocity();
    const Vec3 Om1 = cFrame[icircle]->evalAngularVelocity();
    const Vec3 Om2 = cFrame[iextrusion]->evalAngularVelocity();

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
