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

//    if (dynamic_cast<Contour*>(planarcontour)) {
//      double minRadius=1./epsroot();
//      for (double alpha=planarcontour->getAlphaStart(); alpha<=planarcontour->getAlphaEnd(); alpha+=(planarcontour->getAlphaEnd()-planarcontour->getAlphaStart())*1e-4) {
//        zeta(0) = alpha;
//        double radius=1./planarcontour->getCurvature(zeta);
//        minRadius=(radius<minRadius)?radius:minRadius;
//      }
//      if (circle->getRadius()>minRadius)
//        throw MBSimError("Just one contact point is allowed in Contactpairing Contour-SolidCircle, but either the circle radius is to big or the minimal Radius of Contour is to small.\n minimal radius of Contour="+numtostr(minRadius)+"\n Radius of SolidCircle="+numtostr(circle->getRadius()));
//    }

  }

  void ContactKinematicsCirclePlanarContour::updateg(double t, double &g, std::vector<ContourFrame*> &cFrame, int index) {
    func->setTime(t);
    PlanarContactSearch search(func);
    search.setNodes(planarcontour->getEtaNodes());

    if(searchAllCP==false)
      search.setInitialValue(cFrame[iplanarcontour]->getEta());
    else { 
      search.setSearchAll(true);
      searchAllCP=false;
    }

    cFrame[iplanarcontour]->setEta(search.slv());

    cFrame[iplanarcontour]->getOrientation(false).set(0, planarcontour->getWn(t,cFrame[iplanarcontour]->getZeta()));
    cFrame[iplanarcontour]->getOrientation(false).set(1, planarcontour->getWu(t,cFrame[iplanarcontour]->getZeta()));
    cFrame[iplanarcontour]->getOrientation(false).set(2, planarcontour->getWv(t,cFrame[iplanarcontour]->getZeta()));
    cFrame[icircle]->getOrientation(false).set(0, -cFrame[iplanarcontour]->getOrientation(false).col(0));
    cFrame[icircle]->getOrientation(false).set(2, circle->getFrame()->evalOrientation().col(2));
    cFrame[icircle]->getOrientation(false).set(1, crossProduct(cFrame[icircle]->getOrientation(false).col(2),cFrame[icircle]->getOrientation(false).col(0)));
    cFrame[iplanarcontour]->setPosition(planarcontour->getPosition(t,cFrame[iplanarcontour]->getZeta()));
    cFrame[icircle]->setPosition(circle->getFrame()->evalPosition()+circle->getRadius()*cFrame[icircle]->getOrientation(false).col(0));

    if(planarcontour->isZetaOutside(cFrame[iplanarcontour]->getZeta()))
      g = 1;
    else
      g = cFrame[iplanarcontour]->getOrientation(false).col(0).T() * (cFrame[icircle]->getPosition(false) - cFrame[iplanarcontour]->getPosition(false));
    if(g < -planarcontour->getThickness()) g = 1;
  }

  void ContactKinematicsCirclePlanarContour::updatewb(double t, Vec &wb, double g, std::vector<ContourFrame*> &cFrame) {
    
    const Vec3 n1 = cFrame[icircle]->evalOrientation().col(0);
    const Vec3 u1 = cFrame[icircle]->getOrientation().col(1);
    const Vec3 R1 = circle->getRadius()*u1;
    const Vec3 N1 = u1;

    const Vec3 u2 = cFrame[iplanarcontour]->evalOrientation().col(1);
    const Vec3 v2 = cFrame[iplanarcontour]->getOrientation().col(2);
    const Vec3 R2 = planarcontour->getWs(t,cFrame[iplanarcontour]->getZeta());
    const Vec3 U2 = planarcontour->getParDer1Wu(t,cFrame[iplanarcontour]->getZeta());

    const Vec3 vC1 = cFrame[icircle]->evalVelocity();
    const Vec3 vC2 = cFrame[iplanarcontour]->evalVelocity();
    const Vec3 Om1 = cFrame[icircle]->evalAngularVelocity();
    const Vec3 Om2 = cFrame[iplanarcontour]->evalAngularVelocity();

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
