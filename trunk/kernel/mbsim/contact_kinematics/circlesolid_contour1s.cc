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
 *          rzander@users.berlios.de
 */

#include <config.h>
#include "mbsim/contact_kinematics/circlesolid_contour1s.h"
#include "mbsim/contours/circle_solid.h"
#include "mbsim/contours/contour1s.h"
#include "mbsim/contours/contour1s_analytical.h"
#include "mbsim/functions_contact.h"
#include "mbsim/utils/eps.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/contour_functions.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContactKinematicsCircleSolidContour1s::~ContactKinematicsCircleSolidContour1s() { 
    delete func; 
  }

  void ContactKinematicsCircleSolidContour1s::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<CircleSolid*>(contour[0])) {
      icircle = 0;
      icontour1s = 1;
      circle = static_cast<CircleSolid*>(contour[0]);
      contour1s = static_cast<Contour1s*>(contour[1]);
    } 
    else {
      icircle = 1; 
      icontour1s = 0;
      circle = static_cast<CircleSolid*>(contour[1]);
      contour1s = static_cast<Contour1s*>(contour[0]);
    }

    func = new FuncPairContour1sCircleSolid(circle,contour1s);

    if (dynamic_cast<Contour1sAnalytical*>(contour1s)) {
      double minRadius=1./epsroot();
      for (double alpha=contour1s->getAlphaStart(); alpha<=contour1s->getAlphaEnd(); alpha+=(contour1s->getAlphaEnd()-contour1s->getAlphaStart())*1e-4) {
        double radius=1./static_cast<Contour1sAnalytical*>(contour1s)->getContourFunction1s()->computeCurvature(alpha);
        minRadius=(radius<minRadius)?radius:minRadius;
      }
      if (circle->getRadius()>minRadius)
        throw MBSimError("Error! Just one contact point is allowed in Contactpairing Contour1s-CircleSolid, but either the circle radius is to big or the minimal Radius of Contour1s is to small.\n minimal radius of Contour1sAnalytical="+numtostr(minRadius)+"\n Radius of CircleSolid="+numtostr(circle->getRadius()));
    }

  }

  void ContactKinematicsCircleSolidContour1s::updateg(fmatvec::Vec &g, ContourPointData *cpData) {
    Contact1sSearch search(func);
    search.setNodes(contour1s->getNodes());

    if((cpData[icontour1s].getLagrangeParameterPosition().size() == 1) && (searchAllCP==false))
      search.setInitialValue(cpData[icontour1s].getLagrangeParameterPosition()(0));
    else { 
      search.setSearchAll(true);
      cpData[icontour1s].getLagrangeParameterPosition() = Vec(1);
      searchAllCP=false;
    }
    cpData[icontour1s].getLagrangeParameterPosition()(0) = search.slv();

    cpData[icontour1s].getFrameOfReference().getPosition() = 
      contour1s->computePosition(cpData[icontour1s]);
    cpData[icontour1s].getFrameOfReference().getOrientation().col(0) = 
      contour1s->computeNormal(cpData[icontour1s]);
    cpData[icontour1s].getFrameOfReference().getOrientation().col(1) = 
      contour1s->computeTangent(cpData[icontour1s]);
    cpData[icontour1s].getFrameOfReference().getOrientation().col(2) = 
      -contour1s->computeBinormal(cpData[icontour1s]);

    cpData[icircle].getFrameOfReference().getPosition() =
      circle->getFrame()->getPosition()-
      circle->getRadius()*cpData[icontour1s].getFrameOfReference().getOrientation().col(0);
    cpData[icircle].getFrameOfReference().getOrientation().col(0) = 
      -cpData[icontour1s].getFrameOfReference().getOrientation().col(0);
    cpData[icircle].getFrameOfReference().getOrientation().col(1) =
      -cpData[icontour1s].getFrameOfReference().getOrientation().col(1);
    cpData[icircle].getFrameOfReference().getOrientation().col(2) =
      cpData[icontour1s].getFrameOfReference().getOrientation().col(2);

    Vec WrD = func->computeWrD(cpData[icontour1s].getLagrangeParameterPosition()(0));
    g(0) = -cpData[icontour1s].getFrameOfReference().getOrientation().col(0).T()*WrD;
  }

  void ContactKinematicsCircleSolidContour1s::updatewb(Vec &wb, const Vec &g, ContourPointData* cpData) {
    
    const Vec n1=cpData[icircle].getFrameOfReference().getOrientation().col(0);
    const Vec u1=-cpData[icircle].getFrameOfReference().getOrientation().col(1);
    const Vec R1=circle->getRadius()*u1;
    const Vec N1=u1;

    const double zeta2=cpData[icontour1s].getLagrangeParameterPosition()(0);
    const double sa2=sin(zeta2);
    const double ca2=cos(zeta2);
    const double r2=contour1s->computeDistance(zeta2, 0);
    const double r2s=contour1s->computeDistance(zeta2, 1);
    const double r2ss=contour1s->computeDistance(zeta2, 2);
    Vec Ks2(3, NONINIT);
    Ks2(0)=0;
    Ks2(1)=-r2*sa2+r2s*ca2;
    Ks2(2)=r2*ca2+r2s*sa2;
    const Vec s2=contour1s->getFrame()->getOrientation()*Ks2;
    const Vec u2=-cpData[icontour1s].getFrameOfReference().getOrientation().col(1);
    const Vec v2=-cpData[icontour1s].getFrameOfReference().getOrientation().col(2);
    const Vec R2(s2);
    Vec KU2(3,NONINIT);
    KU2(0)=0;
    KU2(1)=-r2*ca2-r2s*sa2;
    KU2(2)=-r2*sa2+r2s*ca2;
    const double r2q=r2*r2;
    const double r2sq=r2s*r2s;
    const double r2qpr2sq=r2q+r2sq;
    KU2*=(r2q+2*r2sq-r2*r2ss)/sqrt(r2qpr2sq*r2qpr2sq*r2qpr2sq);
    const Vec U2=contour1s->getFrame()->getOrientation()*KU2;

    const Vec vC1 = cpData[icircle].getFrameOfReference().getVelocity();
    const Vec vC2 = cpData[icontour1s].getFrameOfReference().getVelocity();
    const Vec Om1 = cpData[icircle].getFrameOfReference().getAngularVelocity();
    const Vec Om2 = cpData[icontour1s].getFrameOfReference().getAngularVelocity();

    SqrMat A(2,2,NONINIT);
    A(0,0)=-u1.T()*R1;
    A(0,1)=u1.T()*R2;
    A(1,0)=u2.T()*N1;
    A(1,1)=n1.T()*U2;
    Vec b(2,NONINIT);
    b(0)=-u1.T()*(vC2-vC1);
    b(1)=-v2.T()*(Om2-Om1);
    const Vec zetad = slvLU(A,b);

    const Mat tOm1 = tilde(Om1);
    const Mat tOm2 = tilde(Om2);
    
    wb(0) += ((vC2-vC1).T()*N1-n1.T()*tOm1*R1)*zetad(0)+n1.T()*tOm2*R2*zetad(1)-n1.T()*tOm1*(vC2-vC1);
    if (wb.size()>1) {
      const Vec U1=-n1;
      wb(1) += ((vC2-vC1).T()*U1-u1.T()*tOm1*R1)*zetad(0)+u1.T()*tOm2*R2*zetad(1)-u1.T()*tOm1*(vC2-vC1);
    }
  }
      
  void ContactKinematicsCircleSolidContour1s::computeCurvatures(fmatvec::Vec &r, ContourPointData* cpData) {
    r(icircle)=circle->computeCurvature(cpData[icircle]);
    r(icontour1s)=contour1s->computeCurvature(cpData[icontour1s]);
  }

}

