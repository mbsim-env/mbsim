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
 * Contact: mfoerg@users.berlios.de
 *          rzander@users.berlios.de
 */

#include <config.h>
#include "mbsim/contact_kinematics/circlesolid_contour1s.h"
#include "mbsim/contours/circle_solid.h"
#include "mbsim/contours/contour1s.h"
#include "mbsim/contours/contour1s_analytical.h"
#include "mbsim/functions_contact.h"
#include "mbsim/utils/eps.h"
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
      if (circle->getRadius()>minRadius) {
        cout << "Error! Just one contact point is allowed in Contactpairing Contour1s-CircleSolid, but either the circle radius is to big or the minimal Radius of Contour1s is to small. Continuing anyway..." << endl;
        cout << "minimal Radius of Contour1sAnalytical=" << minRadius << endl;
        cout << "Radius of CircleSolid=" << circle->getRadius() << endl;
      }
    }

  }

  void ContactKinematicsCircleSolidContour1s::updateg(fmatvec::Vec &g, ContourPointData *cpData) {
    Contact1sSearch search(func);
    search.setNodes(contour1s->getNodes());

    if(cpData[icontour1s].getLagrangeParameterPosition().size() == 1)
      search.setInitialValue(cpData[icontour1s].getLagrangeParameterPosition()(0));
    else { 
      search.setSearchAll(true);
      cpData[icontour1s].getLagrangeParameterPosition() = Vec(1);
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
    g(0) = -trans(cpData[icontour1s].getFrameOfReference().getOrientation().col(0))*WrD;
  }

  void ContactKinematicsCircleSolidContour1s::updatewb(Vec &wb, const Vec &g, ContourPointData* cpData) {
    
    const Vec KrPC1 = trans(circle->getFrame()->getOrientation())*(cpData[icircle].getFrameOfReference().getPosition() - circle->getFrame()->getPosition());
    const double zeta1=(KrPC1(1)>0) ? acos(KrPC1(0)/nrm2(KrPC1)) : 2.*M_PI - acos(KrPC1(0)/nrm2(KrPC1));
    const double sa1=sin(zeta1);
    const double ca1=cos(zeta1);
    const double r1=circle->getRadius();
    Vec Ks1(3, NONINIT);
    Ks1(0)=-r1*sa1;
    Ks1(1)=r1*ca1;
    Ks1(2)=0;
    const Vec s1=circle->getFrame()->getOrientation()*Ks1;
    const Vec t1=circle->getFrame()->getOrientation().col(2);
    Vec n1=crossProduct(s1, t1);
    n1/=nrm2(n1);
    const Vec u1=s1/nrm2(s1);
    const Vec R1(s1);
    Vec KN1(3,NONINIT);
    KN1(0)=-sa1;
    KN1(1)=ca1;
    KN1(2)=0;
    const Vec N1=circle->getFrame()->getOrientation()*KN1;
    Vec KU1(3,NONINIT);
    KU1(0)=-ca1;
    KU1(1)=-sa1;
    KU1(2)=0;
    const Vec U1=circle->getFrame()->getOrientation()*KU1;

    const Vec KrPC2 = trans(contour1s->getFrame()->getOrientation())*(cpData[icontour1s].getFrameOfReference().getPosition() - contour1s->getFrame()->getPosition());
    const double zeta2=(KrPC2(2)>0) ? acos(KrPC2(1)/nrm2(KrPC2)) : 2.*M_PI - acos(KrPC2(1)/nrm2(KrPC2));
    const double sa2=sin(zeta2);
    const double ca2=cos(zeta2);
    const double r2=contour1s->computeDistance(zeta2, 0);
    const double r2s=contour1s->computeDistance(zeta2, 1);
    const double r2ss=contour1s->computeDistance(zeta2, 2);
    Vec Ks2(3, NONINIT);
    Ks2(0)=0;
    Ks2(1)=-r2*sa2+r2s*ca2;
    Ks2(2)=r2*ca2+r2s*sa2;
    Vec Kt2(3, NONINIT);
    Kt2(0)=1;
    Kt2(1)=0;
    Kt2(2)=0;
    const Vec s2=contour1s->getFrame()->getOrientation()*Ks2;
    const Vec t2=contour1s->getFrame()->getOrientation()*Kt2;
    Vec n2=crossProduct(s2, t2);
    n2/=nrm2(n2);
    const Vec u2=s2/nrm2(s2);
    const Vec v2=crossProduct(n2, u2);
    const Vec R2(s2);
    const double c0=(r2*r2+2.*r2s*r2s-r2*r2ss)/sqrt((r2*r2+r2s*r2s)*(r2*r2+r2s*r2s)*(r2*r2+r2s*r2s));
    const double c1=r2*ca2+r2s*sa2;
    const double c2=-r2*sa2+r2s*ca2;
    Vec KU2(3,NONINIT);
    KU2(0)=0;
    KU2(1)=-c0*c1;
    KU2(2)=c0*c2;
    const Vec U2=contour1s->getFrame()->getOrientation()*KU2;

    const Vec vC1 = cpData[icircle].getFrameOfReference().getVelocity();
    const Vec vC2 = cpData[icontour1s].getFrameOfReference().getVelocity();
    const Vec Om1 = cpData[icircle].getFrameOfReference().getAngularVelocity();
    const Vec Om2 = cpData[icontour1s].getFrameOfReference().getAngularVelocity();

    SqrMat A(2,2,NONINIT);
    A(0,0)=-trans(u1)*R1;
    A(0,1)=trans(u1)*R2;
    A(1,0)=trans(u2)*N1;
    A(1,1)=trans(n1)*U2;
    Vec b(2,NONINIT);
    b(0)=-trans(u1)*(vC2-vC1);
    b(1)=-trans(v2)*(Om2-Om1);
    const Vec zetad = slvLU(A,b);

    const Mat tOm1 = tilde(Om1);
    const Mat tOm2 = tilde(Om2);
    
    wb(0) += (trans(vC2-vC1)*N1-trans(n1)*tOm1*R1)*zetad(0)+trans(n1)*tOm2*R2*zetad(1)-trans(n1)*tOm1*(vC2-vC1);
    if (wb.size()>1) 
      wb(1) += (trans(vC2-vC1)*U1-trans(u1)*tOm1*R1)*zetad(0)+trans(u1)*tOm2*R2*zetad(1)-trans(u1)*tOm1*(vC2-vC1);
  }
      
  void ContactKinematicsCircleSolidContour1s::computeCurvatures(fmatvec::Vec &r, ContourPointData* cpData) {
    r(icircle)=circle->computeCurvature(cpData[icircle]);
    r(icontour1s)=contour1s->computeCurvature(cpData[icontour1s]);
  }

}

