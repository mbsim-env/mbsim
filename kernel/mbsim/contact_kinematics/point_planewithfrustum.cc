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
#include "mbsim/contact_kinematics/point_planewithfrustum.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/planewithfrustum.h"
#include "mbsim/utils/utils.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContactKinematicsPointPlaneWithFrustum::ContactKinematicsPointPlaneWithFrustum() {
  }

  void ContactKinematicsPointPlaneWithFrustum::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      iplane = 1;
      point = static_cast<Point*>(contour[0]);
      plane = static_cast<PlaneWithFrustum*>(contour[1]);
    }
    else {
      ipoint = 1;
      iplane = 0;
      point = static_cast<Point*>(contour[1]);
      plane = static_cast<PlaneWithFrustum*>(contour[0]);
    }
    h=plane->getFrustumHeight();
    double R=plane->getFrustumRadiusOnPlane();
    double r=plane->getFrustumRadiusOnTop();
    rho=plane->getRoundingRadius();

    // Statische Berechnungen aus Geometrieueberlegungen
    ET(0)=h;
    ET(1)=r;
    EP(1)=R;

    // Mittelpunkte der Rundungsradien
    double tan_alpha=h/(R-r);
    double alpha=atan(tan_alpha);
    MT =ET;
    MT(0)-= sign(h)*rho;
    MT(1)-= sign(h)*(1/cos(alpha)-1)*rho/tan_alpha;
    MP = EP;
    MP(0)+= sign(h)*rho;
    MP(1)+= sign(h)*(1/cos(alpha)-1)*rho/tan_alpha;

    rTop=MT(1);
    rPlane=MP(1);
    rFrustumTop=rTop +sign(h)*rho*sin(alpha);
    rFrustumPlane=rPlane - sign(h)*rho*sin(alpha);

    nFrustum(0)=(R-r)/h;
    nFrustum(1)=1;
    nFrustum/=nrm2(nFrustum)*sign(h);

    tFrustum=crossProduct(nFrustum, Vec3("[0; 0; 1]"));
    tFrustum/=nrm2(tFrustum);

    msg(Debug) << "ET=" << trans(ET) << endl;
    msg(Debug) << "EP=" << trans(EP) << endl;
    msg(Debug) << "MT=" << trans(MT) << endl;
    msg(Debug) << "MP=" << trans(MP) << endl;
    msg(Debug) << "nFrustum=" << trans(nFrustum) << endl;
    msg(Debug) << "tFrustum=" << trans(tFrustum) << endl;
    msg(Debug) << "rTop=" << rTop << endl;
    msg(Debug) << "rPlane=" << rPlane << endl;
    msg(Debug) << "rFrustumTop=" << rFrustumTop << endl;
    msg(Debug) << "rFrustumPlane=" << rFrustumPlane << endl;
  }

  void ContactKinematicsPointPlaneWithFrustum::updateg(double t, double &g, std::vector<ContourFrame*> &cFrame, int index) {

    Vec3 WrOPoint = point->getFrame()->evalPosition();
    Vec3 WrOPlane = plane->getFrame()->evalPosition();
    Vec3 WnContour = plane->getFrame()->getOrientation().col(0);
    Vec3 WrPlanePoint = -WrOPlane+WrOPoint;
    double d = nrm2(crossProduct(WnContour, WrPlanePoint));

    // SqrMat AKWTmp(3,3);
    // AKWTmp.col(0)=WnContour;
    // if (d<1e-9)
    //   AKWTmp.col(1) = plane->getFrame()->getOrientation().col(1);
    // else {
    //   AKWTmp.col(1)=crossProduct(crossProduct(AKWTmp.col(0), WrPlanePoint), AKWTmp.col(0));
    //   AKWTmp.col(1)/=nrm2(AKWTmp.col(1));
    // }
    // AKWTmp.col(2)=crossProduct(AKWTmp.col(0), AKWTmp.col(1));
    // SqrMat AKW=trans(AKWTmp);
    SqrMat3 AKW;
    AKW.set(0, WnContour.T());
    if (d<1e-9)
      AKW.set(1, plane->getFrame()->getOrientation().col(1).T());
    else {
      AKW.set(1, crossProduct(crossProduct(AKW.row(0).T(), WrPlanePoint), AKW.row(0).T()).T());
      AKW.set(1, AKW.row(1)/nrm2(AKW.row(1)));
    }
    AKW.set(2, crossProduct(AKW.row(0).T(), AKW.row(1).T()).T());
    Vec3 KrOPoint = AKW*WrPlanePoint;

    msg(Debug) << "\n\n\n\n" << endl;
    msg(Debug) << "WrOPoint=" << trans(WrOPoint) << endl;
    msg(Debug) << "WrOPlane=" << trans(WrOPlane) << endl;
    msg(Debug) << "WnContour=" << trans(WnContour) << endl;
    msg(Debug) << "WrPlanePoint=" << trans(WrPlanePoint) << endl;
    msg(Debug) << "d=" << d << endl;
    msg(Debug) << "AKW=" << AKW << endl;
    msg(Debug) << "KrOPoint=" << trans(KrOPoint) << endl;

    Vec3 Kn;
    Vec3 Kt;
    Vec3 Kb("[0; 0; 1]");
    Vec3 KrCP;
    if (d<rTop) { // Contact with small plane
      msg(Debug) << "Fall 1" << endl;
      Kn=Vec3("[1; 0; 0]");
      Kt=Vec3("[0; -1; 0]");
      g=KrOPoint(0)-h;
      KrCP(0)=h;
      KrCP(1)=KrOPoint(1);
    }
    else if (d<rFrustumTop) { // contact with inner rounding
      msg(Debug) << "Fall 2" << endl;
      double sin_alpha=(KrOPoint(1)-MT(1))/rho;
      double cos_alpha=cos(asin(sin_alpha));
      Kn(0)=cos_alpha;
      Kn(1)=sign(h)*sin_alpha;
      Kt(0)=sign(h)*sin_alpha;
      Kt(1)=-cos_alpha;
      KrCP=MT+sign(h)*rho*Kn;
      Vec3 KrPointCP=-KrOPoint+KrCP;
      g=-sign(KrPointCP.T()*Vec3("[1; 0; 0]"))*nrm2(KrPointCP);
    }
    else if (d<rFrustumPlane) { // contact with frustum
      msg(Debug) << "Fall 3" << endl;
      Kn=nFrustum;
      Kt=tFrustum;
      Vec3 KrExP;
      if (h<0) {
        KrExP=-EP+KrOPoint;
        KrCP=EP+(KrExP.T()*tFrustum)*tFrustum;
      }
      else {
        KrExP=-ET+KrOPoint;
        KrCP=ET+(KrExP.T()*tFrustum)*tFrustum;
      }
      g=sign(KrExP.T()*nFrustum)*nrm2(KrCP-KrOPoint);
    }
    else if (d < rPlane) { // contact with outer rounding
      msg(Debug) << "Fall 4" << endl;
      double sin_alpha=(-KrOPoint(1)+MP(1))/rho;
      double cos_alpha=cos(asin(sin_alpha));
      Kn(0)=cos_alpha;
      Kn(1)=sign(h)*sin_alpha;
      Kt(0)=sign(h)*sin_alpha;
      Kt(1)=-cos_alpha;
      KrCP=MP-sign(h)*rho*Kn;
      Vec3 KrPointCP=-KrOPoint+KrCP;
      g=-sign(KrPointCP.T()*Vec3("[1; 0; 0]"))*nrm2(KrPointCP);
    }
    else { // contact with infinite plane
      msg(Debug) << "Fall 5" << endl;
      Kn=Vec3("[1; 0; 0]");
      Kt=Vec3("[0; -1; 0]");
      g=KrOPoint(0);
      KrCP(0)=0.;
      KrCP(1)=KrOPoint(1);
    }

    SqrMat3 AWK = AKW.T();
    Vec3 Wn = AWK * Kn;
    Vec3 Wt = AWK * Kt;
    Vec3 Wb = AWK * Kb;
    Vec3 WrCP = WrOPlane + AWK * KrCP;

    msg(Debug) << "Kn=" << trans(Kn) << endl;
    msg(Debug) << "Kt=" << trans(Kt) << endl;
    msg(Debug) << "Kb=" << trans(Kb) << endl;
    msg(Debug) << "g=" << g << endl;
    msg(Debug) << "KrCP=" << trans(KrCP) << endl;
    msg(Debug) << "Wn=" << trans(Wn) << endl;
    msg(Debug) << "Wt=" << trans(Wt) << endl;
    msg(Debug) << "Wb=" << trans(Wb) << endl;
    msg(Debug) << "WrCP=" << trans(WrCP) << endl;
    msg(Debug) << "========================" << endl;

    cFrame[ipoint]->setPosition(WrOPoint);
    cFrame[iplane]->setPosition(WrCP);

    cFrame[iplane]->getOrientation(false).set(0, Wn);
    cFrame[iplane]->getOrientation(false).set(1, Wt);
    cFrame[iplane]->getOrientation(false).set(2, -Wb);
    cFrame[ipoint]->getOrientation(false).set(0, -Wn);
    cFrame[ipoint]->getOrientation(false).set(1, -Wt);
    cFrame[ipoint]->getOrientation(false).set(2, -Wb);
  }
}

