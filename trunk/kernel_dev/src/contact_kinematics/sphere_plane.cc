/* Copyright (C) 2007  Martin Förg, Roland Zander
 
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
 * Contact:
 *   mfoerg@users.berlios.de
 *   rzander@users.berlios.de
 *
 */

#include <config.h> 
#include "sphere_plane.h"
#include "contour.h"
#include "utils/contact_utils.h"

namespace MBSim {

  void ContactKinematicsSpherePlane::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Sphere*>(contour[0])) {
      isphere = 0; iplane = 1;
      sphere = static_cast<Sphere*>(contour[0]);
      plane = static_cast<Plane*>(contour[1]);
    } else {
      isphere = 1; iplane = 0;
      sphere = static_cast<Sphere*>(contour[1]);
      plane = static_cast<Plane*>(contour[0]);
    }
  }

  void ContactKinematicsSpherePlane::updateg(Vec &g, ContourPointData *cpData) {

    cpData[iplane].cosy.setOrientation(plane->getCoordinateSystem()->getOrientation());
    cpData[isphere].cosy.getOrientation().col(0) = -plane->getCoordinateSystem()->getOrientation().col(0);
    cpData[isphere].cosy.getOrientation().col(1) = -plane->getCoordinateSystem()->getOrientation().col(1);
    cpData[isphere].cosy.getOrientation().col(2) = plane->getCoordinateSystem()->getOrientation().col(2);

    Vec Wn = cpData[iplane].cosy.getOrientation().col(0);

    Vec Wd = sphere->getCoordinateSystem()->getPosition() - plane->getCoordinateSystem()->getPosition();

    g(0) = trans(Wn)*Wd - sphere->getRadius();

    cpData[isphere].cosy.setPosition(sphere->getCoordinateSystem()->getPosition() - Wn*sphere->getRadius());
    cpData[iplane].cosy.setPosition(cpData[isphere].cosy.getPosition() - Wn*g(0));
  }

  // void ContactKinematicsSpherePlane::stage1(Vec &g, vector<ContourPointData> &cpData) {

  //   Vec Wd = plane->getWrOP() - sphere->getWrOP();
  //   cpData[iplane].Wn = plane->computeWn();
  //   cpData[isphere].Wn = -cpData[iplane].Wn;
  //   g(0) = trans(cpData[iplane].Wn)*Wd - sphere->getRadius();
  // }

  void ContactKinematicsSpherePlane::updategd(const Vec& g, Vec &gd, ContourPointData *cpData) {}

//  void ContactKinematicsSpherePlane::stage2(const Vec& g, Vec &gd, vector<ContourPointData> &cpData) {
//
//    Vec WrPC[2], WvC[2];
//
//    WrPC[isphere] = cpData[iplane].Wn*sphere->getRadius();
//    cpData[isphere].WrOC = sphere->getWrOP()+WrPC[isphere];
//    cpData[iplane].WrOC = cpData[isphere].WrOC+cpData[isphere].Wn*g;
//    WrPC[iplane] = cpData[iplane].WrOC - plane->getWrOP();
//    WvC[isphere] = sphere->getWvP()+crossProduct(sphere->getWomegaC(),WrPC[isphere]);
//    WvC[iplane] = plane->getWvP()+crossProduct(plane->getWomegaC(),WrPC[iplane]);
//    Vec WvD = WvC[iplane] - WvC[isphere]; 
//    gd(0) = trans(cpData[iplane].Wn)*WvD;
//
//    if(cpData[iplane].Wt.cols()) {
//      //      Mat Wt[2];
//      cpData[iplane].Wt.col(0) = computeTangential(cpData[iplane].Wn);
//      if(cpData[iplane].Wt.cols()==2) 
//	cpData[iplane].Wt.col(1) = crossProduct(cpData[iplane].Wn,cpData[iplane].Wt.col(0));
//      cpData[isphere].Wt = -cpData[iplane].Wt; 
//      static Index iT(1,cpData[iplane].Wt.cols());
//      gd(iT) = trans(cpData[iplane].Wt)*WvD;
//    }
//  }

  void ContactKinematicsSpherePlane::updatewb(Vec &wb, const Vec &g, ContourPointData *cpData) {

    Vec v1 = cpData[iplane].cosy.getOrientation().col(2);
    Vec n1 = cpData[iplane].cosy.getOrientation().col(0);
    Vec n2 = cpData[isphere].cosy.getOrientation().col(0);
    Vec u1 = cpData[iplane].cosy.getOrientation().col(1);
    Vec vC1 = cpData[iplane].cosy.getVelocity();
    Vec vC2 = cpData[isphere].cosy.getVelocity();
    Vec Om1 = cpData[iplane].cosy.getAngularVelocity();
    Vec Om2 = cpData[isphere].cosy.getAngularVelocity();

    Vec KrPC2 = trans(sphere->getCoordinateSystem()->getOrientation())*(cpData[isphere].cosy.getPosition() - sphere->getCoordinateSystem()->getPosition());
    Vec zeta2 = computeAnglesOnUnitSphere(KrPC2/sphere->getRadius());
    double a2 = zeta2(0);
    double b2 = zeta2(1);
    Vec s1 = u1;
    Vec t1 = v1;


    double r = sphere->getRadius();
    Mat KR2(3,2,NONINIT);
    Vec Ks2(3,NONINIT);
    Ks2(0) = -r*sin(a2)*cos(b2);
    Ks2(1) = r*cos(a2)*cos(b2);
    Ks2(2) = 0;

    Vec Kt2(3,NONINIT);
    Kt2(0) = -r*cos(a2)*sin(b2);
    Kt2(1) = -r*sin(a2)*sin(b2);
    Kt2(2) = r*cos(b2);

    Vec s2 = sphere->getCoordinateSystem()->getOrientation()*Ks2;
    Vec t2 = sphere->getCoordinateSystem()->getOrientation()*Kt2;
    Vec u2 = s2/nrm2(s2);
    Vec v2 = crossProduct(n2,u2);

    Mat R1(3,2);
    R1.col(0) = s1;
    R1.col(1) = t1;

    Mat R2(3,2);
    R2.col(0) = s2;
    R2.col(1) = t2;

    Mat KU2(3,2,NONINIT);
    KU2(0,0) = -cos(a2);
    KU2(1,0) = -sin(a2);
    KU2(2,0) = 0;
    KU2(0,1) = 0;
    KU2(1,1) = 0;
    KU2(2,1) = 0;

    Mat KV2(3,2,NONINIT);
    KV2(0,0) = sin(a2)*sin(b2);
    KV2(1,0) = -cos(a2)*sin(b2);
    KV2(2,0) = 0;
    KV2(0,1) = -cos(a2)*cos(b2);
    KV2(1,1) = -sin(a2)*cos(b2);
    KV2(2,1) = -sin(b2);

    Mat U2 = sphere->getCoordinateSystem()->getOrientation()*KU2;
    Mat V2 = sphere->getCoordinateSystem()->getOrientation()*KV2;

    //cout << u1 << endl;
    //cout << R1 << endl;
    SqrMat A(4,4,NONINIT);
    A(Index(0,0),Index(0,1)) = -trans(u1)*R1;
    A(Index(0,0),Index(2,3)) = trans(u1)*R2;
    A(Index(1,1),Index(0,1)) = -trans(v1)*R1;
    A(Index(1,1),Index(2,3)) = trans(v1)*R2;
    A(Index(2,2),Index(0,1)).init(0);
    A(Index(2,2),Index(2,3)) = trans(n1)*U2;
    A(Index(3,3),Index(0,1)).init(0);
    A(Index(3,3),Index(2,3)) = trans(n1)*V2;

    Vec b(4,NONINIT);
    b(0) = -trans(u1)*(vC2-vC1);
    b(1) = -trans(v1)*(vC2-vC1);
    b(2) = -trans(v2)*(Om2-Om1);
    b(3) = trans(u2)*(Om2-Om1);
    //cout << A << endl;
    //cout << b << endl;
    Vec zetad =  slvLU(A,b);
    Vec zetad1 = zetad(0,1);
    Vec zetad2 = zetad(2,3);

    Mat tOm1 = tilde(Om1);
    Mat tOm2 = tilde(Om2);
    wb(0) += trans(n1)*(-tOm1*(vC2-vC1) - tOm1*R1*zetad1 + tOm2*R2*zetad2);

    if(wb.size() > 1) {
      wb(1) += trans(u1)*(-tOm1*(vC2-vC1) - tOm1*R1*zetad1 + tOm2*R2*zetad2);
      wb(2) += trans(v1)*(-tOm1*(vC2-vC1) - tOm1*R1*zetad1 + tOm2*R2*zetad2);
    }
      //wb(1) += trans(t1)*(-crossProduct(Om1,vC2-vC1) - crossProduct(Om1,t1)*ad1 + crossProduct(Om2,s2)*ad2);

  }
}
    //Vec KR2(3);
    //KR2(0) = -sphere->getRadius()*sin(a2);
    //KR2(1) = sphere->getRadius()*cos(a2);
    //Vec KU2(3);
    //KU2(0) = -cos(a2);
    //KU2(1) = -sin(a2);
    //Vec R2 = sphere->getCoordinateSystem()->getOrientation()*KR2;
    //Vec U2 = sphere->getCoordinateSystem()->getOrientation()*KU2;
    //Vec u1 = t1;
    //SqrMat A(2,2);
    //A(0,0) = -trans(u1)*R1;
    //A(0,1) = trans(u1)*R2;
    //A(1,0) = 0;
    //A(1,1) = trans(n1)*U2;
    //Vec b(2);
    //b(0) = -trans(u1)*(vC2-vC1);
    //b(1) = -trans(s2)*(Om2-Om1);
    //Vec zetad =  slvLU(A,b);
