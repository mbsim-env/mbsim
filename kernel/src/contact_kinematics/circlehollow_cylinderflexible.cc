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
#include "circlehollow_cylinderflexible.h"
#include "contour.h"
#include "functions_contact.h"

namespace MBSim {

  void ContactKinematicsCircleHollowCylinderFlexible::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<CircleHollow*>(contour[0])) {
      icircle = 0; icylinder = 1;
      circle = static_cast<CircleHollow*>(contour[0]);
      cylinder = static_cast<CylinderFlexible*>(contour[1]);
    } else {
      icircle = 1; icylinder = 0;
      circle = static_cast<CircleHollow*>(contour[1]);
      cylinder = static_cast<CylinderFlexible*>(contour[0]);
    }
    func= new FuncPairContour1sCircleHollow(circle,cylinder);
  }

  ContactKinematicsCircleHollowCylinderFlexible::~ContactKinematicsCircleHollowCylinderFlexible() {
    delete func;
  }

  void ContactKinematicsCircleHollowCylinderFlexible::stage1(Vec &g, vector<ContourPointData> &cpData) {

    Vec WrOP_circle =  circle->getWrOP();

    Contact1sSearch search(func);
    search.setNodes(cylinder->getNodes());

    if(cpData[icylinder].alpha.size()==2 ) {
      search.setInitialValue(cpData[icylinder].alpha(0));
    } else {
      search.setSearchAll   (true);
      cpData[icylinder].alpha = Vec(2);
    }

    cpData[icylinder].alpha(0) = search.slv();
    const double &alphaC = cpData[icylinder].alpha(0);

    Vec WrOP_Cyl =  cylinder->computeWrOC( cpData[icylinder] );
    Vec WrD      =  WrOP_Cyl - WrOP_circle;

    // Radien der beteiligten Koerper
    const double &R = circle  ->getRadius();
    const double &r = cylinder->getRadius();

    // Tangente und Binormale
    Vec WbK = circle->computeWb();
    Vec WtB = (cylinder->computeWt(alphaC)).col(0);

    const double cos_alpha = trans( WtB ) * WbK;
//    const double sin_alpha = sqrt(1-cos_alpha*cos_alpha);

    if( nrm2(WrD) > 0 ) {

      Vec b1 = WrD/nrm2(WrD);
      Vec b2 = crossProduct(WbK,b1); b2 /=nrm2(b2); // ist schon normiert b2 /= nrm2(b2);

      Vec be1, be2;
      double a;
      if( -0.99750 < cos_alpha && cos_alpha < 0.99750) { // bis ca 1Grad Verkippung ...
	be2 = crossProduct(WtB,WbK); be2 /= nrm2(be2);// ist schon normiert
	be1 = crossProduct(be2,WbK); be1 /= nrm2(be1);// ist schon normiert

	a   =  r/cos_alpha;
      }
      else {
	be1 = b1;
	be2 = b2;
	a   = r;
      }

      // Kontaktpaarung Kreis-Ellipse
      FuncPairEllipseCircle *funcPhi= new FuncPairEllipseCircle(R, a, r);

      funcPhi->setDiffVec(WrD);
      funcPhi->setEllipseCOS(be1,be2);

      Contact1sSearch searchPhi(funcPhi);
      searchPhi.setSearchAll(true);
      static const int                SEC = 16;
      static const double            dphi =   2*M_PI/SEC * 1.02;      // ueberschneidung zwischen 0 und 2*Pi erzeugen
      static const double phiStartSpacing = - 2*M_PI/SEC * 0.02 / 2.; // besser fuer konvergenz des Loesers
      searchPhi.setEqualSpacing(SEC,phiStartSpacing,dphi);
      cpData[icylinder].alpha(1) = searchPhi.slv();  /////// war mal phi
      Vec dTilde = funcPhi->computeWrD(cpData[icylinder].alpha(1));

      // Suchen fertig!!!
      delete funcPhi;

      cpData[icircle]  .WrOC = WrOP_circle + R * dTilde/nrm2(dTilde);
      cpData[icylinder].WrOC = WrOP_circle + dTilde;

      Vec normal           = (WrD - trans(WtB)*WrD*WtB );
      cpData[icircle].Wn   = normal/nrm2(normal);
      cpData[icylinder].Wn = - cpData[icircle].Wn;

      g(0) = trans(cpData[icylinder].Wn) * ( cpData[icylinder].WrOC - cpData[icircle].WrOC);

    }
    else { // kontaktpunktvektoren zumindest gueltig dimensionieren
      cpData[icircle]  .WrOC = WrOP_circle;
      cpData[icylinder].WrOC = cylinder->computeWrOC( cpData[icylinder] );

      if(cos_alpha > 0) g(0) = (   R*cos_alpha - r );
      else	      g(0) = ( - R*cos_alpha - r );
    }
  }

  void ContactKinematicsCircleHollowCylinderFlexible::stage2(const Vec& g, Vec &gd, vector<ContourPointData> &cpData) {

    Vec WvCcir = circle->getWvP()                        + tilde(circle->getWomegaC())                      *( cpData[icircle].WrOC-circle->getWrOP() );

    Vec WrDCyl = cpData[icylinder].WrOC                  - cylinder->computeWrOC(cpData[icylinder]);
    Vec WvCcyl = cylinder->computeWvC(cpData[icylinder]) + tilde(cylinder->computeWomega(cpData[icylinder]))*  WrDCyl; 

    Vec WvD    =  WvCcyl - WvCcir;

    gd(0)      = trans(cpData[icylinder].Wn)*WvD;
    if(cpData[0].Wt.cols()) {
      static Index iT(1,cpData[0].Wt.cols());
      cpData[icylinder].Wt.col(0) = (cylinder->computeWt(cpData[icylinder])).col(0);
      cpData[icylinder].Wt.col(1) = crossProduct(cpData[icylinder].Wn,cpData[icylinder].Wt.col(0));
      cpData[icircle].Wt          = - cpData[icylinder].Wt;
      gd(iT)     = trans(cpData[icylinder].Wt)*WvD;
    }

  }

}

