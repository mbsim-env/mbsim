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
#include "point_cylinderflexible.h"
#include "contour.h"
#include "functions_contact.h"

namespace MBSim {

  void ContactKinematicsPointCylinderFlexible::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0; icylinder = 1;
      point = static_cast<Point*>(contour[0]);
      cylinder = static_cast<CylinderFlexible*>(contour[1]);
    } else {
      ipoint = 1; icylinder = 0;
      point = static_cast<Point*>(contour[1]);
      cylinder = static_cast<CylinderFlexible*>(contour[0]);
    }
    func= new FuncPairContour1sPoint(point,cylinder);
  }
  ContactKinematicsPointCylinderFlexible::~ContactKinematicsPointCylinderFlexible() {
    delete func;
  }

  void ContactKinematicsPointCylinderFlexible::stage1(Vec &g, vector<ContourPointData> &cpData) {

    cpData[ipoint].WrOC     = point->getWrOP();

    Contact1sSearch search(func);
    search.setNodes(cylinder->getNodes());

    if(cpData[icylinder].alpha.size()==1) {
      search.setInitialValue(cpData[icylinder].alpha(0));
    } else {
      search.setSearchAll(true);
      cpData[icylinder].alpha = Vec(1,INIT,0.0);
    }

    cpData[icylinder].alpha(0) = search.slv();

    Vec WrOC_CylBuf = cylinder->computeWrOC(cpData[icylinder]);
    Vec WrD         = cpData[ipoint].WrOC - WrOC_CylBuf;

    /*! Pruefen, dass Kontakt nur im Kontaktbereich, sonst Ergebnis verwerfen 
    */
    if(cpData[icylinder].alpha(0) < cylinder->getAlphaStart() || cpData[icylinder].alpha(0) > cylinder->getAlphaEnd() )
      g(0) = 1.0;
    else {
      cpData[ipoint].Wn    = WrD/nrm2(WrD);
      cpData[icylinder].Wn = -cpData[ipoint].Wn;
      const double &r = cylinder->getRadius();
      cpData[icylinder].WrOC  = WrOC_CylBuf - cpData[icylinder].Wn*r; // TODO: Vorzeichen pruefen!!!
      g(0) = trans(cpData[ipoint].Wn)*WrD - r;
    }

    //  cout << " Start" << endl;
    //  cout << "cpData[ipoint].Wn       = " << trans(cpData[ipoint].Wn) << endl;
    //  cout << "cpData[ipoint].WrOC     = " << trans(cpData[ipoint].WrOC) << endl;
    //  cout << "cpData[icylinder].alpha = " << cpData[icylinder].alpha(0) << endl;
    //  cout << "cpData[icylinder].Wn    = " << trans(cpData[icylinder].Wn) << endl;
    //  cout << "cpData[icylinder].WrOC  = " << trans(cpData[icylinder].WrOC) << endl;
    //  cout << "WrD = " << trans(WrD) << endl;
    //  cout << "g = " << g(0) << endl;
    //  cout << "----------------" << endl;
  }

  void ContactKinematicsPointCylinderFlexible::stage2(const Vec& g, Vec &gd, vector<ContourPointData> &cpData) {

    Vec WvCylP = cylinder->computeWvC(cpData[icylinder]) - tilde(cylinder->computeWomega(cpData[icylinder]))*cpData[icylinder].Wn*cylinder->getRadius();

    Vec WvD = point->getWvP() - WvCylP;

    gd(0)  = trans(cpData[ipoint].Wn)*WvD;

    if(cpData[0].Wt.cols()) {
      static Index iT(1,cpData[0].Wt.cols());
      cpData[icylinder].Wt.col(0) = (cylinder->computeWt(cpData[icylinder])).col(0);
      cpData[icylinder].Wt.col(1) = crossProduct(cpData[icylinder].Wn,cpData[icylinder].Wt.col(0));
      cpData[ipoint].Wt           = - cpData[icylinder].Wt;
      gd(iT) = trans(cpData[icylinder].Wt)*WvD;
    }
  }

}

