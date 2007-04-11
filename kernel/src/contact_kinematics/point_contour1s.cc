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
#include "point_contour1s.h"
#include "functions_contact.h"

namespace MBSim {

  void ContactKinematicsPointContour1s::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      icontour = 1;
      point = static_cast<Point*>(contour[0]);
      contour1s = static_cast<Contour1s*>(contour[1]);
    } else {
      ipoint = 1;
      icontour = 0;
      point = static_cast<Point*>(contour[1]);
      contour1s = static_cast<Contour1s*>(contour[0]);
    }
  }

  void ContactKinematicsPointContour1s::stage1(Vec &g, vector<ContourPointData> &cpData) {

    cpData[ipoint].WrOC = point->getWrOP();

    FuncPairContour1sPoint *func= new FuncPairContour1sPoint(point,contour1s);
    Contact1sSearch search(func);
    search.setNodes(contour1s->getNodes());

    if(cpData[icontour].alpha.size()!=0) {
      search.setInitialValue(cpData[icontour].alpha(0));
    } else {
      search.setSearchAll   (true);
      cpData[icontour].alpha = Vec(1);
    }

    cpData[icontour].alpha(0) = search.slv();
    double &alphaC = cpData[icontour].alpha(0);

    /* Pruefen, dass Kontakt nur im Kontaktbereich, sonst Ergebnis verwerfen 
    */
    if(alphaC < contour1s->getAlphaStart() || alphaC > contour1s->getAlphaEnd() )
      g(0) = 1.0;
    else {
      cpData[icontour].Wn    = contour1s->computeWn(cpData[icontour].alpha(0));
      cpData[ipoint].Wn      = -cpData[icontour].Wn;
      cpData[icontour].WrOC  = contour1s->computeWrOC(cpData[icontour].alpha(0));
      g(0) = trans(cpData[icontour].Wn) * (cpData[icontour].WrOC - cpData[ipoint].WrOC);
    }
  }

  void ContactKinematicsPointContour1s::stage2(const Vec& g, Vec &gd, vector<ContourPointData> &cpData) {

    Vec WvD = contour1s->computeWvC(cpData[icontour].alpha(0)) - point->getWvP();

    gd(0)  = trans(cpData[icontour].Wn)*WvD;

    if(cpData[0].Wt.cols()) {
      static Index iT(1,cpData[0].Wt.cols());
      cpData[icontour].Wt = contour1s->computeWt(cpData[icontour].alpha(0))(0,0,2,iT.end()-1);
      cpData[ipoint].Wt   = -cpData[icontour].Wt;
      gd(iT) = trans(cpData[icontour].Wt)*WvD;
    }
  }

}
