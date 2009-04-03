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
#include "point_contourinterpolation.h"
#include <mbsim/contour.h>
#include <mbsim/functions_contact.h>
#include <mbsim/utils/nonlinear_algebra.h>

namespace MBSim {

  void ContactKinematicsPointContourInterpolation::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0; idinterpol = 1;
      point = static_cast<Point*>(contour[0]);
      cinterpol = static_cast<ContourInterpolation*>(contour[1]);
    } 
    else {
      ipoint = 1; idinterpol = 0;
      point = static_cast<Point*>(contour[1]);
      cinterpol = static_cast<ContourInterpolation*>(contour[0]);
    }
    func = new FuncPairPointContourInterpolation(point,cinterpol);
  }

  ContactKinematicsPointContourInterpolation::~ContactKinematicsPointContourInterpolation() {
    delete func;
  }


  // Point - Interpolationskontour
  void ContactKinematicsPointContourInterpolation::stage1(Vec &g, vector<ContourPointData> &cpData) {

//    // MultiDimNewton zur Kontaktpunktsuche
//    MultiDimNewtonMethod rf(func);
//
//    // Nullstellensuche aufsetzen, wenn moeglich mit alter Loesung
//    if(cpData[idinterpol].alpha.size() == cinterpol->getNContourParameters())
//      cpData[idinterpol].alpha = rf.slv(cpData[idinterpol].alpha);
//    else {
//      cpData[idinterpol].type  = EXTINTERPOL;  
//      cpData[idinterpol].alpha = rf.slv( Vec(cinterpol->getNContourParameters(),INIT,0.0) );
//    }
//
//
//    if(cinterpol->testInsideBounds(cpData[idinterpol])) {
//      // im zulaessigen Bereich
//      cpData[ipoint].WrOC = point->getWrOP();
//
//      cpData[idinterpol].type     = EXTINTERPOL;
//      cpData[idinterpol].WrOC     = cinterpol->computeWrOC(cpData[idinterpol]);
//      //    cpData[idinterpol].iWeights = cinterpol->computePointWeights(cpData[idinterpol].alpha); ist eh in stage2
//
//      Vec Wd = cpData[idinterpol].WrOC - cpData[ipoint].WrOC;
//      cpData[idinterpol].Wn       = cinterpol->computeWn(cpData[idinterpol]);
//      cpData[ipoint].Wn           = -cpData[idinterpol].Wn;
//      g(0) = trans(cpData[idinterpol].Wn)*Wd;
//    } else {
//      // ausserhalb zulaessigen Bereich
//      g(0) = 1.0;
//    }  
  }

  void ContactKinematicsPointContourInterpolation::stage2(const Vec& g, Vec &gd, vector<ContourPointData> &cpData) {

//    if(g(0)>0.0) return;
//    // Contour-Interpolation
//    cpData[idinterpol].WrOC = cinterpol->computeWrOC(cpData[idinterpol].alpha);
//
//    // Interpolation
//    cpData[idinterpol].iPoints = cinterpol->getPoints();
//    cpData[idinterpol].iWeights = Vec(cpData[idinterpol].iPoints.size(),NONINIT);
//    for(unsigned int i=0;i<cpData[idinterpol].iPoints.size();i++)
//      cpData[idinterpol].iWeights(i) = cinterpol->computePointWeight(cpData[idinterpol].alpha, i);      
//
//    // Kontakt-Geschwindigkeiten
//    Vec WvD = cinterpol->computeWvC(cpData[idinterpol]) - point->getWvP();; 
//    gd(0) = trans(cpData[idinterpol].Wn)*WvD;
//    if(cpData[0].Wt.cols()) {
//      cpData[idinterpol].Wt = cinterpol->computeWt(cpData[idinterpol]);
//      cpData[ipoint].Wt     = -cpData[idinterpol].Wt;
//      static Index iT(1,cpData[0].Wt.cols());
//      gd(iT) = trans(cpData[idinterpol].Wt)*WvD;
//    }
//
  }

}

