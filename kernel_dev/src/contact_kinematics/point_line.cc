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
#include "point_line.h"
#include "contour.h"

namespace MBSim {

  void ContactKinematicsPointLine::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0; iline = 1;
      point = static_cast<Point*>(contour[0]);
      line = static_cast<Line*>(contour[1]);
    } else {
      ipoint = 1; iline = 0;
      point = static_cast<Point*>(contour[1]);
      line = static_cast<Line*>(contour[0]);
    }
  }

  void ContactKinematicsPointLine::stage1(Vec &g, vector<ContourPointData> &cpData) {
    Vec Wd =  line->getWrOP() - point->getWrOP();
    cpData[iline].Wn = line->computeWn();
    cpData[ipoint].Wn = -cpData[iline].Wn;
    g(0) = trans(cpData[iline].Wn)*Wd;
  }

  void ContactKinematicsPointLine::stage2(const Vec& g, Vec &gd, vector<ContourPointData> &cpData) {
    Vec WrPC[2];

    cpData[ipoint].WrOC = point->getWrOP();
    cpData[iline].WrOC = cpData[ipoint].WrOC + cpData[iline].Wn*g;
    WrPC[iline] = cpData[iline].WrOC - line->getWrOP();
    Vec WvC[2];
    WvC[ipoint] = point->getWvP();
    WvC[iline] = line->getWvP()+crossProduct(line->getWomegaC(),WrPC[iline]);
    Vec WvD = WvC[iline] - WvC[ipoint];
    gd(0) = trans(cpData[iline].Wn)*WvD;

    if(cpData[ipoint].Wt.cols()) {
      cpData[iline].Wt = crossProduct(line->computeWb(),cpData[iline].Wn);
      cpData[ipoint].Wt = -cpData[iline].Wt;
      //if(gd.size() > 1) {
      static Index iT(1,cpData[ipoint].Wt.cols());
      gd(iT) = trans(cpData[iline].Wt)*WvD;
      //}
    }
  }
  
  void ContactKinematicsPointLine::updateb(Vec &b, const Vec &g, vector<ContourPointData> &cpData) {

    //double sd2 = 0; // point
    //double kappa1 = 0; // line
    //Mat Wt = cpData[iline].Wt;
    Vec b1 = line->computeWb(); // Mit - hat es funktioniert
    //cout << b1 << endl;
    Vec Wn1 = cpData[iline].Wn;
    Vec Wn2 = cpData[ipoint].Wn;
    Vec Wt1 = crossProduct(Wn1,b1);
    Vec Wt2 = -Wt1;
    Vec vC1 = line->getMovingFrame()->getVelocity();
    Vec vC2 = point->getMovingFrame()->getVelocity();
    Vec Om1 = line->getMovingFrame()->getAngularVelocity();
    Vec Om2 = point->getMovingFrame()->getAngularVelocity();

    if(true) {
      double kappa_sd2 = -trans(b1)*(Om2 - Om1);
      double sd1 = trans(Wt1)*(vC2 - vC1) - g(0)*trans(b1)*Om1;
      //cout<< "b0 = " << b << endl;
      b(0) += trans(Wn1)*(-crossProduct(Om1,vC1)) + sd1 * trans(b1) * Om1
	    + trans(Wn2)*(-crossProduct(Om2,vC2)) - kappa_sd2*trans(Wt2)*vC2;
      if(b.size() > 1)
	b(1) += trans(Wt1)*(-crossProduct(Om1,vC1)) + trans(Wt2)*(-crossProduct(Om2,vC2)) + kappa_sd2*trans(Wn2)*vC2;
      //cout<< "b1 = " << b << endl;
      //cout << "vc1 = " <<vC1 << endl;
      //cout<< "t1 = " << trans(Wt1)*(-crossProduct(Om1,vC1)) << endl;
      //cout<< "t2 = " << trans(Wt2)*(-crossProduct(Om2,vC2)) << endl;
      //cout<< "t3 = " <<  kappa_sd2*trans(Wn2)*vC2 << endl;
    } 
    else {
      //b(0) -= trans(cpData[iline].Wn)*(crossProduct(line->getMovingFrame()->getAngularVelocity(),line->getMovingFrame()->getVelocity()) - crossProduct(point->getMovingFrame()->getAngularVelocity(),point->getMovingFrame()->getVelocity())) - trans(line->getCb())*(sd[iline]*line->getMovingFrame()->getAngularVelocity());
//cout <<"Here = "<< trans(line->computeWb())*(sd[iline]*line->getMovingFrame()->getAngularVelocity()) << endl;
    }
  }
    //kappa[ipoint] = inf;
    //b(0) += trans(cpData[iline].Wn)*(sd[iline]*crossProduct(line->getWomegaC(),cpData[iline].Wt.col(0)) - sd[ipoint]*crossProduct(point->getWomegaC(),cpData[ipoint].Wt.col(0)) + crossProduct(line->getWomegaC(),line->getWrOP()) - crossProduct(point->getWomegaC(),point->getWrOP())) + trans(cpData[iline].Wt.col(0))*(kappa[iline]*sd[iline]*line->getWvP() - kappa[ipoint]*sd[ipoint]*point->getWvP()) ;

}

