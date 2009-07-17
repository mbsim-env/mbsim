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
#include "edge_edge.h"
#include "mbsim/contours/edge.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsEdgeEdge::assignContours(const vector<Contour*> &contour) {
    iedge0 = 0; iedge1 = 1;
    edge0 = static_cast<Edge*>(contour[0]);
    edge1 = static_cast<Edge*>(contour[1]);
  }

  void ContactKinematicsEdgeEdge::stage1(Vec &g, vector<ContourPointData> &cpData) {
//
//    Vec Wd = edge1->getWrOP() - edge0->getWrOP();
//    Vec Wd0 = edge0->computeWd();
//    Vec Wd1 = edge1->computeWd();
//    cpData[iedge0].Wn = crossProduct(Wd0,Wd1);
//    cpData[iedge0].Wn = cpData[iedge0].Wn/nrm2(cpData[iedge0].Wn);
//    double d = trans(cpData[iedge0].Wn)*Wd;
//    if(d<0) {
//      cpData[iedge0].Wn *= -1;
//      d *= -1;
//    }
//    Vec We0 = edge0->computeWe();
//    Vec We1 = edge1->computeWe();
//    if(trans(cpData[iedge0].Wn)*We0 >= 0 && trans(cpData[iedge0].Wn)*We1 <= 0) {
//      if(-d < -0.01) {
//        g(0) = 1;
//      } else {
//        double t0 = trans(Wd0)*(Wd - Wd1*trans(Wd1)*Wd)/(1.0-trans(Wd0)*Wd1*trans(Wd1)*Wd0);
//        double t1 = t0*trans(Wd1)*Wd0 - trans(Wd1)*Wd;
//
//        if(t1 > edge1->getLimit() || t0 > edge0->getLimit() || t1 < 0 || t0 < 0)
//          g(0) = 1;
//        else {
//          WrPC[iedge1] = t1*Wd1;
//          WrPC[iedge0] = t0*Wd0;
//          g(0) = -d;
//        }
//      }
//    } else if(trans(cpData[iedge0].Wn)*We0 < 0 && trans(cpData[iedge0].Wn)*We1 > 0)
//      g(0) = d;
//    else
//      g(0) = 1;
//
//    cpData[iedge1].Wn = -cpData[iedge0].Wn;
  }

  void ContactKinematicsEdgeEdge::stage2(const Vec &g, Vec &gd, vector<ContourPointData> &cpData) {
//    if(g(0)>0.0) return;
//    cpData[iedge1].WrOC = edge1->getWrOP()+WrPC[iedge1];
//    cpData[iedge0].WrOC = edge0->getWrOP()+WrPC[iedge0];
//
//    Vec WvC[2];
//    WvC[iedge0] = edge0->getWvP()+crossProduct(edge0->getWomegaC(),WrPC[iedge0]);
//    WvC[iedge1] = edge1->getWvP()+crossProduct(edge1->getWomegaC(),WrPC[iedge1]);
//    Vec WvD = WvC[iedge0] - WvC[iedge1];
//    gd(0) = trans(cpData[iedge0].Wn)*WvD;
//
//    if(cpData[iedge0].Wt.cols()) {
//      cpData[iedge0].Wt.col(0) = computeTangential(cpData[iedge0].Wn);
//      cpData[iedge0].Wt.col(1) = crossProduct(cpData[iedge0].Wn,cpData[iedge0].Wt.col(0));
//      cpData[iedge1].Wt = -cpData[iedge0].Wt;
//      static Index iT(1,cpData[iedge0].Wt.cols());
//      gd(iT) = trans(cpData[iedge0].Wt)*WvD;
//    }
//
  }

}

