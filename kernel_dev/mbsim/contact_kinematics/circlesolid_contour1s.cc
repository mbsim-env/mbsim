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
#include "mbsim/functions_contact.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContactKinematicsCircleSolidContour1s::~ContactKinematicsCircleSolidContour1s() { delete func; }

  void ContactKinematicsCircleSolidContour1s::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<CircleSolid*>(contour[0])) {
      icircle = 0; icontour = 1;
      circle = static_cast<CircleSolid*>(contour[0]);
      contour1d = static_cast<Contour1s*>(contour[1]);
    } 
    else {
      icircle = 1; icontour = 0;
      circle = static_cast<CircleSolid*>(contour[1]);
      contour1d = static_cast<Contour1s*>(contour[0]);
    }
    func= new FuncPairContour1sCircleSolid(circle,contour1d);
  }

  void ContactKinematicsCircleSolidContour1s::updateg(fmatvec::Vec &g, ContourPointData *cpData) {}

//  void ContactKinematicsCircleSolidContour1s::stage1(Vec &g, vector<ContourPointData> &cpData) {

//    Contact1sSearch search(func);
//    search.setNodes(contour1d->getNodes());     
//
//    if(cpData[icontour].alpha.size() == 1) {
//      search.setInitialValue(cpData[icontour].alpha(0));
//    } else { 
//      search.setSearchAll   (true);
//      cpData[icontour].alpha = Vec(1);
//    }
//    // gb >0: kein Kontakt;  
//    double gb = 0.0; 
//    double contour1dWidth = contour1d->getWidth();
//
//    if (contour1dWidth) {                     		// calculate gb only if contour1s::width is set. (deafult 0.0)
//      gb = abs(trans(contour1d->computeWb(cpData[icontour].alpha(0)))*(contour1d->getWrOP()-circle->getWrOP()))-contour1dWidth/2.0;
//    }
//    if (gb > 0) {
//      g(0) = gb;}
//    else {
//      cpData[icontour].alpha(0) = search.slv();
//
//      cpData[icontour].Wn = contour1d->computeWn(cpData[icontour].alpha(0));
//      cpData[icircle].Wn = - cpData[icontour].Wn;
//      Vec WrD = func->computeWrD(cpData[icontour].alpha(0));
//
//      g(0) = trans(cpData[icontour].Wn)*WrD;
//    }
//  }

//  void ContactKinematicsCircleSolidContour1s::stage2(const Vec& g, Vec &gd, vector<ContourPointData> &cpData) {

//    Vec WrPCCircle;
//    WrPCCircle = cpData[icontour].Wn*circle->getRadius();
//    cpData[icircle].WrOC = circle->getWrOP()+WrPCCircle;
//    cpData[icontour].WrOC =  contour1d->computeWrOC(cpData[icontour].alpha(0));
//
//    Vec WvC[2];
//    WvC[icircle] = circle->getWvP()+crossProduct(circle->getWomegaC(),WrPCCircle);
//    WvC[icontour] = contour1d->computeWvC(cpData[icontour].alpha(0));
//
//    Vec WvD = WvC[icontour] - WvC[icircle];
//
//    if(cpData[0].Wt.cols()) {
//      static Index iT(1,cpData[0].Wt.cols());
//      cpData[icontour].Wt = (contour1d->computeWt(cpData[icontour].alpha(0)))(0,0,2,iT.end()-1);
//      cpData[icircle].Wt = -cpData[icontour].Wt;
//      gd(iT) = trans(cpData[icontour].Wt)*WvD;
//    }
//
//    gd(0) = trans(cpData[icontour].Wn)*WvD;
//  }

}

