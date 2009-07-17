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
#include "line_contour1s.h"
#include "mbsim/contour.h"
#include "mbsim/contours/line.h"
#include "mbsim/functions_contact.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsLineContour1s::assignContours(const vector<Contour*> &contour) {
//    if(dynamic_cast<Line*>(contour[0])) {
//      iline = 0;
//      icontour = 1;
//      line = static_cast<Line*>(contour[0]);
//      contour1s = static_cast<Contour1s*>(contour[1]);
//    } else {
//      iline = 1;
//      icontour = 0;
//      line = static_cast<Line*>(contour[1]);
//      contour1s = static_cast<Contour1s*>(contour[0]);
//    }
//    func= new FuncPairContour1sLine(line,contour1s);
  }
  ContactKinematicsLineContour1s::~ContactKinematicsLineContour1s() {
   // delete func;
  }

  void ContactKinematicsLineContour1s::stage1(Vec &g, vector<ContourPointData> &cpData) {
//
//    Contact1sSearch search(func);
//    search.setNodes(contour1s->getNodes());     
//    if(cpData[icontour].alpha.size()==1) {
//      search.setInitialValue(cpData[icontour].alpha(0));
//    } else { 
//      search.setSearchAll   (true);
//      cpData[icontour].alpha = Vec(1);
//    }
//
//    cpData[icontour].alpha(0) = search.slv();
//
//    cpData[iline].Wn    = line->computeWn();
//    cpData[icontour].Wn = -cpData[iline].Wn;
//
//    cpData[icontour].WrOC = contour1s->computeWrOC(cpData[icontour]);
//
//    g(0)=trans(cpData[icontour].Wn)*(cpData[icontour].WrOC-line->getWrOP());
//
//    cpData[iline].WrOC = cpData[icontour].WrOC - g(0)*cpData[icontour].Wn;

  }

  void ContactKinematicsLineContour1s::stage2(const Vec &g, Vec &gd, vector<ContourPointData> &cpData) {

//    cpData[icontour].WrOC =  contour1s->computeWrOC(cpData[icontour]);
//    cpData[iline].WrOC = cpData[icontour].WrOC - g(0)*cpData[icontour].Wn;
//
//    Vec WrPCLine;
//    WrPCLine = cpData[iline].WrOC - line->getWrOP();
//
//    Vec WvC[2];
//    WvC[iline] = line->getWvP()+crossProduct(line->getWomegaC(),WrPCLine);
//    WvC[icontour] = contour1s->computeWvC(cpData[icontour]);
//
//    Vec WvD = WvC[icontour] - WvC[iline];
//    gd(0) = trans(cpData[icontour].Wn)*WvD;
//
//
//    if(cpData[0].Wt.cols()) {
//      //    cpData[icontour].Wt = contour1s->computeWt(cpData[icontour].alpha(0))(0,0,2,iT.end()-1);
//      static Index iT(1,cpData[0].Wt.cols());
//      cpData[iline].Wt    = line->computeWt()(0,0,2,iT.end()-1);
//      cpData[icontour].Wt = -cpData[iline].Wt; 
//      gd(iT) = trans(cpData[icontour].Wt)*WvD;
//    }
  }

}

