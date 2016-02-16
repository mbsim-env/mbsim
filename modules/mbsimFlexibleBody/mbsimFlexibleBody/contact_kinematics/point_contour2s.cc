/* Copyright (C) 2004-2015 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 *          rzander@users.berlios.de
 */

#include <config.h> 
#include <mbsimFlexibleBody/contact_kinematics/point_contour2s.h>
#include <mbsimFlexibleBody/contours/contour_2s_neutral_factory.h>
//#include <mbsim/contours/contour.h>
#include <mbsim/functions_contact.h>
#include "mbsim/contours/point.h"

using namespace fmatvec;
using namespace std;
using namespace MBSim;

namespace MBSimFlexibleBody {

  ContactKinematicsPointContour2s::~ContactKinematicsPointContour2s() {
    delete func;
  }

  void ContactKinematicsPointContour2s::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0; icontour2s = 1;
      point = static_cast<Point*>(contour[0]);
      contour2s = static_cast<Contour2s*>(contour[1]);
    }
    else {
      ipoint = 1; icontour2s = 0;
      point = static_cast<Point*>(contour[1]);
      contour2s = static_cast<Contour2s*>(contour[0]);
    }
    func= new FuncPairContour2sPoint(point,contour2s);
  }

  void ContactKinematicsPointContour2s::updateg(double t, double &g, ContourPointData *cpData, int index) {

    throw;
//    cpData[ipoint].getFrameOfReference().setPosition(point->getFrame()->getPosition());
//
//    // contact search on cylinder flexible
//    Contact2sSearch search(func);
//    // if the search grid is given by user, use that; otherwise call setEqualspacing to create a 11 * 11 grid to for initial searching.
//    if ((contour2s->getNodesU().size() != 0) && (contour2s->getNodesV().size() != 0))
//      search.setNodes(contour2s->getNodesU(), contour2s->getNodesV());
//    else
//      search.setEqualSpacing(10, 10, 0, 0, 0.1, 0.1);
//
//    if(cpData[icontour2s].getLagrangeParameterPosition().size() == 2) {
//      search.setInitialValue(cpData[icontour2s].getLagrangeParameterPosition());
//    }
//    else {
//      search.setSearchAll(true);
//      cpData[icontour2s].getLagrangeParameterPosition() = Vec2(INIT,0.);
//    }
//
//    cpData[icontour2s].getLagrangeParameterPosition() = search.slv();
//
//    contour2s->updateKinematicsForFrame(cpData[icontour2s],Frame::position_cosy);
//    Vec WrD = cpData[ipoint].getFrameOfReference().getPosition() - cpData[icontour2s].getFrameOfReference().getPosition();
//
//    // contact in estimated contact area?
//    if(cpData[icontour2s].getLagrangeParameterPosition()(0) < contour2s->getAlphaStart()(0) || cpData[icontour2s].getLagrangeParameterPosition()(0) > contour2s->getAlphaEnd()(0) ||
//       cpData[icontour2s].getLagrangeParameterPosition()(1) < contour2s->getAlphaStart()(1) || cpData[icontour2s].getLagrangeParameterPosition()(1) > contour2s->getAlphaEnd()(1))
//     g = 1.;
//    else {
//      // the normal direction calculate by nurbs is the cross product of the two tangent directions calculate by nurbsurface.deriveAt(), these two tangent direction are along the u and v direction which are
//      // are determinate by the given interpolated points. If the surface deformed, these interpolated points got from the surface may not lie in a retangle grid anymore. So u and v direction may not vertical to each other anymore,
//      // so as the two tangent directions. But in MBSim, we have to guarantee these three direction vectors to be orthogonal to each other. So here we change the second tangent direction to be the cross product of the normal and first tangent direction.
//      cpData[icontour2s].getFrameOfReference().getOrientation().set(2, crossProduct(cpData[icontour2s].getFrameOfReference().getOrientation().col(0), cpData[icontour2s].getFrameOfReference().getOrientation().col(1)));
//      cpData[ipoint].getFrameOfReference().getOrientation().set(0, -cpData[icontour2s].getFrameOfReference().getOrientation().col(0));
//      cpData[ipoint].getFrameOfReference().getOrientation().set(1, -cpData[icontour2s].getFrameOfReference().getOrientation().col(1));
//      cpData[ipoint].getFrameOfReference().getOrientation().set(2,  cpData[icontour2s].getFrameOfReference().getOrientation().col(2));   // to have a legal framework the second tangent is not the negative of the tanget of the disk
//
////      cout << "Normale: " <<  cpData[icontour2s].getFrameOfReference().getOrientation().col(0) << endl;
////      cout << "1.Tangente: " <<  cpData[icontour2s].getFrameOfReference().getOrientation().col(1) << endl;
////      cout << "2.Tangente: " <<  cpData[icontour2s].getFrameOfReference().getOrientation().col(2) << endl;
//
//      g = cpData[icontour2s].getFrameOfReference().getOrientation().col(0).T() * (cpData[ipoint].getFrameOfReference().getPosition() - cpData[icontour2s].getFrameOfReference().getPosition());
//
//    }
  }

}

