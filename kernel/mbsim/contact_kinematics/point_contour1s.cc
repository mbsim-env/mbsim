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
 * Contact: martin.o.foerg@googlemail.com
 *          rzander@users.berlios.de
 */

#include <config.h> 
#include "point_contour1s.h"
#include "mbsim/contours/point.h"
#include "mbsim/functions_contact.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContactKinematicsPointContour1s::ContactKinematicsPointContour1s() :
      ContactKinematics(), ipoint(0), icontour(0), point(0), contour1s(0), useLocal(false) {
  }
  ContactKinematicsPointContour1s::~ContactKinematicsPointContour1s() {
  }

  void ContactKinematicsPointContour1s::assignContours(const vector<Contour*> &contour) {
    if (dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      icontour = 1;
      point = static_cast<Point*>(contour[0]);
      contour1s = static_cast<Contour1s*>(contour[1]);
    }
    else {
      ipoint = 1;
      icontour = 0;
      point = static_cast<Point*>(contour[1]);
      contour1s = static_cast<Contour1s*>(contour[0]);
    }
  }

  void ContactKinematicsPointContour1s::updateg(double t, double &g, ContourPointData *cpData, int index) {
    
    FuncPairContour1sPoint *func = new FuncPairContour1sPoint(point, contour1s); // root function for searching contact parameters
    func->setTime(t);
    Contact1sSearch search(func);
    search.setNodes(contour1s->getNodes()); // defining search areas for contacts

    if (useLocal) { // select start value from last search
      search.setInitialValue(cpData[icontour].getLagrangeParameterPosition()(0));
    }
    else { // define start search with regula falsi
      search.setSearchAll(true);
      useLocal = true;
    }

    cpData[icontour].getLagrangeParameterPosition()(0) = search.slv(); // get contact parameter of neutral fibre

    if (cpData[icontour].getLagrangeParameterPosition()(0) < contour1s->getAlphaStart() || cpData[icontour].getLagrangeParameterPosition()(0) > contour1s->getAlphaEnd())
      g = 1.0;
    else { // calculate the normal distance
      cpData[icontour].getFrameOfReference().setPosition(contour1s->getPosition(t,cpData[icontour]));
      cpData[icontour].getFrameOfReference().getOrientation(false).set(0, contour1s->getWn(t,cpData[icontour]));
      cpData[icontour].getFrameOfReference().getOrientation(false).set(1, contour1s->getWu(t,cpData[icontour]));
      cpData[icontour].getFrameOfReference().getOrientation(false).set(2, contour1s->getWv(t,cpData[icontour]));

      cpData[ipoint].getFrameOfReference().setPosition(point->getFrame()->getPosition(t)); // position of point
      cpData[ipoint].getFrameOfReference().getOrientation().set(0, -cpData[icontour].getFrameOfReference().getOrientation(false).col(0));
      cpData[ipoint].getFrameOfReference().getOrientation().set(1, -cpData[icontour].getFrameOfReference().getOrientation(false).col(1));
      cpData[ipoint].getFrameOfReference().getOrientation().set(2, cpData[icontour].getFrameOfReference().getOrientation(false).col(2));
      g = cpData[icontour].getFrameOfReference().getOrientation(false).col(0).T() * (cpData[ipoint].getFrameOfReference().getPosition(false) - cpData[icontour].getFrameOfReference().getPosition(false));
    }
    delete func;
  }

}

