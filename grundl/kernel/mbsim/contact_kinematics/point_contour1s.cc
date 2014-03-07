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

#include "config.h"
#include "point_contour1s.h"
#include "mbsim/contours/point.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContactKinematicsPointContour1s::ContactKinematicsPointContour1s() :
      ContactKinematics(), ipoint(0), icontour(0), point(0), contour1s(0), func(0), search() {
  }
  ContactKinematicsPointContour1s::~ContactKinematicsPointContour1s() {
    delete func;
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

    func = new FuncPairContour1sPoint(point, contour1s); // root function for searching contact parameters

    search.setFunction(func);
    search.setSearchAll(true); // for first search it is important to have a global search
    search.setNodes(contour1s->getNodes()); // defining search areas for contacts
  }

  void ContactKinematicsPointContour1s::updateg(Vec &g, ContourPointData *cpData, int index) {
    cpData[ipoint].getFrameOfReference().setPosition(point->getFrame()->getPosition()); // position of point
    
    cpData[icontour].getLagrangeParameterPosition()(0) = search.slv(); // get contact parameter of neutral fibre

    if (cpData[icontour].getLagrangeParameterPosition()(0) < contour1s->getAlphaStart() || cpData[icontour].getLagrangeParameterPosition()(0) > contour1s->getAlphaEnd())
      g(0) = 1.0;
    else { // calculate the normal distance
      search.setSearchAll(false);
      search.setInitialValue(cpData[icontour].getLagrangeParameterPosition()(0));

      contour1s->updateKinematicsForFrame(cpData[icontour], position_cosy);
      cpData[ipoint].getFrameOfReference().getOrientation().set(0, -cpData[icontour].getFrameOfReference().getOrientation().col(0));
      cpData[ipoint].getFrameOfReference().getOrientation().set(1, -cpData[icontour].getFrameOfReference().getOrientation().col(1));
      cpData[ipoint].getFrameOfReference().getOrientation().set(2, cpData[icontour].getFrameOfReference().getOrientation().col(2));
      g(0) = cpData[icontour].getFrameOfReference().getOrientation().col(0).T() * (cpData[ipoint].getFrameOfReference().getPosition() - cpData[icontour].getFrameOfReference().getPosition());
    }
  }

}

