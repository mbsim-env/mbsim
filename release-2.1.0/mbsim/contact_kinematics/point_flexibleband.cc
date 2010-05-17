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
 * Contact: thschindler@users.berlios.de
 */

#include<config.h>
#include "mbsim/contact_kinematics/point_flexibleband.h"
#include "mbsim/contour.h"
#include "mbsim/contours/flexible_band.h"
#include "mbsim/contours/point.h"
#include "mbsim/functions_contact.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  ContactKinematicsPointFlexibleBand::ContactKinematicsPointFlexibleBand() : ContactKinematics(),ipoint(0),icontour(0),point(0),band(0) {}
  ContactKinematicsPointFlexibleBand::~ContactKinematicsPointFlexibleBand() {}

  void ContactKinematicsPointFlexibleBand::assignContours(const vector<Contour*>& contour) {	
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      icontour = 1;
      point = static_cast<Point*>(contour[0]);
      band = static_cast<FlexibleBand*>(contour[1]);
    }
    else {
      ipoint = 1;
      icontour = 0;
      point = static_cast<Point*>(contour[1]);
      band = static_cast<FlexibleBand*>(contour[0]);
    }
  }

  void ContactKinematicsPointFlexibleBand::updateg(Vec& g, ContourPointData *cpData) {
    cpData[ipoint].getFrameOfReference().setPosition(point->getFrame()->getPosition()); // position of point

    FuncPairContour1sPoint *func= new FuncPairContour1sPoint(point,band); // root function for searching contact parameters
    Contact1sSearch search(func);
    search.setNodes(band->getNodes()); // defining search areas for contacts

    if(cpData[icontour].getLagrangeParameterPosition().size()!=0) { // select start value from last search
      search.setInitialValue(cpData[icontour].getLagrangeParameterPosition()(0));
    }
    else { // define start search with regula falsi
      search.setSearchAll(true);
      cpData[icontour].getLagrangeParameterPosition() = Vec(2,NONINIT);
    }

    cpData[icontour].getLagrangeParameterPosition()(0) = search.slv(); // get contact parameter of neutral fibre
    cpData[icontour].getLagrangeParameterPosition()(1) = 0.;

    if(cpData[icontour].getLagrangeParameterPosition()(0) < band->getAlphaStart() || cpData[icontour].getLagrangeParameterPosition()(0) > band->getAlphaEnd()) g(0) = 1.;
    else {
      band->updateKinematicsForFrame(cpData[icontour],position_cosy);
      Vec Wd =  cpData[ipoint].getFrameOfReference().getPosition() - cpData[icontour].getFrameOfReference().getPosition();
      Vec Wb = cpData[icontour].getFrameOfReference().getOrientation().col(2).copy();
      cpData[icontour].getLagrangeParameterPosition()(1) = Wb.T()*Wd; // get contact parameter of second tangential direction

      double width = band->getWidth();
      if(cpData[icontour].getLagrangeParameterPosition()(1) > 0.5*width || - cpData[icontour].getLagrangeParameterPosition()(1) > 0.5*width) g(0) = 1.;
      else { // calculate the normal distance
        cpData[icontour].getFrameOfReference().getPosition() += cpData[icontour].getLagrangeParameterPosition()(1)*Wb; 
        cpData[ipoint].getFrameOfReference().getOrientation().col(0) = -cpData[icontour].getFrameOfReference().getOrientation().col(0);
        cpData[ipoint].getFrameOfReference().getOrientation().col(1) = -cpData[icontour].getFrameOfReference().getOrientation().col(1);
        cpData[ipoint].getFrameOfReference().getOrientation().col(2) = cpData[icontour].getFrameOfReference().getOrientation().col(2);
        g(0) = cpData[icontour].getFrameOfReference().getOrientation().col(0).T() * (cpData[ipoint].getFrameOfReference().getPosition() - cpData[icontour].getFrameOfReference().getPosition());
      }
    }
    delete func;
  }

}

