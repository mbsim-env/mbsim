/* Copyright (C) 2004-2011 MBSim Development Team
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
#include <mbsimFlexibleBody/contact_kinematics/point_cylinderflexible.h>
#include <mbsimFlexibleBody/contours/cylinder_flexible.h>
#include <mbsim/contour.h>
#include <mbsim/functions_contact.h>
#include "mbsim/contours/point.h"

using namespace fmatvec;
using namespace std;
using namespace MBSim;

namespace MBSimFlexibleBody {

  ContactKinematicsPointCylinderFlexible::~ContactKinematicsPointCylinderFlexible() {
    delete func;
  }

  void ContactKinematicsPointCylinderFlexible::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0; icylinder = 1;
      point = static_cast<Point*>(contour[0]);
      cylinder = static_cast<CylinderFlexible*>(contour[1]);
    }
    else {
      ipoint = 1; icylinder = 0;
      point = static_cast<Point*>(contour[1]);
      cylinder = static_cast<CylinderFlexible*>(contour[0]);
    }
    func= new FuncPairContour1sPoint(point,cylinder);
  }

  void ContactKinematicsPointCylinderFlexible::updateg(Vec &g, ContourPointData *cpData) {

    cpData[ipoint].getFrameOfReference().setPosition(point->getFrame()->getPosition());

    // contact search on cylinder flexible
    Contact1sSearch search(func);
    search.setNodes(cylinder->getNodes());

    if(cpData[icylinder].getLagrangeParameterPosition().size()==1) {
      search.setInitialValue(cpData[icylinder].getLagrangeParameterPosition()(0));
    }
    else {
      search.setSearchAll(true);
      cpData[icylinder].getLagrangeParameterPosition() = Vec(1,INIT,0.);
    }

    cpData[icylinder].getLagrangeParameterPosition()(0) = search.slv();

    cylinder->updateKinematicsForFrame(cpData[icylinder],position_cosy); 
    Vec3 WrD = cpData[ipoint].getFrameOfReference().getPosition() - cpData[icylinder].getFrameOfReference().getPosition();

    // contact in estimated contact area? 
    if(cpData[icylinder].getLagrangeParameterPosition()(0) < cylinder->getAlphaStart() || cpData[icylinder].getLagrangeParameterPosition()(0) > cylinder->getAlphaEnd() ) g(0) = 1.;
    else {
      cpData[ipoint].getFrameOfReference().getOrientation().set(0, -WrD/nrm2(WrD)); // outpointing normal
      cpData[icylinder].getFrameOfReference().getOrientation().set(0, -cpData[ipoint].getFrameOfReference().getOrientation().col(0));
      cpData[icylinder].getFrameOfReference().getOrientation().set(2, crossProduct(cpData[icylinder].getFrameOfReference().getOrientation().col(0),cpData[icylinder].getFrameOfReference().getOrientation().col(1)));
      cpData[ipoint].getFrameOfReference().getOrientation().set(1, - cpData[icylinder].getFrameOfReference().getOrientation().col(1));
      cpData[ipoint].getFrameOfReference().getOrientation().set(2, cpData[icylinder].getFrameOfReference().getOrientation().col(2));
      g(0) = cpData[ipoint].getFrameOfReference().getOrientation().col(0).T()*WrD - cylinder->getRadius();
    }
  }

}

