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
 *          rzander@users.berlios.de
 */

#include <config.h> 
#include <mbsim/contact_kinematics/point_cylinderflexible.h>
#include <mbsim/contour.h>
#include <mbsim/functions_contact.h>

namespace MBSim {

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

    cpData[ipoint].cosy.setPosition(point->getFrame()->getPosition());

    // contact search on cylinder flexible
    Contact1sSearch search(func);
    search.setNodes(cylinder->getNodes());

    if(cpData[icylinder].alpha.size()==1) {
      search.setInitialValue(cpData[icylinder].alpha(0));
    }
    else {
      search.setSearchAll(true);
      cpData[icylinder].alpha = Vec(1,INIT,0.);
    }

    cpData[icylinder].alpha(0) = search.slv();

    cylinder->updateKinematicsForFrame(cpData[icylinder]); // TODO enum for kinematic election
    Vec WrD = cpData[ipoint].cosy.getPosition() - cpData[icylinder].cosy.getPosition();

    // contact in estimated contact area? 
    if(cpData[icylinder].alpha(0) < cylinder->getAlphaStart() || cpData[icylinder].alpha(0) > cylinder->getAlphaEnd() ) g(0) = 1.;
    else {
      cpData[ipoint].cosy.getOrientation().col(0) = -WrD/nrm2(WrD); // outpointing normal
      cpData[icylinder].cosy.getOrientation().col(0) = -cpData[ipoint].cosy.getOrientation().col(0);
      cpData[icylinder].cosy.getOrientation().col(2) = crossProduct(cpData[icylinder].cosy.getOrientation().col(0),cpData[icylinder].cosy.getOrientation().col(1));
      cpData[ipoint].cosy.getOrientation().col(1) = - cpData[icylinder].cosy.getOrientation().col(1);
      cpData[ipoint].cosy.getOrientation().col(2) = cpData[icylinder].cosy.getOrientation().col(2);
      g(0) = trans(cpData[ipoint].cosy.getOrientation().col(0))*WrD - cylinder->getRadius();
    }
  }

}

