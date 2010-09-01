/* Copyright (C) 2004-2010 MBSim Development Team
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
#include "mbsimFlexibleBody/contact_kinematics/circle_nurbsdisk2s.h"
#include "mbsimFlexibleBody/functions_contact.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  ContactKinematicsCircleNurbsDisk2s::ContactKinematicsCircleNurbsDisk2s() : icircle(0), inurbsdisk(0), nurbsdisk(0), circle(0), LOCALSEARCH(false) {
#ifndef HAVE_NURBS
    throw MBSimError("ERROR(ContactKinematicsCircleNurbsDisk2s::ContactKinematicsCircleNurbsDisk2s): External NURBS library not implemented!");
#endif   
  }

  ContactKinematicsCircleNurbsDisk2s::~ContactKinematicsCircleNurbsDisk2s() {}

  void ContactKinematicsCircleNurbsDisk2s::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Circle*>(contour[0])) {
      icircle = 0;
      inurbsdisk = 1;
      circle = static_cast<Circle*>(contour[0]);
      nurbsdisk = static_cast<NurbsDisk2s*>(contour[1]);
    }
    else {
      icircle = 1;
      inurbsdisk = 0;
      circle = static_cast<Circle*>(contour[1]);
      nurbsdisk = static_cast<NurbsDisk2s*>(contour[0]);
    }
  }

  void ContactKinematicsCircleNurbsDisk2s::updateg(fmatvec::Vec& g, ContourPointData *cpData) {
    FuncPairCircleNurbsDisk2s *func= new FuncPairCircleNurbsDisk2s(circle, nurbsdisk); // root function for searching contact parameters
    Contact1sSearch search(func);

    if(cpData[icircle].getLagrangeParameterPosition().size()!=0  && LOCALSEARCH) { // select start value from last search (local search)
      search.setInitialValue(cpData[icircle].getLagrangeParameterPosition()(0));
    }
    else { // define start search with regula falsi (global search)
      search.setSearchAll(true);
      cpData[icircle].getLagrangeParameterPosition() = Vec(1,NONINIT);
    }

    int SEC = 16; // partition for regula falsi
    double drho = 2.*M_PI/SEC * 1.01; // 10% intersection for improved convergence of solver
    double rhoStartSpacing = -2.*M_PI*0.01*0.5;
    search.setEqualSpacing(SEC,rhoStartSpacing,drho); //set the nodes for regula falsi
    cpData[icircle].getLagrangeParameterPosition()(0) = search.slv(); // get contact parameter

    // point on the circle
    fmatvec::Vec P_circle(3,fmatvec::INIT,0);
    P_circle(0) = cos(cpData[icircle].getLagrangeParameterPosition()(0));   
    P_circle(1) = sin(cpData[icircle].getLagrangeParameterPosition()(0));
    P_circle = circle->getFrame()->getPosition() + circle->getRadius() * circle->getFrame()->getOrientation() * P_circle;  
    cpData[icircle].getFrameOfReference().setPosition(P_circle); // position of the point in world coordinates

    cpData[inurbsdisk].getLagrangeParameterPosition() = nurbsdisk->transformCW(nurbsdisk->getFrame()->getOrientation().T()*(cpData[icircle].getFrameOfReference().getPosition() - nurbsdisk->getFrame()->getPosition()));

    if(cpData[inurbsdisk].getLagrangeParameterPosition()(0) < (nurbsdisk->getAlphaStart())(0) || cpData[inurbsdisk].getLagrangeParameterPosition()(0) > (nurbsdisk->getAlphaEnd())(0)) g(0) = 1.;
    else {
      nurbsdisk->updateKinematicsForFrame(cpData[inurbsdisk],position_cosy); // writes the position, as well as the normal and the tangents into the frame of reference

      cpData[icircle].getFrameOfReference().getOrientation().col(0)= -cpData[inurbsdisk].getFrameOfReference().getOrientation().col(0);
      cpData[icircle].getFrameOfReference().getOrientation().col(1)= -cpData[inurbsdisk].getFrameOfReference().getOrientation().col(1);   
      cpData[icircle].getFrameOfReference().getOrientation().col(2)=  cpData[inurbsdisk].getFrameOfReference().getOrientation().col(2); // to have a legal framework the second tangent is not the negative of the tanget of the disk

      g(0) = cpData[inurbsdisk].getFrameOfReference().getOrientation().col(0).T() * (cpData[icircle].getFrameOfReference().getPosition() - cpData[inurbsdisk].getFrameOfReference().getPosition());
    }

    delete func;
  }

}

