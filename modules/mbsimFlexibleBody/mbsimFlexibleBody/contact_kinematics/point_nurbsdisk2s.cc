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
 */

#include <config.h>
#include "mbsimFlexibleBody/contact_kinematics/point_nurbsdisk2s.h"

using namespace fmatvec;
using namespace std;
using namespace MBSim;

namespace MBSimFlexibleBody {

  ContactKinematicsPointNurbsDisk2s::ContactKinematicsPointNurbsDisk2s() : ipoint(0), inurbsdisk(0), point(0), nurbsdisk(0) {
#ifndef HAVE_NURBS
    throw MBSim::MBSimError("ERROR(ContactKinematicsPointNurbsDisk2s::ContactKinematicsPointNurbsDisk2s): External NURBS library not implemented!");
#endif        
  }

  ContactKinematicsPointNurbsDisk2s::~ContactKinematicsPointNurbsDisk2s() {}

  void ContactKinematicsPointNurbsDisk2s::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      inurbsdisk = 1;
      point = static_cast<Point*>(contour[0]);
      nurbsdisk = static_cast<NurbsDisk2s*>(contour[1]);
    }
    else {
      ipoint = 1;
      inurbsdisk = 0;
      point = static_cast<Point*>(contour[1]);
      nurbsdisk = static_cast<NurbsDisk2s*>(contour[0]);
    }
  }

  void ContactKinematicsPointNurbsDisk2s::updateg(Vec &g, ContourPointData* cpData) {
    cpData[ipoint].getFrameOfReference().setPosition(point->getFrame()->getPosition()); // position of the point in worldcoordinates
    cpData[inurbsdisk].getLagrangeParameterPosition() = nurbsdisk->transformCW(nurbsdisk->getFrame()->getOrientation().T()*(cpData[ipoint].getFrameOfReference().getPosition() - nurbsdisk->getFrame()->getPosition())); // position of the point in the cylinder-coordinates of the disk -> NO CONTACTSEARCH

    /*TESTING*/
    //cout << "Platten-Posi:" << nurbsdisk->getFrame()->getPosition() << endl;
    //cout << "Punkt-Posi:" << cpData[ipoint].getFrameOfReference().getPosition() << endl;
    //cout << "NurbsDisk-Orientation:" << nurbsdisk->getFrame()->getOrientation() << endl;
    //cout << "nach TransformCW: " << cpData[inurbsdisk].getLagrangeParameterPosition() << endl;
    /*END-TESTING*/

    if(cpData[inurbsdisk].getLagrangeParameterPosition()(0) < (nurbsdisk->getAlphaStart())(0) || cpData[inurbsdisk].getLagrangeParameterPosition()(0) > (nurbsdisk->getAlphaEnd())(0)) g(0) = 1.;
    else {
      nurbsdisk->updateKinematicsForFrame(cpData[inurbsdisk],position_cosy); // writes the position, as well as the normal and the tangents into the FrameOfReference
      // cout << "Position auf Scheibe: " << cpData[inurbsdisk].getFrameOfReference().getPosition() << endl;

      cpData[ipoint].getFrameOfReference().getOrientation().set(0, -cpData[inurbsdisk].getFrameOfReference().getOrientation().col(0));
      cpData[ipoint].getFrameOfReference().getOrientation().set(1, -cpData[inurbsdisk].getFrameOfReference().getOrientation().col(1));   
      cpData[ipoint].getFrameOfReference().getOrientation().set(2,  cpData[inurbsdisk].getFrameOfReference().getOrientation().col(2));   // to have a legal framework the second tangent is not the negative of the tanget of the disk

      //cout << "Normale: " <<  cpData[inurbsdisk].getFrameOfReference().getOrientation().col(0) << endl;
      //cout << "1.Tangente: " <<  cpData[inurbsdisk].getFrameOfReference().getOrientation().col(1) << endl;
      //cout << "2.Tangente: " <<  cpData[inurbsdisk].getFrameOfReference().getOrientation().col(2) << endl;

      g(0) = cpData[inurbsdisk].getFrameOfReference().getOrientation().col(0).T() * (cpData[ipoint].getFrameOfReference().getPosition() - cpData[inurbsdisk].getFrameOfReference().getPosition());
      // cout << "Abstand: " << g(0) << endl;
    } 
  }

}

