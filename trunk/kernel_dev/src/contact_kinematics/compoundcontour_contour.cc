/* Copyright (C) 2008  Martin Förg
 
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
 *
 */

#include <config.h> 
#include "compoundcontour_contour.h"
#include "contour.h"
#include "contact_utils.h"

namespace MBSim {
  void ContactKinematicsCompoundContourContour::assignContours(const vector<Contour*> &contour_) {

    if(dynamic_cast<CompoundContour*>(contour_[0])) {
      icompound = 0;
      icontour = 1;
      compound = static_cast<CompoundContour*>(contour_[0]);
      contour = contour_[1];
    }
    else {
      icompound = 1;
      icontour = 0;
      compound = static_cast<CompoundContour*>(contour_[1]);
      contour = contour_[0];
    }

    Contour* c[2];
    c[icontour] = contour;
    numberOfPotentialContactPoints = 0;
    for(unsigned int i=0; i<compound->getNumberOfElements(); i++) {
      c[icompound] = compound->getContourElement(i);
      ContactKinematics *tmp = findContactPairing(c[0],c[1]);
      if(tmp) {
	contactKinematics.push_back(tmp); 
	tmp->assignContours(c[0],c[1]);
	numberOfPotentialContactPoints += tmp->getNumberOfPotentialContactPoints();
      }
    }
    cout << numberOfPotentialContactPoints << endl;
  }

  void ContactKinematicsCompoundContourContour::updateg(vector<Vec> &g, vector<ContourPointData*> &cpData) {
    for(unsigned int i=0; i<contactKinematics.size(); i++) 
      contactKinematics[i]->updateg(g[i], cpData[i]);

  }

  void ContactKinematicsCompoundContourContour::updategd(vector<Vec> &g, vector<Vec> &gd, vector<ContourPointData*> &cpData) {
    for(unsigned int i=0; i<contactKinematics.size(); i++) 
      contactKinematics[i]->updategd(g[i], gd[i], cpData[i]);
  }

}
