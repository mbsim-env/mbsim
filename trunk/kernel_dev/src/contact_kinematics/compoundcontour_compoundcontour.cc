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
#include "compoundcontour_compoundcontour.h"
#include "contour.h"
#include "contact_utils.h"

namespace MBSim {

  void ContactKinematicsCompoundContourCompoundContour::assignContours(const vector<Contour*> &contour) {

    contour0 = static_cast<CompoundContour*>(contour[0]);
    contour1 = static_cast<CompoundContour*>(contour[1]);

    numberOfPotentialContactPoints = 0;
    for(unsigned int i=0; i<contour0->getNumberOfElements(); i++) {
      for(unsigned int j=0; i<contour1->getNumberOfElements(); j++) {
	ContactKinematics *tmp = findContactPairing(contour0->getContourElement(i),contour1->getContourElement(j));
	if(tmp) {
	  contactKinematics.push_back(tmp); 
	  tmp->assignContours(contour0->getContourElement(i),contour1->getContourElement(j));
	  numberOfPotentialContactPoints += tmp->getNumberOfPotentialContactPoints();
	}
      }
    }
  }

  void ContactKinematicsCompoundContourCompoundContour::updateg(vector<Vec> &g, vector<ContourPointData*> &cpData) {
   for(unsigned int i=0, k=0; i<contactKinematics.size(); i++) {
      contactKinematics[i]->updateg(g[k], cpData[k]);
      k += contactKinematics[i]->getNumberOfPotentialContactPoints();
   }
  }

  void ContactKinematicsCompoundContourCompoundContour::updategd(vector<Vec> &g, vector<Vec> &gd, vector<ContourPointData*> &cpData) {
    for(unsigned int i=0, k=0; i<contactKinematics.size(); i++) {
      contactKinematics[i]->updategd(g[k], gd[k], cpData[k]);
      k += contactKinematics[i]->getNumberOfPotentialContactPoints();
    }
  }

  void ContactKinematicsCompoundContourCompoundContour::updatewb(vector<Vec> &wb, vector<Vec> &g, vector<ContourPointData*> &cpData) {
    for(unsigned int i=0, k=0; i<contactKinematics.size(); i++) {
      contactKinematics[i]->updatewb(wb[k], g[k], cpData[k]);
      k += contactKinematics[i]->getNumberOfPotentialContactPoints();
    }
  }

  }
