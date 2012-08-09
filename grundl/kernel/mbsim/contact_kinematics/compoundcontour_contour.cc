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
 */

#include <config.h> 
#include "mbsim/contact_kinematics/compoundcontour_contour.h"
#include "mbsim/contours/compound_contour.h"
#include "mbsim/utils/contact_utils.h"

using namespace fmatvec;
using namespace std;

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
      ContactKinematics *tmp = findContactPairingRigidRigid(c[0]->getType().c_str(), c[1]->getType().c_str());
      if(tmp == 0)
        tmp = findContactPairingRigidRigid(c[1]->getType().c_str(), c[0]->getType().c_str());
      if(tmp == 0)
	throw;
      if(tmp) {
        contactKinematics.push_back(tmp); 
        tmp->assignContours(c[0],c[1]);
        numberOfPotentialContactPoints += tmp->getNumberOfPotentialContactPoints();
      }
    }
  }

  void ContactKinematicsCompoundContourContour::updateg(fmatvec::Vec &g, ContourPointData *cpData, int index) {
    contactKinematics[index]->updateg(g, cpData);
  }

  void ContactKinematicsCompoundContourContour::updateg(std::vector<fmatvec::Vec>::iterator ig, std::vector<ContourPointData*>::iterator icpData) {
    for(unsigned int i=0; i<contactKinematics.size(); i++) {
      contactKinematics[i]->updateg(ig, icpData);
      ig += contactKinematics[i]->getNumberOfPotentialContactPoints();
      icpData += contactKinematics[i]->getNumberOfPotentialContactPoints();
    }
  }

  void ContactKinematicsCompoundContourContour::updatewb(std::vector<fmatvec::Vec>::iterator iwb, std::vector<fmatvec::Vec>::iterator ig, std::vector<ContourPointData*>::iterator icpData) {
    for(unsigned int i=0; i<contactKinematics.size(); i++) {
      contactKinematics[i]->updatewb(iwb, ig, icpData);
      iwb += contactKinematics[i]->getNumberOfPotentialContactPoints();
      ig += contactKinematics[i]->getNumberOfPotentialContactPoints();
      icpData += contactKinematics[i]->getNumberOfPotentialContactPoints();
    }
  }

}

