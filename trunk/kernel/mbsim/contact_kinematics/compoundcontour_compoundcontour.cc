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
#include "mbsim/contact_kinematics/compoundcontour_compoundcontour.h"
#include "mbsim/contours/compound_contour.h"
#include "mbsim/utils/contact_utils.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsCompoundContourCompoundContour::assignContours(const vector<Contour*> &contour) {

    contour0 = static_cast<CompoundContour*>(contour[0]);
    contour1 = static_cast<CompoundContour*>(contour[1]);

    numberOfPotentialContactPoints = 0;

    for(size_t i = 0; i < contactKinematics.size(); i++)
      delete contactKinematics[i];
    contactKinematics.clear();

    for(unsigned int i=0; i<contour0->getNumberOfElements(); i++) {
      for(unsigned int j=0; j<contour1->getNumberOfElements(); j++) {
        ContactKinematics *tmp = findContactPairingRigidRigid(contour0->getContourElement(i)->getType().c_str(), contour1->getContourElement(j)->getType().c_str());
        if (tmp == 0)
          tmp = findContactPairingRigidRigid(contour1->getContourElement(j)->getType().c_str(), contour0->getContourElement(i)->getType().c_str());
        if(tmp) {
          contactKinematics.push_back(tmp); 
          tmp->assignContours(contour0->getContourElement(i),contour1->getContourElement(j));
          numberOfPotentialContactPoints += tmp->getNumberOfPotentialContactPoints();
        }
      }
    }
  }

  void ContactKinematicsCompoundContourCompoundContour::updateg(std::vector<fmatvec::Vec>::iterator ig, std::vector<ContourPointData*>::iterator icpData) {
    for(unsigned int i=0; i<contactKinematics.size(); i++) {
      contactKinematics[i]->updateg(ig, icpData);
      ig += contactKinematics[i]->getNumberOfPotentialContactPoints();
      icpData += contactKinematics[i]->getNumberOfPotentialContactPoints();
    }
  }

  void ContactKinematicsCompoundContourCompoundContour::updateg(std::vector<SingleContact> & contact) {
    for(size_t i = 0; i < contact.size(); i++)
      contactKinematics[i]->updateg(contact[i].getg(), contact[i].getcpData());
  }

  void ContactKinematicsCompoundContourCompoundContour::updatewb(std::vector<fmatvec::Vec>::iterator iwb, std::vector<fmatvec::Vec>::iterator ig, std::vector<ContourPointData*>::iterator icpData) {
    for(unsigned int i=0; i<contactKinematics.size(); i++) {
      contactKinematics[i]->updatewb(iwb, ig, icpData);
      iwb += contactKinematics[i]->getNumberOfPotentialContactPoints();
      ig += contactKinematics[i]->getNumberOfPotentialContactPoints();
      icpData += contactKinematics[i]->getNumberOfPotentialContactPoints();
    }
  }
}

