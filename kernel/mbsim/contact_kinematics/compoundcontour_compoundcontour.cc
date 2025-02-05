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

  ContactKinematicsCompoundContourCompoundContour::~ContactKinematicsCompoundContourCompoundContour() {
    for(auto & contactKinematic : contactKinematics)
      delete contactKinematic;
  }

  void ContactKinematicsCompoundContourCompoundContour::assignContours(const vector<Contour*> &contour) {
    ContactKinematics::assignContours(contour);

    contour0 = static_cast<CompoundContour*>(contour[0]);
    contour1 = static_cast<CompoundContour*>(contour[1]);

//    numberOfPotentialContactPoints = 0;

    for(unsigned int i=0; i<contour0->getNumberOfElements(); i++) {
      for(unsigned int j=0; j<contour1->getNumberOfElements(); j++) {
        ContactKinematics *tmp = findContactPairingRigidRigid(typeid(*contour0->getContourElement(i)), typeid(*contour1->getContourElement(j)));
        if (not tmp)
          tmp = findContactPairingRigidRigid(typeid(*contour1->getContourElement(j)), typeid(*contour0->getContourElement(i)));
        if(tmp) {
          contactKinematics.push_back(tmp); 
          tmp->assignContours(contour0->getContourElement(i),contour1->getContourElement(j));
//          numberOfPotentialContactPoints += tmp->getNumberOfPotentialContactPoints();
        }
      }
    }
  }

  void ContactKinematicsCompoundContourCompoundContour::updateg(vector<SingleContact> &contact) {
    int k=0;
    for(size_t i=0; i<contactKinematics.size(); i++) {
      contactKinematics[i]->updateg(contact[k]);
      bool collision = contact[k].getGeneralizedRelativePosition(false)(0) <= 0;
      if(collision) {
        k++;
        if(k==maxNumContacts) break;
      }
    }
    for(int i=k; i<maxNumContacts; i++)
     contact[i].getGeneralizedRelativePosition(false)(0) = 1;
  }

}

