/*
 * EHDContact.cc
 *
 *  Created on: Aug 12, 2015
 *      Author: local_grundl
 */

#include "ehd_contact.h"

using namespace fmatvec;
using namespace MBSim;

namespace MBSimEHD {
  
  EHDContact::EHDContact(const std::string & name) :
      Contact(name) {
    // TODO Auto-generated constructor stub
    
  }
  
  void EHDContact::updateh(double t, int j) {
    //TODO: implement this update-h routine for the EHD-contact
    //TODO: here the values for the h = J^T*F should be computed. Interface wise the force-contact-law (fcl) can (but not must) be used which is our mesh(?)

    msh->computeSmoothForces(contacts);

    /* Original code:
     for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
     for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
     jter->applyh(t, j);
     }
     */
  }

  EHDContact::~EHDContact() {
    // TODO Auto-generated destructor stub
  }

  void EHDContact::init(Element::InitStage stage) {
    Contact::init(stage);
    if (stage == Element::preInit) {
      // Do some checks and assignments
      if (dynamic_cast<EHDMesh*>(fcl)) {
        msh = static_cast<EHDMesh*>(fcl);
      }
      else {
        throw MBSimError("The Normal contact force law in the EHD-Contact has to be an EHDMesh!");
      }
      if (contactKinematics.size() != 1) {
        throw MBSimError("A EHDContact must have one contact kinematics (by now...?)");
      }
      if (dynamic_cast<ContactKinematicsEHDInterface*>(contactKinematics[0])) {
        msh->setContactKinematics(static_cast<ContactKinematicsEHDInterface*>(contactKinematics[0]));
      }
      else {
        throw MBSimError("A EHDContact may only have one contact kinematics (by now...?)");
      }
    }
  }

} /* namespace MBSimEHD */
