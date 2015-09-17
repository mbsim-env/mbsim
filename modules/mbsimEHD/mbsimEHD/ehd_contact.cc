/*
 * EHDContact.cc
 *
 *  Created on: Aug 12, 2015
 *      Author: local_grundl
 */

#include "ehd_contact.h"
#include "ehd_mesh.h"
#include <mbsim/mbsim_event.h>

using namespace fmatvec;
using namespace MBSim;
using namespace std;

namespace MBSimEHD {
  
  EHDContact::EHDContact(const std::string & name) :
      Contact(name), msh(0) {
  }
  
  void EHDContact::updateg(double t) {
    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
      static_cast<ContactKinematicsEHDInterface*>(contactKinematics[cK])->updateKinematics(contacts[cK]);
    }
  }

  void EHDContact::updateh(double t, int j) {
    //TODO: implement this update-h routine for the EHD-contact
    //TODO: here the values for the h = J^T*F should be computed. Interface wise the force-contact-law (fcl) can (but not must) be used which is our mesh(?)

    msh->computeSmoothForces(contacts);

    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->applyh(t, j);
    }
  }

  EHDContact::~EHDContact() {
  }

  void EHDContact::init(Element::InitStage stage) {
    if (stage == Element::modelBuildup) {
      // Do some checks and assignments
      if (msh == NULL) {
        THROW_MBSIMERROR("No mesh provided for the EHD-Contact!");
      }
      if (contactKinematics.size() != 1) {
        THROW_MBSIMERROR("A EHDContact must have one contact kinematics (by now...?)");
      }
      if (dynamic_cast<ContactKinematicsEHDInterface*>(contactKinematics[0])) {
        msh->setContactKinematics(static_cast<ContactKinematicsEHDInterface*>(contactKinematics[0]));
      }
      else {
        THROW_MBSIMERROR("A EHDContact may only have one contact kinematics (by now...?)");
      }
    }

    // Has to be done afterwards as the msh needs to know the contact kinematics
    Contact::init(stage);
  }

  void EHDContact::setMesh(EHDMesh * msh_) {
    msh = msh_;
    fcl = msh_;
  }

} /* namespace MBSimEHD */
