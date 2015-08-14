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
    Contact(name){
    // TODO Auto-generated constructor stub
    
  }
  
  void EHDContact::updateh(double t, int j) {
    //TODO: implement this update-h routine for the EHD-contact
    //TODO: here the values for the h = J^T*F should be computed. Inteface wise the force-contact-law (fcl) can (but not must) be used which is our mesh(?)

    //TODO: initialize the values needed in the following subroutines!
    VecV D(msh->getndof()); //TODO: what is D?
    msh->PressureAssembly(D);
    throw MBSimError("Do something like mhs->computeForces()");

    /* Original code:
    (*fcl).computeSmoothForces(contacts);

    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->applyh(t, j);
    }
    */
  }

  EHDContact::~EHDContact() {
    // TODO Auto-generated destructor stub
  }

} /* namespace MBSimEHD */
