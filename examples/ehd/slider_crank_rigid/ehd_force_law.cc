#include "ehd_force_law.h"
#include "circlesolid_circlehollow_ehd.h"

#include <mbsim/single_contact.h>

using namespace MBSim;
using namespace fmatvec;
using namespace xercesc;
using namespace MBXMLUtils;

EHDForceLaw::EHDForceLaw() {

}

EHDForceLaw::~EHDForceLaw() {
}

void EHDForceLaw::computeSmoothForces(std::vector<std::vector<SingleContact> > & contacts) {
// Here the force computation for all contact pairings happens
  for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
    for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
      (*jter).getlaN()(0) = 1e7*nrm2((static_cast<ContactKinematicsCircleSolidCircleHollowEHD*>(jter->getContactKinematics()))->getWrD());
    }
  }
}

void EHDForceLaw::initializeUsingXML(xercesc::DOMElement* element) {
  xercesc::DOMElement *e;
  //TODO ...

}
