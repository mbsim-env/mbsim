/*
 * EHDContact.h
 *
 *  Created on: Aug 12, 2015
 *      Author: local_grundl
 */

#ifndef EHDCONTACT_H_
#define EHDCONTACT_H_

#include "mbsim/contact.h"

namespace MBSimEHD {
  
  class EHDContact : public MBSim::Contact {
    public:
      EHDContact(const std::string & name);
      virtual ~EHDContact();
  };

} /* namespace MBSimEHD */

#endif /* EHDCONTACT_H_ */
