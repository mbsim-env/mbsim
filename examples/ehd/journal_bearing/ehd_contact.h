/*
 * EHDContact.h
 *
 *  Created on: Aug 12, 2015
 *      Author: local_grundl
 */

#ifndef EHDCONTACT_H_
#define EHDCONTACT_H_

#include "ehd_mesh.h"

#include "mbsim/contact.h"


namespace MBSimEHD {
  
  class EHDContact : public MBSim::Contact {
    public:
      EHDContact(const std::string & name);
      virtual ~EHDContact();

      void setMesh(EHDMesh * msh_) {
        msh = msh_;
      }

      virtual void init(MBSim::Element::InitStage stage);

      virtual void updateh(double t, int j);

    private:
      /*!
       * \brief mesh used in the Contact
       *
       * \todo: its not good, that it is a pointer
       */
      EHDMesh * msh;
  };

} /* namespace MBSimEHD */

#endif /* EHDCONTACT_H_ */
