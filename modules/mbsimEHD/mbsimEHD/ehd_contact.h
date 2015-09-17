/*
 * EHDContact.h
 *
 *  Created on: Aug 12, 2015
 *      Author: local_grundl
 */

#ifndef EHDCONTACT_H_
#define EHDCONTACT_H_

#include <mbsim/contact.h>

namespace MBSimEHD {
  
  class EHDMesh;

  class EHDContact : public MBSim::Contact {
    public:
      EHDContact(const std::string & name);
      virtual ~EHDContact();

      virtual void init(MBSim::Element::InitStage stage);

      virtual void updateg(double t);
      virtual void updateh(double t, int j);

      void setMesh(EHDMesh * msh_);

    protected:
      /*!
       * \brief mesh used in the Contact
       *
       * \todo: its not good, that it is a pointer
       */
      EHDMesh * msh;
  };

} /* namespace MBSimEHD */

#endif /* EHDCONTACT_H_ */
