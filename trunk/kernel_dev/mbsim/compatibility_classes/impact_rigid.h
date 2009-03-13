#ifndef _IMPACT_RIGID_H_
#define _IMPACT_RIGID_H_

#include "contact_rigid.h"

namespace MBSim {

  class ImpactRigid : public ContactRigid {
    public:
      ImpactRigid(const string &name) : ContactRigid(name) {}
  };

}

#endif
