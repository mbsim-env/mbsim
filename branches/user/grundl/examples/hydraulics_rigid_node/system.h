#ifndef  _SYSTEM_H_
#define  _SYSTEM_H_

#include "mbsim/group.h"

class System : public MBSim::Group {
  public:
    System(const std::string &name, bool setvalued);
};

#endif   /* ----- #ifndef _SYSTEM_H_  ----- */

