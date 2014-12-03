#ifndef  _SYSTEM_H_
#define  _SYSTEM_H_

#include "mbsim/dynamic_system_solver.h"

class System : public MBSim::DynamicSystemSolver {
  public:
    System(const std::string &name);
};

#endif   /* ----- #ifndef _SYSTEM_H_  ----- */

