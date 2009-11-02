#ifndef _SYSTEM_H
#define _SYSTEM_H

#include "mbsim/dynamic_system_solver.h"

class System : public MBSim::DynamicSystemSolver {
  public:
    System(const std::string &projectName, bool setValued);
};

#endif

