#ifndef _PAINLEVE_H
#define _PAINLEVE_H

#include "mbsim/dynamic_system_solver.h"
#include <string>

class System : public MBSim::DynamicSystemSolver {
  public:
    System(const std::string &projectName);
};

#endif /* _PAINLEVE_H */

