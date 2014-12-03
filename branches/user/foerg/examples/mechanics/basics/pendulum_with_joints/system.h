#ifndef _PENDULUM_WITH_JOINTS_H
#define _PENDULUM_WITH_JOINTS_H

#include "mbsim/dynamic_system_solver.h"
#include <string>

class System : public MBSim::DynamicSystemSolver {
  public:
    System(const std::string &projectName); 
};

#endif /* _PENDULUM_WITH_JOINTS_H */

