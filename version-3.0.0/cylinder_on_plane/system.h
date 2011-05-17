#ifndef _PENDULUM_H
#define _PENDULUM_H

#include "mbsim/dynamic_system_solver.h"
#include <string>

class System : public MBSim::MySolver {
  public:
    System(const std::string &projectName); 
};

#endif

