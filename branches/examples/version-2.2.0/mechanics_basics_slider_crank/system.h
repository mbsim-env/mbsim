#ifndef _SLIDERCRANKMECHANISM_H
#define _SLIDERCRANKMECHANISM_H

#include "mbsim/dynamic_system_solver.h"
#include <string>

class System : public MBSim::DynamicSystemSolver {
  public:
    System(const std::string &projectName); 
};

#endif

