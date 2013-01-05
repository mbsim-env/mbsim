#ifndef _GEAR_H
#define _GEAR_H

#include "mbsim/dynamic_system_solver.h"
#include <string>

class Gear : public MBSim::DynamicSystemSolver {
  public:
    Gear(const std::string &projectName); 
};

#endif

