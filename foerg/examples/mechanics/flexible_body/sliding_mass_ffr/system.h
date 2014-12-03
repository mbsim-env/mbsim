#ifndef _WERKZEUGMASCHINE_H
#define _WERKZEUGMASCHINE_H

#include "mbsim/dynamic_system_solver.h"
#include <string>

class SlidingMass : public MBSim::DynamicSystemSolver {
  public:
    SlidingMass(const std::string &projectName);
};

#endif

