#ifndef _WOODPECKER_H
#define _WOODPECKER_H

#include "mbsim/dynamic_system_solver.h"
#include <string>

class Woodpecker : public MBSim::DynamicSystemSolver {
  private:
  public:
    Woodpecker(const std::string &projectName); 
    
};

#endif
