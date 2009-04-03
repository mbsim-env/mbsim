#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "mbsim/dynamic_system_solver.h"
#include <string>

class Robot : public MBSim::DynamicSystemSolver {
  public:
    Robot(const std::string &projectName); 
};

#endif

