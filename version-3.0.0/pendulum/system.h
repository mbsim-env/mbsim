#ifndef _PENDULUM_H
#define _PENDULUM_H

#include "mbsim/dynamic_system_solver.h"
#include <string>

class Pendulum : public MBSim::MySolver2 {
  public:
    Pendulum(const std::string &projectName); 
};

#endif

